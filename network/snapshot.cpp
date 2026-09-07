/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "network/snapshot.h"

#include "McZapkie/MOVER.h"
#include "network/entities.h"
#include "network/session.h"
#include "scene/sn_utils.h"
#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "world/Track.h"

namespace
{

uint32_t const SNAPSHOT_MAGIC{0x4e535545}; // 'EUSN'

// how far a vehicle may sit from where the authority says it is before it gets put back.
// a joining peer is placed exactly; a running one is left to its own physics for small
// errors, because nudging a vehicle every fraction of a second is worse than the error
double const REPOSITION_TOLERANCE_JOIN{0.5};
double const REPOSITION_TOLERANCE_CORRECTION{1.5};

// ---------------------------------------------------------------------------
// what the authority last sent, so that a routine correction only carries changes

// most tracks in a scenery carry no name at all, so a name is useless as an identity.
// their order in the path table comes straight from the scenery file and is therefore the
// same on every peer, which makes the index a usable one
std::vector<TTrack *> g_trackindex;
std::unordered_map<TTrack const *, uint32_t> g_trackids;

void refresh_track_index()
{
	auto const &paths = simulation::Paths.sequence();
	if (g_trackindex.size() == paths.size())
		return;

	g_trackindex.assign(paths.begin(), paths.end());
	g_trackids.clear();
	for (uint32_t i = 0; i < g_trackindex.size(); ++i)
	{
		if (g_trackindex[i] != nullptr)
			g_trackids.emplace(g_trackindex[i], i);
	}
}

uint32_t track_id_of(TTrack const *Track)
{
	if (Track == nullptr)
		return 0xffffffff;

	refresh_track_index();

	auto const lookup = g_trackids.find(Track);
	return lookup != g_trackids.end() ? lookup->second : 0xffffffff;
}

TTrack *track_by_id(uint32_t const Id)
{
	refresh_track_index();

	return (Id < g_trackindex.size() ? g_trackindex[Id] : nullptr);
}

std::unordered_map<std::string, uint64_t> g_lastvehicles;
std::unordered_map<std::string, uint64_t> g_lastmemcells;
std::unordered_map<std::string, int> g_lastswitches;

uint64_t digest_of(std::string const &Text)
{
	uint64_t hash{14695981039346656037ull};
	for (char const character : Text)
	{
		hash ^= (uint64_t)(unsigned char)character;
		hash *= 1099511628211ull;
	}
	return hash;
}

void write_chunk(std::ostream &Stream, network::snapshot_chunk const Id, std::string const &Body)
{
	sn_utils::ls_uint16(Stream, (uint16_t)Id);
	sn_utils::ls_uint32(Stream, (uint32_t)Body.size());
	Stream.write(Body.data(), Body.size());
}

// ---------------------------------------------------------------------------
// session

std::string write_session(bool const Full)
{
	std::ostringstream body;

	sn_utils::ls_uint64(body, Global.simulation_tick);
	sn_utils::ls_int64(body, Global.starting_timestamp);
	sn_utils::ls_uint32(body, Global.random_seed);

	auto const &time = simulation::Time.data();
	sn_utils::ls_uint32(body, (uint32_t)simulation::Time.year_day());
	sn_utils::ls_uint32(body, (uint32_t)(time.wHour * 60 + time.wMinute));
	sn_utils::ls_float64(body, simulation::Time.second());

	sn_utils::ls_float64(body, Global.Overcast);
	sn_utils::ls_float64(body, Global.AirTemperature);

	// the engine drives gameplay randomness, so a joining peer has to pick it up as it is.
	// it is several kilobytes though, far too much to repeat in every routine correction
	sn_utils::s_bool(body, Full);
	if (Full)
	{
		std::ostringstream engine;
		engine << Global.random_engine;
		sn_utils::s_str(body, engine.str());
	}

	return body.str();
}

void read_session(std::istream &Stream, network::snapshot_mode const Mode)
{
	auto const tick = sn_utils::ld_uint64(Stream);
	auto const timestamp = sn_utils::ld_int64(Stream);
	auto const seed = sn_utils::ld_uint32(Stream);

	auto const yearday = sn_utils::ld_uint32(Stream);
	auto const minuteofday = sn_utils::ld_uint32(Stream);
	auto const second = sn_utils::ld_float64(Stream);

	auto const overcast = sn_utils::ld_float64(Stream);
	auto const temperature = sn_utils::ld_float64(Stream);

	bool const hasengine = sn_utils::d_bool(Stream);
	std::string enginestate;
	if (hasengine)
		enginestate = sn_utils::d_str(Stream);

	Global.simulation_tick = tick;
	Global.starting_timestamp = timestamp;
	Global.random_seed = seed;

	simulation::Time.set_time((int)yearday, (int)minuteofday);
	simulation::Time.data().wSecond = (WORD)std::floor(second);
	simulation::Time.data().wMilliseconds = (WORD)((second - std::floor(second)) * 1000.0);

	Global.Overcast = (float)overcast;
	Global.AirTemperature = (float)temperature;

	if (!enginestate.empty())
	{
		std::istringstream engine(enginestate);
		engine >> Global.random_engine;
	}

	(void)Mode;
}

// ---------------------------------------------------------------------------
// vehicles

// everything about one vehicle the rest of the session needs to agree on
struct vehicle_state
{
	std::string name;
	uint32_t track{0xffffffff};
	double translation{0.0};
	uint8_t axlefirst{0};
	uint8_t direction{0};
	glm::dvec3 position{};

	double velocity{0.0};
	double velocitykmh{0.0};
	int32_t diractive{0};
	int32_t mainctrl{0};
	int32_t scndctrl{0};
	int32_t brakectrl{0};
	double localbrake{0.0};

	double pipepress{0.0};
	double brakepress{0.0};
	double scndpipepress{0.0};
	double eqvtpipepress{0.0};
	double compressor{0.0};
	double compressedvolume{0.0};

	// switch positions and permissions - the inputs of the vehicle, not its outputs.
	// getting these right lets the receiving peer derive the rest for itself
	bool mains{false};
	bool battery{false};
	bool converterallow{false};
	bool compressorallow{false};
	std::array<bool, 2> pantenabled{{false, false}};
	std::array<bool, 2> pantdisabled{{false, false}};
	std::array<bool, 2> pantactive{{false, false}};
	int32_t lights[2]{0, 0};
	// door open/close intent, one entry per side; the local model derives the rest
	std::array<bool, 2> dooropen{{false, false}};
	std::array<bool, 2> doorpermit{{false, false}};
	bool aiactive{false};
};

void serialize_vehicle(std::ostream &Stream, vehicle_state const &State)
{
	sn_utils::s_str(Stream, State.name);
	sn_utils::ls_uint32(Stream, State.track);
	sn_utils::ls_float64(Stream, State.translation);
	sn_utils::s_uint8(Stream, State.axlefirst);
	sn_utils::s_uint8(Stream, State.direction);
	sn_utils::s_dvec3(Stream, State.position);

	sn_utils::ls_float64(Stream, State.velocity);
	sn_utils::ls_float64(Stream, State.velocitykmh);
	sn_utils::ls_int32(Stream, State.diractive);
	sn_utils::ls_int32(Stream, State.mainctrl);
	sn_utils::ls_int32(Stream, State.scndctrl);
	sn_utils::ls_int32(Stream, State.brakectrl);
	sn_utils::ls_float64(Stream, State.localbrake);

	sn_utils::ls_float64(Stream, State.pipepress);
	sn_utils::ls_float64(Stream, State.brakepress);
	sn_utils::ls_float64(Stream, State.scndpipepress);
	sn_utils::ls_float64(Stream, State.eqvtpipepress);
	sn_utils::ls_float64(Stream, State.compressor);
	sn_utils::ls_float64(Stream, State.compressedvolume);

	sn_utils::s_bool(Stream, State.mains);
	sn_utils::s_bool(Stream, State.battery);
	sn_utils::s_bool(Stream, State.converterallow);
	sn_utils::s_bool(Stream, State.compressorallow);
	for (int i = 0; i < 2; ++i)
	{
		sn_utils::s_bool(Stream, State.pantenabled[i]);
		sn_utils::s_bool(Stream, State.pantdisabled[i]);
		sn_utils::s_bool(Stream, State.pantactive[i]);
	}
	sn_utils::ls_int32(Stream, State.lights[0]);
	sn_utils::ls_int32(Stream, State.lights[1]);
	for (int i = 0; i < 2; ++i)
	{
		sn_utils::s_bool(Stream, State.dooropen[i]);
		sn_utils::s_bool(Stream, State.doorpermit[i]);
	}
	sn_utils::s_bool(Stream, State.aiactive);
}

vehicle_state deserialize_vehicle(std::istream &Stream)
{
	vehicle_state state;

	state.name = sn_utils::d_str(Stream);
	state.track = sn_utils::ld_uint32(Stream);
	state.translation = sn_utils::ld_float64(Stream);
	state.axlefirst = sn_utils::d_uint8(Stream);
	state.direction = sn_utils::d_uint8(Stream);
	state.position = sn_utils::d_dvec3(Stream);

	state.velocity = sn_utils::ld_float64(Stream);
	state.velocitykmh = sn_utils::ld_float64(Stream);
	state.diractive = sn_utils::ld_int32(Stream);
	state.mainctrl = sn_utils::ld_int32(Stream);
	state.scndctrl = sn_utils::ld_int32(Stream);
	state.brakectrl = sn_utils::ld_int32(Stream);
	state.localbrake = sn_utils::ld_float64(Stream);

	state.pipepress = sn_utils::ld_float64(Stream);
	state.brakepress = sn_utils::ld_float64(Stream);
	state.scndpipepress = sn_utils::ld_float64(Stream);
	state.eqvtpipepress = sn_utils::ld_float64(Stream);
	state.compressor = sn_utils::ld_float64(Stream);
	state.compressedvolume = sn_utils::ld_float64(Stream);

	state.mains = sn_utils::d_bool(Stream);
	state.battery = sn_utils::d_bool(Stream);
	state.converterallow = sn_utils::d_bool(Stream);
	state.compressorallow = sn_utils::d_bool(Stream);
	for (int i = 0; i < 2; ++i)
	{
		state.pantenabled[i] = sn_utils::d_bool(Stream);
		state.pantdisabled[i] = sn_utils::d_bool(Stream);
		state.pantactive[i] = sn_utils::d_bool(Stream);
	}
	state.lights[0] = sn_utils::ld_int32(Stream);
	state.lights[1] = sn_utils::ld_int32(Stream);
	for (int i = 0; i < 2; ++i)
	{
		state.dooropen[i] = sn_utils::d_bool(Stream);
		state.doorpermit[i] = sn_utils::d_bool(Stream);
	}
	state.aiactive = sn_utils::d_bool(Stream);

	return state;
}

vehicle_state read_from(TDynamicObject const &Vehicle)
{
	auto const &mover = *Vehicle.MoverParameters;
	auto const *track = Vehicle.RaTrackGet();

	vehicle_state state;
	state.name = Vehicle.name();
	state.track = track_id_of(track);
	state.translation = Vehicle.RaTranslationGet();
	state.axlefirst = (uint8_t)(Vehicle.iAxleFirst ? 1 : 0);
	state.direction = (uint8_t)(Vehicle.iDirection ? 1 : 0);
	state.position = Vehicle.GetPosition();

	state.velocity = mover.V;
	state.velocitykmh = mover.Vel;
	state.diractive = mover.DirActive;
	state.mainctrl = mover.MainCtrlPos;
	state.scndctrl = mover.ScndCtrlPos;
	state.brakectrl = mover.BrakeCtrlPos;
	state.localbrake = mover.LocalBrakePosA;

	state.pipepress = mover.PipePress;
	state.brakepress = mover.BrakePress;
	state.scndpipepress = mover.ScndPipePress;
	state.eqvtpipepress = mover.EqvtPipePress;
	state.compressor = mover.Compressor;
	state.compressedvolume = mover.CompressedVolume;

	state.mains = mover.Mains;
	state.battery = mover.Battery;
	state.converterallow = mover.ConverterAllow;
	state.compressorallow = mover.CompressorAllow;
	for (int i = 0; i < 2; ++i)
	{
		state.pantenabled[i] = mover.Pantographs[i].valve.is_enabled;
		state.pantdisabled[i] = mover.Pantographs[i].valve.is_disabled;
		state.pantactive[i] = mover.Pantographs[i].is_active;
	}
	state.lights[0] = mover.iLights[0];
	state.lights[1] = mover.iLights[1];
	for (int i = 0; i < 2; ++i)
	{
		state.dooropen[i] = mover.Doors.instances[i].is_open;
		state.doorpermit[i] = mover.Doors.instances[i].open_permit;
	}
	state.aiactive = (Vehicle.Mechanik != nullptr && Vehicle.Mechanik->AIControllFlag);

	return state;
}

// applies everything except where the vehicle is; position is dealt with per consist,
// because moving one vehicle of a coupled set on its own tears the couplers apart.
// only ever called for vehicles somebody else is running
void apply_controls(TDynamicObject &Vehicle, vehicle_state const &State)
{
	auto &mover = *Vehicle.MoverParameters;

	mover.V = State.velocity;
	mover.Vel = State.velocitykmh;
	mover.DirActive = State.diractive;
	mover.MainCtrlPos = State.mainctrl;
	mover.ScndCtrlPos = State.scndctrl;
	mover.BrakeCtrlPos = State.brakectrl;
	mover.LocalBrakePosA = State.localbrake;

	mover.PipePress = State.pipepress;
	mover.BrakePress = State.brakepress;
	mover.ScndPipePress = State.scndpipepress;
	mover.EqvtPipePress = State.eqvtpipepress;
	mover.Compressor = State.compressor;
	mover.CompressedVolume = State.compressedvolume;

	// the appliances go through the vehicle's own switches wherever it has them, so that
	// whatever they drag along stays consistent. range local: every vehicle carries its
	// own state in the update, there is no need to push it down the consist twice
	if (mover.Battery != State.battery)
		mover.BatterySwitch(State.battery, range_t::local);
	if (mover.Mains != State.mains)
		mover.MainSwitch(State.mains, range_t::local);
	if (mover.ConverterAllow != State.converterallow)
		mover.ConverterSwitch(State.converterallow, range_t::local);
	if (mover.CompressorAllow != State.compressorallow)
		mover.CompressorSwitch(State.compressorallow, range_t::local);

	// CabActive and CabOccupied are deliberately not replicated: which cab a player is
	// sitting in is theirs alone, and two people sharing a vehicle each have their own.
	// forcing the server's value on them is what made the camera jump in and out

	for (int i = 0; i < 2; ++i)
	{
		mover.Pantographs[i].valve.is_enabled = State.pantenabled[i];
		mover.Pantographs[i].valve.is_disabled = State.pantdisabled[i];
		mover.Pantographs[i].is_active = State.pantactive[i];
	}

	mover.iLights[0] = State.lights[0];
	mover.iLights[1] = State.lights[1];

	for (int i = 0; i < 2; ++i)
	{
		auto const side = (i == 0 ? side::right : side::left);
		if (mover.Doors.instances[i].open_permit != State.doorpermit[i])
			mover.PermitDoors(side, State.doorpermit[i], range_t::local);
		if (mover.Doors.instances[i].is_open != State.dooropen[i])
			mover.OperateDoors(side, State.dooropen[i], range_t::local);
	}

	if (Vehicle.Mechanik != nullptr && Vehicle.Mechanik->AIControllFlag != State.aiactive)
		Vehicle.Mechanik->TakeControl(State.aiactive);
}

// puts a vehicle back where the authority says it is. place_on_track() wants the distance
// of the vehicle's nose along the track, while what travels in the update is the offset of
// its leading bogie; this inverts the arithmetic that function itself does
void reposition(TDynamicObject &Vehicle, vehicle_state const &State)
{
	TTrack *track = track_by_id(State.track);
	if (track == nullptr)
	{
		static bool reported{false};
		if (!reported)
		{
			reported = true;
			ErrorLog("net: cannot place " + Vehicle.name() + ", track " + std::to_string(State.track) + " is unknown here", logtype::net);
		}
		return;
	}

	double const half = Vehicle.fAxleDist * 0.5;
	double const axleoffset = (State.axlefirst ? -half : half);
	double const sign = (State.direction ? 1.0 : -1.0);
	double const nose = (State.translation - axleoffset) * sign + 0.5 * Vehicle.MoverParameters->Dim.L;

	Vehicle.place_on_track(track, nose, false);
}

std::string write_vehicles(bool const Full, bool const OwnedOnly)
{
	std::ostringstream entries;
	uint32_t count{0};

	for (TDynamicObject const *vehicle : simulation::Vehicles.sequence())
	{
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
			continue;

		if (OwnedOnly && !network::is_locally_simulated(network::Entities.id_of(vehicle->name())))
			continue;

		auto const state = read_from(*vehicle);

		std::ostringstream packed;
		serialize_vehicle(packed, state);
		auto const record = packed.str();

		if (!Full)
		{
			// a vehicle that is standing still and has not been touched needs no update
			auto const digest = digest_of(record);
			auto const previous = g_lastvehicles.find(state.name);
			if (previous != g_lastvehicles.end() && previous->second == digest)
				continue;

			g_lastvehicles[state.name] = digest;
		}
		else
		{
			g_lastvehicles[state.name] = digest_of(record);
		}

		entries.write(record.data(), record.size());
		++count;
	}

	if (count == 0)
		return std::string();

	std::ostringstream body;
	sn_utils::ls_uint32(body, count);
	auto const packed = entries.str();
	body.write(packed.data(), packed.size());

	return body.str();
}

void read_vehicles(std::istream &Stream, network::snapshot_mode const Mode, network::PeerId const Owner, network::snapshot_result &Result)
{
	auto const count = sn_utils::ld_uint32(Stream);
	double const tolerance = (Mode == network::snapshot_mode::join ? REPOSITION_TOLERANCE_JOIN : REPOSITION_TOLERANCE_CORRECTION);

	std::unordered_map<TDynamicObject *, vehicle_state> states;
	states.reserve(count);

	for (uint32_t i = 0; i < count; ++i)
	{
		auto const state = deserialize_vehicle(Stream);

		TDynamicObject *vehicle = simulation::Vehicles.find(state.name);
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
			continue;

		auto const entity = network::Entities.id_of(state.name);

		// a train this peer runs is not corrected by anybody: its own physics is what the
		// rest of the session is being told about, and overwriting it would be the thing
		// that makes the owner's ride stutter
		if ((Mode == network::snapshot_mode::correction) && network::is_locally_simulated(entity))
			continue;

		// and a peer may only move what it is entitled to move
		if ((Owner != network::PEER_NONE) && (network::simulation_owner(entity) != Owner))
			continue;

		apply_controls(*vehicle, state);
		states.emplace(vehicle, state);

		Result.worst_position_error = std::max(Result.worst_position_error, glm::length(vehicle->GetPosition() - state.position));
	}

	Result.vehicles = (uint32_t)states.size();

	// a coupled set is put back as a whole or not at all: correcting one vehicle of a
	// consist while its neighbour stays put is what stretches a coupler until it breaks
	std::unordered_set<TDynamicObject *> visited;

	for (auto const &pair : states)
	{
		if (visited.count(pair.first) > 0)
			continue;

		// the cap is only there so that a malformed consist cannot spin us forever
		int const CONSIST_LIMIT{256};

		std::vector<TDynamicObject *> group;
		TDynamicObject *front = pair.first;
		for (int step = 0; step < CONSIST_LIMIT; ++step)
		{
			TDynamicObject *previous = front->Prev();
			if (previous == nullptr || previous == pair.first || visited.count(previous) > 0)
				break;
			front = previous;
		}

		TDynamicObject *member = front;
		for (int step = 0; step < CONSIST_LIMIT && member != nullptr; ++step, member = member->Next())
		{
			if (visited.count(member) > 0)
				break;
			visited.emplace(member);
			group.emplace_back(member);
		}

		bool wanted{false};
		for (TDynamicObject *member : group)
		{
			auto const known = states.find(member);
			if (known == states.end())
				continue;
			if (glm::length(member->GetPosition() - known->second.position) > tolerance)
			{
				wanted = true;
				break;
			}
		}

		if (!wanted)
			continue;

		for (TDynamicObject *member : group)
		{
			auto const known = states.find(member);
			if (known == states.end())
			{
				// part of the consist is not in this update, so the set cannot be placed
				// as a whole; leaving it alone beats tearing it in half
				wanted = false;
				break;
			}
		}

		if (!wanted)
			continue;

		for (TDynamicObject *member : group)
		{
			reposition(*member, states.at(member));
			++Result.repositioned;
		}
	}
}

// ---------------------------------------------------------------------------
// crews

std::string write_crews()
{
	std::ostringstream entries;
	uint32_t count{0};

	for (auto const &entry : network::Entities.entries())
	{
		auto const crew = network::Crews.crew_of(entry.id);
		if (crew.empty())
			continue;

		sn_utils::ls_uint32(entries, entry.id);
		sn_utils::ls_uint32(entries, (uint32_t)crew.size());
		for (network::PeerId const peer : crew)
			sn_utils::ls_uint32(entries, peer);

		++count;
	}

	std::ostringstream body;
	sn_utils::ls_uint32(body, count);
	auto const packed = entries.str();
	body.write(packed.data(), packed.size());

	return body.str();
}

void read_crews(std::istream &Stream)
{
	auto const count = sn_utils::ld_uint32(Stream);

	for (uint32_t i = 0; i < count; ++i)
	{
		auto const id = sn_utils::ld_uint32(Stream);
		auto const size = sn_utils::ld_uint32(Stream);

		std::vector<network::PeerId> crew;
		crew.reserve(size);
		for (uint32_t p = 0; p < size; ++p)
			crew.emplace_back(sn_utils::ld_uint32(Stream));

		network::Crews.mirror(id, crew);
	}
}

// ---------------------------------------------------------------------------
// memory cells - this is what the signalling reads, so without them a client sees
// semaphores frozen at whatever the scenario started with

std::string write_memcells(bool const Full)
{
	std::ostringstream entries;
	uint32_t count{0};

	for (TMemCell const *cell : simulation::Memory.sequence())
	{
		if (cell == nullptr || cell->name().empty())
			continue;

		std::ostringstream packed;
		sn_utils::s_str(packed, cell->name());
		sn_utils::s_str(packed, cell->Text());
		sn_utils::ls_float64(packed, cell->Value1());
		sn_utils::ls_float64(packed, cell->Value2());
		auto const record = packed.str();

		auto const digest = digest_of(record);
		if (!Full)
		{
			auto const previous = g_lastmemcells.find(cell->name());
			if (previous != g_lastmemcells.end() && previous->second == digest)
				continue;
		}
		g_lastmemcells[cell->name()] = digest;

		entries.write(record.data(), record.size());
		++count;
	}

	if (count == 0)
		return std::string();

	std::ostringstream body;
	sn_utils::ls_uint32(body, count);
	auto const packed = entries.str();
	body.write(packed.data(), packed.size());

	return body.str();
}

void read_memcells(std::istream &Stream)
{
	auto const count = sn_utils::ld_uint32(Stream);

	for (uint32_t i = 0; i < count; ++i)
	{
		auto const name = sn_utils::d_str(Stream);
		auto const text = sn_utils::d_str(Stream);
		auto const value1 = sn_utils::ld_float64(Stream);
		auto const value2 = sn_utils::ld_float64(Stream);

		TMemCell *cell = simulation::Memory.find(name);
		if (cell == nullptr)
			continue;

		if (cell->Text() == text && cell->Value1() == value1 && cell->Value2() == value2)
			continue;

		cell->UpdateValues(text, value1, value2, basic_event::flags::text | basic_event::flags::value1 | basic_event::flags::value2);
	}
}

// ---------------------------------------------------------------------------
// switches

std::string write_switches(bool const Full)
{
	std::ostringstream entries;
	uint32_t count{0};

	for (TTrack *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->name().empty())
			continue;

		int const state = track->GetSwitchState();
		if (state < 0)
			continue;

		if (!Full)
		{
			auto const previous = g_lastswitches.find(track->name());
			if (previous != g_lastswitches.end() && previous->second == state)
				continue;
		}
		g_lastswitches[track->name()] = state;

		sn_utils::s_str(entries, track->name());
		sn_utils::ls_int32(entries, state);
		++count;
	}

	if (count == 0)
		return std::string();

	std::ostringstream body;
	sn_utils::ls_uint32(body, count);
	auto const packed = entries.str();
	body.write(packed.data(), packed.size());

	return body.str();
}

void read_switches(std::istream &Stream)
{
	auto const count = sn_utils::ld_uint32(Stream);

	for (uint32_t i = 0; i < count; ++i)
	{
		auto const name = sn_utils::d_str(Stream);
		auto const state = sn_utils::ld_int32(Stream);

		TTrack *track = simulation::Paths.find(name);
		if (track == nullptr)
			continue;

		if (track->GetSwitchState() != state)
			track->Switch(state);
	}
}

} // namespace

void network::reset_snapshot_history()
{
	g_lastvehicles.clear();
	g_lastmemcells.clear();
	g_lastswitches.clear();
}

std::string network::take_snapshot(bool const Full, bool const OwnedOnly)
{
	auto const vehicles = write_vehicles(Full, OwnedOnly);

	if (OwnedOnly)
	{
		// what a peer says about its own trains, and nothing else: the clock, the crews,
		// the memory cells and the switches all belong to the authority
		if (vehicles.empty())
			return std::string();

		std::ostringstream owned;
		sn_utils::ls_uint32(owned, SNAPSHOT_MAGIC);
		sn_utils::ls_uint32(owned, SNAPSHOT_VERSION);
		sn_utils::ls_uint64(owned, Global.simulation_tick);
		sn_utils::ls_uint32(owned, 1);
		write_chunk(owned, SNAPSHOT_VEHICLES, vehicles);

		return owned.str();
	}

	auto const session = write_session(Full);
	auto const crews = write_crews();
	auto const memcells = write_memcells(Full);
	auto const switches = write_switches(Full);

	if (!Full && vehicles.empty() && memcells.empty() && switches.empty())
	{
		// nothing moved and nobody touched anything; the clock alone is not worth a packet
		return std::string();
	}

	uint32_t chunks{1}; // the session section always goes along
	if (!vehicles.empty())
		++chunks;
	++chunks; // crews
	if (!memcells.empty())
		++chunks;
	if (!switches.empty())
		++chunks;

	std::ostringstream stream;
	sn_utils::ls_uint32(stream, SNAPSHOT_MAGIC);
	sn_utils::ls_uint32(stream, SNAPSHOT_VERSION);
	sn_utils::ls_uint64(stream, Global.simulation_tick);
	sn_utils::ls_uint32(stream, chunks);

	write_chunk(stream, SNAPSHOT_SESSION, session);
	if (!vehicles.empty())
		write_chunk(stream, SNAPSHOT_VEHICLES, vehicles);
	write_chunk(stream, SNAPSHOT_CREWS, crews);
	if (!memcells.empty())
		write_chunk(stream, SNAPSHOT_MEMCELLS, memcells);
	if (!switches.empty())
		write_chunk(stream, SNAPSHOT_SWITCHES, switches);

	auto const blob = stream.str();

	if (Full)
		WriteLog("net: snapshot taken at tick " + std::to_string(Global.simulation_tick) + ", " + std::to_string(blob.size()) + " bytes", logtype::net);

	return blob;
}

network::snapshot_result network::apply_snapshot(std::string const &Blob, snapshot_mode const Mode, PeerId const Owner)
{
	snapshot_result result;

	std::istringstream stream(Blob);

	if (sn_utils::ld_uint32(stream) != SNAPSHOT_MAGIC)
	{
		ErrorLog("net: snapshot rejected, not a snapshot", logtype::net);
		return result;
	}

	auto const version = sn_utils::ld_uint32(stream);
	if (version != SNAPSHOT_VERSION)
	{
		ErrorLog("net: snapshot rejected, version " + std::to_string(version) + " but this build speaks " + std::to_string(SNAPSHOT_VERSION), logtype::net);
		return result;
	}

	auto const tick = sn_utils::ld_uint64(stream);
	auto const chunks = sn_utils::ld_uint32(stream);

	for (uint32_t i = 0; i < chunks; ++i)
	{
		if (!stream.good())
		{
			ErrorLog("net: snapshot truncated", logtype::net);
			return result;
		}

		auto const id = sn_utils::ld_uint16(stream);
		auto const length = sn_utils::ld_uint32(stream);

		std::string body(length, '\0');
		stream.read(body.data(), length);

		std::istringstream chunk(body);

		switch (id)
		{
		case SNAPSHOT_SESSION:
			if (Owner == PEER_NONE)
				read_session(chunk, Mode);
			break;
		case SNAPSHOT_VEHICLES:
			read_vehicles(chunk, Mode, Owner, result);
			break;
		case SNAPSHOT_CREWS:
			if (Owner == PEER_NONE)
				read_crews(chunk);
			break;
		case SNAPSHOT_MEMCELLS:
			if (Owner == PEER_NONE)
				read_memcells(chunk);
			break;
		case SNAPSHOT_SWITCHES:
			if (Owner == PEER_NONE)
				read_switches(chunk);
			break;
		default:
			// a section this build knows nothing about; its length is right there, so it
			// costs nothing to walk past it
			break;
		}
	}

	result.ok = true;

	if (Mode == snapshot_mode::join)
	{
		WriteLog("net: snapshot applied at tick " + std::to_string(tick) + ", " + std::to_string(result.vehicles) + " vehicles, " +
		             std::to_string(result.repositioned) + " put back on the rails",
		         logtype::net);
	}

	return result;
}

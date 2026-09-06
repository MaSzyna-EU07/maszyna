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
#include "world/Track.h"

namespace
{

uint32_t const SNAPSHOT_MAGIC{0x4e535545}; // 'EUSN'

// a vehicle is only put back on the rails when it really is somewhere else; nudging one
// that is already in place would only shake it about
double const REPOSITION_THRESHOLD{0.5};

void write_chunk(std::ostream &Stream, network::snapshot_chunk const Id, std::string const &Body)
{
	sn_utils::ls_uint16(Stream, (uint16_t)Id);
	sn_utils::ls_uint32(Stream, (uint32_t)Body.size());
	Stream.write(Body.data(), Body.size());
}

// ---------------------------------------------------------------------------
// session

std::string write_session()
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

	// the engine drives gameplay randomness, so a joining peer has to pick it up as it is
	std::ostringstream engine;
	engine << Global.random_engine;
	sn_utils::s_str(body, engine.str());

	return body.str();
}

void read_session(std::istream &Stream)
{
	auto const tick = sn_utils::ld_uint64(Stream);
	auto const timestamp = sn_utils::ld_int64(Stream);
	auto const seed = sn_utils::ld_uint32(Stream);

	auto const yearday = sn_utils::ld_uint32(Stream);
	auto const minuteofday = sn_utils::ld_uint32(Stream);
	auto const second = sn_utils::ld_float64(Stream);

	auto const overcast = sn_utils::ld_float64(Stream);
	auto const temperature = sn_utils::ld_float64(Stream);

	auto const enginestate = sn_utils::d_str(Stream);

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
}

// ---------------------------------------------------------------------------
// vehicles

std::string write_vehicles()
{
	std::ostringstream body;
	std::ostringstream entries;
	uint32_t count{0};

	for (TDynamicObject const *vehicle : simulation::Vehicles.sequence())
	{
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
			continue;

		auto const &mover = *vehicle->MoverParameters;
		auto const *track = vehicle->RaTrackGet();

		sn_utils::s_str(entries, vehicle->name());
		sn_utils::s_str(entries, track != nullptr ? track->name() : std::string());
		sn_utils::ls_float64(entries, vehicle->RaTranslationGet());
		sn_utils::s_uint8(entries, (uint8_t)(vehicle->iAxleFirst ? 1 : 0));
		sn_utils::s_uint8(entries, (uint8_t)(vehicle->iDirection ? 1 : 0));
		sn_utils::s_dvec3(entries, vehicle->GetPosition());

		sn_utils::ls_float64(entries, mover.V);
		sn_utils::ls_float64(entries, mover.Vel);
		sn_utils::ls_int32(entries, mover.DirActive);
		sn_utils::ls_int32(entries, mover.MainCtrlPos);
		sn_utils::ls_int32(entries, mover.ScndCtrlPos);
		sn_utils::ls_int32(entries, mover.BrakeCtrlPos);
		sn_utils::ls_float64(entries, mover.LocalBrakePosA);
		sn_utils::ls_float64(entries, mover.PipePress);
		sn_utils::ls_float64(entries, mover.BrakePress);
		sn_utils::ls_float64(entries, mover.ScndPipePress);
		sn_utils::ls_float64(entries, mover.Compressor);
		sn_utils::ls_float64(entries, mover.CompressedVolume);
		sn_utils::s_bool(entries, vehicle->Mechanik != nullptr && vehicle->Mechanik->AIControllFlag);

		++count;
	}

	sn_utils::ls_uint32(body, count);
	auto const packed = entries.str();
	body.write(packed.data(), packed.size());

	return body.str();
}

void read_vehicles(std::istream &Stream)
{
	auto const count = sn_utils::ld_uint32(Stream);
	uint32_t repositioned{0};
	uint32_t missing{0};

	for (uint32_t i = 0; i < count; ++i)
	{
		auto const name = sn_utils::d_str(Stream);
		auto const trackname = sn_utils::d_str(Stream);
		auto const translation = sn_utils::ld_float64(Stream);
		auto const axlefirst = sn_utils::d_uint8(Stream);
		auto const direction = sn_utils::d_uint8(Stream);
		auto const position = sn_utils::d_dvec3(Stream);

		auto const velocity = sn_utils::ld_float64(Stream);
		auto const velocitykmh = sn_utils::ld_float64(Stream);
		auto const diractive = sn_utils::ld_int32(Stream);
		auto const mainctrl = sn_utils::ld_int32(Stream);
		auto const scndctrl = sn_utils::ld_int32(Stream);
		auto const brakectrl = sn_utils::ld_int32(Stream);
		auto const localbrake = sn_utils::ld_float64(Stream);
		auto const pipepress = sn_utils::ld_float64(Stream);
		auto const brakepress = sn_utils::ld_float64(Stream);
		auto const scndpipepress = sn_utils::ld_float64(Stream);
		auto const compressor = sn_utils::ld_float64(Stream);
		auto const compressedvolume = sn_utils::ld_float64(Stream);
		auto const aiactive = sn_utils::d_bool(Stream);

		TDynamicObject *vehicle = simulation::Vehicles.find(name);
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
		{
			++missing;
			continue;
		}

		auto &mover = *vehicle->MoverParameters;

		mover.V = velocity;
		mover.Vel = velocitykmh;
		mover.DirActive = diractive;
		mover.MainCtrlPos = mainctrl;
		mover.ScndCtrlPos = scndctrl;
		mover.BrakeCtrlPos = brakectrl;
		mover.LocalBrakePosA = localbrake;
		mover.PipePress = pipepress;
		mover.BrakePress = brakepress;
		mover.ScndPipePress = scndpipepress;
		mover.Compressor = compressor;
		mover.CompressedVolume = compressedvolume;

		if (vehicle->Mechanik != nullptr && vehicle->Mechanik->AIControllFlag != aiactive)
		{
			vehicle->Mechanik->TakeControl(aiactive);
		}

		if (glm::length(vehicle->GetPosition() - position) <= REPOSITION_THRESHOLD)
		{
			// close enough already; leaving it alone avoids shaking a vehicle that is fine
			continue;
		}

		TTrack *track = (trackname.empty() ? nullptr : simulation::Paths.find(trackname));
		if (track == nullptr)
			continue;

		// place_on_track() takes the distance of the vehicle's nose, while what we recorded
		// is the offset of its leading bogie. this inverts the arithmetic the placement
		// itself does, so the two stay in step if that code ever changes
		double const half = vehicle->fAxleDist * 0.5;
		double const axleoffset = (axlefirst ? -half : half);
		double const sign = (direction ? 1.0 : -1.0);
		double const nose = (translation - axleoffset) * sign + 0.5 * mover.Dim.L;

		vehicle->place_on_track(track, nose, false);
		++repositioned;
	}

	WriteLog("net: snapshot restored " + std::to_string(count) + " vehicles, " + std::to_string(repositioned) + " of them put back on the rails" +
	             (missing > 0 ? ", " + std::to_string(missing) + " unknown here" : ""),
	         logtype::net);
}

// ---------------------------------------------------------------------------
// crews

std::string write_crews()
{
	std::ostringstream body;
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

} // namespace

std::string network::take_snapshot()
{
	std::ostringstream stream;

	sn_utils::ls_uint32(stream, SNAPSHOT_MAGIC);
	sn_utils::ls_uint32(stream, SNAPSHOT_VERSION);
	sn_utils::ls_uint64(stream, Global.simulation_tick);
	sn_utils::ls_uint32(stream, 3); // number of chunks that follow

	write_chunk(stream, SNAPSHOT_SESSION, write_session());
	write_chunk(stream, SNAPSHOT_VEHICLES, write_vehicles());
	write_chunk(stream, SNAPSHOT_CREWS, write_crews());

	auto const blob = stream.str();
	WriteLog("net: snapshot taken at tick " + std::to_string(Global.simulation_tick) + ", " + std::to_string(blob.size()) + " bytes", logtype::net);

	return blob;
}

bool network::apply_snapshot(std::string const &Blob)
{
	std::istringstream stream(Blob);

	if (sn_utils::ld_uint32(stream) != SNAPSHOT_MAGIC)
	{
		ErrorLog("net: snapshot rejected, not a snapshot", logtype::net);
		return false;
	}

	auto const version = sn_utils::ld_uint32(stream);
	if (version != SNAPSHOT_VERSION)
	{
		ErrorLog("net: snapshot rejected, version " + std::to_string(version) + " but this build speaks " + std::to_string(SNAPSHOT_VERSION), logtype::net);
		return false;
	}

	auto const tick = sn_utils::ld_uint64(stream);
	auto const chunks = sn_utils::ld_uint32(stream);

	for (uint32_t i = 0; i < chunks; ++i)
	{
		if (!stream.good())
		{
			ErrorLog("net: snapshot truncated", logtype::net);
			return false;
		}

		auto const id = sn_utils::ld_uint16(stream);
		auto const length = sn_utils::ld_uint32(stream);

		std::string body(length, '\0');
		stream.read(body.data(), length);

		std::istringstream chunk(body);

		switch (id)
		{
		case SNAPSHOT_SESSION:
			read_session(chunk);
			break;
		case SNAPSHOT_VEHICLES:
			read_vehicles(chunk);
			break;
		case SNAPSHOT_CREWS:
			read_crews(chunk);
			break;
		default:
			// a section this build knows nothing about; its length is right there, so it
			// costs nothing to walk past it
			WriteLog("net: snapshot section " + std::to_string(id) + " skipped, unknown to this build", logtype::net);
			break;
		}
	}

	WriteLog("net: snapshot applied, now at tick " + std::to_string(tick), logtype::net);

	return true;
}

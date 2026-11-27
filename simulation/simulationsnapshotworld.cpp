/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "simulation/simulationsnapshotworld.h"

#include "McZapkie/MOVER.h"
#include "application/uitranscripts.h"
#include "model/AnimModel.h"
#include "simulation/simulation.h"
#include "simulation/simulationenvironment.h"
#include "simulation/simulationtest.h"
#include "simulation/simulationtime.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"
#include "utilities/utilities.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "world/EvLaunch.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "world/Track.h"
#include "world/TractionPower.h"

#include <algorithm>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace simulation::snapshot {

namespace {

// the cells the simulation itself writes each frame: the weather, the time and the date for the scripts
constexpr std::string_view scripting_cells{"__simulation."};
// a speed limit beyond any a track has: a value of the file past it is refused
constexpr double big_velocity{1e6};
// the most events sharing a name a snapshot looks among, the most seconds an event waits in the queue
constexpr int most_siblings{100000};
constexpr double longest_wait{1e9};

// the place of each thing of a table, to name the ones without a name of their own
template <typename Item_> class table_index {

public:
	// the place of the item in the sequence, or the size of the sequence for an item which isn't there
	template <typename Sequence_> std::size_t of(Sequence_ const &Sequence, Item_ const *Item)
	{
		if (auto const place{known(Sequence, Item)}; place < Sequence.size())
			return place;
		// the table has grown, or it's the table of another scenery now
		m_places.clear();
		std::size_t place{0};
		for (auto const *item : Sequence)
			m_places.emplace(item, place++);
		return known(Sequence, Item);
	}

private:
	template <typename Sequence_> std::size_t known(Sequence_ const &Sequence, Item_ const *Item) const
	{
		auto const lookup{m_places.find(Item)};
		return lookup != m_places.end() && lookup->second < Sequence.size() && Sequence[lookup->second] == Item ? lookup->second : Sequence.size();
	}

	std::unordered_map<Item_ const *, std::size_t> m_places;
};

// a thing of a table by its reference: its name, or its place
template <typename Table_> auto find_in(Table_ const &Table, std::string const &Reference) -> decltype(Table.find(Reference))
{
	if (auto const index{reference_index(Reference)}; index.has_value())
		return *index < Table.sequence().size() ? Table.sequence()[*index] : nullptr;
	return Table.find(Reference);
}

template <typename Table_, typename Item_> std::string reference_in(Table_ const &Table, table_index<Item_> &Index, Item_ const *Item)
{
	if (Item == nullptr)
		return {};
	auto const place{Index.of(Table.sequence(), Item)};
	if (place >= Table.sequence().size())
		return {};
	return reference(Item->name(), place, Table.find(Item->name()) == Item);
}

table_index<TTrack> &track_index()
{
	static table_index<TTrack> index;
	return index;
}

table_index<TDynamicObject> &vehicle_index()
{
	static table_index<TDynamicObject> index;
	return index;
}

table_index<TTractionPowerSource> &powersource_index()
{
	static table_index<TTractionPowerSource> index;
	return index;
}

table_index<TAnimModel> &model_index()
{
	static table_index<TAnimModel> index;
	return index;
}

// a track the simulation made on the run, where a track of the scenery ends: the loop a road vehicle turns back on,
// the track a train runs off the rails along (TTrack::NullCreate())
bool made_on_the_run(TTrack const &Track)
{
	return (Track.iCategoryFlag & 0x80) != 0;
}

// what the scenery is made of, by which a snapshot tells whether it's put over the scenery it was taken in
void capture_fingerprint(record &Header)
{
	name_hash names;
	auto switches{0};
	auto tracks{0};
	for (auto const *track : Paths.sequence())
	{
		// the tracks made on the run aren't the scenery's
		if (track == nullptr || made_on_the_run(*track))
			continue;
		++tracks;
		names.add(track->name());
		if (track->eType == tt_Switch)
			++switches;
	}
	for (auto const *cell : Memory.sequence())
		names.add(cell != nullptr ? cell->name() : std::string{});
	for (auto const *scenery_event : Events.sequence())
		names.add(scenery_event != nullptr ? scenery_event->m_name : std::string{});
	for (auto const *vehicle : Vehicles.sequence())
		names.add(vehicle != nullptr ? vehicle->name() : std::string{});
	Header.set("tracks", tracks);
	Header.set("switches", switches);
	Header.set("memcells", Memory.sequence().size());
	Header.set("events", Events.sequence().size());
	Header.set("launchers", Events.launchers().size());
	Header.set("vehicles", Vehicles.sequence().size());
	Header.set("names", names.text());
}

void capture_header(document &Document)
{
	auto &header{Document.add("snapshot", std::to_string(format_version))};
	header.set("scenery", Global.SceneryFile);
	header.set("seed", Global.random_seed);
	header.set("timer", Timer::GetTime());
	capture_fingerprint(header);
}

void capture_time(document &Document)
{
	auto const &time{Time.data()};
	auto &entry{Document.add("time", "now")};
	entry.set("year", time.wYear).set("month", time.wMonth).set("day", time.wDay).set("weekday", time.wDayOfWeek);
	entry.set("hour", time.wHour).set("minute", time.wMinute).set("second", time.wSecond).set("milliseconds", Time.milliseconds());
}

bool apply_time(record const &Entry)
{
	// the month and the day pick entries of the tables of the clock: a date which isn't one is refused whole
	auto time{Time.data()};
	if (false == Entry.get("year", time.wYear, WORD{1}, WORD{9999}) && Entry.has("year"))
		return false;
	if (false == Entry.get("month", time.wMonth, WORD{1}, WORD{12}) && Entry.has("month"))
		return false;
	if (constexpr std::array<WORD, 13> days{0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	    false == Entry.get("day", time.wDay, WORD{1}, days[time.wMonth]) && Entry.has("day"))
		return false;
	if (time.wMonth == 2 && time.wDay == 29 && false == ((time.wYear % 4 == 0 && time.wYear % 100 != 0) || time.wYear % 400 == 0))
	{
		Entry.refuse("day", "the year isn't a leap one");
		return false;
	}
	auto milliseconds{Time.milliseconds()};
	if ((false == Entry.get("weekday", time.wDayOfWeek, WORD{0}, WORD{6}) && Entry.has("weekday")) ||
	    (false == Entry.get("hour", time.wHour, WORD{0}, WORD{23}) && Entry.has("hour")) ||
	    (false == Entry.get("minute", time.wMinute, WORD{0}, WORD{59}) && Entry.has("minute")) ||
	    (false == Entry.get("second", time.wSecond, WORD{0}, WORD{59}) && Entry.has("second")) ||
	    (false == Entry.get("milliseconds", milliseconds, 0.0, 999.999999) && Entry.has("milliseconds")))
		return false;
	Time.restore(time, milliseconds);
	// the season goes by the date, and the weather by the season
	Environment.compute_season(Time.year_day());
	return true;
}

void capture_weather(document &Document)
{
	auto &entry{Document.add("weather", "now")};
	entry.set("overcast", Global.Overcast).set("fog", Global.fFogEnd).set("temperature", Global.AirTemperature);
	// the wind: it changes now and then, by numbers drawn at random
	std::vector<double> wind;
	for (auto const value : Environment.wind_state())
		wind.emplace_back(value);
	entry.set("wind", numbers_text(wind));
}

void apply_weather(record const &Entry)
{
	// the weather is drawn from these: a number which isn't one has no place in it
	constexpr auto big{1e9f};
	Entry.get("overcast", Global.Overcast, 0.f, 10.f);
	Entry.get("fog", Global.fFogEnd, 0.f, big);
	Entry.get("temperature", Global.AirTemperature, -100.f, 100.f);
	Environment.compute_weather();
	auto state{Environment.wind_state()};
	if (auto const wind{text_numbers(Entry.text("wind"))}; wind.size() == state.size())
	{
		if (std::ranges::all_of(wind, [](double const Value) { return std::isfinite(Value) && std::abs(Value) < 1e9; }))
		{
			std::ranges::transform(wind, state.begin(), [](double const Value) { return static_cast<float>(Value); });
			Environment.wind_state(state);
		}
		else
			Entry.refuse("wind", "a number of it isn't finite");
	}
}

// the sources of the traction power: the voltage an event may have set, the load of the step before, the fuses
void capture_power(document &Document)
{
	// what a source which has fed nothing yet says of itself, but for its voltage
	std::vector const idle{0.0, 1e-10, 1e-10, 0.0, 0.0, 0.0, 0.0, 0.0};
	for (auto const *source : Powergrid.sequence())
	{
		if (source == nullptr)
			continue;
		auto const state{source->state()};
		// the sources made for single wires are many, and most of them idle
		if (source->IsAutogenerated && std::ranges::equal(state.begin() + 1, state.end(), idle.begin(), idle.end()))
			continue;
		if (auto const name{reference_in(Powergrid, powersource_index(), source)}; false == name.empty())
			Document.add("powersource", name).set("state", numbers_text(state));
	}
}

bool apply_power(record const &Entry)
{
	auto *source{find_in(Powergrid, Entry.name)};
	if (source == nullptr)
		return false;
	auto const state{text_numbers(Entry.text("state"))};
	// the last one counts the times the fuse blew
	if (false == state.empty() && false == whole_number(state.back(), 0, std::numeric_limits<int>::max()).has_value())
	{
		Entry.refuse("state", "the count of its fuse isn't a whole number");
		return true;
	}
	source->state(state);
	return true;
}

// the engine the simulation draws its random numbers from, as the numbers of its state, eight to a line
void capture_random(document &Document)
{
	std::ostringstream state;
	state << Global.random_engine;
	std::istringstream numbers{state.str()};
	std::string text;
	std::string number;
	auto count{0};
	while (numbers >> number)
	{
		if (false == text.empty())
			text += (count % 8 == 0 ? "\n    " : " ");
		text += number;
		++count;
	}
	Document.add("random", "engine").set("state", text);
}

bool apply_random(record const &Entry)
{
	std::istringstream state{Entry.text("state")};
	auto engine{Global.random_engine}; // NOSONAR: a copy of the simulation's reproducible engine, which nothing secure uses
	state >> engine;
	if (state.fail())
		return false;
	Global.random_engine = engine;
	return true;
}

void capture_memcells(document &Document)
{
	auto const &cells{Memory.sequence()};
	for (std::size_t index{0}; index < cells.size(); ++index)
	{
		auto const *cell{cells[index]};
		if (cell == nullptr || cell->name().starts_with(scripting_cells))
			continue;
		auto &entry{Document.add("memcell", reference(cell->name(), index, Memory.find(cell->name()) == cell))};
		entry.set("text", cell->Text()).set("value1", cell->Value1()).set("value2", cell->Value2());
		if (false == cell->StopCommand())
		{
			// the command of the cell isn't waiting to be sent to a train: it's not a command, or it was sent already
			entry.set("sent", true);
		}
	}
}

bool apply_memcell(record const &Entry)
{
	auto *cell{find_in(Memory, Entry.name)};
	if (cell == nullptr)
		return false;
	cell->UpdateValues(Entry.text("text", cell->Text()), Entry.value("value1", cell->Value1()), Entry.value("value2", cell->Value2()),
	                   basic_event::flags::text | basic_event::flags::value1 | basic_event::flags::value2);
	if (Entry.value("sent", false))
		cell->StopCommandSent();
	return true;
}

void capture_tracks(document &Document)
{
	// what an event can change of a track: bit 0x100 of iAction marks the damage, 0x200 the speed limit
	constexpr int changedbyevents{0x100 | 0x200};
	for (auto *track : Paths.sequence())
	{
		if (track != nullptr && track->eType == tt_Switch)
			Document.add("switch", track_reference(track)).set("state", track->GetSwitchState()).set("velocity", track->velocity_limit());
	}
	for (auto const *track : Paths.sequence())
	{
		if (track != nullptr && track->eType != tt_Switch && (track->iAction & changedbyevents) != 0)
			Document.add("track", track_reference(track)).set("velocity", track->velocity_limit()).set("damage", track->iDamageFlag);
	}
}

// the crossings of roads: which of the ways through was taken last. the crossing has it as its current segment, by
// which a driver looking ahead measures it
void capture_crossings(document &Document)
{
	for (auto const *track : Paths.sequence())
	{
		if (track == nullptr || track->eType != tt_Cross || track->SwitchExtension == nullptr)
			continue;
		auto const &segments{track->SwitchExtension->Segments};
		for (auto index{0}; index < 6; ++index)
		{
			if (segments[index] != nullptr && segments[index] == track->Segment)
			{
				Document.add("crossing", track_reference(track)).set("segment", index);
				break;
			}
		}
	}
}

bool apply_crossing(record const &Entry)
{
	auto *track{find_track(Entry.name)};
	if (track == nullptr || track->eType != tt_Cross || track->SwitchExtension == nullptr)
		return false;
	auto const index{Entry.value("segment", -1, 0, 5)};
	if (index < 0 || track->SwitchExtension->Segments[index] == nullptr)
		return false;
	track->Segment = track->SwitchExtension->Segments[index];
	return true;
}

// the tracks made on the run, each by the track of the scenery it was made at and the end of it. they come back in
// the order they were made in, which gives them their places in the table again
void capture_made_tracks(document &Document)
{
	for (auto const *made : Paths.sequence())
	{
		if (made == nullptr || false == made_on_the_run(*made))
			continue;
		for (auto const *track : Paths.sequence())
		{
			if (track == nullptr || made_on_the_run(*track) || (track->trPrev != made && track->trNext != made))
				continue;
			Document.add("madetrack", track_reference(track)).set("end", track->trPrev == made ? 0 : 1);
			break;
		}
	}
}

bool apply_made_track(record const &Entry)
{
	auto *track{find_track(Entry.name)};
	if (track == nullptr)
		return false;
	auto end{0};
	if (false == Entry.get("end", end, 0, 1) && Entry.has("end"))
		return false;
	// made anew unless the scenery has something there by now
	if ((end == 0 ? track->trPrev : track->trNext) == nullptr)
		track->NullCreate(end);
	return true;
}

bool apply_switch(record const &Entry)
{
	auto *track{find_track(Entry.name)};
	if (track == nullptr || track->eType != tt_Switch)
		return false;
	if (auto state{0}; Entry.get("state", state, 0, 1) && state != track->GetSwitchState())
		track->Switch(state);
	if (auto velocity{0.0}; Entry.get("velocity", velocity, -big_velocity, big_velocity))
		track->velocity_limit(velocity);
	return true;
}

bool apply_track(record const &Entry)
{
	auto *track{find_track(Entry.name)};
	if (track == nullptr)
		return false;
	if (auto velocity{0.0}; Entry.get("velocity", velocity, -big_velocity, big_velocity))
		track->velocity_limit(velocity);
	Entry.get("damage", track->iDamageFlag);
	return true;
}

void capture_events(document &Document)
{
	auto const now{Timer::GetTime()};
	for (auto const *queued{Events.begin()}; queued != nullptr; queued = queued->m_next)
	{
		auto &entry{Document.add("event", queued->m_name)};
		if (auto const sibling{event_sibling(queued)}; sibling != 0)
			entry.set("sibling", sibling);
		// when it runs, by the clock of the simulation; the seconds left are for the reader of the file
		entry.set("at", queued->m_launchtime).set("left", queued->m_launchtime - now);
		if (queued->m_activator != nullptr)
			entry.set("activator", vehicle_reference(queued->m_activator));
	}
}

bool apply_event(record const &Entry)
{
	auto sibling{0};
	if (false == Entry.get("sibling", sibling, 0, most_siblings) && Entry.has("sibling"))
		return false;
	auto *found{find_event(Entry.name, sibling)};
	if (found == nullptr)
		return false;
	// without the time it ran at, the seconds it had left count from now. the queue is kept in the order of the times,
	// which a time that isn't a number has none in
	auto const now{Timer::GetTime()};
	auto launchtime{now + Entry.value("left", 0.0, -longest_wait, longest_wait)};
	if (false == Entry.get("at", launchtime, now - longest_wait, now + longest_wait) && Entry.has("at"))
		return false;
	auto const *activator{Entry.has("activator") ? find_vehicle(Entry.text("activator")) : nullptr};
	return Events.restore_queued(found, activator, launchtime);
}

void capture_launchers(document &Document)
{
	auto const launchers{Events.launchers()};
	std::map<std::string, int, std::less<>> named;
	for (auto const *launcher : launchers)
	{
		if (launcher != nullptr)
			++named[launcher->name()];
	}
	for (std::size_t index{0}; index < launchers.size(); ++index)
	{
		auto const *launcher{launchers[index]};
		// a launcher which hasn't counted any time yet is as the scenery makes it
		if (launcher == nullptr || launcher->timer() == 0.0)
			continue;
		Document.add("launcher", reference(launcher->name(), index, named[launcher->name()] == 1 && launcher->name() != "none")).set("timer", launcher->timer());
	}
}

bool apply_launcher(record const &Entry, std::vector<TEventLauncher *> const &Launchers)
{
	TEventLauncher *launcher{nullptr};
	if (auto const index{reference_index(Entry.name)}; index.has_value())
		launcher = (*index < Launchers.size() ? Launchers[*index] : nullptr);
	else if (auto const lookup{std::ranges::find_if(Launchers, [&](TEventLauncher const *Item) { return Item != nullptr && Item->name() == Entry.name; })};
	         lookup != Launchers.end())
		launcher = *lookup;
	if (launcher == nullptr)
		return false;
	launcher->timer(Entry.value("timer", launcher->timer(), 0.0, longest_wait));
	return true;
}

// the models whose lights the events of the scenery switch: the signals, mostly
std::vector<TAnimModel *> switched_models()
{
	std::vector<TAnimModel *> models;
	std::unordered_set<scene::basic_node const *> seen;
	for (auto const *scenery_event : Events.sequence())
	{
		if (dynamic_cast<lights_event const *>(scenery_event) == nullptr)
			continue;
		for (auto *node : scenery_event->target_nodes())
		{
			if (seen.emplace(node).second)
				models.emplace_back(static_cast<TAnimModel *>(node));
		}
	}
	return models;
}

void capture_lights(document &Document)
{
	for (auto *model : switched_models())
	{
		auto const name{reference_in(Instances, model_index(), static_cast<TAnimModel const *>(model))};
		if (name.empty())
			continue;
		record entry{"model", name};
		for (auto light{0}; light < iMaxNumLights; ++light)
		{
			if (auto const state{model->LightGet(light)}; state.has_value())
				entry.set(std::format("light{}", light), std::get<0>(*state));
		}
		if (false == entry.fields.empty())
			Document.records.emplace_back(std::move(entry));
	}
}

bool apply_lights(record const &Entry)
{
	auto *model{find_in(Instances, Entry.name)};
	if (model == nullptr)
		return false;
	for (auto light{0}; light < iMaxNumLights; ++light)
	{
		if (auto state{0.f}; Entry.get(std::format("light{}", light), state, -1e6f, 1e6f))
			model->LightSet(light, state);
	}
	return true;
}

void capture_vehicles(document &Document)
{
	for (auto const *vehicle : Vehicles.sequence())
	{
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
			continue;
		vehicle->snapshot_save(Document.add("vehicle", vehicle_reference(vehicle)));
	}
}

// the couplings as the vehicles have them now
std::vector<coupling_link> current_couplings(std::unordered_map<TMoverParameters const *, TDynamicObject *> const &Owners)
{
	std::vector<coupling_link> links;
	for (auto const *vehicle : Vehicles.sequence())
	{
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
			continue;
		for (auto const side : {end::front, end::rear})
		{
			auto const &joint{vehicle->MoverParameters->Couplers[side]};
			auto const owner{joint.Connected != nullptr ? Owners.find(joint.Connected) : Owners.end()};
			if (owner == Owners.end() || joint.CouplingFlag == coupling::faux)
				continue;
			links.emplace_back(vehicle_reference(vehicle), side, vehicle_reference(owner->second), joint.ConnectedNr, joint.CouplingFlag);
		}
	}
	return links;
}

static_assert(coupling_flags_mask == (coupling::power3x400v << 1) - 1, "the flags a snapshot takes are every bit of the coupling enum");

void part(TDynamicObject &Vehicle, int const End)
{
	auto &joint{Vehicle.MoverParameters->Couplers[End]};
	if (joint.Connected != nullptr && joint.ConnectedNr >= 0 && joint.ConnectedNr < 2)
	{
		auto &other{joint.Connected->Couplers[joint.ConnectedNr]};
		other.Connected = nullptr;
		other.ConnectedNr = -1;
		other.CouplingFlag = coupling::faux;
	}
	joint.Connected = nullptr;
	joint.ConnectedNr = -1;
	joint.CouplingFlag = coupling::faux;
}

// whether the vehicle is there, with its physics
bool with_physics(TDynamicObject const *Vehicle)
{
	return Vehicle != nullptr && Vehicle->MoverParameters != nullptr;
}

// the flags of the link as the two vehicles can have them now: no control between an electric and a diesel locomotive
// (eu07/maszyna#520), whatever an edited file or a vehicle definition changed since the snapshot says
int compatible_flags(TDynamicObject const &Vehicle, TDynamicObject const &Other, coupling_link const &Link)
{
	return Vehicle.MoverParameters->CompatibleCoupling(Link.end, *Other.MoverParameters, Link.otherend, Link.flags);
}

// the links of the plan which change the kind of a coupling the vehicles keep
void change_couplings(std::vector<coupling_link> const &Links)
{
	for (auto const &link : Links)
	{
		auto const *vehicle{find_vehicle(link.vehicle)};
		auto const *other{find_vehicle(link.other)};
		if (false == with_physics(vehicle) || false == with_physics(other))
			continue;
		auto const flags{compatible_flags(*vehicle, *other, link)};
		if (flags == coupling::faux)
			continue;
		vehicle->MoverParameters->Couplers[link.end].CouplingFlag = flags;
		other->MoverParameters->Couplers[link.otherend].CouplingFlag = flags;
	}
}

// the links of the plan to make. returns: how many couldn't be made
int join_couplings(std::vector<coupling_link> const &Links)
{
	auto failed{0};
	for (auto const &link : Links)
	{
		auto *vehicle{find_vehicle(link.vehicle)};
		auto *other{find_vehicle(link.other)};
		if (false == with_physics(vehicle) || false == with_physics(other) ||
		    false == vehicle->MoverParameters->Attach(link.end, link.otherend, other->MoverParameters, link.flags, true, false))
		{
			++failed;
			continue;
		}
		// the vehicles joined in the snapshot, whatever the scenery made of their couplers, as far as they can be
		auto const flags{compatible_flags(*vehicle, *other, link)};
		vehicle->MoverParameters->Couplers[link.end].CouplingFlag = flags;
		other->MoverParameters->Couplers[link.otherend].CouplingFlag = flags;
	}
	return failed;
}

void report_couplings(coupling_plan const &Plan, int const Failed, apply_summary &Summary)
{
	if (false == Plan.part.empty() || false == Plan.join.empty() || false == Plan.change.empty())
		Summary.remarks.emplace_back(std::format("couplings: {} parted, {} joined, {} changed", Plan.part.size(), static_cast<int>(Plan.join.size()) - Failed, Plan.change.size()));
	if (Failed != 0 || false == Plan.missing.empty())
		Summary.notes.emplace_back(std::format("couplings: {} of the snapshot couldn't be made, {} are to vehicles the scenery doesn't have or which stay where it put them",
		                                       Failed, Plan.missing.size()));
	if (false == Plan.refused.empty())
	{
		Summary.notes.emplace_back(std::format("couplings: {} of the snapshot refused, they'd close a ring of vehicles; the first one {} {} to {} {}", Plan.refused.size(),
		                                       Plan.refused.front().vehicle, Plan.refused.front().end, Plan.refused.front().other, Plan.refused.front().otherend));
	}
}

// each vehicle knows the one coupled to it as its neighbour at that end
void find_neighbours(std::unordered_map<TMoverParameters const *, TDynamicObject *> const &Owners)
{
	for (auto *vehicle : Vehicles.sequence())
	{
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
			continue;
		for (auto const side : {end::front, end::rear})
		{
			auto const &joint{vehicle->MoverParameters->Couplers[side]};
			auto &neighbour{vehicle->MoverParameters->Neighbours[side]};
			neighbour = neighbour_data();
			if (auto const owner{joint.Connected != nullptr ? Owners.find(joint.Connected) : Owners.end()}; owner != Owners.end())
			{
				neighbour.vehicle = owner->second;
				neighbour.vehicle_end = joint.ConnectedNr;
			}
		}
	}
	for (auto *vehicle : Vehicles.sequence())
	{
		if (vehicle != nullptr && vehicle->MoverParameters != nullptr && vehicle->bEnabled)
			vehicle->update_neighbours();
	}
}

// the vehicles of the snapshot which stay as the scenery made them: the ones it couldn't put where the snapshot has
// them, and the road traffic waiting at its spawn point
using vehicle_set = std::unordered_set<TDynamicObject const *>;

// parts and joins the vehicles of the scenery until they're coupled the way the snapshot has them. the vehicles Left
// aren't coupled to the ones put where the snapshot has them: the links to them are missing
void apply_couplings(std::vector<record const *> const &Entries, vehicle_set const &Left, apply_summary &Summary)
{
	std::unordered_map<TMoverParameters const *, TDynamicObject *> owners;
	std::vector<std::string> present;
	for (auto *vehicle : Vehicles.sequence())
	{
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
			continue;
		owners.emplace(vehicle->MoverParameters, vehicle);
		if (false == Left.contains(vehicle))
			present.emplace_back(vehicle_reference(vehicle));
	}
	std::vector<record const *> entries;
	std::ranges::copy_if(Entries, std::back_inserter(entries), [&Left](record const *Entry) { return false == Left.contains(find_vehicle(Entry->name)); });
	auto const plan{plan_couplings(current_couplings(owners), read_couplings(entries), present)};

	// the vehicles of the links are of the scenery (Present), with their physics
	for (auto const &link : plan.part)
	{
		if (auto *vehicle{find_vehicle(link.vehicle)}; with_physics(vehicle))
			part(*vehicle, link.end);
	}
	change_couplings(plan.change);
	auto const failed{join_couplings(plan.join)};
	report_couplings(plan, failed, Summary);
	find_neighbours(owners);
}

// a vehicle the scenery keeps out of the simulation until its time comes: a car a road spawn point (roadpoint_node) made
// as the scenery loaded, which waits off the road to be put on it. the snapshot doesn't bring one which was on the
// road back: the spawn point puts its cars on the road and takes them back to its set, and a car put there by the
// snapshot would be one the point doesn't know is out
bool waits_off_the_road(TDynamicObject const &Vehicle)
{
	return false == Vehicle.bEnabled && Vehicle.MyTrack == nullptr;
}

// the vehicles of the snapshot's entries
struct vehicle_entries {
	std::vector<std::pair<TDynamicObject *, record const *>> known; // to put where the snapshot has them
	vehicle_set listed; // every vehicle of the scenery the snapshot has
	vehicle_set left; // the ones of them left as the scenery made them
};

vehicle_entries read_vehicles(std::vector<record const *> const &Entries, apply_summary &Summary)
{
	vehicle_entries result;
	auto traffic{0};
	for (auto const *entry : Entries)
	{
		auto *vehicle{find_vehicle(entry->name)};
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
		{
			++Summary.skipped;
			Summary.notes.emplace_back(std::format("vehicle {} of the snapshot isn't in the scenery", entry->name));
			continue;
		}
		// a vehicle placed twice would have its axles counted out of their sections twice
		if (false == result.listed.emplace(vehicle).second)
		{
			++Summary.skipped;
			Summary.notes.emplace_back(std::format("vehicle {} has more than one entry in the snapshot, the first one is taken", entry->name));
			continue;
		}
		if (false == waits_off_the_road(*vehicle))
		{
			result.known.emplace_back(vehicle, entry);
			continue;
		}
		// off the road in the snapshot too, or a car of the road traffic which isn't brought back
		result.left.emplace(vehicle);
		if (entry->value("removed", false))
		{
			++Summary.entries;
			continue;
		}
		++Summary.skipped;
		++traffic;
	}
	if (traffic != 0)
		Summary.notes.emplace_back(std::format("{} cars of the road traffic of the snapshot aren't put back, their spawn points send cars anew", traffic));
	return result;
}

// puts the vehicles of the snapshot in place. returns: the vehicles of the snapshot left as the scenery made them
vehicle_set apply_vehicles(document const &Document, apply_summary &Summary)
{
	auto const entries{Document.of("vehicle")};
	auto vehicles{read_vehicles(entries, Summary)};
	auto const &known{vehicles.known};
	auto const &listed{vehicles.listed};
	auto &left{vehicles.left};
	// where the vehicles are, then what they're coupled to, then what they're doing. a vehicle which can't be put where
	// the snapshot has it is left out of the rest: coupled to vehicles put elsewhere, it'd be torn off them
	auto misplaced{0};
	for (auto const &[vehicle, entry] : known)
	{
		if (false == vehicle->snapshot_place(*entry))
		{
			++misplaced;
			left.emplace(vehicle);
		}
	}
	if (misplaced != 0)
		Summary.notes.emplace_back(std::format("{} vehicles of the snapshot couldn't be put where it has them, they stay as the scenery made them", misplaced));
	apply_couplings(entries, left, Summary);
	for (auto const &[vehicle, entry] : known)
	{
		if (left.contains(vehicle))
		{
			++Summary.skipped;
			continue;
		}
		vehicle->snapshot_load(*entry);
		++Summary.entries;
	}
	// the vehicles the snapshot doesn't know stay as the scenery made them
	if (auto const unknown{std::ranges::count_if(Vehicles.sequence(), [&](TDynamicObject const *Vehicle) { return Vehicle != nullptr && false == listed.contains(Vehicle); })};
	    unknown != 0)
		Summary.notes.emplace_back(std::format("{} vehicles of the scenery aren't in the snapshot, they stay as the scenery made them", unknown));
	return std::move(left);
}

// a hash of the text of a name, which finds a string key by any kind of string
struct name_hash {
	using is_transparent = void;
	std::size_t operator()(std::string_view const Name) const noexcept { return std::hash<std::string_view>{}(Name); }
};

using isolated_map = std::unordered_map<std::string, TIsolated *, name_hash, std::equal_to<>>;

// the isolated sections, by their names: the scenery makes one for each name (TIsolated::Find())
isolated_map isolated_sections()
{
	isolated_map sections;
	for (auto *section{TIsolated::Root()}; section != nullptr; section = section->Next())
		sections.try_emplace(section->asName, section);
	return sections;
}

// how many axles each isolated section counts. in a running simulation it's the vehicles on its tracks, but not only:
// the vehicles of a consist which follow its first one out through a portal leave the simulation where they are, and
// stay counted there. a snapshot keeps the counts as they are
void capture_isolated(document &Document)
{
	for (auto *section{TIsolated::Root()}; section != nullptr; section = section->Next())
		Document.add("isolated", section->asName).set("axles", section->axles());
}

// sets the counts of the isolated sections to those of the snapshot, after the vehicles are put on their tracks: which
// counts them anew. a section whose count comes out other than the snapshot has is told of
void apply_isolated(document const &Document, apply_summary &Summary)
{
	auto const entries{Document.of("isolated")};
	if (entries.empty())
		return;
	auto const sections{isolated_sections()};
	auto missing{0};
	auto differing{0};
	std::string example;
	for (auto const *entry : entries)
	{
		auto const lookup{sections.find(entry->name)};
		if (lookup == sections.end())
		{
			++Summary.skipped;
			++missing;
			continue;
		}
		// a count beyond any the vehicles of a scenery make
		auto axles{0};
		if (false == entry->get("axles", axles, -1000000, 1000000))
		{
			++Summary.skipped;
			continue;
		}
		if (auto *section{lookup->second}; section->axles() != axles)
		{
			if (differing++ == 0)
				example = std::format("{}: {} axles, the snapshot has {}", section->asName, section->axles(), axles);
			section->axles(axles);
		}
		++Summary.entries;
	}
	if (missing != 0)
		Summary.notes.emplace_back(std::format("{} isolated sections of the snapshot aren't in the scenery", missing));
	// not a fault as such: the vehicles a consist leaves behind as it goes through a portal stay counted where they were
	if (differing != 0)
		Summary.remarks.emplace_back(std::format("{} isolated sections counted other axles with the vehicles placed than the snapshot has, set to its counts; the first one {}",
		                                       differing, example));
}

void capture_drivers(document &Document)
{
	for (auto const *vehicle : Vehicles.sequence())
	{
		if (vehicle != nullptr && vehicle->bEnabled && vehicle->Mechanik != nullptr)
			vehicle->Mechanik->snapshot_save(Document.add("driver", vehicle_reference(vehicle)));
	}
}

// a driver of the consist of the vehicle whom the snapshot doesn't have where the scenery put them: the one to move
// to the vehicle, where the snapshot has a driver and the scenery none (a driver changes ends of a train)
TController *spare_driver(TDynamicObject *Vehicle, std::unordered_set<TDynamicObject const *> const &Taken)
{
	TController *found{nullptr};
	Vehicle->for_each(coupling::coupler, [&found, &Taken](TDynamicObject const *Coupled) {
		if (found == nullptr && Coupled->Mechanik != nullptr && false == Taken.contains(Coupled))
			found = Coupled->Mechanik;
	});
	return found;
}

// the drivers of the snapshot, in the vehicles it has them in. the vehicles Left keep the drivers the scenery gave them
void apply_drivers(document const &Document, vehicle_set const &Left, apply_summary &Summary)
{
	auto const entries{Document.of("driver")};
	if (entries.empty())
		return;
	// the vehicles the snapshot has drivers in
	std::unordered_set<TDynamicObject const *> taken;
	for (auto const *entry : entries)
		taken.emplace(find_vehicle(entry->name));
	auto missing{0};
	for (auto const *entry : entries)
	{
		auto *vehicle{find_vehicle(entry->name)};
		if (Left.contains(vehicle))
		{
			++Summary.skipped;
			continue;
		}
		if (vehicle == nullptr)
		{
			++Summary.skipped;
			++missing;
			continue;
		}
		if (vehicle->Mechanik == nullptr)
		{
			if (auto *spare{spare_driver(vehicle, taken)}; spare != nullptr)
				spare->MoveTo(vehicle);
		}
		if (vehicle->Mechanik == nullptr)
		{
			++Summary.skipped;
			++missing;
			continue;
		}
		vehicle->Mechanik->snapshot_load(*entry);
		++Summary.entries;
	}
	if (missing != 0)
		Summary.notes.emplace_back(std::format("{} drivers of the snapshot have no driver in the scenery to become", missing));
	// the drivers of the scenery the snapshot has no word of, in vehicles it does know, were gone when it was taken
	std::unordered_set<TDynamicObject const *> known;
	for (auto const *entry : Document.of("vehicle"))
		known.emplace(find_vehicle(entry->name));
	std::unordered_set<TController const *> deleted;
	for (auto *vehicle : Vehicles.sequence())
	{
		if (vehicle == nullptr || vehicle->Mechanik == nullptr || taken.contains(vehicle) || false == known.contains(vehicle) || Left.contains(vehicle))
			continue;
		if (vehicle->Mechanik->AIControllFlag)
		{
			deleted.emplace(vehicle->Mechanik);
			SafeDelete(vehicle->Mechanik);
		}
	}
	// the other vehicles of their consists had them in charge: those the snapshot has get their owners anew
	// (apply_owners()), but no vehicle is left pointing at a driver who's no more
	for (auto *vehicle : Vehicles.sequence())
	{
		if (vehicle != nullptr && deleted.contains(vehicle->ctOwner))
			vehicle->ctOwner = nullptr;
	}
}

// the driver in charge of each vehicle; after the drivers are where the snapshot has them. the vehicles Left keep
// theirs
void apply_owners(document const &Document, vehicle_set const &Left)
{
	for (auto const *entry : Document.of("vehicle"))
	{
		auto *vehicle{find_vehicle(entry->name)};
		if (vehicle == nullptr || Left.contains(vehicle))
			continue;
		// a vehicle gone from the simulation has nobody in charge
		if (entry->value("removed", false))
		{
			vehicle->ctOwner = nullptr;
			continue;
		}
		if (false == entry->has("first"))
			continue;
		auto const *owner{find_vehicle(entry->text("owner"))};
		vehicle->ctOwner = (owner != nullptr ? owner->Mechanik : nullptr);
	}
}

void capture_player(document &Document)
{
	if (Train == nullptr || Train->Dynamic() == nullptr)
		return;
	auto const *vehicle{Train->Dynamic()};
	// by its name, which is what the simulator is started in a vehicle with
	Document.add("player", vehicle->name()).set("cab", vehicle->MoverParameters->CabOccupied);
}

// tells whether the snapshot was taken in the scenery it's being put over. returns: false if the scenery isn't what it
// was then, which the snapshot isn't put over unless that's forced
bool check_scenery(record const &Header, bool const Force, apply_summary &Summary)
{
	if (auto const scenery{Header.text("scenery")}; scenery != Global.SceneryFile)
		Summary.notes.emplace_back(std::format(R"(the snapshot was taken in scenery "{}", this one is "{}")", scenery, Global.SceneryFile));
	if (auto const seed{Header.value("seed", Global.random_seed)}; seed != Global.random_seed)
	{
		Summary.notes.emplace_back(
		    std::format("the snapshot was taken with random seed {}, the scenery was loaded with {}: what it draws at random while it loads may differ", seed,
		                Global.random_seed));
	}
	record now;
	capture_fingerprint(now);
	auto const changed{fingerprint_changes(Header, now)};
	if (changed.empty())
		return true;
	// most tracks have no name and are named by their place: a track added early in a file of the scenery moves the
	// places of all after it, and the vehicles would be put on other tracks than they were on
	Summary.notes.emplace_back(std::format("the scenery isn't what it was when the snapshot was taken: {}. things named by their place in the scenery "
	                                       "(\"#12\") may be other ones now",
	                                       changed));
	if (Force)
		return true;
	Summary.notes.emplace_back("the snapshot isn't loaded (--load-snapshot-force loads it all the same)");
	return false;
}

// applies the entries of a type, each by the function given, and counts how it went
template <typename Function_> void apply_each(document const &Document, std::string_view const Type, apply_summary &Summary, Function_ const Function)
{
	auto missing{0};
	std::string example;
	for (auto const *entry : Document.of(Type))
	{
		if (Function(*entry))
		{
			++Summary.entries;
			continue;
		}
		++Summary.skipped;
		if (missing++ == 0)
			example = entry->name;
	}
	if (missing != 0)
		Summary.notes.emplace_back(std::format("{} {} entries of the snapshot have nothing in the scenery to go to, the first one is {}", missing, Type, example));
}

// the snapshots asked for on the command line
struct schedule {
	command_line requests;
	std::optional<document> pending; // the one to load, read ahead for its header
	std::size_t written{0}; // how many of the saves are done
};

schedule &requested()
{
	static schedule instance;
	return instance;
}

} // namespace

std::string track_reference(TTrack const *Track)
{
	return reference_in(Paths, track_index(), Track);
}

TTrack *find_track(std::string const &Reference)
{
	return Reference.empty() ? nullptr : find_in(Paths, Reference);
}

std::string vehicle_reference(TDynamicObject const *Vehicle)
{
	return reference_in(Vehicles, vehicle_index(), Vehicle);
}

TDynamicObject *find_vehicle(std::string const &Reference)
{
	return Reference.empty() ? nullptr : find_in(Vehicles, Reference);
}

int event_sibling(basic_event const *Event)
{
	auto index{0};
	for (auto const *sibling{Events.FindEvent(Event->m_name)}; sibling != nullptr && sibling != Event; sibling = sibling->m_sibling)
		++index;
	return index;
}

basic_event *find_event(std::string const &Name, int const Sibling)
{
	auto *found{Events.FindEvent(Name)};
	auto index{0};
	while (index < Sibling && found != nullptr)
	{
		found = found->m_sibling;
		++index;
	}
	return found;
}

document capture()
{
	document result;
	capture_header(result);
	capture_time(result);
	capture_weather(result);
	capture_memcells(result);
	capture_tracks(result);
	capture_crossings(result);
	capture_made_tracks(result);
	capture_power(result);
	capture_events(result);
	capture_launchers(result);
	capture_lights(result);
	capture_vehicles(result);
	capture_isolated(result);
	capture_drivers(result);
	capture_player(result);
	// the long one last, out of the way of whoever reads the file
	capture_random(result);
	// a file cut short between two entries is told by this
	result.records.front().set("entries", result.records.size());
	return result;
}

apply_summary apply(document const &Document, bool const Force)
{
	apply_summary summary;
	auto const *header{Document.header()};
	if (header == nullptr)
	{
		summary.refused = true;
		summary.notes.emplace_back("not a snapshot, nothing was loaded from it");
		return summary;
	}
	if (false == check_scenery(*header, Force, summary))
	{
		summary.refused = true;
		return summary;
	}

	// nothing in the scenery reacts to the saved state being put back: the vehicles moved to their places don't
	// set off the events of the tracks, the sections they free and take don't change their signals
	Events.suspend(true);

	if (auto timer{0.0}; header->get("timer", timer, 0.0, longest_wait))
		Timer::set_time(timer);
	apply_each(Document, "time", summary, apply_time);
	apply_each(Document, "weather", summary, [](record const &Entry) {
		apply_weather(Entry);
		return true;
	});
	// the switches first: a vehicle is put on the path a switch is set to
	apply_each(Document, "switch", summary, apply_switch);
	apply_each(Document, "track", summary, apply_track);
	apply_each(Document, "crossing", summary, apply_crossing);
	apply_each(Document, "madetrack", summary, apply_made_track);
	apply_each(Document, "powersource", summary, apply_power);
	auto const left{apply_vehicles(Document, summary)};
	// the sections the vehicles were counted in and out of as they were put on their tracks
	apply_isolated(Document, summary);
	apply_drivers(Document, left, summary);
	apply_owners(Document, left);
	// the cells after the vehicles, whose sections write to their cells as they're taken and freed
	apply_each(Document, "memcell", summary, apply_memcell);
	auto const launchers{Events.launchers()};
	apply_each(Document, "launcher", summary, [&launchers](record const &Entry) { return apply_launcher(Entry, launchers); });
	apply_each(Document, "model", summary, apply_lights);

	Events.suspend(false);
	// the queue of the scenery just loaded gives way to the saved one
	if (false == Document.of("event").empty() || header->has("timer"))
	{
		Events.clear_queue();
		apply_each(Document, "event", summary, apply_event);
	}
	// last, whatever was drawn at random on the way here doesn't count
	apply_each(Document, "random", summary, apply_random);
	// the values of the file which weren't taken
	for (auto &refusal : take_refusals())
		summary.notes.emplace_back(std::move(refusal));
	return summary;
}

bool save(std::string const &File)
{
	auto const document{capture()};
	// to a file of its own first, which takes the place of the one asked for once it's whole
	if (auto const error{write_file(File, document)}; false == error.empty())
	{
		ErrorLog(std::format("snapshot: {}", error));
		return false;
	}
	WriteLog(std::format("snapshot: the simulation at {:.2f} s saved to \"{}\", {} entries", Timer::GetTime(), File, document.records.size()));
	return true;
}

std::string save_now()
{
	auto const &time{Time.data()};
	auto scenery{Global.SceneryFile};
	if (auto const extension{scenery.rfind('.')}; extension != std::string::npos)
		scenery.erase(extension);
	std::ranges::replace_if(scenery, [](char const Character) { return Character == '/' || Character == '\\' || Character == ':'; }, '_');
	// the folder eu07.ini names; the game folder may be one the game can't write to
	auto const folder{Global.snapshot_dir.empty() ? std::string{"snapshots"} : Global.snapshot_dir};
	std::error_code error;
	std::filesystem::create_directories(folder, error);
	// two saves in the same second of the simulation (while it's paused, say) don't write over each other
	auto const file{free_file(std::format("{}/{}_{:04}-{:02}-{:02}_{:02}-{:02}-{:02}.snapshot", folder, scenery, time.wYear, time.wMonth, time.wDay, time.wHour,
	                                      time.wMinute, time.wSecond))};
	auto const saved{save(file)};
	ui::Transcripts.AddLine(saved ? std::format("Snapshot saved: {}", file) : std::format("Snapshot not saved, {} can't be written (see errors.txt)", file), 0.f, 10.f,
	                        false);
	return saved ? file : std::string{};
}

void request(command_line const &Requests)
{
	auto &schedule{requested()};
	schedule.requests = Requests;
	// the earliest first
	std::ranges::stable_sort(schedule.requests.saves, {}, &command_line::save_request::time);
	schedule.written = 0;
	schedule.pending.reset();
	if (false == Requests.load.empty())
		schedule.pending = load(Requests.load);
}

// the snapshot to load, if it's to be used
document const *accepted_pending()
{
	auto const &schedule{requested()};
	return schedule.pending.has_value() && accepted(*schedule.pending, schedule.requests.force) ? &*schedule.pending : nullptr;
}

std::optional<std::uint32_t> requested_seed()
{
	auto const *pending{accepted_pending()};
	auto const *header{pending != nullptr ? pending->header() : nullptr};
	if (std::uint32_t seed{0}; header != nullptr && header->get("seed", seed))
		return seed;
	return std::nullopt;
}

std::string requested_player_vehicle()
{
	auto const *pending{accepted_pending()};
	if (pending == nullptr)
		return {};
	auto const players{pending->of("player")};
	return players.empty() ? std::string{} : players.front()->name;
}

void start()
{
	auto &schedule{requested()};
	auto loaded{false};
	if (schedule.pending.has_value())
	{
		auto const document{std::move(*schedule.pending)};
		schedule.pending.reset();
		for (auto const &problem : document.problems)
			ErrorLog(std::format("snapshot: \"{}\": {}", schedule.requests.load, problem));
		for (auto const &warning : document.warnings)
			ErrorLog(std::format("snapshot: \"{}\": {}", schedule.requests.load, warning));
		// a damaged file would put back part of the world, and leave the rest as the scenery makes it
		apply_summary summary;
		if (accepted(document, schedule.requests.force))
		{
			if (false == document.problems.empty())
				ErrorLog(std::format("snapshot: \"{}\" is loaded as far as it could be read, --load-snapshot-force says so", schedule.requests.load));
			summary = apply(document, schedule.requests.force);
		}
		else
		{
			summary.refused = true;
			summary.notes.emplace_back(std::format("\"{}\" isn't loaded: it's damaged (--load-snapshot-force loads what can be read of it)", schedule.requests.load));
		}
		for (auto const &note : summary.notes)
			ErrorLog(std::format("snapshot: {}", note));
		for (auto const &remark : summary.remarks)
			WriteLog(std::format("snapshot: {}", remark));
		loaded = (false == summary.refused);
		if (false == loaded && Global.test.enabled)
		{
			// a test run from a snapshot is no test without it
			test().finish(test_result::error, "the snapshot to load is damaged, missing or of another scenery");
		}
		if (loaded)
		{
			// the simulation goes on, it doesn't start: with nobody to wait for in a cab the drivers and the scenery
			// take the first frame as any other, the way they took it in the run the snapshot comes from
			if (Global.local_start_vehicle == "ghostview")
			{
				// the cells the scripts read the weather and the time from are made by the first, not yet ready, pass.
				// the run the snapshot comes from made them in its first frame; made here, the numbers their making
				// draws at random don't count
				auto const engine{Global.random_engine}; // NOSONAR: a copy of the simulation's reproducible engine, which nothing secure uses
				State.update_scripting_interface();
				Global.random_engine = engine;
				is_ready = true;
			}
			WriteLog(std::format("snapshot: \"{}\" loaded, the simulation goes on from {:.2f} s: {} entries put in place, {} left out", schedule.requests.load,
			                     Timer::GetTime(), summary.entries, summary.skipped));
		}
	}
	// a snapshot asked for at the very time the simulation starts from
	update();
	if (loaded && Timer::GetDeltaTime() == 0.0)
	{
		// the main loop draws a number after every frame of the simulation (eu07_application::generate_sync()), but
		// not after this start. the snapshot was taken at the end of a frame, before that draw: it's made up for here
		Random(1.0, 100.0);
	}
}

void update()
{
	auto &schedule{requested()};
	auto const &saves{schedule.requests.saves};
	// a frame is never short of the time asked for by the rounding of its steps alone
	constexpr double margin{1e-6};
	while (schedule.written < saves.size() && Timer::GetTime() >= saves[schedule.written].time - margin)
	{
		save(saves[schedule.written].file);
		++schedule.written;
	}
}

} // namespace simulation::snapshot

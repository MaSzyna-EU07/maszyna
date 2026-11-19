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
#include "model/AnimModel.h"
#include "simulation/simulation.h"
#include "simulation/simulationenvironment.h"
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
#include <format>
#include <fstream>
#include <map>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace simulation::snapshot {

namespace {

// the cells the simulation itself writes each frame: the weather, the time and the date for the scripts
constexpr std::string_view scripting_cells{"__simulation."};

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
	for (auto const *event : Events.sequence())
		names.add(event != nullptr ? event->m_name : std::string{});
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

void apply_time(record const &Entry)
{
	auto time{Time.data()};
	Entry.get("year", time.wYear);
	Entry.get("month", time.wMonth);
	Entry.get("day", time.wDay);
	Entry.get("weekday", time.wDayOfWeek);
	Entry.get("hour", time.wHour);
	Entry.get("minute", time.wMinute);
	Entry.get("second", time.wSecond);
	Time.restore(time, Entry.value("milliseconds", Time.milliseconds()));
	// the season goes by the date, and the weather by the season
	Environment.compute_season(Time.year_day());
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
	Entry.get("overcast", Global.Overcast);
	Entry.get("fog", Global.fFogEnd);
	Entry.get("temperature", Global.AirTemperature);
	Environment.compute_weather();
	auto state{Environment.wind_state()};
	if (auto const wind{text_numbers(Entry.text("wind"))}; wind.size() == state.size())
	{
		std::ranges::transform(wind, state.begin(), [](double const Value) { return static_cast<float>(Value); });
		Environment.wind_state(state);
	}
}

// the sources of the traction power: the voltage an event may have set, the load of the step before, the fuses
void capture_power(document &Document)
{
	// what a source which has fed nothing yet says of itself, but for its voltage
	std::vector<double> const idle{0.0, 1e-10, 1e-10, 0.0, 0.0, 0.0, 0.0, 0.0};
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
	source->state(text_numbers(Entry.text("state")));
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
	auto engine{Global.random_engine};
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
	auto const index{Entry.value("segment", -1)};
	if (index < 0 || index >= 6 || track->SwitchExtension->Segments[index] == nullptr)
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
	auto const end{Entry.value("end", 0)};
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
	if (auto state{0}; Entry.get("state", state) && state != track->GetSwitchState())
		track->Switch(state);
	if (auto velocity{0.0}; Entry.get("velocity", velocity))
		track->velocity_limit(velocity);
	return true;
}

bool apply_track(record const &Entry)
{
	auto *track{find_track(Entry.name)};
	if (track == nullptr)
		return false;
	if (auto velocity{0.0}; Entry.get("velocity", velocity))
		track->velocity_limit(velocity);
	Entry.get("damage", track->iDamageFlag);
	return true;
}

void capture_events(document &Document)
{
	auto const now{Timer::GetTime()};
	for (auto const *event{Events.begin()}; event != nullptr; event = event->m_next)
	{
		auto &entry{Document.add("event", event->m_name)};
		if (auto const sibling{event_sibling(event)}; sibling != 0)
			entry.set("sibling", sibling);
		// when it runs, by the clock of the simulation; the seconds left are for the reader of the file
		entry.set("at", event->m_launchtime).set("left", event->m_launchtime - now);
		if (event->m_activator != nullptr)
			entry.set("activator", vehicle_reference(event->m_activator));
	}
}

bool apply_event(record const &Entry)
{
	auto *event{find_event(Entry.name, Entry.value("sibling", 0))};
	if (event == nullptr)
		return false;
	// without the time it ran at, the seconds it had left count from now
	auto const launchtime{Entry.value("at", Timer::GetTime() + Entry.value("left", 0.0))};
	auto const *activator{Entry.has("activator") ? find_vehicle(Entry.text("activator")) : nullptr};
	return Events.restore_queued(event, activator, launchtime);
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
	launcher->timer(Entry.value("timer", launcher->timer()));
	return true;
}

// the models whose lights the events of the scenery switch: the signals, mostly
std::vector<TAnimModel *> switched_models()
{
	std::vector<TAnimModel *> models;
	std::unordered_set<scene::basic_node const *> seen;
	for (auto const *event : Events.sequence())
	{
		if (dynamic_cast<lights_event const *>(event) == nullptr)
			continue;
		for (auto *node : event->target_nodes())
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
		if (auto state{0.f}; Entry.get(std::format("light{}", light), state))
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
			auto const &coupler{vehicle->MoverParameters->Couplers[side]};
			auto const owner{coupler.Connected != nullptr ? Owners.find(coupler.Connected) : Owners.end()};
			if (owner == Owners.end() || coupler.CouplingFlag == coupling::faux)
				continue;
			links.emplace_back(vehicle_reference(vehicle), side, vehicle_reference(owner->second), coupler.ConnectedNr, coupler.CouplingFlag);
		}
	}
	return links;
}

// the couplings the snapshot has, for the vehicles it has
std::vector<coupling_link> wanted_couplings(std::vector<record const *> const &Entries)
{
	std::vector<coupling_link> links;
	for (auto const *entry : Entries)
	{
		for (auto const side : {end::front, end::rear})
		{
			auto const key{std::format("coupler{}", static_cast<int>(side))};
			if (false == entry->has(key + "flags"))
				continue;
			links.emplace_back(entry->name, side, entry->text(key), entry->value(key + "end", 0), entry->value(key + "flags", 0));
		}
	}
	return links;
}

void part(TDynamicObject &Vehicle, int const End)
{
	auto &coupler{Vehicle.MoverParameters->Couplers[End]};
	if (coupler.Connected != nullptr && coupler.ConnectedNr >= 0 && coupler.ConnectedNr < 2)
	{
		auto &other{coupler.Connected->Couplers[coupler.ConnectedNr]};
		other.Connected = nullptr;
		other.ConnectedNr = -1;
		other.CouplingFlag = coupling::faux;
	}
	coupler.Connected = nullptr;
	coupler.ConnectedNr = -1;
	coupler.CouplingFlag = coupling::faux;
}

// parts and joins the vehicles of the scenery until they're coupled the way the snapshot has them
void apply_couplings(std::vector<record const *> const &Entries, apply_summary &Summary)
{
	std::unordered_map<TMoverParameters const *, TDynamicObject *> owners;
	std::vector<std::string> present;
	for (auto *vehicle : Vehicles.sequence())
	{
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
			continue;
		owners.emplace(vehicle->MoverParameters, vehicle);
		present.emplace_back(vehicle_reference(vehicle));
	}
	auto const plan{plan_couplings(current_couplings(owners), wanted_couplings(Entries), present)};

	for (auto const &link : plan.part)
	{
		if (auto *vehicle{find_vehicle(link.vehicle)}; vehicle != nullptr)
			part(*vehicle, link.end);
	}
	for (auto const &link : plan.change)
	{
		auto const *vehicle{find_vehicle(link.vehicle)};
		auto const *other{find_vehicle(link.other)};
		if (vehicle == nullptr || other == nullptr)
			continue;
		vehicle->MoverParameters->Couplers[link.end].CouplingFlag = link.flags;
		other->MoverParameters->Couplers[link.otherend].CouplingFlag = link.flags;
	}
	auto failed{0};
	for (auto const &link : plan.join)
	{
		auto *vehicle{find_vehicle(link.vehicle)};
		auto *other{find_vehicle(link.other)};
		if (vehicle == nullptr || other == nullptr || false == vehicle->MoverParameters->Attach(link.end, link.otherend, other->MoverParameters, link.flags, true, false))
		{
			++failed;
			continue;
		}
		// the vehicles joined in the snapshot, whatever the scenery made of their couplers
		vehicle->MoverParameters->Couplers[link.end].CouplingFlag = link.flags;
		other->MoverParameters->Couplers[link.otherend].CouplingFlag = link.flags;
	}
	if (false == plan.part.empty() || false == plan.join.empty() || false == plan.change.empty())
		Summary.remarks.emplace_back(std::format("couplings: {} parted, {} joined, {} changed", plan.part.size(), static_cast<int>(plan.join.size()) - failed, plan.change.size()));
	if (failed != 0 || false == plan.missing.empty())
		Summary.notes.emplace_back(std::format("couplings: {} of the snapshot couldn't be made, {} are to vehicles the scenery doesn't have", failed, plan.missing.size()));

	// each vehicle knows the one coupled to it as its neighbour at that end
	for (auto *vehicle : Vehicles.sequence())
	{
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
			continue;
		for (auto const side : {end::front, end::rear})
		{
			auto const &coupler{vehicle->MoverParameters->Couplers[side]};
			auto &neighbour{vehicle->MoverParameters->Neighbours[side]};
			neighbour = neighbour_data();
			if (auto const owner{coupler.Connected != nullptr ? owners.find(coupler.Connected) : owners.end()}; owner != owners.end())
			{
				neighbour.vehicle = owner->second;
				neighbour.vehicle_end = coupler.ConnectedNr;
			}
		}
	}
	for (auto *vehicle : Vehicles.sequence())
	{
		if (vehicle != nullptr && vehicle->MoverParameters != nullptr && vehicle->bEnabled)
			vehicle->update_neighbours();
	}
}

void apply_vehicles(document const &Document, apply_summary &Summary)
{
	auto const entries{Document.of("vehicle")};
	std::vector<std::pair<TDynamicObject *, record const *>> known;
	std::unordered_set<TDynamicObject const *> listed;
	for (auto const *entry : entries)
	{
		auto *vehicle{find_vehicle(entry->name)};
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
		{
			++Summary.skipped;
			Summary.notes.emplace_back(std::format("vehicle {} of the snapshot isn't in the scenery", entry->name));
			continue;
		}
		known.emplace_back(vehicle, entry);
		listed.emplace(vehicle);
	}
	// where the vehicles are, then what they're coupled to, then what they're doing
	auto misplaced{0};
	for (auto const &[vehicle, entry] : known)
	{
		if (false == vehicle->snapshot_place(*entry))
			++misplaced;
	}
	if (misplaced != 0)
		Summary.notes.emplace_back(std::format("{} vehicles of the snapshot couldn't be put where it has them, they stay where the scenery put them", misplaced));
	apply_couplings(entries, Summary);
	for (auto const &[vehicle, entry] : known)
	{
		vehicle->snapshot_load(*entry);
		++Summary.entries;
	}
	// the vehicles the snapshot doesn't know stay as the scenery made them
	if (auto const unknown{std::ranges::count_if(Vehicles.sequence(), [&](TDynamicObject const *Vehicle) { return Vehicle != nullptr && false == listed.contains(Vehicle); })};
	    unknown != 0)
		Summary.notes.emplace_back(std::format("{} vehicles of the scenery aren't in the snapshot, they stay as the scenery made them", unknown));
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

void apply_drivers(document const &Document, apply_summary &Summary)
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
		if (vehicle == nullptr)
		{
			++Summary.skipped;
			++missing;
			continue;
		}
		if (vehicle->Mechanik == nullptr)
		{
			if (auto *driver{spare_driver(vehicle, taken)}; driver != nullptr)
				driver->MoveTo(vehicle);
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
	for (auto *vehicle : Vehicles.sequence())
	{
		if (vehicle == nullptr || vehicle->Mechanik == nullptr || taken.contains(vehicle) || false == known.contains(vehicle))
			continue;
		if (vehicle->Mechanik->AIControllFlag)
			SafeDelete(vehicle->Mechanik);
	}
}

// the driver in charge of each vehicle; after the drivers are where the snapshot has them
void apply_owners(document const &Document)
{
	for (auto const *entry : Document.of("vehicle"))
	{
		auto *vehicle{find_vehicle(entry->name)};
		if (vehicle == nullptr || false == entry->has("first"))
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

// tells whether the snapshot was taken in the scenery it's being put over
void check_scenery(record const &Header, apply_summary &Summary)
{
	if (auto const scenery{Header.text("scenery")}; scenery != Global.SceneryFile)
		Summary.notes.emplace_back(std::format("the snapshot was taken in scenery \"{}\", this one is \"{}\"", scenery, Global.SceneryFile));
	record now;
	capture_fingerprint(now);
	std::string changed;
	for (auto const &[key, value] : now.fields)
	{
		if (auto const saved{Header.text(key, value)}; saved != value)
			changed += std::format("{}{} {} (snapshot {})", changed.empty() ? "" : ", ", key, value, saved);
	}
	if (false == changed.empty())
	{
		Summary.notes.emplace_back(std::format("the scenery isn't what it was when the snapshot was taken: {}. things named by their place in the scenery "
		                                       "(\"#12\") may be other ones now",
		                                       changed));
	}
	if (auto const seed{Header.value("seed", Global.random_seed)}; seed != Global.random_seed)
	{
		Summary.notes.emplace_back(
		    std::format("the snapshot was taken with random seed {}, the scenery was loaded with {}: what it draws at random while it loads may differ", seed,
		                Global.random_seed));
	}
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
	auto *event{Events.FindEvent(Name)};
	for (auto index{0}; index < Sibling && event != nullptr; ++index)
		event = event->m_sibling;
	return event;
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
	capture_drivers(result);
	capture_player(result);
	// the long one last, out of the way of whoever reads the file
	capture_random(result);
	return result;
}

apply_summary apply(document const &Document)
{
	apply_summary summary;
	auto const *header{Document.header()};
	if (header == nullptr)
	{
		summary.refused = true;
		summary.notes.emplace_back("not a snapshot, nothing was loaded from it");
		return summary;
	}
	check_scenery(*header, summary);

	// nothing in the scenery reacts to the saved state being put back: the vehicles moved to their places don't
	// set off the events of the tracks, the sections they free and take don't change their signals
	Events.suspend(true);

	if (auto timer{0.0}; header->get("timer", timer))
		Timer::set_time(timer);
	apply_each(Document, "time", summary, [](record const &Entry) {
		apply_time(Entry);
		return true;
	});
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
	apply_vehicles(Document, summary);
	apply_drivers(Document, summary);
	apply_owners(Document);
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
	return summary;
}

bool save(std::string const &File)
{
	auto const document{capture()};
	std::ofstream output{File, std::ios::binary | std::ios::trunc};
	if (false == output.is_open())
	{
		ErrorLog(std::format("snapshot: file \"{}\" can't be written", File));
		return false;
	}
	write(output, document);
	output.close();
	if (output.fail())
	{
		ErrorLog(std::format("snapshot: writing file \"{}\" failed", File));
		return false;
	}
	WriteLog(std::format("snapshot: the simulation at {:.2f} s saved to \"{}\", {} entries", Timer::GetTime(), File, document.records.size()));
	return true;
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

std::optional<std::uint32_t> requested_seed()
{
	auto const &pending{requested().pending};
	auto const *header{pending.has_value() ? pending->header() : nullptr};
	if (std::uint32_t seed{0}; header != nullptr && header->get("seed", seed))
		return seed;
	return std::nullopt;
}

std::string requested_player_vehicle()
{
	auto const &pending{requested().pending};
	if (false == pending.has_value())
		return {};
	auto const players{pending->of("player")};
	return players.empty() ? std::string{} : players.front()->name;
}

void start()
{
	auto &schedule{requested()};
	auto const loaded{schedule.pending.has_value()};
	if (loaded)
	{
		auto const document{std::move(*schedule.pending)};
		schedule.pending.reset();
		for (auto const &problem : document.problems)
			ErrorLog(std::format("snapshot: \"{}\": {}", schedule.requests.load, problem));
		auto const summary{apply(document)};
		for (auto const &note : summary.notes)
			ErrorLog(std::format("snapshot: {}", note));
		for (auto const &remark : summary.remarks)
			WriteLog(std::format("snapshot: {}", remark));
		if (false == summary.refused)
		{
			// the simulation goes on, it doesn't start: with nobody to wait for in a cab the drivers and the scenery
			// take the first frame as any other, the way they took it in the run the snapshot comes from
			if (Global.local_start_vehicle == "ghostview")
			{
				// the cells the scripts read the weather and the time from are made by the first, not yet ready, pass.
				// the run the snapshot comes from made them in its first frame; made here, the numbers their making
				// draws at random don't count
				auto const engine{Global.random_engine};
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

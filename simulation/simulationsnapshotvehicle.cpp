/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/DynObj.h"

#include "McZapkie/MOVER.h"
#include "simulation/simulation.h"
#include "simulation/simulationsnapshotmover.h"
#include "simulation/simulationsnapshotworld.h"
#include "vehicle/Driver.h"
#include "world/Track.h"

#include <algorithm>
#include <array>
#include <format>

// the part of a simulation snapshot kept by a vehicle of the scenery: where its axles are on the tracks, which way it
// faces, what it's coupled to, where it's bound for, and through its physics (simulationsnapshotmover.cpp) how fast
// it goes, what it carries and what its machinery is doing

namespace {

namespace snapshot = simulation::snapshot;

// keys of the two axles and the two couplers: "track0", "coupler1flags"
std::string indexed(std::string_view const Part, int const Index, std::string_view const Key = {})
{
	return std::format("{}{}{}", Part, Index, Key);
}

void save_axle(snapshot::record &Entry, int const Index, TTrackFollower const &Axle)
{
	Entry.set(indexed("track", Index), snapshot::track_reference(Axle.GetTrack()));
	Entry.set(indexed("at", Index), Axle.GetTranslation());
	Entry.set(indexed("dir", Index), Axle.GetDirection());
	if (auto const path{Axle.Path()}; path != 0)
		Entry.set(indexed("path", Index), path);
	// which events of the track the axle has yet to set off: of the trains with a driver, and of any vehicle
	Entry.set(indexed("events", Index), Axle.EventFlags() | (Axle.EventallFlags() << 2));
}

// where an entry has an axle
struct axle_place {
	TTrack *track{nullptr};
	int path{0};
	double distance{0.0};
	double direction{1.0};
	int events{15};

	bool put(TTrackFollower &Axle, double const Inset) const
	{
		// Inset keeps the axle off the very ends of the track
		auto const length{track->Length()};
		auto const place{Inset > 0.0 && length > 2.0 * Inset ? std::clamp(distance, Inset, length - Inset) : distance};
		return Axle.Place(track, path, place, direction, events & 3, (events >> 2) & 3);
	}
};

// returns: the place, without its track if the entry doesn't give one which can be used
axle_place read_axle(snapshot::record const &Entry, int const Index)
{
	axle_place place;
	place.track = snapshot::find_track(Entry.text(indexed("track", Index)));
	// the path is a segment of a switch (0, 1) or of a crossing (1..6, the sign the way through it), the events two
	// pairs of flags. the axle is put at a distance along its track, which has to be a number
	if (auto const usable{(Entry.get(indexed("path", Index), place.path, -6, 6) || false == Entry.has(indexed("path", Index))) &&
	                      (Entry.get(indexed("at", Index), place.distance, -1e7, 1e7) || false == Entry.has(indexed("at", Index))) &&
	                      (Entry.get(indexed("dir", Index), place.direction, -1e7, 1e7) || false == Entry.has(indexed("dir", Index))) &&
	                      (Entry.get(indexed("events", Index), place.events, 0, 15) || false == Entry.has(indexed("events", Index)))};
	    false == usable)
		place.track = nullptr;
	return place;
}

// takes the vehicle off the list of the track it's on, if it's on it
void leave_track(TDynamicObject &Vehicle)
{
	if (Vehicle.MyTrack != nullptr && Vehicle.MyTrack->CheckDynamicObject(&Vehicle))
		Vehicle.MyTrack->RemoveDynamicObject(&Vehicle);
	Vehicle.MyTrack = nullptr;
}

// the vehicle whose physics a coupler is joined to: the neighbour at that end as a rule, otherwise looked up
TDynamicObject const *coupled_vehicle(TCoupling const &Coupler, TDynamicObject const *Neighbour)
{
	if (Coupler.Connected == nullptr || Coupler.CouplingFlag == coupling::faux)
		return nullptr;
	if (Neighbour != nullptr && Neighbour->MoverParameters == Coupler.Connected)
		return Neighbour;
	auto const &vehicles{simulation::Vehicles.sequence()};
	auto const lookup{std::ranges::find_if(vehicles, [&](TDynamicObject const *Vehicle) { return Vehicle != nullptr && Vehicle->MoverParameters == Coupler.Connected; })};
	return lookup != vehicles.end() ? *lookup : nullptr;
}

void save_coupler(snapshot::record &Entry, int const Index, TCoupling const &Coupler, TDynamicObject const *Neighbour)
{
	Neighbour = coupled_vehicle(Coupler, Neighbour);
	auto const coupled{Neighbour != nullptr};
	// an end with nothing coupled says so too: the snapshot knows the vehicle stands free there
	Entry.set(indexed("coupler", Index), coupled ? snapshot::vehicle_reference(Neighbour) : std::string{});
	Entry.set(indexed("coupler", Index, "end"), coupled ? Coupler.ConnectedNr : 0);
	Entry.set(indexed("coupler", Index, "flags"), coupled ? Coupler.CouplingFlag : 0);
}

// what the vehicle keeps of its machinery beside its physics: the pantographs as they stand, the passengers getting on and off
void save_machinery(snapshot::record &Entry, TDynamicObject const &Vehicle)
{
	Entry.set("overheadmask", Vehicle.iOverheadMask);
	Entry.set("novolttime", Vehicle.NoVoltTime);
	for (auto index{0}; index < Vehicle.iAnimType[ANIM_PANTS] && Vehicle.pants != nullptr; ++index)
	{
		auto const *pantograph{Vehicle.pants[index].fParamPants};
		if (pantograph == nullptr)
			continue;
		Entry.set(indexed("pant", index, "anglel"), pantograph->fAngleL);
		Entry.set(indexed("pant", index, "angleu"), pantograph->fAngleU);
		Entry.set(indexed("pant", index, "height"), pantograph->PantWys);
		Entry.set(indexed("pant", index, "wire"), pantograph->PantTraction);
		Entry.set(indexed("pant", index, "novolttime"), pantograph->NoVoltTime);
	}
	if (auto const &exchange{Vehicle.m_exchange}; exchange.unload_count != 0.f || exchange.load_count != 0.f)
	{
		Entry.set("exchangeunload", exchange.unload_count).set("exchangeload", exchange.load_count);
		Entry.set("exchangeplatforms", exchange.platforms).set("exchangetime", exchange.time);
	}
	// the rails under the wheel sets: the length of a rail, and for each wheel set how far along its rail it is and
	// whether that's by the joints of the track. a joint of a worn track jolts the vehicle
	Entry.set("raillength", Vehicle.dRailLength);
	if (false == Vehicle.m_axlesounds.empty())
	{
		std::vector<double> joints;
		for (auto const &axle : Vehicle.m_axlesounds)
		{
			joints.emplace_back(axle.distance);
			joints.emplace_back(axle.atjoints ? 1.0 : 0.0);
		}
		Entry.set("railjoints", snapshot::numbers_text(joints));
	}
	// the lighting of the compartments: which of them are lit is drawn at random when it comes on
	Entry.set("sectionlights", Vehicle.SectionLightsActive);
	if (false == Vehicle.Sections.empty())
	{
		std::vector<double> levels;
		for (auto const &section : Vehicle.Sections)
			levels.emplace_back(section.light_level);
		Entry.set("sectionlightlevels", snapshot::numbers_text(levels));
	}
	Entry.set("dimheadlights", Vehicle.DimHeadlights).set("highbeamlights", Vehicle.HighBeamLights);
	Entry.set("headlightsaoff", Vehicle.HeadlightsAoff).set("headlightsboff", Vehicle.HeadlightsBoff);
	// the force the electrodynamic brake of a unit was asked for in the step before, which the next one is held to
	if (Vehicle.MED_oldFED != 0.0)
		Entry.set("medoldfed", Vehicle.MED_oldFED);
}

void load_machinery(TDynamicObject &Vehicle, snapshot::record const &Entry)
{
	Entry.get("overheadmask", Vehicle.iOverheadMask);
	Entry.get("novolttime", Vehicle.NoVoltTime);
	for (auto index{0}; index < Vehicle.iAnimType[ANIM_PANTS] && Vehicle.pants != nullptr; ++index)
	{
		auto *pantograph{Vehicle.pants[index].fParamPants};
		if (pantograph == nullptr)
			continue;
		Entry.get(indexed("pant", index, "anglel"), pantograph->fAngleL);
		Entry.get(indexed("pant", index, "angleu"), pantograph->fAngleU);
		Entry.get(indexed("pant", index, "height"), pantograph->PantWys);
		Entry.get(indexed("pant", index, "wire"), pantograph->PantTraction);
		Entry.get(indexed("pant", index, "novolttime"), pantograph->NoVoltTime);
	}
	auto &exchange{Vehicle.m_exchange};
	exchange = {};
	// the counts of the passengers getting off and on are counted down step by step
	constexpr auto most_passengers{1e6f};
	Entry.get("exchangeunload", exchange.unload_count, -most_passengers, most_passengers);
	Entry.get("exchangeload", exchange.load_count, -most_passengers, most_passengers);
	Entry.get("exchangeplatforms", exchange.platforms);
	Entry.get("exchangetime", exchange.time);
	Entry.get("raillength", Vehicle.dRailLength, -1.0, 1e6);
	if (auto const joints{snapshot::text_numbers(Entry.text("railjoints"))}; joints.size() == 2 * Vehicle.m_axlesounds.size())
	{
		for (std::size_t index{0}; index < Vehicle.m_axlesounds.size(); ++index)
		{
			Vehicle.m_axlesounds[index].distance = joints[2 * index];
			Vehicle.m_axlesounds[index].atjoints = joints[2 * index + 1] != 0.0;
		}
	}
	Entry.get("sectionlights", Vehicle.SectionLightsActive);
	if (auto const levels{snapshot::text_numbers(Entry.text("sectionlightlevels"))}; levels.size() == Vehicle.Sections.size())
	{
		for (std::size_t index{0}; index < levels.size(); ++index)
			Vehicle.Sections[index].light_level = static_cast<float>(levels[index]);
	}
	Entry.get("dimheadlights", Vehicle.DimHeadlights);
	Entry.get("highbeamlights", Vehicle.HighBeamLights);
	Entry.get("headlightsaoff", Vehicle.HeadlightsAoff);
	Entry.get("headlightsboff", Vehicle.HeadlightsBoff);
	Vehicle.MED_oldFED = Entry.value("medoldfed", 0.0, -1e9, 1e9);
}

} // namespace

void TDynamicObject::snapshot_save(simulation::snapshot::record &Entry) const
{
	if (false == bEnabled)
	{
		// gone from the simulation, through a portal say
		Entry.set("removed", true);
		return;
	}
	// place
	save_axle(Entry, 0, Axle0);
	save_axle(Entry, 1, Axle1);
	Entry.set("first", iAxleFirst);
	Entry.set("facing", iDirection);
	// where that is in the world: the reader of the file sees it, and a loaded snapshot is checked against it
	Entry.set("x", vPosition.x).set("y", vPosition.y).set("z", vPosition.z);
	// consist
	for (auto const side : {end::front, end::rear})
		save_coupler(Entry, side, MoverParameters->Couplers[side], MoverParameters->Neighbours[side].vehicle);
	if (ctOwner != nullptr && ctOwner->Vehicle() != nullptr)
		Entry.set("owner", snapshot::vehicle_reference(ctOwner->Vehicle()));
	Entry.set("destination", asDestination);
	// the physics
	snapshot::capture_mover(Entry, *MoverParameters);
	save_machinery(Entry, *this);
}

bool TDynamicObject::snapshot_place(simulation::snapshot::record const &Entry)
{
	if (Entry.value("removed", false))
	{
		if (false == bEnabled)
			return true;
		// the vehicle left the simulation before the snapshot was taken: off the track, and off the list at the next update
		leave_track(*this);
		// the scenery counted its axles in the isolated sections of the tracks it put it on, the run the snapshot comes
		// from counted them out as the train drove off. the events are held while a snapshot is put back, and the saved
		// cells and queue carry what followed from it
		for (auto const *axle : {&Axle0, &Axle1})
		{
			if (auto *track{axle->GetTrack()}; track != nullptr)
				track->AxleCounter(-1, this);
		}
		// nobody's in charge of it: its driver, if it had one, doesn't go on (apply_drivers())
		ctOwner = nullptr;
		bEnabled = false;
		bDynamicRemove = true;
		return true;
	}
	auto const places{std::array{read_axle(Entry, 0), read_axle(Entry, 1)}};
	if (places[0].track == nullptr || places[1].track == nullptr)
		return false;

	// which of the ends of the vehicle leads in its consist, which of its axles is the first: 0 or 1
	Entry.get("facing", iDirection, 0, 1);
	// the axles a hair off the ends of their tracks first, for the body of the vehicle to be worked out from them:
	// Move() does that, and takes an axle standing at the very end of a track over to the next one
	if (constexpr double inset{1e-6}; false == places[0].put(Axle0, inset) || false == places[1].put(Axle1, inset))
		return false;
	// too short a way for the axles to move at all
	Move(1e-300);
	// then exactly where they were
	places[0].put(Axle0, 0.0);
	places[1].put(Axle1, 0.0);
	Entry.get("first", iAxleFirst, 0, 1);

	// the vehicle is on the list of the track its leading axle is on
	if (auto *track{iAxleFirst != 0 ? Axle1.GetTrack() : Axle0.GetTrack()}; track != MyTrack)
	{
		leave_track(*this);
		track->AddDynamicObject(this);
	}
	MoverParameters->Loc = {-vPosition.x, vPosition.z, vPosition.y};
	return true;
}

void TDynamicObject::snapshot_load(simulation::snapshot::record const &Entry)
{
	if (false == bEnabled)
		return;
	Entry.get("destination", asDestination);

	snapshot::apply_mover(*MoverParameters, Entry);
	// a load of another kind than the scenery gave comes with its model. it's loaded here and not by the first frame:
	// laying the load out in the vehicle draws at random, which doesn't count while a snapshot is put back
	if (MoverParameters->LoadTypeChange)
		LoadUpdate();
	update_load_visibility();

	load_machinery(*this, Entry);
}

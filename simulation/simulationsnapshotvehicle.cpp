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

axle_place read_axle(snapshot::record const &Entry, int const Index)
{
	axle_place place;
	place.track = snapshot::find_track(Entry.text(indexed("track", Index)));
	Entry.get(indexed("path", Index), place.path);
	Entry.get(indexed("at", Index), place.distance);
	Entry.get(indexed("dir", Index), place.direction);
	Entry.get(indexed("events", Index), place.events);
	return place;
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
}

bool TDynamicObject::snapshot_place(simulation::snapshot::record const &Entry)
{
	if (Entry.value("removed", false))
	{
		if (false == bEnabled)
			return true;
		// the vehicle left the simulation before the snapshot was taken: off the track, and off the list at the next update
		if (MyTrack != nullptr && MyTrack->RemoveDynamicObject(this))
			MyTrack = nullptr;
		bEnabled = false;
		bDynamicRemove = true;
		return true;
	}
	auto const places{std::array{read_axle(Entry, 0), read_axle(Entry, 1)}};
	if (places[0].track == nullptr || places[1].track == nullptr)
		return false;

	Entry.get("facing", iDirection);
	// the axles a hair off the ends of their tracks first, for the body of the vehicle to be worked out from them:
	// Move() does that, and takes an axle standing at the very end of a track over to the next one
	constexpr double inset{1e-6};
	if (false == places[0].put(Axle0, inset) || false == places[1].put(Axle1, inset))
		return false;
	// too short a way for the axles to move at all
	Move(1e-300);
	// then exactly where they were
	places[0].put(Axle0, 0.0);
	places[1].put(Axle1, 0.0);
	Entry.get("first", iAxleFirst);

	// the vehicle is on the list of the track its leading axle is on
	if (auto *track{iAxleFirst != 0 ? Axle1.GetTrack() : Axle0.GetTrack()}; track != MyTrack)
	{
		if (MyTrack != nullptr)
			MyTrack->RemoveDynamicObject(this);
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

}

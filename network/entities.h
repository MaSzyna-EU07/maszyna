/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>

class TDynamicObject;

namespace network
{

// identifies a participant of the session. the host is a participant like any other,
// so that peer/crew/permission handling does not need a special case for it
using PeerId = uint32_t;
constexpr PeerId PEER_NONE = 0;
constexpr PeerId PEER_HOST = 1;
// remote peers are numbered from here up
constexpr PeerId PEER_FIRST_REMOTE = 2;

// identifies a vehicle for the whole session. unlike TTrain::id() this does not depend
// on the order in which a particular peer happened to create its trains
using NetworkEntityId = uint32_t;
constexpr NetworkEntityId ENTITY_NONE = 0;

// how many people may share one vehicle. the crew itself is a plain container,
// so raising this number does not call for a structural change
constexpr uint8_t CREW_CAPACITY = 2;

// authoritative description of a single vehicle, as published by the server.
// every vehicle is listed, not only the ones worth driving: commands are addressed by
// this id, and a player may walk into any member of the train they are working
struct vehicle_entry
{
	NetworkEntityId id{ENTITY_NONE};
	std::string name;
	// the train this vehicle is currently coupled into, named by the lowest id in the set.
	// a player belongs to a train, not to a single car, so this is what the crew, the
	// permissions and the lobby all work with. it is recomputed as vehicles couple and
	// uncouple, which is what splits a crew between the halves of a train that came apart
	NetworkEntityId consist_id{ENTITY_NONE};
	// how many vehicles that train has, and whether this entry is the one that stands for
	// it in the lobby
	uint8_t consist_size{1};
	bool consist_lead{false};
	// crew of the whole train, not of this vehicle alone
	uint8_t crew_count{0};
	uint8_t crew_capacity{CREW_CAPACITY};
	bool ai_active{false};
	bool claimable{false};
	// has a driving position of its own
	bool drivable{false};

	bool operator==(vehicle_entry const &Other) const
	{
		return id == Other.id && name == Other.name && consist_id == Other.consist_id && consist_size == Other.consist_size && consist_lead == Other.consist_lead &&
		       crew_count == Other.crew_count && crew_capacity == Other.crew_capacity && ai_active == Other.ai_active && claimable == Other.claimable &&
		       drivable == Other.drivable;
	}
	bool operator!=(vehicle_entry const &Other) const { return !(*this == Other); }
};

// maps stable network ids onto the vehicles of the loaded scenario.
// the server owns the numbering and publishes it; a client binds the received ids to
// its own copy of the scenario by vehicle name, which keeps the runtime protocol on ids
class entity_registry
{
public:
	// server side: builds the numbering from the loaded scenario. vehicles are ordered by
	// name so that the result does not depend on the order the scenario happened to spawn them
	void build();
	// client side: takes over the numbering published by the server
	void adopt(std::vector<vehicle_entry> const &Entries);
	// server side: refreshes the volatile part of the published entries
	void refresh();
	void clear();

	bool empty() const { return m_entries.empty(); }
	std::vector<vehicle_entry> const &entries() const { return m_entries; }

	NetworkEntityId id_of(std::string const &Name) const;
	NetworkEntityId id_of(TDynamicObject const *Vehicle) const;
	// the train a vehicle belongs to, and everything coupled into it
	NetworkEntityId consist_of(NetworkEntityId Id) const;
	std::vector<NetworkEntityId> consist_members(NetworkEntityId Consist) const;
	std::string name_of(NetworkEntityId Id) const;
	TDynamicObject *resolve(NetworkEntityId Id) const;
	vehicle_entry const *find(NetworkEntityId Id) const;

private:
	void reindex();

	std::vector<vehicle_entry> m_entries;
	std::unordered_map<std::string, NetworkEntityId> m_byname;
	std::unordered_map<NetworkEntityId, size_t> m_byid;
};

// session wide vehicle registry. stays empty outside of multiplayer
extern entity_registry Entities;

// hands out ids for connecting peers; PEER_HOST is reserved for the local participant
PeerId allocate_peer_id();

// true when this run takes part in a session at all, as host or as client
bool is_multiplayer();
// true when this peer decides what happens in the world. a standalone game and the host
// of a session do; a client does not, it only carries out what the server hands it
bool is_authority();

// true for vehicles that carry a driving position of their own. note that a driving
// trailer has one without having an engine, and without the scenario putting a driver in
bool is_drivable(TDynamicObject const *Vehicle);

} // namespace network

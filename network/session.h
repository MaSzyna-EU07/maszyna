/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "network/entities.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace network
{

// who is working on a given vehicle. a vehicle can hold more than one person, which is
// the whole point of the crew model - the container is not limited by design, only by
// the capacity the server advertises
struct vehicle_crew
{
	NetworkEntityId entity_id{ENTITY_NONE};
	std::vector<PeerId> crew;
	// whether the AI was actually driving before the first human arrived. a vehicle that
	// nobody was driving must not suddenly acquire an AI driver when its crew leaves
	bool ai_before_claim{false};
	// set on the 1 -> 0 transition, cleared once the AI has been handed the vehicle back
	bool ai_restore_pending{false};
	// keeps reconcile() from re-posting the same request every single frame while the
	// previous one is still on its way through the command queue
	int action_cooldown{0};
};

enum class claim_result
{
	granted,
	already_member,
	unknown_vehicle,
	vehicle_missing,
	crew_full
};

std::string describe(claim_result Result);

// crew bookkeeping. authoritative on the server; on a client this is a read only mirror
// fed by CREW_UPDATE, used by the lobby
class crew_registry
{
public:
	claim_result claim(PeerId Peer, NetworkEntityId Id);
	// returns true when the peer really was part of the crew
	bool leave(PeerId Peer, NetworkEntityId Id);
	// removes a peer from every crew it belonged to, e.g. after a disconnect
	void drop_peer(PeerId Peer);

	std::vector<PeerId> crew_of(NetworkEntityId Id) const;
	bool is_member(PeerId Peer, NetworkEntityId Id) const;
	uint8_t count(NetworkEntityId Id) const;
	NetworkEntityId vehicle_of(PeerId Peer) const;

	// client side: takes over the crew of one vehicle as published by the server
	void mirror(NetworkEntityId Id, std::vector<PeerId> const &Crew);

	// server side: brings the world in line with the crew roster - creates the cab of a
	// claimed vehicle, hands the vehicle over from the AI and hands it back when the last
	// person leaves. safe to call every frame, it only acts when something is out of place
	void reconcile();

	// ids whose crew changed since the last call
	std::vector<NetworkEntityId> take_pending_updates();
	// queues every crew for republication, so that a peer which just went active learns
	// about the vehicles that were taken over before it arrived
	void mark_all_pending();

	void clear();

private:
	vehicle_crew &entry(NetworkEntityId Id);

	std::unordered_map<NetworkEntityId, vehicle_crew> m_vehicles;
	std::vector<NetworkEntityId> m_pending;
};

extern crew_registry Crews;

} // namespace network

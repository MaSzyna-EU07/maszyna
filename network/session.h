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
#include "input/command.h"

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
	// how many times we have asked the AI to hand this vehicle over. if it will not, that
	// is worth saying once rather than asking forever
	int ai_attempts{0};
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
	// removes a peer from every crew it belonged to
	void drop_peer(PeerId Peer);
	// the peer lost its connection. its seat is held for a while, so that a short dropout
	// does not empty a cab and hand a vehicle back to the AI under the other driver's nose
	void suspend_peer(PeerId Peer);
	// the peer made it back in time and keeps everything it had
	void resume_peer(PeerId Peer);
	// actually removes the peers whose grace period ran out
	void expire_absences();
	// true while the peer is gone but still holding its seat
	bool is_absent(PeerId Peer) const;

	std::vector<PeerId> crew_of(NetworkEntityId Id) const;
	bool is_member(PeerId Peer, NetworkEntityId Id) const;
	uint8_t count(NetworkEntityId Id) const;
	NetworkEntityId vehicle_of(PeerId Peer) const;

	// client side: takes over the crew of one vehicle as published by the server
	void mirror(NetworkEntityId Id, std::vector<PeerId> const &Crew);
	// puts a peer on a vehicle without asking about capacity; used when somebody walks to
	// another car of the train they are already working
	void take_seat(PeerId Peer, NetworkEntityId Id);

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
	// peers that dropped out, with the moment their seat stops being held
	std::unordered_map<PeerId, double> m_absent;
};

extern crew_registry Crews;

// how long a disconnected peer keeps its place in the crew
constexpr double RECONNECT_GRACE_SECONDS = 60.0;

// settings the server imposes on everybody, carried in the handshake. only things that
// change how the physics comes out belong here - looks and comfort stay with the player
enum session_config_flags : int64_t
{
	CONFIG_FULLPHYSICS = 1 << 0,
	CONFIG_REALISTICCONTROL = 1 << 1
};

// packs the settings this machine is running, for the server to hand out
int64_t pack_session_config();
// takes on the settings the server handed us
void apply_session_config(int64_t Config);
// settings a session needs regardless of who set what, applied on every peer
void enforce_session_settings();

// resolves the identity of a connecting peer. an unknown or empty token means somebody
// new and gets a fresh peer id together with a token of their own; a token this server
// handed out before brings back the peer id that went with it, and with it the seat in
// whatever crew the peer was on
PeerId resolve_peer_identity(uint64_t &Token);

// what a peer is allowed to ask the simulation for
enum class command_verdict
{
	accepted,
	not_in_crew,
	privileged,
	unsupported_target
};

std::string describe(command_verdict Verdict);

// the whole of the authority check for an incoming command. the host passes it too, it
// simply holds every right - there is no separate path for the local participant
command_verdict validate_command(PeerId Peer, user_command Command, uint32_t Recipient);

// a player works a train, not a single car: they may operate anything coupled into the
// set they are sitting in, which is what makes the cab switches of a multiple unit work
bool may_control(PeerId Peer, NetworkEntityId Entity);

// walking into a cab with the in-game key is the same request as pressing the button in
// the lobby, so it goes through the crew registry rather than being treated as a command
// that reaches the whole world. returns the vehicle it resolved to, if any
claim_result claim_by_name(PeerId Peer, std::string const &Vehicle, NetworkEntityId &Entity);

// drops everything the peer is not allowed to ask for and stamps the rest with its
// identity. used for the local participant as well, so that host input goes through the
// very same authority layer as input arriving over the wire, only without the transport
void filter_commands(PeerId Peer, command_queue::commands_map &Commands);

} // namespace network

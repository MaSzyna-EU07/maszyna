#pragma once
#include "network/message.h"
#include "network/entities.h"
#include "input/command.h"
#include <queue>

namespace network
{
struct message
{
	enum type_e
	{
		CLIENT_HELLO = 0,
		SERVER_HELLO,
		FRAME_INFO,
		REQUEST_COMMAND,
		SERVER_REJECT,
		VEHICLE_LIST,
		CLAIM_VEHICLE,
		CLAIM_GRANTED,
		CLAIM_DENIED,
		LEAVE_VEHICLE,
		CREW_UPDATE,
		CLIENT_READY,
		SNAPSHOT,
		REQUEST_RESYNC,
		TYPE_MAX
	};

	type_e type;

	message(type_e t) : type(t) {}
	virtual void serialize(std::ostream &stream) const {}
	virtual void deserialize(std::istream &stream) {}
};

struct client_hello : public message
{
	client_hello() : message(CLIENT_HELLO) {}

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;

	int32_t version;
	uint32_t start_packet;
	// build identification of the connecting simulator, informational only
	std::string app_version;
	// identity handed out by this server on an earlier connection, echoed back so that a
	// player who dropped out gets their seat and their peer id back. it is a reconnect
	// token, not a credential - it says "this is the same session", nothing more
	uint64_t session_token{ 0 };
};

// sent by the server instead of SERVER_HELLO when the client cannot join;
// carries a human readable reason so the player learns what went wrong
struct server_reject : public message
{
	server_reject() : message(SERVER_REJECT) {}

	std::string reason;

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

struct server_hello : public message
{
	server_hello() : message(SERVER_HELLO) {}

	uint32_t seed;
	int64_t timestamp;
    // settings the server imposes on the whole session, see network/session.h
    int64_t config;
    std::string scenario;
	// build identification of the server, informational only
	std::string app_version;
	// identity assigned to the joining peer for the rest of the session
	uint32_t peer_id{ PEER_NONE };
	// hand this back on a later connection to be recognised as the same participant
	uint64_t session_token{ 0 };

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

// authoritative roster of the vehicles a player may take over, broadcast by the server
// whenever it changes (and periodically, so that a peer which just went active gets one)
struct vehicle_list : public message
{
	vehicle_list() : message(VEHICLE_LIST) {}

	std::vector<vehicle_entry> vehicles;

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

// sent once the joining peer has the scenario loaded and can take a snapshot
struct client_ready : public message
{
	client_ready() : message(CLIENT_READY) {}

	std::string scenario;

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

// the state of the world as of one particular tick, handed to a peer that is joining.
// this is what replaces replaying the whole session from its first frame
struct snapshot : public message
{
	snapshot() : message(SNAPSHOT) {}

	uint64_t tick{ 0 };
	// 0 - the peer is joining and takes the world as given
	// 1 - routine correction on top of a simulation that is already running
	uint8_t mode{ 0 };
	std::string blob;

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

// a peer telling the server its world has drifted too far to carry on, and asking to be
// put back in line
struct request_resync : public message
{
	request_resync() : message(REQUEST_RESYNC) {}

	uint64_t tick{ 0 };
	uint64_t state_hash{ 0 };

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

// a peer asking the server to be put on the crew of a vehicle
struct claim_vehicle : public message
{
	claim_vehicle() : message(CLAIM_VEHICLE) {}

	uint32_t entity_id{ ENTITY_NONE };

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

// answer to a claim the server accepted
struct claim_granted : public message
{
	claim_granted() : message(CLAIM_GRANTED) {}

	uint32_t entity_id{ ENTITY_NONE };

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

// answer to a claim the server refused, with a reason the lobby can show
struct claim_denied : public message
{
	claim_denied() : message(CLAIM_DENIED) {}

	uint32_t entity_id{ ENTITY_NONE };
	std::string reason;

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

// a peer stepping off the crew of a vehicle
struct leave_vehicle : public message
{
	leave_vehicle() : message(LEAVE_VEHICLE) {}

	uint32_t entity_id{ ENTITY_NONE };

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

// authoritative crew of one vehicle, broadcast whenever it changes
struct crew_update : public message
{
	crew_update() : message(CREW_UPDATE) {}

	uint32_t entity_id{ ENTITY_NONE };
	std::vector<PeerId> crew;

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

struct request_command : public message
{
	request_command(type_e type) : message(type) {}
	request_command() : message(REQUEST_COMMAND) {}

	command_queue::commands_map commands;
	// logical step the sender was on when it produced these. the protocol is anchored to
	// this counter rather than to whatever frame the renderer happened to be drawing
	uint64_t tick{ 0 };

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

struct frame_info : public request_command
{
	frame_info() : request_command(FRAME_INFO) {}

	double render_dt;
	double dt;
	// digest of the authoritative world after this step; a client compares its own
	uint64_t state_hash{ 0 };
	uint32_t state_hash_version{ 0 };

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

std::shared_ptr<message> deserialize_message(std::istream &stream);
void serialize_message(const message &msg, std::ostream &stream);
} // namespace network

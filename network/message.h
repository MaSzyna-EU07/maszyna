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
    int64_t config;
    std::string scenario;
	// build identification of the server, informational only
	std::string app_version;
	// identity assigned to the joining peer for the rest of the session
	uint32_t peer_id{ PEER_NONE };

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

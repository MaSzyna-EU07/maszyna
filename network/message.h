#pragma once
#include "network/message.h"
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

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

struct request_command : public message
{
	request_command(type_e type) : message(type) {}
	request_command() : message(REQUEST_COMMAND) {}

	command_queue::commands_map commands;

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

struct frame_info : public request_command
{
	frame_info() : request_command(FRAME_INFO) {}

	double render_dt;
	double dt;
	double sync;

	virtual void serialize(std::ostream &stream) const override;
	virtual void deserialize(std::istream &stream) override;
};

std::shared_ptr<message> deserialize_message(std::istream &stream);
void serialize_message(const message &msg, std::ostream &stream);
} // namespace network

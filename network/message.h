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
		TYPE_MAX
	};

	type_e type;

	explicit message(type_e t) : type(t) {}
	virtual void serialize(std::ostream &stream) const { /* message carries no payload */ }
	virtual void deserialize(std::istream &stream) { /* message carries no payload */ }
	virtual ~message() = default;
};

struct client_hello : public message
{
	client_hello() : message(CLIENT_HELLO) {}

	void serialize(std::ostream &stream) const override;
	void deserialize(std::istream &stream) override;

	int32_t version{0};
	uint32_t start_packet{0};
};

struct server_hello : public message
{
	server_hello() : message(SERVER_HELLO) {}

	uint32_t seed{0};
	int64_t timestamp{0};
    int64_t config{0};
    std::string scenario;

	void serialize(std::ostream &stream) const override;
	void deserialize(std::istream &stream) override;
};

struct request_command : public message
{
	using message::message;
	request_command() : message(REQUEST_COMMAND) {}

	command_queue::commands_map commands;

	void serialize(std::ostream &stream) const override;
	void deserialize(std::istream &stream) override;
};

struct frame_info : public request_command
{
	frame_info() : request_command(FRAME_INFO) {}

	double render_dt{0.0};
	double dt{0.0};
	double sync{0.0};

	void serialize(std::ostream &stream) const override;
	void deserialize(std::istream &stream) override;
};

std::shared_ptr<message> deserialize_message(std::istream &stream);
void serialize_message(const message &msg, std::ostream &stream);
} // namespace network

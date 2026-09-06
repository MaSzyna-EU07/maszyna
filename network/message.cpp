#include "stdafx.h"
#include "network/message.h"
#include "scene/sn_utils.h"

void network::client_hello::serialize(std::ostream &stream) const
{
	sn_utils::ls_int32(stream, version);
	sn_utils::ls_uint32(stream, start_packet);
	sn_utils::s_str(stream, app_version);
	sn_utils::ls_uint64(stream, session_token);
}

void network::client_hello::deserialize(std::istream &stream)
{
	version = sn_utils::ld_int32(stream);
	start_packet = sn_utils::ld_uint32(stream);
	app_version = sn_utils::d_str(stream);
	session_token = sn_utils::ld_uint64(stream);
}

void network::server_reject::serialize(std::ostream &stream) const
{
	sn_utils::s_str(stream, reason);
}

void network::server_reject::deserialize(std::istream &stream)
{
	reason = sn_utils::d_str(stream);
}

void network::server_hello::serialize(std::ostream &stream) const
{
	sn_utils::ls_uint32(stream, seed);
	sn_utils::ls_int64(stream, timestamp);
    sn_utils::ls_int64(stream, config);
    sn_utils::s_str(stream, scenario);
	sn_utils::s_str(stream, app_version);
	sn_utils::ls_uint32(stream, peer_id);
	sn_utils::ls_uint64(stream, session_token);
}

void network::server_hello::deserialize(std::istream &stream)
{
	seed = sn_utils::ld_uint32(stream);
	timestamp = sn_utils::ld_int64(stream);
    config = sn_utils::ld_int64(stream);
    scenario = sn_utils::d_str(stream);
	app_version = sn_utils::d_str(stream);
	peer_id = sn_utils::ld_uint32(stream);
	session_token = sn_utils::ld_uint64(stream);
}

void network::vehicle_list::serialize(std::ostream &stream) const
{
	sn_utils::ls_uint32(stream, (uint32_t)vehicles.size());
	for (auto const &entry : vehicles)
	{
		sn_utils::ls_uint32(stream, entry.id);
		sn_utils::s_str(stream, entry.name);
		sn_utils::s_uint8(stream, entry.crew_count);
		sn_utils::s_uint8(stream, entry.crew_capacity);
		sn_utils::s_uint8(stream, (uint8_t)((entry.ai_active ? 1 : 0) | (entry.claimable ? 2 : 0)));
	}
}

void network::vehicle_list::deserialize(std::istream &stream)
{
	vehicles.clear();

	uint32_t const count = sn_utils::ld_uint32(stream);
	vehicles.reserve(count);

	for (uint32_t i = 0; i < count; i++)
	{
		vehicle_entry entry;
		entry.id = sn_utils::ld_uint32(stream);
		entry.name = sn_utils::d_str(stream);
		entry.crew_count = sn_utils::d_uint8(stream);
		entry.crew_capacity = sn_utils::d_uint8(stream);
		uint8_t const flags = sn_utils::d_uint8(stream);
		entry.ai_active = (flags & 1) != 0;
		entry.claimable = (flags & 2) != 0;
		vehicles.emplace_back(entry);
	}
}

void network::client_ready::serialize(std::ostream &stream) const
{
	sn_utils::s_str(stream, scenario);
}

void network::client_ready::deserialize(std::istream &stream)
{
	scenario = sn_utils::d_str(stream);
}

void network::snapshot::serialize(std::ostream &stream) const
{
	sn_utils::ls_uint64(stream, tick);
	sn_utils::s_uint8(stream, mode);
	sn_utils::ls_uint32(stream, (uint32_t)blob.size());
	stream.write(blob.data(), blob.size());
}

void network::snapshot::deserialize(std::istream &stream)
{
	tick = sn_utils::ld_uint64(stream);
	mode = sn_utils::d_uint8(stream);

	uint32_t const size = sn_utils::ld_uint32(stream);
	blob.assign(size, '\0');
	stream.read(blob.data(), size);
}

void network::request_resync::serialize(std::ostream &stream) const
{
	sn_utils::ls_uint64(stream, tick);
	sn_utils::ls_uint64(stream, state_hash);
}

void network::request_resync::deserialize(std::istream &stream)
{
	tick = sn_utils::ld_uint64(stream);
	state_hash = sn_utils::ld_uint64(stream);
}

void network::claim_vehicle::serialize(std::ostream &stream) const
{
	sn_utils::ls_uint32(stream, entity_id);
}

void network::claim_vehicle::deserialize(std::istream &stream)
{
	entity_id = sn_utils::ld_uint32(stream);
}

void network::claim_granted::serialize(std::ostream &stream) const
{
	sn_utils::ls_uint32(stream, entity_id);
}

void network::claim_granted::deserialize(std::istream &stream)
{
	entity_id = sn_utils::ld_uint32(stream);
}

void network::claim_denied::serialize(std::ostream &stream) const
{
	sn_utils::ls_uint32(stream, entity_id);
	sn_utils::s_str(stream, reason);
}

void network::claim_denied::deserialize(std::istream &stream)
{
	entity_id = sn_utils::ld_uint32(stream);
	reason = sn_utils::d_str(stream);
}

void network::leave_vehicle::serialize(std::ostream &stream) const
{
	sn_utils::ls_uint32(stream, entity_id);
}

void network::leave_vehicle::deserialize(std::istream &stream)
{
	entity_id = sn_utils::ld_uint32(stream);
}

void network::crew_update::serialize(std::ostream &stream) const
{
	sn_utils::ls_uint32(stream, entity_id);
	sn_utils::ls_uint32(stream, (uint32_t)crew.size());
	for (PeerId const peer : crew)
		sn_utils::ls_uint32(stream, peer);
}

void network::crew_update::deserialize(std::istream &stream)
{
	entity_id = sn_utils::ld_uint32(stream);

	crew.clear();
	uint32_t const count = sn_utils::ld_uint32(stream);
	crew.reserve(count);
	for (uint32_t i = 0; i < count; i++)
		crew.emplace_back(sn_utils::ld_uint32(stream));
}

void ::network::request_command::serialize(std::ostream &stream) const
{
	sn_utils::ls_uint64(stream, tick);
	sn_utils::ls_uint32(stream, commands.size());
	for (auto const &kv : commands)
	{
		sn_utils::ls_uint32(stream, kv.first);
		sn_utils::ls_uint32(stream, kv.second.size());
		for (command_data const &data : kv.second)
		{
			sn_utils::ls_uint32(stream, (uint32_t)data.command);
			sn_utils::ls_int32(stream, data.action);
			sn_utils::ls_float64(stream, data.param1);
			sn_utils::ls_float64(stream, data.param2);
			sn_utils::ls_float64(stream, data.time_delta);

			sn_utils::s_bool(stream, data.freefly);
			sn_utils::s_vec3(stream, data.location);

			sn_utils::s_str(stream, data.payload);
			sn_utils::ls_uint32(stream, data.source);
		}
	}
}

void network::request_command::deserialize(std::istream &stream)
{
	tick = sn_utils::ld_uint64(stream);

	uint32_t commands_size = sn_utils::ld_uint32(stream);
	for (uint32_t i = 0; i < commands_size; i++)
	{
		uint32_t recipient = sn_utils::ld_uint32(stream);
		uint32_t sequence_size = sn_utils::ld_uint32(stream);

		command_queue::commanddata_sequence sequence;
		for (uint32_t i = 0; i < sequence_size; i++)
		{
			command_data data;
			data.command = (user_command)sn_utils::ld_uint32(stream);
			data.action = sn_utils::ld_int32(stream);
			data.param1 = sn_utils::ld_float64(stream);
			data.param2 = sn_utils::ld_float64(stream);
			data.time_delta = sn_utils::ld_float64(stream);

			data.freefly = sn_utils::d_bool(stream);
			data.location = sn_utils::d_vec3(stream);

			data.payload = sn_utils::d_str(stream);
			data.source = sn_utils::ld_uint32(stream);

			sequence.emplace_back(data);
		}

		commands.emplace(recipient, sequence);
	}
}

void network::frame_info::serialize(std::ostream &stream) const
{
	sn_utils::ls_float64(stream, render_dt);
	sn_utils::ls_float64(stream, dt);
	sn_utils::ls_uint64(stream, state_hash);
	sn_utils::ls_uint32(stream, state_hash_version);

	request_command::serialize(stream);
}

void network::frame_info::deserialize(std::istream &stream)
{
	render_dt = sn_utils::ld_float64(stream);
	dt = sn_utils::ld_float64(stream);
	state_hash = sn_utils::ld_uint64(stream);
	state_hash_version = sn_utils::ld_uint32(stream);

	request_command::deserialize(stream);
}

std::shared_ptr<network::message> network::deserialize_message(std::istream &stream)
{
	message::type_e type = (message::type_e)sn_utils::ld_uint16(stream);

	std::shared_ptr<message> msg;

	if (type == message::CLIENT_HELLO)
		msg = std::make_shared<client_hello>();
	else if (type == message::SERVER_HELLO)
		msg = std::make_shared<server_hello>();
	else if (type == message::FRAME_INFO)
		msg = std::make_shared<frame_info>();
	else if (type == message::REQUEST_COMMAND)
		msg = std::make_shared<request_command>();
	else if (type == message::SERVER_REJECT)
		msg = std::make_shared<server_reject>();
	else if (type == message::VEHICLE_LIST)
		msg = std::make_shared<vehicle_list>();
	else if (type == message::CLAIM_VEHICLE)
		msg = std::make_shared<claim_vehicle>();
	else if (type == message::CLAIM_GRANTED)
		msg = std::make_shared<claim_granted>();
	else if (type == message::CLAIM_DENIED)
		msg = std::make_shared<claim_denied>();
	else if (type == message::LEAVE_VEHICLE)
		msg = std::make_shared<leave_vehicle>();
	else if (type == message::CREW_UPDATE)
		msg = std::make_shared<crew_update>();
	else if (type == message::CLIENT_READY)
		msg = std::make_shared<client_ready>();
	else if (type == message::SNAPSHOT)
		msg = std::make_shared<snapshot>();
	else if (type == message::REQUEST_RESYNC)
		msg = std::make_shared<request_resync>();

	if (!msg) {
		// unknown message type; hand back a marker the peer handlers treat as a protocol error
		return std::make_shared<message>(message::TYPE_MAX);
	}

	msg->deserialize(stream);

	return msg;
}

void network::serialize_message(const message &msg, std::ostream &stream)
{
	sn_utils::ls_uint16(stream, (uint16_t)msg.type);
	msg.serialize(stream);
}

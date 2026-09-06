#include "stdafx.h"
#include "network/network.h"
#include "network/snapshot.h"
#include "network/message.h"
#include "utilities/Logs.h"
#include "scene/sn_utils.h"
#include "utilities/Timer.h"
#include "application/application.h"
#include "utilities/Globals.h"
#include "simulation/simulation.h"
#include <set>
#include <tuple>

// 2 - legacy lockstep protocol
// 3 - handshake carries build identification and an explicit rejection message
// 4 - peer identity, vehicle roster and crew handshake, command source,
//     logical simulation tick and a versioned state digest instead of the position sum
std::uint32_t const EU07_NETWORK_VERSION = 4;

namespace network {

backend_list_t& backend_list() {
    // HACK: static initialization order fiasco fix
    // NOTE: potential static deinitialization order fiasco
    static backend_list_t backend_list;
    return backend_list;
}

}
// connection

void network::connection::disconnect() {
	WriteLog("net: peer dropped", logtype::net);
	state = DEAD;
}

void network::connection::set_handler(std::function<void (const message &)> handler) {
	message_handler = handler;
}

network::connection::connection(bool client, size_t counter) {
	packet_counter = counter;
	is_client = client;
	state = AWAITING_HELLO;
}

void network::connection::connected()
{
	WriteLog("net: socket connected", logtype::net);

	if (is_client) {
		client_hello msg;
		msg.version = EU07_NETWORK_VERSION;
		msg.start_packet = packet_counter;
		msg.app_version = Global.asVersion;
		msg.session_token = Global.network_session_token;
		send_message(msg);
	}
}

void network::connection::send_complete(std::shared_ptr<std::string> buf)
{
	// the shared buffer had to stay alive until asio finished writing it out; now it can go
}

// --------------

// server
network::server::server(std::shared_ptr<std::istream> buf) : backbuffer(buf)
{

}

namespace {

// keeps the log readable: a peer holding a forbidden key would otherwise produce a line
// every single frame, so each peer/command/verdict combination is reported once
void report_rejected_command(network::PeerId Peer, user_command Command, network::command_verdict Verdict)
{
	static std::set<std::tuple<network::PeerId, user_command, network::command_verdict>> reported;

	if (!reported.emplace(Peer, Command, Verdict).second)
		return;

	auto const &description = simulation::Commands_descriptions[static_cast<std::size_t>(Command)];
	ErrorLog("net: command \"" + description.name + "\" from peer " + std::to_string(Peer) + " rejected: " + network::describe(Verdict), logtype::net);
}

} // namespace

void network::server::prune_clients()
{
	for (auto it = clients.begin(); it != clients.end(); ) {
		if ((*it)->state == connection::DEAD) {
			if ((*it)->peer_id != PEER_NONE) {
				WriteLog("net: peer " + std::to_string((*it)->peer_id) + " disconnected", logtype::net);
				// the seat is held for a while in case the peer comes straight back. a
				// vehicle keeps running for whoever is left on board either way; it only
				// falls back to the AI once the crew really is empty
				Crews.suspend_peer((*it)->peer_id);
			}
			it = clients.erase(it);
			continue;
		}
		it++;
	}
}

void network::server::push_delta(const frame_info &msg)
{
	prune_clients();

	for (auto const &client : clients) {
		if (client->state == connection::ACTIVE)
			client->send_message(msg);
	}
}

void network::server::push_message(const message &msg)
{
	prune_clients();

	for (auto const &client : clients) {
		if (client->state == connection::ACTIVE)
			client->send_message(msg);
	}
}

command_queue::commands_map network::server::pop_commands()
{
	command_queue::commands_map map(client_commands_queue);
	client_commands_queue.clear();
	return map;
}

void network::server::handle_message(std::shared_ptr<connection> conn, const message &msg)
{
	if (msg.type == message::TYPE_MAX)
	{
		conn->disconnect();
		return;
	}

	if (msg.type == message::CLIENT_HELLO) {
		const auto& cmd = dynamic_cast<const client_hello&>(msg);

		std::string rejection;
		if (cmd.version != (int32_t)EU07_NETWORK_VERSION) {
			rejection = "incompatible protocol version: server speaks "
			        + std::to_string(EU07_NETWORK_VERSION)
			        + ", client speaks " + std::to_string(cmd.version);
		}
		else if (!Global.ready_to_load || Global.SceneryFile.empty()) {
			// the host has not picked a scenario yet, there is nothing to join
			rejection = "the server has no scenario running yet";
		}

		if (!rejection.empty()) {
			WriteLog("net: rejecting peer: " + rejection, logtype::net);

			server_reject reply;
			reply.reason = rejection;
			conn->send_message(reply);
			conn->disconnect();
			return;
		}

		server_hello reply;
		reply.seed = Global.random_seed;
		reply.timestamp = Global.starting_timestamp;
        reply.config = pack_session_config();
        reply.scenario = Global.SceneryFile;
		reply.app_version = Global.asVersion;
		reply.session_token = cmd.session_token;
		reply.peer_id = resolve_peer_identity(reply.session_token);
		conn->peer_id = reply.peer_id;
		// the peer now goes and loads the scenario. it gets the state of the world as a
		// snapshot when it reports back, instead of replaying the session frame by frame
		conn->state = connection::AWAITING_READY;
		conn->packet_counter = cmd.start_packet;

		conn->send_message(reply);

		WriteLog("net: peer " + std::to_string(conn->peer_id) + " connected, build \"" + cmd.app_version
		         + "\", scenario \"" + Global.SceneryFile + "\"", logtype::net);
	}
	else if (msg.type == message::CLIENT_READY) {
		const auto& cmd = dynamic_cast<const client_ready&>(msg);

		if (cmd.scenario != Global.SceneryFile) {
			WriteLog("net: peer " + std::to_string(conn->peer_id) + " loaded \"" + cmd.scenario
			         + "\" but the session runs \"" + Global.SceneryFile + "\"", logtype::net);

			server_reject reply;
			reply.reason = "scenario mismatch: the session runs \"" + Global.SceneryFile + "\"";
			conn->send_message(reply);
			conn->disconnect();
			return;
		}

		// the roster first, so that the crew section of the snapshot resolves to names
		vehicle_list roster;
		roster.vehicles = Entities.entries();
		conn->send_message(roster);

		snapshot reply;
		reply.tick = Global.simulation_tick;
		reply.blob = take_snapshot();
		conn->send_message(reply);

		// from here on the peer receives the live stream; there is no backlog to catch up on
		conn->state = connection::ACTIVE;

		WriteLog("net: snapshot sent to peer " + std::to_string(conn->peer_id) + " at tick "
		         + std::to_string(reply.tick), logtype::net);

		// and it still has to learn who is where
		Crews.mark_all_pending();
	}
	else if (msg.type == message::REQUEST_RESYNC) {
		const auto& cmd = dynamic_cast<const request_resync&>(msg);

		WriteLog("net: resync requested by peer " + std::to_string(conn->peer_id) + " at tick "
		         + std::to_string(cmd.tick) + " (its digest " + std::to_string(cmd.state_hash) + ")", logtype::net);

		snapshot reply;
		reply.tick = Global.simulation_tick;
		reply.blob = take_snapshot();
		conn->send_message(reply);
	}
	else if (msg.type == message::CLAIM_VEHICLE) {
		const auto& cmd = dynamic_cast<const claim_vehicle&>(msg);

		auto const result = Crews.claim(conn->peer_id, cmd.entity_id);
		if (result == claim_result::granted || result == claim_result::already_member) {
			claim_granted reply;
			reply.entity_id = cmd.entity_id;
			conn->send_message(reply);
		}
		else {
			WriteLog("net: refused claim of vehicle " + std::to_string(cmd.entity_id) + " by peer "
			         + std::to_string(conn->peer_id) + ": " + describe(result), logtype::net);

			claim_denied reply;
			reply.entity_id = cmd.entity_id;
			reply.reason = describe(result);
			conn->send_message(reply);
		}
	}
	else if (msg.type == message::LEAVE_VEHICLE) {
		const auto& cmd = dynamic_cast<const leave_vehicle&>(msg);
		Crews.leave(conn->peer_id, cmd.entity_id);
	}
	else if (msg.type == message::REQUEST_COMMAND) {
		const auto& cmd = dynamic_cast<const request_command&>(msg);

		for (auto const &kv : cmd.commands) {
			for (command_data const &data : kv.second) {
				auto const verdict = validate_command(conn->peer_id, data.command, kv.first);
				if (verdict != command_verdict::accepted) {
					report_rejected_command(conn->peer_id, data.command, verdict);
					continue;
				}

				command_data accepted = data;
				// the source is decided here, never taken from what the peer sent
				accepted.source = conn->peer_id;

				auto lookup = client_commands_queue.emplace(kv.first, command_queue::commanddata_sequence());
				lookup.first->second.emplace_back(accepted);
			}
		}
	}
}

// ------------

void network::client::update()
{
	if (conn && conn->state == connection::DEAD) {
		conn.reset();
	}

	if (!Global.network_reject_reason.empty()) {
		// the server explicitly refused us; retrying would only spam it
		return;
	}

	if (!conn) {
		if (!reconnect_delay) {
			connect();
			reconnect_delay = RECONNECT_DELAY_FRAMES;
		}
		reconnect_delay--;
	}
}

// client
network::frame_delta network::client::get_next_delta(int counter)
{
	auto now = std::chrono::high_resolution_clock::now();
	if (counter == 1) {
		frame_time = now - last_frame;
		last_frame = now;
	}

	if (delta_queue.empty()) {
		// buffer underflow
		return frame_delta();
	}


	float size = delta_queue.size() - consume_counter;
	const auto& entry = delta_queue.front();
	float mult = entry.render_dt / std::chrono::duration_cast<std::chrono::duration<float>>(frame_time).count();

	if (counter == 1 && size < MAX_BUFFER_SIZE * 2.0f) {
		last_target = last_target * TARGET_MIX +
		        (std::min(TARGET_MIN + jitteriness * JITTERINESS_MULTIPIER, MAX_BUFFER_SIZE)) * (1.0f - TARGET_MIX);
		float diff = size - last_target;
		jitteriness = std::max(jitteriness * JITTERINESS_MIX, std::abs(diff));

		float speed = 1.0f + diff * CONSUME_MULTIPIER;

		consume_counter += speed;
	}

	float last_rcv_diff = std::chrono::duration_cast<std::chrono::duration<float>>(now - last_rcv).count();

	if (size > MAX_BUFFER_SIZE || consume_counter > mult || last_rcv_diff > 1.0f) {
		if (consume_counter > mult) {
			consume_counter = std::clamp(consume_counter - mult, -MAX_BUFFER_SIZE, MAX_BUFFER_SIZE);
		}

		frame_delta delta;
		delta.dt = entry.dt;
		delta.tick = entry.tick;
		delta.state_hash = entry.state_hash;
		delta.state_hash_version = entry.state_hash_version;
		delta.commands = entry.commands;
		delta.valid = true;

		delta_queue.pop();

		return delta;
	} else {
		// nothing to push
		return frame_delta();
	}
}

void network::client::send_commands(command_queue::commands_map commands)
{
	if (!conn || conn->state == connection::DEAD || commands.empty())
		return;
	// eh, maybe queue lost messages

	request_command msg;
	msg.tick = Global.simulation_tick;
	msg.commands = commands;

	conn->send_message(msg);
}

void network::client::send_ready()
{
	if (!conn || conn->state == connection::DEAD)
		return;

	client_ready msg;
	msg.scenario = Global.SceneryFile;
	conn->send_message(msg);

	WriteLog("net: scenario loaded, asking the server for a snapshot", logtype::net);
}

void network::client::send_resync_request(uint64_t tick, uint64_t state_hash)
{
	if (!conn || conn->state != connection::ACTIVE)
		return;

	request_resync msg;
	msg.tick = tick;
	msg.state_hash = state_hash;
	conn->send_message(msg);
}

void network::client::send_claim(NetworkEntityId entity_id)
{
	if (!conn || conn->state != connection::ACTIVE)
		return;

	claim_vehicle msg;
	msg.entity_id = entity_id;
	conn->send_message(msg);
}

void network::client::send_leave(NetworkEntityId entity_id)
{
	if (!conn || conn->state != connection::ACTIVE)
		return;

	leave_vehicle msg;
	msg.entity_id = entity_id;
	conn->send_message(msg);
}

void network::client::handle_message(std::shared_ptr<connection> conn, const message &msg)
{
	if (msg.type >= message::TYPE_MAX)
	{
		conn->disconnect();
		return;
	}

	if (msg.type == message::SERVER_REJECT) {
		const auto& cmd = dynamic_cast<const server_reject&>(msg);

		ErrorLog("net: connection refused by the server: " + cmd.reason, logtype::net);
		Global.network_reject_reason = cmd.reason;
		Global.network_status = "Connection refused: " + cmd.reason;
		conn->disconnect();
		return;
	}

	if (msg.type == message::SERVER_HELLO) {
		const auto& cmd = dynamic_cast<const server_hello&>(msg);
		conn->state = connection::ACTIVE;

		bool const firsthandshake { !Global.ready_to_load };

		if (!Global.ready_to_load) {
			Global.random_seed = cmd.seed;
			Global.random_engine.seed(Global.random_seed);
			Global.starting_timestamp = cmd.timestamp;
            apply_session_config(cmd.config);
            Global.SceneryFile = cmd.scenario;
			Global.ready_to_load = true;

			WriteLog("net: scenario handshake: \"" + cmd.scenario + "\", server build \"" + cmd.app_version + "\"", logtype::net);
		} else if (Global.random_seed != cmd.seed) {
			ErrorLog("net: seed mismatch", logtype::net);
			conn->disconnect();
			return;
		}

		if (cmd.peer_id != PEER_NONE) {
			Global.network_peer_id = cmd.peer_id;
			WriteLog("net: assigned peer id " + std::to_string(cmd.peer_id), logtype::net);
		}

		if (cmd.session_token != 0) {
			// kept for the next connection, so that a dropout does not cost us our seat
			Global.network_session_token = cmd.session_token;
		}

		Global.network_status.clear();

		WriteLog("net: accept received", logtype::net);

		if (!firsthandshake && simulation::is_ready) {
			// we are coming back to a session we already have loaded, so there is nobody
			// to wait for: ask for a fresh snapshot straight away
			send_ready();
		}
	}

	if (conn->state != connection::ACTIVE)
		return;

	if (msg.type == message::SNAPSHOT) {
		const auto& cmd = dynamic_cast<const snapshot&>(msg);

		if (apply_snapshot(cmd.blob)) {
			Global.network_snapshot_applied = true;

			// the roster may say we are on a crew already - after a reconnect, or because
			// the claim was granted while we were still loading. walk into that cab
			auto const own = Crews.vehicle_of(Global.network_peer_id);
			if (own != ENTITY_NONE) {
				auto const name = Entities.name_of(own);
				if (!name.empty())
					Global.network_pending_vehicle = name;
			}
		}
		else {
			ErrorLog("net: could not apply the snapshot handed to us", logtype::net);
			Global.network_status = "The server sent a snapshot this build cannot read";
		}
	}

	if (msg.type == message::VEHICLE_LIST) {
		const auto& cmd = dynamic_cast<const vehicle_list&>(msg);
		Entities.adopt(cmd.vehicles);
	}

	if (msg.type == message::CREW_UPDATE) {
		const auto& cmd = dynamic_cast<const crew_update&>(msg);
		Crews.mirror(cmd.entity_id, cmd.crew);
	}

	if (msg.type == message::CLAIM_GRANTED) {
		const auto& cmd = dynamic_cast<const claim_granted&>(msg);

		WriteLog("net: claim of vehicle " + std::to_string(cmd.entity_id) + " granted", logtype::net);
		Global.network_lobby_message.clear();
		// the cab itself is built by the replicated entervehicle the server posts;
		// we only have to walk into it once it shows up
		Global.network_pending_vehicle = Entities.name_of(cmd.entity_id);
	}

	if (msg.type == message::CLAIM_DENIED) {
		const auto& cmd = dynamic_cast<const claim_denied&>(msg);

		WriteLog("net: claim of vehicle " + std::to_string(cmd.entity_id) + " denied: " + cmd.reason, logtype::net);
		Global.network_lobby_message = cmd.reason;
	}

	if (msg.type == message::FRAME_INFO) {
		resume_frame_counter++;

		auto delta = dynamic_cast<const frame_info&>(msg);
		delta_queue.push(delta);
		last_rcv = std::chrono::high_resolution_clock::now();
	}
}

// --------------

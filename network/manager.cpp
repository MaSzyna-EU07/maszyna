/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "network/manager.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/Globals.h"
#include "network/statehash.h"

network::server_manager::server_manager()
{
	backbuffer = std::make_shared<std::fstream>("backbuffer.bin", std::ios::out | std::ios::in | std::ios::trunc | std::ios::binary);
}

command_queue::commands_map network::server_manager::pop_commands()
{
	command_queue::commands_map map;

	for (auto srv : servers)
		add_to_dequemap(map, srv->pop_commands());

	return map;
}

void network::server_manager::push_delta(double render_dt, double dt, uint64_t tick, uint64_t state_hash, const command_queue::commands_map &commands)
{
	if (dt == 0.0 && commands.empty())
		return;

	frame_info msg;
	msg.render_dt = render_dt;
	msg.dt = dt;
	msg.tick = tick;
	msg.state_hash = state_hash;
	msg.state_hash_version = STATE_HASH_VERSION;
	msg.commands = commands;

	for (auto srv : servers)
		srv->push_delta(msg);

	serialize_message(msg, *backbuffer.get());
}

void network::server_manager::create_server(const std::string &backend, const std::string &conf)
{
	auto it = backend_list().find(backend);
	if (it == backend_list().end()) {
		ErrorLog("net: unknown backend: " + backend);
		return;
	}

	servers.emplace_back(it->second->create_server(backbuffer, conf));
}

network::manager::manager()
{
}

void network::server_manager::publish_vehicle_list()
{
	if (Entities.empty())
		return;

	Entities.refresh();

	// resend on every change, and once in a while regardless, so that a peer which has
	// just finished catching up does not have to wait for somebody to move
	bool const changed = (Entities.entries() != last_published_list);
	if (!changed && --publish_countdown > 0)
		return;

	last_published_list = Entities.entries();
	publish_countdown = PUBLISH_INTERVAL_FRAMES;

	vehicle_list msg;
	msg.vehicles = last_published_list;

	for (auto srv : servers)
		srv->push_message(msg);
}

void network::server_manager::update_crews()
{
	Crews.reconcile();

	if (--crew_publish_countdown <= 0) {
		// peers that joined after somebody took a vehicle over have to learn about it too
		crew_publish_countdown = PUBLISH_INTERVAL_FRAMES;
		Crews.mark_all_pending();
	}

	auto const changed = Crews.take_pending_updates();
	for (NetworkEntityId const id : changed) {
		crew_update msg;
		msg.entity_id = id;
		msg.crew = Crews.crew_of(id);

		for (auto srv : servers)
			srv->push_message(msg);
	}
}

void network::server_manager::apply_local_claim(NetworkEntityId entity_id)
{
	auto const result = Crews.claim(PEER_HOST, entity_id);

	if (result == claim_result::granted || result == claim_result::already_member) {
		Global.network_lobby_message.clear();
		Global.network_pending_vehicle = Entities.name_of(entity_id);
	}
	else {
		WriteLog("net: refused local claim of vehicle " + std::to_string(entity_id) + ": " + describe(result), logtype::net);
		Global.network_lobby_message = describe(result);
	}
}

void network::server_manager::apply_local_leave(NetworkEntityId entity_id)
{
	if (Crews.leave(PEER_HOST, entity_id))
		Global.network_leave_pending = true;
}

void network::manager::request_claim(NetworkEntityId entity_id)
{
	if (client) {
		client->send_claim(entity_id);
		return;
	}

	if (servers)
		servers->apply_local_claim(entity_id);
}

void network::manager::request_leave(NetworkEntityId entity_id)
{
	if (client) {
		client->send_leave(entity_id);
		// the server owns the roster, but stepping out of our own cab is a local matter
		Global.network_leave_pending = true;
		return;
	}

	if (servers)
		servers->apply_local_leave(entity_id);
}

void network::manager::update()
{
	for (auto &backend : backend_list())
		backend.second->update();

	if (servers && simulation::is_ready) {
		servers->update_crews();
		servers->publish_vehicle_list();
	}

	if (client)
		client->update();
}

void network::manager::create_server(const std::string &backend, const std::string &conf)
{
	servers.emplace();
	servers->create_server(backend, conf);
}

void network::manager::connect(const std::string &backend, const std::string &conf)
{
	auto it = backend_list().find(backend);
	if (it == backend_list().end()) {
		ErrorLog("net: unknown backend: " + backend);
		return;
	}

	client = it->second->create_client(conf);
}

//std::unordered_map<std::string, std::shared_ptr<network::backend_manager>> network::backend_list;

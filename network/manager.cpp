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
#include "network/snapshot.h"

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
	if (dt == 0.0)
		return;

	// a client no longer replays these frame by frame, so an empty one is only worth
	// sending now and then, to carry the tick and the digest
	if (commands.empty() && (--heartbeat_countdown > 0))
		return;

	if (commands.empty())
		heartbeat_countdown = HEARTBEAT_INTERVAL_FRAMES;

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
	Crews.expire_absences();
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

void network::manager::notify_scenario_loaded()
{
	if (client)
		client->send_ready();
}

void network::server_manager::publish_state()
{
	if (--state_countdown > 0)
		return;

	state_countdown = STATE_INTERVAL_FRAMES;

	// most of the time only what changed goes out; every so often the whole thing does,
	// so that anything a peer missed heals by itself
	bool const full = ((state_updates++ % STATE_FULL_EVERY) == 0);

	auto const blob = take_snapshot(full);
	if (blob.empty())
		return;

	snapshot msg;
	msg.tick = Global.simulation_tick;
	msg.mode = 1; // correction on top of a running simulation
	msg.blob = blob;

	for (auto srv : servers)
		srv->push_message(msg);
}

void network::manager::request_resync(uint64_t tick, uint64_t state_hash)
{
	if (client)
		client->send_resync_request(tick, state_hash);
}

void network::manager::update()
{
	for (auto &backend : backend_list())
		backend.second->update();

	if (servers && Global.simulation_loaded) {
		// note: not simulation::is_ready. that one waits for the local player's cab, and
		// the cab of a claimed vehicle is built by update_crews() - waiting for it here
		// would leave the two waiting on each other
		servers->update_crews();
		servers->publish_vehicle_list();
		servers->publish_state();
	}

	if (client) {
		client->update();

		if (Global.simulation_loaded)
			client->publish_state();
	}
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

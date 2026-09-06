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

void network::server_manager::push_delta(double render_dt, double dt, double sync, const command_queue::commands_map &commands)
{
	if (dt == 0.0 && commands.empty())
		return;

	frame_info msg;
	msg.render_dt = render_dt;
	msg.dt = dt;
	msg.sync = sync;
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

void network::manager::update()
{
	for (auto &backend : backend_list())
		backend.second->update();

	if (servers && simulation::is_ready)
		servers->publish_vehicle_list();

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

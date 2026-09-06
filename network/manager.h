#pragma once
#include <memory>
#include "network/network.h"
#include "input/command.h"

namespace network
{
    class server_manager
	{
	private:
		std::vector<std::shared_ptr<server>> servers;
		std::shared_ptr<std::fstream> backbuffer;

	public:
		server_manager();

		void push_delta(double render_dt, double dt, double sync, const command_queue::commands_map &commands);
		command_queue::commands_map pop_commands();
		void create_server(const std::string &backend, const std::string &conf);
		// refreshes the authoritative vehicle roster and publishes it when it changed
		void publish_vehicle_list();

	private:
		// how many frames may pass between two unconditional broadcasts of the roster
		static const int PUBLISH_INTERVAL_FRAMES = 120;

		std::vector<vehicle_entry> last_published_list;
		int publish_countdown = 0;
	};

    class manager
	{
	public:
		manager();

		std::optional<server_manager> servers;
		std::shared_ptr<network::client> client;

		void create_server(const std::string &backend, const std::string &conf);
		void connect(const std::string &backend, const std::string &conf);
		void update();
	};
}

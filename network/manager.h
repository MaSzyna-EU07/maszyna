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

		void push_delta(double render_dt, double dt, uint64_t tick, uint64_t state_hash, const command_queue::commands_map &commands);
		command_queue::commands_map pop_commands();
		void create_server(const std::string &backend, const std::string &conf);
		// refreshes the authoritative vehicle roster and publishes it when it changed
		void publish_vehicle_list();
		// brings the world in line with the crew roster and announces every change
		void update_crews();
		// crew request coming from the local participant, which needs no transport
		void apply_local_claim(NetworkEntityId entity_id);
		void apply_local_leave(NetworkEntityId entity_id);

	private:
		// how many frames may pass between two unconditional broadcasts of the roster
		static const int PUBLISH_INTERVAL_FRAMES = 120;

		std::vector<vehicle_entry> last_published_list;
		int publish_countdown = 0;
		int crew_publish_countdown = 0;
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

		// routes a lobby request to whoever holds the authority: straight into the crew
		// registry when we are the server, over the wire when we are a client
		void request_claim(NetworkEntityId entity_id);
		void request_leave(NetworkEntityId entity_id);

		// the local scenario is loaded; a client asks the server for a snapshot of the
		// world as it stands right now
		void notify_scenario_loaded();
	};
}

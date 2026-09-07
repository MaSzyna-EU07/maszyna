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
		// sends the peers what the world actually looks like. this is what keeps a client
		// in line - the command stream alone cannot, because a client runs its own physics
		void publish_state();
		// crew request coming from the local participant, which needs no transport
		void apply_local_claim(NetworkEntityId entity_id);
		void apply_local_leave(NetworkEntityId entity_id);
		// hands a line of chat to every peer
		void broadcast_chat(PeerId author, const std::string &text);
		// tells the peers who is taking part
		void publish_roster();

	private:
		// how many frames may pass between two unconditional broadcasts of the roster
		static const int PUBLISH_INTERVAL_FRAMES = 120;

		// how often the authoritative state goes out, and how often it goes out whole
		// rather than as what changed since the last one
		// a horn can be pressed and let go inside a couple of frames, so the state does not
		// wait long before it goes out
		static const int STATE_INTERVAL_FRAMES = 6;
		static const int STATE_FULL_EVERY = 60;

		std::vector<vehicle_entry> last_published_list;
		int publish_countdown = 0;
		int crew_publish_countdown = 0;
		int state_countdown = 0;
		int state_updates = 0;
		int heartbeat_countdown = 0;

		// how many frames with nothing in them may pass before one is sent anyway
		static const int HEARTBEAT_INTERVAL_FRAMES = 10;
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

		// our world drifted away from the authoritative one; ask to be corrected
		void request_resync(uint64_t tick, uint64_t state_hash);

		// something the local participant said. the host says it to everybody at once, a
		// client hands it to the host to say on its behalf
		void say(const std::string &text);
	};
}

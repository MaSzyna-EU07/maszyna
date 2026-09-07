#pragma once
#include <memory>
#include <functional>
#include <optional>
#include <queue>
#include <chrono>
#include "network/message.h"
#include "network/entities.h"
#include "network/session.h"
#include "input/command.h"

namespace network
{
    //m7todo: separate client/server connection class?
    class connection
	{
		friend class server;
		friend class client;

	private:
		bool is_client;

	protected:
		size_t packet_counter;

		// exists only to keep the send buffer alive until asio is done with it
		void send_complete(std::shared_ptr<std::string> buf);

	public:
		std::function<void(const message &msg)> message_handler;

		virtual void connected() = 0;
		virtual void send_message(const message &msg) = 0;
		virtual void send_messages(const std::vector<std::shared_ptr<message>> &messages) = 0;

		connection(bool client = false, size_t counter = 0);
		void set_handler(std::function<void(const message &msg)> handler);

		virtual void disconnect() = 0;

		enum peer_state {
			AWAITING_HELLO,
			// handshake done, the peer is loading the scenario; it gets a snapshot and
			// goes live the moment it says it is ready
			AWAITING_READY,
			ACTIVE,
			DEAD
		};
		peer_state state;

		// session identity of the peer on the other side of this connection
		PeerId peer_id { PEER_NONE };
	};

	class server
	{
	private:
		// the session log is still written out, it is handy when picking a desync apart,
		// but joining no longer means replaying it
		std::shared_ptr<std::istream> backbuffer;

	protected:
		void handle_message(std::shared_ptr<connection> conn, const message &msg);

		std::vector<std::shared_ptr<connection>> clients;

		command_queue::commands_map client_commands_queue;

	protected:
		// drops peers whose socket is gone, together with their crew memberships
		void prune_clients();

	public:
		server(std::shared_ptr<std::istream> buf);
		void push_delta(const frame_info &msg);
		// sends an out of band message to every active peer. unlike push_delta this is not
		// written to the backbuffer, so it does not become part of the replayed history
		void push_message(const message &msg);
		command_queue::commands_map pop_commands();
	};

	// one authoritative simulation step handed to the client
	struct frame_delta
	{
		double dt { 0.0 };
		uint64_t tick { 0 };
		uint64_t state_hash { 0 };
		uint32_t state_hash_version { 0 };
		command_queue::commands_map commands;
		// false when there was nothing ready to consume in this pass
		bool valid { false };
	};

	class client
	{
	protected:
		virtual void connect() = 0;
		void handle_message(std::shared_ptr<connection> conn, const message &msg);
		std::shared_ptr<connection> conn;
		size_t resume_frame_counter = 0;
		size_t reconnect_delay = 0;

		const size_t RECONNECT_DELAY_FRAMES = 60;

		// authoritative frames that have arrived and not been handed to the simulation yet.
		// they are taken in full every render: nothing is held back on purpose. the old
		// design played them out one per frame at the server's pace, which meant the world
		// ran at the wrong speed whenever the two machines drew at different rates, and
		// every frame kept in reserve was a frame of delay on the player's own controls
		std::queue<frame_info> delta_queue;

		// how far out of place a correction found us, in a row. a full resync is only
		// worth asking for when the routine stream is not catching up on its own
		int bad_corrections = 0;
		uint64_t last_resync_tick = 0;
		int state_countdown = 0;
		int state_updates = 0;
		static constexpr int STATE_INTERVAL_FRAMES = 6;
		static constexpr int STATE_FULL_EVERY = 100;
		static constexpr double RESYNC_POSITION_ERROR = 25.0;
		static constexpr int RESYNC_BAD_CORRECTIONS = 5;
		// a correction needs time to take hold; asking again before it has is pointless
		static constexpr uint64_t RESYNC_COOLDOWN_TICKS = 600;

		std::chrono::high_resolution_clock::time_point last_rcv;

	public:
		void update();
		// hands over everything the authority has sent since the last call, merging the
		// commands into the map. returns what the newest of those frames said about itself
		frame_delta take_pending(command_queue::commands_map &commands);
		void send_commands(command_queue::commands_map commands);
		// tells the server the scenario is loaded and a snapshot can be applied
		void send_ready();
		// publishes what our own trains are doing, so that the rest of the session can see
		// them. we run their physics, so this is the authority on them
		void publish_state();
		// our world has drifted too far to carry on; ask to be put back in line
		void send_resync_request(uint64_t tick, uint64_t state_hash);
		// lobby requests; the server is the one that decides
		void send_claim(NetworkEntityId entity_id);
		void send_leave(NetworkEntityId entity_id);
		int get_frame_counter() {
			return resume_frame_counter;
		}
		int get_awaiting_frames() {
			return delta_queue.size();
		}
	};

	class backend_manager
	{
	public:
		virtual std::shared_ptr<server> create_server(std::shared_ptr<std::fstream>, const std::string &conf) = 0;
		virtual std::shared_ptr<client> create_client(const std::string &conf) = 0;
		virtual void update() = 0;
	};

    // HACK: static initialization order fiasco fix
//	extern std::unordered_map<std::string, backend_manager*> backend_list;
    using backend_list_t = std::unordered_map<std::string, backend_manager*>;
    backend_list_t& backend_list();
}

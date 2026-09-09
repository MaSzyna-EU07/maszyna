module;
#include <array>
#include <chrono>
#include <cstddef>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <type_traits>
#include <utility>
#include <deque>
#include <unordered_map>
#include <fstream>
#include <string>
#include <optional>
#include <vector>
#include <memory>

export module eu07.network.manager;
import eu07.network.network;
import eu07.input.command;

export {

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

}  // export

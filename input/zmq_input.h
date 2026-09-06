#pragma once

#include <zmq_addon.hpp>
#include "input/command.h"
#include "hardware/registry/state_registry.h"

class zmq_input
{

    zmq::context_t ctx;
    std::optional<zmq::socket_t> sock;

    enum class input_type
    {
        none,
        toggle, // two commands, each mapped to one state; press event on state change
        impulse, // one command; press event when set, release when cleared
        value // one command; press event, value of specified byte passed as param1
    };

    struct peer_state {
        float update_interval;
        // symbols this peer subscribed to, resolved through the shared TrainState registry
        std::vector<hardware::state_descriptor const *> sopi_list;
        std::vector<std::tuple<input_type, user_command, user_command, bool>> sipo_list;;
        std::chrono::time_point<std::chrono::high_resolution_clock> last_update;
    };

    std::map<uint32_t, peer_state> peers;

    float unpack_float(const zmq::message_t &);
    zmq::message_t pack_float(float f);

    std::unordered_map<std::string, user_command> nametocommandmap;

    command_relay relay;

public:
    zmq_input();
    void poll();
};

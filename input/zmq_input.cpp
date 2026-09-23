#include "stdafx.h"
#include "input/zmq_input.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "vehicle/Train.h"

zmq_input::zmq_input()
{
    sock.emplace(ctx, zmq::socket_type::router);
    sock->bind(Global.zmq_address);

    // build helper translation tables
    std::size_t commandid = 0;
    for( auto const &description : simulation::Commands_descriptions ) {
        nametocommandmap.emplace(
            description.name,
            static_cast<user_command>( commandid ) );
        ++commandid;
    }
}

float zmq_input::unpack_float(const zmq::message_t &msg) {
    if (msg.size() < 4)
        return 0.0f;

    uint8_t *buf = (uint8_t*)msg.data();
    uint32_t v = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
    return reinterpret_cast<float&>(v);
}

zmq::message_t zmq_input::pack_float(float f) {
    uint32_t v = reinterpret_cast<uint32_t&>(f);
    uint8_t buf[4];
    buf[3] = v;
    buf[2] = v >> 8;
    buf[1] = v >> 16;
    buf[0] = v >> 24;
    return zmq::message_t(buf, 4);
}

void zmq_input::poll()
{
    zmq::multipart_t multipart;
    bool ok;
    while ((ok = multipart.recv(*sock, ZMQ_DONTWAIT))) {
        if (multipart.size() < 3)
            continue;

        if (multipart[0].size() != 5)
            continue;

        uint8_t* buf = (uint8_t*)multipart[0].data();
        uint32_t peer_id = buf[1] << 24 | buf[2] << 16 | buf[3] << 8 | buf[4];

        auto peer_it = peers.find(peer_id);
        if (peer_it == peers.end()) {
            peer_it = peers.emplace(peer_id, peer_state()).first;
            peer_it->second.update_interval = 60.0f;
            peer_it->second.last_update = std::chrono::high_resolution_clock::now();
        }

        if (multipart[1] == zmq::message_t("REG_SOPI", 8)) {
            if (multipart.size() < 4)
                continue;

            peer_it->second.update_interval = unpack_float(multipart[2]);
            peer_it->second.sopi_list.clear();

            for (size_t i = 3; i < multipart.size(); i++) {
                std::string chan_name((char*)multipart[i].data(), multipart[i].size());

                auto const *descriptor = hardware::state_registry::instance().find(chan_name);
                if (descriptor != nullptr)
                    peer_it->second.sopi_list.push_back(descriptor);
            }
        }
        if (multipart[1] == zmq::message_t("REG_SIPO", 8)) {
            peer_it->second.sipo_list.clear();

            for (size_t i = 2; i < multipart.size(); i++) {
                std::string chan_name((char*)multipart[i].data(), multipart[i].size());

                std::istringstream stream(chan_name);
                std::string id1, id2;
                std::getline(stream, id1, '/');
                std::getline(stream, id2, '/');

                input_type type = input_type::none;
                user_command cmd1 = user_command::none, cmd2 = user_command::none;

                auto cmd1_it = nametocommandmap.find(id1);
                if (cmd1_it != nametocommandmap.end())
                    cmd1 = cmd1_it->second;

                if (id2 == "" || id2 == "v")
                    type = input_type::value;
                else if (id2 == "i")
                    type = input_type::impulse;
                else {
                    type =  input_type::toggle;
                    auto cmd2_it = nametocommandmap.find(id2);
                    if (cmd2_it != nametocommandmap.end())
                        cmd2 = cmd2_it->second;
                }

                if (cmd1 == user_command::none)
                    type = input_type::none;

                peer_it->second.sipo_list.push_back(std::make_tuple(type, cmd1, cmd2, false));
            }
        }
        if (multipart[1] == zmq::message_t("SIPO_DATA", 9)) {
            if (peer_it->second.sipo_list.size() != multipart.size() - 2) {
                zmq::multipart_t msg;

                zmq::message_t addr;
                addr.copy(multipart[0]);

                msg.add(std::move(addr));
                msg.addstr("UNCF_SIPO");
                msg.send(*sock, ZMQ_DONTWAIT);

                continue;
            }

            size_t i = 1;
            for (auto &entry : peer_it->second.sipo_list) {
                i++;

                float value = unpack_float(multipart[i]);
                input_type type = std::get<0>(entry);

                if (type == input_type::value) {
                    relay.post(std::get<1>(entry), value, 0, GLFW_PRESS, 0);
                    continue;
                }
                else if (type == input_type::none)
                    continue;

                bool state = value > 0.5f;
                bool changed = state != std::get<3>(entry);

                if (!changed)
                    continue;

                auto const action { (
                    type != input_type::impulse ?
                        GLFW_PRESS :
                        state ? GLFW_PRESS : GLFW_RELEASE) };

                auto const command { (
                    type != input_type::toggle ?
                        std::get<1>( entry ) :
                        state ? std::get<1>(entry) : std::get<2>(entry) ) };

                std::get<3>(entry) = state;

                relay.post(command, 0, 0, action, 0);
            }
        }
    }

    hardware::state_snapshot snapshot;
    hardware::state_registry::capture(snapshot);

    auto now = std::chrono::high_resolution_clock::now();

    for (auto peer = peers.begin(); peer != peers.end();) {
        if (std::chrono::duration<float>(now - peer->second.last_update).count() < peer->second.update_interval) {
            ++peer;
            continue;
        }

        peer->second.last_update = now;

        if (peer->second.sopi_list.empty()) {
            ++peer;
            continue;
        }

        uint8_t peerbuf[5] = { 0, (uint8_t)(peer->first >> 24), (uint8_t)(peer->first >> 16), (uint8_t)(peer->first >> 8), (uint8_t)peer->first };
        zmq::multipart_t msg;

        msg.addmem(peerbuf, sizeof(peerbuf));
        msg.addstr("SOPI_DATA");

        for (auto const *field : peer->second.sopi_list)
            msg.add(pack_float(static_cast<float>(field->getter(snapshot).numeric)));

        if (!msg.send(*sock, ZMQ_DONTWAIT))
            peer = peers.erase(peer);
        else
            ++peer;
    }
}

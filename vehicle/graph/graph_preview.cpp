/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#ifdef DBG_NEW
#undef new
#endif
#define ASIO_DISABLE_VISIBILITY
#include <asio.hpp>

#include "vehicle/graph/graph_preview.h"
#include "vehicle/graph/graph_instance.h"
#include "vehicle/graph/blocks/cab.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"
#include "model/MdlMngr.h"
#include "model/Model3d.h"
#include "simulation/simulation.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

#include <array>
#include <chrono>
#include <deque>
#include <nlohmann/json.hpp>

namespace vgraph
{

namespace
{

using json = nlohmann::json;

int constexpr protocol_version{1};
// pushes of the values per second
double constexpr push_rate{10.0};

class preview_client : public std::enable_shared_from_this<preview_client>
{
public:
	explicit preview_client(asio::io_context &Context) : m_socket(Context) {}

	asio::ip::tcp::socket &socket()
	{
		return m_socket;
	}
	void start()
	{
		read();
	}
	bool closed() const
	{
		return m_closed;
	}
	// lines received since the last call
	std::vector<std::string> take_lines()
	{
		std::vector<std::string> lines;
		lines.swap(m_inbox);
		return lines;
	}
	void send(json const &Message)
	{
		if (m_closed)
		{
			return;
		}
		m_outbox.push_back(Message.dump() + "\n");
		if (false == m_writing)
		{
			write();
		}
	}
	// vehicle the client watches, empty when not attached
	std::string attached;
	std::chrono::steady_clock::time_point lastpush;

private:
	void read()
	{
		auto self{shared_from_this()};
		asio::async_read_until(m_socket, m_buffer, '\n', [self](asio::error_code const &Error, std::size_t const Size) {
			if (Error)
			{
				self->m_closed = true;
				return;
			}
			std::string line{asio::buffers_begin(self->m_buffer.data()), asio::buffers_begin(self->m_buffer.data()) + Size};
			self->m_buffer.consume(Size);
			while (false == line.empty() && (line.back() == '\n' || line.back() == '\r'))
			{
				line.pop_back();
			}
			if (false == line.empty())
			{
				self->m_inbox.emplace_back(std::move(line));
			}
			self->read();
		});
	}
	void write()
	{
		if (m_outbox.empty())
		{
			m_writing = false;
			return;
		}
		m_writing = true;
		auto self{shared_from_this()};
		asio::async_write(m_socket, asio::buffer(m_outbox.front()), [self](asio::error_code const &Error, std::size_t) {
			if (Error)
			{
				self->m_closed = true;
				self->m_writing = false;
				return;
			}
			self->m_outbox.pop_front();
			self->write();
		});
	}

	asio::ip::tcp::socket m_socket;
	asio::streambuf m_buffer;
	std::vector<std::string> m_inbox;
	std::deque<std::string> m_outbox;
	bool m_writing{false};
	bool m_closed{false};
};

TDynamicObject *find_vehicle(std::string const &Name)
{
	auto *vehicle{simulation::Vehicles.find(Name)};
	return (vehicle != nullptr && vehicle->MoverParameters != nullptr && vehicle->MoverParameters->Graph ? vehicle : nullptr);
}

json error(std::string const &Message)
{
	return {{"op", "error"}, {"message", Message}};
}

// names of the submodels of the model, sorted, for the pickers of the editor
json submodel_names(TModel3d *Model)
{
	std::vector<std::string> names;
	// the siblings are walked in a loop, only the children recursively
	std::vector<TSubModel *> pending{Model->GetSMRoot()};
	while (false == pending.empty())
	{
		auto *submodel{pending.back()};
		pending.pop_back();
		for (; submodel != nullptr; submodel = submodel->NextGet())
		{
			if (false == submodel->pName.empty())
			{
				names.push_back(submodel->pName);
			}
			if (submodel->ChildGet() != nullptr)
			{
				pending.push_back(submodel->ChildGet());
			}
		}
	}
	std::sort(std::begin(names), std::end(names));
	names.erase(std::unique(std::begin(names), std::end(names)), std::end(names));
	return names;
}

json model_entry(TModel3d *Model)
{
	if (Model == nullptr)
	{
		return nullptr;
	}
	return {{"model", Model->NameGet()}, {"submodels", submodel_names(Model)}};
}

class preview_server
{
public:
	explicit preview_server(int const Port) : m_acceptor(m_context)
	{
		asio::error_code error;
		asio::ip::tcp::endpoint const endpoint{asio::ip::make_address("127.0.0.1"), static_cast<unsigned short>(Port)};
		m_acceptor.open(endpoint.protocol(), error);
		if (false == static_cast<bool>(error))
		{
			m_acceptor.set_option(asio::ip::tcp::acceptor::reuse_address(true), error);
			m_acceptor.bind(endpoint, error);
		}
		if (false == static_cast<bool>(error))
		{
			m_acceptor.listen(asio::socket_base::max_listen_connections, error);
		}
		if (error)
		{
			ErrorLog("Vehicle graph preview: can't listen on 127.0.0.1:" + std::to_string(Port) + ": " + error.message());
			return;
		}
		WriteLog("Vehicle graph preview: listening on 127.0.0.1:" + std::to_string(Port));
		accept();
	}

	void update()
	{
		m_context.restart();
		m_context.poll();
		auto const now{std::chrono::steady_clock::now()};
		for (auto &client : m_clients)
		{
			for (auto const &line : client->take_lines())
			{
				handle(*client, line);
			}
			if (false == client->attached.empty() && now - client->lastpush >= std::chrono::duration<double>(1.0 / push_rate))
			{
				client->lastpush = now;
				push_values(*client);
			}
		}
		m_clients.erase(std::remove_if(std::begin(m_clients), std::end(m_clients), [](auto const &Client) { return Client->closed(); }), std::end(m_clients));
		// the writes started above
		m_context.poll();
	}

private:
	void accept()
	{
		auto client{std::make_shared<preview_client>(m_context)};
		m_acceptor.async_accept(client->socket(), [this, client](asio::error_code const &Error) {
			if (false == static_cast<bool>(Error))
			{
				client->start();
				m_clients.push_back(client);
			}
			accept();
		});
	}

	void handle(preview_client &Client, std::string const &Line)
	{
		auto const message{json::parse(Line, nullptr, false)};
		if (message.is_discarded() || false == message.is_object() || false == message.contains("op") || false == message["op"].is_string())
		{
			Client.send(error("malformed message, expected a json object with \"op\""));
			return;
		}
		auto const op{message["op"].get<std::string>()};
		try
		{
			if (op == "hello")
			{
				Client.send({{"op", "hello"}, {"version", protocol_version}, {"simulator", "MaSzyna " + Global.asVersion}});
			}
			else if (op == "list")
			{
				auto items{json::array()};
				for (auto const *vehicle : simulation::Vehicles.sequence())
				{
					if (vehicle == nullptr || vehicle->MoverParameters == nullptr || nullptr == vehicle->MoverParameters->Graph)
					{
						continue;
					}
					items.push_back({{"name", vehicle->name()}, {"file", vehicle->MoverParameters->Graph->definition().path()}, {"graph", vehicle->MoverParameters->Graph->definition().name()}});
				}
				Client.send({{"op", "vehicles"}, {"items", items}});
			}
			else if (op == "attach")
			{
				auto const name{message.value("vehicle", std::string{})};
				auto *vehicle{find_vehicle(name)};
				if (vehicle == nullptr)
				{
					Client.send(error("no graph vehicle named \"" + name + "\""));
					return;
				}
				Client.attached = name;
				Client.lastpush = {};
				Client.send({{"op", "attached"}, {"vehicle", name}, {"file", vehicle->MoverParameters->Graph->definition().path()}});
			}
			else if (op == "detach")
			{
				Client.attached.clear();
			}
			else if (op == "switch" || op == "force" || op == "unforce" || op == "reload" || op == "submodels")
			{
				auto *vehicle{find_vehicle(Client.attached)};
				if (vehicle == nullptr)
				{
					Client.send(error("not attached to a graph vehicle"));
					return;
				}
				if (op == "switch")
				{
					operate(Client, *vehicle->MoverParameters->Graph, message);
				}
				else if (op == "reload")
				{
					reload(Client, *vehicle);
				}
				else if (op == "submodels")
				{
					submodels(Client, *vehicle);
				}
				else
				{
					force(Client, *vehicle->MoverParameters->Graph, message, op == "force");
				}
			}
			else
			{
				Client.send(error("unknown op \"" + op + "\""));
			}
		}
		catch (json::exception const &Error)
		{
			Client.send(error(std::string{"malformed message: "} + Error.what()));
		}
	}

	void operate(preview_client &Client, graph_instance &Graph, json const &Message)
	{
		auto const node{Message.value("node", std::string{})};
		auto *control{dynamic_cast<cab_switch *>(Graph.find_block(node))};
		if (control == nullptr)
		{
			Client.send(error("\"" + node + "\" isn't a cab switch"));
			return;
		}
		auto const action{Message.value("action", std::string{})};
		auto const value{Message.value("value", 0.0)};
		if (action == "press")
			control->press();
		else if (action == "release")
			control->release();
		else if (action == "toggle")
			control->toggle();
		else if (action == "inc")
			control->increase();
		else if (action == "dec")
			control->decrease();
		else if (action == "set")
			control->set_position(static_cast<int>(std::lround(value)));
		else if (action == "value")
			control->set_value(value);
		else
			Client.send(error("unknown switch action \"" + action + "\""));
	}

	void force(preview_client &Client, graph_instance &Graph, json const &Message, bool const Force)
	{
		auto const port{Message.value("port", std::string{})};
		auto const slot{Graph.definition().find_signal(port)};
		if (slot < 0)
		{
			Client.send(error("unknown signal port \"" + port + "\""));
			return;
		}
		if (Force)
		{
			Graph.force(slot, Message.value("value", 0.0));
		}
		else
		{
			Graph.release(slot);
		}
	}

	void reload(preview_client &Client, TDynamicObject &Vehicle)
	{
		auto &body{*Vehicle.MoverParameters};
		std::vector<diagnostic> diagnostics;
		auto const path{body.Graph->definition().path()};
		auto const graph{load_graph(path, diagnostics)};
		if (graph == nullptr)
		{
			std::string text;
			for (auto const &entry : diagnostics)
			{
				text += (text.empty() ? "" : "\n") + to_string(entry);
			}
			Client.send(error(text));
			return;
		}
		// the body keeps its state; the blocks start anew. the old graph is kept alive, as the cab and the sounds
		// of the vehicle hold addresses of its blocks until they are created again below
		m_retired.push_back(body.Graph);
		auto const old{body.Graph->body()};
		body.Graph = std::make_shared<graph_instance>(graph);
		auto &state{body.Graph->body()};
		state.mover = &body;
		state.native_brake_pipe = old.native_brake_pipe;
		state.native_feed_pipe = old.native_feed_pipe;
		Vehicle.create_graph_sounds();
		if (simulation::Train != nullptr && simulation::Train->Dynamic() == &Vehicle)
		{
			simulation::Train->InitializeCab(body.CabOccupied, body.TypeName + ".mmd");
		}
		WriteLog("Vehicle graph preview: reloaded " + path);
		Client.send({{"op", "reloaded"}});
	}

	// submodels of the models of the vehicle: the vehicle, its low poly interior and the cabs of the .mmd file
	void submodels(preview_client &Client, TDynamicObject &Vehicle)
	{
		auto cabs{json::array()};
		// the cab models are loaded the same way as by TTrain::InitializeCab(), so the models switching the cab would load
		// are shared with it. in a network game the cab models are separate for each train: only the cabs of the
		// player's train can be looked up then
		auto const networked{false == Global.network_servers.empty() || Global.network_client.has_value()};
		auto const occupied{simulation::Train != nullptr && simulation::Train->Dynamic() == &Vehicle};
		if (false == networked || occupied)
		{
			auto const uid{networked ? static_cast<int>(simulation::Train->id()) : 0};
			std::array<bool, 3> listed{false, false, false};
			cParser parser(Vehicle.MoverParameters->TypeName + ".mmd", cParser::buffer_FILE, Vehicle.asBaseDir, true, std::vector<std::string>(), true);
			std::string token;
			do
			{
				token = "";
				parser.getTokens();
				parser >> token;
				if (token.size() != 10 || token.compare(0, 3, "cab") != 0 || token.compare(4, 6, "model:") != 0 || token[3] < '0' || token[3] > '2')
				{
					continue;
				}
				auto const cabindex{token[3] - '0'};
				token = "";
				parser.getTokens();
				parser >> token;
				if (token.empty() || listed[cabindex])
				{
					continue;
				}
				listed[cabindex] = true;
				TModel3d *model{nullptr};
				std::replace(token.begin(), token.end(), '\\', '/');
				if (token != "none")
				{
					erase_leading_slashes(token);
					auto const texturepath{Global.asCurrentTexturePath};
					Global.asCurrentTexturePath = Vehicle.asBaseDir;
					model = TModelsManager::GetModel(Vehicle.asBaseDir + token, true, false, uid);
					Global.asCurrentTexturePath = texturepath;
				}
				else if (cabindex == 1)
				{
					// the controls of cab 1 are looked up in the model of the vehicle then
					model = Vehicle.mdModel;
				}
				if (model != nullptr)
				{
					auto entry{model_entry(model)};
					entry["cab"] = cabindex;
					cabs.push_back(std::move(entry));
				}
			} while (false == token.empty());
		}
		Client.send({{"op", "submodels"}, {"vehicle", model_entry(Vehicle.mdModel)}, {"lowpoly", model_entry(Vehicle.mdLowPolyInt)}, {"cabs", cabs}});
	}

	void push_values(preview_client &Client)
	{
		auto *vehicle{find_vehicle(Client.attached)};
		if (vehicle == nullptr)
		{
			Client.send(error("the vehicle \"" + Client.attached + "\" is gone"));
			Client.attached.clear();
			return;
		}
		auto const &graph{*vehicle->MoverParameters->Graph};
		auto const &definition{graph.definition()};
		json signals = json::object();
		json states = json::object();
		json links = json::object();
		auto const &nodes{definition.nodes()};
		for (std::size_t nodeindex = 0; nodeindex < nodes.size(); ++nodeindex)
		{
			auto const &node{nodes[nodeindex]};
			for (std::size_t portindex = 0; portindex < node.ports.size(); ++portindex)
			{
				auto const &port{node.ports[portindex]};
				if (port.direction != port_direction::out)
				{
					continue;
				}
				auto const name{node.id + "." + port.key};
				if (port.dom == domain::signal)
				{
					signals[name] = graph.signal(node.slots[portindex]);
				}
				else
				{
					auto const &link{graph.link(node.slots[portindex])};
					links[name] = {link.forward, link.back};
				}
			}
			auto const &block{graph.node_block(nodeindex)};
			for (std::size_t stateindex = 0; stateindex < block.state_count(); ++stateindex)
			{
				states[node.id + ":" + block.state_name(stateindex)] = block.state_value(stateindex);
			}
		}
		Client.send({{"op", "values"}, {"time", graph.time()}, {"signals", signals}, {"states", states}, {"links", links}});
	}

	asio::io_context m_context;
	asio::ip::tcp::acceptor m_acceptor;
	std::vector<std::shared_ptr<preview_client>> m_clients;
	std::vector<std::shared_ptr<graph_instance>> m_retired;
};

} // namespace

void preview_update()
{
	if (false == Global.vehiclegraph_preview)
	{
		return;
	}
	static preview_server server{Global.vehiclegraph_preview_port};
	server.update();
}

} // namespace vgraph

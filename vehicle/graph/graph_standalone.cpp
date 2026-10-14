/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_standalone.h"
#include "vehicle/graph/graph_catalog.h"
#include "vehicle/graph/graph_instance.h"
#include "vehicle/graph/graph_lua.h"
#include "vehicle/graph/blocks/cab.h"
#include "McZapkie/MOVER.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>

// command line tools of the vehicle graph. they run without the rest of the simulator; the log service doesn't run
// in this mode, so the results are reported directly to the console
namespace vgraph
{

namespace
{

struct script_event
{
	double time{0.0};
	std::string operation;
	std::string target;
	double value{0.0};
	int line{0};
	std::size_t vehicle{0}; // index of the vehicle in the consist; the target was prefixed "k:" for the vehicle k
};

bool load_script(std::string const &Path, std::vector<script_event> &Events)
{
	std::ifstream file{Path};
	if (false == file.is_open())
	{
		std::printf("can't open script file \"%s\"\n", Path.c_str());
		return false;
	}
	std::string line;
	int linenumber{0};
	while (std::getline(file, line))
	{
		++linenumber;
		auto const comment{line.find('#')};
		if (comment != std::string::npos)
		{
			line.erase(comment);
		}
		std::istringstream stream{line};
		script_event event;
		event.line = linenumber;
		if (false == static_cast<bool>(stream >> event.time))
		{
			// empty line
			continue;
		}
		if (false == static_cast<bool>(stream >> event.operation >> event.target))
		{
			std::printf("%s:%d: expected: <time> <operation> <target> [value]\n", Path.c_str(), linenumber);
			return false;
		}
		auto const needsvalue{event.operation == "set" || event.operation == "value" || event.operation == "force"};
		if (needsvalue && false == static_cast<bool>(stream >> event.value))
		{
			std::printf("%s:%d: operation \"%s\" needs a value\n", Path.c_str(), linenumber, event.operation.c_str());
			return false;
		}
		Events.push_back(event);
	}
	std::stable_sort(std::begin(Events), std::end(Events), [](script_event const &Left, script_event const &Right) { return Left.time < Right.time; });
	return true;
}

bool apply_event(graph_instance &Instance, script_event const &Event)
{
	if (Event.operation == "force" || Event.operation == "unforce")
	{
		auto const slot{Instance.definition().find_signal(Event.target)};
		if (slot < 0)
		{
			std::printf("script line %d: unknown signal port \"%s\"\n", Event.line, Event.target.c_str());
			return false;
		}
		if (Event.operation == "force")
		{
			Instance.force(slot, Event.value);
		}
		else
		{
			Instance.release(slot);
		}
		return true;
	}
	auto *control{dynamic_cast<cab_switch *>(Instance.find_block(Event.target))};
	if (control == nullptr)
	{
		std::printf("script line %d: \"%s\" isn't a cab switch\n", Event.line, Event.target.c_str());
		return false;
	}
	if (Event.operation == "press")
	{
		control->press();
	}
	else if (Event.operation == "release")
	{
		control->release();
	}
	else if (Event.operation == "toggle")
	{
		control->toggle();
	}
	else if (Event.operation == "inc")
	{
		control->increase();
	}
	else if (Event.operation == "dec")
	{
		control->decrease();
	}
	else if (Event.operation == "set")
	{
		control->set_position(static_cast<int>(std::lround(Event.value)));
	}
	else if (Event.operation == "value")
	{
		control->set_value(Event.value);
	}
	else
	{
		std::printf("script line %d: unknown operation \"%s\"\n", Event.line, Event.operation.c_str());
		return false;
	}
	return true;
}

// motion of the vehicle without the track: the forces of the graph, the motion resistance and the mass of the body
// (TMoverParameters::ComputeTotalForce and ComputeMovement, much simplified)
void move_body(TMoverParameters &Body, double const Deltatime)
{
	auto const &state{Body.Graph->body()};
	if (Body.TotalMass <= 0.0)
	{
		return;
	}
	auto const tractive{state.tractive_force};
	auto const resistance{state.brake_force + Body.FrictionForce()};
	double force;
	if (Body.V == 0.0)
	{
		// standing still until the tractive force overcomes the resistance
		force = std::abs(tractive) > resistance ? tractive - (tractive > 0.0 ? resistance : -resistance) : 0.0;
	}
	else
	{
		force = tractive - (Body.V > 0.0 ? resistance : -resistance);
	}
	auto const previous{Body.V};
	Body.AccS = force / Body.TotalMass;
	Body.V += Body.AccS * Deltatime;
	if (previous != 0.0 && Body.V * previous <= 0.0)
	{
		// stopped by the brakes and the resistance
		Body.V = 0.0;
	}
	Body.Vel = std::abs(Body.V) * 3.6;
	Body.nrot = Body.WheelDiameter > 0.0 ? Body.V / (M_PI * Body.WheelDiameter) : 0.0;
	Body.DistCounter += std::abs(Body.V) * Deltatime / 1000.0;
	Body.FTrain = tractive;
	Body.Fb = state.brake_force;
}

std::string format_number(double const Value)
{
	char buffer[32];
	// negative zero is written as zero, so equal results give equal files
	std::snprintf(buffer, sizeof(buffer), "%.9g", Value == 0.0 ? 0.0 : Value);
	return buffer;
}

// vehicle run by -graph-run: the first one, and the ones coupled behind it (-consist)
struct run_vehicle
{
	std::string path;
	bool reversed{false}; // coupled by its rear coupler, facing the other way than the first vehicle
	std::shared_ptr<graph_template const> graph;
	std::unique_ptr<TMoverParameters> body;
	std::vector<std::pair<std::string, int>> signals;
	bool hasbody{false};
	bool mains{false}; // the graph has several line breakers, so the Mains of the body is written as well
	std::string prefix; // of the result columns: empty for the first vehicle, "k:" for the vehicle k
};

// path of a vehicle of the consist: as given, or relative to the directory of the first vehicle's file
std::string consist_path(std::string const &Path, std::string const &First)
{
	if (std::ifstream{Path}.is_open())
	{
		return Path;
	}
	auto const relative{std::filesystem::path{First}.parent_path() / Path};
	if (std::filesystem::path{Path}.is_relative() && std::ifstream{relative}.is_open())
	{
		return relative.string();
	}
	return Path;
}

bool load_vehicle(run_vehicle &Vehicle)
{
	std::vector<diagnostic> diagnostics;
	Vehicle.graph = load_graph(Vehicle.path, diagnostics);
	for (auto const &entry : diagnostics)
	{
		std::printf("%s: %s\n", Vehicle.path.c_str(), to_string(entry).c_str());
	}
	if (Vehicle.graph == nullptr)
	{
		return false;
	}
	auto const &graph{*Vehicle.graph};
	std::printf("%s: %zu blocks, %zu signals, %zu links\n", Vehicle.path.c_str(), graph.nodes().size(), graph.signal_defaults().size(), graph.link_count());
	return true;
}

// the body of the vehicle, filled from the graph as in the simulator, with the body entries the graph doesn't give
// taken from the .fiz file of the same name, if there is one
bool create_body(run_vehicle &Vehicle, std::string const &Name, bool const Ready, int const Cab)
{
	auto const &input{Vehicle.path};
	auto const &graph{Vehicle.graph};
	Vehicle.body = std::make_unique<TMoverParameters>(0.0, "graph", Name, 1);
	auto &body{*Vehicle.body};
	auto fizfile{input};
	fizfile = fizfile.substr(0, std::min(fizfile.size(), fizfile.find_last_of('.'))) + ".fiz";
	if (false == body.AssignGraph(graph, fizfile))
	{
		std::printf("%s: the body section or the block parameters were rejected by the vehicle body\n", input.c_str());
		return false;
	}
	auto const vehicleblocks{std::any_of(std::begin(graph->nodes()), std::end(graph->nodes()), [](auto const &Node) {
		auto const &category{Node.type->category};
		return category != "logic" && category != "cab" && category != "indicators" && category != "script" && category != "module";
	})};
	if (vehicleblocks && (body.Dim.L <= 0.0 || body.Mass <= 0.0))
	{
		// the simulator doesn't load such a vehicle; the graph itself can still be run here
		std::printf("%s: warning: the body has no length or mass (body section, or %s), the simulator won't load the vehicle\n", input.c_str(), fizfile.c_str());
	}
	body.ComputeMass();
	body.CheckLocomotiveParameters(Ready, 1);
	body.CabActive = Cab;
	body.Graph->body().standalone = true;
	Vehicle.hasbody = (false == graph->body().empty());
	Vehicle.mains = std::count_if(std::begin(graph->nodes()), std::end(graph->nodes()), [](auto const &Node) { return Node.type->id == "el.hsb"; }) > 1;
	Vehicle.signals = graph->signal_outputs();
	return true;
}

void write_header(std::ostringstream &Results, run_vehicle const &Vehicle)
{
	auto const &graph{*Vehicle.graph};
	auto const &instance{*Vehicle.body->Graph};
	auto const &prefix{Vehicle.prefix};
	for (auto const &signal : Vehicle.signals)
	{
		Results << ';' << prefix << signal.first;
	}
	for (std::size_t nodeindex = 0; nodeindex < graph.nodes().size(); ++nodeindex)
	{
		auto const &item{instance.node_block(nodeindex)};
		for (std::size_t stateindex = 0; stateindex < item.state_count(); ++stateindex)
		{
			Results << ';' << prefix << graph.nodes()[nodeindex].id << ':' << item.state_name(stateindex);
		}
	}
	if (Vehicle.hasbody)
	{
		// the vehicle body, for graphs describing whole vehicles
		Results << ';' << prefix << "body:v;" << prefix << "body:ft;" << prefix << "body:fb;" << prefix << "body:pg;" << prefix << "body:zg;" << prefix << "body:bc";
	}
	if (Vehicle.mains)
	{
		Results << ';' << prefix << "body:mains";
	}
}

void write_values(std::ostringstream &Results, run_vehicle const &Vehicle)
{
	auto const &graph{*Vehicle.graph};
	auto const &body{*Vehicle.body};
	auto const &instance{*body.Graph};
	for (auto const &signal : Vehicle.signals)
	{
		Results << ';' << format_number(instance.signal(signal.second));
	}
	for (std::size_t nodeindex = 0; nodeindex < graph.nodes().size(); ++nodeindex)
	{
		auto const &item{instance.node_block(nodeindex)};
		for (std::size_t stateindex = 0; stateindex < item.state_count(); ++stateindex)
		{
			Results << ';' << format_number(item.state_value(stateindex));
		}
	}
	if (Vehicle.hasbody)
	{
		Results << ';' << format_number(body.Vel) << ';' << format_number(body.FTrain / 1000.0) << ';' << format_number(body.Fb / 1000.0) << ';' << format_number(body.PipePress)
		        << ';' << format_number(body.Compressor) << ';' << format_number(body.BrakePress);
	}
	if (Vehicle.mains)
	{
		Results << ';' << (body.Mains ? 1 : 0);
	}
}

} // namespace

int graph_catalog_standalone(std::string const &Output)
{
	auto const text{catalog::get().export_json()};
	if (Output.empty())
	{
		std::printf("%s\n", text.c_str());
		return 0;
	}
	std::ofstream file{Output, std::ios::binary};
	if (false == file.is_open())
	{
		std::printf("can't create file \"%s\"\n", Output.c_str());
		return 1;
	}
	file << text << '\n';
	std::printf("block catalog written to %s (%zu block types)\n", Output.c_str(), catalog::get().types().size());
	return 0;
}

int graph_run_standalone(int const Argc, char *Argv[])
{
	// -graph-run <file.vhg> [-script <events.txt>] [-o <results.csv>] [-dt <step>] [-t <duration>] [-every <steps>] [-state braked|ready] [-consist <file.vhg>[:r]]...
	if (Argc < 3)
	{
		std::printf("usage: -graph-run <file.vhg> [-script <events.txt>] [-o <results.csv>] [-dt <step, s>] [-t <duration, s>] [-every <steps>] [-state braked|ready] "
		            "[-consist <file.vhg>[:r]]...\n");
		return 1;
	}
	std::string const input{Argv[2]};
	std::vector<run_vehicle> vehicles(1);
	vehicles.front().path = input;
	std::string script;
	std::string output;
	double deltatime{0.01};
	double duration{0.0};
	int every{1};
	bool ready{false};
	for (int idx = 3; idx + 1 < Argc; idx += 2)
	{
		std::string const option{Argv[idx]};
		std::string const value{Argv[idx + 1]};
		if (option == "-script")
		{
			script = value;
		}
		else if (option == "-o")
		{
			output = value;
		}
		else if (option == "-dt")
		{
			deltatime = std::atof(value.c_str());
		}
		else if (option == "-t")
		{
			duration = std::atof(value.c_str());
		}
		else if (option == "-every")
		{
			every = std::max(1, std::atoi(value.c_str()));
		}
		else if (option == "-state")
		{
			// initial state of the vehicle body, as for a vehicle placed in a scenario standing (braked) or moving (ready)
			ready = (value == "ready");
		}
		else if (option == "-consist")
		{
			// next vehicle of the consist, coupled behind the previous one by its front coupler, or with ":r" reversed (by its rear coupler)
			run_vehicle vehicle;
			vehicle.reversed = (value.size() > 2 && value.ends_with(":r"));
			vehicle.path = consist_path(vehicle.reversed ? value.substr(0, value.size() - 2) : value, input);
			vehicle.prefix = std::to_string(vehicles.size() + 1) + ":";
			vehicles.push_back(std::move(vehicle));
		}
		else
		{
			std::printf("unknown option \"%s\"\n", option.c_str());
			return 1;
		}
	}
	if (deltatime <= 0.0)
	{
		std::printf("the step has to be positive\n");
		return 1;
	}

	// messages of the lua blocks go straight to the console
	set_log_sink([](std::string const &Message) { std::printf("%s\n", Message.c_str()); });

	for (auto &vehicle : vehicles)
	{
		if (false == load_vehicle(vehicle))
		{
			return 1;
		}
	}

	std::vector<script_event> events;
	if (false == script.empty() && false == load_script(script, events))
	{
		return 1;
	}
	for (auto &event : events)
	{
		// "k:" before the target picks the vehicle k of the consist
		auto const separator{event.target.find(':')};
		if (separator == std::string::npos || separator == 0 || event.target.find_first_not_of("0123456789") != separator)
		{
			continue;
		}
		auto const number{std::atoi(event.target.substr(0, separator).c_str())};
		if (number < 1 || number > static_cast<int>(vehicles.size()))
		{
			std::printf("script line %d: there's no vehicle %d in the consist\n", event.line, number);
			return 1;
		}
		event.vehicle = static_cast<std::size_t>(number - 1);
		event.target.erase(0, separator + 1);
	}
	if (duration <= 0.0 && false == events.empty())
	{
		// by default run a second past the last event
		duration = events.back().time + 1.0;
	}
	if (duration <= 0.0)
	{
		return 0;
	}

	for (std::size_t idx = 0; idx < vehicles.size(); ++idx)
	{
		// the first vehicle has the occupied cab
		if (false == create_body(vehicles[idx], idx == 0 ? "graph" : "graph" + std::to_string(idx + 1), ready, idx == 0 ? 1 : 0))
		{
			return 1;
		}
	}
	// the consist: every vehicle coupled to the previous one with all the couplings, so the brake pipes connect as well
	auto const allcouplings{coupling::coupler | coupling::brakehose | coupling::control | coupling::highvoltage | coupling::gangway | coupling::mainhose | coupling::heating |
	                        coupling::permanent | coupling::power24v | coupling::power110v | coupling::power3x400v};
	for (std::size_t idx = 1; idx < vehicles.size(); ++idx)
	{
		auto &previous{vehicles[idx - 1]};
		auto &vehicle{vehicles[idx]};
		previous.body->Attach(previous.reversed ? end::front : end::rear, vehicle.reversed ? end::rear : end::front, vehicle.body.get(), allcouplings, true, false);
	}

	std::ostringstream results;
	results << "time";
	for (auto const &vehicle : vehicles)
	{
		write_header(results, vehicle);
	}
	results << '\n';

	auto const stepcount{static_cast<long long>(std::ceil(duration / deltatime - 1e-9))};
	std::size_t nextevent{0};
	for (long long step = 0; step < stepcount; ++step)
	{
		// events take effect at the beginning of the first step which starts at or after their time
		auto const steptime{step * deltatime};
		while (nextevent < events.size() && events[nextevent].time <= steptime + 1e-9)
		{
			auto const &event{events[nextevent]};
			if (false == apply_event(*vehicles[event.vehicle].body->Graph, event))
			{
				return 1;
			}
			++nextevent;
		}
		// the vehicles in the order of the consist; each one moves on its own, without the forces of the couplers
		for (auto &vehicle : vehicles)
		{
			auto &body{*vehicle.body};
			body.EqvtPipePress = body.PipePress;
			body.compute_movement_graph_(deltatime);
			move_body(body, deltatime);
		}
		if ((step + 1) % every != 0 && step + 1 != stepcount)
		{
			continue;
		}
		results << format_number((step + 1) * deltatime);
		for (auto const &vehicle : vehicles)
		{
			write_values(results, vehicle);
		}
		results << '\n';
	}

	if (output.empty())
	{
		std::printf("%s", results.str().c_str());
		return 0;
	}
	std::ofstream file{output, std::ios::binary};
	if (false == file.is_open())
	{
		std::printf("can't create file \"%s\"\n", output.c_str());
		return 1;
	}
	file << results.str();
	std::printf("%lld steps written to %s\n", stepcount, output.c_str());
	return 0;
}

} // namespace vgraph

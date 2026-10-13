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
#include "vehicle/graph/blocks/cab.h"

#include <cmath>
#include <cstdio>
#include <fstream>
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

std::string format_number(double const Value)
{
	char buffer[32];
	// negative zero is written as zero, so equal results give equal files
	std::snprintf(buffer, sizeof(buffer), "%.9g", Value == 0.0 ? 0.0 : Value);
	return buffer;
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
	// -graph-run <file.vhg> [-script <events.txt>] [-o <results.csv>] [-dt <step>] [-t <duration>] [-every <steps>]
	if (Argc < 3)
	{
		std::printf("usage: -graph-run <file.vhg> [-script <events.txt>] [-o <results.csv>] [-dt <step, s>] [-t <duration, s>] [-every <steps>]\n");
		return 1;
	}
	std::string const input{Argv[2]};
	std::string script;
	std::string output;
	double deltatime{0.01};
	double duration{0.0};
	int every{1};
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

	std::vector<diagnostic> diagnostics;
	auto const graph{load_graph(input, diagnostics)};
	for (auto const &entry : diagnostics)
	{
		std::printf("%s: %s\n", input.c_str(), to_string(entry).c_str());
	}
	if (graph == nullptr)
	{
		return 1;
	}
	std::printf("%s: %zu blocks, %zu signals, %zu links\n", input.c_str(), graph->nodes().size(), graph->signal_defaults().size(), graph->link_count());

	std::vector<script_event> events;
	if (false == script.empty() && false == load_script(script, events))
	{
		return 1;
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

	graph_instance instance{graph};
	auto const signals{graph->signal_outputs()};
	std::ostringstream results;
	results << "time";
	for (auto const &signal : signals)
	{
		results << ';' << signal.first;
	}
	for (std::size_t nodeindex = 0; nodeindex < graph->nodes().size(); ++nodeindex)
	{
		auto const &item{instance.node_block(nodeindex)};
		for (std::size_t stateindex = 0; stateindex < item.state_count(); ++stateindex)
		{
			results << ';' << graph->nodes()[nodeindex].id << ':' << item.state_name(stateindex);
		}
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
			if (false == apply_event(instance, events[nextevent]))
			{
				return 1;
			}
			++nextevent;
		}
		instance.update(deltatime);
		if ((step + 1) % every != 0 && step + 1 != stepcount)
		{
			continue;
		}
		results << format_number((step + 1) * deltatime);
		for (auto const &signal : signals)
		{
			results << ';' << format_number(instance.signal(signal.second));
		}
		for (std::size_t nodeindex = 0; nodeindex < graph->nodes().size(); ++nodeindex)
		{
			auto const &item{instance.node_block(nodeindex)};
			for (std::size_t stateindex = 0; stateindex < item.state_count(); ++stateindex)
			{
				results << ';' << format_number(item.state_value(stateindex));
			}
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

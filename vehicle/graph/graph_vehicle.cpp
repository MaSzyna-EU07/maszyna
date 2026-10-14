/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "McZapkie/MOVER.h"
#include "vehicle/graph/graph_body.h"
#include "vehicle/graph/graph_catalog.h"
#include "vehicle/graph/graph_instance.h"
#include "vehicle/graph/graph_lua.h"
#include "utilities/Logs.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

// vehicle body of a graph vehicle: TMoverParameters filled from the .vhg file. the body section and the parameters of
// the blocks which stand for .fiz entries are written as .fiz lines and read with the code reading the .fiz files, so
// the derived values and the defaults are the same as for a .fiz vehicle with the same entries
namespace
{

std::string format_value(double const Value)
{
	char buffer[32];
	std::snprintf(buffer, sizeof(buffer), "%.10g", Value);
	return buffer;
}

// .fiz lines being built, in the order of the sections
class fiz_lines
{
public:
	void add(std::string const &Section, std::string const &Key, std::string const &Value)
	{
		auto &line{find(Section)};
		line += " " + Key + "=" + Value;
	}
	bool has(std::string const &Section, std::string const &Key) const
	{
		for (auto const &entry : m_lines)
		{
			if (entry.first == Section)
			{
				return (entry.second + " ").find(" " + Key + "=") != std::string::npos;
			}
		}
		return false;
	}
	// the line of the section, empty if nothing was added to it
	std::string get(std::string const &Section) const
	{
		for (auto const &entry : m_lines)
		{
			if (entry.first == Section)
			{
				return entry.first + entry.second;
			}
		}
		return {};
	}
	std::vector<std::pair<std::string, std::string>> const &lines() const
	{
		return m_lines;
	}

private:
	std::string &find(std::string const &Section)
	{
		for (auto &entry : m_lines)
		{
			if (entry.first == Section)
			{
				return entry.second;
			}
		}
		m_lines.emplace_back(Section, std::string{});
		return m_lines.back().second;
	}

	std::vector<std::pair<std::string, std::string>> m_lines;
};

// value of a parameter as written in a .fiz file; values of the .fiz files can't have spaces
std::string fiz_value(vgraph::param_value const &Value, std::vector<std::string> const &Choices, std::vector<std::string> const &Fizvalues)
{
	switch (Value.type)
	{
	case vgraph::param_type::boolean:
		return Value.boolean ? "Yes" : "No";
	case vgraph::param_type::number:
		return format_value(Value.number);
	case vgraph::param_type::choice:
	{
		if (false == Fizvalues.empty())
		{
			for (std::size_t idx = 0; idx < Choices.size() && idx < Fizvalues.size(); ++idx)
			{
				if (Choices[idx] == Value.text)
				{
					return Fizvalues[idx];
				}
			}
		}
		return Value.text;
	}
	default:
	{
		auto text{Value.text};
		std::replace(std::begin(text), std::end(text), ' ', '_');
		return text;
	}
	}
}

// entries of the body sections of a .fiz file which the graph doesn't give. a graph made only of the cab logic, without
// the body section, can this way be tried on an existing vehicle. returns the number of entries taken
int add_fiz_body(fiz_lines &Lines, std::string const &Fizfile)
{
	std::ifstream file{Fizfile, std::ios::binary};
	if (false == file.is_open())
	{
		return 0;
	}
	static char const *const sections[]{"Param.", "Load:", "Dimensions:", "Wheels:", "BuffCoupl.", "Headlights:"};
	int count{0};
	std::string line;
	while (std::getline(file, line))
	{
		if (false == line.empty() && line.back() == '\r')
		{
			line.pop_back();
		}
		line.erase(0, std::min(line.size(), line.find_first_not_of(" \t")));
		if (line.starts_with("//"))
		{
			continue;
		}
		for (auto const *section : sections)
		{
			if (false == line.starts_with(section))
			{
				continue;
			}
			std::istringstream entries{line.substr(std::string_view{section}.size())};
			std::string entry;
			while (entries >> entry)
			{
				if (entry.starts_with("//"))
				{
					break;
				}
				auto const separator{entry.find('=')};
				if (separator == std::string::npos || separator == 0)
				{
					continue;
				}
				auto const key{entry.substr(0, separator)};
				if (false == Lines.has(section, key))
				{
					Lines.add(section, key, entry.substr(separator + 1));
					++count;
				}
			}
			break;
		}
	}
	return count;
}

vgraph::param_value const *node_param(vgraph::graph_template::node_entry const &Node, std::string_view const Key)
{
	auto const index{vgraph::find_param(Node.param_defs, Key)};
	return index >= 0 ? &Node.params[index] : nullptr;
}

double node_number(vgraph::graph_template::node_entry const &Node, std::string_view const Key, double const Default = 0.0)
{
	auto const *value{node_param(Node, Key)};
	return value != nullptr ? value->number : Default;
}

bool node_boolean(vgraph::graph_template::node_entry const &Node, std::string_view const Key)
{
	auto const *value{node_param(Node, Key)};
	return value != nullptr && value->boolean;
}

} // namespace

bool TMoverParameters::LoadGraph(std::string const &File)
{
	// messages of the graphs (lua blocks) go to the log of the simulator
	static bool const logsink{[]() {
		vgraph::set_log_sink([](std::string const &Message) { WriteLog(Message); });
		return true;
	}()};
	(void)logsink;

	WriteLog("LOAD GRAPH FROM " + File);
	std::vector<vgraph::diagnostic> diagnostics;
	auto const graph{vgraph::load_graph(File, diagnostics)};
	for (auto const &entry : diagnostics)
	{
		if (entry.level == vgraph::severity::error)
		{
			ErrorLog("Bad vehicle graph: " + File + ": " + vgraph::to_string(entry));
		}
		else
		{
			WriteLog("Vehicle graph: " + File + ": " + vgraph::to_string(entry));
		}
	}
	if (graph == nullptr)
	{
		ConversionError = 667;
		return false;
	}
	// the .fiz of the same name, if there is one, gives the body entries the graph doesn't have
	auto fizfile{File};
	fizfile = fizfile.substr(0, std::min(fizfile.size(), fizfile.find_last_of('.'))) + ".fiz";
	if (false == AssignGraph(graph, fizfile))
	{
		return false;
	}
	// without length the vehicle can't be placed on the track, and without mass it can't move
	if (Dim.L <= 0.0 || Mass <= 0.0)
	{
		ErrorLog("Bad vehicle graph: " + File + ": the body needs the length and the mass (body section of the graph, or the .fiz file of the same name next to it)");
		ConversionError = 668;
		return false;
	}
	return true;
}

bool TMoverParameters::AssignGraph(std::shared_ptr<vgraph::graph_template const> const &Template, std::string const &Fizfile)
{
	ConversionError = 0;
	fiz_lines lines;

	// body
	for (auto const &entry : Template->body())
	{
		auto const *field{vgraph::find_body_field(entry.first)};
		if (field == nullptr)
		{
			continue;
		}
		if (field->key == "light_color")
		{
			int red{255}, green{255}, blue{255};
			std::sscanf(entry.second.text.c_str(), "%d %d %d", &red, &green, &blue);
			lines.add(field->fiz_section, "LampRed", std::to_string(red));
			lines.add(field->fiz_section, "LampGreen", std::to_string(green));
			lines.add(field->fiz_section, "LampBlue", std::to_string(blue));
			continue;
		}
		lines.add(field->fiz_section, field->fiz_key, fiz_value(entry.second, field->choices, field->fiz_values));
	}

	// equipment parameters of the blocks
	bool nativebrakepipe{true};
	bool nativefeedpipe{true};
	for (auto const &node : Template->nodes())
	{
		auto const &id{node.type->id};
		for (std::size_t idx = 0; idx < node.param_defs.size(); ++idx)
		{
			auto const &definition{node.param_defs[idx]};
			if (definition.fiz_key.empty())
			{
				continue;
			}
			auto const &value{node.params[idx]};
			if (id == "br.distributor" && definition.key == "asb" && value.text == "none")
			{
				continue;
			}
			lines.add(definition.fiz_section, definition.fiz_key, fiz_value(value, definition.choices, {}));
		}
		// entries made of several parameters, or implied by the presence of the block
		if (id == "br.distributor")
		{
			lines.add("Cntrl.", "BrakeSystem", "Pneumatic");
			// the number of positions of the brake controller enables the anti-slip brake setting
			lines.add("Cntrl.", "BCPN", "1");
		}
		else if (id == "br.local_handle")
		{
			lines.add("Cntrl.", "LocalBrake", "PneumaticBrake");
		}
		else if (id == "br.pipe")
		{
			nativebrakepipe = false;
		}
		else if (id == "pn.reservoir")
		{
			nativefeedpipe = false;
		}
		else if (id == "sec.ca")
		{
			std::string system;
			auto const addflag = [&](char const *Flag) { system += (system.empty() ? "" : ",") + std::string{Flag}; };
			if (node_boolean(node, "aware"))
				addflag("Active");
			if (node_boolean(node, "cab_signal"))
				addflag("CabSignal");
			if (node_boolean(node, "separate_ack"))
				addflag("SeparateAcknowledge");
			if (node_boolean(node, "sifa"))
				addflag("Sifa");
			if (false == system.empty())
			{
				lines.add("Security:", "AwareSystem", system);
			}
			auto const minspeed{node_number(node, "min_speed", -1.0)};
			if (minspeed >= 0.0)
			{
				lines.add("Security:", "AwareMinSpeed", format_value(minspeed));
			}
		}
		else if (id == "veh.doors")
		{
			auto const range{node_number(node, "range", 1.0)};
			lines.add("Doors:", "OpenCtrl", "DriverCtrl");
			lines.add("Doors:", "CloseCtrl", "DriverCtrl");
			lines.add("Doors:", "OpenSpeed", format_value(range / std::max(0.01, node_number(node, "open_time", 2.0))));
			lines.add("Doors:", "CloseSpeed", format_value(range / std::max(0.01, node_number(node, "close_time", 3.0))));
			// the graph decides whether the doors have power
			lines.add("Doors:", "DoorVoltage", "0");
		}
		else if (id == "el.pantograph")
		{
			lines.add("Power:", "EnginePower", "CurrentCollector");
		}
		else if (id == "if.heat")
		{
			lines.add("Clima:", "Heating", "Main");
		}
	}

	if (false == Fizfile.empty())
	{
		auto const count{add_fiz_body(lines, Fizfile)};
		if (count > 0)
		{
			WriteLog("graph: " + std::to_string(count) + " body entries from " + Fizfile);
		}
	}

	// sections in the order of the .fiz files
	auto const read = [&](std::string const &Section, auto Reader) {
		auto const line{lines.get(Section)};
		if (false == line.empty())
		{
			WriteLog("graph: " + line);
			Reader(line);
		}
	};
	read("Param.", [this](std::string const &Line) { LoadFIZ_Param(Line); });
	read("Load:", [this](std::string const &Line) { LoadFIZ_Load(Line); });
	read("Dimensions:", [this](std::string const &Line) { LoadFIZ_Dimensions(Line); });
	read("Wheels:", [this](std::string const &Line) { LoadFIZ_Wheels(Line); });
	read("Brake:", [this](std::string const &Line) { LoadFIZ_Brake(Line); });
	read("Doors:", [this](std::string const &Line) { LoadFIZ_Doors(Line); });
	read("BuffCoupl.", [this](std::string const &Line) { LoadFIZ_BuffCoupl(Line, 0); });
	read("Cntrl.", [this](std::string const &Line) { LoadFIZ_Cntrl(Line); });
	read("Headlights:", [this](std::string const &Line) { LoadFIZ_Headlights(Line); });
	read("SpringBrake:", [this](std::string const &Line) { LoadFIZ_SpringBrake(Line); });
	read("Security:", [this](std::string const &Line) { SecuritySystem.load(Line, Vmax); });
	read("Clima:", [this](std::string const &Line) { LoadFIZ_Clima(Line); });
	read("Power:", [this](std::string const &Line) { LoadFIZ_Power(Line); });

	if (EnginePowerSource.SourceType == TPowerSource::CurrentCollector && Power <= 0.0)
	{
		ErrorLog("Bad vehicle graph: " + Template->path() + ": the pantographs need the body power above zero");
	}

	Graph = std::make_shared<vgraph::graph_instance>(Template);
	auto &state{Graph->body()};
	state.mover = this;
	state.native_brake_pipe = nativebrakepipe;
	state.native_feed_pipe = nativefeedpipe;
	return ConversionError == 0;
}

void TMoverParameters::compute_movement_graph_(double const Deltatime)
{
	auto const dt{Deltatime};
	if (LoadStatus > 0)
	{
		LastLoadChangeTime += dt;
	}
	RunInternalCommand();

	TotalCurrent = 0;
	// the pipes take in the flows from the neighbours since the last step, as at the start of UpdatePipePressure and
	// UpdateScndPipePressure
	if (PipePress > 1.0)
	{
		Pipe->Flow(-PipePress * AirLeakRate * dt);
		Pipe->Act();
	}
	PipePress = Pipe->P();
	if (ScndPipePress > 1.0)
	{
		Pipe2->Flow(-ScndPipePress * AirLeakRate * dt);
		Pipe2->Act();
	}
	ScndPipePress = Pipe2->P();

	auto &state{Graph->body()};
	state.velocity = V;
	state.acceleration = AccS;
	state.wheel_speed = nrot;
	state.slipping = SlippingWheels;
	state.cab = CabActive;
	state.distance = DistCounter * 1000.0;
	state.tractive_force = 0.0;
	state.brake_force = 0.0;
	state.current = 0.0;

	Graph->update(dt);

	// the current of the vehicle, for the pantographs of the vehicle model
	Itot = 0.0;
	TotalCurrent = state.current;

	// pipes without blocks still connect the neighbours
	if (state.native_brake_pipe)
	{
		Pipe->Flow(GetDVc(dt));
		Pipe->Act();
		PipePress = Pipe->P();
	}
	if (state.native_feed_pipe)
	{
		Pipe2->Flow(ScndPipeExchange(dt));
		Pipe2->Act();
		ScndPipePress = Pipe2->P();
	}

	update_doors(dt);

	PowerCouplersCheck(dt, coupling::highvoltage);
	PowerCouplersCheck(dt, coupling::power110v);
	PowerCouplersCheck(dt, coupling::power24v);

	Power24vVoltage = std::max(PowerCircuits[0].first, GetTrainsetVoltage(coupling::power24v));
	Power24vIsAvailable = Power24vVoltage > 0;
	Power110vIsAvailable = PowerCircuits[1].first > 0 || GetTrainsetVoltage(coupling::power110v) > 0;
}

void TMoverParameters::graph_forces(double &Tractive, double &Brake) const
{
	auto const &state{Graph->body()};
	Tractive = state.tractive_force;
	Brake = state.brake_force;
}

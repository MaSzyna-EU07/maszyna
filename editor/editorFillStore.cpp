/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorFillStore.hpp"
#include "editor/editorFormat.hpp"

#include "model/AnimModel.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"

#include <algorithm>
#include <cstdlib>
#include <set>
#include <sstream>

namespace fill_store
{

namespace
{

std::string const kMark{"//$u"};

std::vector<area> g_areas;
bool g_loaded{false};

// format of the lines, coordinates in the space of the file:
//   area <id> seed <seed> density <per ha> spacing <m> scale <min> <max> rotation <random|fixed> <deg> ground <0|1>
//     set <kind> <id> objects <count> label <text to the end of the line>
//   outline <count> <x y z>...
//   template <node template to the end of the line>
//   end

std::string trimmed(std::string Text)
{
	auto const first{Text.find_first_not_of(" \t")};
	if (first == std::string::npos)
		return {};
	auto const last{Text.find_last_not_of(" \t\r\n")};
	return Text.substr(first, last - first + 1);
}

void mark(scene::layer_handle const Layer)
{
	if (false == scene::Layers.valid(Layer))
		return;
	auto const &context{scene::Layers.layer(Layer).context_insert()};
	std::vector<std::string> lines;
	for (auto const &entry : g_areas)
	{
		if (entry.layer != Layer)
			continue;
		if (lines.empty())
			lines.emplace_back("# areas filled with models in the editor, the simulation skips these lines");
		auto header{format("area %s seed %u density %.3f spacing %.3f", entry.id.c_str(), static_cast<unsigned>(entry.seed), entry.density, entry.spacing)};
		header += format(" scale %.3f %.3f rotation %s %.3f", entry.scale_min, entry.scale_max, entry.random_rotation ? "random" : "fixed", entry.rotation);
		header += format(" ground %d set %d %d objects %zu label %s", entry.models_as_ground ? 1 : 0, entry.set_kind, entry.set_id, entry.objects, entry.label.c_str());
		lines.push_back(std::move(header));
		auto outline{format("outline %zu", entry.outline.size())};
		for (auto const &point : entry.outline)
		{
			auto const local{context.to_local(point)};
			outline += format(" %.4f %.4f %.4f", local.x, local.y, local.z);
		}
		lines.push_back(std::move(outline));
		for (auto const &entry_template : entry.templates)
			lines.push_back("template " + entry_template);
		lines.emplace_back("end");
	}
	scene::Layers.mark(Layer, kMark, std::move(lines));
}

} // namespace

void reset()
{
	g_areas.clear();
	g_loaded = false;
}

void load()
{
	if (g_loaded || scene::Layers.empty())
		return;
	g_loaded = true;
	g_areas.clear();
	for (auto const layer : scene::Layers.marked_layers(kMark))
	{
		auto const &context{scene::Layers.layer(layer).context_insert()};
		area entry;
		bool open{false};
		for (auto const &text : scene::Layers.marked(layer, kMark))
		{
			std::istringstream line{text};
			std::string keyword;
			line >> keyword;
			if (keyword == "area")
			{
				entry = area{};
				entry.layer = layer;
				std::string seed, density, spacing, scale, rotation, rotationkind, ground, set, objects, label;
				int groundvalue{1};
				unsigned seedvalue{0};
				line >> entry.id >> seed >> seedvalue >> density >> entry.density >> spacing >> entry.spacing >> scale >> entry.scale_min >> entry.scale_max;
				line >> rotation >> rotationkind >> entry.rotation >> ground >> groundvalue >> set >> entry.set_kind >> entry.set_id >> objects >> entry.objects >> label;
				open = (false == line.fail()) && (false == entry.id.empty()) && label == "label";
				if (false == open)
					continue;
				entry.seed = seedvalue;
				entry.random_rotation = (rotationkind != "fixed");
				entry.models_as_ground = (groundvalue != 0);
				std::string rest;
				std::getline(line, rest);
				entry.label = trimmed(rest);
			}
			else if (keyword == "outline" && open)
			{
				std::size_t count{0};
				line >> count;
				for (std::size_t i = 0; i < count && i < 100000; ++i)
				{
					glm::dvec3 point{0.0};
					line >> point.x >> point.y >> point.z;
					if (line.fail())
						break;
					entry.outline.push_back(context.to_world(point));
				}
			}
			else if (keyword == "template" && open)
			{
				std::string rest;
				std::getline(line, rest);
				rest = trimmed(rest);
				if (false == rest.empty())
					entry.templates.push_back(std::move(rest));
			}
			else if (keyword == "end" && open)
			{
				open = false;
				if (entry.outline.size() >= 3)
					g_areas.push_back(std::move(entry));
			}
		}
	}
	if (false == g_areas.empty())
		WriteLog("Editor: " + std::to_string(g_areas.size()) + " filled area(s) read from the scenery", logtype::generic);
}

std::vector<area> const &areas()
{
	load();
	return g_areas;
}

area const *find(std::string const &Id)
{
	load();
	auto const lookup{std::find_if(g_areas.begin(), g_areas.end(), [&](area const &Entry) { return Entry.id == Id; })};
	return lookup != g_areas.end() ? &(*lookup) : nullptr;
}

std::string next_id()
{
	load();
	for (std::size_t number = g_areas.size() + 1;; ++number)
	{
		auto const id{"fill" + std::to_string(number)};
		if (find(id) == nullptr && simulation::Instances.find(model_name(id, 0)) == nullptr)
			return id;
	}
}

void put(area Entry)
{
	load();
	std::set<scene::layer_handle> layers{Entry.layer};
	auto const existing{std::find_if(g_areas.begin(), g_areas.end(), [&](area const &Other) { return Other.id == Entry.id; })};
	if (existing != g_areas.end())
	{
		layers.insert(existing->layer);
		*existing = std::move(Entry);
	}
	else
		g_areas.push_back(std::move(Entry));
	for (auto const layer : layers)
		mark(layer);
}

void forget(std::string const &Id)
{
	load();
	std::set<scene::layer_handle> layers;
	for (auto const &entry : g_areas)
		if (entry.id == Id)
			layers.insert(entry.layer);
	g_areas.erase(std::remove_if(g_areas.begin(), g_areas.end(), [&](area const &Entry) { return Entry.id == Id; }), g_areas.end());
	for (auto const layer : layers)
		mark(layer);
}

std::string model_name(std::string const &Id, std::size_t const Number)
{
	return Id + "_" + std::to_string(Number);
}

bool owns(std::string const &Id, std::string const &Name)
{
	if (Name.size() <= Id.size() + 1 || Name.compare(0, Id.size(), Id) != 0 || Name[Id.size()] != '_')
		return false;
	return std::all_of(Name.begin() + static_cast<std::ptrdiff_t>(Id.size()) + 1, Name.end(), [](char const Character) { return Character >= '0' && Character <= '9'; });
}

} // namespace fill_store

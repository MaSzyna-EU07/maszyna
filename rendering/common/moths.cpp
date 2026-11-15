/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/moths.h"
#include <cmath>
#include <cstdint>
#include <string>
#include "utilities/Globals.h"
#include "model/AnimModel.h"
#include "scene/scenenode.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"

namespace
{

// the model of a swarm, models/ and the extension left out
char const *const Swarmmodel{"animals/cmy"};

// a number 0-1 for the place, the same every time for the same place (whole half metres)
double place_number(glm::dvec3 const &Position)
{
	auto const x{static_cast<std::int64_t>(std::floor(Position.x * 2.0))};
	auto const y{static_cast<std::int64_t>(std::floor(Position.y * 2.0))};
	auto const z{static_cast<std::int64_t>(std::floor(Position.z * 2.0))};
	auto hash{static_cast<std::uint64_t>(x) * 0x9E3779B97F4A7C15ull ^ static_cast<std::uint64_t>(y) * 0xC2B2AE3D27D4EB4Full ^ static_cast<std::uint64_t>(z) * 0x165667B19E3779F9ull};
	hash ^= hash >> 33;
	hash *= 0xff51afd7ed558ccdull;
	hash ^= hash >> 33;
	hash *= 0xc4ceb9fe1a85ec53ull;
	hash ^= hash >> 33;
	return static_cast<double>(hash >> 11) / static_cast<double>(1ull << 53);
}

// moths fly in the warm part of the year
bool moths_fly()
{
	return Global.moths && (Global.Season != "winter:") && (Global.AirTemperature >= Global.moths_temperature);
}

} // namespace

gfx::moths gfx::Moths;

void gfx::moths::update(std::vector<local_lights::light> const &Lights, glm::dvec3 const &Camera, unsigned int const Framestamp)
{
	auto const flying{moths_fly() && (Global.moths_share > 0.f) && (Global.moths_range > 0.f)};
	if (flying)
	{
		auto const range{static_cast<double>(Global.moths_range)};
		for (auto const &light : Lights)
		{
			// the lamps of the scenery models, shining downwards (street and platform lamps; not the signals)
			if ((light.owner == nullptr) || (light.part == nullptr) || (light.direction.y > -0.5f))
			{
				continue;
			}
			auto const key{std::make_pair(light.owner, light.part)};
			auto lookup{m_lamps.find(key)};
			if ((lookup != m_lamps.end()) && (glm::length(lookup->second.position - light.position) > 0.5))
			{
				// another lamp in the place of the one met before (the editor took it away and put a new one)
				if (lookup->second.model != nullptr)
				{
					lookup->second.model->visible(false);
				}
				lookup = m_lamps.end();
				m_lamps.erase(key);
			}
			if (lookup == m_lamps.end())
			{
				if (glm::length(light.position - Camera) > range)
				{
					// decided when the lamp comes closer
					continue;
				}
				swarm entry;
				entry.position = light.position;
				auto chosen{place_number(light.position) < static_cast<double>(Global.moths_share)};
				for (auto const &place : m_places)
				{
					if (glm::length(place - light.position) < 1.0)
					{
						// the other light point of the same lamp has a swarm already
						chosen = false;
						break;
					}
				}
				if (chosen)
				{
					auto const definition{"node " + std::to_string(static_cast<int>(std::ceil(range))) + " 0 none model 0 0 0 0 " + std::string{Swarmmodel} + ".t3d none endmodel"};
					auto *model{simulation::State.create_model(definition, "none", light.position)};
					if (model != nullptr)
					{
						// not a part of the scenery: no layer of it, not saved, not edited
						scene::Layers.count(model->layer(), scene::layer_item::model, -1);
						model->layer(null_handle);
						model->m_dirty = false;
						model->m_generated = true;
						model->visible(false);
						++scene::node_state_version;
						entry.model = model;
						m_places.emplace_back(light.position);
						++m_count;
					}
				}
				lookup = m_lamps.emplace(key, entry).first;
			}
			lookup->second.seen = Framestamp;
		}
	}
	// the swarms of the lamps lit in this frame are shown
	m_shown = 0;
	for (auto &lamp : m_lamps)
	{
		auto &entry{lamp.second};
		if (entry.model == nullptr)
		{
			continue;
		}
		auto const shown{flying && (entry.seen == Framestamp)};
		entry.model->visible(shown);
		m_shown += (shown ? 1 : 0);
	}
}

void gfx::moths::reset()
{
	// the instances go with the scenery
	m_lamps.clear();
	m_places.clear();
	m_count = m_shown = 0;
}

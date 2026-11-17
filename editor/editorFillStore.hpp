/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "scene/scenelayers.h"

// areas filled with models in the editor, kept in the scenery files as "//$u" comment lines: the outline, the models and the
// settings of the fill and the seed of its random numbers, so the area can be picked up again, changed and filled anew. the
// models of an area are named after it, <id>_<number>, which is how they're found again in a scenery loaded later
namespace fill_store
{

struct area
{
	std::string id; // fill<number>, the prefix of the names of its models
	std::string label; // given by the user, shown in the list
	scene::layer_handle layer{null_handle};
	std::vector<glm::dvec3> outline; // world space, in the order it was clicked
	std::vector<std::string> templates; // the node templates the models were drawn from
	int set_kind{0}; // the model set the templates came from (model_set_ref): 0 the list of the tool, 1 a user set, 2 a node bank group
	int set_id{-1};
	float density{200.0f}; // objects per hectare
	float spacing{2.0f}; // minimal distance between the objects, m
	float scale_min{1.0f};
	float scale_max{1.0f};
	bool random_rotation{true};
	float rotation{0.0f}; // yaw of all objects without the random rotation, degrees
	bool models_as_ground{true};
	std::uint32_t seed{0}; // of the random numbers of the fill: the same seed and settings make the same objects
	std::size_t objects{0}; // placed by the last fill
};

void reset();
void load();
std::vector<area> const &areas();
area const *find(std::string const &Id);
// an id no area has and no model is named after yet
std::string next_id();
// the area of the id is replaced, or added; it's kept in the file of its layer
void put(area Entry);
void forget(std::string const &Id);
// the name of the model of the area with the number
std::string model_name(std::string const &Id, std::size_t const Number);
// whether the model name is one of the models of the area
bool owns(std::string const &Id, std::string const &Name);

} // namespace fill_store

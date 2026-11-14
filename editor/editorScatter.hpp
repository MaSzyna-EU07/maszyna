#pragma once

// where the brush of models puts them, and how it turns and scales each: engine-free, so it can be tested on its own

#include <random>
#include <vector>

#include <glm/glm.hpp>

namespace scatter
{

// how the models fill the circle of the brush
enum class fill
{
	random, // anywhere in the circle, no closer to each other than the spacing
	uniform, // on a grid of triangles as wide as the spacing, fixed to the world: strokes over one another don't double it
	feathered, // anywhere, fewer and fewer towards the edge of the circle
};

struct dab_settings
{
	double radius{5.0}; // m
	fill mode{fill::random};
	double spacing{2.0}; // m: the least distance between two models; the step of the grid of the uniform fill
	double density{4.0}; // models a 100 m² for the random and the feathered fills, as much as the spacing lets
};

// the places of the models of one dab of the brush round Centre (x, z): inside the circle, Spacing from each other and from those
// already Taken (by the stroke so far) at least
std::vector<glm::dvec2> dab(glm::dvec2 const &Centre, dab_settings const &Settings, std::vector<glm::dvec2> const &Taken, std::mt19937 &Random); // NOSONAR(cpp:S2245) placement of scenery models, not security

// the distance the cursor goes before the next dab: the circles of two dabs overlap by half
double dab_step(dab_settings const &Settings);

// a setting the same for every model, or drawn for each from a range
enum class vary
{
	fixed,
	random,
};

struct look_settings
{
	vary turn{vary::random};
	glm::vec3 angles{0.0f}; // degrees about x, y, z, for the fixed turn
	glm::vec3 angle_range{0.0f, 180.0f, 0.0f}; // degrees each way about x, y, z, for the random turn
	vary size{vary::fixed};
	glm::vec3 scale{100.0f}; // percent along x, y, z, for the fixed scale
	glm::vec3 scale_range{0.0f}; // percent each way along x, y, z, for the random scale
	bool same_scale{true}; // the random scale one for all the axes (the x range), the model keeps its shape
};

struct look
{
	glm::vec3 angles{0.0f}; // degrees about x, y, z
	glm::vec3 scale{1.0f}; // factors along x, y, z, never under 1%
};

// the turn and the scale of one model
look pick(look_settings const &Settings, std::mt19937 &Random); // NOSONAR(cpp:S2245) placement of scenery models, not security

} // namespace scatter

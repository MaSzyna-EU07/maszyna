/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>

// what a road is made of, as far as laying it out goes. a paved road (asphalt, concrete, setts) keeps to the bends and the angles
// of a built road; a dirt road (a track between fields, through a forest) turns tighter, comes in shorter stretches
// and leaves other roads at sharper angles.
// scenery entry: surfacetype <paved|dirt>, written for a dirt road only
enum class road_surface_type
{
	paved,
	dirt
};

namespace road_surface
{

// the word the scenery uses for the type
inline char const *name(road_surface_type const Type)
{
	return Type == road_surface_type::dirt ? "dirt" : "paved";
}

// the type going by specified word. returns: false if there's no such type, with Type left as it was
inline bool from_name(std::string const &Name, road_surface_type &Type)
{
	if (Name == "paved")
	{
		Type = road_surface_type::paved;
		return true;
	}
	if (Name == "dirt")
	{
		Type = road_surface_type::dirt;
		return true;
	}
	return false;
}

// the scenery entry of the type, followed by a space. a paved road, which all the roads made before the types are, writes nothing,
// so the sceneries are saved as they were
inline std::string entry(road_surface_type const Type)
{
	return Type == road_surface_type::paved ? std::string{} : std::string{"surfacetype "} + name(Type) + ' ';
}

// the material the surface of a road of the type gets unless told otherwise: the default asphalt, or the field road of the game's textures
inline char const *default_material(road_surface_type const Type)
{
	return Type == road_surface_type::dirt ? "droga_polna" : "asphaltdark1";
}

} // namespace road_surface

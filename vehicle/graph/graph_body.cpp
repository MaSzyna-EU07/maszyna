/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_body.h"

namespace vgraph
{

namespace
{

body_field number_field(std::string Key, double Default, std::string Unit, std::string Section, std::string Fizkey)
{
	body_field result;
	result.key = std::move(Key);
	result.type = param_type::number;
	result.unit = std::move(Unit);
	result.default_value = param_value::make_number(Default);
	result.fiz_section = std::move(Section);
	result.fiz_key = std::move(Fizkey);
	return result;
}

body_field text_field(std::string Key, std::string Default, std::string Section, std::string Fizkey)
{
	body_field result;
	result.key = std::move(Key);
	result.type = param_type::text;
	result.default_value = param_value::make_text(std::move(Default));
	result.fiz_section = std::move(Section);
	result.fiz_key = std::move(Fizkey);
	return result;
}

body_field choice_field(std::string Key, std::vector<std::string> Choices, std::vector<std::string> Fizvalues, std::string Default, std::string Section, std::string Fizkey)
{
	body_field result;
	result.key = std::move(Key);
	result.type = param_type::choice;
	result.choices = std::move(Choices);
	result.fiz_values = std::move(Fizvalues);
	result.default_value = param_value::make_choice(std::move(Default));
	result.fiz_section = std::move(Section);
	result.fiz_key = std::move(Fizkey);
	return result;
}

} // namespace

std::vector<body_field> const &body_schema()
{
	// the defaults are the values the simulator uses when the entry is missing from the file
	static std::vector<body_field> const schema{
	    choice_field("category", {"train", "road", "unimog", "ship"}, {"train", "road", "unimog", "ship"}, "train", "Param.", "Category"),
	    number_field("mass", 0.0, "kg", "Param.", "M"),
	    number_field("mass_rotating", 0.0, "kg", "Param.", "Mred"),
	    number_field("vmax", 0.0, "km/h", "Param.", "Vmax"),
	    number_field("power", 0.0, "kW", "Param.", "PWR"),
	    number_field("sand_capacity", 0.0, "kg", "Param.", "SandCap"),
	    number_field("heating_power", 0.0, "kW", "Param.", "HeatingP"),
	    number_field("length", 0.0, "m", "Dimensions:", "L"),
	    number_field("height", 0.0, "m", "Dimensions:", "H"),
	    number_field("width", 0.0, "m", "Dimensions:", "W"),
	    number_field("cx", 0.3, "", "Dimensions:", "Cx"),
	    number_field("wheel_diameter", 0.0, "m", "Wheels:", "D"),
	    number_field("wheel_inertia", 0.0, "kg m2", "Wheels:", "AIM"),
	    number_field("track_gauge", 1.435, "m", "Wheels:", "Tw"),
	    text_field("axle_arrangement", "", "Wheels:", "Axle"),
	    number_field("axle_distance", 0.0, "m", "Wheels:", "Ad"),
	    number_field("bogie_distance", 0.0, "m", "Wheels:", "Bd"),
	    number_field("min_radius", 0.0, "m", "Wheels:", "Rmin"),
	    choice_field("bearing", {"roll", "slide"}, {"Roll", "Slide"}, "slide", "Wheels:", "BearingType"),
	    text_field("coupler_type", "", "BuffCoupl.", "CType"),
	    number_field("coupler_kb", 0.0, "kN/m", "BuffCoupl.", "kB"),
	    number_field("coupler_dmax_b", 0.1, "m", "BuffCoupl.", "DmaxB"),
	    number_field("coupler_fmax_b", 1.0, "kN", "BuffCoupl.", "FmaxB"),
	    number_field("coupler_kc", 0.0, "kN/m", "BuffCoupl.", "kC"),
	    number_field("coupler_dmax_c", 0.1, "m", "BuffCoupl.", "DmaxC"),
	    number_field("coupler_fmax_c", 1.0, "kN", "BuffCoupl.", "FmaxC"),
	    number_field("coupler_beta", 0.0, "", "BuffCoupl.", "beta"),
	    number_field("coupler_allowed_flag", 3.0, "", "BuffCoupl.", "AllowedFlag"),
	    text_field("control_type", "", "BuffCoupl.", "ControlType"),
	    number_field("max_load", 0.0, "", "Load:", "MaxLoad"),
	    text_field("load_unit", "", "Load:", "LoadQ"),
	    text_field("load_accepted", "", "Load:", "LoadAccepted"),
	    number_field("load_speed", 0.0, "1/s", "Load:", "LoadSpeed"),
	    number_field("unload_speed", 0.0, "1/s", "Load:", "UnLoadSpeed"),
	    // three numbers, red green blue 0-255; written as LampRed, LampGreen and LampBlue
	    text_field("light_color", "255 255 255", "Headlights:", "Lamp"),
	    // current collectors; used when the graph has el.pantograph blocks
	    text_field("collector_type", "", "Power:", "PantType"),
	    number_field("collector_min_h", 0.0, "m", "Power:", "MinH"),
	    number_field("collector_max_h", 0.0, "m", "Power:", "MaxH"),
	    number_field("collector_csw", 0.0, "m", "Power:", "CSW"),
	    number_field("collector_max_voltage", 0.0, "V", "Power:", "MaxVoltage"),
	    number_field("collector_min_voltage", 0.0, "V", "Power:", "MinV"),
	    number_field("collector_max_current", 0.0, "A", "Power:", "MaxCurrent"),
	    number_field("collector_layout", 3.0, "", "Power:", "PhysicalLayout"),
	};
	return schema;
}

body_field const *find_body_field(std::string_view const Key)
{
	for (auto const &field : body_schema())
	{
		if (field.key == Key)
		{
			return &field;
		}
	}
	return nullptr;
}

std::vector<profile_info> const &known_profiles()
{
	static std::vector<profile_info> const profiles{
	    {"brake_pipe", 1},
	    {"feed_pipe", 1},
	    {"heating", 1},
	    {"ai", 1},
	};
	return profiles;
}

} // namespace vgraph

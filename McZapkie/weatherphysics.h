/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the weather in the physics of the vehicles (physics.weather.* of eu07.ini, the window Weather physics): what lies on the heads of
// the rails and how it takes the adhesion away, the water and the ice on the brakes, the wind and the density of the air in the drag,
// the ice on the wires breaking the contact of the pantographs. the state of the rails in the open is kept by the weather
// (environment/railsurface.h), each track keeps where the wheels went over it last (TTrack::since_wheels()); the shaders show the
// same state (gl::weather_ubs::surface_state). the functions here only work things out, the tests take them on their own
// the numbers are of the usual measurements of the adhesion of the wheel and the rail (dry about 0.3, wet 0.2, the first drizzle on
// a dirty rail and the frost 0.1, the crushed wet leaves 0.05 and less) and of the brake blocks (the cast iron and the composites
// losing a part of the friction when wet, the blocks and the discs iced by the snow in the running gear), as shares of the dry
// rail and the dry block of the simulation

#pragma once

#include <array>
#include <string_view>

namespace weather_physics
{

struct settings
{
	bool enabled{true}; // physics.weather: off, the rain and the snow change the adhesion as they did before (0.85 and 0.75)
	bool adhesion{true}; // the state of the heads of the rails
	bool brakes{true}; // the water and the ice on the brakes
	bool wind{true}; // the wind in the drag
	bool air_density{true}; // the drag by the density of the air at its temperature
	bool pantographs{true}; // the ice on the wires breaking the contact of the pantographs
	bool wire_wind{true}; // the wind blowing the wires aside: drawn so, and the pantographs meet them so (and lose them)
	bool wire_shadows{true}; // and their shadows blown aside with them (a program of the shadows of their own)

	// the adhesion of a rail, shares of the dry clean one
	float wet{0.75f}; // wet through
	float emulsion{0.6f}; // a little water on the dirt of a dry spell (the first drizzle, the dew), over the wet
	float ice{0.3f}; // glaze or frost on the head
	float snow{0.5f}; // snow on the head, 2 cm or more
	float leaves_dry{0.5f}; // the leaves crushed on the head, dry
	float leaves_wet{0.25f}; // and wet
	float minimum{0.12f}; // the least all of it together gives
	float leaves{1.f}; // how many leaves fall in the autumn, 0..2
	float pass_recovery{600.f}; // s: a rail cleaned by the wheels gets back what the weather lays on it over about this time
	float sand{0.8f}; // the sand gives at least this share of the dry rail

	// the friction of the brakes, shares of the dry one: wet, and iced
	float brake_wet_cast_iron{0.7f};
	float brake_wet_composite{0.85f};
	float brake_wet_disc{0.92f};
	float brake_ice_cast_iron{0.6f};
	float brake_ice_composite{0.45f};
	float brake_ice_disc{0.5f};

	// the drag grows with the angle of the wind across the vehicle: 1 + wind_yaw sin^2 of it
	float wind_yaw{1.5f};

	// the ice on the wires: how often it breaks the contact (a scale of the rate), the length of a break at 1 mm of ice, s, and the
	// share of the breaks a pantograph gets behind another one on the same wire (the first one scrapes the ice off)
	float wire_ice_breaks{1.f};
	float break_time{0.08f};
	float trailing_share{0.3f};

	// the wires blown aside by the wind: the tension of a contact wire, N, and its diameter, m
	float wire_tension{10000.f};
	float wire_diameter{0.012f};
};

// the entries of eu07.ini, the window and the export of the settings take them from here: the key, the label, the member, the range
struct bool_parameter
{
	std::string_view key;
	char const *label;
	bool settings::*member;
};
struct float_parameter
{
	std::string_view key;
	char const *label;
	float settings::*member;
	float minimum;
	float maximum;
	char const *format;
};

inline constexpr std::array<bool_parameter, 8> bool_parameters{{
    {"physics.weather", "Weather in the physics", &settings::enabled},
    {"physics.weather.adhesion", "Heads of the rails", &settings::adhesion},
    {"physics.weather.brakes", "Wet and iced brakes", &settings::brakes},
    {"physics.weather.wind", "Wind in the drag", &settings::wind},
    {"physics.weather.airdensity", "Density of the air in the drag", &settings::air_density},
    {"physics.weather.pantographs", "Ice on the wires and the pantographs", &settings::pantographs},
    {"physics.weather.wirewind", "Wires blown aside by the wind, pantographs torn off", &settings::wire_wind},
    {"physics.weather.wireshadows", "Shadows of the wires blown aside with them", &settings::wire_shadows},
}};

inline constexpr std::array<float_parameter, 22> float_parameters{{
    {"physics.weather.wet", "Adhesion, wet rail", &settings::wet, 0.3f, 1.f, "%.2f"},
    {"physics.weather.emulsion", "Adhesion, first water on a dirty rail", &settings::emulsion, 0.3f, 1.f, "%.2f"},
    {"physics.weather.ice", "Adhesion, ice on the rail", &settings::ice, 0.1f, 1.f, "%.2f"},
    {"physics.weather.snow", "Adhesion, snow on the rail", &settings::snow, 0.1f, 1.f, "%.2f"},
    {"physics.weather.leavesdry", "Adhesion, leaves, dry", &settings::leaves_dry, 0.1f, 1.f, "%.2f"},
    {"physics.weather.leaveswet", "Adhesion, leaves, wet", &settings::leaves_wet, 0.05f, 1.f, "%.2f"},
    {"physics.weather.minimum", "Adhesion, the least", &settings::minimum, 0.05f, 0.5f, "%.2f"},
    {"physics.weather.leaves", "Leaves falling in the autumn", &settings::leaves, 0.f, 2.f, "%.2f"},
    {"physics.weather.passrecovery", "Rail cleaned by the wheels, s", &settings::pass_recovery, 30.f, 7200.f, "%.0f"},
    {"physics.weather.sand", "Sand restores, share of the dry rail", &settings::sand, 0.f, 1.f, "%.2f"},
    {"physics.weather.brakewetcastiron", "Brake friction wet, cast iron", &settings::brake_wet_cast_iron, 0.3f, 1.f, "%.2f"},
    {"physics.weather.brakewetcomposite", "Brake friction wet, composite", &settings::brake_wet_composite, 0.3f, 1.f, "%.2f"},
    {"physics.weather.brakewetdisc", "Brake friction wet, disc", &settings::brake_wet_disc, 0.3f, 1.f, "%.2f"},
    {"physics.weather.brakeicecastiron", "Brake friction iced, cast iron", &settings::brake_ice_cast_iron, 0.2f, 1.f, "%.2f"},
    {"physics.weather.brakeicecomposite", "Brake friction iced, composite", &settings::brake_ice_composite, 0.2f, 1.f, "%.2f"},
    {"physics.weather.brakeicedisc", "Brake friction iced, disc", &settings::brake_ice_disc, 0.2f, 1.f, "%.2f"},
    {"physics.weather.windyaw", "Drag growing with the wind across", &settings::wind_yaw, 0.f, 4.f, "%.2f"},
    {"physics.weather.wireicebreaks", "Breaks of the contact by the ice", &settings::wire_ice_breaks, 0.f, 4.f, "%.2f"},
    {"physics.weather.breaktime", "Break of the contact at 1 mm of ice, s", &settings::break_time, 0.01f, 0.5f, "%.3f"},
    {"physics.weather.trailingshare", "Breaks of a pantograph behind another", &settings::trailing_share, 0.f, 1.f, "%.2f"},
    {"physics.weather.wiretension", "Tension of a contact wire, N", &settings::wire_tension, 3000.f, 30000.f, "%.0f"},
    {"physics.weather.wirediameter", "Diameter of a contact wire, m", &settings::wire_diameter, 0.008f, 0.02f, "%.4f"},
}};

// what lies on the head of a rail in the open, 0..1 each: the water, the dirt of a dry spell (rust, oil, the dust of the brakes),
// the ice (glaze, frost) and the leaves crushed on it
struct railhead
{
	float water{0.f};
	float dirt{0.f};
	float ice{0.f};
	float leaves{0.f};
};

// the adhesion of a rail, a share of the dry clean one: Rail what lies on it in the open (nothing in a tunnel), Snow the snow on the
// head, m, Since the seconds since the wheels last went over the place (they take a part of the dirt, the ice and the leaves off),
// negative when none did. 1 for a dry clean rail
double adhesion_factor(settings const &Settings, railhead const &Rail, double Snow, double Since);

// what the sand gives: a share of the friction of the rail as it is the sanded wheels get, Friction the friction of the rail as it
// is, Dry of the dry clean one, Sandfactor how much the sand adds on a dry rail. the sand is more than a share of the friction it
// falls on: on a slippery rail it gives back at least Settings.sand of the dry one
double sanded(settings const &Settings, double Friction, double Dry, double Sandfactor);

// the material of the brake blocks or pads
enum class brake_material
{
	cast_iron,
	composite,
	disc
};

// the water and the ice on the brakes of a vehicle, 0..1
struct brake_state
{
	double water{0.0};
	double ice{0.0};
};

// the weather the running gear of a vehicle is in: the rain on it, 0..1, the snow around it (falling, blown along the ground, thrown
// up from the track), 0..1, the temperature of the air, degrees C
struct brake_weather
{
	double rain{0.0};
	double snow{0.0};
	double temperature{15.0};
};

// a step of Deltatime of the water and the ice on the brakes: Speed of the vehicle, m/s, Power the brakes turn into heat, W, Mass of
// the vehicle, kg (the heat dries and thaws the blocks: a few seconds of braking dry them, a few more thaw them)
void update_brakes(brake_state &State, brake_weather const &Weather, double Speed, double Power, double Mass, double Deltatime);

// the friction of the brakes, a share of the dry one
double brake_factor(settings const &Settings, brake_material Material, brake_state const &State);

// the density of the air at a temperature, a share of the one at 15 degrees C
double air_density(double Temperature);

// the drag along the track, N, towards the front of the vehicle: Drag its coefficient of V^2 in calm air (Cx times the area), V the
// velocity of the vehicle towards its front, m/s, Along the wind towards its front and Across it, m/s, Density a share of the air at
// 15 degrees, Yaw settings::wind_yaw. without a wind and at 15 degrees it is -Drag |V| V, the drag of the calm air
double aerodynamic_force(double Drag, double V, double Along, double Across, double Density, double Yaw);

// how far the wind blows the middle of a span of the catenary aside, m, towards where it blows: a wire of a span Length long (m,
// horizontally) pulled taut by settings::wire_tension, in a wind Across the span (m/s, signed), thickened by Wireice (radial, m), in
// air of Density (a share of the one at 15 degrees). a span of 65 m goes some 20 cm aside in 20 m/s and 45 cm in 30 m/s; with the
// stagger of the wire and half the width of the strip of a pantograph (0.5..0.6 m) a gale takes the wire off the strip
double wire_sway(settings const &Settings, double Length, double Across, double Wireice, double Density);
// the share of that a point of the span gets, Parameter 0..1 along it: none at the supports, all of it in the middle
inline double sway_profile(double const Parameter)
{
	auto const u{Parameter < 0.0 ? 0.0 : (Parameter > 1.0 ? 1.0 : Parameter)};
	return 4.0 * u * (1.0 - u);
}

// how many times a second the ice on the wire breaks the contact of a pantograph: Wireice the radial thickness of the ice, m, Plaster
// the snow stuck to the vehicle, 0..1 (on the pantograph, it presses it less), Speed m/s, Leading the first pantograph on the wire
double break_rate(settings const &Settings, double Wireice, double Plaster, double Speed, bool Leading);
// how long a break lasts, s: Random 0..1
double break_length(settings const &Settings, double Wireice, double Random);

} // namespace weather_physics

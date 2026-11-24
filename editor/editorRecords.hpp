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
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "editor/editorAlignmentStore.hpp"
#include "editor/editorKilometrage.hpp"
#include "editor/editorProfile.hpp"
#include "editor/editorSwitchStore.hpp"
#include "scene/scenelayers.h"

// the lines the track editor keeps in the scenery files after its //$ marks, which the simulation skips: what each kind says, read
// from the text of the lines (the mark taken off, as scene::node_layers::marked() gives them) and written into it, in one place. The
// coordinates in the lines are those of the file, the records have them in the world: Context is the placement in effect where
// the lines are (basic_layer::context_insert()). What the records stand for in the scene (the tracks, the objects bound to them, the
// switches of a station) is found by those who keep them; the records here are what the lines say
namespace editor_records
{

// //$h, the horizontal alignments designed (alignment_store): the records closed by their end lines, as read, their designs not yet
// made into lines
std::vector<alignment_store::record> read_alignments(std::vector<std::string> const &Lines, scene::layer_context const &Context, scene::layer_handle const Layer);
// the records with their designs made (record::result), the heading comment first
std::vector<std::string> alignment_lines(std::vector<alignment_store::record> const &Records, scene::layer_context const &Context);

// //$r, the switches placed (switch_store)
std::vector<switch_store::record> read_switches(std::vector<std::string> const &Lines, scene::layer_context const &Context, scene::layer_handle const Layer);
std::vector<std::string> switch_lines(std::vector<switch_store::record> const &Records, scene::layer_context const &Context);

// //$k, the kilometrage of the lines (kilometrage)
void read_kilometrage(std::vector<std::string> const &Lines, scene::layer_context const &Context, scene::layer_handle const Layer, std::vector<kilometrage::mark> &Marks, std::vector<kilometrage::via> &Vias);
std::vector<std::string> kilometrage_lines(std::vector<kilometrage::mark> const &Marks, std::vector<kilometrage::via> const &Vias, scene::layer_context const &Context);

// //$p, the grade lines designed in the profile tool
struct grade_profile
{
	scene::layer_handle layer{null_handle};
	double length{0.0};
	glm::dvec3 start{0.0}, end{0.0}, middle{0.0}; // of the route, world space
	double origin{0.0};
	profile::line line;
	double departure{-1.0}; // of the track from the design when it was kept; -1 as kept by an older editor
};
std::vector<grade_profile> read_profiles(std::vector<std::string> const &Lines, scene::layer_context const &Context, scene::layer_handle const Layer);
std::vector<std::string> profile_lines(std::vector<grade_profile> const &Profiles, scene::layer_context const &Context);

// //$t, the tracks whose grade line was fitted, by name, with their shape then
struct fitted_track
{
	std::string name;
	std::uint64_t shape{0};
	bool hand{false}; // edited with the profile tool
};
std::vector<fitted_track> read_fitted(std::vector<std::string> const &Lines);
std::vector<std::string> fitted_lines(std::vector<fitted_track> const &Tracks);

// //$w, the tracks sharing a grade line, by their key (editor_mode::grade_line_key), each with the number of it
struct grade_line
{
	std::string key;
	int number{0};
	bool hand{false};
};
std::vector<grade_line> read_grade_lines(std::vector<std::string> const &Lines);
std::vector<std::string> grade_line_lines(std::vector<grade_line> const &Lines);

// //$b, an object bound to the paths: its kind (infra::name()), category, location, file or name, whether it turns with the track, and
// the paths it's bound to, named by their ends
struct binding
{
	std::string kind;
	int category{0};
	glm::dvec3 location{0.0};
	std::string key{"-"}; // the file of a model or of an include, the name of a memory cell or an event launcher; - for none
	bool turns{false};
	struct anchor
	{
		int path{0};
		glm::dvec3 start{0.0};
		glm::dvec3 end{0.0};
	};
	std::vector<anchor> anchors; // 1 to 4
};
// lines starting with # are comments, none for them; none for a line it can't read either
std::optional<binding> read_binding(std::string const &Line, scene::layer_context const &Context);
std::string binding_line(binding const &Binding, scene::layer_context const &Context);
// the comment the lines of the bindings start with
std::string binding_heading();

// //$s, the stations described in the signalling tool (srk_tool)
enum class block_kind
{
	semi, // semi-automatic: a warning disc before the entry signal
	three, // the automatic one of three aspects: the block sections no shorter than the braking distance
	four // of four aspects, the warning two signals ahead: the sections no shorter than half of it
};
struct station
{
	std::string anchor; // the name of a switch of the station
	std::string code; // the abbreviation of the station, as MMz
	std::string name; // utf-8
	std::string line; // the number of the railway line
	std::string km; // of the axis of the station along the line, empty while unknown
	bool reversed{false}; // the km rise to the left of the schematic, the switches numbered from there
	int braking{0}; // m, the braking distance of the line set by hand, 0: after its speed
	std::map<std::string, block_kind> blocks; // of the lines by their entry signals, as set by hand
	std::map<std::string, bool> ways; // the automatic block of the lines by their entry signals both ways, as set by hand
	std::vector<glm::dvec3> joints_added; // the joints of the track circuits put by hand, on plain tracks
	std::vector<glm::dvec3> joints_removed; // where the planned joints were taken away by hand
};
// none for a line which isn't one of a station, or names no switch
std::optional<station> read_station(std::string const &Line);
std::vector<std::string> station_lines(std::vector<station> const &Stations);

// //$c, where the camera was at the last save
struct camera
{
	glm::dvec3 position{0.0};
	glm::vec3 angle{0.f};
};
std::optional<camera> read_camera(std::vector<std::string> const &Lines);
std::string camera_line(camera const &Camera);

// //$o, whether the orthophoto is on and, with Origin, where the scenery lies on the map, PUWG 1992 northing and easting; the settings
// the older sceneries carry besides are passed over. returns: false if the line says none of it
// Aligned: the origin was aligned by hand in the editor, which //$g of the scenery file doesn't override; a release before it passes the
// word over
std::string orthophoto_line(bool const Enabled, bool const Origin, double const North, double const East, bool const Aligned = false);
bool read_orthophoto(std::string const &Line, bool &Enabled, double &North, double &East, bool *Aligned = nullptr);

// //$z, the area the editor downloads the orthophoto and the terrain model in, PUWG 1992 easting northing: none, below 3 points
std::vector<std::string> area_lines(std::vector<glm::dvec2> const &Polygon);
std::vector<glm::dvec2> read_area(std::vector<std::string> const &Lines);

} // namespace editor_records

/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodebuildtools.h"
#include "editor/editorSettings.hpp"
#include "editor/editorGeometry.hpp"
#include "application/application.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"

#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "scene/scenelayers.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "world/Track.h"
#include "world/TrackRails.h"
#include "world/MemCell.h"
#include "world/Event.h"
#include "vehicle/DynObj.h"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "rendering/renderer.h"
#include "world/Sweep.h"
#include "input/keyboardinput.h"
#include "launcher/textures_scanner.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <array>
#include <bit>
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "scene/sceneterrain.h"
#include "editor/editorSpeed.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include "application/editormodeselftest.h"

#include <ostream>
#include <ranges>

namespace
{

// each item kept in the square of the plan given and in the eight around it, the squares by their columns
template <typename Cells, typename Visit>
void each_in_cells_around(Cells const &Of, std::pair<int, int> const &Cell, Visit &&Seen)
{
	for (int dx = -1; dx <= 1; ++dx)
		for (int dz = -1; dz <= 1; ++dz)
			if (auto const other{Of.find({Cell.first + dx, Cell.second + dz})}; other != Of.end())
				std::ranges::for_each(other->second, [&Seen](auto const &Item) { Seen(Item); });
}

// the track nearest the point in plan, nearer than the reach, but not a turntable: none where none is
TTrack *nearest_not_table(glm::dvec3 const &Point, double const Reach)
{
	TTrack *closest{nullptr};
	auto nearest{Reach};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty() || track->eType == tt_Table)
			continue;
		if (auto const distance{geometry::plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, Point)), Point)}; distance < nearest)
		{
			nearest = distance;
			closest = track;
		}
	}
	return closest;
}

// how many ends of the paths of the track lie within 10 cm of the point in plan
int ends_at(TTrack const &Track, glm::dvec3 const &Point)
{
	int ends{0};
	for (auto const &path : Track.m_paths)
		for (auto const index : {segment_data::point::start, segment_data::point::end})
			ends += glm::length(glm::dvec2{path.points[index].x - Point.x, path.points[index].z - Point.z}) <= 0.1 ? 1 : 0;
	return ends;
}

// the route found from the track to the other, against the start and the middle of a design: none where none is found
std::string route_text(TTrack *From, TTrack *To, glm::dvec3 const &Start, glm::dvec3 const &Middle)
{
	editor_track::route route;
	if (std::string error; false == editor_track::find_route(From, To, route, error))
		return format(" [%s > %s: none]", From->name().c_str(), To->name().c_str());
	auto const samples{editor_track::sample_route(route, 1.0)};
	auto const middle{editor_track::sampled_position(samples, route.length * 0.5)};
	return format(" [%s > %s: %zu spans %.1f m, start %.2f, middle %.2f]", From->name().c_str(), To->name().c_str(), route.spans.size(), route.length, glm::length(glm::dvec2{samples.front().position.x - Start.x, samples.front().position.z - Start.z}),
	              glm::length(glm::dvec2{middle.x - Middle.x, middle.z - Middle.z}));
}

// the routes from the track to each supported track with an end at the end of a design, once for each end of each of the two there
std::string routes_from(TTrack *Track, glm::dvec3 const &Start, glm::dvec3 const &End, glm::dvec3 const &Middle)
{
	std::string routes;
	for (auto *other : simulation::Paths.sequence())
		if (other != nullptr && other != Track && false == other->m_editorremoved && editor_track::is_supported(*other))
			for (int ends = ends_at(*other, End); ends > 0; --ends)
				routes += route_text(Track, other, Start, Middle);
	return routes;
}

// the routes between the supported tracks with an end at the start of a design and the ones with an end at its end
std::string design_routes(glm::dvec3 const &Start, glm::dvec3 const &End, glm::dvec3 const &Middle)
{
	std::string routes;
	for (auto *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved && editor_track::is_supported(*track))
			for (int ends = ends_at(*track, Start); ends > 0; --ends)
				routes += routes_from(track, Start, End, Middle);
	return routes;
}

// the zones of the switches within 5 m of the chainage
template <typename Zones>
std::string switch_zones(Zones const &Of, double const Chainage)
{
	std::string zone;
	for (auto const &one : Of)
		if (Chainage >= one.from - 5.0 && Chainage <= one.to + 5.0)
			zone += format(" [%.1f..%.1f %s]", one.from, one.to, one.name.c_str());
	return zone;
}

// seven points along the path, each in plan and in height
void stream_points(std::ostream &Stream, segment_data const &Path)
{
	geometry::bezier const curve{Path};
	for (int i = 0; i <= 6; ++i)
	{
		auto const point{curve.point(i / 6.0)};
		Stream << ' ' << point.x << ' ' << point.z << ' ' << point.y;
	}
}

// what a WMS tells of itself, as the layers window reads it before one is added
editor_selftest_result selftest_wmsdescribe(std::istringstream &words)
{
	std::string url;
	words >> url;
	auto const info{editor_orthophoto::describe(url)};
	WriteLog(format("SELFTEST wmsdescribe: \"%s\", %zu layers, EPSG:2180 %d%s", info.title.c_str(), info.layers.size(), info.puwg ? 1 : 0, info.error.empty() ? "" : (", error " + info.error).c_str()));
	WriteLog("SELFTEST wmsdescribe terms: " + info.terms);
	for (std::size_t i = 0; i < std::min<std::size_t>(info.layers.size(), 5); ++i)
		WriteLog("SELFTEST wmsdescribe layer " + info.layers[i].first + " \"" + info.layers[i].second + "\"");
	return editor_selftest_result::handled;
}

// a place as typed into the terrain window: the PUWG 1992 and the degrees it's understood as
editor_selftest_result selftest_nmtplace(std::istringstream &words)
{
	std::string text;
	std::getline(words, text);
	glm::dvec2 point;
	if (false == editor_geo::parse_place(text, point))
	{
		WriteLog("SELFTEST nmtplace" + text + ": not understood");
		return editor_selftest_result::handled;
	}
	auto const degrees{editor_geo::to_degrees(point)};
	WriteLog(format("SELFTEST nmtplace%s: X %.1f Y %.1f, %.6f N %.6f E", text.c_str(), point.y, point.x, degrees.x, degrees.y));
	return editor_selftest_result::handled;
}

// how many plain tracks are short, the ballast mapped on each from its start: under 1, 3 and 6 m
editor_selftest_result selftest_shorttracks(std::istringstream &words)
{
	std::string label;
	words >> label;
	std::array<int, 3> counts{};
	int all{0};
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty() || track->eType != tt_Normal)
			continue;
		++all;
		auto const length{geometry::bezier{track->m_paths.front()}.plan_length()};
		for (auto const [i, limit] : {std::pair{0, 1.0}, std::pair{1, 3.0}, std::pair{2, 6.0}})
			counts[i] += length < limit ? 1 : 0;
	}
	WriteLog(format("SELFTEST shorttracks %s: %d plain tracks, under 1 m %d, under 3 m %d, under 6 m %d", label.c_str(), all, counts[0], counts[1], counts[2]));
	return editor_selftest_result::handled;
}

// a place where the ground under a track is off its formation: by how much, where, on which track
struct ground_finding
{
	double off;
	glm::dvec3 at;
	std::string track;
};

// the places found, the farthest off first, Top of them at most
void log_ground_findings(std::vector<ground_finding> &Findings, char const *Kind, int const Top)
{
	std::ranges::sort(Findings, [](ground_finding const &A, ground_finding const &B) { return std::abs(A.off) > std::abs(B.off); });
	for (std::size_t i = 0; i < std::min<std::size_t>(Findings.size(), static_cast<std::size_t>(Top)); ++i)
		WriteLog(format("SELFTEST nmtcheck %s %+.2f m at %.1f %.1f %.1f %s", Kind, Findings[i].off, Findings[i].at.x, Findings[i].at.y, Findings[i].at.z, Findings[i].track.c_str()));
}

// the model under the rails at the first sample within half a metre of the chainage, where it has it there
template <typename Samples>
std::string model_at(Samples const &Of, double const Chainage, double const Rails, editor_nmt const &Model, double const Offset)
{
	auto const sample{std::ranges::find_if(Of, [&](auto const &Sample) { return std::abs(Sample.chainage - Chainage) < 0.51; })};
	if (double model; sample != Of.end() && Model.height(sample->position.x, sample->position.z, model))
		return format(" (model %.2f)", model + Offset + Rails);
	return std::string{};
}

// how far from the rim the track keeps within a centimetre of the height
template <typename Height_at>
double level_for(Height_at const &Height_of, double const Length, double const Height)
{
	double level{0.0};
	for (int k = 0; k * 0.5 <= Length; ++k)
		if (std::abs(Height_of(k * 0.5) - Height) <= 0.01)
			level = k * 0.5;
		else
			break;
	return level;
}

// the heights of the chunk in memory added to the hash
void hash_heights(std::uint64_t &Hash, editor_terrain const &Chunk)
{
	for (auto const height : Chunk.heights())
	{
		auto const bits{std::bit_cast<std::uint32_t>(height)};
		Hash = (Hash ^ bits) * 1099511628211ULL;
	}
}

// the weights of the layers of the chunk at a sample of the paint
template <typename Data>
std::string weights_of(Data const &Of, int const Ix, int const Iz)
{
	std::string text;
	for (std::size_t slot = 0; slot < Of.layer_count(); ++slot)
		text += format(" %d", Of.weight(slot, std::clamp(Ix, 0, Of.paint), std::clamp(Iz, 0, Of.paint)));
	return text;
}

// the paint and the heights of the chunk at the point: its spacing, its base and touch-up, the weights at the paint sample nearest
template <typename Data>
std::string chunk_grid(Data const &Of, double const X, double const Z)
{
	double base{0.0};
	double adjust{0.0};
	if (false == Of.base.empty())
		base = Of.height_in(Of.base, X, Z);
	if (false == Of.adjust.empty())
		adjust = Of.height_in(Of.adjust, X, Z);
	auto grid{format(", spacing %.1f, paint %d, base %s %.3f, touch-up %.3f", Of.spacing(), Of.paint, Of.base.empty() ? "none" : "", base, adjust)};
	if (Of.paint > 0)
	{
		// weights at the paint sample nearest the point
		auto const corner{Of.corner()};
		auto const ix{static_cast<int>(std::lround((X - corner.x) / heightmap::chunk_size * Of.paint))};
		auto const iz{static_cast<int>(std::lround((Z - corner.y) / heightmap::chunk_size * Of.paint))};
		grid += ", weights";
		grid += weights_of(Of, ix, iz);
	}
	return grid;
}

// a place along a track for the report of the fit: where, which way the track runs there, of which track
struct fit_place
{
	glm::dvec3 position;
	glm::dvec2 along;
	TTrack const *track;
};

double constexpr kFitTimbers{2.65};
double constexpr kFitStep{5.0};

std::pair<int, int> fit_cell_of(glm::dvec3 const &Point)
{
	return std::pair<int, int>{static_cast<int>(std::floor(Point.x / kFitTimbers)), static_cast<int>(std::floor(Point.z / kFitTimbers))};
}

// how far the rails of a track are off the model: the most, where, the sum and of how many places
struct fit_tally
{
	double worst{0.0};
	glm::dvec3 worst_at{0.0};
	double sum{0.0};
	int counted{0};
};

// the places of the tracks for the report of the fit, all of them and by the squares of the timbers they lie in
struct fit_places
{
	std::map<std::pair<int, int>, std::vector<fit_place>> cells;
	std::vector<fit_place> all;
};

// the places of the path kept, and the rails at them against the model taken into the tally
void tally_path(TTrack const *Track, segment_data const &Path, editor_nmt const &Model, double const Offset, fit_places &Places, fit_tally &Tally)
{
	geometry::bezier const curve{Path};
	auto const places_on{std::max(1, static_cast<int>(geometry::chord_length(Path) / kFitStep))};
	for (int i = 0; i <= places_on; ++i)
	{
		auto const share{static_cast<double>(i) / places_on};
		auto const point{curve.point(share)};
		auto const tangent{geometry::plan_of(curve.first(share))};
		fit_place const here{point, glm::length(tangent) > 1e-9 ? glm::normalize(tangent) : glm::dvec2{1.0, 0.0}, Track};
		Places.all.push_back(here);
		Places.cells[fit_cell_of(point)].push_back(here);
		double height;
		if (false == Model.height(point.x, point.z, height))
			continue;
		auto const off{point.y - (height + Offset + Track->rail_height())};
		++Tally.counted;
		Tally.sum += off;
		if (std::abs(off) > std::abs(Tally.worst))
		{
			Tally.worst = off;
			Tally.worst_at = point;
		}
	}
}

// of a switch how far its second path is off the plane of the first, to the report of the fit
void stream_switch_plane(std::ostream &Stream, TTrack const &Track, int const Line)
{
	geometry::bezier const main{Track.m_paths[0]};
	geometry::bezier const branch{Track.m_paths[1]};
	auto const direction{main.first(0.0)};
	auto const grade{direction.y / std::max(1e-6, std::hypot(direction.x, direction.z))};
	auto const end{branch.point(1.0)};
	auto const start{branch.point(0.0)};
	// against the first path as far along it, in its plane or on the vertical curve it lies on; past its end against the grade it ends with
	auto const chord{geometry::plan_of(main.point(1.0) - main.point(0.0))};
	auto const along{glm::dot(geometry::plan_of(end - main.point(0.0)), chord) / std::max(1e-9, glm::dot(chord, chord))};
	auto const last{main.first(1.0)};
	auto const on_main{main.point(std::clamp(along, 0.0, 1.0)).y + std::max(0.0, along - 1.0) * glm::length(chord) * last.y / std::max(1e-6, std::hypot(last.x, last.z))};
	Stream << "P " << Track.name() << ' ' << Line << ' ' << end.y - on_main << ' ' << grade * 1000.0 << ' ' << start.x << ' ' << start.z << '\n';
}

// the most the rails of two tracks on common timbers differ in height: by how much, where, how far apart
struct pair_worst
{
	double difference{0.0};
	glm::dvec3 at{0.0};
	double apart{0.0};
};

// the place with the places of the other tracks on common timbers with it, a switch among the two
void pairs_near(fit_place const &Here, fit_places const &Places, std::map<std::pair<std::string, std::string>, pair_worst> &Pairs)
{
	each_in_cells_around(Places.cells, fit_cell_of(Here.position), [&Here, &Pairs](fit_place const &Other) {
		if (Other.track == Here.track || (Here.track->eType != tt_Switch && Other.track->eType != tt_Switch) || Here.track->name() > Other.track->name())
			return;
		glm::dvec2 const offset{Other.position.x - Here.position.x, Other.position.z - Here.position.z};
		if (glm::length(offset) >= kFitTimbers || std::abs(glm::dot(offset, Here.along)) > 0.6 * kFitStep || std::abs(Other.position.y - Here.position.y) > 2.0)
			return;
		auto &worst{Pairs[{Here.track->name(), Other.track->name()}]};
		if (auto const difference{std::abs(Other.position.y - Here.position.y)}; difference >= worst.difference)
			worst = {difference, Here.position, glm::length(offset)};
	});
}

// a spot along a track beside the route of the profile tool: where, which way the track runs there, of which track
struct beside_spot
{
	glm::dvec3 position;
	glm::dvec2 along;
	TTrack const *track;
};

// the spots of the paths of the track, a metre or so apart
void spots_of(TTrack const *Track, std::vector<beside_spot> &Spots)
{
	for (auto const &path : Track->m_paths)
	{
		geometry::bezier const curve{path};
		auto const count{std::max(1, static_cast<int>(geometry::plan_distance(path.points[segment_data::point::start], path.points[segment_data::point::end]) / 1.0))};
		for (int i = 0; i <= count; ++i)
		{
			auto const tangent{geometry::plan_of(curve.first(static_cast<double>(i) / count))};
			Spots.emplace_back(curve.point(static_cast<double>(i) / count), glm::length(tangent) > 1e-9 ? glm::normalize(tangent) : glm::dvec2{1.0, 0.0}, Track);
		}
	}
}

// the spots of the other tracks beside the point, running its way: the nearest on each side within 6 m
void sides_of(glm::dvec3 const &Here, glm::dvec2 const &Along, std::vector<beside_spot> const &Spots, std::array<beside_spot const *, 2> &Sides, std::array<double, 2> &Across)
{
	for (auto const &other : Spots)
	{
		glm::dvec2 const offset{other.position.x - Here.x, other.position.z - Here.z};
		if (std::abs(glm::dot(offset, Along)) > 0.6 || std::abs(glm::dot(Along, other.along)) < 0.985)
			continue;
		auto const side{Along.x * offset.y - Along.y * offset.x};
		if (auto const k{side > 0.0 ? 0 : 1}; std::abs(side) < Across[k])
		{
			Across[k] = std::abs(side);
			Sides[k] = &other;
		}
	}
}

// the place of the samples where the grade line is farthest off the track, and how far
template <typename Samples, typename Line>
std::pair<double, std::size_t> farthest_off(Samples const &Of, Line const &Grade)
{
	double worst{0.0};
	std::size_t at{0};
	for (std::size_t k = 0; k < Of.size(); ++k)
		if (auto const off{std::abs(profile::elevation(Grade, Of[k].chainage) - Of[k].position.y)}; off > worst)
		{
			worst = off;
			at = k;
		}
	return {worst, at};
}

} // namespace

// the terrain commands of the self-test, by the group: the imagery, the terrain model, the tracks on it, the terrain of the editor
// changed and told, and the reports to the files
editor_mode::selftest_result editor_mode::selftest_terrain(std::string const &command, std::istringstream &words)
{
	if (auto const imagery{selftest_terrain_imagery(command, words)}; imagery != selftest_result::unhandled)
		return imagery;
	if (auto const model{selftest_terrain_model(command, words)}; model != selftest_result::unhandled)
		return model;
	if (auto const fit{selftest_terrain_fit(command, words)}; fit != selftest_result::unhandled)
		return fit;
	if (auto const edited{selftest_terrain_edit(command, words)}; edited != selftest_result::unhandled)
		return edited;
	if (auto const told{selftest_terrain_told(command, words)}; told != selftest_result::unhandled)
		return told;
	return selftest_terrain_reports(command, words);
}

editor_mode::selftest_result editor_mode::selftest_terrain_imagery(std::string_view const command, std::istringstream &words)
{
	if (command == "orthophoto")
		return selftest_orthophoto(words);
	if (command == "orthoarea")
		return selftest_orthoarea(words);
	if (command == "orthoareapause")
	{
		int paused{1};
		words >> paused;
		m_area_paused = paused != 0;
		m_orthophoto.area_paused(m_area_paused);
		m_nmt.prefetch_paused(m_area_paused);
		WriteLog(format("SELFTEST orthoareapause %d", paused));
		return selftest_result::handled;
	}
	if (command == "orthohires")
	{
		int hires{1};
		words >> hires;
		auto config{m_orthophoto.settings()};
		config.hires = hires != 0;
		m_orthophoto.settings(config);
		WriteLog(format("SELFTEST orthohires %d", hires));
		return selftest_result::handled;
	}
	if (command == "orthosettings")
		return selftest_orthosettings(words);
	if (command == "nmtsettings")
		return selftest_nmtsettings(words);
	if (command == "wmsdescribe")
		return selftest_wmsdescribe(words);
	if (command == "wms")
	{
		// a WMS layer of the user's on top of the others, as the layers window adds it: wms <address> <layers>
		editorSettings::wms_layer entry;
		words >> entry.url >> entry.layers;
		entry.title = entry.layers;
		EditorSettings.wms_layers().insert(EditorSettings.wms_layers().begin(), entry);
		WriteLog("SELFTEST wms " + entry.url + " " + entry.layers);
		return selftest_result::handled;
	}
	if (command == "wmsstats")
		return selftest_wmsstats();
	if (command == "orthostats")
		return selftest_orthostats();
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_orthophoto(std::istringstream &words)
{
	auto config{m_orthophoto.settings()};
	if (words.str().find(" off") != std::string::npos)
	{
		// as the user turns it off: the terrain model leaves it so
		choose_orthophoto(false);
		WriteLog("SELFTEST orthophoto off");
		return selftest_result::handled;
	}
	int fit{1};
	int inscene{0};
	float height{0.0f};
	words >> config.north >> config.east >> config.radius >> fit >> inscene;
	config.drape = fit != 0;
	config.in_scene = inscene != 0;
	if (words >> height)
		config.height = height;
	m_orthophoto.settings(config);
	m_orthophoto.enabled(true);
	WriteLog(format("SELFTEST orthophoto at %.0f %.0f, radius %d, %s%s", config.north, config.east, config.radius, config.drape ? "fitted" : "flat", config.in_scene ? ", in the scene" : ""));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_orthoarea(std::istringstream &words)
{
	// the download area through the points of the world (x, z), as drawn on the map; clear: none
	std::vector<glm::dvec2> polygon;
	if (words.str().find(" clear") == std::string::npos)
		for (glm::dvec2 point{0.0}; words >> point.x >> point.y;)
			polygon.push_back(m_orthophoto.to_puwg(point));
	m_orthophoto.area(polygon);
	WriteLog(format("SELFTEST orthoarea: %zu points, %d tiles", m_orthophoto.area().size(), m_orthophoto.stats().area_tiles));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_orthosettings(std::istringstream &words)
{
	// as the panel of the orthophoto changes them, kept with the editor
	auto config{m_orthophoto.settings()};
	std::string key;
	double value{0.0};
	auto changed{false};
	while (words >> key >> value)
	{
		changed = true;
		if (key == "radius")
			config.radius = std::clamp(static_cast<int>(value), 0, editor_orthophoto::max_radius);
		else if (key == "opacity")
			config.opacity = std::clamp(static_cast<float>(value), 0.0f, 1.0f);
		else if (key == "lift")
			config.lift = static_cast<float>(value);
		else if (key == "height")
			config.height = static_cast<float>(value);
		else if (key == "year")
			config.year = static_cast<int>(value);
		else if (key == "hires")
			config.hires = value != 0.0;
		else if (key == "drape")
			config.drape = value != 0.0;
		else if (key == "scene")
			config.in_scene = value != 0.0;
		else if (key == "on")
			choose_orthophoto(value != 0.0);
	}
	if (changed)
	{
		m_orthophoto.settings(config);
		save_orthophoto_settings();
	}
	config = m_orthophoto.settings();
	WriteLog(format("SELFTEST orthosettings on %d radius %d opacity %.2f lift %.2f height %.2f year %d hires %d drape %d scene %d origin %.2f %.2f", m_orthophoto.enabled() ? 1 : 0, config.radius, config.opacity, config.lift,
	                config.height, config.year, config.hires ? 1 : 0, config.drape ? 1 : 0, config.in_scene ? 1 : 0, config.north, config.east));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_nmtsettings(std::istringstream &words)
{
	// as the terrain panel changes them, kept with the editor the next frame (keep_nmt_settings); the terrain while laying at
	// once, as its chip keeps it
	auto &state{m_nmt_state};
	std::string key;
	double value{0.0};
	while (words >> key >> value)
	{
		if (key == "band")
			state.band = std::clamp(value, 50.0, 3000.0);
		else if (key == "around")
			m_terrain_add_count = std::clamp(static_cast<int>(value), 1, 8);
		else if (key == "shape")
			state.shape = value != 0.0;
		else if (key == "common")
			state.common = value != 0.0;
		else if (key == "follow")
			state.follow = value != 0.0;
		else if (key == "offset")
			state.offset = value;
		else if (key == "fitted")
			state.fitted = value != 0.0;
		else if (key == "grow")
		{
			state.grow = value != 0.0;
			EditorSettings.terrain_while_laying(state.grow);
			EditorSettings.save();
		}
	}
	WriteLog(format("SELFTEST nmtsettings band %.0f around %d shape %d follow %d grow %d offset %+.2f fitted %d", state.band, m_terrain_add_count, state.shape ? 1 : 0, state.follow ? 1 : 0, state.grow ? 1 : 0, state.offset,
	                state.fitted ? 1 : 0));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_wmsstats() const
{
	for (std::size_t i = 0; i < m_wms.size(); ++i)
	{
		auto const stats{m_wms[i]->stats()};
		WriteLog(format("SELFTEST wmsstats %zu %s: enabled %d, shown %d, loading %d, failed %d", i, m_wms[i]->settings().layers.c_str(), m_wms[i]->enabled() ? 1 : 0, stats.resident, stats.loading, stats.failed));
	}
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_orthostats() const
{
	auto const stats{m_orthophoto.stats()};
	auto const terrain{m_nmt.prefetched()};
	WriteLog(format("SELFTEST orthostats: shown %d, loading %d, failed %d; area %d of %d in the cache, %d failed%s; requests %d, uploads %d, stale %d; terrain area %d of %d, %d failed",
	                stats.resident, stats.loading, stats.failed, stats.area_cached, stats.area_tiles, stats.area_failed, stats.area_paused ? ", paused" : "", stats.requests, stats.uploads, stats.stale, terrain.cached,
	                terrain.total, terrain.failed));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrain_model(std::string_view const command, std::istringstream &words)
{
	if (command == "nmtfit")
	{
		// the heights of the terrain model fitted to the tracks, as the button of the terrain panel does
		nmt_fit();
		WriteLog("SELFTEST nmtfit: " + m_nmt_state.status);
		return selftest_result::handled;
	}
	if (command == "nmtmake")
		return selftest_nmtmake(words);
	if (command == "nmtunder")
	{
		// the terrain under the tracks, the tracks left as they are, as the button of the terrain panel does
		nmt_under_start();
		WriteLog("SELFTEST nmtunder: " + m_nmt_state.status);
		return selftest_result::handled;
	}
	if (command == "nmtpending")
		return selftest_nmtpending();
	if (command == "nmtoffset")
	{
		// the offset of the terrain model set, 0 as in an editor opened anew
		words >> m_nmt_state.offset;
		WriteLog(format("SELFTEST nmtoffset %+.2f", m_nmt_state.offset));
		return selftest_result::handled;
	}
	if (command == "nmtauto")
	{
		// the tracks put onto the terrain model, as the button of the panel of the tracks does, all or the ones not fitted yet;
		// nmtwait waits for the end
		std::string which;
		words >> which;
		nmt_auto_start(which == "unfitted");
		WriteLog("SELFTEST nmtauto " + which + ": " + m_nmt_state.status);
		return selftest_result::handled;
	}
	if (command == "shareexplain")
	{
		// the next sharing of the grade lines tells in the log of the runs of the tracks named
		m_nmt_state.explain = " ";
		for (std::string name; words >> name;)
			m_nmt_state.explain += name + " ";
		return selftest_result::handled;
	}
	if (command == "nmtoffer")
		return selftest_nmtoffer(words);
	if (command == "nmtplace")
		return selftest_nmtplace(words);
	if (command == "nmtgrow")
		return selftest_nmtgrow(words);
	if (command == "nmtwait")
		return selftest_nmtwait();
	if (command == "nmtstate")
	{
		// how far the tracks going onto the terrain are
		WriteLog(format("SELFTEST nmtstate stage %d, lines %zu, refused %zu, left %zu, squares waiting %zu: %s", static_cast<int>(m_nmt_state.stage), m_nmt_state.lines, m_nmt_state.refused, m_nmt_state.left_to_fit.size(), m_nmt.waiting(), m_nmt_state.status.c_str()));
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_nmtmake(std::istringstream &words)
{
	// chunks from the terrain model along the tracks (with the band in metres) or around the camera
	std::string where;
	double band{0.0};
	words >> where >> band;
	if (band > 0.0)
		m_nmt_state.band = band;
	nmt_make(where == "around" ? nmt_keys_around() : nmt_keys_along_tracks());
	WriteLog("SELFTEST nmtmake " + where + ": " + m_nmt_state.status);
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_nmtpending()
{
	// the chunks still waiting for the model: their place and the squares of the model they wait for
	WriteLog(format("SELFTEST nmtpending: %zu chunks, %zu made, model waiting %zu, fitting %d, stage %d, under %d, status %s", m_nmt_state.pending.size(), m_nmt_state.made, m_nmt.waiting(), m_nmt_state.fitting ? 1 : 0, static_cast<int>(m_nmt_state.stage), m_nmt_state.under ? 1 : 0, m_nmt_state.status.c_str()));
	for (auto const &key : m_nmt_state.pending)
	{
		auto const low{heightmap::chunk_corner(key)};
		auto const high{low + glm::dvec2{heightmap::chunk_size}};
		double height{0.0};
		auto const known{m_nmt.height(0.5 * (low.x + high.x), 0.5 * (low.y + high.y), height)};
		WriteLog(format("SELFTEST nmtpending chunk %d,%d at %.0f..%.0f %.0f..%.0f: exists %d, model in %d, height in the middle %d %.2f", key.first, key.second, low.x, high.x, low.y, high.y, m_streamer.exists(key) ? 1 : 0, m_nmt.request(low - glm::dvec2{2.0}, high + glm::dvec2{2.0}) ? 1 : 0, known ? 1 : 0, height));
	}
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_nmtoffer(std::istringstream &words)
{
	// the offer of the terrain to a scenery begun from nothing: logged, and taken with the place given, if any
	double north{0.0};
	double east{0.0};
	words >> north >> east;
	WriteLog(format("SELFTEST nmtoffer: shown %d", m_nmt_state.prompt ? 1 : 0));
	if (north > 0.0 && east > 0.0 && m_nmt_state.prompt)
	{
		m_nmt_map.pick({east, north});
		nmt_accept_offer();
		WriteLog("SELFTEST nmtoffer taken: " + m_nmt_state.status);
	}
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_nmtgrow(std::istringstream &words)
{
	// the cursor of a track laid at the place, its first point placed there unless "chosen" says the tool is only chosen: the terrain
	// grows around it once the laying begins
	auto const active{m_lay.active};
	auto const points{m_lay.points};
	std::string chosen;
	words >> m_lay.mouse.x >> m_lay.mouse.z >> chosen;
	m_lay.active = true;
	if (chosen != "chosen" && m_lay.points.empty())
		m_lay.points.push_back(m_lay.mouse);
	else if (chosen == "chosen")
		m_lay.points.clear();
	m_nmt_state.grow_time = -10.0;
	nmt_grow();
	m_lay.active = active;
	m_lay.points = points;
	WriteLog(format("SELFTEST nmtgrow: %zu chunks waiting, grown %d", m_nmt_state.pending.size(), m_nmt_state.grown ? 1 : 0));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_nmtwait()
{
	// till the model is in and the chunks waiting for it are made
	if (m_nmt_state.fitting || m_nmt_state.stage != nmt_state::automatic::idle || false == m_nmt_state.pending.empty())
	{
		--m_selftest.next;
		return selftest_result::pause;
	}
	WriteLog("SELFTEST nmtwait: " + m_nmt_state.status + (m_nmt_state.error.empty() ? std::string{} : " (" + m_nmt_state.error + ")"));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrain_fit(std::string_view const command, std::istringstream &words)
{
	if (command == "nmtcheck")
		return selftest_nmtcheck(words);
	if (command == "shorttracks")
		return selftest_shorttracks(words);
	if (command == "turntables")
		return selftest_turntables();
	if (command == "fittedstate")
		return selftest_fittedstate(words);
	if (command == "nmtprobe")
		return selftest_nmtprobe(words);
	if (command == "trackterrain")
		return selftest_trackterrain(words);
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_nmtcheck(std::istringstream &words) const
{
	// the ground under the axis of the tracks against their formation (the foot of the ballast), every Step m: over the top of the
	// ballast it sticks out of the track, far below it the track floats; and the places with no terrain at all
	double step{10.0};
	int top{20};
	words >> step >> top;
	std::vector<ground_finding> over;
	std::vector<ground_finding> under;
	std::size_t places{0};
	std::size_t without{0};
	std::size_t bridges{0};
	std::vector<double> offs;
	for (auto const &place : nmt_track_places(step))
	{
		if (false == place.shapes)
		{
			++bridges;
			continue;
		}
		++places;
		double height{0.0};
		if (false == m_streamer.height_at(place.position.x, place.position.z, height))
		{
			++without;
			continue;
		}
		auto const off{height - place.formation};
		offs.push_back(off);
		auto const name{place.track->name().empty() ? std::string{"(noname)"} : place.track->name()};
		if (height > place.crown + 0.02)
			over.emplace_back(height - place.crown, place.position, name);
		else if (off < -0.3)
			under.emplace_back(off, place.position, name);
	}
	std::ranges::sort(offs);
	auto const quantile = [&offs](double const Q) { return offs.empty() ? 0.0 : offs[static_cast<std::size_t>(Q * static_cast<double>(offs.size() - 1))]; };
	WriteLog(format("SELFTEST nmtcheck: %zu places, %zu without terrain, %zu on bridges or in tunnels; ground less formation: 5%% %.2f, median %.2f, 95%% %.2f m; over the top of the ballast %zu, more than 0.3 m under the formation %zu; modifiers %zu",
	                places, without, bridges, quantile(0.05), quantile(0.5), quantile(0.95), over.size(), under.size(), m_streamer.manifest().modifiers.size()));
	log_ground_findings(over, "over the crown", top);
	log_ground_findings(under, "under the formation", top);
	return selftest_result::handled;
}

// the include placed in the scenery whose first value names the turntable, with its place
std::string editor_mode::turntable_include_text(std::string const &Name) const
{
	std::string included;
	for (auto const &item : standing_templates([](std::string const &) { return true; }))
		if (false == item.values.empty() && ToLower(item.values.front()) == Name)
			included += format(" %s at %.3f %.3f %.3f", item.file.c_str(), item.location.x, item.location.y, item.location.z);
	return included;
}

// a track running up to the table: out along its line from the rim, the heights with the model beside them, and how far from the
// rim the line keeps level
void editor_mode::log_turntable_end(nmt_state::turntable const &Table, TTrack *Track, int const Path, bool const Atend) const
{
	auto route{editor_track::run_route(*Track, 80.0, Path)};
	auto const samples{editor_track::sample_route(route, 1.0)};
	if (samples.size() < 2)
		return;
	auto const &points{Table.table->m_paths.front().points};
	auto const &rim{Track->m_paths[Path].points[Atend ? segment_data::point::end : segment_data::point::start]};
	auto const reversed{geometry::plan_distance(samples.back().position, rim) < geometry::plan_distance(samples.front().position, rim)};
	auto const length{samples.back().chainage - samples.front().chainage};
	auto const chainage_at = [&samples, reversed](double const Off) { return reversed ? samples.back().chainage - Off : samples.front().chainage + Off; };
	auto const height_at = [&samples, &chainage_at](double const Off) { return editor_track::sampled_elevation(samples, chainage_at(Off)); };
	// the heights out along the track from the rim, with the model beside them
	std::string text;
	for (auto const off : {0.0, 2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0})
	{
		if (off > length)
			break;
		text += format(" %.0f m %.3f", off, height_at(off));
		text += model_at(samples, chainage_at(off), static_cast<double>(Track->rail_height()), m_nmt, m_nmt_state.offset);
	}
	// level off the rim within a centimetre
	auto const level{level_for(height_at, length, points[segment_data::point::start].y)};
	WriteLog(format("SELFTEST turntable %s track %s: at the rim %.3f m, %+.2f per mille over its first 5 m, level within 1 cm of the table for %.1f m; out along it%s", Table.table->name().c_str(), Track->name().c_str(), rim.y,
	                1000.0 * (height_at(std::min(5.0, length)) - height_at(0.0)) / std::max(1e-6, std::min(5.0, length)), level, text.c_str()));
}

editor_mode::selftest_result editor_mode::selftest_turntables() const
{
	// each turntable: the height of its rails and of the model in its pit; of each track running up to it the heights out along
	// its line from the rim, the model there under the rails beside them, and how far from the rim the line keeps level
	for (auto const &table : nmt_turntables())
	{
		auto const &points{table.table->m_paths.front().points};
		double pit{std::numeric_limits<double>::quiet_NaN()};
		if (double height; m_nmt.height(table.centre.x, table.centre.z, height))
			pit = height + m_nmt_state.offset;
		auto const included{turntable_include_text(table.table->name())};
		WriteLog(format("SELFTEST turntable %s: rails %.3f .. %.3f m, rim %.2f m, the model in the pit %.2f m, %zu tracks run up to it; its include%s", table.table->name().c_str(), points[segment_data::point::start].y,
		                points[segment_data::point::end].y, table.rim, pit, table.ends.size(), included.empty() ? " none" : included.c_str()));
		for (auto const &[track, path, atend] : table.ends)
			log_turntable_end(table, track, path, atend);
	}
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_fittedstate(std::istringstream &words)
{
	// the tracks whose grade line is fitted, as the panel of the tracks tells it; reload: the marks read again from the lines
	// they go to the files with
	std::string reload;
	words >> reload;
	if (reload == "reload")
		m_build_tools->library.fitted_forget();
	std::size_t rails{0};
	std::size_t done{0};
	std::string names;
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 1) == 0)
			continue;
		++rails;
		if (false == m_build_tools->library.grade_fitted(*track))
			continue;
		++done;
		if (names.size() < 200)
			names += " " + track->name();
	}
	std::size_t written{0};
	for (auto const layer : scene::Layers.marked_layers("//$t"))
		written += scene::Layers.marked(layer, "//$t").size();
	WriteLog(format("SELFTEST fittedstate: %zu of %zu tracks fitted, %zu marks kept, %zu lines for the files;", done, rails, m_build_tools->library.fitted_count(), written) + names);
	return selftest_result::handled;
}

// heights across the track at the place, towards the next one: the rail top, the top of the ballast under the rails, the model, the
// ground under the axis and beside the track
void editor_mode::log_nmt_probe(nmt_place const &Place, nmt_place const &Next) const
{
	glm::dvec2 along{Next.position.x - Place.position.x, Next.position.z - Place.position.z};
	if (glm::length(along) < 1e-6)
		return;
	along = glm::normalize(along);
	glm::dvec2 const across{-along.y, along.x};
	std::vector<glm::dvec3> points;
	for (double const offset : {0.0, -2.0, 2.0, -3.5, 3.5, -6.0, 6.0})
		points.emplace_back(Place.position.x + across.x * offset, Place.position.y, Place.position.z + across.y * offset);
	std::vector<char> found;
	auto const ground{ground_heights(points, true, &found)};
	// the model at the point, none where it has none
	auto const model_beside = [this](glm::dvec3 const &Point) {
		double side{std::numeric_limits<double>::quiet_NaN()};
		if (double height; m_nmt.height(Point.x, Point.z, height))
			side = height + m_nmt_state.offset;
		return side;
	};
	auto const model{model_beside(Place.position)};
	std::string text{format("SELFTEST nmtprobe %s at %.0f %.0f: rail top %.2f, ballast top %.2f, foot %.2f, model %.2f; ground/model at axis, -2, 2, -3.5, 3.5, -6, 6 m:", Place.track->name().c_str(), Place.position.x, Place.position.z, Place.position.y, Place.crown, Place.formation, model)};
	for (std::size_t k = 0; k < points.size(); ++k)
		text += (found[k] != 0 ? format(" %.2f", ground[k]) : std::string{" -"}) + format("/%.2f", model_beside(points[k]));
	WriteLog(text);
}

editor_mode::selftest_result editor_mode::selftest_nmtprobe(std::istringstream &words) const
{
	// heights across the tracks: the rail top, the top of the ballast under the rails, the model, the ground under the axis
	// and beside the track
	std::string name;
	double step{250.0};
	words >> name >> step;
	auto const places{nmt_track_places(step)};
	for (std::size_t i = 0; i + 1 < places.size(); ++i)
	{
		auto const &place{places[i]};
		auto const &next{places[i + 1]};
		if (place.track != next.track || place.path != next.path || (false == name.empty() && place.track->name() != name))
			continue;
		log_nmt_probe(place, next);
	}
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_trackterrain(std::istringstream &words)
{
	// the terrain drawn to the named tracks, or their formations taken out of it, as the buttons of the panel of the tracks
	std::string action;
	words >> action;
	std::vector<TTrack *> tracks;
	for (std::string name; words >> name;)
		if (auto *track{simulation::Paths.find(name)})
			tracks.push_back(track);
	if (action == "pull")
		nmt_pull_terrain(tracks);
	else
		nmt_restore_terrain(tracks);
	WriteLog(format("SELFTEST trackterrain %s %zu tracks: %s, %zu modifiers", action.c_str(), tracks.size(), m_nmt_state.status.c_str(), m_streamer.modifiers().size()));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrain_edit(std::string_view const command, std::istringstream &words)
{
	if (command == "terrainmode")
	{
		m_terrain_open = true;
		WriteLog("SELFTEST terrainmode");
		return selftest_result::handled;
	}
	if (command == "terrainsculpt" || command == "terrainpaint" || command == "terrainlevel")
		return selftest_terrain_brush(command, words);
	if (command == "terrainmodifier")
	{
		heightmap::modifier modifier;
		heightmap::corridor_point a;
		heightmap::corridor_point b;
		double formation{0.0};
		double halfwidth{3.5};
		words >> modifier.name >> a.position.x >> a.position.y >> b.position.x >> b.position.y >> formation >> halfwidth;
		b.formation = formation;
		a.formation = b.formation;
		b.half_width = halfwidth;
		a.half_width = b.half_width;
		modifier.points = {a, b};
		m_streamer.modifier(std::move(modifier));
		forget_ground();
		WriteLog(format("SELFTEST terrainmodifier: %zu modifiers", m_streamer.modifiers().size()));
		return selftest_result::handled;
	}
	if (command == "terrainfill")
		return selftest_terrainfill(words);
	if (command == "terrainturn")
	{
		int layer{0};
		float degrees{0.f};
		words >> layer >> degrees;
		m_streamer.layer_rotation(static_cast<std::uint16_t>(layer), degrees);
		WriteLog(format("SELFTEST terrainturn %d: %.1f deg", layer, m_streamer.manifest().layers.at(layer).rotation));
		return selftest_result::handled;
	}
	if (command == "terraintouchup")
	{
		int state{1};
		words >> state;
		m_streamer.touch_up(state != 0);
		WriteLog(format("SELFTEST terraintouchup %d", state));
		return selftest_result::handled;
	}
	if (command == "terrainrestore")
		return selftest_terrainrestore(words);
	if (command == "terrainbake")
		return selftest_terrainbake(words);
	if (command == "terrainselect")
		return selftest_terrainselect(words);
	if (command == "terrainremove")
	{
		// the chunks selected taken out, as the button of the chunk panel and Del do
		remove_selected_chunks();
		WriteLog("SELFTEST terrainremove: " + m_terrain_status);
		return selftest_result::handled;
	}
	if (command == "terrainresolution")
	{
		float spacing{1.f};
		int paint{1};
		words >> spacing >> paint;
		m_terrain_convert_spacing = heightmap::valid_spacing(spacing);
		m_terrain_convert_paint = paint != 0;
		convert_selected_chunks();
		WriteLog("SELFTEST terrainresolution: " + m_terrain_status);
		return selftest_result::handled;
	}
	if (command == "terrainwater" || command == "terrainwaterdefault")
		return selftest_terrainwater(command, words);
	if (command == "terrainadd")
	{
		int count{1};
		float spacing{m_terrain_spacing};
		words >> count >> spacing;
		m_terrain_spacing = heightmap::valid_spacing(spacing);
		add_chunks_around(std::clamp(count, 1, 8));
		WriteLog("SELFTEST terrainadd: " + m_terrain_status);
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_terrain_brush(std::string_view const command, std::istringstream &words)
{
	double x{0.0};
	double z{0.0};
	double radius{10.0};
	double value{1.0};
	int mode{0};
	words >> x >> z >> radius >> value >> mode;
	bool changed{false};
	if (command == "terrainsculpt")
		changed = m_streamer.sculpt(x, z, radius, value);
	else if (command == "terrainlevel")
		changed = m_streamer.level(x, z, radius, value, 1.0, mode);
	else
		changed = m_streamer.paint(x, z, radius, 1.0, static_cast<std::uint16_t>(value));
	forget_ground();
	WriteLog(format("SELFTEST %s %.1f %.1f: %s", std::string{command}.c_str(), x, z, changed ? "changed" : "nothing changed"));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrainfill(std::istringstream &words)
{
	int layer{0};
	words >> layer >> m_paint_feather >> m_paint_rotation;
	m_terrain_layer = layer;
	m_paint_outline.clear();
	double x;
	double z;
	while (words >> x >> z)
		m_paint_outline.emplace_back(x, 0.0, z);
	fill_terrain_area();
	std::string palette;
	for (auto const &entry : m_streamer.manifest().layers)
		palette += format(" [%s %.1f m %.0f deg]", entry.material.c_str(), entry.size, entry.rotation);
	WriteLog("SELFTEST terrainfill: " + m_terrain_status + ", palette" + palette);
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrainrestore(std::istringstream &words)
{
	double x{0.0};
	double z{0.0};
	double radius{10.0};
	words >> x >> z >> radius;
	auto const changed{m_streamer.restore_stitching(x, z, radius, 1.0)};
	forget_ground();
	WriteLog(format("SELFTEST terrainrestore %.1f %.1f: %s, touch-ups left %zu", x, z, changed ? "changed" : "nothing changed", m_streamer.touched_up()));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrainbake(std::istringstream &words)
{
	std::string name;
	words >> name;
	auto const baked{m_streamer.bake_modifier(name)};
	forget_ground();
	WriteLog(format("SELFTEST terrainbake %s: %s, %zu modifiers left", name.c_str(), baked ? "baked" : "not found", m_streamer.modifiers().size()));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrainselect(std::istringstream &words)
{
	int cx{0};
	int cz{0};
	std::string add;
	words >> cx >> cz >> add;
	if (add != "add")
		m_terrain_selection.clear();
	if (m_streamer.exists({cx, cz}))
		m_terrain_selection.insert({cx, cz});
	WriteLog(format("SELFTEST terrainselect: %zu selected", m_terrain_selection.size()));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrainwater(std::string_view const command, std::istringstream &words)
{
	if (command == "terrainwaterdefault")
	{
		std::string material;
		words >> material;
		m_streamer.water_material(material);
	}
	else
	{
		std::size_t index{0};
		std::string material;
		float size{0.f};
		words >> index >> material >> size;
		auto water{m_streamer.water()};
		if (index < water.size())
		{
			water[index].material = material == "-" ? std::string{} : material;
			water[index].size = size;
			m_streamer.water(std::move(water));
		}
	}
	std::string list;
	for (auto const &body : m_streamer.water())
		list += format(" [%s: %s, %.1f m]", body.name.c_str(), body.material.empty() ? "default" : body.material.c_str(), body.size);
	WriteLog("SELFTEST " + std::string{command} + ": default " + m_streamer.manifest().water_material + list);
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrain_told(std::string_view const command, std::istringstream &words)
{
	if (command == "terrainseams")
		return selftest_terrainseams();
	if (command == "danglers")
		return selftest_danglers();
	if (command == "terrainhash")
		return selftest_terrainhash();
	if (command == "terrainprobe")
		return selftest_terrainprobe(words);
	if (command == "terrainconvert")
		return selftest_terrainconvert();
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_terrainseams() const
{
	// the steps of the ground where two chunks in memory meet
	std::vector<editor_terrain *> chunks;
	m_streamer.collect(chunks);
	std::set<heightmap::chunk_key> keys;
	for (auto const *chunk : chunks)
		keys.insert(chunk->key());
	std::size_t edges{0};
	double worst{0.0};
	glm::dvec2 where{0.0};
	// the step of the terrain across the edge of two chunks at the corner, along it in 32 pieces: the greatest of all kept
	auto const edge_steps = [this, &worst, &where](glm::dvec2 const &Corner, int const Dx, int const Dz) {
		for (int i = 0; i <= 32; ++i)
		{
			auto const along{heightmap::chunk_size * i / 32.0};
			glm::dvec2 const point{Corner.x + (Dx == 0 ? along : 0.0), Corner.y + (Dz == 0 ? along : 0.0)};
			glm::dvec2 const inside{Dx * 0.01, Dz * 0.01};
			if (double a, b; m_streamer.height_at(point.x - inside.x, point.y - inside.y, a) && m_streamer.height_at(point.x + inside.x, point.y + inside.y, b) && std::abs(a - b) > worst)
			{
				worst = std::abs(a - b);
				where = point;
			}
		}
	};
	for (auto const &[key_x, key_z] : keys)
		for (auto const [dx, dz] : {std::pair{1, 0}, std::pair{0, 1}})
		{
			if (false == keys.contains({key_x + dx, key_z + dz}))
				continue;
			++edges;
			edge_steps(heightmap::chunk_corner({key_x + dx, key_z + dz}), dx, dz);
		}
	WriteLog(format("SELFTEST terrainseams: %zu chunks, %zu edges, the greatest step %.3f m at %.1f %.1f", keys.size(), edges, worst, where.x, where.y));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_danglers() const
{
	// models the cells hold which aren't in the scene any more, or are about to be destroyed
	std::set<TAnimModel const *> const alive{simulation::Instances.sequence().begin(), simulation::Instances.sequence().end()};
	std::size_t held{0};
	std::size_t gone{0};
	std::size_t retired{0};
	// a model a cell holds: gone from the scene, or about to be destroyed
	auto const count_model = [this, &alive, &held, &gone, &retired](TAnimModel const *Instance) {
		++held;
		if (false == alive.contains(Instance))
			++gone;
		else if (std::ranges::any_of(m_retired, [&](std::pair<TAnimModel *, int> const &Retired) { return Retired.first == Instance; }))
			++retired;
	};
	// the models of both kinds a cell holds
	auto const count_cell = [&count_model](auto const &Cell) {
		for (auto const *sequence : {&Cell.m_instancesopaque, &Cell.m_instancetranslucent})
			std::ranges::for_each(*sequence, count_model);
	};
	for (auto const *section : simulation::Region->m_sections)
	{
		if (section == nullptr)
			continue;
		for (auto const &cell : section->m_cells)
			count_cell(cell);
	}
	WriteLog(format("SELFTEST danglers: %zu held, %zu gone, %zu retired, %zu waiting", held, gone, retired, m_retired.size()));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrainhash() const
{
	// the heights of the chunks in memory hashed, to compare the ground two builds made
	std::vector<editor_terrain *> chunks;
	m_streamer.collect(chunks);
	std::ranges::sort(chunks, [](editor_terrain const *A, editor_terrain const *B) { return A->key() < B->key(); });
	std::uint64_t hash{1469598103934665603ULL};
	for (auto const *chunk : chunks)
		hash_heights(hash, *chunk);
	WriteLog(format("SELFTEST terrainhash: %zu chunks, %016llx", chunks.size(), static_cast<unsigned long long>(hash)));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrainprobe(std::istringstream &words) const
{
	double x{0.0};
	double z{0.0};
	double height{0.0};
	words >> x >> z;
	auto const *chunk{m_streamer.terrain_at(x, z)};
	std::string materials;
	if (chunk != nullptr)
		for (std::size_t slot = 0; slot < std::max<std::size_t>(1, chunk->data().layer_count()); ++slot)
		{
			auto const layer{chunk->data().layers[slot]};
			auto const &palette{m_streamer.manifest().layers};
			materials += (layer < palette.size() ? palette[layer].material : std::string{"?"}) + " ";
		}
	auto const found{m_streamer.height_at(x, z, height)};
	std::string grid;
	if (chunk != nullptr)
		grid = chunk_grid(chunk->data(), x, z);
	WriteLog(format("SELFTEST terrainprobe %.1f %.1f: %s %.3f, %zu chunks, materials %s%s", x, z, found ? "height" : "no terrain", height, m_streamer.resident(), materials.c_str(), grid.c_str()));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrainconvert()
{
	load_ground_files();
	// the box the triangles of the file lie in
	auto const bounds_of = [this](auto const &Source, glm::dvec3 &Low, glm::dvec3 &High) {
		for (auto index = Source.first; index < Source.first + Source.count; ++index)
			for (auto const &vertex : m_ground_conversion.triangles[index])
			{
				Low = glm::min(Low, vertex);
				High = glm::max(High, vertex);
			}
	};
	for (auto const &source : m_ground_conversion.files)
	{
		glm::dvec3 low{1e30};
		glm::dvec3 high{-1e30};
		bounds_of(source, low, high);
		WriteLog("SELFTEST terrain file " + source.file + (source.text ? " (text): " : " (binary): ") + source.message +
		         format(", from %.1f %.1f %.1f to %.1f %.1f %.1f", low.x, low.y, low.z, high.x, high.y, high.z));
	}
	m_ground_conversion.status = convert_ground();
	WriteLog("SELFTEST terrainconvert: " + m_ground_conversion.status);
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_terrain_reports(std::string_view const command, std::istringstream &words)
{
	if (command == "fitreport")
		return selftest_fitreport(words);
	if (command == "gradeline")
		return selftest_gradeline(words);
	if (command == "trackheights")
		return selftest_trackheights(words);
	if (command == "profiledump")
		return selftest_profiledump(words);
	if (command == "besidedump")
		return selftest_besidedump(words);
	if (command == "profileaudit")
		return selftest_profileaudit();
	if (command == "gradelines")
		return selftest_gradelines(words);
	return selftest_result::unhandled;
}

namespace
{
// the terrain model the rails are measured against, with its offset
struct fit_model
{
	editor_nmt const &model;
	double offset;
};

// of the track how far its top of the ballast is off the terrain model, of a switch how far its second path is off the plane of the
// first, to the report of the fit; its places kept
void stream_track_fit(std::ostream &Stream, TTrack const &Track, int const Line, fit_model const &Model, fit_places &Places)
{
	fit_tally tally;
	for (auto const &path : Track.m_paths)
		tally_path(&Track, path, Model.model, Model.offset, Places, tally);
	if (tally.counted > 0)
		Stream << "T " << Track.name() << ' ' << (Track.eType == tt_Switch ? 's' : 'n') << ' ' << Line << ' ' << tally.worst << ' ' << tally.sum / tally.counted << ' ' << tally.worst_at.x << ' ' << tally.worst_at.z << ' ' << tally.worst_at.y << ' ' << tally.counted << '\n';
	if (Track.eType == tt_Switch && Track.m_paths.size() >= 2)
		stream_switch_plane(Stream, Track, Line);
}
} // namespace

// the number of the grade line the track shares, or the one of its own
int editor_mode::track_grade_line(TTrack const &Track) const
{
	auto const found{m_nmt_state.grade_lines.find(grade_line_tool::grade_line_key(Track))};
	return found != m_nmt_state.grade_lines.end() ? found->second : nmt_state::grade_line_own;
}

editor_mode::selftest_result editor_mode::selftest_fitreport(std::istringstream &words) const
{
	// after the tracks went onto the terrain, to a file: of each track how far its top of the ballast is off the terrain model (T),
	// of each switch how far its second path is off the plane of the first (P), of each switch and track on timbers common with it,
	// under 2.65 m apart, the most their rails differ in height (K)
	std::string file;
	words >> file;
	std::ofstream stream(file);
	fit_places places;
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty() || (track->iCategoryFlag & 15) != 1 || track->eType == tt_Table)
			continue;
		stream_track_fit(stream, *track, track_grade_line(*track), {m_nmt, m_nmt_state.offset}, places);
	}
	std::map<std::pair<std::string, std::string>, pair_worst> pairs;
	for (auto const &here : places.all)
		pairs_near(here, places, pairs);
	for (auto const &[names, worst] : pairs)
		stream << "K " << names.first << ' ' << names.second << ' ' << worst.difference << ' ' << worst.apart << ' ' << worst.at.x << ' ' << worst.at.z << '\n';
	for (auto const &place : m_nmt_state.surface_places)
		if (auto const *track{place.track}; false == track->m_editorremoved && place.path < static_cast<int>(track->m_paths.size()))
		{
			auto const point{geometry::bezier{track->m_paths[place.path]}.point(place.share)};
			stream << "Z " << place.surface << ' ' << point.x << ' ' << point.z << ' ' << point.y << ' ' << track->name() << '\n';
		}
	WriteLog("SELFTEST fitreport: " + file);
	return selftest_result::handled;
}

// the common grade line of the route open in the profile tool: lift <m>: its design up by that much and applied; hand: its tracks marked
// as edited by hand; off, on: the route
// taken off it and back; whole 0/1: whether the tracks beside go with it; state told in the log
editor_mode::selftest_result editor_mode::selftest_gradeline(std::istringstream &words)
{
	// the word after the action is its number, or the name of a track
	std::string action;
	std::string word;
	words >> action >> word;
	double const value{std::atof(word.c_str())};
	auto &state{m_profile};
	if (action == "part")
		gradeline_part(words, value);
	if (action == "bump")
		gradeline_bump(words, value);
	if (false == gradeline_action(action, word, value, words))
		gradeline_other(action, word, value, words);
	m_build_tools->grade_lines.grade_lines_load();
	auto const found{state.route.spans.empty() ? m_nmt_state.grade_lines.end() : m_nmt_state.grade_lines.find(grade_line_tool::grade_line_key(*state.route.spans.front().track))};
	WriteLog(format("SELFTEST gradeline %s: route %zu tracks, of line %d, %zu beside, whole %d, %zu tracks on lines, %zu by hand", action.c_str(), state.route.spans.size(), found != m_nmt_state.grade_lines.end() ? found->second : -100, m_build_tools->grade_lines.grade_line_beside().size(), state.whole_grade_line ? 1 : 0, m_nmt_state.grade_lines.size(), m_nmt_state.grade_lines_hand.size()));
	return selftest_result::handled;
}

// the route cut to <value> tracks on each side of the track named
void editor_mode::gradeline_part(std::istringstream &words, double const value)
{
	std::string name;
	words >> name;
	auto const &spans{m_profile.route.spans};
	auto const at{std::ranges::find_if(spans, [&](auto const &Span) { return Span.track->name() == name; })};
	if (at == spans.end())
		return;
	auto const index{std::distance(spans.begin(), at)};
	auto *first{spans[std::max<std::ptrdiff_t>(0, index - static_cast<std::ptrdiff_t>(value))].track};
	auto *last{spans[std::min<std::ptrdiff_t>(spans.size() - 1, index + static_cast<std::ptrdiff_t>(value))].track};
	m_build_tools->profile_edit.profile_open(first, last);
}

// a point more in the middle of the route, up by that much, and applied
void editor_mode::gradeline_bump(std::istringstream &words, double const value)
{
	auto &state{m_profile};
	profile::pvi point;
	double at{state.route.length * 0.5};
	words >> at;
	point.chainage = at;
	point.elevation = profile::elevation(state.line, point.chainage) + value;
	auto &points{state.line.points};
	points.insert(std::ranges::lower_bound(points, point.chainage, {}, &profile::pvi::chainage), point);
	m_build_tools->profile_edit.profile_edited();
	m_build_tools->profile_edit.profile_apply(false);
	WriteLog("SELFTEST gradeline bump: " + state.status + " " + state.error);
	for (auto const &issue : state.issues | std::views::filter([](auto const &Issue) { return Issue.error; }))
		WriteLog(format("SELFTEST gradeline error at %.1f: %s", issue.chainage, issue.text.c_str()));
}

// lift, off, on, picking, move, whole and hand, the actions of the line itself. returns: false for the actions of the tracks
bool editor_mode::gradeline_action(std::string const &action, std::string const &word, double const value, std::istringstream &words)
{
	auto &state{m_profile};
	if (action == "lift")
	{
		for (auto &point : state.line.points)
			point.elevation += value;
		m_build_tools->profile_edit.profile_check();
		m_build_tools->profile_edit.profile_apply(false);
		WriteLog("SELFTEST gradeline lift: " + state.status + " " + state.error);
		for (auto const &issue : state.issues | std::views::filter([](auto const &Issue) { return Issue.error; }))
			WriteLog(format("SELFTEST gradeline error at %.1f: %s", issue.chainage, issue.text.c_str()));
	}
	else if (action == "off" || action == "on")
		m_build_tools->grade_lines.grade_line_take_off(action == "off");
	else if (action == "picking")
		state.picking = value != 0.0;
	else if (action == "move")
		gradeline_move(word, words);
	else if (action == "whole")
		state.whole_grade_line = value != 0.0;
	else if (action == "hand")
	{
		// the tracks of the route marked as edited by hand, without an edit held up by what lies on them
		std::vector<TTrack *> tracks;
		for (auto const &span : state.route.spans)
			tracks.push_back(span.track);
		m_build_tools->library.fitted_set(tracks, true);
	}
	else
		return false;
	return true;
}

// move <name> <dx> <dz>: the track by that in the plan, its ends off the ones it met
void editor_mode::gradeline_move(std::string const &word, std::istringstream &words)
{
	glm::dvec3 offset{0.0};
	words >> offset.x >> offset.z;
	if (auto *track{simulation::Paths.find(word)}; track != nullptr)
	{
		std::vector<std::pair<TTrack *, editor_track::state>> states{{track, editor_track::capture(*track)}};
		editor_track::translate(*track, offset);
		editor_track::commit({track});
		push_track_snapshot(std::move(states), {});
	}
}

// of, rename and pick, the actions of the tracks named or clicked
void editor_mode::gradeline_other(std::string const &action, std::string const &word, double const value, std::istringstream &words)
{
	if (action == "of" || action == "rename")
		gradeline_of(action, word, words);
	else if (action == "pick")
		gradeline_pick(value, words);
}

// of <name|selected>: the line of the track; rename <name> <new>: under the new name, its line going with it
void editor_mode::gradeline_of(std::string const &action, std::string const &name, std::istringstream &words)
{
	std::string renamed;
	words >> renamed;
	auto *track{name == "selected" ? selected_track() : simulation::Paths.find(name)};
	if (track != nullptr && action == "rename" && false == renamed.empty())
		editor_track::rename(*track, renamed);
	m_build_tools->grade_lines.grade_lines_load();
	auto const found{track != nullptr ? m_nmt_state.grade_lines.find(grade_line_tool::grade_line_key(*track)) : m_nmt_state.grade_lines.end()};
	WriteLog(format("SELFTEST gradeline %s %s: %s, line %d%s", action.c_str(), name.c_str(), track != nullptr ? track->name().c_str() : "-", found != m_nmt_state.grade_lines.end() ? found->second : -100,
	                found != m_nmt_state.grade_lines.end() && m_nmt_state.grade_lines_hand.contains(found->first) ? " by hand" : ""));
}

// pick <x> <z> [1]: the track nearest to it clicked while picking, onto the grade line of the route or off it; 1: the track only
void editor_mode::gradeline_pick(double const x, std::istringstream &words)
{
	double z{0.0};
	int alone{0};
	words >> z >> alone;
	glm::dvec3 const point{x, 0.0, z};
	auto *const closest{nearest_not_table(point, 10.0)};
	if (closest != nullptr)
		m_build_tools->grade_lines.grade_line_pick(*closest, alone != 0);
	auto const number{m_build_tools->grade_lines.route_grade_line()};
	WriteLog(format("SELFTEST gradeline pick %s: %s; the route on line %d of %zu tracks", closest != nullptr ? closest->name().c_str() : "-", m_profile.status.c_str(), number, m_build_tools->grade_lines.grade_line_members(number).size()));
}

// the heights of the ends of the paths of all the tracks to the file, for two states of the scenery to be compared, the stay of each
editor_mode::selftest_result editor_mode::selftest_trackheights(std::istringstream &words)
{
	std::string file;
	words >> file;
	std::ofstream stream{file};
	stream << std::fixed << std::setprecision(4);
	auto const grade_mark = [this](TTrack const &Track) {
		if (m_build_tools->library.grade_hand(Track))
			return 'h';
		return m_build_tools->library.grade_fitted(Track) ? 'f' : 'n';
	};
	// the heights of the ends of each path of the track
	auto const heights_of = [&stream, &grade_mark](TTrack const &Track) {
		for (auto const &path : Track.m_paths)
			stream << Track.name() << ' ' << grade_mark(Track) << ' ' << path.points[segment_data::point::start].y << ' ' << path.points[segment_data::point::end].y << '\n';
	};
	for (auto const *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved)
			heights_of(*track);
	WriteLog("SELFTEST trackheights: " + file);
	return selftest_result::handled;
}

// the track of the span of the route open in the profile tool, none past its spans
TTrack const *editor_mode::profile_span_track(std::size_t const Span) const
{
	return Span < m_profile.route.spans.size() ? m_profile.route.spans[Span].track : nullptr;
}

// the route open in the profile tool, as it lies and as the grade line reads it: S chainage height grade track, P chainage height radius
editor_mode::selftest_result editor_mode::selftest_profiledump(std::istringstream &words) const
{
	std::string file;
	words >> file;
	std::ofstream stream{file, std::ios::app};
	stream << std::fixed << std::setprecision(4);
	stream << "R " << m_profile.samples.size() << ' ' << m_profile.line.points.size() << ' ' << m_profile.departure << '\n';
	for (auto const &sample : m_profile.samples)
	{
		auto const *track{profile_span_track(sample.span)};
		double model{std::numeric_limits<double>::quiet_NaN()};
		if (double height; m_nmt.height(sample.position.x, sample.position.z, height))
			model = height + m_nmt_state.offset + (track != nullptr ? track->rail_height() : 0.0);
		stream << "S " << sample.chainage << ' ' << sample.position.y << ' ' << sample.grade << ' ' << (track != nullptr ? track->name() : std::string{"-"}) << ' ' << model << '\n';
	}
	for (auto const &point : m_profile.line.points)
		stream << "P " << point.chainage << ' ' << point.elevation << ' ' << point.radius << '\n';
	// what a fit of the line is told besides: the ends joined, the switches, the heights held, the norms
	auto const &context{m_profile.context};
	auto const &line{m_profile.line};
	stream << "C " << context.length << ' ' << context.start_joined << ' ' << context.start_elevation << ' ' << context.start_grade << ' ' << context.end_joined << ' ' << context.end_elevation << ' ' << context.end_grade << '\n';
	for (auto const &zone : context.switches)
		stream << "Z " << zone.from << ' ' << zone.to << '\n';
	for (auto const &zone : context.humps)
		stream << "H " << zone.from << ' ' << zone.to << '\n';
	for (auto const &[chainage, elevation] : context.fixed)
		stream << "F " << chainage << ' ' << elevation << '\n';
	stream << "L " << line.speed << ' ' << line.norms.grade_max << ' ' << line.norms.grade_switch_max << ' ' << line.norms.radius_factor << ' ' << line.norms.radius_min << ' ' << line.norms.curve_threshold << ' ' << line.norms.element_min << ' '
	       << line.norms.fixed_tolerance << ' ' << line.join_start << ' ' << line.join_end << '\n';
	WriteLog("SELFTEST profiledump: " + file);
	return selftest_result::handled;
}

// the number of the grade line the track shares, -100 where it has none
int editor_mode::beside_line(TTrack const &Track) const
{
	auto const found{m_nmt_state.grade_lines.find(grade_line_tool::grade_line_key(Track))};
	return found != m_nmt_state.grade_lines.end() ? found->second : -100;
}

namespace
{
// every fifth sample of the route to the file, with the nearest nearly parallel track under 6 m on each side: the track there, its
// line, how far off and how much higher, or none
template <typename Samples, typename Spans, typename Line_of>
void stream_beside(std::ostream &Stream, Samples const &samples, Spans const &Route, std::vector<beside_spot> const &Spots, Line_of const &Line)
{
	auto const stream_side = [&Stream, &Line](beside_spot const *Side, double const Across, double const Height) {
		if (Side != nullptr)
			Stream << " | " << Side->track->name() << ' ' << Line(*Side->track) << ' ' << Across << ' ' << Side->position.y - Height;
		else
			Stream << " | -";
	};
	for (std::size_t i = 0; i + 1 < samples.size(); i += 5)
	{
		auto const &here{samples[i]};
		auto const along{glm::normalize(geometry::plan_of(samples[i + 1].position - here.position))};
		std::array<beside_spot const *, 2> sides{nullptr, nullptr};
		std::array<double, 2> across{6.0, 6.0};
		sides_of(here.position, along, Spots, sides, across);
		TTrack const *track{here.span < Route.size() ? Route[here.span].track : nullptr};
		Stream << "B " << here.chainage << ' ' << here.position.y << ' ' << (track != nullptr ? track->name() : std::string{"-"}) << ' ' << (track != nullptr ? Line(*track) : -100);
		for (int k = 0; k < 2; ++k)
			stream_side(sides[k], across[k], here.position.y);
		Stream << '\n';
	}
}
} // namespace

// the tracks beside the route open in the profile tool, every 5 m: T track line, B chainage height track line across height of the nearest
// nearly parallel track under 6 m on each side
editor_mode::selftest_result editor_mode::selftest_besidedump(std::istringstream &words)
{
	std::string file;
	words >> file;
	std::ofstream stream{file, std::ios::app};
	stream << std::fixed << std::setprecision(3);
	m_build_tools->grade_lines.grade_lines_load();
	auto const grade_mark = [this](TTrack const &Track) {
		if (m_build_tools->library.grade_hand(Track))
			return 'h';
		return m_build_tools->library.grade_fitted(Track) ? 'f' : 'n';
	};
	std::unordered_set<TTrack const *> route;
	for (auto const &span : m_profile.route.spans)
	{
		route.insert(span.track);
		stream << "T " << span.track->name() << ' ' << beside_line(*span.track) << ' ' << grade_mark(*span.track) << '\n';
	}
	std::vector<beside_spot> spots;
	for (auto const *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved && false == route.contains(track))
			spots_of(track, spots);
	stream_beside(stream, m_profile.samples, m_profile.route.spans, spots, [this](TTrack const &Track) { return beside_line(Track); });
	WriteLog("SELFTEST besidedump: " + file);
	return selftest_result::handled;
}

// the design leaving the track by more than what it's read again from it for: where it's off most, and the switches about there
void editor_mode::log_profile_off(double const Departure, TTrack const *From, TTrack const *To) const
{
	auto const [worst, at]{farthest_off(m_profile.samples, m_profile.line)};
	if (m_profile.samples.empty())
		return;
	auto const zone{switch_zones(m_profile.context.switches, m_profile.samples[at].chainage)};
	WriteLog(format("SELFTEST profileaudit off: %s .. %s, %.0f m, %.3f m off at %.1f on %s, kept departure %.3f;%s %s", From->name().c_str(), To->name().c_str(), m_profile.route.length, worst, m_profile.samples[at].chainage,
	                m_profile.route.spans[m_profile.samples[at].span].track->name().c_str(), Departure, zone.c_str(), m_profile.status.c_str()));
}

// the design kept in the scenery opened as the profile opens it, counted among those with no route, those off the track and those
// with errors
void editor_mode::profile_audit(std::size_t const Index, std::size_t &Gone, std::size_t &Retaken, std::size_t &Failing)
{
	auto const entry{m_profiles[Index]};
	TTrack *from{nullptr};
	TTrack *to{nullptr};
	auto const stored{m_build_tools->library.profile_stored_route(entry, &from, &to).has_value()};
	// what the route between the ends found is against the design
	if (false == stored && ++Gone <= 5)
		WriteLog(format("SELFTEST profileaudit: no route for the design of %.1f m from %.2f %.2f %.2f to %.2f %.2f %.2f;%s", entry.length, entry.start.x, entry.start.y, entry.start.z, entry.end.x, entry.end.y, entry.end.z,
		                design_routes(entry.start, entry.end, entry.middle).c_str()));
	if (false == stored)
		return;
	m_build_tools->library.profile_open_stored(Index);
	// the design leaves the track by more than what it's read again from it for
	if (m_profile.departure > 0.02)
	{
		++Retaken;
		log_profile_off(entry.departure, from, to);
	}
	std::string errors;
	for (auto const &issue : m_profile.issues | std::views::filter([](auto const &Issue) { return Issue.error; }))
		errors += " | " + issue.text;
	if (false == errors.empty())
	{
		++Failing;
		WriteLog(format("SELFTEST profileaudit: %s .. %s, %.0f m, departure %.3f m; %s%s", from->name().c_str(), to->name().c_str(), m_profile.route.length, m_profile.departure, m_profile.status.c_str(), errors.c_str()));
	}
}

editor_mode::selftest_result editor_mode::selftest_profileaudit()
{
	// every design kept in the scenery opened as the profile opens it: how many have errors, how many were read from the track again
	m_build_tools->library.profile_library_load();
	std::size_t gone{0};
	std::size_t failing{0};
	std::size_t retaken{0};
	auto const count{m_profiles.size()};
	for (std::size_t i = 0; i < count; ++i)
		profile_audit(i, gone, retaken, failing);
	WriteLog(format("SELFTEST profileaudit: %zu designs, %zu with errors, %zu off the track by over 2 cm, %zu with no route", count, failing, retaken, gone));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_gradelines(std::istringstream &words) const
{
	// the shared grade lines to a file for the map of them: name, line, kind, points of the plan with the height
	std::string file;
	words >> file;
	std::ofstream stream(file);
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty() || (track->iCategoryFlag & 15) != 1)
			continue;
		auto const line{track_grade_line(*track)};
		for (auto const &path : track->m_paths)
		{
			stream << track->name() << ' ' << line << ' ' << (track->eType == tt_Switch ? 's' : 'n');
			stream_points(stream, path);
			stream << '\n';
		}
	}
	WriteLog("SELFTEST gradelines: " + file);
	return selftest_result::handled;
}

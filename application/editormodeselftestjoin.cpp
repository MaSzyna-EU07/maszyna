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
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "scene/sceneterrain.h"
#include "editor/editorSpeed.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <array>
#include <string>
#include <string_view>
#include <vector>
#include "application/editormodeselftest.h"
#include "application/editormodetrackshared.h"

namespace
{

// the height of the last point clicked, 0 before the first
double last_point_height(std::vector<glm::dvec3> const &Points)
{
	return Points.empty() ? 0.0 : Points.back().y;
}

// the share of the noise of the points of a test line, by the number of the point: one way, the other, a little
double noise_share(int const Count)
{
	if (Count % 3 == 0)
		return 1.0;
	if (Count % 3 == 1)
		return -0.6;
	return 0.2;
}

// a track an end of a test line is on, as the log names it
std::string end_name(TTrack const &Track)
{
	return Track.name() + (Track.eType == tt_Switch ? " (switch)" : "");
}

// the point of a path as the self-tests name it: c1, c2, end, anything else the start
editor_track::point_kind point_kind_named(std::string_view const Name)
{
	using enum editor_track::point_kind;
	if (Name == "c1")
		return control1;
	if (Name == "c2")
		return control2;
	if (Name == "end")
		return end;
	return start;
}

// whether the stretch of a line from the point A crosses one from the point B
bool stretches_cross(std::vector<glm::dvec2> const &Line, std::size_t const A, std::size_t const B)
{
	auto const r{Line[A + 1] - Line[A]};
	auto const s{Line[B + 1] - Line[B]};
	auto const denominator{r.x * s.y - r.y * s.x};
	if (std::abs(denominator) < 1e-12)
		return false;
	auto const q{Line[B] - Line[A]};
	auto const t{(q.x * s.y - q.y * s.x) / denominator};
	auto const u{(q.x * r.y - q.y * r.x) / denominator};
	return t > 1e-9 && t < 1.0 - 1e-9 && u > 1e-9 && u < 1.0 - 1e-9;
}

// whether a line crosses itself, the stretches next to each other left out
bool crosses_itself(std::vector<glm::dvec2> const &Line)
{
	for (std::size_t a = 0; a + 1 < Line.size(); ++a)
		for (std::size_t b = a + 3; b + 1 < Line.size(); ++b)
			if (stretches_cross(Line, a, b))
				return true;
	return false;
}

// the ring of the gizmo turning about the axis named
int ring_named(std::string_view const Axis)
{
	if (Axis == "x")
		return 0;
	if (Axis == "z")
		return 2;
	return 1;
}

// how far the far end of the second path of the switch is off the first as far along it, in its plane or on the vertical curve it lies
// on: a second path reaching past the end of the first against the grade the first ends with
double off_plane(TTrack const &Switch)
{
	geometry::bezier const main{Switch.m_paths[0]};
	geometry::bezier const branch{Switch.m_paths[1]};
	auto const chord{geometry::plan_of(main.point(1.0) - main.point(0.0))};
	auto const along{glm::dot(geometry::plan_of(branch.point(1.0) - main.point(0.0)), chord) / std::max(1e-9, glm::dot(chord, chord))};
	auto const last{main.first(1.0)};
	return branch.point(1.0).y - (main.point(std::clamp(along, 0.0, 1.0)).y + std::max(0.0, along - 1.0) * glm::length(chord) * last.y / std::max(1e-6, std::hypot(last.x, last.z)));
}

// each item kept in the square of the plan given and in the eight around it, the squares by their columns
template <typename Cells, typename Visit>
void each_in_cells_around(Cells const &Of, std::pair<int, int> const &Cell, Visit &&Seen)
{
	for (int dx = -1; dx <= 1; ++dx)
		for (int dz = -1; dz <= 1; ++dz)
			if (auto const other{Of.find({Cell.first + dx, Cell.second + dz})}; other != Of.end())
				std::ranges::for_each(other->second, [&Seen](auto const &Item) { Seen(Item); });
}

// the end of a path of the track nearest the point given, if nearer than the nearest so far: then the nearest and the point set to it
void nearer_end(TTrack const &Track, glm::dvec3 const &Given, double &Nearest, glm::dvec3 &Point)
{
	for (auto const &path : Track.m_paths)
		for (auto const index : {segment_data::point::start, segment_data::point::end})
			if (auto const distance{geometry::plan_distance(path.points[index], Given)}; distance < Nearest)
			{
				Nearest = distance;
				Point = path.points[index];
			}
}

// the free end 30 to 600 m from the point which is nearest the distance from it, none where none is
editor_track::snap_target end_in_band(glm::dvec3 const &First, double const Distance)
{
	editor_track::snap_target end;
	double best{1e9};
	for (auto const &candidate : editor_track::free_ends(nullptr, 1, First, 600.0, {}))
		if (auto const apart{geometry::plan_distance(candidate.position, First)}; apart >= 30.0 && std::abs(apart - Distance) < best)
		{
			best = std::abs(apart - Distance);
			end = candidate;
		}
	return end;
}

// the line the pieces make, every metre or so, with its headings, and the widest gap and the sharpest kink between the pieces
void trace_pieces(std::vector<segment_data> const &Pieces, std::vector<glm::dvec2> &Line, std::vector<glm::dvec2> &Headings, double &Gap, double &Kink)
{
	for (std::size_t p = 0; p < Pieces.size(); ++p)
	{
		geometry::bezier const curve{Pieces[p]};
		if (p > 0)
		{
			geometry::bezier const before{Pieces[p - 1]};
			Gap = std::max(Gap, glm::distance(before.point(1.0), curve.point(0.0)));
			Kink = std::max(Kink, glm::degrees(std::abs(geometry::signed_angle(geometry::plan_of(before.first(1.0)), geometry::plan_of(curve.first(0.0))))));
		}
		auto const span{curve.plan_length()};
		auto const steps{std::max(2, static_cast<int>(std::ceil(span / 1.0)))};
		for (int k = p == 0 ? 0 : 1; k <= steps; ++k)
		{
			auto const t{curve.parameter(span * k / steps)};
			Line.push_back(geometry::plan_of(curve.point(t)));
			Headings.push_back(geometry::plan_of(curve.first(t)));
		}
	}
}

// how much the line turns all along, and the least radius it turns at: 1e9 where it turns at none
std::pair<double, double> turning_of(std::vector<glm::dvec2> const &Line, std::vector<glm::dvec2> const &Headings)
{
	double turning{0.0};
	double sharpest{1e9};
	for (std::size_t k = 1; k < Headings.size(); ++k)
	{
		auto const angle{std::abs(geometry::signed_angle(Headings[k - 1], Headings[k]))};
		turning += angle;
		if (auto const step{glm::distance(Line[k - 1], Line[k])}; angle > 1e-6 && step > 1e-6)
			sharpest = std::min(sharpest, step / angle);
	}
	return {turning, sharpest};
}

// the faults named in the text counted
void count_faults(std::map<std::string, int, std::less<>> &Tally, std::string const &Faults)
{
	for (auto const *fault : {"start joint", "end joint", "pieces", "loop", "turning", "R "})
		if (Faults.find(fault) != std::string::npos)
			++Tally[fault];
}

// the track of the name, where there is one, added to the tracks
void add_named(std::vector<TTrack *> &Tracks, std::string const &Name)
{
	if (auto *each{simulation::Paths.find(Name)}; each != nullptr)
		Tracks.push_back(each);
}

// the paths of the track logged, their points and controls
void log_gizmo_paths(TTrack const &Each)
{
	for (int i = 0; i < static_cast<int>(Each.m_paths.size()); ++i)
	{
		auto const &path{Each.m_paths[i]};
		auto const &a{path.points[segment_data::point::start]};
		auto const &b{path.points[segment_data::point::end]};
		auto const &c1{path.points[segment_data::point::control1]};
		auto const &c2{path.points[segment_data::point::control2]};
		WriteLog(format("SELFTEST gizmo path %s[%d] %.3f %.3f %.3f -> %.3f %.3f %.3f, c1 %.3f %.3f %.3f, c2 %.3f %.3f %.3f, L %.3f, R %.1f, rolls %.2f %.2f", Each.name().c_str(), i, a.x, a.y, a.z, b.x, b.y, b.z, c1.x, c1.y, c1.z, c2.x, c2.y, c2.z,
		                geometry::bezier{path}.plan_length(), path.radius, path.rolls[0], path.rolls[1]));
	}
}

// the joints at the end of the path of the track logged: the kink and the gap to each path joined there
void log_gizmo_joints(TTrack &Track, int const Path, editor_track::point_kind const Kind)
{
	geometry::bezier const own{Track.m_paths[Path]};
	bool const start{Kind == editor_track::point_kind::start};
	auto const out{start ? -own.first(0.0) : own.first(1.0)};
	for (auto const &[partner_track, partner_point] : editor_track::partners(Track, {Path, Kind}))
	{
		geometry::bezier const theirs{partner_track->m_paths[partner_point.path]};
		auto const in{partner_point.kind == editor_track::point_kind::start ? theirs.first(0.0) : -theirs.first(1.0)};
		auto const angle{glm::degrees(std::acos(std::clamp(glm::dot(glm::normalize(out), glm::normalize(in)), -1.0, 1.0)))};
		auto const gap{glm::distance(editor_track::point_position(Track, {Path, Kind}), editor_track::point_position(*partner_track, partner_point))};
		WriteLog(format("SELFTEST gizmo joint %s[%d].%s - %s[%d]: kink %.4f deg, gap %.4f m", Track.name().c_str(), Path, start ? "start" : "end", partner_track->name().c_str(), partner_point.path, angle, gap));
	}
}

// the line of the log on the vertex of the plan of a line laid: where it is, as designed, and each curve of it as worked out
std::string layplan_vertex_text(alignment::design const &Design, alignment::result const &Shape, std::vector<char> const &Corners, std::size_t const k)
{
	auto const &vertex{Design.vertices[k]};
	auto text{format("SELFTEST layplan vertex %zu: %s at %.3f %.3f, designed R %.1f L %.1f/%.1f", k, Corners[k] != 0 ? "bend" : "traced", k < Shape.vertices.size() ? Shape.vertices[k].x : 0.0,
	                 k < Shape.vertices.size() ? Shape.vertices[k].y : 0.0, vertex.radius, vertex.transition_in, vertex.transition_out)};
	for (auto const &curve : Shape.curves)
		if (curve.vertex == static_cast<int>(k))
		{
			text += format(", %s R %.1f", curve.spirals ? "spirals" : "arc", curve.radius);
			std::ranges::for_each(curve.radii, [&text](auto const Radius) { text += format("/%.1f", Radius); });
			text += format(", L %.1f/%.1f, cant %.0f, deflection %.3f deg, tangents %.1f/%.1f, arc %.1f m, norms %d", curve.transition_in, curve.transition_out, curve.cant, glm::degrees(curve.deflection), curve.tangent_in, curve.tangent_out, curve.arc_length,
			               alignment::meets_norms(curve, Design) ? 1 : 0);
		}
	return text;
}

// every free end of the rails: where, the way out of it and the curvature of its track as it would go on beyond it
editor_selftest_result selftest_endcurves()
{
	for (auto const &end : editor_track::free_ends(nullptr, 1, glm::dvec3{0.0}, 1e7, {}))
	{
		auto const bend{editor_track::bend_at(*end.track, end.position)};
		auto const curvature{bend.has_value() ? bend->curvature : 0.0};
		WriteLog(format("SELFTEST freeend %s[%d].%s: %.3f %.3f %.3f, out %.6f %.6f, k %.6f (R %.1f), roll %.3f", end.track->name().c_str(), end.point.path, end.point.kind == editor_track::point_kind::end ? "end" : "start", end.position.x, end.position.y,
		                end.position.z, bend.has_value() ? bend->direction.x : 0.0, bend.has_value() ? bend->direction.y : 0.0, curvature, std::abs(curvature) > 1e-9 ? 1.0 / std::abs(curvature) : 0.0, bend.has_value() ? bend->roll : 0.0));
	}
	return editor_selftest_result::handled;
}

// an end of a path meeting the point of curvejoint
struct curve_meeting
{
	TTrack const *track;
	int path;
	bool atend;
	double curvature; // as the path goes away from the point
	double roll;
	glm::dvec2 out;
};

// the end of the path of the track, where it meets the point, added to the meetings
void meet_at(TTrack const *Track, int const I, bool const Atend, glm::dvec3 const &Point, std::vector<curve_meeting> &Meetings)
{
	auto const &path{Track->m_paths[I]};
	if (geometry::plan_distance(path.points[Atend ? segment_data::point::end : segment_data::point::start], Point) > 0.05)
		return;
	geometry::bezier const curve{path};
	auto const first{curve.first(Atend ? 1.0 : 0.0)};
	auto const second{curve.second(Atend ? 1.0 : 0.0)};
	auto const speed{std::hypot(first.x, first.z)};
	if (speed < 1e-9)
		return;
	auto const curvature{(first.x * second.z - first.z * second.x) / (speed * speed * speed)};
	glm::dvec2 const along{first.x / speed, first.z / speed};
	Meetings.emplace_back(Track, I, Atend, Atend ? -curvature : curvature, static_cast<double>(Atend ? path.rolls[1] : path.rolls[0]), Atend ? -along : along);
}

// the ends of the paths meeting at the point
std::vector<curve_meeting> meetings_at(glm::dvec3 const &Point)
{
	std::vector<curve_meeting> meetings;
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
		{
			meet_at(track, i, false, Point, meetings);
			meet_at(track, i, true, Point, meetings);
		}
	}
	return meetings;
}

// the meeting logged, its curvature signed the way the line runs into the point, and kept with the ones before the point or after it
void log_meeting(curve_meeting const &Each, glm::dvec2 const &Into, std::vector<std::pair<double, glm::dvec2>> &Before, std::vector<std::pair<double, glm::dvec2>> &After)
{
	auto const leaving{glm::dot(Each.out, Into) > 0.0};
	auto const curvature{leaving ? Each.curvature : -Each.curvature};
	(leaving ? After : Before).emplace_back(curvature, leaving ? Each.out : -Each.out);
	WriteLog(format("SELFTEST curvejoint %s[%d].%s: %s, k %.6f (R %.1f), roll %.3f", Each.track->name().c_str(), Each.path, Each.atend ? "end" : "start", leaving ? "after" : "before", curvature, std::abs(curvature) > 1e-9 ? 1.0 / std::abs(curvature) : 0.0, Each.roll));
}

// the paths meeting at the point: the curvature of each, signed the way the first one runs into the point, and the jumps of the
// curvature and of the direction from the ones before the point to the ones after it
editor_selftest_result selftest_curvejoint(std::istringstream &words)
{
	glm::dvec3 point{0.0};
	words >> point.x >> point.z;
	// the end of a path nearest the point, within 3 m
	auto nearest{3.0};
	auto const given{point};
	for (auto const *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved)
			nearer_end(*track, given, nearest, point);
	auto const meetings{meetings_at(point)};
	if (meetings.empty())
	{
		WriteLog(format("SELFTEST curvejoint %.1f %.1f: no path ends there", point.x, point.z));
		return editor_selftest_result::handled;
	}
	auto const into{-meetings.front().out};
	std::vector<std::pair<double, glm::dvec2>> before;
	std::vector<std::pair<double, glm::dvec2>> after;
	for (auto const &each : meetings)
		log_meeting(each, into, before, after);
	for (auto const &[k0, d0] : before)
		for (auto const &[k1, d1] : after)
			WriteLog(format("SELFTEST curvejoint jump: k %.6f -> %.6f, by %.6f 1/m (%.1f%%), kink %.4f deg", k0, k1, k1 - k0, std::abs(k0) > 1e-9 ? 100.0 * std::abs(k1 - k0) / std::abs(k0) : 0.0, glm::degrees(std::abs(geometry::signed_angle(d0, d1)))));
	return editor_selftest_result::handled;
}

// the tracks are laid to the norms only
editor_selftest_result selftest_norms()
{
	WriteLog("SELFTEST norms: plk");
	return editor_selftest_result::handled;
}

// how the heights of the tracks laid are set: fitted to the ground, at the points clicked, level
editor_selftest_result selftest_layheights(std::istringstream &words)
{
	std::string which;
	words >> which;
	auto heights{editorSettings::lay_heights::fitted};
	if (which == "points")
		heights = editorSettings::lay_heights::points;
	else if (which == "level")
		heights = editorSettings::lay_heights::level;
	EditorSettings.heights(heights);
	WriteLog("SELFTEST layheights: " + which);
	return editor_selftest_result::handled;
}

} // namespace

editor_mode::selftest_result editor_mode::selftest_join_or_lay_to(std::string const &command, std::istringstream &words)
{
	glm::dvec3 a{0.0};
	glm::dvec3 b{0.0};
	words >> a.x >> a.z >> b.x >> b.z;
	double radius{0.0};
	int variant{0};
	int transitions{0};
	double height{0.0};
	if (command == "join")
	{
		// the transitions word left from the free laying: they follow the norms
		words >> radius >> variant >> transitions;
		// of the ends, for sceneries far above zero
		if (words >> height)
		{
			b.y = height;
			a.y = height;
		}
	}
	auto const from{editor_track::find_free_end(nullptr, 1, a, 3.0, {})};
	extend_tool tool;
	if (from.track == nullptr || false == free_end(*from.track, from.point.path, from.point.kind == editor_track::point_kind::end, tool))
	{
		WriteLog("SELFTEST " + command + ": no free end at the first point");
		return selftest_result::handled;
	}
	std::vector<segment_data> pieces;
	std::string error;
	double length{0.0};
	if (command == "join")
	{
		auto const to{editor_track::find_free_end(nullptr, 1, b, 3.0, {from.track})};
		if (to.track == nullptr)
		{
			WriteLog("SELFTEST join: no free end at the second point");
			return selftest_result::handled;
		}
		m_join_radius = radius;
		m_join_variant = variant;
		pieces = join_pieces(tool.track, from.point, tool.point, tool.direction, to, length, error);
	}
	else
	{
		int layout{0};
		words >> layout >> m_lay.radius;
		m_track_layout = static_cast<track_layout>(std::clamp(layout, 0, 3));
		m_extend = tool;
		m_extend.active = true;
		m_extend.mouse = {b.x, tool.point.y, b.z};
		m_extend.snap = {};
		pieces = extend_pieces();
		m_extend.active = false;
	}
	std::vector<TTrack *> created;
	for (auto const &piece : pieces)
		created.push_back(editor_track::create_path(*tool.track, piece));
	editor_track::commit(created);
	digitize_drawn(created);
	if (false == created.empty())
		push_track_snapshot({}, created);
	constexpr std::array<char const *, 6> kinds{"none", "straight", "arc", "curve", "reverse curve", "free curve"};
	auto const &join{m_join_last};
	WriteLog(format("SELFTEST %s: %s, %zu paths, R %.1f m, cant %.0f mm, transitions %.1f m, V %.0f km/h, variants %d%s", command.c_str(), kinds[static_cast<int>(join.kind)], pieces.size(), join.radius, join.cant,
	                join.transition, join.speed, join.variants, error.empty() ? "" : (", " + error).c_str()));
	auto heading{tool.direction};
	glm::dvec3 previous{tool.point};
	for (auto const *track : created)
	{
		auto const &path{track->m_paths.front()};
		geometry::bezier const curve{path};
		auto const begin{glm::normalize(geometry::plan_of(curve.first(0.0)))};
		auto const finish{glm::normalize(geometry::plan_of(curve.first(1.0)))};
		WriteLog(format("SELFTEST %s path %s: L %.2f m, R %.1f m, rolls %.2f %.2f, kink %.4f deg, step %.4f m", command.c_str(), track->name().c_str(), curve.plan_length(), static_cast<double>(path.radius),
		                static_cast<double>(path.rolls[0]), static_cast<double>(path.rolls[1]), glm::degrees(std::abs(geometry::signed_angle(heading, begin))), glm::distance(previous, path.points[segment_data::point::start])));
		heading = finish;
		previous = path.points[segment_data::point::end];
	}
	if (false == created.empty())
		WriteLog(format("SELFTEST %s end: %.3f m from the target", command.c_str(), geometry::plan_distance(previous, b)));
	return selftest_result::handled;
}

namespace
{

// an end of a path of fitcheck, where it may meet another as a joint
struct fit_joint_end
{
	TTrack const *track;
	int path;
	glm::dvec3 point;
	double grade; // rising out of the joint
};

// how high the rails of fitcheck lie over the ground, the least and the most, of the plain tracks and of the switches
struct fit_heights
{
	double track_low{1e9};
	double track_high{-1e9};
	double switch_low{1e9};
	double switch_high{-1e9};
};

// the joints of fitcheck counted, their worst step and their worst break of the grade
struct fit_joints
{
	int count{0};
	double worst_step{0.0};
	double worst_kink{0.0};
};

// the places of the paths of the tracks by the squares of 10 m of the plan: where, the way the path goes, the track
using fit_places = std::map<std::pair<int, int>, std::vector<std::tuple<glm::dvec3, glm::dvec2, TTrack const *>>>;

// the pairs of places of fitcheck side by side, how many are over 5 cm apart in height and the most they are
struct fit_beside
{
	double worst{0.0};
	std::size_t pairs{0};
	std::size_t off{0};
};

// whether the track is one of the plain rails fitcheck looks at
bool fit_counted(TTrack const *Track)
{
	return Track != nullptr && false == Track->m_editorremoved && false == Track->m_paths.empty() && (Track->iCategoryFlag & 15) == 1;
}

// the ends of the paths of the track kept, the height of its rails over the ground taken along them by the clearance given, and the
// switch off its plane counted as a fault
template <typename Clearance>
void fit_track(TTrack const &Track, std::string const &Label, Clearance const &Clear, std::vector<fit_joint_end> &Ends, fit_heights &Heights, int &Faults)
{
	auto const isswitch{Track.eType == tt_Switch};
	auto &low{isswitch ? Heights.switch_low : Heights.track_low};
	auto &high{isswitch ? Heights.switch_high : Heights.track_high};
	for (int index = 0; index < static_cast<int>(Track.m_paths.size()); ++index)
	{
		geometry::bezier const curve{Track.m_paths[index]};
		auto const grade_at = [&curve](double const T, double const Sign) {
			auto const first{curve.first(T)};
			return Sign * first.y / std::max(1e-6, std::hypot(first.x, first.z));
		};
		Ends.emplace_back(&Track, index, curve.point(0.0), grade_at(0.0, 1.0));
		Ends.emplace_back(&Track, index, curve.point(1.0), grade_at(1.0, -1.0));
		for (int tenth = 0; tenth < 10; ++tenth)
			Clear(curve.point(0.05 + 0.1 * tenth), low, high);
	}
	// the far end of the second path against the first as far along it, in its plane or on the vertical curve it lies on
	if (auto const off{isswitch && Track.m_paths.size() >= 2 ? off_plane(Track) : 0.0}; std::abs(off) > 0.03)
	{
		++Faults;
		WriteLog(format("SELFTEST fitcheck %s FAULT switch %s off its plane by %.3f m", Label.c_str(), Track.name().c_str(), off));
	}
}

// the two ends, where they meet as a joint of two tracks: its step and its break of the grade
void fit_joint_of(fit_joint_end const &A, fit_joint_end const &B, std::string const &Label, fit_joints &Joints, int &Faults)
{
	if (A.track == B.track || false == (geometry::plan_distance(A.point, B.point) < 0.05))
		return;
	++Joints.count;
	auto const step{std::abs(A.point.y - B.point.y)};
	// the same grade seen from the other side has the other sign
	auto const kink{std::abs(A.grade + B.grade) * 1000.0};
	Joints.worst_step = std::max(Joints.worst_step, step);
	Joints.worst_kink = std::max(Joints.worst_kink, kink);
	if (step > 0.01 || kink > 3.0)
	{
		++Faults;
		WriteLog(format("SELFTEST fitcheck %s FAULT joint %s / %s at %.1f %.1f: step %.3f m, grade break %.1f permille", Label.c_str(), A.track->name().c_str(), B.track->name().c_str(), A.point.x, A.point.z, step, kink));
	}
}

// the places of the paths of the track, every 5 m or so
void fit_places_of(fit_places &Beside, TTrack const *Track)
{
	for (auto const &path : Track->m_paths)
	{
		geometry::bezier const curve{path};
		auto const places{std::max(1, static_cast<int>(geometry::chord_length(path) / 5.0))};
		for (int i = 0; i < places; ++i)
		{
			auto const place{curve.point((i + 0.5) / places)};
			Beside[{static_cast<int>(std::floor(place.x / 10.0)), static_cast<int>(std::floor(place.z / 10.0))}].emplace_back(place, geometry::plan_of(curve.first((i + 0.5) / places)), Track);
		}
	}
}

// the place with the places of the other tracks beside it, running the same way
void fit_pairs_of(fit_places const &Beside, std::pair<int, int> const &Cell, glm::dvec3 const &Place, glm::dvec2 const &Way, TTrack const *Track, fit_beside &Tally)
{
	each_in_cells_around(Beside, Cell, [&](auto const &Next) {
		auto const &[next_place, next_way, next_track]{Next};
		if (next_track == Track || geometry::plan_distance(next_place, Place) >= 5.1 || std::abs(glm::dot(Way, next_way)) < 0.985 * glm::length(Way) * glm::length(next_way))
			return;
		auto const difference{std::abs(next_place.y - Place.y)};
		if (difference > 2.0)
			return;
		++Tally.pairs;
		Tally.off += difference > 0.05 ? 1 : 0;
		Tally.worst = std::max(Tally.worst, difference);
	});
}

// the tracks side by side under 5.1 m apart against one grade line, tracks over 2 m apart in height lying on another level
fit_beside fit_side_by_side()
{
	fit_places beside;
	for (auto const *track : simulation::Paths.sequence())
		if (fit_counted(track))
			fit_places_of(beside, track);
	fit_beside tally;
	for (auto const &[cell, places] : beside)
		for (auto const &[place, way, track] : places)
			fit_pairs_of(beside, cell, place, way, track, tally);
	return tally;
}

} // namespace

// how the tracks and switches lie on the ground and on each other: steps and kinks of the grade at the joints, the two paths of a
// switch off one plane, the top of the rails against the ground
editor_mode::selftest_result editor_mode::selftest_fitcheck(std::istringstream &words) const
{
	std::string label;
	words >> label;
	std::vector<fit_joint_end> ends;
	auto const clearance = [this](glm::dvec3 const &Point, double &Low, double &High) {
		std::vector<char> found;
		auto const ground{ground_heights({glm::dvec3{Point.x, 5000.0, Point.z}}, true, &found).front()};
		if (found.front() != 0)
		{
			Low = std::min(Low, Point.y - ground);
			High = std::max(High, Point.y - ground);
		}
	};
	fit_heights heights;
	int switches{0};
	int faults{0};
	for (auto const *track : simulation::Paths.sequence())
	{
		if (false == fit_counted(track))
			continue;
		switches += track->eType == tt_Switch ? 1 : 0;
		fit_track(*track, label, clearance, ends, heights, faults);
	}
	fit_joints joints;
	for (std::size_t a = 0; a < ends.size(); ++a)
		for (std::size_t b = a + 1; b < ends.size(); ++b)
			fit_joint_of(ends[a], ends[b], label, joints, faults);
	auto const beside{fit_side_by_side()};
	WriteLog(format("SELFTEST fitcheck %s beside: %zu pairs of places under 5.1 m apart, %zu over 5 cm apart in height, worst %.3f m", label.c_str(), beside.pairs, beside.off, beside.worst));
	WriteLog(format("SELFTEST fitcheck %s: %d switches, %d joints, %d faults, worst step %.3f m, grade break %.1f permille; rails over the ground: tracks %.2f..%.2f m, switches %.2f..%.2f m", label.c_str(), switches, joints.count, faults, joints.worst_step, joints.worst_kink,
	                heights.track_low, heights.track_high, heights.switch_low, heights.switch_high));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_cantcheck(std::istringstream &words) const
{
	// the cant at every joint of plain tracks, in mm across the gauge, a jump logged as a fault, and the steepest ramp of a path
	std::string label;
	words >> label;
	auto const gauge{m_route.design.norms.gauge};
	auto const mm = [&](double const Roll) { return gauge * std::sin(glm::radians(Roll)); };
	struct joint_end
	{
		TTrack const *track;
		glm::dvec3 point;
		double roll; // as the path runs out of the joint
	};
	std::vector<joint_end> ends;
	auto steepest{0.0};
	std::string steepest_name;
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty())
			continue;
		auto const &path{track->m_paths.front()};
		ends.emplace_back(track, path.points[segment_data::point::start], path.rolls[0]);
		ends.emplace_back(track, path.points[segment_data::point::end], -path.rolls[1]);
		auto const length{std::max(0.01, geometry::chord_length(path))};
		if (auto const ramp{std::abs(mm(path.rolls[1]) - mm(path.rolls[0])) / length}; ramp > steepest)
		{
			steepest = ramp;
			steepest_name = track->name();
		}
	}
	int joints{0};
	int faults{0};
	// the two ends, where they meet as a joint of two tracks: the jump of the cant there
	auto const joint_of = [&](joint_end const &A, joint_end const &B) {
		if (A.track == B.track || false == (geometry::plan_distance(A.point, B.point) < 0.05))
			return;
		++joints;
		// the same cant seen from the other side has the other sign
		auto const jump{std::abs(mm(A.roll) + mm(B.roll))};
		if (jump > 1.0)
		{
			++faults;
			WriteLog(format("SELFTEST cantcheck %s FAULT %s / %s at %.2f %.2f: %.1f mm against %.1f mm, jump %.1f mm", label.c_str(), A.track->name().c_str(), B.track->name().c_str(), A.point.x, A.point.z, mm(A.roll), -mm(B.roll), jump));
		}
	};
	for (std::size_t a = 0; a < ends.size(); ++a)
		for (std::size_t b = a + 1; b < ends.size(); ++b)
			joint_of(ends[a], ends[b]);
	WriteLog(format("SELFTEST cantcheck %s: %d joints, %d faults, steepest ramp %.2f mm/m on %s", label.c_str(), joints, faults, steepest, steepest_name.c_str()));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_fittime(std::istringstream &words) const
{
	// the grade line of a straight laid over rolling ground fitted, as at a click, and as while the track follows the
	// cursor: the frame then waits only for the ground to be sampled
	for (double length{0.0}; words >> length;)
	{
		auto const ground = [](double const X) { return 130.0 + 4.0 * std::sin(X / 180.0) + 1.5 * std::sin(X / 37.0) + 0.2 * std::sin(X / 6.0); };
		auto const bed = [&ground](std::vector<glm::dvec3> const &Points, std::vector<glm::dvec2> const &) {
			std::vector<double> heights;
			for (auto const &point : Points)
				heights.push_back(ground(point.x));
			return heights;
		};
		auto const pieces{geometry::straight_piece({0.0, ground(0.0), 0.0}, {1.0, 0.0}, 0.0, length)};
		ground_follow::ends const ends{ground(0.0), std::numeric_limits<double>::quiet_NaN(), ground(length), std::numeric_limits<double>::quiet_NaN()};
		auto const grading{laying_grading(nullptr)};
		auto fitted{pieces};
		auto const start{std::chrono::steady_clock::now()};
		ground_follow::follow(fitted, ends, bed, grading);
		auto const fit{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()};
		ground_follow::fitter fitter;
		auto shown{pieces};
		auto const asked{std::chrono::steady_clock::now()};
		ground_follow::follow_meanwhile(shown, ends, bed, grading, fitter);
		auto const meanwhile{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - asked).count()};
		WriteLog(format("SELFTEST fittime %.0f m: fitted in %.1f ms, %zu pieces; while laid %.2f ms", length, fit, fitted.size(), meanwhile));
	}
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_layspeed(std::istringstream &words)
{
	int own{0};
	words >> m_lay.style.velocity >> own;
	m_lay.own_speed = own != 0;
	WriteLog(format("SELFTEST layspeed: %.0f km/h, own %d", m_lay.style.velocity, m_lay.own_speed ? 1 : 0));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_laymode(std::istringstream &words)
{
	std::string which;
	words >> which;
	if ((which == "curve") != (m_lay.mode == lay_mode::curve))
		lay_switch_mode();
	WriteLog("SELFTEST laymode: " + std::string{m_lay.mode == lay_mode::curve ? "curve" : "straights"});
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_laytrace(std::istringstream &words)
{
	int straights{1};
	words >> m_lay.radii >> straights;
	m_lay.straights_between = straights != 0;
	WriteLog(format("SELFTEST laytrace: radii %d, straights %d", m_lay.radii, straights));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_layp(std::istringstream &words)
{
	m_lay.active = true;
	for (glm::dvec3 point{0.0}; words >> point.x >> point.z;)
		selftest_lay_point(point);
	WriteLog(format("SELFTEST layp: %zu points, %s", m_lay.points.size(), m_lay.status.c_str()));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_layreport(std::istringstream &words)
{
	std::string label;
	words >> label;
	selftest_lay_report(label);
	return selftest_result::handled;
}

namespace
{

// the walk of laywalk: where it ends, how it heads there, how long it is, and its points every so many metres with the way across them
struct lay_walk
{
	glm::dvec2 position;
	double heading;
	double chainage{0.0};
	std::vector<glm::dvec2> along;
	std::vector<glm::dvec2> across;
};

// the walk from the point and the heading given along the segments of the curvature going evenly from the first to the second over
// the length, turned the way the sense gives, its points kept every Spacing metres
lay_walk walk_segments(glm::dvec2 const &Position, double const Heading, double const Sense, double const Spacing, std::vector<std::array<double, 3>> const &Segments)
{
	lay_walk walk{Position, Heading};
	auto next{Spacing};
	// a point of the walk kept where it has come the spacing on from the one before, across it the way it heads there
	auto const keep_point = [&walk, &next, Spacing](auto const &Angle, double const At) {
		if (false == (walk.chainage >= next - 1e-6))
			return;
		auto const now{Angle(At)};
		walk.along.push_back(walk.position);
		walk.across.push_back({-std::sin(now), std::cos(now)});
		next += Spacing;
	};
	for (auto const &[from, to, length] : Segments)
	{
		auto const steps{std::max(1, static_cast<int>(std::ceil(length / 0.05)))};
		auto const step{length / steps};
		auto const start{walk.heading};
		auto const angle = [start, from = from * Sense, to = to * Sense, length = length](double const S) { return start + from * S + (to - from) * S * S / (2.0 * length); };
		for (int i = 0; i < steps; ++i)
		{
			auto const middle{angle((i + 0.5) * step)};
			walk.position += glm::dvec2{std::cos(middle), std::sin(middle)} * step;
			walk.chainage += step;
			keep_point(angle, (i + 1) * step);
		}
		walk.heading = angle(length);
	}
	return walk;
}

} // namespace

// a line of known curves going on from the free end nearest the point, within 60 m: segments of the curvature going evenly from the
// first to the second (1/m, positive turning the way the track of the end turns, to the left from a straight one) over the length. from: the free end
// clicked, then in the mode of a curve the points along the line every Spacing metres and two on the straight after it, 30 and 60 m
// on, left for Enter; into: the same line laid towards the free end, the point 60 m on clicked first, the one 30 m on and the ones
// along the line in the mode of a curve, the free end last, which lays it. The points shifted across by up to Noise
editor_mode::selftest_result editor_mode::selftest_laywalk(std::istringstream &words)
{
	std::string way;
	glm::dvec3 around{0.0};
	double spacing{10.0};
	double noise{0.0};
	words >> way >> around.x >> around.z >> spacing >> noise;
	std::vector<std::array<double, 3>> segments;
	for (std::array<double, 3> segment; words >> segment[0] >> segment[1] >> segment[2];)
		segments.push_back(segment);
	auto const end{editor_track::find_free_end(nullptr, 1, around, 60.0, {})};
	auto const bend{end.track != nullptr ? editor_track::bend_at(*end.track, end.position) : std::nullopt};
	if (false == bend.has_value() || segments.empty() || spacing <= 0.0)
	{
		WriteLog("SELFTEST laywalk: no free end near the point, or no line");
		return selftest_result::handled;
	}
	auto const walk{walk_segments({end.position.x, end.position.z}, std::atan2(bend->direction.y, bend->direction.x), bend->curvature < 0.0 ? -1.0 : 1.0, spacing, segments)};
	auto const &position{walk.position};
	auto const heading{walk.heading};
	auto const chainage{walk.chainage};
	auto const &along{walk.along};
	auto const &across{walk.across};
	glm::dvec2 const out{std::cos(heading), std::sin(heading)};
	glm::dvec2 const side{-out.y, out.x};
	auto const shifted = [&](glm::dvec2 const &Point, glm::dvec2 const &Across, int const Count) {
		auto const shift{noise * noise_share(Count)};
		return glm::dvec3{Point.x + Across.x * shift, 0.0, Point.y + Across.y * shift};
	};
	WriteLog(format("SELFTEST laywalk %s from %s[%d]: %.3f %.3f heading %.4f, k %.6f, line %.1f m to %.3f %.3f heading %.4f, %zu points along it", way.c_str(), end.track->name().c_str(), end.point.path, end.position.x, end.position.z, std::atan2(bend->direction.y, bend->direction.x),
	                bend->curvature, chainage, position.x, position.y, heading, along.size()));
	m_lay.active = true;
	lay_cancel();
	if (m_lay.mode != lay_mode::straights)
		lay_switch_mode();
	if (way == "into")
	{
		selftest_lay_point(shifted(position + out * 60.0, side, 0));
		lay_switch_mode();
		selftest_lay_point(shifted(position + out * 30.0, side, 1));
		// a point along the walk clicked, unless it's within the reach of the snap of the end
		auto const lay_off_end = [&](std::size_t const I) {
			if (glm::distance(along[I], glm::dvec2{end.position.x, end.position.z}) > 3.5)
				selftest_lay_point(shifted(along[I], across[I], static_cast<int>(I)));
		};
		for (auto i{along.size()}; i-- > 0;)
			lay_off_end(i);
		selftest_lay_point(end.position);
		WriteLog(format("SELFTEST laywalk into: %zu points, %s", m_lay.points.size(), m_lay.status.c_str()));
	}
	else
	{
		selftest_lay_point(end.position);
		lay_switch_mode();
		for (std::size_t i = 0; i < along.size(); ++i)
			selftest_lay_point(shifted(along[i], across[i], static_cast<int>(i)));
		selftest_lay_point(shifted(position + out * 30.0, side, 1));
		selftest_lay_point(shifted(position + out * 60.0, side, 2));
		WriteLog(format("SELFTEST laywalk from: %zu points, start %s", m_lay.points.size(), m_lay.start.track != nullptr ? m_lay.start.track->name().c_str() : "-"));
	}
	lay_switch_mode();
	return selftest_result::handled;
}

// the bends of straights from the free end nearest the point, within 60 m: the free end clicked, then each point the distance on
// along the way it goes, which then turns by the angle (degrees, positive the way the track of the end turns, to the left from a
// straight one); the last one the end, left for Enter
editor_mode::selftest_result editor_mode::selftest_laybends(std::istringstream &words)
{
	glm::dvec3 around{0.0};
	words >> around.x >> around.z;
	// the free end nearest the point on the plan, whatever the height of the scenery
	editor_track::snap_target end;
	auto nearest{60.0};
	for (auto const &free : editor_track::free_ends(nullptr, 1, glm::dvec3{0.0}, 1e7, {}))
		if (auto const distance{glm::distance(glm::dvec2{free.position.x, free.position.z}, glm::dvec2{around.x, around.z})}; distance < nearest)
		{
			nearest = distance;
			end = free;
		}
	auto const bend{end.track != nullptr ? editor_track::bend_at(*end.track, end.position) : std::nullopt};
	if (false == bend.has_value())
	{
		WriteLog("SELFTEST laybends: no free end near the point");
		return selftest_result::handled;
	}
	m_lay.active = true;
	lay_cancel();
	if (m_lay.mode != lay_mode::straights)
		lay_switch_mode();
	selftest_lay_point(end.position);
	glm::dvec2 position{end.position.x, end.position.z};
	auto heading{std::atan2(bend->direction.y, bend->direction.x)};
	for (double distance{0.0}; words >> distance;)
	{
		position += glm::dvec2{std::cos(heading), std::sin(heading)} * distance;
		selftest_lay_point({position.x, 0.0, position.y});
		if (double turn{0.0}; words >> turn)
			heading += glm::radians(turn) * (bend->curvature < 0.0 ? -1.0 : 1.0);
	}
	WriteLog(format("SELFTEST laybends from %s[%d] %.3f %.3f: %zu points", end.track->name().c_str(), end.point.path, end.position.x, end.position.z, m_lay.points.size()));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_lay_key(std::string const &command)
{
	m_typed.clear();
	m_lay.mouse_snap = {};
	if (command == "layenter")
		lay_enter();
	else if (command == "layback")
		lay_back();
	else
		lay_cancel();
	WriteLog(format("SELFTEST %s: %zu points, history %zu, %s", command.c_str(), m_lay.points.size(), m_history.size(), m_lay.status.c_str()));
	return selftest_result::handled;
}

// a line of known curves, turning by the deflections given (degrees, to the left positive), the points along its curves clicked as
// they would be on the photo: the start as a bend, then in the mode of a curve from 30 m before the first curve to the end of the
// last one and two on the straight after it, every Spacing metres, shifted across by up to Noise
editor_mode::selftest_result editor_mode::selftest_laysample(std::istringstream &words)
{
	double x{0.0};
	double z{0.0};
	double heading{0.0};
	double spacing{20.0};
	double noise{0.0};
	double deflection{30.0};
	double radius{1000.0};
	words >> x >> z >> heading >> spacing >> noise >> deflection >> radius;
	double apart{0.0};
	double second{0.0};
	double secondradius{0.0};
	auto const two{static_cast<bool>(words >> apart >> second >> secondradius)};
	alignment::design truth;
	truth.norms = laying_norms();
	truth.shape = m_route.design.shape;
	truth.speed = design_speed(nullptr, nullptr);
	glm::dvec2 const start{x, z};
	glm::dvec2 direction{std::cos(glm::radians(heading)), std::sin(glm::radians(heading))};
	auto const corner1{start + direction * 400.0};
	direction = geometry::turned(direction, glm::radians(deflection));
	auto corner2{corner1};
	if (two)
	{
		corner2 = corner1 + direction * apart;
		direction = geometry::turned(direction, glm::radians(second));
	}
	auto const end{corner2 + direction * 400.0};
	truth.start = {start.x, 0.0, start.y};
	truth.end = {end.x, 0.0, end.y};
	truth.start_direction = glm::normalize(corner1 - start);
	truth.end_direction = direction;
	auto const curve_of = [&](double const Radius) {
		alignment::vertex vertex;
		vertex.radius = Radius;
		auto const recommended{alignment::recommend(std::min(truth.speed, alignment::speed_limit(Radius, truth.norms)), Radius, truth.norms)};
		vertex.transition_out = recommended.transition;
		vertex.transition_in = vertex.transition_out;
		vertex.cant = recommended.cant;
		vertex.offset = 400.0;
		return vertex;
	};
	truth.vertices.push_back(curve_of(radius));
	if (two)
		truth.vertices.push_back(curve_of(secondradius));
	auto const result{alignment::compute(truth)};
	for (auto const &curve : result.curves)
		WriteLog(format("SELFTEST laysample truth %d: R %.1f, L %.1f/%.1f, cant %.0f, deflection %.3f deg, valid %d", curve.vertex, curve.radius, curve.transition_in, curve.transition_out, curve.cant, glm::degrees(curve.deflection), result.valid ? 1 : 0));
	if (false == result.valid)
		return selftest_result::handled;
	auto from{result.length};
	auto to{0.0};
	for (auto const &e : result.elements)
		if (e.kind != alignment::element_kind::straight)
		{
			from = std::min(from, e.chainage);
			to = std::max(to, e.chainage + e.length);
		}
	m_lay.active = true;
	lay_cancel();
	if (m_lay.mode != lay_mode::straights)
		lay_switch_mode();
	auto const along = [&](double const Chainage, double const Shift) {
		auto const station{alignment::evaluate(result, Chainage)};
		glm::dvec3 const across{-station.direction.z, 0.0, station.direction.x};
		return station.position + glm::normalize(across) * Shift;
	};
	selftest_lay_point(truth.start);
	lay_switch_mode();
	auto const first{std::max(0.0, from - 30.0)};
	for (int count = 0; first + count * spacing <= to; ++count)
		selftest_lay_point(along(first + count * spacing, noise * noise_share(count)));
	selftest_lay_point(along(to + 30.0, noise * 0.5));
	selftest_lay_point(along(to + 60.0, -noise * 0.5));
	lay_switch_mode();
	WriteLog(format("SELFTEST laysample: %zu points clicked over %.1f m of curves", m_lay.points.size(), to - from));
	selftest_lay_report("sample");
	return selftest_result::handled;
}

namespace
{

// what laydrag is given: whether the line is shown whole, where the arc starts, how it heads and turns, the points clicked on it, and
// how the cursor moves on along it
struct lay_drag_setup
{
	int whole{1};
	int count{6};
	int moves{20};
	int holds{40};
	int legs{4};
	double x{0.0};
	double z{0.0};
	double heading{0.0};
	double radius{600.0};
	double spacing{20.0};
	double step{0.5};
	double noise{0.0};

	// the point of the arc the chainage on, shifted across it outwards
	glm::dvec3 along(double const Chainage, double const Shift) const
	{
		glm::dvec2 const start{x, z};
		glm::dvec2 const direction{std::cos(glm::radians(heading)), std::sin(glm::radians(heading))};
		glm::dvec2 const leftward{-direction.y, direction.x};
		auto const centre{start + leftward * radius};
		auto const angle{Chainage / radius};
		auto const radial{geometry::turned(start - centre, angle)};
		auto const point{centre + radial * (1.0 + Shift / radius)};
		return glm::dvec3{point.x, 0.0, point.y};
	}
	// how far across the point of the count is shifted
	double shift(int const Count) const
	{
		return noise * noise_share(Count);
	}
};

// laydrag from frame to frame: the cursor of each frame and the leg it is in, the line shown, what was counted of the leg the frame
// before was in and of all the legs
struct lay_drag_state
{
	bool started{false};
	std::vector<glm::dvec3> cursor; // frame after frame
	std::vector<int> legs; // of the frames, minus for the ones held still
	std::size_t frame{0};
	std::array<double, 4> shown{};
	int changes{0};
	int partial{0};
	// of the leg the frame before was in
	int leg{0};
	int leg_changes{0};
	int leg_partial{0};
	int leg_frames{0};
	int leg_last{0};
	// over all of them
	int moved_changes{0};
	int moved_partial{0};
	int moved_frames{0};
	int still_lines{0};
	int still_most{0};
	int still_last{0};
	int still_slowest{0};
	int stills{0};

	// the cursor of the legs of the drag, each moving on along the arc from the chainage given, then held where it got
	void plan(lay_drag_setup const &Setup, double Chainage)
	{
		for (int each = 1; each <= Setup.legs; ++each)
		{
			for (int i = 0; i < Setup.moves; ++i)
			{
				Chainage += Setup.step;
				cursor.push_back(Setup.along(Chainage, 0.3 * Setup.shift(i)));
				legs.push_back(each);
			}
			for (int i = 0; i < Setup.holds; ++i)
			{
				cursor.push_back(cursor.back());
				legs.push_back(-each);
			}
		}
	}
	// the leg the frame before was in logged and added up, the one given begun
	void close_leg(int const Leg, std::string const &Error)
	{
		if (leg > 0)
		{
			moved_changes += leg_changes;
			moved_partial += leg_partial;
			moved_frames += leg_frames;
			WriteLog(format("SELFTEST laydrag leg %d moving: %d frames, the line changed %d times, %d frames of a line not worked out whole", leg, leg_frames, leg_changes, leg_partial));
		}
		else if (leg < 0)
		{
			++stills;
			still_lines += leg_changes;
			still_most = std::max(still_most, leg_changes);
			still_last += leg_last;
			still_slowest = std::max(still_slowest, leg_last);
			WriteLog(format("SELFTEST laydrag leg %d still: %d lines one after another, the last after %d frames, %d frames of a line not worked out whole, L %.2f m, heading %.4f, %s", -leg, leg_changes, leg_last, leg_partial, shown[0], shown[3],
			                Error.c_str()));
		}
		leg = Leg;
		leg_last = 0;
		leg_frames = 0;
		leg_partial = 0;
		leg_changes = 0;
	}
	// the frame counted in its leg: whether the line shown changed in it, whether some fit of it wasn't done yet
	void count_frame(bool const Changed, bool const Partial)
	{
		++leg_frames;
		if (Changed)
		{
			++leg_changes;
			leg_last = leg_frames;
		}
		if (Partial)
			++leg_partial;
	}
	// what was counted over all the legs logged
	void log_summary(bool const Whole) const
	{
		WriteLog(format("SELFTEST laydrag %s: moving %d frames, the line changed %d times, %d frames of a line not worked out whole; still %d times, %.1f lines one after another on average, %d at most, the last after %.1f frames on average, %d at most",
		                Whole ? "whole" : "as fitted", moved_frames, moved_changes, moved_partial, stills, stills > 0 ? static_cast<double>(still_lines) / stills : 0.0, still_most, stills > 0 ? static_cast<double>(still_last) / stills : 0.0, still_slowest));
	}
};

// the line shown, by its length, where its tip is and how it heads there
std::array<double, 4> preview_signature(double const Length, std::vector<segment_data> const &Preview)
{
	std::array<double, 4> result{Length, 0.0, 0.0, 0.0};
	if (false == Preview.empty())
	{
		geometry::bezier const curve{Preview.back()};
		auto const tip{curve.point(1.0)};
		auto const along{geometry::plan_of(curve.first(1.0))};
		result = {Length, tip.x, tip.z, std::atan2(along.y, along.x)};
	}
	return result;
}

// whether the line shown now differs from the one shown before
bool signature_changed(std::array<double, 4> const &Now, std::array<double, 4> const &Before)
{
	return std::abs(Now[0] - Before[0]) > 0.005 || std::hypot(Now[1] - Before[1], Now[2] - Before[2]) > 0.005 || std::abs(Now[3] - Before[3]) > 1e-4;
}

} // namespace

// the cursor moved on along the arc of the points clicked, frame after frame, as the hand moves it while laying in the mode of a curve:
// from x z heading (degrees) the start clicked, Count points of the arc of radius R to the left every Spacing metres, then the cursor
// further along it Moves frames Step metres each, Holds frames still, Legs times over, shifted across by up to Noise. Tells how many
// lines were shown one after another while it stood still, how many frames after the last came, and how many frames showed a line
// some fit of which wasn't done yet. Whole 0: each fit shown as it comes, as before
editor_mode::selftest_result editor_mode::selftest_laydrag(std::istringstream &words)
{
	static lay_drag_state drag;
	if (false == drag.started)
	{
		drag = {};
		lay_drag_setup setup;
		words >> setup.whole >> setup.x >> setup.z >> setup.heading >> setup.radius >> setup.spacing >> setup.count >> setup.step >> setup.moves >> setup.holds >> setup.legs >> setup.noise;
		choose_work_area(work_area::tracks);
		show_track_tab(track_tab::lay);
		m_lay.whole = setup.whole != 0;
		m_lay.active = true;
		lay_cancel();
		m_stretch_cache.clear();
		m_turned_last = 0.0;
		if (m_lay.mode != lay_mode::straights)
			lay_switch_mode();
		selftest_lay_point({setup.x, 0.0, setup.z});
		lay_switch_mode();
		for (int k = 1; k <= setup.count; ++k)
			selftest_lay_point(setup.along(setup.spacing * k, setup.shift(k)));
		drag.plan(setup, setup.spacing * setup.count);
		WriteLog(format("SELFTEST laydrag %s: %zu points clicked, %zu frames, %s", m_lay.whole ? "whole" : "as fitted", m_lay.points.size(), drag.cursor.size(), m_lay.status.c_str()));
		drag.started = true;
		drag.shown = preview_signature(m_lay.preview_length, m_lay.preview);
		m_cursor_override = drag.cursor.front();
		drag.frame = 1;
		return selftest_result::again;
	}
	// the line shown now is the one worked out the frame before, for the cursor set then
	auto const now{preview_signature(m_lay.preview_length, m_lay.preview)};
	auto const changed{signature_changed(now, drag.shown)};
	auto const partial{m_fits_waiting && (false == m_lay.whole || false == m_lay.shown.has_value())};
	drag.shown = now;
	auto const leg{drag.legs[drag.frame - 1]};
	if (leg != drag.leg)
		drag.close_leg(leg, m_lay.preview_error);
	drag.count_frame(changed, partial);
	if (drag.frame < drag.cursor.size())
	{
		m_cursor_override = drag.cursor[drag.frame++];
		return selftest_result::again;
	}
	drag.close_leg(leg, m_lay.preview_error);
	drag.log_summary(m_lay.whole);
	m_cursor_override.reset();
	lay_cancel();
	m_lay.whole = true;
	drag.started = false;
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_tracktab(std::istringstream &words)
{
	std::string key;
	words >> key;
	for (auto const &mode : track_modes())
		if (key == mode.key)
			show_track_tab(mode.tab);
	WriteLog(format("SELFTEST tracktab %s: %s", key.c_str(), track_mode_name().c_str()));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_laychain(std::istringstream &words)
{
	int layout{0};
	words >> layout >> m_lay.radius;
	m_track_layout = static_cast<track_layout>(std::clamp(layout, 0, 3));
	m_lay.active = true;
	lay_cancel();
	auto const report = [&](std::string const &Step) {
		auto const *start{m_lay.start.track};
		WriteLog(format("SELFTEST laychain %s: points %zu, start %s%s at %.2f %.2f, history %zu, status %s", Step.c_str(), m_lay.points.size(), start != nullptr ? start->name().c_str() : "-", start != nullptr && start->m_editorremoved ? " (removed)" : "",
		                m_lay.points.empty() ? 0.0 : m_lay.points.front().x, m_lay.points.empty() ? 0.0 : m_lay.points.front().z, m_history.size(), m_lay.status.c_str()));
	};
	for (std::string word; words >> word;)
	{
		if (word == "b")
			lay_back();
		else if (word == "u")
			undo_last();
		else if (word == "r")
			redo_last();
		else if (word == "e")
		{
			m_lay.mouse_snap = {};
			lay_enter();
		}
		else if (word == "t")
			lay_switch_mode();
		else if (word == "p")
			selftest_lay_report("chain");
		else
		{
			glm::dvec3 point{std::strtod(word.c_str(), nullptr), 5000.0, 0.0};
			words >> point.z;
			selftest_lay_point(point);
			word += format(" %.1f", point.z);
		}
		report(word);
	}
	lay_cancel();
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_laystate() const
{
	WriteLog(format("SELFTEST laystate: layout %d, join variant %d, R %.0f m, join R %.0f m, camera %.1f %.1f %.1f, extent %.1f", static_cast<int>(m_track_layout), m_join_variant, m_lay.radius, m_join_radius, Camera.Pos.x,
	                Camera.Pos.y, Camera.Pos.z, static_cast<double>(Global.EditorOrthoExtent)));
	return selftest_result::handled;
}

namespace
{

// the direction turned by the angle
glm::dvec2 monkey_turned(glm::dvec2 const &Direction, double const Angle)
{
	return glm::dvec2{Direction.x * std::cos(Angle) - Direction.y * std::sin(Angle), Direction.x * std::sin(Angle) + Direction.y * std::cos(Angle)};
}

// the point of the plan at the height
glm::dvec3 lifted(glm::dvec2 const &Point, double const Height)
{
	return glm::dvec3{Point.x, Height, Point.y};
}

// the way out of the free end on the plan
glm::dvec2 plan_way(editor_track::snap_target const &End)
{
	return glm::normalize(geometry::plan_of(End.direction) + glm::dvec2{1e-12, 0.0});
}

// the end the line from a free end goes to: the end drawn, else the nearest end 30 to 600 m off; none for a line to a point
editor_track::snap_target end_for(int const Kind, editor_track::snap_target const &Target, editor_track::snap_target const &Start, glm::dvec3 const &First, double const Distance)
{
	if (Kind == 4)
		return editor_track::snap_target{};
	auto const reach{geometry::plan_distance(Target.position, First)};
	if ((Target.track == Start.track && Target.point == Start.point) || reach < 30.0 || reach > 600.0)
		return end_in_band(First, Distance);
	return Target;
}

// the points clicked between the first and the last: at the shares of the way given, off it across by the shares of the chord
void add_inner(std::vector<glm::dvec3> &Points, glm::dvec3 const &First, glm::dvec3 const &Last, std::vector<double> const &Along, std::array<double, 3> const &Across, int const Inner)
{
	auto const chord{geometry::plan_of(Last - First)};
	glm::dvec2 const side{-chord.y, chord.x};
	for (int k = 0; k < Inner; ++k)
		Points.push_back(lifted(geometry::plan_of(First) + chord * Along[k] + side * Across[k], First.y + (Last.y - First.y) * Along[k]));
}

// the numbers a case of laymonkey draws, every one of them whether the case is run or not
struct monkey_case
{
	int kind;
	editor_track::snap_target const *target;
	editor_track::snap_target const *other;
	int layout;
	double radius;
	double distance;
	double bearing;
	int inner;
	std::array<double, 3> shares;
	std::array<double, 3> across;
	std::array<int, 5> modes_drawn;
	bool near_end;
	double near_back;
	double near_side;
	glm::dvec2 beside;
};

// the numbers of the next case drawn, in the order they always were
monkey_case draw_case(std::mt19937 &Random, std::vector<editor_track::snap_target> const &Ends)
{
	auto const uniform = [&Random](double const Low, double const High) { return std::uniform_real_distribution<double>{Low, High}(Random); };
	auto const pick = [&Random](int const Count) { return std::uniform_int_distribution<int>{0, std::max(0, Count - 1)}(Random); };
	monkey_case drawn{};
	drawn.kind = pick(6);
	drawn.target = &Ends[pick(static_cast<int>(Ends.size()))];
	drawn.other = &Ends[pick(static_cast<int>(Ends.size()))];
	drawn.layout = pick(4);
	constexpr std::array<double, 5> radii{150.0, 300.0, 500.0, 1000.0, 2000.0};
	drawn.radius = radii[pick(5)];
	drawn.distance = std::exp(uniform(std::log(10.0), std::log(400.0)));
	drawn.bearing = std::clamp(uniform(-1.0, 1.0) * uniform(0.0, 1.0) * glm::pi<double>(), -2.97, 2.97);
	drawn.inner = 1 + pick(3);
	drawn.shares = {uniform(0.1, 0.9), uniform(0.1, 0.9), uniform(0.1, 0.9)};
	drawn.across = {uniform(-0.25, 0.25), uniform(-0.25, 0.25), uniform(-0.25, 0.25)};
	drawn.modes_drawn = {pick(2), pick(2), pick(2), pick(2), pick(2)};
	drawn.near_end = uniform(0.0, 1.0) < 0.3;
	drawn.near_back = uniform(1.0, 15.0);
	drawn.near_side = uniform(-3.0, 3.0);
	drawn.beside = glm::dvec2{uniform(-4.0, 4.0), uniform(-4.0, 4.0)};
	return drawn;
}

// the line of a case: whether it starts at a free end, the ends it goes from and to, and the points clicked for it
struct monkey_line
{
	bool from_end{false};
	editor_track::snap_target start;
	editor_track::snap_target end;
	std::vector<glm::dvec3> points;
};

// the line of the case: from a free end it starts at another one, to one within 30 to 600 m of it, else to a point; false where the
// end clicked beside has no free end in the reach of the snap
bool monkey_line_of(monkey_case const &Drawn, monkey_line &Line)
{
	auto const kind{Drawn.kind};
	Line.from_end = kind >= 2 && kind <= 4;
	glm::dvec3 first{0.0};
	glm::dvec3 last{0.0};
	if (Line.from_end)
	{
		Line.start = *Drawn.other;
		first = Line.start.position;
		auto const heading{-plan_way(Line.start)};
		// the nearest end in that band instead, any else as the point
		Line.end = end_for(kind, *Drawn.target, Line.start, first, Drawn.distance);
		last = Line.end.track != nullptr ? Line.end.position : lifted(geometry::plan_of(first) + monkey_turned(heading, Drawn.bearing) * Drawn.distance, first.y);
	}
	else
	{
		Line.end = *Drawn.target;
		last = Line.end.position;
		first = lifted(geometry::plan_of(last) - monkey_turned(plan_way(Line.end), Drawn.bearing) * Drawn.distance, last.y);
	}
	Line.points = {first};
	auto const through{kind == 1 || kind == 3 || kind == 4 || kind == 5};
	if (through)
	{
		std::vector<double> along(Drawn.shares.begin(), Drawn.shares.begin() + Drawn.inner);
		std::ranges::sort(along);
		add_inner(Line.points, first, last, along, Drawn.across, Drawn.inner);
	}
	// a point clicked just before the free end, a little to the side of the way into it
	if (through && Drawn.near_end && Line.end.track != nullptr)
	{
		auto const way{plan_way(Line.end)};
		Line.points.back() = lifted(geometry::plan_of(last) - way * Drawn.near_back + glm::dvec2{-way.y, way.x} * Drawn.near_side, last.y);
	}
	// clicked beside the free end: the nearest one within the reach of the snap is the one joined, as the cursor finds it
	if (kind == 5)
	{
		Line.end = editor_track::find_free_end(nullptr, 1, lifted(geometry::plan_of(Drawn.target->position) + Drawn.beside, Drawn.target->position.y), ::track_mode::kLaySnapRadius, {});
		if (Line.end.track == nullptr)
			return false;
		last = Line.end.position;
	}
	Line.points.push_back(last);
	return true;
}

// the faults of the line laid, as the preview shows it, every metre or so: its ends at the free ends, its pieces against each other, a
// loop, the turning wasted and the sharpest curve; empty for a clean one
std::string monkey_faults(std::vector<segment_data> const &Pieces, monkey_line const &Line)
{
	std::vector<glm::dvec2> line;
	std::vector<glm::dvec2> headings;
	double inner_gap{0.0};
	double inner_kink{0.0};
	trace_pieces(Pieces, line, headings, inner_gap, inner_kink);
	auto const [turning, sharpest]{turning_of(line, headings)};
	auto const net{std::abs(geometry::signed_angle(headings.front(), headings.back()))};
	// the line crossing itself, the stretches next to each other left out
	auto const loop{crosses_itself(line)};
	geometry::bezier const head{Pieces.front()};
	geometry::bezier const tail{Pieces.back()};
	auto const &end{Line.end};
	auto const start_gap{Line.from_end ? glm::distance(head.point(0.0), Line.start.position) : 0.0};
	auto const start_angle{Line.from_end ? glm::degrees(std::abs(geometry::signed_angle(geometry::plan_of(head.first(0.0)), -plan_way(Line.start)))) : 0.0};
	auto const end_gap{end.track != nullptr ? glm::distance(tail.point(1.0), end.position) : 0.0};
	auto const end_angle{end.track != nullptr ? glm::degrees(std::abs(geometry::signed_angle(geometry::plan_of(tail.first(1.0)), plan_way(end)))) : 0.0};
	auto const wasted{glm::degrees(turning - net) > 200.0};
	std::string faults;
	if (start_gap > 0.001 || start_angle > 0.1)
		faults += format(" start joint %.4f m %.3f deg;", start_gap, start_angle);
	if (end_gap > 0.001 || end_angle > 0.1)
		faults += format(" end joint %.4f m %.3f deg;", end_gap, end_angle);
	if (inner_gap > 0.001 || inner_kink > 0.1)
		faults += format(" pieces %.4f m %.3f deg;", inner_gap, inner_kink);
	if (loop)
		faults += " loop;";
	if (wasted)
		faults += format(" turning %.0f deg for %.0f;", glm::degrees(turning), glm::degrees(net));
	if (sharpest < 50.0)
		faults += format(" R %.1f m;", sharpest);
	return faults;
}

// what the laying made of the line of a case: its pieces, how long it is, why it was refused
struct monkey_laid
{
	std::vector<segment_data> pieces;
	double length{0.0};
	std::string error;
};

// the modes of the points of the line, as the case drew them, the fifth on for the ones after it
template <typename Mode>
std::vector<Mode> modes_drawn_of(monkey_case const &Drawn, std::size_t const Count, Mode const Curve, Mode const Straights)
{
	std::vector<Mode> modes;
	for (std::size_t k = 0; k < Count; ++k)
		modes.push_back(Drawn.modes_drawn[std::min<std::size_t>(k, 4)] != 0 ? Curve : Straights);
	return modes;
}

// the points clicked, each with its mode
template <typename Mode>
std::string clicked_of(std::vector<glm::dvec3> const &Points, std::vector<Mode> const &Modes, Mode const Curve)
{
	std::string clicked;
	for (std::size_t k = 0; k < Points.size(); ++k)
		clicked += format(" %.1f,%.1f%s", Points[k].x, Points[k].z, Modes[k] == Curve ? "c" : "s");
	return clicked;
}

// a line refused counted, by the start of its error
void count_refused(std::map<std::string, int, std::less<>> &Tally, std::map<std::string, int, std::less<>> &Refused, std::string const &Error)
{
	++Refused[Error.empty() ? std::string{"(no error)"} : Error.substr(0, 60)];
	++Tally["refused"];
}

// the case with faults logged: what it was, its faults, the line laid and the points clicked
void log_monkey_case(int const Index, std::string const &Kind, std::string const &Faults, monkey_case const &Drawn, monkey_line const &Line, monkey_laid const &Laid, std::string const &Clicked)
{
	auto const to{Line.end.track != nullptr ? end_name(*Line.end.track) : std::string{"-"}};
	WriteLog(format("SELFTEST laymonkey case %d %s:%s from %s to %s, layout %d R %.0f, %zu pieces %.1f m, points%s; %s", Index, Kind.c_str(), Faults.c_str(), Line.from_end ? end_name(*Line.start.track).c_str() : "-", to.c_str(),
	                Drawn.layout, Drawn.radius, Laid.pieces.size(), Laid.length, Clicked.c_str(), Laid.error.c_str()));
}

} // namespace

// monkey tests of the laying to and from the free ends of the scenery: Count lines of random points, modes, layouts and radii, from a
// point or a free end, through up to three points, to a free end, one clicked beside a few metres off and snapped as the nearest, or to
// a point. Each line as the preview shows it is checked: its ends at the free ends (a gap over 1 mm, an angle over 0.1 degree), its
// pieces against each other, a loop (the line crossing itself), the turning wasted (over 200 degrees more than it turns from end to
// end) and the sharpest curve. Only: that case alone, as it was in the run
editor_mode::selftest_result editor_mode::selftest_laymonkey(std::istringstream &words)
{
	unsigned seed{1};
	int count{200};
	int only{-1};
	words >> seed >> count >> only;
	std::mt19937 random{seed}; // NOSONAR: seeded on purpose, so that a run of the test can be repeated; nothing secret is drawn
	auto const ends{editor_track::free_ends(nullptr, 1, glm::dvec3{0.0}, 1e7, {})};
	if (ends.empty())
	{
		WriteLog("SELFTEST laymonkey: no free ends");
		return selftest_result::handled;
	}
	auto const saved_start{m_lay.start};
	auto const saved_radius{m_lay.radius};
	auto const saved_layout{m_track_layout};
	constexpr std::array<char const *, 6> kinds{"point-end", "point-points-end", "end-end", "end-points-end", "end-points-point", "point-points-beside"};
	std::map<std::string, int, std::less<>> tally;
	std::map<std::string, int, std::less<>> refused;
	int failed_shown{0};
	for (int index = 0; index < count; ++index)
	{
		// every case draws its numbers whether it's run or not, so one of them alone is the same as in the whole run
		auto const drawn{draw_case(random, ends)};
		monkey_line line;
		if ((only >= 0 && index != only) || false == monkey_line_of(drawn, line))
			continue;
		auto const modes{modes_drawn_of(drawn, line.points.size(), lay_mode::curve, lay_mode::straights)};
		m_lay.start = line.from_end ? line.start : editor_track::snap_target{};
		m_lay.radius = drawn.radius;
		m_track_layout = static_cast<track_layout>(drawn.layout);
		monkey_laid laid;
		laid.pieces = lay_pieces(line.points, modes, line.end, laid.length, laid.error);
		auto const kind_name{std::string{kinds[drawn.kind]}};
		++tally[kind_name];
		if (laid.pieces.empty())
		{
			count_refused(tally, refused, laid.error);
			continue;
		}
		auto const faults{monkey_faults(laid.pieces, line)};
		if (faults.empty())
		{
			++tally["clean"];
			continue;
		}
		count_faults(tally, faults);
		if (failed_shown++ < 40 || only >= 0)
			log_monkey_case(index, kind_name, faults, drawn, line, laid, clicked_of(line.points, modes, lay_mode::curve));
	}
	m_lay.start = saved_start;
	m_lay.radius = saved_radius;
	m_track_layout = saved_layout;
	std::string summary;
	for (auto const &[name, number] : tally)
		summary += format(" %s %d,", name.c_str(), number);
	WriteLog(format("SELFTEST laymonkey seed %u, %d cases:", seed, count) + summary);
	for (auto const &[error, number] : refused)
		WriteLog(format("SELFTEST laymonkey refused %d: %s", number, error.c_str()));
	return selftest_result::handled;
}

namespace
{

// the tracks laid sorted along the line, from the start of the first to the end of the last
void sort_along_line(std::vector<TTrack const *> &Laid)
{
	if (Laid.empty())
		return;
	auto const origin{Laid.front()->m_paths.front().points[segment_data::point::start]};
	auto const along{glm::normalize(geometry::plan_of(Laid.back()->m_paths.front().points[segment_data::point::end] - origin) + glm::dvec2{1e-9, 0.0})};
	std::ranges::stable_sort(Laid, [&](TTrack const *A, TTrack const *B) {
		return glm::dot(geometry::plan_of(A->m_paths.front().points[segment_data::point::start] - origin), along) < glm::dot(geometry::plan_of(B->m_paths.front().points[segment_data::point::start] - origin), along);
	});
}

// the grade of the tangent
double rise_of(glm::dvec3 const &Tangent)
{
	return Tangent.y / std::max(1e-9, glm::length(geometry::plan_of(Tangent)));
}

// the paths of layclicks gone along one after another: how the one before ended, to tell the kink, the step and the break of the grade
struct laid_walk
{
	glm::dvec2 heading{0.0};
	glm::dvec3 previous{0.0};
	double slope{0.0};
	bool first{true};

	// the path of the track logged against the one before it, and kept as the one before the next
	void log(TTrack const &Track, geometry::bezier const &Curve)
	{
		auto const &path{Track.m_paths.front()};
		auto const begin{glm::normalize(geometry::plan_of(Curve.first(0.0)))};
		auto const &a{path.points[segment_data::point::start]};
		auto const &b{path.points[segment_data::point::end]};
		WriteLog(format("SELFTEST layclicks path %s: %.2f %.2f -> %.2f %.2f, L %.2f m, R %.1f m, kink %.4f deg, step %.4f m, y %.3f -> %.3f, grade %.2f -> %.2f permille, grade break %.3f permille", Track.name().c_str(), a.x, a.z, b.x, b.z, Curve.plan_length(), static_cast<double>(path.radius),
		                first ? 0.0 : glm::degrees(std::abs(geometry::signed_angle(heading, begin))), first ? 0.0 : glm::distance(previous, a), a.y, b.y, 1000.0 * rise_of(Curve.first(0.0)), 1000.0 * rise_of(Curve.first(1.0)),
		                first ? 0.0 : 1000.0 * std::abs(rise_of(Curve.first(0.0)) - slope)));
		slope = rise_of(Curve.first(1.0));
		heading = glm::normalize(geometry::plan_of(Curve.first(1.0)));
		previous = b;
		first = false;
	}
};

} // namespace

// the point clicked by the lay tool at x z, as selftest_lay_point clicks it, the heights and the readout of the line it makes logged
void editor_mode::selftest_click_logged(glm::dvec3 Point)
{
	Point.y = 5000.0;
	std::vector<char> found;
	auto const ground{ground_heights({Point}, true, &found)};
	Point.y = found[0] != 0 ? ground[0] : last_point_height(m_lay.points);
	m_lay.mouse_snap = editor_track::find_free_end(nullptr, 1, Point, 3.0, {});
	m_lay.mouse = m_lay.mouse_snap.track != nullptr ? Point : lay_bed(Point);
	WriteLog(format("SELFTEST layclicks ground %.3f, track %.3f", Point.y, m_lay.mouse.y));
	if (false == m_lay.points.empty())
	{
		lay_trim();
		auto points{m_lay.points};
		auto modes{m_lay.modes};
		modes.push_back(m_lay.mode);
		points.push_back(m_lay.mouse_snap.track != nullptr ? m_lay.mouse_snap.position : m_lay.mouse);
		m_lay.preview = lay_pieces(points, modes, m_lay.mouse_snap, m_lay.preview_length, m_lay.preview_error);
		auto readout{track_readout()};
		std::ranges::replace(readout, '\n', '|');
		WriteLog("SELFTEST layclicks readout: " + readout);
	}
	lay_click();
	WriteLog(format("SELFTEST layclicks %.1f %.1f: %s", Point.x, Point.z, m_lay.status.c_str()));
}

editor_mode::selftest_result editor_mode::selftest_layclicks(std::istringstream &words)
{
	int layout{0};
	words >> layout >> m_lay.radius;
	m_track_layout = static_cast<track_layout>(std::clamp(layout, 0, 3));
	auto const active{m_lay.active};
	m_lay.active = true;
	lay_cancel();
	auto const before{m_history.size()};
	for (glm::dvec3 point{0.0}; words >> point.x >> point.z;)
		selftest_click_logged(point);
	// the points clicked are laid at one go, as Enter lays them
	m_typed.clear();
	m_lay.mouse_snap = {};
	lay_enter();
	WriteLog("SELFTEST layclicks enter: " + m_lay.status);
	lay_cancel();
	m_lay.active = active;
	laid_walk walk;
	double hanging{0.0};
	double buried{0.0};
	int paths{0};
	// the paths laid as they lie now, along the line: a grade line fitted afterwards cut some of them
	std::vector<TTrack const *> laid;
	for (auto step{before}; step < m_history.size(); ++step)
		for (auto const *track : m_history[step].created)
			if (track != nullptr && false == track->m_editorremoved && std::ranges::find(laid, track) == laid.end())
				laid.push_back(track);
	sort_along_line(laid);
	// how far the foot of the ballast hangs over the ground and lies under it along the path, every two metres
	auto const ballast_of = [&](geometry::bezier const &Curve, TTrack const *Track) {
		auto const count{std::max(1, static_cast<int>(std::ceil(Curve.plan_length() / 2.0)))};
		for (int k = 0; k <= count; ++k)
		{
			auto const t{Curve.parameter(Curve.plan_length() * k / count)};
			auto const target{bed_heights({Curve.point(t) + glm::dvec3{0.0, 50.0, 0.0}}, {geometry::plan_of(Curve.first(t))}, Track).front()};
			if (std::isnan(target))
				continue;
			hanging = std::max(hanging, Curve.point(t).y - target);
			buried = std::max(buried, target - Curve.point(t).y);
		}
	};
	for (auto const *track : laid)
	{
		geometry::bezier const curve{track->m_paths.front()};
		walk.log(*track, curve);
		ballast_of(curve, track);
		++paths;
	}
	WriteLog(format("SELFTEST layclicks %d paths, ballast foot over the ground up to %.3f m, under it up to %.3f m", paths, hanging, buried));
	return selftest_result::handled;
}

namespace
{

// the track with its neighbours logged: the paths of each, the joints at the ends of the paths of the track
void log_gizmo_show(TTrack &Track)
{
	std::vector shown{&Track};
	for (auto *neighbour : editor_track::neighbours(Track))
		shown.push_back(neighbour);
	WriteLog(format("SELFTEST gizmo %s prev %s next %s", Track.name().c_str(), Track.trPrev != nullptr ? Track.trPrev->name().c_str() : "-", Track.trNext != nullptr ? Track.trNext->name().c_str() : "-"));
	for (auto const *each : shown)
		log_gizmo_paths(*each);
	for (int i = 0; i < static_cast<int>(Track.m_paths.size()); ++i)
	{
		log_gizmo_joints(Track, i, editor_track::point_kind::start);
		log_gizmo_joints(Track, i, editor_track::point_kind::end);
	}
}

} // namespace

// the operation of the gizmo as the self-tests name it: rotate, scale, anything else translate
editor_mode::gizmo_operation editor_mode::gizmo_operation_named(std::string_view const Name)
{
	if (Name == "rotate")
		return gizmo_operation::rotate;
	if (Name == "scale")
		return gizmo_operation::scale;
	return gizmo_operation::translate;
}

// the gizmo commands which set up what it works on: the set of tracks, the set turned, the operation and the point of the track
void editor_mode::selftest_gizmo_set(std::string const &action, std::istringstream &words)
{
	if (action == "set")
	{
		m_track_set.clear();
		for (std::string name; words >> name;)
			add_named(m_track_set, name);
		m_track_set_pivot = track_set_centre();
		WriteLog(format("SELFTEST gizmo set: %zu tracks, centre %.3f %.3f", m_track_set.size(), m_track_set_pivot.x, m_track_set_pivot.z));
	}
	else if (action == "setturn")
	{
		double degrees{0.0};
		words >> degrees;
		if (track_set_begin(true))
		{
			track_set_turn(glm::radians(degrees));
			track_set_end();
		}
	}
	else
	{
		std::string operation;
		words >> operation;
		choose_work_area(work_area::tracks);
		m_gizmo_op = gizmo_operation_named(operation);
		// as the key of the operation does, the gizmo hidden in the tracks is brought out
		m_track_gizmo_on = true;
		int path{-1};
		std::string kind;
		words >> path >> kind;
		m_track_point = {};
		if (path >= 0)
			m_track_point = {path, point_kind_named(kind)};
	}
}

// the drag of the gizmo begun on the selected track with the operation named, at the point of the track given
void editor_mode::selftest_gizmo_begin(TTrack &track, std::istringstream &words)
{
	std::string operation;
	std::string kind;
	int path{-1};
	words >> operation >> path >> kind;
	m_track_point = {};
	if (path >= 0)
		m_track_point = {path, point_kind_named(kind)};
	auto const wanted{gizmo_operation_named(operation)};
	auto const previous{m_gizmo_op};
	m_gizmo_op = wanted;
	auto const chosen{track_gizmo_operation(track, m_track_point.valid())};
	m_gizmo_op = previous;
	track_gizmo_begin(track, m_track_point.valid(), chosen);
	char const *chosen_name{"translate"};
	if (chosen == gizmo_operation::rotate)
		chosen_name = "rotate";
	else if (chosen == gizmo_operation::scale)
		chosen_name = "scale";
	WriteLog(format("SELFTEST gizmo begin %s: %s, %zu tracks dragged, %zu joints, %zu partners%s", operation.c_str(), chosen_name, m_track_drag.size(), m_track_joints.size(), m_track_gizmo_drag.partners.size(),
	                m_track_gizmo_drag.fixed != glm::dvec3{} ? ", fixed direction" : ""));
}

// the gizmo commands of its drag on the selected track: begun, turned, stretched, a control or the point moved, ended
void editor_mode::selftest_gizmo_drag(std::string const &action, TTrack &track, std::istringstream &words)
{
	if (action == "begin")
		selftest_gizmo_begin(track, words);
	else if (action == "turn")
	{
		double degrees{0.0};
		std::string axis{"y"};
		words >> degrees >> axis;
		m_track_gizmo_drag.ring = ring_named(axis);
		track_gizmo_turn(track, m_track_gizmo_drag.ring, glm::radians(degrees));
		track_gizmo_switches(track);
		WriteLog(format("SELFTEST gizmo turn %s: %s", axis.c_str(), track_readout().c_str()));
	}
	else if (action == "length")
	{
		double length{0.0};
		words >> length;
		track_gizmo_length(track, length);
		track_gizmo_switches(track);
		WriteLog(format("SELFTEST gizmo length %.3f of %.3f, %zu pieces", m_track_gizmo_drag.stretch, m_track_gizmo_drag.length, m_track_gizmo_drag.pieces));
	}
	else if (action == "control")
	{
		glm::dvec3 control_point{0.0};
		words >> control_point.x >> control_point.y >> control_point.z;
		track_gizmo_control(track, control_point);
		track_gizmo_switches(track);
	}
	else if (action == "offset")
	{
		glm::dvec3 offset{0.0};
		words >> offset.x >> offset.y >> offset.z;
		if (m_track_point.valid())
		{
			track_gizmo_point(track, editor_track::point_position(track, m_track_point) + offset);
		}
		else
		{
			editor_track::translate(track, offset);
			editor_track::follow(track, m_track_joints);
		}
		track_gizmo_switches(track);
		m_track_dirty = true;
	}
	else if (action == "end")
		track_gizmo_end(track);
}

editor_mode::selftest_result editor_mode::selftest_gizmo(std::istringstream &words)
{
	std::string action;
	words >> action;
	auto *track{selected_track()};
	if (track == nullptr)
	{
		WriteLog("SELFTEST gizmo: nothing selected");
		return selftest_result::handled;
	}
	if (action == "set" || action == "setturn" || action == "op")
		selftest_gizmo_set(action, words);
	else if (action == "show")
		log_gizmo_show(*track);
	else
		selftest_gizmo_drag(action, *track, words);
	return selftest_result::handled;
}

// the point clicked by the lay tool at x z, on the ground there, a free end within 3 m reached
void editor_mode::selftest_lay_point(glm::dvec3 Point)
{
	Point.y = 5000.0;
	std::vector<char> found;
	auto const ground{ground_heights({Point}, true, &found)};
	Point.y = found[0] != 0 ? ground[0] : last_point_height(m_lay.points);
	m_lay.mouse_snap = editor_track::find_free_end(nullptr, 1, Point, 3.0, {});
	m_lay.mouse = m_lay.mouse_snap.track != nullptr ? Point : lay_bed(Point);
	lay_click();
}

// the line the points clicked make now, as Enter would lay it: its vertices and curves, the warnings, the fits of the curves traced
void editor_mode::selftest_lay_report(std::string const &Label)
{
	lay_trim();
	lay_line_plan plan;
	std::string error;
	// timed as the first frame the points are clicked in, the fits kept for them dropped
	m_stretch_cache.clear();
	auto const started{std::chrono::steady_clock::now()};
	auto const made{m_lay.points.size() >= 2 && lay_line(m_lay.points, m_lay.modes, {}, plan, error)};
	auto const took{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count()};
	WriteLog(format("SELFTEST layplan %s: %s, %zu points, %zu vertices, L %.2f m, %.1f ms %s", Label.c_str(), made ? "valid" : "invalid", m_lay.points.size(), plan.design.vertices.size(), plan.shape.length, took, error.c_str()));
	if (false == made)
		return;
	for (std::size_t k = 0; k < plan.design.vertices.size(); ++k)
		WriteLog(layplan_vertex_text(plan.design, plan.shape, plan.corners, k));
	for (auto const &e : plan.shape.elements)
		if (e.kind == alignment::element_kind::straight)
			WriteLog(format("SELFTEST layplan straight at %.1f m: %.1f m", e.chainage, e.length));
	for (auto const &warning : plan.shape.warnings)
		WriteLog("SELFTEST layplan warning: " + warning);
	for (auto const &trace : plan.traces)
		WriteLog(format("SELFTEST layplan trace: %zu points, %zu curves, departure %.3f m, at most %.3f m", trace.points.size(), trace.curves, trace.rms, trace.worst));
}

// the commands of the group which check what is laid, and the settings of the laying
editor_mode::selftest_result editor_mode::selftest_join_checks(std::string const &command, std::istringstream &words)
{
	if (command == "endcurves")
		return selftest_endcurves();
	if (command == "fitcheck")
		return selftest_fitcheck(words);
	if (command == "cantcheck")
		return selftest_cantcheck(words);
	if (command == "curvejoint")
		return selftest_curvejoint(words);
	if (command == "fittime")
		return selftest_fittime(words);
	if (command == "layheights")
		return selftest_layheights(words);
	if (command == "norms")
		return selftest_norms();
	if (command == "layspeed")
		return selftest_layspeed(words);
	if (command == "laymode")
		return selftest_laymode(words);
	if (command == "laytrace")
		return selftest_laytrace(words);
	if (command == "tracktab")
		return selftest_tracktab(words);
	if (command == "laystate")
		return selftest_laystate();
	return selftest_result::unhandled;
}

// the commands of the group which lay the tracks, by the points clicked and by the gizmo
editor_mode::selftest_result editor_mode::selftest_join_laying(std::string const &command, std::istringstream &words)
{
	if (command == "join" || command == "layto")
		return selftest_join_or_lay_to(command, words);
	if (command == "layp")
		return selftest_layp(words);
	if (command == "layreport")
		return selftest_layreport(words);
	if (command == "laywalk")
		return selftest_laywalk(words);
	if (command == "laybends")
		return selftest_laybends(words);
	if (command == "layenter" || command == "laycancel" || command == "layback")
		return selftest_lay_key(command);
	if (command == "laysample")
		return selftest_laysample(words);
	if (command == "laydrag")
		return selftest_laydrag(words);
	if (command == "laychain")
		return selftest_laychain(words);
	if (command == "laymonkey")
		return selftest_laymonkey(words);
	if (command == "layclicks")
		return selftest_layclicks(words);
	if (command == "gizmo")
		return selftest_gizmo(words);
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_join(std::string const &command, std::istringstream &words)
{
	if (auto const checked{selftest_join_checks(command, words)}; checked != selftest_result::unhandled)
		return checked;
	return selftest_join_laying(command, words);
}

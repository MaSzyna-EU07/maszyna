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

} // namespace

editor_mode::selftest_result editor_mode::selftest_join(std::string const &command, std::istringstream &words)
{
	if (command == "join" || command == "layto")
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
	if (command == "endcurves")
	{
		// every free end of the rails: where, the way out of it and the curvature of its track as it would go on beyond it
		for (auto const &end : editor_track::free_ends(nullptr, 1, glm::dvec3{0.0}, 1e7, {}))
		{
			auto const bend{editor_track::bend_at(*end.track, end.position)};
			auto const curvature{bend.has_value() ? bend->curvature : 0.0};
			WriteLog(format("SELFTEST freeend %s[%d].%s: %.3f %.3f %.3f, out %.6f %.6f, k %.6f (R %.1f), roll %.3f", end.track->name().c_str(), end.point.path, end.point.kind == editor_track::point_kind::end ? "end" : "start", end.position.x, end.position.y,
			                end.position.z, bend.has_value() ? bend->direction.x : 0.0, bend.has_value() ? bend->direction.y : 0.0, curvature, std::abs(curvature) > 1e-9 ? 1.0 / std::abs(curvature) : 0.0, bend.has_value() ? bend->roll : 0.0));
		}
		return selftest_result::handled;
	}
	if (command == "fitcheck")
	{
		// how the tracks and switches lie on the ground and on each other: steps and kinks of the grade at the joints, the two paths of a
		// switch off one plane, the top of the rails against the ground
		std::string label;
		words >> label;
		struct joint_end
		{
			TTrack const *track;
			int path;
			glm::dvec3 point;
			double grade; // rising out of the joint
		};
		std::vector<joint_end> ends;
		auto const clearance = [this](glm::dvec3 const &Point, double &Low, double &High) {
			std::vector<char> found;
			auto const ground{ground_heights({glm::dvec3{Point.x, 5000.0, Point.z}}, true, &found).front()};
			if (found.front() != 0)
			{
				Low = std::min(Low, Point.y - ground);
				High = std::max(High, Point.y - ground);
			}
		};
		double track_low{1e9};
		double track_high{-1e9};
		double switch_low{1e9};
		double switch_high{-1e9};
		int switches{0};
		int faults{0};
		for (auto const *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->m_paths.empty() || (track->iCategoryFlag & 15) != 1)
				continue;
			auto const isswitch{track->eType == tt_Switch};
			switches += isswitch ? 1 : 0;
			for (int index = 0; index < static_cast<int>(track->m_paths.size()); ++index)
			{
				geometry::bezier const curve{track->m_paths[index]};
				auto const grade_at = [&](double const T, double const Sign) {
					auto const first{curve.first(T)};
					return Sign * first.y / std::max(1e-6, std::hypot(first.x, first.z));
				};
				ends.emplace_back(track, index, curve.point(0.0), grade_at(0.0, 1.0));
				ends.emplace_back(track, index, curve.point(1.0), grade_at(1.0, -1.0));
				for (int tenth = 0; tenth < 10; ++tenth)
					clearance(curve.point(0.05 + 0.1 * tenth), isswitch ? switch_low : track_low, isswitch ? switch_high : track_high);
			}
			// the far end of the second path against the first as far along it, in its plane or on the vertical curve it lies on
			if (auto const off{isswitch && track->m_paths.size() >= 2 ? off_plane(*track) : 0.0}; std::abs(off) > 0.03)
			{
				++faults;
				WriteLog(format("SELFTEST fitcheck %s FAULT switch %s off its plane by %.3f m", label.c_str(), track->name().c_str(), off));
			}
		}
		int joints{0};
		double worst_step{0.0};
		double worst_kink{0.0};
		// the two ends, where they meet as a joint of two tracks: its step and its break of the grade
		auto const joint_of = [&](joint_end const &A, joint_end const &B) {
			if (A.track == B.track || false == (geometry::plan_distance(A.point, B.point) < 0.05))
				return;
			++joints;
			auto const step{std::abs(A.point.y - B.point.y)};
			// the same grade seen from the other side has the other sign
			auto const kink{std::abs(A.grade + B.grade) * 1000.0};
			worst_step = std::max(worst_step, step);
			worst_kink = std::max(worst_kink, kink);
			if (step > 0.01 || kink > 3.0)
			{
				++faults;
				WriteLog(format("SELFTEST fitcheck %s FAULT joint %s / %s at %.1f %.1f: step %.3f m, grade break %.1f permille", label.c_str(), A.track->name().c_str(), B.track->name().c_str(), A.point.x, A.point.z, step, kink));
			}
		};
		for (std::size_t a = 0; a < ends.size(); ++a)
			for (std::size_t b = a + 1; b < ends.size(); ++b)
				joint_of(ends[a], ends[b]);
		// the tracks side by side under 5.1 m apart against one grade line, tracks over 2 m apart in height lying on another level
		std::map<std::pair<int, int>, std::vector<std::tuple<glm::dvec3, glm::dvec2, TTrack const *>>> beside;
		// the places of the paths of the track, every 5 m or so
		auto const places_of = [&beside](TTrack const *Track) {
			for (auto const &path : Track->m_paths)
			{
				geometry::bezier const curve{path};
				auto const places{std::max(1, static_cast<int>(geometry::chord_length(path) / 5.0))};
				for (int i = 0; i < places; ++i)
				{
					auto const place{curve.point((i + 0.5) / places)};
					beside[{static_cast<int>(std::floor(place.x / 10.0)), static_cast<int>(std::floor(place.z / 10.0))}].emplace_back(place, geometry::plan_of(curve.first((i + 0.5) / places)), Track);
				}
			}
		};
		for (auto const *track : simulation::Paths.sequence())
			if (track != nullptr && false == track->m_editorremoved && false == track->m_paths.empty() && (track->iCategoryFlag & 15) == 1)
				places_of(track);
		double beside_worst{0.0};
		std::size_t beside_pairs{0};
		std::size_t beside_off{0};
		// the place with the places of the other tracks beside it, running the same way
		auto const pairs_of = [&](std::pair<int, int> const &Cell, glm::dvec3 const &Place, glm::dvec2 const &Way, TTrack const *Track) {
			each_in_cells_around(beside, Cell, [&](auto const &Next) {
				auto const &[next_place, next_way, next_track]{Next};
				if (next_track == Track || geometry::plan_distance(next_place, Place) >= 5.1 || std::abs(glm::dot(Way, next_way)) < 0.985 * glm::length(Way) * glm::length(next_way))
					return;
				auto const difference{std::abs(next_place.y - Place.y)};
				if (difference > 2.0)
					return;
				++beside_pairs;
				beside_off += difference > 0.05 ? 1 : 0;
				beside_worst = std::max(beside_worst, difference);
			});
		};
		for (auto const &[cell, places] : beside)
			for (auto const &[place, way, track] : places)
				pairs_of(cell, place, way, track);
		WriteLog(format("SELFTEST fitcheck %s beside: %zu pairs of places under 5.1 m apart, %zu over 5 cm apart in height, worst %.3f m", label.c_str(), beside_pairs, beside_off, beside_worst));
		WriteLog(format("SELFTEST fitcheck %s: %d switches, %d joints, %d faults, worst step %.3f m, grade break %.1f permille; rails over the ground: tracks %.2f..%.2f m, switches %.2f..%.2f m", label.c_str(), switches, joints, faults, worst_step, worst_kink, track_low, track_high, switch_low, switch_high));
		return selftest_result::handled;
	}
	if (command == "cantcheck")
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
	if (command == "curvejoint")
	{
		// the paths meeting at the point: the curvature of each, signed the way the first one runs into the point, and the jumps of the
		// curvature and of the direction from the ones before the point to the ones after it
		glm::dvec3 point{0.0};
		words >> point.x >> point.z;
		// the end of a path nearest the point, within 3 m
		auto nearest{3.0};
		auto const given{point};
		for (auto const *track : simulation::Paths.sequence())
			if (track != nullptr && false == track->m_editorremoved)
				nearer_end(*track, given, nearest, point);
		struct meeting
		{
			TTrack const *track;
			int path;
			bool atend;
			double curvature; // as the path goes away from the point
			double roll;
			glm::dvec2 out;
		};
		std::vector<meeting> meetings;
		// the end of the path of the track, where it meets the point
		auto const meet = [&](TTrack const *Track, int const I, bool const Atend) {
			auto const &path{Track->m_paths[I]};
			if (geometry::plan_distance(path.points[Atend ? segment_data::point::end : segment_data::point::start], point) > 0.05)
				return;
			geometry::bezier const curve{path};
			auto const first{curve.first(Atend ? 1.0 : 0.0)};
			auto const second{curve.second(Atend ? 1.0 : 0.0)};
			auto const speed{std::hypot(first.x, first.z)};
			if (speed < 1e-9)
				return;
			auto const curvature{(first.x * second.z - first.z * second.x) / (speed * speed * speed)};
			glm::dvec2 const along{first.x / speed, first.z / speed};
			meetings.emplace_back(Track, I, Atend, Atend ? -curvature : curvature, static_cast<double>(Atend ? path.rolls[1] : path.rolls[0]), Atend ? -along : along);
		};
		for (auto const *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved)
				continue;
			for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
			{
				meet(track, i, false);
				meet(track, i, true);
			}
		}
		if (meetings.empty())
		{
			WriteLog(format("SELFTEST curvejoint %.1f %.1f: no path ends there", point.x, point.z));
			return selftest_result::handled;
		}
		auto const into{-meetings.front().out};
		std::vector<std::pair<double, glm::dvec2>> before;
		std::vector<std::pair<double, glm::dvec2>> after;
		for (auto const &each : meetings)
		{
			auto const leaving{glm::dot(each.out, into) > 0.0};
			auto const curvature{leaving ? each.curvature : -each.curvature};
			(leaving ? after : before).emplace_back(curvature, leaving ? each.out : -each.out);
			WriteLog(format("SELFTEST curvejoint %s[%d].%s: %s, k %.6f (R %.1f), roll %.3f", each.track->name().c_str(), each.path, each.atend ? "end" : "start", leaving ? "after" : "before", curvature, std::abs(curvature) > 1e-9 ? 1.0 / std::abs(curvature) : 0.0, each.roll));
		}
		for (auto const &[k0, d0] : before)
			for (auto const &[k1, d1] : after)
				WriteLog(format("SELFTEST curvejoint jump: k %.6f -> %.6f, by %.6f 1/m (%.1f%%), kink %.4f deg", k0, k1, k1 - k0, std::abs(k0) > 1e-9 ? 100.0 * std::abs(k1 - k0) / std::abs(k0) : 0.0, glm::degrees(std::abs(geometry::signed_angle(d0, d1)))));
		return selftest_result::handled;
	}
	if (command == "fittime")
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
	if (command == "layheights")
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
		return selftest_result::handled;
	}
	if (command == "norms")
	{
		// the tracks are laid to the norms only
		WriteLog("SELFTEST norms: plk");
		return selftest_result::handled;
	}
	if (command == "layspeed")
	{
		int own{0};
		words >> m_lay.style.velocity >> own;
		m_lay.own_speed = own != 0;
		WriteLog(format("SELFTEST layspeed: %.0f km/h, own %d", m_lay.style.velocity, m_lay.own_speed ? 1 : 0));
		return selftest_result::handled;
	}
	// the point clicked by the lay tool at x z, on the ground there, a free end within 3 m reached
	auto const lay_point = [this](glm::dvec3 Point) {
		Point.y = 5000.0;
		std::vector<char> found;
		auto const ground{ground_heights({Point}, true, &found)};
		Point.y = found[0] != 0 ? ground[0] : last_point_height(m_lay.points);
		m_lay.mouse_snap = editor_track::find_free_end(nullptr, 1, Point, 3.0, {});
		m_lay.mouse = m_lay.mouse_snap.track != nullptr ? Point : lay_bed(Point);
		lay_click();
	};
	// the line the points clicked make now, as Enter would lay it: its vertices and curves, the warnings, the fits of the curves traced
	auto const lay_report = [this](std::string const &Label) {
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
		{
			auto const &vertex{plan.design.vertices[k]};
			auto text{format("SELFTEST layplan vertex %zu: %s at %.3f %.3f, designed R %.1f L %.1f/%.1f", k, plan.corners[k] != 0 ? "bend" : "traced", k < plan.shape.vertices.size() ? plan.shape.vertices[k].x : 0.0,
			                 k < plan.shape.vertices.size() ? plan.shape.vertices[k].y : 0.0, vertex.radius, vertex.transition_in, vertex.transition_out)};
			for (auto const &curve : plan.shape.curves)
				if (curve.vertex == static_cast<int>(k))
				{
					text += format(", %s R %.1f", curve.spirals ? "spirals" : "arc", curve.radius);
					std::ranges::for_each(curve.radii, [&text](auto const Radius) { text += format("/%.1f", Radius); });
					text += format(", L %.1f/%.1f, cant %.0f, deflection %.3f deg, tangents %.1f/%.1f, arc %.1f m, norms %d", curve.transition_in, curve.transition_out, curve.cant, glm::degrees(curve.deflection), curve.tangent_in, curve.tangent_out, curve.arc_length,
					               alignment::meets_norms(curve, plan.design) ? 1 : 0);
				}
			WriteLog(text);
		}
		for (auto const &e : plan.shape.elements)
			if (e.kind == alignment::element_kind::straight)
				WriteLog(format("SELFTEST layplan straight at %.1f m: %.1f m", e.chainage, e.length));
		for (auto const &warning : plan.shape.warnings)
			WriteLog("SELFTEST layplan warning: " + warning);
		for (auto const &trace : plan.traces)
			WriteLog(format("SELFTEST layplan trace: %zu points, %zu curves, departure %.3f m, at most %.3f m", trace.points.size(), trace.curves, trace.rms, trace.worst));
	};
	if (command == "laymode")
	{
		std::string which;
		words >> which;
		if ((which == "curve") != (m_lay.mode == lay_mode::curve))
			lay_switch_mode();
		WriteLog("SELFTEST laymode: " + std::string{m_lay.mode == lay_mode::curve ? "curve" : "straights"});
		return selftest_result::handled;
	}
	if (command == "laytrace")
	{
		int straights{1};
		words >> m_lay.radii >> straights;
		m_lay.straights_between = straights != 0;
		WriteLog(format("SELFTEST laytrace: radii %d, straights %d", m_lay.radii, straights));
		return selftest_result::handled;
	}
	if (command == "layp")
	{
		m_lay.active = true;
		for (glm::dvec3 point{0.0}; words >> point.x >> point.z;)
			lay_point(point);
		WriteLog(format("SELFTEST layp: %zu points, %s", m_lay.points.size(), m_lay.status.c_str()));
		return selftest_result::handled;
	}
	if (command == "layreport")
	{
		std::string label;
		words >> label;
		lay_report(label);
		return selftest_result::handled;
	}
	// a line of known curves going on from the free end nearest the point, within 60 m: segments of the curvature going evenly from the
	// first to the second (1/m, positive turning the way the track of the end turns, to the left from a straight one) over the length. from: the free end
	// clicked, then in the mode of a curve the points along the line every Spacing metres and two on the straight after it, 30 and 60 m
	// on, left for Enter; into: the same line laid towards the free end, the point 60 m on clicked first, the one 30 m on and the ones
	// along the line in the mode of a curve, the free end last, which lays it. The points shifted across by up to Noise
	if (command == "laywalk")
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
		glm::dvec2 position{end.position.x, end.position.z};
		auto heading{std::atan2(bend->direction.y, bend->direction.x)};
		auto const sense{bend->curvature < 0.0 ? -1.0 : 1.0};
		double chainage{0.0};
		std::vector<glm::dvec2> along;
		std::vector<glm::dvec2> across;
		auto next{spacing};
		// a point of the walk kept where it has come the spacing on from the one before, across it the way it heads there
		auto const keep_point = [&](auto const &Angle, double const At) {
			if (false == (chainage >= next - 1e-6))
				return;
			auto const now{Angle(At)};
			along.push_back(position);
			across.push_back({-std::sin(now), std::cos(now)});
			next += spacing;
		};
		for (auto const &[from, to, length] : segments)
		{
			auto const steps{std::max(1, static_cast<int>(std::ceil(length / 0.05)))};
			auto const step{length / steps};
			auto const start{heading};
			auto const angle = [&, from = from * sense, to = to * sense, length = length](double const S) { return start + from * S + (to - from) * S * S / (2.0 * length); };
			for (int i = 0; i < steps; ++i)
			{
				auto const middle{angle((i + 0.5) * step)};
				position += glm::dvec2{std::cos(middle), std::sin(middle)} * step;
				chainage += step;
				keep_point(angle, (i + 1) * step);
			}
			heading = angle(length);
		}
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
			lay_point(shifted(position + out * 60.0, side, 0));
			lay_switch_mode();
			lay_point(shifted(position + out * 30.0, side, 1));
			// a point along the walk clicked, unless it's within the reach of the snap of the end
			auto const lay_off_end = [&](std::size_t const I) {
				if (glm::distance(along[I], glm::dvec2{end.position.x, end.position.z}) > 3.5)
					lay_point(shifted(along[I], across[I], static_cast<int>(I)));
			};
			for (auto i{along.size()}; i-- > 0;)
				lay_off_end(i);
			lay_point(end.position);
			WriteLog(format("SELFTEST laywalk into: %zu points, %s", m_lay.points.size(), m_lay.status.c_str()));
		}
		else
		{
			lay_point(end.position);
			lay_switch_mode();
			for (std::size_t i = 0; i < along.size(); ++i)
				lay_point(shifted(along[i], across[i], static_cast<int>(i)));
			lay_point(shifted(position + out * 30.0, side, 1));
			lay_point(shifted(position + out * 60.0, side, 2));
			WriteLog(format("SELFTEST laywalk from: %zu points, start %s", m_lay.points.size(), m_lay.start.track != nullptr ? m_lay.start.track->name().c_str() : "-"));
		}
		lay_switch_mode();
		return selftest_result::handled;
	}
	// the bends of straights from the free end nearest the point, within 60 m: the free end clicked, then each point the distance on
	// along the way it goes, which then turns by the angle (degrees, positive the way the track of the end turns, to the left from a
	// straight one); the last one the end, left for Enter
	if (command == "laybends")
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
		lay_point(end.position);
		glm::dvec2 position{end.position.x, end.position.z};
		auto heading{std::atan2(bend->direction.y, bend->direction.x)};
		for (double distance{0.0}; words >> distance;)
		{
			position += glm::dvec2{std::cos(heading), std::sin(heading)} * distance;
			lay_point({position.x, 0.0, position.y});
			if (double turn{0.0}; words >> turn)
				heading += glm::radians(turn) * (bend->curvature < 0.0 ? -1.0 : 1.0);
		}
		WriteLog(format("SELFTEST laybends from %s[%d] %.3f %.3f: %zu points", end.track->name().c_str(), end.point.path, end.position.x, end.position.z, m_lay.points.size()));
		return selftest_result::handled;
	}
	if (command == "layenter" || command == "laycancel" || command == "layback")
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
	if (command == "laysample")
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
		lay_point(truth.start);
		lay_switch_mode();
		auto const first{std::max(0.0, from - 30.0)};
		for (int count = 0; first + count * spacing <= to; ++count)
			lay_point(along(first + count * spacing, noise * (count % 3 == 0 ? 1.0 : count % 3 == 1 ? -0.6 : 0.2)));
		lay_point(along(to + 30.0, noise * 0.5));
		lay_point(along(to + 60.0, -noise * 0.5));
		lay_switch_mode();
		WriteLog(format("SELFTEST laysample: %zu points clicked over %.1f m of curves", m_lay.points.size(), to - from));
		lay_report("sample");
		return selftest_result::handled;
	}
	// the cursor moved on along the arc of the points clicked, frame after frame, as the hand moves it while laying in the mode of a curve:
	// from x z heading (degrees) the start clicked, Count points of the arc of radius R to the left every Spacing metres, then the cursor
	// further along it Moves frames Step metres each, Holds frames still, Legs times over, shifted across by up to Noise. Tells how many
	// lines were shown one after another while it stood still, how many frames after the last came, and how many frames showed a line
	// some fit of which wasn't done yet. Whole 0: each fit shown as it comes, as before
	if (command == "laydrag")
	{
		struct drag_state
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
		};
		static drag_state drag;
		auto const signature = [this]() {
			std::array<double, 4> result{m_lay.preview_length, 0.0, 0.0, 0.0};
			if (false == m_lay.preview.empty())
			{
				geometry::bezier const curve{m_lay.preview.back()};
				auto const tip{curve.point(1.0)};
				auto const along{geometry::plan_of(curve.first(1.0))};
				result = {m_lay.preview_length, tip.x, tip.z, std::atan2(along.y, along.x)};
			}
			return result;
		};
		if (false == drag.started)
		{
			drag = {};
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
			words >> whole >> x >> z >> heading >> radius >> spacing >> count >> step >> moves >> holds >> legs >> noise;
			choose_work_area(work_area::tracks);
			show_track_tab(track_tab::lay);
			m_lay.whole = whole != 0;
			m_lay.active = true;
			lay_cancel();
			m_stretch_cache.clear();
			m_turned_last = 0.0;
			if (m_lay.mode != lay_mode::straights)
				lay_switch_mode();
			glm::dvec2 const start{x, z};
			glm::dvec2 const direction{std::cos(glm::radians(heading)), std::sin(glm::radians(heading))};
			glm::dvec2 const leftward{-direction.y, direction.x};
			auto const centre{start + leftward * radius};
			auto const along = [&](double const Chainage, double const Shift) {
				auto const angle{Chainage / radius};
				auto const radial{geometry::turned(start - centre, angle)};
				auto const point{centre + radial * (1.0 + Shift / radius)};
				return glm::dvec3{point.x, 0.0, point.y};
			};
			auto const shift = [&](int const Count) { return noise * noise_share(Count); };
			lay_point({start.x, 0.0, start.y});
			lay_switch_mode();
			for (int k = 1; k <= count; ++k)
				lay_point(along(spacing * k, shift(k)));
			auto chainage{spacing * count};
			// the cursor of a leg of the drag: moving on, then held where it got
			auto const drag_leg = [&](int const Leg) {
				for (int i = 0; i < moves; ++i)
				{
					chainage += step;
					drag.cursor.push_back(along(chainage, 0.3 * shift(i)));
					drag.legs.push_back(Leg);
				}
				for (int i = 0; i < holds; ++i)
				{
					drag.cursor.push_back(drag.cursor.back());
					drag.legs.push_back(-Leg);
				}
			};
			for (int leg = 1; leg <= legs; ++leg)
				drag_leg(leg);
			WriteLog(format("SELFTEST laydrag %s: %zu points clicked, %zu frames, %s", m_lay.whole ? "whole" : "as fitted", m_lay.points.size(), drag.cursor.size(), m_lay.status.c_str()));
			drag.started = true;
			drag.shown = signature();
			m_cursor_override = drag.cursor.front();
			drag.frame = 1;
			return selftest_result::again;
		}
		// the line shown now is the one worked out the frame before, for the cursor set then
		auto const now{signature()};
		auto const changed{std::abs(now[0] - drag.shown[0]) > 0.005 || std::hypot(now[1] - drag.shown[1], now[2] - drag.shown[2]) > 0.005 || std::abs(now[3] - drag.shown[3]) > 1e-4};
		auto const partial{m_fits_waiting && (false == m_lay.whole || false == m_lay.shown.has_value())};
		drag.shown = now;
		auto const leg{drag.legs[drag.frame - 1]};
		auto const close_leg = [&]() {
			if (drag.leg > 0)
			{
				drag.moved_changes += drag.leg_changes;
				drag.moved_partial += drag.leg_partial;
				drag.moved_frames += drag.leg_frames;
				WriteLog(format("SELFTEST laydrag leg %d moving: %d frames, the line changed %d times, %d frames of a line not worked out whole", drag.leg, drag.leg_frames, drag.leg_changes, drag.leg_partial));
			}
			else if (drag.leg < 0)
			{
				++drag.stills;
				drag.still_lines += drag.leg_changes;
				drag.still_most = std::max(drag.still_most, drag.leg_changes);
				drag.still_last += drag.leg_last;
				drag.still_slowest = std::max(drag.still_slowest, drag.leg_last);
				WriteLog(format("SELFTEST laydrag leg %d still: %d lines one after another, the last after %d frames, %d frames of a line not worked out whole, L %.2f m, heading %.4f, %s", -drag.leg, drag.leg_changes, drag.leg_last, drag.leg_partial, drag.shown[0],
				                drag.shown[3], m_lay.preview_error.c_str()));
			}
			drag.leg = leg;
			drag.leg_last = 0;
			drag.leg_frames = 0;
			drag.leg_partial = 0;
			drag.leg_changes = 0;
		};
		if (leg != drag.leg)
			close_leg();
		++drag.leg_frames;
		if (changed)
		{
			++drag.leg_changes;
			drag.leg_last = drag.leg_frames;
		}
		if (partial)
			++drag.leg_partial;
		if (drag.frame < drag.cursor.size())
		{
			m_cursor_override = drag.cursor[drag.frame++];
			return selftest_result::again;
		}
		close_leg();
		WriteLog(format("SELFTEST laydrag %s: moving %d frames, the line changed %d times, %d frames of a line not worked out whole; still %d times, %.1f lines one after another on average, %d at most, the last after %.1f frames on average, %d at most",
		                m_lay.whole ? "whole" : "as fitted", drag.moved_frames, drag.moved_changes, drag.moved_partial, drag.stills, drag.stills > 0 ? static_cast<double>(drag.still_lines) / drag.stills : 0.0, drag.still_most,
		                drag.stills > 0 ? static_cast<double>(drag.still_last) / drag.stills : 0.0, drag.still_slowest));
		m_cursor_override.reset();
		lay_cancel();
		m_lay.whole = true;
		drag.started = false;
		return selftest_result::handled;
	}
	if (command == "tracktab")
	{
		std::string key;
		words >> key;
		for (auto const &mode : track_modes())
			if (key == mode.key)
				show_track_tab(mode.tab);
		WriteLog(format("SELFTEST tracktab %s: %s", key.c_str(), track_mode_name().c_str()));
		return selftest_result::handled;
	}
	if (command == "laychain")
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
				lay_report("chain");
			else
			{
				glm::dvec3 point{std::strtod(word.c_str(), nullptr), 5000.0, 0.0};
				words >> point.z;
				lay_point(point);
				word += format(" %.1f", point.z);
			}
			report(word);
		}
		lay_cancel();
		return selftest_result::handled;
	}
	if (command == "laystate")
	{
		WriteLog(format("SELFTEST laystate: layout %d, join variant %d, R %.0f m, join R %.0f m, camera %.1f %.1f %.1f, extent %.1f", static_cast<int>(m_track_layout), m_join_variant, m_lay.radius, m_join_radius, Camera.Pos.x,
		                Camera.Pos.y, Camera.Pos.z, static_cast<double>(Global.EditorOrthoExtent)));
		return selftest_result::handled;
	}
	// monkey tests of the laying to and from the free ends of the scenery: Count lines of random points, modes, layouts and radii, from a
	// point or a free end, through up to three points, to a free end, one clicked beside a few metres off and snapped as the nearest, or to
	// a point. Each line as the preview shows it is checked: its ends at the free ends (a gap over 1 mm, an angle over 0.1 degree), its
	// pieces against each other, a loop (the line crossing itself), the turning wasted (over 200 degrees more than it turns from end to
	// end) and the sharpest curve. Only: that case alone, as it was in the run
	if (command == "laymonkey")
	{
		unsigned seed{1};
		int count{200};
		int only{-1};
		words >> seed >> count >> only;
		std::mt19937 random{seed}; // NOSONAR: seeded on purpose, so that a run of the test can be repeated; nothing secret is drawn
		auto const uniform = [&](double const Low, double const High) { return std::uniform_real_distribution<double>{Low, High}(random); };
		auto const pick = [&](int const Count) { return std::uniform_int_distribution<int>{0, std::max(0, Count - 1)}(random); };
		auto const turned = [](glm::dvec2 const &Direction, double const Angle) { return glm::dvec2{Direction.x * std::cos(Angle) - Direction.y * std::sin(Angle), Direction.x * std::sin(Angle) + Direction.y * std::cos(Angle)}; };
		auto const lifted = [](glm::dvec2 const &Point, double const Height) { return glm::dvec3{Point.x, Height, Point.y}; };
		// the end the line from a free end goes to: the end drawn, else the nearest end 30 to 600 m off; none for a line to a point
		auto const end_for = [](int const Kind, editor_track::snap_target const &Target, editor_track::snap_target const &Start, glm::dvec3 const &First, double const Distance) {
			if (Kind == 4)
				return editor_track::snap_target{};
			auto const reach{geometry::plan_distance(Target.position, First)};
			if ((Target.track == Start.track && Target.point == Start.point) || reach < 30.0 || reach > 600.0)
				return end_in_band(First, Distance);
			return Target;
		};
		// the points clicked between the first and the last: at the shares of the way given, off it across by the shares of the chord
		auto const add_inner = [&lifted](std::vector<glm::dvec3> &Points, glm::dvec3 const &First, glm::dvec3 const &Last, std::vector<double> const &Along, std::array<double, 3> const &Across, int const Inner) {
			auto const chord{geometry::plan_of(Last - First)};
			glm::dvec2 const side{-chord.y, chord.x};
			for (int k = 0; k < Inner; ++k)
				Points.push_back(lifted(geometry::plan_of(First) + chord * Along[k] + side * Across[k], First.y + (Last.y - First.y) * Along[k]));
		};
		// the points clicked, each with its mode
		auto const clicked_of = [](std::vector<glm::dvec3> const &Points, std::vector<lay_mode> const &Modes) {
			std::string clicked;
			for (std::size_t k = 0; k < Points.size(); ++k)
				clicked += format(" %.1f,%.1f%s", Points[k].x, Points[k].z, Modes[k] == lay_mode::curve ? "c" : "s");
			return clicked;
		};
		auto const plan_way = [](editor_track::snap_target const &End) { return glm::normalize(geometry::plan_of(End.direction) + glm::dvec2{1e-12, 0.0}); };
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
			auto const kind{pick(6)};
			auto const &target{ends[pick(static_cast<int>(ends.size()))]};
			auto const &other{ends[pick(static_cast<int>(ends.size()))]};
			auto const layout{pick(4)};
			constexpr std::array<double, 5> radii{150.0, 300.0, 500.0, 1000.0, 2000.0};
			auto const radius{radii[pick(5)]};
			auto const distance{std::exp(uniform(std::log(10.0), std::log(400.0)))};
			auto const bearing{std::clamp(uniform(-1.0, 1.0) * uniform(0.0, 1.0) * glm::pi<double>(), -2.97, 2.97)};
			auto const inner{1 + pick(3)};
			std::array<double, 3> shares{uniform(0.1, 0.9), uniform(0.1, 0.9), uniform(0.1, 0.9)};
			std::array<double, 3> const across{uniform(-0.25, 0.25), uniform(-0.25, 0.25), uniform(-0.25, 0.25)};
			std::array<int, 5> modes_drawn{pick(2), pick(2), pick(2), pick(2), pick(2)};
			auto const near_end{uniform(0.0, 1.0) < 0.3};
			auto const near_back{uniform(1.0, 15.0)};
			auto const near_side{uniform(-3.0, 3.0)};
			auto const beside{glm::dvec2{uniform(-4.0, 4.0), uniform(-4.0, 4.0)}};
			if (only >= 0 && index != only)
				continue;
			// the line from a free end starts at another one, to one within 30 to 600 m of it, else to a point
			auto const from_end{kind >= 2 && kind <= 4};
			editor_track::snap_target start;
			editor_track::snap_target end;
			glm::dvec3 first{0.0};
			glm::dvec3 last{0.0};
			glm::dvec2 heading{0.0};
			if (from_end)
			{
				start = other;
				first = start.position;
				heading = -plan_way(start);
				// the nearest end in that band instead, any else as the point
				end = end_for(kind, target, start, first, distance);
				last = end.track != nullptr ? end.position : lifted(geometry::plan_of(first) + turned(heading, bearing) * distance, first.y);
			}
			else
			{
				end = target;
				last = end.position;
				first = lifted(geometry::plan_of(last) - turned(plan_way(end), bearing) * distance, last.y);
			}
			std::vector<glm::dvec3> points{first};
			if (kind == 1 || kind == 3 || kind == 4 || kind == 5)
			{
				std::vector<double> along(shares.begin(), shares.begin() + inner);
				std::ranges::sort(along);
				add_inner(points, first, last, along, across, inner);
			}
			// a point clicked just before the free end, a little to the side of the way into it
			if ((kind == 1 || kind == 3 || kind == 4 || kind == 5) && near_end && end.track != nullptr)
			{
				auto const way{plan_way(end)};
				points.back() = lifted(geometry::plan_of(last) - way * near_back + glm::dvec2{-way.y, way.x} * near_side, last.y);
			}
			// clicked beside the free end: the nearest one within the reach of the snap is the one joined, as the cursor finds it
			if (kind == 5)
				end = editor_track::find_free_end(nullptr, 1, lifted(geometry::plan_of(target.position) + beside, target.position.y), ::track_mode::kLaySnapRadius, {});
			if (kind == 5 && end.track == nullptr)
				continue;
			if (kind == 5)
				last = end.position;
			points.push_back(last);
			std::vector<lay_mode> modes;
			for (std::size_t k = 0; k < points.size(); ++k)
				modes.push_back(modes_drawn[std::min<std::size_t>(k, 4)] != 0 ? lay_mode::curve : lay_mode::straights);
			m_lay.start = from_end ? start : editor_track::snap_target{};
			m_lay.radius = radius;
			m_track_layout = static_cast<track_layout>(layout);
			double length{0.0};
			std::string error;
			auto const pieces{lay_pieces(points, modes, end, length, error)};
			auto const kind_name{std::string{kinds[kind]}};
			++tally[kind_name];
			if (pieces.empty())
			{
				++refused[error.empty() ? std::string{"(no error)"} : error.substr(0, 60)];
				++tally["refused"];
				continue;
			}
			// the line as the preview shows it, every metre or so, and its pieces against each other
			std::vector<glm::dvec2> line;
			std::vector<glm::dvec2> headings;
			double inner_gap{0.0};
			double inner_kink{0.0};
			trace_pieces(pieces, line, headings, inner_gap, inner_kink);
			auto const [turning, sharpest]{turning_of(line, headings)};
			auto const net{std::abs(geometry::signed_angle(headings.front(), headings.back()))};
			// the line crossing itself, the stretches next to each other left out
			auto const loop{crosses_itself(line)};
			geometry::bezier const head{pieces.front()};
			geometry::bezier const tail{pieces.back()};
			auto const start_gap{from_end ? glm::distance(head.point(0.0), start.position) : 0.0};
			auto const start_angle{from_end ? glm::degrees(std::abs(geometry::signed_angle(geometry::plan_of(head.first(0.0)), -plan_way(start)))) : 0.0};
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
			if (faults.empty())
			{
				++tally["clean"];
				continue;
			}
			count_faults(tally, faults);
			if (failed_shown++ < 40 || only >= 0)
			{
				auto const clicked{clicked_of(points, modes)};
				auto const to{end.track != nullptr ? end_name(*end.track) : std::string{"-"}};
				WriteLog(format("SELFTEST laymonkey case %d %s:%s from %s to %s, layout %d R %.0f, %zu pieces %.1f m, points%s; %s", index, kind_name.c_str(), faults.c_str(), from_end ? end_name(*start.track).c_str() : "-",
				                to.c_str(), layout, radius, pieces.size(), length, clicked.c_str(), error.c_str()));
			}
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
	if (command == "layclicks")
	{
		int layout{0};
		words >> layout >> m_lay.radius;
		m_track_layout = static_cast<track_layout>(std::clamp(layout, 0, 3));
		auto const active{m_lay.active};
		m_lay.active = true;
		lay_cancel();
		auto const before{m_history.size()};
		for (glm::dvec3 point{0.0}; words >> point.x >> point.z;)
		{
			point.y = 5000.0;
			std::vector<char> found;
			auto const ground{ground_heights({point}, true, &found)};
			point.y = found[0] != 0 ? ground[0] : last_point_height(m_lay.points);
			m_lay.mouse_snap = editor_track::find_free_end(nullptr, 1, point, 3.0, {});
			m_lay.mouse = m_lay.mouse_snap.track != nullptr ? point : lay_bed(point);
			WriteLog(format("SELFTEST layclicks ground %.3f, track %.3f", point.y, m_lay.mouse.y));
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
			WriteLog(format("SELFTEST layclicks %.1f %.1f: %s", point.x, point.z, m_lay.status.c_str()));
		}
		// the points clicked are laid at one go, as Enter lays them
		m_typed.clear();
		m_lay.mouse_snap = {};
		lay_enter();
		WriteLog("SELFTEST layclicks enter: " + m_lay.status);
		lay_cancel();
		m_lay.active = active;
		glm::dvec2 heading{0.0};
		glm::dvec3 previous{0.0};
		double slope{0.0};
		bool first{true};
		double hanging{0.0};
		double buried{0.0};
		int paths{0};
		// the paths laid as they lie now, along the line: a grade line fitted afterwards cut some of them
		std::vector<TTrack const *> laid;
		for (auto step{before}; step < m_history.size(); ++step)
			for (auto const *track : m_history[step].created)
				if (track != nullptr && false == track->m_editorremoved && std::ranges::find(laid, track) == laid.end())
					laid.push_back(track);
		if (false == laid.empty())
		{
			auto const origin{laid.front()->m_paths.front().points[segment_data::point::start]};
			auto const along{glm::normalize(geometry::plan_of(laid.back()->m_paths.front().points[segment_data::point::end] - origin) + glm::dvec2{1e-9, 0.0})};
			std::ranges::stable_sort(laid, [&](TTrack const *A, TTrack const *B) {
				return glm::dot(geometry::plan_of(A->m_paths.front().points[segment_data::point::start] - origin), along) < glm::dot(geometry::plan_of(B->m_paths.front().points[segment_data::point::start] - origin), along);
			});
		}
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
			auto const &path{track->m_paths.front()};
			geometry::bezier const curve{path};
			auto const begin{glm::normalize(geometry::plan_of(curve.first(0.0)))};
			auto const &a{path.points[segment_data::point::start]};
			auto const &b{path.points[segment_data::point::end]};
			auto const rise = [](glm::dvec3 const &Tangent) { return Tangent.y / std::max(1e-9, glm::length(geometry::plan_of(Tangent))); };
			WriteLog(format("SELFTEST layclicks path %s: %.2f %.2f -> %.2f %.2f, L %.2f m, R %.1f m, kink %.4f deg, step %.4f m, y %.3f -> %.3f, grade %.2f -> %.2f permille, grade break %.3f permille", track->name().c_str(), a.x, a.z, b.x, b.z, curve.plan_length(), static_cast<double>(path.radius),
			                first ? 0.0 : glm::degrees(std::abs(geometry::signed_angle(heading, begin))), first ? 0.0 : glm::distance(previous, a), a.y, b.y, 1000.0 * rise(curve.first(0.0)), 1000.0 * rise(curve.first(1.0)),
			                first ? 0.0 : 1000.0 * std::abs(rise(curve.first(0.0)) - slope)));
			slope = rise(curve.first(1.0));
			heading = glm::normalize(geometry::plan_of(curve.first(1.0)));
			previous = b;
			first = false;
			ballast_of(curve, track);
			++paths;
		}
		WriteLog(format("SELFTEST layclicks %d paths, ballast foot over the ground up to %.3f m, under it up to %.3f m", paths, hanging, buried));
		return selftest_result::handled;
	}
	if (command == "gizmo")
	{
		std::string action;
		words >> action;
		auto *track{selected_track()};
		if (track == nullptr)
		{
			WriteLog("SELFTEST gizmo: nothing selected");
			return selftest_result::handled;
		}
		auto const operation_named = [](std::string_view const Name) {
			if (Name == "rotate")
				return gizmo_operation::rotate;
			if (Name == "scale")
				return gizmo_operation::scale;
			return gizmo_operation::translate;
		};
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
		else if (action == "op")
		{
			std::string operation;
			words >> operation;
			choose_work_area(work_area::tracks);
			m_gizmo_op = operation_named(operation);
			// as the key of the operation does, the gizmo hidden in the tracks is brought out
			m_track_gizmo_on = true;
			int path{-1};
			std::string kind;
			words >> path >> kind;
			m_track_point = {};
			if (path >= 0)
				m_track_point = {path, point_kind_named(kind)};
		}
		else if (action == "begin")
		{
			std::string operation;
			std::string kind;
			int path{-1};
			words >> operation >> path >> kind;
			m_track_point = {};
			if (path >= 0)
				m_track_point = {path, point_kind_named(kind)};
			auto const wanted{operation_named(operation)};
			auto const previous{m_gizmo_op};
			m_gizmo_op = wanted;
			auto const chosen{track_gizmo_operation(*track, m_track_point.valid())};
			m_gizmo_op = previous;
			track_gizmo_begin(*track, m_track_point.valid(), chosen);
			WriteLog(format("SELFTEST gizmo begin %s: %s, %zu tracks dragged, %zu joints, %zu partners%s", operation.c_str(), chosen == gizmo_operation::rotate ? "rotate" : chosen == gizmo_operation::scale ? "scale" : "translate", m_track_drag.size(), m_track_joints.size(),
			                m_track_gizmo_drag.partners.size(), m_track_gizmo_drag.fixed != glm::dvec3{} ? ", fixed direction" : ""));
		}
		else if (action == "turn")
		{
			double degrees{0.0};
			std::string axis{"y"};
			words >> degrees >> axis;
			m_track_gizmo_drag.ring = ring_named(axis);
			track_gizmo_turn(*track, m_track_gizmo_drag.ring, glm::radians(degrees));
			track_gizmo_switches(*track);
			WriteLog(format("SELFTEST gizmo turn %s: %s", axis.c_str(), track_readout().c_str()));
		}
		else if (action == "length")
		{
			double length{0.0};
			words >> length;
			track_gizmo_length(*track, length);
			track_gizmo_switches(*track);
			WriteLog(format("SELFTEST gizmo length %.3f of %.3f, %zu pieces", m_track_gizmo_drag.stretch, m_track_gizmo_drag.length, m_track_gizmo_drag.pieces));
		}
		else if (action == "control")
		{
			glm::dvec3 control_point{0.0};
			words >> control_point.x >> control_point.y >> control_point.z;
			track_gizmo_control(*track, control_point);
			track_gizmo_switches(*track);
		}
		else if (action == "offset")
		{
			glm::dvec3 offset{0.0};
			words >> offset.x >> offset.y >> offset.z;
			if (m_track_point.valid())
			{
				track_gizmo_point(*track, editor_track::point_position(*track, m_track_point) + offset);
			}
			else
			{
				editor_track::translate(*track, offset);
				editor_track::follow(*track, m_track_joints);
			}
			track_gizmo_switches(*track);
			m_track_dirty = true;
		}
		else if (action == "end")
			track_gizmo_end(*track);
		else if (action == "show")
		{
			std::vector shown{track};
			for (auto *neighbour : editor_track::neighbours(*track))
				shown.push_back(neighbour);
			WriteLog(format("SELFTEST gizmo %s prev %s next %s", track->name().c_str(), track->trPrev != nullptr ? track->trPrev->name().c_str() : "-", track->trNext != nullptr ? track->trNext->name().c_str() : "-"));
			for (auto const *each : shown)
				log_gizmo_paths(*each);
			for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
			{
				log_gizmo_joints(*track, i, editor_track::point_kind::start);
				log_gizmo_joints(*track, i, editor_track::point_kind::end);
			}
		}
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

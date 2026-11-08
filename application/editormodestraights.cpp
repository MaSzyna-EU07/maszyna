/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http:
*/


#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodebuildtools.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"
#include "world/TrackFeatures.h"
#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <format>
#include <ranges>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <functional>
#include <iterator>
#include <set>
#include "application/editormodetrackshared.h"

using namespace track_mode;

namespace
{

// whether a straight runs within 3 km of the camera
bool straight_nearby(editor_track::straight const &Line, glm::dvec3 const &Camera)
{
	auto const offset{Camera - Line.start};
	auto const along{std::clamp(glm::dot(plan_of(offset), Line.direction), 0.0, Line.length)};
	auto const closest{Line.start + glm::dvec3{Line.direction.x, 0.0, Line.direction.y} * along};
	return plan_distance(closest, Camera) < 3000.0;
}

// a straight over the view, with dots at its ends
void draw_straight(screen_projection const &Projection, ImDrawList *Drawlist, editor_track::straight const &Line, ImU32 const Color, float const Thickness)
{
	if (Line.tracks.empty())
		return;
	Projection.line(Drawlist, Line.start, Line.end, Color, Thickness);
	for (auto const &point : {Line.start, Line.end})
	{
		ImVec2 screen;
		if (Projection.project(point, screen))
			Drawlist->AddCircleFilled(screen, Thickness + 2.0f, Color);
	}
}

// the straights recognized in the scenery near the camera, the one picked in the list lit
void draw_found_straights(screen_projection const &Projection, ImDrawList *Drawlist, std::vector<editor_track::straight> const &Found, int const Listed, glm::dvec3 const &Camera)
{
	for (int i = 0; i < static_cast<int>(Found.size()); ++i)
	{
		auto const &line{Found[i]};
		if (false == straight_nearby(line, Camera))
			continue;
		auto const listed{i == Listed};
		draw_straight(Projection, Drawlist, line, listed ? overlay_color::selected : IM_COL32(255, 255, 255, 110), listed ? 4.0f : 2.0f);
	}
}

// the break of the straight being placed: the rest of it turned toward the handle
void draw_break_preview(screen_projection const &Projection, ImDrawList *Drawlist, editor_track::straight const &Line, double const At, glm::dvec3 const &Point, glm::dvec3 const &Handle)
{
	if (auto const direction{plan_of(Handle - Point)}; glm::length(direction) > 1e-3)
	{
		auto const unit{glm::normalize(direction)};
		auto const rest{std::max(0.0, Line.length - At)};
		Projection.line(Drawlist, Point, Point + glm::dvec3{unit.x, Line.grade, unit.y} * rest, overlay_color::marked, 3.0f);
	}
}

// the shift of a piece of the straight being placed: sideways out to the handle, and on to the end
void draw_shift_preview(screen_projection const &Projection, ImDrawList *Drawlist, editor_track::straight const &Line, glm::dvec3 const &Point, glm::dvec3 const &Handle)
{
	glm::dvec2 const normal{-Line.direction.y, Line.direction.x};
	auto const offset{plan_of(Handle - Point)};
	auto const shift{glm::dot(offset, normal)};
	auto const length{glm::dot(offset, Line.direction)};
	auto const shifted{glm::dvec3{normal.x, 0.0, normal.y} * shift};
	auto const along{glm::dvec3{Line.direction.x, 0.0, Line.direction.y}};
	Projection.line(Drawlist, Point, Point + along * length + shifted, IM_COL32(255, 210, 60, 160), 2.0f);
	Projection.line(Drawlist, Point + along * length + shifted, Line.end + shifted, overlay_color::marked, 3.0f);
}

// the point and the handle of the tool of the straight being placed, with what the tool makes
void draw_straight_tool(screen_projection const &Projection, ImDrawList *Drawlist, int const Tool, editor_track::straight const &Line, double const At, glm::dvec3 const &Point, glm::dvec3 const &Handle)
{
	ImVec2 screen;
	if (Projection.project(Point, screen))
		Drawlist->AddCircleFilled(screen, 7.0f, overlay_color::invalid);
	if (Projection.project(Handle, screen))
		Drawlist->AddCircleFilled(screen, 7.0f, overlay_color::marked);
	if (Tool == 1)
		draw_break_preview(Projection, Drawlist, Line, At, Point, Handle);
	else
		draw_shift_preview(Projection, Drawlist, Line, Point, Handle);
}

// the route fitted between the curves around the straight as it's dragged
void draw_fitted_route(screen_projection const &Projection, ImDrawList *Drawlist, alignment::result const &Result)
{
	if (Result.length <= 0.0 || Result.profile.size() < 2)
		return;
	int const steps{std::clamp(static_cast<int>(Result.length / 2.0), 2, 2000)};
	auto previous{alignment::evaluate(Result, 0.0).position};
	for (int i = 1; i <= steps; ++i)
	{
		auto const next{alignment::evaluate(Result, Result.length * i / steps).position};
		Projection.line(Drawlist, previous, next, Result.valid ? overlay_color::marked : overlay_color::invalid, 3.0f);
		previous = next;
	}
}

// the paths of the straights dragged, as they lie
void draw_dragged_paths(screen_projection const &Projection, ImDrawList *Drawlist, std::vector<editor_track::straight> const &Lines)
{
	for (auto const &member : Lines)
	{
		for (auto const *track : member.tracks)
		{
			auto const &path{track->m_paths.front()};
			Projection.line(Drawlist, path.points[segment_data::point::start], path.points[segment_data::point::end], overlay_color::marked, 3.0f);
		}
	}
}

// the handles of the straight: squares at its ends, a diamond in the middle, the one held ringed
void draw_straight_handles(screen_projection const &Projection, ImDrawList *Drawlist, editor_track::straight const &Line, int const Handle)
{
	std::array<glm::dvec3, 3> const handles{Line.start, Line.end, (Line.start + Line.end) * 0.5};
	for (int i = 0; i < 3; ++i)
	{
		ImVec2 screen;
		if (false == Projection.project(handles[i], screen))
			continue;
		if (i < 2)
			Drawlist->AddRectFilled(ImVec2(screen.x - 6.0f, screen.y - 6.0f), ImVec2(screen.x + 6.0f, screen.y + 6.0f), overlay_color::selected);
		else
			Drawlist->AddCircleFilled(screen, 6.0f, overlay_color::selected, 4);
		if (i == Handle)
			Drawlist->AddCircle(screen, 12.0f, overlay_color::highlight, 16, 2.5f);
	}
}

// the straight moved over to lie between the ends of the preview of a drag
void drag_to(editor_track::straight &Line, glm::dvec3 const &Start, glm::dvec3 const &End)
{
	Line.start = Start;
	Line.end = End;
	auto const planar{plan_of(Line.end - Line.start)};
	auto const length{glm::length(planar)};
	if (length > 1e-6)
	{
		Line.direction = planar / length;
		Line.length = length;
	}
}

// how a neighbouring straight is offered for a parallel snap: lit as the one offered, snapping, at which spacing listed, at what angle to it
struct neighbour_hint
{
	bool highlighted{false};
	bool snaps{false};
	double spacing{0.0};
	double angle{0.0};
};

ImU32 neighbour_tick_colour(neighbour_hint const &Hint)
{
	if (false == Hint.highlighted)
		return IM_COL32(255, 230, 120, 230);
	return Hint.snaps ? IM_COL32(80, 230, 255, 255) : IM_COL32(255, 210, 60, 255);
}

// the distance to a neighbouring straight Lateral off, and the angle between them, as written at the tick to it
std::string neighbour_label(neighbour_hint const &Hint, double const Lateral, double const Angle)
{
	if (Hint.highlighted)
	{
		if (Hint.snaps)
			return format(STR_C("parallel  %.2f m"), Hint.spacing);
		return format("%.2f -> %.2f m  %.1f°", std::abs(Lateral), Hint.spacing, Hint.angle);
	}
	if (Angle > 0.05)
		return format("%.2f m  %.1f°", std::abs(Lateral), Angle);
	return format("%.2f m", std::abs(Lateral));
}

// a tick across from the line to a neighbouring straight where they run side by side, with the distance and the angle written at it
void draw_neighbour_tick(screen_projection const &Projection, ImDrawList *Drawlist, editor_track::straight const &Line, editor_track::straight const &Other, neighbour_hint const &Hint)
{
	glm::dvec2 const normal{-Line.direction.y, Line.direction.x};
	auto const across = [&Line, &normal](glm::dvec3 const &Point) { return glm::dot(plan_of(Point - Line.start), normal); };
	auto const a0{Line.along(Other.start)};
	auto const a1{Line.along(Other.end)};
	auto const from{std::max(0.0, std::min(a0, a1))};
	auto const to{std::min(Line.length, std::max(a0, a1))};
	if (to <= from)
		return;
	auto const middle{(from + to) * 0.5};
	auto const fraction{std::abs(a1 - a0) > 1e-6 ? (middle - a0) / (a1 - a0) : 0.0};
	auto const lateral{across(Other.start) + (across(Other.end) - across(Other.start)) * fraction};
	glm::dvec3 const foot{Line.start.x + Line.direction.x * middle, Line.start.y + Line.grade * middle, Line.start.z + Line.direction.y * middle};
	glm::dvec3 const target{foot.x + normal.x * lateral, foot.y, foot.z + normal.y * lateral};
	auto const tick{neighbour_tick_colour(Hint)};
	if (Hint.highlighted)
		Projection.line(Drawlist, Other.start, Other.end, tick, 3.0f);
	Projection.line(Drawlist, foot, target, tick, Hint.highlighted ? 3.0f : 1.5f);
	ImVec2 screen;
	if (false == Projection.project((foot + target) * 0.5, screen))
		return;
	auto const angle{glm::degrees(std::abs(std::asin(std::clamp(Line.direction.x * Other.direction.y - Line.direction.y * Other.direction.x, -1.0, 1.0))))};
	auto const text{neighbour_label(Hint, lateral, angle)};
	auto const *label{text.c_str()};
	auto const size{ImGui::CalcTextSize(label)};
	Drawlist->AddRectFilled(ImVec2(screen.x - 4.0f, screen.y - 2.0f), ImVec2(screen.x + size.x + 4.0f, screen.y + size.y + 2.0f), IM_COL32(0, 0, 0, 190), 3.0f);
	Drawlist->AddText(screen, Hint.highlighted ? tick : IM_COL32(255, 230, 120, 255), label);
}

// whether the other straight misses the line: past its start or its end
bool misses_along(editor_track::straight const &Line, editor_track::straight const &Other)
{
	auto const a0{Line.along(Other.start)};
	auto const a1{Line.along(Other.end)};
	return std::max(a0, a1) <= 0.0 || std::min(a0, a1) >= Line.length;
}

// the straight of a track put among the neighbours of the line where it runs alongside it near enough; its paths are looked at once
void add_neighbour(std::vector<editor_track::straight> &Neighbours, std::vector<TTrack const *> &Visited, editor_track::straight const &Line, TTrack *Track, editor_track::straight_tolerance const &Tolerance)
{
	if (std::ranges::find(Visited, Track) != Visited.end() || false == editor_track::is_straight(*Track, Tolerance))
		return;
	auto other{editor_track::find_straight(*Track, Tolerance)};
	Visited.insert(Visited.end(), other.tracks.begin(), other.tracks.end());
	if (other.tracks.empty() || std::abs(Line.direction.x * other.direction.y - Line.direction.y * other.direction.x) > std::sin(glm::radians(kParallelFindAngle)))
		return;
	glm::dvec2 const normal{-Line.direction.y, Line.direction.x};
	if (auto const offset{glm::dvec2{(other.start + other.end).x * 0.5 - Line.start.x, (other.start + other.end).z * 0.5 - Line.start.z}}; std::abs(glm::dot(offset, normal)) > 60.0)
		return;
	if (misses_along(Line, other))
		return;
	Neighbours.push_back(std::move(other));
}

// a point of a tile from Low to High, X of the way across and Y down
ImVec2 tile_point(ImVec2 const Low, ImVec2 const High, float const X, float const Y)
{
	return ImVec2(Low.x + (High.x - Low.x) * X, Low.y + (High.y - Low.y) * Y);
}

// the sketch of the break: the rest of the straight turned off from a point
void sketch_break(ImDrawList &List, ImVec2 Low, ImVec2 High, ImU32 Line, ImU32 Made)
{
	List.AddLine(tile_point(Low, High, 0.0f, 0.8f), tile_point(Low, High, 1.0f, 0.8f), (Line & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 60), 1.0f);
	List.AddLine(tile_point(Low, High, 0.0f, 0.8f), tile_point(Low, High, 0.4f, 0.8f), Line, 2.0f);
	List.AddBezierQuadratic(tile_point(Low, High, 0.4f, 0.8f), tile_point(Low, High, 0.62f, 0.8f), tile_point(Low, High, 1.0f, 0.1f), Made, 2.0f);
}

// the sketch of the S-curve: a piece of the straight shifted sideways around an obstacle and back
void sketch_scurve(ImDrawList &List, ImVec2 Low, ImVec2 High, ImU32 Line, ImU32 Made)
{
	List.AddLine(tile_point(Low, High, 0.0f, 0.8f), tile_point(Low, High, 1.0f, 0.8f), (Line & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 60), 1.0f);
	List.AddLine(tile_point(Low, High, 0.0f, 0.8f), tile_point(Low, High, 0.15f, 0.8f), Line, 2.0f);
	List.AddBezierCubic(tile_point(Low, High, 0.15f, 0.8f), tile_point(Low, High, 0.3f, 0.8f), tile_point(Low, High, 0.25f, 0.2f), tile_point(Low, High, 0.42f, 0.2f), Made, 2.0f);
	List.AddLine(tile_point(Low, High, 0.42f, 0.2f), tile_point(Low, High, 0.58f, 0.2f), Made, 2.0f);
	List.AddBezierCubic(tile_point(Low, High, 0.58f, 0.2f), tile_point(Low, High, 0.75f, 0.2f), tile_point(Low, High, 0.7f, 0.8f), tile_point(Low, High, 0.85f, 0.8f), Made, 2.0f);
	List.AddLine(tile_point(Low, High, 0.85f, 0.8f), tile_point(Low, High, 1.0f, 0.8f), Line, 2.0f);
	List.AddCircleFilled(tile_point(Low, High, 0.5f, 0.8f), 2.5f, IM_COL32(230, 90, 70, 255));
}

// the sketch of the group: two straights moving together
void sketch_group(ImDrawList &List, ImVec2 Low, ImVec2 High, ImU32 Line, ImU32 Made)
{
	List.AddLine(tile_point(Low, High, 0.0f, 0.2f), tile_point(Low, High, 1.0f, 0.2f), Line, 2.0f);
	List.AddLine(tile_point(Low, High, 0.0f, 0.8f), tile_point(Low, High, 1.0f, 0.8f), Line, 2.0f);
	for (auto const x : {0.3f, 0.7f})
	{
		List.AddLine(tile_point(Low, High, x, 0.32f), tile_point(Low, High, x, 0.68f), Made, 1.5f);
		List.AddTriangleFilled(tile_point(Low, High, x - 0.08f, 0.38f), tile_point(Low, High, x + 0.08f, 0.38f), tile_point(Low, High, x, 0.26f), Made);
		List.AddTriangleFilled(tile_point(Low, High, x - 0.08f, 0.62f), tile_point(Low, High, x + 0.08f, 0.62f), tile_point(Low, High, x, 0.74f), Made);
	}
}

// the radius of the curves the break and the S-curve make, and the lengths of their transition curves
void render_curve_settings(double &Radius, bool &Automatic, double &Transition)
{
	ImGui::TextDisabled("%s", STR_C("Curve R (m, 0: automatic)"));
	ImGui::SetNextItemWidth(-FLT_MIN);
	ImGui::InputDouble("##straightcurve", &Radius, 10.0, 100.0, "%.0f");
	Radius = std::max(0.0, Radius);
	item_tooltip("Radius of the curves made by the break and the S-curve; automatic: 1.5 times the minimum radius for the speed of the line");
	value_chips("straightcurves", Radius, {0.0, 300.0, 600.0, 1000.0, 2000.0}, "%.0f");
	ImGui::TextDisabled("%s", STR_C("Transition curves"));
	if (int transitions{Automatic ? 0 : 1}; segmented("straighttransitions", transitions,
	                                                  {{0, STR_C("From the line speed"), "Lengths of the transition curves from the speed of the paths and the limits of the curve design"},
	                                                   {1, STR_C("Given length"), "The same length of the transition curves at every curve"}}))
		Automatic = transitions == 0;
	if (false == Automatic)
	{
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::InputDouble("##straighttransition", &Transition, 1.0, 10.0, "%.1f m");
		Transition = std::max(0.0, Transition);
	}
}

// the straights of a group sharing a path with the line taken out of it
void leave_straight_set(std::vector<editor_track::straight> &Set, editor_track::straight const &Line)
{
	std::erase_if(Set, [&Line](editor_track::straight const &Member) { return std::ranges::any_of(Line.tracks, [&Member](TTrack const *Track) { return std::ranges::find(Member.tracks, Track) != Member.tracks.end(); }); });
}

// whether the ends a straight is given can't be laid: not numbers, or too near or too far apart
bool unlaid_ends(std::pair<glm::dvec3, glm::dvec3> const &End)
{
	auto const finite = [](glm::dvec3 const &Point) { return std::isfinite(Point.x) && std::isfinite(Point.y) && std::isfinite(Point.z); };
	return false == finite(End.first) || false == finite(End.second) || false == (glm::distance(End.first, End.second) >= 0.1 && glm::distance(End.first, End.second) <= 100000.0);
}

// the moved end of the straight put the typed length away from the pivot, the way it was dragged, or the way it lay where that tells none
glm::dvec3 typed_reach(glm::dvec3 const &Pivot, glm::dvec3 const &Original, glm::dvec3 Moved, double const Length)
{
	glm::dvec2 heading{Moved.x - Pivot.x, Moved.z - Pivot.z};
	if (glm::length(heading) < 1e-6)
		heading = {Original.x - Pivot.x, Original.z - Pivot.z};
	heading = glm::normalize(heading) * Length;
	Moved.x = Pivot.x + heading.x;
	Moved.z = Pivot.z + heading.y;
	return Moved;
}

// the paths turned and stretched about the pivot as the end of the straight goes from Original over to Moved, the grade going with it
std::function<glm::dvec3(glm::dvec3 const &)> turn_about(glm::dvec3 const &Pivot, glm::dvec3 const &Original, glm::dvec3 const &Moved)
{
	glm::dvec2 const before{Original.x - Pivot.x, Original.z - Pivot.z};
	glm::dvec2 const after{Moved.x - Pivot.x, Moved.z - Pivot.z};
	auto const lengthbefore{glm::length(before)};
	auto const lengthafter{std::max(0.1, glm::length(after))};
	auto const axis{before / std::max(1e-9, lengthbefore)};
	auto const newaxis{after / lengthafter};
	auto const scale{lengthafter / std::max(1e-9, lengthbefore)};
	auto const rise{Moved.y - Original.y};
	return [pivot = Pivot, axis, newaxis, scale, rise, lengthbefore](glm::dvec3 const &Point) {
		glm::dvec2 const offset{Point.x - pivot.x, Point.z - pivot.z};
		auto const along{glm::dot(offset, axis)};
		auto const across{axis.x * offset.y - axis.y * offset.x};
		auto const planar{plan_of(pivot) + newaxis * (along * scale) + glm::dvec2{-newaxis.y, newaxis.x} * across};
		return glm::dvec3{planar.x, Point.y + rise * along / std::max(1e-9, lengthbefore), planar.y};
	};
}

// a path of a straight, by where along the straight it lies
struct straight_span
{
	TTrack *track;
	int path;
	double from;
	double to;
	bool forward;
};

// the paths of the straight by where along it they lie
std::vector<straight_span> straight_spans(editor_track::straight const &Line)
{
	std::vector<straight_span> spans;
	for (std::size_t i = 0; i < Line.tracks.size(); ++i)
	{
		auto const &path{Line.tracks[i]->m_paths[Line.paths[i]]};
		auto const a{Line.along(path.points[segment_data::point::start])};
		auto const b{Line.along(path.points[segment_data::point::end])};
		spans.emplace_back(Line.tracks[i], Line.paths[i], std::min(a, b), std::max(a, b), a < b);
	}
	std::ranges::sort(spans, [](straight_span const &A, straight_span const &B) { return A.from < B.from; });
	return spans;
}

// a run of the spans of a straight, the first and the last of them, -1 for none
struct span_run
{
	int first{-1};
	int last{-1};
};

// the plain track among the spans of the run nearest to the span Middle, -1 for none
int nearest_plain(std::vector<straight_span> const &Spans, span_run const &Run, int const Middle)
{
	int best{-1};
	for (int i = Run.first; i <= Run.last; ++i)
		if (Spans[i].track->eType == tt_Normal && (best < 0 || std::abs(i - Middle) < std::abs(best - Middle)))
			best = i;
	return best;
}

// the run of the spans a reshape lays its curve over: of those from From to End, the plain tracks in a row around the one at Centre, or the
// nearest plain one to it; none where none is plain
span_run reshaped_run(std::vector<straight_span> const &Spans, double const From, double const End, double const Centre)
{
	span_run run;
	for (int i = 0; i < static_cast<int>(Spans.size()); ++i)
	{
		if (Spans[i].to > From && Spans[i].from < End)
		{
			if (run.first < 0)
				run.first = i;
			run.last = i;
		}
	}
	if (run.first < 0)
		return run;
	int middle{run.first};
	for (int i = run.first; i <= run.last; ++i)
		if (Spans[i].from <= Centre && Spans[i].to >= Centre)
			middle = i;
	if (Spans[middle].track->eType != tt_Normal)
	{
		middle = nearest_plain(Spans, run, middle);
		if (middle < 0)
			return {};
	}
	int low{middle};
	int high{middle};
	while (low > run.first && Spans[low - 1].track->eType == tt_Normal)
		--low;
	while (high < run.last && Spans[high + 1].track->eType == tt_Normal)
		++high;
	return {low, high};
}

// a point of the straight in the world, at a distance along it
glm::dvec3 straight_point(editor_track::straight const &Line, double const Along)
{
	auto const planar{plan_of(Line.start) + Line.direction * Along};
	return glm::dvec3{planar.x, Line.start.y + Line.grade * Along, planar.y};
}

// where the last path of the straight is split for a reshape reaching to To: past the stretch reshaped, or most of the way along where that
// ends near its end
double split_point(straight_span const &Span, double const To)
{
	auto const length{Span.to - Span.from};
	return To < Span.to - 1.0 ? std::max(To, Span.from + std::min(1.0, length * 0.5)) : Span.from + length * 0.7;
}

// the path of a span laid again as two pieces split at Cut, the paths made
std::vector<TTrack *> split_span(editor_track::straight const &Line, straight_span const &Span, double const Cut)
{
	editor_track::chain split;
	split.tracks = {Span.track};
	split.forward = {Span.forward};
	std::vector<segment_data> pieces(2);
	pieces[0].points[segment_data::point::start] = straight_point(Line, Span.from);
	pieces[0].points[segment_data::point::end] = straight_point(Line, Cut);
	pieces[1].points[segment_data::point::start] = straight_point(Line, Cut);
	pieces[1].points[segment_data::point::end] = straight_point(Line, Span.to);
	return editor_track::relay(split, pieces);
}

// the split of the last path of the straight taken back
void undo_split(std::vector<TTrack *> const &Created, std::vector<std::pair<TTrack *, editor_track::state>> const &States)
{
	for (auto *track : Created)
		editor_track::retire(*track);
	for (auto const &[track, saved] : States)
	{
		editor_track::apply(*track, saved);
		editor_track::commit({track});
	}
}

// the paths of the straight past the run, as a straight of their own
editor_track::straight straight_tail(editor_track::straight const &Line, std::vector<straight_span> const &Spans, int const Last)
{
	editor_track::straight tail;
	tail.direction = Line.direction;
	for (int i = Last + 1; i < static_cast<int>(Spans.size()); ++i)
	{
		tail.tracks.push_back(Spans[i].track);
		tail.paths.push_back(Spans[i].path);
	}
	tail.start = straight_point(Line, Spans[Last + 1].from);
	tail.end = Line.end;
	tail.length = Line.length - Spans[Last + 1].from;
	tail.grade = Line.grade;
	return tail;
}

// the paths a reshape changes: those the tail moves, and those of the run the curve is laid over
std::vector<TTrack *> reshaped_tracks(editor_track::straight const &Tail, std::vector<straight_span> const &Spans, span_run const &Run)
{
	std::vector<TTrack *> chainpaths;
	for (int i = Run.first; i <= Run.last; ++i)
		chainpaths.push_back(Spans[i].track);
	auto tracks{editor_track::straight_affected({Tail})};
	for (auto *track : chainpaths)
		if (std::ranges::find(tracks, track) == tracks.end())
			tracks.push_back(track);
	return tracks;
}

// the paths put back as they were kept
void restore_states(std::vector<std::pair<TTrack *, editor_track::state>> const &States, std::vector<TTrack *> const &Tracks)
{
	for (auto const &[track, saved] : States)
		editor_track::apply(*track, saved);
	editor_track::commit(Tracks);
}

// the states kept before the split go over those kept after it, for the paths in both
void merge_split_states(std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<std::pair<TTrack *, editor_track::state>> const &Split)
{
	for (auto const &entry : Split)
	{
		if (auto const found{std::ranges::find_if(States, [&entry](auto const &State) { return State.first == entry.first; })}; found != States.end())
			found->second = entry.second;
		else
			States.push_back(entry);
	}
}

// where along the straight the four clicks of the S-curve lie, in order, kept on it and apart
std::array<double, 4> detour_stations(editor_track::straight const &Line, std::vector<glm::dvec3> const &Detour)
{
	std::array<double, 4> stations{Line.along(Detour[0]), Line.along(Detour[1]), Line.along(Detour[2]), Line.along(Detour[3])};
	if (stations[0] > stations[3])
	{
		std::swap(stations[0], stations[3]);
		std::swap(stations[1], stations[2]);
	}
	for (auto &station : stations)
		station = std::clamp(station, 0.0, Line.length);
	stations[1] = std::clamp(stations[1], stations[0] + 1.0, stations[3] - 1.0);
	stations[2] = std::clamp(stations[2], stations[1], stations[3] - 1.0);
	return stations;
}

// the run of plain tracks the S-curve is laid over, from From to To; a path that isn't plain inside it before the last click starts it over
span_run detour_run(std::vector<straight_span> const &Spans, double const From, double const To, double const Last)
{
	span_run run;
	for (int i = 0; i < static_cast<int>(Spans.size()); ++i)
	{
		if (Spans[i].to > From && Spans[i].from < To && Spans[i].track->eType == tt_Normal)
		{
			if (run.first < 0)
				run.first = i;
			run.last = i;
		}
		else if (run.first >= 0 && Spans[i].from < To)
		{
			if (Spans[i].from > Last)
				break;
			run = {};
		}
	}
	return run;
}

// whether the side of the straight toward a neighbour follows the drag: with a curve there to fit again, nothing there, or the end held still
bool side_follows(bool const Curve, TTrack const *Neighbour, bool const Pivot)
{
	return Curve || Neighbour == nullptr || Pivot;
}

// the note of the straights tool, then the group of straights moving together with a button clearing it
void render_straights_status(auto &State)
{
	if (false == State.status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", State.status.c_str());
	if (false == State.set.empty())
	{
		ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.15f, 1.0f), STR_C("Group: %zu straights move together"), State.set.size());
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Clear")))
			State.set.clear();
	}
}

} // namespace

void straights_editor::draw_straights_overlay() const
{
	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
	glm::dvec3 const camera{Global.pCamera.Pos};
	draw_found_straights(projection, drawlist, m_straights.found, m_straights.listed, camera);
	for (auto const &member : m_straights.set)
		draw_straight(projection, drawlist, member, IM_COL32(255, 150, 30, 255), 4.0f);
	draw_straight(projection, drawlist, m_straights.current, overlay_color::selected, 4.0f);
	if (m_straights.tool_placed)
		draw_straight_tool(projection, drawlist, m_straights.tool, m_straights.tool_line, m_straights.tool_at, m_straights.tool_point, m_straights.tool_handle);
	if (m_straights.dragging && m_straights.fitting)
		draw_fitted_route(projection, drawlist, m_route.result);
	else if (m_straights.dragging)
		draw_dragged_paths(projection, drawlist, m_straights.drag_lines);
	else if (false == m_straights.current.tracks.empty())
		draw_straight_handles(projection, drawlist, m_straights.current, m_straights.handle);
	auto line{m_straights.dragging ? m_straights.drag_line : m_straights.current};
	if (line.tracks.empty())
		return;
	glm::dvec3 const offset{m_straights.dragging ? m_straights.preview_start - m_straights.drag_line.start : glm::dvec3{}};
	if (m_straights.dragging)
		drag_to(line, m_straights.preview_start, m_straights.preview_end);
	auto const offer{straight_parallel_offer(m_straights.dragging ? m_straights.drag_line : m_straights.current, offset)};
	for (auto const &other : m_straights.neighbours)
		draw_neighbour_tick(projection, drawlist, line, other, {offer.nearby && offer.neighbour == &other, offer.snaps, offer.spacing, offer.angle});
	if (offer.nearby && false == offer.snaps && std::abs(offer.correction) > 0.02)
	{
		glm::dvec2 const normal{-line.direction.y, line.direction.x};
		auto const shift{glm::dvec3{normal.x, 0.0, normal.y} * offer.correction};
		projection.line(drawlist, line.start + shift, line.end + shift, IM_COL32(80, 230, 255, 210), 3.5f);
	}
}

void straights_editor::find_neighbour_straights()
{
	auto &state{m_straights};
	state.neighbours.clear();
	auto const &line{state.current};
	if (line.tracks.empty())
		return;
	auto const middle{(line.start + line.end) * 0.5};
	std::vector<TTrack const *> visited(line.tracks.begin(), line.tracks.end());
	std::vector<scene::basic_section *> sections;
	simulation::Region->sections(middle, static_cast<float>(line.length * 0.5 + 60.0), sections);
	for (auto const *section : sections)
		for (auto const &cell : section->m_cells)
			for (auto *track : cell.m_paths)
				add_neighbour(state.neighbours, visited, line, track, state.tolerance);
}

parallel_offer straights_editor::straight_parallel_offer(editor_track::straight const &Line, glm::dvec3 const &Offset) const
{
	parallel_offer offer;
	glm::dvec2 const normal{-Line.direction.y, Line.direction.x};
	auto const middle{(Line.start + Line.end) * 0.5 + Offset};
	double best{kParallelHintWindow};
	for (auto const &other : m_straights.neighbours)
	{
		auto const sine{std::abs(Line.direction.x * other.direction.y - Line.direction.y * other.direction.x)};
		if (sine > std::sin(glm::radians(kParallelFindAngle)))
			continue;
		auto const distance{glm::dot(plan_of(other.start - middle), normal)};
		auto const angle{glm::degrees(std::abs(std::asin(std::clamp(sine, 0.0, 1.0))))};
		for (auto const spacing : m_straights.spacings)
		{
			auto const error{std::abs(std::abs(distance) - spacing)};
			if (error < best)
			{
				best = error;
				offer.nearby = true;
				offer.snaps = error < kParallelSnapWindow && angle <= kParallelSnapAngle;
				offer.spacing = spacing;
				offer.distance = std::abs(distance);
				offer.angle = angle;
				offer.correction = distance > 0.0 ? std::abs(distance) - spacing : spacing - std::abs(distance);
				offer.neighbour = &other;
			}
		}
	}
	return offer;
}

glm::dvec3 straights_editor::snap_straight_offset(editor_track::straight const &Line, glm::dvec3 const &Offset) const
{
	auto const offer{straight_parallel_offer(Line, Offset)};
	if (false == offer.snaps)
		return Offset;
	glm::dvec2 const normal{-Line.direction.y, Line.direction.x};
	return Offset + glm::dvec3{normal.x, 0.0, normal.y} * offer.correction;
}

glm::dvec3 straights_editor::snap_straight_direction(glm::dvec3 const &Pivot, glm::dvec3 const &Moved) const
{
	glm::dvec2 const offset{Moved.x - Pivot.x, Moved.z - Pivot.z};
	auto const length{glm::length(offset)};
	if (length < 1e-3)
		return Moved;
	auto const direction{offset / length};
	for (auto const &other : m_straights.neighbours)
	{
		if (auto const sine{direction.x * other.direction.y - direction.y * other.direction.x}; std::abs(sine) > std::sin(glm::radians(kParallelSnapAngle)))
			continue;
		auto const aligned{other.direction * (glm::dot(direction, other.direction) < 0.0 ? -1.0 : 1.0)};
		return {Pivot.x + aligned.x * length, Moved.y, Pivot.z + aligned.y * length};
	}
	return Moved;
}

void straights_editor::render_straights_ui()
{
	auto &state{m_straights};
	ImGui::PushTextWrapPos(0.0f);
	ImGui::TextDisabled("%s", STR_C("Straights recognized from chains of collinear straight paths.\nLMB: select a path to see the straight it belongs to"));
	ImGui::PopTextWrapPos();
	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.42f, 0.24f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.52f, 0.30f, 1.0f));
	if (ImGui::Button(STR_C("Recognize straights in the scenery"), ImVec2(-FLT_MIN, ImGui::GetFrameHeight() * 1.4f)))
	{
		state.found = editor_track::find_straights(state.minimum_length, state.tolerance);
		state.listed = -1;
		state.current_for = nullptr;
	}
	ImGui::PopStyleColor(2);
	if (ImGui::TreeNode(STR_C("What counts as one straight")))
	{
		if (begin_fields("##straighttolerance", 0.55f))
		{
			double angle{glm::degrees(state.tolerance.angle)};
			field_row("Angle tolerance (deg)");
			if (ImGui::InputDouble("##straightangle", &angle, 0.0, 0.0, "%.4f"))
				state.tolerance.angle = glm::radians(std::max(0.0, angle));
			item_tooltip("Largest change of the direction between the paths of one straight");
			field_row("Offset tolerance (m)");
			ImGui::InputDouble("##straightoffset", &state.tolerance.offset, 0.0, 0.0, "%.3f");
			item_tooltip("Largest distance of the paths from the common axis of the straight");
			field_row("Treat as straight above R (m)");
			ImGui::InputDouble("##straightradius", &state.tolerance.radius, 0.0, 0.0, "%.0f");
			item_tooltip("Curves of a larger radius count as straight");
			field_row("Minimum length (m)");
			ImGui::InputDouble("##straightminimum", &state.minimum_length, 0.0, 0.0, "%.0f");
			item_tooltip("Shorter straights aren't listed");
			ImGui::EndTable();
		}
		ImGui::TreePop();
	}
	if (false == state.found.empty())
	{
		ImGui::AlignTextToFramePadding();
		ImGui::Text(STR_C("%zu straights"), state.found.size());
		ImGui::SameLine();
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::InputTextWithHint("##straightfilter", STR_C("path name"), state.filter.data(), state.filter.size());
		ImGui::BeginChild("##straights", ImVec2(0.0f, 160.0f), true);
		for (int i = 0; i < static_cast<int>(state.found.size()); ++i)
		{
			auto const &line{state.found[i]};
			if (line.tracks.empty())
				continue;
			if (state.filter[0] != '\0' && std::ranges::none_of(line.tracks, [&state](TTrack const *Track) { return Track->name().find(state.filter.data()) != std::string::npos; }))
				continue;
			auto const label{std::format("{}.  {}##straight{}", i + 1, describe(line), i)};
			if (ImGui::Selectable(label.c_str(), state.listed == i))
			{
				state.listed = i;
				m_services.set_current_node(line.tracks[line.tracks.size() / 2]);
				m_build.focus_on_node(line.tracks[line.tracks.size() / 2]);
			}
		}
		ImGui::EndChild();
	}

}

void straights_editor::render_straight_ui()
{
	auto &state{m_straights};
	auto const &line{m_designer.current_straight()};
	if (line.tracks.empty())
	{
		if (false == state.status.empty())
			ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", state.status.c_str());
		return;
	}
	metric_tiles({{format("%.2f m", line.length), STR_C("length"), ImGui::GetColorU32(ImGuiCol_Text)},
	              {format("%.2f\xc2\xb0", line.azimuth), STR_C("azimuth"), ImGui::GetColorU32(ImGuiCol_Text)},
	              {format("%+.2f \xe2\x80\xb0", line.grade * 1000.0), STR_C("grade"), ImGui::GetColorU32(ImGuiCol_Text)},
	              {format("%zu", line.tracks.size()), STR_C("paths"), ImGui::GetColorU32(ImGuiCol_TextDisabled)}});
	if (false == m_straights.neighbours.empty())
		ImGui::TextDisabled("%s", STR_C("Nearby straight: the diamond snaps parallel at 4.00 / 4.50 / 4.75 m"));

	// what else can be done with the straight, as tiles with a sketch of it
	auto const gap{std::round(ImGui::GetFontSize() * 0.3f)};
	auto const width{std::floor((ImGui::GetContentRegionAvail().x - 2.0f * gap) / 3.0f)};
	using sketch = void (*)(ImDrawList &, ImVec2, ImVec2, ImU32, ImU32);
	auto const toolbutton = [&state, width](char const *Label, int const Tool, char const *Tooltip, sketch const Sketch) {
		if (bool const active{state.tool == Tool}; sketch_tile(Label, Label, active, width, Sketch))
		{
			state.tool = active ? 0 : Tool;
			state.tool_placed = false;
			state.handle = -1;
		}
		item_tooltip(Tooltip);
	};
	toolbutton(STR_C("Break"), 1, STR_C("Turns the rest of the straight from a point, a curve goes at the break (Ctrl+drag in the view does the same)"), sketch_break);
	ImGui::SameLine(0.0f, gap);
	toolbutton(STR_C("S-curve"), 2, STR_C("Shifts a piece of the straight sideways around an obstacle and back: four clicks on the straight (Shift+click in the view does the same)"), sketch_scurve);
	ImGui::SameLine(0.0f, gap);
	if (bool const member{in_straight_set(line)}; sketch_tile("##group", member ? STR_C("Leave the group") : STR_C("Move with others"), member, width, sketch_group))
	{
		if (member)
			leave_straight_set(state.set, line);
		else
			state.set.push_back(line);
	}
	item_tooltip("Straights of a group move together and stay parallel; Alt+click in the view adds another one");
	if (state.tool != 0)
		render_curve_settings(state.curve_radius, state.auto_transitions, state.transition);
	render_straights_status(state);

	if (false == ImGui::IsAnyItemActive())
	{
		state.edit_start = line.start;
		state.edit_end = line.end;
		state.edit_length = line.length;
		state.edit_azimuth = line.azimuth;
	}
	if (ImGui::CollapsingHeader(STR_C("Exact values")))
		render_straight_exact(line);
}

void straights_editor::render_straight_exact(editor_track::straight const &Line)
{
	auto &state{m_straights};
	if (begin_fields("##straightexact", 0.3f))
	{
		field_row("Start (x, y, z)");
		ImGui::InputScalarN("##straightstart", ImGuiDataType_Double, glm::value_ptr(state.edit_start), 3, nullptr, nullptr, "%.3f");
		field_row("End (x, y, z)");
		ImGui::InputScalarN("##straightend", ImGuiDataType_Double, glm::value_ptr(state.edit_end), 3, nullptr, nullptr, "%.3f");
		ImGui::EndTable();
	}
	if (ImGui::Button(STR_C("Apply ends"), ImVec2(-FLT_MIN, 0.0f)))
		straight_apply(Line, state.edit_start, state.edit_end);
	if (false == begin_fields("##straightmeasures", 0.3f))
		return;
	field_row("Length (m)");
	ImGui::InputDouble("##straightlength", &state.edit_length, 0.0, 0.0, "%.3f");
	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(1);
	if (ImGui::Button(STR_C("Set from the start##length"), ImVec2(-FLT_MIN, 0.0f)))
	{
		glm::dvec3 const direction{Line.direction.x, Line.grade, Line.direction.y};
		straight_apply(Line, Line.start, Line.start + direction * std::max(0.1, state.edit_length));
	}
	field_row("Azimuth (deg)");
	ImGui::InputDouble("##straightazimuth", &state.edit_azimuth, 0.0, 0.0, "%.5f");
	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex(1);
	if (ImGui::Button(STR_C("Turn around the start"), ImVec2(-FLT_MIN, 0.0f)))
	{
		auto const azimuth{glm::radians(state.edit_azimuth)};
		glm::dvec3 const direction{std::sin(azimuth), Line.grade, std::cos(azimuth)};
		straight_apply(Line, Line.start, Line.start + direction * Line.length);
	}
	ImGui::EndTable();
}

bool straights_editor::straights_active()
{
	return m_build.build_ui()->mode() == nodebank_panel::TRACK && m_services.current_track_tab() == editor_track_tab::straights && false == m_designer.current_straight().tracks.empty();
}

int straights_editor::straight_handle_hit()
{
	if (false == straights_active())
		return -1;
	auto const &line{m_designer.current_straight()};
	std::array<glm::dvec3, 3> const handles{line.start, line.end, (line.start + line.end) * 0.5};
	screen_projection const projection;
	float best = kHandleRadius * kHandleRadius;
	int hit{-1};
	for (int i = 0; i < 3; ++i)
	{
		if (auto const distance{projection.mouse_distance2(handles[i])}; distance < best)
		{
			best = distance;
			hit = i;
		}
	}
	return hit;
}

void straights_editor::straight_refresh()
{
	auto &state{m_straights};
	state.current_for = m_services.selected_track();
	state.current = state.current_for != nullptr ? editor_track::find_straight(*state.current_for, state.tolerance) : editor_track::straight{};
	m_build.neighbour_straights_found();
	for (auto &line : state.found)
		if (false == line.tracks.empty())
			line = editor_track::find_straight(*line.tracks.front(), state.tolerance);
	for (auto &line : state.set)
		if (false == line.tracks.empty())
			line = editor_track::find_straight(*line.tracks.front(), state.tolerance);
}

bool straights_editor::in_straight_set(editor_track::straight const &Line) const
{
	for (auto const &member : m_straights.set)
		for (auto const *track : Line.tracks)
			if (std::ranges::find(member.tracks, track) != member.tracks.end())
				return true;
	return false;
}

void straights_editor::straight_apply(editor_track::straight const &Line, glm::dvec3 const &Start, glm::dvec3 const &End)
{
	straights_apply({Line}, {{Start, End}});
}

void straights_editor::straights_apply(std::vector<editor_track::straight> const &Lines, std::vector<std::pair<glm::dvec3, glm::dvec3>> const &Ends)
{
	auto &state{m_straights};
	state.status.clear();
	if (std::ranges::any_of(Ends, unlaid_ends))
	{
		state.status = m_build.refusal(STR_C("The ends have to be numbers, from 0.1 m to 100 km apart"));
		return;
	}
	auto const tracks{editor_track::straight_affected(Lines)};
	if (false == m_build.tracks_editable_for(tracks, state.status))
		return;
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	for (auto *track : tracks)
		states.emplace_back(track, editor_track::capture(*track));
	m_services.push_track_snapshot(std::move(states));
	editor_track::move_straights(Lines, Ends);
	editor_track::commit(tracks);
	straight_refresh();
}

void straights_editor::render_straight_gizmo()
{
	auto &state{m_straights};
	if (state.tool_mouse)
	{
		glm::dvec3 const ground{m_build.cursor_ground_point()};
		state.tool_handle = {ground.x, state.tool_handle.y, ground.z};
		m_build.typed_turn_taken();
		return;
	}
	auto const &line{state.dragging ? state.drag_line : m_designer.current_straight()};
	if (state.handle < 0 || line.tracks.empty())
	{
		state.dragging = false;
		return;
	}

	std::array<glm::dvec3, 3> const anchors{line.start, line.end, (line.start + line.end) * 0.5};
	glm::dvec3 handled{anchors[state.handle]};
	// the ends go onto the ground, the middle moves the line in plan alone
	bool const held{m_build.handle_drag_moved(handled, state.handle == 2 ? nullptr : line.tracks.front())};
	if (held)
	{
		if (false == state.dragging && false == start_straight_drag(line))
			return;
		drag_straight(handled);
	}
	else if (state.dragging)
	{
		end_straight_drag();
	}
}

bool straights_editor::start_straight_drag(editor_track::straight const &Line)
{
	auto &state{m_straights};
	std::vector<editor_track::straight> lines;
	if (in_straight_set(Line))
	{
		for (auto const &member : state.set)
			if (false == member.tracks.empty())
				lines.push_back(editor_track::find_straight(*member.tracks.front(), state.tolerance));
	}
	else
	{
		lines.push_back(Line);
	}
	state.fitting = lines.size() == 1 && start_curve_fit(Line, state.handle);
	auto const tracks{state.fitting ? std::vector<TTrack *>{} : editor_track::straight_affected(lines)};
	if (false == m_build.tracks_editable_for(tracks, state.status))
	{
		m_build.refusal(state.status);
		state.fitting = false;
		return false;
	}
	state.dragging = true;
	state.drag_line = Line;
	state.drag_lines = lines;
	state.drag_states.clear();
	for (auto *track : tracks)
		state.drag_states.emplace_back(track, editor_track::capture(*track));
	if (false == state.fitting)
	{
		m_services.push_track_snapshot(state.drag_states);
		state.drag_step = m_build.history_size();
		m_build.tracks_dragged() = tracks;
	}
	return true;
}

void straights_editor::drag_straight(glm::dvec3 const &Handled)
{
	auto &state{m_straights};
	auto const &grabbed{state.drag_line};
	glm::dvec3 moved{Handled};
	std::function<glm::dvec3(glm::dvec3 const &)> transform;
	if (state.handle == 2)
	{
		auto offset{snap_straight_offset(grabbed, moved - (grabbed.start + grabbed.end) * 0.5)};
		if (auto const values{m_build.typed_numbers()}; false == values.empty())
			offset = glm::dvec3{-grabbed.direction.y, 0.0, grabbed.direction.x} * values[0];
		transform = [offset](glm::dvec3 const &Point) { return Point + offset; };
	}
	else
	{
		auto const pivot{state.handle == 0 ? grabbed.end : grabbed.start};
		auto const original{state.handle == 0 ? grabbed.start : grabbed.end};
		moved = snap_straight_direction(pivot, moved);
		if (auto const values{m_build.typed_numbers()}; false == values.empty() && values[0] > 0.1)
			moved = typed_reach(pivot, original, moved, values[0]);
		transform = turn_about(pivot, original, moved);
	}
	std::vector<std::pair<glm::dvec3, glm::dvec3>> ends;
	for (auto const &member : state.drag_lines)
		ends.emplace_back(transform(member.start), transform(member.end));
	state.preview_start = transform(grabbed.start);
	state.preview_end = transform(grabbed.end);
	if (state.fitting)
	{
		update_curve_fit(state.preview_start, state.preview_end);
		return;
	}
	for (auto const &[track, saved] : state.drag_states)
		editor_track::apply(*track, saved);
	editor_track::move_straights(state.drag_lines, ends);
	m_build.track_drag_moved();
	m_build.track_drag_commit(false);
}

void straights_editor::end_straight_drag()
{
	auto &state{m_straights};
	if (state.fitting)
	{
		state.fitting = false;
		state.dragging = false;
		state.drag_lines.clear();
		if (m_route.result.valid)
			m_designer.route_apply();
		straight_refresh();
		return;
	}
	m_build.track_drag_commit(true);
	state.dragging = false;
	m_build.tracks_dragged().clear();
	state.drag_states.clear();
	state.drag_lines.clear();
	straight_refresh();
}

void editor_mode::cancel_track_tools()
{
	auto &state{m_straights};
	if (state.dragging)
	{
		// the geometry goes back to what it was before the drag
		if (false == state.fitting)
		{
			for (auto const &[track, saved] : state.drag_states)
				editor_track::apply(*track, saved);
			m_track_dirty = true;
			commit_track_drag(true);
			m_track_drag.clear();
			drop_step(state.drag_step);
		}
		state.dragging = false;
		state.fitting = false;
		state.drag_states.clear();
		state.drag_lines.clear();
		m_build_tools->routes.straight_edit.straight_refresh();
	}
	m_extend.active = false;
	m_turntable.placing = false;
	m_signal.placing = false;
	m_signal.moving = false;
	m_handle_drag = {};
	if (m_route_dragging)
	{
		m_route_dragging = false;
		m_route.design = m_build_tools->shared.route_drag_from;
		route_update();
	}
	if (m_point_drag.active)
	{
		auto const step{m_point_drag.moved ? m_point_drag.step : 0};
		m_track_snap = {};
		m_record_watch = {};
		m_point_drag = {};
		m_track_drag.clear();
		m_track_drag_points.clear();
		m_track_joints.clear();
		drop_step(step);
	}
	if (m_switch.placing)
	{
		m_switch.placing = false;
		m_switch.status = STR_C("Placing of the switch cancelled");
	}
	else if (m_switch.armed >= 0)
	{
		m_switch.armed = -1;
		m_switch.status.clear();
	}
	m_track_point = {};
	m_route.vertex = -1;
	m_route.grip = -1;
	state.handle = -1;
	state.tool_placed = false;
	state.detour.clear();
}

void straights_editor::straight_reshape(editor_track::straight const &Line, double const From, double const To, std::function<glm::dvec3(glm::dvec3 const &)> const &Tail, double const Radius) // NOSONAR: the type of the tail is set by the declaration in straights_editor, which the callers share
{
	auto &state{m_straights};
	state.status.clear();
	m_infra_carried.clear();
	auto line{Line};
	auto spans{straight_spans(line)};
	auto run{reshaped_run(spans, From, To, (std::max(From, 0.0) + std::min(To, line.length)) * 0.5)};
	if (run.first < 0)
		return;

	std::vector<std::pair<TTrack *, editor_track::state>> splitstates;
	std::vector<TTrack *> splitcreated;
	if (run.last + 1 >= static_cast<int>(spans.size()))
	{
		auto const &lastspan{spans.back()};
		if (lastspan.track->eType != tt_Normal)
			return;
		auto const cut{split_point(lastspan, To)};
		splitstates.emplace_back(lastspan.track, editor_track::capture(*lastspan.track));
		splitcreated = split_span(line, lastspan, cut);
		line = editor_track::find_straight(*line.tracks.front(), state.tolerance);
		spans = straight_spans(line);
		run = reshaped_run(spans, From, std::min(To, cut), (std::max(From, 0.0) + std::min(To, line.length)) * 0.5);
		if (run.first < 0 || run.last + 1 >= static_cast<int>(spans.size()))
		{
			undo_split(splitcreated, splitstates);
			return;
		}
	}

	auto const tail{straight_tail(line, spans, run.last)};
	auto const tracks{reshaped_tracks(tail, spans, run)};
	if (false == m_build.tracks_editable_for(tracks, m_straights.status))
	{
		undo_split(splitcreated, splitstates);
		return;
	}
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	for (auto *track : tracks)
		states.emplace_back(track, editor_track::capture(*track));

	editor_track::move_straights({tail}, {{Tail(tail.start), Tail(tail.end)}});

	m_route.from = spans[run.first].track;
	m_route.to = spans[run.last].track;
	m_designer.route_reset();
	if (m_route.chain.tracks.empty() || false == m_route.chain.switches.empty())
	{
		if (false == m_route.chain.switches.empty())
			m_route.error = STR_C("The curve would run through a switch");
		restore_states(states, tracks);
		undo_split(splitcreated, splitstates);
		state.status = m_build.refusal(m_route.error);
		return;
	}
	m_route.design.start_reserve = 0.0;
	m_route.design.end_reserve = 0.0;
	for (auto &vertex : m_route.design.vertices)
	{
		vertex.radius = Radius;
		m_designer.route_recommend(vertex);
		if (false == state.auto_transitions)
		{
			vertex.transition_in = state.transition;
			vertex.transition_out = state.transition;
		}
	}
	m_build.route_recompute();
	if (false == m_route.result.valid)
	{
		restore_states(states, tracks);
		undo_split(splitcreated, splitstates);
		state.status = m_build.refusal(m_route.result.errors.empty() ? STR_C("The curve can't be fitted") : m_route.result.errors.front());
		return;
	}
	std::vector<std::size_t> intervals;
	auto pieces{alignment::pieces_at(m_route.result, m_route.design, m_route.chain.tracks.size(), editor_track::chain_joints(m_route.chain), intervals)};
	editor_track::keep_heights(m_route.chain, pieces);
	auto created{editor_track::relay(m_route.chain, pieces, &m_route.to, &intervals)};
	editor_track::commit(tracks);
	merge_split_states(states, splitstates);
	created.insert(created.begin(), splitcreated.begin(), splitcreated.end());
	{
		auto changed{created};
		std::ranges::copy(std::views::keys(states), std::back_inserter(changed));
		m_route_relay.digitize_drawn(changed);
	}
	m_services.push_track_snapshot(std::move(states), std::move(created));
	std::string error;
	editor_track::find_chain(m_route.from, m_route.to, m_route.chain, error, true);
	straight_refresh();
	state.status = STR_C("Done: click a curve to adjust it; Ctrl+Z takes it back") + m_build.infra_carried_status();
}

double straights_editor::straight_tool_radius(editor_track::straight const &Line) const
{
	if (m_straights.curve_radius > 0.0)
		return m_straights.curve_radius;
	double speed{-1.0};
	for (auto const *track : Line.tracks)
		speed = std::max(speed, editor_track::velocity(*track));
	if (speed <= 0.0)
		speed = 100.0;
	auto const recommended{alignment::recommend(speed, 1000.0, m_route.design.norms)};
	return std::max(150.0, std::ceil(recommended.radius_min * 1.5 / 50.0) * 50.0);
}

bool straights_editor::place_straight_tool()
{
	auto &state{m_straights};
	if (m_build.build_ui()->mode() != nodebank_panel::TRACK || m_services.current_track_tab() != editor_track_tab::straights || state.tool == 0)
		return false;
	if (state.tool == 2)
	{
		add_detour_point();
		return true;
	}
	auto const &line{m_designer.current_straight()};
	if (line.tracks.empty())
		return false;
	glm::dvec3 const ground{m_build.cursor_ground_point()};
	auto const along{std::clamp(line.along(ground), 0.0, line.length)};
	glm::dvec3 const axis{line.direction.x, line.grade, line.direction.y};
	state.tool_line = line;
	state.tool_at = along;
	state.tool_point = line.start + axis * along;
	auto const reach{std::min(50.0, std::max(1.0, line.length - along))};
	state.tool_handle = state.tool_point + axis * reach;
	state.tool_placed = true;
	state.handle = -1;
	return true;
}

void straights_editor::apply_straight_tool()
{
	auto &state{m_straights};
	auto const line{state.tool_line};
	auto const offset{plan_of(state.tool_handle - state.tool_point)};
	auto const radius{straight_tool_radius(line)};
	if (state.tool == 1)
	{
		if (glm::length(offset) < 1e-3)
			return;
		auto const unit{glm::normalize(offset)};
		auto const angle{std::atan2(line.direction.x * unit.y - line.direction.y * unit.x, glm::dot(line.direction, unit))};
		if (std::abs(angle) < 1e-5)
			return;
		glm::dvec2 const pivot{state.tool_point.x, state.tool_point.z};
		double speed{-1.0};
		for (auto const *track : line.tracks)
			speed = std::max(speed, editor_track::velocity(*track));
		auto transition{state.transition};
		if (state.auto_transitions)
			transition = alignment::recommend(speed > 0.0 ? speed : 100.0, radius, m_build.lay_norms()).transition;
		auto const reach{radius * std::tan(std::min(std::abs(angle), 3.0) * 0.5) + transition * 0.5 + 10.0};
		straight_reshape(line, state.tool_at - reach, state.tool_at + reach,
			[pivot, angle](glm::dvec3 const &Point) {
				glm::dvec2 const relative{Point.x - pivot.x, Point.z - pivot.y};
				auto const c{std::cos(angle)};
				auto const s{std::sin(angle)};
				return glm::dvec3{pivot.x + relative.x * c - relative.y * s, Point.y, pivot.y + relative.x * s + relative.y * c};
			},
			radius);
	}
	else
	{
		glm::dvec2 const normal{-line.direction.y, line.direction.x};
		auto const shift{glm::dot(offset, normal)};
		auto const length{glm::dot(offset, line.direction)};
		if (std::abs(shift) < 1e-3 || length < 1.0)
			return;
		auto const translation{normal * shift};
		straight_reshape(line, state.tool_at, state.tool_at + length,
			[translation](glm::dvec3 const &Point) { return glm::dvec3{Point.x + translation.x, Point.y, Point.z + translation.y}; },
			radius);
	}
	state.tool_placed = false;
}

void straights_editor::start_straight_gesture(int const Tool)
{
	auto &state{m_straights};
	state.tool_before = state.tool;
	state.tool = Tool;
	if (false == place_straight_tool())
	{
		state.tool = state.tool_before;
		return;
	}
	state.tool_mouse = true;
}

void straights_editor::finish_straight_gesture()
{
	auto &state{m_straights};
	state.tool_mouse = false;
	apply_straight_tool();
	state.tool = state.tool_before;
	state.tool_placed = false;
}

void straights_editor::toggle_straight_set(TTrack &Track)
{
	auto &state{m_straights};
	auto const line{editor_track::find_straight(Track, state.tolerance)};
	if (line.tracks.empty())
		return;
	if (in_straight_set(line))
		leave_straight_set(state.set, line);
	else
		state.set.push_back(line);
	m_services.set_current_node(&Track);
	m_build.track_tab_set(editor_track_tab::straights);
}

std::vector<glm::dvec3> straights_editor::detour_outline() const
{
	auto const &state{m_straights};
	std::vector<glm::dvec3> result;
	if (state.detour.empty())
		return result;
	auto const &line{state.detour_line};
	glm::dvec2 const normal{-line.direction.y, line.direction.x};
	auto points{state.detour};
	if (points.size() < 4)
		points.push_back(state.detour_mouse);
	auto const offset{points.size() >= 2 ? glm::dot(plan_of(points[1] - line.start), normal) : 0.0};
	auto const place = [&](double const Along, double const Lateral) {
		auto const planar{plan_of(line.start) + line.direction * Along + normal * Lateral};
		return glm::dvec3{planar.x, line.start.y + line.grade * Along, planar.y};
	};
	for (std::size_t i = 0; i < points.size(); ++i)
		result.push_back(place(line.along(points[i]), (i == 1 || i == 2) ? offset : 0.0));
	return result;
}

void straights_editor::add_detour_point()
{
	auto &state{m_straights};
	if (state.detour.empty())
	{
		auto const &line{m_designer.current_straight()};
		if (line.tracks.empty())
			return;
		state.detour_line = line;
	}
	state.detour.push_back(m_build.cursor_ground_point());
	if (state.detour.size() >= 4)
	{
		apply_detour();
		state.detour.clear();
	}
}

void straights_editor::apply_detour()
{
	auto &state{m_straights};
	auto line{editor_track::find_straight(*state.detour_line.tracks.front(), state.tolerance)};
	if (line.tracks.empty())
		return;
	glm::dvec2 const normal{-line.direction.y, line.direction.x};
	auto const offset{glm::dot(plan_of(state.detour[1] - line.start), normal)};
	auto const stations{detour_stations(line, state.detour)};
	if (std::abs(offset) < 0.01 || stations[3] - stations[0] < 3.0)
		return;

	auto const radius{straight_tool_radius(line)};
	auto const slope{std::atan(std::abs(offset) / std::max(1.0, std::min(stations[1] - stations[0], stations[3] - stations[2])))};
	double speed{-1.0};
	for (auto const *track : line.tracks)
		speed = std::max(speed, editor_track::velocity(*track));
	auto transition{state.transition};
	if (state.auto_transitions)
		transition = alignment::recommend(speed > 0.0 ? speed : 100.0, radius, m_build.lay_norms()).transition;
	auto const reach{radius * std::tan(slope * 0.5) + transition * 0.5 + 5.0};
	auto const from{std::max(0.0, stations[0] - reach)};
	auto const to{std::min(line.length, stations[3] + reach)};

	auto const spans{straight_spans(line)};
	auto const run{detour_run(spans, from, to, stations[3])};
	if (run.first < 0)
		return;

	m_route.from = spans[run.first].track;
	m_route.to = spans[run.last].track;
	m_designer.route_reset();
	if (m_route.chain.tracks.empty())
		return;
	auto &design{m_route.design};
	auto const startalong{line.along(design.start)};
	auto const endalong{line.along(design.end)};
	auto const place = [&line, &normal](double const Along, double const Lateral) {
		auto const planar{plan_of(line.start) + line.direction * Along + normal * Lateral};
		return planar;
	};
	alignment::vertex vertex;
	vertex.radius = radius;
	m_designer.route_recommend(vertex);
	if (false == state.auto_transitions)
	{
		vertex.transition_out = state.transition;
		vertex.transition_in = vertex.transition_out;
	}
	design.vertices.clear();
	auto first_vertex{vertex};
	first_vertex.offset = std::max(0.1, stations[0] - startalong);
	design.vertices.push_back(first_vertex);
	if (stations[2] - stations[1] < 1.0)
	{
		auto middle{vertex};
		middle.position = place((stations[1] + stations[2]) * 0.5, offset);
		design.vertices.push_back(middle);
	}
	else
	{
		auto second{vertex};
		second.position = place(stations[1], offset);
		auto third{vertex};
		third.position = place(stations[2], offset);
		design.vertices.push_back(second);
		design.vertices.push_back(third);
	}
	auto last_vertex{vertex};
	last_vertex.offset = std::max(0.1, endalong - stations[3]);
	design.vertices.push_back(last_vertex);
	m_build.route_recompute();
	if (false == m_route.result.valid)
		return;
	m_designer.route_apply();
	straight_refresh();
}

bool straights_editor::start_curve_fit(editor_track::straight const &Line, int const Handle)
{
	auto &state{m_straights};
	auto *before{editor_track::outside_neighbour(Line, false)};
	auto *after{editor_track::outside_neighbour(Line, true)};
	auto const curve_at = [&](TTrack *Neighbour, editor_track::curve &Curve) {
		Curve = {};
		return Neighbour != nullptr && Neighbour->eType == tt_Normal && editor_track::find_curve(*Neighbour, state.tolerance, m_route.design.norms.gauge, Curve) && Curve.reversals == 0 && std::abs(Curve.turn) > 1e-4;
	};
	state.fit_has_before = curve_at(before, state.fit_before);
	state.fit_has_after = curve_at(after, state.fit_after);
	if (false == state.fit_has_before && false == state.fit_has_after)
		return false;
	if (false == side_follows(state.fit_has_before, before, Handle == 1) || false == side_follows(state.fit_has_after, after, Handle == 0))
		return false;
	auto const own_end = [&Line](glm::dvec3 const &Point) {
		for (auto *track : Line.tracks)
			if (editor_track::touches(*track, Point))
				return track;
		return Line.tracks.empty() ? nullptr : Line.tracks.front();
	};
	auto const other_end = [](editor_track::curve const &Curve, glm::dvec3 const &Joint) { return editor_track::touches(*Curve.from, Joint) ? Curve.to : Curve.from; };
	m_route.from = state.fit_has_before ? other_end(state.fit_before, Line.start) : own_end(Line.start);
	m_route.to = state.fit_has_after ? other_end(state.fit_after, Line.end) : own_end(Line.end);
	m_designer.route_reset();
	if (m_route.chain.tracks.empty())
		return false;
	return true;
}

void straights_editor::update_curve_fit(glm::dvec3 const &Start, glm::dvec3 const &End)
{
	auto const &state{m_straights};
	auto &design{m_route.design};
	glm::dvec2 const point{Start.x, Start.z};
	glm::dvec2 const direction{glm::normalize(plan_of(End - Start))};
	glm::dvec2 const start{design.start.x, design.start.z};
	glm::dvec2 const end{design.end.x, design.end.z};
	auto const startdirection{glm::normalize(design.start_direction)};
	auto const enddirection{glm::normalize(design.end_direction)};
	auto const first{cross(point - start, direction) / cross(startdirection, direction)};
	auto const last{cross(end - point, direction) / cross(enddirection, direction)};
	auto const make = [&](editor_track::curve const &Curve, double const Offset) {
		alignment::vertex vertex;
		vertex.radius = Curve.radius > 0.0 ? std::round(Curve.radius) : 1000.0;
		m_designer.route_recommend(vertex);
		if (Curve.transition_in > 0.0)
			vertex.transition_in = std::round(Curve.transition_in);
		if (Curve.transition_out > 0.0)
			vertex.transition_out = std::round(Curve.transition_out);
		if (Curve.cant > 0.0)
			vertex.cant = std::round(Curve.cant);
		compound_from(Curve, vertex);
		vertex.offset = std::max(0.1, Offset);
		return vertex;
	};
	design.vertices.clear();
	if (state.fit_has_before)
		design.vertices.push_back(make(state.fit_before, first));
	else
	{
		design.start = Start;
		design.start_direction = direction;
		design.start_reserve = 0.0;
	}
	if (state.fit_has_after)
		design.vertices.push_back(make(state.fit_after, last));
	else
	{
		design.end = End;
		design.end_direction = direction;
		design.end_reserve = 0.0;
	}
	m_build.route_recompute();
}

// what the rest of the editor calls, kept on editor_mode: done by the build tools

parallel_offer editor_mode::straight_parallel_offer(editor_track::straight const &Line, glm::dvec3 const &Offset) const
{
	return m_build_tools->routes.straight_edit.straight_parallel_offer(Line, Offset);
}

void editor_mode::render_straights_ui()
{
	m_build_tools->routes.straight_edit.render_straights_ui();
}

void editor_mode::render_straight_ui()
{
	m_build_tools->routes.straight_edit.render_straight_ui();
}

int editor_mode::straight_handle_hit()
{
	return m_build_tools->routes.straight_edit.straight_handle_hit();
}

void editor_mode::straight_refresh()
{
	m_build_tools->routes.straight_edit.straight_refresh();
}

double editor_mode::straight_tool_radius(editor_track::straight const &Line) const
{
	return m_build_tools->routes.straight_edit.straight_tool_radius(Line);
}

bool editor_mode::place_straight_tool()
{
	return m_build_tools->routes.straight_edit.place_straight_tool();
}

void editor_mode::start_straight_gesture(int const Tool)
{
	m_build_tools->routes.straight_edit.start_straight_gesture(Tool);
}

void editor_mode::toggle_straight_set(TTrack &Track)
{
	m_build_tools->routes.straight_edit.toggle_straight_set(Track);
}

std::vector<glm::dvec3> editor_mode::detour_outline() const
{
	return m_build_tools->routes.straight_edit.detour_outline();
}

void editor_mode::add_detour_point()
{
	m_build_tools->routes.straight_edit.add_detour_point();
}

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
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <format>
#include <functional>
#include <iterator>
#include <ranges>
#include <set>
#include <string_view>
#include "application/editormodetrackshared.h"

using namespace track_mode;

namespace
{

segment_data straight_path(glm::dvec3 const &Start, glm::dvec3 const &End)
{
	segment_data path;
	path.points[segment_data::point::start] = Start;
	path.points[segment_data::point::end] = End;
	return path;
}

// a flag counted into a key by its weight
double flag_weight(bool const Flag, double const Weight)
{
	return Flag ? Weight : 0.0;
}

// the zones of the radii along a switch broken at a cut, where it falls inside one of them by more than 5 cm
void break_zones(std::vector<std::pair<double, double>> &Zones, double const Cut)
{
	auto const inside = [&](std::size_t const I) { return Cut > (I > 0 ? Zones[I - 1].first : 0.0) + 0.05 && Cut < Zones[I].first - 0.05; };
	auto const indices{std::views::iota(std::size_t{0}, Zones.size())};
	if (auto const found{std::ranges::find_if(indices, inside)}; found != indices.end())
		Zones.insert(Zones.begin() + static_cast<std::ptrdiff_t>(*found), {Cut, Zones[*found].second});
}

// the tracks of the cells of the sections within the radius of the point, in the order the sections and their cells hold them
std::vector<TTrack *> tracks_around(glm::dvec3 const &Point, float const Radius)
{
	std::vector<TTrack *> tracks;
	for (auto const *section : simulation::Region->sections(Point, Radius))
		for (auto const &cell : section->m_cells)
			tracks.insert(tracks.end(), cell.m_paths.begin(), cell.m_paths.end());
	return tracks;
}

// the names given one after another, with commas between them
std::string joined_names(std::vector<std::string> const &Names)
{
	std::string names;
	for (auto const &name : Names)
		names += (names.empty() ? "" : ", ") + name;
	return names;
}

// how far in plan the point is off the chord of the first path of the track, from its nearest point
double off_chord(TTrack const &Track, glm::dvec3 const &Point)
{
	auto const &path{Track.m_paths.front()};
	auto const a{plan_of(path.points[segment_data::point::start])};
	auto const b{plan_of(path.points[segment_data::point::end])};
	auto const segment{b - a};
	auto const t{std::clamp(glm::dot(plan_of(Point) - a, segment) / std::max(1e-9, glm::dot(segment, segment)), 0.0, 1.0)};
	return glm::distance(plan_of(Point), a + segment * t);
}

// the crossing of a straight with a track nearest a point so far
struct crossing_hit
{
	TTrack *track{nullptr};
	glm::dvec2 position{0.0};
	double distance{kSlipReach};
};

// the crossing of the straight with the path of the track, taken for the hit where it's nearer the point than the hit so far
void closer_crossing(editor_track::straight const &Line, glm::dvec3 const &Point, TTrack &Track, segment_data const &Path, crossing_hit &Hit)
{
	bezier const curve{Path};
	auto const straight{Path.points[segment_data::point::control1] == glm::dvec3{} && Path.points[segment_data::point::control2] == glm::dvec3{}};
	int const pieces{straight ? 1 : 16};
	auto previous{plan_of(Path.points[segment_data::point::start])};
	for (int i = 1; i <= pieces; ++i)
	{
		auto const next{straight ? plan_of(Path.points[segment_data::point::end]) : plan_of(curve.point(static_cast<double>(i) / pieces))};
		auto const segment{next - previous};
		if (auto const determinant{cross(Line.direction, segment)}; std::abs(determinant) > 1e-9)
		{
			auto const offset{previous - plan_of(Line.start)};
			auto const along{cross(offset, segment) / determinant};
			auto const fraction{cross(offset, Line.direction) / determinant};
			auto const position{plan_of(Line.start) + Line.direction * along};
			if (fraction >= -0.01 && fraction <= 1.01 && along >= 0.0 && along <= Line.length && glm::distance(position, plan_of(Point)) < Hit.distance)
			{
				Hit.distance = glm::distance(position, plan_of(Point));
				Hit.track = &Track;
				Hit.position = position;
			}
		}
		previous = next;
	}
}

// the start of the names of the tracks of a crossing or a slip of the type
char const *slip_prefix(std::string_view const Type)
{
	if (Type == "Skrz")
		return "editor_skrz";
	if (Type == "Rkp")
		return "editor_dkp";
	return "editor_dks";
}

// the part of the Chain between its switches which holds the Track; the chain itself where none does
editor_track::chain part_holding(editor_track::chain const &Chain, TTrack const *Track)
{
	for (auto const &part : editor_track::chain_parts(Chain))
		if (std::ranges::find(part.tracks, Track) != part.tracks.end())
			return part;
	return Chain;
}

// what's told of a switch laid free, Name, on from the track From or with none; Radius: of the curve it's bent into where it is,
// One_sided: whether it's bent so
std::string free_switch_text(std::string const &Name, TTrack const *From, double const Radius, bool const One_sided)
{
	if (Radius > 0.0)
		return format(STR_C("Switch %s laid on from %s bent into R %.0f m, %s"), Name.c_str(), From->name().c_str(), Radius, One_sided ? STR_C("one-sided") : STR_C("two-sided"));
	if (From != nullptr)
		return format(STR_C("Switch %s laid on from %s, its other ends free: lay on from them"), Name.c_str(), From->name().c_str());
	return format(STR_C("Switch %s laid with no track, its ends free: lay on from them"), Name.c_str());
}

// what's told of the drives placed at the switches of a slip or a double switch, Kind, the names of the switches Driven; empty where
// there's nothing to tell
std::string slip_drives_text(std::string const &Kind, std::vector<std::string> const &Driven)
{
	if (Kind == "slip")
	{
		if (Driven.empty())
			return {};
		auto const names{joined_names(Driven)};
		return format(STR_C("Slip with drives at its switches %s: the events of the name of a switch with + and - switch it, also in the map"), names.c_str());
	}
	if (Driven.size() != 2)
		return {};
	return format(STR_C("Double switch %s + %s with drives: events %s+, %s- and %s+, %s- switch them, also in the map"), Driven[0].c_str(), Driven[1].c_str(), Driven[0].c_str(), Driven[0].c_str(), Driven[1].c_str(), Driven[1].c_str());
}

// the straight plain track nearest the Ground in the plan, nearer than Best, other than the Own one; none where there's none
TTrack *nearest_straight_track(glm::dvec3 const &Ground, TTrack const *Own, double const Best, editor_track::straight_tolerance const &Tolerance)
{
	TTrack *target{nullptr};
	auto best{Best};
	for (auto *track : tracks_around(Ground, 40.f))
	{
		if (track == Own || track->eType != tt_Normal || track->m_editorremoved || false == editor_track::is_straight(*track, Tolerance))
			continue;
		if (auto const distance{off_chord(*track, Ground)}; distance < best)
		{
			best = distance;
			target = track;
		}
	}
	return target;
}

// the paths of the switch Own as its template holds them: across its main track from the Origin, to the Side its diverging track goes,
// and along the Axis of it
void crossover_locals(editor_track::switch_template &Shape, TTrack const &Own, glm::dvec3 const &Origin, glm::dvec2 const &Axis, double const Side)
{
	glm::dvec2 const perpendicular{-Axis.y, Axis.x};
	for (auto const i : {0, 1})
	{
		auto const &path{Own.m_paths[i]};
		auto &local{Shape.local[i]};
		local = path;
		auto const tolocal = [&perpendicular, &Axis, Side](glm::dvec3 const &Vector) { return glm::dvec3{glm::dot(plan_of(Vector), perpendicular) * Side, Vector.y, glm::dot(plan_of(Vector), Axis)}; };
		local.transform([&Origin, &tolocal](glm::dvec3 const &Point) { return tolocal(Point - Origin); }, tolocal);
		if (Side < 0.0)
		{
			local.rolls = {-local.rolls[0], -local.rolls[1]};
			for (auto &piece : local.pieces)
				piece.rolls = {-piece.rolls[0], -piece.rolls[1]};
		}
	}
}

// a crossover onto a track parallel to the switch at the Origin of its main track, the other switch facing its Frog and the Tangent of
// its diverging track there on the straight Line, of the Shape of the switch; false where it can't face it
template <typename Crossover_plan>
bool parallel_crossover(Crossover_plan &Plan, glm::dvec3 const &Frog, glm::dvec2 const &Tangent, editor_track::straight const &Line, editor_track::switch_template const &Shape, glm::dvec3 const &Origin)
{
	auto const faced{connection::face(Frog, Tangent, connection::of(Line), 1, Shape)};
	if (false == faced.valid)
		return false;
	Plan.along = faced.partner.along;
	Plan.direction = faced.partner.forward;
	Plan.side = faced.partner.side;
	Plan.paths = faced.partner.paths;
	Plan.inserts.clear();
	if (faced.length >= 0.01)
		Plan.inserts.push_back(faced.insert);
	Plan.connection.valid = true;
	Plan.connection.spacing = std::abs(connection::of(Line).lateral(Origin));
	Plan.connection.straights = {faced.length};
	return true;
}

// the way a switch on a straight going Direction goes, Forward, and the side its diverging track goes to, Side, for a drag by Offset from
// where it was pressed
void oriented(glm::dvec2 const &Offset, glm::dvec2 const &Direction, int &Forward, int &Side)
{
	Forward = glm::dot(Offset, Direction) >= 0.0 ? 1 : -1;
	Side = cross(Direction * static_cast<double>(Forward), Offset) >= 0.0 ? 1 : -1;
}

// whether a switch Length long, Along a straight Line_length long and going Forward from there, lies all on it
bool on_straight(double const Along, int const Forward, double const Length, double const Line_length)
{
	auto const begin{Forward > 0 ? Along : Along - Length};
	return begin >= -1e-6 && begin + Length <= Line_length + 1e-6;
}

} // namespace

std::vector<double> editor_mode::switch_memo_key() const
{
	auto const &tool{m_switch};
	auto const &io{ImGui::GetIO()};
	auto const heading{tool.free_heading.value_or(glm::dvec2{9.0})};
	return {tool.mouse.x, tool.mouse.y, tool.mouse.z, tool.point.x, tool.point.z, io.MousePos.x, io.MousePos.y, heading.x, heading.y, tool.free_end.position.x, tool.free_end.position.z,
	        static_cast<double>(reinterpret_cast<std::uintptr_t>(tool.free_end.track)), tool.along, tool.pressed, static_cast<double>(tool.armed), static_cast<double>(tool.connection),
	        flag_weight(tool.placing, 1.0) + flag_weight(tool.free, 2.0) + flag_weight(tool.curved, 4.0) + flag_weight(tool.one_sided, 8.0) + flag_weight(io.KeyShift, 16.0) + flag_weight(io.KeyCtrl, 32.0) +
	            flag_weight(io.KeyAlt, 64.0),
	        static_cast<double>(m_history.size())};
}

std::vector<segment_data> editor_mode::switch_preview() const
{
	if (auto key{switch_memo_key()}; false == m_switch_memo.fresh(key))
		m_switch_memo.keep(std::move(key), switch_preview_now());
	return m_switch_memo.result;
}

std::vector<segment_data> editor_mode::switch_preview_now() const
{
	auto const &tool{m_switch};
	if (false == tool.placing || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return {};
	if (tool.free)
		return free_switch_paths();
	if (tool.curved)
	{
		int forward;
		int side;
		curved_switch_pose(forward, side);
		if (tool.templates[tool.armed].double_slip)
		{
			curved_slip plan;
			if (false == plan_curved_slip(tool.templates[tool.armed], forward, side, plan))
				return {};
			auto lines{curved_slip_lines(plan, tool.templates[tool.armed].type)};
			auto const reaches{curved_slip_reaches(plan)};
			lines.insert(lines.end(), reaches.begin(), reaches.end());
			return lines;
		}
		return curved_switch_paths(tool.templates[tool.armed], tool.frame, curved_switch_start(forward), forward, side);
	}
	if (tool.templates[tool.armed].double_slip)
	{
		// the slip dragged out on a straight: its two straights crossing, the one it brings along through the quarter of the cursor
		slip_plan plan;
		if (false == plan_double_slip(tool.line, tool.point, tool.templates[tool.armed], plan, lone_slip_side()) || plan.replaces != nullptr)
			return {};
		return {straight_path(plan.ends[0], plan.ends[1]), straight_path(plan.ends[2], plan.ends[3])};
	}
	if (switch_link plan; plan_switch_link(plan))
	{
		plan.paths.insert(plan.paths.end(), plan.link.pieces.begin(), plan.link.pieces.end());
		return plan.paths;
	}
	auto const &line{tool.line};
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const forward{glm::dot(offset, line.direction) >= 0.0 ? 1 : -1};
	glm::dvec2 const direction{line.direction * static_cast<double>(forward)};
	auto const side{direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1};
	// where it's laid as it's let go: kept on the straight, drawn to the joints; on a straight too short for it, where it was pressed
	auto const &shape{tool.templates[tool.armed]};
	if (shape.length > line.length - 0.02)
		return editor_track::place_switch(shape, tool.point, direction, side, line.grade * forward);
	auto const from{switch_from(line, shape.length, tool.along, forward, true)};
	auto const start{line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * (forward > 0 ? from : from + shape.length)};
	return editor_track::place_switch(shape, start, direction, side, line.grade * forward);
}

std::string editor_mode::straight_end_note(editor_track::straight const &Line, bool const Far) const
{
	auto const end{Line.start + glm::dvec3{Line.direction.x, Line.grade, Line.direction.y} * (Far ? Line.length : 0.0)};
	for (std::size_t i = 0; i < Line.tracks.size(); ++i)
	{
		auto const *track{Line.tracks[i]};
		auto const &path{track->m_paths[Line.paths[i]]};
		for (auto const index : {segment_data::point::start, segment_data::point::end})
		{
			if (plan_distance(path.points[index], end) > 0.05)
				continue;
			auto const *other{index == segment_data::point::start ? track->CurrentPrev() : track->CurrentNext()};
			if (other == nullptr)
				return STR_C("the track ends here");
			if (other->eType == tt_Switch)
				return STR_C("a switch begins here");
			if (false == editor_track::is_straight(*other, m_straights.tolerance))
				return STR_C("a curve begins here");
			return STR_C("the line turns here");
		}
	}
	return STR_C("the straight ends here");
}

std::optional<editor_mode::switch_problem> editor_mode::straight_switch_problem(editor_track::straight const &Line, double const Length, double const Along, int const Direction) const
{
	auto const point = [&](double const Distance) { return Line.start + glm::dvec3{Line.direction.x, Line.grade, Line.direction.y} * Distance; };
	switch_problem problem;
	if (Length > Line.length - 0.02)
	{
		// all of the straight, and what ends it at both ends
		problem.text = format(STR_C("The straight is %.1f m long, too short for the %.1f m switch"), Line.length, Length);
		problem.paths.push_back(straight_path(point(0.0), point(Line.length)));
		problem.marks = {{point(0.0), straight_end_note(Line, false)}, {point(Line.length), straight_end_note(Line, true)}};
		return problem;
	}
	// dragged past the end of the straight: moved back onto it, the part past the end and what's there told
	auto const wanted{Direction > 0 ? Along : Along - Length};
	auto const from{std::clamp(wanted, 0.0, Line.length - Length)};
	if (std::abs(from - wanted) < 0.05)
		return std::nullopt;
	auto const beyond{wanted > from};
	auto const edge{beyond ? Line.length : 0.0};
	auto const note{straight_end_note(Line, beyond)};
	problem.text = format(STR_C("The switch goes back %.1f m to stay on the straight: %s"), std::abs(from - wanted), note.c_str());
	problem.paths.push_back(straight_path(point(edge), point(beyond ? wanted + Length : wanted)));
	problem.marks = {{point(edge), note}};
	return problem;
}

std::optional<editor_mode::switch_problem> editor_mode::switch_drag_problem() const
{
	auto const &tool{m_switch};
	if (false == tool.placing || tool.free || tool.connection >= 0 || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()) || tool.templates[tool.armed].double_slip)
		return std::nullopt;
	auto const length{tool.templates[tool.armed].length};
	if (false == tool.curved)
	{
		if (tool.line.tracks.empty())
			return std::nullopt;
		glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
		return straight_switch_problem(tool.line, length, tool.along, glm::dot(offset, tool.line.direction) >= 0.0 ? 1 : -1);
	}
	if (tool.frame.size() < 2)
		return std::nullopt;
	// into a curve: as long as the curve, kept in it
	auto const total{tool.frame.back().station};
	auto const end_of = [&](bool const Last) { return sample_at(tool.frame, Last ? total : 0.0).position; };
	switch_problem problem;
	if (length > total - 0.02)
	{
		problem.text = format(STR_C("The curve is %.1f m long, too short for the %.1f m switch"), total, length);
		for (std::size_t i = 1; i < tool.frame.size(); ++i)
			problem.paths.push_back(straight_path(tool.frame[i - 1].position, tool.frame[i].position));
		problem.marks = {{end_of(false), STR_C("the curve ends here")}, {end_of(true), STR_C("the curve ends here")}};
		return problem;
	}
	int forward;
	int side;
	curved_switch_pose(forward, side);
	auto const wanted{curved_switch_start(forward)};
	auto const station{std::clamp(wanted, forward > 0 ? 0.0 : length, forward > 0 ? total - length : total)};
	if (std::abs(station - wanted) < 0.05)
		return std::nullopt;
	// past the end of the curve along its tangent there
	auto const beyond{wanted > station};
	auto const edge{sample_at(tool.frame, beyond ? total : 0.0)};
	auto const past{beyond ? wanted - station : station - wanted};
	glm::dvec3 const out{edge.tangent.x * (beyond ? past : -past), 0.0, edge.tangent.y * (beyond ? past : -past)};
	problem.text = format(STR_C("The switch goes back %.1f m to stay in the curve: the curve ends here"), past);
	problem.paths.push_back(straight_path(edge.position, edge.position + out));
	problem.marks = {{edge.position, STR_C("the curve ends here")}};
	return problem;
}

void editor_mode::flash_switch_problem(switch_problem Problem)
{
	m_switch_flash = std::move(Problem);
	m_switch_flash_until = ImGui::GetTime() + 3.0;
}

void editor_mode::flash_switch_problem(std::string const &Text, std::vector<segment_data> Paths, std::vector<std::pair<glm::dvec3, std::string>> Marks)
{
	flash_switch_problem({Text, std::move(Paths), std::move(Marks)});
}

bool editor_mode::start_switch_placement()
{
	auto &tool{m_switch};
	if (tool.armed < 0 || m_track_tab != track_tab::turnout)
		return false;
	// what the drag makes is told again as it goes
	if (tool.recognised)
	{
		tool.connection = -1;
		tool.recognised = false;
		tool.partner = nullptr;
	}
	// with no plain track selected, or a press away from it, the click selects
	auto *selected{selected_track()};
	if (selected == nullptr || selected->eType != tt_Normal || selected->m_paths.empty())
		return false;
	glm::dvec3 const ground{cursor_level(selected->m_paths.front().points[segment_data::point::start].y)};
	tool.curved = false;
	tool.free = false;
	tool.status.clear();
	if (current_straight().tracks.empty())
		return start_curved_switch_placement(*selected, ground);
	auto const &line{current_straight()};
	auto const cursor{cursor_on_straight(line)};
	auto const along{std::clamp(line.along(cursor), 0.0, line.length)};
	auto const point{line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * along};
	if (plan_distance(point, cursor) > kSwitchReach)
		return false;
	if (tool.templates[tool.armed].double_slip)
	{
		// at a crossing, where it was pressed; with no track crossing here it's dragged out like a switch, the crossing track it brings
		// along through the quarter the cursor is let go in. Laid only once dragged, as a switch is
		slip_plan plan;
		auto const lone{false == plan_double_slip(line, cursor, tool.templates[tool.armed], plan) && plan.lone};
		tool.along = along;
		tool.point = lone ? point : cursor;
		tool.mouse = cursor;
		tool.line = line;
		tool.placing = true;
		return true;
	}
	tool.along = along;
	tool.point = point;
	tool.mouse = cursor;
	tool.line = line;
	tool.placing = true;
	return true;
}

bool editor_mode::start_curved_switch_placement(TTrack &Track, glm::dvec3 const &Ground)
{
	auto &tool{m_switch};
	// the track that takes no switch blinks
	auto const refuse = [this, &tool, &Track, &Ground](std::string const &Text) {
		tool.status = refused(Text);
		flash_switch_problem(Text, Track.m_paths, {{Ground, ""}});
		return false;
	};
	// a slip is bent into the curve, in a side track only, a crossing goes only where two straights cross
	if (tool.templates[tool.armed].double_slip && tool.templates[tool.armed].type == "Skrz")
		return refuse(STR_C("A crossing goes only where two straights cross"));
	editor_track::curve curve;
	editor_track::chain chain;
	std::string error;
	if (false == editor_track::find_curve(Track, m_straights.tolerance, m_route.design.norms.gauge, curve))
		return refuse(STR_C("The selected track is neither a recognized straight nor a curve"));
	if (false == editor_track::find_chain(curve.from, curve.to, chain, error, true))
		return refuse(error.empty() ? std::string{STR_C("The curve can't be followed")} : error);
	chain = part_holding(chain, &Track);
	tool.frame = sample_chain(chain);
	tool.frame_tracks = chain.tracks;
	// a crossover along the curve goes on onto the straights past its ends, its first switch drawn so as it's dragged
	if (tool.connection >= 0)
	{
		std::vector<curve_sample> frame;
		std::vector<TTrack *> tracks;
		if (curve_of(Track, m_straights.tolerance, m_route.design.norms.gauge, frame, tracks, kCrossoverStraights))
		{
			tool.frame = std::move(frame);
			tool.frame_tracks = std::move(tracks);
		}
	}
	if (tool.frame.size() < 2)
		return false;
	tool.curved = true;
	tool.along = nearest_station(tool.frame, Ground);
	tool.pressed = tool.along;
	tool.point = sample_at(tool.frame, tool.along).position;
	if (plan_distance(tool.point, Ground) > kSwitchReach)
		return false;
	tool.mouse = Ground;
	tool.placing = true;
	return true;
}

void editor_mode::finish_connection_placement()
{
	auto &tool{m_switch};
	if (tool.partner != nullptr)
		insert_partner_connection();
	else if (false == tool.curved)
		insert_connection();
	else
		insert_curved_connection();
	if (tool.recognised)
	{
		tool.connection = -1;
		tool.recognised = false;
		tool.partner = nullptr;
	}
}

void editor_mode::finish_switch_placement()
{
	auto &tool{m_switch};
	if (tool.free)
	{
		tool.placing = false;
		insert_free_switch();
		return;
	}
	if (tool.connection >= 0)
	{
		finish_connection_placement();
		return;
	}
	switch_link plan;
	auto const linked{plan_switch_link(plan)};
	// what the drag ran into blinks a while after it's let go: the switch moved back onto the straight or into the curve, or none laid
	auto const problem{switch_drag_problem()};
	tool.placing = false;
	if (tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return;
	if (tool.templates[tool.armed].double_slip && false == tool.curved)
	{
		insert_double_slip(tool.line, tool.point, tool.templates[tool.armed], lone_slip_side());
		return;
	}
	if (problem.has_value())
		flash_switch_problem(*problem);
	auto const steps{m_history.size()};
	// laid elsewhere than it was dragged to: told with what it's laid of
	auto const moved = [&]() {
		if (false == problem.has_value() || m_history.size() == steps)
			return;
		tool.status += (tool.status.empty() ? "" : "\n") + problem->text;
		ui()->set_status(tool.status, true);
	};
	if (linked)
	{
		insert_linked_switch(plan);
		return moved();
	}
	if (tool.curved)
	{
		int forward;
		int side;
		curved_switch_pose(forward, side);
		if (tool.templates[tool.armed].double_slip)
		{
			insert_curved_slip(forward, side);
			return;
		}
		tool.along = curved_switch_start(forward);
		insert_curved_switch(forward, side);
		return moved();
	}
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const forward{glm::dot(offset, tool.line.direction) >= 0.0 ? 1 : -1};
	glm::dvec2 const direction{tool.line.direction * static_cast<double>(forward)};
	auto const side{direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1};
	insert_switch(tool.line, tool.along, forward, side);
	moved();
	if (false == plan.reason.empty())
		ui()->set_status(plan.reason, true);
}

void editor_mode::insert_switch(editor_track::straight const &Line, double const Along, int const Direction, int const Side)
{
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	if (false == place_switch_on_straight(Line, m_switch.templates[m_switch.armed], nullptr, Along, Direction, Side, true, states, created, removed, switch_name_for_new()))
		return;
	auto const placed{created};
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(placed);
}


bool editor_mode::free_switch_offered() const
{
	auto const &tool{m_switch};
	return m_track_tab == track_tab::turnout && tool.connection < 0 && tool.armed >= 0 && tool.armed < static_cast<int>(tool.templates.size()) && false == tool.templates[tool.armed].double_slip;
}

editor_track::snap_target editor_mode::free_switch_end(glm::dvec3 const &Ground) const
{
	// the free end under the cursor on the screen, however high the tracks are against the ground the cursor is on: a scenery built
	// below or above its terrain; else the nearest one within reach in the plan
	if (false == m_cursor_override.has_value() && ImGui::IsMousePosValid())
	{
		auto const &io{ImGui::GetIO()};
		if (auto const end{snap_free_end(Ground, {io.MousePos.x, io.MousePos.y}, nullptr, kRailCategory, {})}; end.track != nullptr)
			return end;
	}
	editor_track::snap_target result;
	auto nearest{kLaySnapRadius};
	for (auto const &end : editor_track::free_ends(nullptr, kRailCategory, Ground, kLaySnapRadius + 100.0, {}))
		if (auto const distance{plan_distance(end.position, Ground)}; distance < nearest)
		{
			nearest = distance;
			result = end;
		}
	return result;
}

void editor_mode::start_free_switch(glm::dvec3 const &Ground)
{
	auto &tool{m_switch};
	if (false == free_switch_offered())
		return;
	// only on from the free end of a track: a press on the ground far from the tracks lays nothing
	auto const end{free_switch_end(Ground)};
	if (end.track == nullptr)
		return;
	tool.status.clear();
	tool.curved = false;
	tool.line = {};
	tool.free = true;
	tool.free_end = end;
	tool.free_heading.reset();
	tool.one_sided = false;
	// going on from the free end in its direction
	if (auto const outward{plan_of(-tool.free_end.direction)}; glm::length(outward) > 1e-9)
		tool.free_heading = glm::normalize(outward);
	tool.point = tool.free_end.position;
	tool.mouse = tool.point;
	tool.placing = true;
}

void editor_mode::free_switch_pose(glm::dvec3 &Start, glm::dvec2 &Heading, int &Side, double &Grade) const
{
	auto const &tool{m_switch};
	Start = tool.point;
	glm::dvec2 const offset{plan_of(tool.mouse - tool.point)};
	// till the drag is long enough to take the direction from, it follows the cursor, the diverging track to the left
	Heading = tool.free_heading.value_or(glm::length(offset) > 1e-6 ? glm::normalize(offset) : glm::dvec2{0.0, 1.0});
	Side = cross(Heading, offset) >= 0.0 ? 1 : -1;
	Grade = 0.0;
	if (tool.free_end.track != nullptr)
	{
		// the grade of the track it goes on from
		auto const &into{tool.free_end.direction};
		auto const run{glm::length(plan_of(into))};
		Grade = run > 1e-9 ? -into.y / run : 0.0;
	}
}

std::vector<segment_data> editor_mode::free_switch_paths() const
{
	auto const &tool{m_switch};
	if (tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return {};
	glm::dvec3 start;
	glm::dvec2 heading;
	int side;
	double grade;
	free_switch_pose(start, heading, side, grade);
	// dragged on past its end from a free end: bent, with the track into the free end the cursor is at
	if (free_bend bend; free_switch_bend(bend))
	{
		auto paths{bend.paths};
		if (bend.target.track != nullptr && bend.problem.empty())
			paths.insert(paths.end(), bend.link.pieces.begin(), bend.link.pieces.end());
		return paths;
	}
	return editor_track::place_switch(tool.templates[tool.armed], start, heading, side, grade);
}

void editor_mode::insert_free_switch()
{
	auto &tool{m_switch};
	free_bend bend;
	auto const bent{free_switch_bend(bend)};
	tool.free = false;
	// a click with no drag on the ground gives no direction; from a free end it's the end's
	if (tool.free_end.track == nullptr && false == tool.free_heading.has_value())
	{
		tool.status = refused(STR_C("Drag along the main track of the switch, then sideways to the side of the diverging one"));
		return;
	}
	auto const paths{bent ? bend.paths : free_switch_paths()};
	if (paths.size() < 2)
		return;
	auto const made{create_switch_set(tool.templates[tool.armed], paths, tool.free_end.track, switch_name_for_new())};
	if (made.empty())
		return;
	// the track into the free end the cursor was at, in the same step of the history
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created{made};
	auto const linked{bent && bend.target.track != nullptr && bend.problem.empty()};
	if (linked)
		link_tracks(bend.target, bend.link, bend.target.track->eType == tt_Normal ? *bend.target.track : *tool.free_end.track, states, created);
	push_track_snapshot(std::move(states), std::move(created), {});
	straight_refresh();
	switch_placed(made);
	auto text{free_switch_text(made.front()->name(), tool.free_end.track, bent ? bend.radius : 0.0, bend.side == bend.turn)};
	if (linked)
		text += "\n" + link_summary(bend.target, bend.link, editor_track::branch_radius(bend.paths[bend.diverging ? 1 : 0]));
	else if (bent && false == bend.problem.empty())
		text += "\n" + bend.problem;
	if (bent)
		for (auto const &warning : bend.report.warnings)
			text += "\n" + warning;
	for (auto const &warning : free_switch_exits(paths))
		text += "\n" + warning;
	tool.status = text + (tool.status.empty() ? std::string{} : "\n" + tool.status);
	ui()->set_status(tool.status, false);
}

void editor_mode::switch_placed(std::vector<TTrack *> const &Created)
{
	auto const found{std::ranges::find_if(Created, [](TTrack const *Track) { return Track != nullptr && Track->eType == tt_Switch; })};
	if (found == Created.end())
		return;
	auto const slip{std::ranges::count_if(Created, [](TTrack const *Track) { return Track != nullptr && Track->eType == tt_Switch; }) > 1};
	m_switch.placing = false;
	first_step_done(first_step::turnout);
	m_track_point = {};
	m_node = *found;
	ui()->set_node(m_node);
	if (slip)
	{
		auto const known{switch_store::named(**found)};
		if (false == known.has_value() || (known->kind != "double" && known->kind != "slip"))
			return;
		std::vector<std::string> driven;
		for (auto *track : Created)
			if (track != nullptr && track->eType == tt_Switch && place_switch_drive(*track))
			{
				join_last_steps();
				driven.push_back(track->name());
			}
		if (auto const text{slip_drives_text(known->kind, driven)}; false == text.empty())
		{
			m_switch.status = text;
			ui()->set_status(m_switch.status, false);
		}
		// a slip has a drive at each of its switches
		if (known->kind == "slip")
			return;
		advance_switch_name();
		return;
	}
	if (place_switch_drive(**found))
		join_last_steps();
	advance_switch_name();
}

void editor_mode::advance_switch_name()
{
	if (false == m_switch.keep_names.empty())
	{
		m_switch.keep_names.erase(m_switch.keep_names.begin());
		return;
	}
	std::string name{m_switch.name};
	auto digits{name.size()};
	while (digits > 0 && std::isdigit(static_cast<unsigned char>(name[digits - 1])))
		--digits;
	if (digits < name.size() && name.size() - digits < 9)
		name = std::format("{}{}", name.substr(0, digits), std::stoi(name.substr(digits)) + 1);
	else
		name.clear();
	*std::format_to_n(m_switch.name, static_cast<std::ptrdiff_t>(std::size(m_switch.name)) - 1, "{}", name).out = '\0';
}

std::string editor_mode::switch_name_for_new() const
{
	if (false == m_switch.keep_names.empty() && simulation::Paths.find(m_switch.keep_names.front()) == nullptr)
		return m_switch.keep_names.front();
	if (m_switch.name[0] != '\0')
		return m_switch.name;
	for (int i = 1;; ++i)
	{
		auto const name{std::format("z{}", i)};
		if (simulation::Paths.find(name) == nullptr && simulation::Events.FindEvent(name + "+") == nullptr)
			return name;
	}
}

void editor_mode::join_last_steps()
{
	if (m_history.size() < 2)
		return;
	auto &last{m_history.back()};
	// a step of more than the includes placed, as the paths laid after others were taken out, is undone together with the one before
	if (auto const more{false == last.tracks.empty() || false == last.created.empty() || false == last.removed.empty() || false == last.directives.empty() || false == last.sweeps.empty() ||
	                    false == last.sweeps_toggled.empty() || false == last.infra.empty() || last.alignments.has_value() || last.switch_records.has_value() || last.profiles || false == last.copies.empty() ||
	                    last.terrain != nullptr};
	    more)
	{
		last.joined = true;
		return;
	}
	auto instances{std::move(last.instances)};
	m_history.pop_back();
	auto &previous{m_history.back()};
	previous.instances.insert(previous.instances.end(), instances.begin(), instances.end());
}

namespace
{

// the height of the track at the distance along the straight: a straight in the plan need not be one in height, a fitted grade line
// has its vertical curves on it, and a joint laid by the average grade of the whole straight would part from its neighbours
double straight_height(editor_track::straight const &Line, double const Distance)
{
	for (std::size_t i = 0; i < Line.tracks.size(); ++i)
	{
		auto const &path{Line.tracks[i]->m_paths[Line.paths[i]]};
		auto const a{Line.along(path.points[segment_data::point::start])};
		auto const b{Line.along(path.points[segment_data::point::end])};
		if (Distance < std::min(a, b) - 1e-3 || Distance > std::max(a, b) + 1e-3)
			continue;
		bezier const curve{path};
		auto low{0.0};
		auto high{1.0};
		for (int step = 0; step < 24; ++step)
		{
			auto const middle{0.5 * (low + high)};
			if ((Line.along(curve.point(middle)) < Distance) == (a < b))
				low = middle;
			else
				high = middle;
		}
		return curve.point(0.5 * (low + high)).y;
	}
	return Line.start.y + Line.grade * Distance;
}

// the paths of the tracks of the straight
std::vector<segment_data> line_paths(editor_track::straight const &Line)
{
	std::vector<segment_data> paths;
	for (std::size_t i = 0; i < Line.tracks.size(); ++i)
		paths.push_back(Line.tracks[i]->m_paths[Line.paths[i]]);
	return paths;
}

// a track of a straight: from where to where along it it goes, whether its path runs the way of the straight
struct straight_span
{
	TTrack *track;
	int path;
	double from;
	double to;
	bool forward;
};

// the tracks of the straight in the order they go along it
std::vector<straight_span> spans_along(editor_track::straight const &Line)
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

// the first and the last of the Spans overlapping From to To; -1 for both where none does
std::pair<int, int> overlapping(std::vector<straight_span> const &Spans, double const From, double const To)
{
	int first{-1};
	int last{-1};
	for (int i = 0; i < static_cast<int>(Spans.size()); ++i)
		if (Spans[i].to > From + 1e-3 && Spans[i].from < To - 1e-3)
		{
			if (first < 0)
				first = i;
			last = i;
		}
	return {first, last};
}

// the point of the straight at the Distance along it, at the height of its track there
glm::dvec3 straight_point(editor_track::straight const &Line, double const Distance)
{
	auto result{Line.start + glm::dvec3{Line.direction.x, 0.0, Line.direction.y} * Distance};
	result.y = straight_height(Line, Distance);
	return result;
}

// Count straight pieces of the straight one after another from Start to End along it, added to the Pieces
void straight_pieces(editor_track::straight const &Line, double const Start, double const End, std::size_t const Count, std::vector<segment_data> &Pieces)
{
	for (std::size_t i = 0; i < Count; ++i)
	{
		segment_data path;
		path.points[segment_data::point::start] = straight_point(Line, Start + (End - Start) * static_cast<double>(i) / static_cast<double>(Count));
		path.points[segment_data::point::end] = straight_point(Line, Start + (End - Start) * static_cast<double>(i + 1) / static_cast<double>(Count));
		Pieces.push_back(path);
	}
}

} // namespace

bool editor_mode::cut_straight(editor_track::straight const &Line, double const From, double const To, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack **Style)
{
	auto const from{From};
	auto const to{To};
	if (std::string reason; false == tracks_editable(Line.tracks, reason))
	{
		m_switch.status = refused(reason);
		flash_switch_problem(reason, line_paths(Line));
		return false;
	}
	auto const spans{spans_along(Line)};
	auto const [first, last]{overlapping(spans, from, to)};
	if (first < 0)
		return false;
	editor_track::chain chain;
	for (int i = first; i <= last; ++i)
	{
		if (spans[i].track->eType != tt_Normal)
			return false;
		chain.tracks.push_back(spans[i].track);
		chain.forward.push_back(spans[i].forward);
	}
	if (Style != nullptr)
		*Style = spans[first].track;
	auto const before{std::max(0.0, from - spans[first].from)};
	auto const after{std::max(0.0, spans[last].to - to)};
	bool const hasbefore{before > 0.01};
	bool const hasafter{after > 0.01};
	if (false == hasbefore && false == hasafter)
	{
		for (auto *track : chain.tracks)
			editor_track::retire(*track);
		Removed.insert(Removed.end(), chain.tracks.begin(), chain.tracks.end());
		return true;
	}
	auto const count{std::max<std::size_t>(chain.tracks.size(), (hasbefore ? 1 : 0) + (hasafter ? 1 : 0))};
	std::size_t countbefore{0};
	if (hasbefore && hasafter)
		countbefore = std::clamp<std::size_t>(static_cast<std::size_t>(std::round(static_cast<double>(count) * before / (before + after))), 1, count - 1);
	else if (hasbefore)
		countbefore = count;
	std::vector<segment_data> pieces;
	if (hasbefore)
		straight_pieces(Line, spans[first].from, from, countbefore, pieces);
	if (hasafter)
		straight_pieces(Line, to, spans[last].to, count - countbefore, pieces);
	for (auto *track : chain.tracks)
		States.emplace_back(track, editor_track::capture(*track));
	auto const relaid{editor_track::relay(chain, pieces)};
	Created.insert(Created.end(), relaid.begin(), relaid.end());
	return true;
}

bool editor_mode::place_switch_on_straight(editor_track::straight const &Line, editor_track::switch_template const &Shape, TTrack const *Style, double const Along, int const Direction, int const Side, bool const Snap, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, std::string const &Name)
{
	auto const &shape{Shape};
	if (shape.length > Line.length - 0.02)
	{
		m_switch.status = refused(format(STR_C("The straight is %.1f m long, too short for the %.1f m switch"), Line.length, shape.length));
		if (auto const problem{straight_switch_problem(Line, shape.length, Along, Direction)})
			flash_switch_problem(*problem);
		return false;
	}
	auto const from{switch_from(Line, shape.length, Along, Direction, Snap)};
	auto const to{from + shape.length};
	auto const origin{Direction > 0 ? from : to};
	// from the track as it lies there, before it's cut: the switch in its plane between the heights of the two ends
	auto const height_from{straight_height(Line, from)};
	auto const height_to{straight_height(Line, to)};
	TTrack *first{nullptr};
	if (false == cut_straight(Line, from, to, States, Created, Removed, &first))
	{
		// unless the tracks can't be changed, which it told
		if (std::string reason; tracks_editable(Line.tracks, reason))
		{
			m_switch.status = refused(STR_C("The switch would overlap another switch, move it onto plain track"));
			auto const point = [&](double const Distance) { return Line.start + glm::dvec3{Line.direction.x, Line.grade, Line.direction.y} * Distance; };
			flash_switch_problem(m_switch.status, {straight_path(point(from), point(to))});
		}
		return false;
	}
	auto start{Line.start + glm::dvec3{Line.direction.x, 0.0, Line.direction.y} * origin};
	start.y = Direction > 0 ? height_from : height_to;
	glm::dvec2 const direction{Line.direction * static_cast<double>(Direction)};
	auto const paths{editor_track::place_switch(shape, start, direction, Side, (height_to - height_from) / shape.length * Direction)};
	auto const made{create_switch_set(shape, paths, Style != nullptr ? *Style : *first, Name)};
	Created.insert(Created.end(), made.begin(), made.end());
	return true;
}

std::vector<TTrack *> editor_mode::create_switch_set(editor_track::switch_template const &Shape, std::vector<segment_data> const &Paths, TTrack const &Style, std::string const &Name)
{
	return create_switch_set(Shape, Paths, &Style, Name);
}

std::vector<TTrack *> editor_mode::create_switch_set(editor_track::switch_template const &Shape, std::vector<segment_data> const &Paths, TTrack const *Style, std::string const &Name)
{
	// with no track to copy, styled as the lay tool lays its tracks
	auto const switch_of = [&](editor_track::switch_template const &Template, std::vector<segment_data> const &Each, std::string const &Named) {
		return Style != nullptr ? editor_track::create_switch(Template, Each, *Style, Named) : editor_track::create_switch(Template, Each, m_lay.style, Named);
	};
	if (Shape.twofold() && Paths.size() >= 5)
	{
		auto firstshape{Shape};
		firstshape.length = glm::distance(Paths[0].points[segment_data::point::start], Paths[0].points[segment_data::point::end]);
		auto secondshape{Shape};
		secondshape.length = Shape.length - Shape.stagger;
		auto *one{switch_of(firstshape, Paths, Name)};
		auto *two{one != nullptr ? switch_of(secondshape, {Paths[2], Paths[3]}, Name + "b") : nullptr};
		TTrack *link{nullptr};
		if (two != nullptr)
			link = Style != nullptr ? editor_track::create_path(*Style, Paths[4]) : editor_track::create_path(m_lay.style, Paths[4]);
		if (link == nullptr)
		{
			m_switch.status = refused(STR_C("The switch couldn't be created from this template"));
			return {};
		}
		editor_track::commit({one, two, link});
		record_double(*one, *two, Shape);
		m_switch.status = format(STR_C("Double switch %s with its second switch %s inserted"), one->name().c_str(), two->name().c_str());
		return {one, two, link};
	}
	auto *track{switch_of(Shape, Paths, Name)};
	if (track == nullptr)
	{
		m_switch.status = refused(STR_C("The switch couldn't be created from this template"));
		return {};
	}
	editor_track::commit({track});
	record_switch(*track, Shape);
	m_switch.status = format(STR_C("Switch %s inserted, Ctrl+Z takes it back"), track->name().c_str());
	return {track};
}


bool editor_mode::plan_crossover(crossover_plan &Plan) const
{
	auto const &tool{m_extend};
	if (false == tool.active || tool.track == nullptr || tool.track->eType != tt_Switch || tool.track->m_paths.size() < 2 || tool.path != 1 || false == tool.atend)
		return false;
	auto const &own{*tool.track};
	auto const &main{own.m_paths[0]};
	auto const &diverging{own.m_paths[1]};
	auto const origin{main.points[segment_data::point::start]};
	auto const axis{glm::normalize(plan_of(main.points[segment_data::point::end] - origin))};
	glm::dvec2 const perpendicular{-axis.y, axis.x};
	auto const &frog{diverging.points[segment_data::point::end]};
	auto const lateral{glm::dot(plan_of(frog - origin), perpendicular)};
	if (std::abs(lateral) < 0.01)
		return false;
	auto const side{lateral > 0.0 ? 1.0 : -1.0};
	auto const &control_point{(diverging.pieces.empty() ? diverging : diverging.pieces.back()).points[segment_data::point::control2]};
	auto const tangent{glm::normalize(control_point != glm::dvec3{} ? -plan_of(control_point) : plan_of(frog - origin))};
	if (auto const angle{std::atan2(std::abs(cross(axis, tangent)), glm::dot(axis, tangent))}; angle < 1e-4)
		return false;

	auto const &ground{tool.mouse};
	// laying straights the end goes on as a straight, unless it's let go on the track
	auto *const target{nearest_straight_track(ground, &own, m_track_layout == track_layout::straight ? 1.5 : 8.0, m_straights.tolerance)};
	if (target == nullptr)
		return false;
	Plan.target = editor_track::find_straight(*target, m_straights.tolerance);
	auto const &line{Plan.target};
	if (line.tracks.empty())
		return false;
	auto const is_parallel{std::abs(cross(line.direction, axis)) <= std::sin(glm::radians(1.0))};
	// the track the switch is on, going on past it
	if (is_parallel && std::abs(connection::of(line).lateral(origin)) < 1.0)
		return false;
	editor_track::switch_template shape;
	shape.source = tool.track;
	shape.length = glm::dot(plan_of(main.points[segment_data::point::end] - origin), axis);
	crossover_locals(shape, own, origin, axis, side);
	Plan.shape = shape;
	Plan.connection = {};
	if (is_parallel)
		return parallel_crossover(Plan, frog, tangent, line, shape, origin);
	// onto a track at an angle, the switch facing it with an aligning curve between the two: a crossover planned from the
	// switch there, the other switch near where the end is let go
	double constexpr reach{2000.0};
	connection::request request;
	request.type = connection::kind::crossover;
	request.from.direction = axis;
	request.from.grade = (main.points[segment_data::point::end].y - origin.y) / std::max(1e-9, glm::length(plan_of(main.points[segment_data::point::end] - origin)));
	request.from.start = origin - glm::dvec3{axis.x, request.from.grade, axis.y} * reach;
	request.from.length = 2.0 * reach;
	request.to = {connection::of(line)};
	request.along = reach;
	request.forward = 1;
	request.target = request.to.front().along(ground);
	request.first = shape;
	request.second = shape;
	request.radius = m_switch.connection_radius;
	auto const planned{connection::plan(request)};
	if (false == planned.valid || planned.switches.size() < 2 || planned.switches.front().paths.size() < 2)
		return false;
	// on the side the switch there diverges to
	if (plan_distance(planned.switches.front().paths[1].points[segment_data::point::end], frog) > 0.05)
		return false;
	auto const &partner{planned.switches[1]};
	Plan.along = partner.along;
	Plan.direction = partner.forward;
	Plan.side = partner.side;
	Plan.paths = partner.paths;
	Plan.inserts = planned.inserts;
	Plan.connection = planned;
	return true;
}

std::string editor_mode::link_problem(editor_track::snap_target const &Target, editor_track::branch_link const &Link) const
{
	auto const name{Target.track != nullptr && false == Target.track->name().empty() ? Target.track->name() : std::string{STR_C("(no name)")}};
	if (false == Link.valid)
		return Link.shortfall > 0.0 ? format(STR_C("The branch can't lead into %s: the R %.0f m curve needs %.2f m more room"), name.c_str(), Link.radius, Link.shortfall) :
		                              format(STR_C("The branch can't lead into %s"), name.c_str());
	if (Link.trim > 0.0 && (false == stretchable(Target) || geometry::chord_length(Target.track->m_paths[Target.point.path]) < Link.trim + 1.0))
		return format(STR_C("%s would have to be shortened by %.2f m, it's too short or not straight"), name.c_str(), Link.trim);
	return {};
}

bool editor_mode::stretchable(editor_track::snap_target const &Target) const
{
	return Target.track != nullptr && Target.track->eType == tt_Normal && Target.track->m_paths.size() == 1 && editor_track::is_straight(*Target.track, m_straights.tolerance);
}

bool editor_mode::plan_switch_link(switch_link &Plan) const
{
	auto const &tool{m_switch};
	if (false == tool.placing || tool.curved || tool.connection >= 0 || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()) || tool.line.tracks.empty() || tool.templates[tool.armed].double_slip)
		return false;
	auto const &shape{tool.templates[tool.armed]};
	auto const &line{tool.line};
	std::vector<TTrack const *> const exclude(line.tracks.begin(), line.tracks.end());
	auto const category{line.tracks.front()->iCategoryFlag & 15};
	auto const place = [&](double const Along) {
		auto const point{line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * Along};
		return editor_track::place_switch(shape, point, line.direction * static_cast<double>(Plan.forward), Plan.side, line.grade * Plan.forward);
	};
	auto const &io{ImGui::GetIO()};
	if (false == m_cursor_override.has_value())
		Plan.target = snap_free_end(tool.mouse, {io.MousePos.x, io.MousePos.y}, nullptr, category, exclude);
	if (Plan.target.track == nullptr)
		Plan.target = editor_track::find_free_end(nullptr, category, tool.mouse, kLaySnapRadius, exclude);
	if (Plan.target.track != nullptr)
		oriented(plan_of(Plan.target.position - tool.point), line.direction, Plan.forward, Plan.side);
	else
	{
		oriented(plan_of(tool.mouse - tool.point), line.direction, Plan.forward, Plan.side);
		auto const paths{place(tool.along)};
		if (paths.size() < 2)
			return false;
		Plan.target = editor_track::find_free_end(nullptr, category, paths[1].points[segment_data::point::end], kLaySnapRadius, exclude);
		if (Plan.target.track == nullptr)
			return false;
	}
	if (shape.length > line.length - 0.02)
		return false;
	auto const kink{glm::radians(m_joints.kink_angle() * 0.5)};
	auto const plan = [&](double const Along) {
		Plan.along = Along;
		Plan.paths = place(Along);
		if (Plan.paths.size() < 2)
			return false;
		auto const radius{editor_track::branch_radius(Plan.paths[1])};
		Plan.link = editor_track::link_branch(branch_start(Plan.paths[1]), radius, branch_target(Plan.target), kink,
		                                      editor_track::reverse_insert(std::min(editor_track::diverging_speed(radius), design_speed(nullptr, Plan.target.track))), stretchable(Plan.target));
		Plan.reason = link_problem(Plan.target, Plan.link);
		return Plan.reason.empty();
	};
	auto const from{switch_from(line, shape.length, tool.along, Plan.forward, true)};
	auto const snapped{Plan.forward > 0 ? from : from + shape.length};
	auto const paths{place(snapped)};
	if (paths.size() < 2)
		return false;
	if (auto const shift{editor_track::branch_shift(branch_start(paths[1]), line.direction * static_cast<double>(Plan.forward), editor_track::branch_radius(paths[1]), branch_target(Plan.target), kink)}; shift.has_value())
	{
		auto const along{snapped + *shift * Plan.forward};
		Plan.moved = std::abs(*shift);
		if (on_straight(along, Plan.forward, shape.length, line.length) && plan(along) && false == Plan.link.reverse)
			return true;
	}
	Plan.moved = 0.0;
	return plan(snapped);
}

void editor_mode::insert_linked_switch(switch_link const &Plan)
{
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	if (false == place_switch_on_straight(m_switch.line, m_switch.templates[m_switch.armed], nullptr, Plan.along, Plan.forward, Plan.side, false, states, created, removed, switch_name_for_new()))
		return;
	auto const placed{created};
	link_tracks(Plan.target, Plan.link, Plan.target.track->eType == tt_Normal ? *Plan.target.track : *m_switch.line.tracks.front(), states, created);
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	m_switch.status.clear();
	switch_placed(placed);
	auto summary{link_summary(Plan.target, Plan.link, editor_track::branch_radius(Plan.paths[1]))};
	if (Plan.moved >= 0.005)
		summary += format(STR_C(", the switch moved %.2f m"), Plan.moved);
	m_switch.status = summary + (m_switch.status.empty() ? std::string{} : "\n" + m_switch.status);
	ui()->set_status(m_switch.status, false);
}

bool editor_mode::plan_branch_link(editor_track::branch_link &Link, std::string &Reason) const
{
	auto const &tool{m_extend};
	editor_track::snap_target target;
	Reason.clear();
	if (false == extend_snap(target) || tool.track == nullptr || tool.track->eType != tt_Switch || tool.track->m_paths.size() != 2 || tool.path != 1 || false == tool.atend)
		return false;
	if (std::ranges::find(tool.line, target.track) != tool.line.end())
		return false;
	auto const &diverging{tool.track->m_paths[1]};
	auto const radius{editor_track::branch_radius(diverging)};
	Link = editor_track::link_branch(branch_start(diverging), radius, branch_target(target), glm::radians(m_joints.kink_angle() * 0.5),
	                                 editor_track::reverse_insert(std::min(editor_track::diverging_speed(radius), design_speed(tool.track, target.track))), stretchable(target));
	Reason = link_problem(target, Link);
	return Reason.empty();
}

void editor_mode::finish_branch_link(editor_track::branch_link const &Link)
{
	auto *own{m_extend.track};
	auto const target{m_extend.snap};
	m_extend.active = false;
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	auto const &style{target.track->eType == tt_Normal || m_extend.line.empty() ? *target.track : *m_extend.line.front()};
	auto const linked{link_tracks(target, Link, style, states, created)};
	push_track_snapshot(std::move(states), std::move(created));
	m_node = linked.empty() ? own : linked.back();
	ui()->set_node(m_node);
	straight_refresh();
	ui()->set_status(link_summary(target, Link, editor_track::branch_radius(own->m_paths[1])), false);
}

std::vector<TTrack *> editor_mode::link_tracks(editor_track::snap_target const &Target, editor_track::branch_link const &Link, TTrack const &Style, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created)
{
	auto *target{Target.track};
	if (Link.trim != 0.0)
	{
		States.emplace_back(target, editor_track::capture(*target));
		editor_track::move_point(*target, Target.point, Link.landing);
		auto &path{target->m_paths[Target.point.path]};
		path.points[segment_data::point::control1] = glm::dvec3{0.0};
		path.points[segment_data::point::control2] = glm::dvec3{0.0};
		editor_track::commit({target});
	}
	std::vector<TTrack *> pieces;
	for (auto const &piece : Link.pieces)
		pieces.push_back(editor_track::create_path(Style, piece));
	editor_track::commit(pieces);
	digitize_drawn(pieces);
	Created.insert(Created.end(), pieces.begin(), pieces.end());
	return pieces;
}

namespace
{

// the part of the slip curve from Start along Heading, turning by Turn to Finish, between the distances From and To along it; where it
// changes the radius made of pieces, with one curve from end to end for the versions which know no pieces
segment_data slip_arc(editor_track::slip_curve const &Curve, glm::dvec3 const &Start, glm::dvec2 const &Heading, double const Turn, glm::dvec3 const &Finish, double const From, double const To)
{
	auto const whole{Curve.length()};
	auto const place = [&](double const Along) {
		auto const local{Curve.point(Along)};
		auto const planar{plan_of(Start) + Heading * local.x + glm::dvec2{-Heading.y, Heading.x} * (Turn * local.y)};
		return glm::dvec3{planar.x, Start.y + (Finish.y - Start.y) * Along / whole, planar.y};
	};
	auto const bend = [&](double const Begin, double const End, double const Radius, double const Handle0, double const Handle1) {
		auto const d0{turned(Heading, Turn * Curve.heading(Begin))};
		auto const d3{turned(Heading, Turn * Curve.heading(End))};
		segment_data path;
		path.points[segment_data::point::start] = place(Begin);
		path.points[segment_data::point::end] = place(End);
		if (Radius > 0.0)
		{
			path.points[segment_data::point::control1] = {d0.x * Handle0, 0.0, d0.y * Handle0};
			path.points[segment_data::point::control2] = {-d3.x * Handle1, 0.0, -d3.y * Handle1};
		}
		path.radius = static_cast<float>(Radius);
		return path;
	};
	std::vector<std::pair<double, double>> zones;
	for (auto const &zone : {std::pair{Curve.lead, 0.0}, {Curve.lead + Curve.ends, Curve.radius}, {Curve.lead + Curve.ends + Curve.between, Curve.middle}, {whole - Curve.lead, Curve.radius}, {whole, 0.0}})
	{
		if (zone.first <= (zones.empty() ? 0.0 : zones.back().first) + 1e-9)
			continue;
		if (false == zones.empty() && zone.second == zones.back().second)
			zones.back().first = zone.first;
		else
			zones.push_back(zone);
	}
	// broken at the tips of the blades of the tables, so the model of the switch finds them there
	if (Curve.tip > 0.0)
		for (auto const cut : {Curve.tip, whole - Curve.tip})
			break_zones(zones, cut);
	std::vector<segment_data> pieces;
	auto from{From};
	for (auto const &[to, radius] : zones)
	{
		auto const upto{std::min(To, to)};
		if (upto <= from + 1e-6)
			continue;
		auto const handle{radius > 0.0 ? 4.0 / 3.0 * std::tan((upto - from) / radius / 4.0) * radius : 0.0};
		pieces.push_back(bend(from, upto, radius, handle, handle));
		from = upto;
	}
	if (pieces.size() == 1)
		return pieces.front();
	// the handles of the curve from end to end reach 2/3 of the way to the crossing of the tangents
	auto const chord{plan_of(place(To) - place(From))};
	auto const d0{turned(Heading, Turn * Curve.heading(From))};
	auto const d3{turned(Heading, Turn * Curve.heading(To))};
	auto const sine{cross(d0, d3)};
	auto path{bend(From, To, Curve.radius, 2.0 / 3.0 * cross(chord, d3) / sine, 2.0 / 3.0 * cross(d0, chord) / sine)};
	path.pieces = std::move(pieces);
	return path;
}

// the crossing of the straight Line with the rail track of the Category nearest the Point, whatever that track is
crossing_hit nearest_crossing(editor_track::straight const &Line, glm::dvec3 const &Point, int const Category)
{
	crossing_hit hit;
	for (auto *track : tracks_around(Point, 300.f))
	{
		if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != Category || std::ranges::find(Line.tracks, track) != Line.tracks.end())
			continue;
		for (auto const &path : track->m_paths)
			closer_crossing(Line, Point, *track, path, hit);
	}
	return hit;
}

// a double slip laid out of straights within 25 m of the Point, nearer it than the Nearest crossing at Best and the nearest such: the one
// to replace; none where there's none
TTrack *slip_to_replace(glm::dvec3 const &Point, TTrack const *Nearest, double const Best)
{
	TTrack *slip{nullptr};
	double closest{25.0};
	auto const bent = [](TTrack const &Track) {
		auto const known{switch_store::named(Track)};
		return known.has_value() && known->main_radius != 0.0;
	};
	for (auto *track : tracks_around(Point, 40.f))
		if (track->eType == tt_Switch && false == track->m_editorremoved && track->DoubleSlip() && glm::distance(plan_of(track->location()), plan_of(Point)) < closest && false == bent(*track))
		{
			closest = glm::distance(plan_of(track->location()), plan_of(Point));
			slip = track;
		}
	if (slip != nullptr && (Nearest == nullptr || closest < Best))
		return slip;
	return nullptr;
}

// the curves of a slip of the Shape for the tracks crossing at the Angle into the Plan, as long as the straights have room for them,
// Available from the crossing, and how far it goes out from there; false where they haven't, Plan.problem telling. Gauged on the first
// track of the Line
template <typename Slip_plan>
bool slip_shaped(Slip_plan &Plan, editor_track::straight const &Line, editor_track::switch_template const &Shape, double const Angle, double const Available)
{
	bool const plain{Shape.type == "Skrz"};
	auto const *gauged{Line.tracks.front()};
	auto const [blade_lead, blade_entry]{switch_design::secant_blade(Shape.rail, Shape.radius, Shape.ratio)};
	auto curve{editor_track::slip_of(Shape.radius, Shape.middle, Angle, 0.5 * std::abs(gauged->fTrackWidth) + gauged->rail_dimensions().head + 0.002, blade_lead, blade_entry)};
	curve.tip = switch_design::cut_tip(Shape.rail, curve.radius);
	auto half{plain ? Shape.a : curve.tangent()};
	if (plain && half > Available)
	{
		half = Available;
		if (half < 2.0)
		{
			Plan.problem = STR_C("Not enough straight track around the crossing");
			return false;
		}
	}
	else if (half > Available)
	{
		curve = curve.scaled(Available / half);
		half = Available;
		if (curve.radius < 50.0)
		{
			Plan.problem = STR_C("Not enough straight track around the crossing");
			return false;
		}
		Plan.note = format(STR_C("R reduced to %d m, the straights end %d m from the crossing"), static_cast<int>(curve.radius), static_cast<int>(Available));
	}
	Plan.curve = curve;
	Plan.half = half;
	return true;
}

// a slip of the Shape dragged out on the straight Line by the Point where no track crosses it, into the Plan; false where no Side is
// given or the straight hasn't room for it
template <typename Slip_plan>
bool lone_slip(Slip_plan &Plan, editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape, int const Side)
{
	Plan.lone = true;
	Plan.problem = STR_C("No track crosses the selected straight under the cursor");
	if (Side == 0 || Line.tracks.empty())
		return false;
	// no track crossing here: the slip brings the crossing one along, at its own angle, ahead to the side given, its ends free
	auto const angle{std::atan(1.0 / (Shape.ratio > 0.0 ? Shape.ratio : 9.0))};
	auto const first{Line.direction};
	auto const direction{turned(first, static_cast<double>(Side) * angle)};
	auto const alongfirst{std::clamp(glm::dot(plan_of(Point) - plan_of(Line.start), first), 0.0, Line.length)};
	auto const crossing_point{plan_of(Line.start) + first * alongfirst};
	if (false == slip_shaped(Plan, Line, Shape, angle, std::min(alongfirst, Line.length - alongfirst) - 0.5))
		return false;
	Plan.problem.clear();
	auto const half{Plan.half};
	auto const height = [&Line](double const Along) { return Line.start.y + Line.grade * Along; };
	auto const at = [](glm::dvec2 const &Planar, double const Height) { return glm::dvec3{Planar.x, Height, Planar.y}; };
	Plan.crossing = crossing_point;
	Plan.first = first;
	Plan.direction = direction;
	Plan.angle = angle;
	Plan.alongfirst = alongfirst;
	Plan.points = {at(crossing_point - first * half, height(alongfirst - half)), at(crossing_point + first * half, height(alongfirst + half)), at(crossing_point - direction * half, height(alongfirst - half * std::cos(angle))),
	               at(crossing_point + direction * half, height(alongfirst + half * std::cos(angle)))};
	// the straights of both tracks out to the joints of the template, past the frogs of the diamond, as far as the line has room
	auto const reach{std::max(half, std::min(Shape.a, std::min(alongfirst, Line.length - alongfirst) - 0.5))};
	Plan.reach = reach;
	Plan.ends = {at(crossing_point - first * reach, height(alongfirst - reach)), at(crossing_point + first * reach, height(alongfirst + reach)), at(crossing_point - direction * reach, height(alongfirst - reach * std::cos(angle))),
	             at(crossing_point + direction * reach, height(alongfirst + reach * std::cos(angle)))};
	return true;
}

// a change given up: the tracks Created retired, the ones changed put back as they were in the States, the Removed ones revived
void edit_undone(std::vector<TTrack *> const &Created, std::vector<std::pair<TTrack *, editor_track::state>> const &States, std::vector<TTrack *> const &Removed)
{
	for (auto *track : Created)
		editor_track::retire(*track);
	for (auto const &[changed_track, changed_state] : States)
	{
		editor_track::apply(*changed_track, changed_state);
		editor_track::commit({changed_track});
	}
	for (auto *track : Removed)
	{
		editor_track::revive(*track);
		editor_track::commit({track});
	}
}

// the four switches of the double slip with the names of the Base, _a to _d; none where one of them is missing
std::optional<std::array<TTrack *, 4>> slip_switches(std::string const &Base)
{
	std::array<TTrack *, 4> switches{};
	for (int i = 0; i < 4; ++i)
	{
		switches[i] = simulation::Paths.find(Base + "_" + static_cast<char>('a' + i));
		if (switches[i] == nullptr || switches[i]->m_editorremoved || switches[i]->eType != tt_Switch || switches[i]->m_paths.size() < 2)
			return std::nullopt;
	}
	return switches;
}

// an end of a double slip: its switch, where it is, the way into the slip there, the track joined on there
struct slip_end
{
	TTrack *track;
	glm::dvec3 point;
	glm::dvec2 inward;
	TTrack *outer;
};

// the straight path of a switch: the last one with no controls, else its first one
segment_data const &straight_path_of(TTrack const &Track)
{
	segment_data const *straight{nullptr};
	for (auto const &path : Track.m_paths)
		if (path.points[segment_data::point::control1] == glm::dvec3{} && path.points[segment_data::point::control2] == glm::dvec3{})
			straight = &path;
	if (straight == nullptr)
		straight = &Track.m_paths.front();
	return *straight;
}

// the ends of the double slip of the Switches; the short tracks inside it added to its Parts
std::vector<slip_end> slip_ends(std::array<TTrack *, 4> const &Switches, std::vector<TTrack *> &Parts)
{
	std::vector<slip_end> ends;
	for (auto *track : Switches)
	{
		auto const &straight{straight_path_of(*track)};
		auto const start{straight.points[segment_data::point::start]};
		ends.emplace_back(track, start, glm::normalize(plan_of(straight.points[segment_data::point::end] - start)), track->SwitchExtension->pPrevs[0]);
		for (auto *inner : track->SwitchExtension->pNexts)
			if (inner != nullptr && inner->eType == tt_Normal && std::ranges::find(Parts, inner) == Parts.end() && inner->Length() < 5.0)
				Parts.push_back(inner);
	}
	return ends;
}

// the Ends of a double slip in the order A, B across from it, then C and D of the other track; none where no end lies across from A
std::optional<std::array<int, 4>> slip_order(std::vector<slip_end> const &Ends)
{
	int partner{-1};
	for (int i = 1; i < 4; ++i)
		if (glm::dot(Ends[0].inward, Ends[i].inward) < -0.999)
			partner = i;
	if (partner < 0)
		return std::nullopt;
	std::vector<int> others;
	for (int i = 1; i < 4; ++i)
		if (i != partner)
			others.push_back(i);
	if (glm::dot(Ends[0].inward, Ends[others[0]].inward) < 0.0)
		std::swap(others[0], others[1]);
	return std::array{0, partner, others[0], others[1]};
}

// how far from the Crossing a double slip laid again goes out along its tracks: Full as its curves of the Radius want, as far as the
// straights past its Ends go on, less a half metre; as far as it went, Oldhalf, where one of them isn't straight. Status tells the radius
// it's reduced to
double slip_half(std::array<slip_end const *, 4> const &Ends, glm::dvec2 const &Crossing, double const Full, double const Oldhalf, double const Radius, editor_track::straight_tolerance const &Tolerance, std::string &Status)
{
	auto half{Full};
	if (half <= Oldhalf)
		return half;
	for (auto const *e : Ends)
	{
		if (e->outer == nullptr || false == editor_track::is_straight(*e->outer, Tolerance))
		{
			half = Oldhalf;
			auto const radius{Radius * half / Full};
			Status = format(STR_C("R limited to %d m, a neighbouring track isn't straight"), static_cast<int>(radius));
			return half;
		}
		auto const line{editor_track::find_straight(*e->outer, Tolerance)};
		auto const reach{std::abs(glm::dot(plan_of(line.end) - Crossing, -e->inward))};
		auto const reachstart{std::abs(glm::dot(plan_of(line.start) - Crossing, -e->inward))};
		if (std::max(reach, reachstart) - 0.5 < half)
		{
			half = std::min(half, std::max(reach, reachstart) - 0.5);
			auto const radius{Radius * half / Full};
			Status = format(STR_C("R reduced to %d m to fit the straights"), static_cast<int>(radius));
		}
	}
	return half;
}

// the Parts with the tracks Created which aren't among them after them
std::vector<TTrack *> with_created(std::vector<TTrack *> const &Parts, std::vector<TTrack *> const &Created)
{
	std::vector<TTrack *> tracks(Parts.begin(), Parts.end());
	for (auto *track : Created)
		if (std::ranges::find(tracks, track) == tracks.end())
			tracks.push_back(track);
	return tracks;
}

// the sample of a frame at T along the Curve of the Path, the track run Forward or back: at the Position there, the Station along the
// frame
template <typename Sample>
Sample frame_sample(segment_data const &Path, bezier const &Curve, bool const Forward, double const T, glm::dvec3 const &Position, double const Station)
{
	auto const derivative{Forward ? Curve.first(T) : -Curve.first(T)};
	Sample sample;
	sample.station = Station;
	sample.position = Position;
	sample.tangent = glm::normalize(plan_of(derivative));
	auto const roll{Path.rolls[0] + (Path.rolls[1] - Path.rolls[0]) * T};
	sample.roll = static_cast<float>(Forward ? roll : -roll);
	sample.radius = Path.radius;
	return sample;
}

// how far a switch from First, Length long, is drawn to the joint of the Joints nearest one of its ends within Reach of it; nothing where
// no joint is that near
double joint_pull(std::vector<double> const &Joints, double const First, double const Length, double const Reach)
{
	auto nearest{Reach};
	auto shift{0.0};
	for (auto const joint : Joints)
		for (auto const end : {First, First + Length})
			if (auto const off{std::abs(joint - end)}; off > 1e-4 && off < nearest)
			{
				nearest = off;
				shift = joint - end;
			}
	return shift;
}

} // namespace

std::vector<TTrack *> editor_mode::build_double_slip(glm::dvec3 const &PointA, glm::dvec3 const &PointB, glm::dvec3 const &PointC, glm::dvec3 const &PointD, glm::dvec2 const &Crossing, double const Height, glm::dvec2 const &First, glm::dvec2 const &Second, double const Angle, editor_track::slip_curve const &Curve, TTrack const &Style, std::string const &Type, std::function<segment_data(segment_data const &)> const &Bend)
{
	auto const at = [](glm::dvec2 const &Planar, double const Y) { return glm::dvec3{Planar.x, Y, Planar.y}; };
	// the ends of the two diagonals at the crossing are kept apart further than the simulation joins the ends, 2 cm
	auto const halfsine{std::sqrt(std::max(1e-6, (1.0 - glm::dot(First, Second)) * 0.5))};
	auto const gap{std::max(0.25, 0.06 / halfsine)};
	auto const side{cross(First, Second) > 0.0 ? 1.0 : -1.0};
	auto const whole{Curve.length()};
	auto const arc = [&](glm::dvec3 const &Start, glm::dvec2 const &Heading, double const Turn, glm::dvec3 const &Finish, double const From, double const To) { return slip_arc(Curve, Start, Heading, Turn, Finish, From, To); };
	// laid out on a straight, bent where it goes into a curve
	auto const laid = [&](segment_data const &Path) { return Bend ? Bend(Path) : Path; };
	auto const back = [](segment_data const &Path) {
		auto result{reversed(Path)};
		std::ranges::reverse(result.pieces);
		for (auto &piece : result.pieces)
			piece = reversed(piece);
		return result;
	};
	auto const middle{whole * 0.5};
	auto const split{gap};
	auto const centreA{at(Crossing - First * gap, Height)};
	auto const centreB{at(Crossing + First * gap, Height)};
	auto const centreC{at(Crossing - Second * gap, Height)};
	auto const centreD{at(Crossing + Second * gap, Height)};
	auto const taken = [](std::string const &Base) { return simulation::Paths.find(Base + "_a") != nullptr || simulation::Paths.find(Base + "_d") != nullptr || switch_store::has(Base); };
	// the name of the one it's laid in place of, else the next free one; a crossing keeps the names of its tracks, only its record holds
	// the name
	auto base{std::exchange(m_switch.keep_slip, std::string{})};
	auto const prefix{slip_prefix(Type)};
	if (false == base.empty() && taken(base))
		base.clear();
	int number{1};
	while (base.empty() || taken(base))
	{
		base = std::format("{}{}", prefix, number);
		++number;
	}
	std::vector<TTrack *> parts;
	if (Type == "Skrz")
	{
		// the two tracks just cross, their ends far from the crossing
		parts.push_back(editor_track::create_path(Style, laid(straight_path(PointA, PointB))));
		parts.push_back(editor_track::create_path(Style, laid(straight_path(PointC, PointD))));
		std::erase(parts, nullptr);
		record_slip(parts, "crossing", Type, 0.0, Angle, base);
		return parts;
	}
	editor_track::switch_template shape;
	shape.length = glm::distance(plan_of(PointA), Crossing);
	bool const single{Type == "Rkp"};
	parts.push_back(editor_track::create_switch(shape, {laid(straight_path(PointA, centreA)), laid(arc(PointA, First, side, PointD, 0.0, middle - split))}, Style, base + "_a"));
	if (single)
		parts.push_back(editor_track::create_path(Style, laid(straight_path(PointB, centreB))));
	else
		parts.push_back(editor_track::create_switch(shape, {laid(straight_path(PointB, centreB)), laid(back(arc(PointC, Second, -side, PointB, middle + split, whole)))}, Style, base + "_b"));
	if (single)
		parts.push_back(editor_track::create_path(Style, laid(straight_path(PointC, centreC))));
	else
		parts.push_back(editor_track::create_switch(shape, {laid(straight_path(PointC, centreC)), laid(arc(PointC, Second, -side, PointB, 0.0, middle - split))}, Style, base + "_c"));
	parts.push_back(editor_track::create_switch(shape, {laid(straight_path(PointD, centreD)), laid(back(arc(PointA, First, side, PointD, middle + split, whole)))}, Style, base + "_d"));
	parts.push_back(editor_track::create_path(Style, laid(straight_path(centreA, centreB))));
	parts.push_back(editor_track::create_path(Style, laid(straight_path(centreC, centreD))));
	parts.push_back(editor_track::create_path(Style, laid(arc(PointA, First, side, PointD, middle - split, middle + split))));
	if (false == single)
		parts.push_back(editor_track::create_path(Style, laid(arc(PointC, Second, -side, PointB, middle - split, middle + split))));
	std::erase(parts, nullptr);
	record_slip(parts, "slip", Type, Curve.radius, Angle, base, Curve.middle);
	return parts;
}

glm::dvec3 editor_mode::cursor_on_straight(editor_track::straight const &Line) const
{
	// the height of the straight under the cursor, worked out anew till it settles: on a grade the plane of the start of the
	// straight would put the cursor metres away from where it points at the track
	auto cursor{cursor_level(Line.start.y)};
	for (int i = 0; i < 4; ++i)
		cursor = cursor_level(Line.start.y + Line.grade * std::clamp(Line.along(cursor), 0.0, Line.length));
	return cursor;
}

bool editor_mode::plan_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape, slip_plan &Plan, int const Side) const
{
	Plan = slip_plan{};
	// the crossing of the straight with the rail track nearest the cursor, whatever that track is: the slip goes there or
	// nowhere, never to another crossing around
	auto const category{Line.tracks.empty() ? 1 : Line.tracks.front()->iCategoryFlag & 15};
	auto const hit{nearest_crossing(Line, Point, category)};
	auto *const nearest{hit.track};
	auto const crossing{hit.position};
	auto const best{hit.distance};
	// a double slip nearer the cursor than any crossing is replaced by the one chosen, one bent into a curve isn't: it's laid out of
	// straights
	if (auto *const slip{Shape.type == "Rkpd" ? slip_to_replace(Point, nearest, best) : nullptr}; slip != nullptr)
	{
		Plan.replaces = slip;
		return true;
	}
	if (nearest == nullptr)
		return lone_slip(Plan, Line, Point, Shape, Side);
	bool const plain{Shape.type == "Skrz"};
	if (nearest->eType != tt_Normal)
	{
		Plan.problem = STR_C("The track crossing here is a switch: a slip goes only where two straights cross");
		return false;
	}
	if (false == editor_track::is_straight(*nearest, m_straights.tolerance))
	{
		Plan.problem = STR_C("The track crossing here isn't straight: a slip goes only where two straights cross");
		return false;
	}
	auto const second{editor_track::find_straight(*nearest, m_straights.tolerance)};
	if (second.tracks.empty())
	{
		Plan.problem = STR_C("The track crossing here isn't straight: a slip goes only where two straights cross");
		return false;
	}
	auto const first{Line.direction};
	auto direction{second.direction};
	if (glm::dot(first, direction) < 0.0)
		direction = -direction;
	auto const angle{std::atan2(std::abs(cross(first, direction)), glm::dot(first, direction))};
	if (angle < glm::radians(0.5) || angle > glm::radians(plain ? 89.0 : 30.0))
	{
		Plan.problem = STR_C("The straights cross at an angle unsuitable for a double slip");
		return false;
	}
	if (auto const skew{slip_skew(Shape, angle)}; false == skew.empty())
	{
		Plan.problem = skew;
		return false;
	}
	auto const alongfirst{glm::dot(crossing - plan_of(Line.start), Line.direction)};
	auto const alongsecond{glm::dot(crossing - plan_of(second.start), second.direction)};
	if (auto const available{std::min({alongfirst, Line.length - alongfirst, alongsecond, second.length - alongsecond}) - 0.5}; false == slip_shaped(Plan, Line, Shape, angle, available))
		return false;
	auto const half{Plan.half};
	auto const heightfirst = [&](double const Along) { return Line.start.y + Line.grade * Along; };
	auto const heightsecond = [&](double const Along) { return second.start.y + second.grade * Along; };
	auto const sign{glm::dot(second.direction, direction) > 0.0 ? 1.0 : -1.0};
	auto const at = [&](glm::dvec2 const &Planar, double const Height) { return glm::dvec3{Planar.x, Height, Planar.y}; };
	Plan.other = nearest;
	Plan.crossing = crossing;
	Plan.first = first;
	Plan.direction = direction;
	Plan.angle = angle;
	Plan.alongfirst = alongfirst;
	Plan.points = {at(crossing - first * half, heightfirst(alongfirst - half)), at(crossing + first * half, heightfirst(alongfirst + half)), at(crossing - direction * half, heightsecond(alongsecond - sign * half)),
	               at(crossing + direction * half, heightsecond(alongsecond + sign * half))};
	// the tracks crossing are there already, nothing laid out past the slip
	Plan.reach = half;
	Plan.ends = Plan.points;
	return true;
}

std::string editor_mode::slip_skew(editor_track::switch_template const &Shape, double const Angle) const
{
	// rad: the tracks crossing at the skos of the slip to within this, half of what keeps 1:9.51 and 1:10 of the catalogue apart
	auto const tolerance{glm::radians(0.14)};
	auto const fits = [&](editor_track::switch_template const &Template) { return Template.ratio > 0.0 && std::abs(Angle - std::atan(1.0 / Template.ratio)) <= tolerance; };
	if (Shape.type == "Skrz" || false == Shape.double_slip || Shape.ratio <= 0.0 || fits(Shape))
		return {};
	// a slip of the same kind made for the skos of the tracks, of the same rail and radius before the others
	editor_track::switch_template const *fitting{nullptr};
	auto best{-1};
	for (auto const &entry : m_switch.templates)
	{
		if (false == entry.double_slip || entry.type != Shape.type || false == fits(entry))
			continue;
		auto const rank{(entry.rail == Shape.rail ? 2 : 0) + (std::abs(entry.radius - Shape.radius) < 0.5 ? 1 : 0)};
		if (rank > best)
		{
			best = rank;
			fitting = &entry;
		}
	}
	auto const ratio{1.0 / std::tan(Angle)};
	if (fitting == nullptr)
		return format(STR_C("The straights cross at 1:%.2f, %s is made for 1:%.2f and no slip template for that: drag the slip out on one straight, its crossing track comes along at its skos"), ratio, switch_name_translated(Shape.title()).c_str(), Shape.ratio);
	return format(STR_C("The straights cross at 1:%.2f, %s is made for 1:%.2f: %s fits that crossing"), ratio, switch_name_translated(Shape.title()).c_str(), Shape.ratio, switch_name_translated(fitting->title()).c_str());
}

bool editor_mode::insert_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape, int const Side)
{
	auto &state{m_straights};
	state.status.clear();
	slip_plan plan;
	if (false == plan_double_slip(Line, Point, Shape, plan, Side))
	{
		state.status = refused(plan.problem);
		flash_switch_problem(plan.problem, {}, {{Point, ""}});
		return false;
	}
	if (plan.replaces != nullptr)
		return replace_double_slip(*plan.replaces, Shape);
	state.status = plan.note;
	auto *other{plan.other};
	auto const crossing{plan.crossing};
	auto const first{plan.first};
	auto const direction{plan.direction};
	auto const angle{plan.angle};
	auto const alongfirst{plan.alongfirst};
	auto const half{plan.half};
	auto const curve{plan.curve};
	auto const heightfirst = [&](double const Along) { return Line.start.y + Line.grade * Along; };
	auto const &pointA{plan.points[0]};
	auto const &pointB{plan.points[1]};
	auto const &pointC{plan.points[2]};
	auto const &pointD{plan.points[3]};

	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	TTrack *style{nullptr};
	// a slip bringing its crossing track along takes the line up to the joints of its template
	auto const reach{other == nullptr ? std::max(half, plan.reach) : half};
	if (false == cut_straight(Line, alongfirst - reach, alongfirst + reach, states, created, removed, &style))
		return false;
	auto const secondline{other != nullptr ? editor_track::find_straight(*other, m_straights.tolerance) : editor_track::straight{}};
	if (auto const secondalong{other != nullptr ? glm::dot(crossing - plan_of(secondline.start), secondline.direction) : 0.0}; other != nullptr && (secondline.tracks.empty() || false == cut_straight(secondline, secondalong - half, secondalong + half, states, created, removed, nullptr)))
	{
		edit_undone(created, states, removed);
		return false;
	}
	auto &styled{style != nullptr ? *style : *Line.tracks.front()};
	// a single slip joins the pair of the tracks on the side of the click
	auto const nearer{glm::distance((plan_of(pointA) + plan_of(pointD)) * 0.5, plan_of(Point)) <= glm::distance((plan_of(pointC) + plan_of(pointB)) * 0.5, plan_of(Point))};
	auto const parts{Shape.type == "Rkp" && false == nearer ? build_double_slip(pointC, pointD, pointA, pointB, crossing, heightfirst(alongfirst), direction, first, angle, curve, styled, Shape.type)
	                                                        : build_double_slip(pointA, pointB, pointC, pointD, crossing, heightfirst(alongfirst), first, direction, angle, curve, styled, Shape.type)};
	std::vector<TTrack *> made{parts};
	// the straights of both tracks from the ends of the slip out to the joints of the template
	if (reach > half + 0.01)
		for (auto const &[from, to] : {std::pair{plan.ends[0], pointA}, {pointB, plan.ends[1]}, {plan.ends[2], pointC}, {pointD, plan.ends[3]}})
			if (auto *straight{editor_track::create_path(styled, straight_path(from, to))})
				made.push_back(straight);
	editor_track::commit(made);
	created.insert(created.end(), made.begin(), made.end());
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(parts);
	if (other == nullptr)
	{
		state.status = format(STR_C("The slip laid with its crossing track at 1:%.1f, the ends of that track free: lay on from them"), 1.0 / std::tan(angle)) + (plan.note.empty() ? std::string{} : "\n" + plan.note);
		ui()->set_status(state.status, false);
	}
	return true;
}

int editor_mode::lone_slip_side() const
{
	auto const &tool{m_switch};
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const &first{tool.line.direction};
	// the crossing track through the quarter the cursor is in: ahead to the left or behind to the right turns it to the left
	auto const on_left{cross(first, offset) >= 0.0};
	auto const ahead{glm::dot(first, offset) >= 0.0};
	return on_left == ahead ? 1 : -1;
}

bool editor_mode::replace_double_slip(TTrack const &Part, editor_track::switch_template const &Shape)
{
	auto &state{m_straights};
	auto const base{Part.name().substr(0, Part.name().size() - 2)};
	auto const found{slip_switches(base)};
	if (false == found.has_value())
	{
		state.status = refused(STR_C("The double slip isn't complete, it can't be replaced"));
		return false;
	}
	auto const &switches{*found};
	std::vector<TTrack *> parts(switches.begin(), switches.end());
	auto const ends{slip_ends(switches, parts)};
	auto const sequence{slip_order(ends)};
	if (false == sequence.has_value())
	{
		state.status = refused(STR_C("The double slip geometry isn't recognized"));
		return false;
	}
	auto const &a{ends[(*sequence)[0]]};
	auto const &b{ends[(*sequence)[1]]};
	auto const &c{ends[(*sequence)[2]]};
	auto const &d{ends[(*sequence)[3]]};
	auto const first{a.inward};
	auto direction{c.inward};
	auto const determinant{cross(first, direction)};
	if (std::abs(determinant) < 1e-9)
		return false;
	auto const t{cross(plan_of(c.point) - plan_of(a.point), direction) / determinant};
	glm::dvec2 const crossing{plan_of(a.point) + first * t};
	auto const angle{std::atan2(std::abs(cross(first, direction)), glm::dot(first, direction))};
	if (auto const skew{slip_skew(Shape, angle)}; false == skew.empty())
	{
		state.status = refused(skew);
		return false;
	}
	auto const oldhalf{glm::distance(plan_of(a.point), crossing)};
	auto const [blade_lead, blade_entry]{switch_design::secant_blade(Shape.rail, Shape.radius, Shape.ratio)};
	auto curve{editor_track::slip_of(Shape.radius, Shape.middle, angle, 0.5 * std::abs(switches[0]->fTrackWidth) + switches[0]->rail_dimensions().head + 0.002, blade_lead, blade_entry)};
	curve.tip = switch_design::cut_tip(Shape.rail, curve.radius);
	auto const full{curve.tangent()};
	auto const height{(a.point.y + b.point.y) * 0.5};

	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	auto const half{slip_half({&a, &b, &c, &d}, crossing, full, oldhalf, curve.radius, m_straights.tolerance, state.status)};
	for (auto *track : parts)
		editor_track::retire(*track);
	removed.insert(removed.end(), parts.begin(), parts.end());
	TTrack *style{a.outer != nullptr && a.outer->eType == tt_Normal ? a.outer : switches[0]};
	std::array<glm::dvec3, 4> points;
	std::array<slip_end const *, 4> const order{&a, &b, &c, &d};
	for (int i = 0; i < 4; ++i)
	{
		auto const &e{*order[i]};
		auto const outward{-e.inward};
		auto const target{crossing + outward * half};
		points[i] = {target.x, e.point.y, target.y};
		if (half < oldhalf - 0.01)
		{
			auto *filler{editor_track::create_path(*style, straight_path(points[i], e.point))};
			created.push_back(filler);
		}
		else if (half > oldhalf + 0.01 && e.outer != nullptr)
		{
			auto const line{editor_track::find_straight(*e.outer, m_straights.tolerance)};
			auto const alongold{glm::dot(plan_of(e.point) - plan_of(line.start), line.direction)};
			auto const alongnew{glm::dot(target - plan_of(line.start), line.direction)};
			cut_straight(line, std::min(alongold, alongnew), std::max(alongold, alongnew), states, created, removed, nullptr);
		}
	}
	auto const parts2{build_double_slip(points[0], points[1], points[2], points[3], crossing, height, first, direction, angle, curve.scaled(half / full), *style)};
	editor_track::commit(with_created(parts2, created));
	created.insert(created.end(), parts2.begin(), parts2.end());
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	slip_drives_renewed(switches, parts2);
	return true;
}

void editor_mode::slip_drives_renewed(std::array<TTrack *, 4> const &Old, std::vector<TTrack *> const &New)
{
	// the drives of the switches replaced go with them, the new ones get theirs
	std::vector<scene::instance_handle> drives;
	for (auto const *track : Old)
		if (auto const drive{find_switch_drive(*track)})
			drives.push_back(drive->standing.instance);
	if (false == drives.empty() && remove_templates(drives) > 0)
		join_last_steps();
	for (auto *track : New)
		if (track != nullptr && track->eType == tt_Switch && place_switch_drive(*track))
			join_last_steps();
}

std::vector<editor_mode::curve_sample> editor_mode::sample_chain(editor_track::chain const &Chain)
{
	std::vector<curve_sample> result;
	double station{0.0};
	for (std::size_t i = 0; i < Chain.tracks.size(); ++i)
	{
		auto const &path{Chain.tracks[i]->m_paths.front()};
		auto const forward{Chain.forward[i]};
		bezier const curve{path};
		auto const length{glm::distance(curve.p0, curve.p3)};
		int const steps{std::max(8, static_cast<int>(std::ceil(length / 0.5)))};
		// the first point of a track going on right where the one before ended is that one's last; one past a switch the chain leaves
		// out starts apart from it, and its first point is taken too, the joint there on the frame where it is
		auto const continuous{false == result.empty() && glm::distance(curve.point(forward ? 0.0 : 1.0), result.back().position) < 1e-3};
		for (int k = (result.empty() || false == continuous ? 0 : 1); k <= steps; ++k)
		{
			auto const t{forward ? static_cast<double>(k) / steps : 1.0 - static_cast<double>(k) / steps};
			auto const position{curve.point(t)};
			if (false == result.empty())
				station += plan_distance(position, result.back().position);
			result.push_back(frame_sample<curve_sample>(path, curve, forward, t, position, station));
		}
	}
	return result;
}

editor_mode::curve_sample editor_mode::sample_at(std::vector<curve_sample> const &Frame, double const Station)
{
	if (Frame.empty())
		return {};
	if (Station <= Frame.front().station)
		return Frame.front();
	if (Station >= Frame.back().station)
		return Frame.back();
	auto const next{std::ranges::lower_bound(Frame, Station, {}, &curve_sample::station)};
	auto const previous{std::prev(next)};
	auto const span{std::max(1e-9, next->station - previous->station)};
	auto const f{(Station - previous->station) / span};
	curve_sample result;
	result.station = Station;
	result.position = glm::mix(previous->position, next->position, f);
	result.tangent = glm::normalize(glm::mix(previous->tangent, next->tangent, f));
	result.roll = static_cast<float>(previous->roll + (next->roll - previous->roll) * f);
	result.radius = previous->radius;
	return result;
}

double editor_mode::nearest_station(std::vector<curve_sample> const &Frame, glm::dvec3 const &Point)
{
	double best{std::numeric_limits<double>::max()};
	double station{0.0};
	for (std::size_t i = 0; i + 1 < Frame.size(); ++i)
	{
		glm::dvec2 const a{Frame[i].position.x, Frame[i].position.z};
		glm::dvec2 const b{Frame[i + 1].position.x, Frame[i + 1].position.z};
		auto const segment{b - a};
		auto const t{std::clamp(glm::dot(plan_of(Point) - a, segment) / std::max(1e-12, glm::dot(segment, segment)), 0.0, 1.0)};
		auto const distance{glm::distance(plan_of(Point), a + segment * t)};
		if (distance < best)
		{
			best = distance;
			station = Frame[i].station + (Frame[i + 1].station - Frame[i].station) * t;
		}
	}
	return station;
}

std::vector<switch_curve::track_point> editor_mode::frame_points(std::vector<curve_sample> const &Frame) const
{
	auto const gauge{m_route.design.norms.gauge};
	std::vector<switch_curve::track_point> points;
	for (auto const &sample : Frame)
	{
		switch_curve::track_point point;
		point.station = sample.station;
		point.heading = std::atan2(sample.tangent.y, sample.tangent.x);
		// the headings go on through a half turn, so that their differences are the turns between them
		if (false == points.empty())
			point.heading = points.back().heading + std::remainder(point.heading - points.back().heading, 2.0 * glm::pi<double>());
		point.cant = gauge * std::sin(glm::radians(static_cast<double>(sample.roll)));
		points.push_back(point);
	}
	return points;
}

void editor_mode::curved_switch_pose(int &Direction, int &Side) const
{
	auto const &tool{m_switch};
	auto const sample{sample_at(tool.frame, tool.pressed)};
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	Direction = glm::dot(offset, sample.tangent) >= 0.0 ? 1 : -1;
	glm::dvec2 const direction{sample.tangent * static_cast<double>(Direction)};
	Side = direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1;
}

double editor_mode::curved_switch_start(int const Direction, std::string *What) const
{
	auto const &tool{m_switch};
	if (What != nullptr)
		What->clear();
	if (tool.frame.size() < 2 || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return tool.pressed;
	auto const length{tool.templates[tool.armed].length};
	std::string what;
	auto start{switch_curve::snapped_start(switch_curve::ramp_ends(frame_points(tool.frame)), tool.pressed, length, Direction, 3.0, &what)};
	// as on a straight, an end of the switch within half a metre of a joint of the tracks goes right onto it: no path of a few centimetres
	// left beside it, no gap at it
	constexpr double kJointPull{0.5};
	auto const first{Direction > 0 ? start : start - length};
	// the switch drawn to the joint, if it's nearer an end of the switch than any joint before it
	std::vector<double> joints;
	for (auto const *track : tool.frame_tracks)
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
			continue;
		for (auto const index : {segment_data::point::start, segment_data::point::end})
			joints.push_back(nearest_station(tool.frame, track->m_paths.front().points[index]));
	}
	if (auto const shift{joint_pull(joints, first, length, kJointPull)}; shift != 0.0)
	{
		start += shift;
		what = STR("The switch drawn to the joint of the tracks");
	}
	// a start which would put the switch off the curve isn't one
	if (auto const total{tool.frame.back().station}; Direction > 0 ? start < 0.0 || start > total - length : start < length || start > total)
		return tool.pressed;
	if (What != nullptr)
		*What = what;
	return start;
}

namespace
{

// where a switch with a straight frog stops being bent, as the DB bends such switches into a curve: from the start of the switch up to the
// frog, both tracks straight over the frog; along the main track from the start of the switch, where the straight the diverging track
// ends with begins. None for a switch with its curve running through the frog, the ones of table 11.9, nor a double one
std::optional<double> frog_zone(editor_track::switch_template const &Shape)
{
	if (Shape.frog_straight < 0.5 || Shape.twofold() || Shape.double_slip || Shape.type != "Rz")
		return std::nullopt;
	auto const &diverging{Shape.local[1]};
	auto const &last{diverging.pieces.empty() ? diverging : diverging.pieces.back()};
	auto const &end{last.points[segment_data::point::end]};
	auto const &control_point{last.points[segment_data::point::control2]};
	auto direction{control_point != glm::dvec3{} ? -plan_of(control_point) : plan_of(end - last.points[segment_data::point::start])};
	if (glm::length(direction) < 1e-9)
		return std::nullopt;
	direction = glm::normalize(direction);
	// in the plan of a template x runs across the main track, z along it
	auto const along{end.z - Shape.frog_straight * direction.y};
	if (along < 1.0 || along > Shape.length - 0.5)
		return std::nullopt;
	return along;
}

// the pieces of a path of a switch template, one crossing the place along the main track split there
std::vector<segment_data> split_along(segment_data const &Path, double const Along)
{
	std::vector<segment_data> const pieces{Path.pieces.empty() ? std::vector<segment_data>{Path} : Path.pieces};
	std::vector<segment_data> result;
	for (auto const &piece : pieces)
	{
		auto const &start{piece.points[segment_data::point::start]};
		if (auto const &end{piece.points[segment_data::point::end]}; start.z > Along - 1e-3 || end.z < Along + 1e-3)
		{
			result.push_back(piece);
			continue;
		}
		bezier const curve{piece};
		double low{0.0};
		double high{1.0};
		for (int i = 0; i < 60; ++i)
		{
			auto const middle{0.5 * (low + high)};
			(curve.point(middle).z < Along ? low : high) = middle;
		}
		auto const t{0.5 * (low + high)};
		for (auto [from, to] : {std::pair{0.0, t}, std::pair{t, 1.0}})
		{
			auto part{geometry::part_of(piece, from, to)};
			part.pieces.clear();
			result.push_back(part);
		}
	}
	return result;
}

// a path from its start, in the plan, a point every quarter of a metre
std::vector<switch_curve::track_point> plan_points(segment_data const &Path)
{
	std::vector<switch_curve::track_point> points;
	auto const add = [&](segment_data const &Piece) {
		bezier const curve{Piece};
		auto const steps{std::max(4, static_cast<int>(std::ceil(curve.plan_length() / 0.25)))};
		for (int k = points.empty() ? 0 : 1; k <= steps; ++k)
		{
			auto const t{static_cast<double>(k) / steps};
			auto const position{plan_of(curve.point(t))};
			auto const tangent{plan_of(curve.first(t))};
			switch_curve::track_point point;
			point.heading = std::atan2(tangent.y, tangent.x);
			if (false == points.empty())
			{
				point.station = points.back().station + glm::distance(position, plan_of(curve.point(static_cast<double>(k - 1) / steps)));
				point.heading = points.back().heading + std::remainder(point.heading - points.back().heading, 2.0 * glm::pi<double>());
			}
			points.push_back(point);
		}
	};
	if (Path.pieces.empty())
		add(Path);
	for (auto const &piece : Path.pieces)
		add(piece);
	return points;
}

// a cut this near a joint isn't made: no path of a few centimetres, the end of the track there goes onto the end of the switch instead
constexpr double kJointSnap{0.05};

// a track of the scenery changed in the Edit: its state before kept once for Ctrl+Z, none for one made in it
void capture_once(editor_track::edit &Edit, TTrack &Track)
{
	if (std::ranges::find(Edit.created, &Track) == Edit.created.end())
		Edit.capture(Track);
}

// the track of the Candidates the Point falls on split there, unless the point is at a joint of it; the piece split off added to them
void split_at(std::vector<TTrack *> &Candidates, glm::dvec3 const &Point, editor_track::edit &Edit)
{
	for (auto *candidate : std::vector<TTrack *>(Candidates))
	{
		if (candidate->m_editorremoved || candidate->eType != tt_Normal)
			continue;
		auto const t{editor_track::nearest_parameter(*candidate, Point)};
		if (auto const found{editor_track::point_at(*candidate, t)}; plan_distance(found, Point) > 0.05)
			continue;
		auto const &own{candidate->m_paths.front()};
		if (auto const atjoint{std::min(plan_distance(own.points[segment_data::point::start], Point), plan_distance(own.points[segment_data::point::end], Point)) < kJointSnap}; false == atjoint && t > 0.002 && t < 0.998)
		{
			capture_once(Edit, *candidate);
			if (auto *second{editor_track::split_path(*candidate, t)})
			{
				Edit.created.push_back(second);
				Candidates.push_back(second);
			}
		}
		return;
	}
}

// the Candidates lying from From to To along a frame by the middle of each, Station_of telling where a point is along it, retired
template <typename Station_of>
void retired_between(std::vector<TTrack *> const &Candidates, double const From, double const To, Station_of const &Station, editor_track::edit &Edit)
{
	for (auto *candidate : Candidates)
	{
		if (candidate->m_editorremoved)
			continue;
		auto const middle{editor_track::point_at(*candidate, 0.5)};
		if (auto const where{Station(middle)}; where > From + 0.01 && where < To - 0.01)
		{
			editor_track::retire(*candidate);
			Edit.removed.push_back(candidate);
		}
	}
}

// the ends of the Candidates a few centimetres off the Target moved onto it, the tracks moved added to the Snapped
void snap_ends_to(std::vector<TTrack *> const &Candidates, glm::dvec3 const &Target, editor_track::edit &Edit, std::vector<TTrack *> &Snapped)
{
	for (auto *candidate : Candidates)
	{
		if (candidate->m_editorremoved || candidate->eType != tt_Normal)
			continue;
		for (auto const kind : {editor_track::point_kind::start, editor_track::point_kind::end})
		{
			if (auto const off{plan_distance(candidate->m_paths.front().points[kind == editor_track::point_kind::start ? segment_data::point::start : segment_data::point::end], Target)}; off < 1e-4 || off > kJointSnap)
				continue;
			capture_once(Edit, *candidate);
			editor_track::move_point(*candidate, {0, kind}, Target);
			if (std::ranges::find(Snapped, candidate) == Snapped.end())
				Snapped.push_back(candidate);
		}
	}
}

// the end of the Path touching the end of a switch, its end if Atend, moved onto the end of the switch, Switchend, heading the way the
// switch leaves it, Outward
void end_onto(segment_data &Path, bool const Atend, glm::dvec3 const &Switchend, glm::dvec2 const &Outward)
{
	auto &point{Path.points[Atend ? segment_data::point::end : segment_data::point::start]};
	auto &control_point{Path.points[Atend ? segment_data::point::control2 : segment_data::point::control1]};
	auto const other{Path.points[Atend ? segment_data::point::start : segment_data::point::end]};
	auto const handle{glm::length(plan_of(control_point)) > 1e-6 ? glm::length(plan_of(control_point)) : plan_distance(point, other) / 3.0};
	// into the track from the end touching the switch
	auto const into{Outward * (Atend ? -1.0 : 1.0) * handle};
	point = {Switchend.x, point.y, Switchend.z};
	control_point = {into.x, control_point.y, into.y};
}

// the track going on along the line from the end of a switch bent up to the frog, its Main track, where the curve goes on at Ending: its
// end there moved onto the end of the switch, heading as the switch leaves it. Tells how far off the curve that is and how much it's
// turned against the curve going on there, Heading; empty where no track goes on there
std::string track_fitted_on(segment_data const &Main, glm::dvec3 const &Ending, glm::dvec2 const &Heading, editor_track::edit &Edit)
{
	auto const &last{Main.pieces.empty() ? Main : Main.pieces.back()};
	auto const switchend{Main.points[segment_data::point::end]};
	auto const outward{-editor_track::end_direction(last, editor_track::point_kind::end)};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty() ||
		    std::ranges::find(Edit.removed, track) != Edit.removed.end())
			continue;
		auto const &path{track->m_paths.front()};
		auto const atend{plan_distance(path.points[segment_data::point::end], Ending) < 0.05};
		if (false == atend && plan_distance(path.points[segment_data::point::start], Ending) >= 0.05)
			continue;
		capture_once(Edit, *track);
		auto &own{track->m_paths.front()};
		end_onto(own, atend, switchend, outward);
		if (false == own.pieces.empty())
			end_onto(atend ? own.pieces.back() : own.pieces.front(), atend, switchend, outward);
		editor_track::commit({track});
		auto const turn{glm::degrees(std::acos(std::clamp(glm::dot(outward, glm::normalize(Heading)), -1.0, 1.0)))};
		return "\n" + format(STR_C("Track %s going on from the switch fitted to its end, which leaves the curve straight over the frog: %.1f cm off it, turned by %.2f deg"), track->name().c_str(),
		                     100.0 * plan_distance(switchend, Ending), turn);
	}
	return {};
}

// gentler than this a free switch isn't bent, the drag giving the side as along the straight
constexpr double kStraighter{12000.0};

// the Curvature of a free switch from Start along Heading into the straight of the free end Target is at, positive to the left: one
// curve from the start of the switch, none where the two hardly turn, Curvature as it was where they meet behind
double curvature_into(glm::dvec3 const &Start, glm::dvec2 const &Heading, editor_track::snap_target const &Target, double const Curvature)
{
	auto const into{glm::normalize(plan_of(Target.direction))};
	auto const turn{geometry::signed_angle(Heading, into)};
	auto const determinant{cross(Heading, into)};
	if (std::abs(turn) <= glm::radians(0.3))
		return 0.0;
	if (std::abs(determinant) > 1e-9)
	{
		// how far ahead the two straights meet
		auto const ahead{cross(plan_of(Target.position - Start), into) / determinant};
		if (ahead > 0.0)
			return std::copysign(std::tan(0.5 * std::abs(turn)) / ahead, turn);
	}
	return Curvature;
}

// the Curvature drawn to the radii a designer takes, the nearest one within 5 %, else to 10 m; none sharper than 100 m nor than half the
// Radius of the switch
double typical_curvature(double const Curvature, double const Radius)
{
	constexpr auto kRadii{std::to_array({10000.0, 8000.0, 6000.0, 5000.0, 4000.0, 3500.0, 3000.0, 2500.0, 2000.0, 1800.0, 1500.0, 1400.0, 1200.0, 1100.0, 1000.0, 900.0, 800.0, 700.0, 600.0, 500.0, 400.0, 300.0})};
	auto const radius{1.0 / std::abs(Curvature)};
	auto chosen{std::round(radius / 10.0) * 10.0};
	auto closest{0.05};
	for (auto const typical : kRadii)
		if (auto const off{std::abs(radius / typical - 1.0)}; off < closest)
		{
			closest = off;
			chosen = typical;
		}
	return std::copysign(1.0 / std::max(chosen, std::max(100.0, 0.5 * Radius)), Curvature);
}

// the curvature a free switch from Start along Heading is bent with, positive to the left, Toward: from the start to the cursor or the
// free end it's at, Along: how far ahead that is. Of the circle going on along the free end through the cursor, or into the straight of
// the free end, one curve from the start of the switch; with its diverging track into it, the switch bent so that the diverging track
// turns as much, its curve of the Radius taken off, Bend.side then the way it turns. None where it's hardly bent with no free end to go
// into
template <typename Free_bend>
std::optional<double> bend_curvature(Free_bend &Bend, glm::dvec3 const &Start, glm::dvec2 const &Heading, glm::dvec2 const &Toward, double const Along, double const Radius)
{
	auto const lateral{cross(Heading, Toward)};
	auto curvature{2.0 * lateral / (Along * Along + lateral * lateral)};
	if (Bend.target.track != nullptr && glm::length(plan_of(Bend.target.direction)) > 1e-9)
	{
		curvature = curvature_into(Start, Heading, Bend.target, curvature);
		// the main track turning the other way where the curve is gentler than the one of the switch, the same way where it's sharper
		if (Bend.diverging)
		{
			auto const way{curvature >= 0.0 ? 1 : -1};
			curvature = way * (std::abs(curvature) - 1.0 / Radius);
			Bend.side = way;
		}
	}
	if (std::abs(curvature) < 1.0 / kStraighter)
	{
		if (Bend.target.track == nullptr)
			return std::nullopt;
		return 0.0;
	}
	if (Bend.target.track == nullptr)
		return typical_curvature(curvature, Radius);
	return curvature;
}

// the side the diverging track of a free switch bent with the Curvature goes to: as the drag gives it, Side, where it isn't bent, else
// to the inside of the curve the switch Turns along where it's One_sided, to the outside where not
int bend_side(double const Curvature, int const Side, bool const One_sided, int const Turn)
{
	if (Curvature == 0.0)
		return Side;
	return One_sided ? Turn : -Turn;
}

// the curve a free switch is bent into, from Start along Heading at the Grade with the Curvature, Reach along it, a little past its end,
// into the frame Into; Radius: of the curve
template <typename Frame>
void bend_frame(Frame &Into, glm::dvec3 const &Start, glm::dvec2 const &Heading, double const Grade, double const Curvature, double const Reach, double const Radius)
{
	using sample_type = typename Frame::value_type;
	glm::dvec2 const normal{-Heading.y, Heading.x};
	int const steps{std::max(8, static_cast<int>(std::ceil(Reach / 0.5)))};
	for (int i = 0; i <= steps; ++i)
	{
		auto const station{Reach * i / steps};
		auto const angle{Curvature * station};
		auto const planar{Curvature == 0.0 ? plan_of(Start) + Heading * station : plan_of(Start) + (Heading * std::sin(angle) + normal * (1.0 - std::cos(angle))) / Curvature};
		sample_type sample;
		sample.station = station;
		sample.position = {planar.x, Start.y + Grade * station, planar.y};
		sample.tangent = glm::normalize(Heading * std::cos(angle) + normal * std::sin(angle));
		sample.radius = static_cast<float>(Radius);
		Into.push_back(sample);
	}
}

} // namespace

switch_curve::switch_report editor_mode::curved_switch_report(std::vector<segment_data> const &Paths, std::vector<curve_sample> const &Frame, double const Station, int const Direction) const
{
	auto const &tool{m_switch};
	if (Paths.size() < 2 || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return {};
	auto const &shape{tool.templates[tool.armed]};
	auto const frog{frog_zone(shape)};
	return switch_curve::check(plan_points(Paths[0]), plan_points(Paths[1]), frame_points(Frame), Station, Station + Direction * shape.length, shape.radius, shape.ratio, frog.has_value() ? shape.length - *frog : 0.0);
}

segment_data editor_mode::frame_path(segment_data const &Local, std::vector<curve_sample> const &Frame, double const Station, int const Direction, int const Side, std::optional<double> const &Bent)
{
	auto const map = [&](glm::dvec3 const &Point, glm::dvec2 &Tangent, glm::dvec2 const &Localtangent) {
		auto const bent{Bent.has_value() ? std::min(Point.z, *Bent) : Point.z};
		auto const sample{sample_at(Frame, Station + Direction * bent)};
		auto const along{sample.tangent * static_cast<double>(Direction)};
		glm::dvec2 const across{glm::dvec2{-along.y, along.x} * static_cast<double>(Side)};
		Tangent = glm::normalize(along * Localtangent.y + across * Localtangent.x);
		auto const planar{plan_of(sample.position) + along * (Point.z - bent) + across * Point.x};
		auto const level{Point.z > bent ? sample_at(Frame, Station + Direction * Point.z) : sample};
		auto const cant{glm::radians(static_cast<double>(level.roll) * Direction)};
		auto const cone{-Point.x * Side * std::tan(cant)};
		return glm::dvec3{planar.x, level.position.y + Point.y + cone, planar.y};
	};
	auto const hermite = [](glm::dvec3 const &Start, glm::dvec2 const &Startdirection, glm::dvec3 const &End, glm::dvec2 const &Enddirection) {
		auto const handle{plan_distance(Start, End) / 3.0};
		segment_data path;
		path.points[segment_data::point::start] = Start;
		path.points[segment_data::point::end] = End;
		path.points[segment_data::point::control1] = {Startdirection.x * handle, (End.y - Start.y) / 3.0, Startdirection.y * handle};
		path.points[segment_data::point::control2] = {-Enddirection.x * handle, -(End.y - Start.y) / 3.0, -Enddirection.y * handle};
		return path;
	};
	auto const &start{Local.points[segment_data::point::start]};
	auto const &end{Local.points[segment_data::point::end]};
	auto const &control1{Local.points[segment_data::point::control1]};
	auto const &control2{Local.points[segment_data::point::control2]};
	glm::dvec2 const chord{end.x - start.x, end.z - start.z};
	auto const localstart{glm::normalize(control1 != glm::dvec3{} ? plan_of(control1) : chord)};
	auto const localend{glm::normalize(control2 != glm::dvec3{} ? -plan_of(control2) : chord)};
	glm::dvec2 startdirection;
	glm::dvec2 enddirection;
	auto const p0{map(start, startdirection, localstart)};
	auto const p3{map(end, enddirection, localend)};
	auto path{hermite(p0, startdirection, p3, enddirection)};
	{
		auto const cone = [&](double const T) {
			auto const point{bezier{path}.point(T)};
			auto const sample{sample_at(Frame, nearest_station(Frame, point))};
			auto const lateral{glm::dot(plan_of(point) - plan_of(sample.position), glm::dvec2{-sample.tangent.y, sample.tangent.x})};
			return sample.position.y - lateral * std::tan(glm::radians(static_cast<double>(sample.roll)));
		};
		auto const h1{cone(1.0 / 3.0)};
		auto const h2{cone(2.0 / 3.0)};
		auto const y1{3.0 * h1 - 1.5 * h2 - 5.0 / 6.0 * p0.y + p3.y / 3.0};
		auto const y2{3.0 * h2 - 1.5 * h1 + p0.y / 3.0 - 5.0 / 6.0 * p3.y};
		path.points[segment_data::point::control1].y = y1 - p0.y;
		path.points[segment_data::point::control2].y = y2 - p3.y;
	}
	auto const rollstart{sample_at(Frame, Station + Direction * start.z).roll * Direction};
	auto const rollend{sample_at(Frame, Station + Direction * end.z).roll * Direction};
	path.rolls = {static_cast<float>(rollstart), static_cast<float>(rollend)};
	path.radius = Local.radius;
	{
		bezier const curve{path};
		auto const begin{plan_of(curve.first(0.0))};
		auto const finish{plan_of(curve.first(1.0))};
		auto const turn{glm::length(begin) > 1e-9 && glm::length(finish) > 1e-9 ? geometry::signed_angle(begin, finish) : 0.0};
		if (Local.radius != 0.f && std::abs(turn) > 1e-6)
			path.radius = static_cast<float>(std::copysign(curve.plan_length() / std::abs(turn), static_cast<double>(Local.radius)));
	}
	return path;
}

std::vector<segment_data> editor_mode::curved_switch_paths(editor_track::switch_template const &Shape, std::vector<curve_sample> const &Frame, double const Station, int const Direction, int const Side)
{
	// a switch with a straight frog bent up to the frog, beyond it going on along the curve as it heads there; the heights and the cant
	// those of the line where the point is along it
	auto const frog{frog_zone(Shape)};
	std::vector<segment_data> locals{Shape.local.begin(), Shape.local.end()};
	if (Shape.twofold())
	{
		locals.insert(locals.end(), Shape.second.begin(), Shape.second.end());
		locals.push_back(Shape.link);
	}
	std::vector<segment_data> result;
	for (std::size_t i = 0; i < locals.size(); ++i)
	{
		auto const &local{locals[i]};
		auto path{frame_path(local, Frame, Station, Direction, Side, frog)};
		// the tracks of a switch bent up to the frog in pieces: the bent one and the straight one over the frog
		if (auto const pieces{frog.has_value() && i < 2 ? split_along(local, *frog) : local.pieces}; pieces.size() >= 2 || false == local.pieces.empty())
			for (auto const &piece : pieces)
				path.pieces.push_back(frame_path(piece, Frame, Station, Direction, Side, frog));
		// the main tracks follow the curve they are laid into
		if (i % 2 == 0)
			path.radius = sample_at(Frame, Station + Direction * 0.5 * (local.points[segment_data::point::start].z + local.points[segment_data::point::end].z)).radius;
		result.push_back(path);
	}
	return result;
}

void editor_mode::insert_curved_switch(int const Direction, int const Side)
{
	auto &tool{m_switch};
	auto const &shape{tool.templates[tool.armed]};
	auto const total{tool.frame.back().station};
	if (shape.length > total - 0.02)
	{
		tool.status = refused(format(STR_C("The curve is %.1f m long, too short for the %.1f m switch"), total, shape.length));
		return;
	}
	auto const station{std::clamp(tool.along, Direction > 0 ? 0.0 : shape.length, Direction > 0 ? total - shape.length : total)};
	auto const from{Direction > 0 ? station : station - shape.length};
	auto const to{from + shape.length};
	editor_track::edit edit;
	auto const frogged{frog_zone(shape).has_value()};
	std::vector<TTrack *> candidates{tool.frame_tracks};
	// the track the cut falls on split there, unless the cut is at a joint of it
	for (auto const cut : {from, to})
		split_at(candidates, sample_at(tool.frame, cut).position, edit);
	retired_between(candidates, from, to, [&frame = tool.frame](glm::dvec3 const &Point) { return nearest_station(frame, Point); }, edit);
	auto const paths{curved_switch_paths(shape, tool.frame, station, Direction, Side)};
	// a joint a few centimetres off an end of the switch, not cut there: the end of the track left beside it goes right onto the end of
	// the switch, so the two join and no gap is left; the end of one bent up to the frog is fitted to it further on
	std::vector<TTrack *> snapped;
	for (auto const index : {segment_data::point::start, segment_data::point::end})
	{
		if (index == segment_data::point::end && frogged)
			continue;
		snap_ends_to(candidates, paths.front().points[index], edit, snapped);
	}
	editor_track::commit(snapped);
	auto const report{curved_switch_report(paths, tool.frame, station, Direction)};
	// a switch bent up to the frog leaves the curve straight, a little off it at its end: the track going on from there along the line
	// starts where the switch ends, heading as it does, and meets the line again at its other end
	std::string fitted;
	if (frogged)
	{
		auto const endsample{sample_at(tool.frame, station + Direction * shape.length)};
		fitted = track_fitted_on(paths.front(), endsample.position, endsample.tangent * static_cast<double>(Direction), edit);
	}
	auto const made{create_switch_set(shape, paths, *tool.frame_tracks.front(), switch_name_for_new())};
	edit.created.insert(edit.created.end(), made.begin(), made.end());
	if (made.size() == 1)
		tool.status = format(STR_C("Switch %s inserted into the curve, Ctrl+Z takes it back"), made.front()->name().c_str());
	// what the norms say of it stays in the status, after what the drive placed with it tells
	std::string norms;
	for (auto const &warning : report.warnings)
		norms += "\n" + warning;
	norms += fitted;
	tool.status += norms;
	auto const placed{edit.created};
	push_track_snapshot(std::move(edit.states), std::move(edit.created), std::move(edit.removed));
	straight_refresh();
	switch_placed(placed);
	if (false == norms.empty() && tool.status.find(norms) == std::string::npos)
		tool.status += norms;
}

std::vector<std::string> editor_mode::free_switch_exits(std::vector<segment_data> const &Paths) const
{
	auto const &tool{m_switch};
	std::vector<std::string> result;
	if (tool.free_end.track == nullptr || tool.free_end.track->eType != tt_Switch)
		return result;
	// the main and the diverging track of this one, each going on from that switch
	for (std::size_t i = 0; i < std::min<std::size_t>(2, Paths.size()); ++i)
		for (auto const &warning : switch_exit_warnings(tool.free_end.track, tool.free_end.position, nullptr, {}, {Paths[i]}))
			if (std::ranges::find(result, warning) == result.end())
				result.push_back(warning);
	return result;
}

bool editor_mode::free_switch_bend(free_bend &Bend) const
{
	if (auto key{switch_memo_key()}; false == m_bend_memo.fresh(key))
	{
		free_bend bend;
		auto const bent{free_switch_bend_now(bend)};
		m_bend_memo.keep(std::move(key), {bent, bend});
	}
	Bend = m_bend_memo.result.second;
	return m_bend_memo.result.first;
}

bool editor_mode::free_switch_bend_now(free_bend &Bend) const
{
	auto const &tool{m_switch};
	Bend = {};
	if (false == tool.free || tool.free_end.track == nullptr || false == tool.free_heading.has_value() || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return false;
	auto const &shape{tool.templates[tool.armed]};
	if (shape.twofold() || shape.double_slip || shape.radius < 1.0)
		return false;
	glm::dvec3 start;
	glm::dvec2 heading;
	int side;
	double grade;
	free_switch_pose(start, heading, side, grade);
	// a free end the cursor is at, other than the one the switch goes on from
	std::vector<TTrack const *> const exclude{tool.free_end.track};
	auto const &io{ImGui::GetIO()};
	if (false == m_cursor_override.has_value())
		Bend.target = snap_free_end(tool.mouse, {io.MousePos.x, io.MousePos.y}, nullptr, kRailCategory, exclude);
	if (Bend.target.track == nullptr)
		Bend.target = editor_track::find_free_end(nullptr, kRailCategory, tool.mouse, kLaySnapRadius, exclude);
	auto const toward{plan_of((Bend.target.track != nullptr ? Bend.target.position : tool.mouse) - start)};
	auto const along{glm::dot(toward, heading)};
	// near the switch the drag gives the side of its diverging track, as of a switch laid straight; past its end it bends it
	if (along < shape.length)
		return false;
	Bend.diverging = Bend.target.track != nullptr && io.KeyShift;
	auto const curvature{bend_curvature(Bend, start, heading, toward, along, shape.radius)};
	if (false == curvature.has_value())
		return false;
	Bend.radius = *curvature != 0.0 ? 1.0 / std::abs(*curvature) : 0.0;
	Bend.turn = *curvature >= 0.0 ? 1 : -1;
	if (false == Bend.diverging)
		Bend.side = bend_side(*curvature, side, tool.one_sided, Bend.turn);
	// the curve it's bent into, a little past its end
	bend_frame(Bend.frame, start, heading, grade, *curvature, shape.length + 2.0, Bend.radius);
	Bend.paths = curved_switch_paths(shape, Bend.frame, 0.0, 1, Bend.side);
	if (Bend.paths.size() < 2)
		return false;
	Bend.report = curved_switch_report(Bend.paths, Bend.frame, 0.0, 1);
	if (Bend.target.track != nullptr)
		free_bend_link(Bend);
	return true;
}

void editor_mode::free_bend_link(free_bend &Bend) const
{
	auto const &tool{m_switch};
	// the track into it goes on with the curve of the one it leaves, then straight, or with reverse curves
	auto const &branch{Bend.paths[Bend.diverging ? 1 : 0]};
	auto const radius{editor_track::branch_radius(branch)};
	auto speed{design_speed(tool.free_end.track, Bend.target.track)};
	if (Bend.diverging)
		speed = std::min(editor_track::diverging_speed(radius), speed);
	Bend.link = editor_track::link_branch(branch_start(branch), radius > 0.5 ? radius : kStraighter, branch_target(Bend.target), glm::radians(m_joints.kink_angle() * 0.5), editor_track::reverse_insert(speed), stretchable(Bend.target));
	Bend.problem = link_problem(Bend.target, Bend.link);
}

bool editor_mode::plan_curved_slip(editor_track::switch_template const &Shape, int const Direction, int const Side, curved_slip &Plan) const
{
	auto const &tool{m_switch};
	Plan = curved_slip{};
	// a crossing has no slip curves to bend, it goes only where two straights cross
	if (Shape.type == "Skrz")
	{
		Plan.problem = STR_C("A crossing goes only where two straights cross");
		return false;
	}
	if (tool.frame.size() < 2 || tool.frame_tracks.empty())
		return false;
	auto const *gauged{tool.frame_tracks.front()};
	auto const angle{std::atan(1.0 / (Shape.ratio > 0.0 ? Shape.ratio : 9.0))};
	auto const [blade_lead, blade_entry]{switch_design::secant_blade(Shape.rail, Shape.radius, Shape.ratio)};
	auto curve{editor_track::slip_of(Shape.radius, Shape.middle, angle, 0.5 * std::abs(gauged->fTrackWidth) + gauged->rail_dimensions().head + 0.002, blade_lead, blade_entry)};
	curve.tip = switch_design::cut_tip(Shape.rail, curve.radius);
	auto const half{curve.tangent()};
	auto const total{tool.frame.back().station};
	if (2.0 * half > total - 0.02)
	{
		Plan.problem = format(STR_C("The curve is %.1f m long, too short for the %.1f m switch"), total, 2.0 * half);
		return false;
	}
	// the crossing where it was pressed, the ends of the slip drawn to the ends of the cant ramps as the ends of a switch are, the
	// whole of it on the curve
	auto const points{frame_points(tool.frame)};
	auto const start{switch_curve::snapped_start(switch_curve::ramp_ends(points), tool.pressed - Direction * half, 2.0 * half, Direction, 3.0, &Plan.snapped)};
	auto const middle{std::clamp(start + Direction * half, half, total - half)};
	if (std::abs(middle - (start + Direction * half)) > 1e-6)
		Plan.snapped.clear();
	// bent with a curve as sharp as its slip curves, the outer one would turn the other way
	double sharpest{0.0};
	for (int k = 0; middle - half + k * 0.5 <= middle + half + 1e-9; ++k)
		sharpest = std::max(sharpest, std::abs(switch_curve::curvature_at(points, middle - half + k * 0.5)));
	if (sharpest * curve.radius > 1.0 - 1e-6)
	{
		Plan.problem = format(STR_C("The curve of R %.0f m is as sharp as the slip curves of R %.0f m or sharper, the slip isn't bent into it"), 1.0 / sharpest, curve.radius);
		return false;
	}
	auto const sample{sample_at(tool.frame, middle)};
	Plan.middle = middle;
	Plan.half = half;
	// the straights of both tracks out to the joints of the template, as far as the curve goes on
	Plan.reach = std::max(half, std::min(Shape.a, std::min(middle, total - middle) - 0.02));
	Plan.direction = Direction;
	Plan.angle = angle;
	Plan.curve = curve;
	Plan.crossing = plan_of(sample.position);
	Plan.first = sample.tangent * static_cast<double>(Direction);
	Plan.second = turned(Plan.first, static_cast<double>(Side) * angle);
	auto const at = [](glm::dvec2 const &Planar) { return glm::dvec3{Planar.x, 0.0, Planar.y}; };
	Plan.points = {at(Plan.crossing - Plan.first * half), at(Plan.crossing + Plan.first * half), at(Plan.crossing - Plan.second * half), at(Plan.crossing + Plan.second * half)};
	return true;
}

segment_data editor_mode::bent_slip_path(curved_slip const &Plan, segment_data const &Path) const
{
	auto const &frame{m_switch.frame};
	// in the plan of a template: x across the tangent, to the left of the way the slip goes along the curve, z along it from the crossing
	glm::dvec2 const leftward{-Plan.first.y, Plan.first.x};
	auto const local = [&](segment_data const &Piece) {
		auto result{Piece};
		result.pieces.clear();
		for (auto const index : {segment_data::point::start, segment_data::point::end})
		{
			auto const offset{plan_of(Piece.points[index]) - Plan.crossing};
			result.points[index] = {glm::dot(offset, leftward), Piece.points[index].y, glm::dot(offset, Plan.first)};
		}
		for (auto const index : {segment_data::point::control1, segment_data::point::control2})
			if (auto const &handle{Piece.points[index]}; handle != glm::dvec3{})
				result.points[index] = {glm::dot(plan_of(handle), leftward), handle.y, glm::dot(plan_of(handle), Plan.first)};
		return result;
	};
	auto const bent = [&](segment_data const &Piece) {
		auto const flat{local(Piece)};
		auto result{frame_path(flat, frame, Plan.middle, Plan.direction, 1)};
		// the straights of the slip follow the curve they're bent with
		if (Piece.radius == 0.f)
			result.radius = sample_at(frame, Plan.middle + Plan.direction * 0.5 * (flat.points[segment_data::point::start].z + flat.points[segment_data::point::end].z)).radius;
		return result;
	};
	auto result{bent(Path)};
	for (auto const &piece : Path.pieces)
		result.pieces.push_back(bent(piece));
	return result;
}

std::vector<segment_data> editor_mode::curved_slip_lines(curved_slip const &Plan, std::string_view const Type) const
{
	auto const &[a, b, c, d]{Plan.points};
	auto const side{cross(Plan.first, Plan.second) > 0.0 ? 1.0 : -1.0};
	auto const whole{Plan.curve.length()};
	std::vector lines{bent_slip_path(Plan, straight_path(a, b)), bent_slip_path(Plan, slip_arc(Plan.curve, a, Plan.first, side, d, 0.0, whole)), bent_slip_path(Plan, straight_path(c, d))};
	if (Type != "Rkp")
		lines.push_back(bent_slip_path(Plan, slip_arc(Plan.curve, c, Plan.second, -side, b, 0.0, whole)));
	return lines;
}

std::vector<segment_data> editor_mode::curved_slip_reaches(curved_slip const &Plan) const
{
	if (Plan.reach < Plan.half + 0.01)
		return {};
	auto const at = [](glm::dvec2 const &Planar) { return glm::dvec3{Planar.x, 0.0, Planar.y}; };
	auto const &[a, b, c, d]{Plan.points};
	return {bent_slip_path(Plan, straight_path(at(Plan.crossing - Plan.first * Plan.reach), a)), bent_slip_path(Plan, straight_path(b, at(Plan.crossing + Plan.first * Plan.reach))),
	        bent_slip_path(Plan, straight_path(at(Plan.crossing - Plan.second * Plan.reach), c)), bent_slip_path(Plan, straight_path(d, at(Plan.crossing + Plan.second * Plan.reach)))};
}

switch_curve::slip_report editor_mode::curved_slip_report(curved_slip const &Plan, std::string const &Type) const
{
	auto const lines{curved_slip_lines(Plan, Type)};
	std::vector curves(1, plan_points(lines[1]));
	if (lines.size() > 3)
		curves.push_back(plan_points(lines[3]));
	return switch_curve::check_slip({plan_points(lines[0]), plan_points(lines[2])}, curves, frame_points(m_switch.frame), Plan.middle - Plan.direction * Plan.half, Plan.middle + Plan.direction * Plan.half);
}

std::vector<std::string> editor_mode::slip_report_lines(switch_curve::slip_report const &Report)
{
	// a radius counted positive the way the line turns, the other way with a minus
	auto const radius = [](double const Curvature) { return std::abs(Curvature) < 2e-5 ? std::string{STR_C("straight")} : format("%.0f m", 1.0 / Curvature); };
	std::vector lines{format(STR_C("tracks through the slip R %s and %s"), radius(Report.first).c_str(), radius(Report.second).c_str())};
	std::string curves;
	for (auto const curvature : Report.curves)
		curves += (curves.empty() ? "" : ", ") + radius(curvature);
	if (false == curves.empty())
		lines.push_back(format(STR_C("slip curves R %s at their sharpest"), curves.c_str()));
	lines.push_back(format(STR_C("cant %.0f mm at the start, %.0f mm at the end"), Report.cant_start, Report.cant_end));
	return lines;
}

void editor_mode::insert_curved_slip(int const Direction, int const Side)
{
	auto &tool{m_switch};
	auto const &shape{tool.templates[tool.armed]};
	curved_slip plan;
	if (false == plan_curved_slip(shape, Direction, Side, plan))
	{
		tool.status = refused(plan.problem);
		flash_switch_problem(plan.problem, {}, {{sample_at(tool.frame, tool.along).position, ""}});
		return;
	}
	auto const report{curved_slip_report(plan, shape.type)};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	std::vector<TTrack *> candidates{tool.frame_tracks};
	auto &style{*tool.frame_tracks.front()};
	// the curve taken out up to the joints of the template, the straights of both tracks laid out there
	cut_curve(candidates, tool.frame, sample_at(tool.frame, plan.middle - plan.reach).position, sample_at(tool.frame, plan.middle + plan.reach).position, states, created, removed);
	auto const parts{build_double_slip(plan.points[0], plan.points[1], plan.points[2], plan.points[3], plan.crossing, 0.0, plan.first, plan.second, plan.angle, plan.curve, style, shape.type,
	                                   [this, &plan](segment_data const &Path) { return bent_slip_path(plan, Path); })};
	std::vector<TTrack *> made{parts};
	for (auto const &path : curved_slip_reaches(plan))
		if (auto *straight{editor_track::create_path(style, path)})
			made.push_back(straight);
	editor_track::commit(made);
	created.insert(created.end(), made.begin(), made.end());
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(parts);
	// after what the drives placed with it tell: its radii and what the norms say of it
	std::string text{STR_C("Slip bent into the curve, the ends of its crossing track free: lay on from them")};
	for (auto const &line : slip_report_lines(report))
		text += "\n" + line;
	for (auto const &warning : report.warnings)
		text += "\n" + warning;
	for (auto const &note : report.notes)
		text += "\n" + note;
	tool.status += (tool.status.empty() ? "" : "\n") + text;
	ui()->set_status(tool.status, false);
}

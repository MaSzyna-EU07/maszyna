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
#include "application/editormoderailtools.h"
#include "editor/editorSettings.hpp"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "application/editorgestures.h"
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"

#include "utilities/Globals.h"
#include "utilities/Timer.h"
#include "rendering/renderer.h"
#include "world/Track.h"
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
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <format>
#include <functional>
#include <iterator>
#include <set>

#include "application/editormodetrackshared.h"

using namespace track_mode;

namespace
{

// the tab whose card shows: that of the paths with the window closed, that of the straights for the route
editor_track_tab card_tab(bool const Open, editor_track_tab const Tab)
{
	if (false == Open)
		return editor_track_tab::path;
	if (Tab == editor_track_tab::route)
		return editor_track_tab::straights;
	return Tab;
}

// a point clicked along a curve laid, by how far the curve strays from it: green within 10 cm, amber within 30 cm, red beyond, the
// colour of the grips where it's not known
ImU32 lay_point_colour(double const Off)
{
	if (Off < 0.0)
		return overlay_color::grip;
	if (Off <= 0.1)
		return IM_COL32(90, 220, 110, 255);
	if (Off <= 0.3)
		return IM_COL32(255, 200, 60, 255);
	return IM_COL32(255, 90, 70, 255);
}

// the own shape of the Track, a switch on a straight, for a template of its own: its paths in the frame of the switch at the Origin,
// along the Axis and across it by the Normal, mirrored to the other Side
editor_track::switch_template mirrored_template(TTrack *Track, glm::dvec3 const &Origin, glm::dvec2 const &Axis, glm::dvec2 const &Normal, int const Side)
{
	editor_track::switch_template own;
	own.source = Track;
	for (int i = 0; i < 2; ++i)
	{
		auto &local{own.local[i]};
		local = Track->m_paths[i];
		auto const tolocal = [&Normal, &Axis, Side](glm::dvec3 const &Vector) { return glm::dvec3{glm::dot(plan_of(Vector), Normal) * Side, Vector.y, glm::dot(plan_of(Vector), Axis)}; };
		local.transform([&Origin, &tolocal](glm::dvec3 const &Point) { return tolocal(Point - Origin); }, tolocal);
		if (Side >= 0)
			continue;
		local.rolls = {-local.rolls[0], -local.rolls[1]};
		for (auto &piece : local.pieces)
			piece.rolls = {-piece.rolls[0], -piece.rolls[1]};
	}
	return own;
}

// all three coordinates of the Point are numbers
bool all_finite(glm::dvec3 const &Point)
{
	return std::isfinite(Point.x) && std::isfinite(Point.y) && std::isfinite(Point.z);
}

// the events on all six lists of the Track
std::size_t events_count(TTrack &Track)
{
	std::size_t total{0};
	for (int list = 0; list < 6; ++list)
		total += editor_track::events(Track, list).size();
	return total;
}

// the Hints of the keys with the gizmo hidden: it has no handles to drag, Q, W and E bring it out; only the ends of the track are there to
// drag, the control vectors come out with it
template <typename Hint> void gizmo_hidden(std::vector<Hint> &Hints)
{
	auto const named = [](char const *Key) { return [Key](Hint const &Each) { return std::string_view{Each.key} == Key; }; };
	auto const handles{std::ranges::find_if(Hints, named("Gizmo"))};
	if (auto const keys{std::ranges::find_if(Hints, named("Q W E"))}; keys != Hints.end())
	{
		keys->action = "bring out the gizmo: move, turn, lengthen, bend by the control vectors";
		if (handles != Hints.end())
			Hints.erase(handles);
	}
	else if (handles != Hints.end())
		*handles = {"Q W E", "bring out the gizmo: move, turn, lengthen, bend by the control vectors"};
	if (auto const points{std::ranges::find_if(Hints, named("Drag a point"))}; points != Hints.end())
		*points = {"Drag an end", "a joint: the neighbours follow; a free end: a new track out of it, with Ctrl it moves"};
}

// the azimuth from From to To in degrees, 0 to 360
double azimuth_of(glm::dvec3 const &From, glm::dvec3 const &To)
{
	auto const degrees{glm::degrees(std::atan2(To.x - From.x, To.z - From.z))};
	return degrees < 0.0 ? degrees + 360.0 : degrees;
}

// of a list of warnings only the first one by the cursor, the panel lists them all
std::string first_line_of(std::string const &Lines)
{
	auto const end{Lines.find('\n')};
	if (end == std::string::npos)
		return Lines;
	auto const more{static_cast<std::size_t>(std::count(Lines.begin() + end, Lines.end(), '\n'))};
	return Lines.substr(0, end) + "\n" + format(STR_P("and %zu more, in the panel", "and %zu more, in the panel", static_cast<long long>(more)), more);
}

// the plan length of the Pieces together and the smallest of their radii, 0 where all of them are straight
std::pair<double, double> length_and_radius(std::vector<segment_data> const &Pieces)
{
	double length{0.0};
	double radius{0.0};
	for (auto const &piece : Pieces)
	{
		length += bezier{piece}.plan_length();
		if (piece.radius != 0.0f && (radius == 0.0 || std::abs(piece.radius) < radius))
			radius = std::abs(piece.radius);
	}
	return {length, radius};
}

// the axes of the two rails of a track apart
constexpr double kRailAxes{1.5};

// a rail of the Curve on the Side of its axis, -1 or 1, at the height of the curve, as lines of 24 steps in the Colour
void draw_rail(screen_projection const &Projection, ImDrawList *Drawlist, bezier const &Curve, double const Side, ImU32 const Colour, float const Width)
{
	auto const rail_at = [&Curve, Side](double const T) {
		auto const tangent{plan_of(Curve.first(T))};
		if (glm::length(tangent) < 1e-9)
			return Curve.point(T);
		auto const across{glm::normalize(glm::dvec2{-tangent.y, tangent.x}) * (Side * 0.5 * kRailAxes)};
		return Curve.point(T) + glm::dvec3{across.x, 0.0, across.y};
	};
	auto previous{rail_at(0.0)};
	for (int i = 1; i <= 24; ++i)
	{
		auto const next{rail_at(i / 24.0)};
		Projection.line(Drawlist, previous, next, Colour, Width);
		previous = next;
	}
}

// the track being laid as its two rails: their axes 1.5 m apart, the gauge and the head of a rail, at the height of the Path
void draw_overlay_rails(screen_projection const &Projection, ImDrawList *Drawlist, segment_data const &Path, ImU32 const Color)
{
	for (auto const &[colour, width] : {std::pair{IM_COL32(0, 0, 0, 170), 4.0f}, std::pair{Color, 2.0f}})
		for (auto const &piece : Path.pieces.empty() ? std::vector<segment_data>{Path} : Path.pieces)
		{
			bezier const curve{piece};
			for (auto const side : {-1.0, 1.0})
				draw_rail(Projection, Drawlist, curve, side, colour, width);
		}
}

// a circle on the ground as wide as the rails are apart, around the Point; a few pixels across at least, seen from far
void draw_track_width(screen_projection const &Projection, ImDrawList *Drawlist, glm::dvec3 const &Point, ImU32 const Color)
{
	ImVec2 centre;
	if (false == Projection.project(Point, centre))
		return;
	std::array<ImVec2, 24> outline;
	auto seen{true};
	for (std::size_t i = 0; i < outline.size(); ++i)
	{
		auto const angle{2.0 * glm::pi<double>() * static_cast<double>(i) / outline.size()};
		seen = Projection.project(Point + glm::dvec3{std::cos(angle), 0.0, std::sin(angle)} * (0.5 * kRailAxes), outline[i]);
		if (false == seen)
			break;
	}
	auto const dot{false == seen || std::hypot(outline[0].x - centre.x, outline[0].y - centre.y) < 4.0f};
	for (auto const &[colour, width] : {std::pair{IM_COL32(0, 0, 0, 170), 4.0f}, std::pair{Color, 2.0f}})
	{
		if (dot)
			Drawlist->AddCircle(centre, 4.0f, colour, 16, width);
		else
			Drawlist->AddPolyline(outline.data(), static_cast<int>(outline.size()), colour, ImDrawFlags_Closed, width);
	}
}

// the Lines told of a switch bent, each in its colour, in a box beside the Point
void draw_readout_box(screen_projection const &Projection, ImDrawList *Drawlist, glm::dvec3 const &Point, std::vector<std::pair<std::string, ImU32>> const &Lines)
{
	ImVec2 screen;
	if (false == Projection.project(Point, screen))
		return;
	auto const wrap{460.0f};
	auto height{0.0f};
	auto width{0.0f};
	for (auto const &[text, colour] : Lines)
	{
		auto const size{ImGui::CalcTextSize(text.c_str(), nullptr, false, wrap)};
		height += size.y + 2.0f;
		width = std::max(width, size.x);
	}
	ImVec2 corner{screen.x + 24.0f, screen.y + 24.0f};
	Drawlist->AddRectFilled({corner.x - 6.0f, corner.y - 4.0f}, {corner.x + width + 6.0f, corner.y + height + 4.0f}, IM_COL32(10, 20, 30, 215), 4.0f);
	for (auto const &[text, colour] : Lines)
	{
		Drawlist->AddText(ImGui::GetFont(), ImGui::GetFontSize(), corner, colour, text.c_str(), nullptr, wrap);
		corner.y += ImGui::CalcTextSize(text.c_str(), nullptr, false, wrap).y + 2.0f;
	}
}

// the lines told of a switch Bent at a free end: the radii of its tracks, the track into the free end the cursor is at or why it isn't
// laid into it, the warnings; the free end ringed
template <typename Bend> std::vector<std::pair<std::string, ImU32>> bend_readout(screen_projection const &Projection, ImDrawList *Drawlist, Bend const &Bent)
{
	std::vector<std::pair<std::string, ImU32>> lines;
	if (Bent.radius > 0.0)
		lines.emplace_back(format(Bent.side == Bent.turn ? STR_C("one-sided: main R %.0f m, diverging R %.0f m") : STR_C("two-sided: main R %.0f m, diverging R %.0f m"), Bent.radius, editor_track::branch_radius(Bent.paths[1])),
		                   IM_COL32(255, 170, 40, 255));
	if (Bent.target.track != nullptr)
	{
		auto const &branch{Bent.paths[Bent.diverging ? 1 : 0]};
		if (Bent.problem.empty())
			lines.emplace_back(link_summary(Bent.target, Bent.link, editor_track::branch_radius(branch)), IM_COL32(255, 60, 255, 255));
		else
			lines.emplace_back(Bent.problem, overlay_color::invalid);
		ImVec2 screen;
		if (Projection.project(Bent.target.position, screen))
			Drawlist->AddCircle(screen, 14.0f, Bent.problem.empty() ? IM_COL32(255, 60, 255, 255) : overlay_color::invalid, 20, 3.0f);
	}
	for (auto const &warning : Bent.report.warnings)
		lines.emplace_back(warning, overlay_color::invalid);
	return lines;
}

// the width of a hint of the keys Shown at the bottom: its picture of the Words, Picture high, its key in a chip unless the picture is all
// keys, its action
template <typename Hint> float hint_width(Hint const &Shown, std::vector<gesture_word> const &Words, float const Picture, float const Pad)
{
	auto const chip_width{Shown.raw != nullptr && gesture_keys_only(Shown.raw) ? 0.0f : ImGui::CalcTextSize(Shown.key).x + 2.0f * Pad + 5.0f};
	auto const picture_width{Words.empty() ? 0.0f : gesture_width(Picture, Words) + 5.0f};
	return picture_width + chip_width + ImGui::CalcTextSize(Shown.action.c_str()).x;
}

// a radius counted positive the way the line turns, the other way with a minus
std::string radius_text(double const Curvature)
{
	return std::abs(Curvature) < 2e-5 ? std::string{STR_C("straight")} : format("%.0f m", 1.0 / Curvature);
}

} // namespace


TTrack *editor_mode::selected_track() const
{
	return dynamic_cast<TTrack *>(m_node);
}

bool editor_mode::track_editing() const
{
	return m_track_gizmo_using || m_dragging;
}

std::size_t editor_mode::history_revision() const
{
	auto const top{m_history.empty() ? std::uint64_t{0} : m_history.back().serial};
	return static_cast<std::size_t>(top * 1000003u) ^ (m_history.size() << 20) ^ g_redo.size();
}

glm::dvec3 editor_mode::camera_position() const
{
	return Camera.Pos;
}

track_hover const &editor_mode::hovered_track() const
{
	return m_hover;
}

std::vector<TTrack *> const &editor_mode::track_set() const
{
	return m_track_set;
}

editor_track_tab editor_mode::current_track_tab() const
{
	return m_track_tab;
}

void editor_mode::set_current_node(scene::basic_node *Node)
{
	m_node = Node;
	ui()->set_node(m_node);
}

alignment::limits const &editor_mode::route_norms() const
{
	return m_route.design.norms;
}

void editor_mode::clear_track_point()
{
	m_track_point = {};
}

editor_nmt &editor_mode::terrain_model()
{
	return m_nmt;
}

double editor_mode::terrain_model_offset() const
{
	return m_nmt_state.offset;
}

bool editor_mode::terrain_model_fitted() const
{
	return m_nmt_state.fitted;
}

void editor_mode::select_track(scene::basic_node *Node)
{
	auto *track = dynamic_cast<TTrack *>(Node);
	if (track == nullptr)
	{
		m_track_point = {};
		return;
	}
	if (track != m_node)
	{
		m_track_point = {};
		m_track_mode_notice.clear();
	}
	// a track out of the set ends it, the set is the selection only together with its tracks
	if (false == in_track_set(track))
		m_track_set.clear();
	m_node = track;
	ui()->set_node(m_node);
	if (ui()->mode() != nodebank_panel::TRACK)
		return;
	m_track_window_open = true;
	m_track_mode_notice.clear();
	if (track_plan_tab() && plan_tab_for(track) != m_track_tab)
	{
		show_track_tab(plan_tab_for(track));
		return;
	}
	// the mode stays as it is, the path is taken up by it if it can be
	switch (m_track_tab)
	{
	case track_tab::straights:
		if (track->eType == tt_Switch)
			m_track_mode_notice = STR_C("Switches aren't a part of a line: T edits the switch, Esc goes back to Select");
		break;
	case track_tab::route:
		if (std::find(m_route.chain.tracks.begin(), m_route.chain.tracks.end(), track) != m_route.chain.tracks.end())
			break;
		if (track->eType != tt_Normal || false == editor_track::is_supported(*track))
			m_track_mode_notice = STR_C("Switches aren't a part of a line: T edits the switch, Esc goes back to Select");
		else if (false == route_load(*track))
			m_track_mode_notice = m_route.error.empty() ? std::string{STR_C("The curve can't be read from this path, edit it as a single path")} : m_route.error;
		break;
	default:
		break;
	}
}

editor_track::point_ref editor_mode::track_handle_hit() const
{
	auto *track = selected_track();
	// the tracks of a set are moved by its gizmo, not by the points of one of them
	if (track == nullptr || false == editor_track::is_supported(*track) || m_track_tab != track_tab::path || (m_track_set.size() >= 2 && in_track_set(track)))
		return {};

	screen_projection const projection;
	float best = kHandleRadius * kHandleRadius;
	editor_track::point_ref hit;
	for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
	{
		for (auto const kind : kPointKinds)
		{
			// the control vectors only with the gizmo out: hidden, a track is changed by its ends and won't bend by chance
			if (false == m_track_gizmo_on && false == editor_track::is_end(kind))
				continue;
			editor_track::point_ref const point{i, kind};
			float const distance = projection.mouse_distance2(editor_track::point_position(*track, point)) * (editor_track::is_end(kind) ? 0.5f : 1.0f);
			if (distance < best)
			{
				best = distance;
				hit = point;
			}
		}
	}
	return hit;
}

void editor_mode::draw_track_overlay() const
{
	auto const *track = selected_track();
	if (track == nullptr)
		return;

	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
	bool const trackmode = ui()->mode() == nodebank_panel::TRACK;
	// a track of the set is drawn with the set, see draw_track_set()
	if (trackmode && m_track_set.size() >= 2 && in_track_set(track))
		return;

	ImU32 const coursecolor = IM_COL32(40, 220, 255, 220);
	int constexpr samples{32};
	for (auto const &path : track->m_paths)
	{
		auto previous = bezier{path}.point(0.0);
		for (int i = 1; i <= samples; ++i)
		{
			auto const next = bezier{path}.point(static_cast<double>(i) / samples);
			projection.line(drawlist, previous, next, coursecolor, 2.5f);
			previous = next;
		}
	}
	if (false == trackmode || m_route_tab || false == editor_track::is_supported(*track))
		return;

	// the control vectors are drawn, and dragged, only with the gizmo out
	ImU32 const vectorcolor = IM_COL32(200, 200, 200, 160);
	for (int i = 0; m_track_gizmo_on && i < static_cast<int>(track->m_paths.size()); ++i)
	{
		projection.line(drawlist, editor_track::point_position(*track, {i, editor_track::point_kind::start}), editor_track::point_position(*track, {i, editor_track::point_kind::control1}), vectorcolor, 1.0f);
		projection.line(drawlist, editor_track::point_position(*track, {i, editor_track::point_kind::end}), editor_track::point_position(*track, {i, editor_track::point_kind::control2}), vectorcolor, 1.0f);
	}
	for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
	{
		for (auto const kind : kPointKinds)
		{
			if (false == m_track_gizmo_on && false == editor_track::is_end(kind))
				continue;
			editor_track::point_ref const point{i, kind};
			ImVec2 screen;
			if (false == projection.project(editor_track::point_position(*track, point), screen))
				continue;
			bool const selected = (point == m_track_point);
			if (editor_track::is_end(kind))
			{
				ImU32 const color = editor_track::is_connected(*track, point) ? IM_COL32(60, 220, 60, 255) : IM_COL32(240, 60, 60, 255);
				drawlist->AddCircleFilled(screen, 6.0f, color);
			}
			else
			{
				drawlist->AddRectFilled(ImVec2(screen.x - 4.0f, screen.y - 4.0f), ImVec2(screen.x + 4.0f, screen.y + 4.0f), IM_COL32(230, 230, 230, 255));
			}
			if (selected)
				drawlist->AddCircle(screen, 10.0f, IM_COL32(255, 220, 40, 255), 16, 2.5f);
		}
	}
	if (m_track_snap.track != nullptr)
	{
		ImVec2 screen;
		if (projection.project(m_track_snap.position, screen))
			drawlist->AddCircle(screen, 14.0f, IM_COL32(255, 60, 255, 255), 20, 3.0f);
		projection.line(drawlist, m_track_snap.position, m_track_snap.position + m_track_snap.direction * 5.0, IM_COL32(255, 60, 255, 200), 2.0f);
	}
}

void editor_mode::commit_track_drag(bool const Force)
{
	// on letting go the rest of what follows the dragged tracks is yet to come, even with all of the drag committed
	if ((false == m_track_dirty && false == (Force && m_track_drag_on_the_way)) || m_track_drag.empty())
		return;
	auto const now = std::chrono::steady_clock::now();
	if (false == Force && now - m_track_last_commit < std::chrono::milliseconds(100))
		return;
	editor_track::commit(m_track_drag, false == Force);
	m_track_drag_on_the_way = false == Force;
	m_track_dirty = false;
	m_track_last_commit = now;
}

void editor_mode::trim_history()
{
	if (m_max_history_size >= 0 && (int)m_history.size() >= m_max_history_size)
		m_history.erase(m_history.begin(), m_history.begin() + ((int)m_history.size() - m_max_history_size + 1));
}

void editor_mode::push_track_snapshot(std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created, std::vector<TTrack *> Removed)
{
	// a path made and taken out again within the step was never there for an undo or a redo: an undo retiring the paths made and
	// then reviving the ones taken out would bring it back
	for (auto made{Created.begin()}; made != Created.end();)
	{
		auto const gone{std::ranges::find(Removed, *made)};
		if (gone == Removed.end())
		{
			++made;
			continue;
		}
		Removed.erase(gone);
		std::erase_if(States, [&](auto const &Entry) { return Entry.first == *made; });
		made = Created.erase(made);
	}
	if (States.empty() && Created.empty() && Removed.empty())
		return;

	// what the tracks removed carried goes to the ones laid in their place, now or later, in the step which lays them
	std::vector<TTrack *> changed;
	for (auto const &[track, state] : States)
		changed.push_back(track);
	auto kept{Created};
	kept.insert(kept.end(), changed.begin(), changed.end());
	orphans_register(Removed, kept);
	orphans_take_over(Created, changed);
	trim_history();
	auto *track = false == States.empty() ? States.front().first : false == Created.empty() ? Created.front() : Removed.front();
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::TrackEdit;
	snap.node_name = track->name();
	snap.node_ptr = track;
	snap.position = track->location();
	snap.uuid = track->uuid;
	snap.tracks = std::move(States);
	snap.created = std::move(Created);
	snap.removed = std::move(Removed);
	infra_attach(snap);
	snap.alignments = alignment_store::take_changes();
	snap.switch_records = switch_store::take_changes();
	snap.directives.insert(snap.directives.end(), m_drive_buffer.begin(), m_drive_buffer.end());
	m_drive_buffer.clear();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	++m_track_edits;
}

void editor_mode::push_kilometrage_snapshot()
{
	auto before{kilometrage::take_changes()};
	if (false == before.has_value())
		return;
	trim_history();
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Kilometrage;
	snap.kilometrage = std::move(before);
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

void editor_mode::restore_kilometrage(EditorSnapshot &Snapshot) const
{
	if (false == Snapshot.kilometrage.has_value())
		return;
	auto current{kilometrage::current()};
	kilometrage::restore(std::move(*Snapshot.kilometrage));
	Snapshot.kilometrage = std::move(current);
}

bool editor_mode::tracks_editable(std::vector<TTrack *> const &Tracks, std::string &Reason)
{
	for (auto const *track : Tracks)
	{
		std::string reason;
		if (track != nullptr && false == editor_track::can_edit_geometry(*track, reason))
		{
			Reason = name_of(*track) + ": " + reason;
			return false;
		}
	}
	Reason.clear();
	return true;
}

void editor_mode::restore_track_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo)
{
	if (Snapshot.tracks.empty() && Snapshot.created.empty() && Snapshot.removed.empty())
		return;

	++m_track_edits;
	std::vector<TTrack *> tracks;
	for (auto const &[snapshot_track, snapshot_state] : Snapshot.tracks)
		tracks.emplace_back(snapshot_track);
	m_infra_suspended = true;
	EditorSnapshot current{Snapshot};
	current.tracks.clear();
	auto involved{tracks};
	if (Undo)
		involved.insert(involved.end(), Snapshot.created.begin(), Snapshot.created.end());
	for (auto *track : involved)
		current.tracks.emplace_back(track, editor_track::capture(*track));
	current.infra.clear();
	for (auto const &entry : Snapshot.infra)
		if (auto const *binding{infra_find(entry.saved)})
			current.infra.push_back(infra_state_of(*binding));
	if (Snapshot.profiles)
	{
		std::vector<scene::layer_handle> layers;
		for (auto const *library : {&std::as_const(m_profiles), &Snapshot.profile_library})
			for (auto const &entry : *library)
				layers.push_back(entry.layer);
		current.profile_library = std::move(m_profiles);
		m_profiles = Snapshot.profile_library;
		profile_library_mark(std::move(layers));
	}
	alignment_store::settle();
	switch_store::settle();
	if (Snapshot.alignments.has_value())
	{
		current.alignments = alignment_store::records();
		alignment_store::restore(*Snapshot.alignments);
	}
	if (Snapshot.switch_records.has_value())
	{
		current.switch_records = switch_store::records();
		switch_store::restore(*Snapshot.switch_records);
	}
	Opposite.push_back(std::move(current));

	if (Undo)
	{
		for (auto *track : Snapshot.created)
			editor_track::retire(*track);
		for (auto *track : Snapshot.removed)
		{
			editor_track::revive(*track);
			tracks.push_back(track);
		}
	}
	else
	{
		for (auto *track : Snapshot.removed)
			editor_track::retire(*track);
		for (auto *track : Snapshot.created)
			editor_track::revive(*track);
	}
	// names passed round among the tracks: a track whose name another one gets back gives it up first. The drives the step
	// put back have the names of their switches already
	std::vector<scene::instance_handle> restored;
	for (auto const &[handle, directive] : Snapshot.directives)
		restored.push_back(handle);
	for (auto const &[snapshot_track, snapshot_state] : Snapshot.tracks)
		if (snapshot_state.name != snapshot_track->name() && std::ranges::any_of(Snapshot.tracks, [&](auto const &Other) { return Other.first != snapshot_track && Other.second.name == snapshot_track->name(); }))
			editor_track::rename(*snapshot_track, snapshot_track->name() + "~", restored);
	for (auto const &[snapshot_track, snapshot_state] : Snapshot.tracks)
		if (snapshot_state.name != snapshot_track->name())
			editor_track::rename(*snapshot_track, snapshot_state.name, restored);
	for (auto const &[snapshot_track, snapshot_state] : Snapshot.tracks)
		editor_track::apply(*snapshot_track, snapshot_state);
	editor_track::commit(tracks);
	for (auto const &entry : Snapshot.infra)
		infra_restore_state(entry);
	m_infra_buffer.clear();
	m_drive_buffer.clear();
	m_infra_suspended = false;
	if (false == m_route.chain.tracks.empty())
	{
		std::string error;
		if (m_route.from->m_editorremoved || m_route.to->m_editorremoved || false == editor_track::find_chain(m_route.from, m_route.to, m_route.chain, error, true))
		{
			m_route.chain = {};
			m_route.result = {};
			m_route.vertex = -1;
			m_route.grip = -1;
		}
		else
		{
			route_bind_ends();
			route_update();
		}
	}

	lay_after_history(Undo);
	m_track_snap = {};
	m_track_dirty = false;
	m_track_point = {};
	m_node = tracks.empty() || tracks.front()->m_editorremoved ? nullptr : tracks.front();
	ui()->set_node(m_node);
	straight_refresh();
	profile_after_undo();
	if (Snapshot.profiles && false == m_profile.route.spans.empty())
		profile_restore();
}

std::array<editor_mode::track_mode, 12> const &editor_mode::track_modes()
{
	static std::array<track_mode, 12> const modes{{
	    {"Select", "Esc", track_tab::path, "LMB on a path selects it: its points, control vectors and parameters", editor_icons::icon::select, 0},
	    {"Lay track", "L", track_tab::lay, "Lays new track through the clicked points, on the ground or from a free end of a track", editor_icons::icon::lay_track, 0},
	    {"Plan", "G", track_tab::straights, "The plan of the line laid: LMB on a straight drags its ends or the middle, on a curve its vertex and radius. C works too", editor_icons::icon::curve, 0},
	    {"Switch", "T", track_tab::turnout, "Puts switches into a straight or a curve: press on it and drag along the way the switch goes. Edits the geometry of the selected switch", editor_icons::icon::turnout, 0},
	    {"Parallel", "N", track_tab::parallel, "A second track beside a track: LMB on it, then the side and the spacing", editor_icons::icon::parallel, 0},
	    {"Turntable", "U", track_tab::turntable, "Places a turntable, leads tracks out of it and fits the tracks around to it", editor_icons::icon::turntable, 1},
	    {"Signalling", "I", track_tab::srk, "The signals and the signalling of the stations: LMB on a switch lays its station out as a schematic, the signals placed by hand from the templates in the panel", editor_icons::icon::signal, 1},
	    {"Kilometrage", "X", track_tab::kilometrage, "The km of the railway lines: LMB on a track puts a point of the km there, a jump of the km too", editor_icons::icon::kilometrage, 1},
	    {"Objects", "B", track_tab::lineside, "Along the track: hectometre posts, fouling point markers, trainsets", editor_icons::icon::objects, 1},
	    {"Profile", "P", track_tab::profile, "Vertical profile (grade line) along the line", editor_icons::icon::profile, 2},
	    {"Speed", "V", track_tab::speed, "Speed limits of the paths against the speed their geometry allows", editor_icons::icon::speed, 2},
	    {"Joints", "J", track_tab::joints, "Ends of the paths which almost meet, steps, kinks, jumps of the cant and of the grade at the joints", editor_icons::icon::joints, 2},
	    // the infrastructure tool (track_tab::infra) is left out: no button nor key
	}};
	return modes;
}

void editor_mode::show_track_tab(track_tab const Wanted)
{
	auto const Tab{Wanted == track_tab::straights || Wanted == track_tab::route ? plan_tab_for(selected_track()) : Wanted};
	m_track_window_open = true;
	m_roadtool.window = false;
	if (Tab == m_track_tab)
		return;
	m_track_mode_notice.clear();
	// what was going on in the mode left stays behind
	cancel_track_tools();
	m_straights.tool = 0;
	m_lay.points.clear();
	m_lay.modes.clear();
	m_lay.plan = {};
	m_lay.start = {};
	m_lay.preview.clear();
	m_lay.status.clear();
	m_lay.active = Tab == track_tab::lay;
	m_switch.armed = -1;
	m_signal.armed = false;
	m_track_tab = Tab;
	auto *track{selected_track()};
	switch (Tab)
	{
	case track_tab::turnout:
		arm_switch();
		break;
	case track_tab::straights:
		if (track != nullptr && track->eType == tt_Switch)
			m_track_mode_notice = STR_C("Switches aren't a part of a line: LMB on a plain path");
		break;
	case track_tab::route:
		if (track != nullptr && std::find(m_route.chain.tracks.begin(), m_route.chain.tracks.end(), track) == m_route.chain.tracks.end())
			open_route_on(*track);
		break;
	case track_tab::profile:
		if (track != nullptr && m_profile.route.spans.empty())
			profile_open_run(*track);
		break;
	case track_tab::turntable:
		scan_turntables(false);
		turntable_read(track);
		break;
	case track_tab::srk:
		signal_refresh(true);
		break;
	default:
		break;
	}
}

void editor_mode::open_route_on(TTrack &Track)
{
	if (Track.eType != tt_Normal || false == editor_track::is_supported(Track))
		m_track_mode_notice = STR_C("Switches aren't a part of a line: LMB on a plain path");
	else if (false == route_load(Track))
		m_track_mode_notice = m_route.error.empty() ? std::string{STR_C("The curve can't be read from this path, edit it as a single path")} : m_route.error;
}

void editor_mode::arm_switch()
{
	auto &tool{m_switch};
	if (false == tool.collected)
	{
		tool.templates = editor_track::standard_switch_templates();
		tool.collected = true;
	}
	tool.status.clear();
	tool.armed = tool.templates.empty() ? -1 : std::clamp(tool.last, 0, static_cast<int>(tool.templates.size()) - 1);
	if (tool.armed >= 0 && tool.templates[tool.armed].double_slip)
	{
		auto const ordinary{std::find_if(tool.templates.begin(), tool.templates.end(), [](editor_track::switch_template const &Template) { return false == Template.double_slip; })};
		tool.armed = ordinary != tool.templates.end() ? static_cast<int>(ordinary - tool.templates.begin()) : -1;
	}
}

bool editor_mode::track_shortcut(int const Key)
{
	if (Key == GLFW_KEY_M)
	{
		m_tape = {false == m_tape.active, {}, m_tape.mouse};
		return true;
	}
	if (Key == GLFW_KEY_H)
	{
		auto *track{selected_track()};
		if (track == nullptr || track->eType != tt_Switch)
			return false;
		throw_switch(*track);
		return true;
	}
	auto const key{Key == GLFW_KEY_C ? GLFW_KEY_G : Key};
	for (auto const &mode : track_modes())
	{
		if (mode.key[1] != '\0' || key != GLFW_KEY_A + (mode.key[0] - 'A'))
			continue;
		// the key of the mode on leaves it
		auto const on{m_track_tab == mode.tab || (mode.tab == track_tab::straights && track_plan_tab())};
		show_track_tab(m_track_window_open && on ? track_tab::path : mode.tab);
		return true;
	}
	return false;
}

editor_mode::track_tab editor_mode::plan_tab_for(TTrack const *Track) const
{
	if (Track == nullptr)
		return track_plan_tab() ? m_track_tab : track_tab::straights;
	if (Track->eType == tt_Normal && editor_track::is_supported(*Track) && false == editor_track::is_straight(*Track, m_straights.tolerance))
		return track_tab::route;
	return track_tab::straights;
}

bool editor_mode::track_busy() const
{
	auto const &straights{m_straights};
	return (m_lay.active && false == m_lay.points.empty()) || m_turntable.placing || m_signal.placing || m_signal.moving || straights.dragging || m_extend.active || m_point_drag.active || m_handle_drag.active || m_switch.placing || m_track_point.valid() || m_route.vertex >= 0 || m_route.grip >= 0 || false == straights.detour.empty() || straights.tool != 0 || straights.handle >= 0;
}

std::string editor_mode::track_mode_name() const
{
	if (m_tape.active)
		return STR_C("Measure");
	if (false == m_track_window_open)
		return STR_C("Select");
	switch (m_track_tab)
	{
	case track_tab::lay: return STR_C("Lay track");
	case track_tab::turnout: return STR_C("Switch");
	case track_tab::straights:
		if (false == m_straights.detour.empty() || m_straights.tool == 2)
			return STR_C("Plan, straight: S-curve");
		if (m_straights.tool == 1)
			return STR_C("Plan, straight: break");
		return STR_C("Plan, straight");
	case track_tab::route: return STR_C("Plan, curve");
	case track_tab::profile: return STR_C("Vertical profile");
	case track_tab::speed: return STR_C("Speed check");
	case track_tab::joints: return STR_C("Joints");
	case track_tab::lineside: return STR_C("Objects along the track");
	case track_tab::infra: return STR_C("Infrastructure");
	case track_tab::parallel: return STR_C("Parallel track");
	case track_tab::srk: return STR_C("Signalling");
	case track_tab::kilometrage: return STR_C("Kilometrage");
	case track_tab::turntable: return STR_C("Turntable");
	default: return STR_C("Select");
	}
}

void editor_mode::render_track_inspector()
{
	auto const &io{ImGui::GetIO()};
	m_profile.open = m_track_window_open && m_track_tab == track_tab::profile;
	m_rail_tools->speed.set_open(m_track_window_open && m_track_tab == track_tab::speed);
	m_infra.open = m_track_window_open && m_track_tab == track_tab::infra;
	m_joints.set_open(m_track_window_open && m_track_tab == track_tab::joints);
	m_rail_tools->srk.set_open(m_track_window_open && m_track_tab == track_tab::srk);
	m_rail_tools->km.set_open(m_track_window_open && m_track_tab == track_tab::kilometrage);
	if (false == m_track_window_open || m_track_tab != track_tab::lineside)
	{
		m_sweep.open = false;
		m_hekto.tab().open = false;
		m_fouling.tab().open = false;
		m_parallel.tab().open = false;
		m_vehicle.tab().open = false;
	}
	ui()->set_track(m_track_window_open);
	if (false == m_track_window_open)
	{
		// a closed window takes its mode with it
		if (m_track_tab != track_tab::path)
			show_track_tab(track_tab::path);
		m_track_window_open = false;
		return;
	}

	m_route_tab = m_track_tab != track_tab::path;

	// the tools of the mode are drawn in the tool options window and the selected path in the inspector, see
	// render_track_tool_options() and render_track_selection(); the checks list what they find in a window of their own
	if (m_track_tab == track_tab::speed || m_track_tab == track_tab::joints || m_track_tab == track_tab::infra)
	{
		auto const *title{m_track_tab == track_tab::speed ? STR_C("Speed check") : m_track_tab == track_tab::joints ? STR_C("Joints") : STR_C("Infrastructure")};
		ImGui::SetNextWindowSize(ImVec2(600.0f, std::min(320.0f, io.DisplaySize.y - 80.0f)), ImGuiCond_FirstUseEver);
		if (ImGui::Begin((std::string{title} + "###trackanalysis").c_str(), nullptr, ImGuiWindowFlags_NoCollapse))
		{
			switch (m_track_tab)
			{
			case track_tab::speed: m_rail_tools->speed.render_options(); break;
			case track_tab::joints: m_joints.render_options(); break;
			default: render_infra_body(); break;
			}
		}
		ImGui::End();
	}

	m_profile.strip_top = -1.0f;
	if (m_track_tab == track_tab::profile && m_track_window_open)
		render_profile_strip();
	m_rail_tools->srk.render_windows();
}

void editor_mode::render_track_tool_options()
{
	auto *track{selected_track()};
	// the tools themselves are in the toolbar
	render_track_search();
	ImGui::Spacing();
	render_track_guide();
	// the texts of the tools go on to the next line at the edge of the panel, none of them makes it scroll sideways
	ImGui::PushTextWrapPos(0.0f);
	switch (m_track_tab)
	{
	case track_tab::lay:
		render_lay_ui();
		break;
	case track_tab::parallel:
		m_parallel.render_options();
		break;
	case track_tab::turnout:
		if (track != nullptr && track->eType == tt_Switch)
			render_turnout_ui();
		else
			render_switch_ui();
		break;
	case track_tab::straights:
		render_straight_ui();
		if (ImGui::CollapsingHeader(STR_C("Straights in the scenery")))
			render_straights_ui();
		break;
	case track_tab::route:
		render_route_ui();
		break;
	case track_tab::profile: render_profile_body(); break;
	case track_tab::speed:
	case track_tab::joints:
	case track_tab::infra:
		ImGui::TextDisabled("%s", STR_C("What the check finds is listed in its window"));
		break;
	case track_tab::lineside: render_lineside_ui(); break;
	case track_tab::srk:
		render_signal_selected();
		m_rail_tools->srk.render_options();
		break;
	case track_tab::kilometrage: m_rail_tools->km.render_options(); break;
	case track_tab::turntable: render_turntable_ui(); break;
	default:
		render_track_set_ui();
		render_track_spread_ui();
		break;
	}
	ImGui::PopTextWrapPos();
}

void editor_mode::render_track_selection()
{
	m_rail_tools->track_sheet.track_field_settle();
	auto const tracks{selected_tracks()};
	if (tracks.empty())
	{
		ImGui::TextDisabled("%s", STR_C("Nothing selected"));
		if (m_track_tab != track_tab::path)
			return;
		ImGui::Spacing();
		gesture_rows({{"LMB on a track", "its properties show here"}, {"Ctrl+F", "finds a track by its name"}});
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::TextDisabled("%s", STR_C("Or start with:"));
		auto const width{std::floor((ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f)};
		if (ImGui::Button(STR_C("Lay a track  (L)"), ImVec2(width, ImGui::GetFrameHeight() * 1.4f)))
			show_track_tab(track_tab::lay);
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Tutorial"), ImVec2(width, ImGui::GetFrameHeight() * 1.4f)))
			m_first_steps_open = true;
		return;
	}
	// the properties belong to the select tool and to the switch tool, the other tools show the track they work with
	auto const sheet{m_track_tab == track_tab::path || m_track_tab == track_tab::turnout};
	if (tracks.size() >= 2)
	{
		m_rail_tools->track_sheet.render_track_set_header(tracks);
		if (false == sheet)
			return;
		m_rail_tools->track_sheet.render_track_select_tools(tracks);
		render_track_terrain(tracks);
		m_rail_tools->track_sheet.render_track_properties(tracks);
		m_rail_tools->track_style.render_track_style(tracks);
		ImGui::Spacing();
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.18f, 0.15f, 1.0f));
		if (ImGui::Button(format(STR_P("Delete %d track", "Delete %d tracks", static_cast<long long>(tracks.size())), static_cast<int>(tracks.size())).c_str()))
			track_set_delete();
		ImGui::PopStyleColor();
		item_tooltip("Deletes the selected tracks (Del); Ctrl+Z brings them back");
		return;
	}
	auto const *track{tracks.front()};
	if (false == sheet)
	{
		ImGui::TextUnformatted(name_of(*track).c_str());
		ImGui::TextDisabled("%s, %.2f m", STR_C(track_type_name(*track)), track->Length());
		return;
	}
	render_path_ui();
}

void editor_mode::render_track_guide()
{
	// a card of the tool in use: what it is for, and how the mouse and the keys drive it, drawn
	auto const &modes{track_modes()};
	auto const tab{card_tab(m_track_window_open, m_track_tab)};
	auto const found{std::ranges::find_if(modes, [&](track_mode const &Mode) { return Mode.tab == tab; })};
	auto const family{found != modes.end() ? found->family : 0};
	auto const accent{family_colour(family)};
	auto const font{ImGui::GetFontSize()};
	auto const pad{std::round(font * 0.6f)};
	auto *list{ImGui::GetWindowDrawList()};
	auto const top{ImGui::GetCursorScreenPos()};
	auto const width{ImGui::GetContentRegionAvail().x};
	auto const right_edge{top.x + width};
	list->ChannelsSplit(2);
	list->ChannelsSetCurrent(1);
	ImGui::BeginGroup();
	ImGui::Dummy(ImVec2(width, pad * 0.6f));
	ImGui::Indent(pad + 3.0f);
	auto const row{ImGui::GetCursorScreenPos()};
	auto const iconside{std::round(font * 1.7f)};
	if (found != modes.end())
		editor_icons::draw(list, found->picture, ImVec2(row.x + iconside * 0.5f, row.y + iconside * 0.5f), accent);
	ImGui::Dummy(ImVec2(iconside, iconside));
	ImGui::SameLine(0.0f, pad);
	ImGui::BeginGroup();
	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
	ImGui::SetWindowFontScale(1.15f);
	ImGui::TextUnformatted(track_mode_name().c_str());
	ImGui::SetWindowFontScale(1.0f);
	ImGui::PopStyleColor();
	ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(accent));
	ImGui::TextUnformatted(family_name(family));
	ImGui::PopStyleColor();
	ImGui::EndGroup();
	if (found != modes.end() && found->key != nullptr)
	{
		auto const height{std::round(font * 1.35f)};
		auto const cap{std::max(std::round(ImGui::CalcTextSize(found->key).x + height * 0.6f), height)};
		gesture_key(*list, ImVec2(right_edge - pad - cap, row.y + std::round((iconside - height) * 0.5f)), height, found->key);
	}
	ImGui::PushTextWrapPos(right_edge - pad - ImGui::GetWindowPos().x);
	if (found != modes.end())
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextWrapped("%s", STR_C(found->tooltip));
		ImGui::PopStyleColor();
	}
	// the gestures, each drawn: the mouse with the button lit, the keys as caps
	auto const hints{track_key_hints(false)};
	auto const gesture{std::round(font * 1.25f)};
	std::vector<std::vector<gesture_word>> drawn;
	float column{0.0f};
	for (auto const &hint : hints)
	{
		drawn.push_back(gesture_picture(hint.raw != nullptr ? hint.raw : hint.key));
		column = std::max(column, gesture_width(gesture, drawn.back()));
	}
	column = std::min(column, font * 6.0f);
	if (false == hints.empty())
		ImGui::Dummy(ImVec2(0.0f, pad * 0.3f));
	for (std::size_t i = 0; i < hints.size() && i < 7; ++i)
	{
		auto const at{ImGui::GetCursorScreenPos()};
		auto const line{ImGui::GetTextLineHeight()};
		auto const height{std::max(gesture, line)};
		list->PushClipRect(at, ImVec2(at.x + column, at.y + height), true);
		gesture_draw(*list, ImVec2(at.x, at.y + std::round((height - gesture) * 0.5f)), gesture, drawn[i], ImGui::GetColorU32(ImGuiCol_Text));
		list->PopClipRect();
		ImGui::Dummy(ImVec2(column, height));
		ImGui::SameLine(0.0f, pad);
		ImGui::SetCursorPosY(ImGui::GetCursorPosY() + std::round((height - line) * 0.5f));
		if (hints[i].raw == nullptr || false == gesture_keys_only(hints[i].raw))
		{
			ImGui::TextColored(ImVec4(1.0f, 0.83f, 0.3f, 1.0f), "%s", hints[i].key);
			ImGui::SameLine();
		}
		ImGui::TextWrapped("%s", hints[i].action.c_str());
	}
	ImGui::PopTextWrapPos();
	ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
	ImGui::TextUnformatted(STR_C("F1: all the shortcuts"));
	ImGui::PopStyleColor();
	if (ImGui::IsItemHovered())
		ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
	if (ImGui::IsItemClicked())
		m_shortcuts_open = true;
	ImGui::Unindent(pad + 3.0f);
	ImGui::Dummy(ImVec2(width, pad * 0.5f));
	ImGui::EndGroup();
	auto const bottom{ImGui::GetItemRectMax().y};
	list->ChannelsSetCurrent(0);
	auto const rounding{std::round(font * 0.35f)};
	list->AddRectFilled(top, ImVec2(right_edge, bottom), (accent & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 28), rounding);
	list->AddRect(top, ImVec2(right_edge, bottom), (accent & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 70), rounding);
	list->AddRectFilled(top, ImVec2(top.x + 4.0f, bottom), accent, rounding, ImDrawFlags_RoundCornersLeft);
	list->ChannelsMerge();
	if (false == m_track_mode_notice.empty())
	{
		ImGui::Spacing();
		ImGui::PushTextWrapPos(ImGui::GetWindowContentRegionMax().x - 8.0f);
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", m_track_mode_notice.c_str());
		ImGui::PopTextWrapPos();
	}
	ImGui::Spacing();
}

void editor_mode::render_track_name(TTrack &Track)
{
	auto &edit{m_track_name};
	auto const current{Track.name() == "none" ? std::string{} : Track.name()};
	auto const sync = [&]() {
		edit.track = &Track;
		edit.synced = current;
		std::snprintf(edit.text, sizeof(edit.text), "%s", current.c_str());
		edit.uses = editor_track::uses_of_name(Track);
	};
	if (edit.track != &Track)
	{
		sync();
		edit.status.clear();
	}
	else if (edit.synced != current && edit.synced == edit.text)
		sync();

	std::string why;
	if (false == scene::Layers.editable(&Track, &why))
	{
		ImGui::Text(STR_C("Name: %s"), current.empty() ? STR_C("(no name)") : current.c_str());
		ImGui::SameLine();
		ImGui::TextDisabled(STR_C("(can't be changed: %s)"), why.c_str());
		return;
	}
	auto const apply = [&]() {
		std::string name{edit.text};
		std::string reason;
		if (false == editor_track::name_valid(Track, name, reason))
			return;
		auto const before{editor_track::capture(Track)};
		editor_track::rename(Track, name);
		push_track_snapshot({{&Track, before}});
		edit.status = format(STR_C("Renamed from %s"), current.empty() ? STR_C("(no name)") : current.c_str());
		sync();
	};
	if (begin_fields("##tracknamefields", 0.3f))
	{
		field_row("Name##trackname");
		if (ImGui::InputTextWithHint("##trackname", STR_C("(no name)"), edit.text, sizeof(edit.text), ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsNoBlank))
			apply();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Enter renames the path, Esc while typing takes the text back.\nThe events, memory cells and the map of the game find the path by its name"));
		ImGui::EndTable();
	}
	std::string typed{edit.text};
	if (typed == edit.synced)
	{
		if (false == edit.status.empty())
		{
			ImGui::SameLine();
			ImGui::TextColored(ImVec4(0.5f, 0.9f, 0.5f, 1.f), "%s", edit.status.c_str());
		}
		return;
	}
	std::string reason;
	TTrack *owner{nullptr};
	auto const valid{editor_track::name_valid(Track, typed, reason, &owner)};
	ImGui::SameLine();
	if (valid)
	{
		auto const warned{false == edit.uses.events.empty() || false == edit.uses.loose.empty()};
		if (ImGui::SmallButton(warned ? STR_C("Rename anyway") : STR_C("Rename")))
			apply();
		std::vector<std::string> notes;
		if (typed != edit.text)
			notes.push_back(format(STR_C("Saved as %s: the scenery keeps the names in lower case"), typed.c_str()));
		if (edit.uses.cells > 0 || edit.uses.includes > 0)
			notes.push_back(format(STR_C("The new name goes to %d memory cell(s) and %d include(s), the drives of a switch among them"), edit.uses.cells, edit.uses.includes));
		if (warned)
			notes.emplace_back(STR_C("The events listed below work until the scenery is loaded again; then they need the new name in their files"));
		if (false == notes.empty() && ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", lines_of(notes).c_str());
	}
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Keep the old")))
	{
		std::snprintf(edit.text, sizeof(edit.text), "%s", edit.synced.c_str());
		return;
	}
	if (false == valid)
	{
		ImGui::TextColored(ImVec4(1.f, 0.45f, 0.35f, 1.f), "%s", STR_C(reason.c_str()));
		if (owner != nullptr)
		{
			ImGui::SameLine();
			if (ImGui::SmallButton(STR_C("Look at it")))
				focus_track(*owner, 0, owner->location());
		}
		return;
	}
	auto const listed = [](std::vector<std::string> const &Names) {
		std::string result;
		for (std::size_t i = 0; i < Names.size() && i < 4; ++i)
			result += (i > 0 ? ", " : "") + Names[i];
		if (Names.size() > 4)
			result += format(" (+%d)", static_cast<int>(Names.size() - 4));
		return result;
	};
	if (false == edit.uses.events.empty())
		ImGui::TextColored(ImVec4(1.f, 0.75f, 0.3f, 1.f), STR_C("These events aim at the path by the old name and the editor doesn't write them: %s"), listed(edit.uses.events).c_str());
	if (false == edit.uses.loose.empty())
		ImGui::TextColored(ImVec4(1.f, 0.75f, 0.3f, 1.f), STR_C("These events belong to the path by the old name: %s"), listed(edit.uses.loose).c_str());
}

void editor_mode::render_turnout_ui()
{
	auto *track{selected_track()};
	if (track == nullptr || track->eType != tt_Switch || track->m_paths.size() < 2)
		return;
	auto const set{double_of(*track)};
	if (set.has_value())
		track = set->first;
	auto const &main{track->m_paths[0]};
	auto const &diverging{track->m_paths[1]};
	auto const origin{main.points[segment_data::point::start]};
	auto const chord{plan_of(main.points[segment_data::point::end] - origin)};
	auto const &control2{diverging.points[segment_data::point::control2]};
	auto const control{glm::length(plan_of(control2)) > 1e-6 ? control2 : diverging.points[segment_data::point::start] - diverging.points[segment_data::point::end]};
	if (glm::length(chord) < 1e-3 || glm::length(plan_of(control)) < 1e-6)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), STR_C("The geometry of this switch is degenerate, it can't be edited here"));
		return;
	}
	glm::dvec2 const axis{glm::normalize(chord)};
	glm::dvec2 const normal{-axis.y, axis.x};
	auto const length{glm::dot(chord, axis)};
	glm::dvec2 const end{diverging.points[segment_data::point::end].x - origin.x, diverging.points[segment_data::point::end].z - origin.z};
	auto const offset{glm::dot(end, normal)};
	glm::dvec2 const tangent{glm::normalize(glm::dvec2{-control.x, -control.z})};
	auto const angle{std::atan2(std::abs(tangent.x * axis.y - tangent.y * axis.x), glm::dot(tangent, axis))};
	std::string about;
	if (set.has_value())
	{
		auto const &second{set->second->m_paths.front()};
		about = format(STR_C("Double switch %s + %s: length %.3f m, the second switch starts %.3f m on"), set->first->name().c_str(), set->second->name().c_str(), glm::dot(plan_of(second.points[segment_data::point::end] - origin), axis), glm::dot(plan_of(second.points[segment_data::point::start] - origin), axis)) + "\n" +
		        format(STR_C("First switch: diverging %s, end offset %.3f m"), offset > 0.0 ? STR_C("right") : STR_C("left"), std::abs(offset));
	}
	else
		ImGui::Text(STR_C("Length %.3f m, diverging %s, end offset %.3f m"), length, offset > 0.0 ? STR_C("right") : STR_C("left"), std::abs(offset));
	{
		// the switch in plan at one scale: the main track white, the diverging one yellow, the angle at its end, the length under
		auto const font{ImGui::GetFontSize()};
		auto const width{ImGui::GetContentRegionAvail().x};
		auto const height{std::round(font * 4.2f)};
		auto const top{ImGui::GetCursorScreenPos()};
		ImGui::Dummy(ImVec2(width, height));
		if (false == about.empty() && ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", about.c_str());
		auto *list{ImGui::GetWindowDrawList()};
		list->AddRectFilled(top, ImVec2(top.x + width, top.y + height), ImGui::GetColorU32(ImGuiCol_FrameBg), std::round(font * 0.35f));
		auto const local = [&](glm::dvec3 const &Point) {
			glm::dvec2 const relative{Point.x - origin.x, Point.z - origin.z};
			return glm::dvec2{glm::dot(relative, axis), glm::dot(relative, normal)};
		};
		std::vector<std::vector<glm::dvec2>> lines;
		std::vector switches{track};
		if (set.has_value())
			switches.push_back(set->second);
		double far_along{0.0};
		double low_across{0.0};
		double high_across{0.0};
		for (auto const *each : switches)
			for (auto const &path : each->m_paths)
			{
				geometry::bezier const curve{path};
				std::vector<glm::dvec2> points;
				for (int i = 0; i <= 24; ++i)
				{
					auto const point{local(curve.point(i / 24.0))};
					far_along = std::max(far_along, point.x);
					low_across = std::min(low_across, point.y);
					high_across = std::max(high_across, point.y);
					points.push_back(point);
				}
				lines.push_back(std::move(points));
			}
		auto const pad{std::round(font * 0.7f)};
		auto const label{ImGui::GetTextLineHeight()};
		auto const room{height - 2.0f * pad - label};
		auto const scale{static_cast<float>(std::min((width - 2.0 * pad) / std::max(1.0, far_along), room / std::max(0.5, high_across - low_across)))};
		auto const base{top.y + pad + (room - static_cast<float>(high_across - low_across) * scale) * 0.5f - static_cast<float>(low_across) * scale};
		auto const screen = [&](glm::dvec2 const &Point) { return ImVec2(top.x + pad + static_cast<float>(Point.x) * scale, base + static_cast<float>(Point.y) * scale); };
		// the diverging tracks first, the main ones over them
		for (auto const main_line : {false, true})
			for (auto const &line : lines)
			{
				if ((std::abs(line.back().y - line.front().y) < 0.05) != main_line)
					continue;
				std::vector<ImVec2> drawn;
				for (auto const &point : line)
					drawn.push_back(screen(point));
				list->AddPolyline(drawn.data(), static_cast<int>(drawn.size()), main_line ? IM_COL32(235, 235, 235, 255) : IM_COL32(255, 210, 60, 255), 0, 2.0f);
			}
		// the point of the blades, and the angle at the end of the diverging track
		list->AddCircleFilled(screen(glm::dvec2{0.0, 0.0}), 3.0f, IM_COL32(230, 80, 230, 255));
		if (angle > 1e-6)
		{
			auto const text{format("1:%.1f", 1.0 / std::tan(angle))};
			auto const size{ImGui::CalcTextSize(text.c_str())};
			auto const at{screen(glm::dvec2{far_along, offset})};
			list->AddText(ImVec2(std::round(at.x - size.x - 2.0f), std::round(offset > 0.0 ? at.y - size.y - 1.0f : at.y + 1.0f)), IM_COL32(255, 210, 60, 255), text.c_str());
		}
		if (diverging.radius > 0.0f)
		{
			auto const text{format("R %.0f", static_cast<double>(diverging.radius))};
			auto const size{ImGui::CalcTextSize(text.c_str())};
			auto const at{screen(glm::dvec2{far_along * 0.45, offset * 0.25})};
			list->AddText(ImVec2(std::round(at.x - size.x * 0.5f), std::round(offset > 0.0 ? at.y + 3.0f : at.y - size.y - 3.0f)), IM_COL32(255, 210, 60, 200), text.c_str());
		}
		auto const text{format("%.2f m", far_along)};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		auto const y{top.y + height - pad * 0.4f - label};
		list->AddLine(ImVec2(top.x + pad, y + label * 0.5f), ImVec2(top.x + width - pad, y + label * 0.5f), ImGui::GetColorU32(ImGuiCol_TextDisabled, 0.5f));
		list->AddRectFilled(ImVec2(top.x + (width - size.x) * 0.5f - 4.0f, y), ImVec2(top.x + (width + size.x) * 0.5f + 4.0f, y + label), ImGui::GetColorU32(ImGuiCol_FrameBg));
		list->AddText(ImVec2(std::round(top.x + (width - size.x) * 0.5f), y), ImGui::GetColorU32(ImGuiCol_TextDisabled), text.c_str());
	}
	render_switch_template(*track, false);
	// the name, the number, the rail and the kilometre of the switch, with what the scenery keeps of it
	if (ImGui::CollapsingHeader(STR_C("Details##switchdetails")))
	{
		render_track_name(*track);
		render_switch_record(*track);
		if (angle > 1e-6)
			ImGui::Text(STR_C("Angle 1:%.2f (%.4f deg), R in the entry %.0f m"), 1.0 / std::tan(angle), glm::degrees(angle), diverging.radius);
	}
	render_switch_drive(*track, set.has_value());
	if (set.has_value())
		render_switch_drive(*set->second, true);
	{
		bool frogs{track->SwitchExtension->m_frogs};
		bool movable{track->SwitchExtension->m_movablenose};
		char const *const frogtip{"Cuts the rails of the switch at its frog and lays wing rails and check rails.\n"
		                          "Off for a switch whose include has its own model of them"};
		char const *const nosetip{"The nose of the frog moves with the blades over to the wing rail of the track set, leaving no gap; the frog has no check rails"};
		ImGui::BeginDisabled(false == Global.Frogs);
		if (frogs)
			toggle_chips("switchfrogs", {{STR_C("Frog and check rails made from the rails"), &frogs, frogtip}, {STR_C("Movable nose"), &movable, nosetip}});
		else
			toggle_chips("switchfrogs", {{STR_C("Frog and check rails made from the rails"), &frogs, frogtip}});
		ImGui::EndDisabled();
		if (false == Global.Frogs && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("%s", STR_C("Turned on with frogs yes in eu07.ini"));
		if (Global.Frogs && (frogs != track->SwitchExtension->m_frogs || movable != track->SwitchExtension->m_movablenose))
		{
			auto const before{editor_track::capture(*track)};
			track->SwitchExtension->m_frogs = frogs;
			track->SwitchExtension->m_movablenose = movable;
			push_track_snapshot({{track, before}});
			editor_track::commit_parameters(*track);
		}
	}
	if (false == m_switch.status.empty())
		ImGui::TextWrapped("%s", m_switch.status.c_str());

	auto &tool{m_switch};
	ImGui::Separator();
	auto const finish_set = [&](std::string const &Status) {
		switch_store::follow();
		records_to_history();
		straight_refresh();
		if (false == Status.empty())
			ui()->set_status(Status, true);
	};
	int const side{offset > 0.0 ? 1 : -1};
	// a switch of a connection laid whole or of a slip turns over with it only
	auto const flips{template_change_blocked(*track).empty() && false == slip_of(*track).has_value() && false == crossover_of(*track).has_value()};
	if (flips && set.has_value() && ImGui::Button(STR_C("Flip the side")))
	{
		auto const mirror = [&](glm::dvec3 const &Vector) {
			auto const across{glm::dot(plan_of(Vector), normal)};
			return glm::dvec3{Vector.x - 2.0 * across * normal.x, Vector.y, Vector.z - 2.0 * across * normal.y};
		};
		auto paths{double_paths(*set)};
		for (auto &path : paths)
			path.transform([&origin, &mirror](glm::dvec3 const &Point) { return origin + mirror(Point - origin); }, mirror);
		std::string status;
		if (false == move_double_paths(*set, paths, status))
			ui()->set_status(status, true);
		else
		{
			if (auto entry{switch_store::named(*set->first)})
			{
				switch_store::place(*entry, *set->first);
				switch_store::classify(*entry, *set->first);
				switch_store::put(*entry, alignment_store::layer_of(*set->first));
			}
			finish_set(status);
		}
	}
	else if (flips && false == set.has_value() && ImGui::Button(STR_C("Flip the side")))
	{
		std::string notes;
		bool done{false};
		if (geometry::has_controls(main))
		{
			// bent into a curve: bent into it again of its template, turned over
			if (auto const index{switch_template_index(*track)}; index >= 0)
				done = apply_switch_template(*track, tool.templates[index], true, notes);
			else
				notes = STR_C("A switch bent into a curve of no template of the catalogue isn't flipped: delete it and lay it again");
		}
		else
		{
			// its own shape, mirrored
			done = apply_switch_template(*track, mirrored_template(track, origin, axis, normal, side), true, notes);
		}
		if (false == notes.empty())
			ui()->set_status(notes, false == done);
	}
	if (flips)
		item_tooltip("Mirrors the switch: the diverging track goes to the other side");
	double low{0.0};
	double high{0.0};
	double reach{length};
	auto const slides{switch_slides(*track) && (set.has_value() ? switch_slides(*set->second) && double_room(*set, low, high, reach, m_switch.status) : switch_room(*track, low, high, m_switch.status))};
	ImGui::PushID("turnout");
	if (ImGui::TreeNode(STR_C("More settings")))
	{
		if (slides)
		{
			ImGui::PushItemWidth(100.0f);
			ImGui::InputDouble("##switchslide", &tool.slide_typed, 0.0, 0.0, "%.2f m");
			ImGui::PopItemWidth();
			ImGui::SameLine();
			if (ImGui::Button(STR_C("Slide along the straight")))
				slide_switch(*track, std::clamp(tool.slide_typed, low, high - reach));
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s\n%s", STR_C("Moves the switch along its straight by the distance typed, + towards the frog; the straight paths on both sides get shorter or longer. Dragging the switch with the mouse does the same"),
				                  format(STR_C("Room on the straight: %.2f m back, %.2f m ahead"), -low, high - reach).c_str());
		}
		else if (false == set.has_value())
		{
			// one bent into a curve slides along it, the tracks next to it keeping a metre; its curve found only while the settings are open
			render_bent_switch_slide(*track);
		}
		ImGui::TreePop();
	}
	ImGui::PopID();
	ImGui::Separator();
	ImGui::TextDisabled("%s", STR_C("The textures, the speed and the other properties of the switch are in the inspector"));
}

void editor_mode::render_switch_drive(TTrack &Switch, bool const Named)
{
	ImGui::PushID(&Switch);
	if (Named)
		ImGui::TextUnformatted(Switch.name().c_str());
	if (false == EditorIncludes.scanned())
		ImGui::TextDisabled("%s", STR_C("Looking for the templates..."));
	else if (auto const drive{find_switch_drive(Switch, false)})
		render_standing_drive(Switch, drive->family, drive->index);
	else if ((simulation::Events.FindEvent(Switch.name() + "+") != nullptr && simulation::Events.FindEvent(Switch.name() + "-") != nullptr) || (m_track_name.track == &Switch && m_track_name.uses.includes > 0))
		ImGui::TextDisabled(STR_C("Drive: events %s+ and %s- switch it, also in the map"), Switch.name().c_str(), Switch.name().c_str());
	else
	{
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", STR_C("No drive: the switch can't be switched during the game"));
		render_switch_drive_choice();
		if (m_switch.drive >= 0 && ImGui::Button(STR_C("Put the drive by the switch")))
			place_switch_drive(Switch);
	}
	ImGui::PopID();
}

void editor_mode::render_standing_drive(TTrack &Switch, int const Family, int const Variant)
{
	if (begin_fields("##standingdrive", 0.3f))
	{
		field_row("Drive##standing");
		render_drive_combo(Switch, Family);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s\n%s", STR_C("Puts another drive in place of the drive of this switch, on the same side"), format(STR_C("Drive: events %s+ and %s- switch it, also in the map"), Switch.name().c_str(), Switch.name().c_str()).c_str());
		ImGui::EndTable();
	}
	int side{((Variant >> 1) == (Variant & 1)) ? 0 : 1};
	if (segmented("standingside", side, {{0, STR_C("diverging side"), "Puts the drive of the switch on the other side of the track: the same drive, its variant for the other side"},
	                                     {1, STR_C("other side"), "Puts the drive of the switch on the other side of the track: the same drive, its variant for the other side"}}))
		flip_switch_drive(Switch);
}

void editor_mode::render_drive_combo(TTrack const &Switch, int const Family)
{
	if (false == ImGui::BeginCombo("##standing", m_switch.drives[Family].name.c_str()))
		return;
	for (int i = 0; i < static_cast<int>(m_switch.drives.size()); ++i)
		if (ImGui::Selectable(m_switch.drives[i].name.c_str(), Family == i) && i != Family)
			replace_switch_drive(Switch, i, false);
	ImGui::EndCombo();
}

void editor_mode::render_bent_switch_slide(TTrack &Track)
{
	std::vector<curve_sample> bentframe;
	std::array<TTrack *, 2> bentsides{};
	double bentstation{0.0};
	if (false == bent_switch_frame(Track, bentframe, bentsides, bentstation))
		return;
	auto const along{nearest_station(bentframe, Track.m_paths.front().points[segment_data::point::end]) - bentstation};
	auto const back{std::max(0.0, bentstation)};
	auto const ahead{std::max(0.0, bentframe.back().station - bentstation - along)};
	ImGui::PushItemWidth(100.0f);
	ImGui::InputDouble("##switchslide", &m_switch.slide_typed, 0.0, 0.0, "%.2f m");
	ImGui::PopItemWidth();
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Slide along the curve")))
		slide_switch(Track, std::clamp(m_switch.slide_typed, -back, ahead));
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s\n%s", STR_C("Moves the switch along the curve it's bent into by the distance typed, + towards the frog; the tracks of the curve on both sides get shorter or longer, one taken up whole goes and the switch joins what it joined, P4 of another switch say. Dragging the switch with the mouse does the same"),
		                  format(STR_C("Room on the curve: %.2f m back, %.2f m ahead"), back, ahead).c_str());
}

void editor_mode::render_path_events(TTrack &Track)
{
	auto *track{&Track};
	if (m_track_events_for != track && false == ImGui::IsAnyItemActive())
	{
		m_track_events_for = track;
		m_track_event[0] = '\0';
		m_track_events_missing.clear();
	}
	if (auto const count{events_count(*track)}; false == ImGui::CollapsingHeader(format("%s (%zu)###trackevents", STR_C("Events"), count).c_str()))
		return;
	auto const before{editor_track::capture(*track)};
	ImGui::TextDisabled("%s", STR_C("What the track sets off as the vehicles run on it"));
	item_tooltip("event0: a vehicle with a crew stands on the path, event1: runs along it towards its start, event2: towards its end.\n"
	             "eventall0/1/2: the same for any vehicle");
	auto const editable_events{track->Dynamics.empty()};
	for (int list = 0; list < 6; ++list)
		render_event_list(*track, list, editable_events, before);
	ImGui::PushItemWidth(100.0f);
	ImGui::Combo("##eventlist", &m_track_event_list, "event0\0event1\0event2\0eventall0\0eventall1\0eventall2\0");
	ImGui::PopItemWidth();
	ImGui::SameLine();
	ImGui::PushItemWidth(-60.0f);
	auto const entered{ImGui::InputText("##eventname", m_track_event.data(), m_track_event.size(), ImGuiInputTextFlags_EnterReturnsTrue)};
	ImGui::PopItemWidth();
	ImGui::SameLine();
	if ((ImGui::Button(STR_C("Add")) || entered) && m_track_event[0] != '\0')
	{
		if (false == editable_events)
		{
			ui()->set_status(STR_C("The events can't change while vehicles stand on the path"), true);
		}
		else
		{
			editor_track::events(*track, m_track_event_list).emplace_back(m_track_event.data(), nullptr);
			editor_track::bind_events(*track, m_track_events_missing);
			push_track_snapshot({{track, before}});
			track->mark_dirty();
			m_track_event[0] = '\0';
		}
	}
	item_tooltip("Name of the event to add to the chosen list. An event which doesn't exist is kept, and reported");
	if (false == m_track_events_missing.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), STR_C("No such events: %s"), m_track_events_missing.c_str());
}

void editor_mode::render_event_list(TTrack &Track, int const List, bool const Editable, editor_track::state const &Before)
{
	auto &events{editor_track::events(Track, List)};
	for (std::size_t i = 0; i < events.size(); ++i)
	{
		auto const &event{events[i]};
		// bound by the name of the path, not written in the file
		if (event.first.empty())
		{
			ImGui::TextDisabled(STR_C("%s  %s (by the name of the path)"), editor_track::event_keyword(List), event.second != nullptr ? STR_C("auto") : "?");
			continue;
		}
		ImGui::PushID(List * 1000 + static_cast<int>(i));
		if (ImGui::SmallButton("x") && Editable)
		{
			events.erase(events.begin() + i);
			editor_track::bind_events(Track, m_track_events_missing);
			push_track_snapshot({{&Track, Before}});
			Track.mark_dirty();
			ImGui::PopID();
			return;
		}
		ImGui::SameLine();
		if (event.second == nullptr)
			ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), STR_C("%s  %s (no such event)"), editor_track::event_keyword(List), event.first.c_str());
		else
			ImGui::Text("%s  %s", editor_track::event_keyword(List), event.first.c_str());
		ImGui::PopID();
	}
}

void editor_mode::render_path_ui()
{
	auto *track = selected_track();
	if (track == nullptr)
	{
		ImGui::TextDisabled(STR_C("No path selected"));
		return;
	}

	if (false == editor_track::is_supported(*track))
	{
		ImGui::TextDisabled(STR_C("Editing of this path type isn't supported"));
		return;
	}
	render_track_name(*track);
	ImGui::TextDisabled("%s, %.2f m", STR_C(track_type_name(*track)), track->Length());
	if (track->eType == tt_Normal && false == track->m_paths.empty())
	{
		// the track at a glance: tiles of its length, radius, grade, cant and speed, going on to the next row at the edge
		auto const &path{track->m_paths.front()};
		geometry::bezier const curve{path};
		auto const length{curve.plan_length()};
		auto const first{curve.first(0.5)};
		auto const second{curve.second(0.5)};
		auto const speed{std::hypot(first.x, first.z)};
		auto const bend{speed > 1e-9 ? std::abs(first.x * second.z - first.z * second.x) / (speed * speed * speed) : 0.0};
		auto const straight{editor_track::is_straight(*track, m_straights.tolerance) || bend < 1e-5};
		auto const rise{path.points[segment_data::point::end].y - path.points[segment_data::point::start].y};
		auto const roll{std::max(std::abs(path.rolls[0]), std::abs(path.rolls[1]))};
		metric_tiles({{format("%.2f m", track->Length()), STR_C("length"), ImGui::GetColorU32(ImGuiCol_Text)},
		              {straight ? std::string{STR_C("straight")} : format("R %.0f m", 1.0 / bend), STR_C("plan"), straight ? IM_COL32(255, 255, 255, 230) : overlay_color::grip},
		              {format("%+.1f mm/m", length > 1e-6 ? rise / length * 1000.0 : 0.0), STR_C("grade"), ImGui::GetColorU32(ImGuiCol_Text)},
		              {format("%.0f mm", 1500.0 * std::sin(glm::radians(static_cast<double>(roll)))), STR_C("cant"), IM_COL32(240, 170, 60, 255)},
		              {editor_track::velocity(*track) > 0.0 ? format("%.0f km/h", editor_track::velocity(*track)) : std::string{"-"}, STR_C("speed"), ImGui::GetColorU32(ImGuiCol_Text)}});
	}
	std::string reason;
	bool const geometry = editor_track::can_edit_geometry(*track, reason);
	if (false == geometry)
		ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), STR_C("Geometry locked: %s"), reason.c_str());
	if (track->eType == tt_Switch && m_track_tab == track_tab::path)
	{
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Edit the switch (T)")))
			show_track_tab(track_tab::turnout);
		item_tooltip("A switch: the turnout tool sets its geometry, side and drive");
	}
	if (track->eType == tt_Switch && track->SwitchExtension != nullptr)
	{
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Throw (H)")))
			throw_switch(*track);
		item_tooltip(track->SwitchExtension->CurrentIndex ? "Set to the diverging track: throws it over to the main one. In the editor only, to see the blades and the route; the simulation starts with the switches set to the main track"
		                                                   : "Set to the main track: throws it over to the diverging one. In the editor only, to see the blades and the route; the simulation starts with the switches set to the main track");
	}
	// the template of a switch, a slip or a crossing, another one picked in place
	if (geometry)
		render_switch_template(*track, true);
	std::vector const tracks{track};
	m_rail_tools->track_sheet.render_track_select_tools(tracks);
	render_track_terrain(tracks);
	m_rail_tools->track_sheet.render_track_properties(tracks);
	m_rail_tools->track_style.render_track_style(tracks);
	render_path_events(*track);
	// the shape is given by dragging in the view, the numbers are for the exact values
	if (m_track_tab != track_tab::path)
		return;

	auto const before = editor_track::capture(*track);
	if (ImGui::CollapsingHeader(track->eType == tt_Switch ? STR_C("Geometry of the paths") : STR_C("Geometry")))
	{
		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
		{
			ImGui::PushID(i);
			if (track->m_paths.size() > 1)
				ImGui::TextDisabled(STR_C("Path %d"), i + 1);
			render_path_points(*track, i);
			item_tooltip("(+) joined with another track, (-) a free end");

			auto &path = track->m_paths[i];
			// the second path of a tributary is only kept to be written back, nothing is drawn nor run on it
			if (false == geometry || (track->m_tributary && i > 0))
			{
				auto const &start = path.points[segment_data::point::start];
				auto const &end = path.points[segment_data::point::end];
				ImGui::TextDisabled(STR_C("Start (%.2f, %.2f, %.2f)  End (%.2f, %.2f, %.2f)"), start.x, start.y, start.z, end.x, end.y, end.z);
				ImGui::PopID();
				continue;
			}
			ImGui::PushItemWidth(-110.0f);
			render_point_fields(*track, i, before);
			render_cant_radius(*track, path, before);
			ImGui::PopItemWidth();
			ImGui::PopID();
		}
		if (geometry && ImGui::TreeNode(STR_C("Snapping of the ends")))
		{
			if (begin_fields("##snapfields", 0.45f))
			{
				field_row("Snap radius (m)");
				ImGui::DragFloat("##snapradius", &m_track_snap_radius, 0.05f, 0.0f, 50.0f, "%.2f");
				item_tooltip("A dragged free end within this distance of another free end joins it on release");
				ImGui::EndTable();
			}
			toggle_chips("snapflags", {{STR_C("Align with snapped path"), &m_track_align_tangent, "On joining, the end also takes over the direction of the other path, so the joint has no kink"},
			                           {STR_C("Drag connected ends"), &m_track_drag_connected, "Dragging a joined end, or moving and turning the whole path or switch,\nmoves the ends of the neighbouring paths with it"}});
			ImGui::TreePop();
		}
	}
}

void editor_mode::path_field_done(TTrack &Track, editor_track::state const &Before)
{
	if (ImGui::IsItemActivated())
		m_track_field_before = Before;
	if (ImGui::IsItemDeactivatedAfterEdit())
	{
		push_track_snapshot({{&Track, m_track_field_before}});
		editor_track::commit({&Track});
	}
}

void editor_mode::render_path_points(TTrack const &Track, int const Index)
{
	for (auto const kind : kPointKinds)
	{
		editor_track::point_ref const point{Index, kind};
		if (kind != editor_track::point_kind::start)
			ImGui::SameLine();
		std::string label = point_label(kind);
		if (editor_track::is_end(kind))
			label += editor_track::is_connected(Track, point) ? " (+)" : " (-)";
		if (ImGui::Selectable(label.c_str(), m_track_point == point, 0, ImVec2(90.0f, 0.0f)))
			m_track_point = (m_track_point == point ? editor_track::point_ref{} : point);
	}
}

void editor_mode::render_point_fields(TTrack &Track, int const Index, editor_track::state const &Before)
{
	if (false == m_track_point.valid() || m_track_point.path != Index)
		return;
	auto &path = Track.m_paths[Index];
	auto const kind{m_track_point.kind};
	if (editor_track::is_end(kind))
	{
		if (auto position = path.points[kind == editor_track::point_kind::start ? segment_data::point::start : segment_data::point::end]; ImGui::DragScalarN("x, y, z##point", ImGuiDataType_Double, glm::value_ptr(position), 3, 0.01f, nullptr, nullptr, "%.3f") && all_finite(position))
			editor_track::move_point(Track, m_track_point, position);
		item_tooltip("Drag to move, Ctrl+click to type the value");
		path_field_done(Track, Before);
	}
	else
	{
		auto &vector{path.points[kind == editor_track::point_kind::control1 ? segment_data::point::control1 : segment_data::point::control2]};
		if (auto typed{vector}; ImGui::DragScalarN(STR_C("vector##point"), ImGuiDataType_Double, glm::value_ptr(typed), 3, 0.01f, nullptr, nullptr, "%.3f") && all_finite(typed) && glm::length(typed) < 1e5)
			vector = typed;
		item_tooltip("Control vector, relative to its end; zero for a straight");
		path_field_done(Track, Before);
	}
}

void editor_mode::render_cant_radius(TTrack &Track, segment_data &Path, editor_track::state const &Before)
{
	if (false == ImGui::TreeNode(STR_C("Cant and radius")))
		return;
	if (begin_fields("##cantradius", 0.3f))
	{
		field_row("Cant (mm)");
		auto const lift = [](float const Roll) { return static_cast<float>(1500.0 * std::sin(glm::radians(static_cast<double>(Roll)))); };
		auto const roll = [](float const Lift) { return static_cast<float>(glm::degrees(std::asin(std::clamp(static_cast<double>(Lift) / 1500.0, -1.0, 1.0)))); };
		if (std::array<float, 2> cants{lift(Path.rolls[0]), lift(Path.rolls[1])}; ImGui::DragFloat2("##roll", cants.data(), 0.5f, -388.0f, 388.0f, "%.0f") && std::isfinite(cants[0]) && std::isfinite(cants[1]))
			Path.rolls = {roll(cants[0]), roll(cants[1])};
		item_tooltip("Cant at the start and at the end: how much higher one rail is, + raises the left rail looking along the path");
		path_field_done(Track, Before);
		field_row("Radius");
		if (auto radius{Path.radius}; ImGui::InputFloat("##radius", &radius, 0.0f, 0.0f, "%.1f") && std::isfinite(radius))
			Path.radius = radius;
		item_tooltip("Radius stored with the path in the scenery; the shape itself comes from the control vectors");
		path_field_done(Track, Before);
		ImGui::EndTable();
	}
	if (ImGui::Button(STR_C("Straighten"), ImVec2(-FLT_MIN, 0.0f)))
	{
		push_track_snapshot({{&Track, Before}});
		Path.points[segment_data::point::control1] = glm::dvec3{};
		Path.points[segment_data::point::control2] = glm::dvec3{};
		Path.radius = 0.0f;
		editor_track::commit({&Track});
	}
	item_tooltip("Removes the control vectors and the radius, the path becomes a straight between its ends");
	ImGui::TreePop();
}



// the designs kept for the lines the tracks deleted lay along cut down to the tracks left, in the same step of the undo history: what
// the status tells of it
std::string editor_mode::designs_trimmed(std::vector<TTrack *> const &Removed) const
{
	auto const cut{alignment_store::trim(Removed)};
	return cut > 0 ? std::string{STR_P("The design kept for the line is cut down to the tracks left along it", "The designs kept for the lines are cut down to the tracks left along them", static_cast<long long>(cut))} : std::string{};
}

void editor_mode::throw_switch(TTrack &Switch)
{
	if (Switch.eType != tt_Switch || Switch.SwitchExtension == nullptr)
		return;
	// as the simulation throws it, the blades going over at the speed of the switch, moved on frame by frame by move_thrown_switches. To see
	// the blades and the route the paths make; the simulation starts with the switches set to the main track
	auto const state{1 - (Switch.SwitchExtension->CurrentIndex & 1)};
	Switch.Switch(state);
	if (std::ranges::find(m_thrown_switches, &Switch) == m_thrown_switches.end())
		m_thrown_switches.push_back(&Switch);
	ui()->set_status(format(state ? STR_C("Switch %s set to the diverging track, H throws it back") : STR_C("Switch %s set to the main track, H throws it over"), Switch.name().c_str()), false);
}

void editor_mode::move_thrown_switches()
{
	auto const elapsed{static_cast<float>(Timer::GetDeltaRenderTime())};
	for (auto thrown{m_thrown_switches.begin()}; thrown != m_thrown_switches.end();)
	{
		auto *track{*thrown};
		if (track->m_editorremoved || track->eType != tt_Switch || track->SwitchExtension == nullptr)
		{
			thrown = m_thrown_switches.erase(thrown);
			continue;
		}
		auto &extension{*track->SwitchExtension};
		auto const offset_left{extension.fDesiredOffset - extension.fOffset};
		auto const step{elapsed * std::max(extension.fOffsetSpeed, 0.01f)};
		auto const done{std::abs(offset_left) <= step};
		extension.fOffset = done ? extension.fDesiredOffset : extension.fOffset + std::copysign(step, offset_left);
		// the blades laid again where they are; the animation of the cell, if it has the switch, finds it over and lets it go
		track->RaAnimate();
		thrown = done ? m_thrown_switches.erase(thrown) : std::next(thrown);
	}
}

void editor_mode::delete_selected_track()
{
	auto *track{selected_track()};
	if (track == nullptr)
		return;
	if (false == track->Dynamics.empty())
	{
		ui()->set_status(STR_C("The track can't be deleted while vehicles stand on it"), true);
		return;
	}
	if (auto const set{double_of(*track)})
	{
		delete_double_switch(*set);
		return;
	}
	// a slip goes whole, its two tracks left crossing in its place unless Shift asks for all of it
	if (auto const slip{slip_of(*track)})
	{
		delete_slip(*slip);
		return;
	}
	// retired first, so the infrastructure it carried and moves elsewhere is in the same undo step
	if (auto const known{switch_store::of(*track)})
		switch_store::forget(known->name);
	editor_track::retire(*track);
	// a switch leaves its main track behind, so the line isn't broken, unless Shift asks for all of it
	std::vector<TTrack *> created;
	if (track->eType == tt_Switch && false == track->m_paths.empty() && false == Global.shiftState)
	{
		auto *main{editor_track::create_path(*track, track->m_paths.front())};
		editor_track::velocity(*main, speed_check::path_velocity(*track, 0));
		editor_track::commit({main});
		created.push_back(main);
		ui()->set_status(STR_C("The switch is replaced by its main track, Shift+Del removes all of it"), false);
	}
	if (auto const note{designs_trimmed({track})}; false == note.empty())
		ui()->set_status(note, false);
	push_track_snapshot({}, created, {track});
	drives_deleted({track});
	// the model and the events of a turntable can't stay without its track
	if (track->eType == tt_Table)
		for (auto const instance : turntable_includes(*track))
		{
			scene::Layers.removed(instance, true);
			m_history.back().instances.push_back(instance);
		}
	if (std::find(m_route.chain.tracks.begin(), m_route.chain.tracks.end(), track) != m_route.chain.tracks.end())
		m_route = {};
	m_node = nullptr;
	ui()->set_node(nullptr);
	m_track_point = {};
	straight_refresh();
}

void editor_mode::delete_double_switch(double_switch const &Set)
{
	for (auto const *part : {Set.first, Set.link, Set.second})
		if (false == part->Dynamics.empty())
		{
			ui()->set_status(STR_C("The track can't be deleted while vehicles stand on it"), true);
			return;
		}
	if (auto const known{switch_store::named(*Set.first)})
		switch_store::forget(known->name);
	auto main{Set.first->m_paths.front()};
	main.points[segment_data::point::end] = Set.second->m_paths.front().points[segment_data::point::end];
	auto const velocity{speed_check::path_velocity(*Set.first, 0)};
	for (auto *part : {Set.first, Set.link, Set.second})
		editor_track::retire(*part);
	std::vector<TTrack *> created;
	if (false == Global.shiftState)
	{
		auto *straight{editor_track::create_path(*Set.first, main)};
		editor_track::velocity(*straight, velocity);
		editor_track::commit({straight});
		created.push_back(straight);
		ui()->set_status(STR_C("The double switch is replaced by its main track, Shift+Del removes all of it"), false);
	}
	if (auto const note{designs_trimmed({Set.first, Set.link, Set.second})}; false == note.empty())
		ui()->set_status(note, false);
	push_track_snapshot({}, created, {Set.first, Set.link, Set.second});
	drives_deleted({Set.first, Set.second});
	if (std::ranges::any_of(m_route.chain.tracks, [&Set](TTrack const *Each) { return Each == Set.first || Each == Set.link || Each == Set.second; }))
		m_route = {};
	m_node = nullptr;
	ui()->set_node(nullptr);
	m_track_point = {};
	straight_refresh();
}

void editor_mode::delete_slip(slip_set const &Slip)
{
	for (auto const *part : Slip.parts)
		if (false == part->Dynamics.empty())
		{
			ui()->set_status(STR_C("The track can't be deleted while vehicles stand on it"), true);
			return;
		}
	auto const created{slip_taken_out(Slip, false == Global.shiftState)};
	if (false == Global.shiftState)
	{
		// kept in the scenery as a crossing
		std::string base;
		int number{1};
		while (base.empty() || switch_store::has(base))
		{
			base = std::format("editor_skrz{}", number);
			++number;
		}
		record_slip(created, "crossing", "Skrz", 0.0, Slip.record.ratio > 0.0 ? std::atan(1.0 / Slip.record.ratio) : 0.0, base);
		ui()->set_status(STR_C("The slip is replaced by its two tracks crossing, Shift+Del removes all of it"), false);
	}
	if (auto const note{designs_trimmed(Slip.parts)}; false == note.empty())
		ui()->set_status(note, false);
	push_track_snapshot({}, created, Slip.parts);
	drives_deleted(Slip.switches);
	if (std::ranges::any_of(m_route.chain.tracks, [&Slip](TTrack const *Each) { return std::ranges::find(Slip.parts, Each) != Slip.parts.end(); }))
		m_route = {};
	m_node = nullptr;
	ui()->set_node(nullptr);
	m_track_point = {};
	straight_refresh();
}

std::vector<editor_mode::key_hint> editor_mode::track_key_hints(bool const All) const
{
	std::vector<key_hint> hints;
	bool gizmo{false};
	auto const *selected{selected_track()};
	if (false == tool_key_hints(hints) && false == tab_key_hints(hints, selected, gizmo) && false == window_key_hints(hints) && false == plan_key_hints(hints))
		gizmo = selection_key_hints(hints, selected);
	if (gizmo && false == m_track_gizmo_on)
	{
		gizmo_hidden(hints);
		gizmo = false;
	}
	if (auto const *meaning{typed_meaning()}; meaning != nullptr)
		hints.insert(hints.begin() + std::min<std::size_t>(1, hints.size()), {"0-9", meaning});
	if (All)
	{
		if (gizmo && m_gizmo_snap > 0.0f)
			hints.emplace_back("Ctrl", format(STR_C("gizmo snaps by %.2f m"), m_gizmo_snap));
		hints.emplace_back("K", "split the path under the cursor, a break of the grade at the height of the ground");
		hints.push_back({"O", "top view"});
		hints.emplace_back("Esc", "back to Select");
		if (Global.EditorOrtho)
			hints.push_back({"Hold the wheel", "pans the view"});
		hints.push_back({"Ctrl+Z", "undo"});
		hints.push_back({"Ctrl+Y", "redo"});
		hints.emplace_back("L T G N I B P V J", "tools: lay, switch, plan, parallel, signalling, objects, profile, speed, joints");
	}
	for (auto &hint : hints)
	{
		hint.raw = hint.key;
		hint.key = STR_C(hint.key);
		hint.action = STR(hint.action);
	}
	return hints;
}

bool editor_mode::tool_key_hints(std::vector<key_hint> &Hints) const
{
	using enum editor_mode::switch_kind;
	if (m_lay.active)
		lay_key_hints(Hints);
	else if (false == m_straights.detour.empty())
	{
		char const *const steps[] = {"leave the axis", "shifted", "start back", "back on the axis"};
		auto const step{std::min<std::size_t>(m_straights.detour.size(), 3)};
		Hints = {{m_straights.tool == 2 ? "LMB" : "Shift+click", format(STR_C("point %zu of 4: %s"), step + 1, STR_C(steps[step]))}, {"Esc", "cancel"}};
	}
	else if (m_extend.active)
	{
		Hints = {{"Drag", "lays the shape to the cursor, to a free end: joins it"}};
		if (editor_track::snap_target target; extend_snap(target))
			Hints.insert(Hints.end(), {{"Ctrl+wheel", "radius"}, {"Shift+wheel", "another shape"}});
		else if (m_track_layout != track_layout::straight)
			Hints.emplace_back("Ctrl+wheel", "radius");
		Hints.insert(Hints.end(), {{"Release", "build"}, {"Esc", "cancel"}});
	}
	else if (m_switch.placing && m_switch.free)
		Hints = m_switch.free_end.track != nullptr ? std::vector<key_hint>{{"Drag sideways", "side of the diverging track"},
		                                                                   {"Drag on past its end", "bends the main track toward the cursor"},
		                                                                   {"Tab", "the diverging track the other way or the same way"},
		                                                                   {"At a free end", "the main track leads into it, with Shift the diverging one"},
		                                                                   {"Esc", "cancel"}}
		                                           : std::vector<key_hint>{{"Drag along", "the direction of the main track, held from 4 m on"}, {"Drag sideways", "side of the diverging track"}, {"Back to the start", "the direction free again"}, {"Esc", "cancel"}};
	else if (m_switch.placing && m_switch.curved && current_switch_kind() == slip)
		Hints = {{"Drag", "the crossing track ahead through the quarter of the cursor, the slip curve of a single slip on its side"}, {"Esc", "cancel"}};
	else if (m_switch.placing)
		placing_key_hints(Hints);
	else
		return false;
	return true;
}

void editor_mode::lay_key_hints(std::vector<key_hint> &Hints) const
{
	bool const curve{m_lay.mode == lay_mode::curve};
	auto const *tab{curve ? "back to the bends of straights" : "trace a curve on the photo"};
	if (m_lay.points.empty())
		Hints = {{"LMB", "start, on the ground or at a free end"}, {"Tab", tab}, {"Drag from a free end", "lays the shape to the cursor"}, {"Shift+click a straight", "a parallel track (N)"}, {"Alt", "the direction in steps of 15 degrees, or 1:9, 1:6.6... to a straight track"}, {"Esc", "stop laying"}};
	else if (m_lay.points.size() > 1)
	{
		// the points clicked are no track until Enter: it comes right after the click, where the bar at the bottom shows it
		Hints = {{"LMB", curve ? "a point along the middle of the curve, which ends at the last one" : "a bend of the straights, eased with a large curve"}, {"Enter", "lay the points clicked"},
		         {"Backspace", "take back the point"}, {"LMB on a free end", "join and lay"}, {"Tab", tab}, {"Alt", "the direction in steps of 15 degrees, or 1:9, 1:6.6... to a straight track"},
		         {"Esc", "twice: drop the points not laid"}};
	}
	else
	{
		Hints = {{"LMB", curve ? "a point along the middle of the curve, which ends at the last one" : "a bend of the straights, eased with a large curve"}, {"LMB on a free end", "join and lay"}, {"Tab", tab}};
		if (m_lay.mouse_snap.track != nullptr && m_lay.start.track != nullptr)
			Hints.insert(Hints.end(), {{"Ctrl+wheel", "radius"}, {"Shift+wheel", "another shape"}});
		else if (m_lay.mouse_snap.track != nullptr && curve)
			Hints.emplace_back("Ctrl+wheel", "radius");
		Hints.insert(Hints.end(), {{"Alt", "the direction in steps of 15 degrees, or 1:9, 1:6.6... to a straight track"}, {"Enter", "stop laying"},
		                           {"Backspace", "take back the point"}, {"Esc", "stop laying"}});
	}
}

void editor_mode::placing_key_hints(std::vector<key_hint> &Hints) const
{
	if (m_switch.connection >= 0 && m_switch.recognised)
		Hints = std::vector<key_hint>{{"Let go", connection_name(m_switch.connection)}, {"Ctrl+wheel", "R of the aligning curve"}, {"Esc", "cancel"}};
	else if (m_switch.connection >= 0 && false == m_switch.curved)
		Hints = std::vector<key_hint>{{"Drag onto the other track", "the connection there"}, {"Shift+wheel", "another kind"}, {"Ctrl+wheel", "R of the aligning curve"}, {"Alt+wheel", "where a double crossover meets"}, {"Esc", "cancel"}};
	else
		Hints = std::vector<key_hint>{{"Drag along", "direction of the switch"}, {"Drag sideways", "side of the diverging track"}, {"Drag onto the track beside", "a crossover there"}, {"Esc", "cancel"}};
}

bool editor_mode::tab_key_hints(std::vector<key_hint> &Hints, TTrack const *Selected, bool &Gizmo) const
{
	using enum editor_mode::switch_kind;
	if (m_track_tab == track_tab::lay)
		Hints = {{"LMB", "start, on the ground or at a free end"}};
	else if (m_track_tab == track_tab::turnout && (selected_track() == nullptr || selected_track()->eType == tt_Switch))
		Gizmo = turnout_key_hints(Hints, Selected);
	else if (m_track_tab == track_tab::turnout && m_switch.armed < 0)
		Hints = {{"List in the panel", "the type of the switch"}, {"LMB on a track", "another track"}};
	else if (m_track_tab == track_tab::turnout && current_switch_kind() == slip)
		Hints = {{"Press+drag on a straight", "the slip with its crossing track, through the quarter of the cursor"}, {"Press+drag at a crossing", "of two straights at the skos of the slip"}, {"Press+drag on a curve", "the slip bent into it, in a side track only (11.4 pkt 4b)"}};
	else if (m_track_tab == track_tab::turnout)
		Hints = {{"Press+drag on the track", "along: direction, sideways: side"}, {"List in the panel", "the type of the switch"}, {"Press+drag on another track", "a switch on it"}};
	else if (m_track_tab == track_tab::profile && m_profile.route.spans.empty())
		Hints = {{"LMB on a track", "the grade line of its whole line"}, {"Shift+LMB on more tracks", "the route between the outermost ones"}, {"Shift+drag a box", "the route over the tracks in it"}};
	else if (m_track_tab == track_tab::profile && m_profile.picking)
		Hints = {{"LMB on a track", "the stretch from switch to switch onto the common grade line"}, {"LMB on a blue one", "off it"}, {"Shift+LMB", "the track only"}, {"Esc", "done"}};
	else if (m_track_tab == track_tab::profile)
		Hints = {{"Drag a point", "changes the grade"}, {"Double click", "adds a point"}, {"LMB on another track", "its whole line"}, {"Shift+LMB on more tracks", "the route between the outermost ones"}, {"Shift+drag a box", "the route over the tracks in it"}, {"Apply", "writes the heights into the tracks"}};
	else
		return false;
	return true;
}

bool editor_mode::turnout_key_hints(std::vector<key_hint> &Hints, TTrack const *Selected) const
{
	using enum editor_mode::switch_kind;
	switch (current_switch_kind())
	{
	case connection: Hints = {{"Press on a straight", "and drag onto the other track"}, {"Shift+wheel", "another kind"}, {"Ctrl+wheel", "R of the aligning curve"}}; break;
	case slip:
	case crossing:
		Hints = {{"Press+drag at a crossing", "of two straights"}, {"Press+drag on a straight", "with nothing crossing it: the slip with its crossing track, which goes through the quarter of the cursor"}};
		if (current_switch_kind() == slip)
			Hints.emplace_back("Press+drag on a curve", "the slip bent into it, in a side track only (11.4 pkt 4b)");
		break;
	default:
		Hints = {{"Press+drag on a track", "a switch on it: along its direction, sideways its side"}, {"Press+drag from a free end", "a switch going on from the end of a track"}, {"LMB on a track", "selects it"}};
		break;
	}
	if (Selected == nullptr)
		return false;
	Hints.push_back({"Gizmo", "move the selected switch"});
	Hints.push_back({"Del", "remove the switch, its main track stays"});
	return true;
}

bool editor_mode::window_key_hints(std::vector<key_hint> &Hints) const
{
	if (false == m_track_window_open)
		return false;
	switch (m_track_tab)
	{
	case track_tab::speed:
		Hints = {{"LMB on a row", "show the path"}};
		break;
	case track_tab::lineside:
		Hints = {{"LMB on a track", "the line the objects go along"}};
		break;
	case track_tab::parallel:
		Hints = {{"LMB on a track", "the new track goes beside it"}, {"Click on a side in the panel", "the side of the new track"}};
		break;
	case track_tab::srk:
		srk_key_hints(Hints);
		break;
	case track_tab::turntable:
		if (m_turntable.placing)
			Hints = {{"Drag", "turns the bridge"}, {"Release", "places the turntable"}, {"Esc", "cancel"}};
		else if (m_turntable.table != nullptr)
			Hints = {{"LMB around it", "leads a track out"}, {"LMB on a free end", "fits that track to the turntable"}};
		else
			Hints = {{"LMB", "places a new turntable"}};
		break;
	case track_tab::kilometrage:
		Hints = {{"LMB on a track", "a point of the km there"}, {"Mouse over a track", "its km by the cursor"}};
		break;
	case track_tab::joints:
		Hints = {{"LMB on a row", "show the place"}, {"Fix", "joins the ends, aligns them"}};
		break;
	case track_tab::infra:
		Hints = {{"LMB", "select a path"}};
		break;
	default:
		return false;
	}
	return true;
}

void editor_mode::srk_key_hints(std::vector<key_hint> &Hints) const
{
	if (m_signal.moving)
		Hints = {{"Drag along", "moves the signal along the track"}, {"Drag over the track", "to the other side"}, {"Release", "done"}, {"Esc", "cancel"}};
	else if (m_signal.placing)
		Hints = {{"Drag along", "the trains it faces run that way"}, {"Release", "done"}, {"Esc", "cancel"}};
	else if (false == m_rail_tools->srk.drawing())
	{
		if (signal_armed() != nullptr)
			Hints = {{"LMB by a track", "places the signal on that side"}, {"Press+drag along", "the trains it faces run that way"}, {"Esc", "stops placing"}};
		else
			Hints = {{"LMB on a switch", "opens its station"}, {"List in the panel", "the type of the signal"}, {"Drag a signal", "moves it along the track"}, {"LMB on a signal", "selects it"}};
		if (m_signal.selected != 0 && m_signal.selected == m_instance)
			Hints.push_back({"Del", "removes the selected signal with its event"});
	}
	else
		Hints = {{"LMB on a switch", "opens its station"}};
}

bool editor_mode::plan_key_hints(std::vector<key_hint> &Hints) const
{
	if (m_track_tab == track_tab::straights)
		straights_key_hints(Hints);
	else if (m_track_tab == track_tab::route && m_route.chain.tracks.empty())
		Hints = {{"LMB on a straight", "its ends and the middle to drag"}, {"LMB on a curve", "its vertex and radius to drag"}};
	else if (m_track_tab == track_tab::route)
		Hints = {{"Drag a vertex", "the curve follows"}, {"Drag the radius grip", "changes the radius"}, {"Drag in the curvature band", "R up and down, transitions sideways"}, {"Apply", "writes the design into the tracks"}, {"Esc", "release the vertex"}};
	else
		return false;
	return true;
}

void editor_mode::straights_key_hints(std::vector<key_hint> &Hints) const
{
	if (m_straights.tool == 1)
		Hints = {{"Press+drag on the straight", "breaks it there and turns the rest"}, {"Esc", "cancel"}};
	else if (m_straights.tool == 2)
		Hints = {{"LMB on the straight", "point 1 of 4: leave the axis"}, {"Esc", "cancel"}};
	else if (m_straights.dragging && m_straights.handle == 2)
	{
		auto const offer{straight_parallel_offer(m_straights.drag_line, m_straights.preview_start - m_straights.drag_line.start)};
		if (offer.snaps)
			Hints = {{"Release", format(STR_C("parallel at %.2f m"), offer.spacing)}, {"Esc", "cancel"}};
		else if (offer.nearby)
			Hints = {{"Near another straight", format(STR_C("snap at %.2f m  (%.1f°)"), offer.spacing, offer.angle)}, {"Esc", "cancel"}};
		else
			Hints = {{"Close to another straight", "snaps parallel at 4.00 / 4.50 / 4.75 m"}, {"Esc", "cancel"}};
	}
	else if (m_straights.current.tracks.empty())
		Hints = {{"LMB on a straight", "its ends and the middle to drag"}, {"LMB on a curve", "its vertex and radius to drag"}};
	else
		Hints = {{"Drag an end", "the curves at it follow"}, {"Drag the middle", "shifts sideways, snaps parallel"}, {"Ctrl+drag", "breaks the straight"}, {"Shift+click x4", "shifts a piece around an obstacle"}, {"Alt+click", "adds another straight to move together"}};
}

bool editor_mode::selection_key_hints(std::vector<key_hint> &Hints, TTrack const *Selected) const
{
	if (Selected == nullptr)
	{
		Hints = {{"LMB on a track", "selects it"}, {"Drag on the ground", "selects the tracks in a box"}, {"Shift+LMB", "adds a track to the selection, drag: a box"}, {"RMB on a track", "what can be done with it"}, {"L", "lay a new track"}};
		return false;
	}
	if (m_track_tab == track_tab::path && m_track_set.size() >= 2 && in_track_set(Selected))
	{
		Hints = {{"Inspector", "the properties of all of them at once"}, {"Gizmo", "moves them together"}, {"Shift+LMB", "adds or takes out a track"}, {"Ctrl+Shift+V", "sets what's ticked of the track taken"}, {"Esc", "deselects"}, {"Del", "deletes them"}};
		return true;
	}
	if (m_point_drag.moved)
	{
		Hints = {{"Drag", "moves the point, level"}, {"Over a free end", "the end joins it"}, {"Release", "done"}, {"Esc", "cancel"}};
		return false;
	}
	if (m_track_point.valid())
		Hints = {{"Drag a point", "moves it, an end onto the ground (Shift: keeps its height), dropped on a free end joins it"}, {"Gizmo", point_gizmo_action(Selected)}, {"Q W E", "gizmo: move, turn, lengthen"}, {"Esc", "releases the point"}};
	else
		Hints = {{"Drag a point", "moves it, an end dropped on a free end joins it"}, {"Gizmo", track_gizmo_action(Selected)}, {"Q W E", "gizmo: move, turn, lengthen"}, {"Shift+LMB", "adds a track to the selection, drag: a box"}, {"L", "new track, also from a free end"}, {"Esc", "deselects"}, {"Del", "deletes the track"}};
	return true;
}

char const *editor_mode::point_gizmo_action(TTrack const *Selected) const
{
	if (false == editor_track::is_end(m_track_point.kind) && m_gizmo_op == gizmo_operation::rotate)
		return "turns the control vector sideways or up, cant at its end, the joined track keeps the tangent";
	if (false == editor_track::is_end(m_track_point.kind) && m_gizmo_op == gizmo_operation::scale)
		return "length of the control vector";
	if (false == editor_track::is_end(m_track_point.kind))
		return "moves the control point, the joined track keeps the tangent";
	if (m_gizmo_op == gizmo_operation::rotate)
		return "rings about the point: direction, grade, cant; the joined tracks follow";
	if (m_gizmo_op == gizmo_operation::scale && Selected != nullptr && track_gizmo_operation(*Selected, true) == gizmo_operation::scale)
		return "lengthens the track at this end, the curve keeps its radius";
	return "moves the point along the axes";
}

char const *editor_mode::track_gizmo_action(TTrack const *Selected) const
{
	if (m_gizmo_op == gizmo_operation::rotate)
		return "rings: direction, grade, cant; the joined tracks follow";
	if (m_gizmo_op == gizmo_operation::scale && Selected != nullptr && track_gizmo_operation(*Selected, false) == gizmo_operation::scale)
		return "lengthens the track at its free end, the curve keeps its radius";
	return "moves the whole track";
}

// values of the change in progress, shown next to the cursor
std::string editor_mode::track_readout() const
{
	if (m_tape.active)
		return tape_readout();
	if (m_extend.active)
		return extend_readout();
	if (m_lay.active)
		return lay_readout();
	auto const &state{m_straights};
	if (state.tool_placed && state.tool_mouse)
		return straight_tool_readout();
	if (state.dragging)
		return straight_drag_readout();
	if (m_track_set_using && m_track_gizmo_drag.operation == gizmo_operation::rotate)
		return format(STR_C("turned %+.2f deg"), glm::degrees(m_track_gizmo_drag.angle));
	if (auto const *track{selected_track()}; m_track_gizmo_using && track != nullptr)
		return gizmo_readout(*track);
	return {};
}

std::string editor_mode::extend_readout() const
{
	auto const pieces{extend_preview()};
	auto const [length, smallest]{length_and_radius(pieces)};
	auto const radius{m_eased_last.has_value() && false == m_eased_last->pieces.empty() ? m_eased_last->radius : smallest};
	editor_track::snap_target target;
	auto const snapped{extend_snap(target) && target.track != nullptr};
	editor_track::branch_link link;
	std::string reason;
	if (plan_branch_link(link, reason))
		return link_summary(target, link, editor_track::branch_radius(m_extend.track->m_paths[1]));
	if (snapped && pieces.empty())
		return format(STR_C("no track fits to the end of %s"), name_of(*target.track).c_str());
	auto text{radius > 0.0 ? format("L %.2f m   R %.0f m", length, radius) + speed_of(radius, laying_norms()) : format("L %.2f m   %s", length, STR_C("straight"))};
	if (auto const eased{eased_of(m_eased_last)}; false == eased.empty())
		text += "\n" + eased;
	text += "\n" + design_speed_of(m_extend.track, snapped ? target.track : nullptr);
	if (auto const vertical{vertical_of(pieces)}; false == vertical.empty())
		text += "\n" + vertical;
	if (snapped)
		text += "\n" + format(STR_C("joins %s"), name_of(*target.track).c_str());
	for (auto const &warning : switch_exit_warnings(m_extend.track, m_extend.point, snapped ? target.track : nullptr, target.position, pieces))
		text += "\n" + warning;
	if (false == reason.empty())
		text += "\n" + reason;
	return text + "\n" + track_shaping(snapped);
}

std::string editor_mode::lay_readout() const
{
	auto const &lay{m_lay};
	auto text{lay_shape_text()};
	for (auto const &warning : switch_exit_warnings(lay.start.track, lay.points.empty() ? lay.mouse : lay.points.front(), lay.mouse_snap.track, lay.mouse_snap.position, lay.preview))
		text += "\n" + warning;
	if (false == lay.points.empty() && lay.mouse_snap.track != nullptr)
		text += "\n" + format(STR_C("joins %s"), name_of(*lay.mouse_snap.track).c_str());
	if (false == lay.preview_error.empty())
		text += "\n" + first_line_of(lay.preview_error);
	if (lay.points.size() == 1 && lay.mouse_snap.track != nullptr)
		text += "\n" + track_shaping(lay.start.track != nullptr);
	// the bends are laid by default, the bar at the bottom tells of Tab: only the curve traced is named
	if (lay.mode == lay_mode::curve)
		text += "\n" + std::string{STR_C("Curve traced, Tab: the bends of straights")};
	if (ImGui::GetIO().KeyAlt && false == lay.points.empty())
		text += "\n" + (lay.skew.empty() ? format(STR_C("azimuth %.0f deg"), azimuth_of(lay.points.back(), lay.mouse)) : lay.skew);
	// the warning of the first Esc leads, where it's read first
	if (ImGui::GetTime() - lay.escape_time < 3.0)
		text = lay.status + "\n" + text;
	else if (false == lay.status.empty() && ImGui::GetTime() - lay.status_time < 4.0)
		text += "\n" + first_line_of(lay.status);
	return text;
}

std::string editor_mode::lay_shape_text() const
{
	auto const &lay{m_lay};
	if (lay.points.empty())
		return lay.mouse_snap.track != nullptr ? std::string{STR_C("goes on from this end of the track")} : std::string{STR_C("start")};
	if (false == lay.preview.empty() && lay.plan.shape.valid)
		return lay_plan_text();
	if (false == lay.preview.empty())
		return lay_pieces_text();
	return {};
}

std::string editor_mode::lay_plan_text() const
{
	// the curve the cursor makes now: of the last bend, or the one traced
	auto const &lay{m_lay};
	auto const &plan{lay.plan};
	auto text{format("L %.1f m", lay.preview_length)};
	if (false == plan.shape.curves.empty())
	{
		auto const &curve{plan.shape.curves.back()};
		text += "   " + curve_label(curve, laying_norms());
		if (false == alignment::meets_norms(curve, plan.design))
			text += "\n" + STR("this curve doesn't meet the norms, it's laid all the same");
		// the curves next to the free ends going on from the curves of their tracks: the first and the last one
		if (auto const on{curve_goes_on(plan.shape.curves.front())}; false == on.empty())
			text += "\n" + on;
		if (auto const on{curve_goes_on(curve)}; plan.shape.curves.size() > 1 && false == on.empty())
			text += "\n" + on;
	}
	if (lay.mode == lay_mode::curve && false == plan.traces.empty() && false == plan.traces.back().offsets.empty())
	{
		auto const &trace{plan.traces.back()};
		text += "\n" + format(STR_C("%zu points, departure %.0f cm, at most %.0f cm"), trace.points.size(), trace.rms * 100.0, trace.worst * 100.0);
	}
	auto const [from, to]{laying_ends()};
	text += "\n" + design_speed_of(from, to);
	if (auto const vertical{vertical_of(lay.preview)}; false == vertical.empty())
		text += "\n" + vertical;
	return text;
}

std::string editor_mode::lay_pieces_text() const
{
	auto const &lay{m_lay};
	double radius{0.0};
	bool straight{false};
	for (auto const &piece : lay.preview)
	{
		auto const r{piece_plan_radius(piece)};
		if (r < 0.5)
			straight = true;
		else if (radius == 0.0 || r < radius)
			radius = r;
	}
	if (m_eased_last.has_value() && false == m_eased_last->pieces.empty())
		radius = m_eased_last->radius;
	std::string text;
	if (radius > 0.0)
		text = (straight ? format("L %.1f m   %s + R %.0f m", lay.preview_length, STR_C("straight"), radius) : format("L %.1f m   R %.0f m", lay.preview_length, radius)) + speed_of(radius, laying_norms());
	else
		text = format("L %.1f m   %s", lay.preview_length, STR_C("straight"));
	if (auto const eased{eased_of(m_eased_last)}; false == eased.empty())
		text += "\n" + eased;
	auto const [from, to]{laying_ends()};
	text += "\n" + design_speed_of(from, to);
	if (auto const vertical{vertical_of(lay.preview)}; false == vertical.empty())
		text += "\n" + vertical;
	return text;
}

std::string editor_mode::straight_tool_readout() const
{
	auto const &state{m_straights};
	auto const &line{state.tool_line};
	auto const offset{plan_of(state.tool_handle - state.tool_point)};
	if (state.tool == 1)
	{
		if (glm::length(offset) < 1e-3)
			return {};
		auto const turn{glm::degrees(signed_angle(line.direction, glm::normalize(offset)))};
		return format(STR_C("turn %+.3f deg   R %.0f m"), turn, straight_tool_radius(line));
	}
	glm::dvec2 const normal{-line.direction.y, line.direction.x};
	return format(STR_C("shift %+.3f m   over %.2f m"), glm::dot(offset, normal), glm::dot(offset, line.direction));
}

std::string editor_mode::straight_drag_readout() const
{
	auto const &state{m_straights};
	if (state.fitting)
	{
		auto const &result{m_route.result};
		if (false == result.valid)
			return result.errors.empty() ? std::string{STR_C("no curve fits")} : result.errors.front();
		auto text{format(STR_C("curve L %.2f m"), result.length)};
		for (auto const &curve : result.curves)
			text += format(STR_C("   deflection %.3f deg"), glm::degrees(curve.deflection));
		return text;
	}
	auto const &grabbed{state.drag_line};
	if (state.handle == 2)
	{
		glm::dvec2 const normal{-grabbed.direction.y, grabbed.direction.x};
		auto const offset{state.preview_start - grabbed.start};
		auto text{format(STR_C("shift %+.3f m"), glm::dot(plan_of(offset), normal))};
		auto const offer{straight_parallel_offer(grabbed, offset)};
		if (offer.snaps)
			text += "\n" + format(STR_C("parallel  %.2f m"), offer.spacing);
		else if (offer.nearby)
			text += "\n" + format(STR_C("close: %.2f m  %.1f°"), offer.spacing, offer.angle);
		return text;
	}
	return format(STR_C("L %.3f m (%+.3f)   azimuth %.4f deg"), plan_distance(state.preview_start, state.preview_end), plan_distance(state.preview_start, state.preview_end) - grabbed.length,
	              azimuth_of(state.preview_start, state.preview_end));
}

std::string editor_mode::gizmo_readout(TTrack const &Track) const
{
	auto const &drag{m_track_gizmo_drag};
	if (drag.on_control)
		return control_readout(Track);
	if (drag.operation == gizmo_operation::scale && drag.path >= 0)
	{
		auto text{format("L %.3f m (%+.3f)", drag.stretch, drag.stretch - drag.length)};
		if (drag.path < static_cast<int>(drag.paths.size()) && std::abs(drag.paths[drag.path].radius) > 0.5f)
			text += format("   R %.1f m", std::abs(drag.paths[drag.path].radius));
		return text;
	}
	if (drag.operation == gizmo_operation::rotate && drag.ring == 0)
		return format(STR_C("grade changed by %+.1f per mille"), 1000.0 * drag.tilt.grade);
	if (drag.operation == gizmo_operation::rotate && drag.ring == 2)
		return format(STR_C("cant %.0f mm (%+.0f mm)"), 1500.0 * std::sin(glm::radians(drag.tilt.roll0 + drag.tilt.roll)), 1500.0 * (std::sin(glm::radians(drag.tilt.roll0 + drag.tilt.roll)) - std::sin(glm::radians(drag.tilt.roll0))));
	if (drag.operation == gizmo_operation::rotate)
		return format(STR_C("turned %+.2f deg"), glm::degrees(drag.angle));
	if (m_track_point.valid() && m_track_point.path < static_cast<int>(Track.m_paths.size()))
	{
		auto const &path{Track.m_paths[m_track_point.path]};
		auto text{format("L %.3f m", bezier{path}.plan_length())};
		if (path.radius != 0.0f)
			text += format("   R %.1f m", std::abs(path.radius));
		if (m_track_snap.track != nullptr)
			text += "\n" + format(STR_C("joins %s"), name_of(*m_track_snap.track).c_str());
		return text;
	}
	return format(STR_C("moved %.3f m"), glm::distance(editor_track::pivot(Track), m_track_pivot));
}

std::string editor_mode::control_readout(TTrack const &Track) const
{
	auto const &drag{m_track_gizmo_drag};
	auto const control_point{editor_track::point_position(Track, m_track_point) - editor_track::point_position(Track, editor_track::end_of(m_track_point))};
	auto text{format(STR_C("control %.2f m   azimuth %.2f deg"), glm::length(control_point), azimuth_of(glm::dvec3{0.0}, control_point))};
	if (drag.operation == gizmo_operation::rotate && drag.ring == 1)
		text += format(STR_C("   turned %+.2f deg"), glm::degrees(drag.angle));
	if (drag.operation == gizmo_operation::rotate && drag.ring == 0)
		text += format(STR_C("   grade %+.1f per mille"), 1000.0 * grade_of(control_point) * (glm::dot(level(control_point), drag.tilt.basis[2]) >= 0.0 ? 1.0 : -1.0));
	if (drag.operation == gizmo_operation::rotate && drag.ring == 2)
		text += "   " + format(STR_C("cant %.0f mm (%+.0f mm)"), 1500.0 * std::sin(glm::radians(drag.tilt.roll0 + drag.tilt.roll)), 1500.0 * (std::sin(glm::radians(drag.tilt.roll0 + drag.tilt.roll)) - std::sin(glm::radians(drag.tilt.roll0))));
	if (drag.fixed != glm::dvec3{})
		text += "\n" + STR("along the switch");
	return text;
}

std::string editor_mode::track_shaping(bool const Joined) const
{
	if (false == Joined)
	{
		static constexpr std::array<char const *, 4> names{"Arc, then straight", "Straight, then arc", "Arc through the cursor", "Straight"};
		auto text{std::string{STR_C(names[static_cast<std::size_t>(m_track_layout)])}};
		if (m_track_layout == track_layout::arc_straight || m_track_layout == track_layout::straight_arc)
			text += format("   R %.0f m", m_lay.radius);
		// the shape follows the mode of laying, Tab switching it where the laying goes on
		return text + "\n" + (m_track_layout == track_layout::straight ? STR("Tab while laying: an arc, then a straight") : STR("Ctrl+wheel: radius, Tab while laying: a straight")) +
		       STR(", Ctrl+Shift+wheel: speed");
	}
	auto const &join{m_join_last};
	static constexpr std::array<char const *, 6> kinds{"", "straight", "arc", "curve", "reverse curve", "free curve"};
	std::string text{STR_C(kinds[static_cast<std::size_t>(join.kind)])};
	if (join.radius > 0.0)
		text += format("   R %.0f m", join.radius) + (m_join_radius > 0.0 ? std::string{} : std::string{" "} + STR("(auto)"));
	if (join.cant > 0.0)
		text += format(STR_C("   cant %.0f mm"), join.cant);
	if (join.transition > 0.0)
		text += format(STR_C("   transitions %.0f m"), join.transition);
	if (join.speed > 0.0 && (join.cant > 0.0 || join.transition > 0.0))
		text += format("   V %.0f km/h", join.speed);
	if (join.variants > 1)
		text += format("   %d/%d", (m_join_variant % join.variants + join.variants) % join.variants + 1, join.variants);
	return text + "\n" + STR("Ctrl+wheel: radius, Shift+wheel: another shape") + STR(", Ctrl+Shift+wheel: speed");
}

void editor_mode::draw_empty_scenery_hint() const
{
	if (m_terrain_open || (m_track_tab != track_tab::path && m_track_tab != track_tab::lay) || (m_lay.active && false == m_lay.points.empty()) || m_first_steps_open || ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel))
		return;
	auto const &paths{simulation::Paths.sequence()};
	if (std::ranges::any_of(paths, [](TTrack const *Track) { return Track != nullptr && false == Track->m_editorremoved; }))
		return;
	auto *list{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	auto const font{ImGui::GetFontSize()};
	auto const pad{std::round(font * 1.0f)};
	auto const picture{std::round(ImGui::GetTextLineHeight() * 1.3f)};
	auto const title{STR_C("The scenery has no tracks yet")};
	std::vector<std::pair<char const *, char const *>> rows;
	if (m_track_tab == track_tab::path)
		rows.emplace_back("L", "the lay tool");
	rows.emplace_back("LMB", "on the ground: the first point, then the bends of the line, the curves at them made by themselves");
	rows.emplace_back("Enter", "lays the track clicked");
	rows.emplace_back("F1", "all the shortcuts, and the tutorial");
	auto const width{std::min(high.x - low.x - 2.0f * pad, std::round(font * 30.0f))};
	auto const column{std::round(picture * 5.0f)};
	auto const wrap{width - 2.0f * pad - column};
	float height{pad + font * 2.4f + pad * 0.5f};
	for (auto const &[keys, text] : rows)
		height += std::max(picture, ImGui::CalcTextSize(STR_C(text), nullptr, false, wrap).y) + pad * 0.45f;
	height += pad * 0.6f;
	ImVec2 const top{std::round((low.x + high.x - width) * 0.5f), std::round((low.y + high.y - height) * 0.5f)};
	ImVec2 const bottom{top.x + width, top.y + height};
	auto const accent{family_colour(0)};
	list->AddRectFilled(top, bottom, IM_COL32(16, 20, 26, 215), std::round(font * 0.6f));
	list->AddRect(top, bottom, (accent & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 160), std::round(font * 0.6f), 0, 1.5f);
	auto const icon{std::round(font * 2.4f)};
	editor_icons::draw(list, editor_icons::icon::lay_track, ImVec2(top.x + pad + icon * 0.5f, top.y + pad + icon * 0.5f), accent);
	auto const big{font * 1.25f};
	list->AddText(nullptr, big, ImVec2(top.x + pad + icon + pad * 0.6f, std::round(top.y + pad + (icon - big) * 0.5f)), ImGui::GetColorU32(ImGuiCol_Text), title);
	auto y{top.y + pad + icon + pad * 0.5f};
	for (auto const &[keys, label] : rows)
	{
		auto const text{STR_C(label)};
		auto const size{ImGui::CalcTextSize(text, nullptr, false, wrap)};
		auto const line{std::max(picture, size.y)};
		gesture_draw(*list, ImVec2(top.x + pad, std::round(y + (line - picture) * 0.5f)), picture, gesture_picture(keys), ImGui::GetColorU32(ImGuiCol_Text));
		list->AddText(nullptr, 0.0f, ImVec2(top.x + pad + column, std::round(y + (line - size.y) * 0.5f)), ImGui::GetColorU32(ImGuiCol_Text, 0.85f), text, nullptr, wrap);
		y += line + pad * 0.45f;
	}
}

void editor_mode::draw_track_hints()
{
	ImGuiIO const &io = ImGui::GetIO();
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	float const margin{12.0f};
	float const gap{14.0f};
	float const pad{4.0f};
	auto const chip = [&](ImVec2 const At, char const *Text, ImU32 const Background, ImU32 const Foreground) {
		auto const size{ImGui::CalcTextSize(Text)};
		drawlist->AddRectFilled(ImVec2(At.x, At.y - 2.0f), ImVec2(At.x + size.x + 2.0f * pad, At.y + size.y + 2.0f), Background, 3.0f);
		drawlist->AddText(ImVec2(At.x + pad, At.y), Foreground, Text);
		return size.x + 2.0f * pad;
	};
	auto hints{track_key_hints(false)};
	auto const picture{std::round(ImGui::GetTextLineHeight() * 1.15f)};
	std::vector<std::vector<gesture_word>> pictures;
	for (auto const &hint : hints)
		pictures.push_back(gesture_picture(hint.raw != nullptr ? hint.raw : hint.key));

	auto const mode{track_mode_name()};
	auto const left{editor_ui::view_min().x + margin};
	auto const available{editor_ui::view_max().x - editor_ui::view_min().x - 2.0f * margin};
	float width{ImGui::CalcTextSize(mode.c_str()).x + 2.0f * pad};
	std::size_t shown{0};
	for (; shown < std::min<std::size_t>(hints.size(), 3); ++shown)
	{
		auto const next{width + gap + hint_width(hints[shown], pictures[shown], picture, pad)};
		if (next > available)
			break;
		width = next;
	}
	auto const height{ImGui::GetTextLineHeight()};
	auto const bottom{m_profile.strip_top > editor_ui::view_min().y + 4.0f * height ? std::min(editor_ui::view_max().y, m_profile.strip_top) : editor_ui::view_max().y};
	auto const y{bottom - margin - height};
	ImVec2 const from{left - 6.0f, y - pad - 2.0f};
	ImVec2 const to{left + width + 6.0f, y + height + pad};
	auto const hovered{io.MousePos.x > from.x - 40.0f && io.MousePos.x < to.x + 40.0f && io.MousePos.y > from.y - 40.0f && io.MousePos.y < to.y + 40.0f};
	auto const fade = [hovered](ImU32 const Colour) { return hovered ? (Colour & ~IM_COL32_A_MASK) | (((Colour >> IM_COL32_A_SHIFT) & 0xFF) / 4 << IM_COL32_A_SHIFT) : Colour; };
	drawlist->AddRectFilled(from, to, fade(IM_COL32(0, 0, 0, 160)), 4.0f);
	auto x{left};
	auto const &modes{track_modes()};
	auto const tab{card_tab(m_track_window_open, m_track_tab)};
	auto const found{std::ranges::find_if(modes, [&](track_mode const &Mode) { return Mode.tab == tab; })};
	auto const family{(family_colour(found != modes.end() ? found->family : 0) & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 235)};
	x += chip(ImVec2(x, y), mode.c_str(), fade(family), fade(IM_COL32(255, 255, 255, 255)));
	for (std::size_t i = 0; i < shown; ++i)
	{
		x += gap;
		if (false == pictures[i].empty())
			x += gesture_draw(*drawlist, ImVec2(x, std::round(y + (height - picture) * 0.5f)), picture, pictures[i], fade(IM_COL32(255, 255, 255, 230))) + 5.0f;
		if (hints[i].raw == nullptr || false == gesture_keys_only(hints[i].raw))
			x += chip(ImVec2(x, y), hints[i].key, fade(IM_COL32(255, 210, 60, 220)), fade(IM_COL32(20, 20, 20, 255))) + 5.0f;
		drawlist->AddText(ImVec2(x, y), fade(IM_COL32(255, 255, 255, 230)), hints[i].action.c_str());
		x += ImGui::CalcTextSize(hints[i].action.c_str()).x;
	}

	draw_track_readout();
}

void editor_mode::draw_track_readout() const
{
	ImGuiIO const &io = ImGui::GetIO();
	auto readout{track_readout()};
	if (auto const *meaning{typed_meaning()}; meaning != nullptr && false == m_typed.empty())
		readout += (readout.empty() ? "" : "\n") + std::string{"= "} + m_typed + "_   " + STR(meaning);
	if (readout.empty() || ImGui::GetIO().WantCaptureMouse)
		return;
	auto *foreground{ImGui::GetForegroundDrawList(ImGui::GetMainViewport())};
	auto const size{ImGui::CalcTextSize(readout.c_str())};
	// right below the cursor, or on its other side where the view ends: it doesn't cover the panels by the view
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	ImVec2 at{io.MousePos.x + 20.0f, io.MousePos.y + 20.0f};
	if (at.x + size.x + 10.0f > high.x)
		at.x = io.MousePos.x - 20.0f - size.x;
	if (at.y + size.y + 10.0f > high.y)
		at.y = io.MousePos.y - 20.0f - size.y;
	at.x = std::max(at.x, low.x + 10.0f);
	at.y = std::max(at.y, low.y + 10.0f);
	foreground->AddRectFilled(ImVec2(at.x - 6.0f, at.y - 4.0f), ImVec2(at.x + size.x + 6.0f, at.y + size.y + 4.0f), IM_COL32(0, 0, 0, 190), 4.0f);
	foreground->AddRect(ImVec2(at.x - 6.0f, at.y - 4.0f), ImVec2(at.x + size.x + 6.0f, at.y + size.y + 4.0f), overlay_color::marked, 4.0f);
	foreground->AddText(at, IM_COL32(255, 255, 255, 255), readout.c_str());
}

void editor_mode::draw_blade_tips() const
{
	segment_data main;
	double tip{0.0};
	if (m_switch.placing && m_switch.connection < 0)
	{
		auto const preview{switch_preview()};
		if (preview.size() < 2)
			return;
		main = preview[0];
		tip = editor_track::blade_tip_of(preview[1]);
	}
	else if (auto const *selected{selected_track()}; selected != nullptr && selected->eType == tt_Switch && selected->m_paths.size() >= 2 && false == selected->m_editorremoved)
	{
		main = selected->m_paths[0];
		tip = editor_track::blade_tip_of(*selected);
	}
	else
		return;
	// on the photo the tips of the blades are seen where the switch seems to begin, the joint before them isn't
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	geometry::bezier const curve{main};
	auto const mark = [&](double const Distance, ImU32 const Colour, float const Width, char const *Label, int const Side) {
		auto const t{curve.parameter(Distance)};
		auto const at{curve.point(t)};
		auto direction{plan_of(curve.first(t))};
		if (glm::length(direction) < 1e-9)
			return;
		direction = glm::normalize(direction);
		glm::dvec3 const across{-direction.y * 1.6, 0.0, direction.x * 1.6};
		projection.line(drawlist, at - across, at + across, IM_COL32(0, 0, 0, 170), Width + 3.0f);
		projection.line(drawlist, at - across, at + across, Colour, Width);
		ImVec2 screen;
		if (projection.project(at + across * static_cast<double>(Side), screen))
		{
			auto const size{ImGui::CalcTextSize(Label)};
			ImVec2 const corner{Side > 0 ? screen.x + 6.0f : screen.x - 6.0f - size.x, screen.y - 0.5f * size.y};
			drawlist->AddRectFilled({corner.x - 3.0f, corner.y - 1.0f}, {corner.x + size.x + 3.0f, corner.y + size.y + 1.0f}, IM_COL32(0, 0, 0, 160), 3.0f);
			drawlist->AddText(corner, Colour, Label);
		}
	};
	mark(0.0, IM_COL32(220, 220, 220, 230), 2.0f, STR_C("joint before the blades"), -1);
	if (tip > 0.05)
		mark(tip, IM_COL32(255, 170, 40, 255), 3.0f, STR_C("tips of the blades"), 1);
}

void editor_mode::draw_switch_problem(switch_problem const &Problem) const
{
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	// blinking twice a second, never out quite
	auto const pulse{static_cast<float>(0.5 + 0.5 * std::cos(ImGui::GetTime() * 4.0 * glm::pi<double>()))};
	auto const colour{(overlay_color::invalid & ~IM_COL32_A_MASK) | (static_cast<ImU32>(70.0f + 185.0f * pulse) << IM_COL32_A_SHIFT)};
	for (auto const &path : Problem.paths)
		for (auto const &piece : path.pieces.empty() ? std::vector<segment_data>{path} : path.pieces)
		{
			bezier const curve{piece};
			auto previous{curve.point(0.0)};
			for (int i = 1; i <= 24; ++i)
			{
				auto const next{curve.point(i / 24.0)};
				projection.line(drawlist, previous, next, IM_COL32(0, 0, 0, 150), 10.0f);
				projection.line(drawlist, previous, next, colour, 6.0f);
				previous = next;
			}
		}
	auto const label = [&](ImVec2 const &At, std::string const &Text) {
		auto const size{ImGui::CalcTextSize(Text.c_str(), nullptr, false, 460.0f)};
		drawlist->AddRectFilled({At.x - 5.0f, At.y - 3.0f}, {At.x + size.x + 5.0f, At.y + size.y + 3.0f}, IM_COL32(0, 0, 0, 200), 3.0f);
		drawlist->AddText(ImGui::GetFont(), ImGui::GetFontSize(), At, overlay_color::invalid, Text.c_str(), nullptr, 460.0f);
	};
	for (auto const &[point, text] : Problem.marks)
	{
		ImVec2 screen;
		if (false == projection.project(point, screen))
			continue;
		auto const radius{10.0f + 8.0f * pulse};
		drawlist->AddCircle(screen, radius, IM_COL32(0, 0, 0, 150), 24, 6.0f);
		drawlist->AddCircle(screen, radius, colour, 24, 3.0f);
		if (false == text.empty())
			label({screen.x + 26.0f, screen.y - 0.5f * ImGui::GetFontSize()}, text);
	}
	// why, by the cursor
	if (auto const &io{ImGui::GetIO()}; false == Problem.text.empty() && ImGui::IsMousePosValid())
		label({io.MousePos.x + 20.0f, io.MousePos.y + 26.0f}, Problem.text);
}

void editor_mode::draw_build_overlay() const
{
	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
	// over a dark edge: the light colours of the tracks being built stay seen over the light base map of the top view too
	auto const drawpath = [&](segment_data const &Path, ImU32 const Color) {
		for (auto const &[colour, width] : {std::pair{IM_COL32(0, 0, 0, 170), 6.0f}, std::pair{Color, 3.0f}})
			for (auto const &piece : Path.pieces.empty() ? std::vector<segment_data>{Path} : Path.pieces)
			{
				bezier const curve{piece};
				auto previous{curve.point(0.0)};
				for (int i = 1; i <= 24; ++i)
				{
					auto const next{curve.point(i / 24.0)};
					projection.line(drawlist, previous, next, colour, width);
					previous = next;
				}
			}
	};
	// the paths of a track or of a switch, all of them in the Color
	auto const drawpaths = [&drawpath](auto const &Paths, ImU32 const Color) {
		for (auto const &path : Paths)
			drawpath(path, Color);
	};
	// two tracks crossing, from the Ends taken in pairs
	auto const drawcrossing = [&drawpath](auto const &Ends, ImU32 const Color) {
		for (int i = 0; i < 4; i += 2)
		{
			segment_data path;
			path.points[segment_data::point::start] = Ends[i];
			path.points[segment_data::point::end] = Ends[i + 1];
			drawpath(path, Color);
		}
	};
	// a slip is selected whole: all its tracks with the one picked
	if (auto const *selected{selected_track()}; selected != nullptr && false == m_switch.slide.active)
		if (auto const slip{slip_of(*selected)})
			for (auto const *part : slip->parts)
				drawpaths(part->m_paths, overlay_color::highlight);
	// none while a point is dragged: there is nothing to split then
	if (m_track_tab == track_tab::path && false == m_lay.active && false == m_track_gizmo_using && false == m_point_drag.active && false == m_handle_drag.active)
		if (auto const spot{split_spot()}; spot.track != nullptr)
		{
			ImVec2 at;
			if (projection.project(spot.point, at))
			{
				drawlist->AddCircle(at, 6.0f, overlay_color::highlight, 12, 1.5f);
				drawlist->AddText({at.x + 9.0f, at.y - 18.0f}, overlay_color::highlight, STR_C("K: split here"));
			}
		}
	auto const &display{ImGui::GetIO().DisplaySize};
	// in the 3D view only near the camera, where they can be told apart; from above all of them
	glm::dvec3 const eye{Global.pCamera.Pos};
	auto const in_reach = [&](glm::dvec3 const &Point) { return Global.EditorOrtho || glm::distance(Point, eye) <= kDotsRange; };
	// far out the joints of the pieces would cover the tracks, only the free ends are told
	auto const joints{false == Global.EditorOrtho || display.y <= 0.0f || 2.0 * static_cast<double>(Global.EditorOrthoExtent) / display.y <= 1.0};
	// the ends of a path as dots, those on the screen
	auto const drawjoints = [&in_reach, &projection, drawlist, &display](segment_data const &Path) {
		for (auto const point : {segment_data::point::start, segment_data::point::end})
		{
			ImVec2 screen;
			if (in_reach(Path.points[point]) && projection.project(Path.points[point], screen) && screen.x >= 0.0f && screen.y >= 0.0f && screen.x <= display.x && screen.y <= display.y)
				drawlist->AddCircleFilled(screen, 3.5f, kJointColor);
		}
	};
	for (auto const *track : simulation::Paths.sequence())
	{
		if (false == joints)
			break;
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
			continue;
		for (auto const &path : track->m_paths)
			drawjoints(path);
	}
	// the ends joined to nothing, where a train would run off
	for (auto const &point : free_end_points(eye, Global.EditorOrtho ? std::max(kDotsRange, static_cast<double>(Global.EditorOrthoExtent) * 2.0) : kDotsRange))
	{
		ImVec2 screen;
		if (in_reach(point) && projection.project(point, screen) && screen.x >= 0.0f && screen.y >= 0.0f && screen.x <= display.x && screen.y <= display.y)
		{
			drawlist->AddCircleFilled(screen, 5.0f, kFreeEndColor);
			drawlist->AddCircle(screen, 5.0f, IM_COL32(0, 0, 0, 160), 12, 1.0f);
		}
	}
	if (m_extend.active)
	{
		if (m_extend.track != nullptr)
			draw_free_ends(m_extend.track, m_extend.track->iCategoryFlag & 15, {m_extend.track});
		crossover_plan plan;
		editor_track::snap_target target;
		if (extend_snap(target))
		{
			ImVec2 screen;
			if (projection.project(target.position, screen))
				drawlist->AddCircle(screen, 14.0f, IM_COL32(255, 60, 255, 255), 20, 3.0f);
			for (auto const &piece : extend_preview())
				drawpath(piece, overlay_color::marked);
		}
		else if (plan_crossover(plan))
		{
			for (auto const &piece : plan.paths)
				drawpath(piece, overlay_color::marked);
			for (auto const &piece : plan.inserts)
				drawpath(piece, overlay_color::grip);
		}
		else
		{
			for (auto const &piece : extend_preview())
				drawpath(piece, overlay_color::marked);
		}
	}
	// what letting go there makes, by the cursor
	auto const connection_label = [&](std::string const &Text, ImU32 const Colour) {
		ImVec2 screen;
		if (false == projection.project(m_switch.mouse, screen))
			return;
		auto const size{ImGui::CalcTextSize(Text.c_str())};
		drawlist->AddRectFilled(ImVec2(screen.x + 14.0f, screen.y + 10.0f), ImVec2(screen.x + 22.0f + size.x, screen.y + 14.0f + size.y), IM_COL32(0, 0, 0, 180), 3.0f);
		drawlist->AddText(ImVec2(screen.x + 18.0f, screen.y + 12.0f), Colour, Text.c_str());
	};
	if (m_switch.placing && m_switch.connection >= 0 && m_switch.curved)
		// along a curve it's worked out as the switch is let go
		connection_label(connection_name(m_switch.connection), overlay_color::highlight);
	if (m_switch.placing && m_switch.connection >= 0 && false == m_switch.curved)
	{
		connection::request request;
		connection::result plan;
		// over a crossover there already, the double or the scissors one made with it
		auto const valid{m_switch.partner != nullptr ? plan_partner_connection(request, plan) : plan_connection(request, plan)};
		if (valid)
		{
			for (auto const &entry : plan.switches)
				drawpaths(entry.paths, overlay_color::marked);
			for (auto const &insert : plan.inserts)
				drawpath(insert, overlay_color::grip);
			if (plan.crossing)
				drawcrossing(plan.ends, overlay_color::grip);
		}
		else
			for (auto const &piece : switch_preview())
				drawpath(piece, overlay_color::invalid);
		std::string text;
		ImU32 colour{overlay_color::invalid};
		if (valid)
		{
			text = connection_readout(plan);
			colour = plan.warning.empty() ? overlay_color::highlight : overlay_color::warning;
		}
		else if (plan.error.empty())
			text = STR_C("Drag onto the other track");
		else
			text = plan.error;
		connection_label(text, colour);
	}
	else if (auto const *selected{selected_track()}; false == m_switch.placing && m_track_tab == track_tab::turnout && m_switch.connection < 0 && m_switch.armed >= 0 && m_switch.armed < static_cast<int>(m_switch.templates.size()) &&
	                                                 m_switch.templates[m_switch.armed].double_slip && selected != nullptr && selected == m_straights.current_for && false == m_straights.current.tracks.empty() &&
	                                                 false == ImGui::GetIO().WantCaptureMouse)
	{
		// where the slip goes for a click there, before the click
		auto const &line{m_straights.current};
		auto const cursor{cursor_on_straight(line)};
		auto const along{std::clamp(line.along(cursor), 0.0, line.length)};
		if (plan_distance(line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * along, cursor) <= kSwitchReach)
		{
			slip_plan plan;
			auto const valid{plan_double_slip(line, cursor, m_switch.templates[m_switch.armed], plan)};
			std::string text{plan.problem};
			glm::dvec3 marker{cursor};
			ImU32 colour{overlay_color::invalid};
			// with nothing crossing here the slip is dragged out with its crossing track
			if (false == valid && plan.lone)
			{
				colour = overlay_color::highlight;
				text = STR_C("Press and drag: the slip with its crossing track, through the quarter of the cursor");
			}
			else if (valid && plan.replaces != nullptr)
			{
				marker = plan.replaces->location();
				colour = overlay_color::highlight;
				text = STR_C("Press and drag: this double slip is replaced by the one chosen");
			}
			else if (valid)
			{
				drawcrossing(plan.points, overlay_color::marked);
				marker = {plan.crossing.x, line.start.y + line.grade * plan.alongfirst, plan.crossing.y};
				colour = overlay_color::highlight;
				text = plan.note.empty() ? std::string{STR_C("Press and drag: it goes here")} : plan.note;
			}
			ImVec2 screen;
			if (projection.project(marker, screen))
			{
				drawlist->AddCircle(screen, 12.0f, colour, 20, 2.5f);
				drawlist->AddText({screen.x + 16.0f, screen.y - 8.0f}, colour, text.c_str());
			}
		}
	}
	else if (m_switch.placing)
	{
		auto const preview{switch_preview()};
		for (auto const &piece : preview)
			drawpath(piece, overlay_color::marked);
		// moved back onto the straight or into the curve, or too long for it: what's in the way blinks
		if (auto const problem{switch_drag_problem()})
			draw_switch_problem(*problem);
		// laid on from a free end: bent, the radius of the main track and the way of the diverging one, the track into the free end the
		// cursor is at, the norms; from an end of a switch, the change of the curvature from it
		if (m_switch.free && preview.size() >= 2)
		{
			// few lines: the radii, the track into the free end, the warnings; what the keys do the bar at the bottom tells
			std::vector<std::pair<std::string, ImU32>> lines;
			if (free_bend bend; free_switch_bend(bend))
				lines = bend_readout(projection, drawlist, bend);
			for (auto const &warning : free_switch_exits(preview))
				lines.emplace_back(warning, overlay_color::invalid);
			if (false == lines.empty())
				draw_readout_box(projection, drawlist, m_switch.point, lines);
		}
		// into a curve: the ends of the cant ramps, what the start was drawn to, the radii at the ends of the switch and the norms
		if (auto const &tool{m_switch}; tool.curved && tool.connection < 0 && preview.size() >= 2)
		{
			auto const drawramp = [&projection, drawlist](curve_sample const &Sample) {
				glm::dvec3 const across{-Sample.tangent.y * 2.5, 0.0, Sample.tangent.x * 2.5};
				projection.line(drawlist, Sample.position - across, Sample.position + across, IM_COL32(255, 170, 40, 255), 3.0f);
				ImVec2 screen;
				if (projection.project(Sample.position + across, screen))
					drawlist->AddText({screen.x + 4.0f, screen.y - 7.0f}, IM_COL32(255, 170, 40, 255), STR_C("ramp"));
			};
			auto const points{frame_points(tool.frame)};
			for (auto const end : switch_curve::ramp_ends(points))
				drawramp(sample_at(tool.frame, end));
			double station{0.0};
			auto const lines{curved_switch_lines(preview, station)};
			draw_readout_box(projection, drawlist, sample_at(tool.frame, station).position, lines);
		}
		switch_link plan;
		auto const linked{plan_switch_link(plan)};
		ImVec2 screen;
		if (plan.target.track != nullptr && projection.project(plan.target.position, screen))
		{
			drawlist->AddCircle(screen, 14.0f, linked ? IM_COL32(255, 60, 255, 255) : overlay_color::invalid, 20, 3.0f);
			if (false == linked && false == plan.reason.empty())
				drawlist->AddText({screen.x + 18.0f, screen.y - 8.0f}, overlay_color::invalid, plan.reason.c_str());
		}
	}
	// a switch refused or laid elsewhere than dragged to: where and why, a while after it's let go
	if (false == m_switch.placing && ImGui::GetTime() < m_switch_flash_until)
		draw_switch_problem(m_switch_flash);
	if (m_lay.active)
	{
		auto const &lay{m_lay};
		auto const &plan{lay.plan};
		auto const piece_colour = [&](segment_data const &Path) {
			if (false == lay.preview_error.empty() && plan.shape.warnings.empty())
				return overlay_color::warning;
			return piece_plan_radius(Path) < 0.5 ? IM_COL32(255, 255, 255, 230) : overlay_color::marked;
		};
		for (auto const &piece : lay.preview)
			draw_overlay_rails(projection, drawlist, piece, piece_colour(piece));
		auto const label_at = [&](glm::dvec3 const &Point, std::string const &Text, ImU32 const Colour) {
			ImVec2 screen;
			if (false == projection.project(Point, screen))
				return;
			auto const size{ImGui::CalcTextSize(Text.c_str())};
			drawlist->AddRectFilled(ImVec2(screen.x + 6.0f, screen.y - 8.0f), ImVec2(screen.x + 14.0f + size.x, screen.y + 8.0f), IM_COL32(0, 0, 0, 180), 3.0f);
			drawlist->AddText(ImVec2(screen.x + 10.0f, screen.y - 8.0f), Colour, Text.c_str());
		};
		// the straights the next bend slides along, dashed
		for (auto const &[from, to] : plan.guides)
		{
			auto const length{plan_distance(from, to)};
			auto const dashes{std::clamp(static_cast<int>(length / 4.0), 1, 400)};
			for (int k = 0; k < dashes; k += 2)
				projection.line(drawlist, glm::mix(from, to, static_cast<double>(k) / dashes), glm::mix(from, to, static_cast<double>(k + 1) / dashes), IM_COL32(255, 255, 255, 150), 1.5f);
		}
		// the curves of the line laid by the points: the bends eased, the curves traced, in the colour of a warning where they don't meet
		// the norms; the length of the straights between them
		auto const label_design = [&label_at, &plan, this]() {
			for (auto const &curve : plan.shape.curves)
				if (curve.vertex >= 0 && curve.vertex < static_cast<int>(plan.shape.vertex_chainages.size()))
					label_at(alignment::evaluate(plan.shape, plan.shape.vertex_chainages[curve.vertex]).position, curve_label(curve, laying_norms()),
					         alignment::meets_norms(curve, plan.design) ? overlay_color::marked : overlay_color::warning);
			for (auto const &e : plan.shape.elements)
				if (e.kind == alignment::element_kind::straight && e.length >= 20.0)
					label_at(alignment::evaluate(plan.shape, e.chainage + e.length * 0.5).position, format(STR_C("straight %.0f m"), e.length), overlay_color::highlight);
		};
		if (plan.shape.valid)
			label_design();
		for (std::size_t i = 0, n = plan.shape.valid ? 0 : lay.preview.size(); i < n;)
		{
			auto const radius{piece_plan_radius(lay.preview[i])};
			bool const curved{radius >= 0.5};
			std::size_t j{i + 1};
			double length{bezier{lay.preview[i]}.plan_length()};
			double sharpest{radius};
			for (; j < n; ++j)
			{
				auto const next{piece_plan_radius(lay.preview[j])};
				if ((next >= 0.5) != curved)
					break;
				length += bezier{lay.preview[j]}.plan_length();
				if (curved && next < sharpest)
					sharpest = next;
			}
			double along{0.0};
			glm::dvec3 middle{bezier{lay.preview[i]}.point(0.5)};
			for (std::size_t k = i; k < j; ++k)
			{
				auto const piece{bezier{lay.preview[k]}.plan_length()};
				if (along + piece >= length * 0.5)
				{
					middle = bezier{lay.preview[k]}.point(piece > 1e-6 ? std::clamp((length * 0.5 - along) / piece, 0.0, 1.0) : 0.5);
					break;
				}
				along += piece;
			}
			ImVec2 screen;
			if (projection.project(middle, screen))
			{
				auto const label{curved ? format("R %.0f", sharpest) : std::string{STR_C("straight")}};
				auto const size{ImGui::CalcTextSize(label.c_str())};
				drawlist->AddRectFilled(ImVec2(screen.x + 6.0f, screen.y - 8.0f), ImVec2(screen.x + 14.0f + size.x, screen.y + 8.0f), IM_COL32(0, 0, 0, 180), 3.0f);
				drawlist->AddText(ImVec2(screen.x + 10.0f, screen.y - 8.0f), curved ? overlay_color::marked : overlay_color::highlight, label.c_str());
			}
			i = j;
		}
		for (auto const &[colour, width] : {std::pair{IM_COL32(0, 0, 0, 120), 3.0f}, std::pair{IM_COL32(255, 255, 255, 160), 1.0f}})
			for (std::size_t i = 0; i + 1 < lay.points.size(); ++i)
				projection.line(drawlist, lay.points[i], lay.points[i + 1], colour, width);
		// the bends of straights as squares, the points along a curve as dots in the colour of how far the line laid is from them
		auto const departure = [&](glm::dvec3 const &Point) {
			for (auto const &trace : plan.traces)
				for (std::size_t k = 0; k < trace.points.size() && k < trace.offsets.size(); ++k)
					if (glm::distance(trace.points[k], Point) < 1e-6)
						return std::abs(trace.offsets[k]);
			return -1.0;
		};
		for (std::size_t i = 0; i < lay.points.size(); ++i)
		{
			ImVec2 screen;
			if (false == projection.project(lay.points[i], screen))
				continue;
			// the start of the line as wide as the track
			if (i == 0)
			{
				draw_track_width(projection, drawlist, lay.points[i], overlay_color::grip);
				continue;
			}
			if (i < lay.modes.size() && lay.modes[i] == lay_mode::curve)
			{
				auto const off{departure(lay.points[i])};
				auto const colour{lay_point_colour(off)};
				drawlist->AddCircleFilled(screen, 4.5f, colour);
				drawlist->AddCircle(screen, 4.5f, IM_COL32(0, 0, 0, 170), 12, 1.5f);
				continue;
			}
			drawlist->AddRectFilled(ImVec2(screen.x - 4.5f, screen.y - 4.5f), ImVec2(screen.x + 4.5f, screen.y + 4.5f), overlay_color::grip);
			drawlist->AddRect(ImVec2(screen.x - 4.5f, screen.y - 4.5f), ImVec2(screen.x + 4.5f, screen.y + 4.5f), IM_COL32(0, 0, 0, 170), 0.0f, 0, 1.5f);
		}
		// the point the cursor would add along a curve
		if (ImVec2 cursor; lay.mode == lay_mode::curve && false == lay.points.empty() && lay.mouse_snap.track == nullptr && projection.project(lay.mouse, cursor))
		{
			auto const off{departure(lay.mouse)};
			drawlist->AddCircle(cursor, 6.0f, lay_point_colour(off), 16, 2.0f);
		}
		ImVec2 screen;
		if (lay.mouse_snap.track != nullptr && projection.project(lay.mouse_snap.position, screen))
			drawlist->AddCircle(screen, 14.0f, IM_COL32(255, 60, 255, 255), 20, 3.0f);
		if (lay.start.track != nullptr && projection.project(lay.start.position, screen))
			drawlist->AddCircle(screen, 10.0f, IM_COL32(255, 60, 255, 255), 20, 2.0f);
		if (lay.points.empty() && lay.mouse_snap.track == nullptr)
			draw_cursor_ring(lay.mouse);
		if (false == m_extend.active)
			draw_free_ends(nullptr, kRailCategory, {});
	}
	if (false == m_straights.detour.empty())
	{
		auto const outline{detour_outline()};
		for (std::size_t i = 0; i + 1 < outline.size(); ++i)
			projection.line(drawlist, outline[i], outline[i + 1], overlay_color::marked, 3.0f);
		for (std::size_t i = 0; i < outline.size() && i < m_straights.detour.size(); ++i)
		{
			ImVec2 screen;
			if (projection.project(outline[i], screen))
				drawlist->AddCircleFilled(screen, 6.0f, overlay_color::invalid);
		}
	}
	if ((m_point_drag.moved || (m_track_gizmo_using && m_track_point.valid())) && editor_track::is_end(m_track_point.kind) && m_track_drag_points.empty())
		if (auto const *track{selected_track()}; track != nullptr)
			draw_free_ends(track, track->iCategoryFlag & 15, std::vector<TTrack const *>(m_track_drag.begin(), m_track_drag.end()));
}

std::vector<std::pair<std::string, ImU32>> editor_mode::curved_switch_lines(std::vector<segment_data> const &Preview, double &Station) const
{
	auto const &tool{m_switch};
	int forward{0};
	int side{0};
	curved_switch_pose(forward, side);
	std::string snapped;
	std::vector<std::string> notes;
	std::vector<std::string> warnings;
	std::vector<std::pair<std::string, ImU32>> lines;
	if (auto const &shape{tool.templates[tool.armed]}; shape.double_slip)
	{
		// a slip bent: the radii of its tracks and its slip curves, its middle where the station is
		curved_slip plan;
		plan_curved_slip(shape, forward, side, plan);
		snapped = plan.snapped;
		Station = plan.middle;
		auto const report{curved_slip_report(plan, shape.type)};
		for (auto const &line : slip_report_lines(report))
			lines.emplace_back(line, IM_COL32(255, 255, 255, 235));
		notes = report.notes;
		warnings = report.warnings;
	}
	else
	{
		Station = curved_switch_start(forward, &snapped);
		auto const report{curved_switch_report(Preview, m_switch.frame, Station, forward)};
		if (report.curved)
		{
			lines.emplace_back(format(STR_C("main track R %s at the start, %s at the end"), radius_text(report.main_start).c_str(), radius_text(report.main_end).c_str()), IM_COL32(255, 255, 255, 235));
			lines.emplace_back(format(STR_C("diverging track R %s at the start, %s at the end"), radius_text(report.branch_start).c_str(), radius_text(report.branch_end).c_str()), IM_COL32(255, 255, 255, 235));
		}
		lines.emplace_back(format(STR_C("cant %.0f mm at the start, %.0f mm at the end"), report.cant_start, report.cant_end), IM_COL32(255, 255, 255, 235));
		notes = report.notes;
		warnings = report.warnings;
	}
	if (false == snapped.empty())
		lines.emplace(lines.begin(), snapped, IM_COL32(255, 170, 40, 255));
	for (auto const &note : notes)
		lines.emplace_back(note, IM_COL32(170, 200, 230, 235));
	for (auto const &warning : warnings)
		lines.emplace_back(warning, overlay_color::invalid);
	return lines;
}

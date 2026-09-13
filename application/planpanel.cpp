module;
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <numbers>
#include <string>
#include <vector>
#include "imgui/imgui.h"
#include "utilities/Globals_macros.h"
#include "maj0sted/editor/layout.hpp"
#include "maj0sted/editor/model.hpp"
#include "maj0sted/editor/render.hpp"
#include "maj0sted/editor/solution.hpp"
#include "maj0sted/editor/tile_cache.hpp"
#include "maj0sted/io/scn_export.hpp"
#include "maj0sted/editor/join.hpp"
#include "maj0sted/editor/sketch.hpp"
#include "maj0sted/io/document_io.hpp"
#if !defined(_WIN32)
#include <sys/wait.h>
#include <unistd.h>
#endif

module eu07.application.planpanel;
import eu07.glm;
import eu07.utilities.globals;
import eu07.utilities.utilities;
import eu07.rendering.renderer;
import eu07.application.editormode;
import eu07.editor.polandmap;
import eu07.editor.turnouts;

/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

namespace
{

using maj0sted::editor::Element;
using maj0sted::editor::ElementId;
using maj0sted::editor::Free;
using maj0sted::editor::Kind;
using maj0sted::editor::Parallel;
using maj0sted::editor::TrackId;
using maj0sted::editor::TurnoutId;

// the plan works in EPSG:2180, the scenery in the engine's world space around its own zero. the
// georeference says which point of the projection that zero stands for; a fictional scenery leaves
// it at the origin, so the two frames differ only in the naming of the axes - easting runs along
// world x, northing against world z
glm::dvec3 plan_to_world(double const X, double const Y)
{
	return {X - Global.scenery_origin.x, 0.0, -(Y - Global.scenery_origin.y)};
}

// where this very program stands on disk. the editor and the simulator are one exe, so running the
// exported scenery is running ourselves again with the scenery named on the command line
std::string executable_path()
{
#if defined(_WIN32)
	char buffer[MAX_PATH]{};
	auto const length{GetModuleFileNameA(nullptr, buffer, MAX_PATH)};
	return length > 0 ? std::string(buffer, length) : std::string{};
#else
	char buffer[4096]{};
	auto const length{::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1)};
	return length > 0 ? std::string(buffer, static_cast<std::size_t>(length)) : std::string{};
#endif
}

// starts it and lets go: the editor keeps whatever is on its screen, and the second instance is not
// ours to wait for. on unix that is two forks, so nothing is left behind for us to reap
bool spawn_detached(std::string const &Executable, std::vector<std::string> const &Arguments, std::string const &Directory)
{
	if (Executable.empty())
	{
		return false;
	}
#if defined(_WIN32)
	std::string command{'"' + Executable + '"'};
	for (auto const &argument : Arguments)
	{
		command += " \"" + argument + '"';
	}
	STARTUPINFOA startup{};
	startup.cb = sizeof(startup);
	PROCESS_INFORMATION process{};
	if (0 == CreateProcessA(nullptr, command.data(), nullptr, nullptr, FALSE, 0, nullptr, Directory.empty() ? nullptr : Directory.c_str(), &startup, &process))
	{
		return false;
	}
	CloseHandle(process.hThread);
	CloseHandle(process.hProcess);
	return true;
#else
	// laid out before the fork: between forking and exec'ing, a process with threads in it must not
	// go anywhere near the allocator
	std::vector<char *> argv;
	argv.push_back(const_cast<char *>(Executable.c_str()));
	for (auto const &argument : Arguments)
	{
		argv.push_back(const_cast<char *>(argument.c_str()));
	}
	argv.push_back(nullptr);

	auto const first{::fork()};
	if (first < 0)
	{
		return false;
	}
	if (first == 0)
	{
		if (::fork() == 0)
		{
			::setsid();
			if (false == Directory.empty())
			{
				if (::chdir(Directory.c_str()) != 0)
				{
					::_exit(127);
				}
			}
			::execv(Executable.c_str(), argv.data());
		}
		::_exit(0);
	}
	int status{0};
	::waitpid(first, &status, 0);
	return true;
#endif
}

// the construction points of a turnout, by TurnoutMark::kind. the map and the template drawing name
// them from this one list
char const *const mark_names[]{"PR",  "ostrze",           "pieta iglicy", "styk",             "koniec luku",
                               "KR",  "punkt teoretyczny", "dziob u",      "koniec strugania", "koniec rozjazdu"};

// an angle the way a turnout drawing gives it. the kat przylegania is a couple of dozen minutes, so
// degrees alone say nothing
std::string angle_text(double const Radians)
{
	auto const total{std::abs(Radians) * 180.0 / std::numbers::pi};
	auto const degrees{static_cast<int>(total)};
	auto const minutes{(total - degrees) * 60.0};
	char buffer[64];
	std::snprintf(buffer, sizeof(buffer), "%s%d st %02d' %05.2f\"", Radians < 0.0 ? "-" : "", degrees, static_cast<int>(minutes),
	              (minutes - static_cast<int>(minutes)) * 60.0);
	return buffer;
}

// What a bend works out to, said the way a catalogue says it: both radii and the beta the tor
// zasadniczy turns through. Jednostronny is both tracks bent the same way, dwustronny the opposite.
std::string bend_text(double const Bend_, double const Diverging_, double const Angle_)
{
	if (Bend_ == 0.0)
	{
		return "rozjazd zwyczajny: tor zasadniczy prosty";
	}
	auto const way{[](double const Curvature_) { return Curvature_ > 0.0 ? "w lewo" : "w prawo"; }};
	char text[256];
	if (Diverging_ == 0.0)
	{
		std::snprintf(text, sizeof(text), "rozjazd lukowy dwustronny: zasadniczy R %.3f m %s, zwrotny prosty, beta %s", std::abs(1.0 / Bend_), way(Bend_),
		              angle_text(Angle_).c_str());
	}
	else
	{
		std::snprintf(text, sizeof(text), "rozjazd lukowy %s: zasadniczy R %.3f m %s, zwrotny R %.3f m %s, beta %s",
		              Diverging_ * Bend_ > 0.0 ? "jednostronny" : "dwustronny", std::abs(1.0 / Bend_), way(Bend_), std::abs(1.0 / Diverging_),
		              way(Diverging_), angle_text(Angle_).c_str());
	}
	return text;
}

// The two limits Koc puts on bending an ordinary turnout. Reported, never enforced: the drawing is
// the user's, and a warning that says the number is worth more than a control that refuses it
char const *bend_limit(double const Diverging_)
{
	if (Diverging_ == 0.0)
	{
		return nullptr;
	}
	auto const radius{std::abs(1.0 / Diverging_)};
	if (radius < 190.0)
	{
		return "tor zwrotny ponizej 190 m - najmniejszego promienia rozjazdu";
	}
	if (radius < 214.0)
	{
		return "tor zwrotny ponizej 214 m - potrzebne poszerzenie toru zwrotnego";
	}
	return nullptr;
}

// a dashed polyline in screen space, for the lines a drawing means as theoretical. the dash carries
// across the joints between segments so a curve built of several pieces reads as one line
void dashed_polyline(ImDrawList *Drawlist_, std::vector<ImVec2> const &Points_, ImU32 const Colour_, float const Thickness_, float const Dash_ = 7.0f,
                     float const Gap_ = 5.0f)
{
	// the view magnifies to millimetres, so a line can run for millions of pixels: what falls outside
	// the clip rect is stepped over rather than drawn, or the draw list runs out of 16-bit indices
	auto const low{Drawlist_->GetClipRectMin()};
	auto const high{Drawlist_->GetClipRectMax()};
	auto const period{Dash_ + Gap_};
	auto carried{0.0f};
	auto budget{400};
	for (std::size_t i = 1; i < Points_.size(); ++i)
	{
		auto const from{Points_[i - 1]};
		auto const dx{Points_[i].x - from.x};
		auto const dy{Points_[i].y - from.y};
		auto const len{std::sqrt(dx * dx + dy * dy)};
		if (len < 1e-4f)
		{
			continue;
		}

		// the stretch of this piece that the clip rect actually shows, as a distance along it
		auto first{0.0f};
		auto last{len};
		auto visible{true};
		float const edge[]{-dx, dx, -dy, dy};
		float const room[]{from.x - low.x + 8.0f, high.x - from.x + 8.0f, from.y - low.y + 8.0f, high.y - from.y + 8.0f};
		for (int side = 0; side < 4 && visible; ++side)
		{
			if (std::abs(edge[side]) < 1e-6f)
			{
				visible = room[side] >= 0.0f;
			}
			else if (auto const cut{room[side] / edge[side] * len}; edge[side] < 0.0f)
			{
				first = std::max(first, cut);
			}
			else
			{
				last = std::min(last, cut);
			}
		}
		if (false == visible || first >= last)
		{
			carried = std::fmod(carried + len, period);
			continue;
		}

		auto at{std::max(0.0f, first - std::fmod(carried + first, period))};
		while (at < last && budget > 0)
		{
			--budget;
			auto const phase{std::fmod(carried + at, period)};
			if (phase < Dash_)
			{
				auto const to{std::min(at + Dash_ - phase, len)};
				Drawlist_->AddLine(ImVec2(from.x + dx * at / len, from.y + dy * at / len), ImVec2(from.x + dx * to / len, from.y + dy * to / len), Colour_,
				                   Thickness_);
				at = to;
			}
			else
			{
				at += period - phase;
			}
		}
		carried = std::fmod(carried + len, period);
	}
}

editor::turnout_preset const *find_preset(std::string const &Name_)
{
	for (auto const &preset : editor::turnout_presets())
	{
		if (preset.name == Name_)
		{
			return &preset;
		}
	}
	return nullptr;
}

// the catalogue figure written onto a document's own type. a document keeps its own copy of the
// catalogue - that is what lets an instance name its type and nothing more - so this is the one
// place the preset's numbers cross over, and it has to carry all of them
void adopt_preset(editor::turnout_preset const &Preset_, maj0sted::editor::TurnoutType &Type_)
{
	Type_.name = Preset_.name;
	Type_.crossing_n = Preset_.crossing_n;
	Type_.length = Preset_.length;
	Type_.blade.tip_thickness = Preset_.tip_thickness;
	Type_.blade.nose = Preset_.nose;
	Type_.blade.railtop_width = Preset_.railtop_width;
	Type_.pieces.clear();
	for (auto const &piece : Preset_.pieces)
	{
		Type_.pieces.push_back(
		    maj0sted::editor::TurnoutPieceSpec{piece.part, piece.length, piece.radius_start, piece.radius_end, piece.turn_in});
	}
}

void world_to_plan(glm::dvec3 const &World, double &X, double &Y)
{
	X = World.x + Global.scenery_origin.x;
	Y = -World.z + Global.scenery_origin.y;
}

// places a world point on screen using the camera of the most recent colour pass. the engine renders
// camera-relative, so the view matrix carries rotation only and the position is subtracted here
bool world_to_screen(glm::dvec3 const &World, ImVec2 &Screen)
{
	auto const relative{glm::vec3(World - GfxRenderer->Camera_Position())};
	auto const clip{GfxRenderer->Camera_Projection_Matrix() * GfxRenderer->Camera_View_Matrix() * glm::vec4(relative, 1.0f)};

	if (clip.w <= 0.0f)
	{
		return false; // behind the camera
	}

	auto const ndc{glm::vec3(clip) / clip.w};
	auto const display{ImGui::GetIO().DisplaySize};
	Screen = {(ndc.x * 0.5f + 0.5f) * display.x, (0.5f - ndc.y * 0.5f) * display.y};

	return true;
}

// ground under the cursor in the top-down plan view. depth picking is useless here: empty scenery
// has nothing to hit, so the last (or zero) offset would pin every click to the same spot
glm::dvec3 ortho_cursor_world()
{
	auto const width = std::max(1, Global.window_size.x);
	auto const height = std::max(1, Global.window_size.y);
	auto const ndcx = (static_cast<double>(Global.cursor_pos.x) / width) * 2.0 - 1.0;
	auto const ndcy = 1.0 - (static_cast<double>(Global.cursor_pos.y) / height) * 2.0;
	auto const halfheight = static_cast<double>(Global.editor_ortho_extent);
	auto const halfwidth = halfheight * static_cast<double>(width) / static_cast<double>(height);
	// yaw is pinned to north: view x = world x, view y = -world z
	return {Global.pCamera.Pos.x + ndcx * halfwidth, 0.0, Global.pCamera.Pos.z - ndcy * halfheight};
}

ImU32 element_colour(Kind const Kind_, bool const Active)
{
	auto const alpha{Active ? 255 : 200};
	switch (Kind_)
	{
	case Kind::Arc:
		return IM_COL32(255, 190, 90, alpha);
	case Kind::Clothoid:
		return IM_COL32(140, 245, 150, alpha);
	default:
		return IM_COL32(125, 200, 255, alpha);
	}
}

char const *kind_name(Kind const Kind_)
{
	switch (Kind_)
	{
	case Kind::Arc:
		return "Luk";
	case Kind::Clothoid:
		return "Krzywa przejsciowa";
	default:
		return "Prosta";
	}
}

double distance_point_segment(ImVec2 const &Point, ImVec2 const &A, ImVec2 const &B)
{
	auto const dx{B.x - A.x};
	auto const dy{B.y - A.y};
	auto const len2{dx * dx + dy * dy};
	if (len2 <= 0.0f)
	{
		return std::hypot(Point.x - A.x, Point.y - A.y);
	}
	auto t{((Point.x - A.x) * dx + (Point.y - A.y) * dy) / len2};
	t = std::clamp(t, 0.0f, 1.0f);
	return std::hypot(Point.x - (A.x + dx * t), Point.y - (A.y + dy * t));
}

} // namespace

// ---------------------------------------------------------------------------
// lifecycle
// ---------------------------------------------------------------------------

plan_panel::plan_panel(std::string const &Name, bool const Isopen) : ui_panel(Name, Isopen)
{
	size_min = {440, 260};
	size_max = {900, 900};
	seed_catalogue();
	new_track();
	solve();
}

void plan_panel::update()
{
	// the plan belongs on the scenery itself, so opening the tool puts the viewport into the
	// distortion-free view from above and closing it hands the camera back
	Global.editor_ortho = is_open;
}

void plan_panel::render_contents()
{
	handle_scene();
	draw_on_scene();

	render_toolbar();
	render_elements();
	render_turnouts();
	render_diagnostics();
	render_storage();
	render_newmap_dialog();
	render_location_dialog();
	render_template_window();
}

void plan_panel::seed_catalogue()
{
	// the catalogue is data: an instance names its type, so a new figure goes in here and never
	// touches the file format for placements.
	//
	// a plan saved before the iglica became a piece of its own keeps a type of that very name with
	// no blade in it at all - two odcinki, luk i krzyzownicowa. that is not a figure anyone
	// authored, it is what the old writer could say, so it is brought up to the current catalogue
	// rather than left to lay a switch that has no ostrze. a type someone really did edit keeps its
	// iglica and is left alone
	auto refreshed{0};
	for (auto const &preset : editor::turnout_presets())
	{
		auto existing{std::find_if(m_document.turnout_types.begin(), m_document.turnout_types.end(),
		                           [&](auto const &Type_) { return Type_.name == preset.name; })};
		if (existing == m_document.turnout_types.end())
		{
			maj0sted::editor::TurnoutType type;
			adopt_preset(preset, type);
			m_document.turnout_types.push_back(std::move(type));
			continue;
		}
		auto const has_blade{std::any_of(existing->pieces.begin(), existing->pieces.end(), [](auto const &Piece_) { return Piece_.part == 1; })};
		if (false == has_blade)
		{
			adopt_preset(preset, *existing);
			++refreshed;
		}
	}
	if (refreshed > 0)
	{
		m_status = std::to_string(refreshed) + " typ(ow) rozjazdu bylo bez iglicy - wziete z katalogu na nowo";
	}
}

// ---------------------------------------------------------------------------
// document
// ---------------------------------------------------------------------------

void plan_panel::solve()
{
	m_solution = maj0sted::editor::solve(m_document);
	m_rails = maj0sted::editor::render_rails(m_solution);
}

plan_panel::Track *plan_panel::current_track()
{
	return maj0sted::editor::find_track(m_document, m_track);
}

plan_panel::Track const *plan_panel::current_track() const
{
	return maj0sted::editor::find_track(m_document, m_track);
}

void plan_panel::new_track()
{
	Track track;
	track.id = maj0sted::editor::mint_track(m_document);
	track.name = "tor " + std::to_string(m_document.tracks.size() + 1);
	m_document.tracks.push_back(track);
	m_track = track.id;
	std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", track.name.c_str());
	m_pending = false;
	clear_selection();
}

void plan_panel::delete_current_track()
{
	auto const it{std::find_if(m_document.tracks.begin(), m_document.tracks.end(), [&](Track const &Track_) { return Track_.id == m_track; })};
	if (it == m_document.tracks.end())
	{
		return;
	}
	// a turnout standing on it, and anything anchored to that turnout, would be left dangling
	auto const gone{it->id};
	m_document.tracks.erase(it);
	for (auto turnout{m_document.turnouts.begin()}; turnout != m_document.turnouts.end();)
	{
		turnout = (turnout->on == gone) ? m_document.turnouts.erase(turnout) : turnout + 1;
	}
	if (m_document.tracks.empty())
	{
		new_track();
	}
	else
	{
		m_track = m_document.tracks.front().id;
		std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", m_document.tracks.front().name.c_str());
	}
	m_pending = false;
	clear_selection();
	solve();
}

void plan_panel::clear_selection()
{
	m_sel_element = ElementId::none;
	m_sel_turnout = TurnoutId::none;
	m_dragging_turnout = false;
}

void plan_panel::append_point_to(double const X, double const Y)
{
	auto *track{current_track()};
	if (track == nullptr)
	{
		return;
	}

	// an empty track has no direction yet, so the first click only says where it starts
	if (track->elements.empty())
	{
		if (false == m_pending)
		{
			m_pending = true;
			m_startx = X;
			m_starty = Y;
			m_status = "kliknij drugi raz, zeby polozyc pierwsza prosta";
			return;
		}
		auto const dx{X - m_startx};
		auto const dy{Y - m_starty};
		auto const length{std::hypot(dx, dy)};
		if (length < 1.0)
		{
			m_status = "za krotka prosta";
			return;
		}
		track->anchor = maj0sted::editor::AtPose{m_startx, m_starty, std::atan2(dx, dy)};
		track->elements.push_back(maj0sted::editor::make_line(m_document, length));
		m_pending = false;
		solve();
		m_status.clear();
		return;
	}

	// the track ends on the last click, so that end is the corner this one turns at
	auto const *solved{maj0sted::editor::find_track(m_solution, track->id)};
	if (solved == nullptr || solved->elements.empty())
	{
		m_status = "tor nie jest polozony - popraw najpierw to, co nie wyszlo";
		return;
	}
	auto const corner_pose{solved->end};

	auto const dx{X - corner_pose.x};
	auto const dy{Y - corner_pose.y};
	auto const reach{std::hypot(dx, dy)};
	if (reach < 1.0)
	{
		m_status = "za krotki odcinek";
		return;
	}

	auto const corner{maj0sted::editor::fit_corner(corner_pose.hx, corner_pose.hy, dx / reach, dy / reach, m_corner_radius)};
	if (false == corner.ok)
	{
		// straight on: nothing to round. two straights end to end are one straight, so the one
		// already there simply gets longer
		track->elements.back().length += reach;
		solve();
		m_status = "prosto dalej: przedluzona ostatnia prosta do " + to_string(track->elements.back().length, 1) + " m";
		return;
	}

	// the arc eats R*tan(kat/2) off the leg behind the corner and off the one in front. neither may
	// go negative, which is the whole of "naroznik za ostry na tym promieniu"
	auto &previous{track->elements.back()};
	if (previous.kind != Kind::Line || previous.length <= corner.tangent + 0.5)
	{
		m_status = "naroznik za ostry na R=" + to_string(m_corner_radius, 0) + " m: potrzeba " + to_string(corner.tangent, 1) + " m stycznej, a przed nim jest " +
		           to_string(previous.kind == Kind::Line ? previous.length : 0.0, 1) + " m prostej";
		return;
	}
	if (reach <= corner.tangent + 0.5)
	{
		m_status = "klikniete za blisko naroznika: potrzeba " + to_string(corner.tangent, 1) + " m stycznej, a jest " + to_string(reach, 1) + " m";
		return;
	}

	previous.length -= corner.tangent;
	track->elements.push_back(maj0sted::editor::make_arc(m_document, m_corner_radius, corner.hand, corner.arc_length));
	track->elements.push_back(maj0sted::editor::make_line(m_document, reach - corner.tangent));
	solve();
	m_status.clear();
}

void plan_panel::drop_last_vertex()
{
	auto *track{current_track()};
	if (track == nullptr || track->elements.empty())
	{
		m_pending = false;
		return;
	}
	if (track->elements.back().kind != Kind::Line)
	{
		m_status = "ostatni odcinek nie jest prosta - zdejmij go w liscie";
		return;
	}

	track->elements.pop_back();
	// and the arc that rounded the corner in front of it, whose tangent goes back onto the straight
	// it was taken from. what is left ends on the click before this one
	if (false == track->elements.empty() && track->elements.back().kind == Kind::Arc)
	{
		auto const &arc{track->elements.back()};
		auto const tangent{arc.radius > 0.0 ? arc.radius * std::tan(0.5 * arc.length / arc.radius) : 0.0};
		track->elements.pop_back();
		if (false == track->elements.empty() && track->elements.back().kind == Kind::Line)
		{
			track->elements.back().length += tangent;
		}
	}

	clear_selection();
	solve();
}

void plan_panel::append_element(Kind const Kind_)
{
	auto *track{current_track()};
	if (track == nullptr)
	{
		return;
	}

	// two straights end to end are one straight - there would be no corner between them and nothing
	// to grab hold of, so the one already there just gets longer
	if (Kind_ == Kind::Line && false == track->elements.empty() && track->elements.back().kind == Kind::Line)
	{
		track->elements.back().length += m_new_length;
		m_sel_element = track->elements.back().id;
		solve();
		m_status = "prosta doklejona do poprzedniej: " + to_string(track->elements.back().length, 1) + " m";
		return;
	}

	Element element;
	element.id = maj0sted::editor::mint_element(m_document);
	element.kind = Kind_;
	element.radius = Kind_ == Kind::Line ? 0.0 : m_new_radius;
	element.hand = Kind_ == Kind::Line ? 0 : m_new_hand;
	element.length = m_new_length;

	// what the chain is already curving at decides the joint, not what happens to be typed in the
	// boxes: an arc after a transition curve is the arc that curve eases into, so it takes its
	// radius and its hand. a transition after an arc turns the same way the arc does
	std::string inherited;
	if (false == track->elements.empty())
	{
		auto const &previous{track->elements.back()};
		auto const curving{previous.kind != Kind::Line && previous.radius > 0.0};
		if (Kind_ == Kind::Arc && previous.kind == Kind::Clothoid && curving)
		{
			element.radius = previous.radius;
			element.hand = previous.hand;
			inherited = " (R i skret z krzywej przejsciowej)";
		}
		else if (Kind_ == Kind::Arc && previous.kind == Kind::Clothoid)
		{
			// the transition was running out into a straight; now it runs into this arc, so that
			// is what it eases to
			track->elements.back().radius = element.radius;
			track->elements.back().hand = element.hand;
			inherited = " (krzywa przejsciowa dostala jego R i skret)";
		}
		else if (Kind_ == Kind::Clothoid && previous.kind == Kind::Arc)
		{
			// a transition ends on whatever follows it, and for now nothing does: it runs out
			// into a straight. dokladajac za nia luk, przejmie jego promien
			element.radius = 0.0;
			element.hand = previous.hand;
			inherited = " (wybiega w prosta)";
		}
	}

	track->elements.push_back(element);
	m_sel_element = element.id;
	solve();
	m_status = std::string{"dolozono: "} + kind_name(Kind_) + inherited;
}

void plan_panel::drop_last_element()
{
	auto *track{current_track()};
	if (track == nullptr || track->elements.empty())
	{
		return;
	}
	track->elements.pop_back();
	clear_selection();
	solve();
}

void plan_panel::delete_selected_element()
{
	if (m_sel_element == ElementId::none)
	{
		return;
	}
	for (auto &track : m_document.tracks)
	{
		auto const it{std::find_if(track.elements.begin(), track.elements.end(), [&](Element const &Element_) { return Element_.id == m_sel_element; })};
		if (it != track.elements.end())
		{
			track.elements.erase(it);
			break;
		}
	}
	m_sel_element = ElementId::none;
	solve();
}

bool plan_panel::refit_current_track(std::string &Why)
{
	auto *track{current_track()};
	// the solution still holds the geometry from before the change, which is exactly what the
	// straights are supposed to go on standing on
	auto const *solved{maj0sted::editor::find_track(m_solution, m_track)};
	if (track == nullptr || solved == nullptr)
	{
		Why = "tor nie jest polozony";
		return false;
	}

	maj0sted::editor::Skeleton skeleton;
	if (false == maj0sted::editor::skeleton_of(*track, *solved, skeleton, Why))
	{
		return false;
	}
	return maj0sted::editor::lay_along_skeleton(*track, skeleton, Why);
}

void plan_panel::drag_straight_to(double const X, double const Y)
{
	auto *track{current_track()};
	auto const *solved{maj0sted::editor::find_track(m_solution, m_track)};
	if (track == nullptr || solved == nullptr || m_drag_index >= track->elements.size())
	{
		return;
	}

	maj0sted::editor::Skeleton skeleton;
	std::string why;
	if (false == maj0sted::editor::skeleton_of(*track, *solved, skeleton, why))
	{
		m_status = why;
		return;
	}

	// which of the skeleton's lines the grabbed straight is: the straights in order, which is not
	// every second element once corners hold baskets or transition curves
	std::size_t line_index{0};
	for (std::size_t i = 0; i < m_drag_index; ++i)
	{
		if (track->elements[i].kind == Kind::Line)
		{
			++line_index;
		}
	}
	if (line_index >= skeleton.lines.size())
	{
		return;
	}

	// the grabbed straight swings onto the line through its far end and the cursor. every other
	// line stays exactly as it was, which is what keeps their azimuths to the millimetre
	auto const dx{X - m_pivot_x};
	auto const dy{Y - m_pivot_y};
	auto const reach{std::hypot(dx, dy)};
	if (reach < 1.0)
	{
		return;
	}
	// grabbing the end it runs out to swings it forward from the far end; grabbing the end it
	// starts on swings it backward, so the direction of travel is the other way round
	auto const sign{m_drag_end == 1 ? 1.0 : -1.0};
	auto &line{skeleton.lines[line_index]};
	line.px = m_pivot_x;
	line.py = m_pivot_y;
	line.dx = sign * dx / reach;
	line.dy = sign * dy / reach;

	// the free ends of the skeleton stay where they are, except the one being dragged
	if (line_index == 0 && m_drag_end == 0)
	{
		skeleton.start_x = X;
		skeleton.start_y = Y;
	}
	if (line_index + 1 == skeleton.lines.size() && m_drag_end == 1)
	{
		skeleton.end_x = X;
		skeleton.end_y = Y;
	}

	// nothing is written until the whole chain works out, so a bad drag leaves the track alone
	Track candidate{*track};
	if (false == maj0sted::editor::lay_along_skeleton(candidate, skeleton, why))
	{
		m_status = why;
		return;
	}
	// the chain laying out is not enough: shortening a straight can pull it out from under a rozjazd
	// standing on it, and a rozjazd half on an arc is not a rozjazd. the drag stops at that instead
	auto const off_straight{[](maj0sted::editor::Solution const &Solution_) {
		std::size_t count{0};
		for (auto const &diagnostic : Solution_.diagnostics)
		{
			count += diagnostic.code == maj0sted::editor::Code::StationOffTrack ? 1 : 0;
		}
		return count;
	}};

	auto const was{off_straight(m_solution)};
	Track const before{*track};
	*track = std::move(candidate);
	solve();
	if (off_straight(m_solution) > was)
	{
		for (auto const &diagnostic : m_solution.diagnostics)
		{
			if (diagnostic.code == maj0sted::editor::Code::StationOffTrack)
			{
				m_status = "dalej nie: " + diagnostic.text;
				break;
			}
		}
		*track = before;
		solve();
		return;
	}
	m_status.clear();
}

void plan_panel::place_turnout_at(TrackId const On, double const Wx, double const Wy)
{
	double station{0.0};
	if (false == station_on(On, Wx, Wy, station))
	{
		m_status = "nie ma na czym postawic rozjazdu";
		return;
	}
	if (m_document.turnout_types.empty())
	{
		m_status = "katalog rozjazdow jest pusty";
		return;
	}
	auto const type{static_cast<std::size_t>(std::clamp(m_turnout_type, 0, static_cast<int>(m_document.turnout_types.size()) - 1))};

	maj0sted::editor::TurnoutPlacement placement;
	placement.id = maj0sted::editor::mint_turnout(m_document);
	placement.type = m_document.turnout_types[type].name;
	placement.on = On;
	placement.station = station;
	placement.hand = m_new_hand;
	m_document.turnouts.push_back(placement);

	// a switch is worth nothing without somewhere for the branch to go, so one starts at its frog
	Track branch;
	branch.id = maj0sted::editor::mint_track(m_document);
	branch.name = "odnoga " + std::to_string(m_document.tracks.size() + 1);
	branch.anchor = maj0sted::editor::AtPort{placement.id, maj0sted::editor::Port::Frog};
	m_document.tracks.push_back(branch);

	m_track = branch.id;
	std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", branch.name.c_str());
	m_sel_turnout = placement.id;
	m_pending = false;
	solve();
	m_status = "rozjazd " + placement.type + " na " + to_string(station, 1) + " m; odnoga zaczyna sie na krzyzownicy";
}

// ---------------------------------------------------------------------------
// hit tests
// ---------------------------------------------------------------------------

bool plan_panel::hit_straight_end(ImVec2 const &Mouse, std::size_t &OutIndex, int &OutEnd) const
{
	auto const *solved{maj0sted::editor::find_track(m_solution, m_track)};
	auto const *track{current_track()};
	if (solved == nullptr || track == nullptr)
	{
		return false;
	}

	constexpr float tolerance{10.0f};
	auto best{tolerance};
	auto found{false};
	for (std::size_t i = 0; i < solved->elements.size() && i < track->elements.size(); ++i)
	{
		if (track->elements[i].kind != Kind::Line)
		{
			continue;
		}
		// a branch starts on its turnout's frog and cannot be pulled off it
		auto const pinned{i == 0 && false == std::holds_alternative<maj0sted::editor::AtPose>(track->anchor)};
		for (int end = 0; end < 2; ++end)
		{
			if (end == 0 && pinned)
			{
				continue;
			}
			auto const &pose{end == 0 ? solved->elements[i].start : solved->elements[i].end};
			ImVec2 screen;
			if (false == world_to_screen(plan_to_world(pose.x, pose.y), screen))
			{
				continue;
			}
			auto const distance{static_cast<float>(std::hypot(Mouse.x - screen.x, Mouse.y - screen.y))};
			if (distance < best)
			{
				best = distance;
				OutIndex = i;
				OutEnd = end;
				found = true;
			}
		}
	}
	return found;
}

bool plan_panel::hit_element(ImVec2 const &Mouse, ElementId &OutElement, TrackId &OutTrack) const
{
	constexpr float tolerance{8.0f};
	auto best{tolerance};
	auto found{false};
	for (auto const &rail : m_rails)
	{
		if (rail.element == ElementId::none || rail.points.size() < 2)
		{
			continue;
		}
		for (std::size_t i = 1; i < rail.points.size(); ++i)
		{
			ImVec2 a;
			ImVec2 b;
			if (false == world_to_screen(plan_to_world(rail.points[i - 1].x, rail.points[i - 1].y), a) ||
			    false == world_to_screen(plan_to_world(rail.points[i].x, rail.points[i].y), b))
			{
				continue;
			}
			auto const distance{static_cast<float>(distance_point_segment(Mouse, a, b))};
			if (distance < best)
			{
				best = distance;
				OutElement = rail.element;
				OutTrack = rail.track;
				found = true;
			}
		}
	}
	return found;
}

bool plan_panel::hit_turnout(ImVec2 const &Mouse, TurnoutId &OutTurnout) const
{
	constexpr float tolerance{10.0f};
	auto best{tolerance};
	auto found{false};
	for (auto const &turnout : m_solution.turnouts)
	{
		if (false == turnout.valid)
		{
			continue;
		}
		for (auto const &pose : {turnout.start, turnout.frog})
		{
			ImVec2 screen;
			if (false == world_to_screen(plan_to_world(pose.x, pose.y), screen))
			{
				continue;
			}
			auto const distance{std::hypot(Mouse.x - screen.x, Mouse.y - screen.y)};
			if (distance < best)
			{
				best = distance;
				OutTurnout = turnout.id;
				found = true;
			}
		}
	}
	return found;
}

bool plan_panel::station_on(TrackId const Track_, double const Wx, double const Wy, double &OutStation) const
{
	auto const *track{maj0sted::editor::find_track(m_solution, Track_)};
	if (track == nullptr || track->centreline.size() < 2)
	{
		return false;
	}

	auto best{std::numeric_limits<double>::max()};
	double walked{0.0};
	for (std::size_t i = 1; i < track->centreline.size(); ++i)
	{
		auto const &a{track->centreline[i - 1]};
		auto const &b{track->centreline[i]};
		auto const dx{b.x - a.x};
		auto const dy{b.y - a.y};
		auto const len2{dx * dx + dy * dy};
		auto const step{std::sqrt(len2)};
		if (step <= 0.0)
		{
			continue;
		}
		auto t{((Wx - a.x) * dx + (Wy - a.y) * dy) / len2};
		t = std::clamp(t, 0.0, 1.0);
		auto const distance{std::hypot(Wx - (a.x + dx * t), Wy - (a.y + dy * t))};
		if (distance < best)
		{
			best = distance;
			OutStation = walked + step * t;
		}
		walked += step;
	}
	return best < std::numeric_limits<double>::max();
}

plan_panel::TrackId plan_panel::nearest_track_axis(ImVec2 const &Mouse, float const Tolerance) const
{
	auto best{Tolerance};
	auto found{TrackId::none};
	for (auto const &track : m_solution.tracks)
	{
		for (std::size_t i = 1; i < track.centreline.size(); ++i)
		{
			ImVec2 a;
			ImVec2 b;
			if (false == world_to_screen(plan_to_world(track.centreline[i - 1].x, track.centreline[i - 1].y), a) ||
			    false == world_to_screen(plan_to_world(track.centreline[i].x, track.centreline[i].y), b))
			{
				continue;
			}
			auto const distance{static_cast<float>(distance_point_segment(Mouse, a, b))};
			if (distance < best)
			{
				best = distance;
				found = track.id;
			}
		}
	}
	return found;
}

// ---------------------------------------------------------------------------
// the scene
// ---------------------------------------------------------------------------

void plan_panel::handle_scene()
{
	auto const &io{ImGui::GetIO()};
	if (io.WantCaptureMouse && false == m_dragging_turnout)
	{
		return;
	}

	auto const mouse{io.MousePos};
	auto const world{ortho_cursor_world()};
	double wx{0.0};
	double wy{0.0};
	world_to_plan(world, wx, wy);

	// Esc drops every transient errand back to plain selection
	// this ImGui indexes IsKeyPressed by the backend's own key codes; ImGuiKey_ names are slots in
	// io.KeyMap, so the mapping has to be gone through rather than passed straight in
	auto const &keymap{ImGui::GetIO().KeyMap};
	if (keymap[ImGuiKey_Escape] >= 0 && ImGui::IsKeyPressed(keymap[ImGuiKey_Escape]))
	{
		m_pick_turnout = 0;
		m_pick_parallel = 0;
		m_pick_join = 0;
		m_join_first = TrackId::none;
		m_pending = false;
		clear_selection();
		m_status.clear();
	}

	// --- transient errands ------------------------------------------------
	if (m_pick_turnout == 1 && ImGui::IsMouseClicked(0))
	{
		auto const on{nearest_track_axis(mouse, 14.0f)};
		if (on != TrackId::none)
		{
			place_turnout_at(on, wx, wy);
			m_pick_turnout = 0;
		}
		else
		{
			m_status = "kliknij tor, na ktorym ma stanac rozjazd";
		}
		return;
	}
	if (m_pick_parallel != 0 && ImGui::IsMouseClicked(0))
	{
		ElementId reference{ElementId::none};
		TrackId owner{TrackId::none};
		auto *track{current_track()};
		if (track != nullptr && hit_element(mouse, reference, owner) && owner != m_track)
		{
			auto const it{std::find_if(track->elements.begin(), track->elements.end(), [&](Element const &Element_) { return Element_.id == m_sel_element; })};
			if (it != track->elements.end())
			{
				it->hold = Parallel{reference, m_new_offset};
				m_status = "trzyma rownoleglosc, odstep " + to_string(m_new_offset, 2) + " m";
				solve();
			}
		}
		else
		{
			m_status = "wskaz odcinek innego toru";
		}
		m_pick_parallel = 0;
		return;
	}
	if (m_pick_join != 0 && ImGui::IsMouseClicked(0))
	{
		TrackId which{TrackId::none};
		int end{0};
		if (false == hit_track_end(mouse, 18.0f, which, end))
		{
			m_status = "kliknij w koniec toru - zapalone konce sa te, ktore da sie polaczyc";
			return;
		}
		if (m_pick_join == 1)
		{
			m_join_first = which;
			m_join_first_end = end;
			m_pick_join = 2;
			m_status = "teraz wskaz drugi koniec";
			return;
		}
		if (which == m_join_first)
		{
			m_status = "to ten sam tor";
			return;
		}
		join_ends(m_join_first, m_join_first_end, which, end);
		return;
	}

	// --- ctrl: grabbing a straight by one of its ends ---------------------
	// this is the one thing a modifier is needed for: the ends of a straight sit on the track, so
	// a plain click there has to stay a plain click
	if (io.KeyCtrl && ImGui::IsMouseClicked(0))
	{
		std::size_t index{0};
		int end{0};
		if (hit_straight_end(mouse, index, end))
		{
			auto const *solved{maj0sted::editor::find_track(m_solution, m_track)};
			auto const *track{current_track()};
			if (solved != nullptr && index < solved->elements.size())
			{
				// the far end is captured now: re-fitting the arc behind it will move it
				auto const &pivot{end == 1 ? solved->elements[index].start : solved->elements[index].end};
				m_pivot_x = pivot.x;
				m_pivot_y = pivot.y;
				m_drag_index = index;
				m_drag_end = end;
				m_dragging_straight = true;
				m_sel_element = track->elements[index].id;
				if (track != nullptr && std::holds_alternative<maj0sted::editor::Parallel>(track->elements[index].hold))
				{
					m_status = "ten odcinek trzyma rownoleglosc - po puszczeniu tor i tak zostanie przystawiony";
				}
			}
			return;
		}
	}
	if (m_dragging_straight)
	{
		if (ImGui::IsMouseDown(0))
		{
			drag_straight_to(wx, wy);
		}
		else
		{
			m_dragging_straight = false;
		}
		return;
	}

	// --- one click, whatever is under it ----------------------------------
	// clicking something that is already drawn picks it up; clicking bare ground carries the
	// current track on to there. there is nothing to switch between: what you clicked says what
	// you meant
	if (ImGui::IsMouseClicked(0))
	{
		TurnoutId turnout{TurnoutId::none};
		ElementId element{ElementId::none};
		TrackId track{TrackId::none};

		if (hit_turnout(mouse, turnout))
		{
			m_sel_turnout = turnout;
			m_dragging_turnout = true;
		}
		else if (hit_element(mouse, element, track))
		{
			m_sel_element = element;
			m_track = track;
			auto const *chosen{maj0sted::editor::find_track(m_document, track)};
			if (chosen != nullptr)
			{
				std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", chosen->name.c_str());
			}
		}
		else
		{
			append_point_to(wx, wy);
		}
	}

	// right-click takes back the last click, which is the only thing it does
	if (ImGui::IsMouseClicked(1))
	{
		drop_last_vertex();
	}

	if (ImGui::IsMouseDown(0) && m_dragging_turnout && m_sel_turnout != TurnoutId::none)
	{
		auto *placement{maj0sted::editor::find_turnout(m_document, m_sel_turnout)};
		if (placement != nullptr)
		{
			double station{0.0};
			if (station_on(placement->on, wx, wy, station))
			{
				placement->station = station;
				solve();
			}
		}
	}

	if (ImGui::IsMouseReleased(0))
	{
		m_dragging_turnout = false;
	}

	// double-clicking anywhere on a track makes it the one being edited
	if (ImGui::IsMouseDoubleClicked(0))
	{
		auto const track{nearest_track_axis(mouse, 16.0f)};
		if (track != TrackId::none)
		{
			m_track = track;
			auto const *chosen{maj0sted::editor::find_track(m_document, track)};
			if (chosen != nullptr)
			{
				std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", chosen->name.c_str());
				m_status = "edytowany: " + chosen->name;
			}
		}
	}
}

void plan_panel::draw_on_scene()
{
	// the background list paints over the rendered scenery but under every window, so the controls
	// stay readable on top of the drawing
	auto *drawlist{ImGui::GetBackgroundDrawList()};

	// orthophoto under everything else. only a georeferenced map knows where on the ground it sits,
	// so a fictional one gets no imagery
	if (m_showortho && Global.scenery_georeferenced)
	{
		double planx{0.0};
		double plany{0.0};
		world_to_plan(Global.pCamera.Pos, planx, plany);

		// yaw is pinned north-up, so the screen is an axis-aligned box in plan space
		auto const display{ImGui::GetIO().DisplaySize};
		auto const halfheight{static_cast<double>(Global.editor_ortho_extent)};
		auto const halfwidth{halfheight * std::max(1.0f, display.x) / std::max(1.0f, display.y)};
		maj0sted::editor::TileBBox const view{planx - halfwidth, plany - halfheight, planx + halfwidth, plany + halfheight};

		auto const draw_box = [&](maj0sted::editor::TileBBox const &Box, unsigned int const Texture) {
			ImVec2 corners[4];
			auto const ok = world_to_screen(plan_to_world(Box.min_x, Box.max_y), corners[0]) && world_to_screen(plan_to_world(Box.max_x, Box.max_y), corners[1]) &&
			                world_to_screen(plan_to_world(Box.max_x, Box.min_y), corners[2]) && world_to_screen(plan_to_world(Box.min_x, Box.min_y), corners[3]);
			if (false == ok)
			{
				return;
			}
			// north-west corner first, and the texture's first row (v=0) is its northern edge
			drawlist->AddImageQuad(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(Texture)), corners[0], corners[1], corners[2], corners[3], ImVec2(0.0f, 0.0f), ImVec2(1.0f, 0.0f),
			                       ImVec2(1.0f, 1.0f), ImVec2(0.0f, 1.0f));
		};

		// once the view is wider than a kilometre the 100 m orto cells are the wrong scale:
		// switch to a 1 km topo grid that can cover the whole screen
		constexpr double kTopoPreviewMetres = 1000.0;
		auto const span{std::max(halfwidth, halfheight) * 2.0};
		if (span > kTopoPreviewMetres)
		{
			for (auto const &tile : m_topo.collect(view, 80))
			{
				draw_box(tile.box, tile.texture);
			}
		}
		else
		{
			for (auto const &tile : m_ortho.collect(view))
			{
				draw_box(tile.box, tile.texture);
			}
		}
	}

	// the track itself
	std::vector<ImVec2> points;
	for (auto const &rail : m_rails)
	{
		if (rail.points.size() < 2)
		{
			continue;
		}
		auto const active{rail.track == m_track || rail.turnout != TurnoutId::none};
		points.clear();
		points.reserve(rail.points.size());
		for (auto const &point : rail.points)
		{
			ImVec2 screen;
			if (false == world_to_screen(plan_to_world(point.x, point.y), screen))
			{
				continue;
			}
			points.push_back(screen);
		}
		if (points.size() < 2)
		{
			continue;
		}
		auto colour{element_colour(rail.kind, active)};
		auto width{2.5f};
		if (rail.turnout != TurnoutId::none)
		{
			// every element of a rozjazd is drawn as itself: the pre-blade lead runs along
			// the through track, the blade is the piece that leaves the opornica, then the
			// curve and the straight frog rail
			switch (rail.part)
			{
			case 0:
				// the pre-blade lead lies exactly on the through track, so it has to be
				// drawn wide and in a colour of its own or it simply is not there
				colour = IM_COL32(140, 240, 150, 255); // przediglicowy
				width = 4.5f;
				break;
			case 1:
				colour = IM_COL32(120, 235, 255, 245); // iglica
				break;
			case 3:
				colour = IM_COL32(255, 200, 110, 235); // prosta krzyzownicowa
				break;
			default:
				colour = IM_COL32(255, 140, 200, 230); // luk
				break;
			}
		}
		else if (rail.element == m_sel_element && m_sel_element != ElementId::none)
		{
			colour = IM_COL32(255, 255, 255, 245);
		}
		drawlist->AddPolyline(points.data(), static_cast<int>(points.size()), colour, false, width);
	}

	// turnouts: PR, the frog, and the construction points a drawing is dimensioned from
	for (auto const &turnout : m_solution.turnouts)
	{
		if (false == turnout.valid)
		{
			continue;
		}
		auto const selected{turnout.id == m_sel_turnout};

		// the points that sit on the straight - PR, the blade tip and the koniec rozjazdu -
		// are drawn the way a catalogue drawing marks them: a tick across the track, not a
		// dot lost among the rails the turnout runs along
		auto const tick{[&](maj0sted::editor::TurnoutMark const &Mark_, ImU32 const Colour_, float const Half_) {
			ImVec2 centre;
			if (false == world_to_screen(plan_to_world(Mark_.x, Mark_.y), centre))
			{
				return;
			}
			ImVec2 along;
			if (false == world_to_screen(plan_to_world(Mark_.x + turnout.start.hx, Mark_.y + turnout.start.hy), along))
			{
				return;
			}
			auto dx{along.x - centre.x};
			auto dy{along.y - centre.y};
			auto const len{std::sqrt(dx * dx + dy * dy)};
			if (len < 1e-3f)
			{
				return;
			}
			dx /= len;
			dy /= len;
			drawlist->AddLine(ImVec2(centre.x + dy * Half_, centre.y - dx * Half_), ImVec2(centre.x - dy * Half_, centre.y + dx * Half_), Colour_, 2.0f);
		}};

		for (auto const &mark : turnout.marks)
		{
			ImVec2 screen;
			if (false == world_to_screen(plan_to_world(mark.x, mark.y), screen))
			{
				continue;
			}

			// PR and KR carry the switch - both on the diverging path and, for the
			// koniec rozjazdu, on the straight it stands on. the ostrze is the other
			// point worth finding on the drawing, so it gets a colour of its own
			auto const primary{mark.kind == 0 || mark.kind == 5 || mark.kind == 9};
			auto const blade{mark.kind == 1 || mark.kind == 7 || mark.kind == 8};
			drawlist->AddCircleFilled(screen, primary ? (selected ? 6.0f : 5.0f) : (blade ? 4.0f : 3.0f),
			                          primary ? IM_COL32(255, 210, 90, 240) : (blade ? IM_COL32(120, 235, 255, 230) : IM_COL32(255, 150, 210, 190)));
		}
		for (auto const &mark : turnout.marks)
		{
			switch (mark.kind)
			{
			case 0:  // PR
			case 9:  // koniec rozjazdu na torze zasadniczym
				tick(mark, IM_COL32(255, 210, 90, 245), selected ? 14.0f : 11.0f);
				break;
			case 1: // ostrze iglicy: koniec odcinka przediglicowego
				tick(mark, IM_COL32(120, 235, 255, 245), 9.0f);
				break;
			case 7: // A: koniec zestruganego dzioba, iglica ma tu grubosc u
				tick(mark, IM_COL32(140, 240, 150, 245), 7.0f);
				break;
			default:
				break;
			}
		}

		// the blade's own geometry lives in the first few metres of the switch, so at any
		// zoom that shows the whole rozjazd its points land on top of one another. named
		// on leader lines fanned out to the side they can at least be told apart
		if (selected)
		{
			auto ordered{turnout.marks};
			std::sort(ordered.begin(), ordered.end(),
			          [](auto const &Left_, auto const &Right_) { return Left_.station < Right_.station; });

			auto level{0};
			for (auto const &mark : ordered)
			{
				if (mark.kind < 0 || mark.kind >= static_cast<int>(sizeof(mark_names) / sizeof(mark_names[0])))
				{
					continue;
				}
				ImVec2 centre;
				ImVec2 along;
				if (false == world_to_screen(plan_to_world(mark.x, mark.y), centre) ||
				    false == world_to_screen(plan_to_world(mark.x + turnout.start.hx, mark.y + turnout.start.hy), along))
				{
					continue;
				}
				auto dx{along.x - centre.x};
				auto dy{along.y - centre.y};
				auto const len{std::sqrt(dx * dx + dy * dy)};
				if (len < 1e-3f)
				{
					continue;
				}
				dx /= len;
				dy /= len;

				// alternate sides and step further out on each pass, so the leaders fan
				// instead of stacking on one line
				auto const side{(level % 2) == 0 ? 1.0f : -1.0f};
				auto const reach{(30.0f + 17.0f * static_cast<float>(level / 2)) * side};
				ImVec2 const anchor(centre.x + dy * reach, centre.y - dx * reach);

				char label[96];
				if (mark.station > 0.0)
				{
					std::snprintf(label, sizeof(label), "%s  %.3f", mark_names[mark.kind], mark.station);
				}
				else
				{
					std::snprintf(label, sizeof(label), "%s", mark_names[mark.kind]);
				}
				auto const size{ImGui::CalcTextSize(label)};
				ImVec2 const corner(anchor.x - size.x * 0.5f, anchor.y - size.y * 0.5f);

				drawlist->AddLine(centre, anchor, IM_COL32(200, 200, 200, 130), 1.0f);
				drawlist->AddRectFilled(ImVec2(corner.x - 3.0f, corner.y - 2.0f), ImVec2(corner.x + size.x + 3.0f, corner.y + size.y + 2.0f),
				                        IM_COL32(15, 20, 30, 205), 3.0f);
				drawlist->AddText(corner, IM_COL32(235, 240, 250, 245), label);
				++level;
			}
		}
	}

	// where a chain was cut short, say so on the spot rather than only in the list
	for (auto const &track : m_solution.tracks)
	{
		if (track.complete || track.elements.empty())
		{
			continue;
		}
		ImVec2 screen;
		if (world_to_screen(plan_to_world(track.end.x, track.end.y), screen))
		{
			drawlist->AddCircle(screen, 9.0f, IM_COL32(255, 110, 110, 230), 0, 2.0f);
			drawlist->AddLine(ImVec2(screen.x - 6.0f, screen.y - 6.0f), ImVec2(screen.x + 6.0f, screen.y + 6.0f), IM_COL32(255, 110, 110, 230), 2.0f);
			drawlist->AddLine(ImVec2(screen.x - 6.0f, screen.y + 6.0f), ImVec2(screen.x + 6.0f, screen.y - 6.0f), IM_COL32(255, 110, 110, 230), 2.0f);
		}
	}

	// the ends of the edited track's straights, while ctrl says they are what you are after
	if (ImGui::GetIO().KeyCtrl)
	{
		auto const *solved{maj0sted::editor::find_track(m_solution, m_track)};
		auto const *track{current_track()};
		if (solved != nullptr && track != nullptr)
		{
			for (std::size_t i = 0; i < solved->elements.size() && i < track->elements.size(); ++i)
			{
				if (track->elements[i].kind != Kind::Line)
				{
					continue;
				}
				auto const pinned{i == 0 && false == std::holds_alternative<maj0sted::editor::AtPose>(track->anchor)};
				for (int end = 0; end < 2; ++end)
				{
					if (end == 0 && pinned)
					{
						continue;
					}
					auto const &pose{end == 0 ? solved->elements[i].start : solved->elements[i].end};
					ImVec2 screen;
					if (false == world_to_screen(plan_to_world(pose.x, pose.y), screen))
					{
						continue;
					}
					auto const grabbed{m_dragging_straight && i == m_drag_index && end == m_drag_end};
					auto const size{grabbed ? 5.0f : 4.0f};
					drawlist->AddRectFilled(ImVec2(screen.x - size, screen.y - size), ImVec2(screen.x + size, screen.y + size),
					                        grabbed ? IM_COL32(255, 255, 255, 245) : IM_COL32(255, 240, 140, 200));
				}
			}
		}
	}

	// where the run of clicks starts, before there is any track to show for it
	if (m_pending)
	{
		ImVec2 screen;
		if (world_to_screen(plan_to_world(m_startx, m_starty), screen))
		{
			drawlist->AddCircle(screen, 6.0f, IM_COL32(255, 240, 140, 230), 0, 2.0f);
		}
	}

	// scalanie: every end a join could be made from or to, and what would be laid between the one
	// already picked and whichever the cursor is over
	if (m_pick_join != 0)
	{
		auto const mouse{ImGui::GetIO().MousePos};
		TrackId hovered{TrackId::none};
		int hovered_end{0};
		auto const over{hit_track_end(mouse, 18.0f, hovered, hovered_end)};

		for (auto const &track : m_solution.tracks)
		{
			for (int end = 0; end < 2; ++end)
			{
				if (false == loose_end(track, end))
				{
					continue;
				}
				auto const &pose{end == 0 ? track.start : track.end};
				ImVec2 screen;
				if (false == world_to_screen(plan_to_world(pose.x, pose.y), screen))
				{
					continue;
				}
				auto const picked{m_pick_join == 2 && track.id == m_join_first && end == m_join_first_end};
				auto const under{over && track.id == hovered && end == hovered_end};
				auto const colour{picked ? IM_COL32(120, 255, 160, 250) : under ? IM_COL32(255, 255, 255, 250) : IM_COL32(255, 240, 140, 190)};
				// a start is a square and an end is a circle, so which way a join would run is
				// there to be seen and not guessed at
				if (end == 0)
				{
					drawlist->AddRect(ImVec2(screen.x - 6.0f, screen.y - 6.0f), ImVec2(screen.x + 6.0f, screen.y + 6.0f), colour, 0.0f, 0, picked || under ? 3.0f : 1.5f);
				}
				else
				{
					drawlist->AddCircle(screen, 6.0f, colour, 0, picked || under ? 3.0f : 1.5f);
				}
				// the whole track lights up with the end under the cursor, so there is no doubt
				// about which one is being taken hold of
				if (false == (picked || under) || track.centreline.size() < 2)
				{
					continue;
				}
				std::vector<ImVec2> axis;
				axis.reserve(track.centreline.size());
				for (auto const &point : track.centreline)
				{
					ImVec2 at;
					if (world_to_screen(plan_to_world(point.x, point.y), at))
					{
						axis.push_back(at);
					}
				}
				if (axis.size() > 1)
				{
					drawlist->AddPolyline(axis.data(), static_cast<int>(axis.size()), (colour & 0x00ffffffu) | 0x66000000u, false, 6.0f);
				}
			}
		}

		// the proposal itself, before anything is done to the document. it runs from the
		// end pointed at first to the one under the cursor, whichever way round the two
		// tracks happen to be written down: an end is left the way it points, a start is
		// left the way it came, and the arriving end reads the other way about
		if (m_pick_join == 2 && over && hovered != m_join_first)
		{
			auto const *head{maj0sted::editor::find_track(m_solution, m_join_first)};
			auto const *tail{maj0sted::editor::find_track(m_solution, hovered)};
			if (head != nullptr && tail != nullptr)
			{
				auto const leaving{join_pose(*head, m_join_first_end, true)};
				auto const arriving{join_pose(*tail, hovered_end, false)};
				maj0sted::editor::JoinSettings settings;
				settings.radius = m_join_radius;
				auto const plan{maj0sted::editor::plan_join(leaving, arriving, settings)};

				std::string label;
				if (plan.ok)
				{
					std::vector<maj0sted::domain::geometry::XY> points;
					auto walker{leaving};
					for (auto const &element : plan.elements)
					{
						auto const k{element.kind == Kind::Line || element.radius <= 0.0 ? 0.0 : element.hand / element.radius};
						walker = maj0sted::domain::geometry::layout_segment(k, k, element.length, walker, &points);
					}
					std::vector<ImVec2> line;
					line.reserve(points.size());
					for (auto const &point : points)
					{
						ImVec2 screen;
						if (world_to_screen(plan_to_world(point.x, point.y), screen))
						{
							line.push_back(screen);
						}
					}
					if (line.size() > 1)
					{
						dashed_polyline(drawlist, line, IM_COL32(120, 255, 160, 235), 2.5f);
					}
					label = maj0sted::editor::join_kind_name(plan.kind);
					auto laid{0.0};
					for (auto const &element : plan.elements)
					{
						laid += element.length;
					}
					if (laid > 0.0)
					{
						label += ", " + to_string(laid, 1) + " m";
					}
					if (plan.tightest > 0.0)
					{
						label += ", R=" + to_string(plan.tightest, 0) + " m";
					}
				}
				else
				{
					label = plan.why;
				}

				auto const size{ImGui::CalcTextSize(label.c_str())};
				ImVec2 const corner(mouse.x + 16.0f, mouse.y + 16.0f);
				drawlist->AddRectFilled(ImVec2(corner.x - 4.0f, corner.y - 3.0f), ImVec2(corner.x + size.x + 4.0f, corner.y + size.y + 3.0f), IM_COL32(15, 20, 30, 215), 3.0f);
				drawlist->AddText(corner, plan.ok ? IM_COL32(160, 255, 190, 250) : IM_COL32(255, 190, 110, 250), label.c_str());
			}
		}
	}
}

void plan_panel::go_to_plan()
{
	auto minx{std::numeric_limits<double>::max()};
	auto miny{std::numeric_limits<double>::max()};
	auto maxx{std::numeric_limits<double>::lowest()};
	auto maxy{std::numeric_limits<double>::lowest()};

	for (auto const &track : m_solution.tracks)
	{
		for (auto const &point : track.centreline)
		{
			minx = std::min(minx, point.x);
			maxx = std::max(maxx, point.x);
			miny = std::min(miny, point.y);
			maxy = std::max(maxy, point.y);
		}
	}
	if (minx > maxx)
	{
		return; // nothing drawn yet, so there is nowhere to go
	}

	auto &camera{editor_mode::get_camera()};
	auto const centre{plan_to_world((minx + maxx) * 0.5, (miny + maxy) * 0.5)};
	camera.Pos.x = centre.x;
	camera.Pos.z = centre.z;

	// show the whole drawing with a little room around it
	auto const span{std::max({maxx - minx, maxy - miny, 50.0})};
	Global.editor_ortho_extent = std::clamp(static_cast<float>(span * 0.6), 5.0f, 20000.0f);
}

// ---------------------------------------------------------------------------
// panels
// ---------------------------------------------------------------------------

void plan_panel::render_toolbar()
{
	ImGui::Text("widok z gory, %.0f m w poprzek", Global.editor_ortho_extent * 2.0f);
	ImGui::TextDisabled("klik w teren prowadzi tor, klik w odcinek go wybiera, dwuklik zmienia edytowany tor");
	ImGui::TextDisabled("Ctrl + LPM na krancu prostej: przeciagniecie; luki obok dopasuja sie same");
	if (Global.scenery_georeferenced)
	{
		ImGui::Text("zero mapy: %.0f, %.0f (EPSG:2180)", Global.scenery_origin.x, Global.scenery_origin.y);
	}
	else
	{
		ImGui::TextDisabled("mapa fikcyjna, bez georeferencji");
	}

	if (ImGui::Button("Nowa mapa"))
	{
		// opened from the same stack level the modal is drawn on, which is render_contents
		ImGui::OpenPopup("New map");
	}
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip("wybor miejsca: mapa fikcyjna albo punkt w Polsce, na ktorym staje zero scenerii");
	}
	if (Global.scenery_georeferenced)
	{
		ImGui::SameLine();
		ImGui::Checkbox("Ortofoto", &m_showortho);
		if (m_showortho)
		{
			auto const pending{static_cast<int>(m_ortho.pending() + m_topo.pending())};
			if (pending > 0)
			{
				ImGui::SameLine();
				ImGui::TextDisabled("(%d kafli w drodze)", pending);
			}
		}
	}
	ImGui::Separator();

	// how a corner gets rounded when the track is carried on by clicking. a kink is not track, so
	// the turn from where the track heads now to where the click is goes round an arc of this
	// radius; it eats R*tan(kat/2) off the leg behind it and off the one in front
	ImGui::SetNextItemWidth(120.0f);
	ImGui::InputDouble("R w narozniku [m]", &m_corner_radius, 10.0, 100.0, "%.0f");
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip("klikniecie w pusty teren prowadzi tor dalej; zwrot miedzy dotychczasowym kierunkiem\n"
		                  "a klikietym punktem zostaje zaokraglony lukiem o tym promieniu.\n"
		                  "im wiekszy R, tym dluzsza styczna R*tan(kat/2) sciagana z obu prostych\n"
		                  "- dlatego ostry naroznik tuz przy koncu toru sie nie miesci.\n\n"
		                  "prawy klik cofa ostatnie klikniecie");
	}
	ImGui::SameLine();
	ImGui::TextDisabled("prawy klik cofa klikniecie");

	// which track is being edited
	auto *track{current_track()};
	if (ImGui::BeginCombo("Edytowany", track != nullptr ? track->name.c_str() : "-"))
	{
		for (auto const &candidate : m_document.tracks)
		{
			if (ImGui::Selectable(candidate.name.c_str(), candidate.id == m_track))
			{
				m_track = candidate.id;
				std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", candidate.name.c_str());
				clear_selection();
				m_pending = false;
			}
		}
		ImGui::EndCombo();
	}
	if (track != nullptr)
	{
		ImGui::SetNextItemWidth(240.0f);
		if (ImGui::InputText("Nazwa", m_namebuf, sizeof(m_namebuf)))
		{
			track->name = m_namebuf;
		}
	}

	if (ImGui::Button("Nowy tor"))
	{
		new_track();
		solve();
	}
	ImGui::SameLine();
	if (m_document.tracks.size() > 1 && ImGui::Button("Usun tor"))
	{
		delete_current_track();
	}
	ImGui::SameLine();
	if (ImGui::Button("Pokaz calosc"))
	{
		go_to_plan();
	}

	render_join();

	ImGui::Separator();
}

void plan_panel::render_join()
{
	if (m_document.tracks.size() < 2)
	{
		return;
	}
	ImGui::SameLine();
	if (ImGui::Button(m_pick_join != 0 ? "Lacze... (Esc)" : "Polacz"))
	{
		m_pick_join = m_pick_join != 0 ? 0 : 1;
		m_join_first = TrackId::none;
		m_status = m_pick_join != 0 ? "wskaz dwa wolne konce torow; podglad pokaze, co miedzy nimi stanie" : "";
	}
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip("wolne konce torow zapalaja sie na mapie: kwadrat to poczatek, kolo to koniec.\n"
		                  "wskaz dwa dowolne - wyjdzie z tego jeden tor. gdy trzeba, jeden z nich\n"
		                  "zostaje czytany od drugiego konca: w terenie nic sie nie rusza.\n"
		                  "gdy oba leza na jednej prostej, wychodzi jedna prosta, bez zadnego luku");
	}
	if (m_pick_join != 0)
	{
		ImGui::SetNextItemWidth(120.0f);
		ImGui::InputDouble("R polaczenia [m]", &m_join_radius, 10.0, 100.0, "%.0f");
	}
}

void plan_panel::join_ends(TrackId const A, int const Aend, TrackId const B, int const Bend)
{
	maj0sted::editor::JoinSettings settings;
	settings.radius = m_join_radius;

	auto const report{maj0sted::editor::join_ends(m_document, m_solution, A, Aend, B, Bend, settings)};
	if (false == report.ok)
	{
		m_status = report.why;
		return;
	}

	// whichever track was left standing is the one to select afterwards, and it is not
	// always the first clicked - the other one may have been the only one that could turn
	auto const survivor{maj0sted::editor::find_track(m_document, A) != nullptr ? A : B};

	char status[256];
	std::snprintf(status, sizeof(status), "polaczone: %s, %d odcinkow zlanych w jeden, %d rozjazdow przeniesionych%s", maj0sted::editor::join_kind_name(report.join.kind), report.fused,
	              report.turnouts, report.reversed ? ", jeden tor czytany od drugiego konca" : "");
	m_status = status;
	m_track = survivor;
	m_pick_join = 0;
	m_join_first = TrackId::none;
	auto const *joined{maj0sted::editor::find_track(m_document, survivor)};
	if (joined != nullptr)
	{
		std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", joined->name.c_str());
	}
	clear_selection();
	solve();
}

maj0sted::domain::geometry::Pose plan_panel::join_pose(maj0sted::editor::SolvedTrack const &Track_, int const End_, bool const Leaving_)
{
	auto pose{End_ == 1 ? Track_.end : Track_.start};
	// what is laid runs out of one end and into the other, so an end that points the
	// wrong way for that is read backwards - which is exactly what turning the track
	// round would do to it
	if (Leaving_ ? End_ == 0 : End_ == 1)
	{
		pose.hx = -pose.hx;
		pose.hy = -pose.hy;
	}
	return pose;
}

bool plan_panel::loose_end(maj0sted::editor::SolvedTrack const &Track_, int const End_) const
{
	if (Track_.elements.empty())
	{
		return false;
	}
	if (End_ == 0)
	{
		// a track hanging on a turnout's port starts on the frog, and that is not a loose end
		auto const *authored{maj0sted::editor::find_track(m_document, Track_.id)};
		if (authored == nullptr || false == std::holds_alternative<maj0sted::editor::AtPose>(authored->anchor))
		{
			return false;
		}
	}

	// and one already standing on another track's end is joined to it, whether or not the two are
	// one track in the document
	auto const &pose{End_ == 0 ? Track_.start : Track_.end};
	for (auto const &other : m_solution.tracks)
	{
		if (other.id == Track_.id || other.elements.empty())
		{
			continue;
		}
		for (auto const &at : {other.start, other.end})
		{
			if (std::hypot(at.x - pose.x, at.y - pose.y) < 0.10)
			{
				return false;
			}
		}
	}
	return true;
}

bool plan_panel::hit_track_end(ImVec2 const &Mouse, float const Tolerance, TrackId &OutTrack, int &OutEnd) const
{
	auto best{Tolerance};
	auto found{false};
	for (auto const &track : m_solution.tracks)
	{
		for (int end = 0; end < 2; ++end)
		{
			if (false == loose_end(track, end))
			{
				continue;
			}
			auto const &pose{end == 0 ? track.start : track.end};
			ImVec2 screen;
			if (false == world_to_screen(plan_to_world(pose.x, pose.y), screen))
			{
				continue;
			}
			auto const distance{std::hypot(screen.x - Mouse.x, screen.y - Mouse.y)};
			if (distance < best)
			{
				best = static_cast<float>(distance);
				OutTrack = track.id;
				OutEnd = end;
				found = true;
			}
		}
	}
	return found;
}

void plan_panel::render_elements()
{
	auto *track{current_track()};
	if (track == nullptr)
	{
		return;
	}

	if (false == ImGui::CollapsingHeader("Odcinki", ImGuiTreeNodeFlags_DefaultOpen))
	{
		return;
	}

	// what a new element is made of. pick the numbers, then say which of the three it is: there is
	// nothing else to an element, and nothing here is a mode of its own
	ImGui::SetNextItemWidth(90.0f);
	ImGui::InputDouble("dl. [m]", &m_new_length, 5.0, 50.0, "%.1f");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(90.0f);
	ImGui::InputDouble("R [m]", &m_new_radius, 10.0, 100.0, "%.1f");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(110.0f);
	{
		int hand{m_new_hand >= 0 ? 0 : 1};
		if (ImGui::Combo("skret", &hand, "w lewo\0w prawo\0"))
		{
			m_new_hand = hand == 0 ? 1 : -1;
		}
	}

	if (ImGui::Button("+ prosta"))
	{
		append_element(Kind::Line);
	}
	ImGui::SameLine();
	if (ImGui::Button("+ luk"))
	{
		append_element(Kind::Arc);
	}
	ImGui::SameLine();
	if (ImGui::Button("+ krzywa przejsciowa"))
	{
		append_element(Kind::Clothoid);
	}
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip("R jest promieniem na jej koncu; na poczatku bierze sie z poprzedniego odcinka.\n0 = wybiega w prosta");
	}
	ImGui::SameLine();
	if (ImGui::Button("Cofnij") && false == track->elements.empty())
	{
		drop_last_element();
	}

	ImGui::Separator();

	if (track->elements.empty())
	{
		ImGui::TextDisabled("pusty tor. dolóz odcinek ponizej, albo klikaj po mapie: pierwsze dwa klikniecia kladzie prosta.");
		return;
	}

	auto dirty{false};
	// a change of shape - a radius, a hand, the length of a transition - belongs to its own corner
	// and to nothing else, so the track is re-laid along the lines its straights already stood on.
	// a change of length is what it says it is: the element gets longer, and what follows moves
	auto refit{false};
	auto const *solved{maj0sted::editor::find_track(m_solution, track->id)};

	for (std::size_t i = 0; i < track->elements.size(); ++i)
	{
		auto &element{track->elements[i]};
		ImGui::PushID(static_cast<int>(i));

		auto const selected{element.id == m_sel_element};
		auto const *solved_element{solved != nullptr && i < solved->elements.size() ? &solved->elements[i] : nullptr};
		auto const held{std::holds_alternative<Parallel>(element.hold)};

		// the row says the whole element: what it is, how long, and how it turns. a held one says
		// so too, because its shape is not its own
		char label[200];
		switch (element.kind)
		{
		case Kind::Arc:
			std::snprintf(label, sizeof(label), "%zu. Luk R=%.0f %s   %.1f m (%.2f st.)%s%s", i + 1, element.radius, element.hand >= 0 ? "w lewo" : "w prawo", element.length,
			              element.radius > 0.0 ? element.length / element.radius * 180.0 / std::numbers::pi : 0.0, held ? "  [rownolegly]" : "",
			              solved_element != nullptr ? "" : "  (nie polozony)");
			break;
		case Kind::Clothoid:
			std::snprintf(label, sizeof(label), "%zu. Krzywa przejsciowa   %.1f m (do R=%.0f %s)%s", i + 1, element.length, element.radius, element.hand >= 0 ? "w lewo" : "w prawo",
			              solved_element != nullptr ? "" : "  (nie polozony)");
			break;
		default:
			std::snprintf(label, sizeof(label), "%zu. Prosta   %.1f m%s%s", i + 1, element.length, held ? "  [rownolegla]" : "",
			              solved_element != nullptr ? "" : "  (nie polozony)");
			break;
		}
		if (ImGui::Selectable(label, selected))
		{
			m_sel_element = element.id;
		}

		if (selected)
		{
			ImGui::Indent();

			int kind{element.kind == Kind::Line ? 0 : (element.kind == Kind::Arc ? 1 : 2)};
			ImGui::SetNextItemWidth(180.0f);
			if (ImGui::Combo("rodzaj", &kind, "prosta\0luk\0krzywa przejsciowa\0"))
			{
				element.kind = kind == 0 ? Kind::Line : (kind == 1 ? Kind::Arc : Kind::Clothoid);
				dirty = true;
			}

			ImGui::SetNextItemWidth(120.0f);
			if (ImGui::InputDouble("dlugosc [m]", &element.length, 5.0, 50.0, "%.2f"))
			{
				dirty = true;
				// a transition's length is authored, and what it takes out of the corner is the
				// corner's business - so this one is absorbed rather than pushed down the track
				refit = element.kind == Kind::Clothoid;
			}

			if (element.kind != Kind::Line)
			{
				ImGui::SetNextItemWidth(120.0f);
				if (ImGui::InputDouble(element.kind == Kind::Clothoid ? "R na koncu [m]" : "promien [m]", &element.radius, 10.0, 100.0, "%.2f"))
				{
					maj0sted::editor::sync_joint(*track, i);
					dirty = true;
					refit = true;
				}
				if (element.kind == Kind::Clothoid && ImGui::IsItemHovered())
				{
					ImGui::SetTooltip("0 = krzywa wybiegajaca w prosta; promien na poczatku bierze sie z poprzedniego odcinka");
				}
				ImGui::SetNextItemWidth(120.0f);
				int hand{element.hand >= 0 ? 0 : 1};
				if (ImGui::Combo("skret", &hand, "w lewo\0w prawo\0"))
				{
					element.hand = hand == 0 ? 1 : -1;
					maj0sted::editor::sync_joint(*track, i);
					dirty = true;
					refit = true;
				}
				if (element.kind == Kind::Arc && element.radius > 0.0)
				{
					auto turn{element.length / element.radius * 180.0 / std::numbers::pi};
					ImGui::SetNextItemWidth(120.0f);
					if (ImGui::InputDouble("kat zwrotu [st.]", &turn, 1.0, 10.0, "%.3f"))
					{
						element.length = std::max(0.0, turn) * std::numbers::pi / 180.0 * element.radius;
						dirty = true;
					}
					if (ImGui::IsItemHovered())
					{
						ImGui::SetTooltip("ten sam luk, powiedziany katem zamiast dlugoscia");
					}
				}
			}

			// TODO: kontrolki rownoleglosci (trzymanie miedzytorza) chwilowo schowane z gui -
			// samo Parallel/hold i solver zostaja, wraca tu przycisk i pole odstepu

			if (solved_element != nullptr)
			{
				ImGui::TextDisabled("polozony: %.2f m, R %.1f -> %.1f", solved_element->length, std::abs(solved_element->k0) > 1e-9 ? 1.0 / std::abs(solved_element->k0) : 0.0,
				                    std::abs(solved_element->k1) > 1e-9 ? 1.0 / std::abs(solved_element->k1) : 0.0);
			}

			if (ImGui::SmallButton("usun odcinek"))
			{
				ImGui::Unindent();
				ImGui::PopID();
				delete_selected_element();
				return;
			}

			ImGui::Unindent();
		}

		ImGui::PopID();
	}

	if (refit)
	{
		// the change stands either way - it is what was asked for. what a failed fit costs is that
		// the rest of the track follows along instead of staying put, and that is worth saying
		std::string why;
		if (refit_current_track(why))
		{
			m_status = "dopasowano naroznik; reszta toru bez zmian";
		}
		else
		{
			m_status = why + " - dalsza czesc toru poszla za zmiana";
		}
	}
	if (dirty)
	{
		solve();
	}
}

// the geometry of a turnout is the type's, and the type is a list: przediglicowy, iglica, luk,
// prosta krzyzownicowa, in the order they are laid from PR. every length is the catalogue's -
// nothing here is fitted, so the skos and the catalogue length are checked against what the list
// works out to and the difference is said out loud
void plan_panel::render_turnout_type(maj0sted::editor::TurnoutType &Type_)
{
	auto dirty{false};

	ImGui::SetNextItemWidth(90.0f);
	if (ImGui::InputDouble("skos 1:n", &Type_.crossing_n, 0.5, 1.0, "%.3f"))
	{
		dirty = true;
	}
	ImGui::SameLine();
	ImGui::SetNextItemWidth(90.0f);
	if (ImGui::InputDouble("dl. kat. a+b [m]", &Type_.length, 0.1, 1.0, "%.3f"))
	{
		dirty = true;
	}
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip("dlugosc katalogowa to suma stycznych a + b, tak jak na rysunku typu");
	}

	static char const *const part_names[]{"przediglicowy", "iglica", "luk", "krzyzownicowa"};
	for (std::size_t i = 0; i < Type_.pieces.size(); ++i)
	{
		auto &piece{Type_.pieces[i]};
		ImGui::PushID(static_cast<int>(2000 + i));
		piece.part = std::clamp(piece.part, 0, 3);

		char label[200];
		if (piece.radius_start <= 0.0 && piece.radius_end <= 0.0)
		{
			std::snprintf(label, sizeof(label), "%zu. %s   prosta %.3f m", i + 1, part_names[piece.part], piece.length);
		}
		else if (std::abs(piece.radius_start - piece.radius_end) < 1e-9)
		{
			std::snprintf(label, sizeof(label), "%zu. %s   luk R %.0f   %.3f m", i + 1, part_names[piece.part], piece.radius_start, piece.length);
		}
		else
		{
			std::snprintf(label, sizeof(label), "%zu. %s   R %.0f -> %.0f   %.3f m", i + 1, part_names[piece.part], piece.radius_start, piece.radius_end,
			              piece.length);
		}
		if (ImGui::Selectable(label, static_cast<int>(i) == m_sel_piece))
		{
			m_sel_piece = static_cast<int>(i);
		}
		if (static_cast<int>(i) == m_sel_piece)
		{
			ImGui::Indent();
			ImGui::SetNextItemWidth(150.0f);
			if (ImGui::Combo("rola", &piece.part, "przediglicowy\0iglica\0luk\0krzyzownicowa\0"))
			{
				dirty = true;
			}
			ImGui::SetNextItemWidth(110.0f);
			if (ImGui::InputDouble("dlugosc [m]", &piece.length, 0.1, 1.0, "%.3f"))
			{
				dirty = true;
			}
			ImGui::SetNextItemWidth(110.0f);
			if (ImGui::InputDouble("R poczatek [m]", &piece.radius_start, 10.0, 100.0, "%.2f"))
			{
				dirty = true;
			}
			ImGui::SameLine();
			ImGui::SetNextItemWidth(110.0f);
			if (ImGui::InputDouble("R koniec [m]", &piece.radius_end, 10.0, 100.0, "%.2f"))
			{
				dirty = true;
			}
			if (ImGui::IsItemHovered())
			{
				ImGui::SetTooltip("0 = prosta; rowne promienie = luk, rozne = krzywa przejsciowa");
			}
			auto turn_in{piece.turn_in * 180.0 / std::numbers::pi};
			ImGui::SetNextItemWidth(110.0f);
			if (ImGui::InputDouble("nagiecie [st.]", &turn_in, 0.05, 0.5, "%.4f"))
			{
				piece.turn_in = std::max(0.0, turn_in) * std::numbers::pi / 180.0;
				dirty = true;
			}
			if (ImGui::IsItemHovered())
			{
				ImGui::SetTooltip("kat nagiecia iglicy (beta): 0 = iglica styczna do opornicy");
			}
			if (ImGui::SmallButton("usun odcinek"))
			{
				Type_.pieces.erase(Type_.pieces.begin() + static_cast<long>(i));
				m_sel_piece = -1;
				ImGui::Unindent();
				ImGui::PopID();
				solve();
				return;
			}
			ImGui::Unindent();
		}
		ImGui::PopID();
	}
	if (ImGui::SmallButton("dodaj odcinek"))
	{
		Type_.pieces.push_back(maj0sted::editor::TurnoutPieceSpec{2, 1.0, 0.0, 0.0, 0.0});
		m_sel_piece = static_cast<int>(Type_.pieces.size()) - 1;
		dirty = true;
	}

	// the blade rail itself: a tip cannot be ground to nothing, so it is cut back to a thickness
	// and the point where the blade leaves the opornica is only a theoretical one
	ImGui::SetNextItemWidth(110.0f);
	if (ImGui::InputDouble("ostrze u [m]", &Type_.blade.tip_thickness, 0.001, 0.005, "%.4f"))
	{
		dirty = true;
	}
	ImGui::SameLine();
	ImGui::SetNextItemWidth(110.0f);
	if (ImGui::InputDouble("dziob d [m]", &Type_.blade.nose, 0.001, 0.005, "%.4f"))
	{
		dirty = true;
	}
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip("zestrugana dlugosc od ostrza do punktu, w ktorym iglica ma grubosc u (Koc, tablica 5.1)");
	}
	ImGui::SameLine();
	ImGui::SetNextItemWidth(110.0f);
	if (ImGui::InputDouble("glowka w [m]", &Type_.blade.railtop_width, 0.001, 0.005, "%.4f"))
	{
		dirty = true;
	}

	auto const laid{maj0sted::domain::lay_turnout(maj0sted::domain::geometry::Pose{0.0, 0.0, 1.0, 0.0}, maj0sted::editor::to_domain(Type_, 1))};
	if (false == laid.valid)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.55f, 1.0f), "lista nie da sie polozyc: dlugosci > 0, promienie >= 0");
	}
	else
	{
		if (laid.has_blade)
		{
			ImGui::TextDisabled("iglica: ostrze a %.4f m, dziob do %.4f m (o %+.4f), struganie %.3f m, przyleganie %.4f st.", laid.blade_station, laid.nose_end,
			                    laid.blade_residual, laid.planing_length, laid.blade_angle * 180.0 / std::numbers::pi);
		}
		auto const closed{std::abs(laid.angle_residual) <= 1e-6 && (Type_.length <= 0.0 || std::abs(laid.length_residual) <= 1e-3)};
		ImGui::TextColored(closed ? ImVec4(0.6f, 0.9f, 0.6f, 1.0f) : ImVec4(1.0f, 0.75f, 0.45f, 1.0f), "domkniecie: kat %+.5f st., dlugosc %+.4f m",
		                   laid.angle_residual * 180.0 / std::numbers::pi, laid.length_residual);
	}

	if (dirty)
	{
		solve();
	}
	ImGui::Separator();
}

// the catalogue drawing of the chosen type: the diverging path piece by piece in the colours of its
// roles, both tracks' rails, and the construction points - the same list the solver hands the map,
// so the two cannot drift apart. a rozjazd is thirty metres long and a metre and a half wide, so as
// on every real drawing the view across the track is stretched, and by how much is said on screen.
// the blade's own dimensions are millimetres; they come out under the wheel, which magnifies about
// the cursor
void plan_panel::render_template_window()
{
	if (false == m_show_template)
	{
		return;
	}
	ImGui::SetNextWindowSize(ImVec2(1020.0f, 580.0f), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin("Rysunek szablonu rozjazdu", &m_show_template))
	{
		ImGui::End();
		return;
	}
	if (m_document.turnout_types.empty())
	{
		ImGui::TextDisabled("katalog jest pusty");
		ImGui::End();
		return;
	}

	m_turnout_type = std::clamp(m_turnout_type, 0, static_cast<int>(m_document.turnout_types.size()) - 1);
	auto const &type{m_document.turnout_types[static_cast<std::size_t>(m_turnout_type)]};
	auto const start{maj0sted::domain::geometry::Pose{0.0, 0.0, 1.0, 0.0}};
	auto const bend{m_tpl_bend_radius > 1.0 ? (m_tpl_bend_hand == 0 ? 1.0 : -1.0) / m_tpl_bend_radius : 0.0};
	auto const laid{maj0sted::domain::lay_turnout(start, maj0sted::editor::to_domain(type, 1), bend)};
	if (false == laid.valid)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.55f, 1.0f), bend != 0.0 ? "typu \"%s\" nie da sie wygiac na taki luk" : "typ \"%s\" nie da sie polozyc",
		                   type.name.c_str());
		ImGui::SetNextItemWidth(150.0f);
		ImGui::InputDouble("R toru zasadniczego", &m_tpl_bend_radius, 10.0, 100.0, "%.3f");
		ImGui::End();
		return;
	}

	// what the drawing says in words, above what it says in lines
	ImGui::Text("%s      skos 1:%.4g   alfa %s      dlugosc katalogowa %.3f m", type.name.c_str(), type.crossing_n,
	            angle_text(std::atan(1.0 / std::max(type.crossing_n, 1e-9))).c_str(), type.length);
	ImGui::TextDisabled("styczne a %.3f + b %.3f m       na torze zasadniczym %.3f m       odgalezienie %.3f m", laid.tangent_front, laid.tangent_back,
	                    laid.through_length, laid.diverging_length);
	if (laid.has_blade)
	{
		ImGui::TextDisabled("iglica: u %.4f  d %.4f  w %.3f m       ostrze a %.4f m   dziob u %.4f m (o %+.4f)   struganie %.3f m   przyleganie %s",
		                    type.blade.tip_thickness, type.blade.nose, type.blade.railtop_width, laid.blade_station, laid.nose_end, laid.blade_residual,
		                    laid.planing_length, angle_text(laid.blade_angle).c_str());
	}
	else
	{
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.45f, 1.0f), "ten typ nie ma iglicy - zaden odcinek nie ma roli \"iglica\"");
	}
	auto const closed{std::abs(laid.angle_residual) <= 1e-6 && (type.length <= 0.0 || std::abs(laid.length_residual) <= 1e-3)};
	ImGui::TextColored(closed ? ImVec4(0.6f, 0.9f, 0.6f, 1.0f) : ImVec4(1.0f, 0.75f, 0.45f, 1.0f), "domkniecie: kat %+.5f st., dlugosc %+.4f m",
	                   laid.angle_residual * 180.0 / std::numbers::pi, laid.length_residual);

	// lukowanie: the drawing of a bent turnout is the drawing of the same turnout, rotated about its
	// centre - the tangents and the crossing angle do not move, only the radii
	ImGui::SetNextItemWidth(150.0f);
	if (ImGui::InputDouble("R zasadniczego (0 = prosty)", &m_tpl_bend_radius, 10.0, 100.0, "%.3f"))
	{
		m_tpl_bend_radius = std::max(0.0, m_tpl_bend_radius);
	}
	ImGui::SameLine();
	ImGui::SetNextItemWidth(110.0f);
	ImGui::Combo("wygiety", &m_tpl_bend_hand, "w lewo\0w prawo\0");
	ImGui::TextDisabled("%s", bend_text(laid.bend, laid.diverging_curvature, laid.bend_angle).c_str());
	if (auto const *limit{bend_limit(laid.diverging_curvature)}; limit != nullptr)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.45f, 1.0f), "%s", limit);
	}

	ImGui::SetNextItemWidth(180.0f);
	ImGui::SliderFloat("przewyzszenie", &m_tpl_exagg, 1.0f, 60.0f, "x%.0f");
	ImGui::SameLine();
	ImGui::Checkbox("szyny", &m_tpl_rails);
	ImGui::SameLine();
	ImGui::Checkbox("punkty", &m_tpl_marks);
	ImGui::SameLine();
	if (ImGui::SmallButton("dopasuj"))
	{
		m_tpl_zoom = 1.0f;
		m_tpl_pan = 0.0f;
	}
	ImGui::SameLine();
	ImGui::TextDisabled("kolko powieksza, przeciaganie przesuwa");

	auto const avail{ImGui::GetContentRegionAvail()};
	auto const w{std::max(avail.x, 120.0f)};
	auto const h{std::max(avail.y, 120.0f)};
	auto const origin{ImGui::GetCursorScreenPos()};
	ImGui::InvisibleButton("rysunek", ImVec2(w, h));

	auto const span{std::max(laid.through_length, laid.tangent_front + laid.tangent_back) + 2.0};
	auto const scale_of{[&](float const Zoom) { return static_cast<double>(w) * 0.94 / span * static_cast<double>(Zoom); }};
	auto sx{scale_of(m_tpl_zoom)};
	auto centre{span * 0.5 + static_cast<double>(m_tpl_pan)};

	// the wheel magnifies about the cursor, so the blade can be run down to millimetres
	if (ImGui::IsItemHovered() && ImGui::GetIO().MouseWheel != 0.0f)
	{
		auto const offset{static_cast<double>(ImGui::GetIO().MousePos.x - origin.x - w * 0.5f)};
		auto const under{centre + offset / sx};
		m_tpl_zoom = std::clamp(m_tpl_zoom * std::pow(1.25f, ImGui::GetIO().MouseWheel), 1.0f, 5000.0f);
		sx = scale_of(m_tpl_zoom);
		centre = under - offset / sx;
		m_tpl_pan = static_cast<float>(centre - span * 0.5);
	}
	if (ImGui::IsItemActive() && ImGui::IsMouseDown(0))
	{
		m_tpl_pan -= static_cast<float>(static_cast<double>(ImGui::GetIO().MouseDelta.x) / sx);
		centre = span * 0.5 + static_cast<double>(m_tpl_pan);
	}

	auto const sy{sx * static_cast<double>(m_tpl_exagg)};
	auto const to_screen{[&](double const X, double const Y) {
		return ImVec2(origin.x + w * 0.5f + static_cast<float>((X - centre) * sx), origin.y + h * 0.58f - static_cast<float>(Y * sy));
	}};

	auto *drawlist{ImGui::GetWindowDrawList()};
	drawlist->PushClipRect(origin, ImVec2(origin.x + w, origin.y + h), true);
	drawlist->AddRectFilled(origin, ImVec2(origin.x + w, origin.y + h), IM_COL32(16, 20, 26, 255));

	auto const half{maj0sted::domain::kHalfGauge};

	// the tor zasadniczy, and the opornica the blade lies against. bent, it is an arc like any other,
	// so it is walked rather than ruled
	auto const through_at{[&](double const Station_) { return maj0sted::domain::geometry::layout_segment(bend, bend, Station_, start, nullptr); }};
	auto const through_line{[&](double const From_, double const To_, double const Offset_) {
		auto const steps{bend == 0.0 ? 1 : 96};
		std::vector<ImVec2> line;
		line.reserve(static_cast<std::size_t>(steps) + 1);
		for (int i = 0; i <= steps; ++i)
		{
			auto const pose{through_at(From_ + (To_ - From_) * static_cast<double>(i) / static_cast<double>(steps))};
			line.push_back(to_screen(pose.x - pose.hy * Offset_, pose.y + pose.hx * Offset_));
		}
		return line;
	}};

	{
		auto const axis{through_line(-1.0, span, 0.0)};
		drawlist->AddPolyline(axis.data(), static_cast<int>(axis.size()), IM_COL32(105, 115, 130, 220), false, 1.0f);
		if (m_tpl_rails)
		{
			for (int side = -1; side <= 1; side += 2)
			{
				auto const rail{through_line(-1.0, span, half * side)};
				drawlist->AddPolyline(rail.data(), static_cast<int>(rail.size()), IM_COL32(150, 160, 175, 210), false, 1.5f);
			}
		}
	}

	// the diverging path, in the colours of the roles: przediglicowy, iglica, luk, krzyzownicowa.
	// everything up to the punkt A is dashed, because none of it is rail the wheel can run on: before
	// the ostrze there is no blade at all, and from the ostrze to A the blade is the straight planed
	// dziob, not the arc. from A on the blade lies on the arc and the line goes solid
	static ImU32 const part_colour[]{IM_COL32(150, 175, 210, 255), IM_COL32(120, 235, 255, 255), IM_COL32(240, 240, 240, 255), IM_COL32(255, 200, 110, 255)};
	auto const hand{laid.frog.y >= 0.0 ? 1 : -1};
	auto const rail_line{[&](std::vector<maj0sted::domain::geometry::XY> const &Points_, int const Side_) {
		std::vector<ImVec2> rail;
		rail.reserve(Points_.size());
		for (std::size_t i = 0; i < Points_.size(); ++i)
		{
			auto const &back{Points_[i == 0 ? 0 : i - 1]};
			auto const &ahead{Points_[i == 0 ? 1 : i]};
			auto const dx{ahead.x - back.x};
			auto const dy{ahead.y - back.y};
			auto const len{std::hypot(dx, dy)};
			if (len < 1e-12)
			{
				continue;
			}
			rail.push_back(to_screen(Points_[i].x - dy / len * half * Side_, Points_[i].y + dx / len * half * Side_));
		}
		return rail;
	}};

	// the sampling step is metres and the dziob is centimetres, so the two stretches have to part on
	// the exact distance rather than on whichever sample happens to fall nearest it
	auto const cut_at{[](std::vector<maj0sted::domain::geometry::XY> const &Points_, double const Distance_,
	                     std::vector<maj0sted::domain::geometry::XY> &Head_, std::vector<maj0sted::domain::geometry::XY> &Tail_) {
		auto travelled{0.0};
		for (std::size_t i = 0; i < Points_.size(); ++i)
		{
			auto const step{i == 0 ? 0.0 : std::hypot(Points_[i].x - Points_[i - 1].x, Points_[i].y - Points_[i - 1].y)};
			travelled += step;
			if (travelled <= Distance_)
			{
				Head_.push_back(Points_[i]);
				continue;
			}
			if (Tail_.empty() && false == Head_.empty() && step > 1e-12)
			{
				auto const &back{Points_[i - 1]};
				auto const share{(Distance_ - (travelled - step)) / step};
				maj0sted::domain::geometry::XY const meeting{back.x + (Points_[i].x - back.x) * share, back.y + (Points_[i].y - back.y) * share};
				Head_.push_back(meeting);
				Tail_.push_back(meeting);
			}
			Tail_.push_back(Points_[i]);
		}
	}};

	auto const draw_run{[&](std::vector<maj0sted::domain::geometry::XY> const &Points_, ImU32 const Colour_, bool const Theoretical_) {
		if (Points_.size() < 2)
		{
			return;
		}
		std::vector<ImVec2> line;
		line.reserve(Points_.size());
		for (auto const &point : Points_)
		{
			line.push_back(to_screen(point.x, point.y));
		}
		if (Theoretical_)
		{
			dashed_polyline(drawlist, line, (Colour_ & 0x00ffffffu) | 0xb0000000u, 1.6f);
		}
		else
		{
			drawlist->AddPolyline(line.data(), static_cast<int>(line.size()), Colour_, false, 2.5f);
		}
		if (false == m_tpl_rails)
		{
			return;
		}
		for (int side = -1; side <= 1; side += 2)
		{
			auto const rail{rail_line(Points_, side)};
			if (rail.size() < 2)
			{
				continue;
			}
			auto const shade{(Colour_ & 0x00ffffffu) | 0x88000000u};
			if (Theoretical_)
			{
				dashed_polyline(drawlist, rail, shade, 1.0f, 5.0f, 4.0f);
			}
			else
			{
				drawlist->AddPolyline(rail.data(), static_cast<int>(rail.size()), shade, false, 1.2f);
			}
		}
	}};

	auto walker{start};
	auto station{0.0};
	auto heel{0.0};
	for (auto const &segment : laid.path)
	{
		if (segment.turn_in != 0.0)
		{
			auto const c{std::cos(segment.turn_in)};
			auto const s{std::sin(segment.turn_in)};
			walker = {walker.x, walker.y, walker.hx * c - walker.hy * s, walker.hx * s + walker.hy * c};
		}
		std::vector<maj0sted::domain::geometry::XY> points;
		walker = maj0sted::domain::geometry::layout_segment(segment.k0, segment.k1, segment.length, walker, &points);
		auto const from{station};
		station += segment.length;
		if (segment.part == maj0sted::domain::TurnoutPart::Blade)
		{
			heel = station;
		}
		if (points.size() < 2)
		{
			continue;
		}
		auto const colour{part_colour[std::clamp(static_cast<int>(segment.part), 0, 3)]};

		// nothing is real before A; a straight blade is its own true shape, so it stays whole
		auto const bent{segment.k0 != 0.0 || segment.k1 != 0.0};
		auto const upto{laid.has_blade && bent ? std::clamp(laid.nose_end - from, 0.0, segment.length) : 0.0};
		if (upto <= 0.0)
		{
			draw_run(points, colour, false);
			continue;
		}
		std::vector<maj0sted::domain::geometry::XY> ahead;
		std::vector<maj0sted::domain::geometry::XY> behind;
		cut_at(points, upto, behind, ahead);
		draw_run(behind, colour, true);
		draw_run(ahead, colour, false);
	}

	// the dziob as it is really cut: one straight planed rail lying on the opornica at the ostrze - the
	// gap there is nothing at all - and opening to u at the punkt A, where it meets the arc. the blade
	// is the rail on the far side from the way the turnout leads, so it peels off the stock rail the
	// diverging track leaves behind
	if (laid.has_blade && laid.nose_end > laid.blade_station)
	{
		auto const blade_side{-hand};
		auto const blade_colour{part_colour[1]};
		auto const foot_pose{through_at(laid.blade_station)};
		auto const tip{to_screen(foot_pose.x - foot_pose.hy * half * blade_side, foot_pose.y + foot_pose.hx * half * blade_side)};
		auto const nose{to_screen(laid.blade_nose_end.x - laid.blade_nose_end.hy * half * blade_side, laid.blade_nose_end.y + laid.blade_nose_end.hx * half * blade_side)};
		if (m_tpl_rails)
		{
			drawlist->AddLine(tip, nose, (blade_colour & 0x00ffffffu) | 0xee000000u, 2.4f);
		}

		// the same straight said on the axis: half a gauge in from the face, so it leaves the through
		// axis at nothing and meets the arc where the face meets the opornica at u
		drawlist->AddLine(to_screen(foot_pose.x, foot_pose.y), to_screen(laid.blade_nose_end.x, laid.blade_nose_end.y), blade_colour, 2.5f);
		if (std::hypot(nose.x - tip.x, nose.y - tip.y) >= 26.0f)
		{
			char label[64];
			std::snprintf(label, sizeof(label), "dziob prosty d %.4f m, od stycznosci do u", laid.nose_end - laid.blade_station);
			drawlist->AddText(ImVec2((tip.x + nose.x) * 0.5f - ImGui::CalcTextSize(label).x * 0.5f, std::min(tip.y, nose.y) - 20.0f), blade_colour, label);
		}

		// the other blade of the pair: the straight one, which carries the through route. its running
		// edge is the tor zasadniczy's own rail, so it is drawn over it; what is planed is its back,
		// from the ostrze lying on the opornica lukowa down to the rail at A
		auto const straight{through_line(laid.blade_station, heel, half * hand)};
		drawlist->AddPolyline(straight.data(), static_cast<int>(straight.size()), (blade_colour & 0x00ffffffu) | 0xdd000000u, false, 2.4f);
		auto const foot{straight.front()};
		auto const shoulder{straight.back()};
		auto const nose_pose{through_at(laid.nose_end)};
		drawlist->AddLine(to_screen(laid.blade_tip.x - laid.blade_tip.hy * half * hand, laid.blade_tip.y + laid.blade_tip.hx * half * hand),
		                  to_screen(nose_pose.x - nose_pose.hy * half * hand, nose_pose.y + nose_pose.hx * half * hand),
		                  (blade_colour & 0x00ffffffu) | 0xdd000000u, 2.0f);
		if (shoulder.x - foot.x >= 60.0f)
		{
			char label[64];
			std::snprintf(label, sizeof(label), "iglica prosta - jazda na wprost");
			drawlist->AddText(ImVec2((foot.x + shoulder.x) * 0.5f - ImGui::CalcTextSize(label).x * 0.5f, foot.y - (hand > 0 ? 20.0f : -8.0f)), blade_colour, label);
		}
	}

	// the construction points, from the very list the solver reports for a placement
	if (m_tpl_marks)
	{
		auto marks{maj0sted::editor::turnout_marks(laid, start)};
		std::sort(marks.begin(), marks.end(), [](auto const &Left_, auto const &Right_) { return Left_.station < Right_.station; });

		auto level{0};
		for (auto const &mark : marks)
		{
			if (mark.kind < 0 || mark.kind >= static_cast<int>(sizeof(mark_names) / sizeof(mark_names[0])))
			{
				continue;
			}
			auto const at{to_screen(mark.x, mark.y)};
			auto const blade{mark.kind == 1 || mark.kind == 7 || mark.kind == 8};
			auto const primary{mark.kind == 0 || mark.kind == 5 || mark.kind == 9};
			auto const colour{primary ? IM_COL32(255, 210, 90, 245) : (blade ? IM_COL32(120, 235, 255, 245) : IM_COL32(255, 150, 210, 220))};

			// a tick across the track, then the name on a leader fanned out above it
			drawlist->AddLine(ImVec2(at.x, at.y - 11.0f), ImVec2(at.x, at.y + 11.0f), colour, 1.5f);

			auto const reach{34.0f + 16.0f * static_cast<float>(level % 5)};
			ImVec2 const anchor(at.x, origin.y + h * 0.58f - reach - 40.0f);
			char label[96];
			if (mark.station > 0.0)
			{
				std::snprintf(label, sizeof(label), "%s  %.4f", mark_names[mark.kind], mark.station);
			}
			else
			{
				std::snprintf(label, sizeof(label), "%s", mark_names[mark.kind]);
			}
			auto const size{ImGui::CalcTextSize(label)};
			ImVec2 const corner(anchor.x - size.x * 0.5f, anchor.y - size.y * 0.5f);
			drawlist->AddLine(ImVec2(at.x, at.y - 11.0f), ImVec2(anchor.x, corner.y + size.y + 2.0f), IM_COL32(190, 190, 190, 110), 1.0f);
			drawlist->AddRectFilled(ImVec2(corner.x - 3.0f, corner.y - 2.0f), ImVec2(corner.x + size.x + 3.0f, corner.y + size.y + 2.0f),
			                        IM_COL32(12, 16, 24, 215), 3.0f);
			drawlist->AddText(corner, colour, label);
			++level;
		}
	}

	// what the two line styles mean, since the difference between them is the whole point of the drawing
	{
		auto const left{origin.x + 14.0f};
		auto top{origin.y + 12.0f};
		auto const step{ImGui::GetTextLineHeight() + 3.0f};
		std::vector<ImVec2> const sample{ImVec2(left, top + step * 0.5f), ImVec2(left + 34.0f, top + step * 0.5f)};
		dashed_polyline(drawlist, sample, IM_COL32(200, 210, 230, 200), 1.6f);
		drawlist->AddText(ImVec2(left + 42.0f, top), IM_COL32(200, 210, 230, 220), "teoretyczny luk od PR do punktu A: nie ma tam szyny");
		top += step;
		drawlist->AddLine(ImVec2(left, top + step * 0.5f), ImVec2(left + 34.0f, top + step * 0.5f), IM_COL32(120, 235, 255, 255), 2.5f);
		drawlist->AddText(ImVec2(left + 42.0f, top), IM_COL32(120, 235, 255, 235), "dziob iglicy: prosty, od stycznosci z opornica do grubosci u");
	}

	// a metre bar, because with the view stretched across the track no shape can be trusted by eye
	{
		auto const decades{std::pow(10.0, std::floor(std::log10(std::max(span * 0.2, 1e-6))))};
		auto bar{decades};
		while (bar * sx > w * 0.3)
		{
			bar *= 0.5;
		}
		auto const left{origin.x + 14.0f};
		auto const base{origin.y + h - 16.0f};
		auto const right{left + static_cast<float>(bar * sx)};
		drawlist->AddLine(ImVec2(left, base), ImVec2(right, base), IM_COL32(220, 220, 220, 220), 1.5f);
		drawlist->AddLine(ImVec2(left, base - 5.0f), ImVec2(left, base + 5.0f), IM_COL32(220, 220, 220, 220), 1.5f);
		drawlist->AddLine(ImVec2(right, base - 5.0f), ImVec2(right, base + 5.0f), IM_COL32(220, 220, 220, 220), 1.5f);
		char scale_label[128];
		std::snprintf(scale_label, sizeof(scale_label), "%g m wzdluz    poprzecznie x%.0f", bar, m_tpl_exagg);
		drawlist->AddText(ImVec2(right + 8.0f, base - ImGui::GetTextLineHeight() * 0.5f), IM_COL32(220, 220, 220, 220), scale_label);
	}

	drawlist->PopClipRect();
	ImGui::End();
}

void plan_panel::render_turnouts()
{
	if (false == ImGui::CollapsingHeader("Rozjazdy"))
	{
		return;
	}

	if (m_document.turnout_types.empty())
	{
		ImGui::TextDisabled("katalog jest pusty");
		return;
	}

	m_turnout_type = std::clamp(m_turnout_type, 0, static_cast<int>(m_document.turnout_types.size()) - 1);
	auto &chosen{m_document.turnout_types[static_cast<std::size_t>(m_turnout_type)]};
	ImGui::SetNextItemWidth(260.0f);
	if (ImGui::BeginCombo("Typ", chosen.name.c_str()))
	{
		for (std::size_t i = 0; i < m_document.turnout_types.size(); ++i)
		{
			if (ImGui::Selectable(m_document.turnout_types[i].name.c_str(), static_cast<int>(i) == m_turnout_type))
			{
				m_turnout_type = static_cast<int>(i);
			}
		}
		ImGui::EndCombo();
	}

	// a document carries its own copy of the catalogue, so a plan saved before the figure
	// changed keeps the numbers it was saved with - and a type laid before the iglica was a
	// piece of its own has no blade in it at all. this puts the current figure back
	if (auto const *preset{find_preset(chosen.name)}; preset != nullptr)
	{
		if (ImGui::SmallButton("przywroc z katalogu"))
		{
			adopt_preset(*preset, chosen);
			m_sel_piece = -1;
			m_status = "typ \"" + chosen.name + "\" wziety z katalogu na nowo";
			solve();
		}
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("wpisuje na nowo liczby z wbudowanego katalogu: odcinek przediglicowy, iglice, luk, prosta krzyzownicowa oraz u/d/w iglicy");
		}
		ImGui::SameLine();
	}
	if (ImGui::SmallButton(m_show_template ? "zamknij rysunek" : "rysunek szablonu"))
	{
		m_show_template = !m_show_template;
	}

	render_turnout_type(chosen);

	if (ImGui::Button(m_pick_turnout == 1 ? "kliknij tor..." : "Postaw rozjazd"))
	{
		m_pick_turnout = m_pick_turnout == 1 ? 0 : 1;
		m_status = m_pick_turnout == 1 ? "kliknij tor, na ktorym ma stanac rozjazd" : "";
	}

	ImGui::Separator();
	for (std::size_t i = 0; i < m_document.turnouts.size(); ++i)
	{
		auto &placement{m_document.turnouts[i]};
		ImGui::PushID(static_cast<int>(1000 + i));

		auto const *solved{maj0sted::editor::find_turnout(m_solution, placement.id)};
		auto const *on{maj0sted::editor::find_track(m_document, placement.on)};
		char label[200];
		std::snprintf(label, sizeof(label), "%s na %s, km %.1f%s", placement.type.c_str(), on != nullptr ? on->name.c_str() : "?", placement.station,
		              (solved != nullptr && solved->valid) ? "" : "  (nie da sie zlozyc)");
		if (ImGui::Selectable(label, placement.id == m_sel_turnout))
		{
			m_sel_turnout = placement.id;
		}

		if (placement.id == m_sel_turnout)
		{
			ImGui::Indent();
			auto dirty{false};
			ImGui::SetNextItemWidth(130.0f);
			if (ImGui::InputDouble("km", &placement.station, 1.0, 10.0, "%.2f"))
			{
				dirty = true;
			}
			int hand{placement.hand >= 0 ? 0 : 1};
			ImGui::SetNextItemWidth(130.0f);
			if (ImGui::Combo("odgalezia", &hand, "w lewo\0w prawo\0"))
			{
				placement.hand = hand == 0 ? 1 : -1;
				dirty = true;
			}
			if (ImGui::Checkbox("zwrotny w kierunku km", &placement.facing))
			{
				dirty = true;
			}
			// lukowanie: standing on a curve is the ordinary way to get a rozjazd lukowy, and the
			// solver takes the radius from under the PR. asking for one by hand is for a rozjazd
			// bent on a track drawn straight
			if (ImGui::Checkbox("wygiecie z toru pod rozjazdem", &placement.bend_from_track))
			{
				dirty = true;
			}
			if (false == placement.bend_from_track)
			{
				auto radius{placement.bend != 0.0 ? std::abs(1.0 / placement.bend) : 0.0};
				int way{placement.bend >= 0.0 ? 0 : 1};
				ImGui::SetNextItemWidth(130.0f);
				auto touched{ImGui::InputDouble("R zasadniczego", &radius, 10.0, 100.0, "%.3f")};
				ImGui::SameLine();
				ImGui::SetNextItemWidth(110.0f);
				touched = ImGui::Combo("wygiety", &way, "w lewo\0w prawo\0") || touched;
				if (touched)
				{
					placement.bend = radius > 1.0 ? (way == 0 ? 1.0 : -1.0) / radius : 0.0;
					dirty = true;
				}
			}
			if (solved != nullptr && solved->valid)
			{
				ImGui::TextDisabled("styczne a %.3f / b %.3f m (razem %.3f), na torze zasadniczym %.3f m", solved->tangent_front, solved->tangent_back,
				                    solved->tangent_front + solved->tangent_back, solved->through_length);
				ImGui::TextDisabled("odgalezienie %.3f m", solved->diverging_length);
				ImGui::TextDisabled("%s", bend_text(solved->bend, solved->diverging_curvature, solved->bend_angle).c_str());
				if (auto const *limit{bend_limit(solved->diverging_curvature)}; limit != nullptr)
				{
					ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.45f, 1.0f), "%s", limit);
				}
				if (solved->has_blade)
				{
					ImGui::TextDisabled("iglica: ostrze a %.4f m, dziob do %.4f m, struganie %.3f m", solved->blade_station, solved->nose_end, solved->planing_length);
				}
			}
			// a rozjazd left standing across a joint, or off the end, needs a way back that does not
			// mean guessing kilometres by hand: the nearest element long enough to hold it, whole
			if (solved != nullptr && solved->valid)
			{
				auto const *through{maj0sted::editor::find_track(m_solution, placement.on)};
				double curvature{0.0};
				double behind{0.0};
				double ahead{0.0};
				auto const room{through != nullptr && maj0sted::editor::steady_room(*through, placement.station, curvature, behind, ahead)
				                    ? (placement.facing ? ahead : behind)
				                    : -1.0};
				if (through != nullptr && room < solved->through_length - 1e-6)
				{
					ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.45f, 1.0f), "rozjazd nie miesci sie na odcinku toru, na ktorym stoi");
					ImGui::SameLine();
					if (ImGui::SmallButton("wstaw na odcinek"))
					{
						double station{0.0};
						auto best{std::numeric_limits<double>::max()};
						auto walked{0.0};
						for (auto const &element : through->elements)
						{
							if (std::abs(element.k1 - element.k0) < 1e-12 && element.length >= solved->through_length)
							{
								// anywhere on this straight with the whole rozjazd still on it
								auto const low{placement.facing ? walked : walked + solved->through_length};
								auto const high{placement.facing ? walked + element.length - solved->through_length : walked + element.length};
								auto const put{std::clamp(placement.station, low, high)};
								if (std::abs(put - placement.station) < best)
								{
									best = std::abs(put - placement.station);
									station = put;
								}
							}
							walked += element.length;
						}
						if (best < std::numeric_limits<double>::max())
						{
							placement.station = station;
							dirty = true;
						}
						else
						{
							m_status = "zaden odcinek tego toru nie jest dosc dlugi na ten rozjazd";
						}
					}
				}
			}
			if (ImGui::SmallButton("usun rozjazd"))
			{
				m_document.turnouts.erase(m_document.turnouts.begin() + static_cast<long>(i));
				m_sel_turnout = TurnoutId::none;
				ImGui::Unindent();
				ImGui::PopID();
				solve();
				return;
			}
			ImGui::Unindent();
			if (dirty)
			{
				solve();
			}
		}
		ImGui::PopID();
	}
}

void plan_panel::render_diagnostics()
{
	if (m_solution.diagnostics.empty())
	{
		return;
	}

	// a fit that does not work says why, with the figure that matters in it. it used to come back as
	// nullopt and simply disappear from the drawing
	char header[64];
	std::snprintf(header, sizeof(header), "Nie wyszlo (%zu)###diag", m_solution.diagnostics.size());
	if (false == ImGui::CollapsingHeader(header, ImGuiTreeNodeFlags_DefaultOpen))
	{
		return;
	}

	for (std::size_t i = 0; i < m_solution.diagnostics.size(); ++i)
	{
		auto const &diagnostic{m_solution.diagnostics[i]};
		ImGui::PushID(static_cast<int>(2000 + i));
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.60f, 0.55f, 1.0f));
		auto const *track{maj0sted::editor::find_track(m_document, diagnostic.track)};
		if (ImGui::Selectable(diagnostic.text.c_str()))
		{
			if (diagnostic.track != TrackId::none)
			{
				m_track = diagnostic.track;
			}
			if (diagnostic.element != ElementId::none)
			{
				m_sel_element = diagnostic.element;
			}
			if (diagnostic.turnout != TurnoutId::none)
			{
				m_sel_turnout = diagnostic.turnout;
			}
		}
		ImGui::PopStyleColor();
		if (track != nullptr)
		{
			ImGui::SameLine();
			ImGui::TextDisabled("(%s)", track->name.c_str());
		}
		ImGui::PopID();
	}
}

void plan_panel::render_storage()
{
	ImGui::Separator();
	ImGui::SetNextItemWidth(240.0f);
	ImGui::InputText("##path", m_path, sizeof(m_path));
	ImGui::SameLine();
	if (ImGui::Button("Save"))
	{
		double planx{0.0};
		double plany{0.0};
		world_to_plan(editor_mode::get_camera().Pos, planx, plany);
		m_document.view_x = planx;
		m_document.view_y = plany;
		m_document.view_extent = static_cast<double>(Global.editor_ortho_extent);
		m_document.origin_set = true;
		m_document.georeferenced = Global.scenery_georeferenced;
		m_document.origin_x = Global.scenery_origin.x;
		m_document.origin_y = Global.scenery_origin.y;
		m_status = maj0sted::io::save(m_document, m_path) ? "saved to " + std::string(m_path) : "could not write " + std::string(m_path);
	}
	ImGui::SameLine();
	if (ImGui::Button("Load"))
	{
		auto const loaded{maj0sted::io::load(m_path)};
		if (loaded)
		{
			m_document = *loaded;
			seed_catalogue();
			if (m_document.tracks.empty())
			{
				new_track();
			}
			else
			{
				m_track = m_document.tracks.front().id;
				std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", m_document.tracks.front().name.c_str());
			}
			if (m_document.origin_set)
			{
				Global.scenery_georeferenced = m_document.georeferenced;
				Global.scenery_origin = {m_document.origin_x, m_document.origin_y};
			}
			m_document.origin_set = true;
			m_document.georeferenced = Global.scenery_georeferenced;
			m_document.origin_x = Global.scenery_origin.x;
			m_document.origin_y = Global.scenery_origin.y;
			m_pending = false;
			m_pick_join = 0;
			m_join_first = TrackId::none;
			clear_selection();
			solve();
			if (m_document.view_extent > 0.0)
			{
				auto &camera{editor_mode::get_camera()};
				auto const centre{plan_to_world(m_document.view_x, m_document.view_y)};
				camera.Pos.x = centre.x;
				camera.Pos.z = centre.z;
				Global.editor_ortho_extent = std::clamp(static_cast<float>(m_document.view_extent), 5.0f, 20000.0f);
			}
			else
			{
				go_to_plan();
			}
			if (false == m_document.scn_path.empty())
			{
				std::snprintf(m_scn_path, sizeof(m_scn_path), "%s", m_document.scn_path.c_str());
			}
			m_status = "loaded " + std::string(m_path);
		}
		else
		{
			// version 1 files are not read: they described a different model
			m_status = "could not read " + std::string(m_path) + " (potrzebny format m0s 2)";
		}
	}
	ImGui::SetNextItemWidth(240.0f);
	ImGui::InputText("##scnpath", m_scn_path, sizeof(m_scn_path));
	ImGui::SameLine();
	if (ImGui::Button("Export SCN"))
	{
		export_scn(false);
	}
	ImGui::SameLine();
	if (ImGui::Button("Export i uruchom"))
	{
		export_scn(true);
	}
	ImGui::Checkbox("uruchom w edytorze", &m_scn_run_editor);
	ImGui::SameLine();
	// the trainset is what the simulator looks for a player train in: without one it loads the
	// scenery and drops straight back out, so driving needs it and looking around does not
	ImGui::Checkbox("z pociagiem", &m_scn_trainset);
	if (false == m_status.empty())
	{
		ImGui::TextDisabled("%s", m_status.c_str());
	}
	for (auto const &warning : m_scn_warnings)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "%s", warning.c_str());
	}
}

void plan_panel::export_scn(bool const Run_)
{
	solve();
	m_scn_warnings.clear();
	// where a plan exports to belongs to the plan: say it once and it is there the next time the
	// project is opened, whichever MaSzyna it was drawn for
	m_document.scn_path = m_scn_path;
	maj0sted::io::ScnExportOptions opt;
	opt.origin_east = Global.scenery_origin.x;
	opt.origin_north = Global.scenery_origin.y;
	opt.trainset = m_scn_trainset;
	std::ofstream out(m_scn_path, std::ios::binary | std::ios::trunc);
	if (!out)
	{
		m_status = "could not write " + std::string(m_scn_path);
		return;
	}
	auto const result{maj0sted::io::export_scn(m_document, m_solution, opt, out)};
	out.flush();
	if (false == out.good())
	{
		m_status = "could not write " + std::string(m_scn_path);
		return;
	}
	out.close();
	m_status = "wyeksportowano " + std::to_string(result.tracks) + " odcinków i " + std::to_string(result.switches) + " zwrotnic";
	m_scn_warnings = result.warnings;
	if (false == m_scn_warnings.empty())
	{
		m_status += ", " + std::to_string(m_scn_warnings.size()) + " uwag";
	}
	if (Run_)
	{
		run_scn();
	}
}

void plan_panel::run_scn()
{
	// the simulator names a scenery relative to scenery/, so that is what is handed over rather than
	// the path the file was written to
	std::string scenery{m_scn_path};
	std::string root;
	auto const slash{scenery.find_last_of("/\\")};
	if (slash != std::string::npos)
	{
		// the game is wherever the scenery folder is: exporting into another MaSzyna's scenery and
		// then running ours out of this directory would find the file and none of the models
		auto const folder{scenery.substr(0, slash)};
		auto const parent{folder.find_last_of("/\\")};
		root = parent != std::string::npos ? folder.substr(0, parent) : std::string{};
		scenery = scenery.substr(slash + 1);
	}

	std::vector<std::string> arguments{"-s", scenery};
	if (m_scn_run_editor)
	{
		arguments.push_back("-editor");
	}
	if (spawn_detached(executable_path(), arguments, root))
	{
		m_status += "; uruchomiono " + scenery + (root.empty() ? "" : " w " + root);
	}
	else
	{
		m_status += "; nie udało się uruchomić symulatora";
	}
}

// ---------------------------------------------------------------------------
// map dialogs
// ---------------------------------------------------------------------------

void plan_panel::render_newmap_dialog()
{
	if (false == ImGui::BeginPopupModal("New map", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
	{
		return;
	}

	ImGui::TextUnformatted("What is this scenery going to be?");
	ImGui::Spacing();

	if (ImGui::Button("Fictional", ImVec2(160.0f, 0.0f)))
	{
		start_map(false, 0.0, 0.0);
		ImGui::CloseCurrentPopup();
	}
	ImGui::SameLine();
	ImGui::TextDisabled("coordinates mean nothing outside the scenery");

	if (ImGui::Button("Real place", ImVec2(160.0f, 0.0f)))
	{
		// start the picker on the middle of the country, showing the whole of it
		m_pickingplace = true;
		m_pickx = 500000.0;
		m_picky = 350000.0;
		m_mapviewx = m_pickx;
		m_mapviewy = m_picky;
		m_mapscale = 0.0;
		ImGui::CloseCurrentPopup();
		ImGui::OpenPopup("Scenery location");
	}
	ImGui::SameLine();
	ImGui::TextDisabled("pin the scenery's zero to a point in Poland");

	ImGui::Spacing();
	if (ImGui::Button("Cancel"))
	{
		ImGui::CloseCurrentPopup();
	}

	ImGui::EndPopup();
}

void plan_panel::render_location_dialog()
{
	if (m_pickingplace)
	{
		// the popup has to be opened from the same stack level it is drawn on
		ImGui::OpenPopup("Scenery location");
		m_pickingplace = false;
	}

	if (false == ImGui::BeginPopupModal("Scenery location", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
	{
		return;
	}

	// the request is one square image, so the view is square too and the pixels map one to one
	int const mappixels = 512;
	ImVec2 const mapsize{static_cast<float>(mappixels), static_cast<float>(mappixels)};
	auto const origin{ImGui::GetCursorScreenPos()};
	ImGui::InvisibleButton("poland_map", mapsize);
	auto const hovered{ImGui::IsItemHovered()};

	// fit the whole country the first time round; afterwards the wheel decides
	if (m_mapscale <= 0.0)
	{
		glm::dvec2 min;
		glm::dvec2 max;
		editor::poland_extent(min, max);
		m_mapviewx = (min.x + max.x) * 0.5;
		m_mapviewy = (min.y + max.y) * 0.5;
		m_mapscale = mapsize.x / std::max({max.x - min.x, max.y - min.y, 1.0});
	}

	ImVec2 const centre{origin.x + mapsize.x * 0.5f, origin.y + mapsize.y * 0.5f};
	auto const to_screen = [&](double const X, double const Y) {
		return ImVec2{centre.x + static_cast<float>((X - m_mapviewx) * m_mapscale), centre.y - static_cast<float>((Y - m_mapviewy) * m_mapscale)};
	};
	auto const to_map = [&](ImVec2 const &Point, double &X, double &Y) {
		X = m_mapviewx + (Point.x - centre.x) / m_mapscale;
		Y = m_mapviewy - (Point.y - centre.y) / m_mapscale;
	};

	auto const mouse{ImGui::GetIO().MousePos};
	if (hovered && ImGui::GetIO().MouseWheel != 0.0f)
	{
		double anchorx{0.0};
		double anchory{0.0};
		to_map(mouse, anchorx, anchory);
		m_mapscale = std::clamp(m_mapscale * std::pow(1.2, ImGui::GetIO().MouseWheel), 1e-5, 1.0);
		m_mapviewx = anchorx - (mouse.x - centre.x) / m_mapscale;
		m_mapviewy = anchory + (mouse.y - centre.y) / m_mapscale;
	}
	if (hovered && ImGui::IsMouseClicked(0))
	{
		to_map(mouse, m_pickx, m_picky);
	}

	auto *drawlist{ImGui::GetWindowDrawList()};
	auto const mapend{ImVec2(origin.x + mapsize.x, origin.y + mapsize.y)};
	drawlist->PushClipRect(origin, mapend, true);
	drawlist->AddRectFilled(origin, mapend, IM_COL32(22, 30, 38, 255));

	// the topographic base map behind everything, asked for at exactly the box on screen
	double viewminx{0.0};
	double viewminy{0.0};
	double viewmaxx{0.0};
	double viewmaxy{0.0};
	to_map(origin, viewminx, viewmaxy);
	to_map(mapend, viewmaxx, viewminy);
	auto const backdrop{m_topomap.texture_for({viewminx, viewminy, viewmaxx, viewmaxy}, mappixels)};
	if (backdrop != 0)
	{
		// the picture on screen is the one that was fetched, which may lag the current view by a
		// request; drawing it against its own box keeps it registered while the new one arrives
		auto const &covered{m_topomap.covered()};
		drawlist->AddImage(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(backdrop)), to_screen(covered.min_x, covered.max_y), to_screen(covered.max_x, covered.min_y), ImVec2(0.0f, 0.0f),
		                   ImVec2(1.0f, 1.0f));
	}

	auto const picked{to_screen(m_pickx, m_picky)};
	drawlist->AddCircleFilled(picked, 5.0f, IM_COL32(255, 210, 90, 255));
	drawlist->AddLine(ImVec2(picked.x - 12.0f, picked.y), ImVec2(picked.x + 12.0f, picked.y), IM_COL32(255, 210, 90, 200));
	drawlist->AddLine(ImVec2(picked.x, picked.y - 12.0f), ImVec2(picked.x, picked.y + 12.0f), IM_COL32(255, 210, 90, 200));

	drawlist->PopClipRect();

	ImGui::TextDisabled(m_topomap.loading() ? "fetching the topographic map..." : "click to place the scenery's zero, wheel zooms");

	ImGui::SetNextItemWidth(180.0f);
	ImGui::InputDouble("easting", &m_pickx, 100.0, 1000.0, "%.0f");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(180.0f);
	ImGui::InputDouble("northing", &m_picky, 100.0, 1000.0, "%.0f");

	ImGui::Spacing();
	if (ImGui::Button("Use this place", ImVec2(160.0f, 0.0f)))
	{
		start_map(true, m_pickx, m_picky);
		ImGui::CloseCurrentPopup();
	}
	ImGui::SameLine();
	if (ImGui::Button("Cancel"))
	{
		ImGui::CloseCurrentPopup();
	}

	ImGui::EndPopup();
}

void plan_panel::start_map(bool const Georeferenced, double const Originx, double const Originy)
{
	Global.scenery_georeferenced = Georeferenced;
	Global.scenery_origin = {Originx, Originy};

	Document fresh;
	fresh.origin_set = true;
	fresh.georeferenced = Georeferenced;
	fresh.origin_x = Originx;
	fresh.origin_y = Originy;
	m_document = std::move(fresh);
	seed_catalogue();
	new_track();

	m_pick_turnout = 0;
	m_pick_parallel = 0;
	m_pick_join = 0;
	m_join_first = TrackId::none;
	solve();

	// the scenery's zero is where the work starts, whichever frame it stands for
	auto &camera{editor_mode::get_camera()};
	camera.Pos.x = 0.0;
	camera.Pos.z = 0.0;

	// a new map starts on empty ground - whatever scenery was loaded has nothing to do with it
	Global.editor_reset_scenery = true;

	m_status = Georeferenced ? "map pinned at " + to_string(Originx, 0) + ", " + to_string(Originy, 0) + " (EPSG:2180)" : "fictional map";
}

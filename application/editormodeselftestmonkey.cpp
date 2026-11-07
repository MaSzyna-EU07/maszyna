/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// monkey tests: random gestures over the track tools, and the checks of the tracks they're run against

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace
{

// the keys of the tools, the modifiers and the other keys the monkey taps
std::array<char const *, 17> constexpr monkey_tools{"esc", "l", "g", "c", "t", "n", "u", "i", "b", "p", "v", "j", "k", "o", "q", "w", "e"};
std::array<char const *, 4> constexpr monkey_modifiers{"", "ctrl", "shift", "alt"};
std::array<char const *, 6> constexpr monkey_keys{"enter", "backspace", "del", "esc", "tab", "space"};

// a part of the window the gestures go over
struct area
{
	ImVec2 low;
	ImVec2 high;
};

// the random gestures of the mouse and the keys over the 3d view, the tools, their options, the inspector and the profile,
// written as the lines of the script. drawn from the generator seeded as given, in the order they're written, so that a run of
// the test can be repeated
class monkey_script
{
  public:
	explicit monkey_script(unsigned int const Seed);
	// the gestures of a step, the tracks checked every tenth one
	void step(int const Step);
	// the script closed: the tool let go and the tracks checked once more
	std::vector<std::string> const &finish();
	std::size_t panels() const
	{
		return m_panels.size();
	}

  private:
	int pick(int const Count);
	float within(float const Low, float const High);
	ImVec2 in_view();
	ImVec2 somewhere();
	void mouse(ImVec2 const At);
	void click(char const *Button);
	void modified_click();
	void double_click();
	void drag();
	void right_click();
	void wheel();
	void select_track();
	void held_gesture();

	unsigned int m_seed;
	std::mt19937 m_random;
	std::vector<area> m_panels;
	glm::vec2 m_view_low{};
	glm::vec2 m_view_high{};
	std::vector<TTrack const *> m_tracks;
	std::vector<std::string> m_lines;
};

monkey_script::monkey_script(unsigned int const Seed) : m_seed{Seed}, m_random{Seed} // NOSONAR: seeded on purpose, so that a run of the test can be repeated; nothing secret is drawn
{
	for (auto const *name : {"##editortoolbar", "###tooloptions", "###inspector", "###profilestrip"})
		if (auto const *window{ImGui::FindWindowByName(name)}; window != nullptr && (window->Active || window->WasActive))
			m_panels.emplace_back(window->Pos, ImVec2(window->Pos.x + window->Size.x, window->Pos.y + window->Size.y));
	m_view_low = editor_ui::view_min();
	m_view_high = editor_ui::view_max();
	for (auto const *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved && false == track->name().empty())
			m_tracks.push_back(track);
}

int monkey_script::pick(int const Count)
{
	return std::uniform_int_distribution<int>(0, Count - 1)(m_random);
}

float monkey_script::within(float const Low, float const High)
{
	return std::uniform_real_distribution<float>(Low, std::max(Low + 1.0f, High))(m_random);
}

ImVec2 monkey_script::in_view()
{
	return ImVec2(within(m_view_low.x + 8.0f, m_view_high.x - 8.0f), within(m_view_low.y + 8.0f, m_view_high.y - 8.0f));
}

ImVec2 monkey_script::somewhere()
{
	if (m_panels.empty() || pick(3) == 0)
		return in_view();
	auto const &panel{m_panels[pick(static_cast<int>(m_panels.size()))]};
	return ImVec2(within(panel.low.x + 4.0f, panel.high.x - 4.0f), within(panel.low.y + 4.0f, panel.high.y - 4.0f));
}

void monkey_script::mouse(ImVec2 const At)
{
	if (At.x >= m_view_low.x && At.x <= m_view_high.x && At.y >= m_view_low.y && At.y <= m_view_high.y && pick(2) == 0)
		m_lines.push_back(format("monkeyat %u", static_cast<unsigned int>(m_random())));
	else
		m_lines.push_back(format("mouse %.0f %.0f", At.x, At.y));
}

void monkey_script::click(char const *Button)
{
	m_lines.push_back(std::string{"button "} + Button + " press");
	m_lines.emplace_back("wait 1");
	m_lines.push_back(std::string{"button "} + Button + " release");
	m_lines.emplace_back("wait 2");
}

// a click, now and then with a modifier held
void monkey_script::modified_click()
{
	auto const modifier{monkey_modifiers[static_cast<std::size_t>(pick(6) < 4 ? 0 : 1 + pick(3))]};
	mouse(somewhere());
	m_lines.emplace_back("wait 1");
	if (*modifier != '\0')
		m_lines.push_back(std::string{"press "} + modifier);
	click("left");
	if (*modifier != '\0')
		m_lines.push_back(std::string{"release "} + modifier);
}

void monkey_script::double_click()
{
	mouse(somewhere());
	m_lines.emplace_back("wait 1");
	click("left");
	click("left");
}

// a drag, in the view or on a panel; now and then with Shift held, a box of tracks
void monkey_script::drag()
{
	auto const from{pick(3) == 0 ? somewhere() : in_view()};
	auto const boxed{pick(4) == 0};
	mouse(from);
	m_lines.emplace_back("wait 1");
	if (boxed)
		m_lines.emplace_back("press shift");
	m_lines.emplace_back("button left press");
	m_lines.emplace_back("wait 2");
	auto const to{ImVec2(from.x + within(-200.0f, 200.0f), from.y + within(-150.0f, 150.0f))};
	for (int part = 1; part <= 4; ++part)
	{
		mouse(ImVec2(from.x + (to.x - from.x) * static_cast<float>(part) / 4.0f, from.y + (to.y - from.y) * static_cast<float>(part) / 4.0f));
		m_lines.emplace_back("wait 1");
	}
	m_lines.emplace_back("button left release");
	if (boxed)
		m_lines.emplace_back("release shift");
	m_lines.emplace_back("wait 2");
}

void monkey_script::right_click()
{
	mouse(in_view());
	m_lines.emplace_back("wait 1");
	click("right");
	if (pick(2) == 0)
		m_lines.emplace_back("tap esc");
}

// the wheel turned a notch, now and then with a modifier held
void monkey_script::wheel()
{
	auto const modifier{monkey_modifiers[static_cast<std::size_t>(pick(4))]};
	mouse(pick(4) == 0 ? somewhere() : in_view());
	m_lines.emplace_back("wait 1");
	if (*modifier != '\0')
		m_lines.push_back(std::string{"press "} + modifier);
	m_lines.push_back(format("wheel %d", pick(2) == 0 ? 1 : -1));
	m_lines.emplace_back("wait 1");
	if (*modifier != '\0')
		m_lines.push_back(std::string{"release "} + modifier);
}

void monkey_script::select_track()
{
	if (false == m_tracks.empty())
		m_lines.push_back("select " + m_tracks[pick(static_cast<int>(m_tracks.size()))]->name());
	m_lines.emplace_back("wait 1");
	// now and then the plan of its line, with the band of the curvature in the panel, or the grade line of the whole line
	if (auto const tool{pick(4)}; tool < 2)
	{
		m_lines.emplace_back(tool == 0 ? "tracktab G" : "tracktab P");
		m_lines.emplace_back("wait 2");
	}
}

// a gesture held a while: the mouse moved with the button down over a few frames, keys tapped meanwhile
void monkey_script::held_gesture()
{
	mouse(in_view());
	m_lines.emplace_back("wait 1");
	m_lines.emplace_back("button left press");
	m_lines.emplace_back("wait 3");
	mouse(in_view());
	m_lines.emplace_back("wait 1");
	m_lines.push_back(std::string{"press "} + (pick(2) == 0 ? "ctrl" : "shift"));
	m_lines.push_back(format("wheel %d", pick(2) == 0 ? 1 : -1));
	m_lines.emplace_back("wait 1");
	m_lines.emplace_back("release ctrl");
	m_lines.emplace_back("release shift");
	mouse(in_view());
	m_lines.emplace_back("wait 1");
	m_lines.emplace_back("button left release");
	m_lines.emplace_back("wait 2");
}

void monkey_script::step(int const Step)
{
	m_lines.push_back(format("log monkey %u step %d", m_seed, Step));
	switch (pick(12))
	{
	case 0:
	case 1:
		modified_click();
		break;
	case 2:
		double_click();
		break;
	case 3:
		drag();
		break;
	case 4:
		right_click();
		break;
	case 5:
		wheel();
		break;
	case 6:
		m_lines.push_back(std::string{"tap "} + monkey_tools[static_cast<std::size_t>(pick(static_cast<int>(monkey_tools.size())))]);
		m_lines.emplace_back("wait 2");
		break;
	case 7:
		m_lines.push_back(std::string{"tap "} + monkey_keys[static_cast<std::size_t>(pick(static_cast<int>(monkey_keys.size())))]);
		m_lines.emplace_back("wait 2");
		break;
	case 8:
		m_lines.emplace_back(pick(3) == 0 ? "redo" : "undo");
		m_lines.emplace_back("wait 2");
		break;
	case 9:
		select_track();
		break;
	case 10:
		m_lines.push_back(format("type %d", pick(500) - 50));
		m_lines.emplace_back("wait 1");
		break;
	default:
		held_gesture();
		break;
	}
	if (Step % 10 == 9)
		m_lines.emplace_back("invariants");
	if (Step % 25 == 0)
		m_lines.push_back(format("monkeyview %u", static_cast<unsigned int>(m_random())));
}

std::vector<std::string> const &monkey_script::finish()
{
	m_lines.emplace_back("tap esc");
	m_lines.emplace_back("wait 2");
	m_lines.emplace_back("invariants");
	return m_lines;
}

// the tracks with a path, one of them seen by the camera or pointed at by the mouse
std::vector<TTrack const *> tracks_with_paths()
{
	std::vector<TTrack const *> tracks;
	for (auto const *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved && false == track->m_paths.empty())
			tracks.push_back(track);
	return tracks;
}

// the point of the first path of the track at the share of its length: an end, the middle or a point between
glm::dvec3 point_along(TTrack const &Track, double const Share)
{
	auto const &path{Track.m_paths.front()};
	auto const &a{path.points[segment_data::point::start]};
	auto const &b{path.points[segment_data::point::end]};
	auto const c1{a + path.points[segment_data::point::control1]};
	auto const c2{b + path.points[segment_data::point::control2]};
	auto const u{1.0 - Share};
	return u * u * u * a + 3.0 * u * u * Share * c1 + 3.0 * u * Share * Share * c2 + Share * Share * Share * b;
}

// one of the shares of the length the mouse is put at, picked at random
double random_share(std::mt19937 &Random)
{
	std::array<double, 7> constexpr shares{0.0, 0.5, 1.0, 0.25, 0.75, 0.1, 0.9};
	return shares[static_cast<std::size_t>(std::uniform_int_distribution<int>(0, 6)(Random))];
}

// the camera over one of the tracks picked at random, looking at it from 30..120 m; the track it's over
TTrack const &view_random_track(std::vector<TTrack const *> const &Tracks, std::mt19937 &Random, TCamera &View)
{
	auto const &track{*Tracks[std::uniform_int_distribution<std::size_t>(0, Tracks.size() - 1)(Random)]};
	auto const target{point_along(track, 0.5)};
	auto const distance{std::uniform_real_distribution<double>(30.0, 120.0)(Random)};
	auto const angle{std::uniform_real_distribution<double>(0.0, 6.2831853)(Random)};
	glm::dvec3 const from{target.x + std::cos(angle) * distance * 0.7, target.y + distance * 0.6, target.z + std::sin(angle) * distance * 0.7};
	auto const look{glm::normalize(target - from)};
	View.Pos = from;
	View.Angle = glm::vec3(static_cast<float>(std::asin(std::clamp(look.y, -1.0, 1.0))), static_cast<float>(std::atan2(-look.x, -look.z)), 0.0f);
	Global.pCamera = View;
	return track;
}

// the mouse on a track seen in the view, at an end, the middle or a point between: the tracks tried in a random order, the
// middle of the view with none of them seen
glm::dvec2 point_on_track_seen(std::vector<TTrack const *> &Tracks, std::mt19937 &Random)
{
	screen_projection const projection;
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	std::ranges::shuffle(Tracks, Random); // NOSONAR: the seeded generator of the test, so the order can be repeated
	for (auto const *track : Tracks)
	{
		ImVec2 at;
		if (false == projection.project(point_along(*track, random_share(Random)), at) || at.x < low.x + 4.0f || at.y < low.y + 4.0f || at.x > high.x - 4.0f || at.y > high.y - 4.0f)
			continue;
		return glm::dvec2{at.x, at.y};
	}
	return glm::dvec2{(low.x + high.x) * 0.5f, (low.y + high.y) * 0.5f};
}

// a track as it's hashed: its name and its points to the millimetre
std::string hashed_track(TTrack const &Track)
{
	std::string entry{Track.name()};
	for (auto const &path : Track.m_paths)
		for (auto const &point : path.points)
			entry += format(" %.3f %.3f %.3f", point.x, point.y, point.z);
	return entry;
}

// the line of the log with the number of the tracks and a hash of their names and points to the millimetre
std::string track_hash()
{
	std::vector<std::string> entries;
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		entries.push_back(hashed_track(*track));
	}
	std::ranges::sort(entries);
	std::uint64_t hash{1469598103934665603ULL};
	for (auto const &entry : entries)
		for (auto const character : entry)
			hash = (hash ^ static_cast<unsigned char>(character)) * 1099511628211ULL;
	return format("SELFTEST trackhash %zu tracks %016llx", entries.size(), static_cast<unsigned long long>(hash));
}

bool finite_point(glm::dvec3 const &Point)
{
	return std::isfinite(Point.x) && std::isfinite(Point.y) && std::isfinite(Point.z);
}

// what isn't right with the track: points, rolls or a length that aren't finite, or no path; empty with nothing
std::string track_fault(TTrack const &Track)
{
	std::string fault;
	if (Track.m_paths.empty())
		fault = "no path";
	for (auto const &path : Track.m_paths)
	{
		for (auto const &point : path.points)
			if (false == finite_point(point))
				fault = "point not finite";
		if (false == std::isfinite(path.rolls[0]) || false == std::isfinite(path.rolls[1]))
			fault = "roll not finite";
	}
	if (false == std::isfinite(Track.Length()) || Track.Length() < 0.0)
		fault = format("length %f", Track.Length());
	return fault;
}

// the text of the log with the number of the tracks, the history and the frame rate, and the first ten of the tracks that
// aren't right
std::string invariants_report(std::size_t const History)
{
	std::size_t count{0};
	std::vector<std::string> bad;
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		++count;
		if (auto const fault{track_fault(*track)}; false == fault.empty())
			bad.push_back((track->name().empty() ? std::string{"(noname)"} : track->name()) + ": " + fault);
	}
	auto text{format("SELFTEST invariants: %zu tracks, %zu bad, history %zu, %.0f fps", count, bad.size(), History, ImGui::GetIO().Framerate)};
	for (std::size_t i = 0; i < bad.size() && i < 10; ++i)
		text += "\n  " + bad[i];
	return text;
}

} // namespace

//   monkey <seed> <steps>               random gestures of the mouse and the keys over the 3d view, the tools, their options,
//                                        the inspector and the profile, written into the script after this line; the tracks
//                                        are checked every tenth step
//   monkeyview <seed>                    the camera over a track picked at random, looking at it from 30..120 m
//   monkeyat <seed>                      the mouse on a track seen in the view, at an end, the middle or a point between
//   invariants                           logs the tracks whose points, rolls or length aren't finite, or that have no path
//   undoall                              undoes every change in the history
//   trackhash                            logs the number of the tracks and a hash of their names and points to the millimetre
//   tutorial <page>                      opens the tutorial at the lesson, from 0; -1 closes it
//   badge [<x> <y> <gesture>]            the mouse or the keys drawn with the bottom left corner at the point of the window, as
//                                        "LMB" or "Enter", for the clips of the tutorial; with nothing after it none
//   shortcuts [filter]                   opens the list of the shortcuts
editor_mode::selftest_result editor_mode::selftest_monkey(std::string const &command, std::istringstream &words)
{
	if (command == "monkey")
		return selftest_monkey_run(words);
	if (command == "monkeyview" || command == "monkeyat")
		return selftest_monkey_view(command == "monkeyview", words);
	if (command == "undoall")
		return selftest_undoall();
	if (command == "trackhash")
	{
		WriteLog(track_hash());
		return selftest_result::handled;
	}
	if (command == "tutorial")
	{
		int page{0};
		words >> page;
		m_first_steps_open = page >= 0;
		m_tutorial_page = std::max(0, page);
		return selftest_result::handled;
	}
	if (command == "badge")
	{
		std::string gesture;
		words >> m_selftest.badge_at.x >> m_selftest.badge_at.y;
		std::getline(words >> std::ws, gesture);
		m_selftest.badge = gesture;
		return selftest_result::handled;
	}
	if (command == "shortcuts")
	{
		m_shortcuts_open = true;
		return selftest_result::handled;
	}
	if (command == "invariants")
	{
		WriteLog(invariants_report(m_history.size()));
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_monkey_run(std::istringstream &words)
{
	unsigned int seed{1};
	int steps{100};
	words >> seed >> steps;
	monkey_script script{seed};
	for (int step = 0; step < steps; ++step)
		script.step(step);
	auto const &lines{script.finish()};
	WriteLog(format("SELFTEST monkey %u: %d steps, %zu lines, %zu panels", seed, steps, lines.size(), script.panels()));
	m_selftest.lines.insert(m_selftest.lines.begin() + static_cast<std::ptrdiff_t>(m_selftest.next), lines.begin(), lines.end());
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_monkey_view(bool const View, std::istringstream &words)
{
	unsigned int seed{1};
	words >> seed;
	std::mt19937 random{seed}; // NOSONAR: seeded on purpose, so that a run of the test can be repeated; nothing secret is drawn
	auto tracks{tracks_with_paths()};
	if (tracks.empty())
		return selftest_result::handled;
	if (View)
	{
		auto const &track{view_random_track(tracks, random, Camera)};
		m_focus_active = false;
		WriteLog("SELFTEST monkeyview " + (track.name().empty() ? std::string{"(noname)"} : track.name()));
		return selftest_result::handled;
	}
	m_selftest.input.mouse_to(point_on_track_seen(tracks, random));
	return selftest_result::handled;
}

editor_mode::selftest_result editor_mode::selftest_undoall()
{
	std::size_t count{0};
	while (false == m_history.empty() && count < 10000)
	{
		auto const before{m_history.size()};
		undo_last();
		++count;
		if (m_history.size() >= before)
			break;
	}
	WriteLog(format("SELFTEST undoall: %zu undone, %zu left", count, m_history.size()));
	return selftest_result::handled;
}

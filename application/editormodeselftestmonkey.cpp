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
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

//   monkey <seed> <steps>                random gestures of the mouse and the keys over the 3d view, the tools, their options,
//                                        the inspector and the profile, written into the script after this line; the tracks
//                                        are checked every tenth step
//   monkeyview <seed>                    the camera over a track picked at random, looking at it from 30..120 m
//   monkeyat <seed>                      the mouse on a track seen in the view, at an end, the middle or a point between
//   invariants                           logs the tracks whose points, rolls or length aren't finite, or that have no path
//   undoall                              undoes every change in the history
//   trackhash                            logs the number of the tracks and a hash of their names and points to the millimetre
//   tutorial <page>                      opens the tutorial at the lesson, from 0; -1 closes it
//   shortcuts [filter]                   opens the list of the shortcuts
editor_mode::selftest_result editor_mode::selftest_monkey(std::string const &command, std::istringstream &words)
{
	if (command == "monkey")
	{
		unsigned int seed{1};
		int steps{100};
		words >> seed >> steps;
		std::mt19937 random{seed};
		auto const pick = [&](int const Count) { return std::uniform_int_distribution<int>(0, Count - 1)(random); };
		auto const within = [&](float const Low, float const High) { return std::uniform_real_distribution<float>(Low, std::max(Low + 1.0f, High))(random); };
		struct area
		{
			ImVec2 low, high;
		};
		std::vector<area> panels;
		for (auto const *name : {"##editortoolbar", "###tooloptions", "###inspector", "###profilestrip"})
			if (auto const *window{ImGui::FindWindowByName(name)}; window != nullptr && (window->Active || window->WasActive))
				panels.push_back({window->Pos, ImVec2(window->Pos.x + window->Size.x, window->Pos.y + window->Size.y)});
		auto const view_low{editor_ui::view_min()};
		auto const view_high{editor_ui::view_max()};
		auto const in_view = [&]() { return ImVec2(within(view_low.x + 8.0f, view_high.x - 8.0f), within(view_low.y + 8.0f, view_high.y - 8.0f)); };
		auto const somewhere = [&]() {
			if (panels.empty() || pick(3) == 0)
				return in_view();
			auto const &panel{panels[pick(static_cast<int>(panels.size()))]};
			return ImVec2(within(panel.low.x + 4.0f, panel.high.x - 4.0f), within(panel.low.y + 4.0f, panel.high.y - 4.0f));
		};
		std::vector<TTrack *> tracks;
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && false == track->m_editorremoved && false == track->name().empty())
				tracks.push_back(track);
		std::vector<std::string> lines;
		auto const mouse = [&](ImVec2 const At) {
			if (At.x >= view_low.x && At.x <= view_high.x && At.y >= view_low.y && At.y <= view_high.y && pick(2) == 0)
				lines.push_back(format("monkeyat %u", static_cast<unsigned int>(random())));
			else
				lines.push_back(format("mouse %.0f %.0f", At.x, At.y));
		};
		auto const click = [&](char const *Button) {
			lines.push_back(std::string{"button "} + Button + " press");
			lines.emplace_back("wait 1");
			lines.push_back(std::string{"button "} + Button + " release");
			lines.emplace_back("wait 2");
		};
		char const *const tools[]{"esc", "l", "g", "c", "t", "n", "u", "h", "b", "p", "v", "j", "k", "o", "q", "w", "e"};
		char const *const modifiers[]{"", "ctrl", "shift", "alt"};
		for (int step = 0; step < steps; ++step)
		{
			lines.push_back(format("log monkey %u step %d", seed, step));
			switch (pick(12))
			{
			case 0:
			case 1:
			{
				// a click, now and then with a modifier held
				auto const modifier{modifiers[pick(6) < 4 ? 0 : 1 + pick(3)]};
				mouse(somewhere());
				lines.emplace_back("wait 1");
				if (*modifier != '\0')
					lines.push_back(std::string{"press "} + modifier);
				click("left");
				if (*modifier != '\0')
					lines.push_back(std::string{"release "} + modifier);
				break;
			}
			case 2:
				mouse(somewhere());
				lines.emplace_back("wait 1");
				click("left");
				click("left");
				break;
			case 3:
			{
				// a drag, in the view or on a panel; now and then with Shift held, a box of tracks
				auto const from{pick(3) == 0 ? somewhere() : in_view()};
				auto const boxed{pick(4) == 0};
				mouse(from);
				lines.emplace_back("wait 1");
				if (boxed)
					lines.emplace_back("press shift");
				lines.emplace_back("button left press");
				lines.emplace_back("wait 2");
				auto const to{ImVec2(from.x + within(-200.0f, 200.0f), from.y + within(-150.0f, 150.0f))};
				for (int part = 1; part <= 4; ++part)
				{
					mouse(ImVec2(from.x + (to.x - from.x) * part / 4.0f, from.y + (to.y - from.y) * part / 4.0f));
					lines.emplace_back("wait 1");
				}
				lines.emplace_back("button left release");
				if (boxed)
					lines.emplace_back("release shift");
				lines.emplace_back("wait 2");
				break;
			}
			case 4:
				mouse(in_view());
				lines.emplace_back("wait 1");
				click("right");
				if (pick(2) == 0)
					lines.emplace_back("tap esc");
				break;
			case 5:
			{
				auto const modifier{modifiers[pick(4)]};
				mouse(pick(4) == 0 ? somewhere() : in_view());
				lines.emplace_back("wait 1");
				if (*modifier != '\0')
					lines.push_back(std::string{"press "} + modifier);
				lines.push_back(format("wheel %d", pick(2) == 0 ? 1 : -1));
				lines.emplace_back("wait 1");
				if (*modifier != '\0')
					lines.push_back(std::string{"release "} + modifier);
				break;
			}
			case 6:
				lines.push_back(std::string{"tap "} + tools[pick(static_cast<int>(std::size(tools)))]);
				lines.emplace_back("wait 2");
				break;
			case 7:
			{
				char const *const keys[]{"enter", "backspace", "del", "esc", "tab", "space"};
				lines.push_back(std::string{"tap "} + keys[pick(static_cast<int>(std::size(keys)))]);
				lines.emplace_back("wait 2");
				break;
			}
			case 8:
				lines.emplace_back(pick(3) == 0 ? "redo" : "undo");
				lines.emplace_back("wait 2");
				break;
			case 9:
				if (false == tracks.empty())
					lines.push_back("select " + tracks[pick(static_cast<int>(tracks.size()))]->name());
				lines.emplace_back("wait 1");
				// now and then the plan of its line, with the band of the curvature in the panel, or the grade line of the
				// whole line
				if (auto const tool{pick(4)}; tool < 2)
				{
					lines.emplace_back(tool == 0 ? "tracktab G" : "tracktab P");
					lines.emplace_back("wait 2");
				}
				break;
			case 10:
				lines.push_back(format("type %d", pick(500) - 50));
				lines.emplace_back("wait 1");
				break;
			default:
				// a gesture held a while: the mouse moved with the button down over a few frames, keys tapped meanwhile
				mouse(in_view());
				lines.emplace_back("wait 1");
				lines.emplace_back("button left press");
				lines.emplace_back("wait 3");
				mouse(in_view());
				lines.emplace_back("wait 1");
				lines.push_back(std::string{"press "} + (pick(2) == 0 ? "ctrl" : "shift"));
				lines.push_back(format("wheel %d", pick(2) == 0 ? 1 : -1));
				lines.emplace_back("wait 1");
				lines.emplace_back("release ctrl");
				lines.emplace_back("release shift");
				mouse(in_view());
				lines.emplace_back("wait 1");
				lines.emplace_back("button left release");
				lines.emplace_back("wait 2");
				break;
			}
			if (step % 10 == 9)
				lines.emplace_back("invariants");
			if (step % 25 == 0)
				lines.push_back(format("monkeyview %u", static_cast<unsigned int>(random())));
		}
		lines.emplace_back("tap esc");
		lines.emplace_back("wait 2");
		lines.emplace_back("invariants");
		WriteLog(format("SELFTEST monkey %u: %d steps, %zu lines, %zu panels", seed, steps, lines.size(), panels.size()));
		m_selftest.lines.insert(m_selftest.lines.begin() + static_cast<std::ptrdiff_t>(m_selftest.next), lines.begin(), lines.end());
		return selftest_result::handled;
	}
	if (command == "monkeyview" || command == "monkeyat")
	{
		unsigned int seed{1};
		words >> seed;
		std::mt19937 random{seed};
		std::vector<TTrack *> tracks;
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && false == track->m_editorremoved && false == track->m_paths.empty())
				tracks.push_back(track);
		if (tracks.empty())
			return selftest_result::handled;
		auto const share = [&]() {
			double const shares[]{0.0, 0.5, 1.0, 0.25, 0.75, 0.1, 0.9};
			return shares[std::uniform_int_distribution<int>(0, 6)(random)];
		};
		auto const along = [&](TTrack const &Track, double const Share) {
			auto const &path{Track.m_paths.front()};
			auto const &a{path.points[segment_data::point::start]};
			auto const &b{path.points[segment_data::point::end]};
			auto const c1{a + path.points[segment_data::point::control1]};
			auto const c2{b + path.points[segment_data::point::control2]};
			auto const u{1.0 - Share};
			return u * u * u * a + 3.0 * u * u * Share * c1 + 3.0 * u * Share * Share * c2 + Share * Share * Share * b;
		};
		if (command == "monkeyview")
		{
			auto const &track{*tracks[std::uniform_int_distribution<std::size_t>(0, tracks.size() - 1)(random)]};
			auto const target{along(track, 0.5)};
			auto const distance{std::uniform_real_distribution<double>(30.0, 120.0)(random)};
			auto const angle{std::uniform_real_distribution<double>(0.0, 6.2831853)(random)};
			glm::dvec3 const from{target.x + std::cos(angle) * distance * 0.7, target.y + distance * 0.6, target.z + std::sin(angle) * distance * 0.7};
			auto const look{glm::normalize(target - from)};
			Camera.Pos = from;
			Camera.Angle = glm::vec3(static_cast<float>(std::asin(std::clamp(look.y, -1.0, 1.0))), static_cast<float>(std::atan2(-look.x, -look.z)), 0.0f);
			Global.pCamera = Camera;
			m_focus_active = false;
			WriteLog("SELFTEST monkeyview " + (track.name().empty() ? std::string{"(noname)"} : track.name()));
			return selftest_result::handled;
		}
		screen_projection const projection;
		auto const low{editor_ui::view_min()};
		auto const high{editor_ui::view_max()};
		std::shuffle(tracks.begin(), tracks.end(), random);
		for (auto const *track : tracks)
		{
			ImVec2 at;
			if (false == projection.project(along(*track, share()), at) || at.x < low.x + 4.0f || at.y < low.y + 4.0f || at.x > high.x - 4.0f || at.y > high.y - 4.0f)
				continue;
			m_selftest.input.mouse_to(glm::dvec2{at.x, at.y});
			return selftest_result::handled;
		}
		m_selftest.input.mouse_to(glm::dvec2{(low.x + high.x) * 0.5f, (low.y + high.y) * 0.5f});
		return selftest_result::handled;
	}
	if (command == "undoall")
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
	if (command == "trackhash")
	{
		std::vector<std::string> entries;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved)
				continue;
			std::string entry{track->name()};
			for (auto const &path : track->m_paths)
				for (auto const &point : path.points)
					entry += format(" %.3f %.3f %.3f", point.x, point.y, point.z);
			entries.push_back(std::move(entry));
		}
		std::sort(entries.begin(), entries.end());
		std::uint64_t hash{1469598103934665603ull};
		for (auto const &entry : entries)
			for (auto const character : entry)
				hash = (hash ^ static_cast<unsigned char>(character)) * 1099511628211ull;
		WriteLog(format("SELFTEST trackhash %zu tracks %016llx", entries.size(), static_cast<unsigned long long>(hash)));
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
	if (command == "shortcuts")
	{
		m_shortcuts_open = true;
		return selftest_result::handled;
	}
	if (command == "invariants")
	{
		std::size_t count{0};
		std::vector<std::string> bad;
		auto const finite = [](glm::dvec3 const &Point) { return std::isfinite(Point.x) && std::isfinite(Point.y) && std::isfinite(Point.z); };
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved)
				continue;
			++count;
			std::string fault;
			if (track->m_paths.empty())
				fault = "no path";
			for (auto const &path : track->m_paths)
			{
				for (auto const &point : path.points)
					if (false == finite(point))
						fault = "point not finite";
				if (false == std::isfinite(path.rolls[0]) || false == std::isfinite(path.rolls[1]))
					fault = "roll not finite";
			}
			if (false == std::isfinite(track->Length()) || track->Length() < 0.0)
				fault = format("length %f", track->Length());
			if (false == fault.empty())
				bad.push_back((track->name().empty() ? std::string{"(noname)"} : track->name()) + ": " + fault);
		}
		auto text{format("SELFTEST invariants: %zu tracks, %zu bad, history %zu, %.0f fps", count, bad.size(), m_history.size(), ImGui::GetIO().Framerate)};
		for (std::size_t i = 0; i < bad.size() && i < 10; ++i)
			text += "\n  " + bad[i];
		WriteLog(text);
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

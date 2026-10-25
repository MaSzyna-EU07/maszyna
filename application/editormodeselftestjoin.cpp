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
#include <sstream>
#include <array>
#include "application/editormodeselftest.h"

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
				a.y = b.y = height;
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
		char const *const kinds[]{"none", "straight", "arc", "curve", "reverse curve", "free curve"};
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
				for (auto const &path : track->m_paths)
					for (auto const index : {segment_data::point::start, segment_data::point::end})
						if (auto const distance{geometry::plan_distance(path.points[index], given)}; distance < nearest)
						{
							nearest = distance;
							point = path.points[index];
						}
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
		for (auto const *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved)
				continue;
			for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
				for (auto const atend : {false, true})
				{
					auto const &path{track->m_paths[i]};
					if (geometry::plan_distance(path.points[atend ? segment_data::point::end : segment_data::point::start], point) > 0.05)
						continue;
					geometry::bezier const curve{path};
					auto const first{curve.first(atend ? 1.0 : 0.0)};
					auto const second{curve.second(atend ? 1.0 : 0.0)};
					auto const speed{std::hypot(first.x, first.z)};
					if (speed < 1e-9)
						continue;
					auto const curvature{(first.x * second.z - first.z * second.x) / (speed * speed * speed)};
					glm::dvec2 const along{first.x / speed, first.z / speed};
					meetings.push_back({track, i, atend, atend ? -curvature : curvature, static_cast<double>(atend ? path.rolls[1] : path.rolls[0]), atend ? -along : along});
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
			auto const bed = [&](std::vector<glm::dvec3> const &Points, std::vector<glm::dvec2> const &) {
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
		EditorSettings.heights(which == "points" ? editorSettings::lay_heights::points : which == "level" ? editorSettings::lay_heights::level : editorSettings::lay_heights::fitted);
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
		Point.y = found[0] != 0 ? ground[0] : m_lay.points.empty() ? 0.0 : m_lay.points.back().y;
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
			auto text{format("SELFTEST layplan vertex %zu: %s at %.3f %.3f", k, plan.corners[k] != 0 ? "bend" : "traced", k < plan.shape.vertices.size() ? plan.shape.vertices[k].x : 0.0, k < plan.shape.vertices.size() ? plan.shape.vertices[k].y : 0.0)};
			for (auto const &curve : plan.shape.curves)
				if (curve.vertex == static_cast<int>(k))
				{
					text += format(", %s R %.1f", curve.spirals ? "spirals" : "arc", curve.radius);
					for (auto const radius : curve.radii)
						text += format("/%.1f", radius);
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
	if (command == "layradius")
	{
		double low{3000.0};
		double high{6000.0};
		words >> low >> high;
		EditorSettings.corners(low, high);
		WriteLog(format("SELFTEST layradius: %.0f - %.0f m", EditorSettings.corner_low(), EditorSettings.corner_high()));
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
				if (chainage >= next - 1e-6)
				{
					auto const now{angle((i + 1) * step)};
					along.push_back(position);
					across.push_back({-std::sin(now), std::cos(now)});
					next += spacing;
				}
			}
			heading = angle(length);
		}
		glm::dvec2 const out{std::cos(heading), std::sin(heading)};
		glm::dvec2 const side{-out.y, out.x};
		auto const shifted = [&](glm::dvec2 const &Point, glm::dvec2 const &Across, int const Count) {
			auto const shift{noise * (Count % 3 == 0 ? 1.0 : Count % 3 == 1 ? -0.6 : 0.2)};
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
			for (auto i{along.size()}; i-- > 0;)
				if (glm::distance(along[i], glm::dvec2{end.position.x, end.position.z}) > 3.5)
					lay_point(shifted(along[i], across[i], static_cast<int>(i)));
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
		auto const end{editor_track::find_free_end(nullptr, 1, around, 60.0, {})};
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
		WriteLog(format("SELFTEST %s: %zu points, chain %zu, history %zu, %s", command.c_str(), m_lay.points.size(), m_lay.chain.size(), m_history.size(), m_lay.status.c_str()));
		return selftest_result::handled;
	}
	// a line of known curves, turning by the deflections given (degrees, to the left positive), the points along its curves clicked as
	// they would be on the photo: the start as a bend, then in the mode of a curve from 30 m before the first curve to the end of the
	// last one and two on the straight after it, every Spacing metres, shifted across by up to Noise
	if (command == "laysample")
	{
		double x{0.0}, z{0.0}, heading{0.0}, spacing{20.0}, noise{0.0}, deflection{30.0}, radius{1000.0};
		words >> x >> z >> heading >> spacing >> noise >> deflection >> radius;
		double apart{0.0}, second{0.0}, secondradius{0.0};
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
			vertex.transition_in = vertex.transition_out = recommended.transition;
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
		int count{0};
		for (auto chainage{std::max(0.0, from - 30.0)}; chainage <= to; chainage += spacing, ++count)
			lay_point(along(chainage, noise * (count % 3 == 0 ? 1.0 : count % 3 == 1 ? -0.6 : 0.2)));
		lay_point(along(to + 30.0, noise * 0.5));
		lay_point(along(to + 60.0, -noise * 0.5));
		lay_switch_mode();
		WriteLog(format("SELFTEST laysample: %zu points clicked over %.1f m of curves", m_lay.points.size(), to - from));
		lay_report("sample");
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
			WriteLog(format("SELFTEST laychain %s: chain %zu, points %zu, start %s%s at %.2f %.2f, history %zu, status %s", Step.c_str(), m_lay.chain.size(), m_lay.points.size(), start != nullptr ? start->name().c_str() : "-", start != nullptr && start->m_editorremoved ? " (removed)" : "",
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
			point.y = found[0] != 0 ? ground[0] : m_lay.points.empty() ? 0.0 : m_lay.points.back().y;
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
				std::replace(readout.begin(), readout.end(), '\n', '|');
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
				if (track != nullptr && false == track->m_editorremoved && std::find(laid.begin(), laid.end(), track) == laid.end())
					laid.push_back(track);
		if (false == laid.empty())
		{
			auto const origin{laid.front()->m_paths.front().points[segment_data::point::start]};
			auto const along{glm::normalize(geometry::plan_of(laid.back()->m_paths.front().points[segment_data::point::end] - origin) + glm::dvec2{1e-9, 0.0})};
			std::stable_sort(laid.begin(), laid.end(), [&](TTrack const *A, TTrack const *B) {
				return glm::dot(geometry::plan_of(A->m_paths.front().points[segment_data::point::start] - origin), along) < glm::dot(geometry::plan_of(B->m_paths.front().points[segment_data::point::start] - origin), along);
			});
		}
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
			auto const count{std::max(1, static_cast<int>(std::ceil(curve.plan_length() / 2.0)))};
			for (int k = 0; k <= count; ++k)
			{
				auto const t{curve.parameter(curve.plan_length() * k / count)};
				auto const target{bed_heights({curve.point(t) + glm::dvec3{0.0, 50.0, 0.0}}, {geometry::plan_of(curve.first(t))}, track).front()};
				if (std::isnan(target))
					continue;
				hanging = std::max(hanging, curve.point(t).y - target);
				buried = std::max(buried, target - curve.point(t).y);
			}
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
		if (action == "set")
		{
			m_track_set.clear();
			for (std::string name; words >> name;)
				if (auto *each{simulation::Paths.find(name)}; each != nullptr)
					m_track_set.push_back(each);
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
			m_gizmo_op = operation == "rotate" ? gizmo_operation::rotate : operation == "scale" ? gizmo_operation::scale : gizmo_operation::translate;
			// as the key of the operation does, the gizmo hidden in the tracks is brought out
			m_track_gizmo_on = true;
			int path{-1};
			std::string kind;
			words >> path >> kind;
			m_track_point = {};
			if (path >= 0)
				m_track_point = {path, kind == "c1" ? editor_track::point_kind::control1 : kind == "c2" ? editor_track::point_kind::control2 : kind == "end" ? editor_track::point_kind::end : editor_track::point_kind::start};
		}
		else if (action == "begin")
		{
			std::string operation;
			std::string kind;
			int path{-1};
			words >> operation >> path >> kind;
			m_track_point = {};
			if (path >= 0)
				m_track_point = {path, kind == "c1" ? editor_track::point_kind::control1 : kind == "c2" ? editor_track::point_kind::control2 : kind == "end" ? editor_track::point_kind::end : editor_track::point_kind::start};
			auto const wanted{operation == "rotate" ? gizmo_operation::rotate : operation == "scale" ? gizmo_operation::scale : gizmo_operation::translate};
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
			m_track_gizmo_drag.ring = axis == "x" ? 0 : axis == "z" ? 2 : 1;
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
			glm::dvec3 control{0.0};
			words >> control.x >> control.y >> control.z;
			track_gizmo_control(*track, control);
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
			std::vector<TTrack *> shown{track};
			for (auto *neighbour : editor_track::neighbours(*track))
				shown.push_back(neighbour);
			WriteLog(format("SELFTEST gizmo %s prev %s next %s", track->name().c_str(), track->trPrev != nullptr ? track->trPrev->name().c_str() : "-", track->trNext != nullptr ? track->trNext->name().c_str() : "-"));
			for (auto *each : shown)
				for (int i = 0; i < static_cast<int>(each->m_paths.size()); ++i)
				{
					auto const &path{each->m_paths[i]};
					auto const &a{path.points[segment_data::point::start]};
					auto const &b{path.points[segment_data::point::end]};
					auto const &c1{path.points[segment_data::point::control1]};
					auto const &c2{path.points[segment_data::point::control2]};
					WriteLog(format("SELFTEST gizmo path %s[%d] %.3f %.3f %.3f -> %.3f %.3f %.3f, c1 %.3f %.3f %.3f, c2 %.3f %.3f %.3f, L %.3f, R %.1f, rolls %.2f %.2f", each->name().c_str(), i, a.x, a.y, a.z, b.x, b.y, b.z, c1.x, c1.y, c1.z, c2.x, c2.y, c2.z,
					                geometry::bezier{path}.plan_length(), path.radius, path.rolls[0], path.rolls[1]));
				}
			for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
				for (auto const kind : {editor_track::point_kind::start, editor_track::point_kind::end})
				{
					geometry::bezier const own{track->m_paths[i]};
					bool const start{kind == editor_track::point_kind::start};
					auto const out{start ? -own.first(0.0) : own.first(1.0)};
					for (auto const &partner : editor_track::partners(*track, {i, kind}))
					{
						geometry::bezier const theirs{partner.first->m_paths[partner.second.path]};
						auto const in{partner.second.kind == editor_track::point_kind::start ? theirs.first(0.0) : -theirs.first(1.0)};
						auto const angle{glm::degrees(std::acos(std::clamp(glm::dot(glm::normalize(out), glm::normalize(in)), -1.0, 1.0)))};
						auto const gap{glm::distance(editor_track::point_position(*track, {i, kind}), editor_track::point_position(*partner.first, partner.second))};
						WriteLog(format("SELFTEST gizmo joint %s[%d].%s - %s[%d]: kink %.4f deg, gap %.4f m", track->name().c_str(), i, start ? "start" : "end", partner.first->name().c_str(), partner.second.path, angle, gap));
					}
				}
		}
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

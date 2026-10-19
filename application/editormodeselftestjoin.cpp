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
#include "application/editormodeselftest.h"

editor_mode::selftest_result editor_mode::selftest_join(std::string const &command, std::istringstream &words)
{
	if (command == "join" || command == "layto")
	{
		glm::dvec3 a{0.0};
		glm::dvec3 b{0.0};
		words >> a.x >> a.z >> b.x >> b.z;
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
			double radius{0.0};
			int variant{0};
			int transitions{0};
			words >> radius >> variant >> transitions;
			auto const to{editor_track::find_free_end(nullptr, 1, b, 3.0, {from.track})};
			if (to.track == nullptr)
			{
				WriteLog("SELFTEST join: no free end at the second point");
				return selftest_result::handled;
			}
			m_join_radius = radius;
			m_join_variant = variant;
			m_lay.transitions = transitions != 0;
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
			else
			{
				glm::dvec3 point{std::strtod(word.c_str(), nullptr), 5000.0, 0.0};
				words >> point.z;
				std::vector<char> found;
				auto const ground{ground_heights({point}, true, &found)};
				point.y = found[0] != 0 ? ground[0] : m_lay.points.empty() ? 0.0 : m_lay.points.back().y;
				m_lay.mouse_snap = editor_track::find_free_end(nullptr, 1, point, 3.0, {});
				m_lay.mouse = m_lay.mouse_snap.track != nullptr ? point : lay_bed(point);
				lay_click();
				word += format(" %.1f", point.z);
			}
			report(word);
		}
		lay_cancel();
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
			lay_click();
			WriteLog(format("SELFTEST layclicks %.1f %.1f: %s", point.x, point.z, m_lay.status.c_str()));
		}
		lay_cancel();
		m_lay.active = active;
		glm::dvec2 heading{0.0};
		glm::dvec3 previous{0.0};
		double slope{0.0};
		bool first{true};
		double hanging{0.0};
		double buried{0.0};
		int paths{0};
		for (auto step{before}; step < m_history.size(); ++step)
			for (auto const *track : m_history[step].created)
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
	if (command == "jointsmodel")
	{
		// the whole scenery against the terrain model, as the check of the panel does it, waiting for the model
		static bool started{false};
		if (false == started)
		{
			double threshold{0.5};
			words >> threshold;
			m_joints.scope = 1;
			m_joints.model_check = true;
			m_joints.model = threshold;
			joints_start();
			started = true;
		}
		joints_step();
		if (m_joints.next < m_joints.queue.size() || m_joints.model_waiting)
		{
			--m_selftest.next;
			return selftest_result::pause;
		}
		started = false;
		std::size_t count{0};
		for (auto const &issue : m_joints.issues)
			if (issue.type == joint_issue::kind::model && count++ < 40)
				WriteLog(format("SELFTEST jointsmodel: %+.2f m at %.1f %.1f, %s path %d", issue.value, issue.position.x, issue.position.z, issue.track != nullptr ? issue.track->name().c_str() : "-", issue.point.path));
		WriteLog("SELFTEST jointsmodel: " + m_joints.status);
		return selftest_result::handled;
	}
	if (command == "joints")
	{
		std::vector<glm::dvec3> points;
		for (glm::dvec3 point{0.0}; words >> point.x >> point.z;)
			points.push_back(point);
		auto const scope{m_joints.scope};
		auto const model{m_joints.model_check};
		m_joints.scope = 1;
		m_joints.model_check = false;
		joints_start();
		while (m_joints.next < m_joints.queue.size())
			joints_step();
		m_joints.scope = scope;
		m_joints.model_check = model;
		char const *const kinds[]{"gap", "step", "kink", "cant jump", "grade break", "off the terrain model", "free end"};
		int around{0};
		for (auto const &issue : m_joints.issues)
		{
			if (false == points.empty() && issue.type == joint_issue::kind::open)
				continue;
			if (false == points.empty() && std::none_of(points.begin(), points.end(), [&](glm::dvec3 const &Point) { return geometry::plan_distance(Point, issue.position) < 5.0; }))
				continue;
			++around;
			auto const name = [](TTrack const *Track) { return Track == nullptr ? std::string{"-"} : Track->name() + (Track->eType == tt_Switch ? "(switch)" : ""); };
			auto const rise{issue.other != nullptr ? editor_track::point_position(*issue.other, issue.theirs).y - issue.position.y : 0.0};
			WriteLog(format("SELFTEST joints: %s %.3f at %.1f %.1f, %s / %s, rise %.3f", kinds[static_cast<int>(issue.type)], issue.value, issue.position.x, issue.position.z, name(issue.track).c_str(), name(issue.other).c_str(), rise));
		}
		WriteLog(format("SELFTEST joints: %d issues around the points, %zu in all", around, m_joints.issues.size()));
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

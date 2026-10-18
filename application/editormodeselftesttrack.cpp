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

editor_mode::selftest_result editor_mode::selftest_track(std::string const &command, std::istringstream &words)
{
	if (command == "probe")
	{
		// edge of the last laid model as it was made: its distance from the axis and the height of its top over the rail head
		std::string name;
		double step{20.0};
		words >> name >> step;
		if (name == "-")
			name.clear();
		auto *sweep{name.empty() ? m_sweep.edited : simulation::Sweeps.find(name)};
		if (sweep == nullptr)
		{
			WriteLog("SELFTEST probe: nothing laid");
			return selftest_result::handled;
		}
		auto const &definition{sweep->definition()};
		double const sign{definition.lateral >= 0.0 ? 1.0 : -1.0};
		struct bin
		{
			double edge{std::numeric_limits<double>::max()};
			std::vector<std::pair<double, double>> points; // lateral, height over the axis
		};
		std::map<int, bin> bins;
		for (auto const &shape : sweep->create_shapes())
			for (auto const &vertex : shape.data().vertices)
			{
				double lateral;
				auto const station{sweep->project(vertex.position, lateral)};
				if (lateral * sign <= 0.0)
					continue;
				auto const axis{sweep->frame_at(station)};
				auto &entry{bins[static_cast<int>(std::floor(station / step))]};
				entry.edge = std::min(entry.edge, std::abs(lateral));
				entry.points.emplace_back(std::abs(lateral), vertex.position.y - axis.position.y);
			}
		WriteLog(format("SELFTEST probe %s: %zu stretches with the model, %.1f - %.1f m of %.1f m", sweep->name().c_str(), bins.size(), sweep->start(), sweep->end(), sweep->length()));
		for (auto const &[index, entry] : bins)
		{
			auto const station{(index + 0.5) * step};
			if (station < sweep->start() || station > sweep->end())
				continue;
			double top{-1000.0};
			for (auto const &point : entry.points)
				if (point.first < entry.edge + 0.15)
					top = std::max(top, point.second);
			auto const at{sweep->frame_at(station)};
			auto const head{0.18 + sweep_node::rail_head(at.roll, definition.lateral)};
			WriteLog(format("SELFTEST probe %s at %.0f m: edge %.3f m from the axis, its top %.3f m over the rail head (cant %.1f deg, gauge widening %.3f m, cant part %.3f m)", sweep->name().c_str(), station, entry.edge, top - head, at.roll,
			                at.widening[definition.lateral >= 0.0 ? 1 : 0], std::max(0.0, at.cant[definition.lateral >= 0.0 ? 1 : 0])));
		}
		return selftest_result::handled;
	}
	if (command == "profile")
	{
		std::string name;
		double station{0.0};
		words >> name >> station;
		auto *sweep{simulation::Sweeps.find(name)};
		if (sweep == nullptr)
		{
			WriteLog("SELFTEST profile: no such sweep");
			return selftest_result::handled;
		}
		auto const &definition{sweep->definition()};
		std::map<int, std::pair<double, double>> columns;
		for (auto const &shape : sweep->create_shapes())
			for (auto const &vertex : shape.data().vertices)
			{
				double lateral;
				auto const at{sweep->project(vertex.position, lateral)};
				if (std::abs(at - station) > 1.0)
					continue;
				auto const axis{sweep->frame_at(at)};
				auto const height{vertex.position.y - axis.position.y - 0.18 - sweep_node::rail_head(axis.roll, definition.lateral)};
				auto &column{columns.try_emplace(static_cast<int>(std::floor(lateral / 0.1)), std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()).first->second};
				column.first = std::min(column.first, height);
				column.second = std::max(column.second, height);
			}
		std::string text;
		for (auto const &[index, column] : columns)
			text += format(" [%.1f %.2f..%.2f]", index * 0.1, column.first, column.second);
		WriteLog(format("SELFTEST profile %s at %.0f m:", name.c_str(), station) + text);
		return selftest_result::handled;
	}
	if (command == "tracks")
	{
		int tab{0};
		words >> tab;
		show_track_tab(track_tab::lineside);
		m_sweep.expand = tab == 0;
		m_hekto.expand = tab == 1;
		m_fouling.expand = tab == 2;
		m_parallel.expand = tab == 3;
		m_vehicle.expand = tab == 4;
		return selftest_result::handled;
	}
	if (command == "curve")
	{
		sweep_curve();
		for (auto const &piece : m_sweep.curve)
		{
			auto const &a{piece.points[segment_data::point::start]};
			auto const &b{piece.points[segment_data::point::end]};
			WriteLog(format("SELFTEST curve piece %.1f %.1f -> %.1f %.1f", a.x, a.z, b.x, b.z));
		}
		if (false == m_sweep.outline.empty())
			WriteLog(format("SELFTEST curve outline %zu points, %.1f %.1f -> %.1f %.1f, %.1f m", m_sweep.outline.size(), m_sweep.outline.front().x, m_sweep.outline.front().z, m_sweep.outline.back().x, m_sweep.outline.back().z, m_sweep.curve_length));
		return selftest_result::handled;
	}
	if (command == "lay")
	{
		std::string name;
		std::string side;
		words >> name >> side;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr || track->m_paths.empty())
		{
			WriteLog("SELFTEST lay: no track " + name);
			return selftest_result::pause;
		}
		auto const &path{track->m_paths.front()};
		auto const atend{side == "end"};
		editor_track::snap_target start;
		start.track = track;
		start.point = {0, atend ? editor_track::point_kind::end : editor_track::point_kind::start};
		start.position = path.points[atend ? segment_data::point::end : segment_data::point::start];
		auto const &control{path.points[atend ? segment_data::point::control2 : segment_data::point::control1]};
		start.direction = glm::normalize(control != glm::dvec3{} ? control : path.points[atend ? segment_data::point::start : segment_data::point::end] - start.position);
		m_lay.points = {start.position};
		m_lay.start = start;
		for (glm::dvec3 point{0.0, start.position.y, 0.0}; words >> point.x >> point.z;)
			m_lay.points.push_back(point);
		lay_finish({});
		WriteLog("SELFTEST lay: " + m_lay.status);
		return selftest_result::handled;
	}
	if (command == "analyse")
	{
		std::string name;
		words >> name;
		editor_track::curve curve;
		if (auto *track{simulation::Paths.find(name)}; track != nullptr && editor_track::find_curve(*track, m_straights.tolerance, m_route.design.norms.gauge, curve))
		{
			std::string arcs;
			for (auto const &arc : curve.arcs)
				arcs += format(" R%.0f/%.1fdeg/after %.1fm", arc.radius, glm::degrees(arc.turn), arc.transition);
			WriteLog(format("SELFTEST analyse %s: R %.0f, compound %d, in %.1f m, out %.1f m, turn %.1f deg,", name.c_str(), curve.radius, curve.compound ? 1 : 0, curve.transition_in, curve.transition_out, glm::degrees(curve.turn)) + arcs);
		}
		else
			WriteLog("SELFTEST analyse " + name + ": no curve");
		return selftest_result::handled;
	}
	if (command == "route")
	{
		glm::dvec3 point{0.0};
		int single{1};
		words >> point.x >> point.z >> single;
		TTrack *nearest{nullptr};
		double distance{1e9};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty())
				continue;
			auto const d{geometry::plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, point)), point)};
			if (d < distance)
			{
				distance = d;
				nearest = track;
			}
		}
		m_track_tab = track_tab::route;
		m_route.single = (single != 0);
		auto const loaded{nearest != nullptr && route_load(*nearest)};
		WriteLog(format("SELFTEST route: %s from %s, %zu paths, %.3f m, %zu vertices, %d from the design, km %.3f, departed %d by %.3f m, valid %d", loaded ? "loaded" : "failed", nearest != nullptr ? nearest->name().c_str() : "-",
		                m_route.chain.tracks.size(), m_route.chain.length, m_route.design.vertices.size(), m_route.designed, m_route.origin / 1000.0, m_route.departed, m_route.departure, m_route.result.valid ? 1 : 0));
		WriteLog("SELFTEST route status: " + m_route.status + " " + m_route.error);
		for (std::size_t i = 0; i < m_route.design.vertices.size(); ++i)
		{
			auto const &v{m_route.design.vertices[i]};
			auto text{format("SELFTEST route vertex %zu: at %.3f %.3f offset %.3f R %.3f L %.3f / %.3f cant %.1f compound %d share %.4f", i, v.position.x, v.position.y, v.offset, v.radius, v.transition_in, v.transition_out, v.cant, v.compound ? 1 : 0, v.share)};
			for (auto const &arc : v.arcs)
				text += format(" [R %.3f L %.3f share %.4f]", arc.radius, arc.transition, arc.share);
			WriteLog(text);
		}
		return selftest_result::handled;
	}
	if (command == "routevertex" || command == "routearc" || command == "routeshare")
	{
		int index{0};
		words >> index;
		if (index < 0 || index >= static_cast<int>(m_route.design.vertices.size()))
		{
			WriteLog("SELFTEST " + command + ": no vertex " + std::to_string(index));
			return selftest_result::handled;
		}
		auto &vertex{m_route.design.vertices[index]};
		if (command == "routevertex")
			words >> vertex.radius >> vertex.transition_in >> vertex.transition_out >> vertex.cant;
		else if (command == "routeshare")
			words >> vertex.share;
		else
		{
			alignment::compound_arc arc;
			words >> arc.radius >> arc.transition >> arc.share;
			vertex.compound = true;
			vertex.arcs.push_back(arc);
		}
		route_update();
		std::string errors;
		for (auto const &error : m_route.result.errors)
			errors += " " + error;
		WriteLog(format("SELFTEST %s: valid %d, %zu elements, extension %.3f / %.3f", command.c_str(), m_route.result.valid ? 1 : 0, m_route.result.elements.size(), m_route.result.start_extension, m_route.result.end_extension) + errors);
		return selftest_result::handled;
	}
	if (command == "routekm")
	{
		double km{0.0};
		words >> km;
		m_route.origin = km * 1000.0;
		return selftest_result::handled;
	}
	if (command == "routeapply" || command == "routetake")
	{
		if (command == "routeapply")
			route_apply();
		else
			route_take_stored(m_route.departed);
		WriteLog("SELFTEST " + command + ": " + m_route.status + " " + m_route.error);
		return selftest_result::handled;
	}
	if (command == "digitizeall")
	{
		digitize_all();
		WriteLog(format("SELFTEST digitizeall: %zu alignments", alignment_store::records().size()));
		return selftest_result::handled;
	}
	if (command == "alignments")
	{
		auto const &records{alignment_store::records()};
		for (std::size_t r = 0; r < records.size(); ++r)
		{
			auto const &entry{records[r]};
			WriteLog(format("SELFTEST alignment %zu in %s: km %.3f, %zu vertices, %zu elements, consistent %d, length %.3f", r, scene::Layers.layer(entry.layer).name.c_str(), entry.origin / 1000.0, entry.design.vertices.size(), entry.elements.size(), entry.consistent ? 1 : 0, entry.result.length));
			for (auto const &e : entry.elements)
				WriteLog(format("SELFTEST   element %s vertex %d km %.3f L %.3f R %.3f -> %.3f cant %.1f", e.kind.c_str(), e.vertex, (entry.origin + e.chainage) / 1000.0, e.length, e.radius_start, e.radius_end, e.cant));
		}
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty())
				continue;
			auto const found{alignment_store::on_track(*track)};
			std::string text{"SELFTEST   path " + track->name() + ": "};
			if (false == found.has_value())
			{
				auto const closest{alignment_store::at(geometry::bezier{track->m_paths.front()}.point(0.5), 1000.0)};
				text += closest.has_value() ? format("off the alignments, %.3f m from alignment %d", closest->offset, closest->record) : std::string{"off the alignments"};
			}
			else
			{
				auto const e{alignment_store::describe(records[found->record].result.elements[found->element])};
				text += format("alignment %d element %d %s R %.3f -> %.3f, offset %.4f", found->record, found->element, e.kind.c_str(), e.radius_start, e.radius_end, found->offset);
			}
			editor_track::curve curve;
			if (editor_track::find_curve(*track, m_straights.tolerance, m_route.design.norms.gauge, curve))
				text += format(" | curve R %.3f L %.3f / %.3f cant %.1f compound %d arcs %zu %s", curve.radius, curve.transition_in, curve.transition_out, curve.cant, curve.compound ? 1 : 0, curve.arcs.size(), curve.designed.has_value() ? "designed" : "recognized");
			WriteLog(text);
		}
		return selftest_result::handled;
	}
	if (command == "vprofile")
	{
		std::string from;
		std::string to;
		words >> from >> to;
		auto *first{simulation::Paths.find(from)};
		auto *last{simulation::Paths.find(to)};
		if (first == nullptr || last == nullptr)
		{
			WriteLog("SELFTEST vprofile: no such paths");
			return selftest_result::handled;
		}
		m_profile.origin = 0.0;
		profile_open(first, last);
		WriteLog(format("SELFTEST vprofile: %zu spans, %.3f m, km of the start %.3f", m_profile.route.spans.size(), m_profile.route.length, m_profile.origin / 1000.0));
		for (auto const &zone : m_profile.context.switches)
			WriteLog(format("SELFTEST vprofile zone %.3f %.3f %s", zone.from, zone.to, zone.name.c_str()));
		return selftest_result::handled;
	}
	if (command == "vprofileapply")
	{
		// a plain grade line from the height of the start of the open route, applied to it
		double grade{0.0};
		words >> grade;
		auto &state{m_profile};
		if (state.samples.empty())
		{
			WriteLog("SELFTEST vprofileapply: no route open");
			return selftest_result::handled;
		}
		auto const start{state.samples.front().position.y};
		state.line.points = {{0.0, start, 0.0, false}, {state.route.length, start + grade / 1000.0 * state.route.length, 0.0, false}};
		state.line.join_start = false;
		state.line.join_end = false;
		profile_check();
		profile_apply();
		WriteLog("SELFTEST vprofileapply: " + state.status + " " + state.error);
		for (auto const &span : state.route.spans)
		{
			if (span.track->eType != tt_Switch || span.track->SwitchExtension == nullptr)
				continue;
			for (std::size_t i = 0; i < span.track->m_paths.size() && i < 2; ++i)
			{
				auto const &path{span.track->m_paths[i]};
				WriteLog(format("SELFTEST   switch %s path %zu%s: %zu pieces kept, %zu in use, ends %.3f %.3f", span.track->name().c_str(), i + 1, static_cast<int>(i) == span.path ? " (route)" : "", path.pieces.size(), span.track->SwitchExtension->Segments[i]->PieceCount(), path.points[segment_data::point::start].y, path.points[segment_data::point::end].y));
			}
		}
		return selftest_result::handled;
	}
	if (command == "backwardcurve")
	{
		std::string name;
		words >> name;
		auto *track{simulation::Paths.find(name)};
		editor_track::curve curve;
		editor_track::chain chain;
		std::string error;
		if (track == nullptr || false == editor_track::find_curve(*track, m_straights.tolerance, m_route.design.norms.gauge, curve) || false == editor_track::find_chain(curve.from, curve.to, chain, error, true))
		{
			WriteLog("SELFTEST backwardcurve: no curve at " + name);
			return selftest_result::handled;
		}
		std::vector<std::pair<TTrack *, bool>> run;
		for (std::size_t i = chain.tracks.size(); i > 0; --i)
			run.emplace_back(chain.tracks[i - 1], false == chain.forward[i - 1]);
		editor_track::curve backward;
		editor_track::analyse_curve(run, m_route.design.norms.gauge, backward);
		auto text{format("SELFTEST backwardcurve: %zu paths, R %.3f L %.3f / %.3f cant %.1f compound %d %s", run.size(), backward.radius, backward.transition_in, backward.transition_out, backward.cant, backward.compound ? 1 : 0, backward.designed.has_value() ? "designed" : "recognized")};
		for (auto const &arc : backward.arcs)
			text += format(" [R %.3f turn %.4f L %.3f]", arc.radius, arc.turn, arc.transition);
		WriteLog(text);
		return selftest_result::handled;
	}
	if (command == "deletetrack" || command == "deletekind")
	{
		std::string name;
		words >> name;
		if (command == "deletetrack" && name.empty())
		{
			delete_selected_track();
			WriteLog("SELFTEST deletetrack");
			return selftest_result::handled;
		}
		TTrack *chosen{nullptr};
		if (command == "deletetrack")
			chosen = simulation::Paths.find(name);
		else
			for (auto *track : simulation::Paths.sequence())
			{
				if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty())
					continue;
				auto const found{alignment_store::on_track(*track)};
				if (found.has_value() && alignment_store::describe(alignment_store::records()[found->record].result.elements[found->element]).kind == name)
				{
					chosen = track;
					break;
				}
			}
		if (chosen == nullptr || chosen->m_editorremoved)
		{
			WriteLog("SELFTEST " + command + ": no path " + name);
			return selftest_result::handled;
		}
		m_node = chosen;
		ui()->set_node(m_node);
		auto const deleted{chosen->name()};
		delete_selected_track();
		WriteLog("SELFTEST " + command + ": " + deleted + " deleted");
		return selftest_result::handled;
	}
	if (command == "btfcheck")
	{
		std::string name;
		words >> name;
		auto const textfile{Global.asCurrentSceneryPath + name + ".txtf"};
		auto const binaryfile{Global.asCurrentSceneryPath + name + ".btf"};
		auto const written{scene::terrain_file::prepare(textfile, binaryfile) == scene::terrain_file::state::binary};
		auto const kept{scene::terrain_file::extra(binaryfile)};
		WriteLog(format("SELFTEST btfcheck: binary %d, %zu bytes of the editor lines kept", written ? 1 : 0, kept.size()));
		std::istringstream lines{kept};
		std::string line;
		while (std::getline(lines, line))
			WriteLog("SELFTEST btf kept: " + line);
		return selftest_result::handled;
	}
	if (command == "pathsnear")
	{
		glm::dvec3 point{0.0};
		double radius{100.0};
		words >> point.x >> point.z >> radius;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
				continue;
			for (std::size_t i = 0; i < track->m_paths.size(); ++i)
			{
				auto const &path{track->m_paths[i]};
				auto const &a{path.points[segment_data::point::start]};
				auto const &b{path.points[segment_data::point::end]};
				if (geometry::plan_distance(a, point) > radius && geometry::plan_distance(b, point) > radius)
					continue;
				auto const &c1{path.points[segment_data::point::control1]};
				auto const &c2{path.points[segment_data::point::control2]};
				WriteLog(format("SELFTEST path %s/%zu %s: %.3f %.3f -> %.3f %.3f, c1 %.3f %.3f, c2 %.3f %.3f, %.3f m", track->name().c_str(), i, track->eType == tt_Switch ? "switch" : "normal", a.x, a.z, b.x, b.z, c1.x, c1.z, c2.x, c2.z, geometry::bezier{path}.plan_length()));
			}
		}
		return selftest_result::handled;
	}
	if (command == "splitkind" || command == "nudgekind")
	{
		std::string kind;
		glm::dvec3 offset{0.0};
		words >> kind >> offset.x >> offset.z;
		auto const &records{alignment_store::records()};
		TTrack *chosen{nullptr};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty())
				continue;
			auto const found{alignment_store::on_track(*track)};
			if (found.has_value() && alignment_store::describe(records[found->record].result.elements[found->element]).kind == kind)
			{
				chosen = track;
				break;
			}
		}
		if (chosen == nullptr)
		{
			WriteLog("SELFTEST " + command + ": no path over " + kind);
			return selftest_result::handled;
		}
		if (command == "splitkind")
		{
			auto const before{editor_track::capture(*chosen)};
			auto *created{editor_track::split_path(*chosen, 0.5)};
			if (created != nullptr)
				push_track_snapshot({{chosen, before}}, {created});
			WriteLog("SELFTEST splitkind: " + chosen->name() + " split, the second half " + (created != nullptr ? created->name() : std::string{"-"}));
		}
		else
		{
			std::vector<std::pair<TTrack *, editor_track::state>> states{{chosen, editor_track::capture(*chosen)}};
			std::vector<TTrack *> moved{chosen};
			auto const &path{chosen->m_paths.front()};
			for (auto const &end : {path.points[segment_data::point::start], path.points[segment_data::point::end]})
				for (auto *neighbour : {chosen->trPrev, chosen->trNext})
				{
					if (neighbour == nullptr || neighbour->m_paths.empty() || std::find(moved.begin(), moved.end(), neighbour) != moved.end())
						continue;
					auto const &other{neighbour->m_paths.front()};
					for (auto const kind : {editor_track::point_kind::start, editor_track::point_kind::end})
						if (glm::distance(other.points[kind == editor_track::point_kind::start ? segment_data::point::start : segment_data::point::end], end) < 0.01)
						{
							states.emplace_back(neighbour, editor_track::capture(*neighbour));
							editor_track::move_point(*neighbour, {0, kind}, end + offset);
							moved.push_back(neighbour);
							break;
						}
				}
			editor_track::translate(*chosen, offset);
			editor_track::commit(moved);
			push_track_snapshot(std::move(states), {});
			WriteLog("SELFTEST nudgekind: " + chosen->name() + " moved with " + std::to_string(moved.size() - 1) + " joined ends");
		}
		return selftest_result::handled;
	}
	if (command == "digitize")
	{
		glm::dvec3 point{0.0};
		words >> point.x >> point.z;
		TTrack *nearest{nullptr};
		double distance{1e9};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty())
				continue;
			auto const d{geometry::plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, point)), point)};
			if (d < distance)
			{
				distance = d;
				nearest = track;
			}
		}
		auto const kept{nearest != nullptr && digitize_drawn({nearest}, false)};
		WriteLog(format("SELFTEST digitize %s: %d", nearest != nullptr ? nearest->name().c_str() : "-", kept ? 1 : 0));
		return selftest_result::handled;
	}
	if (command == "parallel")
	{
		words >> m_parallel.spacing >> m_parallel.side;
		m_parallel.scope = 1;
		m_parallel.built_for = nullptr;
		parallel_build();
		WriteLog("SELFTEST parallel: " + m_parallel.status + " " + m_parallel.error);
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

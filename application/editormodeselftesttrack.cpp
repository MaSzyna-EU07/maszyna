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
		m_hekto.tab().expand = tab == 1;
		m_fouling.tab().expand = tab == 2;
		m_parallel.tab().expand = tab == 3;
		m_vehicle.tab().expand = tab == 4;
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
	if (command == "digitizescenery")
	{
		// "digitizescenery write" digitizes, without the word only counts what it would do
		std::string mode;
		words >> mode;
		auto const text{digitize_scenery(mode == "write")};
		std::istringstream lines{text};
		std::string line;
		while (std::getline(lines, line))
			WriteLog("SELFTEST digitizescenery: " + line);
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
	if (command == "vprofilemodel")
	{
		// the ground of the terrain model along the open route, and where the camera is
		auto const &state{m_profile};
		std::size_t known{0};
		double low{std::numeric_limits<double>::max()};
		double high{std::numeric_limits<double>::lowest()};
		for (std::size_t i = 0; i < state.model.size() && i < state.samples.size(); ++i)
			if (false == std::isnan(state.model[i]))
			{
				++known;
				low = std::min(low, state.samples[i].position.y - state.model[i]);
				high = std::max(high, state.samples[i].position.y - state.model[i]);
			}
		WriteLog(format("SELFTEST vprofilemodel: %zu of %zu samples known%s, the track %.2f..%.2f m from the model; view %.1f..%.1f, camera %.1f %.1f %.1f angle %.3f %.3f, ortho %d extent %.0f", known, state.samples.size(), state.model_waiting ? " (waiting)" : "", known > 0 ? low : 0.0, known > 0 ? high : 0.0,
		                state.view_from, state.view_to, Camera.Pos.x, Camera.Pos.y, Camera.Pos.z, Camera.Angle.x, Camera.Angle.y, Global.EditorOrtho ? 1 : 0, Global.EditorOrthoExtent));
		return selftest_result::handled;
	}
	if (command == "vprofilezoom")
	{
		// the plot of the open profile from one chainage to another, the heights exaggerated as given (0: fitted)
		double from{0.0}, to{100.0}, exaggeration{0.0};
		words >> from >> to >> exaggeration;
		m_profile.view_from = from;
		m_profile.view_to = to;
		m_profile.exaggeration = static_cast<float>(exaggeration);
		std::string zones;
		for (auto const &zone : m_profile.context.switches)
			zones += format(" [%.1f..%.1f %s]", zone.from, zone.to, zone.name.c_str());
		std::string points;
		for (auto const &point : m_profile.line.points)
			points += format(" (%.1f %.3f R%.0f%s)", point.chainage, point.elevation, point.radius, point.joint ? " joint" : "");
		std::size_t kept{0};
		std::string nearby;
		for (auto const &entry : m_profiles)
		{
			++kept;
			if (false == m_profile.samples.empty() && glm::length(glm::dvec2{entry.start.x - m_profile.samples.front().position.x, entry.start.z - m_profile.samples.front().position.z}) < 50.0)
				nearby += format(" [%.1f m, start %.1f %.1f end %.1f %.1f]", entry.length, entry.start.x, entry.start.z, entry.end.x, entry.end.z);
		}
		WriteLog(format("SELFTEST vprofilezoom library %zu (loaded %d), route %.1f m from %.1f %.1f to %.1f %.1f; near:%s", kept, m_profiles_loaded ? 1 : 0, m_profile.route.length, m_profile.samples.front().position.x, m_profile.samples.front().position.z, m_profile.samples.back().position.x, m_profile.samples.back().position.z, nearby.c_str()));
		std::string issues;
		for (auto const &issue : m_profile.issues)
			issues += std::string{issue.error ? " | error: " : " | "} + issue.text;
		WriteLog(format("SELFTEST vprofilezoom %.1f..%.1f; departure of the track %.3f m; %s; zones%s; points%s%s", from, to, m_profile.departure, m_profile.status.c_str(), zones.c_str(), points.c_str(), issues.c_str()));
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
	if (command == "trackcolours")
	{
		int colouring{0};
		words >> colouring;
		m_track_colouring = static_cast<track_colouring>(std::clamp(colouring, 0, 3));
		return selftest_result::handled;
	}
	if (command == "vprofilerun" || command == "vprofiledel" || command == "vprofileput")
	{
		// the grade line of the line through a track, a point of it taken out as with Del, the line applied
		auto &state{m_profile};
		if (command == "vprofilerun")
		{
			std::string name;
			double length{std::numeric_limits<double>::infinity()};
			words >> name >> length;
			auto *track{simulation::Paths.find(name)};
			if (track == nullptr)
			{
				WriteLog("SELFTEST vprofilerun: no track " + name);
				return selftest_result::handled;
			}
			state.run_length = length;
			profile_open_run(*track);
		}
		else if (command == "vprofiledel")
		{
			int index{-1};
			words >> index;
			auto &points{state.line.points};
			if (index <= 0 || index + 1 >= static_cast<int>(points.size()) || points[index].joint)
			{
				WriteLog(format("SELFTEST vprofiledel: point %d can't be taken out", index));
				return selftest_result::handled;
			}
			profile_remove_point(index);
		}
		else
		{
			std::string option;
			words >> option;
			profile_apply(option == "terrain");
			WriteLog("SELFTEST vprofileput: " + state.status + " " + state.error);
		}
		std::string points;
		for (auto const &point : state.line.points)
			points += format(" (%.1f %.3f R%.0f%s%s)", point.chainage, point.elevation, point.radius, point.automatic ? "" : " fixed", point.joint ? " joint" : "");
		std::string issues;
		for (auto const &issue : state.issues)
			issues += std::string{issue.error ? " | error: " : " | "} + issue.text;
		double low{0.0}, high{0.0}, at_low{0.0}, at_high{0.0};
		for (auto const &sample : state.samples)
		{
			auto const off{sample.position.y - profile::elevation(state.line, sample.chainage)};
			if (off < low)
				low = off, at_low = sample.chainage;
			if (off > high)
				high = off, at_high = sample.chainage;
		}
		WriteLog(format("SELFTEST %s: route %.1f m, %zu samples, track against the line %.3f at %.1f .. %+.3f at %.1f; points%s%s", command.c_str(), state.route.length, state.samples.size(), low, at_low, high, at_high, points.c_str(), issues.c_str()));
		return selftest_result::handled;
	}
	if (command == "vprofileapply")
	{
		// a plain grade line from the height of the start of the open route, applied to it
		double grade{0.0};
		std::string option;
		words >> grade >> option;
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
		profile_apply(option == "terrain");
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
	if (command == "infrascan")
	{
		// what the binding of the objects along the tracks finds over the whole scenery: how many of each kind and group, and
		// the files the rules don't know, the most used first
		double corridor{0.0};
		int top{30};
		std::string usage;
		words >> corridor >> top >> usage;
		m_infra.scope = 3;
		if (corridor > 0.0)
			m_infra.corridor = static_cast<float>(corridor);
		infra_recognize();
		std::map<std::string, int> kinds, groups;
		std::map<std::string, std::pair<int, std::string>> unknown;
		for (auto const &candidate : m_infra.candidates)
		{
			auto const &binding{candidate.binding};
			++kinds[infra::name(binding.type)];
			++groups[infra::name(binding.group)];
			if (binding.group != infra::category::other)
				continue;
			std::string file;
			if (binding.type == infra::kind::include)
				file = *scene::Layers.instance(binding.include).file;
			else if (binding.type == infra::kind::model)
			{
				auto const *model{static_cast<TAnimModel const *>(binding.node)};
				file = model->Model() != nullptr ? model->Model()->NameGet() : std::string{"?"};
			}
			else
				file = infra::name(binding.type);
			auto &entry{unknown[std::string{infra::name(binding.type)} + " " + file]};
			++entry.first;
			entry.second = format("%.1f m", candidate.offset);
		}
		std::string text;
		for (auto const &[name, count] : kinds)
			text += format(" %s %d", name.c_str(), count);
		text += " |";
		for (auto const &[name, count] : groups)
			text += format(" %s %d", name.c_str(), count);
		WriteLog(format("SELFTEST infrascan: %zu objects along %zu paths, corridor %.1f m:", m_infra.candidates.size(), m_infra.tracks.size(), m_infra.corridor) + text + " | " + m_infra.status);
		std::vector<std::pair<int, std::string>> sorted;
		for (auto const &[name, entry] : unknown)
			sorted.emplace_back(entry.first, name + " (e.g. " + entry.second + " off the axis)");
		std::sort(sorted.begin(), sorted.end(), [](auto const &A, auto const &B) { return A.first > B.first; });
		for (std::size_t i = 0; i < sorted.size() && static_cast<int>(i) < top; ++i)
			WriteLog(format("SELFTEST infrascan unknown %d x %s", sorted[i].first, sorted[i].second.c_str()));
		WriteLog(format("SELFTEST infrascan: %zu files the rules don't know", sorted.size()));
		// how each template and model stands by the tracks: the distance from the axis, the height over the rail top, the turn
		// against the path, as tab separated values for the descriptions of the templates
		if (false == usage.empty())
		{
			struct use
			{
				std::string kind;
				std::string group;
				std::vector<double> offsets, heights, turns;
			};
			std::map<std::string, use> uses;
			for (auto const &candidate : m_infra.candidates)
			{
				auto const &binding{candidate.binding};
				if (binding.anchors.empty() || (binding.type != infra::kind::include && binding.type != infra::kind::model))
					continue;
				std::string file;
				if (binding.type == infra::kind::include)
					file = *scene::Layers.instance(binding.include).file;
				else
				{
					auto const *model{static_cast<TAnimModel const *>(binding.node)};
					file = model->Model() != nullptr ? model->Model()->NameGet() : std::string{};
				}
				if (file.empty())
					continue;
				auto &entry{uses[file]};
				entry.kind = infra::name(binding.type);
				entry.group = infra::name(binding.group);
				entry.offsets.push_back(binding.anchors.front().offset);
				entry.heights.push_back(binding.anchors.front().height);
				if (binding.turns)
				{
					auto turn{std::fmod(glm::degrees(binding.yaw), 360.0)};
					entry.turns.push_back(turn < 0.0 ? turn + 360.0 : turn);
				}
			}
			std::ofstream out{usage};
			out << "file\tkind\tgroup\tcount\toffsets\theights\tturns\n";
			auto const list = [](std::vector<double> const &Values) {
				std::string text;
				for (auto const value : Values)
					text += format("%s%.2f", text.empty() ? "" : ",", value);
				return text;
			};
			for (auto const &[file, entry] : uses)
				out << file << '\t' << entry.kind << '\t' << entry.group << '\t' << entry.offsets.size() << '\t' << list(entry.offsets) << '\t' << list(entry.heights) << '\t' << list(entry.turns) << '\n';
			WriteLog(format("SELFTEST infrascan: the use of %zu templates and models written to %s", uses.size(), usage.c_str()));
		}
		return selftest_result::handled;
	}
	if (command == "rollcheck")
	{
		// at each joint near the point the cant of the paths meeting, told along a common direction, and their heights
		glm::dvec3 point{0.0};
		double radius{100.0};
		words >> point.x >> point.z >> radius;
		struct end
		{
			TTrack *track;
			std::size_t path;
			bool last;
			glm::dvec3 position;
			double roll; // along the direction leaving the joint
		};
		std::vector<end> ends;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved)
				continue;
			for (std::size_t i = 0; i < track->m_paths.size(); ++i)
			{
				auto const &path{track->m_paths[i]};
				for (auto const last : {false, true})
				{
					auto const &at{path.points[last ? segment_data::point::end : segment_data::point::start]};
					if (geometry::plan_distance(at, point) > radius)
						continue;
					// the roll of the path told along it; leaving the joint from its end the direction is the other way
					auto const roll{static_cast<double>(path.rolls[last ? 1 : 0])};
					ends.push_back({track, i, last, at, last ? -roll : roll});
				}
			}
		}
		std::size_t joints{0}, wrong{0};
		for (std::size_t a = 0; a < ends.size(); ++a)
			for (std::size_t b = a + 1; b < ends.size(); ++b)
			{
				if (ends[a].track == ends[b].track || glm::distance(ends[a].position, ends[b].position) > 0.02)
					continue;
				++joints;
				// leaving the joint both ways, one path's cant is the other's turned round
				auto const mismatch{std::abs(ends[a].roll + ends[b].roll)};
				if (mismatch > 0.05)
				{
					++wrong;
					WriteLog(format("SELFTEST rollcheck %s/%zu %s and %s/%zu %s at %.2f %.2f %.2f: cant %.3f vs %.3f deg", ends[a].track->name().c_str(), ends[a].path, ends[a].last ? "end" : "start", ends[b].track->name().c_str(), ends[b].path, ends[b].last ? "end" : "start", ends[a].position.x, ends[a].position.y, ends[a].position.z, ends[a].roll, -ends[b].roll));
				}
			}
		WriteLog(format("SELFTEST rollcheck: %zu joints, %zu with the cant not matching", joints, wrong));
		return selftest_result::handled;
	}
	if (command == "freeends")
	{
		// the ends of the rail paths joined to nothing near the point, and how far the nearest other end is from each
		glm::dvec3 point{0.0};
		double radius{100.0};
		words >> point.x >> point.z >> radius;
		for (auto const &end : editor_track::free_ends(nullptr, 1, point, radius, {}))
		{
			double nearest{std::numeric_limits<double>::max()};
			double rise{0.0};
			for (auto *track : simulation::Paths.sequence())
			{
				if (track == nullptr || track == end.track || track->m_editorremoved)
					continue;
				for (auto const &path : track->m_paths)
					for (auto const which : {segment_data::point::start, segment_data::point::end})
						if (auto const distance{glm::distance(path.points[which], end.position)}; distance < nearest)
						{
							nearest = distance;
							rise = path.points[which].y - end.position.y;
						}
			}
			WriteLog(format("SELFTEST freeend %s at %.3f %.3f %.3f, nearest other end %.3f m away, %+.3f m higher", end.track->name().c_str(), end.position.x, end.position.y, end.position.z, nearest, rise));
		}
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
				WriteLog(format("SELFTEST path %s/%zu %s: %.3f %.3f -> %.3f %.3f, c1 %.3f %.3f, c2 %.3f %.3f, %.3f m, y %.3f -> %.3f, cant %.2f -> %.2f deg", track->name().c_str(), i, track->eType == tt_Switch ? "switch" : "normal", a.x, a.z, b.x, b.z, c1.x, c1.z, c2.x, c2.z, geometry::bezier{path}.plan_length(), a.y, b.y, path.rolls[0], path.rolls[1]));
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
	return selftest_result::unhandled;
}

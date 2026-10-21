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

editor_mode::selftest_result editor_mode::selftest_scene(std::string const &command, std::istringstream &words)
{
	auto &test{m_selftest};
	if (command == "camera")
	{
		glm::dvec3 from;
		glm::dvec3 to;
		words >> from.x >> from.y >> from.z >> to.x >> to.y >> to.z;
		auto const look{glm::normalize(to - from)};
		Camera.Pos = from;
		Camera.Angle = glm::vec3(static_cast<float>(std::asin(std::clamp(look.y, -1.0, 1.0))), static_cast<float>(std::atan2(-look.x, -look.z)), 0.0f);
		Global.pCamera = Camera;
		m_focus_active = false;
		return selftest_result::handled;
	}
	if (command == "zoom")
	{
		float factor{1.0f};
		words >> factor;
		Global.ZoomFactor = std::max(factor, 0.01f);
		return selftest_result::handled;
	}
	if (command == "selectat")
	{
		glm::dvec3 point{0.0};
		words >> point.x >> point.z;
		TTrack *nearest{nullptr};
		double distance{1e9};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
				continue;
			auto const d{geometry::plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, point)), point)};
			if (d < distance)
			{
				distance = d;
				nearest = track;
			}
		}
		m_node = distance < 3.0 ? nearest : nullptr;
		m_track_point = {};
		ui()->set_node(m_node);
		WriteLog(format("SELFTEST selectat %.1f %.1f: %s", point.x, point.z, m_node != nullptr ? m_node->name().c_str() : "nothing"));
		return selftest_result::handled;
	}
	if (command == "select")
	{
		std::string name;
		words >> name;
		m_node = simulation::Paths.find(name);
		m_track_point = {};
		ui()->set_node(m_node);
		WriteLog("SELFTEST select " + name + (m_node != nullptr ? "" : ": no such track"));
		return selftest_result::handled;
	}
	if (command == "perf")
	{
		// the frames drawn and the time gone since the last perf, with the timings and the counts the renderer keeps
		std::string label;
		words >> label;
		static auto last{std::chrono::steady_clock::now()};
		static std::size_t lastframe{0};
		auto const now{std::chrono::steady_clock::now()};
		auto const frames{editor_selftest::frames() - lastframe};
		auto const elapsed{std::chrono::duration<double, std::milli>(now - last).count()};
		WriteLog(format("SELFTEST perf %s: %zu frames, %.3f ms per frame | %s | %s", label.c_str(), frames, frames > 0 ? elapsed / frames : 0.0, editor_selftest::one_line(GfxRenderer->info_times()).c_str(), editor_selftest::one_line(GfxRenderer->info_stats()).c_str()));
		last = now;
		lastframe = editor_selftest::frames();
		return selftest_result::handled;
	}
	if (command == "rebuildall")
	{
		// the geometry of every path made anew, as the editor does it after a change, timed
		auto const start{std::chrono::steady_clock::now()};
		auto count{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved)
				continue;
			auto &section{simulation::Region->section(track->location())};
			if (false == section.m_geometrycreated)
				continue;
			track->rebuild_geometry(section.m_geometrybank);
			++count;
		}
		WriteLog(format("SELFTEST rebuildall: %d paths in %.1f ms", count, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()));
		// the sleepers laid again on all the tracks, which the renderer otherwise does as they come in sight, timed by section
		std::map<scene::basic_section const *, double> sectiontimes;
		auto laidcount{0};
		std::size_t parts{0};
		auto laytime{0.0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || false == track->m_sleepers3d.active)
				continue;
			auto const laystart{std::chrono::steady_clock::now()};
			track_sleepers::lay(*track);
			auto const took{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - laystart).count()};
			laytime += took;
			sectiontimes[&simulation::Region->section(track->location())] += took;
			++laidcount;
			for (auto const &instances : track->m_sleepers3d.instances)
				parts += instances.size();
		}
		auto slowest{0.0};
		for (auto const &sectiontime : sectiontimes)
			slowest = std::max(slowest, sectiontime.second);
		WriteLog(format("SELFTEST rebuildall sleepers3d: %d tracks laid in %.1f ms, %zu parts; %zu sections, %.2f ms per section, %.2f ms the slowest", laidcount, laytime, parts, sectiontimes.size(),
		    (sectiontimes.empty() ? 0.0 : laytime / sectiontimes.size()), slowest));
		return selftest_result::handled;
	}
	if (command == "frames")
	{
		std::string name;
		double from{0.0};
		double to{0.0};
		double step{0.1};
		words >> name >> from >> to >> step;
		auto *sweep{simulation::Sweeps.find(name)};
		if (sweep == nullptr)
			return selftest_result::handled;
		auto const side{sweep->definition().lateral >= 0.0 ? 1 : 0};
		if (step <= 0.0)
			return selftest_result::handled;
		for (int i = 0; from + i * step <= to + 1e-9; ++i)
		{
			auto const station{from + i * step};
			auto const at{sweep->frame_at(station)};
			WriteLog(format("SELFTEST frame %s %.2f: %.3f %.3f %.3f widening %.4f cant %.4f setback %.4f", name.c_str(), station, at.position.x, at.position.y, at.position.z, at.widening[side], at.cant[side], sweep->setback(at)));
		}
		return selftest_result::handled;
	}
	if (command == "weather")
	{
		auto const &time{simulation::Time.data()};
		WriteLog(format("SELFTEST weather: %02d:%02d, fog end %.0f m, overcast %.2f", time.wHour, time.wMinute, Global.fFogEnd, Global.Overcast));
		return selftest_result::handled;
	}
	if (command == "projtest")
	{
		screen_projection const projection;
		for (std::size_t i = 0; i < m_sweep.outline.size() && i < 45; i += 4)
		{
			ImVec2 screen;
			auto const shown{projection.project(m_sweep.outline[i], screen)};
			auto const clip{projection.clip(m_sweep.outline[i])};
			WriteLog(format("SELFTEST proj %zu: %.1f %.1f %.1f -> %d %.0f %.0f w %.2f", i, m_sweep.outline[i].x, m_sweep.outline[i].y, m_sweep.outline[i].z, shown ? 1 : 0, screen.x, screen.y, clip.w));
		}
		return selftest_result::handled;
	}
	if (command == "newscenery")
	{
		WriteLog(restart_for_new_scenery() ? "SELFTEST newscenery: started" : "SELFTEST newscenery: failed");
		return selftest_result::pause;
	}
	if (command == "vehicles")
	{
		for (auto *vehicle : simulation::Vehicles.sequence())
		{
			if (vehicle == nullptr)
				continue;
			auto const head{vehicle->HeadPosition()};
			auto const front{vehicle->VectorFront()};
			auto const coupler = [&](int const End) {
				auto const &coupler{vehicle->MoverParameters->Couplers[End]};
				return coupler.Connected != nullptr ? coupler.Connected->Name + "/" + std::to_string(coupler.CouplingFlag) : std::string{"-"};
			};
			WriteLog(format("SELFTEST vehicle %s head %.2f %.2f front %.2f %.2f dir %d coupler0 %s coupler1 %s", vehicle->name().c_str(), head.x, head.z, front.x, front.z, vehicle->iDirection, coupler(0).c_str(), coupler(1).c_str()));
		}
		return selftest_result::handled;
	}
	if (command == "mouseat")
	{
		glm::dvec3 at{0.0};
		words >> at.x >> at.y >> at.z;
		ImVec2 projected;
		if (false == screen_projection{}.project(at, projected))
		{
			WriteLog("SELFTEST mouseat: behind the camera");
			return selftest_result::handled;
		}
		test.input.mouse_to(glm::dvec2{projected.x, projected.y});
		WriteLog(format("SELFTEST mouse %.0f %.0f", projected.x, projected.y));
		return selftest_result::handled;
	}
	if (command == "glide")
	{
		glm::dvec3 at{0.0};
		int steps{8};
		int every{2};
		std::string shots;
		words >> at.x >> at.y >> at.z >> steps >> every >> shots;
		ImVec2 projected;
		if (false == screen_projection{}.project(at, projected))
			return selftest_result::handled;
		glm::dvec2 const from{test.input.mouse().value_or(glm::dvec2{projected.x, projected.y})};
		glm::dvec2 const to{projected.x, projected.y};
		std::vector<std::string> expanded;
		for (int step = 1; step <= std::max(1, steps); ++step)
		{
			auto const t{static_cast<double>(step) / std::max(1, steps)};
			auto const eased{t * t * (3.0 - 2.0 * t)};
			auto const point{glm::mix(from, to, eased)};
			expanded.push_back(format("mouse %.1f %.1f", point.x, point.y));
			expanded.push_back("wait " + std::to_string(std::max(1, every)));
			if (shots == "shot")
				expanded.push_back("screenshot");
		}
		test.lines.insert(test.lines.begin() + static_cast<std::ptrdiff_t>(test.next), expanded.begin(), expanded.end());
		return selftest_result::handled;
	}
	if (test.input.dispatch(command, words))
		return selftest_result::handled;
	if (command == "screenshot")
	{
		Application.queue_screenshot();
		test.wait = 3;
		return selftest_result::pause;
	}
	if (command == "save")
	{
		save();
		return selftest_result::handled;
	}
	if (command == "ortho")
	{
		float extent{0.0f};
		std::string ballast;
		words >> extent >> ballast;
		choose_work_area(work_area::tracks);
		if (Global.EditorOrtho == extent < 0.0f)
			toggle_ortho();
		if (extent > 0.0f)
			Global.EditorOrthoExtent = std::clamp(extent, 5.0f, 50000.0f);
		if (false == ballast.empty())
			Global.EditorOrthoBallast = ballast != "noballast";
		WriteLog(format("SELFTEST ortho extent %.0f ballast %d", Global.EditorOrthoExtent, Global.EditorOrthoBallast ? 1 : 0));
		return selftest_result::handled;
	}
	if (command == "mapstate")
	{
		auto const marked{scene::Layers.empty() ? std::vector<std::string>{} : scene::Layers.marked(scene::layer_handle{1}, "//$o")};
		WriteLog(format("SELFTEST mapstate orthophoto %d top %d extent %.0f mark %s camera %.1f %.1f %.1f", m_orthophoto.enabled() ? 1 : 0, Global.EditorOrtho ? 1 : 0, Global.EditorOrthoExtent, marked.empty() ? "-" : marked.front().c_str(), Camera.Pos.x, Camera.Pos.y, Camera.Pos.z));
		return selftest_result::handled;
	}
	if (command == "selectmodel")
	{
		glm::dvec3 point{0.0};
		words >> point.x >> point.z;
		TAnimModel *nearest{nullptr};
		auto best{50.0};
		for (auto *model : simulation::Instances.sequence())
		{
			if (model == nullptr)
				continue;
			auto const distance{glm::length(glm::dvec2{model->location().x - point.x, model->location().z - point.z})};
			if (distance < best)
			{
				best = distance;
				nearest = model;
			}
		}
		m_node = nearest;
		select_include(0);
		ui()->set_node(m_node);
		WriteLog("SELFTEST selectmodel: " + (nearest != nullptr ? format("%s at %.2f %.2f %.2f", nearest->name().c_str(), nearest->location().x, nearest->location().y, nearest->location().z) : std::string{"none"}));
		return selftest_result::handled;
	}
	if (command == "addmodel")
	{
		glm::dvec3 point{0.0};
		words >> point.x >> point.z;
		TAnimModel *nearest{nullptr};
		auto best{50.0};
		for (auto *model : simulation::Instances.sequence())
		{
			if (model == nullptr)
				continue;
			auto const distance{glm::length(glm::dvec2{model->location().x - point.x, model->location().z - point.z})};
			if (distance < best)
			{
				best = distance;
				nearest = model;
			}
		}
		select_node(nearest, true);
		WriteLog("SELFTEST addmodel: " + (nearest != nullptr ? format("%s at %.2f %.2f %.2f", nearest->name().c_str(), nearest->location().x, nearest->location().y, nearest->location().z) : std::string{"none"}));
		return selftest_result::handled;
	}
	if (command == "selection")
	{
		std::string text;
		for (auto *node : selection_with_groups())
			text += format(" [%s %.2f %.2f %.2f group %d]", node->name().c_str(), node->location().x, node->location().y, node->location().z, static_cast<int>(node->group()));
		WriteLog("SELFTEST selection:" + text);
		return selftest_result::handled;
	}
	if (command == "moveselection" && m_node != nullptr)
	{
		glm::dvec3 offset{0.0};
		words >> offset.x >> offset.y >> offset.z;
		selection_snapshots(EditorSnapshot::Action::Move);
		auto const before{m_node->location()};
		m_editor.translate(m_node, before + offset, true);
		selection_follow(m_node->location() - before, glm::vec3(0.0f), m_node->location());
		return selftest_result::handled;
	}
	if (command == "turnselection" && m_node != nullptr)
	{
		float degrees{0.0f};
		words >> degrees;
		selection_snapshots(EditorSnapshot::Action::Rotate);
		glm::vec3 const turned{0.0f, degrees, 0.0f};
		m_editor.rotate(m_node, turned, 0.0f);
		selection_follow(glm::dvec3(0.0), turned, m_node->location());
		return selftest_result::handled;
	}
	if (command == "copy")
	{
		copy_selection();
		return selftest_result::handled;
	}
	if (command == "paste")
	{
		glm::dvec3 point{0.0};
		words >> point.x >> point.y >> point.z;
		paste_clipboard(point);
		return selftest_result::handled;
	}
	if (command == "group")
	{
		group_selection();
		return selftest_result::handled;
	}
	if (command == "ungroup")
	{
		ungroup_selection();
		return selftest_result::handled;
	}
	if (command == "drop")
	{
		drop_selection_to_ground();
		return selftest_result::handled;
	}
	if (command == "delete")
	{
		delete_selected();
		return selftest_result::handled;
	}
	if (command == "selectinclude")
	{
		glm::dvec3 point{0.0};
		words >> point.x >> point.z;
		scene::instance_handle nearest{0};
		auto best{50.0};
		m_node = nullptr;
		for (scene::instance_handle instance = 1; instance <= scene::Layers.instance_count(); ++instance)
		{
			if (false == scene::Layers.removable(instance))
				continue;
			select_include(instance);
			bend_source source;
			if (false == bend_source_of_selection(source))
				continue;
			auto const at{glm::dvec3(source.transform[3])};
			auto const distance{glm::length(glm::dvec2{at.x - point.x, at.z - point.z})};
			if (distance < best)
			{
				best = distance;
				nearest = instance;
			}
		}
		select_include(nearest);
		WriteLog("SELFTEST selectinclude: " + (nearest != 0 ? *scene::Layers.instance(nearest).file : std::string{"none"}));
		return selftest_result::handled;
	}
	if (command == "modelbox")
	{
		auto *model{dynamic_cast<TAnimModel *>(m_node)};
		bend_source source;
		if (model == nullptr || false == bend_source_of_selection(source))
		{
			WriteLog("SELFTEST modelbox: no model selected");
			return selftest_result::handled;
		}
		scene::node_data data;
		sweep_node probe{data};
		sweep_node::state state;
		state.model = source.file;
		state.skin = source.skin;
		state.scale = source.scale;
		probe.define(state);
		glm::dvec3 boxlow{std::numeric_limits<double>::max()};
		glm::dvec3 boxhigh{-std::numeric_limits<double>::max()};
		for (auto const &item : probe.items())
		{
			auto const transform{source.transform * glm::scale(glm::dmat4(1.0), source.scale) * item.transform};
			std::vector<TSubModel *> pending{item.model->GetSMRoot()};
			while (false == pending.empty())
			{
				auto *submodel{pending.back()};
				pending.pop_back();
				for (; submodel != nullptr; submodel = submodel->Next)
				{
					if (submodel->Child != nullptr)
						pending.push_back(submodel->Child);
					if (submodel->eType != GL_TRIANGLES || submodel->fSquareMinDist > 0.f || (submodel->m_geometry.handle.bank == 0 && submodel->m_geometry.handle.chunk == 0))
						continue;
					glm::dmat4 local{1.0};
					for (auto *parent = submodel; parent != nullptr; parent = parent->Parent)
						if ((parent->iFlags & 0xC000) != 0 && parent->GetMatrix() != nullptr)
							local = glm::dmat4(glm::make_mat4(parent->GetMatrix()->readArray())) * local;
					for (auto const &vertex : GfxRenderer->Vertices(submodel->m_geometry.handle))
					{
						auto const world{glm::dvec3(transform * local * glm::dvec4(glm::dvec3(vertex.position), 1.0))};
						boxlow = glm::min(boxlow, world);
						boxhigh = glm::max(boxhigh, world);
					}
				}
			}
		}
		WriteLog(format("SELFTEST modelbox: %.2f %.2f %.2f - %.2f %.2f %.2f", boxlow.x, boxlow.y, boxlow.z, boxhigh.x, boxhigh.y, boxhigh.z));
		return selftest_result::handled;
	}
	if (command == "key")
	{
		std::string key;
		std::string action;
		std::string mods;
		words >> key >> action >> mods;
		if (key == "RMB")
			m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT, action == "press" ? GLFW_PRESS : GLFW_RELEASE);
		else
		{
			auto const code{key == "S" ? GLFW_KEY_S : key == "W" ? GLFW_KEY_W : key == "CTRL" ? GLFW_KEY_LEFT_CONTROL : 0};
			on_key(code, 0, action == "press" ? GLFW_PRESS : GLFW_RELEASE, mods == "ctrl" ? GLFW_MOD_CONTROL : 0);
		}
		WriteLog(format("SELFTEST key %s %s %s: camera %.2f %.2f %.2f, key S %d, right button %d", key.c_str(), action.c_str(), mods.c_str(), Camera.Pos.x, Camera.Pos.y, Camera.Pos.z, static_cast<int>(input::keys[GLFW_KEY_S]), m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT)));
		return selftest_result::handled;
	}
	if (command == "event")
	{
		std::string name;
		words >> name;
		auto *event{simulation::Events.FindEvent(name)};
		if (event != nullptr)
			simulation::Events.AddToQuery(event, nullptr);
		WriteLog("SELFTEST event " + name + (event != nullptr ? " queued" : ": no such event"));
		return selftest_result::handled;
	}
	if (command == "memcell")
	{
		std::string name;
		words >> name;
		auto *cell{simulation::Memory.find(name)};
		WriteLog("SELFTEST memcell " + name + (cell != nullptr ? format(": %s %.1f %.1f", cell->Text().c_str(), cell->Value1(), cell->Value2()) : std::string{": none"}));
		return selftest_result::handled;
	}
	if (command == "newscenerypopup")
	{
		m_newscenery_asked = true;
		return selftest_result::handled;
	}
	if (command == "openscenerypopup")
	{
		m_openscenery_asked = true;
		std::snprintf(m_openscenery_search, sizeof(m_openscenery_search), "%s", "zz_");
		return selftest_result::handled;
	}
	if (command == "openscenery")
	{
		std::string name;
		words >> name;
		WriteLog(restart_editor(name) ? "SELFTEST openscenery: started " + name : "SELFTEST openscenery: failed");
		return selftest_result::handled;
	}
	if (command == "undo")
	{
		undo_last();
		WriteLog("SELFTEST undo");
		return selftest_result::handled;
	}
	if (command == "redo")
	{
		redo_last();
		WriteLog("SELFTEST redo");
		return selftest_result::handled;
	}
	if (command == "log")
	{
		std::string text;
		std::getline(words, text);
		WriteLog("SELFTEST" + text);
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

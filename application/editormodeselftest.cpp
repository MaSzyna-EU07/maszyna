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

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

editor_mode::selftest_result editor_mode::selftest_dispatch(std::string const &command, std::istringstream &words)
{
	for (auto const group : {&editor_mode::selftest_scene, &editor_mode::selftest_sweep, &editor_mode::selftest_track, &editor_mode::selftest_switch, &editor_mode::selftest_rails, &editor_mode::selftest_join, &editor_mode::selftest_signal, &editor_mode::selftest_terrain})
		if (auto const result{(this->*group)(command, words)}; result != selftest_result::unhandled)
			return result;
	return selftest_result::unhandled;
}

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
		words >> extent;
		choose_work_area(work_area::tracks);
		if (false == Global.EditorOrtho)
			toggle_ortho();
		if (extent > 0.0f)
			Global.EditorOrthoExtent = std::clamp(extent, 5.0f, 50000.0f);
		WriteLog(format("SELFTEST ortho extent %.0f", Global.EditorOrthoExtent));
		return selftest_result::handled;
	}
	if (command == "mapstate")
	{
		auto const marked{scene::Layers.empty() ? std::vector<std::string>{} : scene::Layers.marked(scene::layer_handle{1}, "//$o")};
		WriteLog(format("SELFTEST mapstate orthophoto %d top %d extent %.0f mark %s", m_orthophoto.enabled() ? 1 : 0, Global.EditorOrtho ? 1 : 0, Global.EditorOrthoExtent, marked.empty() ? "-" : marked.front().c_str()));
		return selftest_result::handled;
	}
	if (command == "trainset")
	{
		std::string name;
		std::string vehicle;
		std::string skin;
		int facing{1};
		words >> name >> vehicle >> skin >> facing >> m_vehicle.velocity;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr)
			return selftest_result::handled;
		auto desc{std::make_shared<ui::vehicle_desc>()};
		desc->path = vehicle;
		auto set{std::make_shared<ui::skin_set>()};
		set->skin = skin;
		m_vehicle.consist.clear();
		m_vehicle.vehicle = desc;
		m_vehicle.skin = set;
		vehicle_start(*track, geometry::bezier{track->m_paths.front()}.point(0.5));
		m_vehicle.facing = facing;
		auto const placed{vehicle_place()};
		WriteLog(format("SELFTEST trainset %s placed %d pending %d: %s", name.c_str(), placed ? 1 : 0, static_cast<int>(m_vehicle.trainsets.size()), m_vehicle.status.c_str()));
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

editor_mode::selftest_result editor_mode::selftest_sweep(std::string const &command, std::istringstream &words)
{
	if (command == "sweep")
	{
		std::map<std::string, std::string> values;
		std::string pair;
		while (words >> pair)
		{
			auto const equals{pair.find('=')};
			if (equals != std::string::npos)
				values[pair.substr(0, equals)] = pair.substr(equals + 1);
		}
		auto &tool{m_sweep};
		auto const get = [&](char const *Key, std::string const &Default) { auto const found{values.find(Key)}; return found != values.end() ? found->second : Default; };
		tool.edited = nullptr;
		tool.curve_for = nullptr;
		std::snprintf(tool.model, sizeof(tool.model), "%s", get("model", "").c_str());
		tool.parameters.clear();
		std::stringstream parameters(get("params", ""));
		std::string parameter;
		while (std::getline(parameters, parameter, ','))
		{
			tool.parameters.emplace_back();
			std::snprintf(tool.parameters.back().data(), tool.parameters.back().size(), "%s", parameter.c_str());
		}
		tool.parameters_for = tool.model;
		auto &settings{tool.settings};
		settings.lateral = std::stod(get("lateral", "0"));
		settings.height = std::stod(get("height", "0"));
		auto const side{get("side", "origin")};
		settings.side_anchor = side == "near" ? 1 : side == "far" ? 2 : side == "centre" ? 3 : 0;
		auto const top{get("top", "origin")};
		settings.height_anchor = top == "bottom" ? 1 : top == "top" ? 2 : top == "edge" ? 3 : 0;
		settings.bend = get("mode", "bend") != "repeat";
		settings.step = std::stod(get("step", "0"));
		settings.widen = get("widen", "0") == "1";
		settings.platform = get("platform", "0") == "1";
		settings.face = get("face", "1") == "1";
		settings.tilt = get("tilt", "0") == "1";
		settings.from = std::stod(get("from", "0"));
		settings.to = std::stod(get("to", "-1"));
		auto const length{get("length", "all")};
		tool.length_mode = length == "original" ? 0 : length == "all" ? 2 : 1;
		if (tool.length_mode == 1)
			tool.length = std::stod(length);
		if (values.count("edge"))
			EditorSettings.platform_edge(std::stod(values["edge"]));
		tool.measured.clear();
		tool.open = true;
		sweep_curve();
		sweep_measure();
		if (values.count("widen"))
			settings.widen = get("widen", "0") == "1";
		if (values.count("platform"))
			settings.platform = get("platform", "0") == "1";
		if (get("create", "1") == "1")
		{
			sweep_create();
			WriteLog("SELFTEST sweep: " + tool.status);
		}
		return selftest_result::handled;
	}
	if (command == "drag")
	{
		glm::dvec3 from{0.0};
		glm::dvec3 to{0.0};
		words >> from.x >> from.z >> to.x >> to.z;
		m_sweep.open = true;
		if (false == sweep_press_at(from, false))
		{
			WriteLog("SELFTEST drag: the press wasn't taken");
			return selftest_result::handled;
		}
		sweep_drag_at(to);
		sweep_release();
		WriteLog("SELFTEST drag: " + m_sweep.status);
		return selftest_result::handled;
	}
	if (command == "fouling")
	{
		std::map<std::string, std::string> values;
		std::string pair;
		while (words >> pair)
		{
			auto const equals{pair.find('=')};
			if (equals != std::string::npos)
				values[pair.substr(0, equals)] = pair.substr(equals + 1);
		}
		auto &tool{m_fouling};
		tool.scope = std::stoi(values.count("scope") ? values["scope"] : "2");
		tool.height_mode = values["mode"] == "ground" ? 0 : 1;
		tool.height = std::stod(values.count("height") ? values["height"] : "0");
		tool.dirty = true;
		fouling_update();
		WriteLog("SELFTEST fouling: " + tool.status);
		int shown{0};
		for (auto const &found : tool.found)
			if (found.wrong && shown++ < 5)
				WriteLog(format("SELFTEST fouling: %s %.2f m off, %+.3f m high", found.marker.file.c_str(), found.error, found.height_error));
		if (values["place"] == "1")
		{
			fouling_place();
			tool.dirty = true;
			fouling_update();
			WriteLog("SELFTEST fouling after placing: " + tool.status);
		}
		if (values["fix"] == "1")
		{
			fouling_fix();
			tool.dirty = true;
			fouling_update();
			WriteLog("SELFTEST fouling after the fix: " + tool.status);
		}
		return selftest_result::handled;
	}
	if (command == "hekto")
	{
		std::map<std::string, std::string> values;
		std::string pair;
		while (words >> pair)
		{
			auto const equals{pair.find('=')};
			if (equals != std::string::npos)
				values[pair.substr(0, equals)] = pair.substr(equals + 1);
		}
		auto *track{simulation::Paths.find(values["track"])};
		if (track == nullptr)
		{
			WriteLog("SELFTEST hekto: no such track");
			return selftest_result::handled;
		}
		hekto_start(*track, track->m_paths.front().points[segment_data::point::start]);
		auto &tool{m_hekto};
		tool.dirty = true;
		hekto_update();
		if (values.count("km"))
		{
			tool.km = std::stod(values["km"]);
			tool.growth = values["growth"] == "-1" ? -1 : 1;
		}
		tool.side = values["side"] == "alt" ? 0 : values["side"] == "left" ? -1 : 1;
		tool.dirty = true;
		hekto_update();
		auto const missing{std::count_if(tool.posts.begin(), tool.posts.end(), [](hekto_post const &Post) { return Post.standing < 0; })};
		WriteLog(format("SELFTEST hekto: km %.3f growth %d, %zu posts along %.0f m, %d to place, %zu standing. %s", tool.km, tool.growth, tool.posts.size(), tool.samples.empty() ? 0.0 : tool.samples.back().chainage, static_cast<int>(missing), tool.found.size(), tool.status.c_str()));
		for (std::size_t i = 0; i < tool.posts.size() && i < 4; ++i)
		{
			auto const &post{tool.posts[i]};
			double lateral;
			glm::dvec3 const axis{editor_track::sampled_position(tool.samples, post.chainage)};
			auto const direction{editor_track::sampled_position(tool.samples, post.chainage + 1.0) - axis};
			glm::dvec2 const offset{post.position.x - axis.x, post.position.z - axis.z};
			lateral = glm::length(offset);
			auto const right{(direction.x * offset.y - direction.z * offset.x) * tool.growth >= 0.0};
			WriteLog(format("SELFTEST hekto post %d.%d at %.1f m, %.2f m from the axis, %s of the growing kilometrage, at %.1f %.1f %.1f", post.hectometres / 10, post.hectometres % 10, post.chainage, lateral, right ? "right" : "left", post.position.x, post.position.y,
			                post.position.z));
		}
		if (values["place"] == "1")
		{
			hekto_place();
			WriteLog("SELFTEST hekto: " + tool.status);
		}
		return selftest_result::handled;
	}
	if (command == "spread")
	{
		std::string name;
		words >> name >> m_spread.stop >> m_spread.reach;
		if (auto *track{simulation::Paths.find(name)})
		{
			std::string names;
			for (auto const *other : track_spread_from(*track))
				names += ' ' + other->name();
			WriteLog("SELFTEST spread " + name + ":" + names + " | prev " + (track->trPrev ? track->trPrev->name() : std::string{"-"}) + " next " + (track->trNext ? track->trNext->name() : std::string{"-"}));
		}
		return selftest_result::handled;
	}
	if (command == "turntable")
	{
		glm::dvec3 centre{0.0};
		double yaw{0.0};
		words >> centre.x >> centre.y >> centre.z >> yaw;
		turntable_place(centre, yaw);
		for (double angle; words >> angle;)
		{
			m_turntable.heading = m_turntable.table_yaw + angle;
			m_turntable.exit_length = 10.0;
			track_intent intent;
			intent.what = track_intent::kind::turntable_exit;
			turntable_press(intent);
		}
		WriteLog("SELFTEST turntable: " + (m_turntable.error.empty() ? m_turntable.status : m_turntable.error));
		return selftest_result::handled;
	}
	if (command == "bend" || command == "bendset")
	{
		std::map<std::string, std::string> values;
		std::string pair;
		while (words >> pair)
		{
			auto const equals{pair.find('=')};
			if (equals != std::string::npos)
				values[pair.substr(0, equals)] = pair.substr(equals + 1);
		}
		auto const vector = [](std::string Text) {
			std::replace(Text.begin(), Text.end(), ',', ' ');
			std::istringstream numbers(Text);
			glm::dvec3 result{0.0};
			numbers >> result.x >> result.y >> result.z;
			return result;
		};
		auto const index = [](std::string const &Text, std::vector<std::string> const &Names) {
			return static_cast<int>(std::find(Names.begin(), Names.end(), Text) - Names.begin());
		};
		auto &tool{m_bend};
		auto const along{values.count("along") ? index(values["along"], {"start", "origin", "middle", "end", "point"}) : tool.along_anchor};
		auto const side{values.count("side") ? index(values["side"], {"origin", "near", "far", "middle", "point"}) : tool.side_anchor};
		auto const height{values.count("height") ? index(values["height"], {"origin", "bottom", "top", "edge", "point"}) : tool.height_anchor};
		auto const point{values.count("point") ? vector(values["point"]) : tool.point};
		if (command == "bend")
		{
			tool.edited = nullptr;
			tool.along_anchor = along;
			tool.side_anchor = side;
			tool.height_anchor = height;
			tool.point = point;
			tool.axis = values.count("axis") ? index(values["axis"], {"auto", "z", "x"}) : 0;
			tool.turned = values["turned"] == "1";
			bend_source source;
			if (false == bend_source_of_selection(source))
			{
				std::string reason;
				auto const editable{m_node != nullptr && scene::Layers.editable(m_node, &reason)};
				WriteLog(format("SELFTEST bend: nothing to bend (node %p template %d editable %d %s, instance %d)", static_cast<void *>(m_node), m_node != nullptr && m_node->from_template() ? 1 : 0, editable ? 1 : 0, reason.c_str(), static_cast<int>(m_instance)));
				return selftest_result::handled;
			}
			tool.tracks_for = nullptr;
			bend_find_tracks(glm::dvec3(source.transform[3]), source.model != nullptr ? static_cast<void const *>(source.model) : reinterpret_cast<void const *>(static_cast<std::uintptr_t>(source.instance)));
			tool.track = values.count("track") ? std::stoi(values["track"]) : 0;
			if (values.count("pick"))
				bend_pick(vector(values["pick"]));
			if (tool.tracks.empty())
			{
				WriteLog("SELFTEST bend: no track nearby");
				return selftest_result::handled;
			}
			sweep_node::state state;
			std::vector<TTrack *> tracks;
			bend_definition(source, *tool.tracks[tool.track].first, state, tracks);
			WriteLog(format("SELFTEST bend: track %s, %zu paths, anchor before %.3f %.3f %.3f, lateral %.3f height %.3f from %.3f to %.3f, axis %s flip %d; %s", tool.tracks[tool.track].first->name().c_str(), tracks.size(),
			                tool.marker.x, tool.marker.y, tool.marker.z, state.lateral, state.height, state.from, state.to, state.along_x ? "x" : "z", state.flip ? 1 : 0, tool.status.c_str()));
			if (values["go"] == "1")
			{
				bend_selection();
				WriteLog("SELFTEST bend: " + tool.status);
			}
		}
		else if (auto *sweep{tool.edited})
		{
			auto state{sweep->definition()};
			tool.along_anchor = along;
			if (side != tool.side_anchor || height != tool.height_anchor || point != tool.point)
			{
				tool.side_anchor = side;
				tool.height_anchor = height;
				tool.point = point;
				bend_reanchor(state, side, height, point);
			}
			if (values.count("lateral"))
				state.lateral = std::stod(values["lateral"]);
			if (values.count("up"))
				state.height = std::stod(values["up"]);
			if (values.count("widen"))
				state.widen = values["widen"] == "1";
			if (values.count("pick"))
			{
				bend_pick(vector(values["pick"]));
				state = sweep->definition();
			}
			if (values.count("length"))
			{
				bend_stretch(*sweep, bend_fixed(*sweep, state.point), std::stod(values["length"]), state.point, state);
			}
			bend_edit(state);
			WriteLog("SELFTEST bendset done");
		}
		return selftest_result::handled;
	}
	if (command == "bendprobe")
	{
		auto *sweep{m_bend.edited};
		if (sweep == nullptr)
		{
			WriteLog("SELFTEST bendprobe: nothing bent");
			return selftest_result::handled;
		}
		glm::dvec3 low;
		glm::dvec3 high;
		glm::dvec3 shift;
		sweep->bounds(low, high, shift);
		auto const &state{sweep->definition()};
		auto const marker{bend_marker(*sweep)};
		glm::dvec3 boxlow{std::numeric_limits<double>::max()};
		glm::dvec3 boxhigh{-std::numeric_limits<double>::max()};
		std::size_t vertices{0};
		for (auto const &shape : sweep->create_shapes())
			for (auto const &vertex : shape.data().vertices)
			{
				boxlow = glm::min(boxlow, vertex.position);
				boxhigh = glm::max(boxhigh, vertex.position);
				++vertices;
			}
		WriteLog(format("SELFTEST bendprobe %s: stations %.3f - %.3f, along the reference line %.3f m, model %.3f m, lateral %.3f height %.3f anchors %d/%d point %.3f %.3f %.3f, marker %.3f %.3f %.3f, box %.2f %.2f %.2f - %.2f %.2f %.2f, %zu vertices, removed %d",
		                sweep->name().c_str(), sweep->start(), sweep->end(), sweep->distance_at(sweep->end()) - sweep->distance_at(sweep->start()), high.x - low.x, state.lateral, state.height, state.side_anchor, state.height_anchor, state.point.x,
		                state.point.y, state.point.z, marker.x, marker.y, marker.z, boxlow.x, boxlow.y, boxlow.z, boxhigh.x, boxhigh.y, boxhigh.z, vertices, sweep->m_editorremoved ? 1 : 0));
		return selftest_result::handled;
	}
	if (command == "lineside")
	{
		std::string tab;
		words >> tab;
		show_track_tab(tab == "parallel" ? track_tab::parallel : track_tab::lineside);
		(tab == "vehicle" ? m_vehicle.expand : tab == "hekto" ? m_hekto.expand : tab == "fouling" ? m_fouling.expand : m_parallel.expand) = true;
		return selftest_result::handled;
	}
	if (command == "bendpanel")
	{
		ui()->expand_bend();
		return selftest_result::handled;
	}
	if (command == "gauge")
	{
		std::string name;
		words >> name;
		if (auto *track{simulation::Paths.find(name)})
		{
			scan_gauge(*track);
			WriteLog(format("SELFTEST gauge %s: %d hits", name.c_str(), static_cast<int>(m_gauge.line.hits.size())));
			for (auto const &hit : m_gauge.line.hits)
				WriteLog(format("SELFTEST gauge hit %s depth %.3f at %.2f %.2f %.2f", hit.model.c_str(), hit.depth, hit.point.x, hit.point.y, hit.point.z));
		}
		else
			WriteLog("SELFTEST gauge " + name + ": no such track");
		return selftest_result::handled;
	}
	if (command == "ttincludes")
	{
		std::string name;
		words >> name;
		auto const *track{simulation::Paths.find(name)};
		WriteLog(format("SELFTEST ttincludes %s: track %s, includes %d", name.c_str(), track == nullptr ? "none" : track->m_editorremoved ? "removed" : "present",
		                track != nullptr ? static_cast<int>(turntable_includes(*track).size()) : -1));
		return selftest_result::handled;
	}
	if (command == "checkspans")
	{
		WriteLog("SELFTEST spans: " + scene::Layers.check_sources());
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

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

editor_mode::selftest_result editor_mode::selftest_switch(std::string const &command, std::istringstream &words)
{
	if (command == "curvedswitch")
	{
		std::string name;
		std::string label;
		glm::dvec3 at{0.0};
		glm::dvec3 toward{0.0};
		words >> name >> at.x >> at.z >> toward.x >> toward.z >> label;
		m_node = simulation::Paths.find(name);
		ui()->set_node(m_node);
		show_track_tab(track_tab::turnout);
		arm_switch();
		auto &tool{m_switch};
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
			if (false == label.empty() && tool.templates[i].label.find(label) != std::string::npos)
			{
				tool.armed = i;
				break;
			}
		m_cursor_override = at;
		auto const started{start_switch_placement()};
		m_cursor_override.reset();
		if (false == started)
		{
			WriteLog("SELFTEST curvedswitch: not started, " + tool.status);
			return selftest_result::handled;
		}
		tool.mouse = glm::dvec3{toward.x, tool.point.y, toward.z};
		finish_switch_placement();
		WriteLog("SELFTEST curvedswitch " + (tool.armed >= 0 ? tool.templates[tool.armed].label : std::string{}) + ": " + tool.status);
		return selftest_result::handled;
	}
	if (command == "conecheck")
	{
		std::string name;
		words >> name;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr || track->m_paths.size() < 2)
		{
			WriteLog("SELFTEST conecheck: no switch " + name);
			return selftest_result::handled;
		}
		geometry::bezier const main{track->m_paths[0]};
		geometry::bezier const other{track->m_paths[1]};
		std::string text;
		for (int i = 1; i <= 8; ++i)
		{
			auto const t{i * 0.125};
			auto const point{other.point(t)};
			double best{std::numeric_limits<double>::max()};
			double along{0.0};
			for (int k = 0; k <= 2000; ++k)
				if (auto const distance{geometry::plan_distance(main.point(k / 2000.0), point)}; distance < best)
				{
					best = distance;
					along = k / 2000.0;
				}
			auto const base{main.point(along)};
			auto const direction{glm::normalize(geometry::plan_of(main.first(along)))};
			auto const lateral{(point.x - base.x) * -direction.y + (point.z - base.z) * direction.x};
			auto const &path{track->m_paths[0]};
			auto const roll{glm::radians(path.rolls[0] + (path.rolls[1] - path.rolls[0]) * along)};
			auto const tangent{main.first(along)};
			auto const beyond{(point.x - base.x) * direction.x + (point.z - base.z) * direction.y};
			auto const cone{base.y + beyond * tangent.y / glm::length(geometry::plan_of(tangent)) - lateral * std::tan(roll)};
			text += format(" [t %.3f off %.3f dy %.4f]", t, lateral, point.y - cone);
		}
		WriteLog("SELFTEST conecheck " + name + ":" + text);
		return selftest_result::handled;
	}
	if (command == "switchdrive")
	{
		int index{-1};
		int side{0};
		words >> index >> side;
		scan_switch_drives();
		m_switch.drive = index;
		m_switch.drive_side = side;
		WriteLog("SELFTEST switchdrive: " + (index >= 0 && index < static_cast<int>(m_switch.drives.size()) ? m_switch.drives[index].name : std::string{"none"}));
		return selftest_result::handled;
	}
	if (command == "drivechoice")
	{
		int index{-1};
		int side{0};
		words >> index >> side;
		scan_switch_drives();
		m_switch.drive = index;
		m_switch.drive_side = side;
		m_switch.status.clear();
		auto *track{selected_track()};
		if (track != nullptr && track->eType == tt_Switch && false == track->m_editorremoved)
			apply_switch_drive_choice(*track);
		WriteLog("SELFTEST drivechoice: " + (track != nullptr ? track->name() : std::string{"no switch"}) + ", " + m_switch.status);
		return selftest_result::handled;
	}
	if (command == "drives")
	{
		std::string name;
		words >> name;
		scan_switch_drives();
		std::string files;
		for (auto const &standing : standing_templates([](std::string const &File) { return File.find("napedy/") != std::string::npos; }))
			if (false == standing.values.empty() && standing.values[0] == name)
			{
				files += " " + standing.file;
				for (std::size_t i = 1; i < standing.values.size(); ++i)
					files += (i == 1 ? " (" : " ") + standing.values[i] + (i + 1 == standing.values.size() ? ")" : "");
			}
		for (scene::instance_handle handle = 1; handle <= scene::Layers.instance_count(); ++handle)
			if (scene::Layers.tracked(handle) && scene::Layers.instance(handle).file != nullptr && scene::Layers.instance(handle).file->find("napedy") != std::string::npos)
				files += format(" [%d %s removed %d dead %d]", static_cast<int>(handle), scene::Layers.instance(handle).file->c_str(), scene::Layers.instance(handle).removed ? 1 : 0, scene::Layers.instance(handle).dead ? 1 : 0);
		WriteLog("SELFTEST drives " + name + ": chosen " + (m_switch.drive >= 0 ? m_switch.drives[m_switch.drive].name : std::string{"none"}) + ", side " + std::to_string(m_switch.drive_side) + ", standing" + files + ", steps " + std::to_string(m_history.size()));
		return selftest_result::handled;
	}
	if (command == "driveflip")
	{
		std::string name;
		words >> name;
		auto *track{simulation::Paths.find(name)};
		auto const done{track != nullptr && flip_switch_drive(*track)};
		WriteLog("SELFTEST driveflip " + name + ": " + (done ? "done " : "failed ") + m_switch.status);
		return selftest_result::handled;
	}
	if (command == "driveput" || command == "drivefamily")
	{
		std::string name;
		int family{-1};
		words >> name >> family;
		auto *track{simulation::Paths.find(name)};
		auto const done{track != nullptr && (command == "driveput" ? place_switch_drive(*track) : replace_switch_drive(*track, family, false))};
		WriteLog("SELFTEST " + command + " " + name + ": " + (done ? "done " : "failed ") + m_switch.status);
		return selftest_result::handled;
	}
	if (command == "drivetilt")
	{
		std::string name;
		words >> name;
		auto *model{simulation::Instances.find(name)};
		auto *track{simulation::Paths.find(name.substr(0, name.find('_')))};
		if (model == nullptr || track == nullptr)
		{
			WriteLog("SELFTEST drivetilt: no " + name);
			return selftest_result::handled;
		}
		auto const &path{track->m_paths[0]};
		auto const transform{glm::dmat3(model->rotation_scale())};
		auto const across{transform * glm::dvec3{1.0, 0.0, 0.0}};
		auto const forward{transform * glm::dvec3{0.0, 0.0, 1.0}};
		auto const direction{glm::normalize(glm::dvec2{forward.x, forward.z})};
		auto const lateral{across.x * -direction.y + across.z * direction.x};
		auto const cone{-lateral * std::tan(glm::radians(static_cast<double>(path.rolls[0])))};
		auto const control{path.points[segment_data::point::control1]};
		auto const grade{control.y / std::hypot(control.x, control.z)};
		WriteLog(format("SELFTEST drivetilt %s: angles %.3f %.3f %.3f, +x rises %.4f (cone %.4f), +z rises %.4f (grade %.4f), origin %.3f %.3f %.3f, points %.3f", name.c_str(), model->Angles().x, model->Angles().y, model->Angles().z, across.y, cone, forward.y, grade,
		                model->location().x, model->location().y, model->location().z, path.points[segment_data::point::start].y));
		return selftest_result::handled;
	}
	if (command == "switchplace" || command == "switchcurve" || command == "slipplace")
	{
		std::string label;
		glm::dvec3 point{0.0};
		int direction{1};
		int side{1};
		words >> label >> point.x >> point.z >> direction >> side;
		std::replace(label.begin(), label.end(), '_', ' ');
		auto &tool{m_switch};
		tool.templates = editor_track::standard_switch_templates();
		tool.collected = true;
		tool.armed = -1;
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
			if (tool.templates[i].label.rfind(label, 0) == 0)
			{
				tool.armed = i;
				break;
			}
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
		if (tool.armed < 0 || nearest == nullptr)
		{
			WriteLog("SELFTEST " + command + ": no template " + label + " or no path");
			return selftest_result::handled;
		}
		tool.status.clear();
		if (command == "switchcurve")
		{
			editor_track::curve curve;
			editor_track::chain chain;
			std::string error;
			if (false == editor_track::find_curve(*nearest, m_straights.tolerance, m_route.design.norms.gauge, curve) || false == editor_track::find_chain(curve.from, curve.to, chain, error, true))
			{
				WriteLog("SELFTEST switchcurve: no curve");
				return selftest_result::handled;
			}
			tool.frame = sample_chain(chain);
			tool.frame_tracks = chain.tracks;
			tool.curved = true;
			tool.along = nearest_station(tool.frame, point);
			insert_curved_switch(direction, side);
		}
		else
		{
			auto const line{editor_track::find_straight(*nearest, m_straights.tolerance)};
			if (command == "slipplace")
			{
				WriteLog(format("SELFTEST slipplace: straight from %s, %zu paths, start %.3f %.3f, %.3f m", nearest->name().c_str(), line.tracks.size(), line.start.x, line.start.z, line.length));
				insert_double_slip(line, point, tool.templates[tool.armed]);
			}
			else
				insert_switch(line, std::clamp(line.along(point), 0.0, line.length), direction, side);
		}
		WriteLog("SELFTEST " + command + ": " + tool.status + " " + m_straights.status);
		return selftest_result::handled;
	}
	if (command == "switchlink" || command == "branchlink")
	{
		std::string label;
		glm::dvec3 point{0.0};
		glm::dvec3 target{0.0};
		if (command == "switchlink")
			words >> label;
		words >> point.x >> point.z >> target.x >> target.z;
		std::replace(label.begin(), label.end(), '_', ' ');
		auto const log_link = [&](editor_track::branch_link const &Link, std::string const &Reason) {
			WriteLog(format("SELFTEST %s plan: valid %d, R %.1f m, before %.3f m, after %.3f m, trim %.3f m, shortfall %.3f m, %zu pieces%s", command.c_str(), Link.valid ? 1 : 0, Link.radius, Link.before, Link.after, Link.trim,
			                Link.shortfall, Link.pieces.size(), Reason.empty() ? "" : (", " + Reason).c_str()));
			for (auto const &piece : Link.pieces)
			{
				auto const &a{piece.points[segment_data::point::start]};
				auto const &b{piece.points[segment_data::point::end]};
				WriteLog(format("SELFTEST %s piece: %.3f %.3f %.3f -> %.3f %.3f %.3f, L %.3f m, R %.1f m", command.c_str(), a.x, a.y, a.z, b.x, b.y, b.z, geometry::bezier{piece}.plan_length(), static_cast<double>(piece.radius)));
			}
		};
		if (command == "switchlink")
		{
			show_track_tab(track_tab::turnout);
			auto &tool{m_switch};
			tool.templates = editor_track::standard_switch_templates();
			tool.collected = true;
			tool.armed = -1;
			for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
				if (tool.templates[i].label.rfind(label, 0) == 0)
				{
					tool.armed = i;
					break;
				}
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
			if (tool.armed < 0 || nearest == nullptr)
			{
				WriteLog("SELFTEST switchlink: no template " + label + " or no path");
				return selftest_result::handled;
			}
			m_node = nearest;
			ui()->set_node(m_node);
			m_cursor_override = point;
			if (false == start_switch_placement())
			{
				m_cursor_override.reset();
				WriteLog("SELFTEST switchlink: not started, " + tool.status);
				return selftest_result::handled;
			}
			tool.mouse = glm::dvec3{target.x, tool.point.y, target.z};
			switch_link plan;
			plan_switch_link(plan);
			log_link(plan.link, plan.reason);
			finish_switch_placement();
			m_cursor_override.reset();
			WriteLog("SELFTEST switchlink: " + tool.status);
			return selftest_result::handled;
		}
		TTrack *own{nullptr};
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && false == track->m_editorremoved && track->eType == tt_Switch && track->m_paths.size() == 2 && geometry::plan_distance(track->m_paths[1].points[segment_data::point::end], point) < 3.0)
				own = track;
		if (own == nullptr)
		{
			WriteLog("SELFTEST branchlink: no switch branch end there");
			return selftest_result::handled;
		}
		editor_track::snap_target end;
		end.track = own;
		end.point = {1, editor_track::point_kind::end};
		end.position = own->m_paths[1].points[segment_data::point::end];
		if (false == start_extend(end))
		{
			WriteLog("SELFTEST branchlink: the branch end isn't free");
			return selftest_result::handled;
		}
		m_extend.pressed = {-10000.f, -10000.f};
		m_extend.mouse = glm::dvec3{target.x, m_extend.point.y, target.z};
		m_extend.snap = editor_track::find_free_end(own, own->iCategoryFlag & 15, m_extend.mouse, 3.0, {own});
		editor_track::branch_link link;
		std::string reason;
		plan_branch_link(link, reason);
		log_link(link, reason);
		finish_extend();
		WriteLog(format("SELFTEST branchlink: switch now %.3f %.3f -> %.3f %.3f", own->m_paths[0].points[segment_data::point::start].x, own->m_paths[0].points[segment_data::point::start].z, own->m_paths[0].points[segment_data::point::end].x,
		                own->m_paths[0].points[segment_data::point::end].z));
		return selftest_result::handled;
	}
	if (command == "switchslide" || command == "switchreplace")
	{
		std::string name;
		words >> name;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr || track->eType != tt_Switch || track->m_paths.size() < 2)
		{
			WriteLog("SELFTEST " + command + ": no switch " + name);
			return selftest_result::handled;
		}
		if (command == "switchslide")
		{
			double distance{0.0};
			words >> distance;
			double low{0.0};
			double high{0.0};
			std::string reason;
			auto const room{switch_room(*track, low, high, reason)};
			auto const done{slide_switch(*track, distance)};
			WriteLog(format("SELFTEST switchslide %s %.3f: room %d %.3f .. %.3f, done %d, %s", name.c_str(), distance, room ? 1 : 0, low, high, done ? 1 : 0, m_switch.status.c_str()));
			return selftest_result::handled;
		}
		std::string label;
		int side{1};
		words >> label >> side;
		std::replace(label.begin(), label.end(), '_', ' ');
		auto const templates{editor_track::standard_switch_templates()};
		auto const found{std::find_if(templates.begin(), templates.end(), [&](editor_track::switch_template const &Template) { return Template.label.rfind(label, 0) == 0; })};
		if (found == templates.end())
		{
			WriteLog("SELFTEST switchreplace: no template " + label);
			return selftest_result::handled;
		}
		auto const &main{track->m_paths[0]};
		auto const origin{main.points[segment_data::point::start]};
		auto const chord{geometry::plan_of(main.points[segment_data::point::end] - origin)};
		auto const length{glm::length(chord)};
		auto paths{editor_track::place_switch(*found, origin, chord / length, side, (main.points[segment_data::point::end].y - origin.y) / length)};
		std::string status;
		auto const done{paths.size() >= 2 && move_switch_paths(*track, paths, status)};
		if (done)
		{
			record_switch(*track, *found);
			records_to_history();
			straight_refresh();
		}
		WriteLog(format("SELFTEST switchreplace %s %s: done %d, %s", name.c_str(), found->label.c_str(), done ? 1 : 0, status.c_str()));
		return selftest_result::handled;
	}
	if (command == "switchpieces" || command == "switchbreak")
	{
		double radius{0.0};
		double ratio{0.0};
		std::string name;
		if (command == "switchpieces")
			words >> radius >> ratio;
		words >> name;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->m_paths.size() < 2 || track->SwitchExtension == nullptr)
				continue;
			if (false == name.empty() ? track->name() != name : track->m_paths[1].pieces.empty())
				continue;
			if (command == "switchbreak")
			{
				auto &pieces{track->m_paths[1].pieces};
				if (pieces.size() < 2)
					continue;
				pieces[0].points[segment_data::point::end].x += 0.3;
				editor_track::commit({track});
				WriteLog(format("SELFTEST switchbreak %s: path 2 %.3f m, %zu pieces in use, %zu kept", track->name().c_str(), track->SwitchExtension->Segments[1]->GetLength(), track->SwitchExtension->Segments[1]->PieceCount(), pieces.size()));
				return selftest_result::handled;
			}
			auto const &segment{*track->SwitchExtension->Segments[1]};
			auto const &main{track->m_paths[0]};
			auto const &branch{track->m_paths[1]};
			auto const origin{segment.FastGetPoint(0.0)};
			auto const plan{glm::normalize(geometry::plan_of(main.points[segment_data::point::end] - main.points[segment_data::point::start]))};
			glm::dvec3 const along{plan.x, 0.0, plan.y};
			auto const reach{branch.points[segment_data::point::end] - branch.points[segment_data::point::start]};
			auto const hand{along.x * reach.z - along.z * reach.x > 0.0 ? 1.0 : -1.0};
			auto const normal{glm::dvec3{-along.z, 0.0, along.x} * hand};
			auto const centre{origin + normal * radius};
			auto const angle{std::atan(1.0 / ratio)};
			auto const arc{radius * angle};
			auto const heading0{-std::atan2(along.x, along.z)};
			glm::dvec3 position;
			glm::vec3 rotation;
			double worst{0.0};
			for (int i = 0; i <= 10; ++i)
			{
				segment.RaPositionGet(arc * i / 10.0, position, rotation);
				worst = std::max(worst, std::abs(geometry::plan_distance(position, centre) - radius));
			}
			segment.RaPositionGet(arc, position, rotation);
			auto const expected{centre - normal * radius * std::cos(angle) + along * radius * std::sin(angle)};
			auto const turn{std::abs(std::remainder(rotation.z - heading0, glm::two_pi<double>()))};
			double low{1e9};
			double high{-1e9};
			for (int i = 0; i <= 10; ++i)
			{
				segment.RaPositionGet(arc + 0.05 + (segment.GetLength() - arc - 0.1) * i / 10.0, position, rotation);
				low = std::min(low, static_cast<double>(rotation.z));
				high = std::max(high, static_cast<double>(rotation.z));
			}
			segment.RaPositionGet(arc, position, rotation);
			WriteLog(format("SELFTEST switchpieces %s: path 2 %.3f m in %zu pieces (%zu kept), arc %.3f m, arc end off %.4f m, turn there %.5f (angle %.5f), axis off the arc %.4f m, straight heading spread %.6f, end %.3f %.3f", track->name().c_str(),
			                segment.GetLength(), segment.PieceCount(), branch.pieces.size(), arc, geometry::plan_distance(position, expected), turn, angle, worst, high - low, segment.FastGetPoint(1.0).x, segment.FastGetPoint(1.0).z));
		}
		return selftest_result::handled;
	}
	if (command == "switches")
	{
		for (auto const &entry : switch_store::records())
		{
			std::string parts;
			for (auto const &part : entry.parts)
				parts += part + " ";
			WriteLog(format("SELFTEST switch record %s %s: %s, R %.3f 1:%.3f hand %c a %.3f b %.3f main R %.3f, branch R %.3f, km %s, number %s, %s, parts %s", entry.kind.c_str(), entry.name.c_str(), switch_store::label(entry).c_str(), entry.radius, entry.ratio, entry.hand, entry.a,
			                entry.b, entry.main_radius, switch_store::branch_radius(entry), entry.km.has_value() ? format("%.3f", *entry.km / 1000.0).c_str() : "-", entry.number.c_str(), entry.source.c_str(), parts.c_str()));
		}
		speed_check::options options;
		options.norms = m_route.design.norms;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->m_paths.size() < 2)
				continue;
			auto const known{switch_store::of(*track)};
			geometry::bezier const branch{track->m_paths[1]};
			auto const first{branch.first(0.3)};
			auto const second{branch.second(0.3)};
			auto const planar{std::abs(first.x * second.z - first.z * second.x) / std::pow(first.x * first.x + first.z * first.z, 1.5)};
			auto const verdict{speed_check::check(*track, 1, options)};
			WriteLog(format("SELFTEST switch %s: %s, branch R at 0.3 %.3f, path R %.3f, branch speed %.1f km/h at R %.1f", track->name().c_str(), known.has_value() ? switch_store::label(*known).c_str() : "no record", planar > 1e-9 ? 1.0 / planar : 0.0,
			                track->m_paths[1].radius, verdict.safe, verdict.radius));
		}
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

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
	if (command == "joints")
	{
		std::vector<glm::dvec3> points;
		for (glm::dvec3 point{0.0}; words >> point.x >> point.z;)
			points.push_back(point);
		auto const scope{m_joints.scope};
		m_joints.scope = 1;
		joints_start();
		while (m_joints.next < m_joints.queue.size())
			joints_step();
		m_joints.scope = scope;
		char const *const kinds[]{"gap", "step", "kink", "cant jump", "grade break", "free end"};
		int around{0};
		for (auto const &issue : m_joints.issues)
		{
			if (issue.type == joint_issue::kind::open)
				continue;
			if (std::none_of(points.begin(), points.end(), [&](glm::dvec3 const &Point) { return geometry::plan_distance(Point, issue.position) < 5.0; }))
				continue;
			++around;
			WriteLog(format("SELFTEST joints: %s %.3f at %.1f %.1f, %s / %s", kinds[static_cast<int>(issue.type)], issue.value, issue.position.x, issue.position.z, issue.track != nullptr ? issue.track->name().c_str() : "-",
			                issue.other != nullptr ? issue.other->name().c_str() : "-"));
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

editor_mode::selftest_result editor_mode::selftest_signal(std::string const &command, std::istringstream &words)
{
	if (command == "signal" || command == "signalmove" || command == "signaldelete" || command == "signalrename" || command == "signalprobe")
	{
		std::map<std::string, std::string> values;
		std::string pair;
		while (words >> pair)
		{
			auto const equals{pair.find('=')};
			if (equals != std::string::npos)
				values[pair.substr(0, equals)] = pair.substr(equals + 1);
		}
		show_track_tab(track_tab::signals);
		signal_refresh(true);
		auto &tool{m_signal};
		auto const standing_named = [&](std::string const &Name) -> signal_standing const * {
			auto const found{std::find_if(tool.standing.begin(), tool.standing.end(), [&](signal_standing const &Standing) { return Standing.name == Name; })};
			return found != tool.standing.end() ? &*found : nullptr;
		};
		auto const spot_on = [&](signal_template const &Template, int const Facing, int const Side, signal_spot &Spot) {
			auto *track{simulation::Paths.find(values["track"])};
			if (track == nullptr)
				return false;
			auto const path{values.count("path") ? std::stoi(values["path"]) : 0};
			auto const at{values.count("at") ? std::stod(values["at"]) : 0.5};
			geometry::bezier const curve{track->m_paths[std::clamp(path, 0, static_cast<int>(track->m_paths.size()) - 1)]};
			return signal_spot_at(*track, path, curve.point(at), curve.point(at), Facing, Side, Template, Spot);
		};
		if (command == "signal")
		{
			auto const *item{signal_template_of(values["file"])};
			if (item == nullptr)
			{
				WriteLog("SELFTEST signal: no signal template " + values["file"]);
				return selftest_result::handled;
			}
			for (int k = 0; k < static_cast<int>(tool.templates.size()); ++k)
				if (&tool.templates[k] == item)
				{
					static char const *const kinds[] = {"main", "block", "warning", "shunt", "stop", "substitute", "repeater"};
					for (int i = 0; i < 7; ++i)
						if (item->kind == kinds[i])
						{
							tool.kind = i;
							tool.chosen[i] = k;
						}
				}
			if (values.count("name"))
				std::snprintf(tool.name, sizeof(tool.name), "%s", values["name"].c_str());
			if (values.count("plate"))
				std::snprintf(tool.plate, sizeof(tool.plate), "%s", values["plate"].c_str());
			auto const side{values["side"] == "left" ? -1 : 1};
			auto const facing{values["facing"] == "along" ? 1 : values["facing"] == "against" ? -1 : 0};
			signal_spot spot;
			if (false == spot_on(*signal_armed(), facing, side, spot))
			{
				WriteLog("SELFTEST signal: no such track " + values["track"]);
				return selftest_result::handled;
			}
			auto const placed{signal_place(spot, *signal_armed())};
			WriteLog(format("SELFTEST signal %s: %s", placed ? "placed" : "failed", tool.status.c_str()));
		}
		else if (command == "signalmove")
		{
			auto const *standing{standing_named(values["name"])};
			auto const *item{standing != nullptr ? signal_template_of(standing->include.file) : nullptr};
			if (item == nullptr)
			{
				WriteLog("SELFTEST signalmove: no signal " + values["name"]);
				return selftest_result::handled;
			}
			auto const yaw{glm::radians(standing->include.yaw + 180.0)};
			glm::dvec2 const travel{std::sin(yaw), std::cos(yaw)};
			signal_spot probe;
			if (false == spot_on(*item, 1, 1, probe))
			{
				WriteLog("SELFTEST signalmove: no such track " + values["track"]);
				return selftest_result::handled;
			}
			auto const facing{glm::dot(probe.travel, travel) >= 0.0 ? 1 : -1};
			auto const sideoftravel{glm::dot(geometry::plan_of(standing->include.location) - geometry::plan_of(standing->spot.axis), glm::dvec2{-travel.y, travel.x}) >= 0.0 ? 1 : -1};
			signal_select(standing->include.instance);
			tool.moving = true;
			tool.moved_valid = spot_on(*item, facing, sideoftravel * facing, tool.moved);
			tool.pressed = {-1000.f, -1000.f};
			signal_release();
			WriteLog("SELFTEST signalmove: " + tool.status);
		}
		else if (command == "signaldelete")
		{
			auto const *standing{standing_named(values["name"])};
			auto const deleted{standing != nullptr && signal_delete(standing->include.instance)};
			WriteLog(std::string{"SELFTEST signaldelete "} + values["name"] + (deleted ? ": removed with its event" : ": not removed by the signal tool"));
		}
		else if (command == "signalrename")
		{
			auto const *standing{standing_named(values["name"])};
			if (standing == nullptr)
			{
				WriteLog("SELFTEST signalrename: no signal " + values["name"]);
				return selftest_result::handled;
			}
			select_include(standing->include.instance);
			m_include.values[0] = values["to"];
			apply_include();
			m_include_gesture = false;
			WriteLog("SELFTEST signalrename " + values["name"] + " -> " + values["to"]);
		}
		else
		{
			signal_refresh(true);
			WriteLog(format("SELFTEST signalprobe: %zu signals", tool.standing.size()));
			for (auto const &standing : tool.standing)
			{
				std::string where{"not read"};
				if (standing.track != nullptr)
				{
					auto const &curve{standing.track->m_paths.front()};
					geometry::bezier const bez{curve};
					auto best{std::numeric_limits<double>::max()};
					double parameter{0.0};
					for (int k = 0; k <= 1024; ++k)
					{
						auto const d{geometry::plan_distance(bez.point(k / 1024.0), standing.include.location)};
						if (d < best)
						{
							best = d;
							parameter = k / 1024.0;
						}
					}
					auto const axis{bez.point(parameter)};
					auto tangent{bez.first(parameter)};
					glm::dvec2 travel{tangent.x, tangent.z};
					travel = glm::normalize(travel) * (standing.list == 1 ? -1.0 : 1.0);
					glm::dvec2 const offset{standing.include.location.x - axis.x, standing.include.location.z - axis.z};
					auto const cross{travel.x * offset.y - travel.y * offset.x};
					auto heading{glm::degrees(std::atan2(travel.x, travel.y))};
					auto turn{std::fmod(standing.include.yaw - heading + 720.0, 360.0)};
					auto const &list{editor_track::events(*standing.track, standing.list)};
					auto const bound{std::any_of(list.begin(), list.end(), [&](auto const &Event) { return Event.second != nullptr && ToLower(Event.first) == ToLower(standing.read); })};
					where = std::string{bound ? "bound, " : "unbound, "} + format("%s of %s, %.3f m from the axis, %s of the trains, %.3f m over the axis, model turned %.1f deg from the travel", editor_track::event_keyword(standing.list), standing.track->name().c_str(), glm::length(offset), cross > 0.0 ? "right" : "left",
					               standing.include.location.y - axis.y, turn);
				}
				WriteLog(format("SELFTEST signalprobe %s %s at %.3f %.3f %.3f yaw %.2f read %s: %s", standing.name.c_str(), standing.include.file.c_str(), standing.include.location.x, standing.include.location.y, standing.include.location.z, standing.include.yaw,
				                standing.read.empty() ? "-" : standing.read.c_str(), where.c_str()));
			}
		}
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_terrain(std::string const &command, std::istringstream &words)
{
	if (command == "orthophoto")
	{
		auto config{m_orthophoto.settings()};
		int fit{1};
		int inscene{0};
		words >> config.north >> config.east >> config.radius >> fit >> inscene;
		config.drape = fit != 0;
		config.in_scene = inscene != 0;
		m_orthophoto.settings(config);
		m_orthophoto.enabled(true);
		WriteLog(format("SELFTEST orthophoto at %.0f %.0f, radius %d, %s%s", config.north, config.east, config.radius, config.drape ? "fitted" : "flat", config.in_scene ? ", in the scene" : ""));
		return selftest_result::handled;
	}
	if (command == "terrainmode")
	{
		m_terrain_open = true;
		WriteLog("SELFTEST terrainmode");
		return selftest_result::handled;
	}
	if (command == "terrainsculpt" || command == "terrainpaint" || command == "terrainlevel")
	{
		double x{0.0};
		double z{0.0};
		double radius{10.0};
		double value{1.0};
		int mode{0};
		words >> x >> z >> radius >> value >> mode;
		auto const changed{command == "terrainsculpt" ? m_streamer.sculpt(x, z, radius, value)
		                   : command == "terrainlevel" ? m_streamer.level(x, z, radius, value, 1.0, mode)
		                                               : m_streamer.paint(x, z, radius, 1.0, static_cast<std::uint16_t>(value))};
		forget_ground();
		WriteLog(format("SELFTEST %s %.1f %.1f: %s", command.c_str(), x, z, changed ? "changed" : "nothing changed"));
		return selftest_result::handled;
	}
	if (command == "terrainmodifier")
	{
		heightmap::modifier modifier;
		heightmap::corridor_point a, b;
		double formation{0.0};
		double halfwidth{3.5};
		words >> modifier.name >> a.position.x >> a.position.y >> b.position.x >> b.position.y >> formation >> halfwidth;
		b.formation = formation;
		a.formation = b.formation;
		b.half_width = halfwidth;
		a.half_width = b.half_width;
		modifier.points = {a, b};
		m_streamer.modifier(std::move(modifier));
		forget_ground();
		WriteLog(format("SELFTEST terrainmodifier: %zu modifiers", m_streamer.modifiers().size()));
		return selftest_result::handled;
	}
	if (command == "terrainfill")
	{
		int layer{0};
		words >> layer >> m_paint_feather >> m_paint_rotation;
		m_terrain_layer = layer;
		m_paint_outline.clear();
		double x;
		double z;
		while (words >> x >> z)
			m_paint_outline.emplace_back(x, 0.0, z);
		fill_terrain_area();
		std::string palette;
		for (auto const &entry : m_streamer.manifest().layers)
			palette += format(" [%s %.1f m %.0f deg]", entry.material.c_str(), entry.size, entry.rotation);
		WriteLog("SELFTEST terrainfill: " + m_terrain_status + ", palette" + palette);
		return selftest_result::handled;
	}
	if (command == "terrainturn")
	{
		int layer{0};
		float degrees{0.f};
		words >> layer >> degrees;
		m_streamer.layer_rotation(static_cast<std::uint16_t>(layer), degrees);
		WriteLog(format("SELFTEST terrainturn %d: %.1f deg", layer, m_streamer.manifest().layers.at(layer).rotation));
		return selftest_result::handled;
	}
	if (command == "terraintouchup")
	{
		int state{1};
		words >> state;
		m_streamer.touch_up(state != 0);
		WriteLog(format("SELFTEST terraintouchup %d", state));
		return selftest_result::handled;
	}
	if (command == "terrainrestore")
	{
		double x{0.0};
		double z{0.0};
		double radius{10.0};
		words >> x >> z >> radius;
		auto const changed{m_streamer.restore_stitching(x, z, radius, 1.0)};
		forget_ground();
		WriteLog(format("SELFTEST terrainrestore %.1f %.1f: %s, touch-ups left %zu", x, z, changed ? "changed" : "nothing changed", m_streamer.touched_up()));
		return selftest_result::handled;
	}
	if (command == "terrainbake")
	{
		std::string name;
		words >> name;
		auto const baked{m_streamer.bake_modifier(name)};
		forget_ground();
		WriteLog(format("SELFTEST terrainbake %s: %s, %zu modifiers left", name.c_str(), baked ? "baked" : "not found", m_streamer.modifiers().size()));
		return selftest_result::handled;
	}
	if (command == "terrainselect")
	{
		int cx{0};
		int cz{0};
		std::string add;
		words >> cx >> cz >> add;
		if (add != "add")
			m_terrain_selection.clear();
		if (m_streamer.exists({cx, cz}))
			m_terrain_selection.insert({cx, cz});
		WriteLog(format("SELFTEST terrainselect: %zu selected", m_terrain_selection.size()));
		return selftest_result::handled;
	}
	if (command == "terrainresolution")
	{
		float spacing{1.f};
		int paint{1};
		words >> spacing >> paint;
		m_terrain_convert_spacing = heightmap::valid_spacing(spacing);
		m_terrain_convert_paint = paint != 0;
		convert_selected_chunks();
		WriteLog("SELFTEST terrainresolution: " + m_terrain_status);
		return selftest_result::handled;
	}
	if (command == "terrainwater" || command == "terrainwaterdefault")
	{
		if (command == "terrainwaterdefault")
		{
			std::string material;
			words >> material;
			m_streamer.water_material(material);
		}
		else
		{
			std::size_t index{0};
			std::string material;
			float size{0.f};
			words >> index >> material >> size;
			auto water{m_streamer.water()};
			if (index < water.size())
			{
				water[index].material = material == "-" ? std::string{} : material;
				water[index].size = size;
				m_streamer.water(std::move(water));
			}
		}
		std::string list;
		for (auto const &body : m_streamer.water())
			list += format(" [%s: %s, %.1f m]", body.name.c_str(), body.material.empty() ? "default" : body.material.c_str(), body.size);
		WriteLog("SELFTEST " + command + ": default " + m_streamer.manifest().water_material + list);
		return selftest_result::handled;
	}
	if (command == "terrainadd")
	{
		int count{1};
		float spacing{m_terrain_spacing};
		words >> count >> spacing;
		m_terrain_spacing = heightmap::valid_spacing(spacing);
		add_chunks_around(std::clamp(count, 1, 8));
		WriteLog("SELFTEST terrainadd: " + m_terrain_status);
		return selftest_result::handled;
	}
	if (command == "terrainprobe")
	{
		double x{0.0};
		double z{0.0};
		double height{0.0};
		words >> x >> z;
		auto const *chunk{m_streamer.terrain_at(x, z)};
		std::string materials;
		if (chunk != nullptr)
			for (std::size_t slot = 0; slot < std::max<std::size_t>(1, chunk->data().layer_count()); ++slot)
			{
				auto const layer{chunk->data().layers[slot]};
				auto const &palette{m_streamer.manifest().layers};
				materials += (layer < palette.size() ? palette[layer].material : std::string{"?"}) + " ";
			}
		auto const found{m_streamer.height_at(x, z, height)};
		std::string grid;
		if (chunk != nullptr)
		{
			auto const &data{chunk->data()};
			double base{0.0};
			double adjust{0.0};
			if (false == data.base.empty())
				base = data.height_in(data.base, x, z);
			if (false == data.adjust.empty())
				adjust = data.height_in(data.adjust, x, z);
			grid = format(", spacing %.1f, paint %d, base %s %.3f, touch-up %.3f", data.spacing(), data.paint, data.base.empty() ? "none" : "", base, adjust);
			if (data.paint > 0)
			{
				// weights at the paint sample nearest the point
				auto const corner{data.corner()};
				auto const ix{static_cast<int>(std::lround((x - corner.x) / heightmap::chunk_size * data.paint))};
				auto const iz{static_cast<int>(std::lround((z - corner.y) / heightmap::chunk_size * data.paint))};
				grid += ", weights";
				for (std::size_t slot = 0; slot < data.layer_count(); ++slot)
					grid += format(" %d", data.weight(slot, std::clamp(ix, 0, data.paint), std::clamp(iz, 0, data.paint)));
			}
		}
		WriteLog(format("SELFTEST terrainprobe %.1f %.1f: %s %.3f, %zu chunks, materials %s%s", x, z, found ? "height" : "no terrain", height, m_streamer.resident(), materials.c_str(), grid.c_str()));
		return selftest_result::handled;
	}
	if (command == "terrainconvert")
	{
		load_ground_files();
		for (auto const &source : m_ground_conversion.files)
		{
			glm::dvec3 low{1e30};
			glm::dvec3 high{-1e30};
			for (auto index = source.first; index < source.first + source.count; ++index)
				for (auto const &vertex : m_ground_conversion.triangles[index])
				{
					low = glm::min(low, vertex);
					high = glm::max(high, vertex);
				}
			WriteLog("SELFTEST terrain file " + source.file + (source.text ? " (text): " : " (binary): ") + source.message +
			         format(", from %.1f %.1f %.1f to %.1f %.1f %.1f", low.x, low.y, low.z, high.x, high.y, high.z));
		}
		m_ground_conversion.status = convert_ground();
		WriteLog("SELFTEST terrainconvert: " + m_ground_conversion.status);
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

editor_mode::selftest_result editor_mode::selftest_rails(std::string const &command, std::istringstream &words)
{
	if (command == "blade")
	{
		std::string name;
		words >> name;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->SwitchExtension == nullptr)
				continue;
			if (false == name.empty() && track->name() != name)
				continue;
			auto const vertices = [&]() {
				return std::make_pair(track->Geometry1.empty() ? std::size_t{0} : GfxRenderer->Vertices(track->Geometry1[0]).size(), track->Geometry2.empty() ? std::size_t{0} : GfxRenderer->Vertices(track->Geometry2[0]).size());
			};
			auto const set = [&](int const Path) {
				track->Switch(Path);
				track->SwitchExtension->fOffset = track->SwitchExtension->fDesiredOffset;
				track->RaAnimate();
			};
			set(0);
			auto const straight{vertices()};
			set(1);
			auto const diverging{vertices()};
			set(0);
			auto const frog{track->frog()};
			auto const radius{std::abs(track->m_paths.size() > 1 ? track->m_paths[1].radius : 0.f)};
			for (int path = 0; path < 2; ++path)
			{
				auto const &shape{track->blade(path)};
				auto const &segment{*track->SwitchExtension->Segments[path]};
				auto const old{segment.GetLength() * track_rails::blade_steps(*track, path) / segment.RaSegCount()};
				auto const throat{frog.valid ? frog.at[path] - track_rails::flangeway / std::tan(frog.angle) : 0.0};
				auto const at = [&](double const Width) {
					for (std::size_t i = 1; i < shape.gauge.size(); ++i)
						if (shape.gauge[i].y >= Width)
							return glm::mix(shape.gauge[i - 1].x, shape.gauge[i].x, (Width - shape.gauge[i - 1].y) / std::max(shape.gauge[i].y - shape.gauge[i - 1].y, 1e-9));
					return -1.0;
				};
				auto const gauge = [&](double const Distance) {
					for (std::size_t i = 1; i < shape.gauge.size(); ++i)
						if (shape.gauge[i].x >= Distance)
							return glm::mix(shape.gauge[i - 1].y, shape.gauge[i].y, (Distance - shape.gauge[i - 1].x) / std::max(shape.gauge[i].x - shape.gauge[i - 1].x, 1e-9));
					return shape.gauge.empty() ? 0.0 : shape.gauge.back().y;
				};
				std::string samples;
				auto worst{0.0};
				for (auto const distance : {1.0, 2.0, 4.0, 6.0, 8.0, 10.0})
				{
					auto const analytic{radius > 0.f ? distance * distance / (2.0 * radius) : 0.0};
					samples += format(" %.0f:%.4f/%.4f", distance, gauge(distance), analytic);
					if (shape.valid && distance <= shape.heel)
						worst = std::max(worst, std::abs(gauge(distance) - analytic));
				}
				std::string faults;
				if (track_rails::cut_rails(*track) && false == shape.valid)
					faults += " FAULT blade not cut from the rails";
				if (shape.valid && shape.opening < track_rails::blade_clearance - 1e-4)
					faults += " FAULT flangeway";
				if (shape.valid && frog.valid && shape.heel >= throat)
					faults += " FAULT heel past the throat";
				if (straight != diverging)
					faults += " FAULT vertex count changes";
				WriteLog(format("SELFTEST blade %s path %d: %s, R %.0f, tip %.3f, 50mm %.3f, full head %.3f, heel %.3f (old %.3f), drop %.4f, opening %.4f at %.3f, throat %.3f, d(x)/x^2/2R%s, worst %.4f, vertices %zu/%zu %zu/%zu%s", track->name().c_str(), path, shape.valid ? (shape.rigid ? "geometric rigid" : "geometric") : "old", radius, shape.tip, at(track_rails::blade_lowered + track_rails::blade_gap), shape.full, shape.heel, old, shape.drop, shape.opening, shape.openingat,
				                throat, samples.c_str(), worst, straight.first, diverging.first, straight.second, diverging.second, faults.c_str()));
			}
		}
		return selftest_result::handled;
	}
	if (command == "frog")
	{
		std::string name;
		words >> name;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->SwitchExtension == nullptr)
				continue;
			if (false == name.empty() && track->name() != name)
				continue;
			auto const frog{track->frog()};
			if (false == frog.valid)
			{
				WriteLog(format("SELFTEST frog %s: no frog, paths %.3f and %.3f m, blades %.3f m", track->name().c_str(), track->SwitchExtension->Segments[0]->GetLength(), track->SwitchExtension->Segments[1]->GetLength(), frog.blades[0]));
				continue;
			}
			auto const n{1.0 / std::tan(frog.angle)};
			glm::dvec3 point, across;
			track->SwitchExtension->Segments[0]->plan_frame(frog.at[0], point, across);
			auto const gauge{point + across * (frog.side[0] * 0.5 * std::abs(track->fTrackWidth))};
			auto const slope{std::tan(frog.angle)};
			auto const check{0.5 * track_rails::check_length(slope)};
			WriteLog(format("SELFTEST frog %s: crossing at %.3f / %.3f m (%.3f %.3f %.3f), 1:%.2f, side %d, blades %.3f / %.3f m, throat %.3f, nose %.3f, wing end %.3f, check rails %.3f-%.3f / %.3f-%.3f m", track->name().c_str(),
			    frog.at[0], frog.at[1], gauge.x, gauge.y, gauge.z, n, frog.side[0], frog.blades[0], frog.blades[1], frog.at[0] - track_rails::flangeway / slope, frog.at[0] + track_rails::nose_tip / slope, frog.at[0] + track_rails::wing_reach(slope),
			    frog.at[0] - 0.25 - check, frog.at[0] - 0.25 + check, frog.at[1] - 0.25 - check, frog.at[1] - 0.25 + check));
		}
		return selftest_result::handled;
	}
	if (command == "geomhash")
	{
		std::string name;
		words >> name;
		auto const mix = [](std::uint64_t &Hash, float const Value) {
			Hash = (Hash ^ static_cast<std::uint64_t>(static_cast<std::int64_t>(std::llround(Value * 1e4)))) * 1099511628211ull;
		};
		std::uint64_t total{1469598103934665603ull};
		std::size_t vertices{0};
		int tracks{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1)
				continue;
			if (false == name.empty() && track->name() != name)
				continue;
			std::uint64_t hash{1469598103934665603ull};
			std::size_t count{0};
			int chunks{0};
			std::vector<TTrack::geometryhandle_sequence const *> sequences{&track->Geometry1, &track->Geometry2, &track->GeometryCut1, &track->GeometryCut2};
			for (auto const &tread : track->GeometryTreads)
			{
				sequences.push_back(&tread.geometry);
				sequences.push_back(&tread.cuts);
			}
			for (auto const *sequence : sequences)
			{
				for (auto const &geometry : *sequence)
				{
					++chunks;
					for (auto const &vertex : GfxRenderer->Vertices(geometry))
					{
						for (int i = 0; i < 3; ++i)
						{
							mix(hash, vertex.position[i]);
							mix(hash, vertex.normal[i]);
						}
						mix(hash, vertex.texture.x);
						mix(hash, vertex.texture.y);
						++count;
					}
				}
				mix(hash, -1.f);
			}
			WriteLog(format("SELFTEST geom %s: chunks %d, vertices %zu, hash %016llx", track->name().c_str(), chunks, count, static_cast<unsigned long long>(hash)));
			total = (total ^ hash) * 1099511628211ull;
			vertices += count;
			++tracks;
		}
		WriteLog(format("SELFTEST geom total: tracks %d, vertices %zu, hash %016llx", tracks, vertices, static_cast<unsigned long long>(total)));
		return selftest_result::handled;
	}
	if (command == "frogkey")
	{
		std::string name, value;
		words >> name >> value;
		auto *track{simulation::Paths.find(name)};
		if (track == nullptr || track->eType != tt_Switch || track->SwitchExtension == nullptr)
			WriteLog(format("SELFTEST frogkey %s: no such switch", name.c_str()));
		else
		{
		if (value == "on" || value == "off")
		{
			auto const before{editor_track::capture(*track)};
			track->SwitchExtension->m_frogs = (value == "on");
			push_track_snapshot({{track, before}});
			editor_track::commit_parameters(*track);
		}
		std::ostringstream text;
		track->export_as_text(text);
		WriteLog(format("SELFTEST frogkey %s: key %d, written %d, frog %d, crossings %zu, profile '%s'", name.c_str(), track->SwitchExtension->m_frogs ? 1 : 0, text.str().find(" frogs ") != std::string::npos ? 1 : 0,
		    track->frog().valid ? 1 : 0, track->rail_crossings().size(), track->m_profile1.first.c_str()));
		}
		return selftest_result::handled;
	}
	if (command == "guards")
	{
		std::string name;
		words >> name;
		int count{0};
		int faults{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1)
				continue;
			if (false == name.empty() && track->name() != name)
				continue;
			auto const layout{track->plan_rails(true)};
			auto const size{track->rail_dimensions()};
			auto const guardoffset{-(track_rails::flangeway + size.head)};
			double const laterals[2]{size.low + guardoffset, track_rails::flangeway + size.head + size.low - 0.002 + guardoffset};
			for (int path = 0; path < 2; ++path)
			{
				for (int right = 0; right < 2; ++right)
				{
					auto const side{right ? 1 : -1};
					auto const length{track->eType == tt_Switch ? track->SwitchExtension->Segments[path]->GetLength() : track->Segment->GetLength()};
					auto const drawn = [&](glm::dvec2 const &Miter, double const Square) {
						auto const at{Miter.y == 0.0 ? Square : Miter.x + Miter.y * 0.5 * (laterals[0] + laterals[1])};
						return at > 0.0 && at < length;
					};
					for (auto const &guard : layout.guards[path][right])
					{
						if (false == drawn(guard.frommiter, guard.from) && false == drawn(guard.tomiter, guard.to) && (guard.to <= 0.0 || guard.from >= length))
							continue;
						++count;
						std::string text;
						auto const end = [&](glm::dvec2 const &Miter, bool const Flared, bool const Glued) {
							if (Miter.y == 0.0)
								return std::string{Flared ? "flared" : "square"};
							if (false == drawn(Miter, 0.0))
								return std::string{"past the end of the track"};
							glm::dvec3 points[2];
							for (int i = 0; i < 2; ++i)
								points[i] = track_rails::rail_point(*track, path, side, Miter.x + Miter.y * laterals[i], laterals[i]);
							auto best{1e9};
							double lateral{0.0}, along{0.0};
							TTrack *against{nullptr};
							int againstpath{0}, againstside{0};
							for (auto *other : layout.crossed)
								for (int otherpath = 0; otherpath < (other->eType == tt_Switch ? 2 : 1); ++otherpath)
									for (auto const otherside : {-1, 1})
									{
										double d[2], l[2];
										for (int i = 0; i < 2; ++i)
											track_rails::rail_offset(*other, otherpath, otherside, points[i], d[i], l[i]);
										if (std::abs(l[0]) > 0.5 || std::abs(l[1]) > 0.5 || std::abs(l[0] - l[1]) >= best)
											continue;
										best = std::abs(l[0] - l[1]);
										lateral = 0.5 * (l[0] + l[1]);
										along = 0.5 * (d[0] + d[1]);
										against = other;
										againstpath = otherpath;
										againstside = otherside;
									}
							if (against == nullptr)
							{
								++faults;
								return std::string{"cut along nothing FAULT"};
							}
							auto result{format("along %s/%d%+d at %.3f m, %.1f mm off its gauge line, skew %.1f mm", against->name().c_str(), againstpath, againstside, along, lateral * 1000.0, best * 1000.0)};
							auto fault{best > 0.002};
							if (Glued)
							{
								auto const theirsize{against->rail_dimensions()};
								auto const theiroffset{-(track_rails::flangeway + theirsize.head)};
								double const theirlaterals[2]{theirsize.low + theiroffset, track_rails::flangeway + theirsize.head + theirsize.low - 0.002 + theiroffset};
								auto const reach = [&](glm::dvec2 const &Miter, double const Square, bool const Latest) {
									if (Miter.y == 0.0)
										return Square;
									auto const a{Miter.x + Miter.y * theirlaterals[0]};
									auto const b{Miter.x + Miter.y * theirlaterals[1]};
									return Latest ? std::max(a, b) : std::min(a, b);
								};
								auto covered{false};
								for (auto const &theirs : against->plan_rails(true).guards[againstpath][againstside > 0])
									covered |= (along > reach(theirs.frommiter, theirs.from, false) - 0.002 && along < reach(theirs.tomiter, theirs.to, true) + 0.002);
								result += covered ? ", on its guard rail" : ", no guard rail there";
								fault |= (false == covered || std::abs(lateral + track_rails::flangeway) > 0.002);
							}
							if (fault)
							{
								++faults;
								result += " FAULT";
							}
							return result;
						};
						text += "from " + end(guard.frommiter, guard.flarefrom, false == guard.flarefrom) + "; to " + end(guard.tomiter, guard.flareto, false == guard.flareto);
						for (auto const &cut : guard.plan.cuts)
							if (cut.end > 0.0 && cut.start < length)
								text += format("; break %.3f-%.3f: ", cut.end, cut.start) + end(cut.endmiter, false, false) + " / " + end(cut.startmiter, false, false);
						WriteLog(format("SELFTEST guard %s path %d side %+d %.3f-%.3f m: %s", track->name().c_str(), path, side, guard.from, guard.to, text.c_str()));
					}
				}
			}
		}
		WriteLog(format("SELFTEST guards: %d guard rails, %d faults", count, faults));
		return selftest_result::handled;
	}
	if (command == "throw")
	{
		int path{0};
		words >> path;
		int count{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->SwitchExtension == nullptr)
				continue;
			track->Switch(path);
			track->SwitchExtension->fOffset = track->SwitchExtension->fDesiredOffset;
			track->RaAnimate();
			++count;
		}
		WriteLog(format("SELFTEST switches: %d switches set to path %d", count, path));
		return selftest_result::handled;
	}
	if (command == "railtime")
	{
		track_rails::forget();
		auto const started{std::chrono::steady_clock::now()};
		int tracks{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1)
				continue;
			track->rail_crossings(false);
			++tracks;
		}
		auto const crossed{std::chrono::steady_clock::now()};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1)
				continue;
			track->plan_rails(true);
		}
		auto const finished{std::chrono::steady_clock::now()};
		auto const crossings{std::chrono::duration<double, std::milli>(crossed - started).count()};
		auto const elapsed{std::chrono::duration<double, std::milli>(finished - started).count()};
		WriteLog(format("SELFTEST railtime: the rails of %d tracks laid out in %.1f ms, of which their crossings %.1f ms", tracks, elapsed, crossings));
		return selftest_result::handled;
	}
	if (command == "crossings")
	{
		std::string name;
		words >> name;
		auto const started{std::chrono::steady_clock::now()};
		int tracks{0};
		int found{0};
		int acutes{0};
		int middles{0};
		int mismatched{0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 15) != 1)
				continue;
			if (false == name.empty() && track->name() != name)
				continue;
			++tracks;
			for (auto const &crossing : track->rail_crossings())
			{
				++found;
				auto const middle{crossing.angle > track_rails::middle_angle && crossing.angle <= track_rails::wide_angle};
				middles += middle ? 1 : 0;
				auto const kind{crossing.angle > track_rails::wide_angle ? "wide" : crossing.facing < 0.0 ? (middle ? "middle acute" : "acute") : (middle ? "middle obtuse" : "obtuse")};
				WriteLog(format("SELFTEST crossing %s path %d side %d at %.3f m: %s with %s, 1:%.2f, into %.3f, facing %.3f, nose %d", track->name().c_str(), crossing.path, crossing.side, crossing.at, kind,
				    crossing.track->name().c_str(), 1.0 / std::tan(crossing.angle), crossing.into, crossing.facing, crossing.nose ? 1 : 0));
			}
			auto const gauge = [](TTrack *Track, TTrack::rail_crossing const &Crossing) {
				auto const &segment{Track->eType == tt_Switch ? *Track->SwitchExtension->Segments[Crossing.path] : *Track->Segment};
				glm::dvec3 point, across;
				segment.plan_frame(Crossing.at, point, across);
				return point + across * (Crossing.side * 0.5 * std::abs(Track->fTrackWidth));
			};
			for (auto const &crossing : track->rail_crossings(false))
			{
				if (crossing.angle > track_rails::wide_angle || crossing.facing >= 0.0)
					continue;
				++acutes;
				auto const here{gauge(track, crossing)};
				auto partners{0};
				auto noses{crossing.nose ? 1 : 0};
				for (auto const &other : crossing.track->rail_crossings(false))
				{
					if (other.track != track || other.angle > track_rails::wide_angle || other.facing >= 0.0)
						continue;
					auto const there{gauge(crossing.track, other)};
					if (glm::length(glm::dvec2{there.x - here.x, there.z - here.z}) > 0.05)
						continue;
					++partners;
					noses += other.nose ? 1 : 0;
				}
				if (partners != 1 || noses != 1)
					++mismatched;
				WriteLog(format("SELFTEST acute %s %s path %d side %d at %.3f m with %s (%.2f %.2f %.2f): partners %d, noses %d", partners != 1 || noses != 1 ? "mismatch" : "ok", track->name().c_str(), crossing.path, crossing.side, crossing.at, crossing.track->name().c_str(), here.x, here.y, here.z, partners, noses));
			}
		}
		auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count()};
		WriteLog(format("SELFTEST crossings: %d rail crossings on %d tracks, %d of them middle, %d acute, %d mismatched, %.1f ms", found, tracks, middles, acutes, mismatched, elapsed));
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}

// the script is a text file, one action a line:
//   wait <frames>
//   camera <x> <y> <z> <x> <y> <z>        from the first point, looking at the second
//   select <track name>
//   blade [name]                         logs the blades of each switch cut from the rails: tip, 50 mm head, full head, heel, drop of the head, open flangeway, the gauge lines apart against x^2/2R, vertices in both positions
//   frog [name]                          logs where the inner rails of each switch cross: distance on both paths, 1:n, throat, nose, check rails
//   guards [name]                        logs each guard rail and what each of its ends and breaks is cut along; a fault when a cut is not along a rail or a glued end not on the guard rail of the other track
//   railtime                             lays out the rails of every track, as when its geometry is made, and logs how long it took
//   throw <0|1>                          sets every switch to the path and moves its blades there at once
//   crossings [name]                     logs where rails of tracks cross rails of other tracks and of what kind (acute, obtuse, middle acute or obtuse, wide), checks that each acute crossing has one partner and one nose
//   frogkey name [on|off]                sets or clears the frogs key of a switch (one undo step); logs the key, whether it is written, the frog and the crossings
//   geomhash [name]                      logs the number of vertices and a hash of the drawn geometry of each rail track (rails, trackbed, cut faces), to compare builds
//   sweep <key>=<value>...               model, params (separated by commas), lateral, height, side (origin, near, far,
//                                        centre), top (origin, bottom, top, edge), mode (bend, repeat), step,
//                                        length (original, all, or metres), from, to, widen (0, 1), face (0, 1), tilt (0, 1)
//   selectmodel <x> <z>                  the model instance nearest the point
//   bend <key>=<value>...                the selected model bent along a track: track (index on the list), axis (auto, z, x),
//                                        turned (0, 1), along (start, origin, middle, end, point), side (origin, near, far,
//                                        middle, point), height (origin, bottom, top, edge, point), point=<x>,<y>,<z>,
//                                        pick=<x>,<y>,<z> (a point of the scene picked as the reference point), go (0, 1)
//   selectinclude <x> <z>                the include placed nearest the point
//   addmodel <x> <z>                     the model instance nearest the point added to the selection, or taken out (Shift+LMB)
//   selection                            logs the selected models, with the models of their groups
//   moveselection <x> <y> <z>            the selection moved by the offset, as the gizmo moves it
//   turnselection <degrees>              the selection turned around the model with the gizmo
//   copy, paste <x> <y> <z>, group, ungroup, drop, delete   as the keys of the selection do them
//   bendset <key>=<value>...             the bent model being edited: length, lateral, height, along, side, height
//   bendprobe                            logs the bent model being edited
//   undo, redo
//   key <S|W|CTRL|RMB> <press|release> [ctrl]  a key or right button event, as the window gives it
//   lineside <vehicle|hekto|fouling|parallel>  the tab of the objects along the track in the track window
//   event <name>                         queues the event
//   memcell <name>                       logs the memory cell
//   bendpanel                            unfolds the bend section of the toolset
//   openscenerypopup                     the window of File / Open scenery, its list searched for zz_
//   openscenery <scenery file>           the editor starts again with the scenery
//   checkspans                           logs whether the text the scenery files hold at each node's place starts and ends it
//   spread <track> <stop 0..2> <reach>   logs the paths the spread from the track takes
//   newscenery                           the editor starts again with the wizard of a new scenery
//   mapstate                             logs the orthophoto, the top view and the //$o line of the scenery
//   trainset <track> <vehicle path> <skin> <facing> [km/h]  the trainset tool puts the vehicle in the middle of the track
//   vehicles                             logs each vehicle: its head, facing, and what its couplers hold
//   terrainmode                          the terrain field of work, with its texture browser
//   terrainsculpt <x> <z> <radius> <metres>, terrainpaint <x> <z> <radius> <palette entry>,
//   terrainlevel <x> <z> <radius> <target height> [mode]   as a stroke of the brush
//   terrainadd <count> <spacing>          chunks made around the camera, count x count, as the button of the chunks panel does it
//   terrainprobe <x> <z>                 logs the height of the heightmap terrain and the materials of the chunk
//   terrainconvert                       converts every material of the terrain files of the scenery
//   terrainmodifier <name> <x0> <z0> <x1> <z1> <formation> <half width>   a straight modifier, as the vertical profile makes them
//   terraintouchup <0|1>                 the brushes touch up over the modifiers, or shape the ground under them
//   terrainrestore <x> <z> <radius>      a stroke of the restore brush
//   terrainbake <name>                   the modifier baked into the ground
//   terrainselect <cx> <cz> [add]        the chunk selected with the spacing tool (add: Ctrl)
//   terrainresolution <spacing> [paint 0|1]   the selected chunks converted, as the button of the resolution panel does it
//   terrainfill <palette entry> <blur> <rotation> <x> <z> <x> <z> <x> <z>...   the area outlined filled, as the fill tool does it
//   terrainturn <palette entry> <degrees>  the textures of the palette entry turned
//   terrainwater <index> <material> [size]    material and repeat of a body of water; terrainwaterdefault <material>
//   orthophoto <north> <east> <radius> <fit 0|1> [in scene 0|1]   the imagery layer on, placed and fitted as given
//   join <x> <z> <x> <z> [radius] [variant] [transitions 0|1]   the free ends at the points joined, as a drag from one to the other
//   layto <x> <z> <x> <z> <layout 0..3> <radius>   from the free end at the first point towards the second, as a drag in the layout
//   layclicks <layout 0..3> <radius> <x> <z>...   clicks of the lay tool at the points, a free end within 3 m joined
//   joints <x> <z>...                    the joints check over the scenery, the issues around the points logged
//   route <x> <z> <single 0|1>          the route design of the path nearest the point, a curve or the whole line
//   routevertex <i> <R> <L in> <L out> <cant>, routearc <i> <R> <L> <share>, routeshare <i> <share>, routekm <km>
//   routeapply, routetake                as the buttons of the route design do it
//   alignments                           logs the alignments kept in the scenery and the element under each path
//   digitizeall                          every line not digitized yet kept in the scenery, as the button of the route design does it
//   splitkind <kind>, nudgekind <kind> <dx> <dz>   the first path over an element of the kind split in two, or moved
//   switchplace <template> <x> <z> <direction> <side>   the catalogue template whose label starts so put into the straight
//                                        nearest the point, or into the curve (switchcurve); slipplace <template> <x> <z>
//   switchpieces <R> <n> [name]          logs the diverging path made of pieces of each switch: length, the arc R of 1:n against the axis
//   switchbreak [name]                   the first piece of the diverging path moved off its neighbour, the switch committed
//   switches                             logs the switches kept in the scenery and the speed of each diverging track
//   deletetrack <name>, deletekind <kind>   the path deleted as Del does it, or the first path over an element of the kind
//   btfcheck <terrain file>              the .txtf converted to .btf, the editor lines the binary file keeps logged
//   screenshot
//   save
//   log <text>
//   quit
void editor_mode::selftest_step()
{
	auto &test{m_selftest};
	if (false == test.loaded)
	{
		test.loaded = true;
		auto const *file{std::getenv("EU07_EDITOR_SELFTEST")};
		if (file == nullptr)
			return;
		std::ifstream stream(file);
		std::string line;
		while (std::getline(stream, line))
			if (false == line.empty() && line.front() != '#')
				test.lines.push_back(line);
		WriteLog("SELFTEST: " + std::to_string(test.lines.size()) + " steps from " + file);
	}
	if (test.wait > 0)
	{
		--test.wait;
		return;
	}
	while (test.next < test.lines.size())
	{
		std::istringstream words(test.lines[test.next++]);
		std::string command;
		words >> command;
		if (command == "wait")
		{
			words >> test.wait;
			return;
		}
		if (selftest_dispatch(command, words) == selftest_result::pause)
			return;
		if (command == "quit")
		{
			if (false == test.quitting)
			{
				WriteLog("SELFTEST done");
				test.quitting = true;
				test.wait = 30;
				--test.next;
				return;
			}
			Application.queue_quit(true);
			return;
		}
	}
}

/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/driveruilayer.h"

#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "application/application.h"
#include "utilities/translation.h"
#include "simulation/simulation.h"
#include "vehicle/Train.h"
#include "model/AnimModel.h"
#include "rendering/renderer.h"
#include "DevConsole/devconsole.h"
#include "entitysystem/ECScene.h"
#include "entitysystem/components/BasicComponents.h"
#include "entitysystem/components/RenderComponents.h"
#include "entitysystem/ECSPersistence.h"

driver_ui::driver_ui()
{

	clear_panels();
	// bind the panels with ui object. maybe not the best place for this but, eh
	add_external_panel(&m_aidpanel);
	add_external_panel(&m_scenariopanel);
	add_external_panel(&m_timetablepanel);
	add_external_panel(&m_debugpanel);
	if (Global.gui_showtranscripts)
		add_external_panel(&m_transcriptspanel);

	add_external_panel(&m_trainingcardpanel);
	add_external_panel(&m_vehiclelist);
	add_external_panel(&m_timepanel);
	add_external_panel(&m_mappanel);
	add_external_panel(&m_logpanel);
	add_external_panel(&m_perfgraphpanel);
	add_external_panel(&dev::Console);
	add_external_panel(&m_cameraviewpanel);
	m_logpanel.is_open = false;

	m_aidpanel.title = STR("Driving Aid");

	m_scenariopanel.title = STR("Scenario");
	m_scenariopanel.size_min = {435, 85};
	m_scenariopanel.size_max = {Global.fb_size.x * 0.95f, Global.fb_size.y * 0.95};

	m_timetablepanel.title = STR("%-*.*s    Time: %d:%02d:%02d");
	m_timetablepanel.size_min = {435, 70};
	m_timetablepanel.size_max = {435, Global.fb_size.y * 0.95};

	m_transcriptspanel.title = STR("Transcripts");
	m_transcriptspanel.size_min = {435, 85};
	m_transcriptspanel.size_max = {Global.fb_size.x * 0.95, Global.fb_size.y * 0.95};

	if (Global.gui_defaultwindows)
	{
		m_aidpanel.is_open = true;
		m_scenariopanel.is_open = true;
	}

	if (Global.gui_trainingdefault)
	{
		m_mappanel.is_open = true;
		m_trainingcardpanel.is_open = true;
		m_vehiclelist.is_open = true;
	}

	register_driver_commands();
}

void driver_ui::register_driver_commands()
{
	auto &con = dev::Console;

	// --- UI toggles ---
	con.register_command("debug", "Toggle debug panel",
		[this](const dev::args_t &) {
			m_debugpanel.is_open = !m_debugpanel.is_open;
			dev::Console.print_ok(m_debugpanel.is_open ? "Debug panel opened" : "Debug panel closed");
		});

	con.register_command("map", "Toggle map panel",
		[this](const dev::args_t &) {
			m_mappanel.is_open = !m_mappanel.is_open;
		});

	con.register_command("aid", "Toggle driving-aid panel",
		[this](const dev::args_t &) {
			m_aidpanel.is_open = !m_aidpanel.is_open;
		});

	con.register_command("timetable", "Toggle timetable panel",
		[this](const dev::args_t &) {
			m_timetablepanel.is_open = !m_timetablepanel.is_open;
		});

	// --- Simulation control ---
	con.register_command("pause", "Pause or unpause simulation",
		[](const dev::args_t &) {
			command_relay relay;
			relay.post(user_command::pausetoggle, 0.0, 0.0, GLFW_PRESS, 0);
		});

	con.register_command("debugmode", "Toggle debug mode (enable/disable)",
		[](const dev::args_t &args) {
			command_relay relay;
			relay.post(user_command::debugtoggle, 0.0, 0.0, GLFW_RELEASE, 0);
			dev::Console.print_ok(std::string("Debug mode: ") + (DebugModeFlag ? "on" : "off"));
		});

	// --- Weather ---
	con.register_command("overcast", "Set overcast level: overcast <0.0-1.0>",
		[](const dev::args_t &args) {
			if (args.size() < 2) {
				dev::Console.print_info("Current overcast: " + std::to_string(Global.Overcast));
				return;
			}
			try {
				float v = std::stof(args[1]);
				v = std::max(0.0f, std::min(1.0f, v));
				Global.Overcast = v;
				dev::Console.print_ok("Overcast set to " + std::to_string(v));
			} catch (...) {
				dev::Console.print_error("Expected a float 0.0-1.0");
			}
		});

	// --- Screenshot ---
	con.register_command("screenshot", "Take a screenshot",
		[](const dev::args_t &) {
			Application.queue_screenshot();
			dev::Console.print_ok("Screenshot queued");
		});

	// --- Log ---
	con.register_command("log", "Enable or disable file logging: log [on|off]",
		[](const dev::args_t &args) {
			if (args.size() < 2) {
				dev::Console.print_info(std::string("File logging: ") +
					((Global.iWriteLogEnabled & 1) ? "on" : "off"));
				return;
			}
			if (args[1] == "on") {
				Global.iWriteLogEnabled |= 1;
				dev::Console.print_ok("File logging enabled");
			} else if (args[1] == "off") {
				Global.iWriteLogEnabled &= ~1;
				dev::Console.print_ok("File logging disabled");
			} else {
				dev::Console.print_error("Usage: log [on|off]");
			}
		});

	// --- ECS ---
	con.register_command("ecs.list", "List all ECS entities (optional filter: ecs.list [name_fragment])",
		[](const dev::args_t &args) {
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			std::string filter = args.size() >= 2 ? args[1] : "";
			int count = 0;
			for (auto entity : world.GetEntities()) {
				std::string name = std::to_string(static_cast<unsigned>(entity));
				if (auto *id = world.GetComponent<ECSComponent::Identification>(entity))
					name = id->Name.ToString();
				if (!filter.empty() && name.find(filter) == std::string::npos)
					continue;
				dev::Console.print("  [" + std::to_string(static_cast<unsigned>(entity)) + "] " + name);
				++count;
			}
			dev::Console.print_info("Total: " + std::to_string(count) + " entities");
		});

	con.register_command("ecs.info", "Show components of entity by name: ecs.info <name>",
		[](const dev::args_t &args) {
			if (args.size() < 2) { dev::Console.print_error("Usage: ecs.info <name>"); return; }
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			entt::entity entity = world.FindEntityByName(args[1]);
			if (entity == entt::null) { dev::Console.print_error("Entity not found: " + args[1]); return; }

			dev::Console.print_ok("Entity: " + args[1]);
			if (auto *t = world.GetComponent<ECSComponent::Transform>(entity)) {
				dev::Console.print("  Transform: pos=("
					+ std::to_string(t->Position.x) + ", "
					+ std::to_string(t->Position.y) + ", "
					+ std::to_string(t->Position.z) + ")");
			}
			if (auto *v = world.GetComponent<ECSComponent::Velocity>(entity)) {
				dev::Console.print("  Velocity: ("
					+ std::to_string(v->Value.x) + ", "
					+ std::to_string(v->Value.y) + ", "
					+ std::to_string(v->Value.z) + ")");
			}
			if (world.HasComponent<ECSComponent::MeshRenderer>(entity))
				dev::Console.print("  MeshRenderer: yes");
			if (world.HasComponent<ECSComponent::ParticleEmitter>(entity))
				dev::Console.print("  ParticleEmitter: yes");
			if (auto *id = world.GetComponent<ECSComponent::Identification>(entity))
				dev::Console.print("  ID: " + id->Name.ToString() + " #" + std::to_string(id->Id));
		});

	con.register_command("ecs.setvel", "Set entity velocity: ecs.setvel <name> <x> <y> <z>",
		[](const dev::args_t &args) {
			if (args.size() < 5) { dev::Console.print_error("Usage: ecs.setvel <name> <x> <y> <z>"); return; }
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			entt::entity entity = world.FindEntityByName(args[1]);
			if (entity == entt::null) { dev::Console.print_error("Entity not found: " + args[1]); return; }

			try {
				glm::vec3 vel{ std::stof(args[2]), std::stof(args[3]), std::stof(args[4]) };
				if (auto *v = world.GetComponent<ECSComponent::Velocity>(entity)) {
					v->Value = vel;
					dev::Console.print_ok("Velocity set");
				} else {
					world.AddComponent<ECSComponent::Velocity>(entity).Value = vel;
					dev::Console.print_ok("Velocity component added and set");
				}
			} catch (...) { dev::Console.print_error("Invalid numbers"); }
		});

	con.register_command("ecs.kill", "Destroy entity by name: ecs.kill <name>",
		[](const dev::args_t &args) {
			if (args.size() < 2) { dev::Console.print_error("Usage: ecs.kill <name>"); return; }
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			entt::entity entity = world.FindEntityByName(args[1]);
			if (entity == entt::null) { dev::Console.print_error("Entity not found: " + args[1]); return; }
			world.DestroyEntity(entity);
			dev::Console.print_ok("Entity destroyed: " + args[1]);
		});

	con.register_command("ecs.count", "Show total entity count in current scene",
		[](const dev::args_t &) {
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			dev::Console.print_info("Entities: " + std::to_string(scene->World().GetEntityCount()));
		});

	con.register_command("ecs.setpos", "Teleport entity to position: ecs.setpos <name> <x> <y> <z>",
		[](const dev::args_t &args) {
			if (args.size() < 5) { dev::Console.print_error("Usage: ecs.setpos <name> <x> <y> <z>"); return; }
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			entt::entity entity = world.FindEntityByName(args[1]);
			if (entity == entt::null) { dev::Console.print_error("Entity not found: " + args[1]); return; }

			try {
				glm::dvec3 pos{ std::stod(args[2]), std::stod(args[3]), std::stod(args[4]) };
				if (auto *t = world.GetComponent<ECSComponent::Transform>(entity)) {
					t->Position = pos;
					dev::Console.print_ok("Position set: ("
						+ args[2] + ", " + args[3] + ", " + args[4] + ")");
				} else {
					dev::Console.print_error("Entity has no Transform component");
				}
			} catch (...) { dev::Console.print_error("Invalid numbers"); }
		});

	con.register_command("ecs.spawn", "Spawn a named entity with Transform at position: ecs.spawn <name> <x> <y> <z>",
		[](const dev::args_t &args) {
			if (args.size() < 5) { dev::Console.print_error("Usage: ecs.spawn <name> <x> <y> <z>"); return; }
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			try {
				glm::dvec3 pos{ std::stod(args[2]), std::stod(args[3]), std::stod(args[4]) };
				auto entity = world.CreateEntity();
				auto &id = world.AddComponent<ECSComponent::Identification>(entity);
				id.Name = args[1];
				auto &t = world.AddComponent<ECSComponent::Transform>(entity);
				t.Position = pos;
				dev::Console.print_ok("Spawned entity '" + args[1] + "' at ("
					+ args[2] + ", " + args[3] + ", " + args[4] + ")");
			} catch (...) { dev::Console.print_error("Invalid position arguments"); }
		});

	con.register_command("ecs.find", "Find entities whose name contains a substring: ecs.find <fragment>",
		[](const dev::args_t &args) {
			if (args.size() < 2) { dev::Console.print_error("Usage: ecs.find <fragment>"); return; }
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			const std::string &fragment = args[1];
			int found = 0;
			for (auto entity : world.GetEntities()) {
				auto *id = world.GetComponent<ECSComponent::Identification>(entity);
				if (!id) continue;
				std::string name = id->Name.ToString();
				if (name.find(fragment) != std::string::npos) {
					dev::Console.print("  [" + std::to_string(static_cast<unsigned>(entity)) + "] " + name);
					++found;
				}
			}
			if (found == 0)
				dev::Console.print_warn("No entities found matching: " + fragment);
			else
				dev::Console.print_info(std::to_string(found) + " match(es)");
		});

	con.register_command("ecs.disable", "Disable entity (systems will skip it): ecs.disable <name>",
		[](const dev::args_t &args) {
			if (args.size() < 2) { dev::Console.print_error("Usage: ecs.disable <name>"); return; }
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			entt::entity entity = world.FindEntityByName(args[1]);
			if (entity == entt::null) { dev::Console.print_error("Entity not found: " + args[1]); return; }
			if (!world.HasComponent<ECSComponent::Disabled>(entity))
				world.AddComponent<ECSComponent::Disabled>(entity);
			dev::Console.print_ok("Disabled: " + args[1]);
		});

	con.register_command("ecs.enable", "Re-enable a disabled entity: ecs.enable <name>",
		[](const dev::args_t &args) {
			if (args.size() < 2) { dev::Console.print_error("Usage: ecs.enable <name>"); return; }
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			entt::entity entity = world.FindEntityByName(args[1]);
			if (entity == entt::null) { dev::Console.print_error("Entity not found: " + args[1]); return; }
			world.RemoveComponent<ECSComponent::Disabled>(entity);
			dev::Console.print_ok("Enabled: " + args[1]);
		});

	con.register_command("ecs.addparticles", "Add a particle emitter to an entity: ecs.addparticles <name>",
		[](const dev::args_t &args) {
			if (args.size() < 2) { dev::Console.print_error("Usage: ecs.addparticles <name>"); return; }
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			entt::entity entity = world.FindEntityByName(args[1]);
			if (entity == entt::null) { dev::Console.print_error("Entity not found: " + args[1]); return; }

			auto& emitter = world.AddComponent<ECSComponent::ParticleEmitter>(entity);
			emitter.isActive         = true;
			emitter.spawnRate        = 20.0f;
			emitter.particleLifetime = 2.5f;
			emitter.gravity          = glm::vec3(0.0f, 0.4f, 0.0f);
			emitter.airResistance    = 0.15f;
			emitter.colorFade        = glm::vec4(0.0f, 0.0f, 0.0f, -0.5f);
			if (auto *t = world.GetComponent<ECSComponent::Transform>(entity))
				emitter.emitterLocation = glm::vec3(t->Position);
			dev::Console.print_ok("ParticleEmitter added to: " + args[1]);
		});

	con.register_command("ecs.addspotlight",
		"Spawn SpotLight at position: ecs.addspotlight <name> <x> <y> <z> [r g b] [intensity] [range]",
		[](const dev::args_t &args) {
			if (args.size() < 5) { dev::Console.print_error("Usage: ecs.addspotlight <name> <x> <y> <z> [r g b] [intensity] [range]"); return; }
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			auto entity = world.CreateEntity();

			auto& id = world.AddComponent<ECSComponent::Identification>(entity);
			id.Name = args[1];

			auto& t = world.AddComponent<ECSComponent::Transform>(entity);
			t.Position = glm::dvec3(std::stod(args[2]), std::stod(args[3]), std::stod(args[4]));

			auto& spot = world.AddComponent<ECSComponent::SpotLight>(entity);
			if (args.size() > 7) {
				spot.Color = glm::vec3(std::stof(args[5]), std::stof(args[6]), std::stof(args[7]));
			}
			if (args.size() > 8) spot.Intensity = std::stof(args[8]);
			if (args.size() > 9) spot.Range      = std::stof(args[9]);
			spot.Enabled = true;

			dev::Console.print_ok("SpotLight '" + args[1] + "' spawned at ("
				+ args[2] + ", " + args[3] + ", " + args[4] + ")");
		});

	con.register_command("ecs.clone",
		"Clone entity with optional offset: ecs.clone <src> <newname> [dx dy dz]",
		[](const dev::args_t &args) {
			if (args.size() < 3) { dev::Console.print_error("Usage: ecs.clone <src> <newname> [dx dy dz]"); return; }
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			ECWorld &world = scene->World();

			entt::entity src = world.FindEntityByName(args[1]);
			if (src == entt::null) { dev::Console.print_error("Entity not found: " + args[1]); return; }
			if (world.FindEntityByName(args[2]) != entt::null) { dev::Console.print_error("Name already exists: " + args[2]); return; }

			glm::dvec3 offset{0.0};
			if (args.size() >= 6) {
				offset = glm::dvec3(std::stod(args[3]), std::stod(args[4]), std::stod(args[5]));
			}

			auto dst = world.CreateEntity();
			auto &id = world.AddComponent<ECSComponent::Identification>(dst);
			id.Name = args[2];

			if (auto *t = world.GetComponent<ECSComponent::Transform>(src)) {
				auto &nt = world.AddComponent<ECSComponent::Transform>(dst);
				nt = *t;
				nt.Position += offset;
			}
			if (auto *s = world.GetComponent<ECSComponent::SpotLight>(src))
				world.AddComponent<ECSComponent::SpotLight>(dst) = *s;
			if (auto *p = world.GetComponent<ECSComponent::ParticleEmitter>(src))
				world.AddComponent<ECSComponent::ParticleEmitter>(dst) = *p;
			if (auto *b = world.GetComponent<ECSComponent::Billboard>(src))
				world.AddComponent<ECSComponent::Billboard>(dst) = *b;
			if (auto *l = world.GetComponent<ECSComponent::Line>(src))
				world.AddComponent<ECSComponent::Line>(dst) = *l;
			if (auto *lod = world.GetComponent<ECSComponent::LODController>(src))
				world.AddComponent<ECSComponent::LODController>(dst) = *lod;

			dev::Console.print_ok("Cloned '" + args[1] + "' → '" + args[2] + "'");
		});

	con.register_command("ecs.save",
		"Save ECS entities to JSON: ecs.save [filename=ecs_save.json]",
		[](const dev::args_t &args) {
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			std::string filename = (args.size() > 1) ? args[1] : "ecs_save.json";
			int n = ecs_persistence::save(scene->World(), filename);
			if (n >= 0)
				dev::Console.print_ok("Saved " + std::to_string(n) + " entities to " + filename);
			else
				dev::Console.print_error("Cannot write: " + filename);
		});

	con.register_command("ecs.load",
		"Load ECS entities from JSON: ecs.load [filename=ecs_save.json]",
		[](const dev::args_t &args) {
			ECScene *scene = Application.sceneManager.CurrentScene();
			if (!scene) { dev::Console.print_warn("No active scene"); return; }
			std::string filename = (args.size() > 1) ? args[1] : "ecs_save.json";
			int n = ecs_persistence::load(scene->World(), filename);
			if (n >= 0)
				dev::Console.print_ok("Loaded " + std::to_string(n) + " entities from " + filename);
			else
				dev::Console.print_error("Cannot read or parse: " + filename);
		});

	con.print_info("Driver console ready. Type 'help' for commands.");
}

void driver_ui::render_menu_contents()
{
	ui_layer::render_menu_contents();

	if (ImGui::BeginMenu(STR_C("Mode windows")))
	{
		ImGui::MenuItem(m_aidpanel.title.c_str(), "F1", &m_aidpanel.is_open);
		ImGui::MenuItem(m_scenariopanel.title.c_str(), "F1", &m_aidpanel.is_open);
		ImGui::MenuItem(STR_C("Timetable"), "F2", &m_timetablepanel.is_open);
		ImGui::MenuItem(m_debugpanel.name().c_str(), "F12", &m_debugpanel.is_open);
		ImGui::MenuItem(m_mappanel.name().c_str(), "Tab", &m_mappanel.is_open);
		ImGui::MenuItem(m_vehiclelist.name().c_str(), nullptr, &m_vehiclelist.is_open);
		ImGui::MenuItem(m_trainingcardpanel.name().c_str(), nullptr, &m_trainingcardpanel.is_open);
		ImGui::MenuItem(m_cameraviewpanel.name().c_str(), nullptr, &m_cameraviewpanel.is_open);
		if (DebugModeFlag)
			ImGui::MenuItem(m_perfgraphpanel.name().c_str(), nullptr, &m_perfgraphpanel.is_open);

		if (ImGui::MenuItem(m_timepanel.name().c_str()))
			m_timepanel.open();

		ImGui::EndMenu();
	}
}

void driver_ui::showDebugUI()
{
	m_debugpanel.is_open = !m_debugpanel.is_open;
}

// potentially processes provided input key. returns: true if key was processed, false otherwise
bool driver_ui::on_key(int const Key, int const Action)
{
	if (ui_layer::on_key(Key, Action))
		return true;

	switch (Key)
	{
	case GLFW_KEY_TAB:
	case GLFW_KEY_F1:
	case GLFW_KEY_F2:
	case GLFW_KEY_F3:
	case GLFW_KEY_F10:
	case GLFW_KEY_F12:
	{ // ui mode selectors

		if ((true == Global.ctrlState) || (true == Global.shiftState))
		{
			// only react to keys without modifiers
			return false;
		}

		if (Action != GLFW_PRESS)
		{
			return true;
		} // recognized, but ignored
	}

	default:
	{ // everything else
		break;
	}
	}

	switch (Key)
	{

	case GLFW_KEY_TAB:
	{
		m_mappanel.is_open = !m_mappanel.is_open;

		return true;
	}

	case GLFW_KEY_F1:
	{
		// basic consist info
		auto state = ((m_aidpanel.is_open == false) ? 0 : (m_aidpanel.is_expanded == false) ? 1 : 2);
		state = clamp_circular(++state, 3);

		m_aidpanel.is_open = (state > 0);
		m_aidpanel.is_expanded = (state > 1);

		return true;
	}

	case GLFW_KEY_F2:
	{
		// timetable
		auto state = ((m_timetablepanel.is_open == false) ? 0 : (m_timetablepanel.is_expanded == false) ? 1 : 2);
		state = clamp_circular(++state, 3);

		m_timetablepanel.is_open = (state > 0);
		m_timetablepanel.is_expanded = (state > 1);

		return true;
	}

	case GLFW_KEY_F3:
	{
		// debug panel
		m_scenariopanel.is_open = !m_scenariopanel.is_open;
		return true;
	}

	case GLFW_KEY_F12:
	{
		// debug panel
		if (Global.shiftState)
		{
			m_debugpanel.is_open = !m_debugpanel.is_open;
			return true;
		}
	}

	default:
	{
		break;
	}
	}

	return false;
}

// potentially processes provided mouse movement. returns: true if the input was processed, false otherwise
bool driver_ui::on_cursor_pos(double const Horizontal, double const Vertical)
{
	// intercept mouse movement when the pause window is on
	return m_paused;
}

// potentially processes provided mouse button. returns: true if the input was processed, false otherwise
bool driver_ui::on_mouse_button(int const Button, int const Action)
{
	// intercept mouse movement when the pause window is on
	return m_paused;
}

// updates state of UI elements
void driver_ui::update()
{

	auto const pausemask{1 | 2};
	auto ispaused{(false == DebugModeFlag) && ((Global.iPause & pausemask) != 0)};
	if ((ispaused != m_paused) && (false == Global.ControlPicking))
	{
		set_cursor(ispaused);
	}
	m_paused = ispaused;

	ui_layer::update();
}

void driver_ui::set_cursor(bool const Visible)
{

	if (Visible)
	{
		Application.set_cursor(GLFW_CURSOR_NORMAL);
		Application.set_cursor_pos(Global.window_size.x / 2, Global.window_size.y / 2);
	}
	else
	{
		Application.set_cursor(GLFW_CURSOR_DISABLED);
		Application.set_cursor_pos(0, 0);
	}
}

// render() subclass details
void driver_ui::render_()
{
	const std::string *rec_name = m_trainingcardpanel.is_recording();
	if (rec_name && m_cameraviewpanel.set_state(true))
	{
		m_cameraviewpanel.rec_name = *rec_name;
		m_cameraviewpanel.is_open = true;
	}
	else if (!rec_name)
		m_cameraviewpanel.set_state(false);

	// pause/quit modal
	auto const popupheader{STR_C("Simulation Paused")};

	ImGui::SetNextWindowSize(ImVec2(-1, -1));
	if (ImGui::BeginPopupModal(popupheader, nullptr, 0))
	{
		if ((ImGui::Button(STR_C("Resume"), ImVec2(150, 0))) || (ImGui::IsKeyReleased(ImGui::GetKeyIndex(ImGuiKey_Escape))))
		{
			m_relay.post(user_command::pausetoggle, 0.0, 0.0, GLFW_RELEASE, 0);
		}
		if (ImGui::Button(STR_C("Quit"), ImVec2(150, 0)))
		{
			Application.queue_quit(false);
		}
		if (!m_paused)
			ImGui::CloseCurrentPopup();
		ImGui::EndPopup();
	}
	else if (m_paused)
	{
		ImGui::OpenPopup(popupheader);
	}

	if (Global.desync != 0.0f)
	{
		ImGui::SetNextWindowSize(ImVec2(-1, -1));
		if (ImGui::Begin("network", nullptr, ImGuiWindowFlags_NoCollapse))
			ImGui::Text("desync: %0.2f", Global.desync);
		ImGui::End();
	}
}

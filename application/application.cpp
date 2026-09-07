/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/application.h"
#include "application/drivermode.h"
#include "application/editormode.h"
#include "application/scenarioloadermode.h"
#include "launcher/launchermode.h"

#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "simulation/simulation.h"
#include "simulation/simulationsounds.h"
#include "vehicle/Train.h"
#include "utilities/dictionary.h"
#include "scene/sceneeditor.h"
#include "rendering/renderer.h"
#include "application/uilayer.h"
#include "utilities/Logs.h"
#include "rendering/screenshot.h"
#include "utilities/translation.h"
#include "vehicle/Train.h"
#include "utilities/Timer.h"
#include "utilities/dictionary.h"
#include "version_info.h"
#include "network/statehash.h"
#include <chrono>
#include "utilities/translation.h"

#if WITH_DISCORD_RPC
#include "ref/discord-rpc/include/discord_rpc.h"
#include <discord_rpc.h>
#endif

#include <chrono>
#include "utilities/translation.h"

#ifdef _WIN32
#pragma comment(lib, "dsound.lib")
#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "dbghelp.lib")
#pragma comment(lib, "version.lib")
#endif

#ifdef __unix__
#include <unistd.h>
#include <sys/stat.h>
#endif

eu07_application Application;
screenshot_manager screenshot_man;
ui_layer uilayerstaticinitializer;

#ifdef _WIN32
extern "C"
{
	GLFWAPI HWND glfwGetWin32Window(GLFWwindow *window);
}

LRESULT APIENTRY WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
extern HWND Hwnd;
extern WNDPROC BaseWindowProc;
#endif

// user input callbacks

void focus_callback(GLFWwindow *window, int focus)
{
	Application.on_focus_change(focus != 0);
}

void framebuffer_resize_callback(GLFWwindow *, int w, int h)
{
	Global.fb_size = glm::ivec2(w, h);
}

void window_resize_callback(GLFWwindow *, int w, int h)
{
	Global.window_size = glm::ivec2(w, h);
	Application.on_window_resize(w, h);
}

void cursor_pos_callback(GLFWwindow *window, double x, double y)
{
	Global.cursor_pos = glm::ivec2(x, y);
	Application.on_cursor_pos(x, y);
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
	Application.on_mouse_button(button, action, mods);
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
	Application.on_scroll(xoffset, yoffset);
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{

	Application.on_key(key, scancode, action, mods);
}

void char_callback(GLFWwindow *window, unsigned int c)
{
	Application.on_char(c);
}

// public:

void eu07_application::queue_screenshot()
{
	m_screenshot_queued = true;
}

int eu07_application::run_crashgui()
{
	bool autoup = false;

	while (!glfwWindowShouldClose(m_windows.front()))
	{
		glfwPollEvents();

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		ui_layer::begin_ui_frame_internal();

		bool y, n;

		if (Global.asLang == "pl")
		{
			ImGui::Begin("Raportowanie błędów", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoResize);
			ImGui::TextUnformatted("Podczas ostatniego uruchomienia symulatora wystąpił błąd.\nWysłać raport o błędzie do deweloperów?\n");
			ImGui::TextUnformatted(("Usługa udostępniana przez " + crashreport_get_provider() + "\n").c_str());
			y = ImGui::Button("Tak", ImVec2S(60, 0));
			ImGui::SameLine();
			ImGui::Checkbox("W przyszłości przesyłaj raporty o błędach automatycznie", &autoup);

			ImGui::SameLine();
			ImGui::TextDisabled("(?)");
			if (ImGui::IsItemHovered())
			{
				ImGui::BeginTooltip();
				ImGui::TextUnformatted("W celu wyłączenia tej funkcji będzie trzeba skasować plik crashdumps/autoupload_enabled.conf");
				ImGui::EndTooltip();
			}

			ImGui::NewLine();
			n = ImGui::Button("Nie", ImVec2S(60, 0));
			ImGui::End();
		}
		else
		{
			ImGui::Begin("Crash reporting", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoResize);
			ImGui::TextUnformatted("Crash occurred during last launch of the simulator.\nSend crash report to developers?\n");
			ImGui::TextUnformatted(("Service provided by " + crashreport_get_provider() + "\n").c_str());
			y = ImGui::Button("Yes", ImVec2S(60, 0));
			ImGui::SameLine();
			ImGui::Checkbox("In future send crash reports automatically", &autoup);

			ImGui::SameLine();
			ImGui::TextDisabled("(?)");
			if (ImGui::IsItemHovered())
			{
				ImGui::BeginTooltip();
				ImGui::TextUnformatted("To disable this feature remove file crashdumps/autoupload_enabled.conf");
				ImGui::EndTooltip();
			}

			ImGui::NewLine();
			n = ImGui::Button("No", ImVec2S(60, 0));
			ImGui::End();
		}

		ui_layer::render_internal();
		glfwSwapBuffers(m_windows.front());

		if (y)
		{
			crashreport_upload_accept();
			if (autoup)
				crashreport_set_autoupload();
			return 0;
		}

		if (n)
		{
			crashreport_upload_reject();
			return 0;
		}
	}
	return -1;
}
#if WITH_DISCORD_RPC
void eu07_application::DiscordRPCService()
{
	// initialize discord-rpc
	WriteLog("Initializing Discord Rich Presence...");
	static const char *discord_app_id = "1343662664504840222";
	DiscordEventHandlers handlers;
	memset(&handlers, 0, sizeof(handlers));
	Discord_Initialize(discord_app_id, &handlers, 1, nullptr);

	// calculate startup timestamp
	auto now = std::chrono::system_clock::now();
	auto now_c = std::chrono::system_clock::to_time_t(now);

	// Init RPC object
	static DiscordRichPresence discord_rpc;
	memset(&discord_rpc, 0, sizeof(discord_rpc));
	discord_rpc.startTimestamp = static_cast<int64_t>(now_c);
	discord_rpc.largeImageText = "MaSzyna";

	// run loop
	while (!glfwWindowShouldClose(m_windows.front()) && !m_modestack.empty() && !Global.applicationQuitOrder)
	{
		auto currentMode = m_modestack.top();
		if (currentMode == mode::launcher)
		{
			// in launcher mode

			discord_rpc.state = Translations.lookup_c("In main menu");
			discord_rpc.details = Translations.lookup_c("Browsing scenarios...");
			discord_rpc.largeImageKey = "";
			discord_rpc.largeImageText = "MaSzyna";
			// RPC upload
			Discord_UpdatePresence(&discord_rpc);

			std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // update RPC every 5 secs
			continue;
		}
		else if (currentMode == mode::scenarioloader)
		{
			std::string rpcScnName = Global.SceneryFile;
			if (rpcScnName[0] == '$')
				rpcScnName.erase(0, 1);
			rpcScnName.erase(rpcScnName.size() - 4, 4);
			if (rpcScnName.find('_') != std::string::npos)
			{
				std::replace(rpcScnName.begin(), rpcScnName.end(), '_', ' ');
			}

			// realworld timestamp from datetime
			static std::string state = Translations.lookup_s("Scenery: ") + rpcScnName;
			discord_rpc.state = state.c_str();
			discord_rpc.details = Translations.lookup_c("Loading scenery...");
			discord_rpc.largeImageKey = "logo";
			discord_rpc.largeImageText = "MaSzyna";

			// RPC upload
			Discord_UpdatePresence(&discord_rpc);

			std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // update RPC every 5 secs
			continue;
		}

		if (currentMode != mode::driver)
			continue;

		// Discord RPC updater
		if (simulation::is_ready)
		{
			std::string PlayerVehicle;
			if (simulation::Train != nullptr)
			{
				PlayerVehicle = simulation::Train->name();
				// make to upper
				for (auto &c : PlayerVehicle)
					c = toupper(c);

				PlayerVehicle = Translations.lookup_s("Driving: ") + PlayerVehicle;
				discord_rpc.details = PlayerVehicle.c_str();

				uint16_t playerTrainVelocity = simulation::Train->Dynamic()->GetVelocity();
				if (playerTrainVelocity > 1)
				{
					// ikonka ze jedziemy i nie spimy
					discord_rpc.smallImageKey = "driving";
					std::string smallText = Translations.lookup_s("Speed: ") + std::to_string(playerTrainVelocity) + " km/h";
					discord_rpc.smallImageText = smallText.c_str();
				}
				else
				{
					// krecimy postoj
					discord_rpc.smallImageKey = "halt";
					discord_rpc.smallImageText = Translations.lookup_c("Stopped");
				}
			}

			Discord_UpdatePresence(&discord_rpc);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // update RPC every 5 secs
	}

	Discord_Shutdown();

}
#endif

int eu07_application::init(int Argc, char *Argv[])
{

	int result{0};
	// start logging service
	std::thread sLoggingService(LogService);
	Global.threads.emplace("LogService", std::move(sLoggingService));

	init_debug();
	init_files();
	if ((result = init_settings(Argc, Argv)) != 0)
	{
		ErrorLog("Failed to initialize settings! Maybe you're missing eu07.ini file?");
		return result;
	}

	if (!Global.random_seed)
		Global.random_seed = std::random_device{}();
	Global.random_engine.seed(Global.random_seed);

	// configure the OS console according to Globals.ShowSystemConsole.
	// must run AFTER init_settings (so the ini-loaded value is honoured) and
	// BEFORE the first WriteLog below (so colored \033[...] sequences emitted
	// by utilities/Logs.cpp land in a console with VT processing enabled).
	init_console();

	WriteLog("Starting MaSzyna rail vehicle simulator (release: " + Global.asVersion + ")");
	WriteLog("For online documentation and additional files refer to: http://eu07.pl");
	WriteLog("Authors: Marcin_EU, McZapkie, ABu, Winger, Tolaris, nbmx, OLO_EU, Bart, Quark-t, "
	         "ShaXbee, Oli_EU, youBy, KURS90, Ra, hunter, szociu, Stele, Q, firleju and others");

	if (!crashreport_get_provider().empty())
		WriteLog("Crashdump analysis provided by " + crashreport_get_provider() + "\n");

	{
		WriteLog("// settings");
		std::stringstream settingspipe;
		Global.export_as_text(settingspipe);
		WriteLog(settingspipe.str());
	}

	// cruel way to prevent crashes because of threaded upload from python
	if (Global.NvRenderer)
		Global.python_threadedupload = false;

	WriteLog("// startup");

	if ((result = init_glfw()) != 0)
	{
		ErrorLog("Failed to initialize glfw!");
		return result;
	}
	if (needs_ogl() && (result = init_ogl()) != 0)
	{
		ErrorLog("Failed to initialize ogl!");
		return result;
	}

	if (crashreport_is_pending())
	{ // run crashgui as early as possible
		if ((result = run_crashgui()) != 0)
		{
			ErrorLog("Failed to run crash gui!");
			return result;
		}
	}

	if ((result = init_locale()) != 0)
	{
		ErrorLog("Failed to initialize locales! Maybe you're missing lang directory?");
		return result;
	}

	if ((result = init_gfx()) != 0)
	{
		ErrorLog("Failed to initialize GFX!");
		return result;
	}

	if ((result = init_ui()) != 0)
	{ // ui now depends on activated renderer
		ErrorLog("Failed to init UI!");
		return result;
	}
	if ((result = init_audio()) != 0)
	{
		ErrorLog("Failed to initialize OpenAL");
		return result;
	}
	if ((result = init_data()) != 0)
	{
		ErrorLog("Failed to load data/ contents! Maybe your installation is broken?");
		return result;
	}
	crashreport_add_info("python_enabled", Global.python_enabled ? "yes" : "no");
	if (Global.python_enabled)
	{
		m_taskqueue.init();
	}
	if ((result = init_modes()) != 0)
	{
		ErrorLog("Failed to initialize game modes");
		return result;
	}


	#if WITH_DISCORD_RPC
	// Run DiscordRPC service
	std::thread sDiscordRPC(&eu07_application::DiscordRPCService, this);
	Global.threads.emplace("DiscordRPC", std::move(sDiscordRPC));
	#endif

	if (!init_network())
		return -1;

	return result;
}

void eu07_application::queue_quit(bool direct)
{
	// a network client leaving is its own business; replicating the request would shut
	// down everybody else's session as well
	if (direct || is_client() || !m_modes[m_modestack.top()]->is_command_processor())
	{
		glfwSetWindowShouldClose(m_windows[0], GLFW_TRUE);
		return;
	}

	command_relay relay;
	relay.post(user_command::quitsimulation, 0.0, 0.0, GLFW_PRESS, 0);
}

bool eu07_application::is_server() const
{

	return m_network && m_network->servers;
}

bool eu07_application::is_client() const
{

	return m_network && m_network->client;
}

void eu07_application::request_vehicle_claim(network::NetworkEntityId const Entity)
{
	if (m_network)
		m_network->request_claim(Entity);
}

void eu07_application::request_vehicle_leave(network::NetworkEntityId const Entity)
{
	if (m_network)
		m_network->request_leave(Entity);
}

namespace {

// the state digest is logged when it starts differing and then only this often, so that
// a persistent difference does not bury the rest of the log
uint64_t const NETWORK_DIGEST_LOG_INTERVAL = 1800;

} // namespace

void eu07_application::network_scenario_loaded()
{
	if (m_network)
		m_network->notify_scenario_loaded();
}

int eu07_application::run()
{
	auto frame{0};
	// main application loop
	while (!glfwWindowShouldClose(m_windows.front()) && !m_modestack.empty())
	{
		Timer::subsystem.mainloop_total.start();
		glfwPollEvents();

		if (m_headtrack)
			m_headtrack->update();

		begin_ui_frame();

		// -------------------------------------------------------------------
		// multiplayer command relaying logic can seem a bit complex
		//
		// we are simultaneously:
		// => master (not client) OR slave (client)
		// => server OR not
		//
		// trivia: being client and server is possible

		if (m_modes[m_modestack.top()]->is_command_processor())
		{
			// active mode is doing real calculations (e.g. drivermode)
			int loop_remaining = MAX_NETWORK_PER_FRAME;
			while (--loop_remaining > 0)
			{
				command_queue::commands_map commands_to_exec;
				command_queue::commands_map local_commands = simulation::Commands.pop_intercept_queue();
				network::frame_delta authoritative;

				// if we're the server
				if (m_network && m_network->servers)
				{
					// our own input goes through the same authority layer as everybody
					// else's, only without the transport in between
					network::filter_commands(network::PEER_HOST, local_commands);

					// fetch from network layer command requests received from clients
					command_queue::commands_map remote_commands = m_network->servers->pop_commands();

					// push these into local queue
					add_to_dequemap(local_commands, remote_commands);
				}

				// if we're slave
				if (m_network && m_network->client)
				{
					// take everything the authority has sent since the last frame. the
					// commands are carried out at once; the world itself runs on our own
					// clock rather than replaying the server's frame times, which is what
					// made it run at the wrong speed whenever the two machines drew at
					// different rates, and put every buffered frame between the player and
					// their own controls
					authoritative = m_network->client->take_pending(commands_to_exec);

					// the authority owns the timeline, so we take its step number as ours
					if (authoritative.valid)
						Global.simulation_tick = authoritative.tick;

					// what we do to the train we run ourselves happens now, not after a
					// round trip. it still goes to the server, so that everybody else sees
					// it; the echo of it is dropped when it comes back
					command_queue::commands_map predicted;
					network::collect_predictable(local_commands, predicted);
					add_to_dequemap(commands_to_exec, predicted);

					// and send our local commands to master
					m_network->client->send_commands(local_commands);

					loop_remaining = -1;
				}
				// if we're master
				else
				{
					// just push local commands to execution
					add_to_dequemap(commands_to_exec, local_commands);

					// we are the one counting the steps of this world
					++Global.simulation_tick;

					loop_remaining = -1;
				}

				// send commands to command queue
				simulation::Commands.push_commands(commands_to_exec);

				// do actual frame processing (depending on mode)
				if (!m_modes[m_modestack.top()]->update())
					return 0;

				// update continuous commands
				simulation::Commands.update();

				auto const statehash = (m_network ? network::state_hash() : 0);

				// if we're the server
				if (m_network && m_network->servers)
				{
					// send delta, state digest, and commands we just executed to clients
					double delta = Timer::GetDeltaTime();
					double render = Timer::GetDeltaRenderTime();
					m_network->servers->push_delta(render, delta, Global.simulation_tick, statehash, commands_to_exec);
				}

				// if we're slave
				if (m_network && m_network->client)
				{
					// the digest is a diagnostic, not a control input: a client runs its
					// own physics, so the two will never agree bit for bit and demanding
					// that they do only produces noise. what actually keeps a client in
					// line is the authoritative state the server streams to it
					if (authoritative.valid && authoritative.state_hash_version == network::STATE_HASH_VERSION)
					{
						if (statehash != authoritative.state_hash)
						{
							++m_statemismatches;
							if (m_statemismatches == 1 || (m_statemismatches % NETWORK_DIGEST_LOG_INTERVAL) == 0)
							{
								WriteLog("net: state digest differs at tick " + std::to_string(authoritative.tick) + " (" + std::to_string(m_statemismatches) +
								             " steps): local " + std::to_string(statehash) + ", authoritative " + std::to_string(authoritative.state_hash),
								         logtype::net);
							}
						}
						else if (m_statemismatches != 0)
						{
							WriteLog("net: state digest back in step at tick " + std::to_string(authoritative.tick), logtype::net);
							m_statemismatches = 0;
						}
					}

				}
			}

			m_modes[m_modestack.top()]->set_progress(0.0f, 0.0f);
		}
		else
		{
			// active mode is loader

			// clear local command queue
			simulation::Commands.pop_intercept_queue();

			// do actual frame processing
			if (!m_modes[m_modestack.top()]->update())
				return 0;
		}

		// -------------------------------------------------------------------

		// keep streamed terrain loaded around the active camera in every mode (editor + driver),
		// once the simulation is live (avoid streaming while the scenery is still loading)
		if (simulation::is_ready && EditorTerrain.active())
			EditorTerrain.update(Global.pCamera.Pos);

		m_taskqueue.update();
		opengl_texture::reset_unit_cache();

		if (!GfxRenderer->Render())
			break;

		GfxRenderer->SwapBuffers();

		if (m_modestack.empty())
			break;

		m_modes[m_modestack.top()]->on_event_poll();

		if (m_screenshot_queued)
		{
			m_screenshot_queued = false;
			GfxRenderer->MakeScreenshot();
		}

		if (m_network)
			m_network->update();

		auto const frametime{Timer::subsystem.mainloop_total.stop()};
		if (Global.minframetime.count() != 0.0f && (Global.minframetime - frametime).count() > 0.0f)
		{
			std::this_thread::sleep_for(Global.minframetime - frametime);
		}
	}
	// Handled by application.exit
	// Global.applicationQuitOrder = true;
	// auto it = Global.threads.find("LogService");
	// if (it != Global.threads.end())
	// {
	// 	if (it->second.joinable())
	// 		it->second.join();
	// }
	//
	// it = Global.threads.find("DiscordRPC");
	// if (it != Global.threads.end())
	// {
	// 	if (it->second.joinable())
	// 		it->second.join();
	// }
	return 0;
}

// issues request for a worker thread to perform specified task. returns: true if task was scheduled
bool eu07_application::request(python_taskqueue::task_request const &Task)
{

	auto const result{m_taskqueue.insert(Task)};
	if (false == result && Task.input != nullptr)
	{
		// clean up allocated resources since the worker won't
	}
	return result;
}

// ensures the main thread holds the python gil and can safely execute python calls
void eu07_application::acquire_python_lock()
{

	m_taskqueue.acquire_lock();
}

// frees the python gil and swaps out the main thread
void eu07_application::release_python_lock()
{

	m_taskqueue.release_lock();
}

void eu07_application::exit()
{
	Global.applicationQuitOrder = true;
	for (auto &mode : m_modes)
		mode.reset();

	GfxRenderer->Shutdown();
	m_network.reset();

	//    SafeDelete( simulation::Train );
	SafeDelete(simulation::Region);

	ui_layer::shutdown();

	for (auto *window : m_windows)
	{
		glfwDestroyWindow(window);
	}
	m_taskqueue.exit();
	glfwPollEvents(); // TODO: This fixes a segfault on Wayland when closing. Remove after updating glfw to 3.5.
	glfwTerminate();

	if (!Global.exec_on_exit.empty())
		system(Global.exec_on_exit.c_str());

	auto it = Global.threads.find("LogService");
	if (it != Global.threads.end())
	{
		if (it->second.joinable())
			it->second.join();
	}

	#if WITH_DISCORD_RPC
	it = Global.threads.find("DiscordRPC");
	if (it != Global.threads.end())
	{
		if (it->second.joinable())
			it->second.join();
	}
	#endif
}

void eu07_application::render_ui()
{

	if (m_modestack.empty())
	{
		return;
	}

	m_modes[m_modestack.top()]->render_ui();
}

void eu07_application::begin_ui_frame()
{

	if (m_modestack.empty())
	{
		return;
	}

	m_modes[m_modestack.top()]->begin_ui_frame();
}

bool eu07_application::pop_mode()
{
	if (m_modestack.empty())
	{
		return false;
	}

	m_modes[m_modestack.top()]->exit();
	m_modestack.pop();
	return true;
}

bool eu07_application::push_mode(mode const Mode)
{

	if (Mode >= count_)
		return false;

	if (!m_modes[Mode])
	{
		if (Mode == launcher)
			m_modes[Mode] = std::make_shared<launcher_mode>();
		if (Mode == scenarioloader)
			m_modes[Mode] = std::make_shared<scenarioloader_mode>();
		if (Mode == driver)
			m_modes[Mode] = std::make_shared<driver_mode>();
		if (Mode == editor)
			m_modes[Mode] = std::make_shared<editor_mode>();

		if (!m_modes[Mode]->init())
			return false;
	}

	m_modes[Mode]->enter();
	m_modestack.push(Mode);

	return true;
}

void eu07_application::set_title(std::string const &Title)
{

	glfwSetWindowTitle(m_windows.front(), Title.c_str());
}

void eu07_application::set_progress(float const Progress, float const Subtaskprogress)
{

	if (m_modestack.empty())
	{
		return;
	}

	m_modes[m_modestack.top()]->set_progress(Progress, Subtaskprogress);
}

void eu07_application::set_tooltip(std::string const &Tooltip)
{

	if (m_modestack.empty())
	{
		return;
	}

	m_modes[m_modestack.top()]->set_tooltip(Tooltip);
}

void eu07_application::set_cursor(int const Mode)
{

	ui_layer::set_cursor(Mode);
}

void eu07_application::set_cursor_pos(double const Horizontal, double const Vertical)
{

	glfwSetCursorPos(m_windows.front(), Horizontal, Vertical);
}

/*
// provides keyboard mapping associated with specified control item
std::string
eu07_application::get_input_hint( user_command const Command ) const {

    if( m_modestack.empty() ) { return ""; }

    return m_modes[ m_modestack.top() ]->get_input_hint( Command );
}
*/

void eu07_application::on_key(int const Key, int const Scancode, int const Action, int const Mods)
{

	if (ui_layer::key_callback(Key, Scancode, Action, Mods))
		return;

	if (m_modestack.empty())
	{
		return;
	}

#ifdef __unix__
	if (Key == GLFW_KEY_LEFT_SHIFT || Key == GLFW_KEY_RIGHT_SHIFT)
		Global.shiftState = Action == GLFW_PRESS;
	if (Key == GLFW_KEY_LEFT_CONTROL || Key == GLFW_KEY_RIGHT_CONTROL)
		Global.ctrlState = Action == GLFW_PRESS;
	if (Key == GLFW_KEY_LEFT_ALT || Key == GLFW_KEY_RIGHT_ALT)
		Global.altState = Action == GLFW_PRESS;
#endif

	m_modes[m_modestack.top()]->on_key(Key, Scancode, Action, Mods);
}

void eu07_application::on_cursor_pos(double const Horizontal, double const Vertical)
{

	if (m_modestack.empty())
	{
		return;
	}

	m_modes[m_modestack.top()]->on_cursor_pos(Horizontal, Vertical);
}

void eu07_application::on_mouse_button(int const Button, int const Action, int const Mods)
{

	if (ui_layer::mouse_button_callback(Button, Action, Mods))
		return;

	if (m_modestack.empty())
	{
		return;
	}

	m_modes[m_modestack.top()]->on_mouse_button(Button, Action, Mods);
}

void eu07_application::on_scroll(double const Xoffset, double const Yoffset)
{

	if (ui_layer::scroll_callback(Xoffset, Yoffset))
		return;

	if (m_modestack.empty())
	{
		return;
	}

	m_modes[m_modestack.top()]->on_scroll(Xoffset, Yoffset);
}

void eu07_application::on_char(unsigned int c)
{
	if (ui_layer::char_callback(c))
		return;
}

void eu07_application::on_focus_change(bool focus)
{
	if (Global.bInactivePause && m_network.has_value() && !m_network->client)
	{ // jeśli ma być pauzowanie okna w tle
		command_relay relay;
		relay.post(user_command::focuspauseset, focus ? 1.0 : 0.0, 0.0, GLFW_PRESS, 0);
	}
}

void eu07_application::on_window_resize(int w, int h)
{
	if (m_modestack.empty())
		return;
	m_modes[m_modestack.top()]->on_window_resize(w, h);
}

GLFWwindow *eu07_application::window(int const Windowindex, bool visible, int width, int height, GLFWmonitor *monitor, bool keep_ownership, bool share_ctx)
{

	if (Windowindex >= 0)
	{
		return Windowindex < m_windows.size() ? m_windows[Windowindex] : nullptr;
	}
	// for index -1 create a new child window

	auto const *vmode{glfwGetVideoMode(monitor ? monitor : glfwGetPrimaryMonitor())};

	// match requested video mode to current to allow for
	// fullwindow creation when resolution is the same
	glfwWindowHint(GLFW_RED_BITS, vmode->redBits);
	glfwWindowHint(GLFW_GREEN_BITS, vmode->greenBits);
	glfwWindowHint(GLFW_BLUE_BITS, vmode->blueBits);
	glfwWindowHint(GLFW_REFRESH_RATE, vmode->refreshRate);

	glfwWindowHint(GLFW_VISIBLE, visible);

	auto *childwindow = glfwCreateWindow(width, height, "eu07window", monitor, share_ctx ? m_windows.front() : nullptr);
	if (!childwindow)
		return nullptr;

	if (keep_ownership)
		m_windows.emplace_back(childwindow);

	glfwFocusWindow(m_windows.front()); // restore focus to main window

	return childwindow;
}

// private:
GLFWmonitor *eu07_application::find_monitor(const std::string &str) const
{
	int monitor_count;
	GLFWmonitor **monitors = glfwGetMonitors(&monitor_count);

	for (size_t i = 0; i < monitor_count; i++)
	{
		if (describe_monitor(monitors[i]) == str)
			return monitors[i];
	}

	return nullptr;
}

std::string eu07_application::describe_monitor(GLFWmonitor *monitor) const
{
	std::string name(glfwGetMonitorName(monitor));
	std::replace(std::begin(name), std::end(name), ' ', '_');

	int x, y;
	glfwGetMonitorPos(monitor, &x, &y);

	return name + ":" + std::to_string(x) + "," + std::to_string(y);
}

// private:

bool eu07_application::needs_ogl() const
{
	return !Global.NvRenderer;
}

void eu07_application::init_debug()
{

#if defined(_MSC_VER) && defined(_DEBUG)
	// memory leaks
	_CrtSetDbgFlag(_CrtSetDbgFlag(_CRTDBG_REPORT_FLAG) | _CRTDBG_LEAK_CHECK_DF);
	/*
	// floating point operation errors
	auto state { _clearfp() };
	state = _control87( 0, 0 );
	// this will turn on FPE for #IND and zerodiv
	state = _control87( state & ~( _EM_ZERODIVIDE | _EM_INVALID ), _MCW_EM );
	*/
#endif
}

// Honours Globals.ShowSystemConsole (default: true).
//
// Windows: if the toggle is on, ensure a console is attached (allocate one when
// the binary was linked as the WINDOWS subsystem and none is inherited), then
// reopen the C stdio streams onto it and turn on ENABLE_VIRTUAL_TERMINAL_PROCESSING
// so the ANSI colour escapes used by utilities/Logs.cpp render correctly.
// If the toggle is off, hide the console window (or free an allocated one) so
// the GUI launches without a stray cmd window.
//
// Other platforms: no-op. stdout is the launching terminal (or already detached);
// hiding it on Linux/macOS would mean redirecting to /dev/null and is left to the
// user's shell (e.g. "./eu07 >/dev/null 2>&1").
void eu07_application::init_console()
{
#ifdef _WIN32
	HWND consoleWnd = ::GetConsoleWindow();
	const bool hadConsole = consoleWnd != nullptr;

	if (Global.ShowSystemConsole)
	{
		if (!hadConsole)
		{
			// no console inherited (e.g. WINDOWS subsystem build, or launched
			// detached) -- create one and wire stdio to it so printf in
			// utilities/Logs.cpp actually reaches the user
			if (::AllocConsole())
			{
				FILE *fp = nullptr;
				freopen_s(&fp, "CONOUT$", "w", stdout);
				freopen_s(&fp, "CONOUT$", "w", stderr);
				freopen_s(&fp, "CONIN$",  "r", stdin);
				// keep C++ streams in sync with the redirected C streams
				std::ios::sync_with_stdio(true);
				std::cout.clear();
				std::cerr.clear();
				std::clog.clear();
				std::cin.clear();
				consoleWnd = ::GetConsoleWindow();
			}
		}

		// enable ANSI virtual-terminal processing so the colour escapes in
		// utilities/Logs.cpp ("\033[1;37;41m...", "\033[32m...") render as
		// colours instead of literal garbage on the conhost.
		//
		// IMPORTANT: do NOT rely on GetStdHandle(STD_OUTPUT_HANDLE). After
		// AllocConsole + freopen_s the CRT may have rewired things, and on a
		// console-subsystem build the std handle can also point to a redirected
		// pipe/file rather than the actual console screen buffer -- in either
		// case SetConsoleMode silently fails and the escape bytes leak through
		// as text ("←[32m" etc). Open CONOUT$ directly to get the real screen
		// buffer handle, then flip VT on it.
		auto enable_vt = [](const char *devName) {
			HANDLE h = ::CreateFileA(
				devName,
				GENERIC_READ | GENERIC_WRITE,
				FILE_SHARE_READ | FILE_SHARE_WRITE,
				nullptr,
				OPEN_EXISTING,
				0,
				nullptr);
			if (h == INVALID_HANDLE_VALUE)
				return;
			DWORD mode = 0;
			if (::GetConsoleMode(h, &mode))
			{
				::SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING | ENABLE_PROCESSED_OUTPUT);
			}
			::CloseHandle(h);
		};
		// CONOUT$ is the console's active screen buffer; enabling VT on it
		// affects everything that ends up being written there, regardless of
		// which FILE* / std handle the writer used
		enable_vt("CONOUT$");

		if (consoleWnd)
		{
			::ShowWindow(consoleWnd, SW_SHOW);
			// give the console a recognisable title
			::SetConsoleTitleA("MaSzyna log");
		}
	}
	else
	{
		// user wants no console window -- hide whatever we have
		if (consoleWnd)
		{
			::ShowWindow(consoleWnd, SW_HIDE);
		}
		// if we'd allocated one ourselves on a previous run path we'd FreeConsole here,
		// but on first init the console (if any) was inherited from the launcher and
		// belongs to it -- just hiding the window is the least-surprising behaviour
	}
#endif
}

void eu07_application::init_files()
{

#ifdef _WIN32
	DeleteFile("log.txt");
	DeleteFile("errors.txt");
	CreateDirectory("logs", nullptr);
#elif __unix__
	unlink("log.txt");
	unlink("errors.txt");
	mkdir("logs", 0755);
#endif
}
namespace fs = std::filesystem;

namespace {

// port used when the user did not spell one out in --host / --connect
uint32_t const EU07_DEFAULT_NETWORK_PORT = 7420;

// completes a user supplied endpoint into the "address:port" form expected by the tcp backend.
// note: only the plain ipv4/hostname form is split, anything with more colons is passed through
std::string network_endpoint(std::string const &Argument, std::string const &Defaultaddress)
{
	auto address{Argument};
	std::string port;

	if (std::count(address.begin(), address.end(), ':') == 1)
	{
		auto const separator{address.find(':')};
		port = address.substr(separator + 1);
		address = address.substr(0, separator);
	}

	if (address.empty())
		address = Defaultaddress;
	if (port.empty())
		port = std::to_string(EU07_DEFAULT_NETWORK_PORT);

	return address + ":" + port;
}

void print_usage(std::string const &Executable)
{
	std::cout
	    << "usage: " << Executable << " [options]\n"
	    << "  -s, --scenario <path>        scenario file to load\n"
	    << "  -v, --vehicle <name>         vehicle to start in\n"
	    << "      --host [address:port]    host a multiplayer session (default 0.0.0.0:"
	    << EU07_DEFAULT_NETWORK_PORT << ")\n"
	    << "      --connect <address:port> join a multiplayer session (default port "
	    << EU07_DEFAULT_NETWORK_PORT << ")\n"
	    << "  -h, --help                   this message"
	    << std::endl;
}

} // namespace

int eu07_application::init_settings(int Argc, char *Argv[])
{
	Global.asVersion = VERSION_INFO;

	fs::path iniPath = user_config_path("eu07.ini");

	if (!iniPath.empty() && fs::exists(iniPath))
	{
		Global.LoadIniFile(iniPath.string().c_str());
	}
	else
	{
		Global.LoadIniFile("eu07.ini");
	}

	// process command line arguments
	for (int i = 1; i < Argc; ++i)
	{

		std::string token{Argv[i]};

		if (token == "-s" || token == "--scenario")
		{
			if (i + 1 < Argc)
			{
				Global.SceneryFile = ToLower(Argv[++i]);
			}
		}
		else if (token == "-v" || token == "--vehicle")
		{
			if (i + 1 < Argc)
			{
				Global.local_start_vehicle = ToLower(Argv[++i]);
				Global.local_start_vehicle_override = true;
			}
		}
		else if (token == "--host")
		{
			// the address is optional, so that a bare --host just listens on every interface
			std::string endpoint{std::string("0.0.0.0:") + std::to_string(EU07_DEFAULT_NETWORK_PORT)};
			if (i + 1 < Argc && Argv[i + 1][0] != '-')
			{
				endpoint = network_endpoint(Argv[++i], "0.0.0.0");
			}
			Global.network_servers.emplace_back("tcp", endpoint);
		}
		else if (token == "--connect")
		{
			if (i + 1 >= Argc)
			{
				std::cout << "--connect requires a server address" << std::endl;
				return -1;
			}
			Global.network_client.emplace("tcp", network_endpoint(Argv[++i], "127.0.0.1"));
		}
		else if (token == "-h" || token == "--help")
		{
			print_usage(Argv[0]);
			return -1;
		}
		else
		{
			print_usage(Argv[0]);
			return -1;
		}
	}

	return 0;
}

int eu07_application::init_locale()
{

	Translations.init();

	return 0;
}

int eu07_application::init_glfw()
{
	{
		int glfw_major, glfw_minor, glfw_rev;
		glfwGetVersion(&glfw_major, &glfw_minor, &glfw_rev);
		m_glfwversion = glfw_major * 10000 + glfw_minor * 100 + glfw_minor;
	}

#ifdef GLFW_ANGLE_PLATFORM_TYPE
	if (m_glfwversion >= 30400)
	{
		int platform = GLFW_ANGLE_PLATFORM_TYPE_NONE;
		if (Global.gfx_angleplatform == "opengl")
			platform = GLFW_ANGLE_PLATFORM_TYPE_OPENGL;
		else if (Global.gfx_angleplatform == "opengles")
			platform = GLFW_ANGLE_PLATFORM_TYPE_OPENGLES;
		else if (Global.gfx_angleplatform == "d3d9")
			platform = GLFW_ANGLE_PLATFORM_TYPE_D3D9;
		else if (Global.gfx_angleplatform == "d3d11")
			platform = GLFW_ANGLE_PLATFORM_TYPE_D3D11;
		else if (Global.gfx_angleplatform == "vulkan")
			platform = GLFW_ANGLE_PLATFORM_TYPE_VULKAN;
		else if (Global.gfx_angleplatform == "metal")
			platform = GLFW_ANGLE_PLATFORM_TYPE_METAL;

		glfwInitHint(GLFW_ANGLE_PLATFORM_TYPE, platform);
	}
#endif

	if (glfwInit() == GLFW_FALSE)
	{
		ErrorLog("Bad init: failed to initialize glfw");
		return -1;
	}

	{
		int monitor_count;
		GLFWmonitor **monitors = glfwGetMonitors(&monitor_count);

		WriteLog("available monitors:");
		for (size_t i = 0; i < monitor_count; i++)
		{
			WriteLog(describe_monitor(monitors[i]));
		}
	}

	auto *monitor{find_monitor(Global.fullscreen_monitor)};
	if (!monitor)
		monitor = glfwGetPrimaryMonitor();

	glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);
	if ((Global.gfx_skippipeline || Global.LegacyRenderer) && Global.iMultisampling > 0)
	{
		glfwWindowHint(GLFW_SAMPLES, 1 << Global.iMultisampling);
	}

	crashreport_add_info("gfxrenderer", Global.GfxRenderer);

	if (!needs_ogl())
	{
		glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
	}
	else
	{
		if (!Global.LegacyRenderer)
		{
			Global.bUseVBO = true;
			// activate core profile for opengl 3.3 renderer
			if (!Global.gfx_usegles)
			{
				glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
				glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
				glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
				glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
			}
			else
			{
#ifdef GLFW_CONTEXT_CREATION_API
				if (m_glfwversion >= 30200)
					glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);
#endif
				glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
				glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
				glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
			}
		}
		else
		{
			if (Global.gfx_usegles)
			{
				ErrorLog("legacy renderer not supported in gles mode");
				return -1;
			}
			Global.gfx_shadergamma = false;
			glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);
			glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
			glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
		}

		if (Global.gfx_gldebug)
			glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
	}

	glfwWindowHint(GLFW_SRGB_CAPABLE, !Global.gfx_shadergamma);

	if (Global.fullscreen_windowed)
	{
		auto const mode = glfwGetVideoMode(monitor);
		Global.window_size.x = mode->width;
		Global.window_size.y = mode->height;
		Global.bFullScreen = true;
	}

	auto *mainwindow = window(-1, true, Global.window_size.x, Global.window_size.y, Global.bFullScreen ? monitor : nullptr, true, false);

	if (mainwindow == nullptr)
	{
		ErrorLog("Bad init: failed to create glfw window");
		return -1;
	}

	glfwMakeContextCurrent(mainwindow);
	glfwSwapInterval(Global.VSync ? 1 : 0); // vsync

	{
		int width, height;

		glfwGetFramebufferSize(mainwindow, &width, &height);
		framebuffer_resize_callback(mainwindow, width, height);

		glfwGetWindowSize(mainwindow, &width, &height);
		window_resize_callback(mainwindow, width, height);
	}

#ifdef _WIN32
	// setup wrapper for base glfw window proc, to handle copydata messages
	Hwnd = glfwGetWin32Window(mainwindow);
	BaseWindowProc = (WNDPROC)::SetWindowLongPtr(Hwnd, GWLP_WNDPROC, (LONG_PTR)WndProc);
	// switch off the topmost flag
	::SetWindowPos(Hwnd, HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE);
#endif

	return 0;
}

void eu07_application::init_callbacks()
{

	auto *window{m_windows.front()};
	glfwSetWindowSizeCallback(window, window_resize_callback);
	glfwSetFramebufferSizeCallback(window, framebuffer_resize_callback);
	glfwSetCursorPosCallback(window, cursor_pos_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetKeyCallback(window, key_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetCharCallback(window, char_callback);
	glfwSetWindowFocusCallback(window, focus_callback);
}

int eu07_application::init_ogl()
{
	if (!Global.gfx_usegles)
	{
		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			ErrorLog("Bad init: failed to initialize glad");
			return -1;
		}
	}
	else
	{
		if (!gladLoadGLES2Loader((GLADloadproc)glfwGetProcAddress))
		{
			ErrorLog("Bad init: failed to initialize glad");
			return -1;
		}
	}

	return 0;
}

int eu07_application::init_ui()
{
	if (false == ui_layer::init(m_windows.front()))
	{
		return -1;
	}
	init_callbacks();
	return 0;
}

int eu07_application::init_gfx()
{

	if (Global.GfxRenderer == "default")
	{
		// default render path
		GfxRenderer = gfx_renderer_factory::get_instance()->create("modern");
	}
	else if (Global.GfxRenderer == "experimental")
	{
		GfxRenderer = gfx_renderer_factory::get_instance()->create(Global.GfxRenderer);
	}
	else
	{
		// legacy render path
		GfxRenderer = gfx_renderer_factory::get_instance()->create("legacy");
		Global.DisabledLogTypes |= static_cast<unsigned int>(logtype::material);
	}

	if (!GfxRenderer)
	{
		ErrorLog("no renderer found!");
		return -1;
	}

	if (false == GfxRenderer->Init(m_windows.front()))
	{
		return -1;
	}

	for (const global_settings::extraviewport_config &conf : Global.extra_viewports)
		if (!GfxRenderer->AddViewport(conf))
			return -1;

	if (!Global.headtrack_conf.joy.empty())
		m_headtrack.emplace();

	return 0;
}

int eu07_application::init_audio()
{

	if (Global.bSoundEnabled)
	{
		Global.bSoundEnabled &= audio::renderer.init();
	}
	// NOTE: lack of audio isn't deemed a failure serious enough to throw in the towel
	return 0;
}

int eu07_application::init_data()
{

	// HACK: grab content of the first {} block in load_unit_weights using temporary parser, then parse it normally. on any error our weight list will be empty string
	auto loadweights{cParser(cParser("data/load_weights.txt", cParser::buffer_FILE).getToken<std::string>(true, "{}"), cParser::buffer_TEXT)};
	while (true == loadweights.getTokens(2))
	{
		std::pair<std::string, float> weightpair;
		loadweights >> weightpair.first >> weightpair.second;
		weightpair.first.erase(weightpair.first.end() - 1); // trim trailing ':' from the key
		simulation::Weights.emplace(weightpair.first, weightpair.second);
	}
	cParser override_parser("data/sound_overrides.txt", cParser::buffer_FILE);
	deserialize_map(simulation::Sound_overrides, override_parser);

	return 0;
}

int eu07_application::init_modes()
{
	Global.local_random_engine.seed(std::random_device{}());

	// activate the default mode
	if (Global.network_client)
	{
		// a multiplayer client learns the scenario name from the server handshake,
		// so it goes straight to the loader and waits there for Global.ready_to_load
		push_mode(mode::scenarioloader);
	}
	else if (Global.SceneryFile.empty())
	{
		// no scenario given: let the user pick one (a listening server simply
		// refuses joins until the scenario is up)
		push_mode(mode::launcher);
	}
	else
	{
		push_mode(mode::scenarioloader);
	}

	return 0;
}

bool eu07_application::init_network()
{
	if (!Global.network_servers.empty() || Global.network_client)
	{
		// create network manager
		m_network.emplace();

		// settings a session cannot run without, whichever side we are on
		network::enforce_session_settings();

		// the host takes part in the session like any other peer, with an identity of its
		// own, so that crew and permission handling needs no special case for it
		if (!Global.network_client)
			Global.network_peer_id = network::PEER_HOST;
	}

	for (auto const &pair : Global.network_servers)
	{
		// create all servers
		WriteLog("net: hosting session on " + pair.second + " (" + pair.first + ")", logtype::net);
		m_network->create_server(pair.first, pair.second);
	}

	if (!Global.local_start_vehicle_override && (Global.network_client || !Global.network_servers.empty()))
	{
		// in a multiplayer session the vehicle is picked in the lobby, so both the host
		// and the joining clients start out as observers rather than in a random cab
		Global.local_start_vehicle = "ghostview";
	}

	if (Global.network_client)
	{

		Global.network_status = "Connecting to " + Global.network_client->second + "...";
		WriteLog("net: connecting to " + Global.network_client->second + " (" + Global.network_client->first + ")", logtype::net);

		// create client
		m_network->connect(Global.network_client->first, Global.network_client->second);
	}
	else
	{
		// we're simulation master
		// TODO: sort out this timezone mess
		std::time_t utc_now = std::time(nullptr);

		tm tm_local, tm_utc;
		tm *tmp = std::localtime(&utc_now);
		memcpy(&tm_local, tmp, sizeof(tm));
		tmp = std::gmtime(&utc_now);
		memcpy(&tm_utc, tmp, sizeof(tm));

		int64_t offset = tm_local.tm_hour * 3600 + tm_local.tm_min * 60 + tm_local.tm_sec - (tm_utc.tm_hour * 3600 + tm_utc.tm_min * 60 + tm_utc.tm_sec);

		Global.starting_timestamp = utc_now + offset;
		Global.ready_to_load = true;
	}

	return true;
}

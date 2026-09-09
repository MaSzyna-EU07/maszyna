module;
#include <ostream>
#include <string>
#include <GLFW/glfw3.h>
#include <memory>
#include "utilities/Globals_macros.h"

module eu07.launcher.launchermode;
import eu07.launcher.launcheruilayer;
import eu07.application.application;
import eu07.simulation.simulation;
import eu07.utilities.globals;

launcher_mode::launcher_mode()
{
	m_userinterface = std::make_shared<launcher_ui>();
}

bool launcher_mode::init()
{
	return true;
}

bool launcher_mode::update()
{
	return true;
}

void launcher_mode::enter()
{
	Application.set_cursor( GLFW_CURSOR_NORMAL );
	simulation::is_ready = false;
	Application.set_title(Global.AppName);
}

void launcher_mode::exit()
{

}

void launcher_mode::on_key(const int Key, const int Scancode, const int Action, const int Mods)
{
#ifndef __unix__
	Global.shiftState = Mods & GLFW_MOD_SHIFT ? true : false;
	Global.ctrlState = Mods & GLFW_MOD_CONTROL ? true : false;
	Global.altState = Mods & GLFW_MOD_ALT ? true : false;
#endif
	m_userinterface->on_key(Key, Action);
}

void launcher_mode::on_window_resize(const int w, const int h)
{
	m_userinterface->on_window_resize(w, h);
}

module;
#include <deque>
#include <unordered_map>
#include <string>
#include "utilities/translation_macros.h"
#include "imgui/imgui.h"

export module eu07.widgets.vehicleparams;
import eu07.global_include.interfaces.itexture;
import eu07.simcore;
import eu07.application.uilayer;
import eu07.utilities.translation;
import eu07.input.command;
import eu07.rendering.renderer;

export {


namespace ui
{
class vehicleparams_panel : public ui_panel
{
	std::string m_vehicle_name;
	command_relay m_relay;
	texture_handle vehicle_mini;

	void draw_infobutton(const char *str, ImVec2 pos = ImVec2(-1.0f, -1.0f), const ImVec4 color = ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
	void draw_mini(const TMoverParameters &mover);
  public:
	vehicleparams_panel(const std::string &vehicle);

	void render_contents() override;
};
} // namespace ui

}  // export

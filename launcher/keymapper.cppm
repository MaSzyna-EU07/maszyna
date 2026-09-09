module;
#include <array>
#include <chrono>
#include <cstddef>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <type_traits>
#include <utility>
#include <vector>
#include <deque>
#include <unordered_map>
#include <string>

export module eu07.launcher.keymapper;
import eu07.input.command;
import eu07.application.uilayer;
import eu07.input.driverkeyboardinput;

export {


namespace ui
{
class keymapper_panel : public ui_panel
{
  public:
	keymapper_panel();

	void render_contents() override;
	bool key(int key);

private:
	driverkeyboard_input keyboard;
	user_command bind_active = user_command::none;
	const std::string panelname = (title.empty() ? m_name : title) + "###" + m_name;
};
} // namespace ui

}  // export

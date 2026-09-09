module;
#include <array>
#include <chrono>
#include <cstddef>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <deque>
#include <unordered_map>
#include "utilities/translation_macros.h"

export module eu07.widgets.time;
import eu07.application.uilayer;
import eu07.utilities.translation;
import eu07.input.command;

export {

namespace ui
{
class time_panel : public ui_panel
{
	command_relay m_relay;

	float time;
	int yearday;
	float fog;
	float overcast;
	float temperature;

  public:
	time_panel();

	void render_contents() override;
	void open();
};
} // namespace ui

}  // export

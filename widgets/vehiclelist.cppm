module;
#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

export module eu07.widgets.vehiclelist;
import eu07.application.uilayer;

export {

namespace ui
{
class vehiclelist_panel : public ui_panel
{
	ui_layer &m_parent;
    bool m_first_show = true;

  public:
	vehiclelist_panel(ui_layer &parent);

	void render_contents() override;
};
} // namespace ui

}  // export

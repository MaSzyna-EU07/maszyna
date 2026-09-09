/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include <array>
#include <string>

export module eu07.launcher.launcheruilayer;
import eu07.application.uilayer;
import eu07.launcher.scenery_list;
import eu07.launcher.keymapper;
import eu07.launcher.vehicle_picker;
import eu07.launcher.textures_scanner;
import eu07.launcher.scenery_scanner;

export {



class launcher_ui : public ui_layer {
public:
	launcher_ui();
	bool on_key(int Key, int Action) override;
	void on_window_resize(int w, int h) override;

private:
	void render_() override;
	void close_panels();
	void open_panel(ui_panel *panel);

	ui::vehicles_bank m_vehicles_bank;
	scenery_scanner m_scenery_scanner;

	ui::scenerylist_panel m_scenerylist_panel;
	ui::keymapper_panel m_keymapper_panel;
	ui::vehiclepicker_panel m_vehiclepicker_panel;

	ui_panel *m_current_panel;
};

}  // export

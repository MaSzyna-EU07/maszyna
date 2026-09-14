module;
#include <array>
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
#include <string>

export module eu07.simcore:editoruilayer;
import :fwd;
import eu07.application.uilayer;
import :editoruipanels;
// the plan tool is a module of its own: it stands on the editor's own libraries, not on the
// simulation core, and the panel is held here by value
export import eu07.application.planpanel;

export {
/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/



namespace scene
{


}

class editor_ui : public ui_layer
{

  public:
	// constructors
	editor_ui();
	// methods
	// updates state of UI elements
	void update() override;
	void set_node(scene::basic_node *Node);
	void add_node_template(const std::string &desc);
	float rot_val();
	bool rot_from_last();
	functions_panel::rotation_mode rot_mode();
	const std::string *get_active_node_template(bool bypassRandom = false);
	nodebank_panel::edit_mode mode();
	float getSpacing();
	void toggleBrushSettings(bool isVisible);
	// whether the plan tool has the keyboard and so owns Ctrl+Z / Ctrl+Y. the scene editor listens
	// for the same keys, and cofniecie ma dotyczyc tego, na co uzytkownik patrzy
	bool plan_takes_history() const;

  protected:
	void render_menu_contents() override;

  private:
	// members
	itemproperties_panel m_itempropertiespanel{"Node Properties", true};
	functions_panel m_functionspanel{"Functions", true};
	nodebank_panel m_nodebankpanel{"Node Bank", true};
	brush_object_list m_brushobjects{"Brush properties", false};
	plan_panel m_planpanel{"Plan", false};
	scene::basic_node *m_node{nullptr}; // currently bound scene node, if any
};

}  // export

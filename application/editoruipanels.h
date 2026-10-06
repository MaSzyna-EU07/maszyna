/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/uilayer.h"
#include "utilities/Classes.h"
#include "editor/editorIncludeInfo.hpp"

#include <functional>
/*
// helper, associated bool is set when the primary value was changed and expects processing at the observer's leisure
template<typename Type_>
using changeable = std::pair<Type_, bool>;

// helper, holds a set of changeable properties for a scene node
struct item_properties {

    scene::basic_node const *node { nullptr }; // properties' owner

    changeable<std::string> name {};
    changeable<glm::dvec3> location {};
    changeable<glm::vec3> rotation {};
};
*/
// include of a scenery template selected in the editor, with the values its directive passes to the template.
// owned by the editor mode; the node properties let the user change the values
struct include_selection
{
	scene::instance_handle instance{0};
	std::string target; // name of the template, the way the directive spells it
	include_info info; // description of the template
	std::vector<std::string> values; // value of each parameter in turn
	bool placement{false}; // the position and rotation parameters can be driven by the gizmo
	std::string issue; // why the values can't be changed, if they can't
	bool changed{false}; // set by the UI when a value was changed, cleared by the editor mode once the change is applied
	bool active{false}; // the UI is in the middle of changing a value
};

class itemproperties_panel : public ui_panel
{

  public:
	itemproperties_panel(std::string const &Name, bool const Isopen) : ui_panel(Name, Isopen) {}

	// non-const node pointer so the transform editor in render() can mutate
	// position/rotation/scale of TAnimModel nodes in place. Other node types
	// are still treated as read-only.
	void update(scene::basic_node *Node);
	// binds include of a scenery template to the panel, shown in place of a node; nullptr unbinds
	void include(include_selection *Include)
	{
		m_include = Include;
	}
	void render() override;
	// panel content without the window, also drawn inside the toolset window
	void render_body();

  private:
	// methods
	void update_group();
	bool render_group();
	// renders DragFloat3/DragScalarN widgets for position, rotation and scale
	// of the currently bound TAnimModel; no-op for other node subclasses.
	void render_transform_editor();
	// renders the field with the name of the currently bound TAnimModel, which gives the instance another name; no-op for other node subclasses
	void render_name_editor();
	// renders widgets for the parameters of the bound include
	void render_include();

	// members
	scene::basic_node *m_node{nullptr}; // scene node bound to the panel
	scene::basic_node *m_namednode{nullptr}; // scene node whose name is held by the edit buffer
	char m_namebuffer[128]{}; // name of the node, as shown and typed in
	bool m_nameactive{false}; // the name is being typed in
	std::string m_nameissue; // why the name typed in wasn't taken
	include_selection *m_include{nullptr}; // include of a scenery template bound to the panel
	scene::group_handle m_grouphandle{null_handle}; // scene group bound to the panel
	std::string m_groupprefix;
	std::vector<text_line> m_grouplines;
};

// reference to a set of node templates, used by the random insert, the brush and the area fill
struct model_set_ref
{
	enum class source
	{
		manual, // list assembled by hand in the tool itself (or no set, where the tool has no such list)
		user, // user-defined set (see EditorModelSets), id == set id
		nodebank // node bank group, id == group index (as returned by nodebank_panel::group_names())
	};
	source kind{source::manual};
	int id{-1};
};

class nodebank_panel;

// brush settings, drawn inside the node bank window when the brush mode is active
class brush_object_list
{
  private:
	int idx{-1};

  public:
	void render_options(nodebank_panel &Bank);

	// class use
	std::vector<std::string> Objects; // manual list, used when source is set to manual
	std::string *GetRandomObject();
	bool useRandom = {false};
	model_set_ref source;
	float spacing{1.0f};
};

class nodebank_panel : public ui_panel
{

  public:
	enum edit_mode
	{
		MODIFY,
		COPY,
		ADD,
		BRUSH,
		FILL,
		TRACK
	};
	edit_mode mode = MODIFY;
	int requested_mode{-1}; // tab brought forward on the next frame

	nodebank_panel(std::string const &Name, bool const Isopen);
	void nodebank_reload();
	void render() override;
	void add_template(const std::string &desc);
	const std::string *get_active_template();
	// nodebank groups (collapsing headers), in file order; entries listed before the first header form an unnamed group
	std::vector<std::string> group_names() const;
	// node templates of the group with specified index (as returned by group_names())
	std::vector<std::string> group_templates(std::size_t const Group) const;

	// model sets (user-defined sets and node bank groups)
	// combo selecting a set. Manuallabel names the tool's own hand-made list; nullptr hides that choice. returns: true on change
	bool set_combo(char const *Label, model_set_ref &Ref, char const *Manuallabel);
	// list of templates assembled by hand, with buttons to add the node bank selection, remove entries and save it as a user set
	void manual_list(char const *Id, std::vector<std::string> &List, int &Selected);
	// node templates of referenced set; empty for the manual list or a set which no longer exists
	std::vector<std::string const *> set_entries(model_set_ref const &Ref) const;
	// random template from referenced set, or nullptr if the set is empty
	std::string const *random_template(model_set_ref const &Ref) const;
	std::string set_name(model_set_ref const &Ref, char const *Manuallabel) const;
	// shows the set manager window, optionally with specified user set selected
	void open_sets_window(int const Setid = 0);

	// draws settings of the active edit mode inside its tab
	std::function<void(edit_mode)> mode_options;
	// draws sections attached above the mode tabs (gizmo, node properties)
	std::function<void()> header_sections;

  private:
	// methods:
	std::string generate_node_label(std::string Input) const;
	void render_sets_window();
	// true if the node bank selection is a definition of a node, which the hand-made lists and the model sets can take
	bool node_selected() const;
	// members:
	std::vector<std::pair<std::string, std::shared_ptr<std::string>>> m_nodebank;
	char m_nodesearch[128];
	std::shared_ptr<std::string> m_selectedtemplate;
	// set manager window
	bool m_setsopen{false};
	int m_setsselected{0}; // id of the user set being edited
	int m_setsentry{-1}; // selected template of the edited set
	int m_setsnameid{0}; // id of the set whose name is held in the edit buffer
	char m_setsname[128]{};
	model_set_ref m_setsgroup{model_set_ref::source::nodebank, 0}; // node bank group to append to the edited set
};

// scenery layers of a scenery opened for editing: their visibility, selection lock, the layer receiving new nodes,
// and layer management. the changes are applied to the scenery files on save
class layers_panel : public ui_panel
{

  public:
	layers_panel(std::string const &Name, bool const Isopen) : ui_panel(Name, Isopen) {}

	void render() override;

	// saves the scenery; provided by the editor mode
	std::function<void()> save;
	// outcome of the most recent operation, shown at the bottom of the window
	std::string status;
	bool status_error{false};

  private:
	// methods
	void render_list();
	void render_popups();
	void report(std::string const &Status, bool const Error = false);
	// members
	scene::layer_handle m_selected{null_handle}; // layer the buttons act on
	char m_newname[128]{}; // file name of the layer being created
	scene::layer_handle m_newparent{null_handle}; // layer which is going to include it
	scene::layer_handle m_mergetarget{null_handle};
	std::string m_popuperror;
};

// database of the *.inc scenery templates: the templates found in the scenery directory, with the editor of their
// descriptions (see editorIncludeInfo.hpp). templates whose descriptions are complete are offered by the node bank
class includes_panel : public ui_panel
{

  public:
	includes_panel(std::string const &Name, bool const Isopen) : ui_panel(Name, Isopen) {}

	void render() override;

  private:
	// methods
	void open(std::string const &File);
	void render_list();
	void render_description();
	// members
	// template list
	char m_filter[128]{};
	bool m_describedonly{false};
	bool m_usedonly{false};
	std::vector<std::size_t> m_listed; // entries of the template bank matching the filter
	int m_listrevision{-1}; // revision of the template bank the list was made for
	int m_described{0};
	// description editor
	std::string m_file; // template being edited, relative to the scenery directory
	include_info m_info;
	int m_parameters{0}; // number of parameters the template uses
	bool m_loaded{false};
	std::string m_status;
	bool m_statuserror{false};
};

class functions_panel : public ui_panel
{

  public:
	enum rotation_mode
	{
		RANDOM,
		FIXED,
		DEFAULT
	};
	rotation_mode rot_mode = DEFAULT;

	float rot_value = 0.0f;
	bool rot_from_last = false;

	functions_panel(std::string const &Name, bool const Isopen) : ui_panel(Name, Isopen) {}

	void update(scene::basic_node const *Node);
	void render() override;
	// rotation settings widgets, also drawn inside the node bank window
	void render_controls();

  private:
	// methods

	// members
	scene::basic_node const *m_node{nullptr}; // scene node bound to the panel
	scene::group_handle m_grouphandle{null_handle}; // scene group bound to the panel
	std::string m_groupprefix;
	std::vector<text_line> m_grouplines;
};
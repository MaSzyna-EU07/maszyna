/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editoruipanels.h"
#include "scene/scenenodegroups.h"
#include "scene/scenelayers.h"

#include "utilities/Globals.h"
#include "vehicle/Camera.h"
#include "model/AnimModel.h"
#include "world/Track.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "application/editoruilayer.h"
#include "rendering/renderer.h"
#include "simulation/simulation.h"
#include "editor/editorModelSets.hpp"

namespace
{
// tells why provided text can't be the name of a node, empty text if it can. the scenery parser has to get it back as a single token
std::string node_name_issue(std::string const &Name)
{
	if (Name.find_first_of(" \t\r\n;\"") != std::string::npos)
	{
		return "a name can't hold spaces, semicolons or quotes";
	}
	if (Name.find("//") != std::string::npos || Name.find("/*") != std::string::npos)
	{
		return "a name can't hold a comment mark";
	}
	if (Name == "include")
	{
		return "this word has a meaning of its own in scenery files";
	}
	return {};
}

// list entry; the selected one is drawn with the accent colour, like selections in the starter
bool list_item(char const *Label, bool const Selected)
{
	if (Selected)
		ImGui::PushStyleColor(ImGuiCol_Header, ImGui::GetStyleColorVec4(ImGuiCol_HeaderActive));
	auto const clicked{ImGui::Selectable(Label, Selected)};
	if (Selected)
		ImGui::PopStyleColor();
	return clicked;
}
} // namespace

void itemproperties_panel::update(scene::basic_node *Node)
{
	m_node = Node;

	if (false == is_open)
	{
		return;
	}

	text_lines.clear();
	m_grouplines.clear();

	std::string textline;

	// scenario inspector
	auto const *node{Node};
	auto const &camera{Global.pCamera};

	if (node == nullptr)
	{
		auto const mouseposition{camera.Pos + GfxRenderer->Mouse_Position()};
		textline = "mouse location: [" + to_string(mouseposition.x, 2) + ", " + to_string(mouseposition.y, 2) + ", " + to_string(mouseposition.z, 2) + "]";
		text_lines.emplace_back(textline, Global.UITextColor);
		return;
	}
	/*
	    // TODO: bind receiver in the constructor
	    if( ( m_itemproperties != nullptr )
	     && ( m_itemproperties->node != nullptr ) )  {
	        // fetch node data; skip properties which were changed until they're retrieved by the observer
	        auto const *node { m_itemproperties->node };

	        if( m_itemproperties->name.second == false ) {
	            m_itemproperties->name.first = ( node->name().empty() ? "(none)" : node->name() );
	        }
	        if( m_itemproperties->location.second == false ) {
	            m_itemproperties->location.first = node->location();
	        }
	    }
	*/
	textline = "name: " + (node->name().empty() ? "(none)" : Bezogonkow(node->name())) + "\ntype: " + node->node_type + "\nlocation: [" + to_string(node->location().x, 2) + ", " + to_string(node->location().y, 2) + ", " +
	           to_string(node->location().z, 2) + "]" +
	           " (distance: " + to_string(glm::length(glm::dvec3{node->location().x, 0.0, node->location().z} - glm::dvec3{camera.Pos.x, 0.0, camera.Pos.z}), 1) + " m)" + "\nUUID: " + node->uuid.to_string();
	if (scene::Layers.valid(node->layer()))
	{
		textline += "\nlayer: " + Bezogonkow(scene::Layers.layer(node->layer()).name);
	}
	text_lines.emplace_back(textline, Global.UITextColor);

	// subclass-specific data
	// TBD, TODO: specialized data dump method in each node subclass, or data imports in the panel for provided subclass pointer?
	if (typeid(*node) == typeid(TAnimModel))
	{

		auto const *subnode = static_cast<TAnimModel const *>(node);

		textline = "angle_x: " + to_string(clamp_circular(subnode->vAngle.x, 360.f), 2) + " deg, " + "angle_y: " + to_string(clamp_circular(subnode->vAngle.y, 360.f), 2) + " deg, " +
		           "angle_z: " + to_string(clamp_circular(subnode->vAngle.z, 360.f), 2) + " deg";
		textline += ";\nlights: ";
		if (subnode->iNumLights > 0)
		{
			textline += '[';
			for (int lightidx = 0; lightidx < subnode->iNumLights; ++lightidx)
			{
				textline += std::to_string(subnode->lsLights[lightidx]);
				if (lightidx < subnode->iNumLights - 1)
				{
					textline += ", ";
				}
			}
			textline += ']';
		}
		else
		{
			textline += "(none)";
		}
		text_lines.emplace_back(textline, Global.UITextColor);

		// 3d shape
		auto modelfile{(subnode->pModel != nullptr ? subnode->pModel->NameGet() : "(none)")};
		if (modelfile.find(paths::models) == 0)
		{
			// don't include 'models/' in the path
			modelfile.erase(0, std::string{paths::models}.size());
		}
		// texture
		auto texturefile{(subnode->Material()->replacable_skins[1] != null_handle ? GfxRenderer->Material(subnode->Material()->replacable_skins[1])->GetName() : "(none)")};
		if (texturefile.find(paths::textures) == 0)
		{
			// don't include 'textures/' in the path
			texturefile.erase(0, std::string{paths::textures}.size());
		}
		text_lines.emplace_back("mesh: " + modelfile, Global.UITextColor);
		text_lines.emplace_back("skin: " + texturefile, Global.UITextColor);
	}
	else if (typeid(*node) == typeid(TTrack))
	{

		auto const *subnode = static_cast<TTrack const *>(node);

		std::string isolatedlist;
		for (const TIsolated *iso : subnode->Isolated)
		{
			if (!isolatedlist.empty())
				isolatedlist += ", ";
			isolatedlist += iso->asName;
		}

		// basic attributes
		textline = "isolated: " + (!isolatedlist.empty() ? isolatedlist : "(none)") + "\nvelocity: " + std::to_string(subnode->SwitchExtension ? subnode->SwitchExtension->fVelocity : subnode->fVelocity) +
		           "\nwidth: " + std::to_string(subnode->fTrackWidth) + " m" + "\nfriction: " + to_string(subnode->fFriction, 2) + "\nquality: " + std::to_string(subnode->iQualityFlag);
		text_lines.emplace_back(textline, Global.UITextColor);
		// textures
		auto texturefile{(subnode->m_material1 != null_handle ? GfxRenderer->Material(subnode->m_material1)->GetName() : "(none)")};
		if (texturefile.find(paths::textures) == 0)
		{
			texturefile.erase(0, std::string{paths::textures}.size());
		}
		auto texturefile2{(subnode->m_material2 != null_handle ? GfxRenderer->Material(subnode->m_material2)->GetName() : "(none)")};
		if (texturefile2.find(paths::textures) == 0)
		{
			texturefile2.erase(0, std::string{paths::textures}.size());
		}
		textline = "skins:\n " + texturefile + "\n " + texturefile2;
		text_lines.emplace_back(textline, Global.UITextColor);
		// paths
		textline = "paths: ";
		for (auto const &path : subnode->m_paths)
		{
			textline += "\n [" + to_string(path.points[segment_data::point::start].x, 3) + ", " + to_string(path.points[segment_data::point::start].y, 3) + ", " +
			            to_string(path.points[segment_data::point::start].z, 3) + "]->" + " [" + to_string(path.points[segment_data::point::end].x, 3) + ", " +
			            to_string(path.points[segment_data::point::end].y, 3) + ", " + to_string(path.points[segment_data::point::end].z, 3) + "] ";
		}
		text_lines.emplace_back(textline, Global.UITextColor);
		// events
		textline.clear();

		std::vector<std::pair<std::string, TTrack::event_sequence const *>> const eventsequences{{"ev0", &subnode->m_events0}, {"ev0all", &subnode->m_events0all},
		                                                                                         {"ev1", &subnode->m_events1}, {"ev1all", &subnode->m_events1all},
		                                                                                         {"ev2", &subnode->m_events2}, {"ev2all", &subnode->m_events2all}};

		for (auto const &eventsequence : eventsequences)
		{

			if (eventsequence.second->empty())
			{
				continue;
			}

			textline += (textline.empty() ? "" : "\n") + eventsequence.first + ": [";
			for (auto const &event : *eventsequence.second)
			{
				if (textline.back() != '[')
				{
					textline += ", ";
				}
				textline += event.second != nullptr ? Bezogonkow(event.second->m_name) : event.first + " (missing)";
			}
			textline += "] ";
		}
		text_lines.emplace_back(textline, Global.UITextColor);
	}
	else if (typeid(*node) == typeid(TMemCell))
	{

		auto const *subnode = static_cast<TMemCell const *>(node);

		textline = "data: [" + subnode->Text() + "]" + " [" + to_string(subnode->Value1(), 2) + "]" + " [" + to_string(subnode->Value2(), 2) + "]";
		text_lines.emplace_back(textline, Global.UITextColor);
		textline = "track: " + (subnode->asTrackName.empty() ? "(none)" : Bezogonkow(subnode->asTrackName));
		text_lines.emplace_back(textline, Global.UITextColor);
	}

	update_group();
}

void itemproperties_panel::update_group()
{

	auto const grouphandle{m_node->group()};

	if (grouphandle == null_handle)
	{
		m_grouphandle = null_handle;
		m_groupprefix.clear();
		return;
	}

	auto const &nodegroup{scene::Groups.group(grouphandle)};

	if (m_grouphandle != grouphandle)
	{
		// calculate group name from shared prefix of item names
		std::vector<std::reference_wrapper<std::string const>> names;
		// build list of custom item and event names
		for (auto const *node : nodegroup.nodes)
		{
			auto const &name{node->name()};
			if (name.empty() || name == "none")
			{
				continue;
			}
			names.emplace_back(name);
		}
		for (auto const *event : nodegroup.events)
		{
			auto const &name{event->m_name};
			if (name.empty() || name == "none")
			{
				continue;
			}
			names.emplace_back(name);
		}
		// find the common prefix
		if (names.size() > 1)
		{
			m_groupprefix = names.front();
			for (auto const &name : names)
			{
				// NOTE: first calculation runs over two instances of the same name, but, eh
				auto const prefixlength{len_common_prefix(m_groupprefix, name.get())};
				if (prefixlength > 0)
				{
					m_groupprefix = m_groupprefix.substr(0, prefixlength);
				}
				else
				{
					m_groupprefix.clear();
					break;
				}
			}
		}
		else
		{
			// less than two names to compare means no prefix
			m_groupprefix.clear();
		}
		m_grouphandle = grouphandle;
	}

	m_grouplines.emplace_back("nodes: " + std::to_string(nodegroup.nodes.size()) + "\nevents: " + std::to_string(nodegroup.events.size()), Global.UITextColor);
	m_grouplines.emplace_back("names prefix: " + (m_groupprefix.empty() ? "(none)" : m_groupprefix), Global.UITextColor);
}

void itemproperties_panel::render()
{

	if (false == is_open)
	{
		return;
	}
	if (true == text_lines.empty())
	{
		return;
	}

	auto flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoCollapse | (size.x > 0 ? ImGuiWindowFlags_NoResize : 0);

	if (size.x > 0)
	{
		ImGui::SetNextWindowSize(ImVec2S(size.x, size.y));
	}
	if (size_min.x > 0)
	{
		ImGui::SetNextWindowSizeConstraints(ImVec2S(size_min.x, size_min.y), ImVec2(size_max.x, size_max.y));
	}
	auto const panelname{(title.empty() ? m_name : title) + "###" + m_name};
	if (true == ImGui::Begin(panelname.c_str(), nullptr, flags))
	{
		render_body();
	}
	ImGui::End();
}

void itemproperties_panel::render_body()
{
	if (m_include != nullptr)
	{
		render_include();
		return;
	}
	// header section
	for (auto const &line : text_lines)
	{
		ImGui::TextColored(ImVec4(line.color.r, line.color.g, line.color.b, line.color.a), line.data.c_str());
	}
	// name of the node — TAnimModel only
	render_name_editor();
	// transform editor (position/rotation/scale) — TAnimModel only
	render_transform_editor();
	// group section
	render_group();
}

// Name of a picked TAnimModel. The editor places decorations without names, as each name takes its share of the time
// the scenery needs to load; this is where an instance gets one, for the events to refer to it.
void itemproperties_panel::render_name_editor()
{
	if (m_node == nullptr) { return; }
	if (typeid(*m_node) != typeid(TAnimModel)) { return; }
	auto *picked = static_cast<TAnimModel *>(m_node);
	// the name of an instance defined by a template, or by a file which can't be rewritten, wouldn't make it to the scenery files
	if (false == scene::Layers.editable(picked)) { return; }

	if (m_namednode != m_node || false == m_nameactive)
	{
		// the field shows the name of the node unless it's being typed in
		if (m_namednode != m_node)
		{
			m_nameissue.clear();
		}
		m_namednode = m_node;
		std::snprintf(m_namebuffer, sizeof(m_namebuffer), "%s", picked->name().c_str());
	}
	auto const entered{ImGui::InputTextWithHint("name", "(none)", m_namebuffer, IM_ARRAYSIZE(m_namebuffer), ImGuiInputTextFlags_EnterReturnsTrue)};
	m_nameactive = ImGui::IsItemActive();
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip("Name the events can refer to the node by.\nNodes without a name load quicker, leave it empty unless something needs it");
	}
	if (entered || ImGui::IsItemDeactivatedAfterEdit())
	{
		// names are lower case, like everything else the scenery parser hands over
		std::string name{m_namebuffer};
		for (auto &character : name)
		{
			if (character >= 'A' && character <= 'Z')
			{
				character = static_cast<char>(character - 'A' + 'a');
			}
		}
		if (name == "none")
		{
			name.clear();
		}
		if (name != picked->name())
		{
			m_nameissue = node_name_issue(name);
			if (m_nameissue.empty() && false == simulation::State.rename_model(picked, name))
			{
				m_nameissue = "another model instance goes by this name";
			}
		}
		else
		{
			m_nameissue.clear();
		}
		m_nameactive = false;
	}
	if (false == m_nameissue.empty())
	{
		ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s", m_nameissue.c_str());
	}
}

// In-place editor for position (double precision), rotation (degrees, 0-360),
// and uniform scale (per-axis float, 1.000) of a picked TAnimModel.
// Other node subclasses don't expose these knobs through the same API, so the
// editor short-circuits when the bound node isn't a TAnimModel.
void itemproperties_panel::render_transform_editor()
{
	if (m_node == nullptr) { return; }
	if (typeid(*m_node) != typeid(TAnimModel)) { return; }
	auto *picked = static_cast<TAnimModel *>(m_node);

	if (false == ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen))
	{
		return;
	}

	// Position — full double precision via DragScalarN. World coordinates can grow
	// large; %.6f gives sub-millimetre resolution at typical scenario distances.
	{
		glm::dvec3 location = picked->location();
		double pos[3] = { location.x, location.y, location.z };
		if (ImGui::DragScalarN("position", ImGuiDataType_Double, pos, 3, 0.05f, nullptr, nullptr, "%.6f"))
		{
			picked->location(glm::dvec3(pos[0], pos[1], pos[2]));
		}
	}

	// Rotation — wrapped into [0,360) for display; slider clamps drags to that range.
	{
		glm::vec3 angles{
			clamp_circular(picked->Angles().x),
			clamp_circular(picked->Angles().y),
			clamp_circular(picked->Angles().z)};
		if (ImGui::DragFloat3("rotation (deg)", &angles.x, 0.5f, 0.0f, 360.0f, "%.3f"))
		{
			picked->Angles(angles);
		}
	}

	// Scale — per-axis float, 1.000 display. Clamped to a reasonable positive range.
	{
		glm::vec3 scale = picked->Scale();
		if (ImGui::DragFloat3("scale (x,y,z)", &scale.x, 0.01f, 0.001f, 100.0f, "%.3f"))
		{
			picked->Scale(scale);
		}
	}

	if (ImGui::Button("reset rotation")) { picked->Angles(glm::vec3(0.0f)); }
	ImGui::SameLine();
	if (ImGui::Button("reset scale")) { picked->Scale(glm::vec3(1.0f)); }
}

bool itemproperties_panel::render_group()
{

	if (m_node == nullptr)
	{
		return false;
	}
	if (m_grouplines.empty())
	{
		return false;
	}

	if (false == ImGui::CollapsingHeader("Parent Group"))
	{
		return false;
	}

	for (auto const &line : m_grouplines)
	{
		ImGui::TextColored(ImVec4(line.color.r, line.color.g, line.color.b, line.color.a), line.data.c_str());
	}

	return true;
}

std::string *brush_object_list::GetRandomObject()
{
	static std::string empty; // fallback

	if (Objects.empty())
		return &empty;

	std::uniform_int_distribution<size_t> dist(0, Objects.size() - 1);

	return &Objects[dist(Global.local_random_engine)];
}

void brush_object_list::render_options(nodebank_panel &Bank)
{
	ImGui::SliderFloat("Spacing", &spacing, 0.1f, 20.0f, "%.1f m");
	ImGui::Checkbox("Random model from set", &useRandom);
	if (false == useRandom)
	{
		ImGui::TextDisabled("Paints the template selected in the node bank");
		return;
	}
	Bank.set_combo("Preset", source, "Manual list");
	if (source.kind == model_set_ref::source::manual)
	{
		Bank.manual_list("brushset", Objects, idx);
	}
	else
	{
		auto const count{Bank.set_entries(source).size()};
		ImGui::TextDisabled("%zu templates in set%s", count, count == 0 ? ", the node bank selection is used" : "");
	}
}

nodebank_panel::nodebank_panel(std::string const &Name, bool const Isopen) : ui_panel(Name, Isopen)
{
	size_min = {100, 50};
	size_max = {1000, 1000};

	memset(m_nodesearch, 0, sizeof(m_nodesearch));

	std::ifstream file;
	file.open("nodebank.txt", std::ios_base::in | std::ios_base::binary);

	std::string line;
	while (std::getline(file, line))
	{
		if (line.size() < 4)
		{
			continue;
		}
		auto const labelend{line.find("node")};
		auto const nodedata{(labelend == std::string::npos ? "" : labelend == 0 ? line : line.substr(labelend))};
		auto const label{(labelend == std::string::npos ? line : labelend == 0 ? generate_node_label(nodedata) : line.substr(0, labelend))};

		m_nodebank.push_back({label, std::make_shared<std::string>(nodedata)});
	}
	// sort alphabetically content of each group
	auto groupbegin{m_nodebank.begin()};
	auto groupend{groupbegin};
	while (groupbegin != m_nodebank.end())
	{
		groupbegin = std::find_if(groupend, m_nodebank.end(), [](auto const &Entry) { return false == Entry.second->empty(); });
		groupend = std::find_if(groupbegin, m_nodebank.end(), [](auto const &Entry) { return Entry.second->empty(); });
		std::sort(groupbegin, groupend, [](auto const &Left, auto const &Right) { return Left.first < Right.first; });
	}
}
void nodebank_panel::nodebank_reload()
{
	m_nodebank.clear();
	std::ifstream file;
	file.open("nodebank.txt", std::ios_base::in | std::ios_base::binary);
	std::string line;
	while (std::getline(file, line))
	{
		if (line.size() < 4)
		{
			continue;
		}
		auto const labelend{line.find("node")};
		auto const nodedata{(labelend == std::string::npos ? "" : labelend == 0 ? line : line.substr(labelend))};
		auto const label{(labelend == std::string::npos ? line : labelend == 0 ? generate_node_label(nodedata) : line.substr(0, labelend))};

		m_nodebank.push_back({label, std::make_shared<std::string>(nodedata)});
	}
	// sort alphabetically content of each group
	auto groupbegin{m_nodebank.begin()};
	auto groupend{groupbegin};
	while (groupbegin != m_nodebank.end())
	{
		groupbegin = std::find_if(groupend, m_nodebank.end(), [](auto const &Entry) { return false == Entry.second->empty(); });
		groupend = std::find_if(groupbegin, m_nodebank.end(), [](auto const &Entry) { return Entry.second->empty(); });
		std::sort(groupbegin, groupend, [](auto const &Left, auto const &Right) { return Left.first < Right.first; });
	}
}

void nodebank_panel::render()
{
	if (false == is_open)
	{
		return;
	}

	auto flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoCollapse | (size.x > 0 ? ImGuiWindowFlags_NoResize : 0);

	if (size.x > 0)
	{
		ImGui::SetNextWindowSize(ImVec2S(size.x, size.y));
	}
	else
	{
		// mode settings are drawn above the list, so start with a window tall enough for both
		ImGui::SetNextWindowSize(ImVec2S(440, 640), ImGuiCond_FirstUseEver);
	}
	if (size_min.x > 0)
	{
		ImGui::SetNextWindowSizeConstraints(ImVec2S(size_min.x, size_min.y), ImVec2S(size_max.x, size_max.y));
	}
	auto const panelname{(title.empty() ? name() : title) + "###" + name()};

	if (true == ImGui::Begin(panelname.c_str(), nullptr, flags))
	{
		if (ImGui::Button("Reload node bank"))
		{
			nodebank_reload();
		}
		ImGui::SameLine();
		if (ImGui::Button(m_setsopen ? "Close model sets" : "Model sets..."))
		{
			m_setsopen = !m_setsopen;
		}

		if (header_sections)
		{
			header_sections();
		}

		// edit modes as tabs, each with its own settings
		std::pair<char const *, edit_mode> const modes[] = {{"Select", MODIFY}, {"Insert", ADD}, {"Brush", BRUSH}, {"Area fill", FILL}, {"Copy to bank", COPY}};
		if (ImGui::BeginTabBar("##editmodes", ImGuiTabBarFlags_FittingPolicyResizeDown))
		{
			for (auto const &tab : modes)
			{
				if (false == ImGui::BeginTabItem(tab.first, nullptr, requested_mode == tab.second ? ImGuiTabItemFlags_SetSelected : 0))
				{
					continue;
				}
				mode = tab.second;
				if (mode_options)
				{
					mode_options(mode);
				}
				ImGui::EndTabItem();
			}
			ImGui::EndTabBar();
			requested_mode = -1;
		}

		ImGui::Separator();
		ImGui::PushItemWidth(-1);
		ImGui::InputTextWithHint("Search", "Search node bank", m_nodesearch, IM_ARRAYSIZE(m_nodesearch));
		// the list takes the rest of the window, but keeps a usable height when the sections above are expanded (the window scrolls then)
		auto const listheight{std::max(ImGui::GetContentRegionAvail().y, ImGui::GetTextLineHeightWithSpacing() * 10.0f)};
		if (ImGui::ListBoxHeader("##nodebank", ImVec2(-1, listheight)))
		{
			auto idx{0};
			auto isvisible{false};
			auto const searchfilter{std::string(m_nodesearch)};
			for (auto const &entry : m_nodebank)
			{
				if (entry.second->empty())
				{
					// special case, header indicator
					isvisible = ImGui::CollapsingHeader(entry.first.c_str());
				}
				else
				{
					if (false == isvisible)
					{
						continue;
					}
					if (false == searchfilter.empty() && false == contains(entry.first, searchfilter))
					{
						continue;
					}
					auto const label{" " + entry.first + "##" + std::to_string(idx)};
					if (list_item(label.c_str(), entry.second == m_selectedtemplate))
						m_selectedtemplate = entry.second;
					++idx;
				}
			}
			// scenery templates. placing one adds an include directive to a scenery file, so they take scenery opened for editing
			if (Global.editor_session && ImGui::CollapsingHeader("Scenery templates (.inc)"))
			{
				if (false == EditorIncludes.scanned())
				{
					EditorIncludes.scan();
				}
				auto const &templates{EditorIncludes.entries()};
				std::string const *category{nullptr};
				auto listed{0};
				for (auto const entryidx : EditorIncludes.ready())
				{
					auto const &entry{templates[entryidx]};
					if (false == searchfilter.empty() && false == contains(entry.name, searchfilter) && false == contains(entry.file, searchfilter) && false == contains(entry.category, searchfilter))
					{
						continue;
					}
					if (false == entry.category.empty() && (category == nullptr || *category != entry.category))
					{
						ImGui::TextDisabled(" %s", entry.category.c_str());
					}
					category = &entry.category;
					auto const label{(entry.category.empty() ? " " : "   ") + entry.name + "  (" + Bezogonkow(entry.file) + ")##inc" + std::to_string(entryidx)};
					if (list_item(label.c_str(), entry.statement == m_selectedtemplate))
						m_selectedtemplate = entry.statement;
					++listed;
				}
				if (listed == 0)
				{
					ImGui::TextDisabled(EditorIncludes.ready().empty() ? " (none yet, describe the templates in the Include database window)" : " (no match)");
				}
			}
			ImGui::ListBoxFooter();
		}
		ImGui::PopItemWidth();
	}

	ImGui::End();

	if (m_setsopen)
	{
		render_sets_window();
	}
}

bool nodebank_panel::set_combo(char const *Label, model_set_ref &Ref, char const *Manuallabel)
{
	auto changed{false};
	auto const preview{set_name(Ref, Manuallabel)};
	if (false == ImGui::BeginCombo(Label, preview.c_str()))
	{
		return false;
	}
	if (Manuallabel != nullptr)
	{
		if (ImGui::Selectable(Manuallabel, Ref.kind == model_set_ref::source::manual))
		{
			Ref = model_set_ref{};
			changed = true;
		}
	}
	ImGui::TextDisabled("User sets");
	auto const &sets{EditorModelSets.sets()};
	if (sets.empty())
	{
		ImGui::TextDisabled("  (none yet, see Model sets...)");
	}
	for (auto const &set : sets)
	{
		auto const label{"  " + set.name + " (" + std::to_string(set.templates.size()) + ")##userset" + std::to_string(set.id)};
		if (ImGui::Selectable(label.c_str(), Ref.kind == model_set_ref::source::user && Ref.id == set.id))
		{
			Ref = {model_set_ref::source::user, set.id};
			changed = true;
		}
	}
	ImGui::TextDisabled("Node bank groups");
	auto const groups{group_names()};
	for (int idx = 0; idx < static_cast<int>(groups.size()); ++idx)
	{
		auto const label{"  " + groups[idx] + "##bankgroup" + std::to_string(idx)};
		if (ImGui::Selectable(label.c_str(), Ref.kind == model_set_ref::source::nodebank && Ref.id == idx))
		{
			Ref = {model_set_ref::source::nodebank, idx};
			changed = true;
		}
	}
	ImGui::EndCombo();
	return changed;
}

void nodebank_panel::manual_list(char const *Id, std::vector<std::string> &List, int &Selected)
{
	ImGui::PushID(Id);
	if (ImGui::ListBoxHeader("##list", ImVec2(-1, ImGui::GetTextLineHeightWithSpacing() * 6.5f)))
	{
		for (int idx = 0; idx < static_cast<int>(List.size()); ++idx)
		{
			auto const label{generate_node_label(List[idx]) + "##" + std::to_string(idx)};
			if (list_item(label.c_str(), Selected == idx))
				Selected = idx;
		}
		if (List.empty())
		{
			ImGui::TextDisabled("(empty)");
		}
		ImGui::ListBoxFooter();
	}
	if (ImGui::Button("Add selected"))
	{
		if (node_selected())
			List.push_back(*m_selectedtemplate);
	}
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip("Adds the template selected in the node bank list");
	}
	ImGui::SameLine();
	if (ImGui::Button("Remove") && Selected >= 0 && Selected < static_cast<int>(List.size()))
	{
		List.erase(List.begin() + Selected);
		Selected = -1;
	}
	ImGui::SameLine();
	if (ImGui::Button("Clear"))
	{
		List.clear();
		Selected = -1;
	}
	ImGui::SameLine();
	if (ImGui::Button("Save as set") && false == List.empty())
	{
		open_sets_window(EditorModelSets.create("New set", List));
	}
	ImGui::PopID();
}

std::vector<std::string const *> nodebank_panel::set_entries(model_set_ref const &Ref) const
{
	std::vector<std::string const *> entries;
	if (Ref.kind == model_set_ref::source::user)
	{
		if (auto const *set = EditorModelSets.find(Ref.id))
		{
			for (auto const &entry : set->templates)
				entries.push_back(&entry);
		}
	}
	else if (Ref.kind == model_set_ref::source::nodebank && Ref.id >= 0)
	{
		// same grouping as group_templates(), without copying the templates
		int group{0};
		bool started{false};
		for (auto const &entry : m_nodebank)
		{
			if (entry.second->empty())
			{
				if (started)
					++group;
				started = true;
				continue;
			}
			started = true;
			if (group == Ref.id)
				entries.push_back(entry.second.get());
			else if (group > Ref.id)
				break;
		}
	}
	return entries;
}

std::string const *nodebank_panel::random_template(model_set_ref const &Ref) const
{
	auto const entries{set_entries(Ref)};
	if (entries.empty())
		return nullptr;

	std::uniform_int_distribution<std::size_t> dist(0, entries.size() - 1);
	return entries[dist(Global.local_random_engine)];
}

std::string nodebank_panel::set_name(model_set_ref const &Ref, char const *Manuallabel) const
{
	switch (Ref.kind)
	{
	case model_set_ref::source::user:
	{
		auto const *set = EditorModelSets.find(Ref.id);
		return set != nullptr ? set->name : "(deleted set)";
	}
	case model_set_ref::source::nodebank:
	{
		auto const groups{group_names()};
		return Ref.id >= 0 && Ref.id < static_cast<int>(groups.size()) ? "Node bank: " + groups[Ref.id] : "(missing node bank group)";
	}
	default:
		return Manuallabel != nullptr ? Manuallabel : "(none)";
	}
}

void nodebank_panel::open_sets_window(int const Setid)
{
	m_setsopen = true;
	if (Setid != 0)
	{
		m_setsselected = Setid;
		m_setsentry = -1;
	}
}

void nodebank_panel::render_sets_window()
{
	ImGui::SetNextWindowSize(ImVec2S(600, 380), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin("Model sets", &m_setsopen, ImGuiWindowFlags_NoCollapse))
	{
		ImGui::End();
		return;
	}

	auto const &sets{EditorModelSets.sets()};
	if (EditorModelSets.find(m_setsselected) == nullptr)
	{
		m_setsselected = sets.empty() ? 0 : sets.front().id;
		m_setsentry = -1;
	}

	// left side: the sets
	auto const buttonsheight{ImGui::GetFrameHeightWithSpacing()};
	ImGui::BeginGroup();
	ImGui::BeginChild("##setlist", ImVec2(190 * Global.ui_scale, -buttonsheight), true);
	for (auto const &set : sets)
	{
		auto const label{set.name + " (" + std::to_string(set.templates.size()) + ")##set" + std::to_string(set.id)};
		if (list_item(label.c_str(), set.id == m_setsselected))
		{
			m_setsselected = set.id;
			m_setsentry = -1;
		}
	}
	if (sets.empty())
	{
		ImGui::TextDisabled("No sets yet");
	}
	ImGui::EndChild();
	if (ImGui::Button("New"))
	{
		open_sets_window(EditorModelSets.create("New set"));
	}
	auto const *edited{EditorModelSets.find(m_setsselected)};
	if (edited != nullptr)
	{
		ImGui::SameLine();
		if (ImGui::Button("Duplicate"))
		{
			open_sets_window(EditorModelSets.create(edited->name + " copy", edited->templates));
			edited = EditorModelSets.find(m_setsselected);
		}
		ImGui::SameLine();
		if (ImGui::Button("Delete"))
		{
			ImGui::OpenPopup("##deleteset");
		}
		if (ImGui::BeginPopup("##deleteset"))
		{
			ImGui::Text("Delete set \"%s\"?", edited->name.c_str());
			if (ImGui::Button("Delete##confirm"))
			{
				EditorModelSets.remove(edited->id);
				edited = nullptr;
				ImGui::CloseCurrentPopup();
			}
			ImGui::SameLine();
			if (ImGui::Button("Cancel"))
			{
				ImGui::CloseCurrentPopup();
			}
			ImGui::EndPopup();
		}
	}
	ImGui::EndGroup();

	ImGui::SameLine();

	// right side: content of the selected set
	ImGui::BeginGroup();
	if (edited == nullptr)
	{
		ImGui::TextDisabled("Create a set with \"New\", or use \"Save as set\" on a brush or area fill list.");
		ImGui::TextDisabled("Sets can be chosen in Insert (random model), Brush and Area fill.");
	}
	else
	{
		if (m_setsnameid != edited->id)
		{
			// selection changed, load its name into the edit buffer
			std::snprintf(m_setsname, sizeof(m_setsname), "%s", edited->name.c_str());
			m_setsnameid = edited->id;
		}
		ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize("Rename").x - ImGui::GetStyle().FramePadding.x * 2 - ImGui::GetStyle().ItemSpacing.x);
		auto const entered{ImGui::InputText("##setname", m_setsname, IM_ARRAYSIZE(m_setsname), ImGuiInputTextFlags_EnterReturnsTrue)};
		ImGui::PopItemWidth();
		ImGui::SameLine();
		if (ImGui::Button("Rename") || entered)
		{
			EditorModelSets.rename(edited->id, m_setsname);
			m_setsnameid = 0; // reload, the name could have been adjusted to stay unique
		}

		ImGui::BeginChild("##setentries", ImVec2(0, -buttonsheight * 2), true);
		for (int idx = 0; idx < static_cast<int>(edited->labels.size()); ++idx)
		{
			auto const label{edited->labels[idx] + "##entry" + std::to_string(idx)};
			if (list_item(label.c_str(), idx == m_setsentry))
				m_setsentry = idx;
		}
		if (edited->templates.empty())
		{
			ImGui::TextDisabled("(empty) select a template in the node bank and press \"Add selected\"");
		}
		ImGui::EndChild();

		auto const setid{edited->id};
		if (ImGui::Button("Add selected"))
		{
			if (node_selected())
				EditorModelSets.add(setid, {*m_selectedtemplate});
		}
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("Adds the template selected in the node bank list");
		}
		ImGui::SameLine();
		if (ImGui::Button("Remove") && m_setsentry >= 0)
		{
			EditorModelSets.erase(setid, static_cast<std::size_t>(m_setsentry));
			m_setsentry = -1;
		}
		ImGui::SameLine();
		if (ImGui::Button("Clear"))
		{
			ImGui::OpenPopup("##clearset");
		}
		if (ImGui::BeginPopup("##clearset"))
		{
			ImGui::TextUnformatted("Remove all templates from this set?");
			if (ImGui::Button("Clear##confirm"))
			{
				EditorModelSets.clear(setid);
				m_setsentry = -1;
				ImGui::CloseCurrentPopup();
			}
			ImGui::SameLine();
			if (ImGui::Button("Cancel"))
			{
				ImGui::CloseCurrentPopup();
			}
			ImGui::EndPopup();
		}

		// whole node bank group at once
		ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize("Add group").x - ImGui::GetStyle().FramePadding.x * 2 - ImGui::GetStyle().ItemSpacing.x);
		auto const groups{group_names()};
		m_setsgroup.id = groups.empty() ? -1 : std::clamp(m_setsgroup.id, 0, static_cast<int>(groups.size()) - 1);
		if (ImGui::BeginCombo("##addgroup", m_setsgroup.id >= 0 ? groups[m_setsgroup.id].c_str() : "(no node bank groups)"))
		{
			for (int idx = 0; idx < static_cast<int>(groups.size()); ++idx)
			{
				auto const label{groups[idx] + "##addgroup" + std::to_string(idx)};
				if (ImGui::Selectable(label.c_str(), idx == m_setsgroup.id))
					m_setsgroup.id = idx;
			}
			ImGui::EndCombo();
		}
		ImGui::PopItemWidth();
		ImGui::SameLine();
		if (ImGui::Button("Add group"))
		{
			std::vector<std::string> templates;
			for (auto const *entry : set_entries(m_setsgroup))
				templates.push_back(*entry);
			EditorModelSets.add(setid, templates);
		}
	}
	ImGui::EndGroup();

	ImGui::End();
}

void nodebank_panel::add_template(const std::string &desc)
{

	auto const label{generate_node_label(desc)};
	m_nodebank.push_back({label, std::make_shared<std::string>(desc)});

	std::ofstream file;
	file.open("nodebank.txt", std::ios_base::out | std::ios_base::app | std::ios_base::binary);
	file << label << " " << desc;
}

const std::string *nodebank_panel::get_active_template()
{
	return m_selectedtemplate.get();
}

bool nodebank_panel::node_selected() const
{
	// scenery templates are placed with include directives, the tools which scatter nodes have no use for them
	return m_selectedtemplate && false == m_selectedtemplate->empty() && false == m_selectedtemplate->starts_with(editor_includes::directive_mark);
}

std::vector<std::string> nodebank_panel::group_names() const
{
	std::vector<std::string> names;
	for (auto const &entry : m_nodebank)
	{
		if (entry.second->empty())
			names.emplace_back(entry.first);
		else if (names.empty())
			names.emplace_back("(ungrouped)"); // templates listed before the first header
	}
	return names;
}

std::vector<std::string> nodebank_panel::group_templates(std::size_t const Group) const
{
	std::vector<std::string> templates;
	std::size_t group{0};
	bool started{false};
	for (auto const &entry : m_nodebank)
	{
		if (entry.second->empty())
		{
			// header: opens the next group, unless it's the very first entry
			if (started)
				++group;
			started = true;
			continue;
		}
		started = true;
		if (group == Group)
			templates.emplace_back(*entry.second);
		else if (group > Group)
			break;
	}
	return templates;
}

std::string nodebank_panel::generate_node_label(std::string Input) const
{

	auto tokenizer{cParser(Input)};
	tokenizer.getTokens(9, false); // skip leading tokens
	auto model{tokenizer.getToken<std::string>(false)};
	auto texture{tokenizer.getToken<std::string>(false)};
	replace_slashes(model);
	erase_extension(model);
	replace_slashes(texture);
	return texture == "none" ? model : model + " (" + texture + ")";
}

void functions_panel::update(scene::basic_node const *Node)
{
	m_node = Node;

	if (false == is_open)
	{
		return;
	}

	text_lines.clear();
	m_grouplines.clear();

	std::string textline;

	// scenario inspector
	auto const *node{Node};
	auto const &camera{Global.pCamera};
}

void functions_panel::render()
{

	if (false == is_open)
	{
		return;
	}

	auto flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoCollapse | (size.x > 0 ? ImGuiWindowFlags_NoResize : 0);

	if (size.x > 0)
	{
		ImGui::SetNextWindowSize(ImVec2S(size.x, size.y));
	}
	if (size_min.x > 0)
	{
		ImGui::SetNextWindowSizeConstraints(ImVec2S(size_min.x, size_min.y), ImVec2(size_max.x, size_max.y));
	}
	auto const panelname{(title.empty() ? m_name : title) + "###" + m_name};
	if (true == ImGui::Begin(panelname.c_str(), nullptr, flags))
	{
		// header section
		render_controls();
		for (auto const &line : text_lines)
		{
			ImGui::TextColored(ImVec4(line.color.r, line.color.g, line.color.b, line.color.a), line.data.c_str());
		}
	}
	ImGui::End();
}

void functions_panel::render_controls()
{
	// order matches rotation_mode
	char const *const modes[] = {"Random", "Fixed", "Default (from template)"};
	auto current{static_cast<int>(rot_mode)};
	if (ImGui::Combo("Rotation", &current, modes, IM_ARRAYSIZE(modes)))
	{
		rot_mode = static_cast<rotation_mode>(current);
	}
	if (rot_mode == FIXED)
	{
		// ImGui::Checkbox("Get rotation from last object", &rot_from_last);
		ImGui::SliderFloat("Rotation value", &rot_value, 0.0f, 360.0f, "%.1f deg");
	}
}

namespace
{

std::pair<char const *, scene::layer_item> const layer_itemlabels[] = {{"models", scene::layer_item::model},
                                                                       {"tracks", scene::layer_item::track},
                                                                       {"traction", scene::layer_item::traction},
                                                                       {"power sources", scene::layer_item::powersource},
                                                                       {"memory cells", scene::layer_item::memcell},
                                                                       {"event launchers", scene::layer_item::launcher},
                                                                       {"events", scene::layer_item::event},
                                                                       {"vehicles", scene::layer_item::vehicle},
                                                                       {"sounds", scene::layer_item::sound},
                                                                       {"terrain shapes", scene::layer_item::shape},
                                                                       {"lines", scene::layer_item::lines}};

// what the layer holds, a line per kind of item
std::string layer_content(scene::basic_layer const &Layer)
{
	std::string content;
	for (auto const &itemlabel : layer_itemlabels)
	{
		auto const count{Layer.items[static_cast<std::size_t>(itemlabel.second)]};
		if (count > 0)
		{
			content += (content.empty() ? "" : "\n") + std::string{itemlabel.first} + ": " + std::to_string(count);
		}
	}
	return content;
}

std::string layer_name(scene::layer_handle const Layer)
{
	return scene::Layers.valid(Layer) ? Bezogonkow(scene::Layers.layer(Layer).name) : "(none)";
}

// button which can be greyed out, with an explanation shown when it's hovered
bool action_button(char const *Label, bool const Enabled, std::string const &Reason = std::string())
{
	if (false == Enabled)
	{
		ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
	}
	auto const clicked{ImGui::Button(Label) && Enabled};
	if (false == Enabled)
	{
		ImGui::PopStyleVar();
		if (false == Reason.empty() && ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("%s", Reason.c_str());
		}
	}
	return clicked;
}

// text field bound to a string
bool input_text(char const *Label, std::string &Value)
{
	std::array<char, 256> buffer{};
	Value.copy(buffer.data(), buffer.size() - 1);
	if (false == ImGui::InputText(Label, buffer.data(), buffer.size()))
	{
		return false;
	}
	Value = buffer.data();
	return true;
}

ImVec4 const status_errorcolor{1.0f, 0.45f, 0.4f, 1.0f};

} // namespace

void itemproperties_panel::render_include()
{
	auto &include{*m_include};
	include.active = false;

	ImGui::Text("include: %s", Bezogonkow(include.target).c_str());
	if (false == include.info.name.empty())
	{
		ImGui::TextDisabled("%s", include.info.name.c_str());
	}
	if (false == include.issue.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextColored(status_errorcolor, "%s", Bezogonkow(include.issue).c_str());
		ImGui::PopTextWrapPos();
		return;
	}
	if (include.values.empty())
	{
		ImGui::TextDisabled("The template takes no parameters.");
		return;
	}

	ImGui::PushItemWidth(ImGui::GetFontSize() * 14.0f);
	for (std::size_t idx = 0; idx < include.values.size(); ++idx)
	{
		auto const id{static_cast<int>(idx + 1)};
		auto const lookup{std::find_if(std::begin(include.info.parameters), std::end(include.info.parameters), [=](include_parameter const &Parameter) { return Parameter.id == id; })};
		auto const described{lookup != std::end(include.info.parameters)};
		auto const role{described ? lookup->role : std::string{"free"}};
		// the label from the description, or what the description tells about the parameter
		auto const label{described && false == lookup->label.empty() ? lookup->label : role != "free" ? role : "(p" + std::to_string(id) + ")"};
		auto &value{include.values[idx]};

		ImGui::PushID(id);
		char *end{nullptr};
		auto number{std::strtod(value.c_str(), &end)};
		if ((role.starts_with("pos.") || role.starts_with("rot.") || role == "number") && false == value.empty() && *end == '\0')
		{
			if (ImGui::DragScalar(label.c_str(), ImGuiDataType_Double, &number, role.starts_with("pos.") ? 0.05f : 0.5f, nullptr, nullptr, "%.3f"))
			{
				value = editor_includes::number(number);
				include.changed = true;
			}
		}
		else
		{
			// applied once the field is left, so the template isn't processed with a half-typed value
			input_text(label.c_str(), value);
			if (ImGui::IsItemDeactivatedAfterEdit())
			{
				include.changed = true;
			}
		}
		include.active |= ImGui::IsItemActive();
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("(p%d), %s", id, role.c_str());
		}
		ImGui::PopID();
	}
	ImGui::PopItemWidth();

	ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
	ImGui::TextWrapped("Models of the template follow the changes at once, the rest of it once the scenery is saved and loaded again.");
	ImGui::PopStyleColor();
}

void layers_panel::report(std::string const &Status, bool const Error)
{
	status = Status;
	status_error = Error;
}

void layers_panel::render()
{
	if (false == is_open)
	{
		return;
	}

	// initially next to the toolset window
	ImGui::SetNextWindowPos(ImVec2S(520, 60), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2S(440, 340), ImGuiCond_FirstUseEver);
	auto const panelname{(title.empty() ? m_name : title) + "###" + m_name};
	if (ImGui::Begin(panelname.c_str(), &is_open, ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoCollapse))
	{
		if (scene::Layers.empty())
		{
			// layers are established only when the scenery is loaded for editing, regular load skips the bookkeeping
			ImGui::TextDisabled("Scenery layers are available in an edit session.\nStart the simulator with: -edit <scenery file>");
		}
		else
		{
			if (false == scene::Layers.listed(m_selected))
			{
				m_selected = null_handle;
			}
			auto const selected{m_selected != null_handle};
			auto const removed{selected && scene::Layers.layer(m_selected).removed};
			std::string reason{"Select a layer first"};

			if (ImGui::Button("New..."))
			{
				m_newname[0] = '\0';
				m_newparent = (selected && false == removed && false == scene::Layers.layer(m_selected).binary ? m_selected : scene::layer_handle{1});
				m_popuperror.clear();
				ImGui::OpenPopup("New layer");
			}
			ImGui::SameLine();
			if (removed)
			{
				if (ImGui::Button("Restore"))
				{
					scene::Layers.restore(m_selected);
					report("Layer \"" + layer_name(m_selected) + "\" restored");
				}
			}
			else if (action_button("Remove...", selected && scene::Layers.can_remove(m_selected, reason), reason))
			{
				ImGui::OpenPopup("Remove layer");
			}
			ImGui::SameLine();
			// the scenario file is what holds the scenery together, it stays where it is
			auto const mergeable{selected && false == removed && (scene::Layers.layer(m_selected).created || false == scene::Layers.layer(m_selected).sites.empty())};
			if (action_button("Merge into...", mergeable, selected && false == removed ? "The scenario file can't be merged into another layer" : "Select a layer first"))
			{
				// the layer which includes the selected one is the most likely target
				m_mergetarget = scene::Layers.resolve(scene::Layers.layer(m_selected).parent);
				if (false == scene::Layers.can_merge(m_selected, m_mergetarget, reason))
				{
					m_mergetarget = null_handle;
				}
				ImGui::OpenPopup("Merge layer");
			}
			ImGui::SameLine();
			if (ImGui::Button("Save") && save)
			{
				save();
			}
			if (ImGui::IsItemHovered())
			{
				ImGui::SetTooltip("Writes changed nodes and layers to the scenery files (Ctrl+S)");
			}
			ImGui::Separator();

			render_list();

			if (false == status.empty())
			{
				ImGui::PushTextWrapPos(0.0f);
				if (status_error)
				{
					ImGui::TextColored(status_errorcolor, "%s", status.c_str());
				}
				else
				{
					ImGui::TextUnformatted(status.c_str());
				}
				ImGui::PopTextWrapPos();
			}

			render_popups();
		}
	}
	ImGui::End();
}

void layers_panel::render_list()
{
	// the list takes the window except for the room left for the status
	ImGui::BeginChild("##layers", ImVec2(0.0f, -ImGui::GetTextLineHeightWithSpacing() * 2.5f));
	ImGui::TextDisabled("visible, locked, active, layer file");

	for (auto const handle : scene::Layers.tree())
	{
		// NOTE: copies of the flags, as the calls below can change the layer
		auto const &layer{scene::Layers.layer(handle)};
		auto const isactive{handle == scene::Layers.active()};
		auto const isremoved{layer.removed};
		std::string readonly;
		auto const iswritable{scene::Layers.writable(handle, &readonly)};

		ImGui::PushID(static_cast<int>(handle));
		if (isremoved)
		{
			ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
		}
		// active layer receives new nodes, so it can't be hidden nor locked. removed layer is out of the picture altogether
		auto const fixed{isactive || isremoved};
		auto visible{layer.visible};
		if (ImGui::Checkbox("##visible", &visible) && false == fixed)
		{
			scene::Layers.visible(handle, visible);
		}
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("%s", isactive ? "Active layer is always visible" : "Show or hide models, tracks and traction of the layer");
		}
		ImGui::SameLine();
		auto locked{layer.locked};
		if (ImGui::Checkbox("##locked", &locked) && false == fixed)
		{
			scene::Layers.locked(handle, locked);
		}
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("%s", isactive ? "Active layer can't be locked" : "Nodes of locked layer can't be selected in the viewport");
		}
		ImGui::SameLine();
		if (ImGui::RadioButton("##active", isactive) && false == isactive)
		{
			if (std::string reason; false == scene::Layers.accepts(handle, &reason))
			{
				report("Layer \"" + layer_name(handle) + "\" can't take new nodes" + (reason.empty() ? "" : ": " + reason), true);
			}
			else
			{
				scene::Layers.active(handle);
			}
		}
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("Active layer: nodes created in the editor are placed in it");
		}
		ImGui::SameLine();
		auto label{std::string(2 * scene::Layers.depth(handle), ' ') + layer_name(handle) + " (" + (layer.binary ? "binary terrain" : std::to_string(layer.item_count())) + ")"};
		if (layer.created)
		{
			label += "  [new]";
		}
		if (isremoved)
		{
			label += "  [removed]";
		}
		else if (false == iswritable)
		{
			label += "  [read-only]";
		}
		else if (layer.late)
		{
			label += "  [after FirstInit]";
		}
		if (ImGui::Selectable(label.c_str(), handle == m_selected))
		{
			m_selected = handle;
		}
		if (ImGui::IsItemHovered())
		{
			std::string content{layer.created ? "New layer, its file is created on save" : isremoved ? "Dropped from the scenery on save" : "Scenery file"};
			if (scene::Layers.valid(layer.parent))
			{
				content += "\nincluded by: " + layer_name(scene::Layers.resolve(layer.parent));
			}
			if (false == iswritable)
			{
				content += "\nread-only: " + readonly;
			}
			if (layer.late)
			{
				content += "\nloaded after the scenario initialization (FirstInit): takes no new nodes, only vehicles belong there";
			}
			if (auto const items{layer_content(layer)}; false == items.empty())
			{
				content += "\n" + items;
			}
			ImGui::SetTooltip("%s", content.c_str());
		}
		if (isremoved)
		{
			ImGui::PopStyleVar();
		}
		ImGui::PopID();
	}
	ImGui::EndChild();
}

void layers_panel::render_popups()
{
	if (ImGui::BeginPopupModal("New layer", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
	{
		ImGui::TextUnformatted("File of the new layer, in the scenery directory:");
		ImGui::InputTextWithHint("##name", "name.scm", m_newname, IM_ARRAYSIZE(m_newname));
		if (ImGui::BeginCombo("Included by", layer_name(m_newparent).c_str()))
		{
			for (std::size_t idx = 1; idx <= scene::Layers.size(); ++idx)
			{
				auto const handle{static_cast<scene::layer_handle>(idx)};
				if (false == scene::Layers.listed(handle) || scene::Layers.layer(handle).removed || scene::Layers.layer(handle).binary)
				{
					continue;
				}
				if (ImGui::Selectable((layer_name(handle) + "##" + std::to_string(idx)).c_str(), handle == m_newparent))
				{
					m_newparent = handle;
				}
			}
			ImGui::EndCombo();
		}
		if (false == m_popuperror.empty())
		{
			ImGui::TextColored(status_errorcolor, "%s", m_popuperror.c_str());
		}
		if (ImGui::Button("Create"))
		{
			auto const created{scene::Layers.create(m_newname, m_newparent, m_popuperror)};
			if (created != null_handle)
			{
				m_selected = created;
				report("Layer \"" + layer_name(created) + "\" created and made active. Its file is written on save.");
				ImGui::CloseCurrentPopup();
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Cancel"))
		{
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}

	if (ImGui::BeginPopupModal("Remove layer", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
	{
		if (false == scene::Layers.listed(m_selected))
		{
			ImGui::CloseCurrentPopup();
		}
		else
		{
			auto const &layer{scene::Layers.layer(m_selected)};
			auto included{0};
			for (std::size_t idx = 1; idx <= scene::Layers.size(); ++idx)
			{
				auto const handle{static_cast<scene::layer_handle>(idx)};
				if (handle != m_selected && scene::Layers.listed(handle) && scene::Layers.is_descendant(handle, m_selected))
				{
					++included;
				}
			}
			ImGui::Text("Remove layer \"%s\" from the scenery?", layer_name(m_selected).c_str());
			ImGui::TextUnformatted(layer.created ? "The layer wasn't saved yet, so it leaves no file behind." : "Its include directive is erased on save. The file itself stays on the disk.");
			if (included > 0)
			{
				ImGui::Text("%d layer(s) it includes are removed with it.", included);
			}
			if (auto const items{layer_content(layer)}; false == items.empty())
			{
				ImGui::Separator();
				ImGui::TextUnformatted("The layer holds:");
				ImGui::TextUnformatted(items.c_str());
				ImGui::TextDisabled("Make sure the rest of the scenery doesn't refer to its tracks, events or memory cells.");
			}
			if (ImGui::Button("Remove"))
			{
				auto const name{layer_name(m_selected)};
				if (scene::Layers.remove(m_selected))
				{
					report("Layer \"" + name + "\" removed. It can be restored until the scenery is saved.");
				}
				ImGui::CloseCurrentPopup();
			}
			ImGui::SameLine();
			if (ImGui::Button("Cancel"))
			{
				ImGui::CloseCurrentPopup();
			}
		}
		ImGui::EndPopup();
	}

	if (ImGui::BeginPopupModal("Merge layer", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
	{
		if (false == scene::Layers.listed(m_selected))
		{
			ImGui::CloseCurrentPopup();
		}
		else
		{
			ImGui::Text("Move content of layer \"%s\" to:", layer_name(m_selected).c_str());
			std::string reason;
			auto targets{0};
			if (ImGui::BeginCombo("##target", layer_name(m_mergetarget).c_str()))
			{
				for (std::size_t idx = 1; idx <= scene::Layers.size(); ++idx)
				{
					auto const handle{static_cast<scene::layer_handle>(idx)};
					if (false == scene::Layers.listed(handle) || false == scene::Layers.can_merge(m_selected, handle, reason))
					{
						continue;
					}
					++targets;
					if (ImGui::Selectable((layer_name(handle) + "##" + std::to_string(idx)).c_str(), handle == m_mergetarget))
					{
						m_mergetarget = handle;
					}
				}
				if (targets == 0)
				{
					ImGui::TextDisabled("no layer can take it");
				}
				ImGui::EndCombo();
			}
			ImGui::TextDisabled("The text of the file is moved on save, the emptied file stays on the disk.\n"
			                    "Merged into the layer which includes it, the content stays where the include was.\n"
			                    "Merged into any other layer it's placed at its end, which changes the load order.");
			auto const possible{m_mergetarget != null_handle && scene::Layers.can_merge(m_selected, m_mergetarget, reason)};
			if (action_button("Merge", possible, m_mergetarget != null_handle ? reason : "Pick the target layer"))
			{
				auto const name{layer_name(m_selected)};
				auto const target{m_mergetarget};
				if (scene::Layers.merge(m_selected, target))
				{
					report("Layer \"" + name + "\" merged into \"" + layer_name(target) + "\". The files are changed on save.");
					m_selected = target;
				}
				ImGui::CloseCurrentPopup();
			}
			ImGui::SameLine();
			if (ImGui::Button("Cancel"))
			{
				ImGui::CloseCurrentPopup();
			}
		}
		ImGui::EndPopup();
	}
}

void includes_panel::open(std::string const &File)
{
	m_file = File;
	m_statuserror = false;
	m_parameters = 0;
	std::string error;
	m_loaded = editor_includes::load(File, m_info, error);
	if (false == m_loaded)
	{
		m_status = error;
		m_statuserror = true;
		return;
	}
	auto const described{false == m_info.name.empty() || false == m_info.parameters.empty()};
	// every parameter the template uses gets an entry, described or not
	m_parameters = editor_includes::parameter_count(File);
	for (auto id = 1; id <= m_parameters; ++id)
	{
		m_info.parameter(id);
	}
	m_status = "The template uses " + std::to_string(m_parameters) + " parameter(s).";
	if (false == described)
	{
		// starting point for a new description
		editor_includes::suggest(File, m_info);
		m_status += " It has no description yet; the roles are a suggestion based on how the template uses the parameters.";
	}
}

void includes_panel::render_list()
{
	if (false == EditorIncludes.scanned())
	{
		EditorIncludes.scan();
	}
	auto const &entries{EditorIncludes.entries()};

	ImGui::PushItemWidth(ImGui::GetFontSize() * 12.0f);
	auto changed{ImGui::InputTextWithHint("##filter", "Search templates", m_filter, IM_ARRAYSIZE(m_filter))};
	ImGui::PopItemWidth();
	ImGui::SameLine();
	changed |= ImGui::Checkbox("Described", &m_describedonly);
	if (false == scene::Layers.templates().empty())
	{
		// known only for scenery opened for editing
		ImGui::SameLine();
		changed |= ImGui::Checkbox("Used by the scenery", &m_usedonly);
	}
	ImGui::SameLine();
	if (ImGui::Button("Rescan"))
	{
		EditorIncludes.scan();
	}
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip("Looks through the scenery directory again, for the templates added or changed outside of the editor");
	}

	if (changed || m_listrevision != EditorIncludes.revision())
	{
		m_listrevision = EditorIncludes.revision();
		m_listed.clear();
		m_described = 0;
		std::string const filter{m_filter};
		auto const &used{scene::Layers.templates()};
		for (std::size_t idx = 0; idx < entries.size(); ++idx)
		{
			auto const &entry{entries[idx]};
			m_described += (entry.described ? 1 : 0);
			if ((m_describedonly && false == entry.described) || (m_usedonly && false == used.empty() && used.count(entry.file) == 0))
			{
				continue;
			}
			if (false == filter.empty() && false == contains(entry.file, filter) && false == contains(entry.name, filter) && false == contains(entry.category, filter))
			{
				continue;
			}
			m_listed.emplace_back(idx);
		}
	}
	ImGui::TextDisabled("%d template(s) in the scenery directory, %d described, %d ready for the node bank", static_cast<int>(entries.size()), m_described,
	                    static_cast<int>(EditorIncludes.ready().size()));

	ImGui::BeginChild("##templates", ImVec2(0.0f, ImGui::GetTextLineHeightWithSpacing() * 9.0f), true);
	ImGuiListClipper clipper(static_cast<int>(m_listed.size()));
	while (clipper.Step())
	{
		for (auto row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
		{
			auto const &entry{entries[m_listed[row]]};
			auto const marker{entry.complete ? "[ready]  " : entry.described ? "[incomplete]  " : ""};
			auto const label{marker + Bezogonkow(entry.file) + (entry.name.empty() ? "" : "  -  " + entry.name) + "##" + std::to_string(m_listed[row])};
			if (false == entry.described)
			{
				ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
			}
			auto const clicked{list_item(label.c_str(), entry.file == m_file)};
			if (false == entry.described)
			{
				ImGui::PopStyleColor();
			}
			if (false == entry.complete && false == entry.issue.empty() && ImGui::IsItemHovered())
			{
				ImGui::SetTooltip("Not in the node bank: %s", Bezogonkow(entry.issue).c_str());
			}
			if (clicked)
			{
				open(entry.file);
			}
		}
	}
	if (m_listed.empty())
	{
		ImGui::TextDisabled(entries.empty() ? "(no .inc files in the scenery directory)" : "(no match)");
	}
	ImGui::EndChild();
}

void includes_panel::render_description()
{
	ImGui::Separator();
	ImGui::Text("%s", Bezogonkow(m_file).c_str());
	input_text("Name", m_info.name);
	input_text("Category", m_info.category);
	input_text("Description", m_info.description);

	ImGui::Columns(4, "##parameters", false);
	ImGui::SetColumnWidth(0, ImGui::CalcTextSize("(p000) ").x);
	ImGui::SetColumnWidth(1, ImGui::CalcTextSize("texture  ").x + ImGui::GetFrameHeight() * 2.0f);
	ImGui::TextDisabled("param");
	ImGui::NextColumn();
	ImGui::TextDisabled("role");
	ImGui::NextColumn();
	ImGui::TextDisabled("label");
	ImGui::NextColumn();
	ImGui::TextDisabled("default");
	ImGui::NextColumn();
	for (auto &parameter : m_info.parameters)
	{
		ImGui::PushID(parameter.id);
		ImGui::AlignTextToFramePadding();
		// parameters which keep the template out of the node bank stand out
		if (parameter.id <= m_parameters && parameter.value.empty() && false == editor_includes::automatic(parameter.role))
		{
			ImGui::TextColored(status_errorcolor, "(p%d)", parameter.id);
		}
		else
		{
			ImGui::Text("(p%d)", parameter.id);
		}
		ImGui::NextColumn();
		ImGui::PushItemWidth(-1.0f);
		if (ImGui::BeginCombo("##role", parameter.role.c_str()))
		{
			for (auto const &role : editor_includes::roles)
			{
				if (ImGui::Selectable(role.c_str(), role == parameter.role))
				{
					parameter.role = role;
				}
			}
			ImGui::EndCombo();
		}
		ImGui::PopItemWidth();
		ImGui::NextColumn();
		ImGui::PushItemWidth(-1.0f);
		input_text("##label", parameter.label);
		ImGui::PopItemWidth();
		ImGui::NextColumn();
		ImGui::PushItemWidth(-1.0f);
		input_text("##default", parameter.value);
		ImGui::PopItemWidth();
		ImGui::NextColumn();
		ImGui::PopID();
	}
	ImGui::Columns(1);

	// what the node bank is going to make of the description
	ImGui::PushTextWrapPos(0.0f);
	if (std::string issue; editor_includes::complete(m_info, m_parameters, &issue))
	{
		ImGui::TextUnformatted("Complete: once saved, the template is offered by the node bank (Insert tab, Scenery templates).");
	}
	else
	{
		ImGui::TextColored(status_errorcolor, "Not for the node bank yet: %s.", issue.c_str());
	}
	ImGui::PopTextWrapPos();
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip("A template placed from the node bank gets the cursor location in the pos.* parameters, the rotation set in the\n"
		                  "Insert tab in rot.y, and a unique name (the default value followed by a random suffix) in the name parameter.\n"
		                  "The other parameters receive their default values.");
	}

	if (ImGui::Button("Suggest roles"))
	{
		editor_includes::suggest(m_file, m_info);
	}
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip("Sets roles of the parameters still marked as free, where the way the template uses them tells what they are");
	}
	ImGui::SameLine();
	if (ImGui::Button("Add parameter"))
	{
		m_info.parameter(m_info.parameters.empty() ? 1 : m_info.parameters.back().id + 1);
	}
	ImGui::SameLine();
	if (ImGui::Button("Save to file"))
	{
		std::string error;
		m_statuserror = false == editor_includes::save(m_file, m_info, error);
		m_status = m_statuserror ? error : "Description saved in \"" + m_file + "\"";
		if (false == m_statuserror)
		{
			EditorIncludes.update(m_file);
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Reload"))
	{
		open(m_file);
	}
}

void includes_panel::render()
{
	if (false == is_open)
	{
		return;
	}

	// next to the layers window where the screen is wide enough for both
	auto const position{ImVec2S(970, 60)};
	auto const windowsize{ImVec2S(600, 600)};
	ImGui::SetNextWindowPos(position.x + windowsize.x <= ImGui::GetIO().DisplaySize.x ? position : ImVec2S(560, 110), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(windowsize, ImGuiCond_FirstUseEver);
	auto const panelname{(title.empty() ? m_name : title) + "###" + m_name};
	if (ImGui::Begin(panelname.c_str(), &is_open, ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoCollapse))
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextWrapped("The .inc templates of the scenery directory. Describe a template and its parameters to add it to the node bank; "
		                   "the description is stored in the template file, as //$e comment lines.");
		if (false == Global.editor_session)
		{
			ImGui::TextWrapped("The node bank offers the templates in a scenery opened for editing (-edit).");
		}
		ImGui::PopStyleColor();

		render_list();
		if (m_loaded)
		{
			render_description();
		}
		if (false == m_status.empty())
		{
			ImGui::PushTextWrapPos(0.0f);
			if (m_statuserror)
			{
				ImGui::TextColored(status_errorcolor, "%s", Bezogonkow(m_status).c_str());
			}
			else
			{
				ImGui::TextUnformatted(Bezogonkow(m_status).c_str());
			}
			ImGui::PopTextWrapPos();
		}
	}
	ImGui::End();
}

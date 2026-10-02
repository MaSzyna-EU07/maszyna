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
#include "editor/editorModelSets.hpp"

namespace
{
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
	// header section
	for (auto const &line : text_lines)
	{
		ImGui::TextColored(ImVec4(line.color.r, line.color.g, line.color.b, line.color.a), line.data.c_str());
	}
	// transform editor (position/rotation/scale) — TAnimModel only
	render_transform_editor();
	// group section
	render_group();
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
				if (false == ImGui::BeginTabItem(tab.first))
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
		if (m_selectedtemplate && false == m_selectedtemplate->empty())
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
			if (m_selectedtemplate && false == m_selectedtemplate->empty())
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
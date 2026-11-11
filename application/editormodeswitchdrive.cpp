/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodeshared.h"
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "editor/editorIncludeInfo.hpp"

#include "world/Segment.h"
#include "world/Track.h"
#include "world/TrackRails.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>

namespace
{

std::string drive_file(std::string Text)
{
	std::replace(Text.begin(), Text.end(), '\\', '/');
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char Character) { return static_cast<char>(std::tolower(Character)); });
	return Text;
}

// the track the events of a drive find by the name: the game reads the names of the tracks lower case, the parameters of an include as
// they're written
TTrack *track_named(std::string const &Name)
{
	auto *track{simulation::Paths.find(Name)};
	return track != nullptr ? track : simulation::Paths.find(drive_file(Name));
}

// a template by the name of its file, with no folder: told to the user, who picks the drives by their names
std::string file_name(std::string const &File)
{
	auto const slash{File.find_last_of('/')};
	return slash != std::string::npos ? File.substr(slash + 1) : File;
}

int drive_side(std::string const &Side)
{
	if (Side == "left")
		return 0;
	return Side == "right" ? 1 : -1;
}

std::optional<std::string> pose_value(std::string const &Role, glm::dvec3 const &Local, editor_track::drive_spot const &Spot)
{
	if (Role == "pos.x")
		return editor_includes::number(Local.x);
	if (Role == "pos.y")
		return editor_includes::number(Local.y);
	if (Role == "pos.z")
		return editor_includes::number(Local.z);
	if (Role == "rot.y")
		return editor_includes::number(Spot.yaw);
	if (Role == "rot.x")
		return editor_includes::number(Spot.tilt.x);
	if (Role == "rot.z")
		return editor_includes::number(Spot.tilt.y);
	return std::nullopt;
}

double angle_value(std::vector<std::string> const &Values, std::size_t const Index)
{
	double value{0.0};
	if (Index < Values.size())
		std::sscanf(Values[Index].c_str(), "%lf", &value);
	return value;
}

// the first line of the file of the keys by the movable noses, which tells it from the files of the scenery made by hand
char const *const kNoseHeader{"// made by the scenery editor on each save: T and Shift+T by the movable nose throw its switch, as by its drive"};

// true if the file is one of the keys by the movable noses the editor writes
bool nose_file(std::string const &Path)
{
	std::ifstream input{Path, std::ios_base::binary};
	std::string first;
	std::getline(input, first);
	if (false == first.empty() && first.back() == '\r')
		first.pop_back();
	return first == kNoseHeader;
}

} // namespace

bool editor_mode::scan_switch_drives(bool const Wait)
{
	auto &tool{m_switch};
	if (Wait)
		EditorIncludes.wait();
	else
		EditorIncludes.start();
	if (false == EditorIncludes.scanned() || tool.drives_revision == EditorIncludes.revision())
		return false;
	tool.drives_revision = EditorIncludes.revision();
	// the drive chosen stays chosen, also none
	auto const chosen{tool.drive >= 0 && tool.drive < static_cast<int>(tool.drives.size()) ? std::optional<std::string>{tool.drives[tool.drive].name} : std::nullopt};
	auto const none{false == tool.drives.empty() && tool.drive < 0};
	tool.drives.clear();
	for (auto const &entry : EditorIncludes.entries())
	{
		if (entry.category != "switch drive")
			continue;
		include_info info;
		std::string error;
		if (false == editor_includes::load(entry.file, info, error) || editor_includes::parameter_with_role(info, "track") == 0)
			continue;
		auto const hand{drive_side(info.switch_hand)};
		auto const drive{drive_side(info.switch_drive)};
		if (hand < 0 || drive < 0)
			continue;
		auto const name{info.name.empty() ? entry.file : info.name};
		auto family{std::find_if(tool.drives.begin(), tool.drives.end(), [&](switch_tool::drive_family const &Family) { return Family.name == name; })};
		if (family == tool.drives.end())
		{
			tool.drives.push_back({name, {}});
			family = std::prev(tool.drives.end());
		}
		family->files[hand * 2 + drive] = entry.file;
		family->heights[hand * 2 + drive] = info.switch_height;
		family->preferred |= info.switch_default;
	}
	std::sort(tool.drives.begin(), tool.drives.end(), [](switch_tool::drive_family const &A, switch_tool::drive_family const &B) { return A.name < B.name; });
	auto const preferred{std::find_if(tool.drives.begin(), tool.drives.end(), [](switch_tool::drive_family const &Family) { return Family.preferred; })};
	auto const kept{std::find_if(tool.drives.begin(), tool.drives.end(), [&](switch_tool::drive_family const &Family) { return chosen.has_value() && Family.name == *chosen; })};
	if (tool.drives.empty() || none)
		tool.drive = -1;
	else if (kept != tool.drives.end())
		tool.drive = static_cast<int>(kept - tool.drives.begin());
	else
		tool.drive = preferred != tool.drives.end() ? static_cast<int>(preferred - tool.drives.begin()) : 0;
	return true;
}

void editor_mode::switch_drive_captured(TTrack const &Track)
{
	if (Track.eType != tt_Switch || m_infra_suspended)
		return;
	auto const found{find_switch_drive(Track)};
	if (false == found.has_value())
		return;
	auto const instance{found->standing.instance};
	if (std::any_of(m_drive_buffer.begin(), m_drive_buffer.end(), [&](auto const &Entry) { return Entry.first == instance; }))
		return;
	m_drive_buffer.emplace_back(instance, scene::Layers.directive(instance));
}

void editor_mode::switch_drives_committed(std::vector<TTrack *> const &Tracks)
{
	if (m_infra_suspended)
		return;
	// a copy: each directive set below empties the drives kept
	std::optional<std::map<std::string, switch_drive>> drives;
	for (auto const *track : Tracks)
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch)
			continue;
		if (false == drives.has_value())
			drives = switch_drives_standing();
		auto const found{drives->find(drive_file(track->name()))};
		auto const spot{editor_track::drive_spot_of(*track)};
		// a drive not described for the editor has no parameters known to set it with, nor a height above the rail; it stays put
		if (found == drives->end() || found->second.family < 0 || false == spot.has_value())
			continue;
		auto const &drive{found->second.standing};
		auto const &included{scene::Layers.instance(drive.instance)};
		include_info info;
		std::string error;
		if (included.file == nullptr || false == editor_includes::load(*included.file, info, error))
			continue;
		auto const local{spot->location + glm::dvec3{0.0, m_switch.drives[found->second.family].heights[found->second.index], 0.0} - included.context.offset};
		auto values{drive.values};
		for (auto const &parameter : info.parameters)
		{
			if (parameter.id < 1 || parameter.id > static_cast<int>(values.size()))
				continue;
			if (auto const value{pose_value(parameter.role, local, *spot)})
				values[parameter.id - 1] = *value;
		}
		auto const directive{editor_includes::compose_directive(*included.file, values)};
		if (directive != scene::Layers.directive(drive.instance))
			set_include_directive(drive.instance, directive);
	}
}

std::string editor_mode::missing_drive_variant(int const Family, bool const Rightswitch, bool const Driveright) const
{
	return format(Rightswitch ? STR_C("The drive %s has no variant for a right-hand switch with the drive %s") : STR_C("The drive %s has no variant for a left-hand switch with the drive %s"), m_switch.drives[Family].name.c_str(), Driveright ? STR_C("on the right") : STR_C("on the left"));
}

bool editor_mode::place_switch_drive(TTrack &Switch)
{
	auto &tool{m_switch};
	scan_switch_drives();
	if (tool.drive < 0 || tool.drive >= static_cast<int>(tool.drives.size()) || Switch.eType != tt_Switch || Switch.m_paths.size() < 2)
		return false;
	auto const spot{editor_track::drive_spot_of(Switch)};
	if (false == spot.has_value())
		return false;
	bool const driveright{(tool.drive_side == 0) == spot->rightswitch};
	auto const &family{tool.drives[tool.drive]};
	auto const index{(spot->rightswitch ? 2 : 0) + (driveright ? 1 : 0)};
	template_item item;
	item.file = family.files[index];
	if (item.file.empty())
	{
		tool.status = refused(missing_drive_variant(tool.drive, spot->rightswitch, driveright));
		return false;
	}
	item.location = spot->location + glm::dvec3{0.0, family.heights[index], 0.0};
	item.yaw = spot->yaw;
	item.described = true;
	item.track = Switch.name();
	item.tilt = spot->tilt;
	if (auto const present{find_switch_drive(Switch)})
	{
		// one not described for the editor is kept: it may set the speed over the switch or carry its ballast too
		if (present->family < 0)
		{
			tool.status = format(STR_C("Switch %s keeps its drive %s, which isn't described for the editor"), Switch.name().c_str(), file_name(present->standing.file).c_str());
			return false;
		}
		return replace_switch_drive(Switch, tool.drive, (present->index & 1) != (driveright ? 1 : 0));
	}
	std::string error;
	if (place_templates({item}, error) == 0)
	{
		tool.status = refused(STR_C("The drive couldn't be placed: ") + error);
		return false;
	}
	tool.status = format(STR_C("Switch %s with the drive %s: events %s+ and %s- switch it, also in the map"), Switch.name().c_str(), item.file.c_str(), Switch.name().c_str(), Switch.name().c_str());
	return true;
}

std::optional<editor_mode::switch_drive> editor_mode::find_switch_drive(TTrack const &Switch, bool const Wait)
{
	auto const &drives{switch_drives_standing(Wait)};
	auto const found{drives.find(drive_file(Switch.name()))};
	if (found == drives.end())
		return std::nullopt;
	return found->second;
}

// each include is read from its scenery file, so the drives of all the switches are read at once: done for each switch, the start of a
// scenery of a few hundred took a quarter of a minute. Kept for the frame, while no include comes or goes; the first drive of a switch counts
std::map<std::string, editor_mode::switch_drive> const &editor_mode::switch_drives_standing(bool const Wait)
{
	auto const &tool{m_switch};
	scan_switch_drives(Wait);
	auto &kept{m_drives_standing};
	auto const instances{static_cast<std::size_t>(scene::Layers.instance_count())};
	if (kept.has_value() && kept->frame == ImGui::GetFrameCount() && kept->revision == tool.drives_revision && kept->instances == instances &&
	    std::none_of(kept->by_switch.begin(), kept->by_switch.end(), [](auto const &Drive) {
		    auto const &included{scene::Layers.instance(Drive.second.standing.instance)};
		    return included.removed || included.dead;
	    }))
		return kept->by_switch;
	auto const variants{drive_variants()};
	kept.emplace();
	kept->frame = ImGui::GetFrameCount();
	kept->revision = tool.drives_revision;
	kept->instances = instances;
	for (auto const &standing : standing_templates([&](std::string const &File) { return false == drive_switches(File, variants).empty(); }))
	{
		auto const variant{variants.find(standing.file)};
		auto const [family, index]{variant != variants.end() ? variant->second : std::pair<int, int>{-1, -1}};
		// the drive of a double switch of Rainsted stands for each of its switches, <name>a..<name>d
		for (auto const &suffix : drive_switches(standing.file, variants))
		{
			// a drive described for the editor goes before one of Rainsted set to the same switch, which the editor kept beside it
			auto const [entry, fresh]{kept->by_switch.try_emplace(drive_file(standing.values[0] + suffix), switch_drive{standing, family, index})};
			if (false == fresh && entry->second.family < 0 && family >= 0)
				entry->second = switch_drive{standing, family, index};
		}
	}
	return kept->by_switch;
}

std::vector<std::string> const &editor_mode::drive_switches(std::string const &File, std::map<std::string, std::pair<int, int>, std::less<>> const &Variants)
{
	static std::vector<std::string> const single{std::string{}};
	if (Variants.contains(File))
		return single;
	auto const [found, fresh]{m_drive_templates.try_emplace(File)};
	if (fresh)
		editor_includes::switched_tracks(File, found->second);
	return found->second;
}

std::map<std::string, std::pair<int, int>, std::less<>> editor_mode::drive_variants() const
{
	auto const &tool{m_switch};
	std::map<std::string, std::pair<int, int>, std::less<>> variants;
	for (int family = 0; family < static_cast<int>(tool.drives.size()); ++family)
		for (int index = 0; index < 4; ++index)
			if (false == tool.drives[family].files[index].empty())
				variants[drive_file(tool.drives[family].files[index])] = {family, index};
	return variants;
}

void editor_mode::drives_changed()
{
	m_drives_standing.reset();
	m_drives_named.reset();
}

std::vector<editor_mode::stray_drive> editor_mode::stray_drives(bool const Wait)
{
	scan_switch_drives(Wait);
	auto const instances{static_cast<std::size_t>(scene::Layers.instance_count())};
	auto &kept{m_drives_named};
	if (false == kept.has_value() || kept->revision != m_switch.drives_revision || kept->instances != instances)
	{
		auto const variants{drive_variants()};
		kept.emplace();
		kept->revision = m_switch.drives_revision;
		kept->instances = instances;
		for (auto const &standing : standing_templates([&](std::string const &File) { return false == drive_switches(File, variants).empty(); }))
		{
			stray_drive drive{standing.instance, {}, {}, standing.file, standing.location};
			for (auto const &suffix : drive_switches(standing.file, variants))
				drive.switches.push_back(standing.values[0] + suffix);
			kept->drives.push_back(std::move(drive));
		}
	}
	// a switch is told by the name, as the events of the drive find it in the game; a drive of a double switch is stray with all of its
	// switches gone, while one stands it still throws it
	std::vector<stray_drive> stray;
	for (auto const &drive : kept->drives)
	{
		auto const &included{scene::Layers.instance(drive.instance)};
		// a gate, a derailer or a turntable of the same kind of template throws a track of another kind: only a track gone counts
		if (included.removed || included.dead || std::any_of(drive.switches.begin(), drive.switches.end(), [](std::string const &Name) {
			    auto const *track{track_named(Name)};
			    return track != nullptr && false == track->m_editorremoved;
		    }))
			continue;
		std::string missing;
		for (auto const &name : drive.switches)
			missing += (missing.empty() ? "" : ", ") + name;
		stray.push_back(drive);
		stray.back().name = std::move(missing);
	}
	return stray;
}

void editor_mode::render_stray_drives()
{
	auto const stray{stray_drives()};
	if (stray.empty())
		return;
	ImGui::PushID("straydrives");
	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.55f, 0.25f, 1.0f));
	auto const open{ImGui::TreeNode("##straydrives", STR_P("%zu drive with no switch", "%zu drives with no switch", static_cast<long long>(stray.size())), stray.size())};
	ImGui::PopStyleColor();
	item_tooltip("Drives set to a switch the scenery hasn't got, left when a switch was deleted or renamed by hand or in an earlier editor: their events throw nothing.\nA click goes to the drive and selects it, Delete removes it; or give a switch its name");
	if (open)
	{
		glm::dvec3 const camera{Global.pCamera.Pos};
		for (auto const &drive : stray)
		{
			auto const label{format("%s  (%s, %.0f m)##%d", drive.name.c_str(), file_name(drive.file).c_str(), glm::length(drive.location - camera), static_cast<int>(drive.instance))};
			if (ImGui::Selectable(label.c_str(), m_instance == drive.instance))
			{
				select_include(drive.instance);
				fly_to(drive.location, glm::normalize(glm::dvec3{-1.0, -0.8, -1.0}), 25.0);
			}
		}
		ImGui::TreePop();
	}
	ImGui::PopID();
}

// a red mark by each drive with no switch in sight, so it's seen in the view and over the photo, not only in the list
void editor_mode::draw_stray_drives()
{
	auto const stray{stray_drives()};
	if (stray.empty())
		return;
	// the ones far off would only crowd the view of the whole line
	auto constexpr range{3000.0};
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (auto const &drive : stray)
	{
		ImVec2 at;
		if (glm::length(drive.location - camera) > range || false == projection.project(drive.location, at))
			continue;
		auto const selected{m_instance == drive.instance};
		drawlist->AddCircleFilled(at, 8.0f, IM_COL32(0, 0, 0, 170));
		drawlist->AddCircle(at, 7.0f, selected ? overlay_color::selected : overlay_color::invalid, 0, 3.0f);
		auto const text{format(STR_C("%s: no switch of this name"), drive.name.c_str())};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		ImVec2 const corner{at.x + 12.0f, at.y - 0.5f * size.y};
		drawlist->AddRectFilled({corner.x - 3.0f, corner.y - 1.0f}, {corner.x + size.x + 3.0f, corner.y + size.y + 1.0f}, IM_COL32(0, 0, 0, 160), 3.0f);
		drawlist->AddText(corner, selected ? overlay_color::selected : overlay_color::invalid, text.c_str());
	}
}

void editor_mode::apply_switch_drive_choice(TTrack &Switch)
{
	if (m_switch.drive >= 0)
	{
		place_switch_drive(Switch);
		return;
	}
	auto const found{find_switch_drive(Switch)};
	if (found.has_value() && found->family < 0)
		m_switch.status = refused(format(STR_C("The drive %s isn't described for the editor: it may also set the speed or carry the ballast of the switch, so it's kept as it is"), file_name(found->standing.file).c_str()));
	else if (found.has_value() && remove_templates({found->standing.instance}) > 0)
		m_switch.status = format(STR_C("Switch %s without a drive now"), Switch.name().c_str());
}

bool editor_mode::flip_switch_drive(TTrack &Switch)
{
	return replace_switch_drive(Switch, -1, true);
}

bool editor_mode::replace_switch_drive(TTrack &Switch, int const Family, bool const Flip)
{
	auto &tool{m_switch};
	auto const present{find_switch_drive(Switch)};
	if (false == present.has_value())
	{
		tool.status = refused(format(STR_C("No drive of the switch %s among the described drives"), Switch.name().c_str()));
		return false;
	}
	auto const &found{present->standing};
	// a drive not described for the editor, as one of a double switch, may set the speed over the switch or carry its ballast too: it's kept as it is
	if (present->family < 0)
	{
		tool.status = refused(format(STR_C("The drive %s isn't described for the editor: it may also set the speed or carry the ballast of the switch, so it's kept as it is"), file_name(found.file).c_str()));
		return false;
	}
	auto const family{Family >= 0 ? Family : present->family};
	if (family >= static_cast<int>(tool.drives.size()))
		return false;
	auto const index{Flip ? present->index ^ 1 : present->index};
	template_item item;
	item.file = tool.drives[family].files[index];
	if (item.file.empty())
	{
		tool.status = refused(Flip ? format(STR_C("The drive %s has no variant with the drive on the other side"), tool.drives[family].name.c_str()) : missing_drive_variant(family, (index & 2) != 0, (index & 1) != 0));
		return false;
	}
	if (drive_file(item.file) == found.file)
		return false;
	auto const spot{editor_track::drive_spot_of(Switch)};
	item.location = spot.has_value() ? spot->location + glm::dvec3{0.0, tool.drives[family].heights[index], 0.0} : found.location;
	item.yaw = found.yaw;
	item.described = true;
	item.track = Switch.name();
	item.tilt = {angle_value(found.values, 5), angle_value(found.values, 6)};
	if (remove_templates({found.instance}) == 0)
	{
		tool.status = refused(STR_C("The drive of this switch can't be changed in its file"));
		return false;
	}
	std::string error;
	if (place_templates({item}, error) == 0)
	{
		scene::Layers.removed(found.instance, false);
		m_history.pop_back();
		tool.status = refused(STR_C("The drive couldn't be placed: ") + error);
		return false;
	}
	join_last_steps();
	tool.status = format(index & 1 ? STR_C("Switch %s: the drive %s, on the right now") : STR_C("Switch %s: the drive %s, on the left now"), Switch.name().c_str(), item.file.c_str());
	return true;
}

std::string editor_mode::undescribed_drive(std::vector<TTrack *> const &Switches)
{
	// the drives described for the editor read first: before they are, every drive would pass for one not described
	scan_switch_drives(true);
	std::vector<std::string> names;
	for (auto const *track : Switches)
		if (track != nullptr)
			names.push_back(drive_file(track->name()));
	auto const variants{drive_variants()};
	for (auto const &standing : standing_templates([&](std::string const &File) { return false == variants.contains(File) && false == drive_switches(File, variants).empty(); }))
	{
		auto const &suffixes{drive_switches(standing.file, variants)};
		if (std::any_of(suffixes.begin(), suffixes.end(), [&](std::string const &Suffix) { return std::find(names.begin(), names.end(), drive_file(standing.values[0] + Suffix)) != names.end(); }))
			return file_name(standing.file);
	}
	return {};
}

void editor_mode::drives_deleted(std::vector<TTrack *> const &Switches)
{
	if (m_history.empty())
		return;
	// the names of the switches deleted, as the game finds them; they're retired already
	std::vector<std::string> deleted;
	for (auto const *track : Switches)
		if (track != nullptr && track->eType == tt_Switch)
			deleted.push_back(drive_file(track->name()));
	if (deleted.empty())
		return;
	auto const named = [&](std::string const &Name) { return std::find(deleted.begin(), deleted.end(), drive_file(Name)) != deleted.end(); };
	// an older scenery may have two tracks of a name, the name finding the later; the earlier one, left by the deletion of the other,
	// still has the drive and the ballast, so it's looked for too
	auto const stands = [](std::string const &Name) {
		if (auto const *track{track_named(Name)}; track != nullptr)
			return false == track->m_editorremoved;
		auto const name{drive_file(Name)};
		auto const &tracks{simulation::Paths.sequence()};
		return std::any_of(tracks.begin(), tracks.end(), [&](TTrack const *Track) { return Track != nullptr && false == Track->m_editorremoved && drive_file(Track->name()) == name; });
	};
	auto const take_away = [&](scene::instance_handle const Instance) {
		if (false == scene::Layers.removable(Instance))
			return;
		scene::Layers.removed(Instance, true);
		m_history.back().instances.push_back(Instance);
	};
	auto const variants{drive_variants()};
	// each drive throwing a switch deleted, also one the editor kept beside a drive described for it; the drive of a double switch throws
	// the others too, so it goes with the last of them
	for (auto const &standing : standing_templates([&](std::string const &File) { return false == drive_switches(File, variants).empty(); }))
	{
		auto const &suffixes{drive_switches(standing.file, variants)};
		if (std::any_of(suffixes.begin(), suffixes.end(), [&](std::string const &Suffix) { return named(standing.values[0] + Suffix); }) &&
		    std::none_of(suffixes.begin(), suffixes.end(), [&](std::string const &Suffix) { return stands(standing.values[0] + Suffix); }))
			take_away(standing.instance);
	}
	// the sleepers and the ballast laid by a switch, named by it as its drive is: of a double switch by the name its switches have in
	// common, <name>a..<name>d or <name>_a..<name>_d, so they go with the last of them
	static std::array<char const *, 9> const parts{"", "a", "b", "c", "d", "_a", "_b", "_c", "_d"};
	std::map<std::string, bool, std::less<>> beds;
	for (auto const &standing : standing_templates([](std::string const &) { return true; }))
	{
		auto const &name{standing.values[0]};
		// named by a single switch, it's this switch's alone; a switch named <name>a beside it may be another single one
		if (named(name))
		{
			if (stands(name))
				continue;
		}
		else if (std::none_of(parts.begin(), parts.end(), [&](char const *Part) { return named(name + Part); }) ||
		         std::any_of(parts.begin(), parts.end(), [&](char const *Part) { return stands(name + Part); }))
			continue;
		auto const [bed, fresh]{beds.try_emplace(standing.file)};
		if (fresh)
			bed->second = drive_switches(standing.file, variants).empty() && editor_includes::switch_bed(standing.file);
		if (bed->second)
			take_away(standing.instance);
	}
}

std::size_t editor_mode::drives_to_blade_tips()
{
	// the drives the earlier versions of the editor set at the joint before the blades among them
	std::vector<TTrack *> switches;
	std::vector<std::pair<scene::instance_handle, std::string>> before;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch)
			continue;
		if (auto const found{find_switch_drive(*track)}; found.has_value() && found->family >= 0 && scene::Layers.removable(found->standing.instance))
		{
			switches.push_back(track);
			before.emplace_back(found->standing.instance, scene::Layers.directive(found->standing.instance));
		}
	}
	switch_drives_committed(switches);
	return static_cast<std::size_t>(std::count_if(before.begin(), before.end(), [](auto const &Before) { return scene::Layers.directive(Before.first) != Before.second; }));
}

bool editor_mode::render_switch_drive_choice()
{
	auto &tool{m_switch};
	scan_switch_drives(false);
	if (tool.drives.empty())
	{
		ImGui::TextDisabled("%s", EditorIncludes.searching() ? STR_C("Looking for the templates...") : STR_C("No switch drive models"));
		return false;
	}
	auto const label = [&](int const Index) { return Index < 0 ? std::string{STR_C("no drive")} : tool.drives[Index].name; };
	bool changed{false};
	if (false == begin_fields("##drivefields", 0.3f))
		return false;
	field_row("Drive");
	if (ImGui::BeginCombo("##drive", label(tool.drive).c_str()))
	{
		for (int i = -1; i < static_cast<int>(tool.drives.size()); ++i)
			if (ImGui::Selectable(label(i).c_str(), tool.drive == i))
			{
				tool.drive = i;
				changed = true;
			}
		ImGui::EndCombo();
	}
	ImGui::EndTable();
	item_tooltip("Templates described with category: switch drive, the side of the switch and of the drive, and a parameter with the role track for the name of the switch.\nIts events <name>+ and <name>- switch it with the keys by the switch and in the map during the game");
	if (tool.drive >= 0)
		changed |= segmented("driveside", tool.drive_side, {{0, STR_C("diverging side")}, {1, STR_C("other side")}});
	return changed;
}

// a key by the movable nose of each switch with a drive, the frog is far from the drive at the start of the switch
bool editor_mode::nose_controls(std::vector<std::string> &Rootstatements)
{
	if (scene::Layers.empty())
		return false;
	std::string root{scene::Layers.layer(scene::layer_handle{1}).name};
	if (auto const slash{root.find_last_of("/\\")}; slash != std::string::npos)
		root.erase(0, slash + 1);
	if (auto const dot{root.rfind('.')}; dot != std::string::npos)
		root.erase(dot);
	if (root.empty())
		return false;
	auto const file{root + "_dzioby.scm"};
	std::ostringstream text;
	auto noses{0};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->SwitchExtension == nullptr || false == track->SwitchExtension->m_movablenose || track->name().empty())
			continue;
		auto const &segments{track->SwitchExtension->Segments};
		auto const frog{track->frog()};
		if (false == frog.valid || segments[0] == nullptr || frog.angle <= 0.0 || false == find_switch_drive(*track).has_value())
			continue;
		auto const &name{track->name()};
		// where the inner rails cross, on the gauge line of the main path
		glm::dvec3 point, across;
		segments[0]->plan_frame(frog.at[0], point, across);
		auto const nose{point + across * (frog.side[0] * 0.5 * std::abs(track->fTrackWidth))};
		auto const range{5.0 + track_rails::movable_nose_length(std::tan(frog.angle))};
		text << "node -1 0 " << name << "_dziob eventlauncher " << format("%.3f %.3f %.3f %.1f", nose.x, nose.y, nose.z, range) << " t 0 " << name << "+ " << name << "- end\r\n";
		++noses;
	}
	auto const path{Global.asCurrentSceneryPath + file};
	std::error_code error;
	auto const exists{std::filesystem::exists(path, error)};
	if (noses == 0 && false == exists)
		return false;
	auto const content{std::string{kNoseHeader} + "\r\n" + text.str()};
	std::string current;
	{
		std::ifstream input{path, std::ios_base::binary};
		current.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
	}
	if (current != content)
	{
		std::ofstream output{path, std::ios_base::binary | std::ios_base::trunc};
		output << content;
	}
	// the keys written for this scenery under another name, the file it was copied from, give way to these: with both included
	// each key would stand twice. their files stay on the disk, for the scenery they were made for
	for (std::size_t handle = 1; handle <= scene::Layers.size(); ++handle)
	{
		auto const layer{static_cast<scene::layer_handle>(handle)};
		if (false == scene::Layers.listed(layer) || scene::Layers.layer(layer).removed)
			continue;
		auto name{scene::Layers.layer(layer).name};
		std::replace(name.begin(), name.end(), '\\', '/');
		auto const lower{ToLower(name)};
		if (lower == ToLower(file) || lower.ends_with("/" + ToLower(file)) || false == lower.ends_with("_dzioby.scm"))
			continue;
		if (nose_file(Global.asCurrentSceneryPath + scene::Layers.layer(layer).name) && scene::Layers.remove(layer))
			WriteLog("Editor: " + name + " left out of the scenery, the keys by the movable noses are in " + file + " now");
	}
	auto included{m_nose_included};
	for (std::size_t handle = 1; handle <= scene::Layers.size() && false == included; ++handle)
	{
		auto name{scene::Layers.layer(static_cast<scene::layer_handle>(handle)).name};
		std::replace(name.begin(), name.end(), '\\', '/');
		included = ToLower(name) == ToLower(file) || ToLower(name).ends_with("/" + ToLower(file));
	}
	if (included)
		return false;
	Rootstatements.push_back("include " + file + " end");
	return true;
}

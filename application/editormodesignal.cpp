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
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"
#include "editor/editorSettings.hpp"
#include "editor/editorTrackTrace.hpp"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "rendering/renderer.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "world/Event.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <cctype>
#include <cfloat>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <ranges>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace
{

using geometry::bezier;
using geometry::heading_of;
using geometry::plan_of;

double const kRailTop{0.18};
// m, a signal lowered at most so far onto the ground under it; more is no ground of its own, under a bridge
double const kGroundReach{3.0};
double const kCarryRaise{3.0}; // m, of a signal over where it stands by default, kept as it is dragged; more is of another level
double const kCarryNearest{1.0}; // m, of the distance of a dragged signal from the axis kept, outside it the default one
double const kCarryFarthest{10.0};
double const kSignalRange{1500.0};
double const kSignalHead{4.0};
float const kSignalPick{12.0f};
float const kDragStart{6.0f};
double const kDragTravel{1.0};
double const kRefreshPeriod{2.0};
char const *const kDrawnPlate{"make:signal_plate?"}; // the plate drawn by python/local/signal_plate.py
// looks of the drawn plates, in the order of the setting of the editor
char const *const kPlateLooks[] = {"plain", "toned", "aged"};
char const *const kPlateLookLabels[] = {"plain", "toned", "aged"};
int const kPlateLookCount{3};
double const kSignalReach{10.0};
double const kTrackNearer{0.5}; // m another track has to be nearer by to take a signal moved from its own
double const kOnTrack{1.2}; // m from the axis of a track a signal dropped stands on it rather than by it
double const kDangerReach{1500.0}; // m ahead of a signal the switch it is measured to is looked for
double const kSnapStep{50.0}; // m the distance of a signal to the switch ahead of it is rounded to
double const kSnapReach{2.5}; // m from a round distance a signal dragged or placed goes to it
ImU32 const kWayColour{IM_COL32(90, 200, 255, 235)};
double const kWayAside{1.3}; // m from the axis of the track the way of a signal to the switch ahead is drawn, on its side
// a track laid in the place of a deleted one runs within it of its axis
double const kOrphanReach{1.0};
std::size_t const kOrphanRecords{64};
// the paths reading a signal, drawn in the view and named in its panel
ImU32 const kReadColour{IM_COL32(255, 90, 220, 255)};
char const *const kOtherPath{"On this side the signal would stand on the other path of the switch: place it on the outer side"};

char const *const kKinds[] = {"main", "block", "warning", "shunt", "stop", "substitute", "repeater", "crossing"};
char const *const kKindLabels[] = {"Semaphore", "Automatic block", "Warning signal", "Shunting signal", "Stop signal", "Substitute signal", "Repeater", "Level crossing disc"};
int const kKindCount{8};

int kind_index(std::string const &Kind)
{
	for (int i = 0; i < kKindCount; ++i)
		if (Kind == kKinds[i])
			return i;
	return -1;
}

std::string no_template(std::string const &Kind)
{
	return format(STR_C("no template of the kind %s"), STR(kKindLabels[std::max(0, kind_index(Kind))]).c_str());
}

// a shunting or stop disc shown by its shape, no lights: the signalling of the station knows it, but doesn't place it by itself
bool shaped_disc(std::string_view const Kind, std::string_view const Lamps, int const Arms)
{
	return (Kind == "shunt" || Kind == "stop") && Lamps.empty() && Arms == 0;
}

char const *mount_label(std::string_view const Mount)
{
	if (Mount == "dwarf")
		return STR_C("dwarf");
	if (Mount == "gantry")
		return STR_C("gantry");
	return STR_C("mast");
}

double constexpr kGaugeSlack{0.01}; // m, an entry into the gauge this shallow is let be
double constexpr kGaugeMargin{0.03}; // m, kept from the outline by a signal taken off it
int constexpr kGaugeSteps{4}; // of taking a signal off the gauges, between two tracks too close it can't keep out of both
double constexpr kGaugeLevel{2.0}; // m, of a track from the foot of the signal, beyond it the signal isn't taken aside from it

// the models of the include standing around the place
std::vector<TAnimModel *> include_models(scene::instance_handle const Instance, glm::dvec3 const &Location)
{
	std::vector<TAnimModel *> models;
	for (auto *model : simulation::Instances.sequence())
		if (model != nullptr && model->m_instance == Instance && model->visible() && glm::distance(model->location(), Location) < 60.0)
			models.push_back(model);
	return models;
}

double constexpr kIndicatorStep{0.45}; // m, from one indicator under the lights to the next one down, each 0.42 m high
double constexpr kIndicatorUnder{0.73}; // m, from the lowest lamp down to the first indicator, as the signals with one built in have it
double constexpr kIndicatorHead{3.0}; // m, over the foot, of the first indicator of a signal with no lamp found
double constexpr kIndicatorAhead{0.05}; // m, of an indicator under the lights before the mast
double constexpr kIndicatorReach{1.0}; // m, in the plan, of an indicator named after a signal from its foot to be counted as its

// the place of an indicator under the lights of a signal, from the top: Ie-117 § 12.1 orders them W21, W24, W20, W19, W26a or W26b;
// the others under them
int indicator_rank(std::string const &Sign)
{
	auto const sign{ToLower(Sign)};
	if (sign == "w21")
		return 0;
	if (sign == "w24")
		return 1;
	if (sign == "w20")
		return 2;
	if (sign == "w19")
		return 3;
	if (sign.rfind("w26", 0) == 0)
		return 4;
	return 5;
}

glm::dmat4 placement_of(TAnimModel const &Instance)
{
	auto const angles{Instance.Angles()};
	auto matrix{glm::translate(glm::dmat4(1.0), Instance.location())};
	matrix = glm::rotate(matrix, glm::radians(static_cast<double>(angles.y)), glm::dvec3(0.0, 1.0, 0.0));
	matrix = glm::rotate(matrix, glm::radians(static_cast<double>(angles.x)), glm::dvec3(1.0, 0.0, 0.0));
	matrix = glm::rotate(matrix, glm::radians(static_cast<double>(angles.z)), glm::dvec3(0.0, 0.0, 1.0));
	return glm::scale(matrix, glm::dvec3(Instance.Scale()));
}

// the model of the include with the lights, else the first one
TAnimModel *main_model(std::vector<TAnimModel *> const &Models)
{
	auto const lit{std::find_if(Models.begin(), Models.end(), [](TAnimModel const *Model) { return Model->iNumLights > 0; })};
	return lit != Models.end() ? *lit : Models.empty() ? nullptr : Models.front();
}

// the rotation of the signal, of its model
glm::dmat3 frame_of(std::vector<TAnimModel *> const &Models)
{
	auto const *model{main_model(Models)};
	if (model == nullptr)
		return glm::dmat3(1.0);
	auto const angles{model->Angles()};
	auto matrix{glm::rotate(glm::dmat4(1.0), glm::radians(static_cast<double>(angles.y)), glm::dvec3(0.0, 1.0, 0.0))};
	matrix = glm::rotate(matrix, glm::radians(static_cast<double>(angles.x)), glm::dvec3(1.0, 0.0, 0.0));
	matrix = glm::rotate(matrix, glm::radians(static_cast<double>(angles.z)), glm::dvec3(0.0, 0.0, 1.0));
	return glm::dmat3(matrix);
}

// the lowest point of the lamps among the submodels, the lamp ones and those under them
void lamp_points(TSubModel *Submodel, glm::dmat4 const &Matrix, bool const Lamp, std::unordered_set<TSubModel const *> const &Lamps, double &Lowest)
{
	for (auto *submodel = Submodel; submodel != nullptr; submodel = submodel->NextGet())
	{
		auto matrix{Matrix};
		if ((submodel->iFlags & 0xC000) && submodel->GetMatrix() != nullptr)
			matrix *= glm::dmat4(glm::make_mat4(submodel->GetMatrix()->readArray()));
		auto const lamp{Lamp || Lamps.count(submodel) > 0};
		if (lamp && submodel->eType == GL_TRIANGLES && submodel->m_geometry.handle != null_handle)
		{
			for (auto const &vertex : GfxRenderer->Vertices(submodel->m_geometry.handle))
				Lowest = std::min(Lowest, (matrix * glm::dvec4(glm::dvec3(vertex.position), 1.0)).y);
		}
		else if (lamp)
			Lowest = std::min(Lowest, matrix[3].y);
		lamp_points(submodel->ChildGet(), matrix, lamp, Lamps, Lowest);
	}
}

// the height of the lowest lamp of the models, in the world; the highest double when they have none
double lowest_lamp(std::vector<TAnimModel *> const &Models)
{
	auto lowest{std::numeric_limits<double>::max()};
	for (auto *model : Models)
	{
		if (model == nullptr || model->iNumLights <= 0 || model->Model() == nullptr || model->Model()->GetSMRoot() == nullptr)
			continue;
		std::unordered_set<TSubModel const *> lamps;
		for (std::size_t i = 0; i < static_cast<std::size_t>(model->iNumLights) && i < model->LightsOn.size(); ++i)
		{
			lamps.insert(model->LightsOn[i]);
			lamps.insert(model->LightsOff[i]);
		}
		lamps.erase(nullptr);
		lamp_points(model->Model()->GetSMRoot(), placement_of(*model), false, lamps, lowest);
	}
	return lowest;
}

double nearest_on(TTrack const &Track, int const Path, glm::dvec3 const &Point)
{
	return track_trace::parameter_of(Track, Path, Point);
}

// the way to the switch ahead in words: how far, to what of which switch
std::string danger_text(track_trace::danger const &Danger)
{
	using kind = track_trace::danger::kind;
	if (Danger.what == kind::none || Danger.track == nullptr)
		return format(STR_C("no switch within %.0f m"), kDangerReach);
	auto const name{name_of(*Danger.track)};
	if (Danger.distance < 0.0)
		return format(STR_C("%.1f m inside the fouling zone of %s"), -Danger.distance, name.c_str());
	return format(Danger.what == kind::start ? STR_C("%.1f m to the start of %s") : STR_C("%.1f m to the fouling point of %s"), Danger.distance, name.c_str());
}

glm::dvec2 forward_at(TTrack const &Track, int const Path, double const Parameter)
{
	auto const tangent{bezier{Track.m_paths[Path]}.first(Parameter)};
	glm::dvec2 const planar{tangent.x, tangent.z};
	return glm::length(planar) > 1e-9 ? glm::normalize(planar) : glm::dvec2{0.0, 1.0};
}

glm::dvec2 right_of(glm::dvec2 const &Direction)
{
	return {-Direction.y, Direction.x};
}

glm::dvec2 travel_of(double const Yaw)
{
	auto const yaw{glm::radians(Yaw + 180.0)};
	return {std::sin(yaw), std::cos(yaw)};
}

int travel_side(int const Side, int const List)
{
	return Side * (List == 2 ? 1 : -1);
}

int lean_wanted(int const Side)
{
	return Side > 0 ? 1 : 2;
}

int lean_of(std::string const &Lean)
{
	return Lean == "left" ? 1 : Lean == "right" ? 2 : 0;
}

std::size_t edit_distance(std::string const &A, std::string const &B)
{
	std::vector<std::size_t> row(B.size() + 1);
	for (std::size_t j = 0; j <= B.size(); ++j)
		row[j] = j;
	for (std::size_t i = 1; i <= A.size(); ++i)
	{
		auto diagonal{row[0]};
		row[0] = i;
		for (std::size_t j = 1; j <= B.size(); ++j)
		{
			auto const above{row[j]};
			row[j] = std::min({row[j] + 1, row[j - 1] + 1, diagonal + (A[i - 1] == B[j - 1] ? 0 : 1)});
			diagonal = above;
		}
	}
	return row[B.size()];
}

ImU32 lamp_colour(char const Lamp)
{
	switch (Lamp)
	{
	case 'z': return IM_COL32(40, 210, 80, 255);
	case 'c': return IM_COL32(230, 40, 40, 255);
	case 'p': return IM_COL32(255, 160, 30, 255);
	case 'b': return IM_COL32(235, 235, 235, 255);
	case 'n': return IM_COL32(60, 110, 255, 255);
	default: return IM_COL32(70, 70, 70, 255);
	}
}

// the tile of the palette a template goes to: by its mount and its lights, the semaphores of a mount all on one, their arms and
// the substitute signal chosen under it
std::string group_of(std::string const &Mount, std::string const &Lamps, int const Arms)
{
	return Arms > 0 ? Mount + "|arms" : Mount + "|" + Lamps;
}

std::vector<std::string> const &plate_textures()
{
	static std::vector<std::string> plates;
	static bool scanned{false};
	if (scanned)
		return plates;
	scanned = true;
	std::error_code error;
	for (std::filesystem::directory_iterator file{"textures/tabl", error}, end; file != end; file.increment(error))
	{
		std::string stem;
		try
		{
			stem = ToLower(file->path().stem().string());
		}
		catch (std::exception const &)
		{
			continue;
		}
		if (std::find(plates.begin(), plates.end(), stem) == plates.end())
			plates.push_back(stem);
	}
	std::sort(plates.begin(), plates.end());
	return plates;
}

void draw_signal_icon(ImDrawList *Draw, ImVec2 const Origin, ImVec2 const Size, std::string_view const Mount, std::string const &Lamps, ImU32 const Frame, int const Arms = 0)
{
	auto const lamps{std::max<int>(1, static_cast<int>(Lamps.size()))};
	auto const centre{Origin.x + Size.x * 0.5f};
	auto const bottom{Origin.y + Size.y - 4.0f};
	auto const lamp{std::min(7.0f, (Size.y - 26.0f) / (lamps * 2.2f))};
	auto const headwidth{lamp * 2.0f + 6.0f};
	auto const headheight{lamps * lamp * 2.2f + 4.0f};
	ImU32 const pole{IM_COL32(150, 150, 150, 255)};
	float headtop;
	if (Mount == "dwarf")
	{
		headtop = bottom - headheight;
		Draw->AddRectFilled(ImVec2(centre - headwidth * 0.7f, bottom - 3.0f), ImVec2(centre + headwidth * 0.7f, bottom), pole);
	}
	else if (Mount == "gantry")
	{
		headtop = Origin.y + 10.0f;
		Draw->AddLine(ImVec2(Origin.x + 4.0f, Origin.y + 6.0f), ImVec2(Origin.x + Size.x - 4.0f, Origin.y + 6.0f), pole, 3.0f);
		Draw->AddLine(ImVec2(centre, Origin.y + 6.0f), ImVec2(centre, headtop), pole, 2.0f);
	}
	else
	{
		headtop = Origin.y + 6.0f;
		Draw->AddLine(ImVec2(centre, headtop + headheight), ImVec2(centre, bottom), pole, 3.0f);
		Draw->AddLine(ImVec2(centre - 6.0f, bottom), ImVec2(centre + 6.0f, bottom), pole, 2.0f);
	}
	if (Lamps.empty())
	{
		// a semaphore with its arms, one under the other; a disc or one not described with one
		Draw->AddLine(ImVec2(centre, headtop), ImVec2(centre, headtop + headheight), pole, 3.0f);
		for (int arm = 0; arm < std::max(1, Arms); ++arm)
			Draw->AddLine(ImVec2(centre, headtop + 6.0f + static_cast<float>(arm) * 14.0f), ImVec2(centre + 14.0f, headtop - 2.0f + static_cast<float>(arm) * 14.0f), IM_COL32(230, 60, 60, 255), 4.0f);
		return;
	}
	Draw->AddRectFilled(ImVec2(centre - headwidth * 0.5f, headtop), ImVec2(centre + headwidth * 0.5f, headtop + headheight), IM_COL32(25, 25, 25, 255), 3.0f);
	Draw->AddRect(ImVec2(centre - headwidth * 0.5f, headtop), ImVec2(centre + headwidth * 0.5f, headtop + headheight), Frame, 3.0f);
	for (int i = 0; i < static_cast<int>(Lamps.size()); ++i)
		Draw->AddCircleFilled(ImVec2(centre, headtop + 2.0f + lamp * 1.1f + i * lamp * 2.2f), lamp, lamp_colour(Lamps[i]));
}


// the point of an axis of the track nearest in the plan to the one given: how far, on which path, where on it
double plan_gap(TTrack const &Track, glm::dvec3 const &Point, int &Path, double &Parameter)
{
	auto best{std::numeric_limits<double>::max()};
	for (int path = 0; path < static_cast<int>(Track.m_paths.size()); ++path)
	{
		auto const parameter{nearest_on(Track, path, Point)};
		auto const distance{geometry::plan_distance(bezier{Track.m_paths[path]}.point(parameter), Point)};
		if (distance < best)
		{
			best = distance;
			Path = path;
			Parameter = parameter;
		}
	}
	return best;
}

// the path and its list that read a signal standing by a path of the track, at the point of its axis, for the trains running along the
// path (list 2) or against it (list 1). the track itself, but a switch reads its events to the trains of both its paths: past the frog,
// where the axes of the paths are a gauge apart, the trains of the other path don't pass by the signal any more, so its read goes to the
// track joined to the far end of its path, run on by the trains of that path only, in the list of the trains running from the switch onto
// it or from it to the switch. the trains measure the way back to the signal from there
std::pair<TTrack *, int> reader_at(TTrack &Track, int const Path, glm::dvec3 const &Axis, int const List)
{
	if (Track.eType != tt_Switch || Track.m_paths.size() != 2 || Path < 0 || Path > 1)
		return {&Track, List};
	auto const other{1 - Path};
	if (geometry::plan_distance(bezier{Track.m_paths[other]}.point(nearest_on(Track, other, Axis)), Axis) < Track.fTrackWidth)
		return {&Track, List};
	auto const &own{Track.m_paths[Path]};
	auto const &theirs{Track.m_paths[other]};
	// the toe is the end both paths share
	auto const toe_at_start{glm::distance(own.points[segment_data::point::start], theirs.points[segment_data::point::start]) < 0.01 ||
	                        glm::distance(own.points[segment_data::point::start], theirs.points[segment_data::point::end]) < 0.01};
	auto const heel{own.points[toe_at_start ? segment_data::point::end : segment_data::point::start]};
	auto const outward{(List == 2) == toe_at_start};
	for (auto const &[neighbour, point] : editor_track::connected_points(Track, heel))
		if (neighbour != &Track && neighbour->eType == tt_Normal && false == neighbour->m_editorremoved && false == neighbour->m_paths.empty())
			return {neighbour, outward == (point.kind == editor_track::point_kind::start) ? 2 : 1};
	return {&Track, List};
}

// the foot of a signal by a path of a switch, on the side of its other path, nearer to the axis of that one than it stands off its own:
// the signal would stand on the other path, in the way of its trains
bool on_other_path(TTrack const &Track, int const Path, glm::dvec3 const &Foot, double const Offset)
{
	if (Track.eType != tt_Switch || Track.m_paths.size() != 2 || Path < 0 || Path > 1 || Offset <= 0.0)
		return false;
	auto const other{1 - Path};
	return geometry::plan_distance(bezier{Track.m_paths[other]}.point(nearest_on(Track, other, Foot)), Foot) < Offset - 0.01;
}

bool plan_overlaps(TTrack const &A, TTrack const &B, double const Margin)
{
	auto const box = [](TTrack const &Track, glm::dvec2 &Low, glm::dvec2 &High) {
		Low = glm::dvec2{std::numeric_limits<double>::max()};
		High = -Low;
		for (auto const &path : Track.m_paths)
		{
			bezier const curve{path};
			for (auto const &point : {curve.p0, curve.p1, curve.p2, curve.p3})
			{
				Low = glm::min(Low, plan_of(point));
				High = glm::max(High, plan_of(point));
			}
		}
	};
	glm::dvec2 low_a, high_a, low_b, high_b;
	box(A, low_a, high_a);
	box(B, low_b, high_b);
	return low_a.x - Margin <= high_b.x && low_b.x - Margin <= high_a.x && low_a.y - Margin <= high_b.y && low_b.y - Margin <= high_a.y;
}

std::string listed(std::vector<std::string> const &Names)
{
	std::string text;
	for (auto const &name : Names)
		text += (text.empty() ? "" : ", ") + name;
	return text;
}

} // namespace

bool editor_mode::signal_scan(bool const Wait)
{
	auto &tool{m_signal};
	if (Wait)
		EditorIncludes.wait();
	else
		EditorIncludes.start();
	if (false == EditorIncludes.scanned() || tool.revision == EditorIncludes.revision())
		return false;
	tool.revision = EditorIncludes.revision();
	std::string chosen[kKindCount];
	for (int i = 0; i < kKindCount; ++i)
		if (tool.chosen[i] >= 0 && tool.chosen[i] < static_cast<int>(tool.templates.size()))
			chosen[i] = tool.templates[tool.chosen[i]].file;
	tool.templates.clear();
	tool.indicators.clear();
	for (auto const &entry : EditorIncludes.entries())
	{
		// the indicators, put on a signal named after it: the signal is its first parameter, the description needn't give it
		if (entry.category == "indicator")
		{
			include_info info;
			std::string error;
			if (editor_includes::load(entry.file, info, error) && false == info.indicator_sign.empty())
				tool.indicators.push_back({entry.file, info.name.empty() ? entry.file : info.name, info.description, info.indicator_sign, info.indicator_place == "head" ? std::string{"head"} : std::string{"foot"}, info.indicator_events});
			continue;
		}
		if (entry.category != "signal" || false == entry.complete)
			continue;
		include_info info;
		std::string error;
		if (false == editor_includes::load(entry.file, info, error) || kind_index(info.signal_kind) < 0)
			continue;
		signal_template item;
		item.file = entry.file;
		item.name = info.name.empty() ? entry.file : info.name;
		item.description = info.description;
		item.kind = info.signal_kind;
		item.mount = info.signal_mount.empty() ? std::string{"mast"} : info.signal_mount;
		item.lamps = info.signal_lamps;
		item.lean = info.signal_lean;
		item.read = info.signal_read;
		item.arms = info.signal_arms;
		item.height = info.signal_height;
		item.depth = info.signal_depth;
		item.aspect_events = info.signal_aspects;
		item.repeats = info.signal_repeats;
		for (auto const &parameter : info.parameters)
		{
			item.parameters = std::max(item.parameters, parameter.id);
			if (parameter.id == 6 && parameter.role == "texture")
				item.plate = true;
			// the signal it's linked to: a link, or a text as the older descriptions give it, not a texture or another value
			if (parameter.id == 7 && (parameter.role == "link" || parameter.role == "text"))
				item.linked = true;
		}
		editor_includes::signal_speeds(entry.file, item.speeds);
		if (item.kind == "block")
			item.aspects = editor_includes::block_aspects(entry.file);
		tool.templates.push_back(std::move(item));
	}
	std::sort(tool.indicators.begin(), tool.indicators.end(), [](indicator_template const &A, indicator_template const &B) { return std::tie(A.sign, A.name) < std::tie(B.sign, B.name); });
	std::sort(tool.templates.begin(), tool.templates.end(), [](signal_template const &A, signal_template const &B) {
		return std::make_tuple(kind_index(A.kind), A.lamps.empty(), A.mount != "mast", A.lamps.size(), A.lamps, A.name.size(), A.name) < std::make_tuple(kind_index(B.kind), B.lamps.empty(), B.mount != "mast", B.lamps.size(), B.lamps, B.name.size(), B.name);
	});
	for (int i = 0; i < kKindCount; ++i)
	{
		tool.chosen[i] = -1;
		std::unordered_map<std::string, int> sizes;
		for (auto const &item : tool.templates)
			if (item.kind == kKinds[i])
				++sizes[group_of(item.mount, item.lamps, item.arms)];
		auto best{-1};
		for (int k = 0; k < static_cast<int>(tool.templates.size()); ++k)
		{
			auto const &item{tool.templates[k]};
			if (item.kind != kKinds[i])
				continue;
			if (item.file == chosen[i])
			{
				best = k;
				break;
			}
			if (best < 0 || sizes[group_of(item.mount, item.lamps, item.arms)] > sizes[group_of(tool.templates[best].mount, tool.templates[best].lamps, tool.templates[best].arms)])
				best = k;
		}
		tool.chosen[i] = best;
	}
	return true;
}

editor_mode::signal_template const *editor_mode::signal_current() const
{
	auto const &tool{m_signal};
	if (tool.kind < 0 || tool.kind >= kKindCount)
		return nullptr;
	auto const index{tool.chosen[tool.kind]};
	return index >= 0 && index < static_cast<int>(tool.templates.size()) ? &tool.templates[index] : nullptr;
}

editor_mode::signal_template const *editor_mode::signal_armed() const
{
	return m_signal.armed ? signal_current() : nullptr;
}

editor_mode::signal_template const *editor_mode::signal_template_of(std::string const &File) const
{
	auto const file{ToLower(File)};
	for (auto const &item : m_signal.templates)
		if (item.file == file)
			return &item;
	return nullptr;
}

editor_mode::indicator_template const *editor_mode::indicator_template_of(std::string const &File) const
{
	auto const file{ToLower(File)};
	for (auto const &item : m_signal.indicators)
		if (item.file == file)
			return &item;
	return nullptr;
}

editor_mode::signal_template const *editor_mode::signal_template_for(signal_order const &Order) const
{
	auto const &tool{m_signal};
	auto const kind{kind_index(Order.kind)};
	auto const *item{kind >= 0 && tool.chosen[kind] >= 0 && tool.chosen[kind] < static_cast<int>(tool.templates.size()) ? &tool.templates[tool.chosen[kind]] : nullptr};
	if (auto const *wanted{Order.file.empty() ? nullptr : signal_template_of(Order.file)}; wanted != nullptr && wanted->kind == Order.kind)
		return wanted;
	if (item != nullptr && shaped_disc(item->kind, item->lamps, item->arms))
	{
		// a shaped disc chosen isn't placed by the signalling: a light one of the mount of it, of the tile with the most variants as
		// chosen at the start; of another mount when its mount has none
		std::map<std::string, int, std::less<>> sizes;
		for (auto const &other : tool.templates)
			if (other.kind == Order.kind && false == shaped_disc(other.kind, other.lamps, other.arms))
				++sizes[group_of(other.mount, other.lamps, other.arms)];
		auto const mount{item->mount};
		auto const rank{[&sizes, &mount](signal_template const &Item) { return std::make_pair(Item.mount == mount, sizes[group_of(Item.mount, Item.lamps, Item.arms)]); }};
		item = nullptr;
		for (auto const &other : tool.templates)
			if (other.kind == Order.kind && false == shaped_disc(other.kind, other.lamps, other.arms) && (item == nullptr || rank(other) > rank(*item)))
				item = &other;
	}
	return item;
}

std::string editor_mode::signal_templates_missing(std::vector<signal_order> const &Orders)
{
	signal_scan();
	for (auto const &order : Orders)
		if (signal_template_for(order) == nullptr)
		{
			auto const kind{kind_index(order.kind)};
			if (auto const *chosen{kind >= 0 && m_signal.chosen[kind] >= 0 && m_signal.chosen[kind] < static_cast<int>(m_signal.templates.size()) ? &m_signal.templates[m_signal.chosen[kind]] : nullptr};
			    chosen != nullptr && shaped_disc(chosen->kind, chosen->lamps, chosen->arms))
				return format(STR_C("no light template of the kind %s: a shaped disc isn't placed by the signalling, only by hand"), STR(kKindLabels[kind]).c_str());
			return no_template(order.kind);
		}
	return {};
}

bool editor_mode::signal_spot_at(TTrack &Track, int const Path, glm::dvec3 const &Point, glm::dvec3 const &Cursor, int const Facing, int const Side, signal_template const &Template, signal_spot &Spot) const
{
	if (Track.m_paths.empty() || Track.m_editorremoved)
		return false;
	auto const path{std::clamp(Path, 0, static_cast<int>(Track.m_paths.size()) - 1)};
	auto const parameter{nearest_on(Track, path, Point)};
	auto const axis{bezier{Track.m_paths[path]}.point(parameter)};
	auto const forward{forward_at(Track, path, parameter)};
	auto const right{right_of(forward)};
	auto side{Side};
	if (side == 0)
		side = glm::dot(plan_of(Cursor) - plan_of(axis), right) >= 0.0 ? 1 : -1;
	auto facing{Facing};
	if (facing == 0)
		facing = side;
	Spot.track = &Track;
	Spot.path = path;
	Spot.axis = axis;
	Spot.travel = forward * static_cast<double>(facing);
	Spot.side = side;
	Spot.list = facing > 0 ? 2 : 1;
	Spot.offset = Template.mount == "gantry" ? 0.0 : EditorSettings.signal_offset();
	auto const across{right * static_cast<double>(side) * Spot.offset};
	Spot.position = {axis.x + across.x, 0.0, axis.z + across.y};
	Spot.position.y = signal_default_height(Template, axis, Spot.position);
	auto const yaw{heading_of(Spot.travel) + 180.0};
	Spot.yaw = yaw >= 360.0 ? yaw - 360.0 : yaw;
	std::tie(Spot.reader, Spot.reader_list) = reader_at(Track, path, axis, Spot.list);
	return true;
}

std::string editor_mode::signal_intent_label(signal_template const &Template, signal_spot const &Spot) const
{
	auto const name{m_signal.name[0] != '\0' ? std::string{m_signal.name} : std::string{"?"}};
	auto const right{glm::dot(right_of(Spot.travel), plan_of(Spot.position) - plan_of(Spot.axis)) >= 0.0};
	auto const where{right ? STR(" on the right of the trains") : STR(" on the left of the trains")};
	auto variant{signal_variant(Template, Spot)};
	variant = variant.substr(0, variant.rfind('.'));
	if (on_other_path(*Spot.track, Spot.path, Spot.position, Spot.offset))
		return STR(kOtherPath);
	if (Template.read.empty())
		return format(STR_C("Click: %s %s (%s)%s, the train doesn't read it"), STR(kKindLabels[kind_index(Template.kind)]).c_str(), name.c_str(), variant.c_str(), where.c_str()) + "\n" + STR("Drag along: the trains it faces run the way of the drag");
	auto const event{editor_includes::substitute(Template.read, {name})};
	auto const path{name_of(*Spot.reader)};
	return format(STR_C("Click: %s %s (%s)%s, read by %s %s of %s"), STR(kKindLabels[kind_index(Template.kind)]).c_str(), name.c_str(), variant.c_str(), where.c_str(), editor_track::event_keyword(Spot.reader_list), event.c_str(), path.c_str()) + "\n" + STR("Drag along: the trains it faces run the way of the drag");
}

void editor_mode::signal_refresh(bool const Force)
{
	auto &tool{m_signal};
	auto const scanned{signal_scan(false)};
	auto const now{ImGui::GetTime()};
	if (false == Force && false == scanned && tool.standing_history == history_revision() && now - tool.standing_time < kRefreshPeriod)
		return;
	tool.standing_history = history_revision();
	tool.standing_time = now;
	tool.standing = signal_standing_now();
	if (tool.selected != 0 && std::none_of(tool.standing.begin(), tool.standing.end(), [&](signal_standing const &Standing) { return Standing.include.instance == tool.selected; }))
		tool.selected = 0;
}

int editor_mode::signal_hit() const
{
	screen_projection const projection;
	auto const &mouse{ImGui::GetIO().MousePos};
	auto best{kSignalPick * kSignalPick};
	int result{-1};
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (int i = 0; i < static_cast<int>(m_signal.standing.size()); ++i)
	{
		auto const &location{m_signal.standing[i].include.location};
		if (glm::distance(location, camera) > kSignalRange)
			continue;
		ImVec2 foot, head;
		if (false == projection.project(location, foot) || false == projection.project(location + glm::dvec3{0.0, kSignalHead, 0.0}, head))
			continue;
		ImVec2 const span{head.x - foot.x, head.y - foot.y};
		auto const length2{span.x * span.x + span.y * span.y};
		auto const t{length2 > 1e-6f ? std::clamp(((mouse.x - foot.x) * span.x + (mouse.y - foot.y) * span.y) / length2, 0.0f, 1.0f) : 0.0f};
		auto const dx{foot.x + span.x * t - mouse.x};
		auto const dy{foot.y + span.y * t - mouse.y};
		auto const distance2{dx * dx + dy * dy};
		if (distance2 < best)
		{
			best = distance2;
			result = i;
		}
	}
	return result;
}

void editor_mode::signal_press(track_intent const &Intent)
{
	auto &tool{m_signal};
	auto const &mouse{ImGui::GetIO().MousePos};
	tool.pressed = {mouse.x, mouse.y};
	tool.facing = 0;
	tool.status.clear();
	if (Intent.what == track_intent::kind::signal_move)
	{
		if (Intent.index < 0 || Intent.index >= static_cast<int>(tool.standing.size()))
			return;
		auto const &standing{tool.standing[Intent.index]};
		tool.grab = plan_of(standing.include.location) - plan_of(cursor_level(standing.include.location.y));
		tool.home_valid = signal_standing_spot(standing, tool.home);
		tool.moving = true;
		tool.moved_valid = false;
		signal_select(standing.include.instance);
		return;
	}
	auto const *armed{signal_armed()};
	if (armed == nullptr || Intent.track == nullptr)
		return;
	if (false == signal_spot_at(*Intent.track, Intent.path, m_hover.point, cursor_level(m_hover.point.y), 0, 0, *armed, tool.press))
		return;
	tool.press_raw = tool.press;
	signal_snap(tool.press, *armed);
	tool.placing = true;
}

void editor_mode::signal_move_update()
{
	auto &tool{m_signal};
	auto const &mouse{ImGui::GetIO().MousePos};
	auto const dragged{glm::length(glm::vec2{mouse.x, mouse.y} - tool.pressed) > kDragStart};
	if (tool.placing)
	{
		auto const *armed{signal_armed()};
		auto const &raw{tool.press_raw};
		if (armed == nullptr || raw.track == nullptr || raw.track->m_editorremoved)
		{
			tool.placing = false;
			return;
		}
		if (false == dragged)
			return;
		auto const cursor{cursor_level(raw.axis.y)};
		auto const forward{raw.travel * static_cast<double>(raw.list == 2 ? 1 : -1)};
		auto const along{glm::dot(plan_of(cursor) - plan_of(raw.axis), forward)};
		if (std::abs(along) < kDragTravel)
			return;
		tool.facing = along > 0.0 ? 1 : -1;
		// from where it was pressed each time, so turning it this way and that doesn't walk it along the track
		signal_spot spot;
		if (signal_spot_at(*raw.track, raw.path, raw.axis, cursor, tool.facing, raw.side, *armed, spot))
		{
			signal_snap(spot, *armed);
			tool.press = spot;
		}
		return;
	}
	if (false == tool.moving)
		return;
	if (false == dragged)
		return;
	auto const *found{signal_standing_of(tool.selected)};
	if (found == nullptr)
	{
		tool.moved_valid = false;
		return;
	}
	auto const &location{found->include.location};
	auto const cursor{cursor_level(location.y)};
	glm::dvec3 const held{cursor.x + tool.grab.x, location.y, cursor.z + tool.grab.y};
	// the track under the cursor; off every track, as it is when the signal is dragged beside its track, the track the
	// signal stands by
	auto *track{m_hover.track};
	auto path{m_hover.path};
	if (track == nullptr && tool.home_valid)
	{
		track = tool.home.track;
		path = tool.home.path;
	}
	tool.moved_valid = track != nullptr && signal_drop_at(*found, *track, path, held, 0, tool.moved);
	if (auto const *item{signal_template_of(found->include.file)}; tool.moved_valid && item != nullptr)
	{
		// its distance from the axis and its height as it stood by its own track, before the drag
		if (tool.home_valid)
			signal_carry(tool.home, tool.moved, *item);
		signal_snap(tool.moved, *item);
	}
}

track_trace::danger editor_mode::signal_danger(signal_spot const &Spot) const
{
	if (Spot.track == nullptr || Spot.track->m_editorremoved || Spot.track->m_paths.empty())
		return {};
	auto const path{std::clamp(Spot.path, 0, static_cast<int>(Spot.track->m_paths.size()) - 1)};
	auto const forward{glm::dot(forward_at(*Spot.track, path, track_trace::parameter_of(*Spot.track, path, Spot.axis)), Spot.travel) >= 0.0};
	return track_trace::danger_ahead(*Spot.track, path, Spot.axis, forward, kDangerReach);
}

void editor_mode::signal_snap(signal_spot &Spot, signal_template const &Template) const
{
	// the keys as ImGui has them: Global.altState misses the ones it holds
	if (ImGui::GetIO().KeyAlt)
		return;
	auto const danger{signal_danger(Spot)};
	if (danger.what == track_trace::danger::kind::none || danger.distance < kSnapStep - kSnapReach)
		return;
	auto const round{std::round(danger.distance / kSnapStep) * kSnapStep};
	auto const shift{danger.distance - round};
	if (std::abs(shift) > kSnapReach || std::abs(shift) < 0.005)
		return;
	// that much along the track towards the switch, on the side it was
	std::pair<TTrack *, int> owner{nullptr, 0};
	auto const axis{danger.at(shift, owner)};
	if (owner.first == nullptr || owner.first->m_editorremoved || owner.first->m_paths.empty())
		return;
	auto const facing{glm::dot(forward_at(*owner.first, owner.second, track_trace::parameter_of(*owner.first, owner.second, axis)), Spot.travel) >= 0.0 ? 1 : -1};
	signal_spot snapped;
	if (signal_spot_at(*owner.first, owner.second, axis, axis + (Spot.position - Spot.axis), facing, 0, Template, snapped))
	{
		signal_carry(Spot, snapped, Template);
		Spot = snapped;
	}
}

editor_mode::signal_metre const &editor_mode::signal_metre_of(signal_standing const &Standing) const
{
	auto &metre{m_signal_metre};
	if (metre.instance == Standing.include.instance && metre.history == history_revision() && metre.location == Standing.include.location && metre.yaw == Standing.include.yaw)
		return metre;
	metre.instance = Standing.include.instance;
	metre.history = history_revision();
	metre.location = Standing.include.location;
	metre.yaw = Standing.include.yaw;
	metre.valid = signal_standing_spot(Standing, metre.spot);
	metre.danger = metre.valid ? signal_danger(metre.spot) : track_trace::danger{};
	return metre;
}

std::vector<editor_mode::signal_standing> editor_mode::signal_standing_now() const
{
	std::vector<signal_standing> standing;
	if (scene::Layers.empty() || m_signal.templates.empty())
		return standing;
	auto const found{standing_templates([this](std::string const &File) { return signal_template_of(File) != nullptr || indicator_template_of(File) != nullptr; })};
	// the indicators by the name of their signal
	std::unordered_map<std::string, std::vector<standing_template const *>> marks;
	for (auto const &include : found)
		if (false == include.values.empty() && indicator_template_of(include.file) != nullptr)
			marks[ToLower(include.values.front())].push_back(&include);
	std::unordered_map<std::string, std::vector<std::pair<TTrack *, int>>> readers;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		for (int list = 0; list < 6; ++list)
			for (auto const &event : editor_track::events(*track, list))
				if (false == event.first.empty())
				{
					auto &reads{readers[ToLower(event.first)]};
					if (std::find(reads.begin(), reads.end(), std::make_pair(track, list)) == reads.end())
						reads.emplace_back(track, list);
				}
	}
	for (auto const &include : found)
	{
		auto const *item{signal_template_of(include.file)};
		if (item == nullptr)
			continue;
		signal_standing signal;
		signal.include = include;
		signal.kind = kind_index(item->kind);
		signal.name = include.values.empty() ? std::string{} : include.values.front();
		if (false == item->read.empty())
		{
			signal.read = editor_includes::substitute(item->read, include.values);
			if (auto const reader{readers.find(ToLower(signal.read))}; reader != readers.end())
			{
				signal.reads = reader->second;
				signal.track = signal.reads.front().first;
				signal.list = signal.reads.front().second;
			}
		}
		if (auto const named{marks.find(ToLower(signal.name))}; named != marks.end())
			for (auto const *mark : named->second)
				if (geometry::plan_distance(mark->location, include.location) <= kIndicatorReach)
					signal.indicators.push_back(*mark);
		std::sort(signal.indicators.begin(), signal.indicators.end(), [](standing_template const &A, standing_template const &B) { return A.location.y > B.location.y; });
		standing.push_back(std::move(signal));
	}
	return standing;
}

editor_mode::signal_standing const *editor_mode::signal_standing_of(scene::instance_handle const Instance) const
{
	auto const &standing{m_signal.standing};
	auto const found{std::find_if(standing.begin(), standing.end(), [&](signal_standing const &Standing) { return Standing.include.instance == Instance; })};
	return found != standing.end() ? &*found : nullptr;
}

bool editor_mode::signal_drop_at(signal_standing const &Standing, TTrack &Track, int const Path, glm::dvec3 const &Point, int const Side, signal_spot &Spot) const
{
	auto const *item{signal_template_of(Standing.include.file)};
	if (item == nullptr || Track.m_paths.empty() || Track.m_editorremoved)
		return false;
	auto const path{std::clamp(Path, 0, static_cast<int>(Track.m_paths.size()) - 1)};
	auto const forward{forward_at(Track, path, nearest_on(Track, path, Point))};
	auto const facing{glm::dot(forward, travel_of(Standing.include.yaw)) >= 0.0 ? 1 : -1};
	return signal_spot_at(Track, path, Point, Point, facing, Side * facing, *item, Spot);
}

double editor_mode::signal_default_height(signal_template const &Template, glm::dvec3 const &Axis, glm::dvec3 const &Position) const
{
	auto const rails{Axis.y + kRailTop};
	if (Template.mount == "gantry")
		return rails + static_cast<double>(m_signal.height);
	// as the description of the template has it, its zero this far over the rail heads; the bottom of the model hanging over the
	// ground, down an embankment: the signal on the ground
	auto const zero{rails + Template.height};
	if (false == Template.depth.has_value())
		return zero;
	auto const bottom{zero - *Template.depth};
	auto const ground{ground_at({Position.x, rails, Position.z}).y};
	if (ground < bottom && bottom - ground < kGroundReach)
		return zero - (bottom - ground);
	return zero;
}

double editor_mode::signal_default_change(signal_template const &From, signal_template const &To, glm::dvec3 const &Axis, glm::dvec3 const &Position) const
{
	if (&From == &To || From.file == To.file)
		return 0.0;
	return signal_default_height(To, Axis, Position) - signal_default_height(From, Axis, Position);
}

void editor_mode::signal_carry(signal_spot const &From, signal_spot &To, signal_template const &Template) const
{
	auto const gantry{Template.mount == "gantry"};
	if (false == gantry && From.offset >= kCarryNearest && From.offset <= kCarryFarthest)
		if (auto const away{plan_of(To.position) - plan_of(To.axis)}; glm::length(away) >= 1e-6)
		{
			auto const out{glm::normalize(away) * From.offset};
			To.position.x = To.axis.x + out.x;
			To.position.z = To.axis.z + out.y;
			To.offset = From.offset;
		}
	// raised over where it stands by default, there and here; a height off by more is of another level, a track under a bridge or
	// over it: the default there
	if (auto const raised{From.position.y - signal_default_height(Template, From.axis, From.position)}; std::abs(raised) < kCarryRaise)
		To.position.y = signal_default_height(Template, To.axis, To.position) + raised;
}

bool editor_mode::signal_standing_spot(signal_standing const &Standing, signal_spot &Spot, bool const Moved) const
{
	auto const &location{Standing.include.location};
	TTrack *track{nullptr};
	int path{0};
	auto best{kSignalReach};
	auto const consider = [&](TTrack *Track) {
		if (Track == nullptr || Track->m_editorremoved)
			return;
		for (int p = 0; p < static_cast<int>(Track->m_paths.size()); ++p)
		{
			bezier const curve{Track->m_paths[p]};
			auto const low{glm::min(glm::min(plan_of(curve.p0), plan_of(curve.p1)), glm::min(plan_of(curve.p2), plan_of(curve.p3)))};
			auto const high{glm::max(glm::max(plan_of(curve.p0), plan_of(curve.p1)), glm::max(plan_of(curve.p2), plan_of(curve.p3)))};
			if (glm::length(glm::max(glm::max(low - plan_of(location), plan_of(location) - high), glm::dvec2{0.0})) >= best)
				continue;
			auto const distance{geometry::plan_distance(curve.point(nearest_on(*Track, p, location)), location)};
			if (distance < best)
			{
				best = distance;
				track = Track;
				path = p;
			}
		}
	};
	consider(Standing.track);
	// a signal past the frog of a switch is read by the track joined to its path: it stands by the switch
	if (Standing.track != nullptr && Standing.track->eType == tt_Normal)
		for (auto *neighbour : editor_track::neighbours(*Standing.track))
		{
			if (neighbour == nullptr || neighbour->eType != tt_Switch)
				continue;
			auto const kept{std::make_tuple(track, path, best)};
			consider(neighbour);
			if (track == neighbour && reader_at(*neighbour, path, bezier{neighbour->m_paths[path]}.point(nearest_on(*neighbour, path, location)), 2).first != Standing.track)
				std::tie(track, path, best) = kept;
		}
	// moved, by another track: that one if it's the nearer by more than a little, so a signal only turned between two tracks keeps
	// its own; dropped onto a track it keeps its own too, its side of that one unsure, the gauge would push it anywhere
	if (track == nullptr || Moved)
	{
		auto const kept{std::make_tuple(track, path, best)};
		if (track != nullptr)
		{
			best -= kTrackNearer;
			track = nullptr;
		}
		for (auto *candidate : simulation::Paths.sequence())
			consider(candidate);
		if (track == nullptr || (std::get<0>(kept) != nullptr && best < kOnTrack))
			std::tie(track, path, best) = kept;
	}
	if (track == nullptr)
		return false;
	auto const axis{bezier{track->m_paths[path]}.point(nearest_on(*track, path, location))};
	auto const side{glm::dot(plan_of(location) - plan_of(axis), right_of(travel_of(Standing.include.yaw))) >= 0.0 ? 1 : -1};
	if (false == signal_drop_at(Standing, *track, path, location, side, Spot))
		return false;
	Spot.offset = geometry::plan_distance(location, axis);
	Spot.position = location;
	Spot.yaw = Standing.include.yaw;
	return true;
}

void editor_mode::signal_release()
{
	auto &tool{m_signal};
	if (tool.placing)
	{
		tool.placing = false;
		if (auto const *armed{signal_armed()}; armed != nullptr && tool.press.track != nullptr && false == tool.press.track->m_editorremoved)
			signal_place(tool.press, *armed);
		return;
	}
	if (false == tool.moving)
		return;
	tool.moving = false;
	auto const &mouse{ImGui::GetIO().MousePos};
	if (glm::length(glm::vec2{mouse.x, mouse.y} - tool.pressed) <= kDragStart)
		return;
	if (false == tool.moved_valid)
	{
		tool.status = refused(STR_C("Drop the signal by a track"));
		return;
	}
	tool.moved_valid = false;
	if (auto const *found{signal_standing_of(tool.selected)})
		signal_move_to(*found, tool.moved, STR_C("Signal %s moved, read by %s %s of %s, Ctrl+Z takes it back"), STR_C("Signal %s moved, Ctrl+Z takes it back"));
}

bool editor_mode::signal_move_to(signal_standing const &Standing, signal_spot const &Spot, char const *Read, char const *Unread)
{
	auto &tool{m_signal};
	auto const instance{Standing.include.instance};
	if (false == scene::Layers.tracked(instance) || false == scene::Layers.removable(instance))
	{
		tool.status = refused(STR_C("This signal is in a file the editor doesn't change, it can't be moved"));
		return false;
	}
	if (on_other_path(*Spot.track, Spot.path, Spot.position, Spot.offset))
	{
		tool.status = refused(STR_C(kOtherPath));
		return false;
	}
	auto const &included{scene::Layers.instance(instance)};
	auto file{*included.file};
	signal_spot before;
	auto const stood{signal_standing_spot(Standing, before)};
	auto const *item{signal_template_of(Standing.include.file)};
	if (item != nullptr && stood && travel_side(before.side, before.list) != travel_side(Spot.side, Spot.list))
		file = signal_variant(*item, Spot);
	auto const *next{signal_template_of(file)};
	// kept apart: the refresh below may scan the templates again, and next would point into the old list
	auto const gantry{next != nullptr && next->mount == "gantry"};
	auto values{Standing.include.values};
	auto local{Spot.position - included.context.offset};
	if (item != nullptr && next != nullptr)
		local.y += signal_default_change(*item, *next, Spot.axis, Spot.position);
	values[1] = editor_includes::number(local.x);
	values[2] = editor_includes::number(local.y);
	values[3] = editor_includes::number(local.z);
	values[4] = editor_includes::number(Spot.yaw);
	auto const read{next != nullptr && false == next->read.empty() ? editor_includes::substitute(next->read, values) : Standing.read};
	auto directive{scene::Layers.directive(instance)};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	if (false == Standing.read.empty() && Spot.reader->Dynamics.empty())
	{
		signal_detach_events(Standing.read, states);
		if (std::none_of(states.begin(), states.end(), [&](auto const &State) { return State.first == Spot.reader; }))
			states.emplace_back(Spot.reader, editor_track::capture(*Spot.reader));
		editor_track::events(*Spot.reader, Spot.reader_list).emplace_back(ToLower(read), nullptr);
		std::string missing;
		editor_track::bind_events(*Spot.reader, missing);
		Spot.reader->mark_dirty();
		// the reads picked by hand stay where they were, all but the one the signal got by the track it stood by, which goes with it;
		// turned, the signal is read by the other list of each of them
		auto const turned{glm::dot(travel_of(Standing.include.yaw), travel_of(Spot.yaw)) < 0.0};
		for (std::size_t i = 0; i < Standing.reads.size(); ++i)
		{
			auto [track, list] = Standing.reads[i];
			if (stood ? track == before.reader && list == before.reader_list : i == 0)
				continue;
			if (turned && list % 3 != 0)
				list = list % 3 == 1 ? list + 1 : list - 1;
			if (std::none_of(states.begin(), states.end(), [&](auto const &State) { return State.first == track; }))
				states.emplace_back(track, editor_track::capture(*track));
			auto &events{editor_track::events(*track, list)};
			if (std::none_of(events.begin(), events.end(), [&](auto const &Event) { return ToLower(Event.first) == ToLower(read); }))
				events.emplace_back(ToLower(read), nullptr);
			editor_track::bind_events(*track, missing);
			track->mark_dirty();
		}
	}
	auto const name{Standing.name};
	auto const node{Standing.include.file};
	auto const held{frame_of(include_models(instance, Standing.include.location))};
	set_include_directive(instance, editor_includes::compose_directive(file, values));
	gauge_tool::gauge_hit hit;
	double shift{0.0};
	if (gantry)
		hit = m_gauge.gauge_models(include_models(instance, Spot.position));
	else
		shift = signal_off_gauge(instance, file, values, local, hit);
	std::vector<std::pair<scene::instance_handle, std::string>> carried;
	signal_indicators_follow(Standing, held, carried);
	// taken up by the track it stood by, it would be put back by it as that one changes: it is let go, for the track it stands by now
	infra::binding bound;
	bound.type = infra::kind::include;
	bound.include = instance;
	if (auto const *found{infra_find(bound)}; found != nullptr && found->adopted)
		infra_unbind({static_cast<std::size_t>(found - m_bindings.data())});
	for (auto const &mark : Standing.indicators)
	{
		bound.include = mark.instance;
		if (auto const *found{infra_find(bound)}; found != nullptr && found->adopted)
			infra_unbind({static_cast<std::size_t>(found - m_bindings.data())});
	}
	if (states.empty())
	{
		EditorSnapshot snap;
		snap.directives.emplace_back(instance, std::move(directive));
		snap.directives.insert(snap.directives.end(), carried.begin(), carried.end());
		snap.node_name = node;
		trim_history();
		m_history.push_back(std::move(snap));
		g_redo.clear();
	}
	else
	{
		push_track_snapshot(std::move(states));
		m_history.back().directives.emplace_back(instance, std::move(directive));
		m_history.back().directives.insert(m_history.back().directives.end(), carried.begin(), carried.end());
	}
	signal_refresh(true);
	signal_select(instance);
	auto const *moved{signal_standing_of(instance)};
	tool.status = moved != nullptr && moved->track != nullptr ? format(Read, name.c_str(), editor_track::event_keyword(moved->list), moved->read.c_str(), name_of(*moved->track).c_str()) : format(Unread, name.c_str());
	signal_gauge_note(tool.status, shift, hit, gantry);
	return true;
}

gauge_tool::gauge_hit editor_mode::signal_gauge(scene::instance_handle const Instance)
{
	auto const *standing{signal_standing_of(Instance)};
	if (standing == nullptr)
		return {};
	return m_gauge.gauge_models(include_models(Instance, standing->include.location));
}

double editor_mode::signal_off_gauge(scene::instance_handle const Instance, std::string const &File, std::vector<std::string> &Values, glm::dvec3 Local, gauge_tool::gauge_hit &Hit)
{
	auto const offset{scene::Layers.instance(Instance).context.offset};
	auto const given{Values};
	auto const start{Local};
	gauge_tool::gauge_hit first;
	for (int step = 0;; ++step)
	{
		auto const location{Local + offset};
		Hit = m_gauge.gauge_models(include_models(Instance, location));
		if (step == 0)
			first = Hit;
		if (Hit.depth <= kGaugeSlack || Hit.track == nullptr || Hit.track->m_paths.empty())
			return geometry::plan_distance(Local, start);
		if (step == kGaugeSteps)
		{
			// no room for it: it stays where it was put, the status tells the gauge it enters
			Values = given;
			set_include_directive(Instance, editor_includes::compose_directive(File, Values));
			Hit = first;
			return 0.0;
		}
		// away from the axis of the path whose gauge it enters, as deep as it went in and a little more
		auto const &track{*Hit.track};
		auto const path{std::clamp(Hit.path, 0, static_cast<int>(track.m_paths.size()) - 1)};
		auto const axis{bezier{track.m_paths[path]}.point(nearest_on(track, path, location))};
		// a track on another level, under a bridge or over it, isn't left by a step aside: the signal stays, the gauge told
		if (std::abs(axis.y - location.y) > kGaugeLevel)
			return geometry::plan_distance(Local, start);
		auto const normal{right_of(Hit.direction)};
		auto const side{glm::dot(plan_of(location) - plan_of(axis), normal) >= 0.0 ? 1.0 : -1.0};
		auto const away{normal * (side * (Hit.depth + kGaugeMargin))};
		Local.x += away.x;
		Local.z += away.y;
		Values[1] = editor_includes::number(Local.x);
		Values[2] = editor_includes::number(Local.y);
		Values[3] = editor_includes::number(Local.z);
		set_include_directive(Instance, editor_includes::compose_directive(File, Values));
	}
}

void editor_mode::signal_gauge_note(std::string &Status, double const Shift, gauge_tool::gauge_hit const &Hit, bool const Gantry) const
{
	std::string note;
	if (Hit.depth > kGaugeSlack && Hit.track != nullptr)
		note = Gantry ? format(STR_C("It enters the gauge of %s by %.0f cm: raise the gantry"), name_of(*Hit.track).c_str(), Hit.depth * 100.0)
		              : format(STR_C("It enters the gauge of %s by %.0f cm: there is no room for it here, put it elsewhere or on the other side"), name_of(*Hit.track).c_str(), Hit.depth * 100.0);
	else if (Shift > 0.0)
		note = format(STR_C("Moved %.0f cm sideways to keep out of the gauge."), Shift * 100.0);
	if (note.empty())
		return;
	if (false == Status.empty() && Status.back() != '.')
		Status += '.';
	Status += (Status.empty() ? "" : " ") + note;
}

bool editor_mode::signal_indicator_add(signal_standing const &Standing, indicator_template const &Indicator)
{
	auto &tool{m_signal};
	auto const instance{Standing.include.instance};
	auto const name{Standing.name};
	auto const signal_file{Standing.include.file};
	auto const models{include_models(instance, Standing.include.location)};
	if (auto const *item{signal_template_of(signal_file)}; item != nullptr && item->mount == "gantry")
	{
		// it stands over the track, at the height of the gantry: neither a place under its lights nor a foot fit there
		tool.status = refused(STR_C("A signal on a gantry takes no indicator from the list"));
		return false;
	}
	if (std::any_of(Standing.indicators.begin(), Standing.indicators.end(), [&](standing_template const &Mark) { return ToLower(Mark.file) == ToLower(Indicator.file); }))
	{
		tool.status = refused(format(STR_C("Signal %s has the indicator %s already"), name.c_str(), Indicator.name.c_str()));
		return false;
	}
	glm::dvec3 location{Standing.include.location};
	// the ones under the lights below the place of the new one, each a place down, with the directives they had
	std::vector<std::pair<scene::instance_handle, std::string>> moved;
	if (Indicator.place == "head")
	{
		if (auto const *item{signal_template_of(signal_file)}; item != nullptr && item->mount == "dwarf")
		{
			tool.status = refused(STR_C("A dwarf signal has no mast for an indicator under its lights"));
			return false;
		}
		// in its place by Ie-117 among the ones under the lights, from the top: under the last one it doesn't go above
		std::vector<standing_template const *> heads;
		for (auto const &mark : Standing.indicators)
			if (auto const *other{indicator_template_of(mark.file)}; other != nullptr && other->place == "head")
				heads.push_back(&mark);
		std::sort(heads.begin(), heads.end(), [](standing_template const *A, standing_template const *B) { return A->location.y > B->location.y; });
		auto const rank{indicator_rank(Indicator.sign)};
		std::size_t place{0};
		for (std::size_t i = 0; i < heads.size(); ++i)
			if (auto const *other{indicator_template_of(heads[i]->file)}; other != nullptr && indicator_rank(other->sign) <= rank)
				place = i + 1;
		auto const frame{frame_of(models)};
		if (place < heads.size())
		{
			// where the first one under it stands; that one and the ones under it a place down
			location = heads[place]->location;
			std::string reason;
			for (auto i = place; i < heads.size(); ++i)
				if (false == scene::Layers.tracked(heads[i]->instance) || false == scene::Layers.removable(heads[i]->instance, &reason))
				{
					tool.status = refused(format(STR_C("The indicator can't be moved: %s"), reason.c_str()));
					return false;
				}
			for (auto i = place; i < heads.size(); ++i)
			{
				std::string file;
				std::vector<std::string> values;
				auto directive{scene::Layers.directive(heads[i]->instance)};
				if (false == editor_includes::parse_directive(directive, file, values) || values.size() < 5)
					continue;
				auto const local{heads[i]->location + frame * glm::dvec3{0.0, -kIndicatorStep, 0.0} - scene::Layers.instance(heads[i]->instance).context.offset};
				values[1] = editor_includes::number(local.x);
				values[2] = editor_includes::number(local.y);
				values[3] = editor_includes::number(local.z);
				set_include_directive(heads[i]->instance, editor_includes::compose_directive(file, values));
				moved.emplace_back(heads[i]->instance, std::move(directive));
			}
		}
		else if (false == heads.empty())
			location = heads.back()->location + frame * glm::dvec3{0.0, -kIndicatorStep, 0.0};
		else
		{
			// under the lowest lamp
			auto const lamp{lowest_lamp(models) - Standing.include.location.y};
			location += frame * glm::dvec3{0.0, lamp > kIndicatorUnder + 1.0 && lamp < 50.0 ? lamp - kIndicatorUnder : kIndicatorHead, kIndicatorAhead};
		}
	}
	template_item item;
	item.file = Indicator.file;
	item.location = location;
	item.yaw = Standing.include.yaw;
	item.described = true;
	item.values["#1"] = name;
	std::string error;
	if (place_templates({item}, error, output_layer("srk")) == 0)
	{
		for (auto const &[instance, directive] : moved)
			set_include_directive(instance, directive);
		tool.status = refused(STR_C("The indicator couldn't be placed: ") + error);
		return false;
	}
	// the ones moved down for it go back with it
	history_add_directives(m_history.back(), moved);
	tool.status = format(STR_C("Indicator %s put on signal %s, Ctrl+Z takes it back"), Indicator.name.c_str(), name.c_str());
	// an event both define the scenery keeps once, the indicator's may be the one lost
	std::vector<std::string> own;
	std::vector<std::string> theirs;
	std::string twice;
	if (editor_includes::event_names(signal_file, own) && editor_includes::event_names(Indicator.file, theirs))
		for (auto const &event : theirs)
			if (std::find(own.begin(), own.end(), event) != own.end())
				twice += (twice.empty() ? "" : ", ") + editor_includes::substitute(event, {name});
	if (false == twice.empty())
		tool.status += ". " + format(STR_C("Both the signal and the indicator define %s: the scenery keeps one of each, the indicator may not work"), twice.c_str());
	signal_refresh(true);
	signal_select(instance);
	return true;
}

double editor_mode::signal_lowest_lamp(scene::instance_handle const Instance) const
{
	auto const *standing{signal_standing_of(Instance)};
	if (standing == nullptr)
		return 0.0;
	auto const lowest{lowest_lamp(include_models(Instance, standing->include.location))};
	return lowest < std::numeric_limits<double>::max() ? lowest : 0.0;
}

std::vector<editor_mode::standing_template> editor_mode::signal_indicators_of(std::string const &Name, glm::dvec3 const &Location) const
{
	if (m_signal.indicators.empty() || Name.empty())
		return {};
	auto const name{ToLower(Name)};
	auto marks{standing_templates([this](std::string const &File) { return indicator_template_of(File) != nullptr; })};
	marks.erase(std::remove_if(marks.begin(), marks.end(), [&](standing_template const &Mark) { return Mark.values.empty() || ToLower(Mark.values.front()) != name || geometry::plan_distance(Mark.location, Location) > kIndicatorReach; }), marks.end());
	std::sort(marks.begin(), marks.end(), [](standing_template const &A, standing_template const &B) { return A.location.y > B.location.y; });
	return marks;
}

void editor_mode::signal_indicators_renamed(std::string const &Before, std::string const &After, glm::dvec3 const &Location, std::vector<std::pair<scene::instance_handle, std::string>> &Directives)
{
	if (Before == After)
		return;
	signal_scan(true);
	for (auto const &mark : signal_indicators_of(Before, Location))
	{
		auto const had{scene::Layers.directive(mark.instance)};
		std::string own;
		std::vector<std::string> placed;
		if (false == scene::Layers.tracked(mark.instance) || false == scene::Layers.removable(mark.instance) || false == editor_includes::parse_directive(had, own, placed) || placed.empty())
			continue;
		placed[0] = After;
		if (std::none_of(Directives.begin(), Directives.end(), [&](auto const &Entry) { return Entry.first == mark.instance; }))
			Directives.emplace_back(mark.instance, had);
		set_include_directive(mark.instance, editor_includes::compose_directive(own, placed));
	}
}

void editor_mode::history_add_directives(EditorSnapshot &Step, std::vector<std::pair<scene::instance_handle, std::string>> const &Directives)
{
	if (Directives.empty())
		return;
	if (Step.instance != 0)
	{
		Step.directives.emplace_back(Step.instance, std::move(Step.serialized));
		Step.instance = 0;
		Step.serialized.clear();
	}
	for (auto const &entry : Directives)
		if (std::none_of(Step.directives.begin(), Step.directives.end(), [&](auto const &Taken) { return Taken.first == entry.first; }))
			Step.directives.push_back(entry);
}

void editor_mode::signal_gizmo_begin()
{
	auto &gizmo{m_signal.gizmo};
	if (m_instance == 0 || gizmo.instance == m_instance || signal_template_of(m_include.target) == nullptr)
		return;
	signal_gizmo_end();
	signal_refresh(true);
	auto const *standing{signal_standing_of(m_instance)};
	if (standing == nullptr)
		return;
	gizmo.instance = m_instance;
	gizmo.from = *standing;
	gizmo.frame = frame_of(include_models(m_instance, standing->include.location));
	gizmo.history = m_history.size();
	gizmo.directives.clear();
	for (auto const &mark : standing->indicators)
		if (scene::Layers.tracked(mark.instance) && scene::Layers.removable(mark.instance))
			gizmo.directives.emplace_back(mark.instance, scene::Layers.directive(mark.instance));
}

void editor_mode::signal_gizmo_step()
{
	auto &gizmo{m_signal.gizmo};
	if (gizmo.instance == 0 || gizmo.instance != m_instance)
		return;
	// none before the gizmo made its step: grabbed and let go it changes nothing
	if (m_history.size() != gizmo.history + 1)
		return;
	std::vector<std::pair<scene::instance_handle, std::string>> carried;
	signal_indicators_follow(gizmo.from, gizmo.frame, carried);
	// the step of the gizmo takes them back with the signal
	history_add_directives(m_history.back(), gizmo.directives);
}

void editor_mode::signal_gizmo_end()
{
	auto &gizmo{m_signal.gizmo};
	if (gizmo.instance == 0)
		return;
	auto const instance{gizmo.instance};
	auto const from{std::move(gizmo.from)};
	auto const history{gizmo.history};
	gizmo.instance = 0;
	gizmo.directives.clear();
	if (m_history.size() != history + 1 || false == m_history.back().tracks.empty() || false == scene::Layers.tracked(instance) || false == scene::Layers.removable(instance))
	{
		signal_refresh(true);
		return;
	}
	std::string file;
	std::vector<std::string> values;
	if (false == editor_includes::parse_directive(scene::Layers.directive(instance), file, values) || values.size() < 5)
		return;
	signal_standing moved{from};
	moved.include.location = scene::Layers.instance(instance).context.offset + glm::dvec3{std::strtod(values[1].c_str(), nullptr), std::strtod(values[2].c_str(), nullptr), std::strtod(values[3].c_str(), nullptr)};
	moved.include.yaw = std::strtod(values[4].c_str(), nullptr);
	signal_spot spot;
	auto const by_track{signal_standing_spot(moved, spot, true)};
	// back where it stood, without the step of the gizmo: the move is one of its own, with the read, the gauge and the indicators.
	// Away from every track it stays there, as a drag of the signal tool leaves it
	auto step{std::move(m_history.back())};
	m_history.pop_back();
	for (auto const &[each, directive] : step.directives)
		set_include_directive(each, directive);
	if (step.instance != 0 && false == step.serialized.empty())
		set_include_directive(step.instance, step.serialized);
	signal_refresh(true);
	if (false == by_track)
	{
		m_signal.status = refused(STR_C("Drop the signal by a track"));
		ui()->set_status(m_signal.status, true);
		// the gizmo and the panel where it stands again
		select_include(instance);
		return;
	}
	auto const *standing{signal_standing_of(instance)};
	if (standing == nullptr)
		return;
	auto const done{signal_move_to(*standing, spot, STR_C("Signal %s moved, read by %s %s of %s, Ctrl+Z takes it back"), STR_C("Signal %s moved, Ctrl+Z takes it back"))};
	ui()->set_status(m_signal.status, false == done);
	if (false == done)
		select_include(instance);
}

bool editor_mode::signal_indicator_remove(scene::instance_handle const Indicator)
{
	std::string reason;
	if (false == scene::Layers.tracked(Indicator) || false == scene::Layers.removable(Indicator, &reason))
	{
		m_signal.status = refused(format(STR_C("The indicator can't be removed: %s"), reason.c_str()));
		return false;
	}
	auto const selected{m_signal.selected};
	scene::Layers.removed(Indicator, true);
	EditorSnapshot snap;
	snap.instances.push_back(Indicator);
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	m_signal.status = STR("Indicator removed, Ctrl+Z brings it back");
	signal_refresh(true);
	signal_select(selected);
	return true;
}

bool editor_mode::signal_indicators_swap(signal_standing const &Standing, std::size_t const Upper, std::size_t const Lower)
{
	if (Upper >= Standing.indicators.size() || Lower >= Standing.indicators.size() || Upper == Lower)
		return false;
	auto const &upper{Standing.indicators[Upper]};
	auto const &lower{Standing.indicators[Lower]};
	// under the lights both: a plate at the foot keeps its place
	for (auto const *mark : {&upper, &lower})
		if (auto const *item{indicator_template_of(mark->file)}; item == nullptr || item->place != "head")
		{
			m_signal.status = refused(STR_C("Only the indicators under the lights change places, a plate at the foot keeps its own"));
			return false;
		}
	std::string reason;
	for (auto const instance : {upper.instance, lower.instance})
		if (false == scene::Layers.tracked(instance) || false == scene::Layers.removable(instance, &reason))
		{
			m_signal.status = refused(format(STR_C("The indicator can't be moved: %s"), reason.c_str()));
			return false;
		}
	// each where the other one stands
	std::vector<std::pair<scene::instance_handle, std::string>> directives;
	for (auto const &[mark, place] : {std::make_pair(&upper, lower.location), std::make_pair(&lower, upper.location)})
	{
		std::string file;
		std::vector<std::string> values;
		auto directive{scene::Layers.directive(mark->instance)};
		if (false == editor_includes::parse_directive(directive, file, values) || values.size() < 5)
			continue;
		auto const local{place - scene::Layers.instance(mark->instance).context.offset};
		values[1] = editor_includes::number(local.x);
		values[2] = editor_includes::number(local.y);
		values[3] = editor_includes::number(local.z);
		set_include_directive(mark->instance, editor_includes::compose_directive(file, values));
		directives.emplace_back(mark->instance, std::move(directive));
	}
	if (directives.empty())
		return false;
	auto const *first{indicator_template_of(upper.file)};
	auto const *second{indicator_template_of(lower.file)};
	auto const shown_first{first != nullptr ? first->name : upper.file};
	auto const shown_second{second != nullptr ? second->name : lower.file};
	EditorSnapshot snap;
	snap.directives = std::move(directives);
	snap.node_name = Standing.include.file;
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	m_signal.status = format(STR_C("%s now under %s, Ctrl+Z takes it back"), shown_first.c_str(), shown_second.c_str());
	auto const selected{m_signal.selected};
	signal_refresh(true);
	signal_select(selected);
	return true;
}

bool editor_mode::signal_set_across(signal_standing const &Standing, std::optional<double> const Offset, std::optional<double> const Height)
{
	if (false == Offset.has_value() && false == Height.has_value())
		return false;
	signal_spot spot;
	if (false == signal_standing_spot(Standing, spot))
	{
		m_signal.status = refused(STR_C("No track by the signal to take it from"));
		return false;
	}
	auto const *item{signal_template_of(Standing.include.file)};
	// raised over where it stands by default, kept as it moves across unless given
	auto raised{item != nullptr ? spot.position.y - signal_default_height(*item, spot.axis, spot.position) : 0.0};
	if (Height.has_value())
		raised = *Height;
	if (Offset.has_value())
	{
		// out from the axis the way it stands, on the side it stands
		auto away{plan_of(Standing.include.location) - plan_of(spot.axis)};
		if (glm::length(away) < 1e-3)
			away = right_of(spot.travel) * static_cast<double>(spot.side);
		away = glm::normalize(away);
		spot.offset = *Offset;
		spot.position.x = spot.axis.x + away.x * *Offset;
		spot.position.z = spot.axis.z + away.y * *Offset;
	}
	if (item != nullptr)
		spot.position.y = signal_default_height(*item, spot.axis, spot.position) + raised;
	else if (Height.has_value())
		spot.position.y = spot.axis.y + kRailTop + *Height;
	return signal_move_to(Standing, spot, STR_C("Signal %s moved, read by %s %s of %s, Ctrl+Z takes it back"), STR_C("Signal %s moved, Ctrl+Z takes it back"));
}

void editor_mode::signal_indicators_follow(signal_standing const &Standing, glm::dmat3 const &Before, std::vector<std::pair<scene::instance_handle, std::string>> &Directives)
{
	if (Standing.indicators.empty())
		return;
	auto const instance{Standing.include.instance};
	std::string file;
	std::vector<std::string> values;
	if (false == editor_includes::parse_directive(scene::Layers.directive(instance), file, values) || values.size() < 5)
		return;
	auto const &included{scene::Layers.instance(instance)};
	glm::dvec3 const location{included.context.offset + glm::dvec3{std::strtod(values[1].c_str(), nullptr), std::strtod(values[2].c_str(), nullptr), std::strtod(values[3].c_str(), nullptr)}};
	auto const after{frame_of(include_models(instance, location))};
	auto const back{glm::transpose(Before)};
	for (auto const &mark : Standing.indicators)
	{
		if (false == scene::Layers.tracked(mark.instance) || false == scene::Layers.removable(mark.instance))
			continue;
		std::string own;
		std::vector<std::string> placed;
		auto directive{scene::Layers.directive(mark.instance)};
		if (false == editor_includes::parse_directive(directive, own, placed) || placed.size() < 5)
			continue;
		// at the same place by the signal, in its frame
		auto const local{location + after * (back * (mark.location - Standing.include.location)) - scene::Layers.instance(mark.instance).context.offset};
		placed[1] = editor_includes::number(local.x);
		placed[2] = editor_includes::number(local.y);
		placed[3] = editor_includes::number(local.z);
		placed[4] = values[4];
		auto composed{editor_includes::compose_directive(own, placed)};
		if (composed == directive)
			continue;
		set_include_directive(mark.instance, std::move(composed));
		Directives.emplace_back(mark.instance, std::move(directive));
	}
}

bool editor_mode::render_signal_indicators(signal_standing const &Found)
{
	auto const &tool{m_signal};
	if (tool.indicators.empty())
		return false;
	if (false == Found.indicators.empty())
		ImGui::TextDisabled("%s", STR_C("Indicators on it:"));
	for (auto const &mark : Found.indicators)
	{
		auto const *item{indicator_template_of(mark.file)};
		ImGui::PushID(static_cast<int>(mark.instance));
		ImGui::Bullet();
		ImGui::SameLine();
		ImGui::TextUnformatted(item != nullptr ? item->name.c_str() : mark.file.c_str());
		if (item != nullptr && ImGui::IsItemHovered())
		{
			std::string events;
			for (auto const &[shown, event] : item->events)
				events += "\n" + shown + ": " + Found.name + "_" + event;
			ImGui::SetTooltip("%s\n%s%s", mark.file.c_str(), item->description.c_str(), events.c_str());
		}
		// the ones under the lights, one above another: each changes places with the one above or below it
		auto const head = [&](std::size_t const Index) {
			auto const *other{indicator_template_of(Found.indicators[Index].file)};
			return other != nullptr && other->place == "head";
		};
		auto const index{static_cast<std::size_t>(&mark - Found.indicators.data())};
		std::optional<std::size_t> swap;
		if (head(index))
		{
			ImGui::SameLine();
			ImGui::BeginDisabled(index == 0 || false == head(index - 1));
			if (ImGui::ArrowButton("##up", ImGuiDir_Up))
				swap = index - 1;
			ImGui::EndDisabled();
			item_tooltip("Up: it changes places with the indicator above it. One step of undo");
			ImGui::SameLine();
			ImGui::BeginDisabled(index + 1 >= Found.indicators.size() || false == head(index + 1));
			if (ImGui::ArrowButton("##down", ImGuiDir_Down))
				swap = index + 1;
			ImGui::EndDisabled();
			item_tooltip("Down: it changes places with the indicator below it. One step of undo");
		}
		ImGui::SameLine();
		auto const drop{ImGui::SmallButton(STR_C("Remove"))};
		ImGui::PopID();
		if (swap.has_value())
			return signal_indicators_swap(Found, std::min(index, *swap), std::max(index, *swap));
		if (drop)
			return signal_indicator_remove(mark.instance);
	}
	ImGui::SetNextItemWidth(-1.0f);
	indicator_template const *chosen{nullptr};
	if (ImGui::BeginCombo("##indicator", STR_C("Add an indicator...")))
	{
		std::string sign;
		for (auto const &item : tool.indicators)
		{
			if (item.sign != sign)
			{
				sign = item.sign;
				ImGui::SeparatorText(sign.c_str());
			}
			if (ImGui::Selectable((item.name + "##" + item.file).c_str(), false))
				chosen = &item;
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s\n%s", item.file.c_str(), item.description.c_str());
		}
		ImGui::EndCombo();
	}
	item_tooltip("An indicator of the scenery templates described as one, named after the signal: a lit one under its lights in its place by Ie-117 (from the top W21, W24, W20, W19, W26, the others below them), a plate at its foot. It moves with the signal");
	return chosen != nullptr && signal_indicator_add(Found, *chosen);
}

bool editor_mode::signal_flip(scene::instance_handle const Instance)
{
	auto const *standing{signal_standing_of(Instance)};
	signal_spot spot;
	if (standing == nullptr || false == signal_standing_spot(*standing, spot))
	{
		m_signal.status = refused(STR_C("No track by the signal to take it across"));
		return false;
	}
	auto const across{plan_of(spot.position) - plan_of(spot.axis)};
	auto const mirrored{plan_of(spot.axis) + spot.travel * glm::dot(across, spot.travel) - right_of(spot.travel) * glm::dot(across, right_of(spot.travel))};
	spot.side = -spot.side;
	auto const *item{signal_template_of(standing->include.file)};
	// as raised over the ground on the other side as on this one
	auto const raised{item != nullptr ? spot.position.y - signal_default_height(*item, spot.axis, spot.position) : 0.0};
	spot.position = {mirrored.x, spot.position.y, mirrored.y};
	if (item != nullptr)
		spot.position.y = signal_default_height(*item, spot.axis, spot.position) + raised;
	return signal_move_to(*standing, spot, STR_C("Signal %s moved to the other side of the track, read by %s %s of %s, Ctrl+Z takes it back"), STR_C("Signal %s moved to the other side of the track, Ctrl+Z takes it back"));
}

bool editor_mode::signal_turn(scene::instance_handle const Instance)
{
	auto const *standing{signal_standing_of(Instance)};
	signal_spot spot;
	if (standing == nullptr || false == signal_standing_spot(*standing, spot))
	{
		m_signal.status = refused(STR_C("No track by the signal to turn it to"));
		return false;
	}
	auto const *item{signal_template_of(standing->include.file)};
	auto const position{spot.position};
	if (item == nullptr || false == signal_spot_at(*spot.track, spot.path, spot.axis, spot.axis, spot.list == 2 ? -1 : 1, spot.side, *item, spot))
		return false;
	spot.position = position;
	spot.yaw = std::fmod(standing->include.yaw + 180.0, 360.0);
	return signal_move_to(*standing, spot, STR_C("Signal %s turned to the trains of the other direction, read by %s %s of %s, Ctrl+Z takes it back"), STR_C("Signal %s turned to the trains of the other direction, Ctrl+Z takes it back"));
}

bool editor_mode::signal_swap(scene::instance_handle const Instance, std::string const &File)
{
	auto &tool{m_signal};
	auto const *standing{signal_standing_of(Instance)};
	if (standing == nullptr)
		return false;
	auto const name{standing->name};
	auto const before{standing->include.file};
	signal_changes changes;
	changes.files.emplace_back(Instance, File);
	std::string error;
	signal_batch(changes, {}, {}, error);
	signal_refresh(true);
	auto const *swapped{signal_standing_of(Instance)};
	if (false == error.empty() || swapped == nullptr || swapped->include.file == before)
	{
		tool.status = error.empty() ? format(STR_C("Signal %s can't take the template %s"), name.c_str(), File.c_str()) : error;
		return false;
	}
	signal_select(Instance);
	tool.status = format(STR_C("Signal %s got the template %s, Ctrl+Z takes it back"), name.c_str(), swapped->include.file.c_str());
	return true;
}

namespace
{

// what an event sets off with the time each would run, into Looks: of a multiple the ones of its condition met, of all only those
// changing the look, the animations, the lights and the submodels shown; each event once, a thousand of them at most
void signal_looks(basic_event *First, double const At, std::vector<std::pair<double, basic_event *>> &Looks, std::unordered_set<basic_event const *> &Seen)
{
	for (auto *current{First}; current != nullptr; current = current->m_sibling)
	{
		if (current->m_ignored || false == Seen.insert(current).second || Seen.size() > 1024)
			continue;
		auto const at{At + current->m_delay};
		auto const *multiple{dynamic_cast<multi_event const *>(current)};
		if (multiple == nullptr)
		{
			if (dynamic_cast<animation_event *>(current) != nullptr || dynamic_cast<lights_event *>(current) != nullptr || dynamic_cast<visible_event *>(current) != nullptr)
				Looks.emplace_back(at, current);
			continue;
		}
		for (auto const &[child, otherwise] : multiple->branches())
			if (false == otherwise)
				signal_looks(child, at, Looks, Seen);
	}
}

} // namespace

bool editor_mode::signal_show_aspect(scene::instance_handle const Instance, std::string const &Aspect, std::string const &Event)
{
	auto &tool{m_signal};
	auto const *standing{signal_standing_of(Instance)};
	if (standing == nullptr)
		return false;
	auto const name{ToLower(editor_includes::substitute("(p1)_" + Event, {standing->name}))};
	auto *root{simulation::Events.FindEvent(name)};
	if (root == nullptr)
	{
		tool.status = refused(format(STR_C("Signal %s has no event %s yet: the events of a signal placed come with the scenery saved and opened again"), standing->name.c_str(), name.c_str()));
		return false;
	}
	// what the event sets off, by the time each would run; the time stands in the editor, so the animations are taken to their
	// ends at once
	std::vector<std::pair<double, basic_event *>> looks;
	std::unordered_set<basic_event const *> seen;
	signal_looks(root, 0.0, looks, seen);
	if (looks.empty())
	{
		tool.status = refused(format(STR_C("The event %s changes nothing of how signal %s looks"), name.c_str(), standing->name.c_str()));
		return false;
	}
	std::ranges::stable_sort(looks, [](auto const &A, auto const &B) { return A.first < B.first; });
	for (auto *look : looks | std::views::values)
	{
		// the vehicle that set it off last may be gone by now, and run() names it in the log; the queue sets it anew each time
		look->m_activator = nullptr;
		look->run();
		if (auto const *animation{dynamic_cast<animation_event const *>(look)})
			animation->finish();
	}
	tool.status = format(STR_C("Signal %s shows %s here, nothing of it is saved"), standing->name.c_str(), Aspect.c_str());
	return true;
}

std::string editor_mode::signal_variant(signal_template const &Template, signal_spot const &Spot) const
{
	auto file{Template.file};
	auto const wanted{lean_wanted(glm::dot(right_of(Spot.travel), plan_of(Spot.position) - plan_of(Spot.axis)) >= 0.0 ? 1 : -1)};
	if (lean_of(Template.lean) == 0 || lean_of(Template.lean) == wanted)
		return file;
	auto const group{group_of(Template.mount, Template.lamps, Template.arms)};
	auto best{std::numeric_limits<std::size_t>::max()};
	for (auto const &other : m_signal.templates)
	{
		if (other.kind != Template.kind || group_of(other.mount, other.lamps, other.arms) != group || lean_of(other.lean) != wanted || other.speeds != Template.speeds)
			continue;
		auto const distance{edit_distance(other.name, Template.name)};
		if (distance < best)
		{
			best = distance;
			file = other.file;
		}
	}
	return file;
}

std::string editor_mode::signal_next_name(std::string const &Name) const
{
	if (Name.empty())
		return Name;
	auto digits{Name.size()};
	while (digits > 0 && std::isdigit(static_cast<unsigned char>(Name[digits - 1])))
		--digits;
	if (digits < Name.size() && Name.size() - digits < 9)
		return Name.substr(0, digits) + std::to_string(std::stoi(Name.substr(digits)) + 1);
	auto const last{static_cast<unsigned char>(Name.back())};
	if (std::isalpha(last) && last != 'z' && last != 'Z')
		return Name.substr(0, Name.size() - 1) + static_cast<char>(last + 1);
	return Name + "2";
}

bool editor_mode::signal_place(signal_spot const &Spot, signal_template const &Chosen)
{
	// a copy: the refresh below may scan the templates again, and the one given may be in the old list
	auto const Template{Chosen};
	auto &tool{m_signal};
	std::string name{tool.name};
	if (name.empty() || name.find_first_of(" \t\";") != std::string::npos)
	{
		tool.status = refused(STR_C("Give the signal a name without spaces first"));
		return false;
	}
	if (on_other_path(*Spot.track, Spot.path, Spot.position, Spot.offset))
	{
		tool.status = refused(STR_C(kOtherPath));
		return false;
	}
	signal_refresh(true);
	auto const taken = [&](std::string const &Name) {
		return std::any_of(tool.standing.begin(), tool.standing.end(), [&](signal_standing const &Standing) { return ToLower(Standing.name) == ToLower(Name); });
	};
	for (int guard = 0; taken(name) && guard < 1000; ++guard)
		name = signal_next_name(name);
	auto const file{signal_variant(Template, Spot)};
	template_item item;
	item.file = file;
	item.location = Spot.position;
	if (auto const *variant{signal_template_of(file)}; variant != nullptr)
		item.location.y += signal_default_change(Template, *variant, Spot.axis, Spot.position);
	item.yaw = Spot.yaw;
	item.described = true;
	item.values["name"] = name;
	if (Template.plate)
		item.values["#6"] = tool.plate[0] != '\0' ? std::string{tool.plate} : signal_plate(name, kPlateLooks[std::clamp(EditorSettings.signal_plate_look(), 0, kPlateLookCount - 1)]);
	if (Template.linked)
		item.values["#7"] = tool.linked[0] != '\0' ? std::string{tool.linked} : std::string{"none"};
	std::string error;
	if (place_templates({item}, error, output_layer("srk")) == 0)
	{
		tool.status = refused(STR_C("The signal couldn't be placed: ") + error);
		return false;
	}
	auto const instance{m_history.back().instances.front()};
	auto const *placed{signal_template_of(file)};
	auto const read{placed != nullptr && false == placed->read.empty() ? editor_includes::substitute(placed->read, {name}) : std::string{}};
	auto const path{name_of(*Spot.reader)};
	std::string reason;
	if (read.empty())
		tool.status = format(STR_C("Signal %s placed (%s), the train doesn't read it, Ctrl+Z takes it back"), name.c_str(), file.c_str());
	else if (false == Spot.reader->Dynamics.empty())
		tool.status = format(STR_C("Signal %s placed (%s), but vehicles stand on the path: add %s to its %s by hand, Ctrl+Z takes it back"), name.c_str(), file.c_str(), read.c_str(), editor_track::event_keyword(Spot.reader_list));
	else if (false == scene::Layers.editable(Spot.reader, &reason))
		tool.status = format(STR_C("Signal %s placed (%s), but the path can't take its event: %s, Ctrl+Z takes it back"), name.c_str(), file.c_str(), reason.c_str());
	else
	{
		auto snap{std::move(m_history.back())};
		m_history.pop_back();
		auto const before{editor_track::capture(*Spot.reader)};
		editor_track::events(*Spot.reader, Spot.reader_list).emplace_back(ToLower(read), nullptr);
		std::string missing;
		editor_track::bind_events(*Spot.reader, missing);
		Spot.reader->mark_dirty();
		push_track_snapshot({{Spot.reader, before}});
		m_history.back().instances = std::move(snap.instances);
		tool.status = format(STR_C("Signal %s placed (%s), read by %s %s of %s, Ctrl+Z takes it back. Its logic works after the scenery is saved and opened again"), name.c_str(), file.c_str(), editor_track::event_keyword(Spot.reader_list), read.c_str(), path.c_str());
	}
	// out of the gauge of the tracks around at once, in the same step of the undo; a gantry spans the track, it is only measured
	std::string included;
	std::vector<std::string> values;
	if (editor_includes::parse_directive(scene::Layers.directive(instance), included, values) && values.size() > 4)
	{
		gauge_tool::gauge_hit hit;
		double shift{0.0};
		if (Template.mount == "gantry")
			hit = m_gauge.gauge_models(include_models(instance, Spot.position));
		else
			shift = signal_off_gauge(instance, *scene::Layers.instance(instance).file, values, Spot.position - scene::Layers.instance(instance).context.offset, hit);
		signal_gauge_note(tool.status, shift, hit, Template.mount == "gantry");
	}
	std::snprintf(tool.name, sizeof(tool.name), "%s", signal_next_name(name).c_str());
	signal_refresh(true);
	signal_select(instance);
	first_step_done(first_step::signal);
	return true;
}

void editor_mode::signal_detach_events(std::string const &Read, std::vector<std::pair<TTrack *, editor_track::state>> &States)
{
	auto const name{ToLower(Read)};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		bool carries{false};
		for (int list = 0; list < 6 && false == carries; ++list)
			for (auto const &event : editor_track::events(*track, list))
				carries = carries || ToLower(event.first) == name;
		if (false == carries)
			continue;
		if (std::none_of(States.begin(), States.end(), [&](auto const &State) { return State.first == track; }))
			States.emplace_back(track, editor_track::capture(*track));
		for (int list = 0; list < 6; ++list)
		{
			auto &events{editor_track::events(*track, list)};
			events.erase(std::remove_if(events.begin(), events.end(), [&](auto const &Event) { return ToLower(Event.first) == name; }), events.end());
		}
		std::string missing;
		editor_track::bind_events(*track, missing);
		track->mark_dirty();
	}
}

void editor_mode::signal_split(TTrack &Original, TTrack &Created)
{
	// the bound reads went by the cells they read already, as the split took them; one not bound has no event to tell where it is read
	auto const waiting = [](TTrack &Track) {
		for (auto const index : {1, 2})
			for (auto const &event : editor_track::events(Track, index))
				if (false == event.first.empty() && event.second == nullptr)
					return true;
		return false;
	};
	if (false == waiting(Original) && false == waiting(Created))
		return;
	if (m_signal.templates.empty())
		signal_scan(false);
	std::unordered_map<std::string, glm::dvec3> standing;
	for (auto const &signal : signal_standing_now())
		if (false == signal.read.empty())
			standing.emplace(ToLower(signal.read), signal.include.location);
	auto const gap = [](TTrack const &Piece, glm::dvec3 const &Point) { return geometry::plan_distance(editor_track::point_at(Piece, editor_track::nearest_parameter(Piece, Point)), Point); };
	for (auto const index : {1, 2})
	{
		auto &own{editor_track::events(Original, index)};
		auto &theirs{editor_track::events(Created, index)};
		std::vector<std::pair<std::string, basic_event *>> kept;
		std::vector<std::pair<std::string, basic_event *>> given;
		auto const read_at = [&](std::pair<std::string, basic_event *> const &Event) -> glm::dvec3 const * {
			if (Event.first.empty() || Event.second != nullptr)
				return nullptr;
			auto const found{standing.find(ToLower(Event.first))};
			return found != standing.end() ? &found->second : nullptr;
		};
		for (auto const &event : own)
		{
			auto const *at{read_at(event)};
			(at != nullptr && gap(Created, *at) < gap(Original, *at) ? given : kept).push_back(event);
		}
		for (auto const &event : theirs)
		{
			auto const *at{read_at(event)};
			(at != nullptr && gap(Original, *at) < gap(Created, *at) ? kept : given).push_back(event);
		}
		own = std::move(kept);
		theirs = std::move(given);
	}
}

int editor_mode::signal_read_list(signal_standing const &Standing, TTrack const &Track, int const Path) const
{
	if (Track.m_paths.empty())
		return 2;
	auto const path{std::clamp(Path, 0, static_cast<int>(Track.m_paths.size()) - 1)};
	auto const forward{forward_at(Track, path, nearest_on(Track, path, Standing.include.location))};
	return glm::dot(forward, travel_of(Standing.include.yaw)) >= 0.0 ? 2 : 1;
}

bool editor_mode::signal_read_set(signal_standing const &Standing, std::vector<std::pair<TTrack *, int>> const &Reads)
{
	auto &tool{m_signal};
	if (Standing.read.empty())
		return false;
	for (auto const &[track, list] : Reads)
	{
		std::string reason;
		if (false == track->Dynamics.empty())
		{
			tool.status = refused(format(STR_C("Vehicles stand on %s: its events aren't changed with them on it"), name_of(*track).c_str()));
			return false;
		}
		if (false == scene::Layers.editable(track, &reason))
		{
			tool.status = refused(format(STR_C("%s can't take the event: %s"), name_of(*track).c_str(), reason.c_str()));
			return false;
		}
	}
	auto const instance{Standing.include.instance};
	auto const name{Standing.name};
	auto const read{ToLower(Standing.read)};
	auto const same{Reads.size() == Standing.reads.size() && std::all_of(Reads.begin(), Reads.end(), [&](auto const &Read) { return std::find(Standing.reads.begin(), Standing.reads.end(), Read) != Standing.reads.end(); })};
	if (same)
	{
		tool.status = format(STR_C("Signal %s is read so already"), name.c_str());
		return false;
	}
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	signal_detach_events(Standing.read, states);
	std::string where;
	for (auto const &[track, list] : Reads)
	{
		if (std::none_of(states.begin(), states.end(), [&](auto const &State) { return State.first == track; }))
			states.emplace_back(track, editor_track::capture(*track));
		auto &events{editor_track::events(*track, list)};
		if (std::none_of(events.begin(), events.end(), [&](auto const &Event) { return ToLower(Event.first) == read; }))
			events.emplace_back(read, nullptr);
		std::string missing;
		editor_track::bind_events(*track, missing);
		track->mark_dirty();
		where += (where.empty() ? "" : ", ") + format("%s %s", editor_track::event_keyword(list), name_of(*track).c_str());
	}
	if (states.empty())
		return false;
	push_track_snapshot(std::move(states));
	signal_refresh(true);
	signal_select(instance);
	tool.status = where.empty() ? format(STR_C("Signal %s isn't read by any path now: the trains don't see it, Ctrl+Z takes it back"), name.c_str())
	                            : format(STR_C("Signal %s read by %s, Ctrl+Z takes it back"), name.c_str(), where.c_str());
	return true;
}

bool editor_mode::signal_read_pick(TTrack &Track, int const Path, bool const Add)
{
	auto const *standing{signal_standing_of(m_signal.selected)};
	if (standing == nullptr || standing->read.empty())
	{
		m_signal.picking_read = false;
		return false;
	}
	std::pair<TTrack *, int> const read{&Track, signal_read_list(*standing, Track, Path)};
	auto reads{standing->reads};
	if (false == Add)
		reads = {read};
	else if (auto const found{std::find(reads.begin(), reads.end(), read)}; found != reads.end())
		reads.erase(found);
	else
		reads.push_back(read);
	return signal_read_set(*standing, reads);
}

std::string editor_mode::signal_read_label(TTrack const &Track, int const Path, bool const Add) const
{
	auto const *standing{signal_standing_of(m_signal.selected)};
	if (standing == nullptr)
		return {};
	auto const list{signal_read_list(*standing, Track, Path)};
	auto const there{std::any_of(standing->reads.begin(), standing->reads.end(), [&](auto const &Read) { return Read.first == &Track && Read.second == list; })};
	if (Add)
		return format(there ? STR_C("Click: %s no more read by %s of %s") : STR_C("Click: %s read by %s of %s too"), standing->name.c_str(), editor_track::event_keyword(list), name_of(Track).c_str());
	return format(STR_C("Click: %s read by %s of %s only. Shift+click: one read more or less, Esc: done"), standing->name.c_str(), editor_track::event_keyword(list), name_of(Track).c_str());
}

void editor_mode::orphans_register(std::vector<TTrack *> const &Removed, std::vector<TTrack *> const &Kept)
{
	auto const carries = [](TTrack &Track) {
		for (int list = 0; list < 6; ++list)
			for (auto const &event : editor_track::events(Track, list))
				if (false == event.first.empty())
					return true;
		return false == Track.Isolated.empty();
	};
	// a record of a track removed again, after an undo brought it back, is the step's to make anew
	m_orphans.erase(std::remove_if(m_orphans.begin(), m_orphans.end(), [&](orphan_record const &Record) { return std::find(Removed.begin(), Removed.end(), Record.track) != Removed.end(); }), m_orphans.end());
	// a track which took the circuits over, removed by the step, leaves them to what the step made
	for (auto &record : m_orphans)
		if (std::any_of(record.given.begin(), record.given.end(), [&](TTrack const *Track) { return std::find(Removed.begin(), Removed.end(), Track) != Removed.end(); }))
		{
			record.circuits.clear();
			record.given.clear();
		}
	std::vector<TTrack *> gone;
	for (auto *track : Removed)
		if (track != nullptr && track->m_editorremoved && false == track->m_paths.empty() && carries(*track))
			gone.push_back(track);
	if (gone.empty())
		return;
	// what the step moved onto the tracks left isn't lost
	std::unordered_set<std::string> carried;
	for (auto *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved)
			for (int list = 0; list < 6; ++list)
				for (auto const &event : editor_track::events(*track, list))
					carried.insert(ToLower(event.first));
	std::unordered_set<std::string> circuits;
	for (auto *track : Kept)
		if (track != nullptr && false == track->m_editorremoved)
			for (auto const *isolated : track->Isolated)
				if (isolated != nullptr)
					circuits.insert(ToLower(isolated->asName));
	for (auto *track : gone)
	{
		orphan_record record;
		record.track = track;
		record.fresh = true;
		for (int list = 0; list < 6; ++list)
			for (auto const &event : editor_track::events(*track, list))
			{
				auto const name{ToLower(event.first)};
				if (false == name.empty() && carried.count(name) == 0 && std::none_of(record.events.begin(), record.events.end(), [&](auto const &Event) { return Event.first == name; }))
					record.events.emplace_back(name, list);
			}
		for (auto const *isolated : track->Isolated)
			if (isolated != nullptr && circuits.count(ToLower(isolated->asName)) == 0)
				record.circuits.push_back(isolated->asName);
		if (record.events.empty() && record.circuits.empty())
			continue;
		m_orphans.push_back(std::move(record));
	}
	if (m_orphans.size() > kOrphanRecords)
		m_orphans.erase(m_orphans.begin(), m_orphans.begin() + static_cast<std::ptrdiff_t>(m_orphans.size() - kOrphanRecords));
}

void editor_mode::orphans_take_over(std::vector<TTrack *> const &Created, std::vector<TTrack *> const &Changed)
{
	// a record waits while its track stays deleted, so the step laying another one in its place, undone and laid again, gets it again
	if (std::none_of(m_orphans.begin(), m_orphans.end(), [](orphan_record const &Record) { return Record.track->m_editorremoved; }))
		return;
	std::vector<TTrack *> candidates;
	for (auto const *tracks : {&Created, &Changed})
		for (auto *track : *tracks)
			if (track != nullptr && false == track->m_editorremoved && false == track->m_paths.empty() && track->Dynamics.empty() && editor_track::is_supported(*track) &&
			    std::find(candidates.begin(), candidates.end(), track) == candidates.end())
				candidates.push_back(track);
	if (candidates.empty() && std::none_of(m_orphans.begin(), m_orphans.end(), [](orphan_record const &Record) { return Record.fresh; }))
		return;
	std::unordered_set<std::string> carried;
	for (auto *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved)
			for (int list = 0; list < 6; ++list)
				for (auto const &event : editor_track::events(*track, list))
					carried.insert(ToLower(event.first));
	// not the signals standing as last listed, nor the templates once found: the step taken may hold one of them; looked for once an event
	// waits
	std::vector<signal_standing> signals;
	std::unordered_map<std::string, signal_standing const *> readers;
	auto known{false};
	auto const reader_of = [&](std::string const &Name) -> signal_standing const * {
		if (false == known)
		{
			known = true;
			if (m_signal.templates.empty())
				signal_scan(false);
			signals = signal_standing_now();
			for (auto const &standing : signals)
				if (false == standing.read.empty())
					readers.emplace(ToLower(standing.read), &standing);
		}
		auto const found{readers.find(Name)};
		return found != readers.end() ? found->second : nullptr;
	};
	std::vector<std::string> taken;
	std::vector<std::string> left;
	std::size_t taken_events{0};
	std::size_t left_events{0};
	std::vector<std::string> taken_circuits;
	std::vector<std::string> left_circuits;
	std::size_t deleted{0};
	for (auto &record : m_orphans)
	{
		if (false == record.track->m_editorremoved)
		{
			record.fresh = false;
			continue;
		}
		if (candidates.empty() && false == record.fresh)
			continue;
		auto const &old{*record.track};
		deleted += record.fresh ? 1 : 0;
		for (auto const &[name, list] : record.events)
		{
			if (carried.count(name) > 0)
				continue;
			auto const *standing{reader_of(name)};
			auto got{false};
			// an event no longer there, of a signal taken away, stays off the tracks
			if (false == candidates.empty() && simulation::Events.FindEvent(name) != nullptr)
			{
				// the place on the old axis: by the signal reading it, else the middle of the track
				int old_path{0};
				double old_parameter{0.5};
				auto const by_signal{standing != nullptr && plan_gap(old, standing->include.location, old_path, old_parameter) <= kSignalReach};
				if (false == by_signal)
				{
					old_path = 0;
					old_parameter = 0.5;
				}
				auto const anchor{bezier{old.m_paths[old_path]}.point(old_parameter)};
				auto const old_forward{forward_at(old, old_path, old_parameter)};
				TTrack *best{nullptr};
				int best_path{0};
				double best_parameter{0.0};
				auto nearest{kOrphanReach};
				for (auto *track : candidates)
				{
					int path{0};
					double parameter{0.0};
					auto const gap{plan_gap(*track, anchor, path, parameter)};
					if (gap <= nearest && std::abs(glm::dot(forward_at(*track, path, parameter), old_forward)) > 0.9)
					{
						nearest = gap;
						best = track;
						best_path = path;
						best_parameter = parameter;
					}
				}
				if (best != nullptr)
				{
					// laid the other way round, the trains running towards its end read it at its start
					auto const reversed{glm::dot(forward_at(*best, best_path, best_parameter), old_forward) < 0.0};
					auto target{reversed && list % 3 != 0 ? list + (list % 3 == 1 ? 1 : -1) : list};
					if (by_signal)
					{
						auto const &location{standing->include.location};
						auto const axis{bezier{best->m_paths[best_path]}.point(nearest_on(*best, best_path, location))};
						auto const side{glm::dot(plan_of(location) - plan_of(axis), right_of(travel_of(standing->include.yaw))) >= 0.0 ? 1 : -1};
						signal_spot spot;
						if (signal_drop_at(*standing, *best, best_path, location, side, spot))
							target = spot.list + (list >= 3 ? 3 : 0);
					}
					editor_track::events(*best, target).emplace_back(name, nullptr);
					std::string missing;
					editor_track::bind_events(*best, missing);
					best->mark_dirty();
					carried.insert(name);
					got = true;
				}
			}
			if (got)
			{
				if (standing != nullptr)
					taken.push_back(standing->name);
				else
					++taken_events;
			}
			else if (record.fresh)
			{
				if (standing != nullptr)
					left.push_back(standing->name);
				else
					++left_events;
			}
		}
		if (false == record.circuits.empty() && std::all_of(record.given.begin(), record.given.end(), [](TTrack const *Track) { return Track->m_editorremoved; }))
		{
			auto got{false};
			// a track lying in the place of the old one for the most part: a new one, in the circuit of the track it was made from,
			// or one without a circuit
			for (auto *track : candidates)
			{
				auto const made{std::find(Created.begin(), Created.end(), track) != Created.end()};
				if ((false == made && false == track->Isolated.empty()) || false == plan_overlaps(*track, old, 2.0))
					continue;
				auto const same{track->Isolated.size() == record.circuits.size() && std::equal(record.circuits.begin(), record.circuits.end(), track->Isolated.begin(), [](std::string const &Name, TIsolated const *Isolated) { return Isolated != nullptr && ToLower(Isolated->asName) == ToLower(Name); })};
				if (same)
					continue;
				std::size_t on{0};
				std::size_t all{0};
				for (auto const &path : track->m_paths)
				{
					bezier const curve{path};
					for (int k = 0; k <= 32; ++k)
					{
						int old_path{0};
						double old_parameter{0.0};
						++all;
						if (plan_gap(old, curve.point(k / 32.0), old_path, old_parameter) <= kOrphanReach)
							++on;
					}
				}
				if (all == 0 || on * 2 < all)
					continue;
				editor_track::isolated(*track, record.circuits);
				if (std::find(record.given.begin(), record.given.end(), track) == record.given.end())
					record.given.push_back(track);
				got = true;
			}
			if (got)
				taken_circuits.insert(taken_circuits.end(), record.circuits.begin(), record.circuits.end());
			else if (record.fresh)
				left_circuits.insert(left_circuits.end(), record.circuits.begin(), record.circuits.end());
		}
		record.fresh = false;
	}
	auto const parts = [](std::vector<std::string> const &Signals, std::size_t const Events, std::vector<std::string> const &Circuits) {
		std::vector<std::string> part;
		if (false == Signals.empty())
			part.push_back(format(STR_P("the read of signal %s", "the reads of signals %s", static_cast<long long>(Signals.size())), listed(Signals).c_str()));
		if (Events > 0)
			part.push_back(format(STR_P("%zu event", "%zu events", static_cast<long long>(Events)), Events));
		if (false == Circuits.empty())
			part.push_back(format(STR_P("the track circuit %s", "the track circuits %s", static_cast<long long>(Circuits.size())), listed(Circuits).c_str()));
		return listed(part);
	};
	std::string note;
	if (auto const got{parts(taken, taken_events, taken_circuits)}; false == got.empty())
		note = format(STR_C("Taken over from the deleted track: %s"), got.c_str());
	if (auto const lost{parts(left, left_events, left_circuits)}; false == lost.empty())
		note += (note.empty() ? "" : "\n") +
		        format(STR_P("The deleted track carried %s: a track laid in its place takes it over, Ctrl+Z brings it back", "The deleted tracks carried %s: tracks laid in their place take it over, Ctrl+Z brings them back", static_cast<long long>(std::max<std::size_t>(deleted, 1))), lost.c_str());
	if (false == note.empty())
		m_orphan_note += (m_orphan_note.empty() ? "" : "\n") + note;
}

std::size_t editor_mode::signal_reattach(std::vector<scene::instance_handle> const &Instances)
{
	signal_refresh(true);
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::size_t attached{0};
	for (auto const instance : Instances)
	{
		auto const *standing{signal_standing_of(instance)};
		if (standing == nullptr || standing->read.empty() || standing->track != nullptr || simulation::Events.FindEvent(ToLower(standing->read)) == nullptr)
			continue;
		signal_spot spot;
		if (false == signal_standing_spot(*standing, spot) || spot.reader == nullptr || false == spot.reader->Dynamics.empty())
			continue;
		if (std::none_of(states.begin(), states.end(), [&](auto const &State) { return State.first == spot.reader; }))
			states.emplace_back(spot.reader, editor_track::capture(*spot.reader));
		auto &events{editor_track::events(*spot.reader, spot.reader_list)};
		if (std::none_of(events.begin(), events.end(), [&](auto const &Event) { return ToLower(Event.first) == ToLower(standing->read); }))
			events.emplace_back(ToLower(standing->read), nullptr);
		std::string missing;
		editor_track::bind_events(*spot.reader, missing);
		spot.reader->mark_dirty();
		++attached;
	}
	if (states.empty())
		return 0;
	push_track_snapshot(std::move(states));
	signal_refresh(true);
	return attached;
}

bool editor_mode::signal_delete(scene::instance_handle const Instance)
{
	if (false == scene::Layers.tracked(Instance) || false == scene::Layers.removable(Instance))
		return false;
	signal_scan();
	auto const &included{scene::Layers.instance(Instance)};
	auto const *item{signal_template_of(*included.file)};
	if (item == nullptr || item->read.empty())
		return false;
	std::string file;
	std::vector<std::string> values;
	if (false == editor_includes::parse_directive(scene::Layers.directive(Instance), file, values))
		return false;
	auto const read{editor_includes::substitute(item->read, values)};
	std::vector<scene::instance_handle> marks;
	if (values.size() >= 4)
	{
		glm::dvec3 const location{included.context.offset + glm::dvec3{std::strtod(values[1].c_str(), nullptr), std::strtod(values[2].c_str(), nullptr), std::strtod(values[3].c_str(), nullptr)}};
		for (auto const &mark : signal_indicators_of(values.front(), location))
			if (scene::Layers.tracked(mark.instance) && scene::Layers.removable(mark.instance))
				marks.push_back(mark.instance);
	}
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	signal_detach_events(read, states);
	if (states.empty())
		return false;
	scene::Layers.removed(Instance, true);
	push_track_snapshot(std::move(states));
	m_history.back().instances.push_back(Instance);
	// its indicators with it
	for (auto const mark : marks)
	{
		scene::Layers.removed(mark, true);
		m_history.back().instances.push_back(mark);
	}
	ui()->set_status(format(STR_C("Signal %s removed with its event %s, Ctrl+Z brings both back"), values.empty() ? "" : values.front().c_str(), read.c_str()));
	select_include(0);
	m_signal.selected = 0;
	signal_refresh(true);
	return true;
}

void editor_mode::signal_renamed(std::string const &Before, std::string const &After, EditorSnapshot &Snapshot)
{
	auto const name{ToLower(Before)};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		bool carries{false};
		for (int list = 0; list < 6 && false == carries; ++list)
			for (auto const &event : editor_track::events(*track, list))
				carries = carries || ToLower(event.first) == name;
		if (false == carries)
			continue;
		if (std::none_of(Snapshot.tracks.begin(), Snapshot.tracks.end(), [&](auto const &State) { return State.first == track; }))
			Snapshot.tracks.emplace_back(track, editor_track::capture(*track));
		for (int list = 0; list < 6; ++list)
			for (auto &event : editor_track::events(*track, list))
				if (ToLower(event.first) == name)
					event = {ToLower(After), nullptr};
		std::string missing;
		editor_track::bind_events(*track, missing);
		track->mark_dirty();
	}
	if (Snapshot.tracks.empty())
		return;
	if (Snapshot.instance != 0)
	{
		Snapshot.directives.emplace_back(Snapshot.instance, std::move(Snapshot.serialized));
		Snapshot.instance = 0;
		Snapshot.serialized.clear();
	}
	Snapshot.action = EditorSnapshot::Action::TrackEdit;
	Snapshot.node_ptr = Snapshot.tracks.front().first;
	Snapshot.node_name = Snapshot.tracks.front().first->name();
}

std::string editor_mode::signal_plate(std::string const &Name, std::string const &Look)
{
	// a name the plate can't carry: '|' splits the skin, '&' and '?' the query of the script, and a very long one makes a path
	// longer than a file system takes when the skin is looked for on the disk
	if (Name.empty() || Name.size() > 64 || Name.find_first_of("|&?") != std::string::npos)
		return "0_nothing";
	// the template puts it after a folder, "tabl/(p6)": material_data::assign() takes the folder off a texture made by a script
	return kDrawnPlate + std::string{"look="} + Look + "&name=" + Name;
}

bool editor_mode::signal_plate_drawn(std::string const &Plate)
{
	return Plate.starts_with(kDrawnPlate);
}

bool editor_mode::signal_plate_blank(std::string const &Name, std::string const &Plate)
{
	// what the editor gave before the plates were drawn: no plate, or the texture named like the signal, bpd_tm3 by tm3 too
	auto const plate{ToLower(Plate)};
	return Plate.empty() || Plate == "none" || Plate == "0_nothing" || plate == ToLower(Name) || plate == ToLower(signal_plate_text(Name));
}

std::string editor_mode::signal_plate_text(std::string const &Name)
{
	// the station signalling names its signals in small letters after the stem, bpd_tm3: the plate says Tm3, as its schematic does
	auto text{Name.substr(Name.rfind('_') + 1)};
	if (std::any_of(text.begin(), text.end(), [](char const Letter) { return std::isupper(static_cast<unsigned char>(Letter)) != 0; }))
		return text;
	std::transform(text.begin(), text.end(), text.begin(), [](char const Letter) { return static_cast<char>(std::toupper(static_cast<unsigned char>(Letter))); });
	if (text.size() > 2 && text[0] == 'T' && (text[1] == 'M' || text[1] == 'O'))
		text[1] = static_cast<char>(std::tolower(static_cast<unsigned char>(text[1])));
	return text;
}

std::string editor_mode::signal_plate_look(std::string const &Plate)
{
	auto const begin{Plate.find("look=")};
	if (false == signal_plate_drawn(Plate))
		return kPlateLooks[std::clamp(EditorSettings.signal_plate_look(), 0, kPlateLookCount - 1)]; // a plate not drawn yet takes the one set
	if (begin == std::string::npos)
		return kPlateLooks[1]; // what the script draws when it's not told
	auto const end{Plate.find('&', begin)};
	return Plate.substr(begin + 5, end == std::string::npos ? std::string::npos : end - begin - 5);
}

bool editor_mode::standing_signals(std::vector<standing_signal> &Signals, bool const Wait)
{
	Signals.clear();
	signal_scan(Wait);
	if (m_signal.templates.empty())
		return false;
	signal_refresh(true);
	for (auto const &standing : m_signal.standing)
	{
		if (standing.kind < 0)
			continue;
		std::string file;
		std::vector<std::string> values;
		auto const *item{signal_template_of(standing.include.file)};
		auto const linked{item != nullptr && item->linked && editor_includes::parse_directive(scene::Layers.directive(standing.include.instance), file, values) && values.size() > 6};
		Signals.push_back({standing.include.instance, standing.name, kKinds[standing.kind], standing.include.location, standing.include.yaw, linked ? values[6] : std::string{}});
		auto &signal{Signals.back()};
		signal.file = standing.include.file;
		signal.track = standing.track;
		signal.facing = standing.track == nullptr ? 0 : standing.list % 3 == 2 ? 1 : standing.list % 3 == 1 ? -1 : 0;
		signal.read = standing.read;
	}
	return true;
}

bool editor_mode::signal_heads(std::vector<signal_head> &Heads, bool const Wait)
{
	Heads.clear();
	signal_scan(Wait);
	if (m_signal.templates.empty())
		return false;
	for (int index = 0; index < static_cast<int>(m_signal.templates.size()); ++index)
	{
		auto const &item{m_signal.templates[index]};
		// the signalling knows a template by its lights, by the arms and the aspects of a semaphore, by the events a disc repeats
		if (item.lamps.empty() && item.arms == 0 && item.repeats.empty())
			continue;
		auto const kind{kind_index(item.kind)};
		std::vector<std::string> names;
		for (auto const &aspect : item.aspect_events)
			names.push_back(aspect.first);
		// a semaphore described with its arms but not its aspects: those its arms give, Sr1 and Sr2 of the first, Sr3 of the second
		if (names.empty() && item.arms > 0)
		{
			names = {"Sr1", "Sr2"};
			if (item.arms > 1)
				names.emplace_back("Sr3");
		}
		Heads.emplace_back(item.file, item.name, item.kind, item.mount, item.lamps, item.speeds, kind >= 0 && m_signal.chosen[kind] == index, item.aspects, item.lean, item.arms, std::move(names), item.repeats);
	}
	return true;
}

std::string editor_mode::signal_mount(std::string const &Kind, std::string const &File)
{
	signal_scan();
	if (false == File.empty())
	{
		auto const *item{signal_template_of(File)};
		return item != nullptr ? item->mount : std::string{};
	}
	auto const kind{kind_index(Kind)};
	auto const index{kind >= 0 ? m_signal.chosen[kind] : -1};
	return index >= 0 && index < static_cast<int>(m_signal.templates.size()) ? m_signal.templates[index].mount : std::string{};
}

std::size_t editor_mode::signal_batch(signal_changes const &Changes, std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created, std::string &Error)
{
	signal_scan();
	signal_refresh(true);
	std::vector<template_item> items;
	std::vector<std::pair<signal_spot, std::string>> reads;
	for (auto const &order : Changes.placed)
	{
		auto const *item{signal_template_for(order)};
		if (item == nullptr)
		{
			Error = no_template(order.kind);
			break;
		}
		signal_spot spot;
		if (order.track == nullptr || false == signal_spot_at(*order.track, order.path, order.point, order.point, order.facing, order.side, *item, spot))
			continue;
		template_item placing;
		placing.file = signal_variant(*item, spot);
		placing.location = spot.position;
		placing.yaw = spot.yaw;
		placing.described = true;
		placing.values["name"] = order.name;
		auto const *variant{signal_template_of(placing.file)};
		if (variant == nullptr)
			variant = item;
		placing.location.y += signal_default_change(*item, *variant, spot.axis, spot.position);
		if (variant->plate)
			placing.values["#6"] = signal_plate(order.plate, kPlateLooks[std::clamp(EditorSettings.signal_plate_look(), 0, kPlateLookCount - 1)]);
		if (variant->linked)
			placing.values["#7"] = order.linked.empty() ? std::string{"none"} : order.linked;
		items.push_back(std::move(placing));
		reads.emplace_back(spot, variant->read.empty() ? std::string{} : editor_includes::substitute(variant->read, {order.name}));
	}
	if (false == Error.empty())
	{
		// nothing changed but the tracks cut for them, a step of undo
		if (false == States.empty() || false == Created.empty())
			push_track_snapshot(std::move(States), std::move(Created));
		return 0;
	}
	// the names passed round among the signals go through ones of their own first, or a read event renamed would be renamed again
	items.insert(items.end(), Changes.others.begin(), Changes.others.end());
	EditorSnapshot step;
	step.tracks = std::move(States);
	std::vector<std::pair<scene::instance_handle, std::string>> directives;
	std::unordered_set<scene::instance_handle> changed;
	auto const record = [&](scene::instance_handle const Instance, std::string const &Before) {
		if (changed.insert(Instance).second)
			directives.emplace_back(Instance, Before);
	};
	for (auto const temporary : {true, false})
		for (auto const &rename : Changes.renamed)
		{
			auto const *standing{signal_standing_of(rename.instance)};
			auto const *item{standing != nullptr ? signal_template_of(standing->include.file) : nullptr};
			auto const before{scene::Layers.directive(rename.instance)};
			std::string file;
			std::vector<std::string> values;
			if (item == nullptr || false == editor_includes::parse_directive(before, file, values) || values.empty())
				continue;
			if (temporary && ToLower(values[0]) == ToLower(rename.name))
				continue;
			auto const read_before{item->read.empty() ? std::string{} : editor_includes::substitute(item->read, values)};
			auto const named{ToLower(values[0])};
			values[0] = rename.name + (temporary ? "~" : "");
			// a plate picked by hand stays; the drawn one, none, or the texture named like the signal take the new name, in the look
			// the plate had. A plate of no text asked for is one of a signal left for removal
			if (item->plate && values.size() > 5 && false == rename.plate.empty() && (signal_plate_drawn(values[5]) || signal_plate_blank(named, values[5])))
				values[5] = rename.plate == "0_nothing" ? rename.plate : signal_plate(rename.plate, signal_plate_look(values[5]));
			if (item->linked && values.size() > 6 && false == rename.linked.empty())
				values[6] = rename.linked;
			auto const read_after{item->read.empty() ? std::string{} : editor_includes::substitute(item->read, values)};
			auto const directive{editor_includes::compose_directive(file, values)};
			if (directive == before)
				continue;
			record(rename.instance, before);
			set_include_directive(rename.instance, directive);
			if (read_before != read_after)
				signal_renamed(read_before, read_after, step);
			if (temporary)
				continue;
			for (auto const &mark : standing->indicators)
			{
				auto const had{scene::Layers.directive(mark.instance)};
				std::string own;
				std::vector<std::string> placed;
				if (false == scene::Layers.tracked(mark.instance) || false == scene::Layers.removable(mark.instance) || false == editor_includes::parse_directive(had, own, placed) || placed.empty())
					continue;
				placed[0] = rename.name;
				record(mark.instance, had);
				set_include_directive(mark.instance, editor_includes::compose_directive(own, placed));
			}
		}
	for (auto const &[instance, values] : Changes.values)
	{
		auto const before{scene::Layers.directive(instance)};
		std::string file;
		std::vector<std::string> standing;
		if (false == editor_includes::parse_directive(before, file, standing))
			continue;
		if (auto const directive{editor_includes::compose_directive(file, values)}; directive != before)
		{
			record(instance, before);
			set_include_directive(instance, directive);
		}
	}
	for (auto const &[instance, wanted] : Changes.files)
	{
		auto const *standing{signal_standing_of(instance)};
		auto const *item{standing != nullptr ? signal_template_of(standing->include.file) : nullptr};
		auto const *chosen{signal_template_of(wanted)};
		auto const before{scene::Layers.directive(instance)};
		std::string file;
		std::vector<std::string> values;
		if (item == nullptr || chosen == nullptr || chosen->kind != item->kind || false == editor_includes::parse_directive(before, file, values) || values.empty())
			continue;
		signal_spot spot;
		auto const stood{signal_standing_spot(*standing, spot)};
		auto const *variant{stood ? signal_template_of(signal_variant(*chosen, spot)) : chosen};
		if (variant == nullptr)
			variant = chosen;
		if (variant->file == item->file)
			continue;
		include_info had;
		include_info info;
		std::string missing;
		editor_includes::load(item->file, had, missing);
		editor_includes::load(variant->file, info, missing);
		auto const role_of = [](include_info &Info, int const Id) { return Info.parameter(Id).role; };
		std::vector<std::string> after(std::max(variant->parameters, 1));
		for (int id = 1; id <= static_cast<int>(after.size()); ++id)
		{
			auto &value{after[id - 1]};
			auto const role{role_of(info, id)};
			for (int old = 1; old <= static_cast<int>(values.size()) && value.empty(); ++old)
				if (role != "free" && role != "text" && role_of(had, old) == role)
					value = values[old - 1];
			if (id == 7 && variant->linked)
				value = item->linked && values.size() > 6 ? values[6] : std::string{"none"};
			if (value.empty() && id == 6 && variant->plate)
				value = signal_plate(signal_plate_text(values[0]), kPlateLooks[std::clamp(EditorSettings.signal_plate_look(), 0, kPlateLookCount - 1)]);
			if (value.empty())
				value = info.parameter(id).value.empty() ? std::string{"none"} : info.parameter(id).value;
			// as raised over where the template stands by default as it was over where the one it had does
			if (role == "pos.y" && stood)
				value = editor_includes::number(std::atof(value.c_str()) + signal_default_change(*item, *variant, spot.axis, spot.position));
		}
		auto const read_before{item->read.empty() ? std::string{} : editor_includes::substitute(item->read, values)};
		auto const read_after{variant->read.empty() ? std::string{} : editor_includes::substitute(variant->read, after)};
		record(instance, before);
		set_include_directive(instance, editor_includes::compose_directive(variant->file, after));
		if (false == read_before.empty() && false == read_after.empty() && ToLower(read_before) != ToLower(read_after))
			signal_renamed(read_before, read_after, step);
	}
	std::unordered_set<TTrack const *> const created(Created.begin(), Created.end());
	std::vector<scene::instance_handle> removed;
	for (auto const instance : Changes.removed)
	{
		if (false == scene::Layers.tracked(instance) || false == scene::Layers.removable(instance) || scene::Layers.instance(instance).removed)
			continue;
		auto const *standing{signal_standing_of(instance)};
		auto const *item{standing != nullptr ? signal_template_of(standing->include.file) : nullptr};
		std::string file;
		std::vector<std::string> values;
		if (item != nullptr && false == item->read.empty() && editor_includes::parse_directive(scene::Layers.directive(instance), file, values))
		{
			std::vector<std::pair<TTrack *, editor_track::state>> states;
			signal_detach_events(editor_includes::substitute(item->read, values), states);
			for (auto &state : states)
				if (created.count(state.first) == 0 && std::none_of(step.tracks.begin(), step.tracks.end(), [&](auto const &Taken) { return Taken.first == state.first; }))
					step.tracks.push_back(std::move(state));
		}
		scene::Layers.removed(instance, true);
		removed.push_back(instance);
		if (standing != nullptr)
			for (auto const &mark : standing->indicators)
				if (scene::Layers.tracked(mark.instance) && scene::Layers.removable(mark.instance) && false == scene::Layers.instance(mark.instance).removed)
				{
					scene::Layers.removed(mark.instance, true);
					removed.push_back(mark.instance);
				}
	}
	std::vector<scene::instance_handle> instances;
	if (Error.empty() && false == items.empty())
	{
		if (place_templates(items, Error, output_layer("srk")) == 0)
			reads.clear();
		else
		{
			auto placed{std::move(m_history.back())};
			m_history.pop_back();
			instances = std::move(placed.instances);
		}
	}
	else
		reads.clear();
	for (auto const &[spot, read] : reads)
	{
		if (read.empty() || false == spot.reader->Dynamics.empty() || false == scene::Layers.editable(spot.reader))
			continue;
		if (created.count(spot.reader) == 0 && std::none_of(step.tracks.begin(), step.tracks.end(), [&](auto const &State) { return State.first == spot.reader; }))
			step.tracks.emplace_back(spot.reader, editor_track::capture(*spot.reader));
		editor_track::events(*spot.reader, spot.reader_list).emplace_back(ToLower(read), nullptr);
		std::string missing;
		editor_track::bind_events(*spot.reader, missing);
		spot.reader->mark_dirty();
	}
	auto const count{instances.size()};
	instances.insert(instances.end(), removed.begin(), removed.end());
	if (step.tracks.empty() && Created.empty() && instances.empty() && directives.empty())
		return 0;
	if (step.tracks.empty() && Created.empty())
	{
		EditorSnapshot only;
		auto const *file{scene::Layers.instance(directives.empty() ? instances.front() : directives.front().first).file};
		only.node_name = file != nullptr ? *file : std::string{};
		trim_history();
		m_history.push_back(std::move(only));
		g_redo.clear();
	}
	else
		push_track_snapshot(std::move(step.tracks), std::move(Created));
	auto &pushed{m_history.back()};
	pushed.instances = std::move(instances);
	pushed.directives.insert(pushed.directives.end(), directives.begin(), directives.end());
	signal_refresh(true);
	return count;
}

void editor_mode::signal_select(scene::instance_handle const Instance)
{
	if (Instance != m_signal.selected)
		m_signal.picking_read = false;
	m_signal.selected = Instance;
	if (Instance != 0)
		select_include(Instance);
}

void editor_mode::render_signal_palette()
{
	auto &tool{m_signal};
	ImGui::SetNextItemWidth(-1.0f);
	ImGui::InputTextWithHint("##signalfilter", STR_C("filter: name, w24, sbl..."), tool.filter, sizeof(tool.filter));
	auto const filter{ToLower(tool.filter)};
	auto const *current{signal_current()};
	std::vector<std::vector<int>> groups;
	std::vector<std::string> keys;
	for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
	{
		auto const &item{tool.templates[i]};
		if (item.kind != kKinds[tool.kind])
			continue;
		if (false == filter.empty() && item.name.find(filter) == std::string::npos && ToLower(item.description).find(filter) == std::string::npos)
			continue;
		auto const key{group_of(item.mount, item.lamps, item.arms)};
		auto const found{std::find(keys.begin(), keys.end(), key)};
		if (found == keys.end())
		{
			keys.push_back(key);
			groups.push_back({i});
		}
		else
			groups[found - keys.begin()].push_back(i);
	}
	if (groups.empty())
	{
		ImGui::TextDisabled("%s", STR_C("No template of this kind matches the filter"));
		return;
	}
	ImVec2 const tile{52.0f, 86.0f};
	auto const spacing{ImGui::GetStyle().ItemSpacing.x};
	ImGui::BeginChild("##signaltiles", ImVec2(0.0f, std::min(2.0f * (tile.y + 8.0f) + 20.0f, ImGui::GetContentRegionAvail().y * 0.45f)), true);
	auto const columns{std::max(1, static_cast<int>((ImGui::GetContentRegionAvail().x + spacing) / (tile.x + spacing)))};
	auto *draw{ImGui::GetWindowDrawList()};
	for (int g = 0; g < static_cast<int>(groups.size()); ++g)
	{
		auto const &group{groups[g]};
		auto const &first{tool.templates[group.front()]};
		auto const chosen{current != nullptr && group_of(current->mount, current->lamps, current->arms) == keys[g] && current->kind == first.kind};
		if (g % columns != 0)
			ImGui::SameLine();
		ImGui::PushID(g);
		auto const origin{ImGui::GetCursorScreenPos()};
		if (ImGui::InvisibleButton("##tile", tile))
		{
			if (chosen && tool.armed)
				tool.armed = false;
			else
			{
				tool.chosen[tool.kind] = chosen ? tool.chosen[tool.kind] : group.front();
				tool.armed = true;
				tool.picking_read = false;
			}
		}
		auto const hovered{ImGui::IsItemHovered()};
		auto const armed{chosen && tool.armed};
		draw->AddRectFilled(origin, ImVec2(origin.x + tile.x, origin.y + tile.y), armed ? IM_COL32(45, 110, 60, 255) : chosen ? IM_COL32(60, 90, 140, 255) : hovered ? IM_COL32(60, 60, 60, 255) : IM_COL32(40, 40, 40, 255), 4.0f);
		// a semaphore with the arms of the one chosen on its tile
		auto const &shown{chosen ? *current : first};
		draw_signal_icon(draw, origin, ImVec2(tile.x, tile.y - 14.0f), first.mount, first.lamps, chosen ? overlay_color::selected : IM_COL32(120, 120, 120, 255), shown.arms);
		// a semaphore says arms, a disc its name, a signal with lights its lights
		std::string caption{first.lamps.empty() ? first.name : first.lamps};
		if (first.arms > 0)
			caption = STR_C("arms");
		auto const captionsize{ImGui::CalcTextSize(caption.c_str())};
		draw->AddText(ImVec2(origin.x + std::max(2.0f, (tile.x - captionsize.x) * 0.5f), origin.y + tile.y - 15.0f), IM_COL32(220, 220, 220, 255), caption.c_str());
		if (group.size() > 1)
			draw->AddText(ImVec2(origin.x + 3.0f, origin.y + 2.0f), IM_COL32(170, 170, 170, 255), std::to_string(group.size()).c_str());
		if (hovered)
		{
			ImGui::BeginTooltip();
			ImGui::TextUnformatted(first.description.empty() ? first.name.c_str() : first.description.c_str());
			ImGui::TextDisabled(STR_C("%zu variants, e.g. %s"), group.size(), first.file.c_str());
			ImGui::TextDisabled("%s", first.read.empty() ? STR_C("The train doesn't read it") : STR_C("The train reads it by an event of the path"));
			ImGui::TextDisabled("%s", armed ? STR_C("Click: stop placing") : STR_C("Click: place it by a track, the signalling of the station places its signals of this kind with it too"));
			ImGui::EndTooltip();
		}
		ImGui::PopID();
	}
	ImGui::EndChild();
	if (current == nullptr)
		return;
	auto const key{group_of(current->mount, current->lamps, current->arms)};
	ImGui::SetNextItemWidth(-1.0f);
	if (ImGui::BeginCombo("##signalvariant", current->name.c_str()))
	{
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		{
			auto const &item{tool.templates[i]};
			if (item.kind != current->kind || group_of(item.mount, item.lamps, item.arms) != key)
				continue;
			auto const label{item.name + (item.lean == "left" ? std::string{"  \xE2\x86\x96"} : item.lean == "right" ? std::string{"  \xE2\x86\x97"} : std::string{}) + "##" + item.file};
			if (ImGui::Selectable(label.c_str(), &item == current))
				tool.chosen[tool.kind] = i;
			if (ImGui::IsItemHovered() && false == item.description.empty())
				ImGui::SetTooltip("%s\n%s", item.description.c_str(), item.file.c_str());
		}
		ImGui::EndCombo();
	}
	if (ImGui::IsItemHovered())
	{
		auto const read{current->read.empty() ? std::string{current->kind == "warning" || current->kind == "repeater" ? STR_C("The train doesn't read it: it repeats what the signal ahead of it shows") : STR_C("The train doesn't read it")}
		                                      : format(STR_C("The train reads it by %s on the path it stands by"), editor_includes::substitute(current->read, {tool.name[0] != '\0' ? std::string{tool.name} : std::string{"?"}}).c_str())};
		ImGui::SetTooltip("%s\n%s", STR_C("Variant of the chosen type. A head leaning to one side is swapped for its counterpart leaning towards the track on the other side"), read.c_str());
	}
	if (current->arms > 0)
	{
		// a semaphore: its arms and the substitute signal picked, the template with them taken, of the same tile
		auto const substitute = [](signal_template const &Item) {
			return std::ranges::any_of(Item.aspect_events, [](auto const &Aspect) { return Aspect.first == "Sz"; });
		};
		// of those with them, the one leaning as the chosen one and most like it by its name
		auto const with = [&](int const Arms, bool const Substitute) {
			auto best{-1};
			std::pair<bool, std::size_t> rank;
			for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
			{
				auto const &item{tool.templates[i]};
				if (item.kind != current->kind || group_of(item.mount, item.lamps, item.arms) != key || item.arms != Arms || substitute(item) != Substitute)
					continue;
				std::pair<bool, std::size_t> const score{item.lean != current->lean, edit_distance(item.name, current->name)};
				if (best < 0 || score < rank)
				{
					best = i;
					rank = score;
				}
			}
			return best;
		};
		auto const sz{substitute(*current)};
		auto const arms{current->arms};
		for (auto const count : {1, 2})
		{
			auto const found{with(count, sz)};
			ImGui::BeginDisabled(found < 0);
			if (ImGui::RadioButton(count == 1 ? STR_C("1 arm") : STR_C("2 arms"), arms == count) && found >= 0)
				tool.chosen[tool.kind] = found;
			ImGui::EndDisabled();
			if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			{
				auto const *meaning{count == 1 ? STR_C("Sr1 stop, Sr2 go at the highest speed") : STR_C("Sr3 too, with the second arm: go at no more than 40 km/h")};
				ImGui::SetTooltip("%s", found < 0 ? STR_C("No template of this tile has it") : meaning);
			}
			ImGui::SameLine();
		}
		auto const other{with(arms, false == sz)};
		auto shown{sz};
		ImGui::BeginDisabled(other < 0);
		if (ImGui::Checkbox(STR_C("Sz"), &shown) && other >= 0)
			tool.chosen[tool.kind] = other;
		ImGui::EndDisabled();
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("%s", other < 0 ? STR_C("No template of this tile has it") : STR_C("The substitute signal under the arms: it lets a train past the semaphore at stop"));
	}
	if (false == current->description.empty())
		ImGui::TextWrapped("%s", current->description.c_str());
	if (shaped_disc(current->kind, current->lamps, current->arms))
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", STR_C("The signalling of the station doesn't place a shaped disc by itself, it takes a light one there: place it by hand, or give a signal standing its template"));
		ImGui::PopTextWrapPos();
	}
	if (tool.armed)
	{
		if (ImGui::Button(STR_C("Stop placing (Esc)"), ImVec2(-1.0f, 0.0f)))
			tool.armed = false;
	}
	else if (main_button(STR_C("Place by a track"), ImVec2(-1.0f, 0.0f)))
	{
		tool.armed = true;
		tool.picking_read = false;
	}
	item_tooltip("A click by a track places the chosen signal on that side, a press dragged along it turns it to the trains running that way. The signalling of the station places its signals of each kind with the template chosen here");
}

std::string editor_mode::signal_swap_combo(signal_template const &Item, bool const Across) const
{
	auto const shown = [](signal_template const &Template) { return Template.lamps.empty() ? Template.name : Template.name + "  " + Template.lamps; };
	std::string swap;
	ImGui::SetNextItemWidth(-1.0f);
	if (false == ImGui::BeginCombo("##signalswap", format(STR_C("Template: %s"), shown(Item).c_str()).c_str(), ImGuiComboFlags_HeightLarge))
		return swap;
	for (auto const &other : m_signal.templates)
		if (other.kind == Item.kind && other.mount == Item.mount && ImGui::Selectable((shown(other) + "##" + other.file).c_str(), other.file == Item.file) && other.file != Item.file)
			swap = other.file;
	if (Across)
	{
		auto first{true};
		for (auto const &other : m_signal.templates)
		{
			if (other.kind != Item.kind || other.mount == Item.mount || other.mount == "gantry")
				continue;
			if (std::exchange(first, false))
				ImGui::SeparatorText(STR_C("Of another mount"));
			if (ImGui::Selectable((shown(other) + "  (" + mount_label(other.mount) + ")##" + other.file).c_str(), false))
				swap = other.file;
		}
	}
	ImGui::EndCombo();
	return swap;
}

void editor_mode::render_signal_aspects(signal_standing const &Found, signal_template const &Item)
{
	// the aspects its description lists, each setting the signal to show it here
	ImGui::AlignTextToFramePadding();
	ImGui::TextUnformatted(STR_C("Show:"));
	for (auto const &[aspect, called] : Item.aspect_events)
	{
		ImGui::SameLine();
		if (ImGui::Button((aspect + "##aspect" + called).c_str()))
			signal_show_aspect(Found.include.instance, aspect, called);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip(STR_C("Sets it to %s here, as its event %s sets it in the game. Only how it looks: the trains read it as its events set it last, in a run started from here too. Nothing of it is saved, the scenery opens with it as it was"), aspect.c_str(), editor_includes::substitute("(p1)_" + called, {Found.name}).c_str());
	}
}

bool editor_mode::render_signal_template(signal_standing const &Found, signal_template const &Item)
{
	// a shunting or a stop disc, light or shaped, takes one of the other kind of look or of another mount, the gantries apart
	auto const across{(Item.kind == "shunt" || Item.kind == "stop") && Item.mount != "gantry"};
	auto const swap{signal_swap_combo(Item, across)};
	if (across)
		item_tooltip("Another template of its kind for the signal, light or shaped, of its mount or another: the name, the plate, the place and the event the train reads it by stay. Ctrl+Z takes it back");
	else
		item_tooltip("Another template of its kind and mount for the signal: the name, the plate, the place and the event the train reads it by stay. A head leaning to one side is swapped for its counterpart leaning towards the track. Ctrl+Z takes it back");
	if (false == swap.empty())
	{
		signal_swap(Found.include.instance, swap);
		return true;
	}
	if (false == Item.aspect_events.empty())
		render_signal_aspects(Found, Item);
	else if (shaped_disc(Item.kind, Item.lamps, Item.arms))
		ImGui::TextDisabled("%s", STR_C("It always shows the same: no aspect to set"));
	return false;
}

void editor_mode::render_signal_selected()
{
	auto const *found{m_signal.selected == m_instance ? signal_standing_of(m_signal.selected) : nullptr};
	if (found == nullptr)
		return;
	ImGui::TextUnformatted(format(STR_C("Selected: %s (%s)"), found->name.c_str(), found->include.file.c_str()).c_str());
	item_tooltip("Name, plate and the linked signal: in the node properties. Del removes it with its event");
	if (found->read.empty())
		ImGui::TextDisabled("%s", STR_C("The train doesn't read it"));
	else if (render_signal_reads(*found))
		return;
	// where it stands: from the axis of its track, along it to the switch ahead
	if (auto const &metre{signal_metre_of(*found)}; metre.valid && begin_fields("##signalmetre"))
	{
		auto const *shape{signal_template_of(found->include.file)};
		auto const gantry{shape != nullptr && shape->mount == "gantry"};
		field_row("From the track axis");
		if (gantry)
			ImGui::Text("%.2f m", metre.spot.offset);
		else
		{
			auto offset{static_cast<float>(metre.spot.offset)};
			auto const before{offset};
			ImGui::SetNextItemWidth(-1.0f);
			ImGui::InputFloat("##signalacross", &offset, 0.05f, 0.25f, "%.2f m");
			if (ImGui::IsItemDeactivatedAfterEdit() && std::abs(offset - before) > 0.005f)
			{
				ImGui::EndTable();
				signal_set_across(*found, std::clamp(static_cast<double>(offset), 0.0, 10.0), std::nullopt);
				return;
			}
		}
		item_tooltip("The distance of the mast from the axis of the track it stands by, in the plan: type another one and the signal moves across, as far along the track as it stands. Not nearer than the gauge lets it: the gauge chosen in the Gauge window pushes it out again. One step of undo");
		field_row(gantry ? "Over the rails (m)" : "Raised (m)");
		{
			auto height{static_cast<float>(found->include.location.y - (shape != nullptr ? signal_default_height(*shape, metre.spot.axis, found->include.location) : metre.spot.axis.y + kRailTop))};
			auto const before{height};
			ImGui::SetNextItemWidth(-1.0f);
			ImGui::InputFloat("##signalfoot", &height, 0.05f, 0.25f, "%.2f m");
			if (ImGui::IsItemDeactivatedAfterEdit() && std::abs(height - before) > 0.005f)
			{
				ImGui::EndTable();
				signal_set_across(*found, std::nullopt, std::clamp(static_cast<double>(height), -10.0, 20.0));
				return;
			}
		}
		item_tooltip(gantry ? "The head hangs this high over the rail heads of its track. One step of undo" : "0: the signal stands as high as its include gives for the rail heads of its track, by Ie-117 the top of its foundation 0.10 m under them, of a dwarf 0.20 m; where the ground falls away, down an embankment, lower, its foundation on the ground. Raise it where it stands higher, on a platform. One step of undo");
		field_row("To the switch ahead");
		auto const text{danger_text(metre.danger)};
		if (metre.danger.what == track_trace::danger::kind::none)
			ImGui::TextDisabled("%s", text.c_str());
		else if (metre.danger.distance < 0.0)
			ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", text.c_str());
		else
			ImGui::TextWrapped("%s", text.c_str());
		item_tooltip("Along the track the trains it faces run: to the start of the first switch they run onto, or to the fouling point of one they run onto from behind, where the axes of its tracks are 3.50 m apart, more in a curve under 250 m by the widening of the gauge and on a cant by the leaning of the cars, a W17 marker there or not. Dragged or placed the signal goes to 50, 100, 150 m... from it when that near, Alt keeps it where it is");
		if (metre.danger.what == track_trace::danger::kind::fouling)
			ImGui::TextDisabled("%s", format(STR_C("the axes %.2f m apart there"), metre.danger.spacing).c_str());
		ImGui::EndTable();
	}
	auto const instance{found->include.instance};
	if (auto const *item{signal_template_of(found->include.file)}; item != nullptr && render_signal_template(*found, *item))
		return;
	auto const width{(ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f};
	// either refreshes the signals standing, found pointing into the list of before: the rest of the card in the next frame
	if (ImGui::Button(STR_C("To the other side"), ImVec2(width, 0.0f)))
	{
		signal_flip(instance);
		return;
	}
	item_tooltip("Takes the signal across the track, it keeps facing the same trains. Dragging it over the track does the same");
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Turn around"), ImVec2(width, 0.0f)))
	{
		signal_turn(instance);
		return;
	}
	item_tooltip("Turns the signal to the trains of the other direction, they read it by the other event of the path");
	if (render_signal_indicators(*found))
		return;
	// the gauge of the tracks around, measured again after each change
	auto &gauge{m_signal.gauge};
	if (gauge.instance != instance || gauge.history != history_revision() || gauge.location != found->include.location)
	{
		gauge.instance = instance;
		gauge.history = history_revision();
		gauge.location = found->include.location;
		gauge.hit = signal_gauge(instance);
	}
	if (gauge.hit.depth > kGaugeSlack && gauge.hit.track != nullptr)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), STR_C("In the gauge of %s by %.0f cm"), name_of(*gauge.hit.track).c_str(), gauge.hit.depth * 100.0);
		auto const *item{signal_template_of(found->include.file)};
		if ((item == nullptr || item->mount != "gantry") && ImGui::Button(STR_C("Move it out of the gauge"), ImVec2(-1.0f, 0.0f)))
		{
			signal_spot spot;
			if (signal_standing_spot(*found, spot))
				signal_move_to(*found, spot, STR_C("Signal %s moved out of the gauge, read by %s %s of %s, Ctrl+Z takes it back"), STR_C("Signal %s moved out of the gauge, Ctrl+Z takes it back"));
			else
				m_signal.status = refused(STR_C("No track by the signal to take it from"));
			return;
		}
		item_tooltip("Moves the signal sideways, away from the axis of the track whose gauge it enters, as far as it goes in. The gauge is the one chosen in the Gauge window");
	}
	ImGui::Separator();
}

bool editor_mode::render_signal_reads(signal_standing const &Found)
{
	auto &tool{m_signal};
	if (Found.reads.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), STR_C("No path has %s among its events: the train doesn't see it"), Found.read.c_str());
	else
		ImGui::TextDisabled(STR_C("Read by %s on:"), Found.read.c_str());
	for (std::size_t i = 0; i < Found.reads.size(); ++i)
	{
		auto const &[track, list] = Found.reads[i];
		ImGui::PushID(static_cast<int>(i));
		auto const &events{editor_track::events(*track, list)};
		auto const bound{std::any_of(events.begin(), events.end(), [&](auto const &Event) { return Event.second != nullptr && ToLower(Event.first) == ToLower(Found.read); })};
		ImGui::Bullet();
		ImGui::SameLine();
		ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kReadColour), "%s %s", editor_track::event_keyword(list), name_of(*track).c_str());
		if (false == bound)
			item_tooltip("Bound to its event when the scenery is saved and opened again");
		ImGui::SameLine();
		auto changed{false};
		auto reads{Found.reads};
		if (ImGui::SmallButton(STR_C("Other way")))
		{
			reads[i].second = list % 3 == 2 ? list - 1 : list % 3 == 1 ? list + 1 : list;
			changed = true;
		}
		item_tooltip("The trains of the other direction read it on this path: event1 and event2 swapped");
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Remove")))
		{
			reads.erase(reads.begin() + static_cast<std::ptrdiff_t>(i));
			changed = true;
		}
		item_tooltip("This path doesn't read it any more");
		ImGui::PopID();
		if (changed)
			return signal_read_set(Found, reads);
	}
	auto const width{(ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f};
	if (tool.picking_read)
		ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
	auto const picking{tool.picking_read};
	if (ImGui::Button(picking ? STR_C("Picking a path... (Esc)") : STR_C("Pick the path"), ImVec2(width, 0.0f)))
	{
		tool.picking_read = false == picking;
		if (tool.picking_read)
			tool.armed = false;
	}
	if (picking)
		ImGui::PopStyleColor();
	item_tooltip("Click a track in the view: it reads the signal instead, Shift+click: one read more or less. The list is the one of the trains the signal faces");
	ImGui::SameLine();
	if (ImGui::Button(STR_C("By the path it stands by"), ImVec2(width, 0.0f)))
	{
		signal_spot spot;
		if (signal_standing_spot(Found, spot) && spot.reader != nullptr)
			return signal_read_set(Found, {{spot.reader, spot.reader_list}});
		tool.status = refused(STR_C("No track by the signal to read it"));
	}
	item_tooltip("The read where the editor puts it: the event of the trains it faces on the track it stands by, past the frog of a switch on the track behind it");
	return false;
}

void editor_mode::select_signal(scene::instance_handle const Instance)
{
	signal_refresh(true);
	signal_select(Instance);
}

void editor_mode::render_signal_templates(std::vector<std::string> const &Kinds)
{
	auto &tool{m_signal};
	signal_scan(false);
	ImGui::SeparatorText(STR_C("Templates"));
	if (tool.templates.empty())
	{
		ImGui::TextWrapped("%s", EditorIncludes.searching() ? STR_C("Looking for the templates...") : STR_C("No signal templates: no template of the scenery directory is described with category: signal. tools/editor/signal_headers.py writes the descriptions into the signal templates."));
		return;
	}
	if (false == begin_fields("##signalkinds", 0.4f))
		return;
	auto const shown = [](signal_template const &Template) { return Template.lamps.empty() ? Template.name : Template.name + "  " + Template.lamps; };
	for (int k = 0; k < kKindCount; ++k)
	{
		if (false == Kinds.empty() && std::find(Kinds.begin(), Kinds.end(), kKinds[k]) == Kinds.end())
			continue;
		field_row(kKindLabels[k]);
		auto const index{tool.chosen[k]};
		auto const *current{index >= 0 && index < static_cast<int>(tool.templates.size()) ? &tool.templates[index] : nullptr};
		ImGui::PushID(k);
		ImGui::SetNextItemWidth(-1.0f);
		if (ImGui::BeginCombo("##kindtemplate", current != nullptr ? shown(*current).c_str() : STR_C("none"), ImGuiComboFlags_HeightLarge))
		{
			for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
			{
				auto const &item{tool.templates[i]};
				if (item.kind != kKinds[k])
					continue;
				if (ImGui::Selectable((shown(item) + "##" + item.file).c_str(), i == index))
					tool.chosen[k] = i;
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s\n%s", item.description.empty() ? item.name.c_str() : item.description.c_str(), item.file.c_str());
			}
			ImGui::EndCombo();
		}
		item_tooltip("The template the signals of this kind are placed with; the Signals tool places it too");
		ImGui::PopID();
	}
	ImGui::EndTable();
}

void editor_mode::render_signal_ui()
{
	auto &tool{m_signal};
	signal_scan(false);
	if (tool.templates.empty() && EditorIncludes.searching())
	{
		ImGui::TextDisabled("%s", STR_C("Looking for the templates..."));
		return;
	}
	if (tool.templates.empty())
	{
		ImGui::TextWrapped("%s", STR_C("No signal templates: no template of the scenery directory is described with category: signal. tools/editor/signal_headers.py writes the descriptions into the signal templates."));
		return;
	}
	std::vector<std::string> labels;
	labels.reserve(kKindCount);
	std::vector<segment_option> kinds;
	for (int k = 0; k < kKindCount; ++k)
		if (std::any_of(tool.templates.begin(), tool.templates.end(), [&](signal_template const &Item) { return Item.kind == kKinds[k]; }))
		{
			labels.push_back(STR(kKindLabels[k]));
			kinds.push_back({k, labels.back().c_str()});
		}
	if (false == kinds.empty() && std::none_of(kinds.begin(), kinds.end(), [&](segment_option const &Option) { return Option.value == tool.kind; }))
		tool.kind = kinds.front().value;
	choice_chips("signalkinds", tool.kind, kinds);
	render_signal_palette();
	auto const *armed{signal_current()};
	if (begin_fields("##signalnames", 0.3f))
	{
		field_row("Name");
		ImGui::InputText("##signalname", tool.name, sizeof(tool.name));
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Name of the next signal; it counts up after each one placed. Its events and memory cell are named after it"));
		ImGui::EndTable();
	}
	ImGui::PushID("signalmore");
	if (ImGui::TreeNode(STR_C("More settings")))
	{
		bool missing{false};
		if (begin_fields("##signalplace", 0.45f))
		{
			if (armed != nullptr && armed->plate)
			{
				auto const look{std::clamp(EditorSettings.signal_plate_look(), 0, kPlateLookCount - 1)};
				auto const plate{signal_plate(tool.name, kPlateLooks[look])};
				auto const automatic{signal_plate_drawn(plate) ? format(STR_C("auto: drawn \"%s\""), tool.name) : format(STR_C("auto: %s"), plate.c_str())};
				field_row("Plate");
				ImGui::InputTextWithHint("##signalplate", automatic.c_str(), tool.plate, sizeof(tool.plate));
				auto const typing{ImGui::IsItemActive()};
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("Texture of the plate from textures/tabl, e.g. a-12 or tob. Left empty: a plate with the name of the signal, drawn by python/local/signal_plate.py"));
				auto const &plates{plate_textures()};
				auto const typed{ToLower(tool.plate)};
				auto const exists{typed.empty() || std::binary_search(plates.begin(), plates.end(), typed)};
				if (false == exists)
					ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", STR_C("no such texture"));
				missing = typing || (false == exists && false == typed.empty());
				if (typed.empty())
				{
					field_row("Plate look");
					if (ImGui::BeginCombo("##signalplatelook", STR_C(kPlateLookLabels[look])))
					{
						for (int i = 0; i < kPlateLookCount; ++i)
							if (ImGui::Selectable(STR_C(kPlateLookLabels[i]), i == look))
							{
								EditorSettings.signal_plate_look(i);
								EditorSettings.save();
							}
						ImGui::EndCombo();
					}
					if (ImGui::IsItemHovered())
						ImGui::SetTooltip("%s", STR_C("How the drawn plates look: plain is clean and bright, toned greyish and a little dusty, aged darker and dusty. The look is kept with the plate, a rename doesn't change it"));
				}
			}
			if (armed != nullptr && armed->linked)
			{
				field_row("Linked signal");
				ImGui::InputText("##linked", tool.linked, sizeof(tool.linked));
				item_tooltip("(p7) of the template: the warning signal a semaphore drives, the previous block signal, \"none\" for no link");
			}
			auto offset{static_cast<float>(EditorSettings.signal_offset())};
			field_row("From the axis (m)");
			if (ImGui::InputFloat("##signaloffset", &offset, 0.05f, 0.25f, "%.2f"))
			{
				EditorSettings.signal_offset(std::clamp(static_cast<double>(offset), 0.0, 10.0));
				EditorSettings.save();
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("Distance of the mast from the axis of the track. 2.25 m is what the signals of the sceneries stand at"));
			if (armed != nullptr && armed->mount == "gantry")
			{
				field_row("Over the rails (m)");
				ImGui::InputFloat("##signalheight", &tool.height, 0.1f, 0.5f, "%.2f");
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("A head for a gantry hangs over the axis of the track this high over the rail heads"));
			}
			ImGui::EndTable();
		}
		if (missing)
		{
			auto const &plates{plate_textures()};
			auto const typed{ToLower(tool.plate)};
			int shown{0};
			for (auto const &plate : plates)
			{
				if (typed.empty() || plate.compare(0, typed.size(), typed) != 0 || plate == typed)
					continue;
				if (shown % 6 != 0)
					ImGui::SameLine();
				if (ImGui::SmallButton(plate.c_str()))
					std::snprintf(tool.plate, sizeof(tool.plate), "%s", plate.c_str());
				if (++shown >= 18)
					break;
			}
		}
		ImGui::TreePop();
	}
	ImGui::PopID();
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

void editor_mode::draw_signal_overlay() const
{
	auto const &tool{m_signal};
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	glm::dvec3 const camera{Global.pCamera.Pos};
	auto const arrow = [&](glm::dvec3 const &From, glm::dvec2 const &Direction, ImU32 const Colour) {
		glm::dvec3 const tip{From.x + Direction.x * 6.0, From.y, From.z + Direction.y * 6.0};
		ImVec2 a, b;
		if (false == projection.project(From, a) || false == projection.project(tip, b))
			return;
		drawlist->AddLine(a, b, Colour, 2.5f);
		auto const dx{b.x - a.x};
		auto const dy{b.y - a.y};
		auto const size{std::sqrt(dx * dx + dy * dy)};
		if (size < 1.0f)
			return;
		ImVec2 const unit{dx / size, dy / size};
		ImVec2 const side{-unit.y, unit.x};
		drawlist->AddTriangleFilled(ImVec2(b.x + unit.x * 9.0f, b.y + unit.y * 9.0f), ImVec2(b.x + side.x * 6.0f, b.y + side.y * 6.0f), ImVec2(b.x - side.x * 6.0f, b.y - side.y * 6.0f), Colour);
	};
	auto const mast = [&](glm::dvec3 const &Foot, ImU32 const Colour, float const Width) { projection.line(drawlist, Foot, Foot + glm::dvec3{0.0, kSignalHead, 0.0}, Colour, Width); };
	// a path reading a signal: a thick band along it over the rails, outlined dark to stand out of the ballast, arrows of the way of
	// the trains reading it along the band, a line from the foot of the signal to the path, the event and the path named on a plate
	glm::dvec3 const lift{0.0, kRailTop + 0.05, 0.0};
	ImU32 const outline{IM_COL32(15, 15, 25, 170)};
	auto const thick = [&](glm::dvec3 const &A, glm::dvec3 const &B, ImU32 const Colour, float const Width) {
		projection.line(drawlist, A, B, outline, Width + 3.0f);
		projection.line(drawlist, A, B, Colour, Width);
	};
	auto const read_arrow = [&](glm::dvec3 const &From, glm::dvec2 const &Direction, ImU32 const Colour, float const Width) {
		ImVec2 a;
		ImVec2 b;
		if (false == projection.project(From, a) || false == projection.project(glm::dvec3{From.x + Direction.x * 4.0, From.y, From.z + Direction.y * 4.0}, b))
			return;
		auto const dx{b.x - a.x};
		auto const dy{b.y - a.y};
		auto const size{std::sqrt(dx * dx + dy * dy)};
		if (size < 1.0f)
			return;
		ImVec2 const unit{dx / size, dy / size};
		ImVec2 const side{-unit.y, unit.x};
		auto const head{6.0f + Width * 2.0f};
		ImVec2 const tip{b.x + unit.x * head, b.y + unit.y * head};
		ImVec2 const left{b.x + side.x * head * 0.7f, b.y + side.y * head * 0.7f};
		ImVec2 const right{b.x - side.x * head * 0.7f, b.y - side.y * head * 0.7f};
		drawlist->AddTriangleFilled(tip, left, right, Colour);
		drawlist->AddTriangle(tip, left, right, outline, 1.5f);
	};
	auto const plate = [&](glm::dvec3 const &At, ImU32 const Colour, std::string const &Text) {
		ImVec2 at;
		if (false == projection.project(At, at))
			return;
		auto *font{ImGui::GetFont()};
		auto const size{ImGui::GetFontSize() * 1.2f};
		auto const extent{font->CalcTextSizeA(size, FLT_MAX, 0.0f, Text.c_str())};
		ImVec2 const corner{at.x + 10.0f, at.y + 8.0f};
		drawlist->AddRectFilled(ImVec2(corner.x - 4.0f, corner.y - 2.0f), ImVec2(corner.x + extent.x + 4.0f, corner.y + extent.y + 2.0f), IM_COL32(15, 15, 25, 210), 4.0f);
		drawlist->AddText(font, size, corner, Colour, Text.c_str());
	};
	auto const reading = [&](TTrack const &Track, int const List, glm::dvec3 const &Foot, ImU32 const Colour, float const Width) {
		if (Track.m_paths.empty())
			return;
		auto const way{List % 3 == 1 ? -1.0 : 1.0};
		for (int p = 0; p < static_cast<int>(Track.m_paths.size()); ++p)
		{
			bezier const curve{Track.m_paths[p]};
			auto previous{curve.point(0.0) + lift};
			for (int k = 1; k <= 32; ++k)
			{
				auto const next{curve.point(k / 32.0) + lift};
				thick(previous, next, Colour, Width);
				previous = next;
			}
			// an arrow each 15 m or so, the way the trains reading it run
			auto const length{geometry::plan_distance(curve.point(0.0), curve.point(1.0))};
			auto const count{std::clamp(static_cast<int>(length / 15.0), 1, 12)};
			for (int k = 0; k < count; ++k)
			{
				auto const t{(k + 0.5) / count};
				read_arrow(curve.point(t) + lift, forward_at(Track, p, t) * way, Colour, Width);
			}
		}
		int path{0};
		double parameter{0.0};
		plan_gap(Track, Foot, path, parameter);
		auto const closest{bezier{Track.m_paths[path]}.point(parameter) + lift};
		thick(Foot, closest, Colour, std::max(2.0f, Width * 0.5f));
		plate(closest, Colour, format("%s %s", editor_track::event_keyword(List), name_of(Track).c_str()));
	};
	auto const dim = [](ImU32 const Colour) { return (Colour & 0x00FFFFFFu) | (static_cast<ImU32>(160) << IM_COL32_A_SHIFT); };
	// the way of a signal along its track to the switch ahead, a bar across at the place, how far written there, and its distance from
	// the axis of the track by its foot
	auto const way = [&](signal_spot const &Spot, track_trace::danger const &Danger) {
		plate(Spot.axis + lift, IM_COL32(220, 220, 220, 235), format(STR_C("%.2f m from the axis"), Spot.offset));
		if (Danger.what == track_trace::danger::kind::none || Danger.line.size() < 2)
			return;
		auto const colour{Danger.distance < 0.0 ? overlay_color::invalid : kWayColour};
		auto const low{std::min(0.0, Danger.distance)};
		auto const high{std::max(0.0, Danger.distance)};
		// a dimension line by the track on the side of the signal, off the band of the path reading it
		auto const sign{glm::dot(right_of(Spot.travel), plan_of(Spot.position) - plan_of(Spot.axis)) >= 0.0 ? 1.0 : -1.0};
		std::pair<TTrack *, int> unused;
		auto const across_at = [&](double const Distance) {
			auto const along{plan_of(Danger.at(Distance + 1.0, unused) - Danger.at(Distance - 1.0, unused))};
			return glm::length(along) > 1e-6 ? right_of(glm::normalize(along)) : glm::dvec2{0.0};
		};
		auto const aside = [&](double const Distance, double const Off) {
			auto const across{across_at(Distance) * sign * Off};
			return Danger.at(Distance, unused) + lift + glm::dvec3{across.x, 0.0, across.y};
		};
		auto previous{aside(low, kWayAside)};
		// each whole metre between them, counted by an int: the same points as stepping a double by 1, without a floating counter
		auto const first{std::floor(low) + 1.0};
		for (int whole = 0; first + whole < high; ++whole)
		{
			auto const next{aside(first + whole, kWayAside)};
			thick(previous, next, colour, 3.0f);
			previous = next;
		}
		thick(previous, aside(high, kWayAside), colour, 3.0f);
		// its ends: across the track by the signal, across it all at the place
		thick(aside(0.0, kWayAside - 0.5), aside(0.0, kWayAside + 0.5), colour, 3.0f);
		thick(aside(Danger.distance, -2.5), aside(Danger.distance, 2.5), colour, 4.0f);
		plate(aside(Danger.distance, kWayAside), colour, danger_text(Danger));
	};
	for (int i = 0; i < static_cast<int>(tool.standing.size()); ++i)
	{
		auto const &standing{tool.standing[i]};
		auto const &location{standing.include.location};
		if (glm::distance(location, camera) > kSignalRange)
			continue;
		auto const selected{standing.include.instance == tool.selected};
		auto const hovered{m_intent.what == track_intent::kind::signal_move && m_intent.index == i};
		auto const unread{false == standing.read.empty() && standing.track == nullptr};
		auto const colour{selected ? overlay_color::selected : hovered ? overlay_color::highlight : unread ? overlay_color::invalid : IM_COL32(255, 210, 60, 160)};
		mast(location, colour, selected || hovered ? 3.0f : 1.5f);
		ImVec2 head;
		if ((selected || hovered || unread) && projection.project(location + glm::dvec3{0.0, kSignalHead, 0.0}, head))
		{
			auto const label{standing.name + (unread ? std::string{"  "} + STR(" not read") : std::string{})};
			drawlist->AddText(ImVec2(head.x + 6.0f, head.y - 6.0f), colour, label.c_str());
		}
		if (selected && false == tool.moving)
			if (auto const &metre{signal_metre_of(standing)}; metre.valid)
				way(metre.spot, metre.danger);
		if (selected || hovered)
		{
			arrow(location, travel_of(standing.include.yaw), colour);
			// not while it is dragged: the read it gets where it is dropped is drawn then
			if (false == (selected && tool.moving && tool.moved_valid))
				for (auto const &[track, list] : standing.reads)
					reading(*track, list, location, selected ? kReadColour : dim(kReadColour), selected ? 7.0f : 4.0f);
		}
	}
	// the path a click gives the read of the selected signal to
	if (tool.picking_read && m_intent.what == track_intent::kind::signal_read && m_intent.track != nullptr)
		if (auto const *picked{signal_standing_of(tool.selected)}; picked != nullptr)
			reading(*m_intent.track, signal_read_list(*picked, *m_intent.track, m_intent.path), picked->include.location, overlay_color::highlight, 8.0f);
	auto const preview = [&](signal_spot const &Spot, ImU32 const Colour, bool const Read) {
		projection.line(drawlist, Spot.axis + glm::dvec3{0.0, kRailTop, 0.0}, Spot.position, Colour, 2.0f);
		mast(Spot.position, Colour, 3.0f);
		arrow(Spot.axis + glm::dvec3{0.0, kRailTop, 0.0}, Spot.travel, Colour);
		if (Read && Spot.reader != nullptr)
			reading(*Spot.reader, Spot.reader_list, Spot.position, kReadColour, 6.0f);
		way(Spot, signal_danger(Spot));
		ImVec2 head;
		auto const *armed{signal_armed()};
		if (armed != nullptr && false == armed->lamps.empty() && projection.project(Spot.position + glm::dvec3{0.0, kSignalHead, 0.0}, head))
		{
			auto const lamps{static_cast<int>(armed->lamps.size())};
			drawlist->AddRectFilled(ImVec2(head.x - 6.0f, head.y - 4.0f), ImVec2(head.x + 6.0f, head.y + lamps * 10.0f), IM_COL32(20, 20, 20, 220), 3.0f);
			for (int i = 0; i < lamps; ++i)
				drawlist->AddCircleFilled(ImVec2(head.x, head.y + 1.0f + i * 10.0f), 4.0f, lamp_colour(armed->lamps[i]));
		}
	};
	auto const *template_armed{signal_armed()};
	auto const *dragged{tool.moving ? signal_standing_of(tool.selected) : nullptr};
	if (tool.placing)
		preview(tool.press, overlay_color::grip, template_armed != nullptr && false == template_armed->read.empty());
	else if (tool.moving && tool.moved_valid)
		preview(tool.moved, overlay_color::selected, dragged != nullptr && false == dragged->read.empty());
	else if (m_intent.what == track_intent::kind::signal_place && m_hover.track != nullptr && template_armed != nullptr)
	{
		signal_spot spot;
		if (signal_spot_at(*m_hover.track, m_hover.path, m_hover.point, cursor_level(m_hover.point.y), 0, 0, *template_armed, spot))
		{
			signal_snap(spot, *template_armed);
			preview(spot, overlay_color::marked, false == template_armed->read.empty());
		}
	}
	// the deepest place of the selected signal in the gauge of a track
	if (auto const &gauge{tool.gauge}; tool.selected != 0 && gauge.instance == tool.selected && gauge.history == history_revision() && gauge.hit.depth > kGaugeSlack && gauge.hit.track != nullptr)
	{
		ImU32 const warning{IM_COL32(255, 70, 50, 255)};
		ImVec2 at;
		if (projection.project(gauge.hit.point, at))
		{
			drawlist->AddCircle(at, 9.0f, outline, 0, 5.0f);
			drawlist->AddCircle(at, 9.0f, warning, 0, 3.0f);
		}
		plate(gauge.hit.point, warning, format(STR_C("In the gauge of %s by %.0f cm"), name_of(*gauge.hit.track).c_str(), gauge.hit.depth * 100.0));
	}
}

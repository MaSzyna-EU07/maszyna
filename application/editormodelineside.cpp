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
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"

#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <cctype>
#include <cmath>

namespace
{

using geometry::bezier;
using geometry::plan_distance;
using geometry::plan_of;

double const kSampleStep{2.0}; // m between the samples of the line
double const kLineCorridor{15.0}; // m from the axis, posts farther away belong to another line
double const kOverlayRange{3000.0};
double const kFoulingTrace{250.0}; // m from the start of a switch the tracks are followed for
double const kRailHeight{0.18}; // m from the points of a track to the heads of its rails
int const kRailCategory{1};
char const *const kHektoStyles[] = {"nowy", "stary", "zgnity"};
char const *const kFoulingModels[] = {"ip/pkp/w17.inc", "ip/pkp/w17_new.inc", "ip/pkp/w17_old.inc", "ip/pkp/w17_dwuteo.inc"};

std::string normalized(std::string Text)
{
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char const Character) { return static_cast<char>(Character == '\\' ? '/' : std::tolower(Character)); });
	return Text;
}

bool numeric(std::string const &Text, double &Value)
{
	char *end{nullptr};
	Value = std::strtod(Text.c_str(), &end);
	return false == Text.empty() && *end == '\0';
}

std::string number(double const Value, int const Decimals)
{
	auto text{format("%.*f", Decimals, Value)};
	if (text == "-0" || text.find_first_not_of("-0.") == std::string::npos)
		text.erase(0, text.front() == '-' ? 1 : 0);
	return text;
}

double heading_of(glm::dvec2 const &Direction)
{
	auto const degrees{glm::degrees(std::atan2(Direction.x, Direction.y))};
	return degrees < 0.0 ? degrees + 360.0 : degrees;
}

// index of the sample at or before the chainage
std::size_t sample_index(std::vector<editor_track::route_sample> const &Samples, double const Chainage)
{
	auto const found{std::upper_bound(Samples.begin(), Samples.end(), Chainage, [](double const Value, editor_track::route_sample const &Sample) { return Value < Sample.chainage; })};
	return found == Samples.begin() ? 0 : std::min<std::size_t>(std::distance(Samples.begin(), found) - 1, Samples.size() - 2);
}

glm::dvec2 sampled_direction(std::vector<editor_track::route_sample> const &Samples, double const Chainage)
{
	auto const index{sample_index(Samples, Chainage)};
	auto const &a{Samples[index]};
	auto const &b{Samples[index + 1]};
	auto const span{b.chainage - a.chainage};
	auto const f{span > 1e-9 ? std::clamp((Chainage - a.chainage) / span, 0.0, 1.0) : 0.0};
	auto const direction{a.direction * (1.0 - f) + b.direction * f};
	return glm::length(direction) > 1e-9 ? glm::normalize(direction) : glm::dvec2{0.0, 1.0};
}

// chainage of the point of the line nearest to the point, and how far from the line it is
double project(std::vector<editor_track::route_sample> const &Samples, glm::dvec3 const &Point, double &Distance)
{
	Distance = std::numeric_limits<double>::max();
	double chainage{0.0};
	auto const point{plan_of(Point)};
	for (std::size_t i = 1; i < Samples.size(); ++i)
	{
		auto const a{plan_of(Samples[i - 1].position)};
		auto const b{plan_of(Samples[i].position)};
		auto const ab{b - a};
		auto const length2{glm::dot(ab, ab)};
		auto const t{length2 > 1e-12 ? std::clamp(glm::dot(point - a, ab) / length2, 0.0, 1.0) : 0.0};
		auto const distance{glm::length(point - (a + ab * t))};
		if (distance < Distance)
		{
			Distance = distance;
			chainage = Samples[i - 1].chainage + (Samples[i].chainage - Samples[i - 1].chainage) * t;
		}
	}
	return chainage;
}

// the digits after the rotation make the number of the post, the last one the hectometre
bool hectometres_of(std::vector<std::string> const &Values, int &Hectometres)
{
	if (Values.size() < 6)
		return false;
	Hectometres = 0;
	for (std::size_t i = 5; i < Values.size(); ++i)
	{
		double digit;
		if (false == numeric(Values[i], digit) || digit < 0.0 || digit > 9.0)
			return false;
		Hectometres = Hectometres * 10 + static_cast<int>(digit);
	}
	return true;
}

std::string hekto_file(int const Hectometres, int const Style)
{
	auto style{Style};
	if (style > 2)
	{
		auto const pick{LocalRandom(0.0, 1.0)};
		style = pick < 0.6 ? 0 : pick < 0.9 ? 1 : 2;
	}
	auto const size{Hectometres < 100 ? "1" : Hectometres < 1000 ? "10" : "100"};
	return std::string{"przytorowe/slupek_hekto_"} + kHektoStyles[style] + "_" + size + ".inc";
}

std::vector<std::string> hekto_digits(int const Hectometres)
{
	return {std::to_string((Hectometres / 1000) % 10), std::to_string((Hectometres / 100) % 10), std::to_string((Hectometres / 10) % 10), std::to_string(Hectometres % 10)};
}

std::string km_text(int const Hectometres)
{
	return format("%d.%d", Hectometres / 10, Hectometres % 10);
}

// points along the track from the end of a path on, the way a train goes: through a switch along its main track
std::vector<glm::dvec3> trace(TTrack &Track, int const Path, bool const Forward, double const Limit)
{
	std::vector<glm::dvec3> points;
	auto *track{&Track};
	auto path{Path};
	auto forward{Forward};
	double length{0.0};
	for (int guard = 0; guard < 64 && track != nullptr && length < Limit; ++guard)
	{
		bezier const curve{track->m_paths[path]};
		auto const count{std::clamp(static_cast<int>(curve.plan_length() / 1.0), 4, 400)};
		for (int k = points.empty() ? 0 : 1; k <= count; ++k)
		{
			auto const point{curve.point(forward ? static_cast<double>(k) / count : 1.0 - static_cast<double>(k) / count)};
			if (false == points.empty())
				length += plan_distance(points.back(), point);
			points.push_back(point);
		}
		auto const exit{points.back()};
		TTrack *next{nullptr};
		int nextpath{0};
		bool nextforward{true};
		for (auto const &connection : editor_track::connected_points(*track, exit))
		{
			auto *other{connection.first};
			if (other == track || (other->iCategoryFlag & 15) != kRailCategory)
				continue;
			bool const start{connection.second.kind == editor_track::point_kind::start};
			// entering a switch at its start the train takes the main track
			if (next == nullptr || (other == next && start && connection.second.path == 0))
			{
				next = other;
				nextpath = connection.second.path;
				nextforward = start;
			}
		}
		track = next;
		path = nextpath;
		forward = nextforward;
	}
	return points;
}

// nearest point of the polyline
glm::dvec3 nearest_on(std::vector<glm::dvec3> const &Line, glm::dvec3 const &Point, double &Distance)
{
	Distance = std::numeric_limits<double>::max();
	glm::dvec3 result{Point};
	auto const point{plan_of(Point)};
	for (std::size_t i = 1; i < Line.size(); ++i)
	{
		auto const a{plan_of(Line[i - 1])};
		auto const ab{plan_of(Line[i]) - a};
		auto const length2{glm::dot(ab, ab)};
		auto const t{length2 > 1e-12 ? std::clamp(glm::dot(point - a, ab) / length2, 0.0, 1.0) : 0.0};
		auto const distance{glm::length(point - (a + ab * t))};
		if (distance < Distance)
		{
			Distance = distance;
			result = glm::mix(Line[i - 1], Line[i], t);
		}
	}
	return result;
}

void draw_label(ImDrawList *Draw, ImVec2 const At, ImU32 const Colour, std::string const &Text)
{
	auto const size{ImGui::CalcTextSize(Text.c_str())};
	Draw->AddRectFilled(ImVec2(At.x + 8.0f, At.y - 8.0f), ImVec2(At.x + 14.0f + size.x, At.y - 6.0f + size.y), IM_COL32(0, 0, 0, 150), 3.0f);
	Draw->AddText(ImVec2(At.x + 11.0f, At.y - 7.0f), Colour, Text.c_str());
}

} // namespace

std::vector<editor_mode::standing_template> editor_mode::standing_templates(std::function<bool(std::string const &)> const &Accept) const
{
	std::vector<standing_template> result;
	for (scene::instance_handle handle = 1; handle <= scene::Layers.instance_count(); ++handle)
	{
		if (false == scene::Layers.tracked(handle))
			continue;
		auto const &included{scene::Layers.instance(handle)};
		if (included.removed || included.dead || included.file == nullptr)
			continue;
		auto const file{normalized(*included.file)};
		if (false == Accept(file) || included.context.rotation != glm::vec3{0.f} || included.context.scale != glm::vec3{1.f})
			continue;
		std::string target;
		std::vector<std::string> values;
		if (false == editor_includes::parse_directive(scene::Layers.directive(handle), target, values) || values.size() < 5)
			continue;
		glm::dvec3 location;
		double yaw;
		if (false == numeric(values[1], location.x) || false == numeric(values[2], location.y) || false == numeric(values[3], location.z) || false == numeric(values[4], yaw))
			continue;
		standing_template standing;
		standing.instance = handle;
		standing.file = file;
		standing.location = included.context.offset + location;
		standing.yaw = yaw;
		standing.values = std::move(values);
		result.push_back(std::move(standing));
	}
	return result;
}

std::size_t editor_mode::place_templates(std::vector<template_item> const &Items, std::string &Error)
{
	if (Items.empty())
		return 0;
	if (scene::Layers.empty())
	{
		Error = STR_C("Templates can be placed only in a scenery opened for editing (-edit)");
		return 0;
	}
	auto const layer{scene::Layers.resolve(scene::Layers.active())};
	if (std::string reason; false == scene::Layers.accepts(layer, &reason))
	{
		Error = STR_C("The active layer can't take them: ") + (reason.empty() ? std::string{STR_C("there's no active layer")} : reason);
		return 0;
	}
	auto const context{scene::Layers.layer(layer).context_insert()};
	if (context.rotation != glm::vec3{0.f} || context.scale != glm::vec3{1.f})
	{
		Error = STR_C("The file of the active layer would receive them with a rotation or a scale in effect, pick another layer");
		return 0;
	}
	EditorSnapshot snap;
	for (auto const &item : Items)
	{
		auto const local{context.to_local(item.location)};
		std::string directive;
		if (item.described)
		{
			include_info info;
			if (false == editor_includes::load(item.file, info, Error))
				continue;
			directive = editor_includes::directive(item.file, info, editor_includes::parameter_count(item.file), local, static_cast<float>(item.yaw), item.track, item.tilt);
		}
		else
		{
			std::vector<std::string> values{"none", number(local.x, 3), number(local.y, 3), number(local.z, 3), number(item.yaw, 2)};
			values.insert(values.end(), item.rest.begin(), item.rest.end());
			directive = editor_includes::compose_directive(item.file, values);
		}
		auto const instance{scene::Layers.place(layer, item.file, directive)};
		if (instance == 0)
			continue;
		simulation::State.preview_include(directive, context, layer, instance);
		snap.instances.push_back(instance);
	}
	if (snap.instances.empty())
	{
		Error = STR_C("The active layer can't take them");
		return 0;
	}
	auto const count{snap.instances.size()};
	snap.node_name = Items.front().file;
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	return count;
}

std::size_t editor_mode::move_templates(std::vector<std::pair<standing_template, template_item>> const &Moves)
{
	EditorSnapshot snap;
	for (auto const &[standing, item] : Moves)
	{
		if (false == scene::Layers.tracked(standing.instance) || false == scene::Layers.removable(standing.instance))
			continue;
		auto const &included{scene::Layers.instance(standing.instance)};
		auto values{standing.values};
		auto const local{item.location - included.context.offset};
		values[1] = number(local.x, 3);
		values[2] = number(local.y, 3);
		values[3] = number(local.z, 3);
		values[4] = number(item.yaw, 2);
		auto before{scene::Layers.directive(standing.instance)};
		set_include_directive(standing.instance, editor_includes::compose_directive(*included.file, values));
		snap.directives.emplace_back(standing.instance, std::move(before));
	}
	if (snap.directives.empty())
		return 0;
	auto const count{snap.directives.size()};
	snap.node_name = Moves.front().first.file;
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	return count;
}

std::size_t editor_mode::remove_templates(std::vector<scene::instance_handle> const &Instances)
{
	EditorSnapshot snap;
	for (auto const instance : Instances)
	{
		if (false == scene::Layers.tracked(instance) || false == scene::Layers.removable(instance))
			continue;
		scene::Layers.removed(instance, true);
		snap.instances.push_back(instance);
	}
	if (snap.instances.empty())
		return 0;
	auto const count{snap.instances.size()};
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	select_include(0);
	return count;
}

void editor_mode::hekto_start(TTrack &Track, glm::dvec3 const &Point)
{
	auto &tool{m_hekto};
	tool.track = &Track;
	tool.point = Point;
	tool.km_read = false;
	tool.dirty = true;
	tool.expand = true;
	tool.status.clear();
}

void editor_mode::hekto_read_kilometrage()
{
	auto &tool{m_hekto};
	auto const &found{tool.found};
	if (found.empty())
		return;
	if (found.size() == 1)
	{
		// one post doesn't tell the way the kilometrage grows: it's taken to grow along the longer part of the line past the post
		auto const length{tool.samples.empty() ? 0.0 : tool.samples.back().chainage};
		tool.growth = found.front().chainage > 0.5 * length ? -1 : 1;
		tool.km = found.front().hectometres / 10.0 + tool.growth * (tool.origin - found.front().chainage) / 1000.0;
		tool.status = format(STR_C("Kilometrage read from the post %s, check which way it grows"), km_text(found.front().hectometres).c_str());
		return;
	}
	// a straight line through the numbers of the posts against where they stand
	double sc{0.0}, sk{0.0}, scc{0.0}, sck{0.0};
	for (auto const &post : found)
	{
		auto const km{post.hectometres / 10.0};
		sc += post.chainage;
		sk += km;
		scc += post.chainage * post.chainage;
		sck += post.chainage * km;
	}
	auto const n{static_cast<double>(found.size())};
	auto const denominator{n * scc - sc * sc};
	if (std::abs(denominator) < 1e-9)
		return;
	auto const slope{(n * sck - sc * sk) / denominator};
	auto const intercept{(sk - slope * sc) / n};
	if (std::abs(std::abs(slope) * 1000.0 - 1.0) > 0.2)
	{
		tool.status = format(STR_C("The numbers of the %zu posts along the line don't go by 100 m, the kilometrage wasn't read from them"), found.size());
		return;
	}
	tool.growth = slope > 0.0 ? 1 : -1;
	tool.km = std::round((intercept + slope * tool.origin) * 1000.0) / 1000.0;
	tool.status = format(STR_C("Kilometrage read from %zu posts standing along the line"), found.size());
}

void editor_mode::hekto_update()
{
	auto &tool{m_hekto};
	if (false == tool.dirty && tool.history == m_history.size())
		return;
	tool.dirty = false;
	tool.history = m_history.size();
	tool.posts.clear();
	tool.found.clear();
	tool.error.clear();
	tool.samples.clear();
	if (tool.track == nullptr || tool.track->m_editorremoved)
	{
		tool.track = nullptr;
		return;
	}
	tool.route = editor_track::run_route(*tool.track, tool.reach);
	if (tool.route.spans.empty())
	{
		tool.error = STR_C("No line through this path");
		return;
	}
	tool.samples = editor_track::sample_route(tool.route, kSampleStep);
	if (tool.samples.size() < 2)
	{
		tool.error = STR_C("No line through this path");
		return;
	}
	double distance;
	tool.origin = project(tool.samples, tool.point, distance);
	auto const length{tool.samples.back().chainage};
	for (auto const &standing : standing_templates([](std::string const &File) { return File.find("hekto") != std::string::npos; }))
	{
		hekto_found post;
		post.post = standing;
		if (false == hectometres_of(standing.values, post.hectometres))
			continue;
		post.chainage = project(tool.samples, standing.location, distance);
		if (distance > kLineCorridor || post.chainage <= 0.0 || post.chainage >= length)
			continue;
		tool.found.push_back(post);
	}
	if (false == tool.km_read)
	{
		tool.km_read = true;
		hekto_read_kilometrage();
	}
	auto const chainage_of = [&](int const Hectometres) { return tool.origin + tool.growth * (Hectometres / 10.0 - tool.km) * 1000.0; };
	for (auto &post : tool.found)
	{
		post.expected = chainage_of(post.hectometres);
		post.error = post.chainage - post.expected;
		post.outside = true;
	}
	auto const km_at = [&](double const Chainage) { return tool.km + tool.growth * (Chainage - tool.origin) / 1000.0; };
	auto const low{std::max(0, static_cast<int>(std::ceil(std::min(km_at(0.0), km_at(length)) * 10.0 - 1e-6)))};
	auto const high{static_cast<int>(std::floor(std::max(km_at(0.0), km_at(length)) * 10.0 + 1e-6))};
	for (int hectometres = low; hectometres <= high && tool.posts.size() < 5000; ++hectometres)
	{
		hekto_post post;
		post.hectometres = hectometres;
		post.chainage = chainage_of(hectometres);
		if (post.chainage < 0.0 || post.chainage > length)
			continue;
		auto const axis{editor_track::sampled_position(tool.samples, post.chainage)};
		auto const growing{sampled_direction(tool.samples, post.chainage) * static_cast<double>(tool.growth)};
		glm::dvec2 const left{-growing.y, growing.x};
		// alternately: the even hectometres on the right, the odd ones on the left of the growing kilometrage
		auto const side{tool.side != 0 ? tool.side : (hectometres % 2 == 0 ? 1 : -1)};
		auto const planar{plan_of(axis) + left * (side * tool.offset)};
		post.position = placement_on_ground({planar.x, axis.y, planar.y});
		post.yaw = std::fmod(heading_of(growing) + tool.turn, 360.0);
		// of the posts with this number the one nearest to the place stands for it, the others are duplicates. it stands wrong
		// when it's farther from the place than allowed: along the line, across it, or on the other side
		auto best{std::numeric_limits<double>::max()};
		for (int i = 0; i < static_cast<int>(tool.found.size()); ++i)
		{
			auto &found{tool.found[i]};
			if (found.hectometres != hectometres)
				continue;
			found.outside = false;
			found.distance = plan_distance(found.post.location, post.position);
			if (found.distance < best)
			{
				best = found.distance;
				post.standing = i;
			}
		}
		for (int i = 0; i < static_cast<int>(tool.found.size()); ++i)
		{
			auto &found{tool.found[i]};
			if (found.hectometres != hectometres)
				continue;
			found.duplicate = i != post.standing;
			found.wrong = false == found.duplicate && found.distance > tool.tolerance;
		}
		tool.posts.push_back(post);
	}
}

void editor_mode::hekto_remove_duplicates()
{
	auto &tool{m_hekto};
	hekto_update();
	std::vector<scene::instance_handle> instances;
	for (auto const &found : tool.found)
		if (found.duplicate)
			instances.push_back(found.post.instance);
	auto const removed{remove_templates(instances)};
	tool.status = format(STR_C("%zu duplicate hectometre posts removed, Ctrl+Z brings them back"), removed);
	WriteLog("Editor: " + tool.status);
	tool.dirty = true;
}

void editor_mode::hekto_place()
{
	auto &tool{m_hekto};
	hekto_update();
	std::vector<template_item> items;
	for (auto const &post : tool.posts)
	{
		if (post.standing >= 0)
			continue;
		template_item item;
		item.file = hekto_file(post.hectometres, tool.style);
		item.location = post.position;
		item.yaw = post.yaw;
		item.rest = hekto_digits(post.hectometres);
		items.push_back(std::move(item));
	}
	std::string error;
	auto const placed{place_templates(items, error)};
	tool.status = placed > 0 ? format(STR_C("%zu hectometre posts placed, Ctrl+Z takes them back"), placed) : error;
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.dirty = true;
}

void editor_mode::hekto_fix()
{
	auto &tool{m_hekto};
	hekto_update();
	std::vector<std::pair<standing_template, template_item>> moves;
	for (auto const &post : tool.posts)
	{
		if (post.standing < 0 || false == tool.found[post.standing].wrong)
			continue;
		template_item item;
		item.location = post.position;
		item.yaw = post.yaw;
		moves.emplace_back(tool.found[post.standing].post, item);
	}
	auto const moved{move_templates(moves)};
	tool.status = format(STR_C("%zu hectometre posts moved to their places, Ctrl+Z takes it back"), moved);
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.dirty = true;
}

void editor_mode::render_hekto_ui()
{
	auto &tool{m_hekto};
	auto *selected{selected_track()};
	if (tool.track == nullptr || (selected != nullptr && selected != tool.track && ImGui::SmallButton(STR_C("Along the line of the selected path"))))
	{
		if (selected == nullptr)
		{
			ImGui::TextDisabled("%s", STR_C("Select a path of the line, or RMB on it: Hectometre posts"));
			return;
		}
		hekto_start(*selected, bezier{selected->m_paths.front()}.point(0.0));
	}
	auto const changed = [&](bool const Changed) {
		if (Changed)
			tool.dirty = true;
	};
	ImGui::PushItemWidth(110.0f);
	changed(ImGui::InputDouble(STR_C("Kilometre at the point (km)"), &tool.km, 0.0, 0.0, "%.3f"));
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Read from the posts")))
	{
		tool.km_read = false;
		tool.dirty = true;
	}
	ImGui::TextDisabled("%s", STR_C("The kilometrage grows"));
	ImGui::SameLine();
	changed(ImGui::RadioButton(STR_C("along the line"), &tool.growth, 1));
	ImGui::SameLine();
	changed(ImGui::RadioButton(STR_C("against it"), &tool.growth, -1));
	ImGui::TextDisabled("%s", STR_C("Side, looking where it grows"));
	ImGui::SameLine();
	changed(ImGui::RadioButton(STR_C("right"), &tool.side, 1));
	ImGui::SameLine();
	changed(ImGui::RadioButton(STR_C("left"), &tool.side, -1));
	ImGui::SameLine();
	changed(ImGui::RadioButton(STR_C("alternately"), &tool.side, 0));
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("The even hectometres on the right, the odd ones on the left"));
	changed(ImGui::InputDouble(STR_C("From the axis (m)"), &tool.offset, 0.0, 0.0, "%.2f"));
	tool.offset = std::clamp(tool.offset, 1.5, 30.0);
	char const *styles[] = {"nowy", "stary", "zgnity", "mixed"};
	ImGui::Combo(STR_C("Posts"), &tool.style, styles, IM_ARRAYSIZE(styles));
	char const *turns[] = {"0", "90", "180", "270"};
	int turn{tool.turn / 90};
	if (ImGui::Combo(STR_C("Turned by (deg)"), &turn, turns, IM_ARRAYSIZE(turns)))
	{
		tool.turn = turn * 90;
		tool.dirty = true;
	}
	changed(ImGui::InputDouble(STR_C("Along the line each way (m)"), &tool.reach, 0.0, 0.0, "%.0f"));
	tool.reach = std::clamp(tool.reach, 100.0, 100000.0);
	changed(ImGui::InputDouble(STR_C("A post standing off by more (m) is wrong"), &tool.tolerance, 0.0, 0.0, "%.1f"));
	tool.tolerance = std::clamp(tool.tolerance, 0.5, 50.0);
	ImGui::PopItemWidth();
	hekto_update();
	if (false == tool.error.empty())
	{
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", tool.error.c_str());
		return;
	}
	auto const missing{std::count_if(tool.posts.begin(), tool.posts.end(), [](hekto_post const &Post) { return Post.standing < 0; })};
	auto const wrong{std::count_if(tool.found.begin(), tool.found.end(), [](hekto_found const &Post) { return Post.wrong; })};
	auto const duplicates{std::count_if(tool.found.begin(), tool.found.end(), [](hekto_found const &Post) { return Post.duplicate; })};
	ImGui::Text(STR_C("Line %.0f m, km %s - %s: %d posts to place, %zu standing, %d of them wrong"), tool.samples.empty() ? 0.0 : tool.samples.back().chainage, tool.posts.empty() ? "-" : km_text(std::min(tool.posts.front().hectometres, tool.posts.back().hectometres)).c_str(),
	            tool.posts.empty() ? "-" : km_text(std::max(tool.posts.front().hectometres, tool.posts.back().hectometres)).c_str(), static_cast<int>(missing), tool.found.size(), static_cast<int>(wrong));
	if (missing > 0 && ImGui::Button(format(STR_C("Place %d posts"), static_cast<int>(missing)).c_str()))
		hekto_place();
	if (duplicates > 0)
	{
		if (missing > 0)
			ImGui::SameLine();
		if (ImGui::Button(format(STR_C("Remove %d duplicate posts"), static_cast<int>(duplicates)).c_str()))
			hekto_remove_duplicates();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Posts with a number another post nearer to its place already has"));
	}
	if (wrong > 0 || duplicates > 0)
	{
		if (wrong > 0)
		{
			if (missing > 0 || duplicates > 0)
				ImGui::SameLine();
			if (ImGui::Button(format(STR_C("Move %d wrong posts to their places"), static_cast<int>(wrong)).c_str()))
				hekto_fix();
		}
		ImGui::BeginChild("##hektowrong", ImVec2(0.0f, std::min(6, static_cast<int>(wrong + duplicates)) * ImGui::GetTextLineHeightWithSpacing() + 6.0f), true);
		for (auto const &post : tool.found)
		{
			if (false == post.wrong && false == post.duplicate)
				continue;
			auto const label{post.duplicate ? format(STR_C("km %s duplicate##%u"), km_text(post.hectometres).c_str(), post.post.instance) : format(STR_C("km %s stands %.1f m from its place##%u"), km_text(post.hectometres).c_str(), post.distance, post.post.instance)};
			if (ImGui::Selectable(label.c_str()))
			{
				glm::dvec3 const look{glm::normalize(glm::dvec3{-1.0, -0.8, -1.0})};
				m_focus_start_pos = Camera.Pos;
				m_focus_start_angle = Camera.Angle;
				m_focus_target_pos = post.post.location - look * 20.0;
				m_focus_target_angle = glm::vec3(static_cast<float>(std::asin(look.y)), static_cast<float>(std::atan2(-look.x, -look.z)), 0.0f);
				m_focus_active = true;
				m_focus_time = 0.0;
				m_focus_duration = 0.6;
			}
		}
		ImGui::EndChild();
	}
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

void editor_mode::draw_hekto_overlay() const
{
	auto const &tool{m_hekto};
	if (false == tool.open || tool.track == nullptr)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (auto const &post : tool.posts)
	{
		if (glm::distance(post.position, camera) > kOverlayRange)
			continue;
		ImVec2 screen;
		if (false == projection.project(post.position, screen))
			continue;
		bool const standing{post.standing >= 0 && false == tool.found[post.standing].wrong};
		auto const colour{standing ? IM_COL32(180, 180, 180, 200) : IM_COL32(90, 255, 120, 235)};
		drawlist->AddCircleFilled(screen, 4.0f, colour);
		draw_label(drawlist, screen, colour, km_text(post.hectometres));
	}
	for (auto const &post : tool.found)
	{
		if ((false == post.wrong && false == post.duplicate) || glm::distance(post.post.location, camera) > kOverlayRange)
			continue;
		ImVec2 screen;
		if (false == projection.project(post.post.location, screen))
			continue;
		auto const colour{post.duplicate ? IM_COL32(255, 160, 40, 235) : IM_COL32(255, 70, 60, 235)};
		drawlist->AddCircle(screen, 9.0f, colour, 16, 2.5f);
		draw_label(drawlist, screen, colour, post.duplicate ? format(STR_C("km %s duplicate"), km_text(post.hectometres).c_str()) : format(STR_C("km %s, %.1f m from its place"), km_text(post.hectometres).c_str(), post.distance));
		if (post.duplicate)
			continue;
		for (auto const &target : tool.posts)
			if (target.hectometres == post.hectometres)
				projection.line(drawlist, post.post.location, target.position, IM_COL32(255, 70, 60, 160), 1.5f);
	}
	ImVec2 screen;
	if (projection.project(tool.point, screen))
	{
		drawlist->AddCircle(screen, 7.0f, IM_COL32(90, 200, 255, 235), 16, 2.5f);
		draw_label(drawlist, screen, IM_COL32(90, 200, 255, 235), format("km %.3f", tool.km));
	}
}

void editor_mode::fouling_update()
{
	auto &tool{m_fouling};
	auto *selected{selected_track()};
	if (tool.scope == 0 && selected != tool.selected)
		tool.dirty = true;
	if (false == tool.dirty && tool.history == m_history.size())
		return;
	tool.dirty = false;
	tool.history = m_history.size();
	tool.selected = selected;
	tool.points.clear();
	tool.found.clear();
	tool.error.clear();
	std::vector<TTrack *> switches;
	glm::dvec3 const camera{Global.pCamera.Pos};
	if (tool.scope == 0)
	{
		if (selected == nullptr || selected->eType != tt_Switch)
		{
			tool.error = STR_C("Select a switch, or check around the camera");
			return;
		}
		switches.push_back(selected);
	}
	else
	{
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && track->eType == tt_Switch && false == track->m_editorremoved && (track->iCategoryFlag & 15) == kRailCategory && track->m_paths.size() >= 2 &&
			    (tool.scope == 2 || glm::distance(track->location(), camera) <= tool.reach))
				switches.push_back(track);
	}
	for (auto *track : switches)
	{
		if (track->m_paths.size() < 2 || false == editor_track::is_supported(*track))
			continue;
		auto const main{trace(*track, 0, true, kFoulingTrace)};
		auto const branch{trace(*track, 1, true, kFoulingTrace)};
		if (main.size() < 2 || branch.size() < 2)
			continue;
		glm::dvec3 previous{branch.front()};
		double previousdistance{0.0};
		for (std::size_t i = 1; i < branch.size(); ++i)
		{
			double distance;
			nearest_on(main, branch[i], distance);
			if (distance < tool.spacing)
			{
				previous = branch[i];
				previousdistance = distance;
				continue;
			}
			auto const f{distance - previousdistance > 1e-9 ? (tool.spacing - previousdistance) / (distance - previousdistance) : 1.0};
			auto const onbranch{glm::mix(previous, branch[i], std::clamp(f, 0.0, 1.0))};
			double unused;
			auto const onmain{nearest_on(main, onbranch, unused)};
			fouling_point point;
			point.track = track;
			auto const middle{(onbranch + onmain) * 0.5};
			// the points of the paths are at the rail foot, the heads of the rails are the height of the rail over them
			point.position = tool.height_mode == 1 ? glm::dvec3{middle.x, 0.5 * (onbranch.y + onmain.y) + kRailHeight + tool.height, middle.z} : placement_on_ground({middle.x, std::max(onbranch.y, onmain.y), middle.z});
			auto const back{plan_of(previous - branch[i])};
			point.yaw = std::fmod(heading_of(glm::length(back) > 1e-9 ? glm::normalize(back) : glm::dvec2{0.0, 1.0}) + tool.turn, 360.0);
			tool.points.push_back(point);
			break;
		}
	}
	// markers standing: each goes to the nearest fouling point, the others are left alone
	auto const markers{standing_templates([](std::string const &File) { return File.find("w17") != std::string::npos || File.find("ukres") != std::string::npos; })};
	for (auto const &marker : markers)
	{
		int nearest{-1};
		double best{std::numeric_limits<double>::max()};
		for (int i = 0; i < static_cast<int>(tool.points.size()); ++i)
		{
			auto const distance{plan_distance(marker.location, tool.points[i].position)};
			if (distance < best)
			{
				best = distance;
				nearest = i;
			}
		}
		if (nearest < 0 || best > 40.0)
			continue;
		auto &point{tool.points[nearest]};
		if (point.standing >= 0 && tool.found[point.standing].error <= best)
			continue;
		fouling_found found;
		found.marker = marker;
		found.point = nearest;
		found.error = best;
		found.height_error = marker.location.y - point.position.y;
		found.wrong = best > tool.tolerance || (tool.height_mode == 1 && std::abs(found.height_error) > tool.height_tolerance);
		if (point.standing >= 0)
			tool.found[point.standing] = found;
		else
		{
			point.standing = static_cast<int>(tool.found.size());
			tool.found.push_back(found);
		}
	}
	auto const missing{std::count_if(tool.points.begin(), tool.points.end(), [](fouling_point const &Point) { return Point.standing < 0; })};
	auto const wrong{std::count_if(tool.found.begin(), tool.found.end(), [](fouling_found const &Found) { return Found.wrong; })};
	tool.status = format(STR_C("%zu switches, %zu fouling points: %d without a marker, %d markers standing wrong"), switches.size(), tool.points.size(), static_cast<int>(missing), static_cast<int>(wrong));
}

void editor_mode::fouling_place()
{
	auto &tool{m_fouling};
	fouling_update();
	std::vector<template_item> items;
	for (auto const &point : tool.points)
	{
		if (point.standing >= 0)
			continue;
		template_item item;
		item.file = kFoulingModels[std::clamp(tool.model, 0, 3)];
		item.location = point.position;
		item.yaw = point.yaw;
		if (tool.model != 2)
			item.rest = {"0"};
		items.push_back(std::move(item));
	}
	std::string error;
	auto const placed{place_templates(items, error)};
	tool.status = placed > 0 ? format(STR_C("%zu fouling point markers placed, Ctrl+Z takes them back"), placed) : error;
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.dirty = true;
}

void editor_mode::fouling_fix()
{
	auto &tool{m_fouling};
	fouling_update();
	std::vector<std::pair<standing_template, template_item>> moves;
	for (auto const &found : tool.found)
	{
		if (false == found.wrong)
			continue;
		template_item item;
		auto const &point{tool.points[found.point]};
		if (found.error <= tool.tolerance)
		{
			// in its place, only too high or too low
			item.location = {found.marker.location.x, point.position.y, found.marker.location.z};
			item.yaw = found.marker.yaw;
		}
		else
		{
			item.location = point.position;
			item.yaw = point.yaw;
		}
		moves.emplace_back(found.marker, item);
	}
	auto const moved{move_templates(moves)};
	tool.status = format(STR_C("%zu fouling point markers moved to their places, Ctrl+Z takes it back"), moved);
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.dirty = true;
}

void editor_mode::render_fouling_ui()
{
	auto &tool{m_fouling};
	ImGui::TextDisabled("%s", STR_C("W17 where the axes of the tracks meeting at a switch are the spacing apart,\nhalf way between them"));
	auto const changed = [&](bool const Changed) {
		if (Changed)
			tool.dirty = true;
	};
	changed(ImGui::RadioButton(STR_C("The selected switch"), &tool.scope, 0));
	ImGui::SameLine();
	changed(ImGui::RadioButton(STR_C("Around the camera"), &tool.scope, 1));
	ImGui::SameLine();
	changed(ImGui::RadioButton(STR_C("Whole scenery"), &tool.scope, 2));
	ImGui::PushItemWidth(110.0f);
	if (tool.scope == 1)
		changed(ImGui::InputDouble(STR_C("Around the camera (m)"), &tool.reach, 0.0, 0.0, "%.0f"));
	changed(ImGui::InputDouble(STR_C("Spacing of the axes (m)"), &tool.spacing, 0.0, 0.0, "%.2f"));
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("3.5 m on the straight track; more on the curves, where the cars lean out"));
	changed(ImGui::InputDouble(STR_C("A marker off by more (m) is wrong"), &tool.tolerance, 0.0, 0.0, "%.1f"));
	changed(ImGui::RadioButton(STR_C("On the ground"), &tool.height_mode, 0));
	ImGui::SameLine();
	changed(ImGui::RadioButton(STR_C("Over the rail heads"), &tool.height_mode, 1));
	if (tool.height_mode == 1)
	{
		ImGui::SameLine();
		changed(ImGui::InputDouble(STR_C("by (m)"), &tool.height, 0.0, 0.0, "%.3f"));
		changed(ImGui::InputDouble(STR_C("A marker higher or lower by more (m) is wrong"), &tool.height_tolerance, 0.0, 0.0, "%.3f"));
		tool.height_tolerance = std::clamp(tool.height_tolerance, 0.005, 1.0);
	}
	char const *models[] = {"w17", "w17_new", "w17_old", "w17_dwuteo"};
	ImGui::Combo(STR_C("Marker"), &tool.model, models, IM_ARRAYSIZE(models));
	char const *turns[] = {"0", "90", "180", "270"};
	int turn{tool.turn / 90};
	if (ImGui::Combo(STR_C("Turned by (deg)"), &turn, turns, IM_ARRAYSIZE(turns)))
	{
		tool.turn = turn * 90;
		tool.dirty = true;
	}
	ImGui::PopItemWidth();
	tool.reach = std::clamp(tool.reach, 50.0, 50000.0);
	tool.spacing = std::clamp(tool.spacing, 2.0, 10.0);
	tool.tolerance = std::clamp(tool.tolerance, 0.2, 20.0);
	fouling_update();
	if (false == tool.error.empty())
	{
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", tool.error.c_str());
		return;
	}
	ImGui::TextWrapped("%s", tool.status.c_str());
	auto const missing{std::count_if(tool.points.begin(), tool.points.end(), [](fouling_point const &Point) { return Point.standing < 0; })};
	auto const wrong{std::count_if(tool.found.begin(), tool.found.end(), [](fouling_found const &Found) { return Found.wrong; })};
	if (missing > 0 && ImGui::Button(format(STR_C("Place %d markers"), static_cast<int>(missing)).c_str()))
		fouling_place();
	if (wrong > 0)
	{
		if (missing > 0)
			ImGui::SameLine();
		if (ImGui::Button(format(STR_C("Move %d wrong markers to their places"), static_cast<int>(wrong)).c_str()))
			fouling_fix();
	}
}

void editor_mode::draw_fouling_overlay() const
{
	auto const &tool{m_fouling};
	if (false == tool.open)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (auto const &point : tool.points)
	{
		if (glm::distance(point.position, camera) > kOverlayRange)
			continue;
		ImVec2 screen;
		if (false == projection.project(point.position, screen))
			continue;
		bool const standing{point.standing >= 0 && false == tool.found[point.standing].wrong};
		auto const colour{standing ? IM_COL32(180, 180, 180, 200) : IM_COL32(90, 255, 120, 235)};
		drawlist->AddCircleFilled(screen, 4.0f, colour);
		draw_label(drawlist, screen, colour, "W17");
	}
	for (auto const &found : tool.found)
	{
		if (false == found.wrong || glm::distance(found.marker.location, camera) > kOverlayRange)
			continue;
		ImVec2 screen;
		if (false == projection.project(found.marker.location, screen))
			continue;
		drawlist->AddCircle(screen, 9.0f, IM_COL32(255, 70, 60, 235), 16, 2.5f);
		draw_label(drawlist, screen, IM_COL32(255, 70, 60, 235), format(STR_C("W17 %.1f m off, %+.2f m high"), found.error, found.height_error));
		projection.line(drawlist, found.marker.location, tool.points[found.point].position, IM_COL32(255, 70, 60, 160), 1.5f);
	}
}

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
#include "application/editormodelineside.h"
#include "application/editormodeshared.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"
#include "editor/editorKilometrage.hpp"

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
using geometry::heading_of;
using geometry::plan_of;

double const kSampleStep{2.0}; // m between the samples of the line
double const kLineCorridor{15.0}; // m from the axis, posts farther away belong to another line
double const kOverlayRange{3000.0};
float const kFoulingPick{12.0f}; // px from a point of the fouling overlay a click takes it
double const kFoulingTrace{250.0}; // m from the start of a switch the tracks are followed for
double const kRailHeight{0.18}; // m from the points of a track to the heads of its rails
int const kRailCategory{1};
char const *const kHektoStyles[] = {"nowy", "stary", "zgnity"};
char const *const kFoulingModels[] = {"ip/pkp/w17.inc", "ip/pkp/w17_new.inc", "ip/pkp/w17_old.inc", "ip/pkp/w17_dwuteo.inc"};

std::string number(double const Value, int const Decimals)
{
	auto text{format("%.*f", Decimals, Value)};
	if (text == "-0" || text.find_first_not_of("-0.") == std::string::npos)
		text.erase(0, text.front() == '-' ? 1 : 0);
	return text;
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
		auto const file{editor_includes::normalized(*included.file)};
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

scene::layer_handle editor_mode::output_layer(std::string const &Part)
{
	if (scene::Layers.empty())
		return null_handle;
	auto stem{scene::Layers.layer(scene::layer_handle{1}).name};
	if (auto const slash{stem.find_last_of("/\\")}; slash != std::string::npos)
		stem.erase(0, slash + 1);
	if (auto const dot{stem.rfind('.')}; dot != std::string::npos)
		stem.erase(dot);
	stem.erase(0, stem.find_first_not_of('$'));
	if (stem.empty())
		stem = "scenery";
	std::string reason;
	// a file of that name the scenery doesn't include, or one which can't take more, gives way to the next number
	for (int attempt = 1; attempt <= 9; ++attempt)
	{
		auto const name{ToLower(stem + "_" + Part + (attempt > 1 ? std::to_string(attempt) : std::string{}) + ".scm")};
		auto layer{null_handle};
		for (std::size_t index = 1; index <= scene::Layers.size() && layer == null_handle; ++index)
			if (auto const &existing{scene::Layers.layer(static_cast<scene::layer_handle>(index))}; false == existing.dead && false == existing.removed && existing.name == name)
				layer = scene::Layers.resolve(static_cast<scene::layer_handle>(index));
		if (layer == null_handle)
			layer = scene::Layers.create(name, scene::layer_handle{1}, reason, false);
		if (layer != null_handle && scene::Layers.accepts(layer, &reason))
		{
			scene::Layers.visible(layer, true);
			scene::Layers.locked(layer, false);
			return layer;
		}
	}
	WriteLog("Editor: no layer for " + Part + ", the active one takes it: " + reason, logtype::generic);
	return null_handle;
}

std::size_t editor_mode::place_templates(std::vector<template_item> const &Items, std::string &Error, scene::layer_handle const Layer)
{
	if (Items.empty())
		return 0;
	if (scene::Layers.empty())
	{
		Error = STR_C("Templates can be placed only in a scenery opened for editing (-edit)");
		return 0;
	}
	auto const layer{scene::Layers.resolve(Layer != null_handle ? Layer : scene::Layers.active())};
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
			directive = editor_includes::directive(item.file, info, editor_includes::parameter_count(item.file), local, static_cast<float>(item.yaw), item.track, item.tilt, item.values);
		}
		else
		{
			auto const name{item.values.find("name")};
			std::vector<std::string> values{name != item.values.end() ? name->second : std::string{"none"}, number(local.x, 3), number(local.y, 3), number(local.z, 3), number(item.yaw, 2)};
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

void hekto_tool::start(TTrack &Track, glm::dvec3 const &Point)
{
	auto &tool{m_hekto};
	tool.track = &Track;
	tool.point = Point;
	tool.km_read = false;
	tool.dirty = true;
	m_tab.expand = true;
	tool.status.clear();
}

void hekto_tool::hekto_read_kilometrage()
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

void hekto_tool::hekto_update()
{
	auto &tool{m_hekto};
	if (false == tool.dirty && tool.history == m_history.history_revision())
		return;
	tool.dirty = false;
	tool.history = m_history.history_revision();
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
	for (auto const &standing : m_templates.standing_templates([](std::string const &File) { return File.find("hekto") != std::string::npos; }))
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
	tool.line_set = kilometrage::line_at(tool.point);
	tool.line = tool.own_km ? std::string{} : tool.line_set;
	if (tool.line.empty() && false == tool.km_read)
	{
		tool.km_read = true;
		hekto_read_kilometrage();
	}
	auto const place = [&](hekto_post &Post) {
		auto const axis{editor_track::sampled_position(tool.samples, Post.chainage)};
		auto const growing{sampled_direction(tool.samples, Post.chainage) * static_cast<double>(Post.growth)};
		glm::dvec2 const left{-growing.y, growing.x};
		// alternately: the even hectometres on the right, the odd ones on the left of the growing kilometrage
		auto const side{tool.side != 0 ? tool.side : (Post.hectometres % 2 == 0 ? 1 : -1)};
		auto const planar{plan_of(axis) + left * (side * tool.offset)};
		Post.position = m_ground.placement_on_ground({planar.x, axis.y, planar.y});
		Post.yaw = std::fmod(heading_of(growing) + tool.turn, 360.0);
		tool.posts.push_back(Post);
	};
	auto const *axis{tool.line.empty() ? nullptr : kilometrage::axis_of(tool.line)};
	if (axis != nullptr)
	{
		// the hectometres of the kilometrage of the line, the jumps in it too
		double km{0.0};
		if (kilometrage::km_at(tool.line, tool.point, km))
			tool.km = std::round(km * 1000.0) / 1000.0;
		tool.growth = kilometrage::growth_at(tool.line, tool.point, sampled_direction(tool.samples, tool.origin)) < 0 ? -1 : 1;
		for (auto const &hectometre : kilometrage::hectometres(*axis))
		{
			hekto_post post;
			post.hectometres = hectometre.number;
			post.chainage = project(tool.samples, hectometre.at, distance);
			if (distance > kLineCorridor || post.chainage <= 0.0 || post.chainage >= length || hectometre.number < 0 || tool.posts.size() >= 5000)
				continue;
			post.growth = glm::dot(hectometre.rising, sampled_direction(tool.samples, post.chainage)) < 0.0 ? -1 : 1;
			place(post);
		}
		std::sort(tool.posts.begin(), tool.posts.end(), [](hekto_post const &A, hekto_post const &B) { return A.chainage < B.chainage; });
	}
	else
	{
		auto const km_at = [&](double const Chainage) { return tool.km + tool.growth * (Chainage - tool.origin) / 1000.0; };
		auto const low{std::max(0, static_cast<int>(std::ceil(std::min(km_at(0.0), km_at(length)) * 10.0 - 1e-6)))};
		auto const high{static_cast<int>(std::floor(std::max(km_at(0.0), km_at(length)) * 10.0 + 1e-6))};
		for (int hectometres = low; hectometres <= high && tool.posts.size() < 5000; ++hectometres)
		{
			hekto_post post;
			post.hectometres = hectometres;
			post.chainage = tool.origin + tool.growth * (hectometres / 10.0 - tool.km) * 1000.0;
			post.growth = tool.growth;
			if (post.chainage < 0.0 || post.chainage > length)
				continue;
			place(post);
		}
	}
	// each post standing goes with the place of its number nearest to it; of the posts going with a place the one nearest to it
	// stands for it, the others are duplicates. it stands wrong when it's farther from the place than allowed: along the line,
	// across it, or on the other side
	for (auto &found : tool.found)
	{
		found.outside = true;
		found.place = -1;
		for (int i = 0; i < static_cast<int>(tool.posts.size()); ++i)
		{
			auto const &post{tool.posts[i]};
			if (post.hectometres != found.hectometres)
				continue;
			auto const distance{plan_distance(found.post.location, post.position)};
			if (found.place < 0 || distance < found.distance)
			{
				found.outside = false;
				found.place = i;
				found.distance = distance;
				found.expected = post.chainage;
				found.error = found.chainage - post.chainage;
			}
		}
	}
	for (int i = 0; i < static_cast<int>(tool.found.size()); ++i)
	{
		auto const &found{tool.found[i]};
		if (found.place < 0)
			continue;
		auto &post{tool.posts[found.place]};
		if (post.standing < 0 || found.distance < tool.found[post.standing].distance)
			post.standing = i;
	}
	for (int i = 0; i < static_cast<int>(tool.found.size()); ++i)
	{
		auto &found{tool.found[i]};
		found.duplicate = found.place >= 0 && tool.posts[found.place].standing != i;
		found.wrong = found.place >= 0 && false == found.duplicate && found.distance > tool.tolerance;
	}
}

void hekto_tool::hekto_remove_duplicates()
{
	auto &tool{m_hekto};
	hekto_update();
	std::vector<scene::instance_handle> instances;
	for (auto const &found : tool.found)
		if (found.duplicate)
			instances.push_back(found.post.instance);
	auto const removed{m_templates.remove_templates(instances)};
	tool.status = format(STR_P("%zu duplicate hectometre post removed, Ctrl+Z brings it back", "%zu duplicate hectometre posts removed, Ctrl+Z brings them back", static_cast<long long>(removed)), removed);
	WriteLog("Editor: " + tool.status);
	tool.dirty = true;
}

void hekto_tool::hekto_place()
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
	auto const layer{items.empty() ? null_handle : m_templates.output_layer("hektometry")};
	auto const placed{m_templates.place_templates(items, error, layer)};
	tool.status = placed > 0 ? format(STR_P("%zu hectometre post placed, Ctrl+Z takes it back", "%zu hectometre posts placed, Ctrl+Z takes them back", static_cast<long long>(placed)), placed) : error;
	if (placed > 0 && layer != null_handle)
		tool.status += format(STR_C(" (layer \"%s\")"), scene::Layers.layer(layer).name.c_str());
	if (placed > 0)
		m_guide.first_step_done(editor_first_step::posts);
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.dirty = true;
}

void hekto_tool::hekto_fix()
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
	auto const moved{m_templates.move_templates(moves)};
	tool.status = format(STR_P("%zu hectometre post moved to its place, Ctrl+Z takes it back", "%zu hectometre posts moved to their places, Ctrl+Z takes it back", static_cast<long long>(moved)), moved);
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.dirty = true;
}

void hekto_tool::render_options()
{
	auto &tool{m_hekto};
	auto *selected{m_selection.selected_track()};
	if (tool.track == nullptr || (selected != nullptr && selected != tool.track && ImGui::SmallButton(STR_C("Along the line of the selected path"))))
	{
		if (selected == nullptr)
		{
			ImGui::TextDisabled("%s", STR_C("Select a path of the line, or RMB on it: Hectometre posts"));
			return;
		}
		start(*selected, bezier{selected->m_paths.front()}.point(0.0));
	}
	auto const changed = [&](bool const Changed) {
		if (Changed)
			tool.dirty = true;
	};
	if (false == tool.line.empty())
	{
		ImGui::TextWrapped("%s", format(STR_C("The km of line %s, as the Kilometrage tool sets them, its jumps too"), tool.line.c_str()).c_str());
		ImGui::TextDisabled("%s", format(STR_C("km %s at the point"), kilometrage::text(tool.km).c_str()).c_str());
		if (ImGui::SmallButton(STR_C("Give the km here instead")))
		{
			tool.own_km = true;
			tool.km_read = false;
			tool.dirty = true;
		}
		item_tooltip("The posts go by the km given at the point and the way it grows, without the jumps of the line");
	}
	else
	{
		if (false == tool.line_set.empty() && ImGui::SmallButton(format(STR_C("Follow the km of line %s"), tool.line_set.c_str()).c_str()))
		{
			tool.own_km = false;
			tool.dirty = true;
		}
		if (begin_fields("##hektokm", 0.45f))
		{
			field_row("Kilometre at the point (km)");
			changed(ImGui::InputDouble("##hektokilometre", &tool.km, 0.0, 0.0, "%.3f"));
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(1);
			if (ImGui::Button(STR_C("Read from the posts"), ImVec2(-FLT_MIN, 0.0f)))
			{
				tool.km_read = false;
				tool.dirty = true;
			}
			ImGui::EndTable();
		}
		ImGui::TextDisabled("%s", STR_C("The kilometrage grows"));
		changed(segmented("growth", tool.growth, {{1, STR_C("along the line")}, {-1, STR_C("against it")}}));
	}
	ImGui::TextDisabled("%s", STR_C("Side, looking where it grows"));
	changed(segmented("side", tool.side, {{1, STR_C("on the right")}, {-1, STR_C("on the left")}, {0, STR_C("alternately"), "The even hectometres on the right, the odd ones on the left"}}));
	if (ImGui::TreeNode(STR_C("More settings")))
	{
		ImGui::TextDisabled("%s", STR_C("Look"));
		segmented("hektostyle", tool.style, {{0, STR_C("new")}, {1, STR_C("old")}, {2, STR_C("rotten")}, {3, STR_C("mixed"), "New, old and rotten ones at random, mostly new"}});
		ImGui::TextDisabled("%s", STR_C("Turned by (deg)"));
		if (segmented("hektoturn", tool.turn, {{0, "0"}, {90, "90"}, {180, "180"}, {270, "270"}}))
			tool.dirty = true;
		if (begin_fields("##hektofields", 0.6f))
		{
			field_row("From the axis (m)");
			changed(ImGui::InputDouble("##hektooffset", &tool.offset, 0.0, 0.0, "%.2f"));
			field_row("Along the line each way (m)");
			changed(ImGui::InputDouble("##hektoreach", &tool.reach, 0.0, 0.0, "%.0f"));
			field_row("Tolerance (m)");
			changed(ImGui::InputDouble("##hektotolerance", &tool.tolerance, 0.0, 0.0, "%.1f"));
			item_tooltip("A post standing farther than this from its place counts as wrong");
			ImGui::EndTable();
		}
		ImGui::TreePop();
	}
	tool.offset = std::clamp(tool.offset, 1.5, 30.0);
	tool.reach = std::clamp(tool.reach, 100.0, 100000.0);
	tool.tolerance = std::clamp(tool.tolerance, 0.5, 50.0);
	hekto_update();
	if (false == tool.error.empty())
	{
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", tool.error.c_str());
		return;
	}
	auto const missing{std::count_if(tool.posts.begin(), tool.posts.end(), [](hekto_post const &Post) { return Post.standing < 0; })};
	auto const wrong{std::count_if(tool.found.begin(), tool.found.end(), [](hekto_found const &Post) { return Post.wrong; })};
	auto const duplicates{std::count_if(tool.found.begin(), tool.found.end(), [](hekto_found const &Post) { return Post.duplicate; })};
	ImGui::Text(STR_C("To place: %d"), static_cast<int>(missing));
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip(STR_C("Line %.0f m, km %s - %s. Standing: %zu, %d of them wrong"), tool.samples.empty() ? 0.0 : tool.samples.back().chainage, tool.posts.empty() ? "-" : km_text(std::min(tool.posts.front().hectometres, tool.posts.back().hectometres)).c_str(),
		                  tool.posts.empty() ? "-" : km_text(std::max(tool.posts.front().hectometres, tool.posts.back().hectometres)).c_str(), tool.found.size(), static_cast<int>(wrong));
	if (missing > 0)
	{
		if (main_button(format(STR_P("Place %d post", "Place %d posts", static_cast<long long>(missing)), static_cast<int>(missing)).c_str(), ImVec2(-1.0f, tall_button_height())))
			hekto_place();
	}
	if (duplicates > 0)
	{
		if (ImGui::Button(format(STR_P("Remove %d duplicate post", "Remove %d duplicate posts", static_cast<long long>(duplicates)), static_cast<int>(duplicates)).c_str()))
			hekto_remove_duplicates();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Posts with a number another post nearer to its place already has"));
	}
	if (wrong > 0 || duplicates > 0)
	{
		if (wrong > 0)
		{
			if (duplicates > 0)
				ImGui::SameLine();
			if (ImGui::Button(format(STR_P("Move %d wrong post to its place", "Move %d wrong posts to their places", static_cast<long long>(wrong)), static_cast<int>(wrong)).c_str()))
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
				m_view.fly_to(post.post.location, glm::normalize(glm::dvec3{-1.0, -0.8, -1.0}), 20.0);
			}
		}
		ImGui::EndChild();
	}
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

void hekto_tool::draw_overlay() const
{
	auto const &tool{m_hekto};
	if (false == m_tab.open || tool.track == nullptr)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
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

void fouling_tool::fouling_update()
{
	auto &tool{m_fouling};
	auto *selected{m_selection.selected_track()};
	if (tool.scope == 0 && selected != tool.selected)
		tool.dirty = true;
	if (false == tool.dirty && tool.history == m_history.history_revision())
		return;
	tool.dirty = false;
	tool.history = m_history.history_revision();
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
			point.position = tool.height_mode == 1 ? glm::dvec3{middle.x, 0.5 * (onbranch.y + onmain.y) + kRailHeight + tool.height, middle.z} : m_ground.placement_on_ground({middle.x, std::max(onbranch.y, onmain.y), middle.z});
			auto const back{plan_of(previous - branch[i])};
			point.yaw = std::fmod(heading_of(glm::length(back) > 1e-9 ? glm::normalize(back) : glm::dvec2{0.0, 1.0}) + tool.turn, 360.0);
			tool.points.push_back(point);
			break;
		}
	}
	// markers standing: each goes to the nearest fouling point, the others are left alone
	auto const markers{m_templates.standing_templates([](std::string const &File) { return File.find("w17") != std::string::npos || File.find("ukres") != std::string::npos; })};
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

void fouling_tool::place(int const Only)
{
	auto &tool{m_fouling};
	fouling_update();
	std::vector<template_item> items;
	for (int i = 0; i < static_cast<int>(tool.points.size()); ++i)
	{
		auto const &point{tool.points[i]};
		if (point.standing >= 0 || (Only >= 0 && i != Only))
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
	auto const placed{m_templates.place_templates(items, error)};
	tool.status = placed > 0 ? format(STR_P("%zu fouling point marker placed, Ctrl+Z takes it back", "%zu fouling point markers placed, Ctrl+Z takes them back", static_cast<long long>(placed)), placed) : error;
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.dirty = true;
}

void fouling_tool::fix(int const Only)
{
	auto &tool{m_fouling};
	fouling_update();
	std::vector<std::pair<standing_template, template_item>> moves;
	for (int i = 0; i < static_cast<int>(tool.found.size()); ++i)
	{
		auto const &found{tool.found[i]};
		if (false == found.wrong || (Only >= 0 && i != Only))
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
	auto const moved{m_templates.move_templates(moves)};
	tool.status = format(STR_P("%zu fouling point marker moved to its place, Ctrl+Z takes it back", "%zu fouling point markers moved to their places, Ctrl+Z takes it back", static_cast<long long>(moved)), moved);
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.dirty = true;
}

std::optional<std::pair<int, bool>> fouling_tool::hit() const
{
	auto const &tool{m_fouling};
	screen_projection const projection;
	auto const &mouse{ImGui::GetIO().MousePos};
	glm::dvec3 const camera{Global.pCamera.Pos};
	std::optional<std::pair<int, bool>> result;
	auto best{kFoulingPick * kFoulingPick};
	auto const consider = [&](glm::dvec3 const &Position, int const Index, bool const Marker) {
		ImVec2 screen;
		if (glm::distance(Position, camera) > kOverlayRange || false == projection.project(Position, screen))
			return;
		auto const distance2{(screen.x - mouse.x) * (screen.x - mouse.x) + (screen.y - mouse.y) * (screen.y - mouse.y)};
		if (distance2 < best)
		{
			best = distance2;
			result = std::make_pair(Index, Marker);
		}
	};
	for (int i = 0; i < static_cast<int>(tool.points.size()); ++i)
		if (tool.points[i].standing < 0)
			consider(tool.points[i].position, i, false);
	for (int i = 0; i < static_cast<int>(tool.found.size()); ++i)
		if (tool.found[i].wrong)
			consider(tool.found[i].marker.location, i, true);
	return result;
}

void fouling_tool::render_options()
{
	auto &tool{m_fouling};
	auto const changed = [&](bool const Changed) {
		if (Changed)
			tool.dirty = true;
	};
	changed(segmented("foulingscope", tool.scope, {{0, STR_C("The selected switch")}, {1, STR_C("Around the camera")}, {2, STR_C("Whole scenery")}}));
	fouling_update();
	if (false == tool.error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%s", tool.error.c_str());
	else
	{
		ImGui::TextWrapped("%s", tool.status.c_str());
		auto const missing{std::count_if(tool.points.begin(), tool.points.end(), [](fouling_point const &Point) { return Point.standing < 0; })};
		auto const wrong{std::count_if(tool.found.begin(), tool.found.end(), [](fouling_found const &Found) { return Found.wrong; })};
		auto const tall{tall_button_height()};
		if (missing > 0)
		{
			if (main_button(format(STR_P("Place %d marker", "Place %d markers", static_cast<long long>(missing)), static_cast<int>(missing)).c_str(), ImVec2(-1.0f, tall)))
				place();
		}
		if (wrong > 0)
		{
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.4f, 0.1f, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.48f, 0.15f, 1.0f));
			if (ImGui::Button(format(STR_P("Move %d wrong marker to its place", "Move %d wrong markers to their places", static_cast<long long>(wrong)), static_cast<int>(wrong)).c_str(), ImVec2(-1.0f, tall)))
				fix();
			ImGui::PopStyleColor(2);
		}
		if (missing + wrong > 0)
		{
			// each one: a click flies there, the button fixes it alone; in the view a click on its point does the same
			ImGui::TextDisabled("%s", STR_C("Click the green or red point in the view: that one only"));
			ImGui::BeginChild("##foulinglist", ImVec2(0.0f, std::min(8, static_cast<int>(missing + wrong)) * ImGui::GetFrameHeightWithSpacing() + 6.0f), true);
			auto const look{glm::normalize(glm::dvec3{-1.0, -0.8, -1.0})};
			for (int i = 0; i < static_cast<int>(tool.points.size()); ++i)
			{
				auto const &point{tool.points[i]};
				if (point.standing >= 0)
					continue;
				ImGui::PushID(i);
				if (ImGui::SmallButton(STR_C("Place")))
					place(i);
				ImGui::SameLine();
				if (ImGui::Selectable(format(STR_C("%s: no W17"), point.track->name().c_str()).c_str()))
					m_view.fly_to(point.position, look, 25.0);
				ImGui::PopID();
			}
			for (int i = 0; i < static_cast<int>(tool.found.size()); ++i)
			{
				auto const &found{tool.found[i]};
				if (false == found.wrong)
					continue;
				ImGui::PushID(1000000 + i);
				if (ImGui::SmallButton(STR_C("Fix")))
					fix(i);
				ImGui::SameLine();
				if (ImGui::Selectable(format(STR_C("%s: W17 %.1f m off, %+.2f m high"), tool.points[found.point].track->name().c_str(), found.error, found.height_error).c_str()))
					m_view.fly_to(found.marker.location, look, 25.0);
				ImGui::PopID();
			}
			ImGui::EndChild();
		}
	}
	if (ImGui::TreeNode(STR_C("More settings")))
	{
		if (begin_fields("##foulingfields", 0.6f))
		{
			if (tool.scope == 1)
			{
				field_row("Around the camera (m)");
				changed(ImGui::InputDouble("##foulingreach", &tool.reach, 0.0, 0.0, "%.0f"));
			}
			field_row("Spacing of the axes (m)");
			changed(ImGui::InputDouble("##foulingspacing", &tool.spacing, 0.0, 0.0, "%.2f"));
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s\n%s", STR_C("W17 where the axes of the tracks meeting at a switch are the spacing apart,\nhalf way between them"), STR_C("3.5 m on the straight track; more on the curves, where the cars lean out"));
			field_row("Tolerance (m)");
			changed(ImGui::InputDouble("##foulingtolerance", &tool.tolerance, 0.0, 0.0, "%.1f"));
			item_tooltip("A marker standing farther than this from its place counts as wrong");
			ImGui::EndTable();
		}
		changed(segmented("foulingheight", tool.height_mode, {{0, STR_C("On the ground")}, {1, STR_C("Over the rail heads")}}));
		if (tool.height_mode == 1 && begin_fields("##foulingheights", 0.6f))
		{
			field_row("by (m)");
			changed(ImGui::InputDouble("##foulingby", &tool.height, 0.0, 0.0, "%.3f"));
			field_row("Height tolerance (m)");
			changed(ImGui::InputDouble("##foulingheighttolerance", &tool.height_tolerance, 0.0, 0.0, "%.3f"));
			item_tooltip("A marker higher or lower than its place by more than this counts as wrong");
			tool.height_tolerance = std::clamp(tool.height_tolerance, 0.005, 1.0);
			ImGui::EndTable();
		}
		ImGui::TextDisabled("%s", STR_C("Marker"));
		segmented("foulingmodel", tool.model,
		          {{0, "w17", "Round post (ip/pkp/w17.inc)"}, {1, "w17_new", "Round post, taller than w17 (ip/pkp/w17_new.inc)"}, {2, "w17_old", "Square post (ip/pkp/w17_old.inc)"}, {3, STR_C("I-beam"), "Post of an HEB I-beam (ip/pkp/w17_dwuteo.inc)"}});
		ImGui::TextDisabled("%s", STR_C("Turned by (deg)"));
		if (segmented("foulingturn", tool.turn, {{0, "0"}, {90, "90"}, {180, "180"}, {270, "270"}}))
			tool.dirty = true;
		ImGui::TreePop();
	}
	tool.reach = std::clamp(tool.reach, 50.0, 50000.0);
	tool.spacing = std::clamp(tool.spacing, 2.0, 10.0);
	tool.tolerance = std::clamp(tool.tolerance, 0.2, 20.0);
}

void fouling_tool::draw_overlay() const
{
	auto const &tool{m_fouling};
	if (false == m_tab.open)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
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

editor_selftest_result hekto_tool::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "hekto")
	{
		auto values{selftest_values(Words)};
		auto *track{simulation::Paths.find(values["track"])};
		if (track == nullptr)
		{
			WriteLog("SELFTEST hekto: no such track");
			return editor_selftest_result::handled;
		}
		start(*track, track->m_paths.front().points[segment_data::point::start]);
		auto &tool{m_hekto};
		if (values.count("reach"))
			tool.reach = std::stod(values["reach"]);
		tool.own_km = values["own"] == "1";
		tool.dirty = true;
		hekto_update();
		if (values.count("km"))
		{
			tool.km = std::stod(values["km"]);
			tool.growth = values["growth"] == "-1" ? -1 : 1;
		}
		tool.side = values["side"] == "alt" ? 0 : values["side"] == "left" ? -1 : 1;
		tool.dirty = true;
		hekto_update();
		auto const missing{std::count_if(tool.posts.begin(), tool.posts.end(), [](hekto_post const &Post) { return Post.standing < 0; })};
		auto const wrong{std::count_if(tool.found.begin(), tool.found.end(), [](hekto_found const &Post) { return Post.wrong; })};
		auto const duplicates{std::count_if(tool.found.begin(), tool.found.end(), [](hekto_found const &Post) { return Post.duplicate; })};
		WriteLog(format("SELFTEST hekto: line %s km %.3f growth %d, %zu posts along %.0f m, %d to place, %zu standing, %d wrong, %d duplicate. %s", tool.line.empty() ? "-" : tool.line.c_str(), tool.km, tool.growth, tool.posts.size(), tool.samples.empty() ? 0.0 : tool.samples.back().chainage, static_cast<int>(missing), tool.found.size(),
		                static_cast<int>(wrong), static_cast<int>(duplicates), tool.status.c_str()));
		for (std::size_t i = 0; i < tool.posts.size() && i < 4; ++i)
		{
			auto const &post{tool.posts[i]};
			double lateral;
			glm::dvec3 const axis{editor_track::sampled_position(tool.samples, post.chainage)};
			auto const direction{editor_track::sampled_position(tool.samples, post.chainage + 1.0) - axis};
			glm::dvec2 const offset{post.position.x - axis.x, post.position.z - axis.z};
			lateral = glm::length(offset);
			auto const right{(direction.x * offset.y - direction.z * offset.x) * post.growth >= 0.0};
			WriteLog(format("SELFTEST hekto post %d.%d at %.1f m, %.2f m from the axis, %s of the growing kilometrage, at %.1f %.1f %.1f", post.hectometres / 10, post.hectometres % 10, post.chainage, lateral, right ? "right" : "left", post.position.x, post.position.y,
			                post.position.z));
		}
		if (values["list"] == "1")
		{
			std::string list;
			for (auto const &post : tool.posts)
				list += format(" %d@%.0f", post.hectometres, post.chainage);
			WriteLog("SELFTEST hekto posts:" + list);
		}
		if (values["place"] == "1")
		{
			hekto_place();
			WriteLog("SELFTEST hekto: " + tool.status);
		}
		return editor_selftest_result::handled;
	}
	return editor_selftest_result::unhandled;
}

glm::dvec3 fouling_tool::position_of(std::pair<int, bool> const &Hit) const
{
	return Hit.second ? m_fouling.found[Hit.first].marker.location : m_fouling.points[Hit.first].position;
}

void fouling_tool::check_selected_switch()
{
	m_fouling.scope = 0;
	m_fouling.dirty = true;
	m_tab.expand = true;
}

editor_selftest_result fouling_tool::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "fouling")
	{
		auto values{selftest_values(Words)};
		auto &tool{m_fouling};
		if (values["show"] == "1")
		{
			m_view.show_track_tab(editor_track_tab::lineside);
			m_tab.expand = true;
		}
		tool.scope = std::stoi(values.count("scope") ? values["scope"] : "2");
		tool.height_mode = values["mode"] == "ground" ? 0 : 1;
		tool.height = std::stod(values.count("height") ? values["height"] : "0");
		tool.dirty = true;
		fouling_update();
		WriteLog("SELFTEST fouling: " + tool.status);
		int shown{0};
		for (auto const &found : tool.found)
			if (found.wrong && shown++ < 5)
				WriteLog(format("SELFTEST fouling: %s %.2f m off, %+.3f m high", found.marker.file.c_str(), found.error, found.height_error));
		if (values["place"] == "one")
		{
			// the first fouling point without a marker alone, as a click on it in the view
			for (int index = 0; index < static_cast<int>(tool.points.size()); ++index)
				if (tool.points[index].standing < 0)
				{
					place(index);
					break;
				}
			tool.dirty = true;
			fouling_update();
			WriteLog("SELFTEST fouling one: " + tool.status);
		}
		if (values["place"] == "1")
		{
			place();
			tool.dirty = true;
			fouling_update();
			WriteLog("SELFTEST fouling after placing: " + tool.status);
		}
		if (values["fix"] == "1")
		{
			fix();
			tool.dirty = true;
			fouling_update();
			WriteLog("SELFTEST fouling after the fix: " + tool.status);
		}
		return editor_selftest_result::handled;
	}
	return editor_selftest_result::unhandled;
}

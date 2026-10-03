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
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"

#include "editor/editorIncludeInfo.hpp"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "world/Event.h"
#include "world/EvLaunch.h"
#include "world/MemCell.h"
#include "world/Track.h"
#include "world/Traction.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <set>
#include <optional>
#include <sstream>
#include <unordered_set>

namespace
{

using geometry::plan_distance;
using geometry::plan_of;

std::string const kBindingMark{"//$b"};

template <typename Type_>
bool contains(std::vector<Type_> const &Items, Type_ const &Item)
{
	return std::find(Items.begin(), Items.end(), Item) != Items.end();
}

ImU32 colour(infra::category const Category, int const Alpha = 255)
{
	switch (Category)
	{
	case infra::category::signal: return IM_COL32(255, 80, 80, Alpha);
	case infra::category::sign: return IM_COL32(255, 220, 60, Alpha);
	case infra::category::pole: return IM_COL32(160, 160, 255, Alpha);
	case infra::category::catenary: return IM_COL32(80, 200, 255, Alpha);
	case infra::category::logic: return IM_COL32(220, 120, 255, Alpha);
	case infra::category::other: return IM_COL32(200, 200, 200, Alpha);
	}
	return IM_COL32(200, 200, 200, Alpha);
}

// nodes still present in the scene; the editor deletes models outright
class live_nodes
{
  public:
	void refresh()
	{
		std::array<std::size_t, 4> const sizes{simulation::Instances.sequence().size(), simulation::Traction.sequence().size(), simulation::Memory.sequence().size(), simulation::Events.launchers().size()};
		if (sizes == m_sizes && m_valid)
			return;
		m_sizes = sizes;
		m_valid = true;
		m_nodes.clear();
		for (auto *node : simulation::Instances.sequence())
			m_nodes.insert(node);
		for (auto *node : simulation::Traction.sequence())
			m_nodes.insert(node);
		for (auto *node : simulation::Memory.sequence())
			m_nodes.insert(node);
		for (auto *node : simulation::Events.launchers())
			m_nodes.insert(node);
	}
	bool alive(scene::basic_node const *Node) const
	{
		return Node != nullptr && m_nodes.count(Node) > 0;
	}

  private:
	std::unordered_set<scene::basic_node const *> m_nodes;
	std::array<std::size_t, 4> m_sizes{};
	bool m_valid{false};
};
live_nodes Live;

bool alive(infra::binding const &Binding)
{
	if (Binding.type == infra::kind::include)
	{
		if (false == scene::Layers.tracked(Binding.include))
			return false;
		auto const &included{scene::Layers.instance(Binding.include)};
		return false == included.removed && false == included.dead;
	}
	return Live.alive(Binding.node);
}

// placement parameters of an include
struct include_place
{
	std::string file;
	std::vector<std::string> values;
	std::array<int, 3> position{-1, -1, -1}; // indices of the values
	int yaw{-1};
	glm::dvec3 offset{0.0};
	std::string description; // name and category of the template
};

bool numeric(std::string const &Text, double &Value)
{
	char *end{nullptr};
	Value = std::strtod(Text.c_str(), &end);
	return false == Text.empty() && *end == '\0';
}

bool include_place_of(scene::instance_handle const Instance, include_place &Place)
{
	if (false == scene::Layers.tracked(Instance))
		return false;
	auto const &included{scene::Layers.instance(Instance)};
	if (included.removed || included.dead || included.file == nullptr)
		return false;
	// with a rotation or a scale in effect the parameters don't translate to a location in a single way
	if (included.context.rotation != glm::vec3{0.f} || included.context.scale != glm::vec3{1.f})
		return false;
	if (false == editor_includes::parse_directive(scene::Layers.directive(Instance), Place.file, Place.values))
		return false;
	static std::map<std::string, include_info> descriptions;
	auto found{descriptions.find(*included.file)};
	if (found == descriptions.end())
	{
		include_info info;
		std::string error;
		editor_includes::load(*included.file, info, error);
		editor_includes::suggest(*included.file, info);
		found = descriptions.emplace(*included.file, std::move(info)).first;
	}
	auto const &info{found->second};
	auto const index = [&](char const *Role) {
		auto const result{editor_includes::parameter_with_role(info, Role) - 1};
		double value;
		return (result >= 0 && result < static_cast<int>(Place.values.size()) && numeric(Place.values[result], value)) ? result : -1;
	};
	Place.position = {index("pos.x"), index("pos.y"), index("pos.z")};
	Place.yaw = index("rot.y");
	Place.offset = included.context.offset;
	Place.description = info.name + ' ' + info.category;
	return Place.position[0] >= 0 && Place.position[1] >= 0 && Place.position[2] >= 0;
}

glm::dvec3 include_location(include_place const &Place)
{
	glm::dvec3 result{Place.offset};
	for (int axis = 0; axis < 3; ++axis)
	{
		double value{0.0};
		numeric(Place.values[Place.position[axis]], value);
		result[axis] += value;
	}
	return result;
}

// points which define the object, and its rotation around the vertical axis if it has one
bool object_points(infra::binding const &Binding, std::vector<glm::dvec3> &Points, std::optional<double> &Yaw)
{
	Points.clear();
	Yaw.reset();
	switch (Binding.type)
	{
	case infra::kind::model:
	{
		auto const *model{static_cast<TAnimModel const *>(Binding.node)};
		Points.push_back(model->location());
		Yaw = model->Angles().y;
		return true;
	}
	case infra::kind::include:
	{
		include_place place;
		if (false == include_place_of(Binding.include, place))
			return false;
		Points.push_back(include_location(place));
		if (place.yaw >= 0)
		{
			double value{0.0};
			numeric(place.values[place.yaw], value);
			Yaw = value;
		}
		return true;
	}
	case infra::kind::traction:
	{
		auto const *traction{static_cast<TTraction const *>(Binding.node)};
		Points = {traction->pPoint1, traction->pPoint2, traction->pPoint3, traction->pPoint4};
		return true;
	}
	case infra::kind::memcell:
	case infra::kind::launcher: Points.push_back(Binding.node->location()); return true;
	}
	return false;
}

std::string object_key(infra::binding const &Binding)
{
	std::string result;
	switch (Binding.type)
	{
	case infra::kind::model:
	{
		auto const *model{static_cast<TAnimModel const *>(Binding.node)};
		result = model->Model() != nullptr ? model->Model()->NameGet() : std::string{};
		break;
	}
	case infra::kind::include: result = *scene::Layers.instance(Binding.include).file; break;
	case infra::kind::traction: break;
	case infra::kind::memcell:
	case infra::kind::launcher: result = Binding.node->name(); break;
	}
	if (result.empty() || result.find_first_of(" \t\r\n") != std::string::npos)
		result = "-";
	return result;
}

std::string object_label(infra::binding const &Binding)
{
	switch (Binding.type)
	{
	case infra::kind::model:
	{
		auto const *model{static_cast<TAnimModel const *>(Binding.node)};
		auto name{model->Model() != nullptr ? model->Model()->NameGet() : std::string{}};
		if (false == model->name().empty() && model->name() != "none")
			name = model->name() + " (" + name + ')';
		return name;
	}
	case infra::kind::include: return *scene::Layers.instance(Binding.include).file;
	case infra::kind::traction: return Binding.node->name().empty() || Binding.node->name() == "none" ? std::string{"traction"} : Binding.node->name();
	case infra::kind::memcell: return "memcell " + Binding.node->name();
	case infra::kind::launcher: return "eventlauncher " + Binding.node->name();
	}
	return {};
}

void move_model(TAnimModel *Model, glm::dvec3 const &Location, glm::vec3 const &Angles)
{
	if (glm::distance(Model->location(), Location) > 1e-6)
	{
		simulation::Region->erase(Model);
		Model->location(Location);
		simulation::Region->insert(Model);
	}
	Model->Angles(Angles);
}

void move_traction(TTraction *Traction, std::array<glm::dvec3, 4> const &Points)
{
	if (Traction->pPoint1 == Points[0] && Traction->pPoint2 == Points[1] && Traction->pPoint3 == Points[2] && Traction->pPoint4 == Points[3])
		return;
	// the height of the lowest dropper stays the same
	auto const minheight{(Traction->pPoint3.y - Traction->pPoint1.y + Traction->pPoint4.y - Traction->pPoint2.y) * 0.5 - Traction->fHeightDifference};
	simulation::Region->erase_and_unregister(Traction);
	Traction->pPoint1 = Points[0];
	Traction->pPoint2 = Points[1];
	Traction->pPoint3 = Points[2];
	Traction->pPoint4 = Points[3];
	Traction->fHeightDifference = (Points[2].y - Points[0].y + Points[3].y - Points[1].y) * 0.5 - minheight;
	Traction->Init();
	Traction->location(glm::mix(Points[1], Points[0], 0.5));
	Traction->m_area.radius = -1.f;
	simulation::Region->insert_and_register(Traction);
	Traction->m_geometry = gfx::geometry_handle{};
	auto &section{simulation::Region->section(Traction->location())};
	if (section.m_geometrycreated)
		Traction->create_geometry(section.m_geometrybank);
	Traction->mark_dirty();
}

void move_memcell(TMemCell *Memcell, glm::dvec3 const &Location)
{
	if (glm::distance(Memcell->location(), Location) <= 1e-6)
		return;
	simulation::Region->erase(Memcell);
	Memcell->location(Location);
	simulation::Region->insert(Memcell);
}

void move_launcher(TEventLauncher *Launcher, glm::dvec3 const &Location)
{
	if (glm::distance(Launcher->location(), Location) <= 1e-6)
		return;
	// global and radio launchers aren't a part of the region
	auto const placed{false == Launcher->IsGlobal() && false == Launcher->IsRadioActivated()};
	if (placed)
		simulation::Region->erase(Launcher);
	Launcher->location(Location);
	if (placed)
		simulation::Region->insert(Launcher);
	Launcher->mark_dirty();
}

// bounding circle of the paths of a track, in the plan
struct track_area
{
	TTrack *track{nullptr};
	glm::dvec3 centre{0.0};
	double radius{0.0};
};

track_area area_of(TTrack &Track)
{
	track_area result;
	result.track = &Track;
	std::vector<glm::dvec3> points;
	for (auto const &path : Track.m_paths)
	{
		auto const &start{path.points[segment_data::point::start]};
		auto const &end{path.points[segment_data::point::end]};
		points.insert(points.end(), {start, end, start + path.points[segment_data::point::control1], end + path.points[segment_data::point::control2]});
	}
	if (points.empty())
		return result;
	for (auto const &point : points)
		result.centre += point / static_cast<double>(points.size());
	for (auto const &point : points)
		result.radius = std::max(result.radius, plan_distance(point, result.centre));
	return result;
}

// the point lies sideways of the path, not beyond one of its ends
bool beside(infra::station const &Station, glm::dvec3 const &Point, bool const Interior)
{
	if (Interior)
		return true;
	auto const axis{infra::frame_at(Station)};
	return std::abs(glm::dot(plan_of(Point - axis.point), axis.forward)) <= 0.5;
}

// nearest point of the tracks, sideways from the axis within the corridor
template <typename Areas_>
bool nearest_beside(Areas_ const &Areas, glm::dvec3 const &Point, double const Corridor, infra::station &Station)
{
	auto best{-1.0};
	for (auto const &entry : Areas)
	{
		track_area const &area{entry};
		if (plan_distance(Point, area.centre) > area.radius + Corridor)
			continue;
		infra::station station;
		bool interior;
		auto const distance{infra::project(*area.track, Point, station, interior)};
		if (distance <= Corridor && (best < 0.0 || distance < best) && beside(station, Point, interior))
		{
			best = distance;
			Station = station;
		}
	}
	return best >= 0.0;
}

// paths of the whole scenery by cells of the plan, for the lookups reaching up to kReach from the axis
class track_index
{
  public:
	static constexpr double kReach{55.0};
	void add(TTrack &Track)
	{
		auto const index{m_areas.size()};
		m_areas.push_back(area_of(Track));
		std::set<cell_key> cells;
		for (auto const &path : Track.m_paths)
			for (auto const &point : infra::outline(path, kCell * 0.5))
				for (auto x = key(point.x - kReach - kCell * 0.25); x <= key(point.x + kReach + kCell * 0.25); ++x)
					for (auto z = key(point.z - kReach - kCell * 0.25); z <= key(point.z + kReach + kCell * 0.25); ++z)
						cells.insert({x, z});
		for (auto const &cell : cells)
			m_cells[cell].push_back(index);
	}
	bool nearest(glm::dvec3 const &Point, double const Corridor, infra::station &Station) const
	{
		auto const cell{m_cells.find({key(Point.x), key(Point.z)})};
		if (cell == m_cells.end())
			return false;
		std::vector<std::reference_wrapper<track_area const>> areas;
		for (auto const index : cell->second)
			areas.emplace_back(m_areas[index]);
		return nearest_beside(areas, Point, std::min(Corridor, kReach), Station);
	}
	bool empty() const
	{
		return m_areas.empty();
	}

  private:
	static constexpr double kCell{64.0};
	using cell_key = std::pair<long long, long long>;
	static long long key(double const Value)
	{
		return static_cast<long long>(std::floor(Value / kCell));
	}
	std::vector<track_area> m_areas;
	std::map<cell_key, std::vector<std::size_t>> m_cells;
};

// lookup of points by a grid of 1 m cells in the plan
template <typename Type_>
class point_grid
{
  public:
	void add(glm::dvec3 const &Point, Type_ Item)
	{
		m_cells[key(Point)].emplace_back(Point, Item);
	}
	template <typename Match_>
	std::optional<Type_> find(glm::dvec3 const &Point, double const Tolerance, Match_ const &Match) const
	{
		auto const centre{key(Point)};
		for (auto x = centre.first - 1; x <= centre.first + 1; ++x)
			for (auto z = centre.second - 1; z <= centre.second + 1; ++z)
			{
				auto const cell{m_cells.find({x, z})};
				if (cell == m_cells.end())
					continue;
				for (auto const &entry : cell->second)
					if (glm::distance(entry.first, Point) <= Tolerance && Match(entry.second))
						return entry.second;
			}
		return std::nullopt;
	}

  private:
	using cell_key = std::pair<long long, long long>;
	static cell_key key(glm::dvec3 const &Point)
	{
		return {static_cast<long long>(std::floor(Point.x)), static_cast<long long>(std::floor(Point.z))};
	}
	std::map<cell_key, std::vector<std::pair<glm::dvec3, Type_>>> m_cells;
};

double turn_of(infra::binding const &Binding, double const Yaw)
{
	auto const axis{infra::frame_at(Binding.anchors.front().at)};
	return clamp_circular(Yaw - infra::heading(axis.forward));
}

// the objects which the stored bindings can name, by their locations
struct bindable_objects
{
	point_grid<TTrack *> paths;
	point_grid<TAnimModel *> models;
	point_grid<TTraction *> traction;
	point_grid<scene::basic_node *> logic;
	point_grid<scene::instance_handle> includes;

	bindable_objects()
	{
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && false == track->m_editorremoved)
				for (auto const &path : track->m_paths)
					paths.add(path.points[segment_data::point::start], track);
		for (auto *model : simulation::Instances.sequence())
			if (model != nullptr && false == model->from_template() && model->Model() != nullptr)
				models.add(model->location(), model);
		for (auto *piece : simulation::Traction.sequence())
			if (piece != nullptr && false == piece->from_template())
				traction.add(piece->pPoint1, piece);
		for (auto *memcell : simulation::Memory.sequence())
			if (memcell != nullptr && false == memcell->from_template())
				logic.add(memcell->location(), memcell);
		for (auto *launcher : simulation::Events.launchers())
			if (launcher != nullptr && false == launcher->from_template())
				logic.add(launcher->location(), launcher);
		for (scene::instance_handle handle = 1; handle <= scene::Layers.instance_count(); ++handle)
		{
			include_place place;
			if (include_place_of(handle, place))
				includes.add(include_location(place), handle);
		}
	}
};

// format of the lines, coordinates in the space of the file:
//   <kind> <category> <x y z of the object> <file or name, - if none> <turns 0/1> <anchor count> { <path> <start x y z> <end x y z> }
// the object is the model, the include (position from its parameters), the traction piece (its first point), the memory
// cell or the event launcher found at the location; the anchors name the paths by their ends
std::optional<infra::binding> read_binding(std::string const &Text, scene::layer_context const &Context, bindable_objects const &Objects)
{
	double const tolerance{0.05};
	std::istringstream line{Text};
	std::string kindname, key;
	int group{0}, turns{0}, count{0};
	glm::dvec3 location;
	line >> kindname >> group >> location.x >> location.y >> location.z >> key >> turns >> count;
	if (line.fail() || count < 1 || count > 4)
		return std::nullopt;
	location = Context.to_world(location);
	infra::binding binding;
	binding.group = static_cast<infra::category>(std::clamp(group, 0, static_cast<int>(infra::category::other)));
	binding.turns = (turns != 0);
	auto found{false};
	if (kindname == infra::name(infra::kind::model))
	{
		binding.type = infra::kind::model;
		auto const model{Objects.models.find(location, tolerance, [&](TAnimModel *Model) { return Model->Model()->NameGet() == key; })};
		if ((found = model.has_value()))
			binding.node = *model;
	}
	else if (kindname == infra::name(infra::kind::include))
	{
		binding.type = infra::kind::include;
		auto const handle{Objects.includes.find(location, tolerance, [&](scene::instance_handle Handle) { return *scene::Layers.instance(Handle).file == key; })};
		if ((found = handle.has_value()))
			binding.include = *handle;
	}
	else if (kindname == infra::name(infra::kind::traction))
	{
		binding.type = infra::kind::traction;
		auto const piece{Objects.traction.find(location, tolerance, [](TTraction *) { return true; })};
		if ((found = piece.has_value()))
			binding.node = *piece;
	}
	else if (kindname == infra::name(infra::kind::memcell) || kindname == infra::name(infra::kind::launcher))
	{
		binding.type = (kindname == infra::name(infra::kind::memcell) ? infra::kind::memcell : infra::kind::launcher);
		auto const node{Objects.logic.find(location, tolerance, [&](scene::basic_node *Node) {
			auto const ismemcell{dynamic_cast<TMemCell *>(Node) != nullptr};
			return ismemcell == (binding.type == infra::kind::memcell) && (key == "-" || Node->name() == key);
		})};
		if ((found = node.has_value()))
			binding.node = *node;
	}
	std::vector<glm::dvec3> points;
	std::optional<double> yaw;
	if (false == found || false == object_points(binding, points, yaw))
		return std::nullopt;
	for (int i = 0; i < count && found; ++i)
	{
		int path{0};
		glm::dvec3 start, end;
		line >> path >> start.x >> start.y >> start.z >> end.x >> end.y >> end.z;
		start = Context.to_world(start);
		end = Context.to_world(end);
		auto const track{Objects.paths.find(start, tolerance, [&](TTrack *Track) {
			return path >= 0 && path < static_cast<int>(Track->m_paths.size()) &&
			       glm::distance(Track->m_paths[path].points[segment_data::point::start], start) <= tolerance &&
			       glm::distance(Track->m_paths[path].points[segment_data::point::end], end) <= tolerance;
		})};
		if (line.fail() || false == track.has_value())
		{
			found = false;
			break;
		}
		auto const &point{points[std::min<std::size_t>(i, points.size() - 1)]};
		double s;
		bool interior;
		infra::project((*track)->m_paths[path], point, s, interior);
		binding.anchors.push_back(infra::make_anchor({*track, path, s}, point));
	}
	if (false == found || binding.anchors.size() != points.size())
		return std::nullopt;
	binding.turns = binding.turns && yaw.has_value();
	if (binding.turns)
		binding.yaw = turn_of(binding, *yaw);
	binding.label = object_label(binding);
	return binding;
}

} // namespace

void editor_mode::infra_load()
{
	if (m_bindings_loaded || scene::Layers.empty())
		return;
	m_bindings_loaded = true;
	auto const marked{scene::Layers.marked_layers(kBindingMark)};
	if (marked.empty())
		return;

	bindable_objects const objects;
	auto resolved{0};
	for (auto const layer : marked)
	{
		auto const &context{scene::Layers.layer(layer).context_insert()};
		for (auto const &text : scene::Layers.marked(layer, kBindingMark))
		{
			if (text.empty() || text.front() == '#')
				continue;
			auto binding{read_binding(text, context, objects)};
			if (false == binding.has_value())
			{
				m_bindings_unresolved.emplace_back(layer, text);
				continue;
			}
			m_bindings.push_back(std::move(*binding));
			++resolved;
		}
	}
	WriteLog("Editor: " + std::to_string(resolved) + " object(s) bound to the paths read from the scenery" +
	             (m_bindings_unresolved.empty() ? std::string{} : ", " + std::to_string(m_bindings_unresolved.size()) + " binding(s) whose objects or paths weren't found are kept as they are"),
	         logtype::generic);
}

void editor_mode::infra_store()
{
	if (false == m_bindings_loaded || scene::Layers.empty())
		return;
	Live.refresh();
	m_bindings.erase(std::remove_if(m_bindings.begin(), m_bindings.end(), [](infra::binding const &Binding) { return false == alive(Binding); }), m_bindings.end());

	std::map<scene::layer_handle, std::vector<std::string>> lines;
	for (auto const layer : scene::Layers.marked_layers(kBindingMark))
		lines[layer];
	for (auto const &entry : m_bindings_unresolved)
		lines[entry.first].push_back(entry.second);
	for (auto const &binding : m_bindings)
	{
		std::vector<glm::dvec3> points;
		std::optional<double> yaw;
		if (binding.anchors.empty() || false == object_points(binding, points, yaw))
			continue;
		auto layer{scene::Layers.resolve(binding.anchors.front().at.track->layer())};
		if (false == scene::Layers.valid(layer) || false == scene::Layers.accepts(layer))
			layer = scene::Layers.active();
		if (false == scene::Layers.valid(layer))
			layer = 1;
		auto const &context{scene::Layers.layer(layer).context_insert()};
		auto const local = [&](glm::dvec3 const &Point) {
			auto const point{context.to_local(Point)};
			return format("%.4f %.4f %.4f", point.x, point.y, point.z);
		};
		auto text{std::string{infra::name(binding.type)} + ' ' + std::to_string(static_cast<int>(binding.group)) + ' ' + local(points.front()) + ' ' + object_key(binding) + ' ' +
		          (binding.turns ? "1 " : "0 ") + std::to_string(binding.anchors.size())};
		for (auto const &anchor : binding.anchors)
		{
			auto const &path{anchor.at.track->m_paths[anchor.at.path]};
			text += ' ' + std::to_string(anchor.at.path) + ' ' + local(path.points[segment_data::point::start]) + ' ' + local(path.points[segment_data::point::end]);
		}
		lines[layer].push_back(std::move(text));
	}
	for (auto &entry : lines)
	{
		if (false == entry.second.empty())
			entry.second.insert(entry.second.begin(), "# objects bound to the paths in the track editor, they follow changes of the track; the simulation skips these lines");
		scene::Layers.mark(entry.first, kBindingMark, std::move(entry.second));
	}
}

std::vector<TTrack *> editor_mode::infra_scope() const
{
	std::vector<TTrack *> result;
	auto const add = [&](TTrack *Track) {
		if (Track != nullptr && false == Track->m_editorremoved && editor_track::is_supported(*Track) && false == contains(result, Track))
			result.push_back(Track);
	};
	auto *selected{selected_track()};
	switch (m_infra.scope)
	{
	case 0: add(selected); break;
	case 1:
		if (selected != nullptr)
			for (auto const &span : editor_track::run_route(*selected, m_infra.reach).spans)
				add(span.track);
		break;
	case 2:
		for (auto const &span : m_profile.route.spans)
			add(span.track);
		break;
	case 3:
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && false == track->m_editorremoved && editor_track::is_supported(*track))
				result.push_back(track);
		break;
	default: break;
	}
	return result;
}

infra::binding *editor_mode::infra_find(infra::binding const &Binding)
{
	for (auto &binding : m_bindings)
		if (binding.same(Binding))
			return &binding;
	return nullptr;
}

void editor_mode::infra_recognize()
{
	auto &state{m_infra};
	infra_load();
	state.candidates.clear();
	state.hovered = -1;
	state.error.clear();
	state.status.clear();
	state.tracks = infra_scope();
	if (state.tracks.empty())
	{
		state.error = (state.scope == 2 ? "Open the vertical profile of a route first" : state.scope == 3 ? "The scenery has no paths the editor can handle" : "Select a path first");
		return;
	}
	track_index index;
	std::unordered_set<std::string> names;
	for (auto *track : state.tracks)
	{
		index.add(*track);
		if (false == track->name().empty() && track->name() != "none")
			names.insert(track->name());
	}
	Live.refresh();

	auto const corridor{static_cast<double>(state.corridor)};
	auto const add = [&](infra::binding Binding, std::string const &Reason, double const Reach) {
		std::vector<glm::dvec3> points;
		std::optional<double> yaw;
		if (false == object_points(Binding, points, yaw))
			return;
		for (auto const &point : points)
		{
			infra::station station;
			if (false == index.nearest(point, Reach, station))
				return;
			Binding.anchors.push_back(infra::make_anchor(station, point));
		}
		if (std::abs(Binding.anchors.front().height) > 40.0)
			return;
		Binding.turns = yaw.has_value() && Binding.type != infra::kind::traction;
		if (Binding.turns)
			Binding.yaw = turn_of(Binding, *yaw);
		Binding.label = object_label(Binding);
		infra_candidate candidate;
		candidate.offset = Binding.anchors.front().offset;
		candidate.reason = Reason;
		candidate.bound = (infra_find(Binding) != nullptr);
		candidate.chosen = (false == candidate.bound && Binding.group != infra::category::other);
		candidate.binding = std::move(Binding);
		state.candidates.push_back(std::move(candidate));
	};
	auto const rule_reason = [](infra::category const Group, char const *What) { return Group == infra::category::other ? std::string{"nearby, no rule matches the "} + What : std::string{"the "} + What + " matches a rule"; };

	for (auto *model : simulation::Instances.sequence())
	{
		// models of the includes follow along with their include
		if (model == nullptr || model->from_template() || model->Model() == nullptr)
			continue;
		infra::binding binding;
		binding.type = infra::kind::model;
		binding.node = model;
		binding.group = infra::classify(model->Model()->NameGet() + ' ' + model->name(), state.rules);
		add(binding, rule_reason(binding.group, "file name"), corridor);
	}
	for (scene::instance_handle handle = 1; handle <= scene::Layers.instance_count(); ++handle)
	{
		include_place place;
		if (false == include_place_of(handle, place))
			continue;
		infra::binding binding;
		binding.type = infra::kind::include;
		binding.include = handle;
		binding.group = infra::classify(place.file + ' ' + place.description, state.rules);
		add(binding, rule_reason(binding.group, "template or its description"), corridor);
	}
	for (auto *piece : simulation::Traction.sequence())
	{
		if (piece == nullptr || piece->from_template())
			continue;
		infra::binding binding;
		binding.type = infra::kind::traction;
		binding.node = piece;
		binding.group = infra::category::catenary;
		add(binding, "wire over the track", corridor);
	}
	for (auto *memcell : simulation::Memory.sequence())
	{
		if (memcell == nullptr || memcell->from_template())
			continue;
		infra::binding binding;
		binding.type = infra::kind::memcell;
		binding.node = memcell;
		binding.group = infra::category::logic;
		auto const named{false == memcell->asTrackName.empty() && names.count(memcell->asTrackName) > 0};
		add(binding, named ? "names the path " + memcell->asTrackName : std::string{"nearby"}, named ? std::max(corridor, 50.0) : corridor);
	}
	for (auto *launcher : simulation::Events.launchers())
	{
		if (launcher == nullptr || launcher->from_template())
			continue;
		infra::binding binding;
		binding.type = infra::kind::launcher;
		binding.node = launcher;
		binding.group = infra::category::logic;
		add(binding, "nearby", corridor);
	}
	std::stable_sort(state.candidates.begin(), state.candidates.end(), [](infra_candidate const &A, infra_candidate const &B) { return A.binding.group < B.binding.group; });
	state.status = format("%d object(s) found along %d path(s)", static_cast<int>(state.candidates.size()), static_cast<int>(state.tracks.size()));
}

void editor_mode::infra_bind_chosen()
{
	auto &state{m_infra};
	auto count{0};
	Live.refresh();
	for (auto &candidate : state.candidates)
	{
		if (false == candidate.chosen || candidate.bound || false == alive(candidate.binding))
			continue;
		if (auto *existing{infra_find(candidate.binding)})
			*existing = candidate.binding;
		else
			m_bindings.push_back(candidate.binding);
		candidate.bound = true;
		candidate.chosen = false;
		++count;
	}
	state.status = format("%d object(s) bound, they follow the changes of the track. The bindings are kept in the scenery files on save", count);
}

void editor_mode::infra_unbind(std::vector<std::size_t> Indices)
{
	std::sort(Indices.begin(), Indices.end());
	Indices.erase(std::unique(Indices.begin(), Indices.end()), Indices.end());
	for (auto index{Indices.rbegin()}; index != Indices.rend(); ++index)
	{
		if (*index >= m_bindings.size())
			continue;
		for (auto &candidate : m_infra.candidates)
			if (candidate.binding.same(m_bindings[*index]))
				candidate.bound = false;
		m_bindings.erase(m_bindings.begin() + *index);
	}
}

infra::object_state editor_mode::infra_state_of(infra::binding const &Binding) const
{
	infra::object_state result;
	result.saved = Binding;
	switch (Binding.type)
	{
	case infra::kind::model:
	{
		auto const *model{static_cast<TAnimModel const *>(Binding.node)};
		result.location = model->location();
		result.angles = model->Angles();
		break;
	}
	case infra::kind::include: result.directive = scene::Layers.directive(Binding.include); break;
	case infra::kind::traction:
	{
		auto const *traction{static_cast<TTraction const *>(Binding.node)};
		result.points = {traction->pPoint1, traction->pPoint2, traction->pPoint3, traction->pPoint4};
		break;
	}
	case infra::kind::memcell:
	case infra::kind::launcher: result.location = Binding.node->location(); break;
	}
	return result;
}

void editor_mode::infra_restore_state(infra::object_state const &State)
{
	Live.refresh();
	if (false == alive(State.saved))
		return;
	if (auto *binding{infra_find(State.saved)})
		*binding = State.saved;
	switch (State.saved.type)
	{
	case infra::kind::model: move_model(static_cast<TAnimModel *>(State.saved.node), State.location, State.angles); break;
	case infra::kind::include:
		if (false == State.directive.empty() && State.directive != scene::Layers.directive(State.saved.include))
			set_include_directive(State.saved.include, State.directive);
		break;
	case infra::kind::traction: move_traction(static_cast<TTraction *>(State.saved.node), State.points); break;
	case infra::kind::memcell: move_memcell(static_cast<TMemCell *>(State.saved.node), State.location); break;
	case infra::kind::launcher: move_launcher(static_cast<TEventLauncher *>(State.saved.node), State.location); break;
	}
}

void editor_mode::infra_buffer(infra::binding const &Binding, bool const Refresh)
{
	for (auto &entry : m_infra_buffer)
	{
		if (false == entry.saved.same(Binding))
			continue;
		if (Refresh && false == entry.moved)
			entry = infra_state_of(Binding);
		return;
	}
	if (false == Refresh && false == m_history.empty() && m_history.back().action == EditorSnapshot::Action::TrackEdit)
	{
		// a drag keeps a single undo step, which holds the state from before the drag already
		for (auto const &entry : m_history.back().infra)
			if (entry.saved.same(Binding))
				return;
	}
	m_infra_buffer.push_back(infra_state_of(Binding));
}

void editor_mode::track_captured(TTrack const &Track)
{
	if (m_infra_suspended || false == m_infra.follow)
		return;
	infra_load();
	if (m_bindings.empty())
		return;
	Live.refresh();
	for (auto &binding : m_bindings)
	{
		if (std::none_of(binding.anchors.begin(), binding.anchors.end(), [&](infra::anchor const &Anchor) { return Anchor.at.track == &Track; }) || false == alive(binding))
			continue;
		// an object moved by hand since it was bound is measured again, as long as the path is the way it was measured
		std::vector<glm::dvec3> points;
		std::optional<double> yaw;
		if (object_points(binding, points, yaw) && points.size() == binding.anchors.size())
		{
			for (std::size_t i = 0; i < points.size(); ++i)
			{
				auto &anchor{binding.anchors[i]};
				if (anchor.at.track != &Track || anchor.at.path >= static_cast<int>(Track.m_paths.size()))
					continue;
				auto const &path{Track.m_paths[anchor.at.path]};
				if (glm::distance(path.points[segment_data::point::start], anchor.start) > 1e-4 || glm::distance(path.points[segment_data::point::end], anchor.end) > 1e-4)
					continue;
				auto placed{anchor};
				if (glm::distance(infra::place(placed), points[i]) <= 0.01)
					continue;
				double s;
				bool interior;
				infra::project(path, points[i], s, interior);
				anchor = infra::make_anchor({anchor.at.track, anchor.at.path, s}, points[i]);
			}
			if (binding.turns && yaw.has_value() && binding.anchors.front().at.track == &Track)
				binding.yaw = turn_of(binding, *yaw);
		}
		infra_buffer(binding, true);
	}
}

void editor_mode::tracks_committed(std::vector<TTrack *> const &Tracks)
{
	if (m_infra_suspended || false == m_infra.follow)
		return;
	infra_load();
	if (m_bindings.empty())
		return;
	Live.refresh();
	std::vector<track_area> areas;
	for (auto *track : Tracks)
		if (false == track->m_editorremoved)
			areas.push_back(area_of(*track));
	// paths which took over may have been committed before the old ones were retired
	std::optional<std::vector<track_area>> everywhere;
	auto const take_over = [&](glm::dvec3 const &Foot, infra::station &Station) {
		if (nearest_beside(areas, Foot, 30.0, Station))
			return true;
		if (false == everywhere.has_value())
		{
			everywhere.emplace();
			for (auto *track : simulation::Paths.sequence())
				if (track != nullptr && false == track->m_editorremoved && editor_track::is_supported(*track))
					everywhere->push_back(area_of(*track));
		}
		// only a path running where the old one did, not the one alongside
		return nearest_beside(*everywhere, Foot, 1.0, Station);
	};
	for (auto &binding : m_bindings)
	{
		auto const affected{std::any_of(binding.anchors.begin(), binding.anchors.end(), [&](infra::anchor const &Anchor) {
			return Anchor.at.track != nullptr && (Anchor.at.track->m_editorremoved || contains(Tracks, Anchor.at.track));
		})};
		if (false == affected || false == alive(binding))
			continue;
		auto moved{binding};
		auto lost{false};
		for (auto &anchor : moved.anchors)
		{
			auto *track{anchor.at.track};
			if (false == track->m_editorremoved && false == contains(Tracks, track))
				continue;
			// the object goes square to the new axis from the point of the old one, so a shift of the track carries it
			// sideways rather than along; its own path goes first, then the ones which took over
			infra::station station;
			auto found{false};
			if (false == track->m_editorremoved && anchor.at.path < static_cast<int>(track->m_paths.size()))
			{
				double s;
				bool interior;
				auto const distance{infra::project(track->m_paths[anchor.at.path], anchor.foot, s, interior)};
				station = {track, anchor.at.path, s};
				found = (distance <= 30.0 && beside(station, anchor.foot, interior));
			}
			if (false == found && false == take_over(anchor.foot, station))
			{
				lost = true;
				break;
			}
			auto const offset{anchor.offset};
			auto const height{anchor.height};
			anchor = infra::make_anchor(station, anchor.foot);
			anchor.offset = offset;
			anchor.height = height;
		}
		if (lost)
		{
			binding.lost = true;
			continue;
		}
		infra_buffer(binding, false);
		moved.lost = false;
		binding = std::move(moved);
		infra_move(binding);
		for (auto &entry : m_infra_buffer)
			if (entry.saved.same(binding))
				entry.moved = true;
	}
}

void editor_mode::track_retired(TTrack &)
{
	tracks_committed({});
}

void editor_mode::track_split(TTrack &Original, TTrack &Created, segment_data const &First)
{
	if (m_infra_suspended || Created.m_paths.empty())
		return;
	auto const length{infra::plan_length(First)};
	auto const &second{Created.m_paths.front()};
	for (auto &binding : m_bindings)
		for (auto &anchor : binding.anchors)
		{
			if (anchor.at.track != &Original || anchor.at.path != 0)
				continue;
			if (anchor.at.s > length)
			{
				anchor.at.track = &Created;
				anchor.at.s -= length;
				anchor.start = second.points[segment_data::point::start];
				anchor.end = second.points[segment_data::point::end];
				anchor.length = infra::plan_length(second);
			}
			else
			{
				anchor.end = First.points[segment_data::point::end];
				anchor.length = length;
			}
		}
}

void editor_mode::infra_move(infra::binding &Binding)
{
	if (Binding.anchors.empty())
		return;
	auto const location{infra::place(Binding.anchors.front())};
	auto const facing = [&]() { return clamp_circular(infra::heading(infra::frame_at(Binding.anchors.front().at).forward) + Binding.yaw); };
	switch (Binding.type)
	{
	case infra::kind::model:
	{
		auto *model{static_cast<TAnimModel *>(Binding.node)};
		auto angles{model->Angles()};
		if (Binding.turns)
			angles.y = static_cast<float>(facing());
		move_model(model, location, angles);
		break;
	}
	case infra::kind::include:
	{
		include_place place;
		if (false == include_place_of(Binding.include, place))
			return;
		for (int axis = 0; axis < 3; ++axis)
			place.values[place.position[axis]] = editor_includes::number(location[axis] - place.offset[axis]);
		if (Binding.turns && place.yaw >= 0)
			place.values[place.yaw] = editor_includes::number(facing());
		auto const directive{editor_includes::compose_directive(place.file, place.values)};
		if (directive != scene::Layers.directive(Binding.include))
			set_include_directive(Binding.include, directive);
		break;
	}
	case infra::kind::traction:
	{
		if (Binding.anchors.size() != 4)
			return;
		std::array<glm::dvec3, 4> points;
		for (std::size_t i = 0; i < 4; ++i)
			points[i] = infra::place(Binding.anchors[i]);
		move_traction(static_cast<TTraction *>(Binding.node), points);
		break;
	}
	case infra::kind::memcell: move_memcell(static_cast<TMemCell *>(Binding.node), location); break;
	case infra::kind::launcher: move_launcher(static_cast<TEventLauncher *>(Binding.node), location); break;
	}
}

void editor_mode::infra_attach(EditorSnapshot &Snapshot)
{
	auto const involved = [&](infra::object_state const &State) {
		if (State.moved)
			return true;
		for (auto const &anchor : State.saved.anchors)
		{
			if (contains(Snapshot.created, anchor.at.track))
				return true;
			for (auto const &entry : Snapshot.tracks)
				if (entry.first == anchor.at.track)
					return true;
		}
		return false;
	};
	for (auto &entry : m_infra_buffer)
		if (involved(entry))
			Snapshot.infra.push_back(std::move(entry));
	m_infra_buffer.clear();
}

void editor_mode::render_infra_window()
{
	auto &state{m_infra};
	if (false == state.open)
		return;
	ImGui::SetNextWindowSize(ImVec2(560, 620), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin("Infrastructure along the track###trackinfra", &state.open))
	{
		ImGui::End();
		return;
	}
	infra_load();
	Live.refresh();

	auto follow{state.follow};
	if (ImGui::Checkbox("Bound objects follow the changes of the track", &follow))
	{
		state.follow = follow;
		if (follow)
			infra_rebase();
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Bound objects keep their chainage along the path, the offset from its axis and the height above the rail top;\n"
		                  "models and includes turn with the path and stay upright. The move is a part of the undo step of the change.");
	ImGui::SameLine();
	ImGui::Checkbox("Show in 3D", &state.show);
	ImGui::Text("Objects bound in the scenery: %d", static_cast<int>(m_bindings.size()));
	if (false == m_bindings_unresolved.empty())
	{
		ImGui::SameLine();
		ImGui::TextColored(ImVec4(1.f, 0.8f, 0.3f, 1.f), "(%d stored binding(s) not found, kept as they are)", static_cast<int>(m_bindings_unresolved.size()));
	}

	ImGui::Separator();
	render_infra_search();
	render_infra_candidates();
	render_infra_bound();
	if (false == state.error.empty())
		ImGui::TextColored(ImVec4(1.f, 0.4f, 0.35f, 1.f), "%s", state.error.c_str());
	else if (false == state.status.empty())
		ImGui::TextWrapped("%s", state.status.c_str());
	ImGui::End();
}

// what changed while the bindings did not follow the track becomes their new starting point
void editor_mode::infra_rebase()
{
	for (auto &binding : m_bindings)
	{
		std::vector<glm::dvec3> points;
		std::optional<double> yaw;
		if (false == alive(binding) || false == object_points(binding, points, yaw) || points.size() != binding.anchors.size())
			continue;
		for (std::size_t i = 0; i < points.size(); ++i)
		{
			auto &anchor{binding.anchors[i]};
			if (anchor.at.track == nullptr || anchor.at.track->m_editorremoved || anchor.at.path >= static_cast<int>(anchor.at.track->m_paths.size()))
			{
				binding.lost = true;
				continue;
			}
			double s;
			bool interior;
			infra::project(anchor.at.track->m_paths[anchor.at.path], points[i], s, interior);
			anchor = infra::make_anchor({anchor.at.track, anchor.at.path, s}, points[i]);
		}
		if (binding.turns && yaw.has_value())
			binding.yaw = turn_of(binding, *yaw);
	}
}

void editor_mode::render_infra_search()
{
	auto &state{m_infra};
	static char const *const scopes[] = {"Selected path", "Line through the selected path", "Route of the vertical profile", "Whole scenery"};
	ImGui::SetNextItemWidth(260.f);
	ImGui::Combo("Look along", &state.scope, scopes, IM_ARRAYSIZE(scopes));
	if (state.scope == 1)
	{
		ImGui::SetNextItemWidth(120.f);
		ImGui::InputFloat("Reach each way, m", &state.reach, 100.f, 500.f, "%.0f");
		state.reach = std::clamp(state.reach, 10.f, 20000.f);
	}
	ImGui::SetNextItemWidth(120.f);
	ImGui::InputFloat("Corridor each side of the axis, m", &state.corridor, 0.5f, 2.f, "%.1f");
	state.corridor = std::clamp(state.corridor, 0.5f, 50.f);
	if (ImGui::TreeNode("Rules for names of the files"))
	{
		ImGui::TextDisabled("Parts of the names, separated by spaces; the first rule which matches decides");
		for (std::size_t i = 0; i < state.rules.size(); ++i)
		{
			auto &rule{state.rules[i]};
			char buffer[512];
			std::snprintf(buffer, sizeof(buffer), "%s", rule.keywords.c_str());
			ImGui::SetNextItemWidth(360.f);
			if (ImGui::InputText((std::string{infra::name(rule.group)} + "##rule" + std::to_string(i)).c_str(), buffer, sizeof(buffer)))
				rule.keywords = buffer;
		}
		if (ImGui::SmallButton("Default rules"))
			state.rules = infra::default_rules();
		ImGui::TextDisabled("Includes are matched by the template name and its //$e description, models by the file and node name;\n"
		                    "traction is always the catenary, memory cells and event launchers the logic");
		ImGui::TreePop();
	}
	if (ImGui::Button("Find objects"))
		infra_recognize();

}

void editor_mode::render_infra_candidates()
{
	auto &state{m_infra};
	auto &candidates{state.candidates};
	auto const choose = [&](std::optional<infra::category> const Group, bool const Chosen) {
		for (auto &candidate : candidates)
			if (false == Group.has_value() || candidate.binding.group == *Group)
				candidate.chosen = Chosen && false == candidate.bound;
	};
	state.hovered = -1;
	if (false == candidates.empty())
	{
		int chosen{0};
		for (auto const &candidate : candidates)
			chosen += candidate.chosen ? 1 : 0;
		ImGui::SameLine();
		if (ImGui::Button(format("Bind the chosen (%d)", chosen).c_str()))
			infra_bind_chosen();
		ImGui::SameLine();
		if (ImGui::SmallButton("All"))
			choose({}, true);
		ImGui::SameLine();
		if (ImGui::SmallButton("None"))
			choose({}, false);

		ImGui::BeginChild("##infracandidates", ImVec2(0.f, 260.f), true);
		for (auto const group : infra::categories())
		{
			auto const count{std::count_if(candidates.begin(), candidates.end(), [&](infra_candidate const &Candidate) { return Candidate.binding.group == group; })};
			if (count == 0)
				continue;
			ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(colour(group)));
			auto const open{ImGui::TreeNodeEx(format("%s (%d)", infra::name(group), static_cast<int>(count)).c_str(), group == infra::category::other ? 0 : ImGuiTreeNodeFlags_DefaultOpen)};
			ImGui::PopStyleColor();
			ImGui::SameLine();
			if (ImGui::SmallButton((std::string{"all##g"} + infra::name(group)).c_str()))
				choose(group, true);
			ImGui::SameLine();
			if (ImGui::SmallButton((std::string{"none##g"} + infra::name(group)).c_str()))
				choose(group, false);
			if (false == open)
				continue;
			// the candidates are sorted by the category
			auto const first{static_cast<int>(std::distance(candidates.begin(), std::find_if(candidates.begin(), candidates.end(), [&](infra_candidate const &Candidate) { return Candidate.binding.group == group; })))};
			ImGuiListClipper clipper(static_cast<int>(count));
			while (clipper.Step())
			for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
			{
				auto const i{static_cast<std::size_t>(first + row)};
				auto &candidate{candidates[i]};
				ImGui::PushID(static_cast<int>(i));
				if (candidate.bound)
				{
					ImGui::TextDisabled("[bound]");
				}
				else
				{
					ImGui::Checkbox("##chosen", &candidate.chosen);
				}
				ImGui::SameLine();
				ImGui::Text("%s", candidate.binding.label.c_str());
				if (ImGui::IsItemHovered())
				{
					state.hovered = static_cast<int>(i);
					ImGui::SetTooltip("%s, %s\n%.2f m %s of the axis, %.2f m above the rail top\n%s", infra::name(candidate.binding.type), candidate.binding.anchors.front().at.track->name().c_str(),
					                  std::abs(candidate.offset), candidate.offset >= 0.0 ? "right" : "left", candidate.binding.anchors.front().height, candidate.reason.c_str());
				}
				ImGui::SameLine();
				ImGui::TextDisabled("%s %+.1f m", infra::name(candidate.binding.type), candidate.offset);
				ImGui::PopID();
			}
			ImGui::TreePop();
		}
		ImGui::EndChild();
	}
}

void editor_mode::render_infra_bound()
{
	if (ImGui::TreeNode("Bound objects along the scope"))
	{
		auto const scope{infra_scope()};
		std::unordered_set<TTrack const *> const tracks(scope.begin(), scope.end());
		std::vector<std::size_t> along;
		for (std::size_t i = 0; i < m_bindings.size(); ++i)
			if (std::any_of(m_bindings[i].anchors.begin(), m_bindings[i].anchors.end(), [&](infra::anchor const &Anchor) { return tracks.count(Anchor.at.track) > 0; }))
				along.push_back(i);
		ImGui::Text("%d of %d", static_cast<int>(along.size()), static_cast<int>(m_bindings.size()));
		if (false == along.empty())
		{
			ImGui::SameLine();
			if (ImGui::SmallButton("Unbind all of these"))
			{
				infra_unbind(along);
				along.clear();
			}
		}
		std::optional<std::size_t> unbind;
		ImGui::BeginChild("##infrabound", ImVec2(0.f, 160.f), true);
		ImGuiListClipper clipper(static_cast<int>(along.size()));
		while (clipper.Step())
		for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
		{
			auto const index{along[row]};
			auto const &binding{m_bindings[index]};
			ImGui::PushID(static_cast<int>(index));
			if (ImGui::SmallButton("unbind"))
				unbind = index;
			ImGui::SameLine();
			ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(binding.lost ? IM_COL32(255, 80, 70, 255) : colour(binding.group)));
			ImGui::Text("%s%s", binding.label.c_str(), binding.lost ? "  (no path to follow)" : "");
			ImGui::PopStyleColor();
			ImGui::PopID();
		}
		ImGui::EndChild();
		if (unbind)
			infra_unbind({*unbind});
		ImGui::TreePop();
	}
}

void editor_mode::draw_infra_overlay() const
{
	auto const &state{m_infra};
	if (false == state.open || false == state.show)
		return;
	Live.refresh();
	screen_projection const projection;
	ImDrawList *drawlist{ImGui::GetBackgroundDrawList()};
	auto const camera{GfxRenderer->Camera_Position()};
	auto const draw = [&](infra::binding const &Binding, ImU32 const Colour, float const Size) {
		if (Binding.anchors.empty() || glm::distance(Binding.anchors.front().foot, camera) > 2000.0)
			return;
		std::vector<glm::dvec3> points;
		std::optional<double> yaw;
		if (false == alive(Binding) || false == object_points(Binding, points, yaw))
			return;
		for (std::size_t i = 0; i < points.size() && i < Binding.anchors.size(); ++i)
		{
			auto anchor{Binding.anchors[i]};
			auto const foot{infra::frame_at(anchor.at).point};
			projection.line(drawlist, foot, points[i], Colour, 1.5f);
		}
		ImVec2 screen;
		if (projection.project(points.front(), screen))
			drawlist->AddCircleFilled(screen, Size, Colour);
	};
	for (auto const &binding : m_bindings)
		draw(binding, binding.lost ? overlay_color::invalid : IM_COL32(90, 230, 110, 220), 3.5f);
	for (int i = 0; i < static_cast<int>(state.candidates.size()); ++i)
	{
		auto const &candidate{state.candidates[i]};
		if (candidate.bound)
			continue;
		draw(candidate.binding, colour(candidate.binding.group, candidate.chosen ? 255 : 110), i == state.hovered ? 9.f : (candidate.chosen ? 5.f : 3.f));
	}
}

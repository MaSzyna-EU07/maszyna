/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/Sweep.h"
#include "editor/editorGauge.hpp"

#include "model/MdlMngr.h"
#include "model/Model3d.h"
#include "rendering/renderer.h"
#include "scene/scene.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/parser.h"
#include "vehicle/DynObj.h"
#include "world/Track.h"

#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>

namespace
{

double const sample_spacing{0.5}; // m along the curve between the samples
std::size_t const vertex_limit{4000000}; // copies beyond this many vertices are left out
double const slice_width{1.0}; // m of the model, the faces of a bent copy are cut into
double const edge_band{0.3}; // m from the side facing the curve, where the top of the edge is looked for
double const outer_reach{30.0};
double const overhang_band{0.15};
double const overhang_depth{0.05};

struct cubic
{
	glm::dvec3 p0, p1, p2, p3;

	explicit cubic(segment_data const &Piece)
	{
		p0 = Piece.points[segment_data::point::start];
		p3 = Piece.points[segment_data::point::end];
		auto const &control1{Piece.points[segment_data::point::control1]};
		auto const &control2{Piece.points[segment_data::point::control2]};
		if (control1 == glm::dvec3{} && control2 == glm::dvec3{})
		{
			p1 = glm::mix(p0, p3, 1.0 / 3.0);
			p2 = glm::mix(p0, p3, 2.0 / 3.0);
		}
		else
		{
			p1 = p0 + control1;
			p2 = p3 + control2;
		}
	}
	glm::dvec3 point(double const T) const
	{
		auto const u{1.0 - T};
		return u * u * u * p0 + 3.0 * u * u * T * p1 + 3.0 * u * T * T * p2 + T * T * T * p3;
	}
	glm::dvec3 first(double const T) const
	{
		auto const u{1.0 - T};
		return 3.0 * u * u * (p1 - p0) + 6.0 * u * T * (p2 - p1) + 3.0 * T * T * (p3 - p2);
	}
};

double plan_distance(glm::dvec3 const &A, glm::dvec3 const &B)
{
	return glm::length(glm::dvec2{B.x - A.x, B.z - A.z});
}

// the gauge changes from 26 m ahead of a curve or of a cant ramp (PKP PLK ST-T2), so the tracks beyond the ends are looked at this far
double const lead_reach{26.0};

segment_data turned_piece(segment_data const &Path)
{
	segment_data result{Path};
	result.points[segment_data::point::start] = Path.points[segment_data::point::end];
	result.points[segment_data::point::end] = Path.points[segment_data::point::start];
	result.points[segment_data::point::control1] = Path.points[segment_data::point::control2];
	result.points[segment_data::point::control2] = Path.points[segment_data::point::control1];
	result.rolls = {-Path.rolls[1], -Path.rolls[0]};
	return result;
}

bool same_points(segment_data const &A, segment_data const &B)
{
	for (int i = 0; i < 4; ++i)
		if (glm::distance(A.points[i], B.points[i]) > 1e-3)
			return false;
	return true;
}

glm::dvec3 plan_heading(segment_data const &Piece, double const T)
{
	auto const direction{cubic{Piece}.first(T)};
	glm::dvec3 const plan{direction.x, 0.0, direction.z};
	return glm::length(plan) > 1e-9 ? glm::normalize(plan) : glm::dvec3{0.0, 0.0, 1.0};
}

// paths going on from the end of the last of the pieces, each turned to lead away from it, up to the lead reach
std::vector<segment_data> paths_beyond(segment_data Last, std::vector<segment_data> const &Own)
{
	std::vector<segment_data> result;
	double length{0.0};
	while (length < lead_reach && result.size() < 16)
	{
		auto const at{Last.points[segment_data::point::end]};
		if (simulation::Region == nullptr || false == simulation::Region->point_inside(at))
			break;
		auto const &paths{simulation::Region->section(at).cell(at).m_directories.paths};
		std::optional<segment_data> next;
		for (auto *track : paths)
		{
			if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 1) == 0)
				continue;
			for (auto const &path : track->m_paths)
				for (auto const forward : {true, false})
				{
					auto const candidate{forward ? path : turned_piece(path)};
					if (glm::distance(candidate.points[segment_data::point::start], at) > 0.01 || glm::dot(plan_heading(Last, 1.0), plan_heading(candidate, 0.0)) < 0.9)
						continue;
					auto const known = [&](segment_data const &Piece) { return same_points(Piece, candidate) || same_points(turned_piece(Piece), candidate); };
					if (std::any_of(Own.begin(), Own.end(), known) || std::any_of(result.begin(), result.end(), known))
						continue;
					auto const straight{glm::dot(plan_heading(candidate, 0.0), plan_heading(candidate, 1.0))};
					if (next.has_value() && straight <= glm::dot(plan_heading(*next, 0.0), plan_heading(*next, 1.0)))
						continue;
					next = candidate;
				}
		}
		if (false == next.has_value())
			break;
		cubic const curve{*next};
		for (int k = 1; k <= 8; ++k)
			length += plan_distance(curve.point((k - 1) / 8.0), curve.point(k / 8.0));
		result.push_back(*next);
		Last = *next;
	}
	return result;
}

// triangles of the model which share the look, in the space of the model
struct model_part
{
	material_handle material{null_handle};
	lighting_data lighting;
	bool translucent{false};
	std::vector<world_vertex> vertices;
};

// without the parts lit at night (the glow, the pool of light on the ground), which the instances show only with the lights on
void gather_parts(TSubModel *Submodel, glm::dmat4 const &Transform, material_data const &Skins, std::array<TSubModel *, 8> const &Lit, std::vector<model_part> &Parts)
{
	for (auto *submodel = Submodel; submodel != nullptr; submodel = submodel->Next)
	{
		if (std::find(Lit.begin(), Lit.end(), submodel) != Lit.end())
			continue;
		auto transform{Transform};
		if ((submodel->iFlags & 0xC000) != 0 && submodel->GetMatrix() != nullptr)
		{
			transform = Transform * glm::dmat4(glm::make_mat4(submodel->GetMatrix()->readArray()));
		}
		// the nearest level of detail only, the others would be drawn over it
		if (submodel->eType == GL_TRIANGLES && submodel->fSquareMinDist <= 0.f && (submodel->m_geometry.handle.bank != 0 || submodel->m_geometry.handle.chunk != 0))
		{
			auto material{submodel->m_material};
			if (material < 0)
			{
				material = Skins.replacable_skins[std::clamp(-material, 1, 4)];
			}
			model_part part;
			part.material = material;
			part.lighting.ambient = submodel->f4Ambient;
			part.lighting.diffuse = submodel->f4Diffuse;
			part.lighting.specular = submodel->f4Specular;
			part.translucent = material != null_handle && GfxRenderer->Material(material)->get_or_guess_opacity() == 0.0f;
			auto const normals{glm::transpose(glm::inverse(glm::dmat3(transform)))};
			auto const &vertices{GfxRenderer->Vertices(submodel->m_geometry.handle)};
			auto const &indices{GfxRenderer->Indices(submodel->m_geometry.handle)};
			auto const take = [&](gfx::basic_vertex const &Vertex) {
				auto vertex{Vertex.to_world()};
				vertex.position = glm::dvec3(transform * glm::dvec4(vertex.position, 1.0));
				auto const normal{normals * glm::dvec3(vertex.normal)};
				vertex.normal = glm::length(normal) > 1e-9 ? glm::vec3(glm::normalize(normal)) : vertex.normal;
				part.vertices.push_back(vertex);
			};
			if (false == indices.empty())
			{
				for (std::size_t i = 0; i + 2 < indices.size(); i += 3)
				{
					take(vertices[indices[i]]);
					take(vertices[indices[i + 1]]);
					take(vertices[indices[i + 2]]);
				}
			}
			else
			{
				for (std::size_t i = 0; i + 2 < vertices.size(); i += 3)
				{
					take(vertices[i]);
					take(vertices[i + 1]);
					take(vertices[i + 2]);
				}
			}
			if (false == part.vertices.empty())
			{
				Parts.push_back(std::move(part));
			}
		}
		if (submodel->Child != nullptr)
		{
			gather_parts(submodel->Child, transform, Skins, Lit, Parts);
		}
	}
}

world_vertex blend(world_vertex const &A, world_vertex const &B, double const T)
{
	world_vertex result;
	result.position = glm::mix(A.position, B.position, T);
	auto const normal{glm::mix(A.normal, B.normal, static_cast<float>(T))};
	result.normal = glm::length(normal) > 1e-6f ? glm::normalize(normal) : A.normal;
	result.texture = glm::mix(A.texture, B.texture, static_cast<float>(T));
	return result;
}

// the part of the polygon on one side of the plane Along = Cut: Side 1 keeps what's beyond it, -1 what's before it
std::vector<world_vertex> clip(std::vector<world_vertex> const &Polygon, std::function<double(glm::dvec3 const &)> const &Along, double const Cut, double const Side)
{
	std::vector<world_vertex> result;
	for (std::size_t i = 0; i < Polygon.size(); ++i)
	{
		auto const &current{Polygon[i]};
		auto const &next{Polygon[(i + 1) % Polygon.size()]};
		auto const a{Side * (Along(current.position) - Cut)};
		auto const b{Side * (Along(next.position) - Cut)};
		if (a >= 0.0)
			result.push_back(current);
		if ((a >= 0.0) != (b >= 0.0))
			result.push_back(blend(current, next, a / (a - b)));
	}
	return result;
}

// triangles cut across the axis of the model into slices, so a long face bends with the curve instead of spanning it straight
std::vector<world_vertex> slices(std::vector<world_vertex> const &Vertices, std::function<double(glm::dvec3 const &)> const &Along, double const Low, double const Width)
{
	std::vector<world_vertex> result;
	result.reserve(Vertices.size());
	for (std::size_t i = 0; i + 2 < Vertices.size(); i += 3)
	{
		auto const a0{Along(Vertices[i].position)};
		auto const a1{Along(Vertices[i + 1].position)};
		auto const a2{Along(Vertices[i + 2].position)};
		auto const first{static_cast<int>(std::floor((std::min({a0, a1, a2}) - Low) / Width))};
		auto const last{static_cast<int>(std::ceil((std::max({a0, a1, a2}) - Low) / Width)) - 1};
		if (last <= first)
		{
			result.insert(result.end(), Vertices.begin() + i, Vertices.begin() + i + 3);
			continue;
		}
		std::vector<world_vertex> const triangle{Vertices[i], Vertices[i + 1], Vertices[i + 2]};
		for (int slice = first; slice <= last; ++slice)
		{
			auto const polygon{clip(clip(triangle, Along, Low + slice * Width, 1.0), Along, Low + (slice + 1) * Width, -1.0)};
			for (std::size_t k = 1; k + 1 < polygon.size(); ++k)
			{
				result.push_back(polygon[0]);
				result.push_back(polygon[k]);
				result.push_back(polygon[k + 1]);
			}
		}
	}
	return result;
}

struct laid_model
{
	std::vector<model_part> parts;
	glm::dvec3 origin{0.0};
	glm::dvec3 low{std::numeric_limits<double>::max()};
	glm::dvec3 high{-std::numeric_limits<double>::max()};
	bool bent{false};
};

struct layout
{
	std::vector<laid_model> models;
	glm::dvec3 low{std::numeric_limits<double>::max()};
	glm::dvec3 high{-std::numeric_limits<double>::max()};
	glm::dvec3 shift{0.0};
	bool edge_far{false};
};

bool lay_out(sweep_node const &Sweep, layout &Layout)
{
	auto const &definition{Sweep.definition()};
	auto const scaling{glm::scale(glm::dmat4(1.0), definition.scale)};
	for (auto const &entry : Sweep.items())
	{
		laid_model model;
		auto const transform{scaling * entry.transform};
		gather_parts(entry.model->Root, transform, entry.skins, entry.model->instance_parts().lights_on, model.parts);
		model.origin = glm::dvec3(transform[3]);
		if (false == model.parts.empty())
			Layout.models.push_back(std::move(model));
	}
	if (Layout.models.empty())
		return false;
	auto &low{Layout.low};
	auto &high{Layout.high};
	for (auto &model : Layout.models)
	{
		for (auto const &part : model.parts)
			for (auto const &vertex : part.vertices)
			{
				auto const local{Sweep.axes(vertex.position)};
				model.low = glm::min(model.low, local);
				model.high = glm::max(model.high, local);
			}
		low = glm::min(low, model.low);
		high = glm::max(high, model.high);
	}
	for (auto &model : Layout.models)
		model.bent = definition.bend && (Layout.models.size() == 1 || model.high.x - model.low.x >= 0.5 * (high.x - low.x));
	if (std::any_of(Layout.models.begin(), Layout.models.end(), [](laid_model const &Model) { return Model.bent; }))
	{
		low = glm::dvec3{std::numeric_limits<double>::max()};
		high = glm::dvec3{-std::numeric_limits<double>::max()};
		for (auto const &model : Layout.models)
			if (model.bent)
			{
				low = glm::min(low, model.low);
				high = glm::max(high, model.high);
			}
	}
	auto const nearside{definition.lateral >= 0.0 ? low.y : high.y};
	auto const farside{definition.lateral >= 0.0 ? high.y : low.y};
	double edgetop{-std::numeric_limits<double>::max()};
	double nearbottom{std::numeric_limits<double>::max()}, farbottom{std::numeric_limits<double>::max()};
	for (auto const &model : Layout.models)
		if (model.bent || false == definition.bend)
			for (auto const &part : model.parts)
				for (auto const &vertex : part.vertices)
				{
					auto const local{Sweep.axes(vertex.position)};
					if (std::abs(local.y - nearside) <= edge_band)
						edgetop = std::max(edgetop, local.z);
					if (std::abs(local.y - nearside) <= overhang_band)
						nearbottom = std::min(nearbottom, local.z);
					if (std::abs(local.y - farside) <= overhang_band)
						farbottom = std::min(farbottom, local.z);
				}
	if (edgetop == -std::numeric_limits<double>::max())
		edgetop = high.z;
	Layout.edge_far = farbottom > nearbottom + overhang_depth;
	auto const point{Sweep.axes(definition.scale * definition.point)};
	double const sides[]{0.0, -nearside, -farside, -0.5 * (low.y + high.y), -point.y};
	double const heights[]{0.0, -low.z, -high.z, -edgetop, -point.z};
	Layout.shift = glm::dvec3{0.0, sides[std::clamp(definition.side_anchor, 0, 4)], heights[std::clamp(definition.height_anchor, 0, 4)]};
	return true;
}

} // namespace

// height of the head of the rail on the side of the offset over the points of the track, less the height of the rail; 0 for no side
double sweep_node::rail_head(double const Roll, double const Lateral)
{
	auto const roll{glm::radians(Roll)};
	auto const raise{Global.bRollFix ? 0.75 * std::abs(std::sin(roll)) : 0.0};
	if (Lateral == 0.0)
		return raise;
	return raise - (Lateral > 0.0 ? 1.0 : -1.0) * 0.75 * std::sin(roll);
}

bool sweep_node::state::operator==(state const &Other) const
{
	if (model != Other.model || skin != Other.skin || bend != Other.bend || step != Other.step || from != Other.from || to != Other.to || lateral != Other.lateral || height != Other.height || along_x != Other.along_x ||
	    flip != Other.flip || mirror != Other.mirror || tilt != Other.tilt || face != Other.face || widen != Other.widen || platform != Other.platform || parameters != Other.parameters || side_anchor != Other.side_anchor || height_anchor != Other.height_anchor || point != Other.point || scale != Other.scale || pieces.size() != Other.pieces.size())
	{
		return false;
	}
	for (std::size_t i = 0; i < pieces.size(); ++i)
	{
		if (pieces[i].points != Other.pieces[i].points || pieces[i].rolls != Other.pieces[i].rolls)
		{
			return false;
		}
	}
	return true;
}

sweep_node::sweep_node(scene::node_data const &Nodedata) : basic_node(Nodedata) {}

void sweep_node::import(cParser &Input, glm::dvec3 const &Offset)
{
	m_state = state{};
	m_state.model = Input.getToken<std::string>();
	m_state.skin = Input.getToken<std::string>();
	auto token{Input.getToken<std::string>()};
	while (false == token.empty() && token != "endsweep")
	{
		if (token == "mode")
		{
			m_state.bend = Input.getToken<std::string>() != "repeat";
		}
		else if (token == "step")
		{
			Input.getTokens();
			Input >> m_state.step;
		}
		else if (token == "range")
		{
			Input.getTokens(2);
			Input >> m_state.from >> m_state.to;
		}
		else if (token == "offset")
		{
			Input.getTokens(2);
			Input >> m_state.lateral >> m_state.height;
		}
		else if (token == "axis")
		{
			m_state.along_x = Input.getToken<std::string>() == "x";
		}
		else if (token == "flip")
		{
			m_state.flip = true;
		}
		else if (token == "mirror")
		{
			m_state.mirror = true;
		}
		else if (token == "tilt")
		{
			m_state.tilt = true;
		}
		else if (token == "widen")
		{
			m_state.widen = true;
		}
		else if (token == "platform")
		{
			m_state.platform = true;
		}
		else if (token == "noface")
		{
			m_state.face = false;
		}
		else if (token == "parameters")
		{
			int count{0};
			Input.getTokens();
			Input >> count;
			m_state.parameters.clear();
			for (int i = 0; i < count; ++i)
				m_state.parameters.push_back(Input.getToken<std::string>(false));
		}
		else if (token == "anchor")
		{
			auto const side{Input.getToken<std::string>()};
			auto const height{Input.getToken<std::string>()};
			m_state.side_anchor = side == "near" ? 1 : side == "far" ? 2 : side == "centre" ? 3 : side == "point" ? 4 : 0;
			m_state.height_anchor = height == "bottom" ? 1 : height == "top" ? 2 : height == "edge" ? 3 : height == "point" ? 4 : 0;
		}
		else if (token == "point")
		{
			Input.getTokens(3);
			Input >> m_state.point.x >> m_state.point.y >> m_state.point.z;
		}
		else if (token == "scale")
		{
			Input.getTokens(3);
			Input >> m_state.scale.x >> m_state.scale.y >> m_state.scale.z;
		}
		else if (token == "piece")
		{
			segment_data piece;
			piece.deserialize(Input, Offset);
			m_state.pieces.push_back(piece);
		}
		else
		{
			ErrorLog("Bad sweep: unknown property \"" + token + "\" of \"" + m_name + "\" in file \"" + Input.Name() + "\"");
		}
		token = Input.getToken<std::string>();
	}
	rebuild_samples();
}

void sweep_node::define(state const &State)
{
	auto const shown{m_shapes.shown()};
	if (shown)
	{
		hide();
	}
	m_state = State;
	rebuild_samples();
	if (shown)
	{
		show();
	}
}

void sweep_node::sample_piece(segment_data const &Piece, std::vector<sample> &Samples, double &Station)
{
	cubic const curve{Piece};
	auto const count{std::clamp(static_cast<int>(plan_distance(curve.p0, curve.p3) / sample_spacing), 4, 4000)};
	for (int k = Samples.empty() ? 0 : 1; k <= count; ++k)
	{
		auto const t{static_cast<double>(k) / count};
		auto const position{curve.point(t)};
		if (false == Samples.empty())
		{
			Station += plan_distance(Samples.back().position, position);
		}
		auto tangent{curve.first(t)};
		tangent = glm::length(tangent) > 1e-9 ? glm::normalize(tangent) : glm::dvec3{0.0, 0.0, 1.0};
		Samples.push_back({Station, position, tangent, Piece.rolls[0] + (Piece.rolls[1] - Piece.rolls[0]) * t, {}, {}});
	}
}

// the structure gauge widens ahead of the curves and of the cant, the same way the gauge check of the editor has it
void sweep_node::gauge_along(std::vector<sample> &Samples)
{
	if (Samples.size() < 3)
		return;
	std::vector<double> chainage, curvature, cant;
	for (std::size_t i = 0; i < Samples.size(); ++i)
	{
		auto const &before{Samples[i == 0 ? 0 : i - 1]};
		auto const &after{Samples[std::min(i + 1, Samples.size() - 1)]};
		glm::dvec2 const a{before.tangent.x, before.tangent.z};
		glm::dvec2 const b{after.tangent.x, after.tangent.z};
		auto const span{after.station - before.station};
		auto const turn{glm::length(a) > 1e-9 && glm::length(b) > 1e-9 ? std::atan2(a.x * b.y - a.y * b.x, glm::dot(a, b)) : 0.0};
		chainage.push_back(Samples[i].station);
		curvature.push_back(span > 1e-9 ? turn / span : 0.0);
		cant.push_back(1.5 * std::abs(std::sin(glm::radians(Samples[i].roll))));
	}
	auto const sections{gauge::sections(gauge::kind::unified, chainage, curvature, cant)};
	for (std::size_t i = 0; i < Samples.size() && i < sections.size(); ++i)
	{
		Samples[i].widening = sections[i].lower;
		Samples[i].cant = sections[i].cant;
	}
}

void sweep_node::beside_switches(std::vector<sample> &Samples)
{
	if (Samples.size() < 3)
		return;
	auto const side{m_state.lateral >= 0.0 ? 1 : 0};
	auto const sign{m_state.lateral >= 0.0 ? 1.0 : -1.0};
	auto const edgeheight{std::max(0.0, m_state.height - gauge::rail_height)};
	auto const total = [&](sample const &Sample) { return Sample.widening[side] + std::max(0.0, Sample.cant[side]) * edgeheight / 1.5; };
	auto const plan = [](glm::dvec3 const &Vector) {
		glm::dvec3 const flat{Vector.x, 0.0, Vector.z};
		return glm::length(flat) > 1e-9 ? glm::normalize(flat) : glm::dvec3{0.0, 0.0, 1.0};
	};
	glm::dvec3 low{std::numeric_limits<double>::max()}, high{-std::numeric_limits<double>::max()};
	for (auto const &sample : Samples)
	{
		low = glm::min(low, sample.position);
		high = glm::max(high, sample.position);
	}
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->m_paths.size() < 2)
			continue;
		auto const points{track->m_paths.front().points[segment_data::point::start]};
		if (points.x < low.x - 1.0 || points.x > high.x + 1.0 || points.z < low.z - 1.0 || points.z > high.z + 1.0)
			continue;
		std::size_t at{0};
		auto best{std::numeric_limits<double>::max()};
		for (std::size_t i = 0; i < Samples.size(); ++i)
			if (auto const distance{plan_distance(Samples[i].position, points)}; distance < best)
			{
				best = distance;
				at = i;
			}
		if (best > sample_spacing)
			continue;
		auto const &near{Samples[at]};
		auto const along{plan(near.tangent)};
		glm::dvec3 const toward{points.x - near.position.x, 0.0, points.z - near.position.z};
		if (std::abs(glm::dot(toward, glm::dvec3{-along.z, 0.0, along.x})) > 0.05)
			continue;
		auto const s0{near.station + glm::dot(toward, along)};
		for (auto const &path : track->m_paths)
		{
			auto const heading{glm::dot(plan_heading(path, 0.0), along)};
			if (std::abs(heading) < 0.9)
				continue;
			auto const beyond{paths_beyond(path, {path})};
			std::vector<sample> chain;
			double station{0.0};
			if (heading > 0.0)
			{
				for (auto const &sample : Samples)
					if (sample.station >= s0 - outer_reach && sample.station < s0 - 1e-3)
					{
						station = sample.station;
						chain.push_back(sample);
					}
				sample_piece(path, chain, station);
				for (auto const &piece : beyond)
					sample_piece(piece, chain, station);
			}
			else
			{
				for (auto piece = beyond.rbegin(); piece != beyond.rend(); ++piece)
					sample_piece(turned_piece(*piece), chain, station);
				sample_piece(turned_piece(path), chain, station);
				auto const shift{station - s0};
				for (auto const &sample : Samples)
					if (sample.station > s0 + 1e-3 && sample.station <= s0 + outer_reach)
					{
						chain.push_back(sample);
						chain.back().station += shift;
					}
			}
			gauge_along(chain);
			for (auto &here : Samples)
			{
				if (plan_distance(here.position, points) > outer_reach + 2.0 * lead_reach + 100.0)
					continue;
				auto const forward{plan(here.tangent)};
				sample const *nearest{nullptr};
				auto closest{std::numeric_limits<double>::max()};
				for (auto const &point : chain)
				{
					auto const gap{std::abs(glm::dot(glm::dvec3{point.position.x - here.position.x, 0.0, point.position.z - here.position.z}, forward))};
					if (gap < closest)
					{
						closest = gap;
						nearest = &point;
					}
				}
				if (nearest == nullptr || closest > sample_spacing)
					continue;
				auto const offset{sign * glm::dot(glm::dvec3{nearest->position.x - here.position.x, 0.0, nearest->position.z - here.position.z}, glm::dvec3{-forward.z, 0.0, forward.x})};
				if (offset > 0.05)
					continue;
				auto const needed{offset + total(*nearest)};
				if (needed > total(here))
				{
					here.widening[side] = needed;
					here.cant[side] = 0.0;
				}
			}
		}
	}
}

bool sweep_node::bridge()
{
	auto const turned = [](segment_data const &Path) {
		segment_data result{Path};
		result.points[segment_data::point::start] = Path.points[segment_data::point::end];
		result.points[segment_data::point::end] = Path.points[segment_data::point::start];
		result.points[segment_data::point::control1] = Path.points[segment_data::point::control2];
		result.points[segment_data::point::control2] = Path.points[segment_data::point::control1];
		result.rolls = {-Path.rolls[1], -Path.rolls[0]};
		return result;
	};
	auto const same = [](segment_data const &A, segment_data const &B) {
		for (int i = 0; i < 4; ++i)
			if (glm::distance(A.points[i], B.points[i]) > 1e-3)
				return false;
		return true;
	};
	auto const length = [](segment_data const &Piece) {
		std::vector<sample> samples;
		double station{0.0};
		sample_piece(Piece, samples, station);
		return station;
	};
	auto const heading = [](segment_data const &Piece, double const T) {
		auto const direction{cubic{Piece}.first(T)};
		glm::dvec3 const plan{direction.x, 0.0, direction.z};
		return glm::length(plan) > 1e-9 ? glm::normalize(plan) : glm::dvec3{0.0, 0.0, 1.0};
	};
	auto state{m_state};
	auto changed{false};
	double station{0.0};
	for (std::size_t i = 0; i + 1 < state.pieces.size(); ++i)
	{
		station += length(state.pieces[i]);
		auto const target{state.pieces[i + 1].points[segment_data::point::start]};
		auto at{state.pieces[i].points[segment_data::point::end]};
		if (glm::distance(at, target) < 0.01)
			continue;
		auto const chord{plan_distance(at, target)};
		std::vector<segment_data> fill;
		auto previous{state.pieces[i]};
		auto reached{false};
		for (int step = 0; step < 20 && false == reached; ++step)
		{
			std::optional<segment_data> best;
			auto bestdistance{std::numeric_limits<double>::max()};
			for (auto *track : simulation::Paths.sequence())
			{
				if (track == nullptr || track->m_editorremoved)
					continue;
				for (auto const &path : track->m_paths)
					for (auto const forward : {true, false})
					{
						auto const candidate{forward ? path : turned(path)};
						if (glm::distance(candidate.points[segment_data::point::start], at) > 0.01 || same(candidate, previous) || same(turned(candidate), previous))
							continue;
						if (glm::dot(heading(previous, 1.0), heading(candidate, 0.0)) < 0.5)
							continue;
						auto const distance{glm::distance(candidate.points[segment_data::point::end], target)};
						if (distance < bestdistance)
						{
							bestdistance = distance;
							best = candidate;
						}
					}
			}
			if (false == best.has_value())
				break;
			fill.push_back(*best);
			previous = *best;
			at = best->points[segment_data::point::end];
			reached = glm::distance(at, target) < 0.01;
		}
		if (false == reached)
			continue;
		double added{0.0};
		for (auto const &piece : fill)
			added += length(piece);
		auto const shift{added - chord};
		if (state.from >= station)
			state.from += shift;
		if (state.to >= station)
			state.to += shift;
		state.pieces.insert(state.pieces.begin() + i + 1, fill.begin(), fill.end());
		station += added;
		i += fill.size();
		changed = true;
	}
	if (changed)
	{
		WriteLog("Sweep \"" + m_name + "\": the gaps between its pieces filled with the paths which join them");
		define(state);
	}
	return changed;
}

void sweep_node::refresh()
{
	auto const current{m_state};
	define(current);
}

void sweep_node::rebuild_samples()
{
	m_boundsvalid = false;
	m_samples.clear();
	std::vector<segment_data> before, after;
	if ((m_state.widen || m_state.platform) && false == m_state.pieces.empty())
	{
		before = paths_beyond(turned_piece(m_state.pieces.front()), m_state.pieces);
		after = paths_beyond(m_state.pieces.back(), m_state.pieces);
	}
	double station{0.0};
	for (auto piece = before.rbegin(); piece != before.rend(); ++piece)
		sample_piece(turned_piece(*piece), m_samples, station);
	auto const head{station};
	auto const first{m_samples.empty() ? std::size_t{0} : m_samples.size() - 1};
	for (auto const &piece : m_state.pieces)
		sample_piece(piece, m_samples, station);
	auto const last{m_samples.size()};
	for (auto const &piece : after)
		sample_piece(piece, m_samples, station);
	gauge_along(m_samples);
	if (m_state.widen || m_state.platform)
		beside_switches(m_samples);
	// the paths beyond the ends only lead the gauge in, the copies stay on the pieces
	m_samples.erase(m_samples.begin() + last, m_samples.end());
	m_samples.erase(m_samples.begin(), m_samples.begin() + first);
	for (auto &sample : m_samples)
		sample.station -= head;
	m_distances.clear();
	glm::dvec3 previous{0.0};
	for (auto const &sample : m_samples)
	{
		frame at;
		at.widening = sample.widening;
		at.cant = sample.cant;
		glm::dvec3 left{-sample.tangent.z, 0.0, sample.tangent.x};
		left = glm::length(left) > 1e-9 ? glm::normalize(left) : glm::dvec3{-1.0, 0.0, 0.0};
		auto const point{sample.position + left * (m_state.lateral + setback(at))};
		m_distances.push_back(m_distances.empty() ? 0.0 : m_distances.back() + std::max(1e-6, plan_distance(previous, point)));
		previous = point;
	}
	if (false == m_samples.empty())
	{
		location(frame_at(0.5 * length()).position);
	}
	m_area.radius = -1.f;
}

double sweep_node::length() const
{
	return m_samples.empty() ? 0.0 : m_samples.back().station;
}

double sweep_node::start() const
{
	return m_state.from;
}

double sweep_node::end() const
{
	return m_state.to < 0.0 ? length() : std::max(m_state.to, start());
}

double sweep_node::distance_at(double const Station) const
{
	if (m_samples.size() < 2)
		return Station;
	if (Station <= m_samples.front().station)
		return m_distances.front() + Station - m_samples.front().station;
	if (Station >= m_samples.back().station)
		return m_distances.back() + Station - m_samples.back().station;
	auto const after{static_cast<std::size_t>(std::upper_bound(m_samples.begin(), m_samples.end(), Station, [](double const Value, sample const &Sample) { return Value < Sample.station; }) - m_samples.begin())};
	auto const before{after - 1};
	auto const span{m_samples[after].station - m_samples[before].station};
	auto const f{span > 1e-9 ? (Station - m_samples[before].station) / span : 0.0};
	return m_distances[before] + (m_distances[after] - m_distances[before]) * f;
}

double sweep_node::station_at(double const Distance) const
{
	if (m_samples.size() < 2)
		return Distance;
	if (Distance <= m_distances.front())
		return m_samples.front().station + Distance - m_distances.front();
	if (Distance >= m_distances.back())
		return m_samples.back().station + Distance - m_distances.back();
	auto const after{static_cast<std::size_t>(std::upper_bound(m_distances.begin(), m_distances.end(), Distance) - m_distances.begin())};
	auto const before{after - 1};
	auto const span{m_distances[after] - m_distances[before]};
	auto const f{span > 1e-12 ? (Distance - m_distances[before]) / span : 0.0};
	return m_samples[before].station + (m_samples[after].station - m_samples[before].station) * f;
}

double sweep_node::setback(frame const &At) const
{
	if (false == m_state.widen && false == m_state.platform)
		return 0.0;
	auto const side{m_state.lateral >= 0.0 ? 1 : 0};
	auto const edgeheight{std::max(0.0, m_state.height - gauge::rail_height)};
	auto const widening{At.widening[side] + std::max(0.0, At.cant[side]) * edgeheight / 1.5};
	return m_state.lateral >= 0.0 ? widening : -widening;
}

double sweep_node::rise(frame const &At) const
{
	if (m_state.tilt)
		return rail_head(At.roll, 0.0);
	if (false == m_state.platform)
		return rail_head(At.roll, m_state.lateral);
	// PKP PLK ST-T2 table 3: H = Hi + D/2 -+ D (XB + dbS) / 1500 over the lower rail, with the cant D of the gauge, which
	// changes from 20 m (inner side) and 26 m (outer side) ahead of the cant ramp, as the platform has to. taken as the
	// turn of the edge with the plane of the rail heads, as the table has it only to the first order, and on the outer
	// side, where the edge stays at XB + dbS instead of XB + dbS - dbD, the plane is higher by dbD D / 1500 there
	auto const side{m_state.lateral >= 0.0 ? 1 : 0};
	auto const cant{At.cant[side]};
	auto const lower{rail_head(At.roll, 0.0) - 0.75 * std::abs(std::sin(glm::radians(At.roll)))};
	auto const edgeheight{std::max(0.0, m_state.height - gauge::rail_height)};
	auto const slope{std::clamp(cant / 1.5, -0.5, 0.5)};
	auto const level{std::sqrt(1.0 - slope * slope)};
	auto const across{std::abs(m_state.lateral) + std::abs(setback(At))};
	return lower + 0.5 * std::abs(cant) - (across - edgeheight * slope) * slope / level + edgeheight * (level - 1.0);
}

glm::dvec3 sweep_node::axes(glm::dvec3 const &Vector) const
{
	auto const &definition{m_state};
	auto const needsflip{definition.along_x ? definition.lateral < 0.0 : definition.lateral > 0.0};
	auto const flip{definition.flip != (definition.face && needsflip)};
	glm::dvec3 result{definition.along_x ? Vector.x : Vector.z, definition.along_x ? Vector.z : -Vector.x, Vector.y};
	if (flip)
	{
		result.x = -result.x;
		result.y = -result.y;
	}
	if (definition.mirror)
	{
		result.y = -result.y;
	}
	return result;
}

bool sweep_node::bounds(glm::dvec3 &Low, glm::dvec3 &High, glm::dvec3 &Shift) const
{
	if (false == m_boundsvalid)
	{
		layout laid;
		m_boundsfound = lay_out(*this, laid);
		m_low = laid.low;
		m_high = laid.high;
		m_shift = laid.shift;
		m_edgefar = laid.edge_far;
		m_boundsvalid = true;
	}
	Low = m_low;
	High = m_high;
	Shift = m_shift;
	return m_boundsfound;
}

bool sweep_node::edge_far() const
{
	glm::dvec3 low, high, shift;
	return bounds(low, high, shift) && m_edgefar;
}

sweep_node::frame sweep_node::frame_at(double const Station) const
{
	frame result;
	if (m_samples.empty())
	{
		return result;
	}
	auto const found{std::upper_bound(m_samples.begin(), m_samples.end(), Station, [](double const Value, sample const &Sample) { return Value < Sample.station; })};
	auto const after{found == m_samples.end() ? std::prev(found) : found};
	auto const before{after == m_samples.begin() ? after : std::prev(after)};
	auto const span{after->station - before->station};
	auto const f{span > 1e-9 ? std::clamp((Station - before->station) / span, 0.0, 1.0) : 0.0};
	result.position = glm::mix(before->position, after->position, f);
	// beyond the ends the curve goes on straight
	if (Station < m_samples.front().station)
	{
		result.position = m_samples.front().position + m_samples.front().tangent * (Station - m_samples.front().station);
	}
	else if (Station > m_samples.back().station)
	{
		result.position = m_samples.back().position + m_samples.back().tangent * (Station - m_samples.back().station);
	}
	auto forward{glm::mix(before->tangent, after->tangent, f)};
	forward = glm::length(forward) > 1e-9 ? glm::normalize(forward) : glm::dvec3{0.0, 0.0, 1.0};
	glm::dvec3 left{-forward.z, 0.0, forward.x};
	left = glm::length(left) > 1e-9 ? glm::normalize(left) : glm::dvec3{-1.0, 0.0, 0.0};
	glm::dvec3 up{glm::normalize(glm::cross(left, forward))};
	result.roll = before->roll + (after->roll - before->roll) * f;
	for (int side = 0; side < 2; ++side)
	{
		result.widening[side] = before->widening[side] + (after->widening[side] - before->widening[side]) * f;
		result.cant[side] = before->cant[side] + (after->cant[side] - before->cant[side]) * f;
	}
	if (m_state.tilt)
	{
		auto const roll{glm::radians(result.roll)};
		auto const turnedleft{left * std::cos(roll) - up * std::sin(roll)};
		up = up * std::cos(roll) + left * std::sin(roll);
		left = turnedleft;
	}
	result.forward = forward;
	result.left = left;
	result.up = up;
	return result;
}

// models a template is made of, with their placement in the space of the template
std::vector<sweep_node::item> sweep_node::items() const
{
	std::vector<item> result;
	auto const &definition{m_state};
	auto const ends = [](std::string const &Name, std::string const &Extension) { return Name.size() >= Extension.size() && Name.compare(Name.size() - Extension.size(), Extension.size(), Extension) == 0; };
	auto name{definition.model};
	std::transform(name.begin(), name.end(), name.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	if (false == ends(name, ".inc"))
	{
		item single;
		single.model = TModelsManager::GetModel(definition.model, false, false);
		if (false == definition.skin.empty() && definition.skin != "none")
			single.skins.assign(definition.skin);
		if (single.model != nullptr)
			result.push_back(single);
		return result;
	}
	cParser parser(definition.model, cParser::buffer_FILE, Global.asCurrentSceneryPath, Global.bLoadTraction, definition.parameters);
	std::vector<glm::dvec3> offsets{glm::dvec3{0.0}};
	glm::dvec3 rotation{0.0};
	auto token{parser.getToken<std::string>()};
	while (false == token.empty())
	{
		if (token == "origin")
		{
			glm::dvec3 offset;
			parser.getTokens(3);
			parser >> offset.x >> offset.y >> offset.z;
			offsets.push_back(offsets.back() + offset);
		}
		else if (token == "endorigin")
		{
			if (offsets.size() > 1)
				offsets.pop_back();
		}
		else if (token == "rotate")
		{
			parser.getTokens(3);
			parser >> rotation.x >> rotation.y >> rotation.z;
		}
		else if (token == "node")
		{
			double rangemax, rangemin;
			std::string nodename, type;
			parser.getTokens(4);
			parser >> rangemax >> rangemin >> nodename >> type;
			if (type == "model")
			{
				glm::dvec3 location;
				double yaw;
				parser.getTokens(4);
				parser >> location.x >> location.y >> location.z >> yaw;
				auto const model{parser.getToken<std::string>()};
				auto const skin{parser.getToken<std::string>()};
				auto rest{parser.getToken<std::string>()};
				while (false == rest.empty() && rest != "endmodel")
					rest = parser.getToken<std::string>();
				item entry;
				entry.model = TModelsManager::GetModel(model, false, false);
				if (skin != "none" && false == skin.empty() && skin.front() != '*')
					entry.skins.assign(skin);
				// placed the way the scenery places a model in an origin turned by a rotate statement
				auto const place{glm::rotateY(location, glm::radians(rotation.y)) + offsets.back()};
				glm::dmat4 transform{glm::translate(glm::dmat4(1.0), place)};
				transform = glm::rotate(transform, glm::radians(rotation.y + yaw), glm::dvec3{0.0, 1.0, 0.0});
				transform = glm::rotate(transform, glm::radians(rotation.x), glm::dvec3{1.0, 0.0, 0.0});
				transform = glm::rotate(transform, glm::radians(rotation.z), glm::dvec3{0.0, 0.0, 1.0});
				entry.transform = transform;
				if (entry.model != nullptr && rangemin >= 0.0)
					result.push_back(entry);
			}
			else
			{
				// whatever else the template defines isn't laid along the curve
				auto rest{parser.getToken<std::string>()};
				while (false == rest.empty() && rest != "end" + type && rest != "end" && rest != "endtri" && rest != "endline")
					rest = parser.getToken<std::string>();
			}
		}
		token = parser.getToken<std::string>();
	}
	return result;
}

double sweep_node::project(glm::dvec3 const &Point, double &Lateral) const
{
	double best{std::numeric_limits<double>::max()};
	double station{0.0};
	Lateral = 0.0;
	glm::dvec2 const point{Point.x, Point.z};
	for (std::size_t i = 1; i < m_samples.size(); ++i)
	{
		glm::dvec2 const a{m_samples[i - 1].position.x, m_samples[i - 1].position.z};
		glm::dvec2 const b{m_samples[i].position.x, m_samples[i].position.z};
		auto const ab{b - a};
		auto const length2{glm::dot(ab, ab)};
		if (length2 < 1e-12)
			continue;
		auto const t{std::clamp(glm::dot(point - a, ab) / length2, 0.0, 1.0)};
		auto const distance{glm::length(point - (a + ab * t))};
		if (distance < best)
		{
			best = distance;
			station = m_samples[i - 1].station + (m_samples[i].station - m_samples[i - 1].station) * t;
			auto const side{ab.x * (point - a).y - ab.y * (point - a).x};
			Lateral = side >= 0.0 ? distance : -distance;
		}
	}
	return station;
}

double sweep_node::model_length() const
{
	auto const pieces{items()};
	auto const scaling{glm::scale(glm::dmat4(1.0), m_state.scale)};
	double low{std::numeric_limits<double>::max()}, high{-std::numeric_limits<double>::max()};
	for (auto const &entry : pieces)
	{
		std::vector<model_part> parts;
		gather_parts(entry.model->Root, scaling * entry.transform, entry.skins, entry.model->instance_parts().lights_on, parts);
		for (auto const &part : parts)
			for (auto const &vertex : part.vertices)
			{
				auto const along{m_state.along_x ? vertex.position.x : vertex.position.z};
				low = std::min(low, along);
				high = std::max(high, along);
			}
	}
	return high > low ? high - low : 0.0;
}

std::vector<scene::shape_node> sweep_node::create_shapes() const
{
	std::vector<scene::shape_node> shapes;
	auto const &definition{m_state};
	if (m_samples.size() < 2)
		return shapes;
	layout laid;
	if (false == lay_out(*this, laid))
		return shapes;
	auto &pieces{laid.models};
	auto const low{laid.low};
	auto const high{laid.high};
	auto const anchor_shift{laid.shift};
	auto const own{high.x - low.x};
	auto const first{distance_at(start())};
	auto const stretch{distance_at(end()) - first};
	if (own < 1e-3 || stretch < 1e-3)
		return shapes;
	if (definition.bend)
	{
		auto const along = [&](glm::dvec3 const &Position) { return axes(Position).x; };
		for (auto &model : pieces)
			if (model.bent)
				for (auto &part : model.parts)
					part.vertices = slices(part.vertices, along, low.x, slice_width);
	}
	auto const copy_length{definition.step > 0.01 ? definition.step : own};
	// copies end to end fill the stretch exactly, each one squeezed or stretched a bit; repeated ones go at even steps
	auto const copies{definition.bend ? std::max(1, static_cast<int>(std::round(stretch / copy_length))) : static_cast<int>(std::floor(stretch / copy_length + 1e-6)) + 1};
	auto const spacing{definition.bend ? stretch / copies : copy_length};
	auto const scale{definition.bend ? spacing / own : 1.0};
	std::size_t percopy{0};
	for (auto const &model : pieces)
		for (auto const &part : model.parts)
			percopy += part.vertices.size();
	auto const limit{std::max<std::size_t>(1, vertex_limit / std::max<std::size_t>(1, percopy))};
	if (static_cast<std::size_t>(copies) > limit)
		ErrorLog("Bad sweep: \"" + m_name + "\" would make " + std::to_string(copies) + " copies of \"" + definition.model + "\", only " + std::to_string(limit) + " are made");
	auto const count{std::min<std::size_t>(copies, limit)};
	struct batch
	{
		material_handle material;
		lighting_data lighting;
		bool translucent;
		std::vector<world_vertex> vertices;
	};
	std::vector<batch> batches;
	auto const target = [&](model_part const &Part) -> std::vector<world_vertex> & {
		for (auto &entry : batches)
			if (entry.material == Part.material && entry.lighting == Part.lighting && entry.translucent == Part.translucent)
				return entry.vertices;
		batches.push_back({Part.material, Part.lighting, Part.translucent, {}});
		return batches.back().vertices;
	};
	// heights are given from the head of the rail on the side of the copies. the cant turns the track about its inner rail when the
	// simulation raises it for the cant, about the axis otherwise; a copy which leans with the cant goes up with the axis
	auto const cone = [&](frame const &At) { return rise(At); };
	auto const upright = [](frame At) {
		glm::dvec3 forward{At.forward.x, 0.0, At.forward.z};
		forward = glm::length(forward) > 1e-9 ? glm::normalize(forward) : glm::dvec3{0.0, 0.0, 1.0};
		At.forward = forward;
		At.left = glm::dvec3{-forward.z, 0.0, forward.x};
		At.up = glm::dvec3{0.0, 1.0, 0.0};
		return At;
	};
	glm::dvec3 const offset{0.0, definition.height, 0.0};
	for (std::size_t copy = 0; copy < count; ++copy)
	{
		auto const base{first + spacing * static_cast<double>(copy)};
		for (auto const &model : pieces)
		{
			// a model which doesn't bend stands upright at its point: the place of its origin along the curve
			auto const anchor{axes(model.origin) + anchor_shift};
			auto const stand{upright(frame_at(station_at(definition.bend ? base + (anchor.x - low.x) * scale : base)))};
			for (auto const &part : model.parts)
			{
				auto &vertices{target(part)};
				auto const first{vertices.size()};
				for (auto const &vertex : part.vertices)
				{
					auto const local{axes(vertex.position) + anchor_shift};
					auto const direction{axes(glm::dvec3(vertex.normal))};
					world_vertex placed{vertex};
					if (model.bent)
					{
						auto const at{frame_at(station_at(base + (local.x - low.x) * scale))};
						placed.position = at.position + at.left * (local.y + definition.lateral + setback(at)) + at.up * local.z + offset + glm::dvec3{0.0, cone(at), 0.0};
						placed.normal = glm::vec3(glm::normalize(at.forward * direction.x + at.left * direction.y + at.up * direction.z));
					}
					else
					{
						auto const along{definition.bend ? local.x - anchor.x : local.x};
						placed.position = stand.position + stand.forward * along + stand.left * (local.y + definition.lateral + setback(stand)) + stand.up * local.z + offset + glm::dvec3{0.0, cone(stand), 0.0};
						placed.normal = glm::vec3(glm::normalize(stand.forward * direction.x + stand.left * direction.y + stand.up * direction.z));
					}
					vertices.push_back(placed);
				}
				if (definition.mirror)
				{
					// a mirror image turns the triangles inside out
					for (auto i = first; i + 2 < vertices.size(); i += 3)
						std::swap(vertices[i + 1], vertices[i + 2]);
				}
			}
		}
	}
	for (auto &entry : batches)
	{
		if (entry.vertices.empty())
			continue;
		scene::shape_node shape;
		shape.make_terrain(entry.material, std::move(entry.vertices), glm::dvec3{0.0});
		shape.lighting(entry.lighting);
		shape.translucent(entry.translucent);
		shapes.emplace_back(std::move(shape));
	}
	return shapes;
}

void sweep_node::show()
{
	if (m_merged || m_shapes.shown() || m_editorremoved)
	{
		return;
	}
	m_shapes.show(create_shapes(), location());
}

void sweep_node::hide()
{
	m_shapes.hide();
}

float sweep_node::radius_()
{
	return static_cast<float>(0.5 * length() + std::abs(m_state.lateral) + 25.0);
}

void sweep_node::serialize_(std::ostream &Output) const
{
	// TODO: implement
}

void sweep_node::deserialize_(std::istream &Input)
{
	// TODO: implement
}

void sweep_node::export_as_text_(std::ostream &Output) const
{
	auto const &sweep{m_state};
	Output << "sweep " << sweep.model << ' ' << (sweep.skin.empty() ? std::string{"none"} : sweep.skin) << ' ';
	Output << "mode " << (sweep.bend ? "bend" : "repeat") << ' ';
	if (sweep.step > 0.0)
	{
		Output << "step " << sweep.step << ' ';
	}
	if (sweep.from != 0.0 || sweep.to >= 0.0)
	{
		Output << "range " << sweep.from << ' ' << sweep.to << ' ';
	}
	if (sweep.lateral != 0.0 || sweep.height != 0.0)
	{
		Output << "offset " << sweep.lateral << ' ' << sweep.height << ' ';
	}
	if (sweep.along_x)
	{
		Output << "axis x ";
	}
	if (sweep.flip)
	{
		Output << "flip ";
	}
	if (sweep.mirror)
	{
		Output << "mirror ";
	}
	if (sweep.tilt)
	{
		Output << "tilt ";
	}
	if (false == sweep.face)
	{
		Output << "noface ";
	}
	if (sweep.widen)
	{
		Output << "widen ";
	}
	if (sweep.platform)
	{
		Output << "platform ";
	}
	if (false == sweep.parameters.empty())
	{
		Output << "parameters " << sweep.parameters.size() << ' ';
		for (auto const &parameter : sweep.parameters)
			Output << (parameter.empty() ? std::string{"none"} : parameter) << ' ';
	}
	if (sweep.side_anchor != 0 || sweep.height_anchor != 0)
	{
		char const *sides[] = {"origin", "near", "far", "centre", "point"};
		char const *heights[] = {"origin", "bottom", "top", "edge", "point"};
		Output << "anchor " << sides[std::clamp(sweep.side_anchor, 0, 4)] << ' ' << heights[std::clamp(sweep.height_anchor, 0, 4)] << ' ';
	}
	if (sweep.side_anchor == 4 || sweep.height_anchor == 4)
	{
		Output << "point " << sweep.point.x << ' ' << sweep.point.y << ' ' << sweep.point.z << ' ';
	}
	if (sweep.scale != glm::dvec3{1.0})
	{
		Output << "scale " << sweep.scale.x << ' ' << sweep.scale.y << ' ' << sweep.scale.z << ' ';
	}
	auto const precision{Output.precision(std::numeric_limits<double>::digits10)};
	for (auto const &piece : sweep.pieces)
	{
		auto const &start{piece.points[segment_data::point::start]};
		auto const &control1{piece.points[segment_data::point::control1]};
		auto const &control2{piece.points[segment_data::point::control2]};
		auto const &end{piece.points[segment_data::point::end]};
		Output << "\npiece " << start.x << ' ' << start.y << ' ' << start.z << ' ' << piece.rolls[0] << ' ' << control1.x << ' ' << control1.y << ' ' << control1.z << ' ' << control2.x << ' ' << control2.y << ' ' << control2.z << ' '
		       << end.x << ' ' << end.y << ' ' << end.z << ' ' << piece.rolls[1] << ' ' << piece.radius;
	}
	Output.precision(precision);
	Output << "\nendsweep\n";
}

void sweep_table::create_geometry(scene::scratch_data &Scratchpad)
{
	if (simulation::Region == nullptr)
	{
		return;
	}
	for (auto *sweep : m_items)
	{
		if (sweep == nullptr)
		{
			continue;
		}
		if (sweep->bridge())
		{
			if (Global.editor_session)
				sweep->mark_dirty();
		}
		else if (sweep->definition().widen || sweep->definition().platform)
		{
			sweep->refresh();
		}
		if (Global.editor_session && false == Global.NvRenderer)
		{
			// scenery opened for editing: the copies are kept apart, so they can be changed later
			sweep->show();
			continue;
		}
		for (auto &shape : sweep->create_shapes())
		{
			simulation::Region->insert(shape, Scratchpad, false);
		}
		sweep->merged(true);
	}
}

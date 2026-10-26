/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/windscreenwater.h"

#include "model/Model3d.h"
#include "model/material.h"
#include "rendering/renderer.h"
#include "simulation/simulation.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"

namespace gfx
{

namespace
{

// the param of mat_rain_windscreen the slot of a glass goes to (water_slot: param[2].z) and the one of the size of its drops
// (rain_scale: param[2].y)
std::size_t constexpr SLOT_PARAM{2};
// how fast the rain and the snow fall, m/s (as the particles of the rain and the snow, gfx::gpu_particles)
float constexpr RAIN_FALL{7.f};
float constexpr SNOW_FALL{1.1f};
// the share of the wind the rain drifts with (as the particles of the rain and the snow)
double constexpr WIND_SHARE{0.5};
// the smallest raindrop landing as a drop of its own, mm: the smaller ones are the mist of the wet glass
float constexpr LEAST_DROP{0.5f};
// the water a flake of snow brings, mm3 on average (water_spawn.comp: 0.3 .. 2.5)
float constexpr FLAKE_WATER{1.4f};
// the most landings tried on a glass in a step
std::uint32_t constexpr MOST_TRIES{1024};
// the longest step, seconds: a longer frame slows the water down rather than let it jump
double constexpr LONGEST_STEP{0.05};

// whether the material is a glass: one of mat_rain_windscreen, with a mask of the wipers
bool is_glass(opengl_material const *Material)
{
	return (Material != nullptr) && (Material->shader != nullptr) && (Material->shader->texture_conf.count("wipermask") > 0) && (Material->shader->param_conf.count("water_slot") > 0);
}

// calls Visit for every submodel of the branch, its children and its siblings
template <typename Visit> void visit_submodels(TSubModel *Submodel, Visit &&Visitor)
{
	while (Submodel != nullptr)
	{
		Visitor(Submodel);
		visit_submodels(Submodel->ChildGet(), Visitor);
		Submodel = Submodel->NextGet();
	}
}

// the vertices of a submodel, wherever they are kept
gfx::vertex_array const &submodel_vertices(TSubModel const *Submodel)
{
	return (Submodel->m_geometry.handle != null_handle ? GfxRenderer->Vertices(Submodel->m_geometry.handle) : Submodel->Vertices);
}

// the transformation of a submodel to the space of the model
glm::mat4 submodel_transform(TSubModel const *Submodel)
{
	float4x4 matrix;
	Submodel->ParentMatrix(&matrix);
	return glm::make_mat4(matrix.readArray());
}

} // namespace

bool windscreen_water::available()
{
	if (m_state == 0)
	{
		m_state = (gpu_init() ? 1 : -1);
		if (m_state < 0)
		{
			WriteLog("windscreen water: the simulation can't be used, the windscreens make their water up");
		}
	}
	return m_state > 0;
}

void windscreen_water::release()
{
	assign(false);
	m_glasses = {};
	m_glasscount = 0;
	m_vehicle = nullptr;
	m_cab = nullptr;
}

void windscreen_water::reset()
{
	release();
	m_state = 0;
}

void windscreen_water::assign(bool const Assigned)
{
	if (false == bool(m_host.material))
	{
		return;
	}
	for (std::size_t slot = 0; slot < m_glasscount; ++slot)
	{
		auto const &glass{m_glasses[slot]};
		if (auto *material{m_host.material(glass.material)}; is_glass(material))
		{
			material->params[SLOT_PARAM].z = (Assigned && glass.baked ? static_cast<float>(slot + 1) : 0.f);
		}
	}
}

void windscreen_water::rasterize(TSubModel *Submodel, material_handle const Material, glm::vec3 const &Inside, std::vector<glm::vec4> &Axes) const
{
	if ((Submodel->m_material != Material) || (Submodel->eType != GL_TRIANGLES))
	{
		return;
	}
	auto const &vertices{submodel_vertices(Submodel)};
	if (vertices.size() < 3)
	{
		return;
	}
	auto const transform{submodel_transform(Submodel)};
	gfx::index_array const *indices{nullptr};
	if ((Submodel->m_geometry.handle != null_handle) && (Submodel->m_geometry.index_count > 0))
	{
		indices = &GfxRenderer->Indices(Submodel->m_geometry.handle);
	}
	else if (false == Submodel->Indices.empty())
	{
		indices = &Submodel->Indices;
	}
	auto const count{indices != nullptr ? indices->size() : vertices.size()};
	auto const size{static_cast<int>(gl::WATER_MAP)};
	for (std::size_t first = 0; first + 2 < count; first += 3)
	{
		std::array<glm::vec3, 3> position;
		std::array<glm::vec2, 3> uv;
		bool valid{true};
		for (std::size_t corner = 0; corner < 3; ++corner)
		{
			auto const index{indices != nullptr ? static_cast<std::size_t>((*indices)[first + corner]) : first + corner};
			if (index >= vertices.size())
			{
				valid = false;
				break;
			}
			position[corner] = glm::vec3{transform * glm::vec4{vertices[index].position, 1.f}};
			uv[corner] = vertices[index].texture;
		}
		if (false == valid)
		{
			continue;
		}
		// the steps along the glass for the steps of u and v: the edges of the triangle in both spaces
		glm::mat2 const uvedges{uv[1] - uv[0], uv[2] - uv[0]};
		auto const det{glm::determinant(uvedges)};
		if (std::abs(det) < 1e-12f)
		{
			continue;
		}
		auto const inverse{glm::inverse(uvedges)};
		auto const e1{position[1] - position[0]};
		auto const e2{position[2] - position[0]};
		// [tu tv] = [e1 e2] * inverse
		auto const tu{e1 * inverse[0][0] + e2 * inverse[0][1]};
		auto const tv{e1 * inverse[1][0] + e2 * inverse[1][1]};
		auto const centre{(position[0] + position[1] + position[2]) / 3.f};
		auto const out{glm::dot(glm::cross(tu, tv), centre - Inside) >= 0.f ? 1.f : -1.f};
		// a triangle laid on a repeat of the texture is taken to the first one (the mask of the wipers repeats with it)
		auto const shift{glm::floor((uv[0] + uv[1] + uv[2]) / 3.f)};
		for (auto &corner : uv)
		{
			corner -= shift;
		}
		auto const lower{glm::max(glm::ivec2{glm::floor(glm::min(glm::min(uv[0], uv[1]), uv[2]) * static_cast<float>(size))}, glm::ivec2{0})};
		auto const upper{glm::min(glm::ivec2{glm::ceil(glm::max(glm::max(uv[0], uv[1]), uv[2]) * static_cast<float>(size))}, glm::ivec2{size - 1})};
		for (int y = lower.y; y <= upper.y; ++y)
		{
			for (int x = lower.x; x <= upper.x; ++x)
			{
				// the middle of the texel inside the triangle (barycentric)
				auto const point{(glm::vec2{static_cast<float>(x), static_cast<float>(y)} + 0.5f) / static_cast<float>(size)};
				auto const weights{inverse * (point - uv[0])};
				if ((weights.x < -1e-4f) || (weights.y < -1e-4f) || (weights.x + weights.y > 1.f + 1e-4f))
				{
					continue;
				}
				auto const texel{static_cast<std::size_t>(y) * gl::WATER_MAP + static_cast<std::size_t>(x)};
				Axes[texel * 2] = glm::vec4{tu, 1.f};
				Axes[texel * 2 + 1] = glm::vec4{tv, out};
			}
		}
	}
}

void windscreen_water::build(TModel3d &Cab)
{
	auto *root{Cab.GetSMRoot()};
	if ((root == nullptr) || (false == bool(m_host.material)))
	{
		return;
	}
	// the materials of the glasses, in the order they come in the model; and a point inside the cab, the middle of the box
	// around the whole model (the glasses are around it)
	std::vector<material_handle> materials;
	glm::vec3 lower{std::numeric_limits<float>::max()};
	glm::vec3 upper{std::numeric_limits<float>::lowest()};
	visit_submodels(root, [&](TSubModel *Submodel) {
		auto const &vertices{submodel_vertices(Submodel)};
		if (false == vertices.empty())
		{
			auto const transform{submodel_transform(Submodel)};
			for (auto const &vertex : vertices)
			{
				auto const position{glm::vec3{transform * glm::vec4{vertex.position, 1.f}}};
				lower = glm::min(lower, position);
				upper = glm::max(upper, position);
			}
		}
		auto const material{Submodel->m_material};
		if ((material != null_handle) && (material > 0) && (Submodel->eType == GL_TRIANGLES) && (std::find(materials.begin(), materials.end(), material) == materials.end()) && is_glass(m_host.material(material)))
		{
			materials.push_back(material);
		}
	});
	if (materials.empty())
	{
		return;
	}
	if (materials.size() > gl::WATER_SLOTS)
	{
		WriteLog("windscreen water: the cab has " + std::to_string(materials.size()) + " glasses, the water of the first " + std::to_string(gl::WATER_SLOTS) + " is simulated");
		materials.resize(gl::WATER_SLOTS);
	}
	auto const inside{(lower + upper) * 0.5f};
	auto const texels{static_cast<std::size_t>(gl::WATER_MAP) * gl::WATER_MAP};
	auto const size{static_cast<int>(gl::WATER_MAP)};
	for (auto const material : materials)
	{
		std::vector<glm::vec4> axes(texels * 2, glm::vec4{0.f});
		visit_submodels(root, [&](TSubModel *Submodel) { rasterize(Submodel, material, inside, axes); });
		auto &glass{m_glasses[m_glasscount]};
		glass = windscreen_water::glass{};
		glass.material = material;
		auto const *opengl{m_host.material(material)};
		glass.mask = opengl->textures[opengl->shader->texture_conf.at("wipermask").id];
		for (std::size_t texel = 0; texel < texels; ++texel)
		{
			if (axes[texel * 2].w > 0.f)
			{
				auto const density{glm::length(glm::cross(glm::vec3{axes[texel * 2]}, glm::vec3{axes[texel * 2 + 1]}))};
				glass.densest = std::max(glass.densest, density);
				glass.area += density / static_cast<float>(texels);
			}
		}
		if (glass.densest <= 0.f)
		{
			continue;
		}
		// the edges spread out by two texels, so the filtering of the map near them takes the glass along
		for (int pass = 0; pass < 2; ++pass)
		{
			auto const source{axes};
			for (int y = 0; y < size; ++y)
			{
				for (int x = 0; x < size; ++x)
				{
					auto const texel{static_cast<std::size_t>(y * size + x)};
					if (source[texel * 2].w > 0.f)
					{
						continue;
					}
					for (int k = 0; k < 8; ++k)
					{
						static int const dx[]{1, -1, 0, 0, 1, 1, -1, -1};
						static int const dy[]{0, 0, 1, -1, 1, -1, 1, -1};
						auto const nx{x + dx[k]};
						auto const ny{y + dy[k]};
						if ((nx < 0) || (ny < 0) || (nx >= size) || (ny >= size))
						{
							continue;
						}
						auto const neighbour{static_cast<std::size_t>(ny * size + nx)};
						if (source[neighbour * 2].w > 0.f)
						{
							axes[texel * 2] = glm::vec4{glm::vec3{source[neighbour * 2]}, 0.5f};
							axes[texel * 2 + 1] = source[neighbour * 2 + 1];
							break;
						}
					}
				}
			}
		}
		gpu_setup(m_glasscount, axes);
		WriteLog("windscreen water: glass " + std::to_string(m_glasscount + 1) + " of the cab, material " + opengl->GetName() + ", " + std::to_string(glass.area) + " m2");
		++m_glasscount;
	}
}

void windscreen_water::update(double const Seconds)
{
	if (false == available())
	{
		return;
	}
	auto const *vehicle{simulation::Train != nullptr ? simulation::Train->Dynamic() : nullptr};
	auto *cab{vehicle != nullptr ? vehicle->mdKabina : nullptr};
	if ((vehicle != m_vehicle) || (cab != m_cab))
	{
		// another cab: its glasses dry
		release();
		m_vehicle = vehicle;
		m_cab = cab;
		m_bladesbefore = glm::vec4{-1.f};
		if (cab != nullptr)
		{
			build(*cab);
		}
	}
	if (m_glasscount == 0)
	{
		return;
	}

	gl::water_ubs block{};
	// the blades of the wipers of the vehicle, as the glasses read them (the channels of the mask follow the wipers of the cab
	// the vehicle is driven from)
	glm::vec4 blades{-1.f};
	if (auto const cabactive{vehicle->MoverParameters->CabActive}; cabactive != 0)
	{
		auto const count{static_cast<int>(vehicle->dWiperPos.size())};
		for (int i = 0; i < 4; ++i)
		{
			if (i < count)
			{
				auto const index{cabactive > 0 ? i : count - 1 - i};
				blades[i] = static_cast<float>(std::clamp(vehicle->dWiperPos[index], 0.0, 1.0));
			}
		}
	}
	// the space of the model of the cab, drawn with the matrix of the vehicle
	auto const toworld{glm::dmat3{vehicle->mMatrix}};
	auto const tomodel{glm::inverse(toworld)};
	auto const up{glm::normalize(glm::vec3{tomodel * glm::dvec3{0.0, 1.0, 0.0}})};
	auto const air{glm::vec3{tomodel * (glm::dvec3{simulation::Environment.wind()} * WIND_SHARE - vehicle->VectorFront() * vehicle->MoverParameters->V)}};
	auto const heavy{std::clamp(Global.Overcast - 1.f, 0.f, 1.f)};
	auto const snow{Global.Weather == "snow:"};
	auto const fall{snow ? SNOW_FALL : RAIN_FALL};

	// the masks of the wipers baked into the maps once the textures are there
	for (std::size_t slot = 0; slot < m_glasscount; ++slot)
	{
		auto &glass{m_glasses[slot]};
		if (false == glass.baked)
		{
			block.counts = glm::uvec4{0u, 0u, static_cast<std::uint32_t>(slot), 0u};
			glass.baked = gpu_bake(slot, glass.mask, block);
		}
	}
	assign(true);

	auto const seconds{std::clamp(Seconds, 0.0, LONGEST_STEP)};
	if (seconds <= 0.0)
	{
		return;
	}
	m_time += seconds;
	++m_step;

	// the drops landing: as many raindrops a second come to a square metre of the ground as the rain gives water (from light rain at
	// 1 mm/h to a downpour at 30 mm/h) in the drops of the Marshall-Palmer distribution above the least landing as a drop of its own
	// (exponential, the mean diameter falling with the rain); a glass gets them in the share it faces the rain coming at it
	// against the cab (water_spawn.comp)
	float perground{0.f}; // the drops coming to a square metre of the ground in a second
	auto const rate{1.f + 29.f * std::pow(heavy, 1.5f)}; // mm/h
	auto const lambda{4.1f * std::pow(rate, -0.21f)}; // 1 / mm
	if (heavy > 0.f)
	{
		auto const water{rate / 3600.f * 1e6f}; // mm3 a second on a square metre
		if (snow)
		{
			perground = water * 0.25f / FLAKE_WATER;
		}
		else
		{
			// the share of the water in the drops above the least one, and their mean volume
			auto const x{lambda * LEAST_DROP};
			auto const share{std::exp(-x) * (1.f + x + x * x / 2.f + x * x * x / 6.f)};
			auto const d0{LEAST_DROP};
			auto const mean{0.5236f * (d0 * d0 * d0 + 3.f * d0 * d0 / lambda + 6.f * d0 / (lambda * lambda) + 6.f / (lambda * lambda * lambda))};
			perground = water * share / mean;
		}
	}
	// the landings tried on a glass: as if the whole of its uv held the glass at its densest, the ones at the less dense places and
	// facing the rain less are left out. the share of a landing left over goes to the next step
	auto const coming{glm::length(air - up * fall)};
	std::uint32_t most{0};
	m_tries = 0;
	for (std::size_t slot = 0; slot < m_glasscount; ++slot)
	{
		auto &glass{m_glasses[slot]};
		auto const *material{m_host.material(glass.material)};
		auto const simulated{glass.baked && is_glass(material)};
		glass.landings += static_cast<double>(perground / fall * coming * glass.densest) * seconds;
		auto const whole{std::floor(glass.landings)};
		glass.landings -= whole;
		auto const tries{simulated ? static_cast<std::uint32_t>(std::min<double>(whole, MOST_TRIES)) : 0u};
		most = std::max(most, tries);
		m_tries += tries;
		auto const scale{(material != nullptr) && std::isfinite(material->params[SLOT_PARAM].y) ? std::max(material->params[SLOT_PARAM].y, 0.05f) : 1.f};
		block.slots[slot] = glm::vec4{static_cast<float>(tries), 1.f / glass.densest, simulated ? 1.f : 0.f, scale};
	}

	block.step = glm::vec4{static_cast<float>(seconds), static_cast<float>(m_time), heavy, snow ? 1.f : 0.f};
	block.up = glm::vec4{up, Global.AirTemperature};
	block.air = glm::vec4{air, fall};
	block.blades = blades;
	block.blades_before = glm::vec4{
	    m_bladesbefore.x < 0.f ? blades.x : m_bladesbefore.x, m_bladesbefore.y < 0.f ? blades.y : m_bladesbefore.y, m_bladesbefore.z < 0.f ? blades.z : m_bladesbefore.z, m_bladesbefore.w < 0.f ? blades.w : m_bladesbefore.w};
	block.rain = glm::vec4{LEAST_DROP, lambda, 1.f, 0.f};
	block.counts = glm::uvec4{m_step * 2654435761u, most, 0u, 0u};
	m_bladesbefore = blades;
	gpu_step(block, most);
}

std::string windscreen_water::info() const
{
	if ((m_state <= 0) || (m_glasscount == 0))
	{
		return {};
	}
	return "windscreen water: " + std::to_string(m_glasscount) + " glasses, " + std::to_string(m_tries) + " drops landing tried in the frame\n";
}

} // namespace gfx

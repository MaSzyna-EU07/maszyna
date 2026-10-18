/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the heightmap terrain made from the digital terrain model of GUGiK: the scenery placed as its orthophoto is, the heights
// of the model moved up or down to meet the tracks it has already, the chunks around the camera or along the tracks made as
// the model comes in, and the formation of each track with its slopes kept as a modifier of the terrain

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace
{

// how many places of the tracks the offset of the model is worked out from at most, each needing its square of the model
std::size_t constexpr fit_places{48};

bool rail(TTrack const &Track)
{
	return (Track.iCategoryFlag & 15) == 1 && false == Track.m_editorremoved && false == Track.m_paths.empty();
}

// a bridge or a tunnel keeps the model under it as it is
bool shapes_ground(TTrack const &Track)
{
	return Track.eEnvironment != e_bridge && Track.eEnvironment != e_tunnel;
}

} // namespace

std::vector<editor_mode::nmt_place> editor_mode::nmt_track_places(double const Step) const
{
	std::vector<nmt_place> result;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track))
			continue;
		auto const ballast{profile_ballast(*track)};
		for (std::size_t path = 0; path < track->m_paths.size(); ++path)
		{
			geometry::bezier const curve{track->m_paths[path]};
			auto const &points{track->m_paths[path].points};
			auto const chord{glm::distance(points[segment_data::point::start], points[segment_data::point::end])};
			auto const count{std::max(1, static_cast<int>(std::ceil(chord * 1.1 / Step)))};
			for (int i = 0; i <= count; ++i)
			{
				auto const at{curve.point(static_cast<double>(i) / count)};
				result.push_back({at, at.y - ballast.first, ballast.second, track, static_cast<int>(path), shapes_ground(*track)});
			}
		}
	}
	return result;
}

void editor_mode::nmt_locate()
{
	auto const &config{m_orthophoto.settings()};
	m_nmt.place(config.north, config.east);
}

void editor_mode::nmt_fit()
{
	nmt_locate();
	auto &state{m_nmt_state};
	state.fit.clear();
	auto const places{nmt_track_places(25.0)};
	std::vector<nmt_place> shaped;
	std::copy_if(places.begin(), places.end(), std::back_inserter(shaped), [](nmt_place const &Place) { return Place.shapes; });
	if (shaped.empty())
	{
		state.offset = 0.0;
		state.fitted = true;
		state.status = STR("No tracks to fit the terrain model to: its heights are taken as they are");
		return;
	}
	// spread over the whole scenery, each place asking for its square of the model
	auto const every{std::max<std::size_t>(1, shaped.size() / fit_places)};
	for (std::size_t i = 0; i < shaped.size(); i += every)
		state.fit.push_back(shaped[i]);
	state.fitting = true;
	state.status = STR("Fetching the terrain model under the tracks...");
}

void editor_mode::nmt_finish_fit()
{
	auto &state{m_nmt_state};
	std::vector<double> differences;
	for (auto const &place : state.fit)
		if (double height; m_nmt.height(place.position.x, place.position.z, height))
			differences.push_back(place.formation - height);
	state.fitting = false;
	if (differences.empty())
	{
		state.status = STR("The terrain model has no data under the tracks: is the scenery placed where it is?");
		return;
	}
	std::sort(differences.begin(), differences.end());
	state.offset = differences[differences.size() / 2];
	std::vector<double> deviations;
	for (auto const difference : differences)
		deviations.push_back(std::abs(difference - state.offset));
	std::sort(deviations.begin(), deviations.end());
	state.spread = deviations[deviations.size() / 2];
	state.fitted = true;
	state.samples = differences.size();
	state.status = format(STR_C("The tracks lie %+.2f m against the terrain model (%zu places, they differ by %.2f m around it)"), state.offset, state.samples, state.spread);
	if (state.spread > 10.0)
		state.status += STR(". That's far too much: the place of the scenery is likely wrong, check the orthophoto");
	WriteLog("Editor: NMT - " + state.status, logtype::generic);
}

std::vector<heightmap::chunk_key> editor_mode::nmt_keys_around() const
{
	std::vector<heightmap::chunk_key> result;
	auto const centre{heightmap::chunk_at(Camera.Pos.x, Camera.Pos.z)};
	auto const first{-(m_terrain_add_count - 1) / 2};
	for (int dz = 0; dz < m_terrain_add_count; ++dz)
		for (int dx = 0; dx < m_terrain_add_count; ++dx)
			result.push_back({centre.first + first + dx, centre.second + first + dz});
	return result;
}

std::vector<heightmap::chunk_key> editor_mode::nmt_keys_along_tracks() const
{
	std::set<heightmap::chunk_key> keys;
	auto const reach{static_cast<int>(std::ceil(m_nmt_state.band / heightmap::chunk_size))};
	for (auto const &place : nmt_track_places(heightmap::chunk_size * 0.5))
	{
		auto const at{heightmap::chunk_at(place.position.x, place.position.z)};
		for (int dz = -reach; dz <= reach; ++dz)
			for (int dx = -reach; dx <= reach; ++dx)
			{
				heightmap::chunk_key const key{at.first + dx, at.second + dz};
				auto const low{heightmap::chunk_corner(key)};
				// the chunk reaches within the band of the track
				glm::dvec2 const nearest{std::clamp(place.position.x, low.x, low.x + heightmap::chunk_size), std::clamp(place.position.z, low.y, low.y + heightmap::chunk_size)};
				if (glm::distance(nearest, glm::dvec2{place.position.x, place.position.z}) <= m_nmt_state.band)
					keys.insert(key);
			}
	}
	return {keys.begin(), keys.end()};
}

void editor_mode::nmt_make(std::vector<heightmap::chunk_key> const &Keys)
{
	auto &state{m_nmt_state};
	if (false == ensure_terrain())
	{
		state.status = STR("The heightmap terrain can't be opened");
		return;
	}
	nmt_locate();
	std::size_t queued{0};
	for (auto const &key : Keys)
		if (false == m_streamer.exists(key) && std::find(state.pending.begin(), state.pending.end(), key) == state.pending.end())
		{
			state.pending.push_back(key);
			++queued;
		}
	state.made = 0;
	// the terrain comes with its photos
	if (queued > 0 && false == m_orthophoto.enabled())
		m_orthophoto.enabled(true);
	state.status = queued > 0 ? format(STR_C("%zu chunks to make from the terrain model"), queued) : STR("There's terrain there already");
}

bool editor_mode::add_chunk_from_nmt(heightmap::chunk_key const &Key)
{
	auto const low{heightmap::chunk_corner(Key)};
	auto const high{low + glm::dvec2{heightmap::chunk_size}};
	if (false == m_nmt.request(low - glm::dvec2{2.0}, high + glm::dvec2{2.0}))
		return false;
	auto const cells{heightmap::cells_for(m_terrain_spacing)};
	auto const side{static_cast<std::size_t>(cells) + 1};
	auto const step{heightmap::chunk_size / cells};
	std::vector<double> heights(side * side, m_terrain_baseheight);
	std::vector<char> found(side * side, 0);
	bool any{false};
	for (std::size_t iz = 0; iz < side; ++iz)
		for (std::size_t ix = 0; ix < side; ++ix)
		{
			auto const index{iz * side + ix};
			if (double height; m_nmt.height(low.x + ix * step, low.y + iz * step, height))
			{
				heights[index] = height + m_nmt_state.offset;
				found[index] = 1;
				any = true;
			}
		}
	// where the model has no data the chunk goes on level with its nearest point
	if (any)
		fill_from_nearest(heights, found, side);
	m_streamer.add_chunk(Key.first, Key.second, m_terrain_spacing, [&](double const X, double const Z) {
		auto const ix{static_cast<std::size_t>(std::clamp<long>(std::lround((X - low.x) / step), 0, cells))};
		auto const iz{static_cast<std::size_t>(std::clamp<long>(std::lround((Z - low.y) / step), 0, cells))};
		return static_cast<float>(heights[iz * side + ix]);
	});
	return true;
}

void editor_mode::nmt_shape_tracks()
{
	auto &state{m_nmt_state};
	if (false == ensure_terrain())
		return;
	// one modifier per path: the formation under its axis and the slopes from its edges to the ground. laid over each other the
	// modifiers keep the highest of the slopes going down and the lowest of those going up, so tracks side by side share one
	// formation instead of a saw of their own ones
	std::map<std::pair<TTrack *, int>, heightmap::modifier> modifiers;
	double constexpr step{5.0};
	for (auto const &place : nmt_track_places(step))
	{
		if (false == place.shapes)
			continue;
		auto &modifier{modifiers[{place.track, place.path}]};
		if (modifier.name.empty())
		{
			modifier.name = "nmt " + (place.track->name().empty() ? std::string{"track"} : place.track->name()) + (place.track->m_paths.size() > 1 ? " " + std::to_string(place.path) : std::string{});
			modifier.slope = state.slope;
			modifier.reach = state.reach;
		}
		modifier.points.push_back({{place.position.x, place.position.z}, place.formation, place.half_width});
	}
	for (auto &[path, modifier] : modifiers)
		if (modifier.points.size() >= 2)
			m_streamer.modifier(std::move(modifier));
	forget_ground();
	state.shaped = modifiers.size();
}

void editor_mode::nmt_update()
{
	auto &state{m_nmt_state};
	if (state.fitting)
	{
		bool all{true};
		for (auto const &place : state.fit)
			all = m_nmt.request(glm::dvec2{place.position.x, place.position.z} - glm::dvec2{1.0}, glm::dvec2{place.position.x, place.position.z} + glm::dvec2{1.0}) && all;
		if (all)
			nmt_finish_fit();
		return;
	}
	if (state.pending.empty())
		return;
	// a few chunks a frame, those whose model is in
	int made{0};
	for (auto it{state.pending.begin()}; it != state.pending.end() && made < 4;)
	{
		if (m_streamer.exists(*it))
		{
			it = state.pending.erase(it);
			continue;
		}
		if (add_chunk_from_nmt(*it))
		{
			it = state.pending.erase(it);
			++made;
			++state.made;
			continue;
		}
		++it;
	}
	if (made > 0)
	{
		forget_ground();
		state.status = format(STR_C("Chunks made from the terrain model: %zu, waiting for it: %zu"), state.made, state.pending.size());
		if (state.pending.empty())
		{
			if (state.shape)
			{
				nmt_shape_tracks();
				state.status += format(STR_C("; the formation of %zu paths shaped"), state.shaped);
			}
			// the photos laid onto the new ground
			m_orthophoto.refit();
		}
	}
	if (auto const error{m_nmt.error()}; false == error.empty())
		state.error = error;
}

void editor_mode::render_nmt_ui()
{
	auto &state{m_nmt_state};
	nmt_locate();
	if (false == m_nmt.placed())
	{
		ImGui::TextWrapped("%s", STR_C("The scenery isn't placed anywhere: give its place in the settings of the orthophoto first"));
		if (ImGui::Button(STR_C("Orthophoto settings...")))
			m_orthophoto_window = true;
		return;
	}
	auto const &config{m_orthophoto.settings()};
	ImGui::TextDisabled(STR_C("Place: N %.0f, E %.0f (PUWG 1992), as the orthophoto"), config.north, config.east);
	if (ImGui::Button(STR_C("Fit to the tracks")))
		nmt_fit();
	item_tooltip("Compares the formation of the tracks with the terrain model and moves the model up or down to meet them, as a scenery built at height 0 needs");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(110.0f);
	ImGui::InputDouble(STR_C("Height offset (m)"), &state.offset, 0.0, 0.0, "%.2f");
	ImGui::SetNextItemWidth(110.0f);
	ImGui::InputDouble(STR_C("Band along the tracks (m)"), &state.band, 0.0, 0.0, "%.0f");
	state.band = std::clamp(state.band, 50.0, 3000.0);
	ImGui::Checkbox(STR_C("Formation and slopes along the tracks"), &state.shape);
	item_tooltip("Each track gets its formation with slopes 1:1.5 down or up to the terrain, as a modifier of the terrain; bridges and tunnels keep the model under them");
	ImGui::BeginDisabled(state.fitting);
	if (ImGui::Button(STR_C("Along the tracks")))
		nmt_make(nmt_keys_along_tracks());
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Around the camera")))
		nmt_make(nmt_keys_around());
	ImGui::EndDisabled();
	if (false == state.pending.empty() || m_nmt.waiting() > 0)
		ImGui::TextDisabled(STR_C("Squares of the model being fetched: %zu"), m_nmt.waiting());
	if (false == state.status.empty())
		ImGui::TextWrapped("%s", state.status.c_str());
	if (false == state.error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", state.error.c_str());
	ImGui::TextDisabled("%s", STR_C("Terrain model: GUGiK, geoportal.gov.pl"));
}

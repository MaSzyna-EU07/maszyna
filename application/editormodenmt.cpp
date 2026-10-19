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
#include "editor/editorSettings.hpp"
#include "model/AnimModel.h"
#include "utilities/Globals.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <set>

namespace
{

// how many places of the tracks the offset of the model is worked out from at most, each needing its square of the model
std::size_t constexpr fit_places{48};
// the terrain under the tracks: places along them every so many m, the squares they're kept in, how far from the tracks the
// difference at them draws the model; past that the offset of the whole scenery
double constexpr anchor_step{25.0};
double constexpr anchor_cell{100.0};
double constexpr anchor_reach{300.0};
double constexpr anchor_near{5.0};

std::pair<int, int> anchor_cell_of(double const X, double const Z)
{
	return {static_cast<int>(std::floor(X / anchor_cell)), static_cast<int>(std::floor(Z / anchor_cell))};
}

// a path whose ends lie this far apart is an error of the scenery, as a point typed wrong: no grade line, no formation and no
// terrain along it
double constexpr broken_path{5000.0};

bool rail(TTrack const &Track)
{
	if ((Track.iCategoryFlag & 15) != 1 || Track.m_editorremoved || Track.m_paths.empty())
		return false;
	auto const &path{Track.m_paths.front()};
	return geometry::plan_distance(path.points[segment_data::point::start], path.points[segment_data::point::end]) <= broken_path;
}

// a choice of the new scenery as a card: a sketch of the ground a track lies on, the name and what it means; true when clicked.
// the real terrain: hills under an aerial photo, the track along them; none: the flat plane at 0 drawn as a grid
bool ground_card(char const *Id, char const *Title, char const *Text, bool const Hills, bool const Enabled, float const Width)
{
	auto const font{ImGui::GetFontSize()};
	auto const pad{std::round(font * 0.6f)};
	auto const sketch{std::round(font * 4.0f)};
	auto const wrap{Width - 2.0f * pad};
	auto const title{ImGui::CalcTextSize(Title, nullptr, false, wrap)};
	auto const text{ImGui::CalcTextSize(Text, nullptr, false, wrap)};
	auto const height{pad + sketch + pad * 0.6f + title.y + pad * 0.3f + text.y + pad};
	auto const top{ImGui::GetCursorScreenPos()};
	ImGui::BeginDisabled(false == Enabled);
	auto const clicked{ImGui::InvisibleButton(Id, ImVec2(Width, height))};
	ImGui::EndDisabled();
	auto const hovered{Enabled && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)};
	auto *list{ImGui::GetWindowDrawList()};
	ImVec2 const bottom{top.x + Width, top.y + height};
	auto const rounding{std::round(font * 0.4f)};
	auto const accent{family_colour(0)};
	list->AddRectFilled(top, bottom, ImGui::GetColorU32(hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), rounding);
	list->AddRect(top, bottom, hovered ? accent : ImGui::GetColorU32(ImGuiCol_Border), rounding, 0, hovered ? 2.0f : 1.0f);
	auto const alpha{Enabled ? 1.0f : 0.35f};
	auto const tint = [&](ImU32 const Colour) { return (Colour & ~IM_COL32_A_MASK) | (static_cast<ImU32>(((Colour >> IM_COL32_A_SHIFT) & 0xFF) * alpha) << IM_COL32_A_SHIFT); };
	ImVec2 const low{top.x + pad, top.y + pad};
	ImVec2 const high{top.x + Width - pad, top.y + pad + sketch};
	list->AddRectFilled(low, high, tint(IM_COL32(20, 26, 34, 255)), rounding * 0.6f);
	list->PushClipRect(low, high, true);
	int constexpr steps{48};
	auto const x_of = [&](float const Share) { return low.x + (high.x - low.x) * Share; };
	if (Hills)
	{
		auto const ground = [](float const Share) { return 0.45f + 0.18f * std::sin(Share * 7.0f + 0.4f) + 0.08f * std::sin(Share * 17.0f); };
		std::vector<ImVec2> line;
		for (int i = 0; i <= steps; ++i)
		{
			auto const share{static_cast<float>(i) / steps};
			line.emplace_back(x_of(share), low.y + (high.y - low.y) * ground(share));
		}
		// the strips under the line drawn without the soft edges, else seams show between them
		auto const flags{list->Flags};
		list->Flags &= ~ImDrawListFlags_AntiAliasedFill;
		for (int i = 0; i < steps; ++i)
			list->AddQuadFilled(line[i], line[i + 1], ImVec2(line[i + 1].x, high.y), ImVec2(line[i].x, high.y), tint(IM_COL32(70, 110, 55, 200)));
		list->Flags = flags;
		list->AddPolyline(line.data(), static_cast<int>(line.size()), tint(IM_COL32(150, 200, 110, 255)), 0, 1.5f);
		// the track goes along the ground, a little above it on embankments, below it in cuttings
		std::vector<ImVec2> track;
		for (int i = 0; i <= steps; ++i)
		{
			auto const share{static_cast<float>(i) / steps};
			auto const smooth{0.45f + 0.09f * std::sin(share * 7.0f + 0.4f)};
			track.emplace_back(x_of(share), low.y + (high.y - low.y) * smooth - 2.0f);
		}
		list->AddPolyline(track.data(), static_cast<int>(track.size()), tint(IM_COL32(235, 235, 235, 255)), 0, 2.5f);
	}
	else
	{
		auto const horizon{low.y + (high.y - low.y) * 0.35f};
		auto const grid{tint(IM_COL32(120, 130, 150, 120))};
		for (int i = 0; i <= 4; ++i)
		{
			auto const t{static_cast<float>(i) / 4.0f};
			auto const y{horizon + (high.y - horizon) * t * t};
			list->AddLine(ImVec2(low.x, y), ImVec2(high.x, y), grid);
		}
		auto const middle{(low.x + high.x) * 0.5f};
		for (int i = -6; i <= 6; ++i)
			list->AddLine(ImVec2(middle + i * (high.x - low.x) * 0.06f, horizon), ImVec2(middle + i * (high.x - low.x) * 0.22f, high.y), grid);
		auto const level{horizon + (high.y - horizon) * 0.45f};
		list->AddLine(ImVec2(low.x, level), ImVec2(high.x, level), tint(IM_COL32(235, 235, 235, 255)), 2.5f);
	}
	list->PopClipRect();
	auto y{high.y + pad * 0.6f};
	list->AddText(nullptr, 0.0f, ImVec2(low.x, y), tint(ImGui::GetColorU32(ImGuiCol_Text)), Title, nullptr, wrap);
	y += title.y + pad * 0.3f;
	list->AddText(nullptr, 0.0f, ImVec2(low.x, y), tint(ImGui::GetColorU32(ImGuiCol_TextDisabled)), Text, nullptr, wrap);
	return clicked && Enabled;
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
	// a track with no trackbed of its own, as the switches of the older sceneries whose bed is an include, lies in the bed of
	// the others
	std::vector<double> depths;
	for (auto *track : simulation::Paths.sequence())
		if (track != nullptr && rail(*track) && profile_own_bed(*track))
			depths.push_back(profile_ballast(*track).first);
	std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
	auto const usual{depths.empty() ? 0.0 : depths[depths.size() / 2]};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track))
			continue;
		auto const rails{static_cast<double>(track->rail_height())};
		for (std::size_t path = 0; path < track->m_paths.size(); ++path)
		{
			// the ballast drawn under the path: a switch has it of the plain track it goes on into, as the renderer lays it
			auto const *source{track};
			if (track->eType == tt_Switch && track->SwitchExtension != nullptr && path < 2)
			{
				auto const *previous{track->SwitchExtension->pPrevs[path]};
				auto const *next{track->SwitchExtension->pNexts[path]};
				source = previous != nullptr && previous->eType != tt_Switch ? previous : next != nullptr && next->eType != tt_Switch ? next : track;
			}
			auto ballast{profile_ballast(*source)};
			if (false == profile_own_bed(*source))
				ballast.first = usual;
			geometry::bezier const curve{track->m_paths[path]};
			auto const &points{track->m_paths[path].points};
			auto const chord{glm::distance(points[segment_data::point::start], points[segment_data::point::end])};
			auto const count{std::max(1, static_cast<int>(std::ceil(chord * 1.1 / Step)))};
			for (int i = 0; i <= count; ++i)
			{
				auto const at{curve.point(static_cast<double>(i) / count)};
				result.push_back({at, at.y - ballast.first, at.y - rails, ballast.second, track, static_cast<int>(path), shapes_ground(*track)});
			}
		}
	}
	// a track passing over another one by more than the clearance, as at a flyover whose bridge is a model: where the slopes of its
	// embankment would bury the track below, it stands on a bridge, and the ground there is the lower track's
	double constexpr clearance{3.0};
	double constexpr cell{20.0};
	auto const key_of = [&](double const X, double const Z) { return (static_cast<std::int64_t>(std::floor(X / cell)) << 32) ^ (static_cast<std::int64_t>(std::floor(Z / cell)) & 0xffffffff); };
	std::unordered_map<std::int64_t, std::vector<std::size_t>> grid;
	double widest{0.0};
	double lowest{std::numeric_limits<double>::max()}, highest{std::numeric_limits<double>::lowest()};
	for (std::size_t i = 0; i < result.size(); ++i)
	{
		grid[key_of(result[i].position.x, result[i].position.z)].push_back(i);
		widest = std::max(widest, result[i].half_width);
		lowest = std::min(lowest, result[i].crown);
		highest = std::max(highest, result[i].formation);
	}
	// the farthest the slopes of an embankment over a track can reach
	auto const reach{2.0 * widest + m_nmt_state.slope * std::max(0.0, highest - lowest)};
	auto const span{static_cast<int>(std::ceil(std::min(reach, 60.0) / cell))};
	std::vector<char> over(result.size(), 0);
	for (std::size_t i = 0; i < result.size(); ++i)
	{
		auto const &place{result[i]};
		if (false == place.shapes)
			continue;
		auto const cx{static_cast<std::int64_t>(std::floor(place.position.x / cell))};
		auto const cz{static_cast<std::int64_t>(std::floor(place.position.z / cell))};
		for (auto dz = -span; dz <= span && 0 == over[i]; ++dz)
			for (auto dx = -span; dx <= span && 0 == over[i]; ++dx)
			{
				auto const found{grid.find(((cx + dx) << 32) ^ ((cz + dz) & 0xffffffff))};
				if (found == grid.end())
					continue;
				for (auto const j : found->second)
				{
					auto const &other{result[j]};
					if (other.track == place.track)
						continue;
					auto const height{place.formation - other.crown};
					if (height < clearance)
						continue;
					auto const distance{glm::distance(glm::dvec2{place.position.x, place.position.z}, glm::dvec2{other.position.x, other.position.z})};
					if (distance <= place.half_width + other.half_width + m_nmt_state.slope * height)
					{
						over[i] = 1;
						break;
					}
				}
			}
	}
	for (std::size_t i = 0; i < result.size(); ++i)
		if (over[i] != 0)
			result[i].shapes = false;
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
	state.under = false;
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

void editor_mode::nmt_under_start()
{
	nmt_locate();
	auto &state{m_nmt_state};
	state.fit.clear();
	state.anchors.clear();
	state.anchor_cells.clear();
	for (auto const &place : nmt_track_places(anchor_step))
		if (place.shapes)
			state.fit.push_back(place);
	if (state.fit.empty())
	{
		state.status = STR("No tracks to make the terrain under");
		return;
	}
	state.under = true;
	state.fitting = true;
	state.status = STR("Fetching the terrain model under the tracks...");
}

void editor_mode::nmt_under_anchors()
{
	auto &state{m_nmt_state};
	struct found
	{
		glm::dvec2 at;
		double difference;
		TTrack const *track;
		int path;
	};
	std::vector<found> places;
	for (auto const &place : state.fit)
		if (double height; m_nmt.height(place.position.x, place.position.z, height))
			places.push_back({{place.position.x, place.position.z}, place.crown - height, place.track, place.path});
	state.anchors.clear();
	state.anchor_cells.clear();
	for (std::size_t i = 0; i < places.size(); ++i)
	{
		// a place where the model is off, as a bridge it doesn't know of or a cutting dug since, doesn't pull the terrain around
		// it: the median of the places around it along the same path
		std::vector<double> around;
		for (auto j{i >= 4 ? i - 4 : 0}; j < std::min(places.size(), i + 5); ++j)
			if (places[j].track == places[i].track && places[j].path == places[i].path)
				around.push_back(places[j].difference);
		std::nth_element(around.begin(), around.begin() + around.size() / 2, around.end());
		state.anchor_cells[anchor_cell_of(places[i].at.x, places[i].at.y)].push_back(state.anchors.size());
		state.anchors.emplace_back(places[i].at.x, places[i].at.y, around[around.size() / 2]);
	}
}

double editor_mode::nmt_offset_at(double const X, double const Z) const
{
	auto const &state{m_nmt_state};
	if (false == state.under || state.anchors.empty())
		return state.offset;
	// the places near weigh the most, down to nothing at the reach; the offset of the scenery weighs as one place at the reach
	auto const cell{anchor_cell_of(X, Z)};
	auto const span{static_cast<int>(std::ceil(anchor_reach / anchor_cell))};
	auto const outermost{1.0 / (anchor_reach * anchor_reach + anchor_near * anchor_near)};
	double sum{state.offset * outermost};
	double weights{outermost};
	for (int dz = -span; dz <= span; ++dz)
		for (int dx = -span; dx <= span; ++dx)
		{
			auto const found{state.anchor_cells.find({cell.first + dx, cell.second + dz})};
			if (found == state.anchor_cells.end())
				continue;
			for (auto const index : found->second)
			{
				auto const &anchor{state.anchors[index]};
				auto const squared{(anchor.x - X) * (anchor.x - X) + (anchor.y - Z) * (anchor.y - Z)};
				if (squared >= anchor_reach * anchor_reach)
					continue;
				auto const weight{1.0 / (squared + anchor_near * anchor_near) - outermost};
				sum += anchor.z * weight;
				weights += weight;
			}
		}
	return sum / weights;
}

void editor_mode::nmt_finish_fit()
{
	auto &state{m_nmt_state};
	std::vector<double> differences;
	for (auto const &place : state.fit)
		if (double height; m_nmt.height(place.position.x, place.position.z, height))
			differences.push_back(place.crown - height);
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
	state.status = format(STR_P("The tracks lie %+.2f m against the terrain model (%zu place, they differ by %.2f m around it)", "The tracks lie %+.2f m against the terrain model (%zu places, they differ by %.2f m around it)", state.samples), state.offset, state.samples, state.spread);
	if (state.spread > 10.0)
		state.status += STR(". That's far too much: the place of the scenery is likely wrong, check the orthophoto");
	WriteLog("Editor: NMT - " + state.status, logtype::generic);
	if (false == state.under)
		return;
	// the terrain under the tracks: drawn to them place by place, then made along them
	nmt_under_anchors();
	auto const status{state.status};
	nmt_make(nmt_keys_along_tracks(), true);
	state.status = status + "\n" + state.status;
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

void editor_mode::nmt_make(std::vector<heightmap::chunk_key> const &Keys, bool const Rebuild)
{
	auto &state{m_nmt_state};
	if (false == ensure_terrain())
	{
		state.status = STR("The heightmap terrain can't be opened");
		return;
	}
	nmt_locate();
	if (Rebuild)
	{
		// the terrain made for the tracks before, as they lay then: the formation of the earlier runs, and the chunks along them,
		// whose heights went with it
		auto const dropped{m_streamer.drop_modifiers("nmt ")};
		std::size_t removed{0};
		for (auto const &key : Keys)
			if (m_streamer.exists(key))
			{
				m_streamer.remove_chunk(key.first, key.second);
				++removed;
			}
		forget_ground();
		WriteLog(format("Editor: NMT - made anew: %zu chunks, %zu formations of the tracks forgotten", removed, dropped), logtype::generic);
	}
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
	state.status = queued > 0 ? format(STR_P("%zu chunk to make from the terrain model", "%zu chunks to make from the terrain model", queued), queued) : STR("There's terrain there already");
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
	// the offset over the chunk, every sixteenth of its side and between them along straight lines: it changes slowly
	int constexpr coarse{16};
	std::array<double, (coarse + 1) * (coarse + 1)> offsets;
	for (int iz = 0; iz <= coarse; ++iz)
		for (int ix = 0; ix <= coarse; ++ix)
			offsets[iz * (coarse + 1) + ix] = nmt_offset_at(low.x + ix * heightmap::chunk_size / coarse, low.y + iz * heightmap::chunk_size / coarse);
	auto const offset_at = [&](double const X, double const Z) {
		auto const u{std::clamp((X - low.x) / heightmap::chunk_size * coarse, 0.0, static_cast<double>(coarse))};
		auto const v{std::clamp((Z - low.y) / heightmap::chunk_size * coarse, 0.0, static_cast<double>(coarse))};
		auto const i{std::min(static_cast<int>(u), coarse - 1)};
		auto const j{std::min(static_cast<int>(v), coarse - 1)};
		auto const fu{u - i};
		auto const fv{v - j};
		auto const at = [&](int const I, int const J) { return offsets[J * (coarse + 1) + I]; };
		return (at(i, j) * (1.0 - fu) + at(i + 1, j) * fu) * (1.0 - fv) + (at(i, j + 1) * (1.0 - fu) + at(i + 1, j + 1) * fu) * fv;
	};
	bool any{false};
	for (std::size_t iz = 0; iz < side; ++iz)
		for (std::size_t ix = 0; ix < side; ++ix)
		{
			auto const index{iz * side + ix};
			if (double height; m_nmt.height(low.x + ix * step, low.y + iz * step, height))
			{
				heights[index] = height + offset_at(low.x + ix * step, low.y + iz * step);
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

std::size_t editor_mode::nmt_shape_tracks(std::vector<TTrack *> const *Only)
{
	auto &state{m_nmt_state};
	if (false == ensure_terrain())
		return 0;
	auto const wanted = [&](TTrack const *Track) { return Only == nullptr || std::find(Only->begin(), Only->end(), Track) != Only->end(); };
	// one modifier per path: the formation under its axis and the slopes from its edges to the ground. laid over each other the
	// modifiers keep the highest of the slopes going down and the lowest of those going up, so tracks side by side share one
	// formation instead of a saw of their own ones
	// a path broken by a bridge, as over another track, gets a modifier for each piece on the ground
	std::map<std::tuple<TTrack *, int, int>, heightmap::modifier> modifiers;
	std::map<std::pair<TTrack *, int>, int> pieces;
	double constexpr step{5.0};
	std::pair<TTrack *, int> previous{nullptr, -1};
	bool broken{false};
	for (auto const &place : nmt_track_places(step))
	{
		if (false == wanted(place.track))
			continue;
		std::pair<TTrack *, int> const path{place.track, place.path};
		if (path != previous)
			broken = false;
		previous = path;
		if (false == place.shapes)
		{
			broken = true;
			continue;
		}
		auto &piece{pieces[path]};
		if (broken)
		{
			++piece;
			broken = false;
		}
		auto &modifier{modifiers[{place.track, place.path, piece}]};
		if (modifier.name.empty())
		{
			modifier.name = "nmt " + (place.track->name().empty() ? std::string{"track"} : place.track->name()) + (place.track->m_paths.size() > 1 ? " " + std::to_string(place.path) : std::string{}) + (piece > 0 ? " #" + std::to_string(piece) : std::string{});
			modifier.slope = state.slope;
			modifier.reach = state.reach;
		}
		modifier.points.push_back({{place.position.x, place.position.z}, place.formation, place.half_width});
	}
	std::vector<heightmap::modifier> laid;
	for (auto &[path, modifier] : modifiers)
		if (modifier.points.size() >= 2)
			laid.push_back(std::move(modifier));
	if (Only != nullptr)
	{
		// the modifiers the tracks don't have any more: of the pieces of a path gone, of the tracks removed
		std::vector<std::string> gone;
		for (auto const *track : *Only)
		{
			if (track == nullptr || track->name().empty())
				continue;
			auto const own{"nmt " + track->name()};
			for (auto const &modifier : m_streamer.modifiers())
				if ((modifier.name == own || modifier.name.starts_with(own + " ")) && std::none_of(laid.begin(), laid.end(), [&](heightmap::modifier const &Laid) { return Laid.name == modifier.name; }))
					gone.push_back(modifier.name);
		}
		for (auto const &name : gone)
			m_streamer.remove_modifier(name);
	}
	auto const count{laid.size()};
	m_streamer.modifiers(std::move(laid));
	forget_ground();
	if (Only == nullptr)
		state.shaped = modifiers.size();
	return count;
}

void editor_mode::nmt_update()
{
	auto &state{m_nmt_state};
	nmt_offer();
	if (state.stage != nmt_state::automatic::idle)
	{
		nmt_auto_step();
		return;
	}
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
				state.status += format(STR_P("; the formation of %zu path shaped", "; the formation of %zu paths shaped", state.shaped), state.shaped);
			}
			// the photos laid onto the new ground
			m_orthophoto.refit();
			first_step_done(first_step::nmt);
		}
	}
	if (auto const error{m_nmt.error()}; false == error.empty())
		state.error = STR(error);
}

void editor_mode::render_nmt_ui()
{
	auto &state{m_nmt_state};
	nmt_locate();
	auto const busy{state.fitting || state.stage != nmt_state::automatic::idle};
	step_heading(1, STR_C("Location"));
	if (false == m_nmt.placed() || state.moving)
	{
		// where the scenery is, right here: the terrain model and the orthophoto take it
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", STR_C("Where the point 0,0 of the scenery lies on the map:"));
		ImGui::PopTextWrapPos();
		ImGui::SetNextItemWidth(120.0f);
		ImGui::InputDouble("X##nmtplacenorth", &state.north, 0.0, 0.0, "%.0f");
		item_tooltip("X of the point 0,0 in the PUWG 1992 coordinates of the Polish maps, metres to the north, as geoportal.gov.pl gives it");
		ImGui::SameLine();
		ImGui::SetNextItemWidth(120.0f);
		ImGui::InputDouble("Y##nmtplaceeast", &state.east, 0.0, 0.0, "%.0f");
		item_tooltip("Y of the point 0,0 in the PUWG 1992 coordinates of the Polish maps, metres to the east, as geoportal.gov.pl gives it");
		ImGui::BeginDisabled(state.north < 100000.0 || state.east < 100000.0);
		if (ImGui::Button(STR_C("The scenery is there")))
		{
			nmt_place_scenery();
			state.moving = false;
		}
		ImGui::EndDisabled();
		if (state.moving)
		{
			ImGui::SameLine();
			if (ImGui::Button(STR_C("Cancel")))
				state.moving = false;
		}
		if (false == m_nmt.placed())
			return;
	}
	else
	{
		auto const &config{m_orthophoto.settings()};
		ImGui::Text(STR_C("N %.0f, E %.0f"), config.north, config.east);
		ImGui::SameLine();
		ImGui::TextDisabled("%s", STR_C("PUWG 1992"));
		ImGui::SameLine();
		ImGui::BeginDisabled(busy);
		if (ImGui::SmallButton(STR_C("Change...")))
		{
			state.north = config.north;
			state.east = config.east;
			state.moving = true;
		}
		ImGui::EndDisabled();
		item_tooltip("The place is shared with the orthophoto");
	}

	auto const tracks{std::any_of(simulation::Paths.sequence().begin(), simulation::Paths.sequence().end(), [](TTrack const *Track) { return Track != nullptr && rail(*Track); })};
	step_heading(2, tracks ? STR_C("What gives way") : STR_C("Terrain"));
	if (tracks)
	{
		auto const spacing{ImGui::GetStyle().ItemSpacing.x};
		auto const width{std::floor((ImGui::GetContentRegionAvail().x - spacing) * 0.5f)};
		char const *const titles[]{STR_C("Tracks onto the terrain"), STR_C("Terrain under the tracks")};
		char const *const texts[]{STR_C("The tracks go down or up onto the real ground: grade lines fitted to it within the limits, switches level with their tracks"),
		                          STR_C("The tracks stay where they are; the real ground is drawn to them and keeps its relief farther away")};
		auto const wrap{width - 16.0f};
		auto const textheight{std::max(ImGui::CalcTextSize(texts[0], nullptr, false, wrap).y, ImGui::CalcTextSize(texts[1], nullptr, false, wrap).y)};
		for (int index = 0; index < 2; ++index)
		{
			if (index > 0)
				ImGui::SameLine();
			if (nmt_choice_card(index, titles[index], texts[index], width, textheight, state.mode == index) && false == busy)
				state.mode = index;
		}
	}
	else
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", STR_C("No tracks yet: the terrain is made around the camera, the tracks laid then go along it"));
		ImGui::PopTextWrapPos();
	}

	step_heading(3, STR_C("Load"));
	if (tracks && m_streamer.resident() > 0)
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.75f, 0.3f, 1.0f));
		ImGui::TextWrapped("%s", STR_C("The scenery has terrain already: along the tracks it's made anew from the model, its painting too"));
		ImGui::PopStyleColor();
	}
	ImGui::BeginDisabled(busy);
	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.45f, 0.75f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.26f, 0.55f, 0.88f, 1.0f));
	char const *label{tracks ? (state.mode == 0 ? STR_C("Load the terrain, put the tracks onto it") : STR_C("Load the terrain under the tracks")) : STR_C("Load the terrain around the camera")};
	if (ImGui::Button(label, ImVec2(-1.0f, ImGui::GetFrameHeight() * 1.5f)))
	{
		if (false == tracks)
		{
			state.offset = 0.0;
			nmt_make(nmt_keys_around());
		}
		else if (state.mode == 0)
			nmt_auto_start();
		else
			nmt_under_start();
	}
	ImGui::PopStyleColor(2);
	ImGui::EndDisabled();
	if (ImGui::IsItemHovered())
	{
		auto const *text{tracks ? (state.mode == 0 ? STR_C("The terrain model is loaded along the tracks and the tracks go onto it, wherever the scenery was built: the top of the ballast at the height of the model, the grade line of every line fitted to it within the limits of the line, switches and the tracks beside them on one grade line, the formation and the slopes made along them. Where the model is off the tracks keep their line and it's reported") :
		                                             STR_C("The tracks aren't touched: the terrain model is loaded along them and drawn to them, at each place by as much as the tracks lie above or below it there, fading out away from them, so its relief stays and the tracks don't end up in deep cuttings; then the formation and the slopes along them. For a scenery built level or at its own heights")) :
		                           STR_C("Loads the terrain model, the real relief of the ground, around the camera")};
		ImGui::SetTooltip("%s\n%s", text, STR_C("Terrain model: GUGiK, geoportal.gov.pl"));
	}
	// how far it got
	if (state.stage == nmt_state::automatic::fetching || state.fitting)
	{
		auto const waiting{m_nmt.waiting()};
		ImGui::ProgressBar(-1.0f * static_cast<float>(ImGui::GetTime()), ImVec2(-1.0f, 0.0f), format(STR_P("Terrain model: %zu square left", "Terrain model: %zu squares left", waiting), waiting).c_str());
	}
	else if (state.stage == nmt_state::automatic::fitting && state.tracks > 0)
	{
		auto const fraction{1.0f - static_cast<float>(state.left.size()) / static_cast<float>(std::max<std::size_t>(state.tracks, state.left.size()))};
		ImGui::ProgressBar(fraction, ImVec2(-1.0f, 0.0f), format(STR_P("Grade lines: %zu fitted, %zu track left", "Grade lines: %zu fitted, %zu tracks left", state.left.size()), state.lines, state.left.size()).c_str());
	}
	else if (false == state.pending.empty())
	{
		auto const total{state.made + state.pending.size()};
		ImGui::ProgressBar(static_cast<float>(state.made) / static_cast<float>(std::max<std::size_t>(1, total)), ImVec2(-1.0f, 0.0f), format(STR_C("Chunks: %zu of %zu"), state.made, total).c_str());
	}
	if (false == state.status.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", state.status.c_str());
		ImGui::PopTextWrapPos();
	}
	if (false == state.error.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", state.error.c_str());
		ImGui::PopTextWrapPos();
	}
	ImGui::PushID("nmtmore");
	if (tracks && ImGui::TreeNode(STR_C("More settings")))
	{
		if (begin_fields("##nmtband", 0.5f))
		{
			field_row("Band along the tracks (m)");
			ImGui::InputDouble("##nmtbandwidth", &state.band, 50.0, 250.0, "%.0f");
			item_tooltip("How far from the tracks, on both sides, the terrain model is loaded");
			ImGui::EndTable();
		}
		toggle_chips("nmtshape", {{STR_C("Formation and slopes along the tracks"), &state.shape,
		                           "Each track gets its formation with slopes 1:1.5 down or up to the terrain, as a modifier of the terrain; bridges and tunnels keep the model under them"}});
		ImGui::TreePop();
	}
	ImGui::PopID();
	state.band = std::clamp(state.band, 50.0, 3000.0);
	if (tracks && ImGui::TreeNode(STR_C("By hand: the model as it is, the tracks stay")))
	{
		if (begin_fields("##nmtoffset", 0.5f))
		{
			field_row("Height offset (m)");
			ImGui::InputDouble("##nmtheightoffset", &state.offset, 0.1, 1.0, "%.2f");
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(1);
			if (ImGui::Button(STR_C("Fit to the tracks"), ImVec2(-FLT_MIN, 0.0f)))
				nmt_fit();
			item_tooltip("Compares the formation of the tracks with the terrain model and moves the model up or down to meet them, as a scenery built at height 0 needs");
			ImGui::EndTable();
		}
		auto const half{std::floor((ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f)};
		ImGui::BeginDisabled(busy);
		if (ImGui::Button(STR_C("Load along the tracks"), ImVec2(half, 0.0f)))
			nmt_make(nmt_keys_along_tracks());
		item_tooltip("Loads the terrain model as it is along the tracks, with the height offset above; the tracks stay where they are");
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Load around the camera"), ImVec2(-FLT_MIN, 0.0f)))
			nmt_make(nmt_keys_around());
		item_tooltip("Loads the terrain model as it is around the camera, with the height offset above; the tracks stay where they are");
		ImGui::EndDisabled();
		ImGui::TreePop();
	}
}

bool editor_mode::nmt_choice_card(int const Index, char const *Title, char const *Text, float const Width, float const TextHeight, bool const Chosen) const
{
	auto const line{ImGui::GetTextLineHeight()};
	auto const picture{std::round(line * 3.2f)};
	auto const pad{8.0f};
	ImVec2 const size{Width, pad + picture + pad * 0.5f + line + 4.0f + TextHeight + pad};
	auto const at{ImGui::GetCursorScreenPos()};
	ImGui::PushID(Index);
	auto const clicked{ImGui::InvisibleButton("##nmtcard", size)};
	ImGui::PopID();
	auto const hovered{ImGui::IsItemHovered()};
	auto *draw{ImGui::GetWindowDrawList()};
	ImVec2 const end{at.x + size.x, at.y + size.y};
	draw->AddRectFilled(at, end, ImGui::GetColorU32(Chosen ? ImGuiCol_Header : hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), 6.0f);
	draw->AddRect(at, end, Chosen ? ImGui::GetColorU32(ImGuiCol_CheckMark) : ImGui::GetColorU32(ImGuiCol_Border), 6.0f, 0, Chosen ? 2.0f : 1.0f);
	// the cross-section: the ground and the track over it, the one giving way moving
	ImVec2 const low{at.x + pad, at.y + pad};
	ImVec2 const high{end.x - pad, at.y + pad + picture};
	draw->AddRectFilled(low, high, IM_COL32(18, 22, 30, 255), 4.0f);
	draw->PushClipRect(low, high, true);
	auto const phase{0.5f - 0.5f * std::cos(static_cast<float>(ImGui::GetTime()) * 1.6f)};
	auto const amount{Chosen || hovered ? phase : 1.0f};
	auto const steps{48};
	auto const ground = [&](float const U) { return 0.62f + 0.22f * std::sin(U * 6.2f + 0.6f) * std::sin(U * 2.3f + 1.1f); };
	auto const level{0.4f};
	std::vector<ImVec2> terrain, rails;
	for (int i = 0; i <= steps; ++i)
	{
		auto const u{static_cast<float>(i) / steps};
		auto const x{low.x + (high.x - low.x) * u};
		auto const closeness{std::clamp(1.0f - std::abs(u - 0.5f) * 2.4f, 0.0f, 1.0f)};
		float soil{ground(u)};
		float track{level};
		if (Index == 0)
			track = level + (ground(u) - 0.1f - level) * amount;
		else
			soil = ground(u) + (level + 0.1f - ground(u)) * amount * (0.35f + 0.65f * closeness);
		terrain.emplace_back(x, low.y + (high.y - low.y) * soil);
		rails.emplace_back(x, low.y + (high.y - low.y) * track);
	}
	for (int i = 0; i < steps; ++i)
		draw->AddQuadFilled(terrain[i], terrain[i + 1], ImVec2(terrain[i + 1].x, high.y), ImVec2(terrain[i].x, high.y), IM_COL32(120, 85, 50, 255));
	draw->AddPolyline(terrain.data(), static_cast<int>(terrain.size()), IM_COL32(170, 120, 70, 255), 0, 2.0f);
	draw->AddPolyline(rails.data(), static_cast<int>(rails.size()), IM_COL32(215, 220, 230, 255), 0, 3.0f);
	for (int i = 2; i < steps; i += 4)
		draw->AddLine(ImVec2(rails[i].x, rails[i].y - 3.0f), ImVec2(rails[i].x, rails[i].y + 3.0f), IM_COL32(150, 120, 90, 255), 2.0f);
	draw->PopClipRect();
	auto *font{ImGui::GetFont()};
	auto const titley{high.y + pad * 0.5f};
	draw->AddText(font, ImGui::GetFontSize(), ImVec2(low.x, titley), ImGui::GetColorU32(Chosen ? ImGuiCol_Text : ImGuiCol_Text, Chosen ? 1.0f : 0.85f), Title);
	if (Chosen)
		draw->AddText(font, ImGui::GetFontSize(), ImVec2(low.x + 0.6f, titley), ImGui::GetColorU32(ImGuiCol_Text), Title);
	draw->AddText(font, ImGui::GetFontSize(), ImVec2(low.x, titley + line + 4.0f), ImGui::GetColorU32(ImGuiCol_TextDisabled), Text, nullptr, Width - 2.0f * pad);
	return clicked;
}

void editor_mode::nmt_place_scenery()
{
	auto &state{m_nmt_state};
	auto config{m_orthophoto.settings()};
	config.north = state.north;
	config.east = state.east;
	m_orthophoto.settings(config);
	m_orthophoto_origin_edit = {config.north, config.east};
	save_orthophoto_settings();
	nmt_locate();
}

void editor_mode::nmt_offer()
{
	auto &state{m_nmt_state};
	if (state.offered || false == m_lay.active || false == EditorSettings.terrain_offer())
		return;
	state.offered = true;
	if (m_streamer.resident() > 0 || std::any_of(simulation::Paths.sequence().begin(), simulation::Paths.sequence().end(), [](TTrack const *Track) { return Track != nullptr && rail(*Track); }))
		return;
	nmt_locate();
	state.north = m_orthophoto.settings().north;
	state.east = m_orthophoto.settings().east;
	state.prompt = true;
}

void editor_mode::nmt_accept_offer()
{
	auto &state{m_nmt_state};
	if (false == m_nmt.placed())
		nmt_place_scenery();
	// no tracks to fit to: the heights of the model as they are
	state.offset = 0.0;
	nmt_make(nmt_keys_around());
	state.prompt = false;
}

void editor_mode::render_nmt_prompt()
{
	auto &state{m_nmt_state};
	if (false == state.prompt)
		return;
	char const *const title{STR_C("Terrain for the new scenery###nmtoffer")};
	if (false == ImGui::IsPopupOpen(title))
		ImGui::OpenPopup(title);
	ImGui::SetNextWindowSize(ImVec2(520.0f * Global.ui_scale, 0.0f));
	if (false == ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_NoResize))
		return;
	ImGui::TextWrapped("%s", STR_C("The scenery has no terrain yet. Load the terrain model and the aerial photos around here? The tracks laid then go along the real ground."));
	bool const placed{m_nmt.placed()};
	if (false == placed)
	{
		ImGui::TextWrapped("%s", STR_C("Where is it? PUWG 1992 of this point, X to the north and Y to the east, as geoportal.gov.pl gives them:"));
		ImGui::SetNextItemWidth(150.0f);
		ImGui::InputDouble("X##nmtnorth", &state.north, 0.0, 0.0, "%.0f");
		ImGui::SameLine();
		ImGui::SetNextItemWidth(150.0f);
		ImGui::InputDouble("Y##nmteast", &state.east, 0.0, 0.0, "%.0f");
	}
	auto const known{placed || (state.north > 100000.0 && state.east > 100000.0)};
	ImGui::Spacing();
	auto const gap{std::round(ImGui::GetFontSize() * 0.6f)};
	auto const width{std::floor((ImGui::GetContentRegionAvail().x - gap) * 0.5f)};
	if (ground_card("##terrain", STR_C("Load the terrain"), STR_C("The terrain model and the aerial photos of geoportal.gov.pl around here; the tracks go along the real ground"), true, known, width))
	{
		nmt_accept_offer();
		ImGui::CloseCurrentPopup();
	}
	if (false == known)
		item_tooltip("Type where the scenery lies first");
	ImGui::SameLine(0.0f, gap);
	if (ground_card("##flat", STR_C("Without terrain"), STR_C("The tracks lie on the flat plane at the height 0; the terrain can be loaded later in Terrain (F5)"), false, true, width))
	{
		state.prompt = false;
		ImGui::CloseCurrentPopup();
	}
	ImGui::Spacing();
	bool ask{EditorSettings.terrain_offer()};
	if (ImGui::Checkbox(STR_C("Ask with every new scenery"), &ask))
	{
		EditorSettings.terrain_offer(ask);
		EditorSettings.save();
	}
	ImGui::EndPopup();
}

void editor_mode::nmt_lift_scenery(double const Offset)
{
	glm::dvec3 const shift{0.0, Offset, 0.0};
	// the tracks, joined again as they were
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> tracks;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
			continue;
		states.emplace_back(track, editor_track::capture(*track));
		editor_track::translate(*track, shift);
		tracks.push_back(track);
	}
	editor_track::commit(tracks);
	auto const lifted{states.size()};
	push_track_snapshot(std::move(states));
	// the includes, by the place in their parameters
	std::vector<std::pair<standing_template, template_item>> moves;
	for (auto const &standing : standing_templates([](std::string const &) { return true; }))
	{
		template_item item;
		item.file = standing.file;
		item.location = standing.location + shift;
		item.yaw = standing.yaw;
		moves.emplace_back(standing, item);
	}
	if (move_templates(moves) > 0 && m_history.size() >= 2)
		m_history.back().joined = true;
	// the models, a group as a whole
	std::set<int> groups;
	for (auto *instance : simulation::Instances.sequence())
	{
		if (instance == nullptr)
			continue;
		if (instance->group() > 1 && false == groups.insert(instance->group()).second)
			continue;
		push_snapshot(instance, EditorSnapshot::Action::Move);
		m_history.back().joined = true;
		m_editor.translate(instance, instance->location() + shift, true);
	}
	WriteLog(format("Editor: NMT - the scenery moved %+.2f m: %zu tracks, %zu includes, the models", Offset, lifted, moves.size()), logtype::generic);
}

std::size_t editor_mode::nmt_join_ends()
{
	// ends which meet in the plan, laid apart in height by hand, so the game doesn't join them, are one joint before the grade
	// lines run over them; farther apart than a track over another can be, they lie on two levels
	double constexpr kLevel{2.0}; // m
	struct end
	{
		TTrack *track;
		int path;
		bool atend;
		glm::dvec3 point;
		glm::dvec2 outward;
	};
	auto const linked = [](TTrack const &Track, int const Path, bool const Atend) {
		if (Track.eType == tt_Switch && Track.SwitchExtension)
			return (Atend ? Track.SwitchExtension->pNexts[Path & 1] : Track.SwitchExtension->pPrevs[Path & 1]) != nullptr;
		return (Atend ? Track.trNext : Track.trPrev) != nullptr;
	};
	std::map<std::pair<int, int>, std::vector<end>> cells;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track) || track->Length() < 0.01)
			continue;
		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
			for (auto const atend : {false, true})
			{
				if (linked(*track, i, atend))
					continue;
				auto const &path{track->m_paths[i]};
				auto const &point{path.points[atend ? segment_data::point::end : segment_data::point::start]};
				auto const &control{path.points[atend ? segment_data::point::control2 : segment_data::point::control1]};
				auto const &other{path.points[atend ? segment_data::point::start : segment_data::point::end]};
				auto const outward{geometry::plan_of(control != glm::dvec3{} ? -control : point - other)};
				cells[{static_cast<int>(std::floor(point.x)), static_cast<int>(std::floor(point.z))}].push_back({track, i, atend, point, outward});
			}
	}
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> changed;
	std::set<std::tuple<TTrack const *, int, bool>> taken;
	auto const remember = [&](TTrack *Track) {
		if (std::find(changed.begin(), changed.end(), Track) != changed.end())
			return;
		states.emplace_back(Track, editor_track::capture(*Track));
		changed.push_back(Track);
	};
	std::size_t joined{0};
	for (auto const &[cell, ends] : cells)
		for (auto const &a : ends)
		{
			if (a.track->eType != tt_Normal || taken.contains({a.track, a.path, a.atend}))
				continue;
			end const *match{nullptr};
			for (int dx = -1; dx <= 1 && match == nullptr; ++dx)
				for (int dz = -1; dz <= 1 && match == nullptr; ++dz)
				{
					auto const found{cells.find({cell.first + dx, cell.second + dz})};
					if (found == cells.end())
						continue;
					for (auto const &b : found->second)
						if (b.track != a.track && false == taken.contains({b.track, b.path, b.atend}) && geometry::plan_distance(a.point, b.point) <= geometry::same_point &&
						    std::abs(a.point.y - b.point.y) <= kLevel && glm::dot(a.outward, b.outward) < -0.5 * glm::length(a.outward) * glm::length(b.outward))
						{
							match = &b;
							break;
						}
				}
			if (match == nullptr || (match->track->eType != tt_Normal && false == editor_track::is_supported(*match->track)))
				continue;
			auto const height{match->track->eType == tt_Normal ? (a.point.y + match->point.y) * 0.5 : match->point.y};
			for (auto const *side : {&a, match})
			{
				taken.insert({side->track, side->path, side->atend});
				if (side->track->eType != tt_Normal)
					continue;
				remember(side->track);
				side->track->m_paths[side->path].points[side->atend ? segment_data::point::end : segment_data::point::start].y = height;
			}
			++joined;
		}
	if (changed.empty())
		return 0;
	editor_track::commit(changed);
	push_track_snapshot(std::move(states));
	if (m_history.size() >= 2 && m_nmt_state.lifted != 0.0)
		m_history.back().joined = true;
	WriteLog(format("Editor: NMT - %zu joints of tracks laid apart in height joined", joined), logtype::generic);
	return joined;
}

void editor_mode::nmt_auto_start()
{
	auto &state{m_nmt_state};
	state.under = false;
	nmt_locate();
	state.probes.clear();
	for (auto const &place : nmt_track_places(20.0))
		if (place.shapes)
			state.probes.push_back(place);
	if (state.probes.empty())
	{
		state.status = STR("No tracks to put onto the terrain");
		return;
	}
	state.lines = 0;
	state.refused = 0;
	state.departures = 0;
	state.lifted = 0.0;
	state.stage = nmt_state::automatic::fetching;
	state.status = STR("Fetching the terrain model under the tracks...");
}

bool editor_mode::nmt_fit_line(TTrack &Track)
{
	auto &profile{m_profile};
	auto const &done{m_nmt_state.done};
	auto const fitted = [&](TTrack const *Other) { return std::find(done.begin(), done.end(), Other) != done.end(); };
	// the line runs over the tracks not fitted yet around the track: the ones fitted stay as they are
	auto route{editor_track::run_route(Track, profile.run_length)};
	auto const at{std::find_if(route.spans.begin(), route.spans.end(), [&](auto const &Span) { return Span.track == &Track; })};
	if (at == route.spans.end())
		return false;
	auto first{at};
	while (first != route.spans.begin() && false == fitted(std::prev(first)->track))
		--first;
	auto last{at};
	while (std::next(last) != route.spans.end() && false == fitted(std::next(last)->track))
		++last;
	route.spans = {first, std::next(last)};
	profile.open = true;
	profile.status.clear();
	profile.error.clear();
	auto *from{route.spans.front().track};
	auto *to{route.spans.back().track};
	profile_take(std::move(route), from, to);
	if (false == profile.error.empty() || profile.samples.empty())
		return false;
	// an end which goes on into a line fitted already keeps its height and grade; any other end is free, the tracks past it are
	// fitted later and meet this line then
	auto const joint = [&](bool const Atend, double &Height) {
		double grade;
		TTrack *adjoining{nullptr};
		int path{0};
		if (false == editor_track::adjoining_grade(profile.route, Atend, grade, &adjoining, &path) || adjoining == nullptr || false == fitted(adjoining))
			return false;
		auto const &end{Atend ? profile.samples.back().position : profile.samples.front().position};
		auto const &points{adjoining->m_paths[path].points};
		auto const &start{points[segment_data::point::start]};
		auto const &finish{points[segment_data::point::end]};
		Height = geometry::plan_distance(start, end) < geometry::plan_distance(finish, end) ? start.y : finish.y;
		return true;
	};
	profile.context.start_joined = profile.context.start_joined && joint(false, profile.context.start_elevation);
	profile.context.end_joined = profile.context.end_joined && joint(true, profile.context.end_elevation);
	// the ground of the line: the terrain model on its axis, the top of the ballast with the rails on it, and the rail tops of
	// the lines fitted already beside it on the same level as the model here, which tell its height the more closely the
	// nearer they are. What doesn't agree with the line, a bridge over it, the ground under a viaduct, the fit lets go of by
	// itself
	double constexpr kModel{0.25}; // m, the tolerance of the model
	double constexpr kRail{0.02}; // m, of a rail top next to the line, growing by kSpread a metre of the distance
	double constexpr kSpread{0.01};
	double constexpr kLevel{0.5}; // m, a rail top farther from the model or from the line lies on another level
	double constexpr kReach{50.0};
	double constexpr kSteepest{40.0}; // per mille, the steepest grade the ground may give a line
	double constexpr kCell{50.0};
	auto const nearest = [&](glm::dvec3 const &Point, double &Distance) {
		Distance = std::numeric_limits<double>::max();
		double height{0.0};
		auto const cx{static_cast<int>(std::floor(Point.x / kCell))};
		auto const cz{static_cast<int>(std::floor(Point.z / kCell))};
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz)
			{
				auto const cell{m_nmt_state.laid.find({cx + dx, cz + dz})};
				if (cell == m_nmt_state.laid.end())
					continue;
				for (auto const &laid : cell->second)
					if (auto const distance{glm::length(glm::dvec2{laid.x - Point.x, laid.z - Point.z})}; distance < Distance)
					{
						Distance = distance;
						height = laid.y;
					}
			}
		return height;
	};
	// an end of the line, or the other path of a switch of it, which leads through the tracks not fitted yet into a line fitted
	// already, as at a crossover, ties the two where it leaves the line: there the line keeps to the height of the other, on
	// along its grade over the tracks between, give or take what these can climb with curves of the least radius at both ends
	double constexpr kJoin{0.002}; // m
	double constexpr kFarthest{200.0};
	profile::ground_fit options;
	auto const length_of = [](TTrack const &Track, int const Path) { return Track.eType == tt_Switch ? Track.SwitchExtension->Segments[Path]->GetLength() : Track.Segment->GetLength(); };
	auto const on_route = [&](TTrack const *Other) { return std::any_of(profile.route.spans.begin(), profile.route.spans.end(), [&](auto const &Span) { return Span.track == Other; }); };
	std::vector<editor_track::route_span> exits;
	exits.push_back({profile.route.spans.front().track, profile.route.spans.front().path, false == profile.route.spans.front().forward});
	exits.push_back({profile.route.spans.back().track, profile.route.spans.back().path, profile.route.spans.back().forward});
	for (auto const &span : profile.route.spans)
		if (span.track->eType == tt_Switch && span.track->m_paths.size() >= 2)
			for (auto const outward : {true, false})
				exits.push_back({span.track, 1 - span.path, outward});
	for (auto const &exiting : exits)
	{
		auto const &exit{exiting.track->m_paths[exiting.path].points[exiting.forward ? segment_data::point::end : segment_data::point::start]};
		editor_track::route step;
		step.spans.push_back(exiting);
		double between{0.0};
		for (int hops = 0; hops < 50 && between < kFarthest; ++hops)
		{
			double grade;
			TTrack *next{nullptr};
			int path{0};
			bool forward{true};
			if (false == editor_track::adjoining_grade(step, true, grade, &next, &path, &forward) || next == nullptr || on_route(next))
				break;
			if (fitted(next))
			{
				std::size_t nearest{0};
				for (std::size_t i = 1; i < profile.samples.size(); ++i)
					if (geometry::plan_distance(profile.samples[i].position, exit) < geometry::plan_distance(profile.samples[nearest].position, exit))
						nearest = i;
				auto const &points{next->m_paths[path].points};
				auto const height{points[forward ? segment_data::point::start : segment_data::point::end].y - grade * between};
				auto const leeway{kJoin + between * between / (4.0 * profile.line.norms.radius_min)};
				options.bounds.push_back({profile.samples[nearest].chainage, height - leeway, height + leeway});
				break;
			}
			step.spans = {{next, path, forward}};
			between += length_of(*next, path);
		}
	}
	auto const count{profile.samples.size()};
	std::vector<double> sampled(count), model(count, std::numeric_limits<double>::quiet_NaN());
	std::vector<double> chainages, ground, tolerances, reaches;
	for (std::size_t i = 0; i < count; ++i)
	{
		auto const &sample{profile.samples[i]};
		auto const *track{sample.span < profile.route.spans.size() ? profile.route.spans[sample.span].track : nullptr};
		sampled[i] = sample.chainage;
		if (double height; track != nullptr && m_nmt.height(sample.position.x, sample.position.z, height))
			model[i] = height + m_nmt_state.offset + track->rail_height();
		chainages.push_back(sample.chainage);
		ground.push_back(model[i]);
		tolerances.push_back(kModel);
		reaches.push_back(kModel);
		double distance;
		auto const beside{nearest(sample.position, distance)};
		if (distance < kReach && (false == std::isfinite(model[i]) || std::abs(beside - model[i]) <= kLevel))
		{
			chainages.push_back(sample.chainage);
			ground.push_back(beside);
			tolerances.push_back(std::max(kRail, kSpread * distance));
			reaches.push_back(kLevel);
		}
	}
	if (std::none_of(ground.begin(), ground.end(), [](double const Height) { return std::isfinite(Height); }))
		return false;
	// the switches of the tracks beside, the ones of the crossovers above all, lie on bearers common with this track or close
	// to it: no break of the grade on them either, so the tracks of the station stay in one plane
	for (auto *other : simulation::Paths.sequence())
	{
		if (other == nullptr || other->eType != tt_Switch || other->m_paths.empty() || false == rail(*other))
			continue;
		if (std::any_of(profile.route.spans.begin(), profile.route.spans.end(), [&](auto const &Span) { return Span.track == other; }))
			continue;
		auto const &points{other->m_paths.front().points};
		double from{std::numeric_limits<double>::max()};
		double to{std::numeric_limits<double>::lowest()};
		for (auto const end : {segment_data::point::start, segment_data::point::end})
		{
			auto const &point{points[end]};
			double best{25.0};
			double chainage{0.0};
			for (auto const &sample : profile.samples)
				if (auto const distance{glm::length(glm::dvec2{sample.position.x - point.x, sample.position.z - point.z})}; distance < best)
				{
					best = distance;
					chainage = sample.chainage;
				}
			if (best < 25.0)
			{
				from = std::min(from, chainage);
				to = std::max(to, chainage);
			}
		}
		if (from < to)
			profile.context.switches.push_back({from, to, other->name()});
	}
	std::sort(profile.context.switches.begin(), profile.context.switches.end(), [](auto const &Left, auto const &Right) { return Left.from < Right.from; });
	auto settings{profile.line};
	settings.join_start = true;
	settings.join_end = true;
	settings.speed = 0.0;
	for (auto const &span : profile.route.spans)
		settings.speed = std::max(settings.speed, editor_track::velocity(*span.track));
	if (settings.speed <= 0.0)
		settings.speed = 100.0;
	options.tolerance = kModel;
	options.grade_max = kSteepest;
	auto line{profile::fit_ground(chainages, ground, profile.context, settings, options, tolerances, reaches)};
	if (line.points.size() < 2)
		return false;
	profile.line = std::move(line);
	profile.selected = -1;
	profile_check();
	auto const fitted_line{profile.line};
	profile_apply();
	auto const applied{profile.error.empty()};
	// where the line parts from the model: under it a gap, a bridge or a viaduct, over it a wall or a structure above the track
	auto const parted{profile::departures(fitted_line, sampled, model, 2.0 * kModel)};
	if (false == parted.empty())
		++m_nmt_state.departures;
	std::size_t agreeing{0};
	std::size_t modelled{0};
	for (std::size_t i = 0; i < count; ++i)
		if (std::isfinite(model[i]))
		{
			++modelled;
			agreeing += std::abs(profile::elevation(fitted_line, sampled[i]) - model[i]) <= kModel ? 1 : 0;
		}
	// how soon past a switch or its shared timbers the grade changes, the nearest break or start of a curve of the line
	if (applied)
	{
		auto const breaks{profile::breaks(profile.line)};
		double nearest{std::numeric_limits<double>::max()};
		std::string where;
		for (auto const &zone : profile.context.switches)
			for (auto const at : breaks)
			{
				auto const gap{at >= zone.to ? at - zone.to : at <= zone.from ? zone.from - at : -1.0};
				if (gap >= 0.0 && gap < nearest)
				{
					nearest = gap;
					where = format("%.1f m %s %s (%.1f..%.1f), the break at %.1f", gap, at >= zone.to ? "past" : "before", zone.name.c_str(), zone.from, zone.to, at);
				}
			}
		if (false == where.empty())
			WriteLog(format("Editor: NMT line %s .. %s: the grade changes %s", from->name().c_str(), to->name().c_str(), where.c_str()), logtype::generic);
	}
	// where the paths didn't take the line
	double off{0.0};
	std::size_t off_at{0};
	if (applied)
		for (std::size_t i = 0; i < profile.samples.size(); ++i)
			if (auto const departure{std::abs(profile.samples[i].position.y - profile::elevation(profile.line, profile.samples[i].chainage))}; departure > off)
			{
				off = departure;
				off_at = i;
			}
	if (off > 0.05)
	{
		auto const &sample{profile.samples[off_at]};
		auto const *track{profile.route.spans[sample.span].track};
		WriteLog(format("Editor: NMT line %s: the path %s (%s, %.0f m) is off the line by %.3f m at %.0f m", Track.name().c_str(), track->name().c_str(), track->eType == tt_Switch ? "switch" : "track", track->Length(), off, sample.chainage), logtype::generic);
	}
	if (applied)
		for (auto const &sample : profile.samples)
			m_nmt_state.laid[{static_cast<int>(std::floor(sample.position.x / kCell)), static_cast<int>(std::floor(sample.position.z / kCell))}].push_back(sample.position);
	// the line in the log: how much of it lies on the model, where it parts from it, its grades, why it wasn't applied, its
	// course in the plan
	std::string text{format("Editor: NMT line %s: %s .. %s, %.0f m, on the model within %.2f m over %.1f%%, parts from it", applied ? "applied" : "refused", from->name().c_str(), to->name().c_str(), profile.route.length, kModel,
	                        100.0 * static_cast<double>(agreeing) / static_cast<double>(std::max<std::size_t>(1, modelled)))};
	for (auto const &stretch : parted)
		text += format(" %s %.0f..%.0f %.1f m", stretch.depth < 0.0 ? "under" : "over", stretch.from, stretch.to, std::abs(stretch.depth));
	text += ", grades";
	for (std::size_t i = 1; i < fitted_line.points.size(); ++i)
	{
		auto const &a{fitted_line.points[i - 1]};
		auto const &b{fitted_line.points[i]};
		text += format(" %+.2f/%.0f", 1000.0 * (b.elevation - a.elevation) / std::max(1e-6, b.chainage - a.chainage), b.chainage - a.chainage);
	}
	if (false == applied)
	{
		text += "; " + profile.error;
		for (auto const &issue : profile.issues)
			if (issue.error)
				text += " | " + issue.text;
	}
	text += "; plan";
	for (std::size_t i = 0; i < profile.samples.size(); i += 10)
		text += format(" %.1f,%.1f", profile.samples[i].position.x, profile.samples[i].position.z);
	text += format(" %.1f,%.1f", profile.samples.back().position.x, profile.samples.back().position.z);
	WriteLog(text, logtype::generic);
	return applied;
}

void editor_mode::nmt_auto_step()
{
	auto &state{m_nmt_state};
	if (state.stage == nmt_state::automatic::fetching)
	{
		bool all{true};
		for (auto const &place : state.probes)
			all = m_nmt.request(glm::dvec2{place.position.x, place.position.z} - glm::dvec2{1.0}, glm::dvec2{place.position.x, place.position.z} + glm::dvec2{1.0}) && all;
		auto const waiting{m_nmt.waiting()};
		state.status = format(STR_P("Fetching the terrain model under the tracks: %zu square left", "Fetching the terrain model under the tracks: %zu squares left", waiting), waiting);
		if (false == all)
			return;
		// the scenery goes to the height of the model as a whole first, so the lines fitted one after another meet at heights near
		// their own; the grade lines then follow the model
		std::vector<double> differences;
		for (auto const &place : state.probes)
			if (double height; m_nmt.height(place.position.x, place.position.z, height))
				differences.push_back(height - place.crown);
		if (differences.empty())
		{
			state.stage = nmt_state::automatic::idle;
			state.status = STR("The terrain model has no data under the tracks: is the scenery placed where it is?");
			return;
		}
		std::nth_element(differences.begin(), differences.begin() + differences.size() / 2, differences.end());
		auto const lift{differences[differences.size() / 2]};
		// tracks of files that aren't saved, as of shared includes, would stay where they were: the model comes to the scenery
		std::vector<TTrack *> scenery;
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && false == track->m_editorremoved && false == track->m_paths.empty())
				scenery.push_back(track);
		std::string reason;
		state.lifted = 0.0;
		state.offset = 0.0;
		if (std::abs(lift) > 0.05 && tracks_editable(scenery, reason))
		{
			nmt_lift_scenery(lift);
			state.lifted = lift;
		}
		else if (std::abs(lift) > 0.05)
		{
			state.offset = -lift;
			WriteLog(format("Editor: NMT - the terrain model offset %+.2f m to the scenery, which can't be moved: ", -lift) + reason, logtype::generic);
		}
		state.fitted = true;
		nmt_join_ends();
		// the longest tracks first: the main lines lead, the sidings join them
		state.left.clear();
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && rail(*track))
				state.left.push_back(track);
		std::sort(state.left.begin(), state.left.end(), [](TTrack *Left, TTrack *Right) { return Left->Length() > Right->Length(); });
		state.tracks = state.left.size();
		state.split = 0;
		state.done.clear();
		state.laid.clear();
		state.stage = nmt_state::automatic::fitting;
		return;
	}
	// a few lines a frame
	for (int line = 0; line < 2 && false == state.left.empty(); ++line)
	{
		auto *track{state.left.front()};
		auto const name{track->name()};
		auto const known{simulation::Paths.sequence().size()};
		if (nmt_fit_line(*track))
			++state.lines;
		else
			++state.refused;
		// the tracks of the line are done, whether it went or not; so is every path made meanwhile, split off by the grade line or
		// by the lines following it over the crossovers: the next line over them would lay them again, and split them again
		std::set<TTrack const *> done{track};
		state.done.push_back(track);
		for (auto const &span : m_profile.route.spans)
			if (done.insert(span.track).second)
				state.done.push_back(span.track);
		auto const &sequence{simulation::Paths.sequence()};
		std::string made;
		for (auto i{known}; i < sequence.size(); ++i)
			if (auto *piece{sequence[i]}; piece != nullptr && done.insert(piece).second)
			{
				state.done.push_back(piece);
				if (made.size() < 300)
					made += ' ' + piece->name();
			}
		auto const added{sequence.size() - known};
		state.split += added;
		if (added > 20)
			WriteLog(format("Editor: NMT line %s made %zu paths:%s", name.c_str(), added, made.c_str()), logtype::generic);
		if (state.split > state.tracks / 2 + 200)
		{
			// something lays the same tracks over and over: stopped before it takes all the memory
			state.left.clear();
			state.error = format(STR_P("Stopped: the grade lines split off %zu path from %zu tracks, more than they should; Ctrl+Z takes the changes back", "Stopped: the grade lines split off %zu paths from %zu tracks, more than they should; Ctrl+Z takes the changes back", state.split), state.split, state.tracks);
			WriteLog("Editor: NMT - " + state.error, logtype::generic);
			break;
		}
		state.left.erase(std::remove_if(state.left.begin(), state.left.end(), [&](TTrack *Track) { return done.contains(Track); }), state.left.end());
	}
	state.status = format(STR_C("Scenery moved %+.2f m; grade lines fitted to the terrain: %zu, refused by the checks: %zu, tracks left: %zu"), state.lifted, state.lines, state.refused, state.left.size());
	if (state.departures > 0)
		state.status += format(STR_P("\n%zu line parts from the terrain model by over 0.5 m: the model or the track is off there, the terrain follows the tracks (the log names it)", "\n%zu lines part from the terrain model by over 0.5 m: the model or the track is off there, the terrain follows the tracks (the log names them)", state.departures), state.departures);
	if (false == state.left.empty())
		return;
	m_profile.open = false;
	state.done.clear();
	state.laid.clear();
	state.stage = nmt_state::automatic::idle;
	WriteLog("Editor: NMT - " + state.status, logtype::generic);
	nmt_make(nmt_keys_along_tracks(), true);
}

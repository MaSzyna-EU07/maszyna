/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodekm.h"
#include "application/editorgestures.h"
#include "application/editormodeshared.h"
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorTrack.hpp"

#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace
{

using geometry::plan_distance;
using geometry::plan_of;

double constexpr kOverlayRange{3000.0};
double constexpr kTickRange{1500.0}; // m, of the hectometres shown in the scenery
double constexpr kLabelRange{700.0}; // m, of the km written there, farther ones crowd the horizon
double constexpr kDigitRange{250.0}; // m, of the hectometres written there
double constexpr kRailHead{0.18}; // m, above the axis of a path
double constexpr kPickReach{15.0}; // m from a point of the km a click on a track takes it
double constexpr kReadoutReach{30.0}; // m from the axis of a line its km is shown by the mouse
double constexpr kPostReach{60000.0}; // m along the line each way, of the posts read
double constexpr kPostCorridor{15.0}; // m from the track, posts farther away belong to another line
double constexpr kPostTolerance{10.0}; // m, of the distance between two posts against their numbers
double constexpr kCurve{10000.0}; // m, of the radius a jump of the km is on a curve under
double constexpr kStation{300.0}; // m from a switch a jump of the km is at a station

ImU32 constexpr kAxis{IM_COL32(110, 200, 255, 220)};
ImU32 constexpr kGuessed{IM_COL32(110, 200, 255, 90)};
ImU32 constexpr kTick{IM_COL32(235, 240, 250, 230)};
ImU32 constexpr kKm{IM_COL32(255, 215, 90, 255)};
ImU32 constexpr kJump{IM_COL32(255, 95, 215, 255)};
ImU32 constexpr kMark{IM_COL32(95, 220, 140, 255)};
ImU32 constexpr kAstray{IM_COL32(240, 90, 80, 255)};

// the digits after the rotation make the number of a hectometre post, the last one the hectometre
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

// chainage of the point of the samples nearest to the point, and how far from them it is
double project(std::vector<editor_track::route_sample> const &Samples, glm::dvec3 const &Point, double &Distance)
{
	Distance = std::numeric_limits<double>::max();
	double chainage{0.0};
	auto const point{plan_of(Point)};
	for (std::size_t i = 1; i < Samples.size(); ++i)
	{
		auto const a{plan_of(Samples[i - 1].position)};
		auto const ab{plan_of(Samples[i].position) - a};
		auto const length2{glm::dot(ab, ab)};
		auto const t{length2 > 1e-12 ? std::clamp(glm::dot(point - a, ab) / length2, 0.0, 1.0) : 0.0};
		if (auto const distance{glm::length(point - (a + ab * t))}; distance < Distance)
		{
			Distance = distance;
			chainage = Samples[i - 1].chainage + (Samples[i].chainage - Samples[i - 1].chainage) * t;
		}
	}
	return chainage;
}

std::size_t sample_at(std::vector<editor_track::route_sample> const &Samples, double const Chainage)
{
	auto const found{std::lower_bound(Samples.begin(), Samples.end(), Chainage, [](editor_track::route_sample const &Sample, double const Value) { return Sample.chainage < Value; })};
	return std::min<std::size_t>(static_cast<std::size_t>(found - Samples.begin()), Samples.size() - 1);
}

// the number of a line: blanks taken out
std::string line_of(std::string const &Text)
{
	std::string result;
	for (auto const c : Text)
		if (false == std::isspace(static_cast<unsigned char>(c)))
			result += c;
	return result;
}

// a km typed as PKP PLK write it, 100,234, or with a point
bool km_of(char const *Text, double &Km)
{
	std::string text{Text};
	std::replace(text.begin(), text.end(), ',', '.');
	char *end{nullptr};
	auto const value{std::strtod(text.c_str(), &end)};
	if (end == text.c_str() || false == std::isfinite(value))
		return false;
	Km = value;
	return true;
}

std::string mark_text(kilometrage::mark const &Mark)
{
	return Mark.beyond.has_value() ? kilometrage::text(Mark.km) + " = " + kilometrage::text(*Mark.beyond) : kilometrage::text(Mark.km);
}

void draw_label(ImDrawList *List, ImVec2 const At, ImU32 const Ground, ImU32 const Colour, std::string const &Text)
{
	auto const size{ImGui::CalcTextSize(Text.c_str())};
	List->AddRectFilled(ImVec2(At.x - size.x * 0.5f - 4.0f, At.y - size.y - 4.0f), ImVec2(At.x + size.x * 0.5f + 4.0f, At.y), Ground, 3.0f);
	List->AddText(ImVec2(At.x - size.x * 0.5f, At.y - size.y - 2.0f), Colour, Text.c_str());
}

void draw_arrow(screen_projection const &Projection, ImDrawList *List, glm::dvec3 const &From, glm::dvec2 const &Way, double const Length, ImU32 const Colour)
{
	glm::dvec3 const along{Way.x, 0.0, Way.y};
	glm::dvec3 const across{-Way.y, 0.0, Way.x};
	auto const tip{From + along * Length};
	Projection.line(List, From, tip, Colour, 3.0f);
	Projection.line(List, tip, tip - along * (Length * 0.3) + across * (Length * 0.15), Colour, 3.0f);
	Projection.line(List, tip, tip - along * (Length * 0.3) - across * (Length * 0.15), Colour, 3.0f);
}

} // namespace

void km_tool::refresh()
{
	m_drawn.clear();
	for (auto const &line : kilometrage::lines())
	{
		auto const *axis{kilometrage::axis_of(line)};
		if (axis == nullptr || axis->points.size() < 2)
			continue;
		drawn entry;
		entry.line = line;
		entry.points = axis->points;
		auto const low{axis->anchors.front().chainage - 1.0};
		auto const high{axis->anchors.back().chainage + 1.0};
		for (auto const chainage : axis->chainage)
			entry.guessed.push_back(chainage < low || chainage > high ? 1 : 0);
		for (auto const &anchor : axis->anchors)
			if (std::abs(anchor.after - anchor.before) >= 1e-6)
			{
				auto const way{kilometrage::direction_on(*axis, anchor.chainage)};
				entry.ticks.push_back({kilometrage::point_on(*axis, anchor.chainage), {-way.y, 0.0, way.x}, 0, true});
			}
		for (auto const &hectometre : kilometrage::hectometres(*axis))
			entry.ticks.push_back({hectometre.at, {-hectometre.rising.y, 0.0, hectometre.rising.x}, hectometre.number, false});
		m_drawn.push_back(std::move(entry));
		// the shorter track where the tracks run apart
		for (auto const &side : axis->sides)
		{
			drawn apart;
			apart.line = line;
			apart.points = side.points;
			apart.guessed.assign(side.points.size(), 0);
			m_drawn.push_back(std::move(apart));
		}
	}
	m_revision = kilometrage::revision();
	m_drawn_once = true;
}

void km_tool::update()
{
	if (false == m_open)
		return;
	auto const &marks{kilometrage::marks()};
	// after an undo the point edited may be gone
	if (m_edited >= 0 && (m_edited >= static_cast<int>(marks.size()) || false == m_pick.has_value() || plan_distance(marks[m_edited].point, m_pick->point) > 0.5))
		m_edited = -1;
	if (false == m_drawn_once || kilometrage::revision() != m_revision)
		refresh();
}

void km_tool::forget(scene::basic_node const *Node)
{
	if (m_pick.has_value() && m_pick->track == Node)
		m_pick->track = nullptr;
}

glm::dvec2 km_tool::rising() const
{
	if (false == m_pick.has_value())
		return {0.0, 1.0};
	return m_pick->along * (m_against ? -1.0 : 1.0);
}

std::string km_tool::jump_warning(glm::dvec3 const &Point) const
{
	auto *track{kilometrage::track_near(Point, 6.0)};
	if (track == nullptr)
		return {};
	auto route{editor_track::run_route(*track, kStation + 50.0)};
	auto const samples{editor_track::sample_route(route, 2.0)};
	if (samples.size() < 2)
		return {};
	double gap{0.0};
	auto const at{project(samples, Point, gap)};
	bool curve{false};
	bool station{false};
	for (auto const &sample : samples)
	{
		auto const apart{std::abs(sample.chainage - at)};
		if (apart <= 30.0 && std::abs(sample.curvature) > 1.0 / kCurve)
			curve = true;
		if (apart <= kStation && sample.span < route.spans.size() && route.spans[sample.span].track->eType != tt_Normal)
			station = true;
	}
	if (curve && station)
		return STR("Id-1 § 19: the km shouldn't jump on a curve nor at a station, move the jump to a straight on the open line");
	if (curve)
		return STR("Id-1 § 19: the km shouldn't jump on a curve, move the jump to a straight");
	if (station)
		return STR("Id-1 § 19: the km shouldn't jump at a station, move the jump out to the open line");
	return {};
}

void km_tool::pick(TTrack &Track, glm::dvec3 const &Point)
{
	auto const t{editor_track::nearest_parameter(Track, Point)};
	picked entry;
	entry.track = &Track;
	entry.point = editor_track::point_at(Track, t);
	auto const ahead{plan_of(editor_track::point_at(Track, std::min(1.0, t + 0.01)) - editor_track::point_at(Track, std::max(0.0, t - 0.01)))};
	entry.along = glm::length(ahead) > 1e-9 ? glm::normalize(ahead) : glm::dvec2{0.0, 1.0};
	entry.path = kilometrage::path_near(Track, Point, entry.on);
	m_status.clear();
	m_route_status.clear();
	m_edited = -1;
	auto const &marks{kilometrage::marks()};
	auto nearest{kPickReach};
	for (std::size_t index = 0; index < marks.size(); ++index)
		if (auto const distance{plan_distance(marks[index].point, entry.point)}; distance < nearest)
		{
			nearest = distance;
			m_edited = static_cast<int>(index);
		}
	if (m_edited >= 0)
	{
		auto const &mark{marks[m_edited]};
		entry.point = mark.point;
		m_line = mark.line;
		m_km = mark.km;
		m_jump = mark.beyond.has_value();
		m_beyond = mark.beyond.value_or(mark.km);
		m_against = glm::dot(mark.way, entry.along) < 0.0;
	}
	else
	{
		// the line the point lies on, and its km there
		if (auto const line{kilometrage::line_at(entry.point, kReadoutReach)}; false == line.empty())
			m_line = line;
		double km{0.0};
		if (false == m_line.empty() && kilometrage::km_at(m_line, entry.point, km, kReadoutReach))
		{
			m_km = std::round(km * 1000.0) / 1000.0;
			auto const *axis{kilometrage::axis_of(m_line)};
			double gap{0.0};
			auto const along{kilometrage::chainage_at(*axis, entry.point, gap)};
			m_against = glm::dot(kilometrage::direction_on(*axis, along) * static_cast<double>(axis->growth), entry.along) < 0.0;
		}
		m_jump = false;
		m_beyond = m_km;
	}
	m_pick = entry;
}

std::string km_tool::readout(glm::dvec3 const &Point) const
{
	std::string result;
	for (auto const &line : kilometrage::lines())
	{
		double km{0.0};
		if (false == kilometrage::km_at(line, Point, km, kReadoutReach))
			continue;
		if (false == result.empty())
			result += '\n';
		result += format(STR_C("line %s: km %s"), line.c_str(), kilometrage::text(km).c_str());
	}
	return result;
}

void km_tool::put()
{
	m_status.clear();
	if (false == m_pick.has_value())
		return;
	auto const line{line_of(m_line)};
	if (line.empty())
	{
		m_status = STR("Give the number of the line first");
		return;
	}
	if (m_jump && std::abs(m_beyond - m_km) < 0.0005)
	{
		m_status = STR("A jump needs another km on from the point");
		return;
	}
	kilometrage::mark entry;
	entry.line = line;
	entry.point = m_pick->point;
	entry.way = rising();
	entry.km = m_km;
	if (m_jump)
		entry.beyond = m_beyond;
	if (m_edited >= 0)
		kilometrage::change(static_cast<std::size_t>(m_edited), entry);
	else
	{
		kilometrage::put(entry);
		m_edited = static_cast<int>(kilometrage::marks().size()) - 1;
	}
	m_line = line;
	m_history.push_kilometrage_snapshot();
	m_status = format(STR_C("Line %s: km %s at the point"), line.c_str(), mark_text(entry).c_str());
}

void km_tool::remove()
{
	m_status.clear();
	if (m_edited < 0)
		return;
	kilometrage::remove(static_cast<std::size_t>(m_edited));
	m_edited = -1;
	m_history.push_kilometrage_snapshot();
	m_status = STR("The point of the km removed");
}

void km_tool::show(std::size_t const Index)
{
	auto const &marks{kilometrage::marks()};
	if (Index >= marks.size())
		return;
	auto const point{marks[Index].point};
	if (auto *track{kilometrage::track_near(point, 6.0)}; track != nullptr)
		pick(*track, point);
	m_view.fly_to(point, glm::normalize(glm::dvec3{-1.0, -0.8, -1.0}), 40.0);
}

void km_tool::show_via(std::size_t const Index)
{
	auto const &vias{kilometrage::vias()};
	if (Index >= vias.size())
		return;
	auto const point{vias[Index].point};
	auto const line{vias[Index].line};
	int path{0};
	if (auto *track{kilometrage::track_near(point, 6.0, path)}; track != nullptr)
	{
		pick(*track, point);
		m_line = line;
	}
	m_view.fly_to(point, glm::normalize(glm::dvec3{-1.0, -0.8, -1.0}), 40.0);
}

int km_tool::via_here() const
{
	if (false == m_pick.has_value() || m_pick->track == nullptr)
		return -1;
	auto const &vias{kilometrage::vias()};
	for (std::size_t index = 0; index < vias.size(); ++index)
	{
		if (vias[index].line != m_line)
			continue;
		int path{0};
		if (kilometrage::track_near(vias[index].point, 6.0, path) == m_pick->track && path == m_pick->path)
			return static_cast<int>(index);
	}
	return -1;
}

void km_tool::lead_along()
{
	m_route_status.clear();
	if (false == m_pick.has_value() || m_pick->track == nullptr)
		return;
	auto const line{line_of(m_line)};
	if (line.empty())
	{
		m_route_status = STR("Give the number of the line first");
		return;
	}
	auto const astray_of = [&]() {
		auto const *axis{kilometrage::axis_of(line)};
		return axis != nullptr ? axis->astray.size() : kilometrage::marks().size();
	};
	auto const before{kilometrage::current()};
	auto const astray{astray_of()};
	kilometrage::via entry;
	entry.line = line;
	entry.point = m_pick->on;
	kilometrage::put(entry);
	auto const *axis{kilometrage::axis_of(line)};
	if (axis == nullptr || false == axis->runs_along(m_pick->track, m_pick->path))
	{
		kilometrage::restore(before);
		m_route_status = STR("The line can't run along this track: no way leads from it onto the line");
		return;
	}
	m_line = line;
	m_history.push_kilometrage_snapshot();
	m_route_status = format(STR_C("Line %s runs along this track now"), line.c_str());
	if (auto const lost{astray_of()}; lost > astray)
		m_route_status += "\n" + format(STR_P("%zu point of the km is off the line now: the track leads it away from it", "%zu points of the km are off the line now: the track leads it away from them", lost - astray), lost - astray);
}

void km_tool::drop_via()
{
	m_route_status.clear();
	auto const index{via_here()};
	if (index < 0)
		return;
	kilometrage::remove_via(static_cast<std::size_t>(index));
	m_history.push_kilometrage_snapshot();
	m_route_status = format(STR_C("Line %s runs along the straight paths of the switches here again"), m_line.c_str());
}

void km_tool::read_posts()
{
	m_status.clear();
	auto const line{line_of(m_line)};
	if (false == m_pick.has_value() || m_pick->track == nullptr)
	{
		m_status = STR("Click a track of the line first");
		return;
	}
	if (line.empty())
	{
		m_status = STR("Give the number of the line first");
		return;
	}
	auto route{editor_track::run_route(*m_pick->track, kPostReach)};
	auto const samples{editor_track::sample_route(route, 2.0)};
	if (samples.size() < 2)
	{
		m_status = STR("No line through this track");
		return;
	}
	auto const length{samples.back().chainage};
	struct post
	{
		double chainage{0.0};
		int hectometres{0};
	};
	std::vector<post> posts;
	for (auto const &standing : m_templates.standing_templates([](std::string const &File) { return File.find("hekto") != std::string::npos; }))
	{
		post entry;
		if (false == hectometres_of(standing.values, entry.hectometres))
			continue;
		double distance{0.0};
		entry.chainage = project(samples, standing.location, distance);
		if (distance > kPostCorridor || entry.chainage <= 0.0 || entry.chainage >= length)
			continue;
		posts.push_back(entry);
	}
	if (posts.size() < 2)
	{
		m_status = STR("Fewer than two hectometre posts stand along the line, nothing to read");
		return;
	}
	std::sort(posts.begin(), posts.end(), [](post const &A, post const &B) { return A.chainage < B.chainage; });
	// runs of posts numbered by the distance between them, a post of the same number by the other track taken once
	auto const fits = [](post const &From, post const &To, int const Sign) {
		auto const counted{(To.hectometres - From.hectometres) * 100.0};
		auto const apart{To.chainage - From.chainage};
		return counted != 0.0 && (Sign == 0 || (counted > 0.0 ? 1 : -1) == Sign) && std::abs(std::abs(counted) - apart) <= std::max(kPostTolerance, apart * 0.02);
	};
	auto const sign_of = [](std::vector<post> const &Run) { return Run.size() < 2 ? 0 : Run.back().hectometres > Run.front().hectometres ? 1 : -1; };
	std::vector<std::vector<post>> runs;
	std::size_t twins{0};
	for (auto const &entry : posts)
	{
		if (false == runs.empty())
		{
			auto &run{runs.back()};
			if (entry.hectometres == run.back().hectometres && entry.chainage - run.back().chainage < kPostTolerance * 3.0)
			{
				++twins;
				continue;
			}
			if (fits(run.back(), entry, sign_of(run)))
			{
				run.push_back(entry);
				continue;
			}
		}
		runs.push_back({entry});
	}
	// a post alone between the runs is one out of place
	std::size_t stray{0};
	std::vector<std::vector<post>> kept;
	for (auto &run : runs)
	{
		if (run.size() < 2)
		{
			++stray;
			continue;
		}
		if (false == kept.empty() && fits(kept.back().back(), run.front(), sign_of(kept.back())) && sign_of(kept.back()) == sign_of(run))
			kept.back().insert(kept.back().end(), run.begin(), run.end());
		else
			kept.push_back(std::move(run));
	}
	std::size_t rising{0};
	std::size_t falling{0};
	for (auto const &run : kept)
		(sign_of(run) > 0 ? rising : falling) += run.size();
	auto const growth{rising >= falling ? 1 : -1};
	kept.erase(std::remove_if(kept.begin(), kept.end(), [&](std::vector<post> const &Run) {
		if (sign_of(Run) == growth)
			return false;
		stray += Run.size();
		return true;
	}), kept.end());
	if (kept.empty())
	{
		m_status = STR("The numbers of the posts don't go by 100 m anywhere along the line, nothing read");
		return;
	}
	auto const mark_at = [&](double const Chainage, double const Km, std::optional<double> const Beyond) {
		kilometrage::mark entry;
		entry.line = line;
		entry.point = editor_track::sampled_position(samples, Chainage);
		entry.way = samples[sample_at(samples, Chainage)].direction * static_cast<double>(growth);
		entry.km = Km;
		entry.beyond = Beyond;
		return entry;
	};
	std::vector<kilometrage::mark> made;
	std::size_t jumps{0};
	std::size_t misplaced{0};
	for (std::size_t r = 0; r < kept.size(); ++r)
	{
		auto const &run{kept[r]};
		auto const &first{run.front()};
		auto const &last{run.back()};
		// the jump to the next run at the post it starts the irregular hectometre at, along the rising km
		auto const jump_before{r > 0 && growth > 0};
		auto const jump_after{r + 1 < kept.size() && growth < 0};
		if (jump_before)
		{
			auto const &previous{kept[r - 1].back()};
			made.push_back(mark_at(first.chainage, previous.hectometres / 10.0 + (first.chainage - previous.chainage) / 1000.0, first.hectometres / 10.0));
		}
		else
			made.push_back(mark_at(first.chainage, first.hectometres / 10.0, std::nullopt));
		if (jump_after)
		{
			auto const &next{kept[r + 1].front()};
			made.push_back(mark_at(last.chainage, next.hectometres / 10.0 + (next.chainage - last.chainage) / 1000.0, last.hectometres / 10.0));
		}
		else
			made.push_back(mark_at(last.chainage, last.hectometres / 10.0, std::nullopt));
		if (jump_before || jump_after)
		{
			++jumps;
			if (false == jump_warning((jump_before ? made[made.size() - 2] : made.back()).point).empty())
				++misplaced;
		}
	}
	// the points of the line along the stretch read give way to the ones read
	auto const &standing{kilometrage::marks()};
	for (auto index{static_cast<int>(standing.size()) - 1}; index >= 0; --index)
	{
		if (standing[index].line != line)
			continue;
		double gap{0.0};
		project(samples, standing[index].point, gap);
		if (gap <= kPostCorridor)
			kilometrage::remove(static_cast<std::size_t>(index));
	}
	for (auto &entry : made)
		kilometrage::put(std::move(entry));
	m_line = line;
	m_edited = -1;
	m_history.push_kilometrage_snapshot();
	m_status = format(STR_C("%zu posts read: %zu points of the km, %zu jumps"), posts.size() - twins, made.size(), jumps);
	if (stray > 0)
		m_status += "\n" + format(STR_P("%zu post doesn't fit its neighbours and was left out", "%zu posts don't fit their neighbours and were left out", stray), stray);
	if (misplaced > 0)
		m_status += "\n" + format(STR_P("%zu jump lies on a curve or at a station, Id-1 § 19 wants none there", "%zu jumps lie on a curve or at a station, Id-1 § 19 wants none there", misplaced), misplaced);
}

void km_tool::render_options()
{
	ImGui::TextDisabled("%s", STR_C("The km of a railway line: the points where the km is known, from a post or a document, and where it jumps. The km of the rest go from them along the axis of the line: between the tracks of a double-track one, along the longer track where the tracks run on formations of their own, the shorter one getting an irregular hectometre"));
	ImGui::Spacing();
	gesture_rows({{"LMB on a track", "a point of the km there, or the one standing by it"}});
	auto const &marks{kilometrage::marks()};
	auto const lines{kilometrage::lines()};
	std::size_t jumps{0};
	for (auto const &mark : marks)
		if (mark.beyond.has_value())
			++jumps;
	metric_tiles({{format("%zu", lines.size()), STR_C("lines"), kAxis}, {format("%zu", marks.size()), STR_C("points"), kMark}, {format("%zu", jumps), STR_C("jumps"), kJump}});

	step_heading(1, STR_C("A point of the km"));
	if (false == m_pick.has_value())
		ImGui::TextDisabled("%s", STR_C("Click a track where the km is known: by a post, a station or a bridge"));
	else
	{
		if (begin_fields("##kmpoint"))
		{
			field_row("Line");
			std::array<char, 32> buffer{};
			m_line.copy(buffer.data(), buffer.size() - 1);
			if (ImGui::InputTextWithHint("##kmline", "91", buffer.data(), buffer.size()))
				m_line = line_of(buffer.data());
			item_tooltip("The number of the railway line, as 91: a scenery can hold several lines, each with its own km");
			auto const km_field = [&](char const *Label, char const *Id, double &Value) {
				field_row(Label);
				std::array<char, 32> text{};
				kilometrage::text(Value).copy(text.data(), text.size() - 1);
				if (ImGui::InputText(Id, text.data(), text.size()))
					km_of(text.data(), Value);
			};
			km_field("Km", "##kmkm", m_km);
			item_tooltip("The km of the point, as PKP PLK write it: 100,234 is km 100 and 234 m. At a jump: the km the point is reached at, coming along the rising km");
			field_row("Jump");
			ImGui::Checkbox("##kmjump", &m_jump);
			item_tooltip("The km jumps at the point: a line built anew, shortened or lengthened keeps the km of the rest. The irregular hectometre is "
			             "marked by a black stripe under the digit of the post (Id-1 § 19)");
			if (m_jump)
			{
				km_field("Km on", "##kmbeyond", m_beyond);
				item_tooltip("The km on from the point, along the rising km");
			}
			ImGui::EndTable();
		}
		auto const *axis{m_line.empty() ? nullptr : kilometrage::axis_of(m_line)};
		auto const fixed{axis != nullptr && axis->anchors.size() > 1 && std::abs(axis->anchors.back().before - axis->anchors.front().before) > 1e-6};
		if (false == fixed)
		{
			int way{m_against ? 1 : 0};
			if (segmented("##kmway", way, {{0, STR_C("Rise this way"), "The km rise the way the arrow at the point shows"}, {1, STR_C("Rise the other way"), "The km rise against the arrow"}}))
				m_against = way == 1;
		}
		if (m_jump)
			if (auto const warning{jump_warning(m_pick->point)}; false == warning.empty())
				ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", warning.c_str());
		if (main_button(m_edited >= 0 ? STR_C("Change the point") : STR_C("Put the point of the km"), ImVec2(-1.0f, tall_button_height())))
			put();
		item_tooltip("The point goes to the file of the scenery with the track, the simulation skips it. One step of undo");
		if (m_edited >= 0)
		{
			if (ImGui::Button(STR_C("Remove the point"), ImVec2(-1.0f, 0.0f)))
				remove();
			item_tooltip("One step of undo");
		}
	}

	step_heading(2, STR_C("The way through a station"));
	auto const *line_axis{m_line.empty() ? nullptr : kilometrage::axis_of(m_line)};
	if (false == m_pick.has_value() || m_pick->track == nullptr)
		ImGui::TextDisabled("%s", STR_C("Through a station the km run along the straight paths of the switches. Click a track of the station to lead them along it instead"));
	else if (line_axis == nullptr)
		ImGui::TextDisabled("%s", STR_C("Put a point of the km of the line first"));
	else if (auto const via{via_here()}; via >= 0)
	{
		if (std::find(line_axis->blocked.begin(), line_axis->blocked.end(), static_cast<std::size_t>(via)) != line_axis->blocked.end())
			ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f), "%s", STR_C("The line doesn't run along this track: another track chosen leads it elsewhere"));
		else
			ImGui::TextUnformatted(format(STR_C("Line %s runs along this track, as chosen"), m_line.c_str()).c_str());
		if (ImGui::Button(STR_C("Back onto the straight paths"), ImVec2(-1.0f, 0.0f)))
			drop_via();
		item_tooltip("The track chosen is forgotten, the line runs along the straight paths of the switches here again. One step of undo");
	}
	else if (line_axis->runs_along(m_pick->track, m_pick->path))
		ImGui::TextDisabled("%s", format(STR_C("Line %s runs along this track"), m_line.c_str()).c_str());
	else
	{
		ImGui::TextDisabled("%s", format(STR_C("Line %s doesn't run along this track"), m_line.c_str()).c_str());
		if (ImGui::Button(STR_C("Lead the km along this track"), ImVec2(-1.0f, 0.0f)))
			lead_along();
		item_tooltip("The line runs along the track clicked, along the path of the switch clicked, and on along the straight paths of the switches "
		             "until it meets its tracks again; where it doesn't, as at a junction, it goes on that way. One step of undo");
	}
	if (false == m_route_status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", m_route_status.c_str());

	step_heading(3, STR_C("From the posts"));
	if (ImGui::Button(STR_C("Read the km from the posts"), ImVec2(-1.0f, 0.0f)))
		read_posts();
	item_tooltip("Reads the hectometre posts standing along the line through the track clicked: a point of the km at the first and the last "
	             "post of each run numbered by 100 m, a jump where two runs meet. The points of the line along it before give way. One step of undo");
	if (false == m_status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", m_status.c_str());

	step_heading(4, STR_C("The points of the lines"));
	if (lines.empty())
	{
		ImGui::TextDisabled("%s", STR_C("None yet"));
		return;
	}
	for (auto const &line : lines)
	{
		auto const *axis{kilometrage::axis_of(line)};
		auto const count{static_cast<std::size_t>(std::count_if(marks.begin(), marks.end(), [&](kilometrage::mark const &Mark) { return Mark.line == line; }))};
		if (false == ImGui::CollapsingHeader(format("%s %s (%zu)###kmline%s", STR_C("Line"), line.c_str(), count, line.c_str()).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
			continue;
		if (axis == nullptr)
		{
			ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f), "%s", STR_C("No track by the points of the line"));
			continue;
		}
		ImGui::TextDisabled("%s", format(STR_C("%.1f km of the axis, %s"), axis->length() / 1000.0, axis->between ? STR_C("between the tracks") : STR_C("along the track")).c_str());
		if (false == axis->sides.empty())
		{
			ImGui::TextDisabled("%s", format(STR_P("%zu stretch with the tracks on formations of their own", "%zu stretches with the tracks on formations of their own", axis->sides.size()), axis->sides.size()).c_str());
			item_tooltip("Where the tracks part more than 15 m and come together again, the km run along the longer one, the shorter one goes by "
			             "its own length and its last hectometre before they meet takes up the difference, as PKP GK-1 gives it (6.1.12)");
		}
		auto const &vias{kilometrage::vias()};
		for (std::size_t index = 0; index < vias.size(); ++index)
		{
			if (vias[index].line != line)
				continue;
			auto const blocked{std::find(axis->blocked.begin(), axis->blocked.end(), index) != axis->blocked.end()};
			double km{0.0};
			auto const label{kilometrage::km_at(line, vias[index].point, km, kReadoutReach) ? format(STR_C("Along the track chosen by km %s"), kilometrage::text(km).c_str()) : std::string{STR_C("Along the track chosen")}};
			if (blocked)
				ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.45f, 0.4f, 1.0f));
			if (ImGui::Selectable((label + "##kmvia" + std::to_string(index)).c_str(), false))
				show_via(index);
			if (blocked)
				ImGui::PopStyleColor();
			item_tooltip(blocked ? "The line doesn't run along this track: another track chosen leads it elsewhere" : "Through the station the line runs along this track, not along the straight paths of the switches");
		}
		auto const rows{axis->anchors.size() + axis->astray.size()};
		if (false == ImGui::BeginTable(("##kmmarks" + line).c_str(), 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY, ImVec2(0.0f, std::min(260.0f, ImGui::GetTextLineHeightWithSpacing() * (rows + 1.5f)))))
			continue;
		ImGui::TableSetupScrollFreeze(0, 1);
		ImGui::TableSetupColumn(STR_C("Km"), ImGuiTableColumnFlags_WidthStretch, 0.4f);
		ImGui::TableSetupColumn(STR_C("In the scenery"), ImGuiTableColumnFlags_WidthStretch, 0.22f);
		ImGui::TableSetupColumn(STR_C("By the km"), ImGuiTableColumnFlags_WidthStretch, 0.22f);
		ImGui::TableSetupColumn(STR_C("Off by"), ImGuiTableColumnFlags_WidthStretch, 0.16f);
		ImGui::TableHeadersRow();
		auto const growth{axis->growth};
		for (std::size_t i = 0; i < axis->anchors.size(); ++i)
		{
			auto const &anchor{axis->anchors[i]};
			auto const &mark{marks[anchor.mark]};
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			if (ImGui::Selectable((mark_text(mark) + "##kmmark" + std::to_string(anchor.mark)).c_str(), static_cast<int>(anchor.mark) == m_edited, ImGuiSelectableFlags_SpanAllColumns))
				show(anchor.mark);
			if (i == 0)
				continue;
			auto const &previous{axis->anchors[i - 1]};
			auto const scenery{anchor.chainage - previous.chainage};
			auto const counted{std::abs((growth > 0 ? anchor.before - previous.after : previous.before - anchor.after) * 1000.0)};
			ImGui::TableNextColumn();
			ImGui::Text("%.0f m", scenery);
			ImGui::TableNextColumn();
			ImGui::Text("%.0f m", counted);
			ImGui::TableNextColumn();
			auto off{scenery > 1.0 ? (scenery - counted) / scenery * 100.0 : 0.0};
			if (std::abs(off) < 0.05)
				off = 0.0;
			if (std::abs(off) > 2.0)
				ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%+.1f%%", off);
			else
				ImGui::Text("%+.1f%%", off);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("How much longer the stretch is in the scenery than by the km: the km between the points are stretched to it evenly"));
		}
		for (auto const index : axis->astray)
		{
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.45f, 0.4f, 1.0f));
			if (ImGui::Selectable((mark_text(marks[index]) + "##kmmark" + std::to_string(index)).c_str(), static_cast<int>(index) == m_edited, ImGuiSelectableFlags_SpanAllColumns))
				show(index);
			ImGui::PopStyleColor();
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("The point is away from the axis of the line the other points make: the main paths of the switches don't lead to it"));
		}
		ImGui::EndTable();
	}
}

void km_tool::draw_overlay() const
{
	if (false == m_open)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	glm::dvec3 const camera{Global.pCamera.Pos};
	auto const near_enough = [&](glm::dvec3 const &Point, double const Range) { return Global.EditorOrtho || glm::distance(Point, camera) < Range; };
	glm::dvec3 const lift{0.0, kRailHead, 0.0};
	for (auto const &entry : m_drawn)
	{
		// the axis, faint where the km run on past the last point
		for (std::size_t i = 1; i < entry.points.size(); i += 5)
		{
			auto const next{std::min(i + 4, entry.points.size() - 1)};
			if (near_enough(entry.points[next], kOverlayRange))
				projection.line(drawlist, entry.points[i - 1] + lift, entry.points[next] + lift, entry.guessed[next] != 0 ? kGuessed : kAxis, Global.EditorOrtho ? 4.0f : 2.5f);
		}
		for (auto const &tick : entry.ticks)
		{
			if (false == near_enough(tick.at, kTickRange))
				continue;
			auto const centre{tick.at + lift};
			auto const whole{tick.hectometres % 10 == 0};
			auto const reach{tick.jump ? 3.5 : whole ? 2.6 : 1.4};
			projection.line(drawlist, centre - tick.across * reach, centre + tick.across * reach, tick.jump ? kJump : whole ? kKm : kTick, tick.jump || whole ? 3.5f : 2.0f);
			if (tick.jump)
				continue;
			ImVec2 screen;
			if (whole && near_enough(tick.at, kLabelRange) && projection.project(centre + tick.across * reach + glm::dvec3{0.0, 0.6, 0.0}, screen))
				draw_label(drawlist, screen, IM_COL32(0, 0, 0, 170), kKm, format("km %d", tick.hectometres / 10));
			else if (false == whole && near_enough(tick.at, kDigitRange) && projection.project(centre + tick.across * reach + glm::dvec3{0.0, 0.4, 0.0}, screen))
				draw_label(drawlist, screen, IM_COL32(0, 0, 0, 140), kTick, std::to_string(std::abs(tick.hectometres) % 10));
		}
	}
	// the points of the km: a pole with the km over it, the arrow the way the km rise
	auto const &marks{kilometrage::marks()};
	for (std::size_t index = 0; index < marks.size(); ++index)
	{
		auto const &mark{marks[index]};
		if (false == near_enough(mark.point, kLabelRange * 2.0))
			continue;
		auto astray{true};
		for (auto const &entry : m_drawn)
			if (entry.line == mark.line)
				if (auto const *axis{kilometrage::axis_of(mark.line)}; axis != nullptr)
					astray = std::find(axis->astray.begin(), axis->astray.end(), index) != axis->astray.end();
		auto const colour{astray ? kAstray : mark.beyond.has_value() ? kJump : kMark};
		auto const foot{mark.point + lift};
		auto const top{foot + glm::dvec3{0.0, 3.0, 0.0}};
		projection.line(drawlist, foot, top, colour, static_cast<int>(index) == m_edited ? 4.0f : 2.5f);
		draw_arrow(projection, drawlist, foot, mark.way, 6.0, colour);
		ImVec2 screen;
		if (projection.project(top, screen))
			draw_label(drawlist, screen, colour, IM_COL32(16, 18, 22, 255), mark.line + ": " + mark_text(mark));
	}
	// the tracks chosen for the way through a station: a rhombus on the track
	auto const &vias{kilometrage::vias()};
	for (std::size_t index = 0; index < vias.size(); ++index)
	{
		auto const &via{vias[index]};
		ImVec2 screen;
		if (false == near_enough(via.point, kLabelRange * 2.0) || false == projection.project(via.point + lift, screen))
			continue;
		auto blocked{false};
		if (auto const *axis{kilometrage::axis_of(via.line)}; axis != nullptr)
			blocked = std::find(axis->blocked.begin(), axis->blocked.end(), index) != axis->blocked.end();
		auto const colour{blocked ? kAstray : kAxis};
		auto const r{8.0f};
		drawlist->AddQuadFilled(ImVec2(screen.x, screen.y - r), ImVec2(screen.x + r, screen.y), ImVec2(screen.x, screen.y + r), ImVec2(screen.x - r, screen.y), colour);
		drawlist->AddQuad(ImVec2(screen.x, screen.y - r), ImVec2(screen.x + r, screen.y), ImVec2(screen.x, screen.y + r), ImVec2(screen.x - r, screen.y), IM_COL32(16, 18, 22, 255), 1.5f);
		draw_label(drawlist, ImVec2(screen.x, screen.y - r - 2.0f), colour, IM_COL32(16, 18, 22, 255), format(STR_C("%s: this way"), via.line.c_str()));
	}
	if (m_pick.has_value() && m_edited < 0)
	{
		auto const foot{m_pick->point + lift};
		ImVec2 screen;
		if (projection.project(foot, screen))
			drawlist->AddCircle(screen, 9.0f, kKm, 20, 2.5f);
		draw_arrow(projection, drawlist, foot, rising(), 8.0, kKm);
	}
}

editor_selftest_result km_tool::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "kmpick")
	{
		// kmpick track=<name> [at=<0..1>]: a click on the track
		auto const values{selftest_values(Words)};
		auto const found{values.find("track")};
		auto *track{found != values.end() ? simulation::Paths.find(found->second) : nullptr};
		if (track == nullptr)
		{
			WriteLog("SELFTEST kmpick: no such track");
			return editor_selftest_result::handled;
		}
		auto const at{values.count("at") ? std::atof(values.at("at").c_str()) : 0.5};
		auto const path{values.count("path") ? std::atoi(values.at("path").c_str()) : 0};
		pick(*track, path > 0 && path < static_cast<int>(track->m_paths.size()) ? geometry::bezier{track->m_paths[path]}.point(at) : editor_track::point_at(*track, at));
		auto const *axis{m_line.empty() ? nullptr : kilometrage::axis_of(m_line)};
		WriteLog(format("SELFTEST kmpick %s: line %s km %s jump %d against %d edited %d at %.1f %.1f path %d along %d via %d", track->name().c_str(), m_line.c_str(), kilometrage::text(m_km).c_str(), m_jump ? 1 : 0, m_against ? 1 : 0, m_edited, m_pick->point.x, m_pick->point.z,
		                m_pick->path, axis != nullptr && axis->runs_along(m_pick->track, m_pick->path) ? 1 : 0, via_here()));
		return editor_selftest_result::handled;
	}
	if (Command == "kmset")
	{
		// kmset line=<number> km=<km> [beyond=<km>] [against=1]: the fields set and the point put
		auto const values{selftest_values(Words)};
		if (auto const line{values.find("line")}; line != values.end())
			m_line = line->second;
		if (auto const km{values.find("km")}; km != values.end())
			km_of(km->second.c_str(), m_km);
		m_jump = values.count("beyond") > 0;
		if (m_jump)
			km_of(values.at("beyond").c_str(), m_beyond);
		if (auto const against{values.find("against")}; against != values.end())
			m_against = against->second == "1";
		put();
		WriteLog("SELFTEST kmset: " + m_status);
		return editor_selftest_result::handled;
	}
	if (Command == "kmvia")
	{
		// kmvia [line=<number>] [remove=1]: the line led along the track picked, or back onto the straight paths there
		auto const values{selftest_values(Words)};
		if (auto const line{values.find("line")}; line != values.end())
			m_line = line->second;
		if (values.count("remove") > 0)
			drop_via();
		else
			lead_along();
		auto text{m_route_status};
		std::replace(text.begin(), text.end(), '\n', ';');
		WriteLog("SELFTEST kmvia: " + text);
		return editor_selftest_result::handled;
	}
	if (Command == "kmremove")
	{
		remove();
		WriteLog("SELFTEST kmremove: " + m_status);
		return editor_selftest_result::handled;
	}
	if (Command == "kmposts")
	{
		// kmposts line=<number>: the km read from the posts along the line through the track picked
		auto const values{selftest_values(Words)};
		if (auto const line{values.find("line")}; line != values.end())
			m_line = line->second;
		read_posts();
		WriteLog("SELFTEST kmposts: " + m_status);
		return editor_selftest_result::handled;
	}
	if (Command == "kmstate")
	{
		WriteLog(format("SELFTEST kmstate: picked %d line %s km %s jump %d edited %d", m_pick.has_value() ? 1 : 0, m_line.c_str(), kilometrage::text(m_km).c_str(), m_jump ? 1 : 0, m_edited));
		return editor_selftest_result::handled;
	}
	if (Command == "kmshow")
	{
		// kmshow mark=<index>: the point of the km picked, the camera over it
		auto const values{selftest_values(Words)};
		auto const index{values.count("mark") ? std::atoi(values.at("mark").c_str()) : 0};
		show(static_cast<std::size_t>(std::max(index, 0)));
		WriteLog(format("SELFTEST kmshow %d: line %s km %s edited %d", index, m_line.c_str(), kilometrage::text(m_km).c_str(), m_edited));
		return editor_selftest_result::handled;
	}
	if (Command == "kmlist")
	{
		refresh();
		for (auto const &entry : m_drawn)
		{
			std::string list;
			for (std::size_t i = 0; i < entry.ticks.size() && entry.points.size() < 2000; ++i)
				list += format(" %d@%.0f,%.0f", entry.ticks[i].hectometres, entry.ticks[i].at.x, entry.ticks[i].at.z);
			WriteLog(format("SELFTEST kmlist ticks %s: %zu points, %zu ticks", entry.line.c_str(), entry.points.size(), entry.ticks.size()) + list);
		}
		auto const &marks{kilometrage::marks()};
		WriteLog(format("SELFTEST kmlist: %zu points", marks.size()));
		for (auto const &line : kilometrage::lines())
		{
			auto const *axis{kilometrage::axis_of(line)};
			if (axis == nullptr)
			{
				WriteLog("SELFTEST kmlist line " + line + ": no axis");
				continue;
			}
			std::string list;
			for (auto const &anchor : axis->anchors)
				list += format(" %s@%.0f/%.1f", mark_text(marks[anchor.mark]).c_str(), anchor.chainage, anchor.gap);
			for (auto const &side : axis->sides)
				WriteLog(format("SELFTEST kmlist apart: %.1f m of track by %.1f m of axis (%.1f..%.1f), km %s..%s", side.length, side.to - side.from, side.from, side.to, kilometrage::text(kilometrage::km_on(*axis, side, 0.0)).c_str(),
				                kilometrage::text(kilometrage::km_on(*axis, side, side.length)).c_str()));
			WriteLog(format("SELFTEST kmlist line %s: axis %.0f m %s growth %d, %zu astray, %zu blocked, km %s..%s:", line.c_str(), axis->length(), axis->between ? "between" : "along", axis->growth, axis->astray.size(), axis->blocked.size(), kilometrage::text(kilometrage::km_on(*axis, 0.0)).c_str(),
			                kilometrage::text(kilometrage::km_on(*axis, axis->length())).c_str()) +
			         list);
		}
		return editor_selftest_result::handled;
	}
	if (Command == "kmat")
	{
		// kmat track=<name> [at=<0..1>]: the km of the lines there
		auto const values{selftest_values(Words)};
		auto const found{values.find("track")};
		auto *track{found != values.end() ? simulation::Paths.find(found->second) : nullptr};
		if (track == nullptr)
		{
			WriteLog("SELFTEST kmat: no such track");
			return editor_selftest_result::handled;
		}
		auto const at{values.count("at") ? std::atof(values.at("at").c_str()) : 0.5};
		auto text{readout(editor_track::point_at(*track, at))};
		std::replace(text.begin(), text.end(), '\n', ';');
		WriteLog(format("SELFTEST kmat %s %.2f: ", track->name().c_str(), at) + (text.empty() ? std::string{"-"} : text));
		return editor_selftest_result::handled;
	}
	return editor_selftest_result::unhandled;
}

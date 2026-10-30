/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// a crossover between two tracks running side by side along a curve: planned as between straights in the frame of the curve
// (along it, across it) and laid onto the curve, the switches, the insert and the other track on the plane the cant tilts the
// inner track to. The outer track goes up to that plane over the crossover, with ramps on both sides

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editormodeshared.h"
#include "application/editormodetrackshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorSwitchStore.hpp"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "world/Track.h"

#include <algorithm>

using geometry::bezier;
using geometry::plan_distance;
using geometry::plan_of;

namespace
{

// of the ramps the outer track goes up to the crossover and down from it by, a share of the change of grade which needs a
// vertical curve: the ramps break the grade line without one
double constexpr kRampShare{0.9};
double constexpr kOtherReach{10.0}; // m, from the release to the other track
double constexpr kStraightReach{track_mode::kCrossoverStraights};

// the first curved track a straight track leads to along its straight, either way, within the reach
TTrack *curve_ahead(TTrack &Track, editor_track::straight_tolerance const &Tolerance, double const Reach)
{
	TTrack *found{nullptr};
	auto nearest{Reach};
	for (auto const throughend : {true, false})
	{
		TTrack const *previous{&Track};
		TTrack *current{throughend ? Track.trNext : Track.trPrev};
		double distance{0.0};
		for (int count = 0; current != nullptr && count < 1000 && distance < nearest; ++count)
		{
			if (current->eType != tt_Normal || current->m_editorremoved || current->m_paths.empty())
				break;
			if (false == editor_track::is_straight(*current, Tolerance))
			{
				found = current;
				nearest = distance;
				break;
			}
			auto const &path{current->m_paths.front()};
			distance += plan_distance(path.points[segment_data::point::start], path.points[segment_data::point::end]);
			auto *next{current->trPrev == previous ? current->trNext : current->trPrev};
			previous = current;
			current = next;
		}
	}
	return found;
}

} // namespace

// the plan of a connection between straights laid onto the curve: its x across the curve, minus to the left, its z the station
struct editor_mode::curve_mapping
{
	std::vector<curve_sample> const &frame;

	glm::dvec3 point(glm::dvec3 const &Local) const
	{
		auto const sample{sample_at(frame, Local.z)};
		glm::dvec2 const left{-sample.tangent.y, sample.tangent.x};
		auto const lateral{-Local.x};
		auto const planar{plan_of(sample.position) + left * lateral};
		return {planar.x, sample.position.y + Local.y - lateral * std::tan(glm::radians(static_cast<double>(sample.roll))), planar.y};
	}
	glm::dvec2 tangent(glm::dvec3 const &Local, glm::dvec2 const &Direction) const
	{
		auto const sample{sample_at(frame, Local.z)};
		glm::dvec2 const left{-sample.tangent.y, sample.tangent.x};
		return glm::normalize(sample.tangent * Direction.y - left * Direction.x);
	}
	float roll(glm::dvec3 const &Local) const { return sample_at(frame, Local.z).roll; }
	// the height of the tilted plane at the point of the plan
	double height(glm::dvec2 const &Planar) const
	{
		auto const sample{sample_at(frame, nearest_station(frame, {Planar.x, 0.0, Planar.y}))};
		auto const lateral{glm::dot(Planar - plan_of(sample.position), glm::dvec2{-sample.tangent.y, sample.tangent.x})};
		return sample.position.y - lateral * std::tan(glm::radians(static_cast<double>(sample.roll)));
	}

	segment_data path(segment_data const &Local) const
	{
		auto const &start{Local.points[segment_data::point::start]};
		auto const &end{Local.points[segment_data::point::end]};
		auto const &control1{Local.points[segment_data::point::control1]};
		auto const &control2{Local.points[segment_data::point::control2]};
		glm::dvec2 const chord{end.x - start.x, end.z - start.z};
		auto const localstart{glm::normalize(control1 != glm::dvec3{} ? plan_of(control1) : chord)};
		auto const localend{glm::normalize(control2 != glm::dvec3{} ? -plan_of(control2) : chord)};
		auto const p0{point(start)};
		auto const p3{point(end)};
		auto const t0{tangent(start, localstart)};
		auto const t3{tangent(end, localend)};
		auto const handle{plan_distance(p0, p3) / 3.0};
		segment_data result;
		result.points[segment_data::point::start] = p0;
		result.points[segment_data::point::end] = p3;
		result.points[segment_data::point::control1] = {t0.x * handle, (p3.y - p0.y) / 3.0, t0.y * handle};
		result.points[segment_data::point::control2] = {-t3.x * handle, -(p3.y - p0.y) / 3.0, -t3.y * handle};
		// the heights of the inner points on the tilted plane
		auto const h1{height(plan_of(bezier{result}.point(1.0 / 3.0)))};
		auto const h2{height(plan_of(bezier{result}.point(2.0 / 3.0)))};
		auto const y1{3.0 * h1 - 1.5 * h2 - 5.0 / 6.0 * p0.y + p3.y / 3.0};
		auto const y2{3.0 * h2 - 1.5 * h1 + p0.y / 3.0 - 5.0 / 6.0 * p3.y};
		result.points[segment_data::point::control1].y = y1 - p0.y;
		result.points[segment_data::point::control2].y = y2 - p3.y;
		// the cant of a path is told along it: a path running against the stations of the curve has it the other way round
		result.rolls = {roll(start) * (localstart.y >= 0.0 ? 1.f : -1.f), roll(end) * (localend.y >= 0.0 ? 1.f : -1.f)};
		result.radius = Local.radius;
		bezier const curve{result};
		auto const begin{plan_of(curve.first(0.0))};
		auto const finish{plan_of(curve.first(1.0))};
		auto const turn{glm::length(begin) > 1e-9 && glm::length(finish) > 1e-9 ? geometry::signed_angle(begin, finish) : 0.0};
		if (std::abs(turn) > 1e-6)
			result.radius = static_cast<float>(curve.plan_length() / turn);
		else
			result.radius = 0.f;
		for (auto const &piece : Local.pieces)
			result.pieces.push_back(path(piece));
		return result;
	}
};

// the curve the track is a part of, as a frame of samples, and its tracks
bool editor_mode::curve_of(TTrack &Track, editor_track::straight_tolerance const &Tolerance, double const Gauge, std::vector<curve_sample> &Frame, std::vector<TTrack *> &Tracks, double const Beyond)
{
	auto *base{&Track};
	if (Beyond > 0.0 && Track.eType == tt_Normal && editor_track::is_straight(Track, Tolerance))
		base = curve_ahead(Track, Tolerance, Beyond);
	if (base == nullptr)
		return false;
	editor_track::curve curve;
	editor_track::chain chain;
	std::string error;
	if (false == editor_track::find_curve(*base, Tolerance, Gauge, curve) || false == editor_track::find_chain(curve.from, curve.to, chain, error, true))
		return false;
	for (auto const &part : editor_track::chain_parts(chain))
		if (std::find(part.tracks.begin(), part.tracks.end(), base) != part.tracks.end())
		{
			chain = part;
			break;
		}
	if (Beyond > 0.0)
	{
		// the straights past both ends, as far as the reach, taken along the curve: the one before it from its far end
		auto const taken = [&](editor_track::straight_run const &Run) {
			std::size_t count{0};
			for (double reached{0.0}; count < Run.tracks.size() && reached < Beyond; ++count)
			{
				auto const &path{Run.tracks[count]->m_paths.front()};
				reached += plan_distance(path.points[segment_data::point::start], path.points[segment_data::point::end]);
			}
			return count;
		};
		auto const before{editor_track::straight_beyond(chain, false, Tolerance)};
		auto const after{editor_track::straight_beyond(chain, true, Tolerance)};
		editor_track::chain extended;
		for (auto i{taken(before)}; i-- > 0;)
		{
			extended.tracks.push_back(before.tracks[i]);
			extended.forward.push_back(false == before.outward[i]);
		}
		extended.tracks.insert(extended.tracks.end(), chain.tracks.begin(), chain.tracks.end());
		extended.forward.insert(extended.forward.end(), chain.forward.begin(), chain.forward.end());
		for (std::size_t i = 0, count = taken(after); i < count; ++i)
		{
			extended.tracks.push_back(after.tracks[i]);
			extended.forward.push_back(after.outward[i]);
		}
		chain = std::move(extended);
	}
	if (std::find(chain.tracks.begin(), chain.tracks.end(), &Track) == chain.tracks.end())
		return false;
	Frame = sample_chain(chain);
	Tracks = chain.tracks;
	return Frame.size() >= 2;
}

// the paths of the curve between the points cut off at both, right at the points, and taken out, the switch to go in their place
void editor_mode::cut_curve(std::vector<TTrack *> &Candidates, std::vector<curve_sample> const &Frame, glm::dvec3 const &First, glm::dvec3 const &Second, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed)
{
	auto const one{nearest_station(Frame, First)};
	auto const two{nearest_station(Frame, Second)};
	auto const From{std::min(one, two)};
	auto const To{std::max(one, two)};
	for (auto const &point : {First, Second})
	{
		for (auto *candidate : std::vector<TTrack *>(Candidates))
		{
			if (candidate->m_editorremoved || candidate->eType != tt_Normal)
				continue;
			auto const t{editor_track::nearest_parameter(*candidate, point)};
			if (plan_distance(editor_track::point_at(*candidate, t), point) > 0.05)
				continue;
			if (t > 0.002 && t < 0.998)
			{
				if (std::find(Created.begin(), Created.end(), candidate) == Created.end() && std::none_of(States.begin(), States.end(), [&](auto const &Entry) { return Entry.first == candidate; }))
					States.emplace_back(candidate, editor_track::capture(*candidate));
				if (auto *second{editor_track::split_path(*candidate, t)})
				{
					Created.push_back(second);
					Candidates.push_back(second);
				}
			}
			break;
		}
	}
	for (auto *candidate : Candidates)
	{
		if (candidate->m_editorremoved)
			continue;
		auto const where{nearest_station(Frame, editor_track::point_at(*candidate, 0.5))};
		if (where > From + 0.01 && where < To - 0.01)
		{
			editor_track::retire(*candidate);
			// a piece split off in this step goes as if it had never been made: kept among the ones made and the ones taken out
			// too, an undo would retire it and then bring it back
			if (auto const made{std::find(Created.begin(), Created.end(), candidate)}; made != Created.end())
				Created.erase(made);
			else
				Removed.push_back(candidate);
		}
	}
}

void editor_mode::insert_curved_connection()
{
	auto &tool{m_switch};
	tool.placing = false;
	auto const fail = [&](std::string const &Reason) {
		tool.status = Reason;
		ui()->set_status(Reason, true);
	};
	if (tool.connection != static_cast<int>(connection::kind::crossover))
		return fail(STR("Along a curve only a crossover can be laid"));
	// the curve pressed on with the straights past its ends: from a transition the crossover goes on onto the straight
	auto pressedframe{tool.frame};
	auto pressedtracks{tool.frame_tracks};
	auto *pressed{tool.frame_tracks.front()};
	{
		auto nearest{std::numeric_limits<double>::max()};
		for (auto *track : tool.frame_tracks)
			if (false == track->m_editorremoved)
				if (auto const distance{plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, tool.point)), tool.point)}; distance < nearest)
				{
					nearest = distance;
					pressed = track;
				}
		std::vector<curve_sample> frame;
		std::vector<TTrack *> tracks;
		if (curve_of(*pressed, m_straights.tolerance, m_route.design.norms.gauge, frame, tracks, kStraightReach))
		{
			pressedframe = std::move(frame);
			pressedtracks = std::move(tracks);
		}
	}
	// the other track: one beside this curve near the release, running along with it
	TTrack *other{nullptr};
	auto best{kOtherReach};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty() || std::find(pressedtracks.begin(), pressedtracks.end(), track) != pressedtracks.end())
			continue;
		auto const distance{plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, tool.mouse)), tool.mouse)};
		if (distance < best)
		{
			best = distance;
			other = track;
		}
	}
	if (other == nullptr)
		return fail(STR("Drag onto the other track"));
	std::vector<curve_sample> otherframe;
	std::vector<TTrack *> othertracks;
	if (false == curve_of(*other, m_straights.tolerance, m_route.design.norms.gauge, otherframe, othertracks, kStraightReach))
		return fail(STR("The other track isn't a curve running along this one"));
	// the inner track is the frame: the plane of its cant carries the crossover, the outer track goes up onto it
	auto innerframe{pressedframe};
	auto innertracks{pressedtracks};
	auto station{nearest_station(innerframe, tool.point)};
	auto const at{sample_at(innerframe, station)};
	auto const across{sample_at(otherframe, nearest_station(otherframe, at.position))};
	glm::dvec2 const left{-at.tangent.y, at.tangent.x};
	auto offset{glm::dot(plan_of(across.position) - plan_of(at.position), left)};
	auto lift{at.position.y - offset * std::tan(glm::radians(static_cast<double>(at.roll))) - across.position.y};
	bool swapped{false};
	if (lift < -0.005)
	{
		// the plane falls towards the other track: that one is the inner one
		std::swap(innerframe, otherframe);
		std::swap(innertracks, othertracks);
		station = nearest_station(innerframe, at.position);
		auto const inner{sample_at(innerframe, station)};
		glm::dvec2 const innerleft{-inner.tangent.y, inner.tangent.x};
		offset = glm::dot(plan_of(at.position) - plan_of(inner.position), innerleft);
		lift = inner.position.y - offset * std::tan(glm::radians(static_cast<double>(inner.roll))) - at.position.y;
		swapped = true;
	}
	WriteLog(format("Editor: curved crossover - at station %.1f, inner roll %.3f deg, other roll %.3f deg, offset %.3f m, lift %.3f m%s", station, at.roll, across.roll, offset, lift, swapped ? ", the other track is the inner one" : ""), logtype::generic);
	if (std::abs(offset) < 3.0 || std::abs(offset) > 30.0)
		return fail(format(STR_C("The tracks are %.2f m apart, too close or too far for a crossover"), std::abs(offset)));

	// the crossover planned as between two straights, in the frame of the inner curve
	auto const total{innerframe.back().station};
	connection::request request;
	request.type = connection::kind::crossover;
	request.from.start = {0.0, 0.0, 0.0};
	request.from.direction = {0.0, 1.0};
	request.from.length = total;
	connection::line to;
	to.start = {-offset, 0.0, 0.0};
	to.direction = {0.0, 1.0};
	to.length = total;
	request.to.push_back(to);
	request.first = tool.templates[tool.armed];
	auto const second{tool.connection_second >= 0 && tool.connection_second < static_cast<int>(tool.templates.size()) ? tool.connection_second : tool.armed};
	request.second = tool.templates[second];
	request.radius = tool.connection_radius;
	auto const release{nearest_station(innerframe, tool.mouse)};
	auto const press{swapped ? nearest_station(innerframe, tool.point) : station};
	request.forward = release >= press ? 1 : -1;
	// the plan starts on the track pressed on
	if (swapped)
	{
		request.from.start = {-offset, 0.0, 0.0};
		request.to.front().start = {0.0, 0.0, 0.0};
	}
	request.along = press;
	request.target = release;
	// the side of the travel along the starting track the other one lies on
	auto const across_plan{request.to.front().start.x - request.from.start.x};
	request.side = -request.forward * across_plan >= 0.0 ? 1 : -1;
	auto plan{connection::plan(request)};
	if (false == plan.valid)
	{
		// the other side, if the side was taken the wrong way round
		request.side = -request.side;
		plan = connection::plan(request);
	}
	if (false == plan.valid)
		return fail(plan.error.empty() ? std::string{STR("The crossover doesn't fit on the curve")} : plan.error);

	curve_mapping const mapping{innerframe};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	auto *style{pressed};
	// the kept grade lines before the outer track went up, the design of which goes up with it
	std::optional<std::vector<stored_profile>> library;
	auto const undo_all = [&]() {
		for (auto *track : created)
			editor_track::retire(*track);
		for (auto entry{states.rbegin()}; entry != states.rend(); ++entry)
		{
			editor_track::apply(*entry->first, entry->second);
			editor_track::commit({entry->first});
		}
		for (auto *track : removed)
		{
			editor_track::revive(*track);
			editor_track::commit({track});
		}
		if (library.has_value())
			profiles_restore(std::move(*library));
		straight_refresh();
	};

	// the outer track up onto the plane of the inner one over the crossover, with ramps on both sides
	auto const outerline{swapped ? 0 : 1};
	double low{std::numeric_limits<double>::max()}, high{std::numeric_limits<double>::lowest()};
	for (auto const &entry : plan.switches)
		for (auto const &path : entry.paths)
			for (auto const end : {segment_data::point::start, segment_data::point::end})
			{
				low = std::min(low, path.points[end].z);
				high = std::max(high, path.points[end].z);
			}
	auto *outertrack{swapped ? pressed : other};
	for (auto const &entry : plan.switches)
		WriteLog(format("Editor: curved crossover - planned switch on %d: %.3f %.3f -> %.3f %.3f; extent %.3f..%.3f", entry.line, entry.paths.front().points[segment_data::point::start].x, entry.paths.front().points[segment_data::point::start].z, entry.paths.front().points[segment_data::point::end].x, entry.paths.front().points[segment_data::point::end].z, low, high), logtype::generic);
	std::size_t raised{0};
	glm::dvec3 flatfrom{0.0}, flatto{0.0};
	double ramplength{0.0};
	if (std::abs(lift) > 0.002)
	{
		auto route{editor_track::run_route(*outertrack, 3000.0)};
		auto const samples{editor_track::sample_route(route, 1.0)};
		if (samples.size() < 2)
			return fail(STR("The outer track can't be followed"));
		// the chainage of the point projected onto the line between the samples
		auto const chainage_of = [&](glm::dvec3 const &Point) {
			double best{std::numeric_limits<double>::max()};
			double chainage{0.0};
			for (std::size_t i = 0; i + 1 < samples.size(); ++i)
			{
				auto const a{plan_of(samples[i].position)};
				auto const run{plan_of(samples[i + 1].position) - a};
				auto const t{std::clamp(glm::dot(plan_of(Point) - a, run) / std::max(1e-12, glm::dot(run, run)), 0.0, 1.0)};
				if (auto const distance{glm::length(plan_of(Point) - (a + run * t))}; distance < best)
				{
					best = distance;
					chainage = samples[i].chainage + (samples[i + 1].chainage - samples[i].chainage) * t;
				}
			}
			return chainage;
		};
		auto const outerx{swapped ? 0.0 : -offset};
		auto c1{chainage_of(mapping.point({outerx, 0.0, low}))};
		auto c2{chainage_of(mapping.point({outerx, 0.0, high}))};
		// the top of the ramps where the switch on the outer track ends: no short paths left between the two
		if (c1 > c2)
			std::swap(c1, c2);
		auto const ramp{std::abs(lift) / (std::max(0.5, kRampShare * m_profile.line.norms.curve_threshold) / 1000.0)};
		flatfrom = editor_track::sampled_position(samples, c1);
		flatto = editor_track::sampled_position(samples, c2);
		ramplength = ramp;
		auto const share = [&](double const Chainage) {
			if (Chainage <= c1 - ramp || Chainage >= c2 + ramp)
				return 0.0;
			if (Chainage < c1)
				return (Chainage - (c1 - ramp)) / ramp;
			if (Chainage > c2)
				return ((c2 + ramp) - Chainage) / ramp;
			return 1.0;
		};
		auto const slope = [&](double const Chainage) {
			if (Chainage > c1 - ramp && Chainage < c1)
				return 1.0 / ramp;
			if (Chainage > c2 && Chainage < c2 + ramp)
				return -1.0 / ramp;
			return 0.0;
		};
		std::vector<double> breaks;
		for (auto const chainage : {c1 - ramp, c1, c2, c2 + ramp})
			if (chainage > 0.5 && chainage < route.length - 0.5)
				breaks.push_back(chainage);
		// the design of the outer track before, kept or read from the track, to go up with it
		auto *first{route.spans.front().track};
		auto *last{route.spans.back().track};
		profile_library_load();
		library = m_profiles;
		profile_take(route, first, last);
		auto const before{m_profile.line};
		std::vector<std::pair<TTrack *, editor_track::state>> raisestates;
		std::vector<TTrack *> raisecreated;
		editor_track::apply_profile(
		    route, [&](double const Chainage) { return editor_track::sampled_elevation(samples, Chainage) + lift * share(Chainage); },
		    [&](double const Chainage) { return editor_track::sampled_grade(samples, Chainage) + lift * slope(Chainage); }, breaks, raisestates, raisecreated);
		raised = raisestates.size() + raisecreated.size();
		states.insert(states.end(), raisestates.begin(), raisestates.end());
		created.insert(created.end(), raisecreated.begin(), raisecreated.end());
		// the raised grade line kept as the design of the outer track, in place of the one it had: its points gone up with it
		// and the corners of the ramps, straight between them as the ramps are
		if (false == route.spans.empty())
		{
			profile_take(route, route.spans.front().track, route.spans.back().track);
			std::vector<profile::pvi> points;
			for (auto point : before.points)
			{
				point.elevation += lift * share(point.chainage);
				points.push_back(point);
			}
			for (auto const chainage : breaks)
			{
				profile::pvi corner;
				corner.chainage = chainage;
				corner.elevation = profile::elevation(before, chainage) + lift * share(chainage);
				corner.automatic = false;
				points.push_back(corner);
			}
			std::sort(points.begin(), points.end(), [](auto const &Left, auto const &Right) { return Left.chainage < Right.chainage; });
			std::vector<profile::pvi> line;
			for (auto const &point : points)
			{
				if (false == line.empty() && point.chainage - line.back().chainage < 0.5)
				{
					// the corner of a ramp before a point of the design at the same place
					if (false == point.automatic && point.radius == 0.0)
						line.back() = point;
					continue;
				}
				line.push_back(point);
			}
			if (line.size() >= 2)
			{
				m_profile.line = before;
				m_profile.line.points = std::move(line);
				m_profile.selected = -1;
				profile_check();
				profile_store();
			}
		}
		WriteLog(format("Editor: curved crossover - the outer track %s raised by %.3f m from %.1f to %.1f m of its line, ramps %.1f m", outertrack->name().c_str(), lift, c1, c2, ramp), logtype::generic);
	}
	// the outer curve again, as it lies now
	std::vector<curve_sample> outerframe;
	std::vector<TTrack *> outertracks;
	{
		auto const probe{mapping.point({swapped ? 0.0 : -offset, 0.0, 0.5 * (low + high)})};
		TTrack *closest{nullptr};
		auto nearest{1.0};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty())
				continue;
			auto const distance{plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, probe)), probe)};
			if (distance < nearest)
			{
				nearest = distance;
				closest = track;
			}
		}
		if (closest == nullptr || false == curve_of(*closest, m_straights.tolerance, m_route.design.norms.gauge, outerframe, outertracks, kStraightReach))
		{
			undo_all();
			return fail(STR("The outer track can't be followed after it went up"));
		}
	}
	// the inner curve again: the outer track may be split where the ramps end
	if (false == swapped)
		innertracks = pressedtracks;

	std::vector<TTrack *> switches;
	for (auto const &entry : plan.switches)
	{
		auto const &shape{entry.second ? request.second : request.first};
		std::vector<segment_data> paths;
		for (auto const &local : entry.paths)
			paths.push_back(mapping.path(local));
		auto const onouter{entry.line == outerline};
		auto &frame{onouter ? outerframe : innerframe};
		auto &candidates{onouter ? outertracks : innertracks};
		// the other track runs along at the distance the crossover was planned for, else its switch would miss it
		for (auto const end : {segment_data::point::start, segment_data::point::end})
		{
			auto const &point{paths.front().points[end]};
			if (plan_distance(sample_at(frame, nearest_station(frame, point)).position, point) > 0.1)
			{
				undo_all();
				return fail(STR("The other track doesn't run along at the same distance there"));
			}
		}
		cut_curve(candidates, frame, paths.front().points[segment_data::point::start], paths.front().points[segment_data::point::end], states, created, removed);
		// the ends of the main track of the switch onto the ends of the paths cut off, which the curve put a little apart
		auto const snap = [&](glm::dvec3 const &Point) {
			glm::dvec3 found{Point};
			auto nearest{0.1};
			for (auto *candidate : candidates)
			{
				if (candidate->m_editorremoved || candidate->m_paths.empty())
					continue;
				for (auto const which : {segment_data::point::start, segment_data::point::end})
					if (auto const distance{glm::distance(candidate->m_paths.front().points[which], Point)}; distance < nearest)
					{
						nearest = distance;
						found = candidate->m_paths.front().points[which];
					}
			}
			return found;
		};
		auto const begin{paths.front().points[segment_data::point::start]};
		auto const finish{paths.front().points[segment_data::point::end]};
		auto const start{snap(begin)};
		auto const end{snap(finish)};
		for (auto &path : paths)
		{
			if (glm::distance(path.points[segment_data::point::start], begin) < 1e-6)
			{
				path.points[segment_data::point::start] = start;
				if (false == path.pieces.empty())
					path.pieces.front().points[segment_data::point::start] = start;
			}
		}
		paths.front().points[segment_data::point::end] = end;
		if (false == paths.front().pieces.empty())
			paths.front().pieces.back().points[segment_data::point::end] = end;
		auto const made{create_switch_set(shape, paths, *style, switch_name_for_new())};
		if (made.empty())
		{
			undo_all();
			return fail(tool.status.empty() ? std::string{STR("A switch of the crossover couldn't be made")} : tool.status);
		}
		created.insert(created.end(), made.begin(), made.end());
		for (auto *track : made)
			if (track->eType == tt_Switch)
				switches.push_back(track);
		advance_switch_name();
	}
	std::vector<TTrack *> inserts;
	for (auto const &insert : plan.inserts)
		if (auto *track{editor_track::create_path(*style, mapping.path(insert))})
			inserts.push_back(track);
	editor_track::commit(inserts);
	digitize_drawn(inserts);
	created.insert(created.end(), inserts.begin(), inserts.end());
	if (switches.empty())
	{
		undo_all();
		return fail(STR("The crossover couldn't be laid"));
	}
	record_connection(request.type, switches, plan, request.first);
	// the lift kept with the crossover, so it can be taken back when the crossover moves or goes
	if (std::abs(lift) > 0.002)
		for (auto entry : switch_store::connections(*switches.front()))
			if (entry.kind == "connection" && entry.name == switches.front()->name())
			{
				entry.lift = lift;
				entry.lift_from = flatfrom;
				entry.lift_to = flatto;
				entry.ramp = ramplength;
				switch_store::put(entry, entry.layer);
			}
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	// the design of the outer track goes back down with it
	if (library.has_value())
		profiles_to_history(std::move(*library));
	straight_refresh();
	first_step_done(first_step::turnout);
	m_node = switches.front();
	ui()->set_node(m_node);
	std::string names;
	for (auto *track : switches)
	{
		if (place_switch_drive(*track))
			join_last_steps();
		names += (names.empty() ? "" : ", ") + track->name();
	}
	tool.status = format(STR_C("Crossover laid along the curve, switches %s, the tracks %.2f m apart"), names.c_str(), std::abs(offset));
	if (std::abs(lift) > 0.002)
		tool.status += format(STR_C("\nThe outer track went up %.3f m onto the plane of the cant of the inner one, with ramps of %.1f per mille"), std::abs(lift), std::max(0.5, kRampShare * m_profile.line.norms.curve_threshold));
	tool.status += "\n" + connection_readout(plan);
	ui()->set_status(tool.status, false);
}

// the crossover laid along a curve the switch is a part of: its record, its two switches and the insert between them
std::optional<editor_mode::curved_crossover> editor_mode::crossover_of(TTrack const &Switch) const
{
	if (Switch.eType != tt_Switch || Switch.m_editorremoved)
		return std::nullopt;
	for (auto const &entry : switch_store::connections(Switch))
	{
		if (entry.kind != "connection" || entry.type != connection::word(connection::kind::crossover) || entry.parts.size() != 2)
			continue;
		curved_crossover result;
		result.record = entry;
		result.first = simulation::Paths.find(entry.parts[0]);
		result.second = simulation::Paths.find(entry.parts[1]);
		if (result.first == nullptr || result.second == nullptr || result.first->m_editorremoved || result.second->m_editorremoved || result.first->m_paths.size() < 2 || result.second->m_paths.size() < 2)
			continue;
		auto const &end{result.first->m_paths[1].points[segment_data::point::end]};
		auto const &other{result.second->m_paths[1].points[segment_data::point::end]};
		for (auto const &[track, point] : editor_track::connected_points(*result.first, end))
		{
			if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.size() != 1)
				continue;
			auto const &path{track->m_paths.front()};
			if (plan_distance(path.points[segment_data::point::start], other) < 0.05 || plan_distance(path.points[segment_data::point::end], other) < 0.05)
				result.insert = track;
		}
		if (result.insert != nullptr)
			return result;
	}
	return std::nullopt;
}

// the crossover taken out: the main tracks of its switches plain paths again, of the same shape, the insert gone, the outer track
// down by the lift it went up by. Gives the plain paths in place of the first and the second switch
bool editor_mode::take_out_crossover(curved_crossover const &Crossover, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack *&First, TTrack *&Second)
{
	TTrack const *style{nullptr};
	for (auto const &[track, point] : editor_track::connected_points(*Crossover.first, Crossover.first->m_paths.front().points[segment_data::point::start]))
		if (track != nullptr && track->eType == tt_Normal && false == track->m_editorremoved)
			style = track;
	if (style == nullptr)
		style = Crossover.insert;
	std::vector<TTrack *> plains;
	for (auto *track : {Crossover.first, Crossover.second})
	{
		auto *plain{editor_track::create_path(*style, track->m_paths.front())};
		if (plain == nullptr)
			return false;
		plains.push_back(plain);
		Created.push_back(plain);
	}
	for (auto *track : {Crossover.first, Crossover.second, Crossover.insert})
	{
		editor_track::retire(*track);
		Removed.push_back(track);
	}
	editor_track::commit(plains);
	switch_store::forget(Crossover.first->name());
	switch_store::forget(Crossover.second->name());
	First = plains[0];
	Second = plains[1];
	auto const &record{Crossover.record};
	if (std::abs(record.lift) <= 0.002 || record.ramp <= 0.0)
		return true;
	// the outer track is the one the level stretch of the lift lies on
	TTrack *outer{nullptr};
	for (auto *plain : plains)
		if (plan_distance(editor_track::point_at(*plain, editor_track::nearest_parameter(*plain, record.lift_from)), record.lift_from) < 1.0 ||
		    plan_distance(editor_track::point_at(*plain, editor_track::nearest_parameter(*plain, record.lift_to)), record.lift_to) < 1.0)
			outer = plain;
	if (outer == nullptr)
		return true;
	auto route{editor_track::run_route(*outer, 3000.0)};
	auto const samples{editor_track::sample_route(route, 1.0)};
	if (samples.size() < 2)
		return true;
	auto const chainage_of = [&](glm::dvec3 const &Point) {
		double best{std::numeric_limits<double>::max()};
		double chainage{0.0};
		for (std::size_t i = 0; i + 1 < samples.size(); ++i)
		{
			auto const a{plan_of(samples[i].position)};
			auto const run{plan_of(samples[i + 1].position) - a};
			auto const t{std::clamp(glm::dot(plan_of(Point) - a, run) / std::max(1e-12, glm::dot(run, run)), 0.0, 1.0)};
			if (auto const distance{glm::length(plan_of(Point) - (a + run * t))}; distance < best)
			{
				best = distance;
				chainage = samples[i].chainage + (samples[i + 1].chainage - samples[i].chainage) * t;
			}
		}
		return chainage;
	};
	auto c1{chainage_of(record.lift_from)};
	auto c2{chainage_of(record.lift_to)};
	if (c1 > c2)
		std::swap(c1, c2);
	auto const ramp{record.ramp};
	auto const share = [&](double const Chainage) {
		if (Chainage <= c1 - ramp || Chainage >= c2 + ramp)
			return 0.0;
		if (Chainage < c1)
			return (Chainage - (c1 - ramp)) / ramp;
		if (Chainage > c2)
			return ((c2 + ramp) - Chainage) / ramp;
		return 1.0;
	};
	auto const slope = [&](double const Chainage) {
		if (Chainage > c1 - ramp && Chainage < c1)
			return 1.0 / ramp;
		if (Chainage > c2 && Chainage < c2 + ramp)
			return -1.0 / ramp;
		return 0.0;
	};
	std::vector<double> breaks;
	for (auto const chainage : {c1 - ramp, c1, c2, c2 + ramp})
		if (chainage > 0.5 && chainage < route.length - 0.5)
			breaks.push_back(chainage);
	profile_take(route, route.spans.front().track, route.spans.back().track);
	auto const before{m_profile.line};
	editor_track::apply_profile(
	    route, [&](double const Chainage) { return editor_track::sampled_elevation(samples, Chainage) - record.lift * share(Chainage); },
	    [&](double const Chainage) { return editor_track::sampled_grade(samples, Chainage) - record.lift * slope(Chainage); }, breaks, States, Created);
	// the design of the outer track down with it, the corners of the ramps left out where the line runs on straight through them
	profile_take(route, route.spans.front().track, route.spans.back().track);
	std::vector<profile::pvi> points;
	for (auto point : before.points)
	{
		point.elevation -= record.lift * share(point.chainage);
		points.push_back(point);
	}
	for (std::size_t k = 1; k + 1 < points.size();)
	{
		auto const in{(points[k].elevation - points[k - 1].elevation) / std::max(1e-6, points[k].chainage - points[k - 1].chainage)};
		auto const out{(points[k + 1].elevation - points[k].elevation) / std::max(1e-6, points[k + 1].chainage - points[k].chainage)};
		if (std::abs(out - in) < 1e-7)
			points.erase(points.begin() + static_cast<std::ptrdiff_t>(k));
		else
			++k;
	}
	if (points.size() >= 2)
	{
		m_profile.line = before;
		m_profile.line.points = std::move(points);
		m_profile.selected = -1;
		profile_check();
		profile_store();
	}
	WriteLog(format("Editor: curved crossover - the outer track %s down by %.3f m again", outer->name().c_str(), record.lift), logtype::generic);
	return true;
}

// the crossover along a curve moved along it by the distance, from its first switch towards the second: taken out and laid again
bool editor_mode::slide_curved_connection(TTrack &Switch, double const Distance)
{
	auto const crossover{crossover_of(Switch)};
	if (false == crossover.has_value())
		return false;
	auto &tool{m_switch};
	auto const shape{crossover_template(crossover->record)};
	if (shape < 0)
	{
		tool.status = STR_C("The template the crossover was laid of isn't among the templates");
		ui()->set_status(tool.status, true);
		return false;
	}
	std::vector<std::string> const names{crossover->first->name(), crossover->second->name()};
	auto const firststart{crossover->first->m_paths.front().points[segment_data::point::start]};
	auto const firstend{crossover->first->m_paths.front().points[segment_data::point::end]};
	auto const secondstart{crossover->second->m_paths.front().points[segment_data::point::start]};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	TTrack *first{nullptr};
	TTrack *second{nullptr};
	// the design of the outer track comes down with the lift taken back, an undo puts both back
	profile_library_load();
	auto library{m_profiles};
	if (false == take_out_crossover(*crossover, states, created, removed, first, second))
		return false;
	// the new places of the press and of the release, along the two curves, or the two straights
	std::vector<curve_sample> firstframe, secondframe;
	std::vector<TTrack *> firsttracks, secondtracks;
	glm::dvec3 press, release;
	if (curve_of(*first, m_straights.tolerance, m_route.design.norms.gauge, firstframe, firsttracks, kStraightReach) && curve_of(*second, m_straights.tolerance, m_route.design.norms.gauge, secondframe, secondtracks, kStraightReach))
	{
		auto const towards{nearest_station(firstframe, firstend) >= nearest_station(firstframe, firststart) ? 1.0 : -1.0};
		press = sample_at(firstframe, nearest_station(firstframe, firststart) + towards * Distance).position;
		auto const ahead{nearest_station(secondframe, secondstart) >= nearest_station(secondframe, firststart) ? 1.0 : -1.0};
		release = sample_at(secondframe, nearest_station(secondframe, secondstart) + ahead * Distance).position;
	}
	else
	{
		auto direction{plan_of(firstend - firststart)};
		direction = glm::length(direction) > 1e-9 ? glm::normalize(direction) : glm::dvec2{0.0, 1.0};
		press = firststart + glm::dvec3{direction.x, 0.0, direction.y} * Distance;
		release = secondstart + glm::dvec3{direction.x, 0.0, direction.y} * Distance;
		firsttracks = {first};
	}
	switch_store::follow();
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	records_to_history();
	if (std::abs(crossover->record.lift) > 0.002)
		profiles_to_history(std::move(library));
	// laid again
	if (false == lay_crossover(shape, press, release, 3.0, names))
	{
		auto const reason{tool.status};
		undo_last();
		tool.status = reason.empty() ? std::string{STR_C("The crossover doesn't fit there")} : reason;
		ui()->set_status(tool.status, true);
		return false;
	}
	join_last_steps();
	tool.status = format(STR_C("Crossover moved %.2f m"), Distance) + "\n" + tool.status;
	ui()->set_status(tool.status, false);
	return true;
}

std::size_t editor_mode::follow_crossovers(std::vector<editor_track::route_sample> const &Before)
{
	auto &state{m_profile};
	auto const changedroute{state.route};
	auto *changedfrom{state.from};
	auto *changedto{state.to};
	auto const after{state.samples};
	auto const changedline{state.line};
	if (Before.size() < 2 || after.size() < 2)
		return 0;
	double constexpr follow{25.0};
	double constexpr leave{50.0};
	// how much the route went up at the point beside it, and how far the point is from it
	auto const change_at = [&](glm::dvec3 const &Point, double &Distance, double &Chainage) {
		Distance = std::numeric_limits<double>::max();
		Chainage = 0.0;
		for (std::size_t i = 0; i + 1 < after.size(); ++i)
		{
			auto const a{plan_of(after[i].position)};
			auto const run{plan_of(after[i + 1].position) - a};
			auto const t{std::clamp(glm::dot(plan_of(Point) - a, run) / std::max(1e-12, glm::dot(run, run)), 0.0, 1.0)};
			if (auto const distance{glm::length(plan_of(Point) - (a + run * t))}; distance < Distance)
			{
				Distance = distance;
				Chainage = after[i].chainage + (after[i + 1].chainage - after[i].chainage) * t;
			}
		}
		auto const weight{std::clamp((leave - Distance) / (leave - follow), 0.0, 1.0)};
		return weight * (editor_track::sampled_elevation(after, Chainage) - editor_track::sampled_elevation(Before, Chainage));
	};
	std::vector<TTrack const *> done;
	std::size_t followed{0};
	for (auto const &span : changedroute.spans)
	{
		if (span.track == nullptr || span.track->eType != tt_Switch)
			continue;
		auto const crossover{crossover_of(*span.track)};
		if (false == crossover.has_value())
			continue;
		auto *other{crossover->first == span.track ? crossover->second : crossover->first};
		// on the same grade line it goes with the route anyway, grade_line_follow sees to it
		if (m_profile.whole_grade_line && on_route_grade_line(*other))
			continue;
		if (std::any_of(changedroute.spans.begin(), changedroute.spans.end(), [&](auto const &Span) { return Span.track == other; }) || std::find(done.begin(), done.end(), other) != done.end())
			continue;
		done.push_back(other);
		auto route{editor_track::run_route(*other, 3000.0)};
		auto const samples{editor_track::sample_route(route, 1.0)};
		if (samples.size() < 2)
			continue;
		// the change of each point of the other line
		std::vector<double> changes(samples.size(), 0.0);
		std::vector<double> distances(samples.size(), 0.0);
		bool any{false};
		for (std::size_t i = 0; i < samples.size(); ++i)
		{
			double chainage;
			changes[i] = change_at(samples[i].position, distances[i], chainage);
			any = any || std::abs(changes[i]) > 0.001;
		}
		if (false == any)
			continue;
		auto const change = [&](double const Chainage) {
			auto const at{std::clamp(Chainage, samples.front().chainage, samples.back().chainage)};
			auto const next{std::lower_bound(samples.begin(), samples.end(), at, [](auto const &Sample, double const Value) { return Sample.chainage < Value; })};
			if (next == samples.begin())
				return changes.front();
			if (next == samples.end())
				return changes.back();
			auto const i{static_cast<std::size_t>(std::distance(samples.begin(), next))};
			auto const f{(at - samples[i - 1].chainage) / std::max(1e-9, samples[i].chainage - samples[i - 1].chainage)};
			return changes[i - 1] + (changes[i] - changes[i - 1]) * f;
		};
		// where the grade line of the route breaks, the other line breaks too
		std::vector<double> breaks;
		for (auto const &point : changedline.points)
		{
			auto const at{editor_track::sampled_position(after, point.chainage)};
			double best{std::numeric_limits<double>::max()};
			double chainage{0.0};
			for (auto const &sample : samples)
				if (auto const distance{plan_distance(sample.position, at)}; distance < best)
				{
					best = distance;
					chainage = sample.chainage;
				}
			if (best < follow && chainage > 0.5 && chainage < route.length - 0.5)
				breaks.push_back(chainage);
		}
		// and where it starts and stops running beside the route
		for (std::size_t i = 1; i < samples.size(); ++i)
			for (auto const bound : {follow, leave})
				if ((distances[i - 1] < bound) != (distances[i] < bound) && samples[i].chainage > 0.5 && samples[i].chainage < route.length - 0.5)
					breaks.push_back(samples[i].chainage);
		std::sort(breaks.begin(), breaks.end());
		breaks.erase(std::unique(breaks.begin(), breaks.end(), [](double const A, double const B) { return std::abs(A - B) < 1.0; }), breaks.end());
		profile_take(route, route.spans.front().track, route.spans.back().track);
		auto const designed{m_profile.line};
		std::vector<std::pair<TTrack *, editor_track::state>> states;
		std::vector<TTrack *> created;
		editor_track::apply_profile(
		    route, [&](double const Chainage) { return editor_track::sampled_elevation(samples, Chainage) + change(Chainage); },
		    [&](double const Chainage) { return editor_track::sampled_grade(samples, Chainage) + (change(Chainage + 0.5) - change(Chainage - 0.5)); }, breaks, states, created);
		profile_library_load();
		auto library{m_profiles};
		push_track_snapshot(std::move(states), std::move(created));
		m_history.back().profiles = true;
		m_history.back().profile_library = std::move(library);
		join_last_steps();
		// its design changed by as much
		profile_take(route, route.spans.front().track, route.spans.back().track);
		std::vector<profile::pvi> points;
		for (auto point : designed.points)
		{
			point.elevation += change(point.chainage);
			points.push_back(point);
		}
		for (auto const chainage : breaks)
		{
			profile::pvi point;
			point.chainage = chainage;
			point.elevation = profile::elevation(designed, chainage) + change(chainage);
			points.push_back(point);
		}
		std::sort(points.begin(), points.end(), [](auto const &Left, auto const &Right) { return Left.chainage < Right.chainage; });
		std::vector<profile::pvi> line;
		for (auto const &point : points)
			if (line.empty() || point.chainage - line.back().chainage >= 0.5)
				line.push_back(point);
		if (line.size() >= 2)
		{
			m_profile.line = designed;
			m_profile.line.points = std::move(line);
			m_profile.selected = -1;
			profile_check();
			profile_store();
		}
		++followed;
		WriteLog(format("Editor: vertical profile - the track %s across the crossover %s followed the change", other->name().c_str(), crossover->first->name().c_str()), logtype::generic);
	}
	if (followed > 0)
	{
		// the route the user works on open again
		profile_take(changedroute, changedfrom, changedto);
	}
	return followed;
}

int editor_mode::crossover_template(switch_store::record const &Record)
{
	auto &tool{m_switch};
	show_track_tab(track_tab::turnout);
	arm_switch();
	for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
	{
		auto const &candidate{tool.templates[i]};
		if (candidate.source == nullptr && false == candidate.double_slip && false == candidate.twofold() && std::abs(candidate.radius - Record.radius) < 0.5 && std::abs(candidate.ratio - Record.ratio) < 0.01 && (Record.rail == "-" || candidate.rail == Record.rail))
			return i;
	}
	// switches of no template of the catalogue, as the digitized ones of the old sceneries: the first one is the template
	if (Record.source != "measured" || Record.parts.empty())
		return -1;
	auto own{editor_track::template_of_switch(simulation::Paths.find(Record.parts.front()))};
	if (false == own.has_value())
		return -1;
	tool.templates.push_back(std::move(*own));
	return static_cast<int>(tool.templates.size()) - 1;
}

bool editor_mode::lay_crossover(int const Shape, glm::dvec3 const &Press, glm::dvec3 const &Release, double const Reach, std::vector<std::string> const &Names)
{
	auto &tool{m_switch};
	tool.keep_names = Names;
	struct forget_names
	{
		std::vector<std::string> &names;
		~forget_names() { names.clear(); }
	} const forget{tool.keep_names};
	// the plain path the press falls on
	TTrack *closest{nullptr};
	auto nearest{Reach};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty())
			continue;
		if (auto const distance{plan_distance(editor_track::point_at(*track, editor_track::nearest_parameter(*track, Press)), Press)}; distance < nearest)
		{
			nearest = distance;
			closest = track;
		}
	}
	if (closest == nullptr || Shape < 0)
		return false;
	show_track_tab(track_tab::turnout);
	arm_switch();
	tool.armed = Shape;
	tool.connection = static_cast<int>(connection::kind::crossover);
	tool.connection_second = -1;
	m_node = closest;
	ui()->set_node(m_node);
	// the press onto the path, where the line went sideways
	auto const press{editor_track::point_at(*closest, editor_track::nearest_parameter(*closest, Press))};
	m_cursor_override = press;
	auto const started{start_switch_placement()};
	m_cursor_override.reset();
	if (false == started)
		return false;
	tool.mouse = {Release.x, tool.point.y, Release.z};
	auto const steps{m_history.size()};
	if (tool.curved)
		insert_curved_connection();
	else
		insert_connection();
	if (m_history.size() == steps)
		return false;
	// both switches joined to their tracks at the start and at the end of the main track, else the line under them moved
	// and it's taken back
	for (auto *track : std::vector<TTrack *>{m_node != nullptr ? dynamic_cast<TTrack *>(m_node) : nullptr})
		if (auto const crossover{track != nullptr ? crossover_of(*track) : std::nullopt})
			for (auto *part : {crossover->first, crossover->second})
				for (auto const which : {segment_data::point::start, segment_data::point::end})
				{
					auto const joined{editor_track::connected_points(*part, part->m_paths.front().points[which])};
					if (std::none_of(joined.begin(), joined.end(), [&](auto const &Entry) { return Entry.first != part && Entry.first != nullptr && false == Entry.first->m_editorremoved; }))
					{
						undo_last();
						tool.status = STR_C("The crossover wouldn't join the tracks there");
						return false;
					}
				}
	return true;
}

/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorNormsReport.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"

#include "utilities/translation.h"
#include "world/Track.h"

#include <algorithm>
#include <cmath>

namespace norms_report
{

namespace
{

double constexpr kBase{1500.0}; // mm between the rails the cant is measured on, as the simulator takes the rolls (Segment.cpp)
double constexpr kStraight{1.0 / 30000.0};

// the speed limit of the path, 0 for none (or a damaged track): a switch keeps one for both its tracks, -2 and below for the diverging
// one alone
double path_speed(TTrack &Track, int const Path)
{
	auto velocity{Track.VelocityGet()};
	if (Track.SwitchExtension && (Track.eType == tt_Switch || Track.eType == tt_Cross))
		velocity = Track.SwitchExtension->fVelocity;
	if (Track.eType == tt_Switch && velocity <= -2.0)
		return Path == 0 ? 0.0 : -velocity;
	return velocity > 0.0 ? velocity : 0.0;
}

double curvature_of(geometry::bezier const &Curve, double const T)
{
	auto const first{Curve.first(T)};
	auto const second{Curve.second(T)};
	auto const planar{std::hypot(first.x, first.z)};
	return planar > 1e-9 ? (first.x * second.z - first.z * second.x) / (planar * planar * planar) : 0.0;
}

unsigned features_of(TTrack const &Track, int const Path)
{
	unsigned result{0};
	if (Track.eType == tt_Cross)
		return norms::feature::switch_track | norms::feature::crossing;
	if (Track.eType != tt_Switch)
		return result;
	result |= norms::feature::switch_track;
	if (Path > 0)
		result |= norms::feature::diverging;
	if (Track.SwitchExtension && Track.SwitchExtension->m_movablenose)
		result |= norms::feature::movable_frog;
	if (false == Track.m_paths.empty() && std::abs(curvature_of(geometry::bezier{Track.m_paths[0]}, 0.5)) > kStraight)
		result |= norms::feature::curved_switch;
	return result;
}

std::string label(norms::profile_id const Profile)
{
	return std::string{norms::profile_of(Profile).label};
}

// the place of the limit, with its document where that isn't the one the profile or the hard limit is named after
std::string place(norms::finding const &Finding, std::string_view const Named)
{
	if (Finding.from.document == Named || Finding.from.document.empty())
		return std::string{Finding.from.place};
	return std::string{Finding.from.document} + " " + std::string{Finding.from.place};
}


std::string kind_text(norms::element const &Element)
{
	auto const radius = [](double const Curvature) { return std::abs(Curvature) > 1e-9 ? format("%.0f", 1.0 / Curvature) : std::string{"-"}; };
	switch (Element.kind)
	{
	case norms::element_kind::arc: return format(STR_C("arc R %s"), radius(Element.curvature_start).c_str());
	case norms::element_kind::transition: return format(STR_C("transition R %s -> %s"), radius(Element.curvature_start).c_str(), radius(Element.curvature_end).c_str());
	default: return STR_C("straight");
	}
}

} // namespace

sampled_line sample(std::vector<span> const &Line, double const Step, bool const TrackSpeeds)
{
	sampled_line result;
	double chainage{0.0};
	for (auto const &span : Line)
	{
		auto const from{chainage};
		if (span.track == nullptr || span.path < 0 || span.path >= static_cast<int>(span.track->m_paths.size()))
		{
			result.spans.emplace_back(from, from);
			continue;
		}
		auto const &path{span.track->m_paths[span.path]};
		// a path made of several curves is sampled along them, its own curve stands in for them in older versions only
		std::vector<segment_data const *> parts;
		if (path.pieces.empty())
			parts.push_back(&path);
		else
			for (auto const &part : path.pieces)
				parts.push_back(&part);
		if (false == span.forward)
			std::ranges::reverse(parts);
		auto const speed{TrackSpeeds ? path_speed(*span.track, span.path) : 0.0};
		auto const features{features_of(*span.track, span.path)};
		auto const side{span.forward ? 1.0 : -1.0};
		for (auto const *part : parts)
		{
			geometry::bezier const curve{*part};
			auto const count{std::max(2, static_cast<int>(std::ceil(curve.plan_length(16) / std::max(0.1, Step))))};
			auto previous{geometry::plan_of(curve.point(span.forward ? 0.0 : 1.0))};
			for (int j = 0; j <= count; ++j)
			{
				auto const share{static_cast<double>(j) / count};
				auto const t{span.forward ? share : 1.0 - share};
				auto const position{curve.point(t)};
				chainage += glm::distance(previous, geometry::plan_of(position));
				previous = geometry::plan_of(position);
				auto const along{curve.first(t) * side};
				auto const planar{std::hypot(along.x, along.z)};
				auto const roll{glm::radians(static_cast<double>(part->rolls[0]) + static_cast<double>(part->rolls[1] - part->rolls[0]) * t)};
				norms::sample next;
				next.chainage = chainage;
				next.at = {position.x, position.z};
				next.grade = planar > 1e-9 ? along.y / planar : 0.0;
				next.curvature = curvature_of(curve, t) * side;
				next.cant = kBase * std::sin(roll) * side;
				next.speed = speed;
				next.features = features;
				result.samples.push_back(next);
			}
		}
		result.spans.emplace_back(from, chainage);
	}
	return result;
}

checked_line check(std::vector<span> const &Line, options const &Options, norms::area_map const *Areas, int const Own)
{
	checked_line result;
	if (Line.empty())
		return result;
	auto const sampled{sample(Line, Options.step, Options.track_speeds)};
	std::string name;
	if (Line.front().track != nullptr)
		name = Line.front().track->name();
	result.described = norms::describe(sampled.samples, name);
	norms::area_map own_areas;
	own_areas.scenery = Options.profile;
	auto const findings{norms::evaluate(result.described, Options.line, Areas != nullptr ? *Areas : own_areas)};
	for (auto const &finding : findings)
	{
		// the span the finding begins on: the last one starting at or before it, past an empty one
		std::size_t index{0};
		for (std::size_t i = 0; i < sampled.spans.size(); ++i)
			if (sampled.spans[i].first <= finding.chainage + 1e-6 && sampled.spans[i].second > sampled.spans[i].first)
				index = i;
		if (Own >= 0 && static_cast<int>(index) != Own)
			continue;
		auto const &span{Line[index]};
		entry next;
		next.finding = finding;
		next.kind = finding.element < result.described.elements.size() ? result.described.elements[finding.element].kind : norms::element_kind::straight;
		next.line = name;
		next.track = span.track;
		next.path = span.path;
		if (span.track != nullptr && span.path >= 0 && span.path < static_cast<int>(span.track->m_paths.size()))
		{
			geometry::bezier const curve{span.track->m_paths[span.path]};
			auto const [from, to]{sampled.spans[index]};
			auto const into{std::clamp(finding.chainage - from, 0.0, to - from)};
			next.position = curve.point(std::clamp(curve.parameter(span.forward ? into : (to - from) - into), 0.0, 1.0));
		}
		result.entries.push_back(next);
	}
	return result;
}

char const *traffic_name(norms::traffic const Kind)
{
	switch (Kind)
	{
	case norms::traffic::passenger: return STR_C("passenger traffic");
	case norms::traffic::freight: return STR_C("freight traffic");
	default: return STR_C("mixed traffic");
	}
}

char const *tier_name(norms::tier const Tier)
{
	switch (Tier)
	{
	case norms::tier::prefer: return STR_C("note");
	case norms::tier::warn: return STR_C("warning");
	case norms::tier::block: return STR_C("hard limit");
	default: return "-";
	}
}

char const *parameter_name(norms::parameter const What, norms::element_kind const Kind)
{
	using enum norms::parameter;
	switch (What)
	{
	case cant_deficiency: return STR_C("cant deficiency");
	case cant_excess: return STR_C("cant excess");
	case deficiency_jump: return STR_C("abrupt change of the cant deficiency");
	case cant_gradient: return STR_C("cant ramp");
	case cant_rate: return STR_C("cant change rate");
	case deficiency_rate: return STR_C("cant deficiency change rate");
	case element_length: return Kind == norms::element_kind::arc ? STR_C("arc") : STR_C("straight");
	case transition_length: return STR_C("transition curve");
	case transition_shift: return STR_C("shift of the arc by its transition");
	case radius: return STR_C("radius");
	case cant:
	case cant_min: return STR_C("cant");
	case vertical_radius: return STR_C("vertical curve");
	case grade: return STR_C("grade");
	case profile_boundary: return STR_C("profile boundary");
	default: return "-";
	}
}

std::string value_text(norms::parameter const What, double const Value)
{
	using enum norms::parameter;
	switch (What)
	{
	case cant_gradient: return format("%.2f mm/m", Value);
	case cant_rate:
	case deficiency_rate: return format("%.0f mm/s", Value);
	case element_length:
	case transition_length:
	case intermediate_length: return format("%.1f m", Value);
	case transition_shift: return format("%.3f m", Value);
	case radius:
	case vertical_radius: return format("%.0f m", Value);
	case grade: return format(STR_C("%.1f per mille"), Value);
	default: return format("%.0f mm", Value);
	}
}

std::string message(norms::finding const &Finding, norms::element_kind const Kind)
{
	if (Finding.what == norms::parameter::profile_boundary)
		return format(STR_C("arc across the boundary of %s and %s, to be settled"), label(Finding.profile).c_str(), label(Finding.other).c_str());
	// the limit named by the profile, or by its document when hard; the place with its document where that's another one
	auto const &profile{norms::profile_of(Finding.profile)};
	auto const who{Finding.hard ? std::string{Finding.from.document} : std::string{profile.label}};
	auto const where{place(Finding, Finding.hard ? Finding.from.document : profile.document)};
	auto const limit{value_text(Finding.what, Finding.limit)};
	std::string text;
	if (Finding.what == norms::parameter::vertical_radius && Finding.value <= 0.0)
		text = format(STR_C("break of the grade with no vertical curve, %s asks for a radius of at least %s (%s)"), who.c_str(), limit.c_str(), where.c_str());
	else
	{
		auto const least{norms::minimum(Finding.what)};
		char const *pattern{nullptr};
		if (Finding.level == norms::tier::prefer)
			pattern = least ? STR_C("%s %s, %s recommends at least %s (%s)") : STR_C("%s %s, %s recommends at most %s (%s)");
		else
			pattern = least ? STR_C("%s %s, %s asks for at least %s (%s)") : STR_C("%s %s, %s allows at most %s (%s)");
		text = format(pattern, parameter_name(Finding.what, Kind), value_text(Finding.what, Finding.value).c_str(), who.c_str(), limit.c_str(), where.c_str());
	}
	if (false == Finding.confirmed)
		text += STR_C(", the limit not confirmed");
	return text;
}

std::string header(options const &Options)
{
	auto const &line{Options.line};
	auto const vmin{line.vmin > 0.0 ? line.vmin : norms::slow_speed(line.kind, line.vmax)};
	auto text{format(STR_C("Norms: %s, line %.0f km/h, slow trains %.0f km/h, %s, %s, %.0f Tg a year"), std::string{norms::profile_of(Options.profile).name}.c_str(), line.vmax, vmin,
	                 traffic_name(line.kind), line.track == norms::track_kind::main ? STR_C("main track") : STR_C("siding"), line.load)};
	if (Options.track_speeds)
		text += STR_C(", the speed of the tracks where they have one");
	return text + "\n";
}

std::string text(checked_line const &Line, bool const Elements)
{
	auto const &line{Line.described};
	auto const length{line.elements.empty() ? 0.0 : line.elements.back().end()};
	auto result{format(STR_C("Line %s: %.1f m, %zu elements, %zu findings"), line.name.c_str(), length, line.elements.size(), Line.entries.size()) + "\n"};
	if (Elements)
	{
		for (auto const &element : line.elements)
			result += format("  %8.1f %8.1f  %s", element.start, element.length, kind_text(element).c_str()) + (element.speed > 0.0 ? format("  %.0f km/h", element.speed) : std::string{}) +
			          ((element.features & norms::feature::switch_track) != 0 ? std::string{"  "} + STR_C("switch") : std::string{}) + "\n";
		result += STR_C("  cant (m: mm):");
		for (auto const &point : line.cant)
			result += format(" %.1f: %.1f", point.chainage, point.cant);
		result += "\n";
		result += STR_C("  grade (m: per mille):");
		for (auto const &piece : line.grades)
			result += format(" %.1f: %.2f", piece.start, piece.grade_start * 1000.0) + (piece.grade_end != piece.grade_start ? format(" .. %.2f", piece.grade_end * 1000.0) : std::string{});
		result += "\n";
	}
	for (auto const &entry : Line.entries)
	{
		auto const &finding{entry.finding};
		auto const track{entry.track != nullptr ? entry.track->name() : std::string{}};
		result += format("  %8.1f  %-10s  %s  [%s]", finding.chainage, tier_name(finding.level), message(finding, entry.kind).c_str(), track.c_str()) + "\n";
	}
	return result;
}

} // namespace norms_report

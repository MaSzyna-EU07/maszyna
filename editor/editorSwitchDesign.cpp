/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorSwitchDesign.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "world/RailLayout.h"

#include "utilities/translation.h"
#include "utilities/utilities.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <sstream>

namespace switch_design
{

namespace
{

double constexpr kPi{3.14159265358979323846};
double constexpr kMinRadius{10.0};
double constexpr kTransitionStep{5.0};
double constexpr kLink{1.0};
double constexpr kTipStep{0.01};
double constexpr kHalfGauge{0.7175};
double constexpr kSecantAngle{25.0 / 60.0};
double constexpr kSecantLead{0.6};
double constexpr kCutTips[][2]{{190.0, 1.2483}, {300.0, 1.5}, {500.0, 2.165}, {760.0, 2.633}, {1200.0, 3.3347}};

glm::dvec2 heading_of(double const Angle)
{
	return {std::sin(Angle), std::cos(Angle)};
}

glm::dvec3 spatial(glm::dvec2 const &Plan)
{
	return {Plan.x, 0.0, Plan.y};
}

double curvature_of(double const Radius)
{
	return Radius != 0.0 ? 1.0 / Radius : 0.0;
}

segment_data piece(glm::dvec2 const &From, glm::dvec2 const &To, double const Heading0, double const Heading1, double const Handle0, double const Handle1, double const Radius)
{
	segment_data result;
	result.points[segment_data::point::start] = spatial(From);
	result.points[segment_data::point::end] = spatial(To);
	if (Handle0 > 0.0 || Handle1 > 0.0)
	{
		result.points[segment_data::point::control1] = spatial(heading_of(Heading0) * Handle0);
		result.points[segment_data::point::control2] = spatial(-heading_of(Heading1) * Handle1);
	}
	result.radius = static_cast<float>(std::abs(Radius));
	return result;
}

struct walker
{
	glm::dvec2 at{0.0};
	double heading{0.0};
	double sharpest{0.0};
	std::vector<segment_data> pieces;

	void note(double const Radius)
	{
		if (Radius != 0.0 && (sharpest == 0.0 || std::abs(Radius) < sharpest))
			sharpest = std::abs(Radius);
	}

	void straight(double const Length)
	{
		auto const to{at + heading_of(heading) * Length};
		pieces.push_back(piece(at, to, heading, heading, 0.0, 0.0, 0.0));
		at = to;
	}

	void arc(double const Length, double const Radius)
	{
		note(Radius);
		auto const angle{Length / Radius};
		auto const count{std::max(1, static_cast<int>(std::ceil(std::abs(angle) / (kPi * 0.5) - 1e-9)))};
		auto const step{angle / count};
		auto const handle{4.0 / 3.0 * std::abs(Radius * std::tan(step * 0.25))};
		for (int i = 0; i < count; ++i)
		{
			auto const chord{2.0 * std::abs(Radius * std::sin(step * 0.5))};
			auto const to{at + heading_of(heading + step * 0.5) * chord};
			pieces.push_back(piece(at, to, heading, heading + step, handle, handle, Radius));
			at = to;
			heading += step;
		}
	}

	void transition(double const Length, double const Radius1, double const Radius2)
	{
		note(Radius1);
		note(Radius2);
		auto const k1{curvature_of(Radius1)};
		auto const k2{curvature_of(Radius2)};
		auto const start{heading};
		auto const heading_at = [&](double const S) { return start + k1 * S + (k2 - k1) * S * S / (2.0 * Length); };
		auto const count{std::max(2, static_cast<int>(std::ceil(Length / kTransitionStep)))};
		auto const step{Length / count};
		for (int i = 0; i < count; ++i)
		{
			auto const s0{step * i};
			int const samples{32};
			glm::dvec2 offset{0.0};
			for (int j = 0; j <= samples; ++j)
			{
				auto const weight{j == 0 || j == samples ? 1.0 : (j % 2 == 1 ? 4.0 : 2.0)};
				offset += heading_of(heading_at(s0 + step * j / samples)) * weight;
			}
			offset *= step / (3.0 * samples);
			auto const h0{heading_at(s0)};
			auto const h1{heading_at(s0 + step)};
			auto const middle{curvature_of(Radius1) + (k2 - k1) * (s0 + step * 0.5) / Length};
			auto const handle{glm::length(offset) / 3.0};
			pieces.push_back(piece(at, at + offset, h0, h1, handle, handle, middle != 0.0 ? 1.0 / middle : 0.0));
			at += offset;
		}
		heading = heading_at(Length);
	}
};

void mirror(segment_data &Path)
{
	auto const flip = [](glm::dvec3 const &Point) { return glm::dvec3{-Point.x, Point.y, Point.z}; };
	Path.transform(flip, flip);
}

void shift(segment_data &Path, double const Along)
{
	Path.transform([&](glm::dvec3 const &Point) { return Point + glm::dvec3{0.0, 0.0, Along}; }, [](glm::dvec3 const &Vector) { return Vector; });
}

segment_data straight_path(double const From, double const To)
{
	segment_data result;
	result.points[segment_data::point::start] = {0.0, 0.0, From};
	result.points[segment_data::point::end] = {0.0, 0.0, To};
	return result;
}

std::string checked(std::vector<part> const &Parts)
{
	if (Parts.empty())
		return "The diverging track has no parts";
	for (auto const &part : Parts)
	{
		if (false == (part.length >= 0.05))
			return "A part of the diverging track is shorter than 5 cm";
		if (part.length > 1000.0)
			return "A part of the diverging track is longer than 1000 m";
		for (auto const radius : {part.radius1, part.radius2})
			if (std::isnan(radius) || (radius != 0.0 && std::abs(radius) < kMinRadius))
				return "A radius of the diverging track is under 10 m";
	}
	return {};
}

// the point where the tangent at the end of the diverging track meets the main track, as the distance along it
double tangent_point(glm::dvec2 const &End, double const Angle)
{
	return End.y - End.x / std::tan(Angle);
}

std::string word(std::string const &Text)
{
	if (Text.empty())
		return "-";
	auto result{Text};
	std::replace(result.begin(), result.end(), ' ', '_');
	return result;
}

std::string unword(std::string const &Text)
{
	if (Text == "-")
		return {};
	auto result{Text};
	std::replace(result.begin(), result.end(), '_', ' ');
	return result;
}

std::filesystem::path own_path()
{
	auto path{user_config_path("eu07_switch_templates.txt")};
	return path.empty() ? std::filesystem::path{"eu07_switch_templates.txt"} : path;
}

void write_parts(std::ostream &Output, std::vector<part> const &Parts)
{
	Output << ' ' << Parts.size();
	for (auto const &part : Parts)
		Output << ' ' << format("%.4f %.4f %.4f", part.length, part.radius1, part.radius2);
}

} // namespace

bool double_switch(std::string const &Type)
{
	return Type == "Rpd" || Type == "Rpj" || Type == "Rps";
}

bool symmetric_switch(std::string const &Type)
{
	return Type == "Rls";
}

path diverging(std::vector<part> const &Parts, double const Lead, double const Entry, double const Tip)
{
	walker walk;
	if (Lead > 0.0)
		walk.straight(Lead);
	auto const start{Lead > 0.0 ? 0.0 : glm::radians(Entry)};
	walk.heading += glm::radians(Entry);
	auto const lay = [&](double const Length, double const Radius1, double const Radius2) {
		if (Radius1 == 0.0 && Radius2 == 0.0)
			walk.straight(Length);
		else if (Radius1 == Radius2)
			walk.arc(Length, Radius1);
		else
			walk.transition(Length, Radius1, Radius2);
	};
	auto split{Entry > 0.0 ? 0.0 : Tip - Lead};
	for (auto const &part : Parts)
	{
		if (split > kTipStep && split < part.length - kTipStep)
		{
			auto const k1{curvature_of(part.radius1)};
			auto const k2{curvature_of(part.radius2)};
			auto const k{k1 + (k2 - k1) * split / part.length};
			auto const middle{part.radius1 == part.radius2 ? part.radius1 : (k != 0.0 ? 1.0 / k : 0.0)};
			lay(split, part.radius1, middle);
			lay(part.length - split, middle, part.radius2);
		}
		else
			lay(part.length, part.radius1, part.radius2);
		split -= part.length;
	}
	path result;
	result.angle = walk.heading;
	result.radius = walk.sharpest;
	if (walk.pieces.size() == 1)
	{
		result.whole = walk.pieces.front();
		return result;
	}
	auto const end{walk.at};
	auto handle0{glm::length(end) / 3.0};
	auto handle1{handle0};
	if (std::abs(walk.heading) > 1e-6)
	{
		auto const along{tangent_point(end, walk.heading)};
		auto const back{end.x / std::sin(walk.heading)};
		if (along > 0.0 && back > 0.0)
		{
			handle0 = along * 2.0 / 3.0;
			handle1 = back * 2.0 / 3.0;
		}
	}
	result.whole = piece({0.0, 0.0}, end, start, walk.heading, handle0, handle1, walk.sharpest);
	result.whole.pieces = std::move(walk.pieces);
	return result;
}

double blade_tip(std::vector<part> const &Parts, double const Lead, double const Entry, double const Tip, bool const Symmetric)
{
	if (Entry > 0.0)
		return Lead;
	if (Tip > Lead)
		return Tip;
	auto const reach = [&](double const Gap) {
		glm::dvec2 at{0.0, Lead};
		auto heading{0.0};
		auto apart{0.0};
		for (auto const &part : Parts)
		{
			auto const k1{curvature_of(part.radius1)};
			auto const k2{curvature_of(part.radius2)};
			auto const steps{std::max(1, static_cast<int>(std::ceil(part.length / kTipStep)))};
			auto const step{part.length / steps};
			for (int i = 0; i < steps; ++i)
			{
				auto const k{k1 + (k2 - k1) * (i + 0.5) / steps};
				auto const next{at + heading_of(heading + 0.5 * k * step) * step};
				heading += k * step;
				auto const nextapart{(Symmetric ? 2.0 : 1.0) * (next.x + kHalfGauge * (1.0 - std::cos(heading)))};
				if (nextapart >= Gap)
					return glm::mix(at.y, next.y, std::clamp((Gap - apart) / std::max(nextapart - apart, 1e-12), 0.0, 1.0));
				at = next;
				apart = nextapart;
			}
		}
		return Lead; };
	return std::max({Lead, reach(track_rails::blade_point), reach(track_rails::blade_nose) - track_rails::blade_planed});
}

double cut_tip(std::string const &Rail, double const Radius)
{
	if (Rail != "60E1" && Rail != "49E1")
		return 0.0;
	for (auto const &cut : kCutTips)
		if (std::abs(Radius - cut[0]) < 0.5)
			return cut[1];
	return 0.0;
}

std::pair<double, double> secant_blade(std::string const &Rail, double const Radius, double const Ratio)
{
	if (Rail != "S42" || Ratio <= 0.0)
		return {0.0, 0.0};
	for (auto const &entry : catalogue_entries())
	{
		if (entry.type != "Rz" || entry.rail != Rail || entry.entry <= 0.0 || entry.parts.empty() || std::abs(entry.parts.front().radius1 - Radius) > 0.5)
			continue;
		auto turn{glm::radians(entry.entry)};
		for (auto const &part : entry.parts)
			turn += 0.5 * (curvature_of(part.radius1) + curvature_of(part.radius2)) * part.length;
		if (std::abs(turn - std::atan(1.0 / Ratio)) < 1e-4)
			return {entry.lead, glm::radians(entry.entry)};
	}
	return {kSecantLead, glm::radians(kSecantAngle)};
}

shape build(design const &Design, std::string &Error)
{
	shape result;
	Error.clear();
	bool const twofold{double_switch(Design.type)};
	bool const symmetric{symmetric_switch(Design.type)};
	if (Design.type != "Rz" && false == twofold && false == symmetric)
	{
		Error = "Unknown type of switch";
		return result;
	}
	Error = checked(Design.parts);
	// both switches of a symmetric double switch have the shape of the first one
	bool const mirrored{Design.type == "Rps"};
	if (Error.empty() && twofold && false == mirrored && false == Design.parts2.empty())
		Error = checked(Design.parts2);
	if (Error.empty() && false == (Design.lead >= 0.0 && Design.lead <= 30.0))
		Error = "The pre-blade stretch has to be between 0 and 30 m";
	if (Error.empty() && false == (Design.entry >= 0.0 && Design.entry <= 5.0))
		Error = "The angle of the blade has to be between 0 and 5 degrees";
	if (false == Error.empty())
		return result;
	auto const tip{blade_tip(Design.parts, Design.lead, Design.entry, Design.tip, symmetric)};
	auto const first{diverging(Design.parts, Design.lead, Design.entry, tip)};
	if (first.angle <= 1e-4 || first.angle >= kPi / 4.0)
	{
		Error = "The diverging track has to leave to the side at an angle between 0 and 45 degrees";
		return result;
	}
	auto const end{glm::dvec2{first.whole.points[segment_data::point::end].x, first.whole.points[segment_data::point::end].z}};
	// the tracks of a symmetric switch part by twice the angle of each
	result.ratio = 1.0 / std::tan(symmetric ? 2.0 * first.angle : first.angle);
	result.radius = first.radius;
	result.offset = end.x;
	result.tip = tip;
	result.a = Design.a > 0.0 ? Design.a : tangent_point(end, first.angle);
	if (symmetric)
	{
		result.length = end.y;
		result.b = Design.b > 0.0 ? Design.b : result.length - result.a;
		auto other{first.whole};
		mirror(other);
		result.paths = {other, first.whole};
		return result;
	}
	if (false == twofold)
	{
		result.length = Design.main > 0.0 ? Design.main : end.y;
		if (result.length < 1.0)
		{
			Error = "The main track is shorter than 1 m";
			return result;
		}
		result.b = Design.b > 0.0 ? Design.b : result.length - result.a;
		result.paths = {straight_path(0.0, result.length), first.whole};
		return result;
	}
	auto const reach{end.y};
	if (Design.stagger < 1.0 || Design.stagger > reach - 1.0)
	{
		Error = "The second switch has to start at least 1 m along the main track and 1 m before the end of the diverging track";
		return result;
	}
	auto const lead2{Design.lead2 >= 0.0 && false == mirrored ? Design.lead2 : Design.lead};
	auto const entry2{Design.entry2 >= 0.0 && false == mirrored ? Design.entry2 : Design.entry};
	auto const &parts2{Design.parts2.empty() || mirrored ? Design.parts : Design.parts2};
	auto const tip2{blade_tip(parts2, lead2, entry2, Design.tip2 >= 0.0 && false == mirrored ? Design.tip2 : Design.tip, false)};
	if (lead2 > 30.0 || entry2 > 5.0)
	{
		Error = "The pre-blade stretch of the second switch has to be at most 30 m and the angle of its blade at most 5 degrees";
		return result;
	}
	auto second{diverging(parts2, lead2, entry2, tip2)};
	if (second.angle <= 1e-4 || second.angle >= kPi / 4.0)
	{
		Error = "The diverging track of the second switch has to leave to the side at an angle between 0 and 45 degrees";
		return result;
	}
	if (Design.type == "Rpd" || mirrored)
		mirror(second.whole);
	shift(second.whole, Design.stagger);
	auto const secondend{second.whole.points[segment_data::point::end].z};
	result.length = Design.main > 0.0 ? Design.main : std::max(reach, secondend);
	if (result.length < Design.stagger + 1.0)
	{
		Error = "The main track ends less than 1 m after the start of the second switch";
		return result;
	}
	result.b = Design.b > 0.0 ? Design.b : reach - result.a;
	result.stagger = Design.stagger;
	result.tip2 = Design.stagger + tip2;
	result.radius = std::min(first.radius, second.radius > 0.0 ? second.radius : first.radius);
	// the simulation joins switches only to plain tracks, so a short one lies between the two
	auto const link{std::min(kLink, Design.stagger * 0.5)};
	result.paths = {straight_path(0.0, Design.stagger - link), first.whole, straight_path(Design.stagger, result.length), second.whole, straight_path(Design.stagger - link, Design.stagger)};
	return result;
}

bool measure_clicks(std::vector<glm::dvec2> const &Clicks, measured &Result, std::string &Error)
{
	if (Clicks.size() < 4)
	{
		Error = "Too few points";
		return false;
	}
	auto const main{glm::normalize(Clicks[1] - Clicks[0])};
	auto way{glm::normalize(Clicks[3] - Clicks[2])};
	if (glm::dot(way, main) < 0.0)
		way = -way;
	auto const across{main.x * way.y - main.y * way.x};
	auto const angle{std::atan2(std::abs(across), glm::dot(main, way))};
	if (angle < glm::radians(1.0) || angle > glm::radians(30.0))
	{
		Error = "The tracks clicked don't part as a switch's do";
		return false;
	}
	auto const offset{Clicks[2] - Clicks[0]};
	Result.a = (offset.x * way.y - offset.y * way.x) / across;
	Result.b = (main.x * offset.y - main.y * offset.x) / across;
	Result.ratio = 1.0 / std::tan(angle);
	Result.main = glm::length(Clicks[1] - Clicks[0]);
	Result.past = glm::dot(offset, main) - Result.main;
	if (Result.a <= 0.0 || Result.b <= 0.0)
	{
		Error = "The end clicked doesn't lie past the start on the way of the track";
		return false;
	}
	if (Result.main <= Result.a)
	{
		Error = "The end of the main track clicked lies before the point where the tangents meet";
		return false;
	}
	// the circle through the points on the arc, by least squares of the algebraic distance
	Result.radius = 0.0;
	if (Clicks.size() >= 7)
	{
		glm::dvec2 middle{0.0};
		for (std::size_t i = 4; i < Clicks.size(); ++i)
			middle += Clicks[i];
		middle /= static_cast<double>(Clicks.size() - 4);
		double uu{0.0}, uv{0.0}, vv{0.0}, uuu{0.0}, vvv{0.0}, uvv{0.0}, vuu{0.0};
		for (std::size_t i = 4; i < Clicks.size(); ++i)
		{
			auto const point{Clicks[i] - middle};
			uu += point.x * point.x;
			uv += point.x * point.y;
			vv += point.y * point.y;
			uuu += point.x * point.x * point.x;
			vvv += point.y * point.y * point.y;
			uvv += point.x * point.y * point.y;
			vuu += point.y * point.x * point.x;
		}
		auto const determinant{uu * vv - uv * uv};
		if (std::abs(determinant) > 1e-9)
		{
			auto const u{(0.5 * (uuu + uvv) * vv - 0.5 * (vvv + vuu) * uv) / determinant};
			auto const v{(0.5 * (vvv + vuu) * uu - 0.5 * (uuu + uvv) * uv) / determinant};
			Result.radius = std::sqrt(u * u + v * v + (uu + vv) / static_cast<double>(Clicks.size() - 4));
		}
	}
	return true;
}

bool fit_tangents(design &Design, double const Radius, double const Ratio, double const A, double const B, tangent_fit const Kind, std::string &Error, double const First)
{
	if (Radius < 10.0 || Ratio <= 1.0)
	{
		Error = "The radius or the angle makes no switch";
		return false;
	}
	double constexpr kNear{0.02}; // m, what the tables round a tangent by
	double constexpr kFlattest{5000.0}; // m, an arc this flat is a straight
	auto const angle{std::atan(1.0 / Ratio)};
	auto const tangent{Radius * std::tan(angle * 0.5)};
	auto const a{A > 0.0 ? A : tangent};
	auto const b{B > 0.0 ? B : tangent};
	std::vector<part> parts;
	auto lead{0.0};
	auto entry{0.0};
	// two arcs leaving the main track tangentially: with the longer tangent L and the shorter S, the flatter arc turning first by t,
	// (R1 - R) sin t = L + S cos angle - R sin angle and (R1 - R) (1 - cos t) = S sin angle - R (1 - cos angle)
	auto const longer{std::max(a, b)};
	auto const shorter{std::min(a, b)};
	auto const along{longer + shorter * std::cos(angle) - Radius * std::sin(angle)};
	auto const off{shorter * std::sin(angle) - Radius * (1.0 - std::cos(angle))};
	if (Kind == tangent_fit::two_arcs && std::abs(a - b) > kNear && along > 0.0 && off > 0.0)
	{
		auto const turn{2.0 * std::atan2(off, along)};
		auto const flatter{Radius + along / std::sin(turn)};
		if (turn < angle && flatter <= kFlattest)
		{
			parts = {{flatter * turn, flatter, flatter}, {Radius * (angle - turn), Radius, Radius}};
			if (a < b)
				std::swap(parts[0], parts[1]);
		}
	}
	if (Kind == tangent_fit::arc_straight_arc)
	{
		// turned by t on the first arc: L sin t + (R1 - R) (1 - cos t) = b sin angle - R (1 - cos angle) and
		// L cos t + (R1 - R) sin t = a + b cos angle - R sin angle
		auto const first{First > 0.0 ? First : Radius};
		auto const more{first - Radius};
		auto const ahead{a + b * std::cos(angle) - Radius * std::sin(angle)};
		auto const aside{b * std::sin(angle) - Radius * (1.0 - std::cos(angle))};
		auto const rest = [&](double const Turn) { return ahead * std::tan(Turn) + more * (1.0 - 1.0 / std::cos(Turn)) - aside; };
		auto const straight_of = [&](double const Turn) { return (ahead - more * std::sin(Turn)) / std::cos(Turn); };
		if (ahead <= 0.0 || aside <= 0.0 || rest(angle) < 0.0)
		{
			Error = format(STR_C("a and b too short for an arc, a straight and an arc at R %.0f m: the tangent of the arc alone is %.3f m"), Radius, tangent);
			return false;
		}
		auto low{0.0};
		auto high{angle};
		for (int i = 0; i < 60; ++i)
		{
			auto const middle{(low + high) * 0.5};
			(rest(middle) > 0.0 ? high : low) = middle;
		}
		auto const turn{(low + high) * 0.5};
		auto const straight{straight_of(turn)};
		if (straight < 0.0)
		{
			Error = format(STR_C("a and b too short for an arc, a straight and an arc at R %.0f m: the tangent of the arc alone is %.3f m"), Radius, tangent);
			return false;
		}
		parts = {{first * turn, first, first}, {straight, 0.0, 0.0}, {Radius * (angle - turn), Radius, Radius}};
	}
	if (Kind == tangent_fit::transition && longer > tangent + kNear)
	{
		// the end of the transition of the length and the arc from it up to the angle: the longer tangent told by the length alone
		auto const reached = [&](double const Length, double &Tangent, double &Other) {
			double x{0.0};
			double z{0.0};
			int constexpr kSteps{200};
			for (int i = 0; i < kSteps; ++i)
			{
				auto const s{(i + 0.5) * Length / kSteps};
				auto const turned{s * s / (2.0 * Radius * std::max(Length, 1e-9))};
				x += std::sin(turned) * Length / kSteps;
				z += std::cos(turned) * Length / kSteps;
			}
			auto const from{Length / (2.0 * Radius)};
			x += Radius * (std::cos(from) - std::cos(angle));
			z += Radius * (std::sin(angle) - std::sin(from));
			Tangent = z - x / std::tan(angle);
			Other = x / std::sin(angle);
		};
		auto low{0.0};
		auto high{2.0 * Radius * angle * (1.0 - 1e-6)};
		double tangent_of;
		double other;
		for (int i = 0; i < 60; ++i)
		{
			auto const middle{(low + high) * 0.5};
			reached(middle, tangent_of, other);
			(tangent_of > longer ? high : low) = middle;
		}
		auto const length{(low + high) * 0.5};
		reached(length, tangent_of, other);
		if (std::abs(tangent_of - longer) > kNear)
		{
			Error = format(STR_C("A transition curve at R %.0f doesn't reach the tangent of %.3f m"), Radius, longer);
			return false;
		}
		if (shorter < other - kNear)
		{
			Error = format(STR_C("The shorter tangent can't be less than %.3f m with this transition curve"), other);
			return false;
		}
		auto const straight{std::max(0.0, shorter - other)};
		part const arc{Radius * angle - length * 0.5, Radius, Radius};
		if (a >= b)
		{
			parts = {{length, 0.0, Radius}, arc};
			if (straight > 0.01)
				parts.push_back({straight, 0.0, 0.0});
		}
		else
		{
			parts = {arc, {length, Radius, 0.0}};
			lead = straight;
		}
	}
	if (parts.empty())
	{
		// one arc; a nearer than its tangent: the blade meets the stock rail at an angle, the arc turning the rest of the way
		auto const centre = [&](double const Entry) { return Radius * (std::sin(angle) - std::sin(Entry)) - Radius * (std::cos(Entry) - std::cos(angle)) / std::tan(angle); };
		if (a < tangent - kNear)
		{
			auto low{0.0};
			auto high{angle};
			for (int i = 0; i < 60; ++i)
			{
				auto const middle{(low + high) * 0.5};
				(centre(middle) > a ? low : high) = middle;
			}
			entry = (low + high) * 0.5;
		}
		else if (a > tangent + kNear)
			lead = a - tangent;
		auto const arcend{Radius * (std::cos(entry) - std::cos(angle)) / std::sin(angle)};
		if (b < arcend - kNear)
		{
			Error = format(STR_C("b can't be shorter than the end of the arc, %.3f m"), arcend);
			return false;
		}
		parts = {{Radius * (angle - entry), Radius, Radius}};
		if (b - arcend > 0.01)
			parts.push_back({b - arcend, 0.0, 0.0});
	}
	Design.parts = std::move(parts);
	Design.lead = lead;
	Design.entry = glm::degrees(entry);
	Design.a = a;
	Design.b = b;
	Design.main = a + b;
	return true;
}

bool fit_clicks(design &Design, std::vector<glm::dvec2> const &Clicks, tangent_fit const Kind, double const First, clicks_fit &Result, std::string &Error)
{
	measured found;
	if (false == measure_clicks(Clicks, found, Error))
		return false;
	auto const main{glm::normalize(Clicks[1] - Clicks[0])};
	auto const beside{Clicks[2] - Clicks[0]};
	auto const side{main.x * beside.y - main.y * beside.x < 0.0 ? -1.0 : 1.0};
	// the points in the space of the switch, x across and y along the main track
	std::vector<glm::dvec2> points;
	for (std::size_t i = 4; i < Clicks.size(); ++i)
	{
		auto const offset{Clicks[i] - Clicks[0]};
		points.push_back({(main.x * offset.y - main.y * offset.x) * side, glm::dot(main, offset)});
	}
	// how far the points are from the diverging track of the design: the worst and the mean square
	auto const off = [&](design const &Trial, double &Worst) {
		std::string problem;
		auto const shape{build(Trial, problem)};
		if (shape.paths.size() < 2)
			return std::numeric_limits<double>::max();
		std::vector<glm::dvec2> line;
		std::vector<segment_data> pieces{shape.paths[1].pieces};
		if (pieces.empty())
			pieces.push_back(shape.paths[1]);
		for (auto const &piece : pieces)
		{
			geometry::bezier const curve{piece};
			for (int j = 0; j <= 16; ++j)
			{
				auto const at{curve.point(j / 16.0)};
				line.push_back({at.x, at.z});
			}
		}
		auto sum{0.0};
		Worst = 0.0;
		for (auto const &point : points)
		{
			auto nearest{std::numeric_limits<double>::max()};
			for (std::size_t k = 0; k + 1 < line.size(); ++k)
			{
				auto const run{line[k + 1] - line[k]};
				auto const t{std::clamp(glm::dot(point - line[k], run) / std::max(1e-12, glm::dot(run, run)), 0.0, 1.0)};
				nearest = std::min(nearest, glm::length(point - (line[k] + run * t)));
			}
			sum += nearest * nearest;
			Worst = std::max(Worst, nearest);
		}
		return sum / static_cast<double>(std::max<std::size_t>(1, points.size()));
	};
	double constexpr kAboutAsClose{0.01}; // m of the mean, what clicking off a photo can't tell apart
	auto best{std::numeric_limits<double>::max()};
	design chosen;
	auto any{false};
	for (auto const kind : {Kind, tangent_fit::arc_straight_arc, tangent_fit::straights, tangent_fit::transition, tangent_fit::two_arcs})
	{
		// the best radius of the shape, from a coarse row of them closing in
		auto low{40.0};
		auto high{2000.0};
		auto best_radius{0.0};
		auto best_of_kind{std::numeric_limits<double>::max()};
		for (int pass = 0; pass < 3; ++pass)
		{
			int constexpr kRow{24};
			for (int i = 0; i <= kRow; ++i)
			{
				auto const radius{low * std::pow(high / low, static_cast<double>(i) / kRow)};
				design trial{Design};
				std::string problem;
				double worst;
				if (false == fit_tangents(trial, radius, found.ratio, found.a, found.b, kind, problem, First))
					continue;
				if (auto const value{off(trial, worst)}; value < best_of_kind)
				{
					best_of_kind = value;
					best_radius = radius;
				}
			}
			if (best_radius <= 0.0)
				break;
			auto const step{std::pow(high / low, 1.0 / kRow)};
			low = best_radius / step;
			high = best_radius * step;
		}
		if (best_radius <= 0.0)
			continue;
		// the shape asked for keeps its place unless another is clearly closer
		auto const mean{std::sqrt(best_of_kind)};
		if (false == any || mean < best - kAboutAsClose)
		{
			design trial{Design};
			std::string problem;
			fit_tangents(trial, best_radius, found.ratio, found.a, found.b, kind, problem, First);
			off(trial, Result.worst);
			Result.kind = kind;
			Result.radius = best_radius;
			trial.main = found.main;
			trial.b = found.main - found.a;
			chosen = trial;
			best = mean;
			any = true;
		}
	}
	if (false == any)
	{
		Error = "No shape fits these tangents";
		return false;
	}
	Design = chosen;
	return true;
}

namespace
{

// the catalogue as the build wrote it out of editor/data/turnouts.txt, the bytes of its text
constexpr unsigned char kCatalogueText[]{
#include "editor_turnouts_txt.inc"
};

// a switch of the catalogue as its line gives it: what the instruction tells of it, which its parts are made of
struct ordinary
{
	std::string name;
	std::string rail;
	double radius{0.0};
	double ratio{0.0}; // at the end of the diverging track; of the two tracks parting, for a symmetric switch
	double a{0.0}; // 0: from the geometry
	double b{0.0}; // 0: the diverging track curved up to its end
	double diverging{0.0}; // 0: the same as b
	bool movable{false};
	std::string type{"Rz"};
	bool secant{false}; // the blades cut into the stock rails at 25' (S42)
	double cut{0.0}; // the cut tip of the blade from the start of the curve, 0: none told
};

design made_of(ordinary const &Ordinary)
{
	design entry;
	entry.name = Ordinary.name;
	entry.rail = Ordinary.rail;
	entry.type = Ordinary.type;
	entry.movable = Ordinary.movable;
	auto const symmetric{symmetric_switch(entry.type)};
	// each track of a symmetric switch turns by a half of the angle between them
	auto const angle{std::atan(1.0 / Ordinary.ratio) * (symmetric ? 0.5 : 1.0)};
	auto const radius{Ordinary.radius};
	auto const tangent{radius * std::tan(angle * 0.5)};
	auto const a{Ordinary.a > 0.0 ? Ordinary.a : tangent};
	auto const across = [&](double const Entry) { return radius * (std::cos(Entry) - std::cos(angle)); };
	// the centre nearer than the tangent of the arc: the blade meets the stock rail at an angle and the arc turns the rest of the way;
	// further: the diverging track runs along the main one for a while
	// within a couple of centimetres of the tangent the catalogue only rounds it
	auto const centre = [&](double const Entry) { return radius * (std::sin(angle) - std::sin(Entry)) - across(Entry) / std::tan(angle); };
	auto const fit = [&](double const Centre) {
		auto low{0.0};
		auto high{angle};
		for (int i = 0; i < 60; ++i)
		{
			auto const middle{(low + high) * 0.5};
			(centre(middle) > Centre ? low : high) = middle;
		}
		return (low + high) * 0.5; };
	auto entryangle{0.0};
	auto lead{a > tangent + 0.02 ? a - tangent : 0.0};
	if (Ordinary.secant)
	{
		entryangle = glm::radians(kSecantAngle) * (symmetric ? 0.5 : 1.0);
		lead = a - centre(entryangle);
		if (lead <= 0.0)
		{
			lead = kSecantLead;
			entryangle = fit(a - lead);
		}
	}
	else if (a < tangent - 0.02)
		entryangle = fit(a);
	else if (Ordinary.cut > 0.0 && false == symmetric)
		entry.tip = lead + Ordinary.cut;
	entry.entry = glm::degrees(entryangle);
	entry.lead = lead;
	entry.a = a;
	auto const arcend{across(entryangle) / std::sin(angle)};
	entry.b = Ordinary.b > 0.0 ? Ordinary.b : arcend;
	entry.parts.push_back({radius * (angle - entryangle), radius, radius});
	auto const rest{(Ordinary.diverging > 0.0 ? Ordinary.diverging : entry.b) - arcend};
	if (rest > 0.05)
		entry.parts.push_back({rest, 0.0, 0.0});
	if (false == symmetric)
		entry.main = a + entry.b;
	if (Ordinary.a <= 0.0 || Ordinary.b <= 0.0)
		entry.note = "(from the geometry)";
	return entry;
}

// the words of a line of the file: blanks between them, a word in quotes keeping its blanks; what follows the word source is the rest
// of the line as it is
struct line_words
{
	std::vector<std::string> words;
	std::vector<bool> quoted;
	std::string source;
	bool sourced{false};
};

bool split_words(std::string_view const Line, line_words &Into, std::string &Error)
{
	std::size_t at{0};
	while (at < Line.size())
	{
		if (Line[at] == ' ' || Line[at] == '\t' || Line[at] == '\r')
		{
			++at;
			continue;
		}
		if (Line[at] == '"')
		{
			auto const close{Line.find('"', at + 1)};
			if (close == std::string_view::npos)
			{
				Error = "a quote not closed";
				return false;
			}
			Into.words.emplace_back(Line.substr(at + 1, close - at - 1));
			Into.quoted.push_back(true);
			at = close + 1;
			continue;
		}
		auto end{Line.find_first_of(" \t\r", at)};
		if (end == std::string_view::npos)
			end = Line.size();
		auto const word{Line.substr(at, end - at)};
		at = end;
		if (word == "source" && Into.words.size() >= 2)
		{
			auto rest{Line.substr(std::min(at + 1, Line.size()))};
			while (false == rest.empty() && (rest.back() == '\r' || rest.back() == ' '))
				rest.remove_suffix(1);
			Into.source = rest;
			Into.sourced = true;
			return true;
		}
		Into.words.emplace_back(word);
		Into.quoted.push_back(false);
	}
	return true;
}

bool number_of(std::string_view const Text, double &Value)
{
	auto const *first{Text.data()};
	auto const *last{Text.data() + Text.size()};
	auto const [end, error]{std::from_chars(first, last, Value)};
	return error == std::errc{} && end == last && std::isfinite(Value);
}

// the values of an entry by their keys, each key once
class entry_values
{
  public:
	std::string name;
	std::string source;
	std::vector<part> parts;
	std::vector<part> parts2;
	bool has_parts{false};
	bool has_parts2{false};
	double bending{0.0};
	double bending_exception{0.0};

	bool read(line_words const &Line, std::string &Error)
	{
		auto const &words{Line.words};
		name = Line.quoted[1] ? words[1] : unword(words[1]);
		source = Line.source;
		std::size_t at{2};
		while (at < words.size())
		{
			auto const &key{words[at++]};
			if (m_values.contains(key) || ((key == "parts" || key == "parts2") && (key == "parts" ? has_parts : has_parts2)))
			{
				Error = std::format("{} given twice", key);
				return false;
			}
			if (key == "parts" || key == "parts2")
			{
				if (false == read_parts(words, at, key == "parts" ? parts : parts2))
				{
					Error = std::format("{} not as many as said, or not numbers", key);
					return false;
				}
				(key == "parts" ? has_parts : has_parts2) = true;
				continue;
			}
			if (key == "bending")
			{
				if (false == read_bending(words, at))
				{
					Error = "bending is - or two radii";
					return false;
				}
				m_values[key] = "";
				continue;
			}
			if (false == known(key))
			{
				Error = std::format("'{}' isn't a key", key);
				return false;
			}
			if (at >= words.size())
			{
				Error = std::format("{} has no value", key);
				return false;
			}
			m_values[key] = words[at++];
		}
		return true;
	}
	bool has(std::string const &Key) const { return m_values.contains(Key); }
	std::string text(std::string const &Key, std::string const &Otherwise = {}) const
	{
		auto const found{m_values.find(Key)};
		return found != m_values.end() ? found->second : Otherwise;
	}
	// the number of the key, 0 where it isn't given or is -; false where it isn't a number
	bool number(std::string const &Key, double &Value) const
	{
		Value = 0.0;
		auto const found{m_values.find(Key)};
		if (found == m_values.end() || found->second == "-")
			return true;
		return number_of(found->second, Value);
	}

  private:
	std::map<std::string, std::string> m_values;

	static bool known(std::string_view const Key)
	{
		static constexpr std::array<std::string_view, 35> keys{"type", "rail", "radius", "ratio", "a", "b", "diverging", "blades", "cut", "fit", "stagger", "radius2",
		                                                       "ratio2", "a2", "b2", "diverging2", "blades2", "cut2", "middle", "note", "frog", "locks", "drive", "speed",
		                                                       "spacing", "main", "lead", "entry", "tip", "lead2", "entry2", "tip2", "heel", "throw", "movable"};
		return std::ranges::find(keys, Key) != keys.end();
	}
	static bool read_parts(std::vector<std::string> const &Words, std::size_t &At, std::vector<part> &Parts)
	{
		double count{0.0};
		if (At >= Words.size() || false == number_of(Words[At++], count) || count < 0.0 || count > 64.0 || count != std::floor(count))
			return false;
		Parts.resize(static_cast<std::size_t>(count));
		for (auto &each : Parts)
		{
			if (At + 3 > Words.size() || false == number_of(Words[At], each.length) || false == number_of(Words[At + 1], each.radius1) || false == number_of(Words[At + 2], each.radius2))
				return false;
			At += 3;
		}
		return true;
	}
	bool read_bending(std::vector<std::string> const &Words, std::size_t &At)
	{
		if (At < Words.size() && Words[At] == "-")
		{
			++At;
			return true;
		}
		if (At + 2 > Words.size() || false == number_of(Words[At], bending) || false == number_of(Words[At + 1], bending_exception))
			return false;
		At += 2;
		return true;
	}
};

// the switch of the keys with the suffix: the first one of a double switch with none, its second one with 2
bool ordinary_of(entry_values const &Values, std::string const &Suffix, ordinary &Result, std::string &Error)
{
	Result.rail = Values.text("rail", "-");
	Result.type = Values.text("type", "Rz");
	Result.movable = Values.text("frog") == "movable";
	auto const blades{Values.text("blades" + Suffix, "tangent")};
	if (blades != "tangent" && blades != "secant")
	{
		Error = "blades are tangent or secant";
		return false;
	}
	Result.secant = blades == "secant";
	if (false == Values.number("radius" + Suffix, Result.radius) || false == Values.number("ratio" + Suffix, Result.ratio) || false == Values.number("a" + Suffix, Result.a) ||
	    false == Values.number("b" + Suffix, Result.b) || false == Values.number("diverging" + Suffix, Result.diverging) || false == Values.number("cut" + Suffix, Result.cut))
	{
		Error = "a value of the geometry isn't a number";
		return false;
	}
	if (Result.radius < kMinRadius || Result.ratio <= 1.0)
	{
		Error = std::format("no radius{} of 10 m at least or no ratio{} above 1", Suffix, Suffix);
		return false;
	}
	return true;
}

// the switch given by its parts, as the template editor makes it
design explicit_of(entry_values const &Values, std::string &Error)
{
	design entry;
	entry.type = Values.text("type", "Rz");
	entry.rail = Values.text("rail", "-");
	entry.parts = Values.parts;
	entry.parts2 = Values.parts2;
	struct numbered
	{
		char const *key;
		double *value;
	};
	std::array const numbers{numbered{"main", &entry.main},   numbered{"stagger", &entry.stagger}, numbered{"a", &entry.a},          numbered{"b", &entry.b},
	                         numbered{"lead", &entry.lead},   numbered{"entry", &entry.entry},     numbered{"tip", &entry.tip},      numbered{"heel", &entry.heel},
	                         numbered{"throw", &entry.bladethrow}};
	for (auto const &[key, value] : numbers)
		if (false == Values.number(key, *value))
			Error = std::format("{} isn't a number", key);
	// of the second switch, negative: the same as the first
	for (auto const &[key, value] : {numbered{"lead2", &entry.lead2}, numbered{"entry2", &entry.entry2}, numbered{"tip2", &entry.tip2}})
		if (Values.has(key) && false == Values.number(key, *value))
			Error = std::format("{} isn't a number", key);
	return entry;
}

std::optional<design> design_of(entry_values const &Values, std::string &Error)
{
	auto const type{Values.text("type", "Rz")};
	if (type != "Rz" && false == symmetric_switch(type) && false == double_switch(type) && false == slip_switch(type))
	{
		Error = std::format("no type {}", type);
		return std::nullopt;
	}
	design made;
	if (Values.has_parts)
		made = explicit_of(Values, Error);
	else if (Values.has("fit"))
	{
		auto const kind{Values.text("fit")};
		std::array const kinds{std::pair{"straights", tangent_fit::straights}, std::pair{"transition", tangent_fit::transition}, std::pair{"arc_straight_arc", tangent_fit::arc_straight_arc},
		                       std::pair{"two_arcs", tangent_fit::two_arcs}};
		auto const found{std::ranges::find_if(kinds, [&](auto const &Each) { return kind == Each.first; })};
		ordinary given;
		if (found == kinds.end())
			Error = std::format("no fit {}", kind);
		else if (ordinary_of(Values, "", given, Error))
		{
			made.rail = given.rail;
			if (false == fit_tangents(made, given.radius, given.ratio, given.a, given.b, found->second, Error))
				return std::nullopt;
			made.radius = given.radius;
			made.ratio = given.ratio;
		}
	}
	else if (slip_switch(type))
	{
		made.type = type;
		made.rail = Values.text("rail", "-");
		if (false == Values.number("radius", made.radius) || false == Values.number("ratio", made.ratio) || false == Values.number("middle", made.middle) ||
		    false == Values.number("a", made.a))
			Error = "a value of the geometry isn't a number";
		else if (made.ratio <= 0.0 || made.a <= 0.0 || (type != "Skrz" && made.radius < kMinRadius))
			Error = "a slip has a radius, a ratio and a half length a, a crossing a ratio and a";
	}
	else if (double_switch(type))
	{
		ordinary first;
		ordinary second;
		double stagger{0.0};
		if (ordinary_of(Values, "", first, Error) && (type == "Rps" || ordinary_of(Values, "2", second, Error)) && Values.number("stagger", stagger))
		{
			// the both switches of a symmetric double switch have the shape of the first one
			if (type == "Rps")
				second = first;
			made = made_of(first);
			auto const other{made_of(second)};
			made.type = type;
			made.stagger = stagger;
			if (made.type != "Rps")
			{
				made.parts2 = other.parts;
				made.lead2 = other.lead;
				made.entry2 = other.entry;
				made.tip2 = other.tip;
			}
			made.main = 0.0;
			made.b = 0.0;
			made.note.clear();
			made.radius = first.radius;
			made.ratio = first.ratio;
		}
	}
	else
	{
		ordinary given;
		if (ordinary_of(Values, "", given, Error))
		{
			made = made_of(given);
			made.radius = given.radius;
			made.ratio = given.ratio;
		}
	}
	if (false == Error.empty())
		return std::nullopt;
	made.name = Values.name;
	if (Values.has("note"))
		made.note = Values.text("note");
	made.movable = Values.text("frog") == "movable" || Values.text("movable") == "1";
	made.blades = Values.text("blades");
	double locks{0.0};
	if (false == Values.number("cut", made.cut) || false == Values.number("speed", made.speed) || false == Values.number("spacing", made.spacing) || false == Values.number("locks", locks))
	{
		Error = "cut, speed, spacing or locks isn't a number";
		return std::nullopt;
	}
	made.locks = static_cast<int>(locks);
	made.drive = Values.text("drive") == "-" ? std::string{} : Values.text("drive");
	made.bending = Values.bending;
	made.bending_exception = Values.bending_exception;
	made.source = Values.source;
	return made;
}

} // namespace

bool slip_switch(std::string const &Type)
{
	return Type == "Rkp" || Type == "Rkpd" || Type == "Skrz";
}

glm::dvec2 slip_curve::point(double const Along) const
{
	auto const arc = [](glm::dvec2 const &From, double const Heading, double const Radius, double const Length) {
		if (Radius <= 0.0)
			return From + glm::dvec2{std::cos(Heading), std::sin(Heading)} * Length;
		auto const to{Heading + Length / Radius};
		return From + Radius * glm::dvec2{std::sin(to) - std::sin(Heading), std::cos(Heading) - std::cos(to)};
	};
	auto const along{Along - lead};
	auto result{arc({0.0, 0.0}, 0.0, 0.0, std::min(Along, lead))};
	if (along > 0.0)
		result = arc(result, entry, radius, std::min(along, ends));
	if (along > ends)
		result = arc(result, heading(lead + ends), middle, std::min(along - ends, between));
	if (along > ends + between)
		result = arc(result, heading(lead + ends + between), radius, std::min(along - ends - between, ends));
	if (along > 2.0 * ends + between)
		result = arc(result, angle, 0.0, along - 2.0 * ends - between);
	return result;
}

double slip_curve::heading(double const Along) const
{
	if (Along < lead)
		return 0.0;
	if (Along > length() - lead)
		return angle;
	auto const along{Along - lead};
	auto const inside{std::clamp(along - ends, 0.0, between)};
	return entry + (std::min(along, ends) + std::max(0.0, along - ends - between)) / radius + (middle > 0.0 ? inside / middle : 0.0);
}

double slip_curve::tangent() const
{
	auto const end{point(length())};
	return end.x - end.y / std::tan(angle);
}

double slip_curve::offset() const
{
	return glm::distance(point(length() * 0.5), glm::dvec2{tangent(), 0.0});
}

slip_curve slip_of(double const Radius, double const Middle, double const Angle, double const Pass, double const Lead, double const Entry)
{
	if (Entry > 0.0 && Angle * 0.5 <= Entry + 1e-6)
		return slip_of(Radius, Middle, Angle, Pass);
	auto const make = [&](double const Turn) {
		return slip_curve{Radius, Radius * Turn, Middle, Middle * (Angle - 2.0 * (Turn + Entry)), Angle, 0.0, Lead, Entry};
	};
	if (Middle < 0.0 && Angle >= 1e-6)
	{
		// a straight in the middle, as long as the arcs of Radius turning by a half of Angle pass the crossing as near
		auto low{0.0};
		auto high{4.0 * Pass / std::tan(Angle * 0.5)};
		for (int i = 0; i < 60; ++i)
		{
			auto const between{(low + high) * 0.5};
			(slip_curve{Radius, Radius * (Angle * 0.5 - Entry), 0.0, between, Angle, 0.0, Lead, Entry}.offset() > Pass ? high : low) = between;
		}
		return {Radius, Radius * (Angle * 0.5 - Entry), 0.0, (low + high) * 0.5, Angle, 0.0, Lead, Entry};
	}
	if (Middle <= 0.0 || Angle < 1e-6)
		return make(Angle * 0.5 - Entry);
	// the curve passes the crossing nearer as its end arcs turn further, the middle one being flatter
	auto low{0.0};
	auto high{Angle * 0.5 - Entry};
	for (int i = 0; i < 60; ++i)
	{
		auto const turn{(low + high) * 0.5};
		((make(turn).offset() > Pass) == (Middle > Radius) ? low : high) = turn;
	}
	return make((low + high) * 0.5);
}

std::vector<design> read_designs(std::string_view const Text, std::vector<std::string> *Errors)
{
	std::vector<design> result;
	std::size_t start{0};
	int number{0};
	while (start < Text.size())
	{
		auto end{Text.find('\n', start)};
		if (end == std::string_view::npos)
			end = Text.size();
		auto const line{Text.substr(start, end - start)};
		start = end + 1;
		++number;
		line_words words;
		std::string error;
		if (false == split_words(line, words, error))
		{
			if (Errors != nullptr)
				Errors->push_back(std::format("line {}: {}", number, error));
			continue;
		}
		if (words.words.empty() || words.words.front().starts_with('#') || words.words.front() != "switch")
			continue;
		entry_values values;
		if (words.words.size() < 2 || false == values.read(words, error))
		{
			if (Errors != nullptr)
				Errors->push_back(std::format("line {}: {}", number, error.empty() ? "no name" : error));
			continue;
		}
		auto made{design_of(values, error)};
		if (false == made.has_value() || made->name.empty())
		{
			if (Errors != nullptr)
				Errors->push_back(std::format("line {} ({}): {}", number, values.name, error.empty() ? "no name" : error));
			continue;
		}
		result.push_back(std::move(*made));
	}
	return result;
}

std::string_view catalogue_text()
{
	static std::string const text{std::begin(kCatalogueText), std::end(kCatalogueText)};
	return text;
}

std::vector<design> const &catalogue_entries()
{
	static std::vector<design> const entries{read_designs(catalogue_text())};
	return entries;
}

std::vector<design> catalogue()
{
	std::vector<design> result;
	for (auto const &entry : catalogue_entries())
		if (false == slip_switch(entry.type))
			result.push_back(entry);
	return result;
}

std::vector<design> slips()
{
	std::vector<design> result;
	for (auto const &entry : catalogue_entries())
		if (slip_switch(entry.type))
			result.push_back(entry);
	return result;
}

std::vector<design> own()
{
	std::ifstream file{own_path(), std::ios::binary};
	if (false == file.is_open())
		return {};
	std::string const text{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
	auto result{read_designs(text)};
	// a template of the user is given by its parts; a line giving a switch the way the catalogue does is taken too
	for (auto &entry : result)
		entry.own = true;
	return result;
}

std::string own_line(design const &Design)
{
	std::string line{"switch " + word(Design.name) + " type " + Design.type + " rail " + word(Design.rail) + ' ' +
	                 format("main %.4f stagger %.4f a %.4f b %.4f lead %.4f entry %.4f tip %.4f", Design.main, Design.stagger, Design.a, Design.b, Design.lead, Design.entry, Design.tip)};
	if (Design.lead2 >= 0.0 || Design.entry2 >= 0.0 || Design.tip2 >= 0.0)
		line += format(" lead2 %.4f entry2 %.4f tip2 %.4f", Design.lead2, Design.entry2, Design.tip2);
	if (Design.heel > 0.0 || Design.bladethrow > 0.0)
		line += format(" heel %.4f throw %.4f", Design.heel, Design.bladethrow);
	// only where it's so: the builds before this one leave out a template with a key they don't know
	if (Design.movable)
		line += " frog movable";
	std::ostringstream parts;
	parts << " parts";
	write_parts(parts, Design.parts);
	if (false == Design.parts2.empty())
	{
		parts << " parts2";
		write_parts(parts, Design.parts2);
	}
	return line + parts.str();
}

bool save_own(std::vector<design> const &Designs, std::string &Error)
{
	auto const path{own_path()};
	std::error_code error;
	if (path.has_parent_path())
		std::filesystem::create_directories(path.parent_path(), error);
	std::ofstream file{path, std::ios::trunc};
	if (false == file.is_open())
	{
		Error = format(STR_C("Can't write %s"), path.string().c_str());
		return false;
	}
	file << "# switch templates made in the track editor of MaSzyna, in the format of the catalogue of the editor (editor/data/turnouts.txt)\n";
	file << "# switch <name> type <Rz|Rpd|Rpj> rail <rail> main <m> stagger <m> a <m> b <m> lead <m> entry <deg> tip <m> [lead2 <m> entry2 <deg> tip2 <m>] [heel <m> throw <m>] [frog movable] parts <count> (<length> <R at the start> <R at the end>)... parts2 <count> ...\n";
	file << "# R 0 is a straight, the same R at both ends an arc, different ones a transition curve; 0 for main, a, b: from the geometry\n";
	for (auto const &entry : Designs)
		file << own_line(entry) << '\n';
	if (file.fail())
	{
		Error = format(STR_C("Can't write %s"), path.string().c_str());
		return false;
	}
	return true;
}

} // namespace switch_design

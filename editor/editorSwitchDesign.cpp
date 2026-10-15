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

#include "utilities/utilities.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace switch_design
{

namespace
{

double constexpr kPi{3.14159265358979323846};
double constexpr kMinRadius{10.0};
double constexpr kTransitionStep{5.0};
double constexpr kLink{1.0};

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
		if (part.length < 0.05)
			return "A part of the diverging track is shorter than 5 cm";
		for (auto const radius : {part.radius1, part.radius2})
			if (radius != 0.0 && std::abs(radius) < kMinRadius)
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

bool read_parts(std::istream &Input, std::vector<part> &Parts)
{
	std::size_t count{0};
	Input >> count;
	if (Input.fail() || count > 64)
		return false;
	Parts.resize(count);
	for (auto &part : Parts)
		Input >> part.length >> part.radius1 >> part.radius2;
	return false == Input.fail();
}

} // namespace

bool double_switch(std::string const &Type)
{
	return Type == "Rpd" || Type == "Rpj";
}

path diverging(std::vector<part> const &Parts, double const Lead, double const Entry)
{
	walker walk;
	if (Lead > 0.0)
		walk.straight(Lead);
	auto const start{Lead > 0.0 ? 0.0 : glm::radians(Entry)};
	walk.heading += glm::radians(Entry);
	for (auto const &part : Parts)
	{
		if (part.radius1 == 0.0 && part.radius2 == 0.0)
			walk.straight(part.length);
		else if (part.radius1 == part.radius2)
			walk.arc(part.length, part.radius1);
		else
			walk.transition(part.length, part.radius1, part.radius2);
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

shape build(design const &Design, std::string &Error)
{
	shape result;
	Error.clear();
	bool const twofold{double_switch(Design.type)};
	if (Design.type != "Rz" && false == twofold)
	{
		Error = "Unknown type of switch";
		return result;
	}
	Error = checked(Design.parts);
	if (Error.empty() && twofold && false == Design.parts2.empty())
		Error = checked(Design.parts2);
	if (Error.empty() && (Design.lead < 0.0 || Design.lead > 30.0))
		Error = "The pre-blade stretch has to be between 0 and 30 m";
	if (Error.empty() && (Design.entry < 0.0 || Design.entry > 5.0))
		Error = "The angle of the blade has to be between 0 and 5 degrees";
	if (false == Error.empty())
		return result;
	auto const first{diverging(Design.parts, Design.lead, Design.entry)};
	if (first.angle <= 1e-4 || first.angle >= kPi / 4.0)
	{
		Error = "The diverging track has to leave to the side at an angle between 0 and 45 degrees";
		return result;
	}
	auto const end{glm::dvec2{first.whole.points[segment_data::point::end].x, first.whole.points[segment_data::point::end].z}};
	result.ratio = 1.0 / std::tan(first.angle);
	result.radius = first.radius;
	result.offset = end.x;
	result.tip = Design.lead;
	result.a = Design.a > 0.0 ? Design.a : tangent_point(end, first.angle);
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
	auto const lead2{Design.lead2 >= 0.0 ? Design.lead2 : Design.lead};
	auto const entry2{Design.entry2 >= 0.0 ? Design.entry2 : Design.entry};
	if (lead2 > 30.0 || entry2 > 5.0)
	{
		Error = "The pre-blade stretch of the second switch has to be at most 30 m and the angle of its blade at most 5 degrees";
		return result;
	}
	auto second{diverging(Design.parts2.empty() ? Design.parts : Design.parts2, lead2, entry2)};
	if (second.angle <= 1e-4 || second.angle >= kPi / 4.0)
	{
		Error = "The diverging track of the second switch has to leave to the side at an angle between 0 and 45 degrees";
		return result;
	}
	if (Design.type == "Rpd")
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
	result.tip2 = Design.stagger + lead2;
	result.radius = std::min(first.radius, second.radius > 0.0 ? second.radius : first.radius);
	// the simulation joins switches only to plain tracks, so a short one lies between the two
	auto const link{std::min(kLink, Design.stagger * 0.5)};
	result.paths = {straight_path(0.0, Design.stagger - link), first.whole, straight_path(Design.stagger, result.length), second.whole, straight_path(Design.stagger - link, Design.stagger)};
	return result;
}

std::vector<design> catalogue()
{
	struct ordinary
	{
		char const *name;
		double radius;
		double ratio;
		double a;
		double b;
	};
	ordinary const ordinaries[] = {
		{"Rz 1:9 R190", 190.0, 9.0, 10.523, 16.615},
		{"Rz 1:9 R300", 300.0, 9.0, 16.615, 16.615},
		{"Rz 1:12 R500", 500.0, 12.0, 20.797, 0.0},
		{"Rz 1:14 R760", 760.0, 14.0, 0.0, 0.0},
		{"Rz 1:18,5 R1200", 1200.0, 18.5, 32.409, 34.943},
		{"Rz 1:26,5 R2500", 2500.0, 26.5, 0.0, 0.0},
	};
	auto const make = [](ordinary const &Ordinary) {
		design entry;
		entry.name = Ordinary.name;
		auto const angle{std::atan(1.0 / Ordinary.ratio)};
		entry.a = Ordinary.a > 0.0 ? Ordinary.a : Ordinary.radius * std::tan(angle * 0.5);
		auto const estimated{Ordinary.b <= 0.0};
		entry.b = estimated ? 1.87 * Ordinary.ratio : Ordinary.b;
		entry.parts.push_back({Ordinary.radius * angle, Ordinary.radius, Ordinary.radius});
		auto const rest{(entry.a + entry.b - Ordinary.radius * std::sin(angle)) / std::cos(angle)};
		if (rest > 0.01)
		{
			entry.parts.push_back({rest, 0.0, 0.0});
			entry.main = entry.a + entry.b;
		}
		else
			entry.b = 0.0;
		if (estimated)
			entry.note = "(b est.)";
		return entry;
	};
	std::vector<design> result;
	for (auto const &ordinary : ordinaries)
		result.push_back(make(ordinary));
	{
		design curved;
		curved.name = "Rz 1:7,5 R190";
		auto const angle{std::atan(1.0 / 7.5)};
		curved.parts.push_back({190.0 * angle, 190.0, 190.0});
		curved.note = "(curved to the end)";
		result.push_back(curved);
	}
	struct twofold
	{
		char const *name;
		char const *type;
		std::size_t ordinary;
		double stagger;
	};
	twofold const twofolds[] = {
		{"Rpd 1:9 R190", "Rpd", 0, 5.0},
		{"Rpj 1:9 R190", "Rpj", 0, 5.0},
		{"Rpd 1:9 R300", "Rpd", 1, 6.0},
		{"Rpj 1:9 R300", "Rpj", 1, 6.0},
	};
	for (auto const &entry : twofolds)
	{
		auto made{make(ordinaries[entry.ordinary])};
		made.name = entry.name;
		made.type = entry.type;
		made.stagger = entry.stagger;
		made.main = 0.0;
		made.b = 0.0;
		made.note = format("(second switch %.1f m on, assumed)", entry.stagger);
		result.push_back(made);
	}
	return result;
}

std::vector<design> own()
{
	std::vector<design> result;
	std::ifstream file{own_path()};
	if (false == file.is_open())
		return result;
	std::string text;
	while (std::getline(file, text))
	{
		if (text.empty() || text.front() == '#')
			continue;
		std::istringstream line{text};
		design entry;
		entry.own = true;
		std::string keyword;
		std::string name;
		line >> keyword >> name;
		if (keyword != "switch")
			continue;
		entry.name = unword(name);
		bool good{true};
		while (good && line >> keyword)
		{
			if (keyword == "type")
				line >> entry.type;
			else if (keyword == "rail")
			{
				line >> entry.rail;
				if (entry.rail.empty())
					entry.rail = "-";
			}
			else if (keyword == "main")
				line >> entry.main;
			else if (keyword == "stagger")
				line >> entry.stagger;
			else if (keyword == "lead")
				line >> entry.lead;
			else if (keyword == "entry")
				line >> entry.entry;
			else if (keyword == "lead2")
				line >> entry.lead2;
			else if (keyword == "entry2")
				line >> entry.entry2;
			else if (keyword == "a")
				line >> entry.a;
			else if (keyword == "b")
				line >> entry.b;
			else if (keyword == "parts")
				good = read_parts(line, entry.parts);
			else if (keyword == "parts2")
				good = read_parts(line, entry.parts2);
			else
				good = false;
			good = good && false == line.fail();
		}
		if (good && false == entry.name.empty())
			result.push_back(std::move(entry));
	}
	return result;
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
		Error = "Can't write " + path.string();
		return false;
	}
	file << "# switch templates made in the track editor of MaSzyna\n";
	file << "# switch <name> type <Rz|Rpd|Rpj> rail <rail> main <m> stagger <m> a <m> b <m> lead <m> entry <deg> [lead2 <m> entry2 <deg>] parts <count> (<length> <R at the start> <R at the end>)... parts2 <count> ...\n";
	file << "# R 0 is a straight, the same R at both ends an arc, different ones a transition curve; 0 for main, a, b: from the geometry\n";
	for (auto const &entry : Designs)
	{
		file << "switch " << word(entry.name) << " type " << entry.type << " rail " << word(entry.rail) << ' ' << format("main %.4f stagger %.4f a %.4f b %.4f lead %.4f entry %.4f", entry.main, entry.stagger, entry.a, entry.b, entry.lead, entry.entry);
		if (entry.lead2 >= 0.0 || entry.entry2 >= 0.0)
			file << format(" lead2 %.4f entry2 %.4f", entry.lead2, entry.entry2);
		file << " parts";
		write_parts(file, entry.parts);
		if (false == entry.parts2.empty())
		{
			file << " parts2";
			write_parts(file, entry.parts2);
		}
		file << '\n';
	}
	if (file.fail())
	{
		Error = "Can't write " + path.string();
		return false;
	}
	return true;
}

} // namespace switch_design

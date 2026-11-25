// the catalogue of the turnouts as a data file (editor/data/turnouts.txt, editor-tracks-domain.md §7): the switches read from it the same
// to the bit as the catalogue the editor had in its code, kept here as it was; the slips and the crossings as the switch tool had them;
// each type against what the instruction and the standard tell of it (a, b, R, the angle, the pre-blade stretch, the tip and the angle of
// the blade, the speed, the bending and the spacing of tab. 11.9 and 11.2); the rules for a blade tip where no data is, against the types
// which have it; the templates of the user in the same format keeping a movable nose

#include "stdafx.h"
#include "doctest.h"
#include "editor/editorSwitchCurve.hpp"
#include "editor/editorSwitchDesign.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace {

// the catalogue as editorSwitchDesign.cpp had it in its code before it became a data file
namespace frozen {

using namespace switch_design;
double constexpr kSecantAngle{25.0 / 60.0};
double constexpr kSecantLead{0.6};

// clang-format off
std::vector<design> old_catalogue()
{
	// the switches of the instruction Id-4 of PKP PLK, tables 8-1 (60E1), 8-2 (49E1) and 8-3 (S42): a from the start of the switch
	// to its centre, b on from there along the main track, and along the diverging track where the scheme gives it another length
	struct ordinary
	{
		char const *name;
		char const *rail;
		double radius;
		double ratio; // at the end of the diverging track; of the two tracks parting, for a symmetric switch
		double a; // 0: from the geometry
		double b; // 0: the diverging track curved up to its end
		double diverging{0.0}; // 0: the same as b
		bool movable{false};
		char const *type{"Rz"};
		bool tangent{false}; // an S42 switch with blades lying tangent to the stock rails, its a too long for blades cutting into them at 25'
	};
	ordinary const ordinaries[] = {
		{"Rz 60E1-190-1:9", "60E1", 190.0, 9.0, 10.523, 16.615},
		{"Rz 60E1-300-1:9", "60E1", 300.0, 9.0, 16.615, 16.615},
		{"Rz 60E1-500-1:12", "60E1", 500.0, 12.0, 20.797, 20.797},
		{"Rz 60E1-500-1:12 movable nose", "60E1", 500.0, 12.0, 20.797, 20.797, 0.0, true},
		{"Rz 60E1-760-1:14", "60E1", 760.0, 14.0, 27.108, 27.108},
		{"Rz 60E1-1200-1:18,5", "60E1", 1200.0, 18.5, 32.409, 32.409},
		{"Rz 60E1-1200-1:18,5 movable nose", "60E1", 1200.0, 18.5, 32.409, 32.409, 0.0, true},
		{"Rz 60E1-2500-1:26,5 movable nose", "60E1", 2500.0, 26.5, 47.153, 47.153, 0.0, true},
		{"Rz 49E1-190-1:7,5", "49E1", 190.0, 7.5, 12.611, 12.611},
		{"Rz 49E1-190-1:7,5 long", "49E1", 190.0, 7.5, 12.611, 17.428},
		{"Rz 49E1-190-1:9", "49E1", 190.0, 9.0, 10.523, 16.615},
		{"Rz 49E1-190-1:9 gauge 1524", "49E1", 190.0, 9.0, 10.523, 17.417},
		{"Rz 49E1-190-1:7,5/1:6,6", "49E1", 190.0, 6.6, 14.312, 10.910, 14.312},
		{"Rz 49E1-300-1:9", "49E1", 300.0, 9.0, 16.615, 16.615},
		{"Rz 49E1-300-1:9,403", "49E1", 300.0, 9.403, 15.907, 17.343, 17.322},
		{"Rz 49E1-500-1:12", "49E1", 500.0, 12.0, 20.797, 20.797},
		{"Rz 49E1-500-1:14", "49E1", 500.0, 14.0, 17.834, 24.537},
		{"Rz 49E1-1200-1:18,5", "49E1", 1200.0, 18.5, 32.409, 32.409},
		// table 3-2, without a scheme
		{"Rls 49E1-215-1:4,8", "49E1", 215.0, 4.8, 0.0, 0.0, 0.0, false, "Rls"},
		{"Rz S42-190-1:6,6", "S42", 190.0, 6.6, 13.374, 12.560, 14.172},
		{"Rz S42-190-1:7", "S42", 190.0, 7.0, 12.573, 13.361},
		{"Rz S42-190-1:7,5", "S42", 190.0, 7.5, 11.691, 14.243, 14.235},
		{"Rz S42-205-1:9 I", "S42", 205.0, 9.0, 10.517, 16.500},
		{"Rz S42-205-1:9 II", "S42", 205.0, 9.0, 10.517, 17.591},
		{"Rz S42-230-1:9,51 I", "S42", 230.0, 9.51, 11.046, 16.008, 16.308},
		{"Rz S42-230-1:9,51 II", "S42", 230.0, 9.51, 11.046, 18.342, 18.642},
		{"Rz S42-230-1:9,51 III", "S42", 230.0, 9.51, 11.046, 16.008, 18.642},
		{"Rz S42-265-1:10 I", "S42", 265.0, 10.0, 11.9692, 18.1508},
		{"Rz S42-265-1:10 II", "S42", 265.0, 10.0, 11.9692, 19.1918},
		{"Rz S42-500-1:9", "S42", 500.0, 9.0, 25.055, 13.743, 27.513},
		{"Rz S42-500-1:12", "S42", 500.0, 12.0, 18.21526, 20.58234, 20.58202},
		{"Rz S42-500-1:14", "S42", 500.0, 14.0, 15.288, 26.323},
		{"Rz S42-70-1:5 industrial", "S42", 70.0, 5.0, 7.112, 8.708},
		{"Rz S42-140-1:7 industrial", "S42", 140.0, 7.0, 7.670, 12.932},
		{"Rls S42-140-1:5 industrial", "S42", 140.0, 5.0, 7.131, 8.715, 0.0, false, "Rls"},
		{"Rls S42-200-1:7 industrial", "S42", 200.0, 7.0, 6.493, 12.148, 0.0, false, "Rls"},
		{"Rz S42-205-1:6,6 industrial", "S42", 205.0, 6.6, 14.588, 13.602, 15.352},
		{"Rz S42-205-1:7 industrial", "S42", 205.0, 7.0, 13.710, 14.480},
		{"Rz S42-205-1:7,5 industrial", "S42", 205.0, 7.5, 12.754, 15.436, 15.427},
		{"Rz S42-300-1:9 industrial", "S42", 300.0, 9.0, 16.615, 16.615, 0.0, false, "Rz", true},
		{"Rz S42-205-1:9 gauge 1524", "S42", 205.0, 9.0, 10.513, 18.394},
	};
	auto const make = [](ordinary const &Ordinary) {
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
		if (entry.rail == "S42" && false == Ordinary.tangent)
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
		else if (auto const cut{cut_tip(entry.rail, radius)}; cut > 0.0 && false == symmetric)
			entry.tip = lead + cut;
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
	};
	std::vector<design> result;
	for (auto const &ordinary : ordinaries)
		result.push_back(make(ordinary));
	// not of Id-4: the tangents as measured, too long for one arc alone
	for (auto const &[name, kind] : {std::pair{"Rz S42-140-1:6,6 16/12", tangent_fit::straights}, std::pair{"Rz S42-140-1:6,6 16/12 KP", tangent_fit::transition}, std::pair{"Rz S42-140-1:6,6 16/12 \xc5\x82uk-prosta-\xc5\x82uk", tangent_fit::arc_straight_arc}})
	{
		design fitted;
		fitted.name = name;
		fitted.rail = "S42";
		std::string problem;
		if (fit_tangents(fitted, 140.0, 6.6, 16.0, 12.0, kind, problem))
			result.push_back(fitted);
	}
	// double switches: the second one starts on the main track of the first, as far on as the schemes give it, a one sided double
	// switch of Id-4 has its second switch on the diverging track of the first, here on the main one at the same place
	ordinary const r190_75{"", "", 190.0, 7.5, 12.611, 12.611};
	ordinary const r190_75long{"", "", 190.0, 7.5, 12.611, 12.611, 16.615};
	ordinary const r190_9{"", "", 190.0, 9.0, 10.523, 16.615};
	ordinary const r190_9tangent{"", "", 190.0, 9.0, 10.523, 16.615, 0.0, false, "Rz", true};
	ordinary const r190_66{"", "", 190.0, 6.6, 14.312, 10.910, 14.312};
	ordinary const r300_9{"", "", 300.0, 9.0, 16.615, 16.615};
	ordinary const r500_12{"", "", 500.0, 12.0, 20.797, 20.797};
	struct twofold
	{
		char const *name;
		char const *type;
		char const *rail;
		ordinary first;
		ordinary second;
		double stagger;
		char const *note;
	};
	twofold const twofolds[] = {
		{"Rpd 49E1-190-1:7,5/1:7,5", "Rpd", "49E1", r190_75long, r190_75, 10.523, ""},
		{"Rpd 49E1-190-1:9/1:7,5", "Rpd", "49E1", r190_9, r190_75, 10.523, ""},
		{"Rpd 49E1-190-1:9/1:9", "Rpd", "49E1", r190_9, r190_9, 10.523, ""},
		{"Rpd 49E1-190-1:7,5/1:7,5/1:6,6", "Rpd", "49E1", r190_75, r190_66, 10.523, ""},
		{"Rpd 49E1-190-1:7,5/1:6,6/1:7,5", "Rpd", "49E1", r190_66, r190_75, 10.523, ""},
		{"Rpd 49E1-190-1:7,5/1:6,6/1:7,5/1:6,6", "Rpd", "49E1", r190_66, r190_66, 10.523, ""},
		{"Rpd 49E1-190-1:9/1:7,5/1:6,6", "Rpd", "49E1", r190_9, r190_66, 10.523, ""},
		{"Rpj 49E1-190-1:9", "Rpj", "49E1", r190_9, r190_9, 5.258, "(approx.)"},
		{"Rpd S42-190-1:9", "Rpd", "S42", r190_9tangent, r190_9tangent, 10.523, ""},
		{"Rpd S42-190-1:9 II", "Rpd", "S42", {"", "", 190.0, 9.0, 9.626, 15.500}, {"", "", 190.0, 9.0, 9.626, 17.591}, 9.5, ""},
		{"Rpj S42-190/180-1:10", "Rpj", "S42", {"", "", 190.0, 10.0, 0.0, 19.192, 0.0, false, "Rz", true}, {"", "", 180.0, 10.0, 0.0, 19.192, 0.0, false, "Rz", true}, 5.504, "(approx.)"},
		{"Rpj S42-140-1:9 industrial", "Rpj", "S42", {"", "", 140.0, 9.0, 9.274, 17.590, 0.0, false, "Rz", true}, {"", "", 140.0, 9.0, 9.274, 17.590, 0.0, false, "Rz", true}, 9.263, "(approx.)"},
		{"Rpd S42-140-1:7 industrial", "Rpd", "S42", {"", "", 140.0, 7.0, 7.670, 12.932}, {"", "", 140.0, 7.0, 7.670, 12.932}, 6.9, ""},
		// symmetric double switches, not in Id-4 but for its scheme: the same switch to both sides, the second one starting as close behind
		// the first one as their blades let them, past the heel of the blades of the first one and the short track between the two
		{"Rps 60E1-300-1:9", "Rps", "60E1", r300_9, r300_9, 14.2, "(designed)"},
		{"Rps 60E1-500-1:12", "Rps", "60E1", r500_12, r500_12, 18.0, "(designed)"},
	};
	for (auto const &entry : twofolds)
	{
		auto first{entry.first};
		auto other{entry.second};
		first.rail = other.rail = entry.rail;
		auto made{make(first)};
		auto const second{make(other)};
		made.name = entry.name;
		made.rail = entry.rail;
		made.type = entry.type;
		made.stagger = entry.stagger;
		if (made.type != "Rps")
		{
			made.parts2 = second.parts;
			made.lead2 = second.lead;
			made.entry2 = second.entry;
			made.tip2 = second.tip;
		}
		made.main = 0.0;
		made.b = 0.0;
		made.note = entry.note;
		result.push_back(made);
	}
	return result;
}
// clang-format on

// the slips and the crossings of the switch tool (editorTrack.cpp) before they became lines of the data file
struct slipping {
	char const *type;
	char const *name;
	double radius;
	double middle;
	double half;
};
std::vector<slipping> const slips{
    {"Rkp", "60E1-190-1:9", 190.0, 0.0, 16.615},
    {"Rkp", "49E1-190-1:9", 190.0, 0.0, 16.615},
    {"Rkp", "49E1-190-1:6,6", 190.0, -1.0, 17.396},
    {"Rkp", "49E1-300-1:9", 300.0, -1.0, 22.639},
    {"Rkp", "49E1-500-1:9", 500.0, -1.0, 27.693},
    {"Rkp", "S42-230-1:9,51", 230.0, 0.0, 18.642},
    {"Rkp", "S42-265-1:10", 265.0, 0.0, 19.192},
    {"Rkpd", "60E1-190-1:9", 190.0, 0.0, 16.615},
    {"Rkpd", "49E1-190-1:9", 190.0, 0.0, 16.615},
    {"Rkpd", "49E1-190-1:6,6", 190.0, -1.0, 17.396},
    {"Rkpd", "49E1-300-1:9", 300.0, -1.0, 22.639},
    {"Rkpd", "49E1-500-1:9", 500.0, -1.0, 27.693},
    {"Rkpd", "49E1-190-1:9 gauge 1524", 190.0, 0.0, 17.420},
    {"Rkpd", "S42-205-1:9", 205.0, 0.0, 17.591},
    {"Rkpd", "S42-205-1:9 II", 205.0, 0.0, 18.394},
    {"Rkpd", "S42-230-1:9,51", 230.0, 0.0, 18.642},
    {"Rkpd", "S42-230-1:9,51 Ia", 230.0, 0.0, 18.342},
    {"Rkpd", "S42-265-1:10", 265.0, 0.0, 19.192},
};
struct crossing {
	char const *name;
	double half;
};
std::vector<crossing> const crossings{
    {"60E1-1:9", 16.615},     {"60E1-1:4,444", 10.907},  {"49E1-1:2,9008", 6.904},          {"49E1-1:3,224", 7.920}, {"49E1-1:3,683", 9.881},
    {"49E1-1:4,44", 10.907},  {"49E1-1:6,6", 17.396},    {"49E1-1:7,5", 18.512},            {"49E1-1:9", 16.615},    {"49E1-1:9 long", 17.016},
    {"49E1-1:18,5", 32.409},  {"S42-1:2", 5.743},        {"S42-1:2,9008", 8.250},           {"S42-1:3,429", 9.681},  {"S42-1:3,43", 8.421},
    {"S42-1:4,44", 11.788},   {"S42-1:4,444 gauge 1524", 12.189}, {"S42-1:7", 12.932},      {"S42-1:9", 17.591},     {"S42-1:10", 19.192},
};
double ratio_of(std::string const &Name) {
	auto const at{Name.find("1:")};
	auto number{Name.substr(at + 2)};
	std::replace(number.begin(), number.end(), ',', '.');
	return std::stod(number);
}

} // namespace frozen

void check_same_parts(std::vector<switch_design::part> const &Old, std::vector<switch_design::part> const &New) {
	REQUIRE(New.size() == Old.size());
	for (std::size_t i = 0; i < Old.size(); ++i) {
		CAPTURE(i);
		CHECK(New[i].length == Old[i].length);
		CHECK(New[i].radius1 == Old[i].radius1);
		CHECK(New[i].radius2 == Old[i].radius2);
	}
}

// the values the editor had: the same doubles, not within a tolerance
void check_same(switch_design::design const &Old, switch_design::design const &New) {
	CHECK(New.name == Old.name);
	CHECK(New.type == Old.type);
	CHECK(New.rail == Old.rail);
	CHECK(New.main == Old.main);
	check_same_parts(Old.parts, New.parts);
	CHECK(New.lead == Old.lead);
	CHECK(New.entry == Old.entry);
	CHECK(New.tip == Old.tip);
	CHECK(New.heel == Old.heel);
	CHECK(New.bladethrow == Old.bladethrow);
	CHECK(New.stagger == Old.stagger);
	check_same_parts(Old.parts2, New.parts2);
	CHECK(New.lead2 == Old.lead2);
	CHECK(New.entry2 == Old.entry2);
	CHECK(New.tip2 == Old.tip2);
	CHECK(New.a == Old.a);
	CHECK(New.b == Old.b);
	CHECK(New.note == Old.note);
	CHECK(New.own == Old.own);
	CHECK(New.movable == Old.movable);
}

double constexpr kSecant{25.0 / 60.0}; // degrees, the angle of the secant blades of S42

// ST-T1-A6 tab. 11.2: the least spacing of the tracks of a single crossover by the basic switch, of 60E1 and 49E1
double table_11_2_spacing(double const Radius, double const Ratio) {
	struct row {
		double radius;
		double ratio;
		double spacing;
	};
	static constexpr std::array rows{row{190.0, 9.0, 3.70}, row{300.0, 9.403, 4.00}, row{300.0, 9.0, 4.35}, row{500.0, 12.0, 4.00}, row{760.0, 14.0, 4.75}, row{1200.0, 18.5, 4.35}, row{2500.0, 26.5, 4.80}};
	for (auto const &each : rows) {
		if (std::abs(each.radius - Radius) < 0.5 && std::abs(each.ratio - Ratio) < 1e-6) {
			return each.spacing;
		}
	}
	return 0.0;
}

} // namespace

TEST_CASE("turnout catalogue: the switches of the data file are the ones the editor had in its code, to the bit") {
	auto const old{frozen::old_catalogue()};
	auto const now{switch_design::catalogue()};
	REQUIRE(now.size() == old.size());
	for (std::size_t i = 0; i < old.size(); ++i) {
		CAPTURE(old[i].name);
		check_same(old[i], now[i]);
	}
}

TEST_CASE("turnout catalogue: the slips and the crossings of the data file are the ones of the switch tool") {
	auto const now{switch_design::slips()};
	REQUIRE(now.size() == frozen::slips.size() + frozen::crossings.size());
	std::size_t at{0};
	for (auto const &old : frozen::slips) {
		auto const &entry{now[at++]};
		auto const name{std::string{old.name}};
		CAPTURE(name);
		CHECK(entry.type == old.type);
		CHECK(entry.name == std::string{old.type} + ' ' + name);
		CHECK(entry.rail == name.substr(0, name.find('-')));
		CHECK(entry.radius == old.radius);
		CHECK(entry.middle == old.middle);
		CHECK(entry.a == old.half);
		CHECK(entry.ratio == frozen::ratio_of(name));
		CHECK(entry.blades == (entry.rail == "S42" ? "secant" : "tangent"));
	}
	for (auto const &old : frozen::crossings) {
		auto const &entry{now[at++]};
		auto const name{std::string{old.name}};
		CAPTURE(name);
		CHECK(entry.type == "Skrz");
		CHECK(entry.name == "Skrz " + name);
		CHECK(entry.rail == name.substr(0, name.find('-')));
		CHECK(entry.a == old.half);
		CHECK(entry.ratio == frozen::ratio_of(name));
	}
}

TEST_CASE("turnout catalogue: every line of the data file is read, each type with its source") {
	std::vector<std::string> errors;
	auto const all{switch_design::read_designs(switch_design::catalogue_text(), &errors)};
	for (auto const &error : errors) {
		CAPTURE(error);
		CHECK(false);
	}
	CHECK(all.size() == switch_design::catalogue_entries().size());
	CHECK(all.size() == switch_design::catalogue().size() + switch_design::slips().size());
	std::map<std::string, int> names;
	for (auto const &entry : all) {
		CAPTURE(entry.name);
		CHECK(++names[entry.name] == 1);
		CHECK_FALSE(entry.source.empty());
		CHECK_FALSE(entry.own);
	}
}

TEST_CASE("turnout catalogue: each switch against Id-4 and ST-T1-A6, a, b, R, the angle, the pre-blade stretch, the blade, the speed") {
	for (auto const &entry : switch_design::catalogue()) {
		CAPTURE(entry.name);
		std::string error;
		auto const shape{switch_design::build(entry, error)};
		REQUIRE(error.empty());
		REQUIRE(shape.paths.size() >= 2);
		auto const twofold{switch_design::double_switch(entry.type)};
		auto const symmetric{switch_design::symmetric_switch(entry.type)};
		REQUIRE(entry.radius > 0.0);
		REQUIRE(entry.ratio > 1.0);
		// R and the angle 1:n at the end of the diverging track, of the two tracks of a symmetric switch
		if (false == twofold) {
			CHECK(shape.radius == doctest::Approx(entry.radius).epsilon(1e-12));
		}
		CHECK(shape.ratio == doctest::Approx(entry.ratio).epsilon(1e-9));
		// a as the catalogue gives it, the tangent of the arc where it gives none; the main track a + b
		auto const angle{std::atan(1.0 / entry.ratio) * (symmetric ? 0.5 : 1.0)};
		auto const tangent{entry.radius * std::tan(angle * 0.5)};
		CHECK(shape.a == entry.a);
		if (entry.type == "Rz") {
			CHECK(shape.length == doctest::Approx(entry.a + entry.b).epsilon(1e-12));
		}
		// the blades: tangent ones leave the main track at no angle, from the start of the curve or a pre-blade stretch where a is longer
		// than the tangent of the arc; secant ones of S42 cut into the stock rail at 25', the pre-blade stretch what a leaves
		REQUIRE((entry.blades == "tangent" || entry.blades == "secant"));
		// fitted to the tangents measured, not of Id-4
		auto const fitted{entry.source.find("not of Id-4") != std::string::npos};
		auto const tip{switch_design::blade_tip(entry.parts, entry.lead, entry.entry, entry.tip, symmetric)};
		if (entry.blades == "secant") {
			CHECK(entry.rail == "S42");
			CHECK(entry.lead > 0.0);
			CHECK(tip == entry.lead);
			if (entry.name != "Rz S42-140-1:7 industrial" && entry.name != "Rpd S42-140-1:7 industrial") {
				CHECK(entry.entry == doctest::Approx(symmetric ? 0.5 * kSecant : kSecant).epsilon(1e-12));
			}
		} else if (entry.a < tangent - 0.02) {
			// a shorter than the tangent: the blade meets the stock rail at the angle which makes it
			CHECK(entry.entry > 0.0);
		} else if (false == fitted) {
			CHECK(entry.entry == 0.0);
			CHECK(entry.lead == (entry.a > tangent + 0.02 ? doctest::Approx(entry.a - tangent).epsilon(1e-12) : doctest::Approx(0.0)));
			if (entry.cut > 0.0) {
				CHECK(entry.tip == doctest::Approx(entry.lead + entry.cut).epsilon(1e-12));
				CHECK(tip == entry.tip);
			} else if (false == symmetric && entry.parts.front().radius1 == entry.radius) {
				// no data of the blade: where it gets 5 mm from the stock rail, 125 mm of it planed
				CHECK(std::abs(tip - entry.lead - (std::sqrt(2.0 * entry.radius * 0.005) - 0.125)) <= 0.01);
			}
		}
		// the speed of tab. 11.2 by the radius, of the sharper switch of a double one
		auto sharpest{entry.radius};
		if (twofold && false == entry.parts2.empty()) {
			sharpest = std::min(sharpest, entry.parts2.front().radius1);
		}
		CHECK(entry.speed == switch_curve::basic_speed(sharpest));
		// the bending of tab. 11.9: the basic switches of 60E1 and 49E1 of the radii it lists, none of the double and symmetric ones
		auto const limit{switch_curve::bent_limit_of(entry.radius)};
		if (limit.has_value() && entry.type == "Rz" && entry.rail != "S42") {
			CHECK(entry.bending == limit->main);
			CHECK(entry.bending_exception == limit->main_exception);
		} else {
			CHECK(entry.bending == 0.0);
		}
		// the spacing of tab. 11.2
		CHECK(entry.spacing == (entry.type == "Rz" && entry.rail != "S42" ? table_11_2_spacing(entry.radius, entry.ratio) : 0.0));
		CHECK(entry.movable == (entry.name.find("movable nose") != std::string::npos));
	}
}

TEST_CASE("turnout catalogue: the rules for a blade tip where no data is, against the types which have it") {
	// the cut tips of 60E1 and 49E1 from the sheets SKMd-4, from the start of the curve: e1 of the straight blade, e2 of the curved one
	struct sheet {
		double radius;
		double e1;
		double e2;
	};
	std::array const sheets{sheet{190.0, 1.248, 1.248}, sheet{300.0, 1.500, 1.468}, sheet{500.0, 2.165, 2.142}, sheet{1200.0, 3.335, 3.321}};
	for (auto const &each : sheets) {
		CAPTURE(each.radius);
		// the rule of the editor: where the blade gets 5 mm from the stock rail, 125 mm of it planed, about sqrt(2R 0.005) - 0.125 (the
		// gauge face turning away by a few millimetres); and where the axes are 4.5 mm apart
		switch_design::part const arc{each.radius * 0.1, each.radius, each.radius};
		auto const editor{switch_design::blade_tip({arc}, 0.0, 0.0, 0.0, false)};
		auto const rule{std::sqrt(2.0 * each.radius * 0.005) - 0.125};
		auto const offset{std::sqrt(2.0 * each.radius * 0.0045)};
		MESSAGE("R ", each.radius, ": SKMd-4 ", each.e1, " / ", each.e2, ", the editor's rule ", editor, " (", editor - each.e1, "), sqrt(2R 0.005) - 0.125 ", rule, " (", rule - each.e1, "), 4.5 mm off ", offset, " (",
		        offset - each.e1, ")");
		CHECK(std::abs(editor - rule) <= 0.005);
		CHECK(std::abs(editor - each.e1) <= 0.11);
		CHECK(std::abs(offset - each.e1) <= 0.15);
		// the catalogue keeps the sheets' values, not the rule
		for (auto const &entry : switch_design::catalogue()) {
			if (entry.type == "Rz" && entry.rail != "S42" && std::abs(entry.radius - each.radius) < 0.5 && entry.cut > 0.0) {
				CAPTURE(entry.name);
				CHECK(std::abs(entry.cut - each.e1) < 0.001);
			}
		}
	}
	// the secant blades of S42: no sheet found, the angle of 25' is the rule
	for (auto const &entry : switch_design::catalogue()) {
		if (entry.blades == "secant") {
			CAPTURE(entry.name);
			CHECK(entry.source.find("unconfirmed") != std::string::npos);
		}
	}
}

TEST_CASE("turnout catalogue: a template of the user is a line of the same format and keeps its movable nose") {
	switch_design::design own;
	own.name = "My switch 1";
	own.type = "Rz";
	own.rail = "60E1";
	own.main = 41.594;
	own.a = 20.797;
	own.b = 20.797;
	own.parts = {{41.5, 500.0, 500.0}, {0.25, 0.0, 0.0}};
	own.movable = true;
	auto const line{switch_design::own_line(own)};
	CAPTURE(line);
	CHECK(line.find("frog movable") != std::string::npos);
	CHECK(line.find("switch My_switch_1 ") == 0);
	std::vector<std::string> errors;
	auto const back{switch_design::read_designs(line, &errors)};
	CHECK(errors.empty());
	REQUIRE(back.size() == 1);
	CHECK(back.front().name == own.name);
	CHECK(back.front().movable);
	CHECK(back.front().main == own.main);
	CHECK(back.front().a == own.a);
	check_same_parts(own.parts, back.front().parts);
	// without one, the line as the builds before wrote it, which read no key they don't know
	own.movable = false;
	CHECK(switch_design::own_line(own).find("frog") == std::string::npos);
	// and a line they wrote reads as it did
	auto const older{switch_design::read_designs("# switch templates\nswitch Old_one type Rpd rail 49E1 main 0.0000 stagger 10.5230 a 0.0000 b 0.0000 lead 0.0000 entry 0.0000 tip 0.0000 lead2 0.0000 "
	                                             "entry2 0.0000 tip2 0.0000 parts 2 12.6110 190.0000 190.0000 4.0000 0.0000 0.0000 parts2 1 12.6110 190.0000 190.0000\r\n")};
	REQUIRE(older.size() == 1);
	CHECK(older.front().name == "Old one");
	CHECK(older.front().type == "Rpd");
	CHECK(older.front().stagger == 10.523);
	CHECK_FALSE(older.front().movable);
	CHECK(older.front().parts.size() == 2);
	CHECK(older.front().parts2.size() == 1);
	CHECK(older.front().lead2 == 0.0);
	std::string error;
	CHECK(switch_design::build(older.front(), error).paths.size() == 5);
}

TEST_CASE("turnout catalogue: a line which isn't one is left out, why told") {
	std::vector<std::string> errors;
	auto const read{switch_design::read_designs("switch \"A\" type Rz rail 60E1 radius 500 ratio 12 colour red\n"
	                                            "switch \"B\" type Rz rail 60E1 radius 500 ratio twelve\n"
	                                            "switch \"C\" type Rz rail 60E1 radius 500 radius 300 ratio 12\n"
	                                            "switch \"D type Rz rail 60E1 radius 500 ratio 12\n"
	                                            "switch \"E\" type Rq rail 60E1 radius 500 ratio 12\n"
	                                            "switch \"F\" type Rz rail 60E1 radius 500 ratio 12 a 20.797 b 20.797 source made up\n",
	                                            &errors)};
	REQUIRE(read.size() == 1);
	CHECK(read.front().name == "F");
	CHECK(read.front().source == "made up");
	REQUIRE(errors.size() == 5);
	CHECK(errors[0].find("colour") != std::string::npos);
	CHECK(errors[1].find("number") != std::string::npos);
	CHECK(errors[2].find("twice") != std::string::npos);
	CHECK(errors[3].find("quote") != std::string::npos);
	CHECK(errors[4].find("Rq") != std::string::npos);
}

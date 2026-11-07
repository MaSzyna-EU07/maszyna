/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodesrk.h"
#include "application/editorgestures.h"
#include "application/editormodeshared.h"
#include "application/editorprojection.h"
#include "application/editoruilayer.h"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"
#include "editor/editorKilometrage.hpp"
#include "editor/editorSwitchStore.hpp"
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
#include <format>
#include <limits>
#include <map>
#include <ranges>
#include <string_view>
#include <tuple>
#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace
{

char const *const kMark{"//$s"};
double constexpr kOverlayRange{4000.0};
double constexpr kJointRange{1500.0}; // m, of the joints shown in the scenery
double constexpr kLabelRange{700.0}; // m, of the names shown there, farther ones crowd the horizon
double constexpr kRailHead{0.18}; // m, above the axis of a path

double constexpr kLineLimit{30000.0}; // m, of a line followed out to the next station
double constexpr kGapReach{1500.0}; // m, the tracks going on beyond the end of a line found within it
double constexpr kBlockShare{1.1}; // of the braking distance, the block sections towards the end of the tracks
double constexpr kPredicted{305.0}; // m, out from the next station to its entry signal, while it has none
// the events of a block section, switching the block signal guarding it
char const *const kControl{"sbl_odstep.inc"};
// the SHP track resonator: its model on the axis, 0.2 m below it, turned as a signal facing the trains, and its event
char const *const kResonator{"rezonator.inc"};
double constexpr kShpBefore{200.0}; // m, of the resonator before an entry signal, a block signal and a warning disc
double constexpr kShpTolerance{5.0}; // m, either way
double constexpr kShpWindow{100.0}; // m, either way of its place, a resonator standing is taken for the one of the signal
double constexpr kResonatorDrop{0.2}; // m, of its origin below the axis
double constexpr kBoundaryReach{30.0}; // m, either way of a block signal, a click while drawing the boundaries takes it away
double constexpr kBoundaryEnd{50.0}; // m, of a block boundary drawn from the entry signals at the ends of the line
double constexpr kWrongTrack{30.0}; // m, a standing wrong-track block signal counts for the one of a boundary within it of its place

using name_set = srk_tool::name_set;
// a map by the names, looked up by their views too
template <typename Value_> using name_map = std::unordered_map<std::string, Value_, srk_tool::name_hash, std::equal_to<>>;

// m, of a train at the speed, as Ie-4 § 8 takes it: the least block section of the automatic block, a warning disc that far before
// its signal
double braking_distance(double const Speed)
{
	if (Speed <= 60.0)
		return 400.0;
	if (Speed <= 100.0)
		return 700.0;
	return Speed <= 140.0 ? 1000.0 : 1300.0;
}

ImU32 constexpr kPlain{IM_COL32(170, 176, 188, 255)};
ImU32 constexpr kSwitch{IM_COL32(240, 200, 90, 255)};
ImU32 constexpr kApproach{IM_COL32(95, 100, 112, 255)};
ImU32 constexpr kHovered{IM_COL32(255, 255, 255, 255)};
ImU32 constexpr kSignal{IM_COL32(240, 95, 85, 255)};
ImU32 constexpr kJoint{IM_COL32(255, 255, 255, 235)};
ImU32 constexpr kUseful{IM_COL32(120, 230, 255, 255)};
double constexpr kTrainMargin{25.0}; // m, of a useful length the train leaves: 10 to stop, 10 to see the signal, 5 to the joint, ST-T1-A6
double constexpr kUnequal{20.0}; // m, the useful lengths of the two ways differing more are shown: ST-T1-A6 wants them the same
ImU32 constexpr kUnfit{IM_COL32(255, 190, 40, 255)};
double constexpr kFastest{250.0}; // km/h, a track set faster is taken for one without a limit
ImVec4 constexpr kWrong{1.0f, 0.42f, 0.36f, 1.0f};
ImVec4 constexpr kDoubtful{1.0f, 0.75f, 0.3f, 1.0f};
ImVec4 constexpr kRight{0.45f, 0.85f, 0.45f, 1.0f};

bool is_switch(TTrack const &Track)
{
	return Track.eType != tt_Normal;
}

// a word of the record line: blanks taken out, - for none
std::string word_of(std::string const &Text)
{
	std::string result;
	for (auto const c : Text)
		if (false == std::isspace(static_cast<unsigned char>(c)))
			result += c;
	return result.empty() ? std::string{"-"} : result;
}

std::string shown(std::string const &Word)
{
	return Word == "-" ? std::string{} : Word;
}

// the items of a list of the record line, "a=1,b=2", each with its value; the ones without one left out
std::vector<std::pair<std::string, std::string>> pairs_of(std::string const &Word)
{
	std::vector<std::pair<std::string, std::string>> result;
	std::size_t from{0};
	while (from < Word.size())
	{
		auto const comma{std::min(Word.find(',', from), Word.size())};
		auto const item{Word.substr(from, comma - from)};
		if (auto const equals{item.find('=')}; equals != std::string::npos && equals > 0)
			result.emplace_back(item.substr(0, equals), item.substr(equals + 1));
		from = comma + 1;
	}
	return result;
}

// the name of the station, the rest of the record line
void read_name(std::istringstream &Words, std::string &Name)
{
	std::getline(Words, Name);
	Name.erase(0, Name.find_first_not_of(' '));
	// windows-1250 like the rest of the scenery file; earlier builds wrote it in utf-8, which stays as it is
	if (std::u32string decoded; false == utf8_to_utf32(Name, decoded))
		Name = win1250_to_utf8(Name);
}

// a word of the record line written as it is: one without blanks, commas and equals signs
bool plain_word(std::string const &Word)
{
	return false == Word.empty() && Word.find_first_of(" ,=\t") == std::string::npos;
}

// the number of the switch of a unit: of its name once named after the station, of its record, else of its place
std::string unit_number(station::unit const &Unit, std::string_view const Stem, std::size_t const Index)
{
	if (auto const &name{Unit.parts.front()->name()}; name.size() > Stem.size() && name.compare(0, Stem.size(), Stem) == 0)
	{
		auto digits{name.substr(Stem.size())};
		while (false == digits.empty() && std::isalpha(static_cast<unsigned char>(digits.back())))
			digits.pop_back();
		if (false == digits.empty() && std::ranges::all_of(digits, [](char const Digit) { return std::isdigit(static_cast<unsigned char>(Digit)) != 0; }))
			return digits;
	}
	if (auto const record{switch_store::named(*Unit.parts.front())}; record && record->number != "-")
		return record->number;
	return std::to_string(Index + 1);
}

// the number of the station track a circuit is named after: bpd_it5b of track 5; empty for the other names, bpd_ita of the
// circuit before the entry signal A among them
std::string track_of(std::string_view const Name, std::string const &Stem)
{
	auto const head{Stem + "it"};
	if (Stem.empty() || Name.size() <= head.size() || Name.compare(0, head.size(), head) != 0)
		return {};
	std::string number{Name.substr(head.size())};
	if (std::ranges::none_of(number, [](char const Letter) { return std::isdigit(static_cast<unsigned char>(Letter)) != 0; }))
		return {};
	if (number.size() > 1 && std::isalpha(static_cast<unsigned char>(number.back())) && std::isdigit(static_cast<unsigned char>(number[number.size() - 2])))
		number.pop_back();
	return number;
}

// the circuit between an entry signal and the station: bpd_ita
bool entry_circuit(std::string_view const Name, std::string const &Stem)
{
	auto const head{Stem + "it"};
	return false == Stem.empty() && Name.size() > head.size() && Name.compare(0, head.size(), head) == 0 &&
	       std::all_of(Name.begin() + head.size(), Name.end(), [](char const Letter) { return std::isalpha(static_cast<unsigned char>(Letter)) != 0; });
}

// the name of a signal as its plate and the schematic show it: bpd_tm3 as Tm3, bpd_f1 as F1
std::string signal_shown(std::string const &Name, std::string_view const Stem)
{
	auto text{false == Stem.empty() && Name.size() > Stem.size() && ToLower(Name).compare(0, Stem.size(), Stem) == 0 ? Name.substr(Stem.size()) : Name};
	std::ranges::transform(text, text.begin(), [](char const Letter) { return static_cast<char>(std::toupper(static_cast<unsigned char>(Letter))); });
	if (text.size() > 2 && text[0] == 'T' && (text[1] == 'M' || text[1] == 'O'))
		text[1] = static_cast<char>(std::tolower(static_cast<unsigned char>(text[1])));
	return text;
}

// a signal in the names of the block sections: a block signal by its number, a station one by its letter, with the number of the
// track: 2411, A2
std::string section_tag(std::string const &Name, int const Track)
{
	auto const cut{Name.rfind('_')};
	auto tag{cut == std::string::npos ? Name : Name.substr(cut + 1)};
	std::ranges::transform(tag, tag.begin(), [](char const Letter) { return static_cast<char>(std::toupper(static_cast<unsigned char>(Letter))); });
	return std::format("{}{}", tag, Track);
}

// the way the trains a signal faces run, in the plan: it stands turned to them
glm::dvec2 travel_of(double const Yaw)
{
	auto const yaw{glm::radians(Yaw + 180.0)};
	return {std::sin(yaw), std::cos(yaw)};
}

// how far the point lies from the tracks of the station, in the plan
double gap_to(station::layout const &Layout, glm::dvec3 const &Point)
{
	auto best{std::numeric_limits<double>::max()};
	for (auto const &path : Layout.paths)
		for (auto const &point : path.points)
			best = std::min(best, glm::distance(glm::dvec2{point.x, point.z}, glm::dvec2{Point.x, Point.z}));
	return best;
}

ImU32 circuit_colour(std::string const &Name)
{
	static std::array<ImU32, 6> const palette{IM_COL32(110, 200, 250, 255), IM_COL32(250, 150, 90, 255), IM_COL32(150, 220, 120, 255), IM_COL32(230, 120, 200, 255), IM_COL32(240, 220, 100, 255), IM_COL32(150, 140, 250, 255)};
	return palette[std::hash<std::string>{}(Name) % palette.size()];
}

template <typename Record_> std::string anchor_of(Record_ const &Entry)
{
	return Entry.track != nullptr && false == Entry.track->m_editorremoved ? Entry.track->name() : Entry.anchor;
}

// the name without the code of the station: bpd_4 as 4
std::string without_stem(std::string const &Name, std::string_view const Stem)
{
	return false == Stem.empty() && Name.size() > Stem.size() && Name.compare(0, Stem.size(), Stem) == 0 ? Name.substr(Stem.size()) : Name;
}

// a limit of a useful length in words, as "from" and "to" take it
std::string limit_text(station::limit const &Limit, std::string const &Stem)
{
	using enum station::limit::kind;
	auto const name{without_stem(Limit.name, Stem)};
	switch (Limit.what)
	{
	case signal:
		return format(STR_C("signal %s"), name.c_str());
	case fouling:
		return format(STR_C("the fouling point of switch %s"), name.c_str());
	case start:
		return format(STR_C("the start of switch %s"), name.c_str());
	case buffer:
		return STR_C("the buffer stop");
	case crossing:
		return STR_C("the level crossing");
	default:
		return name.empty() ? std::string{STR_C("the joint")} : format(STR_C("the joint by switch %s"), name.c_str());
	}
}

// the points of the track from one distance along it to the other
std::vector<glm::dvec3> stretch_of(station::lengths const &Track, double const From, double const To)
{
	auto const at = [&](double const Along) {
		auto const after{std::min<std::size_t>(std::ranges::upper_bound(Track.distances, Along) - Track.distances.begin(), Track.distances.size() - 1)};
		auto const before{after > 0 ? after - 1 : 0};
		auto const span{Track.distances[after] - Track.distances[before]};
		return glm::mix(Track.points[before], Track.points[after], span > 1e-9 ? std::clamp((Along - Track.distances[before]) / span, 0.0, 1.0) : 0.0);
	};
	std::vector result{at(std::min(From, To))};
	for (std::size_t i = 0; i < Track.distances.size(); ++i)
		if (Track.distances[i] > std::min(From, To) + 0.05 && Track.distances[i] < std::max(From, To) - 0.05)
			result.push_back(Track.points[i]);
	result.push_back(at(std::max(From, To)));
	if (From > To)
		std::ranges::reverse(result);
	return result;
}

glm::dvec3 axis_point(station::layout const &Layout, double const Chainage)
{
	auto const after{std::ranges::lower_bound(Layout.chainage, Chainage)};
	if (after == Layout.chainage.begin())
		return Layout.axis.front();
	if (after == Layout.chainage.end())
		return Layout.axis.back();
	auto const i{static_cast<std::size_t>(after - Layout.chainage.begin())};
	auto const step{Layout.chainage[i] - Layout.chainage[i - 1]};
	return glm::mix(Layout.axis[i - 1], Layout.axis[i], step > 1e-9 ? (Chainage - Layout.chainage[i - 1]) / step : 0.0);
}

// km/h, a signal shows for the route at the speed: 40, 60 or 100 with a light strip, Ie-1; 0 for the highest the line allows
int shown_speed(double const Speed, double const Line)
{
	if (Speed <= 0.0 || Speed >= Line - 1.0)
		return 0;
	auto shown{40};
	if (Speed >= 98.0)
		shown = 100;
	else if (Speed >= 58.0)
		shown = 60;
	return shown >= Line ? 0 : shown;
}

// km/h, a signal slows the trains to with the next one at stop standing that far on, Ie-4 § 8 ust. 5; 140 with a W21 indicator
int shortened_speed(double const Length)
{
	if (Length < 400.0)
		return 40;
	if (Length < 700.0)
		return 60;
	return Length < 1000.0 ? 100 : 140;
}

// what the lights of a signal must show for the routes of its trains
struct head_need
{
	std::vector<int> speeds; // km/h, the lower ones
	bool informs{false};
	bool white{false};
	int shortened{0};
	bool indicator{false};
};

// a speed the head shows, added once
void add_speed(std::vector<int> &Speeds, int const Speed)
{
	if (Speed > 0 && std::ranges::find(Speeds, Speed) == Speeds.end())
		Speeds.push_back(Speed);
}

// the lower speed of a route shorter than the braking distance, Slower, where it's below the one the head shows, Limit: the indicator
// above 100 km/h
void add_shortened(head_need &Need, int const Slower, double const Limit)
{
	if (Slower >= Limit)
		return;
	if (Slower > 100)
		Need.indicator = true;
	else
	{
		add_speed(Need.speeds, Slower);
		Need.shortened = Need.shortened == 0 ? Slower : std::min(Need.shortened, Slower);
	}
}

head_need need_of(std::vector<station::route> const &Routes, bool const Main, double const Line, double const Braking)
{
	head_need need;
	need.white = Main;
	for (auto const &route : Routes)
	{
		auto const shown{Main ? shown_speed(route.speed, Line) : 0};
		add_speed(need.speeds, shown);
		auto const told{route.next >= 0 && route.warning < 0};
		need.informs = need.informs || told;
		if (false == Main || false == told || route.length >= Braking)
			continue;
		add_shortened(need, shortened_speed(route.length), shown > 0 ? shown : Line);
	}
	std::ranges::sort(need.speeds);
	return need;
}

bool has_speed(std::vector<int> const &Speeds, int const Speed)
{
	return std::ranges::find(Speeds, Speed) != Speeds.end();
}

// the template has the lights the need takes, by its chambers: an orange one to tell the next signal shows stop, a lower orange one
// for a lower speed, a strip for 60 and 100 by the speeds its events set, Ie-1 § 7, two orange ones to show a lower speed with the
// next signal at stop
bool has_aspect(signal_head const &Head, char const *Name)
{
	return std::ranges::find(Head.aspect_names, std::string_view{Name}) != Head.aspect_names.end();
}

// a semaphore, by the aspects of its description: Sr1 and Sr2, Sr3 of the second arm for any lower speed, 40 km/h being the one it
// gives, Sz on a main signal. It tells nothing of the next signal, a warning disc does that
bool shows_arms(signal_head const &Head, head_need const &Need)
{
	if (false == has_aspect(Head, "Sr1") || false == has_aspect(Head, "Sr2") || (Need.white && false == has_aspect(Head, "Sz")))
		return false;
	return Need.speeds.empty() || has_aspect(Head, "Sr3");
}

bool shows(signal_head const &Head, head_need const &Need)
{
	if (Head.arms > 0)
		return shows_arms(Head, Need);
	auto const &lamps{Head.lamps};
	auto const count = [&lamps](char const Lamp) { return std::ranges::count(lamps, Lamp); };
	if (count('z') == 0 || count('c') == 0 || (Need.white && count('b') == 0))
		return false;
	if (Need.informs && count('p') == 0)
		return false;
	if (Need.speeds.empty())
		return true;
	if (lamps.find('p', lamps.find('c')) == std::string::npos || (Need.informs && count('p') < 2))
		return false;
	return std::ranges::all_of(Need.speeds, [&Head](int const Speed) { return has_speed(Head.speeds, Speed); });
}

// the more lights and strips beyond the need, the more
int surplus(signal_head const &Head, head_need const &Need)
{
	if (Head.arms > 0)
		return Head.arms * 4 + (has_aspect(Head, "Sz") && false == Need.white ? 1 : 0);
	auto result{static_cast<int>(Head.lamps.size()) * 4};
	for (auto const speed : {60, 100})
		if (has_speed(Head.speeds, speed) && false == has_speed(Need.speeds, speed))
			++result;
	return result;
}

// of the templates of the kind and the mount that fit the need, the one with the least beyond it, the most like the one named;
// -1 for none. A semaphore is swapped for a semaphore only, a signal with lights for one with lights
int best_head(std::vector<signal_head> const &Heads, std::string_view const Kind, std::string_view const Mount, std::string_view const Like, int const Aspects, head_need const &Need, bool const Arms)
{
	auto best{-1};
	std::tuple<int, int, std::size_t> score;
	for (int index = 0; index < static_cast<int>(Heads.size()); ++index)
	{
		auto const &head{Heads[index]};
		if (head.kind != Kind || head.mount != Mount || head.aspects != Aspects || (head.arms > 0) != Arms || false == shows(head, Need))
			continue;
		std::size_t common{0};
		while (common < head.name.size() && common < Like.size() && head.name[common] == Like[common])
			++common;
		std::tuple<int, int, std::size_t> const candidate{surplus(head, Need), -static_cast<int>(common), head.name.size()};
		if (best < 0 || candidate < score)
		{
			best = index;
			score = candidate;
		}
	}
	return best;
}

// the aspects a semaphore needs for it: Sr1, Sr2, Sr3 for a lower speed, Sz on a main signal
std::string need_arms_text(head_need const &Need)
{
	std::string text{"Sr1, Sr2"};
	if (false == Need.speeds.empty())
		text += ", Sr3 (" + format(STR_C("%s km/h"), "40") + ")";
	if (Need.white)
		text += ", Sz";
	return text;
}

// the lights of the need in words: green, two orange, red, white; 40, 60 km/h with an orange strip
std::string need_text(head_need const &Need)
{
	std::string text{STR("green")};
	if (Need.informs && false == Need.speeds.empty())
		text += ", " + STR("two orange");
	else if (Need.informs || false == Need.speeds.empty())
		text += ", " + STR("orange");
	text += ", " + STR("red");
	if (Need.white)
		text += ", " + STR("white");
	if (Need.speeds.empty())
		return text;
	std::string speeds;
	for (auto const speed : Need.speeds)
		speeds += std::format("{}{}", speeds.empty() ? "" : ", ", speed);
	text += "; " + format(STR_C("%s km/h"), speeds.c_str());
	if (has_speed(Need.speeds, 60))
		text += ", " + STR("an orange strip");
	if (has_speed(Need.speeds, 100))
		text += ", " + STR("a green strip");
	return text;
}

ImU32 lamp_colour(char const Lamp)
{
	switch (Lamp)
	{
	case 'z': return IM_COL32(40, 210, 80, 255);
	case 'c': return IM_COL32(230, 40, 40, 255);
	case 'p': return IM_COL32(255, 160, 30, 255);
	case 'b': return IM_COL32(235, 235, 235, 255);
	case 'n': return IM_COL32(60, 110, 255, 255);
	default: return IM_COL32(70, 70, 70, 255);
	}
}

// the lights of a template in a row, from its top on the left, its strips after them
void lamp_row(std::string const &Lamps, std::vector<int> const &Speeds)
{
	auto const height{ImGui::GetTextLineHeight()};
	auto const radius{height * 0.3f};
	auto const origin{ImGui::GetCursorScreenPos()};
	auto *draw{ImGui::GetWindowDrawList()};
	auto x{origin.x + radius};
	for (auto const lamp : Lamps)
	{
		draw->AddCircleFilled(ImVec2(x, origin.y + height * 0.5f), radius, lamp_colour(lamp));
		x += radius * 2.6f;
	}
	for (auto const &[speed, colour] : {std::make_pair(60, lamp_colour('p')), std::make_pair(100, lamp_colour('z'))})
		if (has_speed(Speeds, speed))
		{
			draw->AddRectFilled(ImVec2(x, origin.y + height * 0.38f), ImVec2(x + radius * 3.0f, origin.y + height * 0.62f), colour, 1.0f);
			x += radius * 3.6f;
		}
	ImGui::Dummy(ImVec2(std::max(1.0f, x - origin.x), height));
}

// m, the routes of the trains are followed from a signal at most, beyond it the next one needs no telling of
double route_reach(double const Braking)
{
	return std::max(3000.0, 3.0 * Braking);
}

// how far the point lies from the path in the plan, Parameter: of the nearest point of it, found among points 8 m or so apart and
// narrowed down between the two by them
double nearest_of(TTrack const &Track, int const Path, glm::dvec3 const &Point, double &Parameter)
{
	geometry::bezier const curve{Track.m_paths[Path]};
	auto const gap = [&](double const At) { return geometry::plan_distance(curve.point(At), Point); };
	auto const count{std::clamp(static_cast<int>(curve.plan_length(16) / 8.0), 16, 512)};
	auto best{std::numeric_limits<double>::max()};
	for (int k = 0; k <= count; ++k)
		if (auto const distance{gap(static_cast<double>(k) / count)}; distance < best)
		{
			best = distance;
			Parameter = static_cast<double>(k) / count;
		}
	auto low{std::max(0.0, Parameter - 1.0 / count)};
	auto high{std::min(1.0, Parameter + 1.0 / count)};
	for (int step = 0; step < 40; ++step)
	{
		auto const third{(high - low) / 3.0};
		auto const first{low + third};
		auto const second{high - third};
		if (gap(first) < gap(second))
			high = second;
		else
			low = first;
	}
	if (auto const narrowed{(low + high) * 0.5}; gap(narrowed) < best)
	{
		best = gap(narrowed);
		Parameter = narrowed;
	}
	return best;
}

// a standing signal by the track its trains read it on, no track for a signal of another kind or by none
station::post post_of(standing_signal const &Signal)
{
	using role = station::signal::role;
	station::post post;
	if (Signal.kind == "main")
		post.kind = role::entry;
	else if (Signal.kind == "block")
		post.kind = role::block;
	else if (Signal.kind == "warning")
		post.kind = role::warning;
	else if (Signal.kind == "shunt")
		post.kind = role::shunting;
	else
		return post;
	post.point = Signal.position;
	// the track it's read on, where it stands by it: an event put on a track before the signal leaves the track it stands by
	if (Signal.track != nullptr && Signal.facing != 0 && false == Signal.track->m_editorremoved)
		for (int candidate = 0; candidate < static_cast<int>(Signal.track->m_paths.size()); ++candidate)
			if (double parameter{0.0}; nearest_of(*Signal.track, candidate, Signal.position, parameter) < 4.5)
			{
				post.track = Signal.track;
				post.facing = Signal.facing;
				return post;
			}
	int path{0};
	post.track = kilometrage::track_near(Signal.position, 4.5, path);
	if (post.track == nullptr)
		return post;
	geometry::bezier const curve{post.track->m_paths[path]};
	auto parameter{0.0};
	nearest_of(*post.track, path, Signal.position, parameter);
	auto const tangent{curve.first(parameter)};
	auto const travel{travel_of(Signal.yaw)};
	post.facing = travel.x * tangent.x + travel.y * tangent.z >= 0.0 ? 1 : -1;
	return post;
}

// the entry signal of the next station along the line: the first main signal 100 m out or more facing out, of the standing ones,
// -1 for none; Distance: out to it, else to where the signals of the line end
int next_entry(station::line const &Line, std::vector<standing_signal> const &Standing, double &Distance)
{
	auto found{-1};
	for (std::size_t k = 0; k < Standing.size(); ++k)
	{
		if (Standing[k].kind != "main")
			continue;
		double gap{0.0};
		auto const distance{station::distance_on(Line, Standing[k].position, gap)};
		if (gap > 4.5 || distance < 100.0 || distance > Line.length() - 1.0)
			continue;
		if (auto const way{station::point_on(Line, distance).way}; glm::dot(travel_of(Standing[k].yaw), glm::dvec2{way.x, way.z}) < 0.87)
			continue;
		if (found < 0 || distance < Distance)
		{
			found = static_cast<int>(k);
			Distance = distance;
		}
	}
	if (found < 0)
		Distance = Line.station ? Line.length() - kPredicted : Line.length();
	return found;
}

// where the trains run into the event of an SHP track resonator: the end of the track it's put on, the way they run
struct shp_trigger
{
	glm::dvec3 point{0.0};
	glm::dvec3 way{0.0, 0.0, 1.0};
};

bool shp_event(std::string_view const Name)
{
	return Name.size() > 4 && ToLower(std::string{Name.substr(Name.size() - 4)}) == "_shp";
}

// the trains run into the event of the list of the track at its start, towards its end, or at its end the other way
shp_trigger trigger_of(TTrack const &Track, bool const Towards_end)
{
	geometry::bezier const curve{Track.m_paths.front()};
	auto const tangent{curve.first(Towards_end ? 0.0 : 1.0)};
	glm::dvec3 way{tangent.x, 0.0, tangent.z};
	if (glm::length(way) > 1e-9)
		way = glm::normalize(way) * (Towards_end ? 1.0 : -1.0);
	else
		way = glm::dvec3{0.0, 0.0, 1.0};
	return {curve.point(Towards_end ? 0.0 : 1.0), way};
}

std::vector<shp_trigger> shp_triggers()
{
	std::vector<shp_trigger> result;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
			continue;
		for (auto const list : {1, 2, 4, 5})
			for (auto const &[name, handler] : editor_track::events(*track, list))
				if (shp_event(name))
					result.push_back(trigger_of(*track, list % 3 == 2));
	}
	return result;
}

// m before the signal of the point of its way the trains run into the event at, the nearest within 2.5 m; the lowest double for
// none
double before_on_way(std::vector<station::approach_point> const &Way, shp_trigger const &Trigger)
{
	auto before{std::numeric_limits<double>::lowest()};
	auto nearest{2.5};
	for (auto const &point : Way)
		if (auto const gap{geometry::plan_distance(point.point, Trigger.point)}; gap < nearest && glm::dot(point.way, Trigger.way) > 0.7)
		{
			nearest = gap;
			before = point.distance;
		}
	return before;
}

// m before the signal the trains run into the event of an SHP resonator, the one nearest the place wanted within kShpWindow of it;
// -1 for none
double shp_before(std::vector<station::approach_point> const &Way, glm::dvec3 const &Signal, glm::dvec3 const &Travel, std::vector<shp_trigger> const &Triggers, double const Wanted)
{
	auto best{-1.0};
	auto miss{std::numeric_limits<double>::max()};
	for (auto const &trigger : Triggers)
	{
		if (geometry::plan_distance(trigger.point, Signal) > Wanted + kShpWindow + 10.0 || glm::dot(trigger.way, Travel) < 0.7)
			continue;
		auto before{std::numeric_limits<double>::lowest()};
		if (geometry::plan_distance(trigger.point, Signal) < 12.0)
			before = glm::dot(glm::dvec3{Signal.x - trigger.point.x, 0.0, Signal.z - trigger.point.z}, Travel);
		else
			before = before_on_way(Way, trigger);
		if (before < -10.0 || std::abs(before - Wanted) > kShpWindow)
			continue;
		if (std::abs(before - Wanted) < miss)
		{
			miss = std::abs(before - Wanted);
			best = std::max(0.0, before);
		}
	}
	return best;
}

// m from the point to the nearer end of the track
double end_gap(TTrack const &Track, glm::dvec3 const &Point)
{
	auto const &ends{Track.m_paths.front().points};
	return std::min(glm::distance(Point, ends[segment_data::point::start]), glm::distance(Point, ends[segment_data::point::end]));
}

// of the way of the trains, the point the SHP resonator wanted m before the signal goes at, within kShpTolerance: a joint of the
// tracks, Joint, its event put on the plain track the trains run into there, else a point of a plain track away from its ends, the
// track cut at it; -1 for none
int resonator_at(std::vector<station::approach_point> const &Way, double const Wanted, bool &Joint)
{
	auto best{-1};
	auto miss{kShpTolerance + 1e-6};
	Joint = false;
	for (std::size_t k = 0; k + 1 < Way.size(); ++k)
		if (Way[k + 1].track != Way[k].track && Way[k].track->eType == tt_Normal && std::abs(Way[k].distance - Wanted) < miss)
		{
			miss = std::abs(Way[k].distance - Wanted);
			best = static_cast<int>(k);
			Joint = true;
		}
	if (best >= 0)
		return best;
	for (std::size_t k = 0; k < Way.size(); ++k)
		if (Way[k].track->eType == tt_Normal && end_gap(*Way[k].track, Way[k].point) > 0.5 && std::abs(Way[k].distance - Wanted) < miss)
		{
			miss = std::abs(Way[k].distance - Wanted);
			best = static_cast<int>(k);
		}
	return best;
}

// the template of a block signal of the aspects wanted, Like: the one standing, empty for the one chosen in Signals, taken
// where it has them, else the one of their mount with them, its counterpart by the name first, then of its lights; not the first of
// a line, whose events tell a station signal; Wrong: of a wrong-track one, standing on the left, its head leaning right where one
// does, a head leaning so only for it; empty for none
// the head leans as a block signal of the way wants it: right for a wrong-track one
bool leans_for(signal_head const &Head, bool const Wrong)
{
	return (Head.lean == "right") == Wrong;
}

// the block signal standing, Like: its template, empty for the one chosen in Signals; nullptr for none
signal_head const *block_like(std::vector<signal_head> const &Heads, std::string_view const Like)
{
	signal_head const *like{nullptr};
	for (auto const &head : Heads)
		if (head.kind == "block" && (Like.empty() ? head.chosen : head.file == Like))
			like = &head;
	return like;
}

// the name of the template of the other aspects: its 3 a 4, its 4 a 3
std::string counterpart_of(signal_head const *Like)
{
	auto counterpart{Like != nullptr ? Like->name : std::string{}};
	if (auto const digit{counterpart.find_first_of("34")}; digit != std::string::npos)
		counterpart[digit] = counterpart[digit] == '3' ? '4' : '3';
	return counterpart;
}

// how unlike the head is to the one standing: of another mount the most, then by the name, then by the lights; 0 for none standing
int unlikeness(signal_head const &Head, signal_head const *Like, std::string const &Counterpart)
{
	if (Like == nullptr)
		return 0;
	auto score{0};
	if (Head.mount != Like->mount)
		score += 4;
	if (Head.name != Counterpart)
		score += 2;
	if (Head.lamps != Like->lamps)
		score += 1;
	return score;
}

// of the templates of the block signals of the aspects, the one most like the one standing, the shortest file of those; nullptr
// for none
signal_head const *best_block(std::vector<signal_head> const &Heads, int const Aspects, bool const Wrong, bool const Any_leans, signal_head const *Like)
{
	auto const counterpart{counterpart_of(Like)};
	signal_head const *best{nullptr};
	auto rank{std::make_pair(std::numeric_limits<int>::max(), std::size_t{0})};
	for (auto const &head : Heads)
	{
		if (head.kind != "block" || head.aspects != Aspects || ToLower(head.name).find("pierwszy") != std::string::npos || (Any_leans && false == leans_for(head, Wrong)))
			continue;
		if (auto const score{std::make_pair(unlikeness(head, Like, counterpart), head.file.size())}; score < rank)
		{
			rank = score;
			best = &head;
		}
	}
	return best;
}

std::string block_template(std::vector<signal_head> const &Heads, std::string_view const Like, int const Aspects, bool const Wrong = false)
{
	auto const any_leans{std::ranges::any_of(Heads, [Aspects, Wrong](signal_head const &Head) { return Head.kind == "block" && Head.aspects == Aspects && leans_for(Head, Wrong); })};
	auto const *like{block_like(Heads, Like)};
	if (like != nullptr && like->aspects == Aspects && (leans_for(*like, Wrong) || false == any_leans))
		return like->file;
	auto const *best{best_block(Heads, Aspects, Wrong, any_leans, like)};
	return best != nullptr ? best->file : std::string{};
}

// what a warning disc tells of the semaphore after it: stop, the highest speed and, with a second arm, 40 km/h
std::vector<std::string> disc_classes(signal_head const &Main)
{
	std::vector<std::string> classes{"stop", "vmax"};
	if (has_aspect(Main, "Sr3"))
		classes.emplace_back("40");
	return classes;
}

// of the events a disc repeats, how many apart for the classes given; -1 when it has no event for one of them
int disc_apart(signal_head const &Disc, std::vector<std::string> const &Classes)
{
	std::vector<std::string> events;
	for (auto const &type : Classes)
	{
		auto const found{std::ranges::find_if(Disc.repeats, [&](auto const &Repeat) { return Repeat.first == type; })};
		if (found == Disc.repeats.end() || found->second.empty())
			return -1;
		events.push_back(found->second);
	}
	std::ranges::sort(events);
	return static_cast<int>(std::ranges::unique(events).begin() - events.begin());
}

// the disc tells apart all the semaphore after it shows
bool disc_tells(signal_head const &Disc, signal_head const &Main)
{
	auto const classes{disc_classes(Main)};
	return disc_apart(Disc, classes) == static_cast<int>(classes.size());
}

// the warning disc of a semaphore, of the discs described with the events they repeat: one telling apart what the semaphore does,
// stop, the highest speed and, with a second arm, 40 km/h, with the fewest aspects, of the mount given, any for none. Empty for the
// chosen disc: before a signal with lights, before a semaphore not described, with no disc described so
std::string disc_template(std::vector<signal_head> const &Heads, signal_head const *Main, std::string_view const Mount)
{
	if (Main == nullptr || Main->arms == 0)
		return {};
	auto const classes{disc_classes(*Main)};
	signal_head const *chosen{nullptr};
	for (auto const &head : Heads)
		if (head.kind == "warning" && head.chosen)
			chosen = &head;
	signal_head const *best{nullptr};
	std::tuple<int, std::size_t, int, std::size_t> rank;
	for (auto const &head : Heads)
	{
		if (head.kind != "warning" || head.repeats.empty() || (false == Mount.empty() && head.mount != Mount))
			continue;
		auto const apart{disc_apart(head, classes)};
		if (apart < 0)
			continue;
		std::tuple<int, std::size_t, int, std::size_t> const score{-apart, head.aspect_names.size(), &head == chosen ? 0 : 1, head.file.size()};
		if (best == nullptr || score < rank)
		{
			best = &head;
			rank = score;
		}
	}
	return best != nullptr ? best->file : std::string{};
}

// a shaped warning disc checked by the events it repeats: of the semaphores described after it, whether it tells apart all they
// show, and the template which would
struct disc_check
{
	bool told{false}; // there's a semaphore described after it
	bool unsaid{false}; // its template doesn't say what it repeats: the one described offered, the fitting leaves it be
	bool fits{true};
	std::string fit; // the template which tells all apart, empty for none
};

// Mains: for each route of the disc the semaphore described with its arms it leads to, none for the other routes. File: the template
// of the disc, Template: its head, none when it isn't among the scanned
disc_check check_disc(std::vector<signal_head> const &Heads, std::unordered_map<std::string, std::size_t, string_hash, std::equal_to<>> const &By_file, std::vector<signal_head const *> const &Mains, signal_head const *Template, std::string const &File, std::string const &Mount)
{
	disc_check result;
	auto const described{Template != nullptr && false == Template->repeats.empty()};
	for (auto const *main : Mains)
	{
		if (main == nullptr)
			continue;
		auto fit{disc_template(Heads, main, Mount)};
		if (auto const offered{By_file.find(fit)}; fit == File || offered == By_file.end() || false == disc_tells(Heads[offered->second], *main))
			fit.clear();
		if (false == described)
		{
			// one not described tells nothing to check: the one described is offered as a hint
			if (fit.empty())
				continue;
			result.told = true;
			result.unsaid = true;
			if (result.fit.empty())
				result.fit = fit;
			continue;
		}
		result.told = true;
		if (disc_tells(*Template, *main))
			continue;
		result.fits = false;
		if (result.fit.empty())
			result.fit = fit;
	}
	return result;
}

// the block signals switched by the block sections of the editor, lower case
name_set guarded(std::vector<standing_template> const &Controls)
{
	name_set result;
	for (auto const &section : Controls)
		for (std::size_t index = 5; index < section.values.size() && index < 7; ++index)
			result.insert(ToLower(section.values[index]));
	return result;
}

// a block signal standing by the track of the line facing its trains, or Wrong: its wrong-track ones, Distance: how far out along it
bool faces_along(station::line const &Line, standing_signal const &Signal, double &Distance, bool const Wrong = false)
{
	double gap{0.0};
	Distance = station::distance_on(Line, Signal.position, gap);
	if (gap > 4.5 || Distance < 50.0 || Distance > Line.length() - 1.0)
		return false;
	auto const way{station::point_on(Line, Distance).way};
	return (Line.inbound != Wrong ? -1.0 : 1.0) * glm::dot(travel_of(Signal.yaw), glm::dvec2{way.x, way.z}) >= 0.87;
}

bool block_along(station::line const &Line, standing_signal const &Signal, double &Distance, bool const Wrong = false)
{
	auto const along{faces_along(Line, Signal, Distance, Wrong)};
	return Signal.kind == "block" && along;
}

// a warning disc by the line facing its wrong-track trains, on no other line out of the station: of the semi-automatic block they
// run on, the automatic one of the track the other way
bool wrong_disc(std::vector<station::line> const &Lines, std::size_t const Index, standing_signal const &Signal)
{
	if (double distance{0.0}; Signal.kind != "warning" || false == faces_along(Lines[Index], Signal, distance, true))
		return false;
	for (std::size_t other = 0; other < Lines.size(); ++other)
	{
		double gap{0.0};
		auto const at{station::distance_on(Lines[other], Signal.position, gap)};
		if (other != Index && gap < 4.5 && at > 50.0 && at < Lines[other].length() - 1.0)
			return false;
	}
	return true;
}

// the way the trains of the line run out along it, 1 out, -1 in
double out_of(station::line const &Line)
{
	return Line.inbound ? -1.0 : 1.0;
}

// what the overlay of the scenery draws with: the projection onto the screen, the draw list, the camera
struct overlay_frame
{
	screen_projection const &projection;
	ImDrawList *drawlist{nullptr};
	glm::dvec3 camera{0.0};
	glm::dvec3 lift{0.0, kRailHead, 0.0}; // the rail heads above the axes
	// the point near enough the camera to be drawn, anywhere in the ortho view
	bool near_enough(glm::dvec3 const &Point, double const Range) const { return Global.EditorOrtho || glm::distance(Point, camera) < Range; }
};

void draw_circuit_tracks(overlay_frame const &Frame, station::layout const &Layout, std::vector<int> const &Circuit_of, std::vector<srk_tool::circuit_mark> const &Circuits)
{
	for (std::size_t index = 0; index < Layout.paths.size() && index < Circuit_of.size(); ++index)
	{
		if (Circuit_of[index] < 0)
			continue;
		auto const colour{(Circuits[Circuit_of[index]].colour & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 200)};
		auto const width{Global.EditorOrtho ? 6.0f : 3.0f};
		auto const &points{Layout.paths[index].points};
		for (std::size_t i = 1; i < points.size(); ++i)
			if (Frame.near_enough(points[i], kOverlayRange))
				Frame.projection.line(Frame.drawlist, points[i - 1] + Frame.lift, points[i] + Frame.lift, colour, width);
	}
}

void draw_joints(overlay_frame const &Frame, std::vector<srk_tool::joint_mark> const &Joints)
{
	for (auto const &joint : Joints)
	{
		if (false == Frame.near_enough(joint.at, kJointRange))
			continue;
		auto const centre{joint.at + Frame.lift};
		Frame.projection.line(Frame.drawlist, centre - joint.across * 1.7, centre + joint.across * 1.7, kJoint, 4.0f);
		Frame.projection.line(Frame.drawlist, centre + joint.across * 1.7, centre + joint.across * 1.7 + glm::dvec3{0.0, 1.2, 0.0}, kJoint, 2.0f);
		if (ImVec2 top; Frame.projection.project(centre + joint.across * 1.7 + glm::dvec3{0.0, 1.2, 0.0}, top))
			Frame.drawlist->AddCircleFilled(top, 3.0f, kJoint);
	}
}

void draw_circuit_labels(overlay_frame const &Frame, std::vector<srk_tool::circuit_mark> const &Circuits)
{
	for (auto const &mark : Circuits)
	{
		ImVec2 screen;
		if (false == mark.labelled || false == Frame.near_enough(mark.world, kLabelRange) || false == Frame.projection.project(mark.world + glm::dvec3{0.0, 0.8, 0.0}, screen))
			continue;
		auto const size{ImGui::CalcTextSize(mark.text.c_str())};
		Frame.drawlist->AddRectFilled(ImVec2(screen.x - size.x * 0.5f - 3.0f, screen.y - size.y - 2.0f), ImVec2(screen.x + size.x * 0.5f + 3.0f, screen.y + 2.0f), IM_COL32(0, 0, 0, 160), 3.0f);
		Frame.drawlist->AddText(ImVec2(screen.x - size.x * 0.5f, screen.y - size.y), mark.colour, mark.text.c_str());
	}
}

void draw_switch_names(overlay_frame const &Frame, station::layout const &Layout)
{
	for (auto const *track : Layout.switches)
	{
		auto const start{track->m_paths.front().points[segment_data::point::start] + glm::dvec3{0.0, 1.0, 0.0}};
		ImVec2 screen;
		if (false == Frame.near_enough(start, kLabelRange) || false == Frame.projection.project(start, screen))
			continue;
		auto const &text{track->name()};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		Frame.drawlist->AddRectFilled(ImVec2(screen.x - 3.0f, screen.y - size.y - 4.0f), ImVec2(screen.x + size.x + 3.0f, screen.y), IM_COL32(0, 0, 0, 160), 3.0f);
		Frame.drawlist->AddText(ImVec2(screen.x, screen.y - size.y - 2.0f), kSwitch, text.c_str());
	}
}

// the points beside the stretch of the track, on the right of the trains running along it, above its rails
std::vector<glm::dvec3> beside_of(std::vector<glm::dvec3> const &Points, glm::dvec3 const &Lift)
{
	std::vector<glm::dvec3> beside;
	for (std::size_t i = 0; i < Points.size(); ++i)
	{
		auto const back{i > 0 && i + 1 == Points.size() ? i - 1 : i};
		auto const way{Points[std::min(i + 1, Points.size() - 1)] - Points[back]};
		glm::dvec2 const along{glm::normalize(glm::dvec2{way.x, way.z} + glm::dvec2{1e-12, 0.0})};
		beside.push_back(Points[i] + Lift + glm::dvec3{-along.y, 0.1, along.x} * 0.9);
	}
	return beside;
}

void draw_useful(overlay_frame const &Frame, station::lengths const &Track)
{
	for (auto const &part : Track.ways)
	{
		if (part.length <= 0.0)
			continue;
		auto const beside{beside_of(stretch_of(Track, part.rear.along, part.head.along), Frame.lift)};
		for (std::size_t i = 1; i < beside.size(); ++i)
			if (Frame.near_enough(beside[i], kOverlayRange))
				Frame.projection.line(Frame.drawlist, beside[i - 1], beside[i], kUseful, 3.0f);
		auto const top{beside.back() + glm::dvec3{0.0, 2.5, 0.0}};
		Frame.projection.line(Frame.drawlist, beside.front(), beside.front() + glm::dvec3{0.0, 1.0, 0.0}, kUseful, 2.0f);
		Frame.projection.line(Frame.drawlist, beside.back(), top, kUseful, 2.0f);
		ImVec2 screen;
		if (false == Frame.near_enough(top, kLabelRange) || false == Frame.projection.project(top, screen))
			continue;
		auto const text{format(STR_C("track %s: %.0f m"), Track.number.c_str(), part.length)};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		Frame.drawlist->AddRectFilled(ImVec2(screen.x - size.x * 0.5f - 4.0f, screen.y - size.y - 4.0f), ImVec2(screen.x + size.x * 0.5f + 4.0f, screen.y), IM_COL32(0, 0, 0, 180), 3.0f);
		Frame.drawlist->AddText(ImVec2(screen.x - size.x * 0.5f, screen.y - size.y - 2.0f), kUseful, text.c_str());
	}
}

void draw_boundary(overlay_frame const &Frame, srk_tool::boundary const &Boundary)
{
	auto const colour{Boundary.remove ? IM_COL32(255, 90, 80, 255) : IM_COL32(110, 230, 120, 255)};
	for (auto const &[point, way] : Boundary.joints)
	{
		glm::dvec3 const across{-way.z, 0.0, way.x};
		Frame.projection.line(Frame.drawlist, point + Frame.lift - across * 2.0, point + Frame.lift + across * 2.0, colour, 4.0f);
	}
	for (auto const &post : Boundary.posts)
	{
		auto const base{post.point + Frame.lift + post.side * 2.4};
		Frame.projection.line(Frame.drawlist, post.point + Frame.lift, base, colour, 2.0f);
		Frame.projection.line(Frame.drawlist, base, base + glm::dvec3{0.0, 6.0, 0.0}, colour, 3.0f);
		ImVec2 screen;
		if (false == Frame.projection.project(base + glm::dvec3{0.0, 6.5, 0.0}, screen))
			continue;
		auto const &text{post.name};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		Frame.drawlist->AddRectFilled(ImVec2(screen.x - size.x * 0.5f - 4.0f, screen.y - size.y - 4.0f), ImVec2(screen.x + size.x * 0.5f + 4.0f, screen.y), colour, 3.0f);
		Frame.drawlist->AddText(ImVec2(screen.x - size.x * 0.5f, screen.y - size.y - 2.0f), IM_COL32(0, 0, 0, 255), text.c_str());
	}
}

void draw_signal_marks(overlay_frame const &Frame, std::vector<srk_tool::signal_mark> const &Marks)
{
	for (auto const &mark : Marks)
	{
		ImVec2 screen;
		if (false == Frame.near_enough(mark.head, kLabelRange) || false == Frame.projection.project(mark.head, screen))
			continue;
		auto const text{mark.unfit ? mark.text + " !" : mark.text};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		Frame.drawlist->AddRectFilled(ImVec2(screen.x - size.x * 0.5f - 4.0f, screen.y - size.y - 4.0f), ImVec2(screen.x + size.x * 0.5f + 4.0f, screen.y), mark.unfit ? kUnfit : kSignal, 3.0f);
		Frame.drawlist->AddText(ImVec2(screen.x - size.x * 0.5f, screen.y - size.y - 2.0f), mark.unfit ? IM_COL32(0, 0, 0, 255) : IM_COL32(255, 255, 255, 255), text.c_str());
	}
}

// a pin over the signal of the row under the mouse in the list of the heads
void draw_head_hover(overlay_frame const &Frame, srk_tool::head_check const &Head)
{
	ImVec2 label;
	if (false == Frame.projection.project(Head.position + glm::dvec3{0.0, 7.0, 0.0}, label))
		return;
	ImVec2 const foot{label.x, label.y - ImGui::GetTextLineHeight() - 8.0f};
	ImVec2 const top{label.x, foot.y - 30.0f};
	Frame.drawlist->AddLine(foot, top, kHovered, 3.0f);
	Frame.drawlist->AddCircle(ImVec2(top.x, top.y - 9.0f), 9.0f, kHovered, 0, 3.0f);
}

// a text field of the record of the station, true with the text typed into it
bool record_field(char const *Label, char const *Id, std::string_view const Value, char const *Hint, std::string &Typed)
{
	field_row(Label);
	std::array<char, 128> buffer{};
	Value.copy(buffer.data(), buffer.size() - 1);
	if (false == ImGui::InputTextWithHint(Id, Hint, buffer.data(), buffer.size()))
		return false;
	Typed = std::string{buffer.data()};
	return true;
}

// how many of the signals checked have their resonators, and the ones standing off or nowhere
std::string shp_summary(std::size_t const Found, std::size_t const Needed, std::size_t const Off, std::size_t const Nowhere)
{
	auto text{format(STR_P("%zu of %zu signal has its SHP resonator", "%zu of %zu signals have their SHP resonators", Needed), Found, Needed)};
	if (Off > 0)
		text += ", " + format(STR_P("%zu farther than 5 m from its place", "%zu farther than 5 m from their places", Off), Off);
	if (Nowhere > 0)
		text += ", " + format(STR_P("%zu with the tracks ending short of the place of its resonator", "%zu with the tracks ending short of the places of their resonators", Nowhere), Nowhere);
	return text;
}

// the kind of a switch in the list of the station, its type when the store knows it
std::string switch_type_text(TTrack const &Track)
{
	if (auto const known{switch_store::named(Track)}; known)
		return switch_store::label(*known);
	if (Track.eType == tt_Switch)
		return STR_C("switch");
	if (Track.eType == tt_Cross)
		return STR_C("crossing");
	return STR_C("turntable");
}

// why a line has its block, for the table of the lines
char const *reason_text(srk_tool::block_reason const Why)
{
	using enum srk_tool::block_reason;
	switch (Why)
	{
	case hand: return STR_C("set by hand");
	case standing: return STR_C("block signals stand along it");
	case double_track: return STR_C("a double-track line");
	default: return STR_C("a single-track line");
	}
}

// the block of a line, for the table of the lines
char const *kind_text(srk_tool::block_kind const Kind)
{
	switch (Kind)
	{
	case srk_tool::block_kind::three: return STR_C("SBL, 3 aspects");
	case srk_tool::block_kind::four: return STR_C("SBL, 4 aspects");
	default: return STR_C("semi-automatic");
	}
}

// where a line out of the station leads, for its row
std::string line_end(srk_tool::line_row const &Row)
{
	if (false == Row.next.empty())
		return Row.next;
	if (Row.station)
		return STR_C("station");
	return STR_C("track end");
}

// where a line out of the station leads, for the tooltip of its row
std::string line_end_tooltip(srk_tool::line_row const &Row)
{
	if (false == Row.next.empty())
		return format(STR_C("To the entry signal %s of the next station"), Row.next.c_str());
	if (Row.station)
		return STR_C("To the next station, its entry signal not standing yet");
	return STR_C("To the end of the tracks");
}

// the block shown in the choice of a line
std::string block_preview(srk_tool::line_row const &Row)
{
	if (Row.legacy)
		return STR_C("of the scenery");
	if (Row.why == srk_tool::block_reason::hand)
		return kind_text(Row.block);
	return format(STR_C("%s, auto"), kind_text(Row.block));
}

// the tooltip of the row of a line out of the station
void line_tooltip(srk_tool::line_row const &Row)
{
	ImGui::BeginTooltip();
	ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
	ImGui::TextUnformatted(format(STR_C("The line out beyond the entry signal %s"), Row.entry.c_str()).c_str());
	if (Row.number > 0)
		ImGui::TextUnformatted((format(STR_C("Track %d of a double-track line"), Row.number) + ", " + (Row.inbound ? STR("its trains run in to the station") : STR("its trains run out of the station"))).c_str());
	else
		ImGui::TextUnformatted(STR_C("A single-track line"));
	ImGui::TextUnformatted(format(STR_P("%zu block signal stands along it", "%zu block signals stand along it", Row.blocks), Row.blocks).c_str());
	ImGui::TextUnformatted(format(STR_C("Block: %s, %s"), kind_text(Row.block), reason_text(Row.why)).c_str());
	if (Row.number > 0 && Row.block != srk_tool::block_kind::semi)
		ImGui::TextUnformatted(Row.both ? STR_C("Wrong-track running: SBL both ways, the block signals with N") : STR_C("Wrong-track running: the mixed block, a warning disc before the entry signal"));
	if (Row.legacy)
		ImGui::TextColored(kDoubtful, "%s", STR_C("No block section of the editor switches its block signals: it keeps the block of the scenery as it stands"));
	if (Row.leftovers > 0)
	{
		ImGui::TextColored(kDoubtful, "%s", (Row.block != srk_tool::block_kind::semi ? format(STR_P("%zu warning disc stands along it, of the semi-automatic block: Set the line signals takes it away", "%zu warning discs stand along it, of the semi-automatic block: Set the line signals takes them away", Row.leftovers), Row.leftovers)
		                                                                             : format(STR_P("%zu block signal stands along it, of the automatic block: Set the line signals takes it away", "%zu block signals stand along it, of the automatic block: Set the line signals takes them away", Row.leftovers), Row.leftovers)).c_str());
		ImGui::TextDisabled("%s", STR_C("LMB: fly to the first of them"));
	}
	else
		ImGui::TextDisabled("%s", STR_C("LMB: fly to the entry signal"));
	ImGui::PopTextWrapPos();
	ImGui::EndTooltip();
}

// a note in the colour of the disabled text, wrapped
void dimmed(char const *Text)
{
	ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
	ImGui::TextWrapped("%s", Text);
	ImGui::PopStyleColor();
}

// the tooltip of the row of a station track in the list of the lengths
void length_tooltip(station::lengths const &Track, std::array<char const *, 2> const &Ways, std::string const &Stem)
{
	ImGui::BeginTooltip();
	ImGui::TextUnformatted(format(STR_C("Track %s"), Track.number.c_str()).c_str());
	for (int way = 0; way < 2; ++way)
	{
		auto const &part{Track.ways[way]};
		ImGui::TextUnformatted(format(STR_C("%s: %.0f m, from %s to %s"), Ways[way], part.length, limit_text(part.rear, Stem).c_str(), limit_text(part.head, Stem).c_str()).c_str());
	}
	ImGui::TextDisabled(STR_C("Construction length %.0f m: between the switches at its ends, the switches within it left out"), Track.construction);
	if (std::abs(Track.ways[0].length - Track.ways[1].length) > kUnequal)
		ImGui::TextColored(kDoubtful, "%s", STR_C("The two ways differ: their trains stop on other stretches of the track, between other switches or signals"));
	ImGui::TextDisabled("%s", STR_C("LMB: show it"));
	ImGui::EndTooltip();
}

// the useful length of a station track one way, in its cell of the list
void length_cell(station::stand const &Part, bool const Unequal)
{
	if (Unequal)
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, Part.head.what == station::limit::kind::signal ? 1.0f : 0.6f), "%.0f m", Part.length);
	else if (Part.head.what == station::limit::kind::signal)
		ImGui::Text("%.0f m", Part.length);
	else
		ImGui::TextDisabled("%.0f m", Part.length);
}

// the title of a signal in the tooltip of its row, with a %s for its name
char const *head_title(srk_tool::head_check const &Head)
{
	if (Head.kind == "main")
		return STR_C("Main signal %s");
	if (Head.kind == "block")
		return STR_C("Block signal %s");
	return STR_C("Warning disc %s");
}

// a route of the trains past a signal, in the tooltip of its row
std::string route_text(srk_tool::head_check const &Head, std::size_t const Index, double const Line, std::string const &Stem)
{
	auto const &route{Head.routes[Index]};
	std::string text;
	if (route.next >= 0)
		text = format(STR_C("To %s: %.0f m"), Head.nexts[Index].c_str(), route.length);
	else if (route.ended)
		text = format(STR_C("To the end of the tracks: %.0f m"), route.length);
	else
		text = format(STR_C("No signal within %.0f m"), route.length);
	if (Head.kind == "main")
		text += ", " + (route.speed > 0.0 && shown_speed(route.speed, Line) > 0 ? format(STR_C("%d km/h through the switches"), shown_speed(route.speed, Line)) : format(STR_C("%.0f km/h"), Line));
	std::string turnouts;
	for (auto const *turnout : route.turnouts)
		turnouts += (turnouts.empty() ? "" : ", ") + without_stem(turnout->name(), Stem);
	if (false == turnouts.empty())
		text += ", " + format(STR_P("off the straight at switch %s", "off the straight at switches %s", route.turnouts.size()), turnouts.c_str());
	if (route.warning >= 0)
		text += ", " + STR("past a warning disc");
	return text;
}

// how the template of a signal fits its routes, in the tooltip of its row: the lights, or the aspects of a semaphore or of a
// warning disc described
void head_fit_note(srk_tool::head_check const &Head)
{
	if (Head.arms > 0)
	{
		// a semaphore described with its arms and its aspects
		if (false == Head.fits && Head.fit.empty())
			ImGui::TextColored(kWrong, "%s", STR_C("No semaphore of its mount has these aspects"));
		else if (false == Head.fits)
			ImGui::TextColored(kDoubtful, "%s", format(STR_C("Its arms don't show all of it: %s does"), Head.fit.c_str()).c_str());
		else if (Head.more)
			ImGui::TextDisabled("%s", format(STR_C("It has more than it needs: %s would do. The fitting leaves it be: events written by hand may use its aspects"), Head.fit.c_str()).c_str());
	}
	else if (Head.told)
	{
		// a shaped warning disc before a semaphore described, by the events it repeats; one not described only offered the one
		if (Head.unsaid)
			ImGui::TextDisabled("%s", format(STR_C("Its template doesn't say what it tells, so it isn't checked: %s says so and tells all its semaphore shows. The fitting leaves it be, swap it by hand if you like"), Head.fit.c_str()).c_str());
		else if (false == Head.fits && Head.fit.empty())
			ImGui::TextColored(kWrong, "%s", STR_C("No disc of its mount tells all its semaphore shows"));
		else if (false == Head.fits)
			ImGui::TextColored(kDoubtful, "%s", format(STR_C("It doesn't tell apart all its semaphore shows: %s does"), Head.fit.c_str()).c_str());
	}
	else if (Head.lamps.empty() && Head.kind != "warning")
		ImGui::TextDisabled("%s", STR_C("A semaphore with arms: its lights aren't checked"));
	else if (false == Head.fits && Head.fit.empty())
		ImGui::TextColored(kWrong, "%s", STR_C("No template of its kind and mount has these lights"));
	else if (false == Head.fits)
		ImGui::TextColored(kDoubtful, "%s", format(STR_C("Its lights don't show all of it: %s does"), Head.fit.c_str()).c_str());
	else if (Head.more)
		ImGui::TextDisabled("%s", format(STR_C("It has more lights than it needs: %s will do"), Head.fit.c_str()).c_str());
}

// where the SHP resonator of a signal belongs and where it stands, in the tooltip of its row
void head_shp_note(srk_tool::head_check const &Head)
{
	using enum srk_tool::shp_state;
	auto const state{Head.shp_of()};
	if (state == needless)
		return;
	auto const wanted{Head.shp > 0.0 ? format(STR_C("its SHP resonator belongs %.0f m before it"), Head.shp) : std::string{STR_C("its SHP resonator belongs at it, it tells of the next signal")}};
	if (state == missing || state == beyond)
		ImGui::TextColored(kWrong, "%s", (wanted + ", " + (state == beyond ? STR("the tracks before it end short of it") : STR("none stands"))).c_str());
	else
		ImGui::TextColored(state == off ? kDoubtful : kRight, "%s", (wanted + ", " + format(STR_C("its event %.0f m before it"), Head.shp_found)).c_str());
}

// the tooltip of the row of a signal in the list of the heads
void head_tooltip(srk_tool::head_check const &Head, double const Line, std::string const &Stem)
{
	ImGui::BeginTooltip();
	ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32.0f);
	ImGui::TextUnformatted(format(head_title(Head), Head.name.c_str()).c_str());
	ImGui::TextDisabled("%s", Head.file.c_str());
	for (std::size_t k = 0; k < Head.routes.size(); ++k)
		ImGui::BulletText("%s", route_text(Head, k, Line, Stem).c_str());
	if (Head.kind != "warning")
	{
		head_need need;
		need.speeds = Head.speeds;
		need.informs = Head.informs;
		need.white = Head.white;
		ImGui::TextUnformatted(format(STR_C("Needs: %s"), (Head.arms > 0 ? need_arms_text(need) : need_text(need)).c_str()).c_str());
	}
	if (Head.shortened > 0)
		ImGui::TextColored(kDoubtful, "%s", format(STR_C("The next signal stands nearer than the braking distance: with it at stop this one slows the trains to %d km/h, Ie-4 § 8 ust. 5"), Head.shortened).c_str());
	if (Head.indicator)
		ImGui::TextColored(kDoubtful, "%s", STR_C("Give it a W21 indicator \"14\": 140 km/h with the next signal at stop, Ie-4 § 8 ust. 5"));
	head_fit_note(Head);
	for (auto const &problem : Head.problems)
		ImGui::TextColored(kWrong, "%s", problem.c_str());
	head_shp_note(Head);
	ImGui::TextDisabled("%s", STR_C("LMB: fly to it"));
	ImGui::PopTextWrapPos();
	ImGui::EndTooltip();
}

// the speeds a main signal shows, in its cell of the list of the heads
void head_speeds_cell(srk_tool::head_check const &Head, double const Line)
{
	if (Head.kind != "main")
	{
		ImGui::TextDisabled("-");
		return;
	}
	auto text{format("%.0f", Line)};
	for (auto const speed : std::views::reverse(Head.speeds))
		text += std::format(", {}", speed);
	ImGui::TextUnformatted(text.c_str());
}

// how the template of a signal fits it, in its cell of the list of the heads
void head_state_cell(srk_tool::head_check const &Head)
{
	if (Head.lamps.empty() && Head.arms == 0 && false == Head.told)
		ImGui::TextDisabled("-");
	else if (Head.unsaid)
		ImGui::TextDisabled("%s", STR_C("not described"));
	else if (false == Head.fits)
	{
		auto const *missing{Head.arms > 0 || Head.told ? STR_C("aspects missing") : STR_C("lights missing")};
		ImGui::TextColored(Head.fit.empty() ? kWrong : kDoubtful, "%s", Head.fit.empty() ? STR_C("no template") : missing);
	}
	else if (Head.more)
		ImGui::TextDisabled("%s", STR_C("more than needed"));
	else
		ImGui::TextColored(kRight, "%s", STR_C("fits"));
	if (false == Head.problems.empty())
	{
		ImGui::SameLine();
		ImGui::TextColored(kWrong, "%s", STR_C("too near"));
	}
}

// the SHP resonator of a signal, in its cell of the list of the heads
void head_shp_cell(srk_tool::head_check const &Head)
{
	using enum srk_tool::shp_state;
	switch (Head.shp_of())
	{
	case needless: ImGui::TextDisabled("-"); break;
	case missing: ImGui::TextColored(kWrong, "%s", STR_C("none")); break;
	case beyond: ImGui::TextColored(kWrong, "%s", STR_C("no track")); break;
	case off: ImGui::TextColored(kDoubtful, "%.0f m", Head.shp_found); break;
	default: ImGui::TextColored(kRight, "%s", Head.shp > 0.0 ? format("%.0f m", Head.shp_found).c_str() : STR_C("at it")); break;
	}
}

// of the block boundaries along the line, the one within reach of the distance, the end of them for none
auto near_stop(srk_tool::line_row const &Row, double const Distance)
{
	auto const found{std::ranges::min_element(Row.stops, [Distance](auto const &A, auto const &B) { return std::abs(A.first - Distance) < std::abs(B.first - Distance); })};
	return found != Row.stops.end() && std::abs(found->first - Distance) < kBoundaryReach ? found : Row.stops.end();
}

// m out along the line, the block boundaries before and beyond the distance, the one taken away left out
std::pair<double, double> stops_around(srk_tool::line_row const &Row, double const At, bool const Remove)
{
	auto previous{0.0};
	auto next{Row.length};
	for (auto const &[along, name] : Row.stops)
	{
		if (Remove && std::abs(along - At) < kBoundaryReach)
			continue;
		if (along < At)
			previous = std::max(previous, along);
		else
			next = std::min(next, along);
	}
	return {previous, next};
}

// a block signal of the boundary drawn, on the outside of the track
void post_at(srk_tool::boundary &Boundary, station::line const &Way, double const Distance, std::string const &Name)
{
	auto const on{station::point_on(Way, Distance)};
	glm::dvec3 const travel{on.way * out_of(Way)};
	Boundary.posts.emplace_back(on.point, glm::dvec3{-travel.z, 0.0, travel.x}, Name);
}

// a number standing already gets a letter, as Set the line signals gives it
std::string unused_name(name_set const &Names, std::string const &Base)
{
	auto name{Base};
	for (char letter = 'a'; Names.contains(ToLower(name)) && letter <= 'z'; ++letter)
		name = Base + letter;
	return name;
}

// what a click on the line places, with the lengths of the block sections either side
std::string boundary_label(srk_tool::line_row const &Row, std::string const &Names, double const At, double const Previous, double const Next)
{
	auto const least{Row.block == srk_tool::block_kind::four ? Row.braking * 0.5 : Row.braking};
	auto label{format(STR_C("Click: block signals %s here: the block sections %.2f km and %.2f km"), Names.c_str(), (At - Previous) / 1000.0, (Next - At) / 1000.0)};
	if (std::min(At - Previous, Next - At) < least)
		label += "\n" + format(Row.block == srk_tool::block_kind::four ? STR_C("The block section of %.2f km is shorter than half the braking distance, %.0f m, of the 4-aspect block") : STR_C("The block section of %.2f km is shorter than the braking distance, %.0f m"), std::min(At - Previous, Next - At) / 1000.0, least);
	return label;
}

// the names of the switches of the numbering
using switch_names = std::vector<std::pair<TTrack *, std::string>>;

// the events aiming at the switches listed, the first six of them
std::string listed_of(std::vector<std::string> const &Unwritten)
{
	std::string listed;
	for (std::size_t i = 0; i < Unwritten.size() && i < 6; ++i)
		listed += (i > 0 ? ", " : "") + Unwritten[i];
	if (Unwritten.size() > 6)
		listed += format(" (+%zu)", Unwritten.size() - 6);
	return listed;
}

// the switches given their names; the names go round among them: a switch whose name another one gets gives it up first
void rename_switches(switch_names const &Names)
{
	for (auto const &[track, name] : Names)
		if (track->name() != name && std::ranges::any_of(Names, [&track](auto const &Other) { return Other.first != track && Other.second == track->name(); }))
			editor_track::rename(*track, track->name() + "~");
	for (auto const &[track, name] : Names)
		editor_track::rename(*track, name);
}

// the tracks of the station as captured before a change, the tracks themselves changed after
using track_states = std::vector<std::pair<TTrack *, editor_track::state>>;

// the tracks cut at the joints planned, each from its end back, so the pieces left to cut stay in the track; Points: of all the
// joints
void cut_at_joints(std::vector<station::joint> const &Planned, std::vector<glm::dvec3> &Points, track_states &Before, std::vector<TTrack *> &Created)
{
	std::vector<std::pair<TTrack *, std::vector<glm::dvec3>>> cuts;
	for (auto const &entry : Planned)
	{
		Points.push_back(entry.point);
		if (entry.track == nullptr)
			continue;
		auto found{std::ranges::find_if(cuts, [&entry](auto const &Cut) { return Cut.first == entry.track; })};
		if (found == cuts.end())
			found = cuts.insert(cuts.end(), {entry.track, {}});
		found->second.push_back(entry.point);
	}
	for (auto &[track, at] : cuts)
	{
		Before.emplace_back(track, editor_track::capture(*track));
		TTrack const *const piece{track};
		std::ranges::sort(at, [piece](glm::dvec3 const &A, glm::dvec3 const &B) { return editor_track::nearest_parameter(*piece, A) > editor_track::nearest_parameter(*piece, B); });
		for (auto const &point : at)
			if (auto *cut{editor_track::split_path(*track, editor_track::nearest_parameter(*track, point))})
				Created.push_back(cut);
	}
}

// a station track keeps the name most of it has, given by hand or so far, unless another one has it too; the others take
// the numbers left on their side. Taken: the names kept
std::vector<std::string> kept_names(std::vector<station::circuit> const &Circuits, std::string const &Stem, name_set &Taken)
{
	std::vector<std::string> names(Circuits.size());
	std::map<std::string, std::size_t, std::less<>> held;
	for (std::size_t index = 0; index < Circuits.size(); ++index)
	{
		auto const &circuit{Circuits[index]};
		if (circuit.unit >= 0)
			continue;
		std::map<std::string, std::size_t, std::less<>> counts;
		for (auto const *track : circuit.tracks)
			if (track->Isolated.size() == 1 && false == track_of(track->Isolated.front()->asName, Stem).empty())
				++counts[track->Isolated.front()->asName];
		if (auto const most{std::ranges::max_element(counts, [](auto const &A, auto const &B) { return A.second < B.second; })}; most != counts.end())
		{
			names[index] = most->first;
			++held[names[index]];
		}
	}
	for (auto &name : names)
		if (false == name.empty() && held[name] > 1)
			name.clear();
		else if (false == name.empty())
			Taken.insert(name);
	return names;
}

// the name of a station track after its number, the next one on its side while another track has it
std::string free_circuit_name(std::string Number, std::string const &Stem, name_set &Taken)
{
	while (Taken.contains(Stem + "it" + Number))
	{
		std::size_t digits{0};
		while (digits < Number.size() && std::isdigit(static_cast<unsigned char>(Number[digits])))
			++digits;
		if (digits == 0)
			break;
		Number = std::format("{}{}", std::stoi(Number.substr(0, digits)) + 2, Number.substr(digits));
	}
	auto name{Stem + "it" + Number};
	Taken.insert(name);
	return name;
}

// the tracks of a circuit given its name, the ones not captured yet captured first
void isolate_circuit(station::circuit const &Circuit, std::string const &Name, std::unordered_set<TTrack const *> &Captured, track_states &Before)
{
	for (auto *track : Circuit.tracks)
	{
		if (track->Isolated.size() == 1 && track->Isolated.front()->asName == Name)
			continue;
		if (Captured.insert(track).second)
			Before.emplace_back(track, editor_track::capture(*track));
		editor_track::isolated(*track, {Name});
	}
}

// the circuits of the station as cut named: of the switches after their units, the station tracks after their numbers
void name_circuits(station::layout const &Layout, std::vector<station::unit> const &Units, std::vector<station::circuit> const &Circuits, std::string const &Code, std::vector<TTrack *> const &Created, track_states &Before)
{
	auto const numbers{station::track_numbers(Layout, Circuits)};
	auto const stem{station::prefix(Code)};
	std::unordered_set<TTrack const *> captured(Created.begin(), Created.end());
	for (auto const &[track, state] : Before)
		captured.insert(track);
	name_set taken;
	auto names{kept_names(Circuits, stem, taken)};
	for (std::size_t index = 0; index < Circuits.size(); ++index)
	{
		auto const &circuit{Circuits[index]};
		auto &name{names[index]};
		if (circuit.unit >= 0)
			name = stem + "iz" + unit_number(Units[circuit.unit], stem, circuit.unit);
		else if (name.empty())
			name = free_circuit_name(numbers[index], stem, taken);
		isolate_circuit(circuit, name, captured, Before);
	}
}

// the records of the switches given the numbers of their units; how many were
std::size_t renumber_records(std::vector<station::unit> const &Units)
{
	std::size_t numbered{0};
	for (std::size_t index = 0; index < Units.size(); ++index)
		if (auto const record{switch_store::named(*Units[index].parts.front())}; record && record->number != std::to_string(index + 1))
		{
			switch_store::number(record->name, std::to_string(index + 1));
			++numbered;
		}
	return numbered;
}

// the name of a signal of the station in the scenery, with the code of the station
std::string full_name(std::string const &Stem, station::signal const &Signal)
{
	return Stem + ToLower(Signal.name);
}

// m, how far a standing signal stands from the place of one planned and still counts for it
double match_reach(station::signal::role const Kind)
{
	if (Kind == station::signal::role::entry)
		return 400.0;
	if (Kind == station::signal::role::exit)
		return 60.0;
	return 40.0;
}

// the standing signals by the places of the ones planned, each with how near: of the plan, of the standing ones
std::vector<std::tuple<double, std::size_t, std::size_t>> match_candidates(std::vector<station::signal> const &Plan, std::vector<standing_signal> const &Standing)
{
	std::vector<std::tuple<double, std::size_t, std::size_t>> candidates;
	for (std::size_t i = 0; i < Plan.size(); ++i)
	{
		auto const &planned{Plan[i]};
		glm::dvec2 const travel{glm::normalize(glm::dvec2{planned.travel.x, planned.travel.z} + glm::dvec2{1e-12, 0.0})};
		auto const reach{match_reach(planned.kind)};
		for (std::size_t k = 0; k < Standing.size(); ++k)
		{
			auto const &signal{Standing[k]};
			if ((signal.kind == "shunt") != (planned.kind == station::signal::role::shunting) || (signal.kind != "shunt" && signal.kind != "main") || glm::dot(travel_of(signal.yaw), travel) < 0.87)
				continue;
			glm::dvec2 const offset{signal.position.x - planned.point.x, signal.position.z - planned.point.z};
			auto const along{glm::dot(offset, travel)};
			auto const across{std::abs(offset.x * travel.y - offset.y * travel.x)};
			if (std::abs(along) < reach && across < 3.0)
				candidates.emplace_back(std::abs(along) + across * 10.0, i, k);
		}
	}
	return candidates;
}

// of the plan, the standing signal each one is, -1 for none: by their places, then by their names the ones moved away; Used: the
// standing ones taken so
std::vector<int> match_signals(std::vector<station::signal> const &Plan, std::vector<standing_signal> const &Standing, std::string const &Stem, std::vector<char> &Used)
{
	std::vector match(Plan.size(), -1);
	auto candidates{match_candidates(Plan, Standing)};
	std::ranges::sort(candidates);
	for (auto const &[score, i, k] : candidates)
		if (match[i] < 0 && 0 == Used[k])
		{
			match[i] = static_cast<int>(k);
			Used[k] = 1;
		}
	for (std::size_t i = 0; i < Plan.size(); ++i)
		for (std::size_t k = 0; k < Standing.size() && match[i] < 0; ++k)
			if (0 == Used[k] && ToLower(Standing[k].name) == full_name(Stem, Plan[i]))
			{
				match[i] = static_cast<int>(k);
				Used[k] = 1;
			}
	return match;
}

// the circuit from an entry signal to the station: the line cut by it
struct entry_cut
{
	std::size_t signal{0};
	TTrack *track{nullptr};
	double parameter{0.0};
	glm::dvec3 point{0.0};
	bool outward{false};
	std::vector<TTrack *> named; // between the station and the cut
	std::vector<TTrack *> beyond; // out from the cut to the signal and on
};

// of the tracks of the line in from the entry signal, the one nearest the point, with the parameter and the point on it
std::size_t nearest_of_run(station::signal const &Planned, glm::dvec3 const &At, entry_cut &Entry)
{
	auto best{std::numeric_limits<double>::max()};
	std::size_t nearest{0};
	for (std::size_t k = 0; k < Planned.run.size(); ++k)
	{
		auto const t{editor_track::nearest_parameter(*Planned.run[k].first, At)};
		auto const point{editor_track::point_at(*Planned.run[k].first, t)};
		if (auto const distance{glm::distance(glm::dvec2{point.x, point.z}, glm::dvec2{At.x, At.z})}; distance < best)
		{
			best = distance;
			nearest = k;
			Entry.parameter = t;
			Entry.point = point;
		}
	}
	return nearest;
}

// the cut of the circuit of an entry signal standing at the position: at the joint the trains pass the signal to, in from it
// towards the station; the reason while a track there can't be cut or changed
std::string plan_cut(station::signal const &Planned, glm::dvec3 const &Position, entry_cut &Entry)
{
	auto const run{station::run_line(Planned)};
	double gap{0.0};
	auto const signal_at{station::distance_on(run, Position, gap)};
	auto const at{station::point_on(run, std::max(0.0, signal_at - station::signal_drop)).point};
	auto const nearest{nearest_of_run(Planned, at, Entry)};
	for (std::size_t k = 0; k < nearest; ++k)
		Entry.named.push_back(Planned.run[k].first);
	for (std::size_t k = nearest + 1; k < Planned.run.size(); ++k)
		Entry.beyond.push_back(Planned.run[k].first);
	Entry.track = Planned.run[nearest].first;
	Entry.outward = Planned.run[nearest].second;
	auto const &ends{Entry.track->m_paths.front().points};
	auto const to_start{glm::distance(Entry.point, ends[segment_data::point::start])};
	auto const to_end{glm::distance(Entry.point, ends[segment_data::point::end])};
	std::string reason;
	if (std::min(to_start, to_end) <= 0.3)
	{
		// at an end already
		((to_end < to_start) == Entry.outward ? Entry.named : Entry.beyond).push_back(Entry.track);
		Entry.track = nullptr;
	}
	else if (false == editor_track::can_edit_geometry(*Entry.track, reason))
		return format(STR_C("%s can't be cut: %s"), name_of(*Entry.track).c_str(), reason.c_str());
	for (auto const *part : Entry.named)
		if (false == scene::Layers.editable(part, &reason))
			return format(STR_C("%s can't be changed: %s"), name_of(*part).c_str(), reason.c_str());
	return {};
}

// the circuits from the entry signals to the station: the line cut by each one, all of it checked before any is; the reason while
// a track can't be cut or changed
std::string plan_cuts(std::vector<station::signal> const &Plan, std::vector<int> const &Match, std::vector<standing_signal> const &Standing, std::vector<entry_cut> &Cuts)
{
	for (std::size_t i = 0; i < Plan.size(); ++i)
	{
		auto const &planned{Plan[i]};
		if (planned.kind != station::signal::role::entry || planned.run.empty())
			continue;
		entry_cut entry;
		entry.signal = i;
		if (auto const problem{plan_cut(planned, Match[i] >= 0 ? Standing[Match[i]].position : planned.point, entry)}; false == problem.empty())
			return problem;
		Cuts.push_back(std::move(entry));
	}
	return {};
}

// the kinds of the signals placed, of the plan the ones with no standing signal
std::vector<signal_order> wanted_of(std::vector<station::signal> const &Plan, std::vector<int> const &Match)
{
	std::vector<signal_order> wanted;
	for (std::size_t i = 0; i < Plan.size(); ++i)
		if (Match[i] < 0)
		{
			wanted.emplace_back();
			wanted.back().kind = Plan[i].kind == station::signal::role::shunting ? "shunt" : "main";
		}
	return wanted;
}

// the track of the circuit of an entry signal cut, the signal on the piece the trains pass it on
void split_at_entry(entry_cut &Entry, station::signal &Planned, std::unordered_set<TTrack const *> &Captured, track_states &Before, std::vector<TTrack *> &Created)
{
	Captured.insert(Entry.track);
	Before.emplace_back(Entry.track, editor_track::capture(*Entry.track));
	auto *piece{editor_track::split_path(*Entry.track, Entry.parameter)};
	if (piece == nullptr)
		return;
	Created.push_back(piece);
	Captured.insert(piece);
	Entry.named.push_back(Entry.outward ? Entry.track : piece);
	Entry.beyond.push_back(Entry.outward ? piece : Entry.track);
	if (Planned.track == Entry.track)
		Planned.track = Entry.outward ? piece : Entry.track;
}

// the tracks of the circuit of an entry signal given its name, the ones beyond its joint out of it
void name_entry_parts(entry_cut const &Entry, std::string const &Name, std::unordered_set<TTrack const *> &Captured, track_states &Before)
{
	for (auto *part : Entry.named)
	{
		if (part->Isolated.size() == 1 && part->Isolated.front()->asName == Name)
			continue;
		if (Captured.insert(part).second)
			Before.emplace_back(part, editor_track::capture(*part));
		editor_track::isolated(*part, {Name});
	}
	// out of the circuit beyond its joint, where an earlier one ended at the signal
	for (auto *part : Entry.beyond)
	{
		if (part->Isolated.size() != 1 || ToLower(part->Isolated.front()->asName) != Name)
			continue;
		if (Captured.insert(part).second)
			Before.emplace_back(part, editor_track::capture(*part));
		editor_track::isolated(*part, {});
	}
}

// the circuits of the entry signals cut and named
void cut_entries(std::vector<entry_cut> &Cuts, std::vector<station::signal> &Plan, std::string const &Stem, track_states &Before, std::vector<TTrack *> &Created)
{
	std::unordered_set<TTrack const *> captured;
	for (auto &entry : Cuts)
	{
		auto &planned{Plan[entry.signal]};
		if (entry.track != nullptr)
			split_at_entry(entry, planned, captured, Before, Created);
		name_entry_parts(entry, Stem + "it" + ToLower(planned.name), captured, Before);
	}
}

// the signals of the plan placed where none stands, the standing ones renamed after it; the others keep their names, unless one
// of these takes it
void orders_of(std::vector<station::signal> const &Plan, std::vector<int> const &Match, std::vector<standing_signal> const &Standing, std::vector<char> const &Used, std::string const &Stem, std::vector<signal_order> &Placed, std::vector<signal_rename> &Renamed)
{
	for (std::size_t i = 0; i < Plan.size(); ++i)
	{
		auto const &planned{Plan[i]};
		auto const plate{ToLower(planned.name)};
		if (Match[i] >= 0)
		{
			if (ToLower(Standing[Match[i]].name) != full_name(Stem, planned))
				Renamed.emplace_back(Standing[Match[i]].instance, full_name(Stem, planned), plate);
			continue;
		}
		signal_order order;
		order.track = planned.track;
		order.point = planned.point;
		order.facing = planned.facing;
		order.side = planned.side;
		order.kind = planned.kind == station::signal::role::shunting ? "shunt" : "main";
		order.name = full_name(Stem, planned);
		order.plate = plate;
		Placed.push_back(std::move(order));
	}
	for (std::size_t k = 0; k < Standing.size(); ++k)
		if (0 == Used[k] && std::ranges::any_of(Plan, [&Stem, &Standing, k](station::signal const &Signal) { return full_name(Stem, Signal) == ToLower(Standing[k].name); }))
			Renamed.emplace_back(Standing[k].instance, ToLower(Standing[k].name) + "_old", signal_shown(Standing[k].name, Stem) + "_old");
}

// the posts of the signals standing, then of the main and the shunting ones placed; From: of the posts, the main ones placed
std::vector<station::post> posts_with(std::vector<standing_signal> const &Everywhere, std::vector<signal_order> const &Placed, std::vector<std::size_t> &From)
{
	std::vector<station::post> posts;
	for (auto const &signal : Everywhere)
		if (auto const post{post_of(signal)}; post.track != nullptr)
			posts.push_back(post);
	for (auto const &order : Placed)
	{
		if (order.kind == "main")
			From.push_back(posts.size());
		posts.emplace_back(order.track, order.facing, order.point, order.kind == "shunt" ? station::signal::role::shunting : station::signal::role::entry);
	}
	return posts;
}

// how setting the signals went, the signals placed without the lights they need listed
std::string signals_status(std::size_t const Count, std::size_t const Renamed, std::string const &Error, bool const Touched, std::vector<std::string> const &Unfit)
{
	std::string status;
	if (false == Error.empty())
		status = format(STR_C("The signals couldn't be placed: %s"), Error.c_str()) + (Touched ? ". " + STR("Ctrl+Z takes back the tracks cut") : std::string{});
	else
		status = format(STR_P("%zu signal placed", "%zu signals placed", Count), Count) + ", " + format(STR_P("%zu renamed", "%zu renamed", Renamed), Renamed);
	if (Unfit.empty())
		return status;
	std::string names;
	for (auto const &name : Unfit)
		names += (names.empty() ? "" : ", ") + name;
	status += "\n" + format(STR_C("No template has what %s need, they got the one chosen in Signals: see Signal heads"), names.c_str());
	return status;
}

} // namespace

srk_tool::shp_state srk_tool::head_check::shp_of() const
{
	using enum shp_state;
	if (shp < 0.0)
		return needless;
	if (shp_found < 0.0)
		return shp_short ? beyond : missing;
	return std::abs(shp_found - shp) > kShpTolerance ? off : placed;
}

void srk_tool::load()
{
	if (m_loaded)
		return;
	m_loaded = true;
	m_records.clear();
	if (scene::Layers.empty())
		return;
	for (auto const layer : scene::Layers.marked_layers(kMark))
		for (auto const &text : scene::Layers.marked(layer, kMark))
		{
			std::istringstream words{text};
			std::string word;
			if ((words >> word).fail() || word != "station")
				continue;
			record entry;
			entry.layer = layer;
			words >> entry.code;
			entry.read(words);
			entry.code = shown(entry.code);
			entry.line = shown(entry.line);
			entry.km = shown(entry.km);
			entry.track = simulation::Paths.find(entry.anchor);
			if (false == entry.anchor.empty())
				m_records.push_back(std::move(entry));
		}
}

namespace
{

// the block in the words of a record: sbl4, sbl or sbl3, semi for any other
srk_tool::block_kind block_named(std::string_view const Kind)
{
	using enum srk_tool::block_kind;
	if (Kind == "sbl4")
		return four;
	if (Kind == "sbl" || Kind == "sbl3")
		return three;
	return semi;
}

} // namespace

void srk_tool::record::read(std::istringstream &Words)
{
	std::string word;
	while (Words >> word)
	{
		if (word == "line")
			Words >> line;
		else if (word == "km")
			Words >> km;
		else if (word == "from")
			reversed = (Words >> word) && word == "right";
		else if (word == "braking")
			Words >> braking;
		else if (word == "blocks" && Words >> word)
			for (auto const &[signal, kind] : pairs_of(word))
				blocks[signal] = block_named(kind);
		else if (word == "ways" && Words >> word)
			for (auto const &[signal, both] : pairs_of(word))
				ways[signal] = both == "2";
		else if (word == "anchor")
			Words >> anchor;
		else if (word == "name")
		{
			read_name(Words, name);
			break;
		}
	}
}

std::string srk_tool::record::written() const
{
	auto const braking_text{braking > 0 ? format(" braking %d", braking) : std::string{}};
	std::string blocks_text;
	for (auto const &[signal, kind] : blocks)
	{
		if (false == plain_word(signal))
			continue;
		std::string_view suffix{"=semi"};
		if (kind == block_kind::four)
			suffix = "=sbl4";
		else if (kind == block_kind::three)
			suffix = "=sbl";
		blocks_text += std::format("{}{}{}", blocks_text.empty() ? " blocks " : ",", signal, suffix);
	}
	std::string ways_text;
	for (auto const &[signal, both] : ways)
		if (plain_word(signal))
			ways_text += std::format("{}{}{}", ways_text.empty() ? " ways " : ",", signal, both ? "=2" : "=1");
	return format("station %s line %s km %s from %s anchor %s%s%s%s name %s", word_of(code).c_str(), word_of(line).c_str(), word_of(km).c_str(), reversed ? "right" : "left", anchor_of(*this).c_str(), braking_text.c_str(), blocks_text.c_str(), ways_text.c_str(), utf8_to_win1250(name).c_str());
}

void srk_tool::store()
{
	if (false == m_changed || scene::Layers.empty())
		return;
	m_changed = false;
	std::vector<scene::layer_handle> layers{scene::Layers.marked_layers(kMark)};
	for (auto const &entry : m_records)
		if (std::ranges::find(layers, entry.layer) == layers.end())
			layers.push_back(entry.layer);
	for (auto const layer : layers)
	{
		std::vector<std::string> lines;
		for (auto const &entry : m_records)
		{
			if (entry.layer != layer)
				continue;
			if (lines.empty())
				lines.emplace_back("# stations described in the track editor, the simulation skips these lines");
			lines.push_back(entry.written());
		}
		scene::Layers.mark(layer, kMark, std::move(lines));
	}
}

void srk_tool::open(TTrack &Track)
{
	m_status.clear();
	auto found{station::find(Track)};
	if (false == found.has_value())
	{
		m_status = STR_C("No switch near this track: it isn't a part of a station");
		return;
	}
	m_layout = std::move(found);
	m_plot = station::generalize(*m_layout);
	m_schematic.m_hover = -1;
	m_schematic.m_view_state.exaggeration = 0.0f;
	m_history_revision = m_editor.history.history_revision();
	// the record of the station, by any of its switches
	m_record = -1;
	for (int index = 0; index < static_cast<int>(m_records.size()) && m_record < 0; ++index)
	{
		auto &entry{m_records[index]};
		if (entry.track == nullptr)
			entry.track = simulation::Paths.find(entry.anchor);
		if (std::ranges::find(m_layout->switches, entry.track) != m_layout->switches.end())
			m_record = index;
	}
	m_anchor = m_record >= 0 ? m_records[m_record].track : m_layout->switches.front();
	m_reversed = m_record >= 0 && m_records[m_record].reversed;
	m_schematic.survey();
}

void srk_tool::refresh()
{
	m_history_revision = m_editor.history.history_revision();
	if (m_anchor == nullptr || m_anchor->m_editorremoved)
	{
		m_layout.reset();
		return;
	}
	auto const view{m_schematic.m_view_state};
	auto const status{m_status};
	open(*m_anchor);
	if (m_layout.has_value())
	{
		m_schematic.m_view_state = view;
		m_status = status;
	}
}

srk_tool::record &srk_tool::record_of_open()
{
	if (m_record < 0)
	{
		record entry;
		auto *first{m_layout->switches.front()};
		entry.anchor = first->name();
		entry.track = first;
		// beside the records of the other stations, in the file of the scenery they were written in under another name too
		for (auto const layer : scene::Layers.marked_layers(kMark))
			if (scene::Layers.accepts(layer))
			{
				entry.layer = layer;
				break;
			}
		if (entry.layer == null_handle)
			entry.layer = m_editor.templates.output_layer("srk");
		if (entry.layer == null_handle)
			entry.layer = alignment_store::layer_of(*first);
		m_records.push_back(std::move(entry));
		m_record = static_cast<int>(m_records.size()) - 1;
		m_anchor = first;
	}
	return m_records[m_record];
}

std::string srk_tool::title() const
{
	if (m_record >= 0)
	{
		auto const &entry{m_records[m_record]};
		if (false == entry.name.empty())
			return entry.code.empty() ? entry.name : entry.name + " (" + entry.code + ")";
		if (false == entry.code.empty())
			return entry.code;
	}
	return format(STR_C("Station of %s"), m_anchor != nullptr ? m_anchor->name().c_str() : "");
}

void srk_tool::steps::find_all()
{
	m_found.clear();
	for (auto *track : station::stations())
		m_found.emplace_back(track->name(), station::switches_of(*track).size());
	std::ranges::sort(m_found, [](auto const &A, auto const &B) { return A.second > B.second; });
}

void srk_tool::update()
{
	if (false == m_open)
		return;
	load();
	// the stations follow their anchors renamed
	for (auto &entry : m_records)
		if (auto const name{anchor_of(entry)}; name != entry.anchor)
		{
			entry.anchor = name;
			m_changed = true;
		}
	if ((m_layout.has_value() || m_anchor != nullptr) && m_editor.history.history_revision() != m_history_revision)
		refresh();
	else if (m_layout.has_value() && false == m_signals_known)
		survey_signals();
	if (m_track_lengths.m_length_hover >= 0)
		m_track_lengths.m_length_shown = m_track_lengths.m_length_hover;
	else if (m_track_lengths.m_length_canvas >= 0)
		m_track_lengths.m_length_shown = m_track_lengths.m_length_canvas;
	else
		m_track_lengths.m_length_shown = m_track_lengths.length_of(m_editor.selection.selected_track());
	if (m_track_lengths.m_length_shown >= static_cast<int>(m_track_lengths.m_lengths.size()))
		m_track_lengths.m_length_shown = -1;
	// a track of another station selected opens it
	auto *track{m_editor.selection.selected_track()};
	if (track == nullptr || track == m_picked)
		return;
	m_picked = track;
	if (m_layout.has_value() && std::ranges::any_of(m_layout->paths, [track](station::path const &Path) { return Path.track == track && false == Path.approach; }))
		return;
	open(*track);
}

void srk_tool::forget(scene::basic_node const *Node)
{
	if (Node == m_picked)
		m_picked = nullptr;
	for (auto &entry : m_records)
		if (entry.track == Node)
			entry.track = nullptr;
	if (Node == m_anchor)
	{
		m_anchor = nullptr;
		if (m_layout.has_value())
			if (auto const other{std::ranges::find_if(m_layout->switches, [Node](TTrack const *Track) { return Track != Node; })}; other != m_layout->switches.end())
				m_anchor = *other;
	}
	if (m_layout.has_value() && std::ranges::any_of(m_layout->paths, [Node](station::path const &Path) { return Path.track == Node; }))
	{
		// laid out again from its anchor at the next update
		m_layout.reset();
		m_schematic.m_hover = -1;
		m_history_revision = static_cast<std::size_t>(-1);
	}
}

void srk_tool::draw_overlay() const
{
	if (false == m_open || false == m_layout.has_value())
		return;
	screen_projection const projection;
	overlay_frame const frame{projection, ImGui::GetBackgroundDrawList(ImGui::GetMainViewport()), glm::dvec3{Global.pCamera.Pos}};
	// the track circuits: the tracks in their colours, a bar across them at each joint with a post, the station tracks named
	draw_circuit_tracks(frame, *m_layout, m_schematic.m_circuit_of, m_schematic.m_circuits);
	draw_joints(frame, m_schematic.m_joints);
	draw_circuit_labels(frame, m_schematic.m_circuits);
	if (m_schematic.m_hover >= 0 && m_schematic.m_hover < static_cast<int>(m_layout->paths.size()))
	{
		auto const &points{m_layout->paths[m_schematic.m_hover].points};
		for (std::size_t i = 1; i < points.size(); ++i)
			projection.line(frame.drawlist, points[i - 1], points[i], kHovered, 4.0f);
	}
	draw_switch_names(frame, *m_layout);
	// the useful lengths of the station track shown, beside it on the right of the trains, a post where they stop
	if (m_track_lengths.m_length_shown >= 0 && m_track_lengths.m_length_shown < static_cast<int>(m_track_lengths.m_lengths.size()))
		draw_useful(frame, m_track_lengths.m_lengths[m_track_lengths.m_length_shown]);
	// the block boundary under the mouse while they are drawn: a bar across each track at the joint and a post where each signal goes
	if (drawing() && m_line_survey.m_boundary.valid)
		draw_boundary(frame, m_line_survey.m_boundary);
	draw_signal_marks(frame, m_schematic.m_signal_marks);
	if (m_head_checks.m_head_hover >= 0 && m_head_checks.m_head_hover < static_cast<int>(m_head_checks.m_heads.size()))
		draw_head_hover(frame, m_head_checks.m_heads[m_head_checks.m_head_hover]);
}

void srk_tool::steps::render_found()
{
	if (main_button(STR_C("Find the stations of the scenery"), ImVec2(-1.0f, tall_button_height())))
		find_all();
	item_tooltip("Each group of switches joined by tracks shorter than 1.5 km is a station");
	if (m_found.empty())
		return;
	ImGui::TextDisabled("%s", format(STR_P("%zu station", "%zu stations", m_found.size()), m_found.size()).c_str());
	if (false == ImGui::BeginChild("##srkfound", ImVec2(0.0f, std::min(300.0f, ImGui::GetTextLineHeightWithSpacing() * (static_cast<float>(m_found.size()) + 1.0f))), true))
	{
		ImGui::EndChild();
		return;
	}
	for (std::size_t index = 0; index < m_found.size(); ++index)
	{
		auto const &[name, count]{m_found[index]};
		auto const described{std::ranges::find_if(m_tool.m_records, [&name](record const &Entry) { return anchor_of(Entry) == name; })};
		auto label{format(STR_P("%s: %zu switch", "%s: %zu switches", count), name.c_str(), count)};
		if (described != m_tool.m_records.end() && false == described->name.empty())
			label = described->name + " - " + label;
		if (ImGui::Selectable(std::format("{}##srkfound{}", label, index).c_str(), false))
			if (auto *track{simulation::Paths.find(name)}; track != nullptr)
			{
				m_tool.open(*track);
				m_tool.m_editor.selection.set_current_node(track);
				m_tool.m_picked = track;
				m_tool.m_editor.view.focus_track(*track, 0, track->m_paths.front().points[segment_data::point::start]);
			}
	}
	ImGui::EndChild();
}

void srk_tool::render_options()
{
	if (false == m_layout.has_value())
	{
		ImGui::TextDisabled("%s", STR_C("The signalling of a station: its schematic, the numbers of the switches and the tracks, the track circuits and the signals"));
		if (false == m_status.empty())
			ImGui::TextColored(kDoubtful, "%s", m_status.c_str());
		ImGui::Spacing();
		m_steps.render_found();
		m_editor.signals.render_signal_templates();
		return;
	}
	auto const &layout{*m_layout};
	ImGui::TextUnformatted(title().c_str());
	std::unordered_set<TTrack const *> tracks;
	for (auto const &path : layout.paths)
		if (false == path.approach)
			tracks.insert(path.track);
	metric_tiles({{format("%zu", layout.switches.size()), STR_C("switches"), kSwitch},
	              {format("%zu", tracks.size() - layout.switches.size()), STR_C("tracks"), kPlain},
	              {format("%.0f m", layout.to - layout.from), STR_C("length"), ImGui::GetColorU32(ImGuiCol_Text)}});
	m_line_survey.render_problems();
	if (begin_fields("##srkstation"))
	{
		render_record_fields(layout);
		auto const changed{render_braking()};
		ImGui::EndTable();
		if (changed)
		{
			m_changed = true;
			survey_signals();
		}
	}
	render_steps(layout);
	m_head_checks.render_shp();
	render_track_number();
	if (false == m_status.empty())
		ImGui::TextColored(kDoubtful, "%s", m_status.c_str());
	m_editor.signals.render_signal_templates();
	ImGui::Spacing();
	m_track_lengths.m_length_hover = -1;
	if (ImGui::CollapsingHeader(format("%s (%zu)###srklengths", STR_C("Track lengths"), m_track_lengths.m_lengths.size()).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
		m_track_lengths.render_lengths();
	ImGui::Spacing();
	m_head_checks.m_head_hover = -1;
	if (ImGui::CollapsingHeader(format("%s (%zu)###srkheads", STR_C("Signal heads"), static_cast<std::size_t>(std::ranges::count_if(m_head_checks.m_heads, [](head_check const &Head) { return Head.station; }))).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
		m_head_checks.render_heads();
	ImGui::Spacing();
	if (ImGui::CollapsingHeader(format("%s (%zu)###srkswitches", STR_C("Switches"), layout.switches.size()).c_str(), ImGuiTreeNodeFlags_DefaultOpen) &&
	    ImGui::BeginTable("##srkswitchlist", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY, ImVec2(0.0f, std::min(320.0f, ImGui::GetTextLineHeightWithSpacing() * (static_cast<float>(layout.switches.size()) + 1.5f)))))
		render_switch_list(layout);
	ImGui::Spacing();
	if (ImGui::Button(STR_C("Another station"), ImVec2(-1.0f, 0.0f)))
	{
		m_layout.reset();
		m_anchor = nullptr;
		m_record = -1;
		m_schematic.m_hover = -1;
	}
	item_tooltip("Leaves this station; LMB on a switch opens another one");
}

void srk_tool::render_record_fields(station::layout const &Layout)
{
	auto const *entry{m_record >= 0 ? &m_records[m_record] : nullptr};
	auto const of = [entry](std::string record::*Field) { return entry != nullptr ? std::string_view{entry->*Field} : std::string_view{}; };
	if (std::string typed; record_field("Name", "##srkname", of(&record::name), "Biała Podlaska", typed))
	{
		record_of_open().name = typed;
		m_changed = true;
	}
	if (std::string typed; record_field("Code", "##srkcode", of(&record::code), "BPd", typed))
	{
		record_of_open().code = shown(word_of(typed));
		m_changed = true;
	}
	item_tooltip("The abbreviation of the station: the names of its switches, track circuits and signals begin with it");
	auto const line_hint{line_of_open()};
	if (std::string typed; record_field("Line", "##srkline", of(&record::line), line_hint.empty() ? "2" : line_hint.c_str(), typed))
	{
		record_of_open().line = shown(word_of(typed));
		m_changed = true;
	}
	item_tooltip("The number of the railway line the station lies on, as the block sections between the stations are named after it");
	if (double known{0.0}; km_of_line((Layout.from + Layout.to) * 0.5, known))
	{
		field_row("Km");
		ImGui::TextDisabled("%s", format(STR_C("%s, of the kilometrage of line %s"), kilometrage::text(known).c_str(), line_of_open().c_str()).c_str());
		item_tooltip("The km of the middle of the station, as the Kilometrage tool sets them for the line, the jumps too: the block signals of the lines are numbered after them");
	}
	else
	{
		auto const hint{Layout.kilometrage && km_of_open((Layout.from + Layout.to) * 0.5, known) ? format("%.3f", known) : std::string{"189.563"}};
		if (std::string typed; record_field("Km", "##srkkm", of(&record::km), hint.c_str(), typed))
		{
			auto km{word_of(typed)};
			std::ranges::replace(km, ',', '.');
			record_of_open().km = shown(km);
			m_changed = true;
		}
		item_tooltip("The km of the middle of the station along the line, rising the way the switches are numbered: the block signals of the lines are numbered after the km");
	}
}

bool srk_tool::render_braking()
{
	auto changed{false};
	field_row("Line speed");
	auto const set{set_speed_of_open()};
	if (set > 0.0)
		ImGui::TextWrapped("%s", format(STR_C("%.0f km/h, the highest set on its main tracks"), set).c_str());
	else
		ImGui::TextColored(kWrong, "%s", format(STR_C("%.0f km/h taken: no track of the station has a speed set"), speed_of_open()).c_str());
	item_tooltip("The speed of the trains through the station, the highest one its main tracks and the lines out of it are set to in the Track tool, up to 250 km/h: the braking distance and the lights of the signals go by it. Without one 160 km/h is taken, or the speed of the braking distance set by hand");
	field_row("Braking distance");
	auto const by_hand{m_record >= 0 ? m_records[m_record].braking : 0};
	auto const after{format(STR_C("%.0f m, after the speed"), braking_distance(set > 0.0 ? set : 160.0))};
	ImGui::SetNextItemWidth(-1.0f);
	if (ImGui::BeginCombo("##srkbrakingset", by_hand > 0 ? format(STR_C("%d m, set by hand"), by_hand).c_str() : after.c_str()))
	{
		if (ImGui::Selectable(after.c_str(), by_hand == 0))
		{
			record_of_open().braking = 0;
			changed = true;
		}
		for (auto const distance : {400, 700, 1000, 1300})
			if (ImGui::Selectable(format(STR_C("%d m, set by hand"), distance).c_str(), by_hand == distance))
			{
				record_of_open().braking = distance;
				changed = true;
			}
		ImGui::EndCombo();
	}
	item_tooltip("The braking distance of the trains on the line, Ie-4 § 8: 400 m up to 60 km/h, 700 m up to 100, 1000 m up to 140, 1300 m above. The block sections and the warning discs stand no nearer, a signal telling of the next one standing nearer slows the trains. Set it by hand where the line has another one");
	return changed;
}

void srk_tool::render_steps(station::layout const &Layout)
{
	step_heading(1, STR_C("Switch numbers"));
	if (int from{m_reversed ? 1 : 0}; segmented("##srkfrom", from,
	                                            {{0, STR_C("From the left"), Layout.kilometrage ? "From the lower km, as PKP PLK numbers them" : "From the west end of the station"},
	                                             {1, STR_C("From the right"), Layout.kilometrage ? "From the higher km" : "From the east end of the station"}}))
	{
		m_reversed = from == 1;
		record_of_open().reversed = m_reversed;
		m_changed = true;
	}
	if (ImGui::Button(STR_C("Number the switches"), ImVec2(-1.0f, 0.0f)))
		m_steps.number_switches();
	item_tooltip("Numbers the switches 1, 2, 3... along the station and names them after its code, as on the lines of PKP PLK: bpd_1, "
	             "the parts of a slip bpd_15a to bpd_15d. The drives and the objects naming a switch follow it, events written by hand keep the old names. One step of undo");
	step_heading(2, STR_C("Track circuits"));
	if (main_button(STR_C("Set the track circuits"), ImVec2(-1.0f, tall_button_height())))
		m_steps.set_circuits();
	item_tooltip("Cuts the tracks of the station at the joints of the track circuits and names the circuits, as on the lines of PKP PLK: a "
	             "joint 1.5 m before the start of a switch, 4.2 m beyond the fouling point of each branch, where the axes are 3.5 m apart, as "
	             "Ie-4 gives it, in the middle of a link too short for both. Each switch gets bpd_iz<number>, each station track "
	             "bpd_it<number>: 1 and 2 the main tracks, track 1 on the right along the km, the odd ones beyond it, the even ones on the "
	             "other side. Run again after changes, it keeps the numbers the tracks have. One step of undo");
	step_heading(3, STR_C("Signals"));
	if (ImGui::Button(STR_C("Set the signals"), ImVec2(-1.0f, 0.0f)))
		m_steps.set_signals();
	item_tooltip("Places the signals of the station, as on the lines of PKP PLK, and names them after its code: an entry signal 300 m out on "
	             "each line, A, B... from the start of the km, Z, Y... from the end, the circuit from 15 m beyond it to the station bpd_ita; "
	             "an exit signal 15 m before the joint at the end of each station track a train is sent out of, F1, G2 towards the start, R1, "
	             "P2 towards the end; a shunting signal Tm1, Tm2... at the end of a shorter track. A signal standing by the place of one gets "
	             "its name, none is doubled. Each kind with its template chosen in Signals below. One step of undo");
	step_heading(4, STR_C("Lines"));
	m_line_survey.render_lines();
	if (ImGui::Button(STR_C("Set the line signals"), ImVec2(-1.0f, 0.0f)))
		m_line_survey.set_lines();
	item_tooltip("Places the signals of the lines out of the station, from its entry signals on, by the block chosen for each line above. "
	             "The automatic block: block signals evenly to the entry signals of the next station, a section no shorter than the braking "
	             "distance, numbered after the hectometre ahead of them, odd on track 1, even on track 2, and the block sections from 15 m "
	             "beyond them named as on the lines of PKP PLK, 2411-2571@LK2.1. The semi-automatic block: a warning disc ToA the braking "
	             "distance before the entry signal. The braking distance after the speed of the line, as Ie-4 gives it: 400 m up to 60 km/h, "
	             "700 m up to 100, 1000 m up to 140, 1300 m above. Needs the line and the km of the station. Run it at the next station too, it "
	             "keeps what stands. One step of undo");
}

void srk_tool::head_checks::render_shp()
{
	step_heading(5, STR_C("SHP"));
	std::array<std::size_t, 5> states{};
	for (auto const &head : m_heads)
		++states[static_cast<std::size_t>(head.shp_of())];
	auto const of = [&states](shp_state const State) { return states[static_cast<std::size_t>(State)]; };
	auto const needed{m_heads.size() - of(shp_state::needless)};
	auto const found{of(shp_state::off) + of(shp_state::placed)};
	auto const off{of(shp_state::off)};
	auto const nowhere{of(shp_state::beyond)};
	ImGui::BeginDisabled(of(shp_state::missing) == 0);
	if (ImGui::Button(format("%s (%zu)###srkshp", STR_C("Place the SHP resonators"), of(shp_state::missing)).c_str(), ImVec2(-1.0f, 0.0f)))
		set_shp();
	ImGui::EndDisabled();
	item_tooltip("Places the SHP track resonators the signals checked lack, rezonator.inc, as PKP PLK places them: 200 m before an entry "
	             "signal, a block signal and a warning disc, at an exit signal telling of the next one, as one onto a line of the automatic "
	             "block, 5 m either way. Its event goes on the track the trains run into there, at a joint of the tracks within 5 m, else the "
	             "track is cut. A resonator standing is one whose event ends with _shp, put on a track the trains run into within 100 m of its "
	             "place, so the ones of the old sceneries count too. One step of undo");
	if (m_heads_known && needed > 0)
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextWrapped("%s", shp_summary(found, needed, off, nowhere).c_str());
		ImGui::PopStyleColor();
	}
}

void srk_tool::render_track_number()
{
	auto const *selected{m_editor.selection.selected_track()};
	if (selected == nullptr || selected->Isolated.empty())
		return;
	auto const number{track_of(selected->Isolated.front()->asName, stem_of_open())};
	if (number.empty() || false == begin_fields("##srktrack"))
		return;
	field_row("Track number");
	std::array<char, 16> buffer{};
	number.copy(buffer.data(), buffer.size() - 1);
	if (ImGui::InputText("##srktracknumber", buffer.data(), buffer.size(), ImGuiInputTextFlags_EnterReturnsTrue) || ImGui::IsItemDeactivatedAfterEdit())
		m_steps.renumber_track(*selected, buffer.data());
	item_tooltip("The number of the selected station track: type another one and its track circuits are renamed, bpd_it7a, bpd_it7b. One step of undo");
	ImGui::EndTable();
}

void srk_tool::render_switch_list(station::layout const &Layout)
{
	ImGui::TableSetupScrollFreeze(0, 1);
	ImGui::TableSetupColumn(STR_C("Switch"), ImGuiTableColumnFlags_WidthStretch, 0.35f);
	ImGui::TableSetupColumn(STR_C("Type"), ImGuiTableColumnFlags_WidthStretch, 0.45f);
	ImGui::TableSetupColumn(STR_C("km"), ImGuiTableColumnFlags_WidthStretch, 0.2f);
	ImGui::TableHeadersRow();
	auto const *selected{m_editor.selection.selected_track()};
	for (std::size_t index = 0; index < Layout.switches.size(); ++index)
	{
		auto *track{Layout.switches[index]};
		auto const start{track->m_paths.front().points[segment_data::point::start]};
		ImGui::TableNextRow();
		ImGui::TableNextColumn();
		if (ImGui::Selectable(std::format("{}##srksw{}", track->name(), index).c_str(), track == selected, ImGuiSelectableFlags_SpanAllColumns))
		{
			m_editor.selection.set_current_node(track);
			m_picked = track;
			m_editor.view.focus_track(*track, 0, start);
		}
		ImGui::TableNextColumn();
		ImGui::TextUnformatted(switch_type_text(*track).c_str());
		ImGui::TableNextColumn();
		auto const along{station::project(Layout, start).x};
		if (Layout.kilometrage)
			ImGui::Text("%.3f", station::km_of(Layout, along));
		else
			ImGui::Text("%.0f m", along);
	}
	ImGui::EndTable();
}

void srk_tool::line_survey::render_lines()
{
	if (m_lines.empty())
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextWrapped("%s", m_lines_problem.empty() ? STR_C("No line leads out of the station beyond its entry signals") : m_lines_problem.c_str());
		ImGui::PopStyleColor();
		return;
	}
	auto changed{false};
	if (ImGui::BeginTable("##srklinelist", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
	{
		ImGui::TableSetupColumn(STR_C("Line"), ImGuiTableColumnFlags_WidthStretch, 0.26f);
		ImGui::TableSetupColumn(STR_C("To"), ImGuiTableColumnFlags_WidthStretch, 0.29f);
		ImGui::TableSetupColumn(STR_C("Block"), ImGuiTableColumnFlags_WidthStretch, 0.45f);
		ImGui::TableHeadersRow();
		for (std::size_t index = 0; index < m_lines.size() && false == changed; ++index)
			changed = render_line_row(index);
		ImGui::EndTable();
	}
	if (render_wrong_track(changed))
		changed = true;
	if (changed)
		m_tool.survey_signals();
	ImGui::Checkbox(STR_C("Draw the block boundaries"), &m_drawing);
	item_tooltip("LMB on a track of a line of the automatic block, beyond the entry signal: a block boundary there, the tracks cut "
	             "across the line, block signals 15 m before the joint, numbered after the hectometre ahead of them, on a line both ways "
	             "the wrong-track ones 15 m beyond it, with N, and the new block sections named; LMB on a block boundary standing takes its "
	             "signals away, the sections either side joined. A line whose block signals stand keeps them: Set the line signals places "
	             "them evenly only on a line without any. One step of undo each");
	if (m_drawing)
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextWrapped("%s", STR_C("LMB on a track of a line beyond the entry signal: a block boundary there; on one standing: taken away"));
		ImGui::PopStyleColor();
	}
}

void srk_tool::line_survey::set_block(line_row const &Row, std::optional<block_kind> const Kind)
{
	auto &entry{m_tool.record_of_open()};
	for (auto const *row : std::initializer_list<line_row const *>{&Row, Row.partner >= 0 && Row.partner < static_cast<int>(m_lines.size()) ? &m_lines[Row.partner] : nullptr})
	{
		if (row == nullptr)
			continue;
		if (Kind.has_value())
			entry.blocks[row->entry] = *Kind;
		else
			entry.blocks.erase(row->entry);
	}
	m_tool.m_changed = true;
}

void srk_tool::line_survey::set_ways(line_row const &Row, std::optional<bool> const Both)
{
	auto &entry{m_tool.record_of_open()};
	if (Both.has_value())
		entry.ways[Row.entry] = *Both;
	else
		entry.ways.erase(Row.entry);
	m_tool.m_changed = true;
}

bool srk_tool::line_survey::render_line_row(std::size_t const Index)
{
	auto const &row{m_lines[Index]};
	ImGui::PushID(static_cast<int>(Index));
	ImGui::TableNextRow();
	ImGui::TableNextColumn();
	auto const track{row.number == 0 ? std::string{STR_C("single track")} : format(STR_C("track %d"), row.number)};
	if (row.leftovers > 0)
		ImGui::PushStyleColor(ImGuiCol_Text, kDoubtful);
	auto const clicked{ImGui::Selectable(format("%s, %s##srkline", row.entry.c_str(), track.c_str()).c_str(), false, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap)};
	if (row.leftovers > 0)
		ImGui::PopStyleColor();
	if (clicked)
	{
		auto const &position{row.leftovers > 0 ? row.leftover : row.position};
		auto const &travel{row.leftovers > 0 ? row.leftover_travel : row.travel};
		m_tool.m_editor.view.fly_to(position + glm::dvec3{0.0, 4.0, 0.0}, glm::normalize(glm::dvec3{travel.x, -0.3, travel.y}), 30.0);
	}
	if (ImGui::IsItemHovered())
		line_tooltip(row);
	ImGui::TableNextColumn();
	ImGui::TextUnformatted(format("%s %.1f km", line_end(row).c_str(), row.length / 1000.0).c_str());
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", line_end_tooltip(row).c_str());
	ImGui::TableNextColumn();
	ImGui::SetNextItemWidth(-1.0f);
	auto const preview{block_preview(row)};
	ImGui::BeginDisabled(row.legacy);
	auto changed{false};
	if (ImGui::BeginCombo("##srkblock", preview.c_str()))
	{
		changed = render_block_choices(row);
		ImGui::EndCombo();
	}
	ImGui::EndDisabled();
	item_tooltip(row.legacy ? "Block signals stand along the line, none of them switched by a block section of the editor: the line keeps the block of the scenery as it stands" :
	                          "The block of the line, both tracks of a double-track one alike. Automatic (SBL): block signals with block sections, no "
	                          "warning disc, of 3 or 4 aspects, the templates of the block signals standing swapped for those of the aspects. "
	                          "Semi-automatic: a warning disc ToA the braking distance before the entry signal. By itself: the automatic one where "
	                          "block signals stand along the line, of their aspects, or it's double-track, of the aspects of the block signal "
	                          "chosen in Signals below, else the semi-automatic one");
	ImGui::PopID();
	return changed;
}

bool srk_tool::line_survey::render_block_choices(line_row const &Row)
{
	using enum srk_tool::block_reason;
	using enum srk_tool::block_kind;
	auto changed{false};
	if (ImGui::Selectable(format(STR_C("Auto: %s, %s"), kind_text(Row.own), reason_text(Row.own_why)).c_str(), Row.why != hand))
	{
		set_block(Row, std::nullopt);
		changed = true;
	}
	for (auto const automatic : {three, four})
	{
		ImGui::BeginDisabled(Row.number == 0);
		if (ImGui::Selectable(automatic == three ? STR_C("Automatic block, 3 aspects (SBL)") : STR_C("Automatic block, 4 aspects (SBL)"), Row.why == hand && Row.block == automatic))
		{
			set_block(Row, automatic);
			changed = true;
		}
		ImGui::EndDisabled();
		if (Row.number == 0)
			item_tooltip("Placed on a double-track line only: on a single track the block signals would face both ways, the editor leaves those standing as they are");
		else
			item_tooltip(automatic == three ? "A block signal tells of the next one, Ie-1: red, orange for the next at stop, green. The block sections no shorter than the braking distance" :
			                                              "A block signal tells of the next two, Ie-1: red, orange for the next at stop, green flashing for the one after it at stop, green. The block sections no shorter than half the braking distance, two of them no shorter than it");
	}
	if (ImGui::Selectable(STR_C("Semi-automatic block"), Row.why == hand && Row.block == semi))
	{
		set_block(Row, semi);
		changed = true;
	}
	return changed;
}

bool srk_tool::line_survey::render_wrong_track(bool const Changed)
{
	// the wrong-track trains on the automatic block too, or on the semi-automatic one: the mixed block, the double-track lines of the
	// automatic block of the station alike
	std::vector<line_row const *> two_track;
	for (auto const &row : m_lines)
		if (row.number > 0 && row.block != block_kind::semi && false == row.legacy)
			two_track.push_back(&row);
	if (two_track.empty())
		return false;
	auto const all = [&two_track](bool const Both) { return std::ranges::all_of(two_track, [Both](line_row const *Row) { return Row->both == Both; }); };
	int ways{0};
	if (all(true))
		ways = 2;
	else if (all(false))
		ways = 1;
	ImGui::TextDisabled("%s", STR_C("Wrong-track running"));
	ImGui::BeginDisabled(Changed);
	auto changed{false};
	if (segmented("##srkways", ways,
	              {{2, STR_C("SBL both ways"), "At each block boundary the block signals of both tracks either way: of the trains of the track 15 m "
	                                         "before the joint, for the wrong-track ones 15 m beyond it, with N (523N), on the outside of the line, as "
	                                         "on the lines of PKP PLK"},
	               {1, STR_C("Mixed"), "The automatic block for the trains of the track, the semi-automatic one for the wrong-track ones: no N "
	                                   "signals, a warning disc ToB the braking distance before the entry signal of the wrong-track trains, on the "
	                                   "outside of the line. Set the line signals places it and takes the N signals away"}}))
	{
		for (auto const *row : two_track)
			set_ways(*row, (ways == 2) == row->own_both ? std::nullopt : std::optional<bool>{ways == 2});
		changed = true;
	}
	ImGui::EndDisabled();
	if (ways == 0)
		ImGui::TextDisabled("%s", STR_C("Each line as it stands: a choice sets all of them"));
	return changed;
}

std::string srk_tool::line_survey::boundary_hover(glm::dvec3 const &Point, bool &Valid)
{
	m_boundary = {};
	Valid = false;
	if (false == m_tool.drawing())
		return {};
	double at{0.0};
	auto const best{nearest_line(Point, at)};
	if (best < 0)
		return STR("Drawing the block boundaries: LMB on a track of a line out of the station, beyond its entry signal");
	auto const &row{m_lines[best]};
	auto const &line{m_line_ways[best]};
	if (row.legacy)
		return format(STR_C("No block section of the editor switches the block signals beyond %s: the line keeps them as they stand"), row.entry.c_str());
	if (row.block == block_kind::semi)
		return format(STR_C("The line beyond %s has the semi-automatic block: choose the automatic one for it in the table of the lines"), row.entry.c_str());
	if (row.number == 0)
		return format(STR_C("The line beyond %s is single-track: the editor draws the block boundaries of double-track lines only"), row.entry.c_str());
	if (at < kBoundaryEnd || at > row.length - kBoundaryEnd)
		return format(STR_C("A block boundary stands %.0f m or more from the entry signals"), kBoundaryEnd);
	m_boundary.entry = row.entry;
	m_boundary.point = station::point_on(line, at).point;
	auto const tracks{boundary_tracks(static_cast<std::size_t>(best), at)};
	m_boundary.remove = near_stop(row, at) != row.stops.end();
	auto const [previous, next]{stops_around(row, at, m_boundary.remove)};
	for (auto const &[index, distance] : tracks)
		boundary_posts(index, distance);
	std::string names;
	for (auto const &post : m_boundary.posts)
		names += (names.empty() ? "" : ", ") + post.name;
	m_boundary.valid = true;
	Valid = true;
	if (m_boundary.remove)
		return format(STR_C("Click: take the block signals %s away, the block sections either side joined: %.2f km"), names.c_str(), (next - previous) / 1000.0);
	return boundary_label(row, names, at, previous, next);
}

int srk_tool::line_survey::nearest_line(glm::dvec3 const &Point, double &At) const
{
	auto best{-1};
	auto nearest{3.0};
	for (std::size_t index = 0; index < m_lines.size() && index < m_line_ways.size(); ++index)
	{
		double gap{0.0};
		auto const distance{station::distance_on(m_line_ways[index], Point, gap)};
		if (gap < nearest && distance > 0.0 && distance < m_line_ways[index].length())
		{
			nearest = gap;
			best = static_cast<int>(index);
			At = distance;
		}
	}
	return best;
}

std::vector<std::pair<std::size_t, double>> srk_tool::line_survey::boundary_tracks(std::size_t const Best, double const At) const
{
	// the other track across, the block signals standing within reach on both
	auto const &row{m_lines[Best]};
	std::vector<std::pair<std::size_t, double>> tracks{{Best, At}};
	if (row.partner < 0 || row.partner >= static_cast<int>(std::min(m_lines.size(), m_line_ways.size())) || m_lines[row.partner].legacy || m_lines[row.partner].block == block_kind::semi)
		return tracks;
	double gap{0.0};
	if (auto const across{station::distance_on(m_line_ways[row.partner], m_boundary.point, gap)}; gap < 12.0)
		tracks.emplace_back(static_cast<std::size_t>(row.partner), across);
	return tracks;
}

void srk_tool::line_survey::boundary_posts(std::size_t const Index, double const Distance)
{
	// the joint across each track, the block signal of its trains before it, the wrong-track one beyond it, both on the outside
	auto const &other{m_lines[Index]};
	auto const &way{m_line_ways[Index]};
	auto const drop{station::signal_drop * out_of(way)};
	auto joint{Distance};
	if (m_boundary.remove)
	{
		auto const stop{near_stop(other, Distance)};
		if (stop == other.stops.end())
			return;
		joint = stop->first;
		post_at(m_boundary, way, joint - drop, stop->second);
		auto const wrong{std::ranges::min_element(other.wrong, [joint, drop](auto const &A, auto const &B) { return std::abs(A.first - joint - drop) < std::abs(B.first - joint - drop); })};
		if (wrong != other.wrong.end() && std::abs(wrong->first - joint - drop) < kWrongTrack)
			post_at(m_boundary, way, wrong->first, wrong->second);
	}
	else
	{
		auto const name{unused_name(m_signal_names, station::block_number(km_along(way, other.km_number, joint - drop), way.number, other.rising))};
		post_at(m_boundary, way, joint - drop, signal_shown(name, {}));
		if (other.both)
			post_at(m_boundary, way, joint + drop, signal_shown(unused_name(m_signal_names, name + "N"), {}));
	}
	auto const on{station::point_on(way, joint)};
	m_boundary.joints.emplace_back(on.point, on.way);
}

void srk_tool::line_survey::boundary_click()
{
	if (false == m_tool.drawing() || false == m_boundary.valid)
		return;
	block_edit const edit{m_boundary.entry, m_boundary.point, m_boundary.remove};
	m_boundary = {};
	set_lines(&edit);
}

void srk_tool::track_lengths::render_lengths()
{
	using enum station::limit::kind;
	if (m_lengths.empty())
	{
		if (m_tool.stem_of_open().empty())
			dimmed(STR_C("Give the station its code first: its tracks are numbered and measured after it"));
		else if (m_numbered)
			dimmed(STR_C("The station tracks have no ends to be measured between"));
		else
		{
			dimmed(STR_C("Set the track circuits first: the station tracks are measured by their numbers, from their joints, the fouling points and the signals"));
			if (ImGui::Button((std::string{STR_C("Set the track circuits")} + "##srklengthcircuits").c_str(), ImVec2(-1.0f, 0.0f)))
				m_tool.m_steps.set_circuits();
		}
		return;
	}
	auto const stem{m_tool.stem_of_open()};
	std::array<char const *, 2> const ways{STR_C("To the right"), STR_C("To the left")};
	if (ImGui::BeginTable("##srklengthlist", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY, ImVec2(0.0f, std::min(320.0f, ImGui::GetTextLineHeightWithSpacing() * (static_cast<float>(m_lengths.size()) + 1.5f)))))
	{
		ImGui::TableSetupScrollFreeze(0, 1);
		ImGui::TableSetupColumn(STR_C("Track"), ImGuiTableColumnFlags_WidthStretch, 0.16f);
		ImGui::TableSetupColumn(ways[0], ImGuiTableColumnFlags_WidthStretch, 0.28f);
		ImGui::TableSetupColumn(ways[1], ImGuiTableColumnFlags_WidthStretch, 0.28f);
		ImGui::TableSetupColumn(STR_C("Train up to"), ImGuiTableColumnFlags_WidthStretch, 0.28f);
		ImGui::TableHeadersRow();
		for (auto const index : std::views::iota(std::size_t{0}, m_lengths.size()))
			render_length_row(index, ways, stem);
		ImGui::EndTable();
	}
	if (std::ranges::none_of(m_lengths, [](station::lengths const &Track) { return Track.ways[0].head.what == station::limit::kind::signal || Track.ways[1].head.what == station::limit::kind::signal; }))
	{
		dimmed(STR_C("No signal stands by the station tracks yet, the lengths run between the fouling points: set the signals for the useful ones"));
		if (ImGui::Button((std::string{STR_C("Set the signals")} + "##srklengthsignals").c_str(), ImVec2(-1.0f, 0.0f)))
			m_tool.m_steps.set_signals();
	}
	if (ImGui::Button(STR_C("Copy the list of tracks"), ImVec2(-1.0f, 0.0f)))
	{
		ImGui::SetClipboardText(lengths_table().c_str());
		m_tool.m_status = format(STR_P("The lengths of %zu track copied", "The lengths of %zu tracks copied", m_lengths.size()), m_lengths.size());
	}
	item_tooltip("The lengths of the station tracks as a table to paste into a spreadsheet: the construction length and the useful one each way with its limits, as the list of the tracks of a station gives them");
	dimmed(STR_C("The useful length: where a train stands, from the signal it stops by back to the joint, the fouling point or the level crossing behind it, between those where no signal stands (dimmed). The longest train leaves 25 m of it: 10 m to stop, 10 m to see the signal, 5 m to the joint"));
}

void srk_tool::head_checks::render_head_aspects(head_check const &Head)
{
	if (Head.arms > 0 || (Head.lamps.empty() && false == Head.aspect_names.empty()))
	{
		// a semaphore or a disc described: its aspects by name
		std::string names;
		for (auto const &name : Head.aspect_names)
			names += (names.empty() ? "" : " ") + name;
		ImGui::TextUnformatted(names.c_str());
	}
	else if (Head.lamps.empty())
		ImGui::TextDisabled("%s", Head.kind == "warning" ? STR_C("disc") : STR_C("arms"));
	else
		lamp_row(Head.lamps, Head.own);
}

void srk_tool::track_lengths::render_length_row(std::size_t const Index, std::array<char const *, 2> const &Ways, std::string const &Stem)
{
	auto const &track{m_lengths[Index]};
	ImGui::TableNextRow();
	ImGui::TableNextColumn();
	if (ImGui::Selectable(std::format("{}##srklength{}", track.number, Index).c_str(), static_cast<int>(Index) == m_length_shown, ImGuiSelectableFlags_SpanAllColumns))
		show_length(track);
	if (ImGui::IsItemHovered())
	{
		m_length_hover = static_cast<int>(Index);
		length_tooltip(track, Ways, Stem);
	}
	auto const unequal{std::abs(track.ways[0].length - track.ways[1].length) > kUnequal};
	for (auto const &part : track.ways)
	{
		ImGui::TableNextColumn();
		length_cell(part, unequal);
	}
	ImGui::TableNextColumn();
	ImGui::Text("%.0f m", std::max(0.0, std::min(track.ways[0].length, track.ways[1].length) - kTrainMargin));
}

void srk_tool::track_lengths::show_length(station::lengths const &Track)
{
	// the track at the middle of its useful length selected, the camera over it
	auto const &way{Track.ways[0]};
	auto const middle{stretch_of(Track, way.rear.along, way.head.along)};
	auto const &at{middle[middle.size() / 2]};
	station::path const *nearest{nullptr};
	auto best{std::numeric_limits<double>::max()};
	for (auto const &path : m_tool.m_layout->paths)
		for (auto const &point : path.points)
			if (auto const distance{glm::distance(glm::dvec2{point.x, point.z}, glm::dvec2{at.x, at.z})}; false == path.approach && distance < best)
			{
				best = distance;
				nearest = &path;
			}
	if (nearest == nullptr)
		return;
	m_tool.m_editor.selection.set_current_node(nearest->track);
	m_tool.m_picked = nearest->track;
	m_tool.m_editor.view.focus_track(*nearest->track, nearest->index, at);
}

void srk_tool::head_checks::render_heads()
{
	if (false == m_heads_known)
	{
		dimmed(STR_C("Looking for the signal templates..."));
		return;
	}
	if (std::ranges::none_of(m_heads, [](head_check const &Head) { return Head.station; }))
	{
		dimmed(STR_C("No main signal stands by the station yet: its lights are checked once one does"));
		if (ImGui::Button((std::string{STR_C("Set the signals")} + "##srkheads").c_str(), ImVec2(-1.0f, 0.0f)))
			m_tool.m_steps.set_signals();
		return;
	}
	auto const line{m_tool.speed_of_open()};
	std::vector<std::size_t> rows;
	std::size_t fine{0};
	for (std::size_t index = 0; index < m_heads.size(); ++index)
	{
		auto const &head{m_heads[index]};
		if (head.station || false == head.fits || false == head.problems.empty() || (head.shp_of() != shp_state::needless && head.shp_of() != shp_state::placed))
			rows.push_back(index);
		else
			++fine;
	}
	if (ImGui::BeginTable("##srkheadlist", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY, ImVec2(0.0f, std::min(320.0f, ImGui::GetTextLineHeightWithSpacing() * (static_cast<float>(rows.size()) + 1.5f)))))
	{
		ImGui::TableSetupScrollFreeze(0, 1);
		ImGui::TableSetupColumn(STR_C("Signal"), ImGuiTableColumnFlags_WidthStretch, 0.2f);
		ImGui::TableSetupColumn(STR_C("Speeds"), ImGuiTableColumnFlags_WidthStretch, 0.2f);
		ImGui::TableSetupColumn(STR_C("Lights"), ImGuiTableColumnFlags_WidthStretch, 0.21f);
		ImGui::TableSetupColumn(STR_C("State"), ImGuiTableColumnFlags_WidthStretch, 0.24f);
		ImGui::TableSetupColumn(STR_C("SHP"), ImGuiTableColumnFlags_WidthStretch, 0.15f);
		ImGui::TableHeadersRow();
		for (auto const index : rows)
			render_head_row(index, line);
		ImGui::EndTable();
	}
	if (fine > 0)
		ImGui::TextDisabled("%s", format(STR_P("%zu signal of the lines fits, its SHP too", "%zu signals of the lines fit, their SHP too", fine), fine).c_str());
	auto const fitting{static_cast<std::size_t>(std::ranges::count_if(m_heads, [](head_check const &Head) { return false == Head.fit.empty() && Head.fit != Head.file && (false == Head.fits || Head.more); }))};
	ImGui::BeginDisabled(fitting == 0);
	if (main_button(format("%s (%zu)###srkfitheads", STR_C("Fit the lights"), fitting).c_str(), ImVec2(-1.0f, 0.0f)))
		fit_heads();
	ImGui::EndDisabled();
	item_tooltip("Gives each signal marked the template of its kind and mount with the lights its routes need and the least beyond them, the name, the plate and the place kept, the trains reading it as before. One step of undo");
	dimmed(STR_C("The lights go by the routes of the trains: 40 km/h through the switches 1:9, 60 through 1:12 and 1:14, 100 through 1:18.5, by their radii, for these a lower orange light, with an orange strip for 60, a green one for 100, Ie-1; an upper orange one where the next signal can show stop; the white one of the substitute signal. A signal telling of the next one standing nearer than the braking distance slows the trains with it at stop, Ie-4 § 8 ust. 5"));
}

void srk_tool::head_checks::render_head_row(std::size_t const Index, double const Line)
{
	auto const &head{m_heads[Index]};
	ImGui::TableNextRow();
	ImGui::TableNextColumn();
	if (ImGui::Selectable(std::format("{}##srkhead{}", head.name, Index).c_str(), static_cast<int>(Index) == m_head_hover, ImGuiSelectableFlags_SpanAllColumns))
	{
		m_tool.m_editor.view.fly_to(head.position + glm::dvec3{0.0, 4.0, 0.0}, glm::normalize(glm::dvec3{head.travel.x, -0.3, head.travel.y}), 30.0);
		m_tool.m_editor.signals.select_signal(head.instance);
	}
	if (ImGui::IsItemHovered())
	{
		m_head_hover = static_cast<int>(Index);
		head_tooltip(head, Line, m_tool.stem_of_open());
	}
	ImGui::TableNextColumn();
	head_speeds_cell(head, Line);
	ImGui::TableNextColumn();
	render_head_aspects(head);
	ImGui::TableNextColumn();
	head_state_cell(head);
	ImGui::TableNextColumn();
	head_shp_cell(head);
}

std::string srk_tool::track_lengths::lengths_table() const
{
	auto const stem{m_tool.stem_of_open()};
	std::string result{STR_C("Track\tConstruction length [m]\tUseful to the right [m]\tfrom\tto\tUseful to the left [m]\tfrom\tto")};
	result += '\n';
	for (auto const &track : m_lengths)
	{
		result += format("%s\t%.0f", track.number.c_str(), track.construction);
		for (auto const &part : track.ways)
			result += format("\t%.0f\t%s\t%s", part.length, limit_text(part.rear, stem).c_str(), limit_text(part.head, stem).c_str());
		result += '\n';
	}
	return result;
}

int srk_tool::track_lengths::length_of(TTrack const *Track) const
{
	if (Track == nullptr || Track->Isolated.empty())
		return -1;
	auto const number{track_of(Track->Isolated.front()->asName, m_tool.stem_of_open())};
	for (std::size_t index = 0; index < m_lengths.size() && false == number.empty(); ++index)
		if (m_lengths[index].number == number)
			return static_cast<int>(index);
	return -1;
}

void srk_tool::steps::number_switches()
{
	m_tool.m_status.clear();
	auto const code{m_tool.m_record >= 0 ? m_tool.m_records[m_tool.m_record].code : std::string{}};
	if (code.empty())
	{
		m_tool.m_status = STR_C("Give the station its code first: the switches are named after it");
		return;
	}
	if (m_tool.m_records[m_tool.m_record].reversed != m_tool.m_reversed)
	{
		m_tool.m_records[m_tool.m_record].reversed = m_tool.m_reversed;
		m_tool.m_changed = true;
	}
	auto const units{station::numbering(*m_tool.m_layout, m_tool.m_reversed)};
	switch_names names;
	std::unordered_set<TTrack const *> own;
	for (std::size_t index = 0; index < units.size(); ++index)
		for (std::size_t part = 0; part < units[index].parts.size(); ++part)
		{
			names.emplace_back(units[index].parts[part], station::switch_name(code, static_cast<int>(index) + 1, part, units[index].parts.size()));
			own.insert(units[index].parts[part]);
		}
	if (false == names_allowed(names, own) || false == renames_confirmed(code, names))
		return;
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::map<std::string, std::string> parts; // NOSONAR: switch_store::rename takes the maps with the default comparator
	std::map<std::string, std::string> bases; // NOSONAR: switch_store::rename takes the maps with the default comparator
	for (auto const &[track, name] : names)
		if (track->name() != name)
		{
			before.emplace_back(track, editor_track::capture(*track));
			parts[track->name()] = name;
		}
	for (std::size_t index = 0; index < units.size(); ++index)
		if (false == units[index].record.empty())
			bases[units[index].record] = station::switch_name(code, static_cast<int>(index) + 1, 0, 1);
	switch_store::rename(parts, bases);
	auto following{following_of(names)};
	rename_switches(names);
	auto const numbered{renumber_records(units)};
	if (before.empty() && numbered == 0)
	{
		m_tool.m_status = STR_C("The switches have these numbers already");
		return;
	}
	if (before.empty())
		before.emplace_back(units.front().parts.front(), editor_track::capture(*units.front().parts.front()));
	auto const followed{following.size()};
	if (following.empty())
		m_tool.m_editor.history.push_track_snapshot(std::move(before));
	else
	{
		std::string error;
		m_tool.m_editor.signals.signal_batch({{}, {}, {}, std::move(following)}, std::move(before), {}, error);
	}
	m_tool.m_status = format(STR_P("%zu switch numbered", "%zu switches numbered", units.size()), units.size());
	if (followed > 0)
		m_tool.m_status += ", " + format(STR_P("%zu object of the scenery follows them", "%zu objects of the scenery follow them", followed), followed);
}

bool srk_tool::steps::names_allowed(std::vector<std::pair<TTrack *, std::string>> &Names, std::unordered_set<TTrack const *> const &Own)
{
	for (auto &[track, name] : Names)
	{
		std::string reason;
		TTrack *owner{nullptr};
		if (false == scene::Layers.editable(track, &reason))
		{
			m_tool.m_status = format(STR_C("%s can't be renamed: %s"), track->name().c_str(), reason.c_str());
			return false;
		}
		if (false == editor_track::name_valid(*track, name, reason, &owner) && (owner == nullptr || false == Own.contains(owner)))
		{
			m_tool.m_status = format(STR_C("%s can't be named %s: %s"), track->name().c_str(), name.c_str(), reason.c_str());
			return false;
		}
	}
	return true;
}

bool srk_tool::steps::renames_confirmed(std::string const &Code, std::vector<std::pair<TTrack *, std::string>> const &Names)
{
	// the events aiming at a switch by its old name or named after it, <name>:event0, the editor doesn't write: renamed, the
	// switch would lose them. they're listed first, the same renames clicked again go ahead
	std::vector<std::string> unwritten;
	std::string key{Code};
	for (auto const &[track, name] : Names)
	{
		if (track->name() == name)
			continue;
		key += ' ' + track->name() + '>' + name;
		auto const uses{editor_track::uses_of_name(*track)};
		for (auto const *events : {&uses.events, &uses.loose})
			for (auto const &aimed : *events)
				unwritten.push_back(track->name() + ": " + aimed);
	}
	if (unwritten.empty() || m_numbering_confirm == key)
	{
		m_numbering_confirm.clear();
		return true;
	}
	m_numbering_confirm = key;
	m_tool.m_status = format(STR_C("These events aim at the switches by their old names, the editor doesn't write them and they'd lose them: %s. Number the switches again to rename them anyway"), listed_of(unwritten).c_str());
	return false;
}

std::vector<std::pair<scene::instance_handle, std::vector<std::string>>> srk_tool::steps::following_of(std::vector<std::pair<TTrack *, std::string>> const &Names) const
{
	// the objects of the scenery naming a switch, as the lanterns of rainsted/lwzm.inc, follow it
	std::map<std::string, std::string, std::less<>> renamed;
	for (auto const &[track, name] : Names)
		if (track->name() != name)
			renamed[ToLower(track->name())] = name;
	std::vector<std::pair<scene::instance_handle, std::vector<std::string>>> following;
	for (auto &standing : m_tool.m_editor.templates.standing_templates([](std::string const &) { return true; }))
	{
		auto changed{false};
		for (auto &value : standing.values)
			if (auto const found{renamed.find(ToLower(value))}; found != renamed.end())
			{
				value = found->second;
				changed = true;
			}
		if (changed)
			following.emplace_back(standing.instance, std::move(standing.values));
	}
	return following;
}

void srk_tool::steps::set_circuits()
{
	m_tool.m_status.clear();
	auto const code{m_tool.m_record >= 0 ? m_tool.m_records[m_tool.m_record].code : std::string{}};
	if (code.empty())
	{
		m_tool.m_status = STR_C("Give the station its code first: the track circuits are named after it");
		return;
	}
	auto const units{station::numbering(*m_tool.m_layout, m_tool.m_reversed)};
	auto const planned{station::joints(*m_tool.m_layout, units)};
	if (false == circuits_editable(planned))
		return;
	track_states before;
	std::vector<TTrack *> created;
	std::vector<glm::dvec3> points;
	cut_at_joints(planned, points, before, created);
	// the circuits of the station as cut
	std::size_t count{0};
	if (auto const found{station::find(*m_tool.m_anchor)})
	{
		auto const circuits{station::circuits(*found, units, points)};
		name_circuits(*found, units, circuits, code, created, before);
		count = circuits.size();
	}
	if (before.empty() && created.empty())
	{
		m_tool.m_status = STR_C("The track circuits are set already");
		return;
	}
	m_tool.m_editor.history.push_track_snapshot(std::move(before), created);
	m_tool.m_status = format(STR_P("%zu track circuit", "%zu track circuits", count), count) + ", " + format(STR_P("%zu cut", "%zu cuts", created.size()), created.size());
}

bool srk_tool::steps::circuits_editable(std::vector<station::joint> const &Planned)
{
	for (auto const &entry : Planned)
		if (std::string reason; entry.track != nullptr && false == editor_track::can_edit_geometry(*entry.track, reason))
		{
			m_tool.m_status = format(STR_C("%s can't be cut: %s"), name_of(*entry.track).c_str(), reason.c_str());
			return false;
		}
	for (auto const &path : m_tool.m_layout->paths)
		if (std::string reason; false == path.approach && false == scene::Layers.editable(path.track, &reason))
		{
			m_tool.m_status = format(STR_C("%s can't be changed: %s"), name_of(*path.track).c_str(), reason.c_str());
			return false;
		}
	return true;
}

std::string srk_tool::stem_of_open() const
{
	return m_record >= 0 && false == m_records[m_record].code.empty() ? station::prefix(m_records[m_record].code) : std::string{};
}

std::string srk_tool::line_of_open() const
{
	if (m_record >= 0 && false == m_records[m_record].line.empty())
		return m_records[m_record].line;
	if (false == m_layout.has_value())
		return {};
	return kilometrage::line_at(axis_point(*m_layout, (m_layout->from + m_layout->to) * 0.5));
}

bool srk_tool::km_of_line(double const Chainage, double &Km) const
{
	auto const line{line_of_open()};
	if (false == m_layout.has_value() || line.empty())
		return false;
	// along the axis of the station, on beyond its switches the way the km go through it
	auto const &layout{*m_layout};
	auto const within{std::clamp(Chainage, layout.from, layout.to)};
	double low{0.0};
	double high{0.0};
	if (false == kilometrage::km_at(line, axis_point(layout, within), Km) || false == kilometrage::km_at(line, axis_point(layout, layout.from), low) || false == kilometrage::km_at(line, axis_point(layout, layout.to), high))
		return false;
	Km += (Chainage - within) / 1000.0 * (high >= low ? 1.0 : -1.0);
	return true;
}

bool srk_tool::km_of_open(double const Chainage, double &Km) const
{
	if (false == m_layout.has_value())
		return false;
	if (km_of_line(Chainage, Km))
		return true;
	if (m_record >= 0 && false == m_records[m_record].km.empty())
	{
		char *end{nullptr};
		auto const &text{m_records[m_record].km};
		auto const middle{std::strtod(text.c_str(), &end)};
		if (end != text.c_str())
		{
			Km = middle + (Chainage - (m_layout->from + m_layout->to) * 0.5) / 1000.0 * (m_reversed ? -1.0 : 1.0);
			return true;
		}
	}
	if (false == m_layout->kilometrage)
		return false;
	Km = station::km_of(*m_layout, Chainage);
	return true;
}

bool srk_tool::station_tracks(std::vector<station::numbered> &Tracks, std::unordered_set<TTrack const *> &Inside) const
{
	Tracks.clear();
	Inside.clear();
	auto const stem{stem_of_open()};
	if (false == m_layout.has_value() || stem.empty())
		return false;
	std::map<std::string, std::map<std::string, std::vector<TTrack *>, std::less<>>, std::less<>> numbers;
	for (auto const &path : m_layout->paths)
	{
		if (path.track->m_editorremoved || path.track->Isolated.empty() || Inside.contains(path.track))
			continue;
		auto const &name{path.track->Isolated.front()->asName};
		if (name.size() <= stem.size() || name.compare(0, stem.size(), stem) != 0 || entry_circuit(name, stem))
			continue;
		Inside.insert(path.track);
		if (auto const number{track_of(name, stem)}; false == number.empty())
			numbers[number][name].push_back(path.track);
	}
	for (auto &[number, circuits] : numbers)
	{
		station::numbered entry;
		entry.number = number;
		for (auto &[name, members] : circuits)
			entry.circuits.push_back(std::move(members));
		Tracks.push_back(std::move(entry));
	}
	return false == Tracks.empty();
}

void srk_tool::steps::renumber_track(TTrack const &Track, std::string const &Number)
{
	m_tool.m_status.clear();
	auto const stem{m_tool.stem_of_open()};
	auto const from{Track.Isolated.empty() ? std::string{} : track_of(Track.Isolated.front()->asName, stem)};
	std::string to;
	for (auto const letter : Number)
		if (std::isalnum(static_cast<unsigned char>(letter)))
			to += static_cast<char>(std::tolower(static_cast<unsigned char>(letter)));
	if (from.empty() || to.empty() || to == from)
		return;
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::unordered_set<TTrack const *> seen;
	for (auto const &path : m_tool.m_layout->paths)
	{
		if (path.track->Isolated.empty() || false == seen.insert(path.track).second)
			continue;
		auto const number{track_of(path.track->Isolated.front()->asName, stem)};
		if (number == to)
		{
			m_tool.m_status = format(STR_C("The station has a track %s already"), to.c_str());
			return;
		}
		if (number == from)
			before.emplace_back(path.track, editor_track::capture(*path.track));
	}
	for (auto const &[track, state] : before)
	{
		auto const &name{track->Isolated.front()->asName};
		editor_track::isolated(*track, {stem + "it" + to + name.substr(stem.size() + 2 + from.size())});
	}
	if (before.empty())
		return;
	m_tool.m_editor.history.push_track_snapshot(std::move(before));
	m_tool.m_status = format(STR_C("Track %s is track %s now"), from.c_str(), to.c_str());
}

void srk_tool::steps::set_signals()
{
	m_tool.m_status.clear();
	auto const stem{m_tool.stem_of_open()};
	if (stem.empty())
	{
		m_tool.m_status = STR_C("Give the station its code first: the signals are named after it");
		return;
	}
	auto const &layout{*m_tool.m_layout};
	std::vector<station::numbered> tracks;
	std::unordered_set<TTrack const *> inside;
	if (false == m_tool.station_tracks(tracks, inside))
	{
		m_tool.m_status = STR_C("Set the track circuits first: the signals stand at their joints");
		return;
	}
	auto plan{station::signals(layout, tracks, [&inside](TTrack const &Track) { return inside.contains(&Track); })};
	std::vector<standing_signal> standing;
	m_tool.m_editor.signals.standing_signals(standing, true);
	std::erase_if(standing, [&layout](standing_signal const &Signal) { return gap_to(layout, Signal.position) > 8.0; });
	// the standing ones by their places, then by their names the ones moved away
	std::vector<char> used(standing.size(), 0);
	auto const match{match_signals(plan, standing, stem, used)};
	std::vector<entry_cut> cuts;
	if (auto const problem{plan_cuts(plan, match, standing, cuts)}; false == problem.empty())
	{
		m_tool.m_status = problem;
		return;
	}
	// a template for each signal placed, before any track is cut for them
	if (auto const missing{m_tool.m_editor.signals.signal_templates_missing(wanted_of(plan, match))}; false == missing.empty())
	{
		m_tool.m_status = format(STR_C("The signals couldn't be placed: %s"), missing.c_str());
		return;
	}
	track_states before;
	std::vector<TTrack *> created;
	cut_entries(cuts, plan, stem, before, created);
	std::vector<signal_order> placed;
	std::vector<signal_rename> renamed;
	orders_of(plan, match, standing, used, stem, placed, renamed);
	if (placed.empty() && renamed.empty() && before.empty() && created.empty())
	{
		m_tool.m_status = STR_C("The signals are set already");
		return;
	}
	auto const unfit{m_tool.m_head_checks.fit_placed(placed, stem)};
	std::string error;
	auto const touched{false == before.empty() || false == created.empty()};
	auto const count{m_tool.m_editor.signals.signal_batch({placed, renamed, {}, {}, {}}, std::move(before), std::move(created), error)};
	m_tool.m_status = signals_status(count, renamed.size(), error, touched, unfit);
}

std::vector<std::string> srk_tool::head_checks::fit_placed(std::vector<signal_order> &Placed, std::string const &Stem)
{
	// the lights of the main signals placed after the routes of their trains, to the signals standing and to these
	std::vector<std::string> unfit;
	std::vector<signal_head> heads;
	m_tool.m_editor.signals.signal_heads(heads, true);
	auto const chosen{std::ranges::find_if(heads, [](signal_head const &Head) { return Head.kind == "main" && Head.chosen; })};
	if (chosen == heads.end())
		return unfit;
	std::vector<standing_signal> everywhere;
	m_tool.m_editor.signals.standing_signals(everywhere, true);
	std::vector<std::size_t> from;
	auto const posts{posts_with(everywhere, Placed, from)};
	auto const found{station::routes(posts, from, route_reach(m_tool.braking_of_open()))};
	auto const line{m_tool.speed_of_open()};
	std::size_t next{0};
	for (auto &order : Placed)
	{
		if (order.kind != "main")
			continue;
		auto const need{need_of(found[next++], true, line, m_tool.braking_of_open())};
		if (auto const best{best_head(heads, "main", chosen->mount, chosen->name, 0, need, chosen->arms > 0)}; best >= 0)
			order.file = heads[best].file;
		else
			unfit.push_back(signal_shown(order.name, Stem));
	}
	return unfit;
}

bool srk_tool::line_survey::lines_of_open(std::vector<standing_signal> const &Standing, std::vector<station::line> &Lines, std::vector<station::signal> &Entries, std::vector<std::size_t> &Owners, std::string &Problem) const
{
	Lines.clear();
	Entries.clear();
	Owners.clear();
	auto const stem{m_tool.stem_of_open()};
	if (false == m_tool.m_layout.has_value())
		return false;
	if (stem.empty())
	{
		Problem = STR_C("Give the station its code first: the signals are named after it");
		return false;
	}
	std::vector<station::numbered> tracks;
	std::unordered_set<TTrack const *> inside;
	if (false == m_tool.station_tracks(tracks, inside))
	{
		Problem = STR_C("Set the track circuits first: the signals stand at their joints");
		return false;
	}
	auto const plan{station::signals(*m_tool.m_layout, tracks, [&inside](TTrack const &Track) { return inside.contains(&Track); })};
	// the lines begin at the entry signals standing
	for (auto const &planned : plan)
	{
		if (planned.kind != station::signal::role::entry)
			continue;
		auto const found{std::ranges::find_if(Standing, [&stem, &planned](standing_signal const &Signal) { return Signal.kind == "main" && ToLower(Signal.name) == stem + ToLower(planned.name); })};
		if (found == Standing.end())
			continue;
		Entries.push_back(planned);
		Owners.push_back(static_cast<std::size_t>(found - Standing.begin()));
	}
	if (Entries.empty())
	{
		Problem = STR_C("Set the signals first: the lines begin at the entry signals");
		return false;
	}
	Lines = station::lines_out(*m_tool.m_layout, Entries, kLineLimit);
	if (Lines.empty())
	{
		Problem = STR_C("No line leads out of the station beyond its entry signals");
		return false;
	}
	// the distances from the entry signals as they stand
	for (auto &line : Lines)
	{
		double gap{0.0};
		auto const at{station::distance_on(line, Standing[Owners[line.entry]].position, gap)};
		if (gap < 4.5)
			for (auto &distance : line.distances)
				distance -= at;
	}
	return true;
}

srk_tool::block_found srk_tool::line_survey::block_of(station::line const &Line, std::string const &Entry, std::vector<standing_signal> const &Standing, std::vector<signal_head> const &Heads) const
{
	using enum srk_tool::block_reason;
	block_found found;
	auto four{false};
	for (auto const &signal : Standing)
	{
		if (signal.kind != "block")
			continue;
		double gap{0.0};
		if (auto const distance{station::distance_on(Line, signal.position, gap)}; gap > 4.5 || distance < 50.0 || distance > Line.length() - 1.0)
			continue;
		++found.blocks;
		four = four || std::ranges::any_of(Heads, [&signal](signal_head const &Head) { return Head.file == signal.file && Head.aspects == 4; });
	}
	if (found.blocks > 0)
	{
		found.own = four ? block_kind::four : block_kind::three;
		found.own_why = standing;
	}
	else if (Line.number > 0)
	{
		auto const chosen{std::ranges::find_if(Heads, [](signal_head const &Head) { return Head.kind == "block" && Head.chosen; })};
		found.own = chosen != Heads.end() && chosen->aspects == 4 ? block_kind::four : block_kind::three;
		found.own_why = double_track;
	}
	else
	{
		found.own = block_kind::semi;
		found.own_why = single;
	}
	found.kind = found.own;
	found.why = found.own_why;
	if (m_tool.m_record >= 0)
		if (auto const set{m_tool.m_records[m_tool.m_record].blocks.find(Entry)}; set != m_tool.m_records[m_tool.m_record].blocks.end())
		{
			found.kind = set->second;
			found.why = hand;
		}
	return found;
}

bool srk_tool::line_survey::both_ways(std::vector<station::line> const &Lines, std::size_t const Index, std::string const &Entry, std::vector<standing_signal> const &Standing, bool &Hand, bool &Own) const
{
	auto const &Line{Lines[Index]};
	auto along{false};
	auto wrong{false};
	auto disc{false};
	for (auto const &signal : Standing)
	{
		double distance{0.0};
		along = along || block_along(Line, signal, distance);
		wrong = wrong || block_along(Line, signal, distance, true);
		disc = disc || wrong_disc(Lines, Index, signal);
	}
	// block signals for the wrong-track trains, or none of either but a warning disc for them: the mixed block
	Own = Line.number > 0 && (wrong || (false == along && false == disc));
	Hand = false;
	if (m_tool.m_record >= 0)
		if (auto const set{m_tool.m_records[m_tool.m_record].ways.find(Entry)}; set != m_tool.m_records[m_tool.m_record].ways.end())
		{
			Hand = true;
			return Line.number > 0 && set->second;
		}
	return Own;
}

std::string srk_tool::line_survey::km_line(station::line const &Line) const
{
	auto const described{m_tool.line_of_open()};
	auto const out{station::point_on(Line, std::min(2000.0, Line.length() * 0.5)).point};
	if (double km{0.0}; false == kilometrage::km_at(described, out, km))
		if (auto const found{kilometrage::line_at(out)}; false == found.empty())
			return found;
	return described;
}

double srk_tool::line_survey::km_along(station::line const &Line, std::string const &Number, double const Distance) const
{
	double km{0.0};
	if (kilometrage::km_at(Number, station::point_on(Line, Distance).point, km) || false == m_tool.m_layout.has_value())
		return km;
	auto const middle{(m_tool.m_layout->from + m_tool.m_layout->to) * 0.5};
	auto const chainage{station::project(*m_tool.m_layout, station::point_on(Line, 0.0).point).x};
	m_tool.km_of_open(chainage < middle ? chainage - Distance : chainage + Distance, km);
	return km;
}

bool srk_tool::line_survey::km_rising(station::line const &Line, std::string const &Number) const
{
	auto const at{station::point_on(Line, 100.0)};
	if (auto const growth{kilometrage::growth_at(Number, at.point, {at.way.x, at.way.z})}; growth != 0)
		return (growth > 0) != Line.inbound;
	return (km_along(Line, Number, 1000.0) > km_along(Line, Number, 0.0)) != Line.inbound;
}

double srk_tool::line_survey::line_braking(station::line const &Line) const
{
	if (m_tool.m_record >= 0 && m_tool.m_records[m_tool.m_record].braking > 0)
		return static_cast<double>(m_tool.m_records[m_tool.m_record].braking);
	auto speed{0.0};
	for (std::size_t k = 0; k < Line.points.size() && k < Line.owners.size(); ++k)
		if (auto const set{Line.tracks[Line.owners[k]]->VelocityGet()}; Line.distances[k] >= 0.0 && set <= kFastest)
			speed = std::max(speed, set);
	return braking_distance(speed > 0.0 ? speed : 160.0);
}

namespace
{

// whether a point is along a line, beyond its first 50 m
bool on_line(station::line const &Line, glm::dvec3 const &Point)
{
	double gap{0.0};
	auto const distance{station::distance_on(Line, Point, gap)};
	return gap < 4.5 && distance > 50.0 && distance < Line.length() - 1.0;
}

// whether a point is along a line of another block than Automatic tells, Lines: the lines listed, Ways: their tracks
bool other_block_at(std::vector<srk_tool::line_row> const &Lines, std::vector<station::line> const &Ways, bool const Automatic, glm::dvec3 const &Point)
{
	for (std::size_t other = 0; other < Lines.size(); ++other)
		if ((Lines[other].block != srk_tool::block_kind::semi) != Automatic && on_line(Ways[other], Point))
			return true;
	return false;
}

// the signals of the other block along the lines, Ways: their tracks, but where a line shares its track with a line of that block:
// how many, the first of them
// whether the signal stands along the line of the row, Index, and is one of the other block
bool leftover_signal(std::vector<srk_tool::line_row> const &Lines, std::vector<station::line> const &Ways, std::size_t const Index, standing_signal const &Signal)
{
	auto const &row{Lines[Index]};
	auto const automatic{row.block != srk_tool::block_kind::semi};
	if (Signal.kind != (automatic ? "warning" : "block") || false == on_line(Ways[Index], Signal.position))
		return false;
	if (automatic && false == row.both && wrong_disc(Ways, Index, Signal))
		return false;
	return false == other_block_at(Lines, Ways, automatic, Signal.position);
}

void leftovers_found(std::vector<srk_tool::line_row> &Lines, std::vector<station::line> const &Ways, std::vector<standing_signal> const &Standing)
{
	using enum srk_tool::block_kind;
	for (std::size_t index = 0; index < Lines.size(); ++index)
	{
		auto &row{Lines[index]};
		if (row.legacy || (row.number == 0 && row.block != semi))
			continue;
		for (auto const &signal : Standing)
		{
			if (false == leftover_signal(Lines, Ways, index, signal) || row.leftovers++ > 0)
				continue;
			row.leftover = signal.position;
			row.leftover_travel = travel_of(signal.yaw);
		}
	}
}

} // namespace

bool srk_tool::line_survey::line_signals(line_row &Row, station::line const &Line, std::vector<station::line> const &Lines, std::vector<standing_signal> const &Standing, name_set const &Guards, std::string const &Stem) const
{
	using enum srk_tool::block_kind;
	auto switched{false};
	for (auto const &signal : Standing)
	{
		double distance{0.0};
		if (block_along(Line, signal, distance, true))
		{
			Row.wrong.emplace_back(distance, signal_shown(signal.name, Stem));
			Row.posts.emplace_back(distance, signal_shown(signal.name, Stem), true, km_along(Line, Row.km_number, distance));
		}
		if (false == block_along(Line, signal, distance))
			continue;
		Row.stops.emplace_back(distance + station::signal_drop * out_of(Line), signal_shown(signal.name, Stem));
		Row.posts.emplace_back(distance, signal_shown(signal.name, Stem), false, km_along(Line, Row.km_number, distance));
		switched = switched || Guards.contains(ToLower(signal.name));
	}
	// the warning discs: of the semi-automatic block, of the wrong-track trains of the mixed one
	for (auto const &signal : Standing)
	{
		double distance{0.0};
		auto const mixed_lines{Row.block != semi && wrong_disc(Lines, static_cast<std::size_t>(&Line - Lines.data()), signal)};
		if (signal.kind == "warning" && (mixed_lines || (Row.block == semi && faces_along(Line, signal, distance))))
		{
			faces_along(Line, signal, distance, mixed_lines);
			Row.posts.emplace_back(distance, signal_shown(signal.name, Stem), mixed_lines, km_along(Line, Row.km_number, distance));
		}
	}
	return switched;
}

std::vector<std::pair<double, double>> srk_tool::line_survey::line_sections(line_row &Row, station::line const &Line) const
{
	for (std::size_t k = 0; k < Line.points.size() && k < Line.distances.size() && k < Line.owners.size(); ++k)
	{
		auto const *track{Line.owners[k] < Line.tracks.size() ? Line.tracks[Line.owners[k]] : nullptr};
		if (auto const name{track == nullptr || track->Isolated.empty() ? std::string{} : track->Isolated.front()->asName}; Row.sections.empty() || Row.sections.back().name != name)
			Row.sections.emplace_back(Line.distances[k], Line.distances[k], name);
		Row.sections.back().to = Line.distances[k];
		if (k > 0 && Row.sections.size() > 1 && Row.sections.back().from == Line.distances[k])
			Row.sections[Row.sections.size() - 2].to = Line.distances[k];
	}
	std::vector<std::pair<double, double>> bare;
	for (std::size_t k = 1; k + 1 < Row.sections.size(); ++k)
		if (Row.sections[k].name.empty() && Row.sections[k].from >= 0.0 && Row.sections[k].to - Row.sections[k].from >= 1.0)
			bare.emplace_back(Row.sections[k].from, Row.sections[k].to);
	std::erase_if(Row.sections, [](line_row::run const &Run) { return Run.name.empty() || Run.to - std::max(0.0, Run.from) < 1.0; });
	for (auto &run : Row.sections)
	{
		run.km_from = km_along(Line, Row.km_number, run.from);
		run.km_to = km_along(Line, Row.km_number, run.to);
	}
	return bare;
}

void srk_tool::line_survey::survey_lines(std::vector<standing_signal> const &Standing)
{
	using enum srk_tool::block_kind;
	m_lines.clear();
	m_line_ways.clear();
	m_lines_problem.clear();
	std::vector<station::line> lines;
	std::vector<station::signal> entries;
	std::vector<std::size_t> owners;
	if (false == lines_of_open(Standing, lines, entries, owners, m_lines_problem))
		return;
	std::vector<signal_head> heads;
	m_tool.m_editor.signals.signal_heads(heads, false);
	auto const guards{guarded(m_tool.m_editor.templates.standing_templates([](std::string_view const File) { return File == kControl; }))};
	auto const stem{m_tool.stem_of_open()};
	for (auto const &line : lines)
	{
		line_row row;
		row.entry = entries[line.entry].name;
		row.number = line.number;
		row.inbound = line.inbound;
		row.partner = line.partner;
		row.station = line.station;
		row.position = Standing[owners[line.entry]].position;
		row.travel = travel_of(Standing[owners[line.entry]].yaw);
		auto const block{block_of(line, row.entry, Standing, heads)};
		row.block = block.kind;
		row.why = block.why;
		row.own = block.own;
		row.own_why = block.own_why;
		row.blocks = block.blocks;
		row.braking = line_braking(line);
		row.km_number = km_line(line);
		row.rising = km_rising(line, row.km_number);
		row.both = both_ways(lines, static_cast<std::size_t>(&line - lines.data()), row.entry, Standing, row.both_hand, row.own_both);
		auto const switched{line_signals(row, line, lines, Standing, guards, stem)};
		std::ranges::sort(row.stops);
		std::ranges::sort(row.wrong);
		// the schematic: straight out of the entry signal along the chainage, the track circuits along it in the colours of their names
		auto const entry_at{station::project(*m_tool.m_layout, station::point_on(line, 0.0).point)};
		row.chainage = entry_at.x;
		row.offset = entry_at.y;
		row.outward = station::project(*m_tool.m_layout, station::point_on(line, std::min(50.0, line.length())).point).x >= entry_at.x ? 1.0 : -1.0;
		double probe{0.0};
		row.km_known = kilometrage::km_at(row.km_number, station::point_on(line, 0.0).point, probe) || m_tool.km_of_open(row.chainage, probe);
		auto bare{line_sections(row, line)};
		row.legacy = false == row.stops.empty() && false == switched;
		if (false == row.legacy && row.block != semi)
			row.bare = std::move(bare);
		auto const next{next_entry(line, Standing, row.length)};
		row.next = next >= 0 ? signal_shown(Standing[next].name, stem) : std::string{};
		m_lines.push_back(std::move(row));
		m_line_ways.push_back(line);
	}
	// the signals of the other block along a line, but where it shares its track with a line of that block
	leftovers_found(m_lines, m_line_ways, Standing);
}

namespace
{

// whether a point is by the station open or by one of the lines out of it
bool near_open(station::layout const &Layout, std::vector<station::line> const &Ways, glm::dvec3 const &Point)
{
	if (gap_to(Layout, Point) <= 8.0)
		return true;
	return std::ranges::any_of(Ways, [&](station::line const &Line) {
		double gap{0.0};
		auto const distance{station::distance_on(Line, Point, gap)};
		return gap <= 8.0 && distance > -50.0 && distance < Line.length() + 1.0;
	});
}

// how far beyond the end of a line, broken off, the tracks go on in its way: to the first free end ahead of it; none where none is
std::optional<double> tracks_beyond(station::line const &Line)
{
	auto const plan = [](glm::dvec3 const &Point) { return glm::dvec2{Point.x, Point.z}; };
	auto const end{Line.points.back()};
	auto const step{plan(Line.points.back() - Line.points[Line.points.size() - 2])};
	if (glm::length(step) <= 1e-9)
		return std::nullopt;
	auto const way{glm::normalize(step)};
	std::vector<TTrack const *> const own(Line.tracks.begin(), Line.tracks.end());
	for (auto const &target : editor_track::free_ends(nullptr, 1, end, kGapReach, own))
	{
		auto const ahead{plan(target.position - end)};
		auto const along{glm::dot(ahead, way)};
		if (auto const direction{plan(target.direction)}; along < 5.0 || along < glm::length(ahead) * 0.8 || glm::length(direction) < 1e-9 || std::abs(glm::dot(glm::normalize(direction), way)) < 0.7)
			continue;
		return glm::length(ahead);
	}
	return std::nullopt;
}

// the names of the track circuits of the scenery, in lower case
std::unordered_set<std::string, string_hash, std::equal_to<>> scenery_circuits()
{
	std::unordered_set<std::string, string_hash, std::equal_to<>> circuits;
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		for (auto const *isolated : track->Isolated)
			if (isolated != nullptr)
				circuits.insert(ToLower(isolated->asName));
	}
	return circuits;
}

} // namespace

void srk_tool::line_survey::survey_problems(std::vector<standing_signal> const &Standing)
{
	m_problems.clear();
	if (false == m_tool.m_layout.has_value())
		return;
	auto const stem{m_tool.stem_of_open()};
	auto const by_open = [this](glm::dvec3 const &Point) { return near_open(*m_tool.m_layout, m_line_ways, Point); };
	for (auto const &signal : Standing)
		if (false == signal.read.empty() && signal.track == nullptr && by_open(signal.position))
			m_problems.emplace_back(format(STR_C("Signal %s: no track reads it"), signal_shown(signal.name, stem).c_str()), signal.position, signal.instance);
	for (std::size_t index = 0; index < m_lines.size() && index < m_line_ways.size(); ++index)
	{
		auto const &row{m_lines[index]};
		auto const &line{m_line_ways[index]};
		// the tracks go on beyond a gap: one missing, deleted maybe
		auto const broken{false == row.station && line.points.size() > 1 && line.length() < kLineLimit - 50.0};
		if (auto const beyond{broken ? tracks_beyond(line) : std::nullopt}; beyond.has_value())
			m_problems.emplace_back(format(STR_C("The line out of %s breaks off %.0f m beyond it, the tracks go on %.0f m farther"), row.entry.c_str(), line.length(), *beyond), line.points.back());
		for (auto const &[bare_from, bare_to] : row.bare)
			m_problems.push_back({format(STR_C("The line out of %s: %.0f m of tracks %.0f m beyond the signal are in no block section"), row.entry.c_str(), bare_to - bare_from, bare_from), station::point_on(line, (bare_from + bare_to) * 0.5).point, 0, true});
	}
	// the block sections their controls switch, the tracks gone
	auto const circuits{scenery_circuits()};
	for (auto const &block_control : m_tool.m_editor.templates.standing_templates([](std::string_view const File) { return File == kControl; }))
		if (false == block_control.values.empty() && false == circuits.contains(ToLower(block_control.values[0])) && by_open(block_control.location))
			m_problems.emplace_back(format(STR_C("Block section %s has no track"), block_control.values[0].c_str()), block_control.location);
}

void srk_tool::line_survey::render_problems()
{
	if (m_problems.empty())
		return;
	ImGui::Spacing();
	ImGui::PushStyleColor(ImGuiCol_Text, kWrong);
	auto const open{ImGui::CollapsingHeader(format("%s (%zu)###srkproblems", STR_C("To repair"), m_problems.size()).c_str(), ImGuiTreeNodeFlags_DefaultOpen)};
	ImGui::PopStyleColor();
	item_tooltip("What the station and its lines need yet: signals no track reads, lines broken off where a track is missing, tracks of a line of the automatic block in no block section, block sections whose tracks are gone. Click a row to see the place");
	if (false == open)
		return;
	for (std::size_t index = 0; index < m_problems.size(); ++index)
	{
		auto const &problem{m_problems[index]};
		ImGui::PushStyleColor(ImGuiCol_Text, problem.signal != 0 || problem.lines ? kDoubtful : kWrong);
		if (ImGui::Selectable(std::format("{}##srkproblem{}", problem.text, index).c_str()))
			m_tool.m_editor.view.fly_to(problem.at + glm::dvec3{0.0, 4.0, 0.0}, glm::normalize(glm::dvec3{0.3, -0.5, 0.8}), 60.0);
		ImGui::PopStyleColor();
	}
	auto const fixable{static_cast<std::size_t>(std::ranges::count_if(m_problems, [](problem const &Problem) { return Problem.signal != 0 || Problem.lines; }))};
	if (fixable == 0)
	{
		ImGui::TextDisabled("%s", STR_C("A track missing is laid in the Track tool: laid in the place of a deleted one it takes over its events and its track circuit"));
		return;
	}
	if (main_button(format("%s (%zu)###srkfix", STR_C("Repair"), fixable).c_str(), ImVec2(-1.0f, 0.0f)))
		fix_problems();
	item_tooltip("The signals no track reads are read by the track they stand by again, as moving them does, the lines with tracks in no block section get their line signals set again. A track missing is laid in the Track tool. A step of undo each");
}

void srk_tool::line_survey::fix_problems()
{
	std::vector<scene::instance_handle> signals;
	auto lines{false};
	for (auto const &problem : m_problems)
	{
		if (problem.signal != 0)
			signals.push_back(problem.signal);
		lines = lines || problem.lines;
	}
	std::vector<std::string> done;
	if (false == signals.empty())
	{
		auto const read{m_tool.m_editor.signals.signal_reattach(signals)};
		done.push_back(format(STR_P("%zu of %zu signal read again", "%zu of %zu signals read again", static_cast<long long>(signals.size())), read, signals.size()));
		if (read < signals.size())
			done.back() += STR(": the rest stand farther than 10 m from a track, by vehicles or their events are gone");
	}
	auto const status{done.empty() ? std::string{} : done.front()};
	if (lines)
	{
		m_tool.m_status.clear();
		set_lines();
		if (false == status.empty())
			m_tool.m_status = status + (m_tool.m_status.empty() ? "" : "\n" + m_tool.m_status);
	}
	else
		m_tool.m_status = status;
}

// the state of set_lines(), each of its steps a function of its own: the lines out of the open station and the block of each
struct srk_tool::lines_set
{
	srk_tool &tool;
	block_edit const *edit{nullptr};
	std::string stem;
	std::vector<standing_signal> standing;
	std::vector<station::line> lines;
	std::vector<station::signal> entries;
	std::vector<std::size_t> entry_standing;
	std::vector<signal_head> heads;
	std::vector<standing_template> controls;
	name_set guards;
	// of each line, 1: the automatic block, placed on a double-track line only, 2: the automatic block left as it stands, on a
	// single-track line its signals facing both ways on a track, or with block signals the block sections of the editor don't switch,
	// 0: the semi-automatic one
	std::vector<char> automatic;
	std::vector<int> aspects;
	std::vector<char> both; // the automatic block both ways
	std::vector<std::vector<std::size_t>> along; // the block signals standing along each line facing its trains
	std::vector<std::vector<std::size_t>> wrong; // and facing its wrong-track trains
	// the lines set: all of them, or the one a block boundary is drawn on with the other track of a double-track one
	std::vector<char> processed;
	int edited{-1};
	double drawn{0.0};

	lines_set(srk_tool &Tool, block_edit const *Edit) : tool{Tool}, edit{Edit}, stem{Tool.stem_of_open()} {}

	// the standing signals, the lines out of the station, the heads and the controls of the block sections; false with the reason
	// while the station has no lines
	bool gather()
	{
		tool.m_editor.signals.standing_signals(standing, true);
		if (false == tool.m_line_survey.lines_of_open(standing, lines, entries, entry_standing, tool.m_status))
			return false;
		tool.m_editor.signals.signal_heads(heads, true);
		controls = tool.m_editor.templates.standing_templates([](std::string_view const File) { return File == kControl; });
		guards = guarded(controls);
		return true;
	}
	void classify()
	{
		automatic.assign(lines.size(), 0);
		aspects.assign(lines.size(), 3);
		both.assign(lines.size(), 0);
		along.resize(lines.size());
		wrong.resize(lines.size());
		for (std::size_t index = 0; index < lines.size(); ++index)
			classify_line(index);
	}
	void classify_line(std::size_t const Index)
	{
		auto const &line{lines[Index]};
		auto const kind{tool.m_line_survey.block_of(line, entries[line.entry].name, standing, heads).kind};
		aspects[Index] = kind == block_kind::four ? 4 : 3;
		auto hand{false};
		auto own_both{false};
		both[Index] = tool.m_line_survey.both_ways(lines, Index, entries[line.entry].name, standing, hand, own_both) ? 1 : 0;
		auto switched{false};
		for (std::size_t k = 0; k < standing.size(); ++k)
		{
			if (double distance{0.0}; block_along(line, standing[k], distance, true))
				wrong[Index].push_back(k);
			else if (block_along(line, standing[k], distance))
				along[Index].push_back(k);
			else
				continue;
			switched = switched || guards.contains(ToLower(standing[k].name));
		}
		if (kind != block_kind::semi)
			automatic[Index] = line.number > 0 && (along[Index].empty() || switched) ? 1 : 2;
	}
	// the lines set; false with the reason where the line a boundary is drawn on can't be set
	bool target()
	{
		processed.assign(lines.size(), edit == nullptr ? 1 : 0);
		if (edit == nullptr)
			return true;
		for (std::size_t index = 0; index < lines.size(); ++index)
			if (entries[lines[index].entry].name == edit->entry)
				edited = static_cast<int>(index);
		double gap{0.0};
		if (edited >= 0)
			drawn = station::distance_on(lines[edited], edit->point, gap);
		if (edited < 0 || gap > 5.0)
		{
			tool.m_status = STR_C("The line drawn on isn't there any more");
			return false;
		}
		if (automatic[edited] != 1)
		{
			tool.m_status = format(STR_C("The line beyond %s has no automatic block the editor sets: choose one for it in the table of the lines"), edit->entry.c_str());
			return false;
		}
		processed[edited] = 1;
		if (auto const partner{lines[edited].partner}; partner >= 0 && automatic[partner] == 1)
		{
			station::distance_on(lines[partner], station::point_on(lines[edited], drawn).point, gap);
			processed[partner] = gap < 12.0 ? 1 : 0;
		}
		return true;
	}
	// false with the reason while a line of the automatic block is set and the station has no line or km to name its signals after
	bool km_known() const
	{
		auto const described{tool.line_of_open()};
		double known{0.0};
		if (std::ranges::any_of(std::views::iota(std::size_t{0}, lines.size()), [this](std::size_t const Index) { return processed[Index] && automatic[Index] == 1; }) &&
			(false == tool.km_of_open(0.0, known) || described.empty()))
		{
			tool.m_status = STR_C("Give the station its line and its km first: the block signals and the block sections are named after them");
			return false;
		}
		return true;
	}
};

// the signals planned along the lines set, the standing ones they take, the block sections between them
struct srk_tool::lines_plan : lines_set
{
	struct planned_signal
	{
		station::signal::role kind{station::signal::role::block};
		std::string name;
		std::string plate;
		std::size_t line{0};
		double distance{0.0}; // out along the line
		std::string linked{"none"};
		int match{-1}; // of the standing signals
		bool kept{false}; // standing as it is, its name and its plate kept
		bool wrong{false}; // facing the wrong-track trains
		int twin{-1}; // the block signal of the other way at its boundary, of the planned ones
	};
	struct far_end
	{
		int signal{-1}; // the entry signal of the next station standing, of the standing signals
		double distance{0.0}; // out to it, or to where the signals of the line end
	};
	// a block section of a track of a double-track line, between the signals in the order the trains pass them
	struct section
	{
		std::string name;
		std::size_t line{0};
		double from{0.0};
		double to{0.0};
		std::string guard; // the block signal at its start, empty for a station one
		std::string wrong_guard; // the one at its end facing the wrong-track trains, empty for none
		glm::dvec3 start{0.0};
	};
	// a signal along a track of a line, in the order its trains pass them
	struct stop
	{
		double distance{0.0};
		std::string name;
		int signal{-1}; // of the planned ones, -1 for a station signal
		int standing{-1};
	};
	// a line planned: the least block section of it, the drop of a signal before its joint, the wrong-track ones standing paired
	// with a boundary
	struct line_work
	{
		std::size_t index{0};
		double least{0.0};
		double drop{0.0};
		std::unordered_set<std::size_t> paired;
	};
	using candidate = std::tuple<double, std::size_t, std::size_t>;
	using name_taken = std::unordered_set<std::string, string_hash, std::equal_to<>>;

	std::vector<planned_signal> planned;
	std::vector<far_end> ends;
	// the lines of the kilometrage the lines out run along, at a junction another one than the line of the station
	std::vector<std::string> numbers;
	// the standing signals taken away: of a boundary drawn away, of the other block left along a line
	std::vector<char> gone;
	std::vector<char> used; // the standing ones taken: gone, of a station or by a signal planned
	std::vector<section> sections;
	std::vector<std::pair<std::size_t, std::string>> relinked; // standing station signals, the block signal before each

	using lines_set::lines_set;

	double km_on(std::size_t const Index, double const Distance) const
	{
		return tool.m_line_survey.km_along(lines[Index], numbers[Index], Distance);
	}
	bool rising(std::size_t const Index) const
	{
		return tool.m_line_survey.km_rising(lines[Index], numbers[Index]);
	}
	double braking(std::size_t const Index) const
	{
		return tool.m_line_survey.line_braking(lines[Index]);
	}
	static double facing_out(station::line const &Line, standing_signal const &Signal, double const Distance)
	{
		auto const way{station::point_on(Line, Distance).way};
		return glm::dot(travel_of(Signal.yaw), glm::dvec2{way.x, way.z});
	}

	void plan_lines()
	{
		ends.resize(lines.size());
		for (auto const &line : lines)
			numbers.push_back(tool.m_line_survey.km_line(line));
		for (std::size_t index = 0; index < lines.size(); ++index)
			ends[index].signal = next_entry(lines[index], standing, ends[index].distance);
		gone.assign(standing.size(), 0);
		for (std::size_t index = 0; index < lines.size(); ++index)
			if (processed[index])
				plan_line(index);
	}
	void plan_line(std::size_t const Index)
	{
		auto const &line{lines[Index]};
		// the block signal of a boundary facing its trains before the joint, the wrong-track one beyond it
		line_work work{Index, braking(Index) * (aspects[Index] == 4 ? 0.5 : 1.0), station::signal_drop * out_of(line), {}};
		if (automatic[Index] == 1)
			plan_block(work);
		plan_disc(Index);
		// the signals of the other block left along the line: the warning discs of a line of the automatic block, the block signals of
		// one of the semi-automatic one
		if (edit != nullptr || automatic[Index] == 2)
			return;
		drop_other_block(Index);
	}
	int plan(std::size_t const Index, double const Distance, std::string const &Name, int const Match, bool const Wrong)
	{
		planned_signal signal;
		signal.name = Name;
		signal.plate = Match >= 0 ? std::string{} : Name;
		signal.line = Index;
		signal.distance = Distance;
		signal.match = Match;
		signal.kept = Match >= 0;
		signal.wrong = Wrong;
		planned.push_back(std::move(signal));
		return static_cast<int>(planned.size() - 1);
	}
	int nearest_wrong(line_work const &Work, double const Wanted, double &At) const
	{
		int found{-1};
		auto nearest{kWrongTrack};
		for (auto const k : wrong[Work.index])
		{
			double distance{0.0};
			block_along(lines[Work.index], standing[k], distance, true);
			if (0 == gone[k] && false == Work.paired.contains(k) && std::abs(distance - Wanted) < nearest)
			{
				nearest = std::abs(distance - Wanted);
				found = static_cast<int>(k);
				At = distance;
			}
		}
		return found;
	}
	// the wrong-track one: the one standing beyond the joint, else a new one there; none of one way only
	void add_wrong(line_work &Work, int const Normal)
	{
		auto at{planned[Normal].distance + 2.0 * Work.drop};
		auto const match{nearest_wrong(Work, at, at)};
		if (match >= 0)
			Work.paired.insert(static_cast<std::size_t>(match));
		if (false == both[Work.index])
		{
			if (match >= 0)
				gone[match] = 1;
			return;
		}
		auto const twin{plan(Work.index, at, match >= 0 ? standing[match].name : planned[Normal].name + "N", match, true)};
		planned[twin].twin = Normal;
		planned[Normal].twin = twin;
	}
	void plan_block(line_work &Work)
	{
		auto const boundary{boundary_on(Work.index)};
		if (false == along[Work.index].empty() || boundary.has_value())
			plan_standing(Work, boundary);
		else
			plan_even(Work);
		// the wrong-track ones of no boundary taken away
		for (auto const k : wrong[Work.index])
			if (false == Work.paired.contains(k))
				gone[k] = 1;
	}
	// the boundary drawn, the joint on the line drawn on and across on its other track
	std::optional<double> boundary_on(std::size_t const Index) const
	{
		if (static_cast<int>(Index) == edited)
			return drawn;
		if (edited < 0)
			return std::nullopt;
		double gap{0.0};
		if (auto const across{station::distance_on(lines[Index], station::point_on(lines[edited], drawn).point, gap)}; gap < 12.0)
			return across;
		return std::nullopt;
	}
	// the block signals standing kept as they stand, the boundary drawn added or the ones standing at it taken away
	void plan_standing(line_work &Work, std::optional<double> const &Boundary)
	{
		auto const index{Work.index};
		auto const &line{lines[index]};
		for (auto const k : along[index])
		{
			double distance{0.0};
			block_along(line, standing[k], distance);
			if (Boundary.has_value() && edit->remove && std::abs(distance + Work.drop - *Boundary) < kBoundaryReach)
			{
				gone[k] = 1;
				auto at{distance + 2.0 * Work.drop};
				if (auto const twin{nearest_wrong(Work, at, at)}; twin >= 0)
					gone[twin] = 1;
				continue;
			}
			add_wrong(Work, plan(index, distance, standing[k].name, static_cast<int>(k), false));
		}
		if (Boundary.has_value() && false == edit->remove)
			add_wrong(Work, plan(index, *Boundary - Work.drop, station::block_number(km_on(index, *Boundary - Work.drop), line.number, rising(index)), -1, false));
	}
	void plan_even(line_work &Work)
	{
		auto const index{Work.index};
		auto const &line{lines[index]};
		for (auto const distance : even_places(Work))
			add_wrong(Work, plan(index, distance, station::block_number(km_on(index, distance), line.number, rising(index)), -1, false));
	}
	// the block signals, Ie-4 § 51: the sections no shorter than the braking distance, half of it with four aspects, and no longer than
	// twice it, the line to the next station parted evenly, towards the end of the tracks a little over the least each
	std::vector<double> even_places(line_work const &Work) const
	{
		auto const &line{lines[Work.index]};
		auto const &end{ends[Work.index]};
		auto const least{Work.least};
		std::vector<double> places;
		if (line.station || end.signal >= 0)
		{
			// as many on both tracks of the line, where both have the automatic block
			auto const has_partner{line.partner >= 0 && line.partner < static_cast<int>(lines.size()) && automatic[line.partner] == 1};
			auto const reach{has_partner ? (end.distance + ends[line.partner].distance) * 0.5 : end.distance};
			double other{least};
			if (has_partner)
				other = braking(line.partner) * (aspects[line.partner] == 4 ? 0.5 : 1.0);
			auto const parts{std::max(1.0, std::floor(reach / std::max(least, other)))};
			for (int k = 1; k < parts; ++k)
				places.push_back(end.distance * k / parts);
		}
		else
			for (int k = 1; least * kBlockShare * k <= end.distance - 100.0; ++k)
				places.push_back(least * kBlockShare * k);
		return places;
	}
	// the warning disc of the entry signal, the braking distance before it: of the semi-automatic block, or of the mixed one where it's
	// the entry signal of the wrong-track trains, on the outside of the line facing them
	void plan_disc(std::size_t const Index)
	{
		auto const &line{lines[Index]};
		auto const mixed{automatic[Index] == 1 && line.number > 0 && false == line.inbound && false == both[Index] && edit == nullptr};
		if (automatic[Index] != 0 && false == mixed)
			return;
		if (auto const distance{std::min(braking(Index), ends[Index].distance - 100.0)}; distance >= 400.0)
		{
			auto const &entry{entries[line.entry]};
			planned_signal signal;
			signal.kind = station::signal::role::warning;
			signal.name = stem + "to" + ToLower(entry.name);
			signal.plate = "to" + ToLower(entry.name);
			signal.line = Index;
			signal.distance = distance;
			signal.wrong = mixed;
			planned.push_back(std::move(signal));
		}
	}
	void drop_other_block(std::size_t const Index)
	{
		auto const &line{lines[Index]};
		for (std::size_t k = 0; k < standing.size(); ++k)
		{
			if (automatic[Index] == 1 ? standing[k].kind != "warning" : std::ranges::find(along[Index], k) == along[Index].end())
				continue;
			// the one of the wrong-track trains of the mixed block kept
			if (automatic[Index] == 1 && false == both[Index] && wrong_disc(lines, Index, standing[k]))
				continue;
			auto const shared{shared_track(Index, standing[k])};
			double gap{0.0};
			auto const distance{station::distance_on(line, standing[k].position, gap)};
			if (false == shared && gap < 4.5 && distance > 50.0 && distance < line.length() - 1.0)
				gone[k] = 1;
		}
	}
	// where the line shares its track with one of the other block
	bool shared_track(std::size_t const Index, standing_signal const &Signal) const
	{
		for (std::size_t other = 0; other < lines.size(); ++other)
		{
			if ((automatic[other] == 0) == (automatic[Index] == 0))
				continue;
			double gap{0.0};
			if (auto const distance{station::distance_on(lines[other], Signal.position, gap)}; gap < 4.5 && distance > 50.0 && distance < lines[other].length() - 1.0)
				return true;
		}
		return false;
	}

	// the standing ones by their places, then by their names
	void match_standing()
	{
		used = gone;
		for (auto const &signal : planned)
			if (signal.match >= 0)
				used[signal.match] = 1;
		for (auto const index : entry_standing)
			used[index] = 1;
		for (auto const &end : ends)
			if (end.signal >= 0)
				used[end.signal] = 1;
		auto candidates{match_candidates()};
		std::ranges::sort(candidates);
		for (auto const &[score, i, k] : candidates)
			if (planned[i].match < 0 && 0 == used[k])
			{
				planned[i].match = static_cast<int>(k);
				used[k] = 1;
				double gap{0.0};
				planned[i].distance = station::distance_on(lines[planned[i].line], standing[k].position, gap);
			}
		for (auto &signal : planned)
			for (std::size_t k = 0; k < standing.size() && signal.match < 0; ++k)
				if (0 == used[k] && ToLower(standing[k].name) == ToLower(signal.name))
				{
					signal.match = static_cast<int>(k);
					used[k] = 1;
				}
	}
	std::vector<candidate> match_candidates() const
	{
		std::vector<candidate> candidates;
		for (std::size_t i = 0; i < planned.size(); ++i)
			if (planned[i].match < 0)
				signal_candidates(i, candidates);
		return candidates;
	}
	void signal_candidates(std::size_t const Index, std::vector<candidate> &Candidates) const
	{
		auto const &signal{planned[Index]};
		auto const &line{lines[signal.line]};
		auto const kind{signal.kind == station::signal::role::block ? "block" : "warning"};
		auto const reach{signal.kind == station::signal::role::block ? 300.0 : 400.0};
		for (std::size_t k = 0; k < standing.size(); ++k)
		{
			if (standing[k].kind != kind)
				continue;
			double gap{0.0};
			auto const distance{station::distance_on(line, standing[k].position, gap)};
			auto const facing{facing_out(line, standing[k], distance)};
			if (gap < 4.5 && std::abs(distance - signal.distance) < reach && (line.inbound != signal.wrong ? -facing : facing) > 0.87)
				Candidates.emplace_back(std::abs(distance - signal.distance) + gap * 10.0, Index, k);
		}
	}
	// a number another signal of the scenery has already gets a letter
	void name_letters()
	{
		name_taken taken;
		for (std::size_t k = 0; k < standing.size(); ++k)
			if (0 == used[k])
				taken.insert(ToLower(standing[k].name));
		for (auto const &signal : planned)
			if (signal.kept)
				taken.insert(ToLower(signal.name));
		// the wrong-track ones after their twins: 523N beside 523
		for (auto const wrong_track : {false, true})
			for (std::size_t i = 0; i < planned.size(); ++i)
				if (planned[i].kind == station::signal::role::block && false == planned[i].kept && planned[i].wrong == wrong_track)
					letter_name(i, taken);
	}
	void letter_name(std::size_t const Index, name_taken &Taken)
	{
		auto &signal{planned[Index]};
		auto const base{signal.wrong && signal.twin >= 0 ? planned[signal.twin].name + "N" : signal.name};
		signal.name = base;
		for (char letter = 'a'; Taken.contains(ToLower(signal.name)) && letter <= 'z'; ++letter)
			signal.name = base + letter;
		Taken.insert(ToLower(signal.name));
		signal.plate = signal.name;
	}

	// the block sections of each track of a double-track line, between the signals in the order the trains pass them
	void build_sections()
	{
		for (std::size_t index = 0; index < lines.size(); ++index)
			if (automatic[index] == 1 && processed[index])
				sections_of(index);
	}
	void sections_of(std::size_t const Index)
	{
		auto const order{stops_of(Index)};
		for (std::size_t i = 0; i < order.size(); ++i)
		{
			link_stop(order, i);
			if (i > 0)
				sections.push_back(section_between(Index, order, i));
		}
	}
	std::vector<stop> stops_of(std::size_t const Index) const
	{
		auto const &line{lines[Index]};
		std::vector<stop> order{{0.0, standing[entry_standing[line.entry]].name, -1, static_cast<int>(entry_standing[line.entry])}};
		for (std::size_t i = 0; i < planned.size(); ++i)
			if (planned[i].line == Index && false == planned[i].wrong)
				order.emplace_back(planned[i].distance, planned[i].name, static_cast<int>(i), planned[i].match);
		if (ends[Index].signal >= 0)
			order.emplace_back(ends[Index].distance, standing[ends[Index].signal].name, -1, ends[Index].signal);
		std::ranges::sort(order, [](stop const &A, stop const &B) { return A.distance < B.distance; });
		if (line.inbound)
			std::ranges::reverse(order);
		return order;
	}
	// the joints the trains pass the signals to: on beyond a block signal, in towards the station from an entry one
	double joint_of(std::size_t const Index, stop const &Stop) const
	{
		if (Stop.signal >= 0)
			return Stop.distance + (lines[Index].inbound ? -station::signal_drop : station::signal_drop);
		return Stop.distance + (Stop.distance > 0.0 ? station::signal_drop : -station::signal_drop);
	}
	// the wrong-track one of a stop, the wrong-track trains pass them the other way
	int twin_of(std::vector<stop> const &Order, std::size_t const At) const
	{
		return At < Order.size() && Order[At].signal >= 0 ? planned[Order[At].signal].twin : -1;
	}
	void link_stop(std::vector<stop> const &Order, std::size_t const At)
	{
		auto const &here{Order[At]};
		auto const block_before{At > 0 && Order[At - 1].signal >= 0};
		auto const linked{block_before ? Order[At - 1].name : std::string{"none"}};
		if (here.signal >= 0)
			planned[here.signal].linked = linked;
		else if (block_before && here.standing >= 0 && false == standing[here.standing].linked.empty() && ToLower(standing[here.standing].linked) != ToLower(linked))
			relinked.emplace_back(here.standing, linked);
		auto const wrong_before{twin_of(Order, At + 1)};
		auto const wrong_linked{wrong_before >= 0 ? planned[wrong_before].name : std::string{"none"}};
		if (auto const twin{twin_of(Order, At)}; twin >= 0)
			planned[twin].linked = wrong_linked;
		else if (At == 0 && here.signal < 0 && here.standing >= 0 && false == standing[here.standing].linked.empty())
			relink_wrong(here, wrong_before, wrong_linked);
	}
	// the station signal the wrong-track trains come to: after the last wrong-track block signal, after none once it's gone
	void relink_wrong(stop const &Here, int const WrongBefore, std::string const &WrongLinked)
	{
		auto const &had{standing[Here.standing].linked};
		auto const dropped{dropped_named(had)};
		if (ToLower(had) != ToLower(WrongLinked) && (WrongBefore >= 0 || dropped))
			relinked.emplace_back(Here.standing, WrongLinked);
	}
	bool dropped_named(std::string const &Name) const
	{
		for (std::size_t k = 0; k < standing.size(); ++k)
			if (gone[k] && ToLower(standing[k].name) == ToLower(Name))
				return true;
		return false;
	}
	section section_between(std::size_t const Index, std::vector<stop> const &Order, std::size_t const At) const
	{
		auto const &line{lines[Index]};
		auto const &here{Order[At]};
		auto const &previous{Order[At - 1]};
		// the lower km first, as on the lines of PKP PLK: 2411-2571@LK2.1
		auto const ascending{rising(Index)};
		auto const &low{ascending ? previous : here};
		auto const &high{ascending ? here : previous};
		section found;
		found.name = ToLower(std::format("{}-{}@lk{}.{}", section_tag(low.name, line.number), section_tag(high.name, line.number), numbers[Index], line.number));
		found.line = Index;
		found.from = std::min(joint_of(Index, previous), joint_of(Index, here));
		found.to = std::max(joint_of(Index, previous), joint_of(Index, here));
		found.guard = previous.signal >= 0 ? previous.name : std::string{};
		if (auto const twin{twin_of(Order, At)}; twin >= 0)
			found.wrong_guard = planned[twin].name;
		found.start = station::point_on(line, previous.distance).point;
		return found;
	}
};

// the changes of set_lines(): the tracks cut where the sections meet and named, the signals placed, the block sections switched
struct srk_tool::lines_change : lines_plan
{
	struct cut
	{
		TTrack *track{nullptr};
		glm::dvec3 point{0.0};
	};

	std::string disc_mount;
	std::vector<cut> cuts;
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::vector<TTrack *> created;
	std::unordered_set<TTrack const *> captured;
	std::unordered_map<TTrack const *, std::vector<TTrack *>> pieces; // of each track cut, itself and the pieces cut off it
	signal_changes changes;
	std::size_t blocks{0};
	std::size_t discs{0};
	std::size_t swapped{0};
	std::size_t renamed{0};
	std::size_t removed{0};
	bool controlled{false};

	using lines_plan::lines_plan;

	// the template of a signal placed: of a block signal by the aspects of its line, of the warning disc by the entry signal it tells
	// of, of the mount of the disc chosen
	std::string template_of(planned_signal const &Signal) const
	{
		if (Signal.kind == station::signal::role::block)
			return block_template(heads, {}, aspects[Signal.line], Signal.wrong);
		auto const &entry{standing[entry_standing[lines[Signal.line].entry]]};
		auto const main{std::ranges::find_if(heads, [&entry](signal_head const &Head) { return Head.file == entry.file; })};
		return disc_template(heads, main != heads.end() ? std::to_address(main) : nullptr, disc_mount);
	}
	// a template for each signal placed, before any track is cut for them; false with the reason where one is missing
	bool templates_ready()
	{
		disc_mount = tool.m_editor.signals.signal_mount("warning", {});
		std::vector<signal_order> wanted;
		for (auto const &signal : planned)
			if (signal.match < 0)
			{
				auto const block{signal.kind == station::signal::role::block};
				wanted.emplace_back();
				wanted.back().kind = block ? "block" : "warning";
				wanted.back().file = template_of(signal);
			}
		if (auto const missing{tool.m_editor.signals.signal_templates_missing(wanted)}; false == missing.empty())
		{
			tool.m_status = format(STR_C("The signals couldn't be placed: %s"), missing.c_str());
			return false;
		}
		return true;
	}
	bool add_cut(station::line const &Line, double const Distance)
	{
		auto const at{station::point_on(Line, Distance)};
		if (at.track == nullptr || std::ranges::any_of(cuts, [&](cut const &Cut) { return glm::distance(Cut.point, at.point) < 0.5; }))
			return true;
		if (auto const &ends_of{at.track->m_paths.front().points}; std::min(glm::distance(at.point, ends_of[segment_data::point::start]), glm::distance(at.point, ends_of[segment_data::point::end])) <= 0.3)
			return true;
		if (std::string reason; false == editor_track::can_edit_geometry(*at.track, reason))
		{
			tool.m_status = format(STR_C("%s can't be cut: %s"), name_of(*at.track).c_str(), reason.c_str());
			return false;
		}
		cuts.emplace_back(at.track, at.point);
		return true;
	}
	// the tracks cut where the sections meet, all of it checked before any is; false with the reason where one can't be
	bool cuts_checked()
	{
		for (auto const &found : sections)
			if (false == add_cut(lines[found.line], found.from) || false == add_cut(lines[found.line], found.to))
				return false;
		for (auto const &line : lines)
			for (auto const *part : line.tracks)
				if (std::string reason; false == scene::Layers.editable(part, &reason) && std::ranges::any_of(sections, [&](section const &Found) { return &lines[Found.line] == &line; }))
				{
					tool.m_status = format(STR_C("%s can't be changed: %s"), name_of(*part).c_str(), reason.c_str());
					return false;
				}
		return true;
	}
	TTrack *piece_at(TTrack *Track, glm::dvec3 const &Point) const
	{
		auto found{pieces.find(Track)};
		if (found == pieces.end())
			return Track;
		TTrack *best{Track};
		auto nearest{std::numeric_limits<double>::max()};
		for (auto *piece : found->second)
		{
			auto const point{editor_track::point_at(*piece, editor_track::nearest_parameter(*piece, Point))};
			if (auto const distance{glm::distance(point, Point)}; distance < nearest)
			{
				nearest = distance;
				best = piece;
			}
		}
		return best;
	}
	void split_tracks()
	{
		for (auto const &entry : cuts)
		{
			auto *track{piece_at(entry.track, entry.point)};
			auto const parameter{editor_track::nearest_parameter(*track, entry.point)};
			if (auto const &ends_of{track->m_paths.front().points}; std::min(glm::distance(entry.point, ends_of[segment_data::point::start]), glm::distance(entry.point, ends_of[segment_data::point::end])) <= 0.3)
				continue;
			if (captured.insert(track).second)
				before.emplace_back(track, editor_track::capture(*track));
			if (auto *piece{editor_track::split_path(*track, parameter)})
				keep_piece(entry.track, piece);
		}
	}
	void keep_piece(TTrack *Track, TTrack *Piece)
	{
		created.push_back(Piece);
		captured.insert(Piece);
		auto &list{pieces[Track]};
		if (list.empty())
			list.push_back(Track);
		list.push_back(Piece);
	}
	// the sections named, each track by its middle
	void name_sections()
	{
		for (std::size_t index = 0; index < lines.size(); ++index)
			if (processed[index])
				for (auto *part : line_parts(index))
					name_part(index, part);
	}
	std::vector<TTrack *> line_parts(std::size_t const Index) const
	{
		std::vector<TTrack *> parts;
		for (auto *track : lines[Index].tracks)
		{
			auto const found{pieces.find(track)};
			if (found == pieces.end())
				parts.push_back(track);
			else
				parts.insert(parts.end(), found->second.begin(), found->second.end());
		}
		return parts;
	}
	void name_part(std::size_t const Index, TTrack *Part)
	{
		double gap{0.0};
		auto const distance{station::distance_on(lines[Index], editor_track::point_at(*Part, 0.5), gap)};
		if (gap > 1.0)
			return;
		auto const found{std::ranges::find_if(sections, [&](section const &Section) { return Section.line == Index && distance > Section.from && distance < Section.to; })};
		if (found == sections.end() || (Part->Isolated.size() == 1 && ToLower(Part->Isolated.front()->asName) == found->name))
			return;
		if (captured.insert(Part).second)
			before.emplace_back(Part, editor_track::capture(*Part));
		editor_track::isolated(*Part, {found->name});
	}
	// the signals placed on the track the trains run on up to them
	void make_changes()
	{
		for (auto const &signal : planned)
			if (signal.match >= 0)
				change_standing(signal);
			else
				place_signal(signal);
		for (auto const &[index, linked] : relinked)
			changes.renamed.emplace_back(standing[index].instance, standing[index].name, std::string{}, linked);
		for (std::size_t k = 0; k < standing.size(); ++k)
			if (gone[k])
				changes.removed.push_back(standing[k].instance);
		removed = changes.removed.size();
	}
	void change_standing(planned_signal const &Signal)
	{
		auto const block{Signal.kind == station::signal::role::block};
		auto const &found{standing[Signal.match]};
		if (auto const relink{block && false == found.linked.empty() && ToLower(found.linked) != ToLower(Signal.linked)}; ToLower(found.name) != ToLower(Signal.name) || relink)
			changes.renamed.emplace_back(found.instance, Signal.name, Signal.plate, relink ? Signal.linked : std::string{});
		if (ToLower(found.name) != ToLower(Signal.name))
			++renamed;
		// of the aspects of its line
		if (auto const head{std::ranges::find_if(heads, [&](signal_head const &Head) { return Head.file == found.file; })}; block && head != heads.end() && head->aspects != 0 && (head->aspects != aspects[Signal.line] || (head->lean == "right") != Signal.wrong))
			if (auto const other{block_template(heads, found.file, aspects[Signal.line], Signal.wrong)}; false == other.empty() && other != found.file)
			{
				changes.files.emplace_back(found.instance, other);
				++swapped;
			}
	}
	void place_signal(planned_signal const &Signal)
	{
		auto const block{Signal.kind == station::signal::role::block};
		auto const &line{lines[Signal.line]};
		// its trains run in, the wrong-track ones of an outbound track too; a wrong-track one stands on the outside, on their left
		auto const inbound{line.inbound != Signal.wrong};
		auto const at{station::point_on(line, Signal.distance)};
		auto const approach{station::point_on(line, Signal.distance + (inbound ? 1.0 : -1.0))};
		signal_order order;
		order.track = piece_at(approach.track, approach.point);
		order.point = at.point;
		order.facing = inbound == approach.outward ? -1 : 1;
		order.side = Signal.wrong ? -order.facing : order.facing;
		order.kind = block ? "block" : "warning";
		order.file = template_of(Signal);
		order.name = Signal.name;
		order.plate = Signal.plate;
		order.linked = Signal.linked;
		changes.placed.push_back(std::move(order));
		++(block ? blocks : discs);
	}
	// the block signals switched by the sections they guard, the wrong-track one the 7th, or the one guard twice
	void set_controls()
	{
		include_info control_info;
		std::string missing;
		controlled = editor_includes::load(kControl, control_info, missing);
		if (false == controlled)
			return;
		for (auto const &found : sections)
			control_section(found);
		drop_controls();
	}
	void control_section(section const &Found)
	{
		auto const already{std::ranges::find_if(controls, [&](standing_template const &Control) { return false == Control.values.empty() && ToLower(Control.values[0]) == Found.name; })};
		if (Found.guard.empty() && Found.wrong_guard.empty())
		{
			// guarded by station signals only, once its wrong-track signal is gone
			if (already != controls.end())
				changes.removed.push_back(already->instance);
			return;
		}
		std::array<std::string, 2> const guards_of{Found.guard.empty() ? Found.wrong_guard : Found.guard, Found.wrong_guard.empty() ? Found.guard : Found.wrong_guard};
		if (already != controls.end())
		{
			change_guards(*already, guards_of);
			return;
		}
		template_item item;
		item.file = kControl;
		item.location = Found.start;
		item.described = true;
		item.values["name"] = Found.name;
		item.values["#6"] = guards_of[0];
		item.values["#7"] = guards_of[1];
		changes.others.push_back(std::move(item));
	}
	void change_guards(standing_template const &Control, std::array<std::string, 2> const &Guards)
	{
		auto values{Control.values};
		values.resize(std::max<std::size_t>(values.size(), 7));
		auto differ{false};
		for (std::size_t k = 0; k < Guards.size(); ++k)
			if (ToLower(values[5 + k]) != ToLower(Guards[k]))
			{
				values[5 + k] = Guards[k];
				differ = true;
			}
		if (differ)
			changes.values.emplace_back(Control.instance, std::move(values));
	}
	// those of the sections gone from the lines set taken away with them
	void drop_controls()
	{
		std::unordered_set<std::string, string_hash, std::equal_to<>> named;
		for (auto const &found : sections)
			named.insert(found.name);
		for (auto const &standing_control : controls)
		{
			if (standing_control.values.empty() || named.contains(ToLower(standing_control.values[0])))
				continue;
			if (control_on_lines(standing_control))
				changes.removed.push_back(standing_control.instance);
		}
	}
	bool control_on_lines(standing_template const &Control) const
	{
		for (std::size_t index = 0; index < lines.size(); ++index)
		{
			double gap{0.0};
			if (auto const distance{station::distance_on(lines[index], Control.location, gap)}; processed[index] && automatic[index] != 2 && gap < 4.5 && distance > 0.0 && distance < lines[index].length())
				return true;
		}
		return false;
	}
	void finish()
	{
		if (changes.placed.empty() && changes.renamed.empty() && changes.others.empty() && changes.values.empty() && changes.files.empty() && changes.removed.empty() && before.empty() && created.empty())
		{
			tool.m_status = STR_C("The lines are set already");
			return;
		}
		std::string error;
		auto const touched{false == before.empty() || false == created.empty()};
		tool.m_editor.signals.signal_batch(changes, std::move(before), std::move(created), error);
		if (false == error.empty())
		{
			tool.m_status = format(STR_C("The signals couldn't be placed: %s"), error.c_str()) + (touched ? ". " + STR("Ctrl+Z takes back the tracks cut") : std::string{});
			return;
		}
		tool.m_status = format(STR_P("%zu block signal placed", "%zu block signals placed", blocks), blocks) + ", " + format(STR_P("%zu warning disc", "%zu warning discs", discs), discs) + ", " +
						format(STR_P("%zu renamed", "%zu renamed", renamed), renamed) + ", " + format(STR_P("%zu block section", "%zu block sections", sections.size()), sections.size());
		if (removed > 0)
			tool.m_status += ", " + format(STR_P("%zu signal taken away", "%zu signals taken away", removed), removed);
		if (swapped > 0)
			tool.m_status += ", " + format(STR_P("%zu given the template of the aspects of its line", "%zu given the templates of the aspects of their lines", swapped), swapped);
		tool.m_status += ". " + STR("Ctrl+Z takes it back");
		if (false == controlled && std::ranges::any_of(sections, [](section const &Found) { return false == Found.guard.empty(); }))
			tool.m_status += "\n" + format(STR_C("%s isn't in the scenery folder: the block signals won't change their lights"), kControl);
	}
};

void srk_tool::line_survey::set_lines(block_edit const *Edit)
{
	m_tool.m_status.clear();
	lines_change work{m_tool, Edit};
	if (false == work.gather())
		return;
	work.classify();
	if (false == work.target() || false == work.km_known())
		return;
	work.plan_lines();
	work.match_standing();
	work.name_letters();
	work.build_sections();
	if (false == work.templates_ready() || false == work.cuts_checked())
		return;
	work.split_tracks();
	work.name_sections();
	work.make_changes();
	work.set_controls();
	work.finish();
}

namespace
{

// a mark of a signal put in the plot by the nearest point of the schematic within 6 m of where it stands: onto that point, to its side
template <typename Mark>
void mark_drawn(Mark &Signal, station::layout const &Layout, station::drawing const &Plot)
{
	auto nearest{6.0};
	for (std::size_t path = 0; path < Layout.paths.size(); ++path)
		for (std::size_t k = 0; k < Layout.paths[path].schematic.size(); ++k)
			if (auto const &point{Layout.paths[path].schematic[k]}; glm::distance(point, Signal.at) < nearest)
			{
				nearest = glm::distance(point, Signal.at);
				Signal.drawn = Plot.paths[path][k];
				Signal.side = Signal.at.y > point.y ? 1.0f : -1.0f;
			}
}

} // namespace

void srk_tool::survey_signals()
{
	m_schematic.m_signal_marks.clear();
	m_line_survey.m_signal_names.clear();
	std::vector<standing_signal> standing;
	m_signals_known = m_layout.has_value() && m_editor.signals.standing_signals(standing, false);
	for (auto const &signal : standing)
		m_line_survey.m_signal_names.insert(ToLower(signal.name));
	if (false == m_signals_known)
	{
		m_head_checks.m_heads.clear();
		m_head_checks.m_heads_known = false;
		m_line_survey.m_lines.clear();
		m_line_survey.m_line_ways.clear();
		m_line_survey.m_problems.clear();
		m_line_survey.m_lines_problem = STR("Looking for the signal templates...");
		if (false == m_track_lengths.m_measured)
			m_track_lengths.measure({});
		return;
	}
	m_head_checks.check_heads(standing);
	m_line_survey.survey_lines(standing);
	m_line_survey.survey_problems(standing);
	auto const stem{stem_of_open()};
	std::vector<station::signal> stops;
	for (auto const &signal : standing)
	{
		if (gap_to(*m_layout, signal.position) > 8.0)
			continue;
		auto const travel{travel_of(signal.yaw)};
		auto const at{station::project(*m_layout, signal.position)};
		auto const ahead{station::project(*m_layout, signal.position + glm::dvec3{travel.x, 0.0, travel.y} * 5.0)};
		auto const unfit{std::ranges::any_of(m_head_checks.m_heads, [&](head_check const &Head) { return Head.instance == signal.instance && (false == Head.fits || false == Head.problems.empty()); })};
		m_schematic.m_signal_marks.emplace_back(at, ahead.x > at.x, signal_shown(signal.name, stem), signal.position + glm::dvec3{0.0, signal.kind == "shunt" ? 3.0 : 7.0, 0.0}, unfit);
		// in the plot on the track it stands by, to its side
		auto &mark{m_schematic.m_signal_marks.back()};
		mark.drawn = m_plot.at(at);
		mark_drawn(mark, *m_layout, m_plot);
		if (signal.kind != "main" && signal.kind != "shunt")
			continue;
		station::signal stop;
		stop.name = signal_shown(signal.name, stem);
		stop.point = signal.position;
		stop.travel = glm::dvec3{travel.x, 0.0, travel.y};
		stops.push_back(std::move(stop));
	}
	m_track_lengths.measure(stops);
}

void srk_tool::track_lengths::measure(std::vector<station::signal> const &Signals)
{
	m_measured = true;
	m_lengths.clear();
	std::vector<station::numbered> tracks;
	std::unordered_set<TTrack const *> inside;
	m_numbered = m_tool.m_layout.has_value() && m_tool.station_tracks(tracks, inside);
	if (false == m_numbered)
		return;
	m_lengths = station::track_lengths(*m_tool.m_layout, tracks, Signals);
	std::ranges::sort(m_lengths, [](station::lengths const &A, station::lengths const &B) {
		return std::make_pair(std::atoi(A.number.c_str()), A.number) < std::make_pair(std::atoi(B.number.c_str()), B.number);
	});
}

double srk_tool::set_speed_of_open() const
{
	if (false == m_layout.has_value())
		return 0.0;
	auto const highest = [](auto const &Tracks) {
		auto speed{0.0};
		for (auto *track : Tracks)
			if (auto const set{track->VelocityGet()}; track->eType == tt_Normal && set <= kFastest)
				speed = std::max(speed, set);
		return speed;
	};
	std::vector<TTrack *> through{m_layout->main};
	through.insert(through.end(), m_layout->second.begin(), m_layout->second.end());
	for (auto const &path : m_layout->paths)
		if (path.approach)
			through.push_back(path.track);
	if (auto const speed{highest(through)}; speed > 0.0)
		return speed;
	std::vector<TTrack *> all;
	for (auto const &path : m_layout->paths)
		all.push_back(path.track);
	return highest(all);
}

double srk_tool::speed_of_open() const
{
	if (auto const speed{set_speed_of_open()}; speed > 0.0)
		return speed;
	auto const set{m_record >= 0 ? m_records[m_record].braking : 0};
	if (set <= 0)
		return 160.0;
	if (set <= 400)
		return 60.0;
	if (set <= 700)
		return 100.0;
	if (set <= 1000)
		return 140.0;
	return 160.0;
}

double srk_tool::braking_of_open() const
{
	auto const set{m_record >= 0 ? m_records[m_record].braking : 0};
	return set > 0 ? static_cast<double>(set) : braking_distance(speed_of_open());
}

namespace
{

// a signal a route leads to added to the order of the heads checked: where it isn't in it yet, isn't a main signal and has routes
void order_taken(int const Index, std::vector<station::post> const &Posts, std::vector<int> const &Routes_of, std::vector<char> &Checked, std::vector<std::size_t> &Order)
{
	if (Index < 0 || 0 != Checked[Index] || Posts[Index].kind == station::signal::role::entry || Routes_of[Index] < 0)
		return;
	Checked[Index] = 1;
	Order.push_back(static_cast<std::size_t>(Index));
}

// the order the heads are checked in: the signals of the station, Own, those their routes lead to, then those whose routes lead to any
// of these. Found: the routes of the signals, Routes_of: of each post, an index into Found
std::vector<std::size_t> heads_order(std::vector<station::post> const &Posts, std::vector<char> const &Own, std::vector<std::vector<station::route>> const &Found, std::vector<int> const &Routes_of)
{
	std::vector<char> checked(Own);
	std::vector<std::size_t> order;
	for (std::size_t index = 0; index < Posts.size(); ++index)
		if (Own[index])
			order.push_back(index);
	// the order grows while it's gone through
	std::size_t at{0};
	while (at < order.size())
	{
		for (auto const &route : Found[Routes_of[order[at]]])
			for (auto const index : {route.warning, route.next})
				order_taken(index, Posts, Routes_of, checked, order);
		++at;
	}
	for (auto grown{true}; grown;)
	{
		grown = false;
		for (std::size_t index = 0; index < Posts.size(); ++index)
		{
			if (checked[index] || Routes_of[index] < 0 || Posts[index].kind == station::signal::role::entry)
				continue;
			auto const &routes{Found[Routes_of[index]]};
			if (std::ranges::none_of(routes, [&](station::route const &Route) { return Route.next >= 0 && checked[Route.next]; }))
				continue;
			checked[index] = 1;
			order.push_back(index);
			grown = true;
		}
	}
	return order;
}

// how far before a signal its SHP resonator is wanted, Kind: of its post, Own: it's a signal of the station. Before the block signals
// and the warning discs; before the entry signals of the station facing into it, at the main signals with a route to a signal no warning
// disc tells of; left as it is for the others
void shp_wanted(srk_tool::head_check &Check, station::layout const &Layout, station::signal::role const Kind, bool const Own)
{
	glm::dvec3 const travel{Check.travel.x, 0.0, Check.travel.y};
	if (Kind != station::signal::role::entry)
		Check.shp = kShpBefore;
	else if (Own)
	{
		auto const along{station::project(Layout, Check.position).x};
		auto const ahead{station::project(Layout, Check.position + travel * 10.0).x};
		if ((along < Layout.from && ahead > along) || (along > Layout.to && ahead < along))
			Check.shp = kShpBefore;
		else if (std::ranges::any_of(Check.routes, [](station::route const &Route) { return Route.next >= 0 && Route.warning < 0; }))
			Check.shp = 0.0;
	}
}

// the problems of a warning disc: no main signal ahead of it to tell of, one nearer than the braking distance
void warning_problems(srk_tool::head_check &Check, double const Braking)
{
	for (std::size_t k = 0; k < Check.routes.size(); ++k)
	{
		auto const &route{Check.routes[k]};
		if (route.next < 0)
			Check.problems.push_back(format(STR_C("No main signal within %.0f m ahead of it to tell of"), route.length));
		else if (route.length < Braking - 1.0)
			Check.problems.push_back(format(STR_C("It stands %.0f m before %s, less than the braking distance of %.0f m: the trains don't stop by the signal in time"), route.length, Check.nexts[k].c_str(), Braking));
	}
}

// the problems of a block signal, Template: its own, if known: the block sections shorter than the braking distance, than half of it in
// the 4-aspect block
void block_problems(srk_tool::head_check &Check, signal_head const *Template, double const Braking)
{
	for (std::size_t k = 0; k < Check.routes.size(); ++k)
	{
		auto const &route{Check.routes[k]};
		if (route.next < 0 || route.warning >= 0)
			continue;
		if (Template != nullptr && Template->aspects == 4)
		{
			if (route.length < Braking * 0.5 - 1.0)
				Check.problems.push_back(format(STR_C("The block section to %s is %.0f m long, shorter than half the braking distance of %.0f m, of the 4-aspect block"), Check.nexts[k].c_str(), route.length, Braking));
		}
		else if (route.length < Braking - 1.0)
			Check.problems.push_back(format(STR_C("The block section to %s is %.0f m long, shorter than the braking distance of %.0f m, Ie-4 § 8 ust. 1"), Check.nexts[k].c_str(), route.length, Braking));
	}
}

// whether a head, Template: its own, shows what it needs, the template it's best changed to and whether that one shows less beyond
void head_fitted(srk_tool::head_check &Check, std::vector<signal_head> const &Heads, std::string_view const Kind, signal_head const &Template, head_need const &Need)
{
	Check.fits = shows(Template, Need);
	auto const best{best_head(Heads, Kind, Template.mount, Template.name, Template.aspects, Need, Template.arms > 0)};
	if (best < 0)
		return;
	Check.fit = Heads[best].file;
	Check.more = Check.fits && surplus(Heads[best], Need) < surplus(Template, Need);
}

using head_files = std::unordered_map<std::string, std::size_t, string_hash, std::equal_to<>>;

// the posts of the signals which stand by a track within the reach of a line of a point; Owners: of each, the index of its signal
std::vector<station::post> posts_near(std::vector<standing_signal> const &Standing, glm::dvec3 const &Middle, std::vector<std::size_t> &Owners)
{
	std::vector<station::post> posts;
	for (std::size_t k = 0; k < Standing.size(); ++k)
	{
		if (auto const within{geometry::plan_distance(Standing[k].position, Middle) <= kLineLimit}; false == within)
			continue;
		if (auto const post{post_of(Standing[k])}; post.track != nullptr)
		{
			posts.push_back(post);
			Owners.push_back(k);
		}
	}
	return posts;
}

// the signals the routes are found from: the main signals of the station, marked in Own, the block signals and the warning discs of
// the lines in and out of it
std::vector<std::size_t> signals_from(std::vector<station::post> const &Posts, station::layout const &Layout, std::vector<char> &Own)
{
	using enum station::signal::role;
	std::vector<std::size_t> from;
	for (std::size_t index = 0; index < Posts.size(); ++index)
	{
		if (Posts[index].kind == entry && gap_to(Layout, Posts[index].point) <= 8.0)
			Own[index] = 1;
		if (Own[index] || Posts[index].kind == block || Posts[index].kind == warning)
			from.push_back(index);
	}
	return from;
}

// the names of the signals the routes lead to, shown; empty for a route to none
std::vector<std::string> nexts_shown(std::vector<station::route> const &Routes, std::vector<standing_signal> const &Standing, std::vector<std::size_t> const &Owners, std::string_view const Stem)
{
	std::vector<std::string> nexts;
	for (auto const &route : Routes)
		nexts.push_back(route.next >= 0 ? signal_shown(Standing[Owners[route.next]].name, Stem) : std::string{});
	return nexts;
}

// the semaphores described with their arms the routes lead to, none for a route to another signal or to none
std::vector<signal_head const *> semaphores_ahead(std::vector<station::route> const &Routes, std::vector<signal_head> const &Heads, head_files const &By_file, std::vector<standing_signal> const &Standing, std::vector<std::size_t> const &Owners)
{
	std::vector<signal_head const *> mains;
	for (auto const &route : Routes)
	{
		auto const ahead{route.next >= 0 ? By_file.find(Standing[Owners[route.next]].file) : By_file.end()};
		mains.push_back(ahead != By_file.end() && Heads[ahead->second].arms > 0 ? &Heads[ahead->second] : nullptr);
	}
	return mains;
}

// whether a head is listed before another: those of the station first, by their chainage, the others by how far from its middle
bool listed_before(srk_tool::head_check const &A, srk_tool::head_check const &B, station::layout const &Layout, glm::dvec3 const &Middle)
{
	if (A.station != B.station)
		return A.station;
	if (A.station)
		return station::project(Layout, A.position).x < station::project(Layout, B.position).x;
	return geometry::plan_distance(A.position, Middle) < geometry::plan_distance(B.position, Middle);
}

// what the template of the head has taken onto its check: its lamps, its arms, the names of its aspects, its speeds; nothing for none
void take_template(auto &Check, signal_head const *Template)
{
	if (Template == nullptr)
		return;
	Check.lamps = Template->lamps;
	Check.arms = Template->arms;
	Check.aspect_names = Template->aspect_names;
	Check.own = Template->speeds;
}

// a shaped disc before a semaphore described with its arms: the events it repeats tell apart all the semaphore shows. One not
// described so may as well, it is offered the one described as a hint and the fitting leaves it be. A disc with lights isn't
// checked, nor one whose template isn't among the scanned, its mount not known
void disc_fitted(auto &Check, std::vector<signal_head> const &Heads, head_files const &By_file, std::vector<standing_signal> const &Standing, std::vector<std::size_t> const &Owners, signal_head const *Template,
                 std::string const &Mount)
{
	auto const described{Template != nullptr && false == Template->repeats.empty()};
	if (Mount.empty() || false == (described || Template == nullptr || Template->lamps.empty()))
		return;
	auto disc{check_disc(Heads, By_file, semaphores_ahead(Check.routes, Heads, By_file, Standing, Owners), Template, Check.file, Mount)};
	Check.told = disc.told;
	Check.unsaid = disc.unsaid;
	Check.fits = disc.fits;
	Check.fit = std::move(disc.fit);
}

} // namespace

void srk_tool::head_checks::check_heads(std::vector<standing_signal> const &Standing)
{
	using enum station::signal::role;
	m_heads.clear();
	std::vector<signal_head> heads;
	m_heads_known = m_tool.m_layout.has_value() && m_tool.m_editor.signals.signal_heads(heads, false);
	if (false == m_heads_known)
		return;
	auto const &layout{*m_tool.m_layout};
	auto const stem{m_tool.stem_of_open()};
	auto const line{m_tool.speed_of_open()};
	auto const braking{m_tool.braking_of_open()};
	auto const middle{axis_point(layout, (layout.from + layout.to) * 0.5)};
	std::vector<std::size_t> owners;
	auto const posts{posts_near(Standing, middle, owners)};
	std::vector<char> own(posts.size(), 0);
	auto const from{signals_from(posts, layout, own)};
	auto const found{station::routes(posts, from, route_reach(braking))};
	std::vector routes_of(posts.size(), -1);
	for (std::size_t k = 0; k < from.size(); ++k)
		routes_of[from[k]] = static_cast<int>(k);
	auto const order{heads_order(posts, own, found, routes_of)};
	head_files by_file;
	for (std::size_t index = 0; index < heads.size(); ++index)
		by_file.try_emplace(heads[index].file, index);
	auto const triggers{shp_triggers()};
	for (auto const index : order)
	{
		auto const &signal{Standing[owners[index]]};
		auto const &post{posts[index]};
		head_check check;
		check.instance = signal.instance;
		check.name = signal_shown(signal.name, stem);
		check.kind = signal.kind;
		check.position = signal.position;
		check.travel = travel_of(signal.yaw);
		check.file = signal.file;
		check.station = own[index] != 0;
		check.routes = found[routes_of[index]];
		check.nexts = nexts_shown(check.routes, Standing, owners, stem);
		auto const known{by_file.find(signal.file)};
		auto const *template_of{known != by_file.end() ? &heads[known->second] : nullptr};
		take_template(check, template_of);
		glm::dvec3 const travel{check.travel.x, 0.0, check.travel.y};
		shp_wanted(check, layout, post.kind, own[index] != 0);
		if (check.shp >= 0.0)
		{
			auto const way{station::approach(*post.track, post.point, post.facing, check.shp + kShpWindow)};
			check.shp_found = shp_before(way, signal.position, travel, triggers, check.shp);
			check.shp_short = way.empty() || way.back().distance < check.shp - kShpTolerance;
		}
		if (post.kind == warning)
		{
			warning_problems(check, braking);
			disc_fitted(check, heads, by_file, Standing, owners, template_of, m_tool.m_editor.signals.signal_mount("warning", signal.file));
			m_heads.push_back(std::move(check));
			continue;
		}
		auto const main{post.kind == entry};
		auto const need{need_of(check.routes, main, line, braking)};
		check.speeds = need.speeds;
		check.informs = need.informs;
		check.white = need.white;
		check.shortened = need.shortened;
		check.indicator = need.indicator;
		if (false == main)
			block_problems(check, template_of, braking);
		if (template_of != nullptr)
			head_fitted(check, heads, signal.kind, *template_of, need);
		m_heads.push_back(std::move(check));
	}
	std::ranges::stable_sort(m_heads, [&layout, &middle](head_check const &A, head_check const &B) { return listed_before(A, B, layout, middle); });
}

void srk_tool::head_checks::fit_heads()
{
	m_tool.m_status.clear();
	signal_changes changes;
	// a semaphore described having what it needs keeps its arms, for the events written by hand to the aspects of a second one
	for (auto const &head : m_heads)
		if (false == head.fit.empty() && head.fit != head.file && (false == head.fits || (head.more && head.arms == 0)))
			changes.files.emplace_back(head.instance, head.fit);
	if (changes.files.empty())
	{
		m_tool.m_status = std::ranges::any_of(m_heads, [](head_check const &Head) { return false == Head.fits; }) ? STR_C("No template has what the signals marked need") : STR_C("The signals have what they need already");
		return;
	}
	std::string error;
	m_tool.m_editor.signals.signal_batch(changes, {}, {}, error);
	if (false == error.empty())
	{
		m_tool.m_status = format(STR_C("The signals couldn't be changed: %s"), error.c_str());
		return;
	}
	std::vector<standing_signal> standing;
	m_tool.m_editor.signals.standing_signals(standing, true);
	std::size_t changed{0};
	for (auto const &head : m_heads)
		changed += static_cast<std::size_t>(std::ranges::count_if(standing, [&](standing_signal const &Signal) { return Signal.instance == head.instance && Signal.file != head.file; }));
	if (changed == 0)
		m_tool.m_status = STR_C("The signals have what they need already");
	else
		m_tool.m_status = format(STR_P("%zu signal got what it needs, Ctrl+Z takes it back", "%zu signals got what they need, Ctrl+Z takes it back", changed), changed);
}

namespace
{

using event_names = std::unordered_set<std::string, string_hash, std::equal_to<>>;

// where the SHP resonator of a signal is placed
struct shp_placing
{
	std::string signal; // shown
	std::string name; // of the resonator, its event is <name>_shp
	TTrack *track{nullptr}; // the trains run into its event on, where a joint of the tracks lies
	TTrack *cut{nullptr}; // the track cut at the point, else
	int facing{1}; // the trains run on the track, 1: towards the end of its path
	glm::dvec3 point{0.0};
	glm::dvec3 way{0.0, 0.0, 1.0};
	glm::dvec3 inside{0.0}; // of the track the trains run into, a metre on from the point
};

// the names of the events of the tracks of the scenery, in lower case
event_names scenery_events()
{
	event_names taken;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		for (int list = 0; list < 6; ++list)
			for (auto const &[event_name, event_target] : editor_track::events(*track, list))
				taken.insert(ToLower(event_name));
	}
	return taken;
}

// where the SHP resonator of a head which misses it goes, a name not taken yet given to it, Taken: the names of the events taken;
// none, the problem added to Problems, where it can't go
std::optional<shp_placing> shp_place(srk_tool::head_check const &Head, std::vector<standing_signal> const &Standing, event_names &Taken, std::vector<std::string> &Problems)
{
	if (Head.shp_of() == srk_tool::shp_state::beyond)
	{
		Problems.push_back(format(STR_C("%s: the tracks before it end short of %.0f m"), Head.name.c_str(), Head.shp));
		return std::nullopt;
	}
	auto const found{std::ranges::find_if(Standing, [&](standing_signal const &Signal) { return Signal.instance == Head.instance; })};
	if (found == Standing.end())
		return std::nullopt;
	auto const post{post_of(*found)};
	if (post.track == nullptr)
	{
		Problems.push_back(format(STR_C("%s stands by no track"), Head.name.c_str()));
		return std::nullopt;
	}
	auto const way{station::approach(*post.track, post.point, post.facing, Head.shp + kShpTolerance + 1.0)};
	if (way.empty() || way.back().distance < Head.shp - kShpTolerance)
	{
		Problems.push_back(format(STR_C("%s: the tracks before it end short of %.0f m"), Head.name.c_str(), Head.shp));
		return std::nullopt;
	}
	auto joint{false};
	auto const best{resonator_at(way, Head.shp, joint)};
	if (best < 0)
	{
		Problems.push_back(format(STR_C("%s: no plain track within %.0f m of the place of its SHP resonator"), Head.name.c_str(), kShpTolerance));
		return std::nullopt;
	}
	auto const &at{way[best]};
	if (std::string reason; false == scene::Layers.editable(at.track, &reason) || (false == joint && false == editor_track::can_edit_geometry(*at.track, reason)))
	{
		Problems.push_back(format(STR_C("%s can't be changed: %s"), name_of(*at.track).c_str(), reason.c_str()));
		return std::nullopt;
	}
	if (false == at.track->Dynamics.empty())
	{
		Problems.push_back(format(STR_C("%s: vehicles stand on %s"), Head.name.c_str(), name_of(*at.track).c_str()));
		return std::nullopt;
	}
	shp_placing entry;
	auto name{found->name + "_rez"};
	for (int number = 2; Taken.contains(ToLower(name) + "_shp") && number < 100; ++number)
		name = std::format("{}_rez{}", found->name, number);
	Taken.insert(ToLower(name) + "_shp");
	entry.signal = Head.name;
	entry.name = name;
	entry.facing = at.facing;
	entry.point = at.point;
	entry.way = at.way;
	if (joint)
		entry.track = at.track;
	else
	{
		entry.cut = at.track;
		entry.inside = best > 0 ? way[best - 1].point : at.point + at.way;
	}
	return entry;
}

// the tracks changed placing the SHP resonators: how they were before, the pieces cut off them
struct shp_edit
{
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::vector<TTrack *> created;
	std::unordered_set<TTrack const *> captured;
	std::unordered_map<TTrack const *, std::vector<TTrack *>> pieces; // of each track cut, itself and the pieces cut off it

	// how a track is before it's changed, kept the first time only
	void capture(TTrack *Track)
	{
		if (captured.insert(Track).second)
			before.emplace_back(Track, editor_track::capture(*Track));
	}

	// the piece of a track nearest a point; the track itself where it isn't cut
	TTrack *piece_at(TTrack *Track, glm::dvec3 const &Point) const
	{
		auto found{pieces.find(Track)};
		if (found == pieces.end())
			return Track;
		TTrack *nearest{Track};
		auto gap{std::numeric_limits<double>::max()};
		for (auto *piece : found->second)
			if (auto const distance{glm::distance(editor_track::point_at(*piece, editor_track::nearest_parameter(*piece, Point)), Point)}; distance < gap)
			{
				gap = distance;
				nearest = piece;
			}
		return nearest;
	}

	// the tracks cut at the places of the resonators which aren't at a joint; how many are cut
	std::size_t cut(std::vector<shp_placing> const &Places)
	{
		std::size_t cuts{0};
		for (auto const &entry : Places)
		{
			if (entry.cut == nullptr)
				continue;
			auto *track{piece_at(entry.cut, entry.point)};
			if (end_gap(*track, entry.point) <= 0.3)
				continue;
			capture(track);
			auto *piece{editor_track::split_path(*track, editor_track::nearest_parameter(*track, entry.point))};
			if (piece == nullptr)
				continue;
			created.push_back(piece);
			captured.insert(piece);
			auto &list{pieces[entry.cut]};
			if (list.empty())
				list.push_back(entry.cut);
			list.push_back(piece);
			++cuts;
		}
		return cuts;
	}
};

// the resonator of a signal as placed: below the place of its event, turned against the trains
template_item resonator_item(shp_placing const &Entry)
{
	template_item item;
	item.file = kResonator;
	item.location = Entry.point - glm::dvec3{0.0, kResonatorDrop, 0.0};
	auto const yaw{geometry::heading_of({Entry.way.x, Entry.way.z}) + 180.0};
	item.yaw = yaw >= 360.0 ? yaw - 360.0 : yaw;
	item.values["name"] = Entry.name;
	return item;
}

// the resonator placed: the track cut at its place reads its event; false where the track couldn't be cut there
bool resonator_placed(shp_edit &Edit, shp_placing const &Entry, signal_changes &Changes)
{
	auto *track{Entry.track != nullptr ? Edit.piece_at(Entry.track, Entry.point) : Edit.piece_at(Entry.cut, Entry.inside)};
	// the trains run into the track at the point: at the start of its path running towards its end
	if (glm::distance(track->m_paths.front().points[Entry.facing > 0 ? segment_data::point::start : segment_data::point::end], Entry.point) > 1.0)
		return false;
	Edit.capture(track);
	editor_track::events(*track, Entry.facing > 0 ? 2 : 1).emplace_back(ToLower(Entry.name) + "_shp", nullptr);
	std::string missing;
	editor_track::bind_events(*track, missing);
	track->mark_dirty();
	Changes.others.push_back(resonator_item(Entry));
	return true;
}

} // namespace

void srk_tool::head_checks::set_shp()
{
	m_tool.m_status.clear();
	if (false == m_tool.m_layout.has_value())
		return;
	include_info resonator;
	if (std::string unreadable; false == editor_includes::load(kResonator, resonator, unreadable))
	{
		m_tool.m_status = format(STR_C("%s isn't in the scenery folder: the SHP resonators can't be placed"), kResonator);
		return;
	}
	std::vector<standing_signal> standing;
	m_tool.m_editor.signals.standing_signals(standing, true);
	check_heads(standing);
	auto taken{scenery_events()};
	std::vector<shp_placing> places;
	std::vector<std::string> problems;
	for (auto const &head : m_heads)
	{
		if (head.shp_of() != shp_state::missing && head.shp_of() != shp_state::beyond)
			continue;
		if (auto entry{shp_place(head, standing, taken, problems)}; entry.has_value())
			places.push_back(std::move(*entry));
	}
	if (places.empty())
	{
		m_tool.m_status = problems.empty() ? STR_C("The signals have their SHP resonators already") : problems.front();
		return;
	}
	shp_edit edit;
	auto const cuts{edit.cut(places)};
	signal_changes changes;
	for (auto const &entry : places)
		if (false == resonator_placed(edit, entry, changes))
			problems.push_back(format(STR_C("%s: the track couldn't be cut at the place of its SHP resonator"), entry.signal.c_str()));
	std::string error;
	auto const placed{changes.others.size()};
	m_tool.m_editor.signals.signal_batch(changes, std::move(edit.before), std::move(edit.created), error);
	if (false == error.empty())
	{
		m_tool.m_status = format(STR_C("The SHP resonators couldn't be placed: %s. Ctrl+Z takes back the tracks cut"), error.c_str());
		return;
	}
	m_tool.m_status = format(STR_P("%zu SHP resonator placed", "%zu SHP resonators placed", placed), placed) + ", " + format(STR_P("%zu track cut", "%zu tracks cut", cuts), cuts) + ". " + STR("Ctrl+Z takes it back, the resonators work after the scenery is saved and opened again");
	for (auto const &problem : problems)
		m_tool.m_status += "\n" + problem;
}

namespace
{

// a mark of a track circuit put on its track, on the point of it nearest the middle of its points: there in the schematic, in the
// scenery and in the plot
template <typename Mark>
void circuit_placed(Mark &Circuit, int const Index, station::layout const &Layout, std::vector<int> const &Circuit_of, station::drawing const &Plot)
{
	auto nearest{std::numeric_limits<double>::max()};
	glm::dvec2 on{Circuit.at};
	for (std::size_t path = 0; path < Layout.paths.size(); ++path)
	{
		if (Circuit_of[path] != Index)
			continue;
		for (std::size_t k = 0; k < Layout.paths[path].schematic.size(); ++k)
			if (auto const distance{glm::distance(Layout.paths[path].schematic[k], Circuit.at)}; distance < nearest)
			{
				nearest = distance;
				on = Layout.paths[path].schematic[k];
				Circuit.world = Layout.paths[path].points[k];
				Circuit.drawn = Plot.paths[path][k];
			}
	}
	Circuit.at = on;
}

// the name of a track circuit as the plot shows it: bpd_it3 as It3, bpd_ita, Entry: before the entry signal A, as ItA
void circuit_shown(std::string &Text, std::string const &Stem, bool const Entry)
{
	if (false == Stem.empty() && Text.compare(0, Stem.size(), Stem) == 0)
		Text.erase(0, Stem.size());
	if (false == Text.empty())
		Text.front() = static_cast<char>(std::toupper(static_cast<unsigned char>(Text.front())));
	if (Entry && Text.size() > 2)
		std::transform(Text.begin() + 2, Text.end(), Text.begin() + 2, [](char const Letter) { return static_cast<char>(std::toupper(static_cast<unsigned char>(Letter))); });
}

// an end of a path of the layout: where, its way out, the circuit of the path, where in the plot
struct circuit_end
{
	glm::dvec3 point;
	glm::dvec3 way;
	int circuit;
	glm::dvec2 drawn;
};

// both ends of each path of the layout of two points or more
std::vector<circuit_end> circuit_ends(station::layout const &Layout, std::vector<int> const &Circuit_of, station::drawing const &Plot)
{
	std::vector<circuit_end> ends;
	for (std::size_t index = 0; index < Layout.paths.size(); ++index)
	{
		auto const &points{Layout.paths[index].points};
		if (points.size() < 2)
			continue;
		auto const circuit{Circuit_of[index]};
		ends.emplace_back(points.front(), points[1] - points.front(), circuit, Plot.paths[index].front());
		ends.emplace_back(points.back(), points.back() - points[points.size() - 2], circuit, Plot.paths[index].back());
	}
	return ends;
}

} // namespace

void srk_tool::schematic::survey()
{
	m_circuit_of.assign(m_tool.m_layout.has_value() ? m_tool.m_layout->paths.size() : 0, -1);
	m_circuits.clear();
	m_ticks.clear();
	m_ticks_drawn.clear();
	m_tool.m_track_lengths.m_lengths.clear();
	m_tool.m_track_lengths.m_measured = false;
	m_tool.m_track_lengths.m_numbered = false;
	if (false == m_tool.m_layout.has_value())
		return;
	auto const &layout{*m_tool.m_layout};
	auto const stem{m_tool.stem_of_open()};
	std::vector<double> weights;
	for (std::size_t index = 0; index < layout.paths.size(); ++index)
	{
		auto const &path{layout.paths[index]};
		if (path.track->Isolated.empty())
			continue;
		// of the lines leading out only the stretches up to the joints of the station
		auto const &name{path.track->Isolated.front()->asName};
		if (path.approach && (stem.empty() || name.compare(0, stem.size(), stem) != 0))
			continue;
		auto found{std::ranges::find_if(m_circuits, [&](circuit_mark const &Mark) { return Mark.text == name; })};
		if (found == m_circuits.end())
		{
			found = m_circuits.emplace(m_circuits.end(), glm::dvec2{0.0}, glm::dvec3{0.0}, name, circuit_colour(name));
			found->labelled = stem.empty() || name.compare(0, stem.size() + 2, stem + "iz") != 0;
			weights.push_back(0.0);
		}
		auto const circuit{static_cast<std::size_t>(found - m_circuits.begin())};
		m_circuit_of[index] = static_cast<int>(circuit);
		for (auto const &point : path.schematic)
		{
			found->at += point;
			weights[circuit] += 1.0;
		}
		found->length += path.length;
	}
	for (std::size_t index = 0; index < m_circuits.size(); ++index)
	{
		auto &mark{m_circuits[index]};
		mark.at /= std::max(1.0, weights[index]);
		circuit_placed(mark, static_cast<int>(index), layout, m_circuit_of, m_tool.m_plot);
		auto const entry{entry_circuit(mark.text, stem)};
		circuit_shown(mark.text, stem, entry);
	}
	survey_joints(layout);
	m_tool.survey_signals();
}

// the joints: the ends where paths of different circuits meet
void srk_tool::schematic::survey_joints(station::layout const &Layout)
{
	auto const ends{circuit_ends(Layout, m_circuit_of, m_tool.m_plot)};
	m_joints.clear();
	for (std::size_t i = 0; i < ends.size(); ++i)
		for (std::size_t j = i + 1; j < ends.size(); ++j)
			if (ends[i].circuit != ends[j].circuit && (ends[i].circuit >= 0 || ends[j].circuit >= 0) && glm::distance(ends[i].point, ends[j].point) < 0.05 &&
			    std::ranges::none_of(m_joints, [&](joint_mark const &Joint) { return glm::distance(Joint.at, ends[i].point) < 0.05; }))
			{
				m_ticks.push_back(station::project(Layout, ends[i].point));
				m_ticks_drawn.push_back(ends[i].drawn);
				glm::dvec2 const way{glm::normalize(glm::dvec2{ends[i].way.x, ends[i].way.z} + glm::dvec2{1e-12, 0.0})};
				m_joints.emplace_back(ends[i].point, glm::dvec3{-way.y, 0.0, way.x});
			}
}

void srk_tool::schematic::fit_view(float const Width, float const Height)
{
	auto const &layout{*m_tool.m_layout};
	auto &view{m_view_state};
	// the switches and the signals out on the lines
	auto from{layout.from};
	auto to{layout.to};
	for (auto const &mark : m_signal_marks)
	{
		from = std::min(from, mark.at.x);
		to = std::max(to, mark.at.x);
	}
	if (m_fit_lines)
		for (auto const &row : m_tool.m_line_survey.m_lines)
		{
			from = std::min(from, row.chainage + row.outward * row.length);
			to = std::max(to, row.chainage + row.outward * row.length);
		}
	from = m_tool.m_plot.x(from);
	to = m_tool.m_plot.x(to);
	auto const span{std::max(50.0, to - from)};
	view.from = from - span * 0.04 - 30.0;
	view.to = to + span * 0.04 + 30.0;
	auto low{std::numeric_limits<double>::max()};
	auto high{std::numeric_limits<double>::lowest()};
	for (std::size_t index = 0; index < layout.paths.size() && index < m_tool.m_plot.paths.size(); ++index)
		if (false == layout.paths[index].approach)
			for (auto const &point : m_tool.m_plot.paths[index])
			{
				low = std::min(low, point.y);
				high = std::max(high, point.y);
			}
	if (low > high)
	{
		low = 0.0;
		high = 0.0;
	}
	view.centre = (low + high) * 0.5;
	auto const across{(view.to - view.from) / std::max(1.0f, Width)};
	auto const vertical{(high - low + 2.0 * station::row_height) / std::max(1.0f, Height * 0.85f)};
	view.exaggeration = static_cast<float>(std::clamp(across / vertical, 1.0, 500.0));
}

namespace
{

// the square of the distance of the mouse on the screen from the nearest of the lines between the points of a path drawn, Screen_of:
// where a point of the plot is on the screen; Limit where none is nearer
template <typename Screen>
float drawn_gap2(std::vector<glm::dvec2> const &Points, Screen const &Screen_of, ImVec2 const &Mouse, float const Limit)
{
	auto nearest{Limit};
	for (std::size_t i = 1; i < Points.size(); ++i)
	{
		auto const a{Screen_of(Points[i - 1])};
		auto const b{Screen_of(Points[i])};
		auto const dx{b.x - a.x};
		auto const dy{b.y - a.y};
		auto const length2{dx * dx + dy * dy};
		auto const t{length2 > 0.0f ? std::clamp(((Mouse.x - a.x) * dx + (Mouse.y - a.y) * dy) / length2, 0.0f, 1.0f) : 0.0f};
		auto const ex{a.x + t * dx - Mouse.x};
		auto const ey{a.y + t * dy - Mouse.y};
		if (ex * ex + ey * ey < nearest)
			nearest = ex * ex + ey * ey;
	}
	return nearest;
}

// the joints at the ends of a block section of a line straightened, but at the ends of the line; X_of: where a distance along the line
// is on the screen
template <typename Line_x>
void joints_drawn(ImDrawList *Draw, srk_tool::line_row const &Row, srk_tool::line_row::run const &Run, float const Y, Line_x const &X_of)
{
	for (auto const end : {Run.from, Run.to})
		if (end > 1.0 && end < Row.length - 1.0)
			Draw->AddLine(ImVec2(X_of(Row, end), Y - 5.0f), ImVec2(X_of(Row, end), Y + 5.0f), kJoint, 2.0f);
}

// a useful length drawn along its line on the screen: a tick at its rear end, an arrowhead where the trains stop
void useful_drawn(ImDrawList *Draw, std::vector<ImVec2> const &Line)
{
	Draw->AddPolyline(Line.data(), static_cast<int>(Line.size()), kUseful, ImDrawFlags_None, 2.0f);
	auto const &rear_end{Line.front()};
	auto const &head{Line.back()};
	Draw->AddLine(ImVec2(rear_end.x, rear_end.y - 4.0f), ImVec2(rear_end.x, rear_end.y + 4.0f), kUseful, 2.0f);
	auto const behind{Line.size() > 1 ? Line.size() - 2 : 0};
	auto const way_x{head.x >= Line[behind].x ? 1.0f : -1.0f};
	Draw->AddTriangleFilled(ImVec2(head.x - 7.0f * way_x, head.y - 4.5f), ImVec2(head.x - 7.0f * way_x, head.y + 4.5f), ImVec2(head.x + 1.0f * way_x, head.y), kUseful);
}

} // namespace

// where the schematic is drawn on the canvas: its corner and size, the metres of the plot across a pixel and up one, the view, the list
// drawn into. The plot draws the generalized schematic: its x shortens the chainage between the switches, its height puts the tracks on
// rows
struct srk_tool::canvas_frame
{
	station::drawing const &plot;
	view const &shown;
	ImVec2 corner;
	ImVec2 size;
	double across{1.0};
	double vertical{1.0};
	ImDrawList *draw{nullptr};

	float plot_x(double const X) const
	{
		return corner.x + static_cast<float>((X - shown.from) / across);
	}
	float plot_y(double const Elevation) const
	{
		return corner.y + size.y * 0.5f - static_cast<float>((Elevation - shown.centre) / vertical);
	}
	float x_of(double const Chainage) const
	{
		return plot_x(plot.x(Chainage));
	}
	ImVec2 screen_of(glm::dvec2 const &Point) const
	{
		return ImVec2(plot_x(Point.x), plot_y(Point.y));
	}
	double chainage_at(float const Mouse) const
	{
		return plot.chainage(shown.from + (Mouse - corner.x) * across);
	}
	std::vector<glm::dvec2> const &drawn_of(std::size_t const Index) const
	{
		static std::vector<glm::dvec2> const nothing;
		return Index < plot.paths.size() ? plot.paths[Index] : nothing;
	}
	// a line out of the station, how far out along it
	float line_x(line_row const &Row, double const Distance) const
	{
		return x_of(Row.chainage + Row.outward * Distance);
	}
	void path(std::size_t const Index, ImU32 const Colour, float const Width) const
	{
		auto const &points{drawn_of(Index)};
		for (std::size_t i = 1; i < points.size(); ++i)
			draw->AddLine(screen_of(points[i - 1]), screen_of(points[i]), Colour, Width);
	}
};

// the labels drawn on the canvas, none over another: each at the first of its places where it covers none placed before
struct srk_tool::canvas_labels
{
	ImDrawList *draw{nullptr};
	std::vector<ImVec4> placed;

	void label(std::string const &Text, ImU32 const Colour, std::initializer_list<ImVec2> Places)
	{
		auto const extent{ImGui::CalcTextSize(Text.c_str())};
		for (auto const &place : Places)
		{
			ImVec4 const box{place.x - extent.x * 0.5f - 2.0f, place.y - 1.0f, place.x + extent.x * 0.5f + 2.0f, place.y + extent.y + 1.0f};
			if (std::ranges::any_of(placed, [&box](ImVec4 const &Other) { return box.x < Other.z && Other.x < box.z && box.y < Other.w && Other.y < box.w; }))
				continue;
			placed.push_back(box);
			draw->AddRectFilled(ImVec2(box.x, box.y), ImVec2(box.z, box.w), IM_COL32(22, 24, 28, 200), 2.0f);
			draw->AddText(ImVec2(place.x - extent.x * 0.5f, place.y), Colour, Text.c_str());
			return;
		}
	}
};

void srk_tool::schematic::render_canvas(float const Height)
{
	auto &view{m_view_state};
	auto const &io{ImGui::GetIO()};
	ImVec2 const size{std::max(200.0f, ImGui::GetContentRegionAvail().x), std::max(80.0f, Height)};
	ImVec2 const corner{ImGui::GetCursorScreenPos()};
	ImGui::InvisibleButton("##srkcanvas", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonMiddle);
	bool const hovered{ImGui::IsItemHovered()};
	if (view.exaggeration <= 0.0f)
		fit_view(size.x, size.y);
	// the wheel zooms about the mouse, with Ctrl it exaggerates the offsets; a drag moves the plot
	if (hovered && io.MouseWheel != 0.0f)
		canvas_wheel(corner, size);
	auto const across{(view.to - view.from) / size.x};
	auto const vertical{across / view.exaggeration};
	if (ImGui::IsItemActive() && (ImGui::IsMouseDragging(ImGuiMouseButton_Left, kDragPixels) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle, kDragPixels)))
	{
		m_dragging = true;
		view.from -= io.MouseDelta.x * across;
		view.to -= io.MouseDelta.x * across;
		view.centre += io.MouseDelta.y * vertical;
	}
	canvas_frame const frame{m_tool.m_plot, view, corner, size, across, vertical, ImGui::GetWindowDrawList()};
	// the path under the mouse
	if (hovered && false == m_dragging)
		m_hover = path_hovered(frame);
	else if (false == hovered && false == ImGui::IsItemActive())
		m_hover = -1;
	// a line out of the station under the mouse, how far out along it
	int line_hover{-1};
	double line_at{0.0};
	if (hovered && false == m_dragging && m_hover < 0)
		line_hovered(frame, line_hover, line_at);
	m_tool.m_track_lengths.m_length_canvas = m_hover >= 0 && false == m_dragging ? m_tool.m_track_lengths.length_of(m_tool.m_layout->paths[m_hover].track) : -1;
	if (ImGui::IsItemDeactivated())
		canvas_click(line_hover, line_at);
	draw_canvas(frame, line_hover, line_at);
	if (hovered && m_hover >= 0 && false == m_dragging)
		path_tooltip(frame);
	else if (line_hover >= 0)
		line_out_tooltip(line_hover, line_at);
}

void srk_tool::schematic::canvas_wheel(ImVec2 const &Corner, ImVec2 const &Size)
{
	auto &view{m_view_state};
	auto const &io{ImGui::GetIO()};
	if (io.KeyCtrl)
		view.exaggeration = std::clamp(view.exaggeration * std::pow(1.2f, io.MouseWheel), 1.0f, 500.0f);
	else
	{
		auto const at{view.from + (io.MousePos.x - Corner.x) / Size.x * (view.to - view.from)};
		auto const factor{std::pow(0.8, static_cast<double>(io.MouseWheel))};
		view.from = at - (at - view.from) * factor;
		view.to = at + (view.to - at) * factor;
	}
}

// the path under the mouse, -1 for none
int srk_tool::schematic::path_hovered(canvas_frame const &Frame) const
{
	int hover{-1};
	auto nearest{7.0f * 7.0f};
	auto const &mouse{ImGui::GetIO().MousePos};
	auto const screen_of = [&Frame](glm::dvec2 const &Point) { return Frame.screen_of(Point); };
	for (int index = 0; index < static_cast<int>(m_tool.m_layout->paths.size()); ++index)
		if (auto const gap{drawn_gap2(Frame.drawn_of(index), screen_of, mouse, nearest)}; gap < nearest)
		{
			nearest = gap;
			hover = index;
		}
	return hover;
}

// the line out of the station under the mouse, Hover, and how far out along it, At
void srk_tool::schematic::line_hovered(canvas_frame const &Frame, int &Hover, double &At) const
{
	auto const &io{ImGui::GetIO()};
	for (std::size_t index = 0; index < m_tool.m_line_survey.m_lines.size() && index < m_tool.m_line_survey.m_line_ways.size() && Hover < 0; ++index)
	{
		auto const &row{m_tool.m_line_survey.m_lines[index]};
		auto const distance{(Frame.chainage_at(io.MousePos.x) - row.chainage) * row.outward};
		if (std::abs(io.MousePos.y - Frame.plot_y(m_tool.m_plot.at({row.chainage, row.offset}).y)) < 6.0f && distance >= 0.0 && distance <= row.length)
		{
			Hover = static_cast<int>(index);
			At = distance;
		}
	}
}

// a click on the canvas let go: the path under the mouse selected, else the line out shown in the 3d view
void srk_tool::schematic::canvas_click(int const LineHover, double const LineAt)
{
	if (false == m_dragging && m_hover >= 0)
	{
		auto const &path{m_tool.m_layout->paths[m_hover]};
		m_tool.m_editor.selection.set_current_node(path.track);
		m_tool.m_picked = path.track;
		m_tool.m_editor.view.focus_track(*path.track, path.index, path.points[path.points.size() / 2]);
	}
	else if (false == m_dragging && LineHover >= 0)
	{
		auto const on{station::point_on(m_tool.m_line_survey.m_line_ways[LineHover], LineAt)};
		auto const out{m_tool.m_line_survey.m_lines[LineHover].inbound ? -1.0 : 1.0};
		m_tool.m_editor.view.fly_to(on.point + glm::dvec3{0.0, 4.0, 0.0}, glm::normalize(glm::dvec3{on.way.x * out, -0.3, on.way.z * out}), 30.0);
	}
	m_dragging = false;
}

void srk_tool::schematic::draw_canvas(canvas_frame const &Frame, int const LineHover, double const LineAt)
{
	auto const &layout{*m_tool.m_layout};
	auto *draw{Frame.draw};
	auto const &corner{Frame.corner};
	ImVec2 const corner_end{corner.x + Frame.size.x, corner.y + Frame.size.y};
	draw->AddRectFilled(corner, corner_end, IM_COL32(22, 24, 28, 255));
	draw->PushClipRect(corner, corner_end, true);
	draw_ruler(Frame);
	auto const *selected{m_tool.m_editor.selection.selected_track()};
	draw_paths(Frame);
	draw_lines(Frame, LineHover, LineAt);
	for (std::size_t index = 0; index < layout.paths.size(); ++index)
		if (layout.paths[index].track == selected)
			Frame.path(index, overlay_color::selected, 4.0f);
	if (m_hover >= 0)
		Frame.path(static_cast<std::size_t>(m_hover), kHovered, 4.0f);
	auto const useful{useful_lengths(Frame)};
	// the labels, none over another: the switches first, a dot at the start and the name over or under it in turn, the number
	// alone once named after the station; then the station tracks under or over them, the longest first
	canvas_labels labels{draw, {}};
	for (auto const &[text, at] : useful)
		labels.label(text, kUseful, {at});
	label_switches(Frame, labels);
	label_signals(Frame, labels);
	label_lines(Frame, labels);
	label_circuits(Frame, labels);
	draw->PopClipRect();
}

// the km, or the chainage, along the top
void srk_tool::schematic::draw_ruler(canvas_frame const &Frame) const
{
	auto *draw{Frame.draw};
	auto const &corner{Frame.corner};
	auto const first{Frame.chainage_at(corner.x)};
	auto const last{Frame.chainage_at(corner.x + Frame.size.x)};
	auto const metres{(last - first) / Frame.size.x * 90.0};
	double step{10.0};
	for (auto const candidate : {10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0})
		if ((step = candidate) >= metres)
			break;
	// the km of the station where it has them, else the chainage; the shortened stretches skip the marks too near
	double km_from{0.0};
	double km_to{0.0};
	auto const kilometrage{m_tool.km_of_open(first, km_from) && m_tool.km_of_open(last, km_to) && std::abs(km_to - km_from) > 1e-6};
	auto const low{kilometrage ? std::min(km_from, km_to) * 1000.0 : first};
	auto const high{kilometrage ? std::max(km_from, km_to) * 1000.0 : last};
	auto shown{std::numeric_limits<float>::lowest()};
	auto const lowest_mark{std::ceil(low / step) * step};
	for (int k = 0; lowest_mark + k * step <= high; ++k)
	{
		auto const mark{lowest_mark + k * step};
		auto const x{Frame.x_of(kilometrage ? first + (mark / 1000.0 - km_from) / (km_to - km_from) * (last - first) : mark)};
		auto const label{kilometrage ? format("km %.3f", mark / 1000.0) : format("%.0f m", mark)};
		if (std::abs(x - shown) < ImGui::CalcTextSize(label.c_str()).x + 12.0f)
			continue;
		shown = x;
		draw->AddLine(ImVec2(x, corner.y), ImVec2(x, corner.y + Frame.size.y), IM_COL32(255, 255, 255, 16));
		draw->AddText(ImVec2(x + 3.0f, corner.y + 2.0f), IM_COL32(150, 155, 165, 255), label.c_str());
	}
}

// the paths of the station: the lines out, the circuits in their colours, the switches, the joints
void srk_tool::schematic::draw_paths(canvas_frame const &Frame) const
{
	auto const &layout{*m_tool.m_layout};
	for (std::size_t index = 0; index < layout.paths.size(); ++index)
		if (layout.paths[index].approach)
			Frame.path(index, kApproach, 2.0f);
	for (std::size_t index = 0; index < layout.paths.size(); ++index)
		if (auto const &path{layout.paths[index]}; false == path.approach || m_circuit_of[index] >= 0)
		{
			ImU32 colour{kPlain};
			if (m_circuit_of[index] >= 0)
				colour = m_circuits[m_circuit_of[index]].colour;
			else if (is_switch(*path.track))
				colour = kSwitch;
			Frame.path(index, colour, 2.5f);
		}
	for (auto const &tick : m_ticks_drawn)
	{
		auto const at{Frame.screen_of(tick)};
		Frame.draw->AddLine(ImVec2(at.x, at.y - 5.0f), ImVec2(at.x, at.y + 5.0f), IM_COL32(255, 255, 255, 220), 2.0f);
	}
}

// the lines out straightened beyond the entry signals: the block sections in the colours of their names, the joints between
// them, the block signals each a flag pointing the way of its trains
void srk_tool::schematic::draw_lines(canvas_frame const &Frame, int const LineHover, double const LineAt) const
{
	auto *draw{Frame.draw};
	auto const line_x = [&Frame](line_row const &Row, double const Distance) { return Frame.line_x(Row, Distance); };
	for (std::size_t index = 0; index < m_tool.m_line_survey.m_lines.size(); ++index)
	{
		auto const &row{m_tool.m_line_survey.m_lines[index]};
		auto const y{Frame.plot_y(m_tool.m_plot.at({row.chainage, row.offset}).y)};
		draw->AddLine(ImVec2(line_x(row, 0.0), y), ImVec2(line_x(row, row.length), y), kApproach, 2.0f);
		for (auto const &run : row.sections)
		{
			auto const lit{static_cast<int>(index) == LineHover && LineAt >= run.from && LineAt <= run.to};
			draw->AddLine(ImVec2(line_x(row, std::max(0.0, run.from)), y), ImVec2(line_x(row, run.to), y), lit ? kHovered : circuit_colour(run.name), lit ? 4.0f : 2.5f);
			joints_drawn(draw, row, run, y, line_x);
		}
		for (auto const &post : row.posts)
		{
			auto const x{line_x(row, post.distance)};
			auto const way{static_cast<float>(row.outward * (row.inbound ? -1.0 : 1.0) * (post.wrong ? -1.0 : 1.0))};
			draw->AddTriangleFilled(ImVec2(x - 4.0f * way, y - 4.0f), ImVec2(x - 4.0f * way, y + 4.0f), ImVec2(x + 4.0f * way, y), kSignal);
		}
	}
}

// the useful lengths of the station track shown: of the trains to the right over it, to the left under it, an arrowhead where
// they stop; their labels and where they go
std::vector<std::pair<std::string, ImVec2>> srk_tool::schematic::useful_lengths(canvas_frame const &Frame) const
{
	std::vector<std::pair<std::string, ImVec2>> useful;
	if (m_tool.m_track_lengths.m_length_shown < 0 || m_tool.m_track_lengths.m_length_shown >= static_cast<int>(m_tool.m_track_lengths.m_lengths.size()))
		return useful;
	auto const &track{m_tool.m_track_lengths.m_lengths[m_tool.m_track_lengths.m_length_shown]};
	for (int way = 0; way < 2; ++way)
	{
		auto const &part{track.ways[way]};
		if (part.length <= 0.0)
			continue;
		auto const shift{way == 0 ? -7.0f : 7.0f};
		std::vector<ImVec2> line;
		for (auto const &point : stretch_of(track, part.rear.along, part.head.along))
		{
			auto at{Frame.screen_of(m_tool.m_plot.at(station::project(*m_tool.m_layout, point)))};
			at.y += shift;
			line.push_back(at);
		}
		useful_drawn(Frame.draw, line);
		auto const &middle{line[line.size() / 2]};
		useful.emplace_back(format("%.0f m", part.length), ImVec2(middle.x, way == 0 ? middle.y - ImGui::GetTextLineHeight() - 3.0f : middle.y + 3.0f));
	}
	return useful;
}

void srk_tool::schematic::label_switches(canvas_frame const &Frame, canvas_labels &Labels) const
{
	auto const &layout{*m_tool.m_layout};
	auto const stem{m_tool.stem_of_open()};
	auto const height{ImGui::GetTextLineHeight()};
	for (std::size_t index = 0; index < layout.switches.size(); ++index)
	{
		auto const *track{layout.switches[index]};
		auto at{Frame.screen_of(m_tool.m_plot.at(station::project(layout, track->m_paths.front().points[segment_data::point::start])))};
		for (std::size_t path = 0; path < layout.paths.size(); ++path)
			if (layout.paths[path].track == track && layout.paths[path].index == 0 && false == Frame.drawn_of(path).empty())
				at = Frame.screen_of(Frame.drawn_of(path).front());
		Frame.draw->AddCircleFilled(at, 3.5f, kSwitch);
		auto text{track->name()};
		if (false == stem.empty() && text.size() > stem.size() && text.compare(0, stem.size(), stem) == 0)
			text.erase(0, stem.size());
		ImVec2 const over{at.x, at.y - height - 6.0f};
		ImVec2 const under{at.x, at.y + 6.0f};
		if (index % 2 == 0)
			Labels.label(text, kSwitch, {over, under});
		else
			Labels.label(text, kSwitch, {under, over});
	}
}

// the signals: a flag pointing the way of the trains they face
void srk_tool::schematic::label_signals(canvas_frame const &Frame, canvas_labels &Labels) const
{
	auto const height{ImGui::GetTextLineHeight()};
	for (auto const &mark : m_signal_marks)
	{
		auto at{Frame.screen_of(mark.drawn)};
		at.y -= mark.side * 6.0f;
		auto const way{mark.ahead ? 1.0f : -1.0f};
		auto const colour{mark.unfit ? kUnfit : kSignal};
		Frame.draw->AddTriangleFilled(ImVec2(at.x - 4.0f * way, at.y - 4.0f), ImVec2(at.x - 4.0f * way, at.y + 4.0f), ImVec2(at.x + 4.0f * way, at.y), colour);
		Labels.label(mark.unfit ? mark.text + " !" : mark.text, colour, {ImVec2(at.x, at.y - height - 5.0f), ImVec2(at.x, at.y + 5.0f)});
	}
}

// the block signals of the lines with their km, the lengths of the block sections, on the outside of the line: over the upper
// track of a double-track one, under the lower
void srk_tool::schematic::label_lines(canvas_frame const &Frame, canvas_labels &Labels) const
{
	auto const height{ImGui::GetTextLineHeight()};
	for (auto const &row : m_tool.m_line_survey.m_lines)
	{
		auto const y{Frame.plot_y(m_tool.m_plot.at({row.chainage, row.offset}).y)};
		auto const up{row.partner < 0 || row.partner >= static_cast<int>(m_tool.m_line_survey.m_lines.size()) || row.offset >= m_tool.m_line_survey.m_lines[row.partner].offset};
		auto const outside = [up, y](float const Tall, float const Level) { return up ? y - 5.0f - (Tall + 3.0f) * Level + 3.0f : y + 5.0f + (Tall + 3.0f) * (Level - 1.0f); };
		auto const tall{height * (row.km_known ? 2.0f : 1.0f)};
		for (auto const wrong_track : {false, true})
			for (auto const &post : row.posts | std::views::filter([wrong_track](line_row::post const &Post) { return Post.wrong == wrong_track; }))
			{
				auto const x{Frame.line_x(row, post.distance)};
				Labels.label(row.km_known ? post.name + "\n" + format("%.3f", post.km) : post.name, kSignal, {ImVec2(x, outside(tall, 1.0f)), ImVec2(x, outside(tall, 2.0f))});
			}
		for (auto const &run : row.sections)
		{
			auto const x{Frame.line_x(row, (std::max(0.0, run.from) + run.to) * 0.5)};
			Labels.label(format("%.2f km", (run.to - std::max(0.0, run.from)) / 1000.0), circuit_colour(run.name), {ImVec2(x, outside(height, 1.0f)), ImVec2(x, outside(height, 2.0f))});
		}
	}
}

// the station tracks under or over them, the longest first
void srk_tool::schematic::label_circuits(canvas_frame const &Frame, canvas_labels &Labels) const
{
	auto const height{ImGui::GetTextLineHeight()};
	std::vector<std::size_t> order;
	for (std::size_t index = 0; index < m_circuits.size(); ++index)
		if (m_circuits[index].labelled)
			order.push_back(index);
	std::ranges::sort(order, [this](std::size_t A, std::size_t B) { return m_circuits[A].length > m_circuits[B].length; });
	for (auto const index : order)
	{
		auto const &mark{m_circuits[index]};
		auto const at{Frame.screen_of(mark.drawn)};
		Labels.label(mark.text, mark.colour, {ImVec2(at.x, at.y + 4.0f), ImVec2(at.x, at.y - height - 4.0f)});
	}
}

// the path under the mouse: its name, its kind and length, its km and its circuit, the useful length of its station track
void srk_tool::schematic::path_tooltip(canvas_frame const &Frame) const
{
	auto const &layout{*m_tool.m_layout};
	auto const &path{layout.paths[m_hover]};
	auto const along{Frame.chainage_at(ImGui::GetIO().MousePos.x)};
	ImGui::BeginTooltip();
	ImGui::TextUnformatted(path.track->name().empty() ? STR_C("(no name)") : path.track->name().c_str());
	char const *kind{STR_C("track")};
	if (is_switch(*path.track))
		kind = STR_C("switch");
	else if (path.approach)
		kind = STR_C("line out of the station");
	ImGui::TextDisabled("%s, %.1f m", kind, path.length);
	if (layout.kilometrage)
		ImGui::TextDisabled("km %.3f", station::km_of(layout, along));
	if (false == path.track->Isolated.empty())
		ImGui::TextDisabled(STR_C("track circuit %s"), path.track->Isolated.front()->asName.c_str());
	if (m_tool.m_track_lengths.m_length_canvas >= 0 && m_tool.m_track_lengths.m_length_canvas < static_cast<int>(m_tool.m_track_lengths.m_lengths.size()))
	{
		auto const &track{m_tool.m_track_lengths.m_lengths[m_tool.m_track_lengths.m_length_canvas]};
		ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kUseful), STR_C("Track %s, useful length %.0f m to the right, %.0f m to the left"), track.number.c_str(), track.ways[0].length, track.ways[1].length);
	}
	ImGui::TextDisabled("%s", STR_C("LMB: select it"));
	ImGui::EndTooltip();
}

// the line out under the mouse: its entry signal and track, the block section there with its km
void srk_tool::schematic::line_out_tooltip(int const LineHover, double const LineAt) const
{
	auto const &row{m_tool.m_line_survey.m_lines[LineHover]};
	ImGui::BeginTooltip();
	ImGui::TextUnformatted((format(STR_C("The line out beyond the entry signal %s"), row.entry.c_str()) + ", " + (row.number == 0 ? std::string{STR_C("single track")} : format(STR_C("track %d"), row.number))).c_str());
	if (auto const run{std::ranges::find_if(row.sections, [LineAt](line_row::run const &Run) { return LineAt >= Run.from && LineAt <= Run.to; })}; run != row.sections.end())
	{
		ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(circuit_colour(run->name)), STR_C("Block section %s, %.2f km"), run->name.c_str(), (run->to - std::max(0.0, run->from)) / 1000.0);
		if (row.km_known)
			ImGui::TextDisabled(STR_C("km %.3f to %.3f"), std::min(run->km_from, run->km_to), std::max(run->km_from, run->km_to));
	}
	else
		ImGui::TextDisabled("%s", STR_C("No track circuit here"));
	if (row.km_known && LineHover < static_cast<int>(m_tool.m_line_survey.m_line_ways.size()))
		ImGui::TextDisabled("km %.3f", m_tool.m_line_survey.km_along(m_tool.m_line_survey.m_line_ways[LineHover], row.km_number, LineAt));
	ImGui::TextDisabled("%s", STR_C("LMB: show it in the 3d view"));
	ImGui::EndTooltip();
}

void srk_tool::render_windows()
{
	m_track_lengths.m_length_canvas = -1;
	if (false == m_open || false == m_schematic.m_strip || false == m_layout.has_value())
		return;
	auto const &io{ImGui::GetIO()};
	// along the bottom of the 3d view, as the vertical profile
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	auto const width{std::max(480.0f, high.x - low.x)};
	auto const height{std::clamp((high.y - low.y) * 0.4f, 220.0f, std::max(220.0f, high.y - low.y - 80.0f))};
	ImGui::SetNextWindowPos(ImVec2(low.x, high.y - height), ImGuiCond_Appearing);
	ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Appearing);
	if (auto const name{std::string{STR_C("Station schematic")} + "  " + title() + "###srkstrip"}; ImGui::Begin(name.c_str(), nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
	{
		auto const position{ImGui::GetWindowPos()};
		if (auto const size{ImGui::GetWindowSize()}; false == ImGui::IsWindowDocked() && (position.y + size.y > io.DisplaySize.y + 1.0f || position.x + 80.0f > io.DisplaySize.x || position.x + size.x < 80.0f))
			ImGui::SetWindowPos(ImVec2(std::clamp(position.x, 0.0f, std::max(0.0f, io.DisplaySize.x - size.x)), std::max(0.0f, std::min(position.y, io.DisplaySize.y - size.y))));
		if (ImGui::Button(STR_C("Fit")))
		{
			m_schematic.m_fit_lines = false;
			m_schematic.m_view_state.exaggeration = 0.0f;
		}
		item_tooltip("Shows the whole station");
		ImGui::SameLine();
		ImGui::BeginDisabled(m_line_survey.m_lines.empty());
		if (ImGui::Button(STR_C("Lines")))
		{
			m_schematic.m_fit_lines = true;
			m_schematic.m_view_state.exaggeration = 0.0f;
		}
		ImGui::EndDisabled();
		item_tooltip("Shows the station with its lines out to the next stations: their block sections, the block signals with their km");
		ImGui::SameLine();
		ImGui::SetNextItemWidth(140.0f);
		ImGui::SliderFloat("##srkexaggeration", &m_schematic.m_view_state.exaggeration, 1.0f, 500.0f, STR_C("offsets x%.0f"), ImGuiSliderFlags_Logarithmic);
		item_tooltip("How much the offsets of the tracks are stretched against the lengths, also Ctrl+wheel over the plot");
		ImGui::SameLine();
		ImGui::TextDisabled("%s", STR_C("Wheel: zoom, drag: move, LMB on a track: select it"));
		m_schematic.render_canvas(ImGui::GetContentRegionAvail().y);
	}
	ImGui::End();
}

namespace
{

// the speeds listed in the log: 40/60/100, - for none
std::string speeds_listed(std::vector<int> const &Speeds)
{
	std::string text;
	for (auto const speed : Speeds)
		text += std::format("{}{}", text.empty() ? "" : "/", speed);
	return text.empty() ? std::string{"-"} : text;
}

// a route of a head in the log: the signal it leads to, its end or none, its length and speed, :to where it leads past a warning
// signal, the turnouts on it
std::string route_logged(srk_tool::head_check const &Head, std::size_t const K)
{
	auto const &route{Head.routes[K]};
	char const *next{"none"};
	if (route.next >= 0)
		next = Head.nexts[K].c_str();
	else if (route.ended)
		next = "end";
	return format(" %s:%.0f:%.0f%s%s", next, route.length, route.speed, route.warning >= 0 ? ":to" : "", route.turnouts.empty() ? "" : format(":%zut", route.turnouts.size()).c_str());
}

// the line of the log of a head checked
std::string head_logged(srk_tool::head_check const &Head)
{
	std::string routes;
	for (std::size_t k = 0; k < Head.routes.size(); ++k)
		routes += route_logged(Head, k);
	return format("SELFTEST srkheads %s %s%s %s lamps=%s own=%s need=%s%s%s short=%d%s fits=%d more=%d told=%d%s fit=%s shp=%.0f/%.0f routes%s problems=%zu", Head.name.c_str(), Head.kind.c_str(), Head.station ? "" : "(line)", Head.file.c_str(), Head.lamps.c_str(), speeds_listed(Head.own).c_str(), speeds_listed(Head.speeds).c_str(),
	              Head.informs ? "+inform" : "", Head.white ? "+white" : "", Head.shortened, Head.indicator ? " w21" : "", Head.fits ? 1 : 0, Head.more ? 1 : 0, Head.told ? 1 : 0, Head.unsaid ? "(unsaid)" : "", Head.fit.empty() ? "-" : Head.fit.c_str(), Head.shp, Head.shp_found, routes.c_str(),
	              Head.problems.size());
}

// the stops along a line in the log: name@distance each
std::string stops_listed(std::vector<std::pair<double, std::string>> const &Stops)
{
	std::string text;
	for (auto const &[distance, name] : Stops)
		text += format(" %s@%.0f", name.c_str(), distance);
	return text;
}

// the block in the words of the self-test: sbl4, sbl or semi
char const *block_word(srk_tool::block_kind const Block)
{
	if (Block == srk_tool::block_kind::four)
		return "sbl4";
	if (Block == srk_tool::block_kind::three)
		return "sbl";
	return "semi";
}

// the line of the log of a line out of the station
std::string row_logged(srk_tool::line_row const &Row)
{
	return format("SELFTEST srkblock %s track %d %s to %s %.0f m blocks %zu: %s why %d own %d ways %d%s leftovers %zu legacy %d stops%s wrong%s", Row.entry.c_str(), Row.number, Row.inbound ? "in" : "out", Row.next.empty() ? "-" : Row.next.c_str(), Row.length, Row.blocks, block_word(Row.block),
	              static_cast<int>(Row.why), static_cast<int>(Row.own), Row.both ? 2 : 1, Row.both_hand ? " hand" : "", Row.leftovers, Row.legacy ? 1 : 0, stops_listed(Row.stops).c_str(), stops_listed(Row.wrong).c_str());
}

// the block sections checked, each logged where it's wrong: without its signal, twice, switched by a signal which doesn't stand;
// how many are wrong
std::size_t sections_checked(std::vector<standing_template> const &Controls, std::unordered_map<std::string, int, string_hash, std::equal_to<>> const &Names)
{
	std::size_t bad{0};
	std::unordered_map<std::string, int, string_hash, std::equal_to<>> sections;
	for (auto const &block_control : Controls)
	{
		if (block_control.values.size() < 6)
		{
			++bad;
			WriteLog("SELFTEST srkcheck: a block section without its signal");
			continue;
		}
		if (++sections[ToLower(block_control.values[0])] > 1)
		{
			++bad;
			WriteLog("SELFTEST srkcheck: block section " + block_control.values[0] + " twice");
		}
		for (std::size_t index = 5; index < block_control.values.size() && index < 7; ++index)
		{
			if (Names.contains(ToLower(block_control.values[index])))
				continue;
			++bad;
			WriteLog("SELFTEST srkcheck: block section " + block_control.values[0] + " switched by " + block_control.values[index] + ", which doesn't stand");
		}
	}
	return bad;
}

// the points of the paths of the schematic as drawn: on the rows or between them, the bends, the steps back along x
struct drawn_points
{
	std::size_t on_rows{0};
	std::size_t between{0};
	std::size_t bends{0};
	std::size_t backwards{0};

	// the points of a path drawn, Schematic: those of the schematic it's drawn from
	template <typename Drawn, typename Schematic>
	void count(Drawn const &Points, Schematic const &Plan)
	{
		for (auto const &point : Points)
		{
			auto const row_at{point.y / station::row_height};
			if (std::abs(row_at - std::round(row_at)) < 1e-6)
				++on_rows;
			else
				++between;
		}
		for (std::size_t k = 2; k < Points.size(); ++k)
		{
			auto const a{Points[k - 1] - Points[k - 2]};
			auto const b{Points[k] - Points[k - 1]};
			if (std::abs(a.x) > 1e-6 && std::abs(b.x) > 1e-6 && (std::abs(a.y) > 1e-6 || std::abs(b.y) > 1e-6) && std::abs(a.y / a.x - b.y / b.x) > 1e-3)
				++bends;
		}
		for (std::size_t k = 1; k < Points.size(); ++k)
			if ((Points[k].x - Points[k - 1].x) * (Plan[k].x - Plan[k - 1].x) < 0.0)
				++backwards;
	}
};

// every fourth point of the schematic of a path in the log: offset/row>row drawn
template <typename Schematic, typename Drawn>
std::string schematic_listed(Schematic const &Plan, Drawn const &Points)
{
	std::string schematic;
	for (std::size_t k = 0; k < Plan.size(); k += 4)
		schematic += format(" %.0f/%.2f>%.2f", Plan[k].x, Plan[k].y, Points[k].y / station::row_height);
	return schematic;
}

// the signals standing along a line out and its circuits, by the distance out, as the self-test lists them
std::string line_listed(station::line const &Line, std::vector<standing_signal> const &Standing)
{
	std::vector<std::pair<double, std::string>> along;
	for (auto const &signal : Standing)
	{
		double gap{0.0};
		auto const distance{station::distance_on(Line, signal.position, gap)};
		if (gap < 4.5 && distance < Line.length())
			along.emplace_back(distance, format("%s(%s,%s)", signal.name.c_str(), signal.kind.c_str(), signal.linked.c_str()));
	}
	std::string last;
	for (std::size_t i = 1; i < Line.points.size(); ++i)
	{
		auto const *track{Line.tracks[Line.owners[i]]};
		auto const name{track->Isolated.empty() ? std::string{"-"} : track->Isolated.front()->asName};
		if (name != last)
			along.emplace_back(Line.distances[i], "[" + name + "]");
		last = name;
	}
	std::ranges::sort(along);
	std::string list;
	for (auto const &[distance, text] : along)
		list += format(" %.0f:%s", distance, text.c_str());
	return list;
}

// the block of the name a self-test gives: sbl4, sbl, else the semi-automatic one
srk_tool::block_kind block_named(std::string const &Kind)
{
	if (Kind == "sbl4")
		return srk_tool::block_kind::four;
	if (Kind == "sbl")
		return srk_tool::block_kind::three;
	return srk_tool::block_kind::semi;
}

// the signals named more than once logged by the self-test, how many names
std::size_t duplicates_logged(auto const &Names)
{
	std::size_t bad{0};
	for (auto const &[name, count] : Names)
		if (count > 1)
		{
			++bad;
			WriteLog(format("SELFTEST srkcheck: %d signals named %s", count, name.c_str()));
		}
	return bad;
}

// the signals whose event no track reads logged by the self-test, how many
std::size_t unread_logged(std::vector<standing_signal> const &Standing)
{
	std::size_t unread{0};
	for (auto const &signal : Standing)
		if (false == signal.read.empty() && signal.track == nullptr)
		{
			++unread;
			WriteLog("SELFTEST srkcheck: signal " + signal.name + " read by no track, its event " + signal.read);
		}
	return unread;
}

} // namespace

editor_selftest_result srk_tool::selftests::selftest_open(std::istringstream &Words)
{
	// srkopen track=<name>: the station of the track, as a click on it opens it
	auto const values{selftest_values(Words)};
	auto const found{values.find("track")};
	auto *track{found != values.end() ? simulation::Paths.find(found->second) : nullptr};
	m_tool.m_open = true;
	m_tool.load();
	if (track == nullptr)
	{
		WriteLog("SELFTEST srkopen: no such track");
		return editor_selftest_result::handled;
	}
	m_tool.open(*track);
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srkopen: " + m_tool.m_status);
		return editor_selftest_result::handled;
	}
	auto const &layout{*m_tool.m_layout};
	std::unordered_set<TTrack const *> tracks;
	std::size_t approach{0};
	for (auto const &path : layout.paths)
		if (path.approach)
			++approach;
		else
			tracks.insert(path.track);
	WriteLog(format("SELFTEST srkopen %s: %zu switches, %zu tracks, %zu paths out, axis %.0f m, switches over %.0f..%.0f, offsets %.1f..%.1f, km %s, record %d", m_tool.m_anchor->name().c_str(), layout.switches.size(), tracks.size(), approach,
	                layout.chainage.back(), layout.from, layout.to, layout.low, layout.high, layout.kilometrage ? format("%.3f", layout.km).c_str() : "-", m_tool.m_record));
	WriteLog(format("SELFTEST srkopen lines %zu, main %zu tracks, second %zu tracks", layout.lines.size(), layout.main.size(), layout.second.size()));
	for (auto const *switch_track : layout.switches)
	{
		auto const at{station::project(layout, switch_track->m_paths.front().points[segment_data::point::start])};
		WriteLog(format("SELFTEST srkopen switch %s at %.1f offset %.1f", switch_track->name().c_str(), at.x, at.y));
	}
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_number(std::istringstream &Words)
{
	// srknumber [from=right]: the switches of the open station numbered
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srknumber: no station open");
		return editor_selftest_result::handled;
	}
	auto const values{selftest_values(Words)};
	auto const from{values.find("from")};
	m_tool.m_reversed = from != values.end() && from->second == "right";
	m_tool.m_steps.number_switches();
	std::string circuit_list;
	for (auto const &unit : station::numbering(*m_tool.m_layout, m_tool.m_reversed))
		for (auto const *part : unit.parts)
		{
			auto const record{switch_store::named(*part)};
			circuit_list += ' ' + part->name() + '/' + (record ? record->number : std::string{"?"});
		}
	WriteLog("SELFTEST srknumber: " + m_tool.m_status);
	WriteLog("SELFTEST srknumber names:" + circuit_list);
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_circuits(std::istringstream & /*Words*/)
{
	// srkcircuits: the open station cut into its track circuits
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srkcircuits: no station open");
		return editor_selftest_result::handled;
	}
	m_tool.m_steps.set_circuits();
	WriteLog("SELFTEST srkcircuits: " + m_tool.m_status);
	m_tool.refresh();
	std::map<std::string, std::pair<std::size_t, double>, std::less<>> found;
	for (auto const &path : m_tool.m_layout->paths)
		if (false == path.track->Isolated.empty())
		{
			auto &[joints, metres]{found[path.track->Isolated.front()->asName]};
			++joints;
			metres += path.length;
		}
	std::string list;
	for (auto const &[name, entry] : found)
		list += format(" %s:%zu/%.0f", name.c_str(), entry.first, entry.second);
	WriteLog(format("SELFTEST srkcircuits %zu named, %zu joints:", found.size(), m_tool.m_schematic.m_ticks.size()) + list);
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_track(std::istringstream &Words)
{
	// srktrack circuit=<name> number=<number>: the station track of the circuit renumbered
	auto const values{selftest_values(Words)};
	auto const circuit{values.find("circuit")};
	auto const number{values.find("number")};
	TTrack const *found{nullptr};
	if (m_tool.m_layout.has_value() && circuit != values.end())
		for (auto const &path : m_tool.m_layout->paths)
			if (false == path.track->Isolated.empty() && path.track->Isolated.front()->asName == circuit->second)
				found = path.track;
	if (found == nullptr || number == values.end())
	{
		WriteLog("SELFTEST srktrack: no station open, or no such circuit");
		return editor_selftest_result::handled;
	}
	m_tool.m_steps.renumber_track(*found, number->second);
	WriteLog("SELFTEST srktrack: " + m_tool.m_status);
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_signals(std::istringstream & /*Words*/)
{
	// srksignals: the signals of the open station set
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srksignals: no station open");
		return editor_selftest_result::handled;
	}
	m_tool.m_steps.set_signals();
	WriteLog("SELFTEST srksignals: " + m_tool.m_status);
	m_tool.refresh();
	std::string list;
	for (auto const &mark : m_tool.m_schematic.m_signal_marks)
	{
		// and the joint nearest to it, in the plan
		auto nearest{std::numeric_limits<double>::max()};
		for (auto const &joint : m_tool.m_schematic.m_joints)
			nearest = std::min(nearest, glm::distance(glm::dvec2{joint.at.x, joint.at.z}, glm::dvec2{mark.head.x, mark.head.z}));
		list += format(" %s@%.0f/%.1f%s~%.1f", mark.text.c_str(), mark.at.x, mark.at.y, mark.ahead ? ">" : "<", nearest);
	}
	WriteLog(format("SELFTEST srksignals %zu:", m_tool.m_schematic.m_signal_marks.size()) + list);
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_lines(std::istringstream & /*Words*/)
{
	// srklines: the signals of the lines out of the open station set
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srklines: no station open");
		return editor_selftest_result::handled;
	}
	m_tool.m_line_survey.set_lines();
	WriteLog("SELFTEST srklines: " + m_tool.m_status);
	m_tool.refresh();
	std::vector<station::numbered> tracks;
	std::unordered_set<TTrack const *> inside;
	if (false == m_tool.station_tracks(tracks, inside))
		return editor_selftest_result::handled;
	auto const plan{station::signals(*m_tool.m_layout, tracks, [&inside](TTrack const &Track) { return inside.contains(&Track); })};
	std::vector<standing_signal> standing;
	m_tool.m_editor.signals.standing_signals(standing, true);
	for (auto const &line : station::lines_out(*m_tool.m_layout, plan, kLineLimit))
	{
		auto const list{line_listed(line, standing)};
		WriteLog(format("SELFTEST srklines %s track %d %s %.0f m%s:", plan[line.entry].name.c_str(), line.number, line.inbound ? "in" : "out", line.length(), line.station ? " to a station" : "") + list);
	}
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_lengths(std::istringstream &Words)
{
	// srklengths [show=<number>]: the lengths of the station tracks of the open station, the one shown drawn
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srklengths: no station open");
		return editor_selftest_result::handled;
	}
	std::vector<standing_signal> standing;
	m_tool.m_editor.signals.standing_signals(standing, true);
	m_tool.refresh();
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srklengths: the station is gone");
		return editor_selftest_result::handled;
	}
	auto const values{selftest_values(Words)};
	auto const show{values.find("show")};
	auto const code = [](station::limit const &Limit) {
		static constexpr std::array<char const *, 6> kinds{"S", "J", "F", "St", "B", "X"};
		return std::string{kinds[static_cast<std::size_t>(Limit.what)]} + ":" + Limit.name + format("@%.1f", Limit.along);
	};
	for (auto const &track : m_tool.m_track_lengths.m_lengths)
	{
		WriteLog(format("SELFTEST srklengths %s: right %.1f (%s .. %s), left %.1f (%s .. %s), construction %.1f", track.number.c_str(), track.ways[0].length, code(track.ways[0].rear).c_str(), code(track.ways[0].head).c_str(), track.ways[1].length,
		                code(track.ways[1].rear).c_str(), code(track.ways[1].head).c_str(), track.construction));
	}
	WriteLog(format("SELFTEST srklengths: %zu tracks", m_tool.m_track_lengths.m_lengths.size()));
	if (auto const copy{values.find("copy")}; copy != values.end() && copy->second == "1")
		WriteLog("SELFTEST srklengths table:\n" + m_tool.m_track_lengths.lengths_table());
	// the track shown selected
	for (auto const &path : m_tool.m_layout->paths)
		if (show != values.end() && false == path.track->Isolated.empty() && track_of(path.track->Isolated.front()->asName, m_tool.stem_of_open()) == show->second)
		{
			m_tool.m_editor.selection.set_current_node(path.track);
			m_tool.m_picked = path.track;
			break;
		}
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_heads(std::istringstream &Words)
{
	// srkheads [braking=<m>] [fit=1]: the lights of the signals of the open station checked, fitted
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srkheads: no station open");
		return editor_selftest_result::handled;
	}
	auto const values{selftest_values(Words)};
	if (auto const braking{values.find("braking")}; braking != values.end())
	{
		m_tool.record_of_open().braking = std::atoi(braking->second.c_str());
		m_tool.m_changed = true;
	}
	std::vector<standing_signal> standing;
	m_tool.m_editor.signals.standing_signals(standing, true);
	std::vector<signal_head> heads;
	m_tool.m_editor.signals.signal_heads(heads, true);
	m_tool.refresh();
	if (auto const fit{values.find("fit")}; fit != values.end() && fit->second == "1" && m_tool.m_layout.has_value())
	{
		m_tool.m_head_checks.fit_heads();
		WriteLog("SELFTEST srkheads fit: " + m_tool.m_status);
		m_tool.refresh();
	}
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srkheads: the station is gone");
		return editor_selftest_result::handled;
	}
	WriteLog(format("SELFTEST srkheads: speed %.0f (set %.0f), braking %.0f, %zu checked, templates %zu", m_tool.speed_of_open(), m_tool.set_speed_of_open(), m_tool.braking_of_open(), m_tool.m_head_checks.m_heads.size(), heads.size()));
	for (auto const &head : m_tool.m_head_checks.m_heads)
	{
		WriteLog(head_logged(head));
		for (auto const &problem : head.problems)
			WriteLog("SELFTEST srkheads   " + problem);
	}
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_shp(std::istringstream & /*Words*/)
{
	// srkshp: the SHP resonators of the open station placed, the signals listed with them
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srkshp: no station open");
		return editor_selftest_result::handled;
	}
	m_tool.m_head_checks.set_shp();
	WriteLog("SELFTEST srkshp: " + m_tool.m_status);
	m_tool.refresh();
	if (false == m_tool.m_layout.has_value())
		return editor_selftest_result::handled;
	std::vector<standing_signal> standing;
	m_tool.m_editor.signals.standing_signals(standing, true);
	m_tool.m_head_checks.check_heads(standing);
	for (auto const &head : m_tool.m_head_checks.m_heads)
		if (head.shp >= 0.0)
			WriteLog(format("SELFTEST srkshp %s %s wanted %.0f found %.0f", head.name.c_str(), head.kind.c_str(), head.shp, head.shp_found));
	for (auto const &include : m_tool.m_editor.templates.standing_templates([](std::string_view const File) { return File == kResonator; }))
		WriteLog(format("SELFTEST srkshp resonator %s at %.2f %.2f %.2f yaw %.1f", include.values.empty() ? "-" : include.values.front().c_str(), include.location.x, include.location.y, include.location.z, include.yaw));
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_block(std::istringstream &Words)
{
	// srkblock entry=<A> [kind=sbl|sbl4|semi|auto] [ways=1|2|auto]: the block of the line out beyond the entry signal, both ways or
	// not with the other track of the line, the lines listed
	if (m_tool.m_editor.history.history_revision() != m_tool.m_history_revision)
		m_tool.refresh();
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srkblock: no station open");
		return editor_selftest_result::handled;
	}
	auto const values{selftest_values(Words)};
	auto const entry{values.find("entry")};
	if (auto const kind{values.find("kind")}; entry != values.end() && kind != values.end())
	{
		if (kind->second == "auto")
			m_tool.record_of_open().blocks.erase(entry->second);
		else
			m_tool.record_of_open().blocks[entry->second] = block_named(kind->second);
		m_tool.m_changed = true;
	}
	if (auto const ways{values.find("ways")}; entry != values.end() && ways != values.end())
		selftest_ways(entry->second, ways->second);
	m_tool.survey_signals();
	if (false == m_tool.m_line_survey.m_lines_problem.empty())
		WriteLog("SELFTEST srkblock: " + m_tool.m_line_survey.m_lines_problem);
	for (auto const &row : m_tool.m_line_survey.m_lines)
		WriteLog(row_logged(row));
	return editor_selftest_result::handled;
}

// the ways the self-test gives the block of the line out beyond the entry signal, and of the other track of the line: 2, 1 or auto
void srk_tool::selftests::selftest_ways(std::string const &Entry, std::string const &Ways)
{
	auto const row{std::ranges::find_if(m_tool.m_line_survey.m_lines, [&Entry](line_row const &Row) { return Row.entry == Entry; })};
	std::vector<std::string> entries{Entry};
	if (row != m_tool.m_line_survey.m_lines.end() && row->partner >= 0 && row->partner < static_cast<int>(m_tool.m_line_survey.m_lines.size()))
		entries.push_back(m_tool.m_line_survey.m_lines[row->partner].entry);
	for (auto const &name : entries)
		if (Ways == "auto")
			m_tool.record_of_open().ways.erase(name);
		else
			m_tool.record_of_open().ways[name] = Ways == "2";
	m_tool.m_changed = true;
}

editor_selftest_result srk_tool::selftests::selftest_boundary(std::istringstream &Words)
{
	// srkboundary entry=<A> at=<m>|stop=<n> [nudge=<m>]: a click while drawing the block boundaries, on the line beyond the entry
	// signal at the distance out or at a block signal standing along it, nudge metres aside
	if (m_tool.m_editor.history.history_revision() != m_tool.m_history_revision)
		m_tool.refresh();
	auto const values{selftest_values(Words)};
	auto const entry{values.contains("entry") ? values.at("entry") : std::string{}};
	auto at{values.contains("at") ? std::atof(values.at("at").c_str()) : 0.0};
	auto const nudge{values.contains("nudge") ? std::atof(values.at("nudge").c_str()) : 0.0};
	auto const found{std::ranges::find_if(m_tool.m_line_survey.m_lines, [&](line_row const &Row) { return Row.entry == entry; })};
	if (false == m_tool.m_layout.has_value() || found == m_tool.m_line_survey.m_lines.end())
	{
		WriteLog("SELFTEST srkboundary: no line beyond " + entry);
		return editor_selftest_result::handled;
	}
	if (values.contains("stop") && false == found->stops.empty())
		at = found->stops[static_cast<std::size_t>(std::abs(std::atoi(values.at("stop").c_str()))) % found->stops.size()].first;
	auto const place{station::point_on(m_tool.m_line_survey.m_line_ways[found - m_tool.m_line_survey.m_lines.begin()], at)};
	m_tool.m_line_survey.m_drawing = true;
	bool valid{false};
	auto const label{m_tool.m_line_survey.boundary_hover(place.point + glm::dvec3{-place.way.z, 0.0, place.way.x} * nudge, valid)};
	WriteLog(format("SELFTEST srkboundary %s at %.0f: %s%s", entry.c_str(), at, valid ? "" : "(refused) ", label.c_str()));
	if (valid)
	{
		m_tool.m_line_survey.boundary_click();
		WriteLog("SELFTEST srkboundary: " + m_tool.m_status);
	}
	m_tool.m_line_survey.m_drawing = false;
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_fit(std::istringstream &Words)
{
	// srkfit lines|station|from=<m> to=<m>: the schematic fitting the lines out, the station, or the chainage between
	std::string word;
	std::string mode;
	std::map<std::string, std::string, std::less<>> bounds;
	while (Words >> word)
		if (auto const equals{word.find('=')}; equals != std::string::npos)
			bounds[word.substr(0, equals)] = word.substr(equals + 1);
		else
			mode = word;
	m_tool.m_schematic.m_fit_lines = mode == "lines";
	m_tool.m_schematic.m_view_state.exaggeration = 0.0f;
	if (bounds.contains("from") && bounds.contains("to") && m_tool.m_layout.has_value())
	{
		m_tool.m_schematic.fit_view(1000.0f, 400.0f);
		m_tool.m_schematic.m_view_state.from = std::atof(bounds.at("from").c_str());
		m_tool.m_schematic.m_view_state.to = std::atof(bounds.at("to").c_str());
	}
	WriteLog("SELFTEST srkfit " + mode);
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_draw(std::istringstream &Words)
{
	// srkdraw on|off: the block boundaries drawn by hand, or not
	std::string word;
	Words >> word;
	m_tool.m_line_survey.m_drawing = word != "off";
	WriteLog(std::string{"SELFTEST srkdraw "} + (m_tool.m_line_survey.m_drawing ? "on" : "off"));
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_check(std::istringstream & /*Words*/)
{
	// srkcheck: the block sections of the scenery checked: each switched by a block signal standing, none twice, no two signals of
	// one name
	std::vector<standing_signal> standing;
	m_tool.m_editor.signals.standing_signals(standing, true);
	std::unordered_map<std::string, int, string_hash, std::equal_to<>> names;
	for (auto const &signal : standing)
		++names[ToLower(signal.name)];
	std::size_t bad{duplicates_logged(names)};
	auto const controls{m_tool.m_editor.templates.standing_templates([](std::string_view const File) { return File == kControl; })};
	bad += sections_checked(controls, names);
	auto const unread{unread_logged(standing)};
	auto const circuits{scenery_circuits()};
	std::size_t bare{0};
	for (auto const &block_control : controls)
		if (false == block_control.values.empty() && false == circuits.contains(ToLower(block_control.values[0])))
		{
			++bare;
			WriteLog("SELFTEST srkcheck: block section " + block_control.values[0] + " without a track");
		}
	bad += unread + bare;
	if (m_tool.m_layout.has_value())
	{
		m_tool.survey_signals();
		for (auto const &problem : m_tool.m_line_survey.m_problems)
			WriteLog(format("SELFTEST srkcheck: open station: %s at %.1f %.1f%s%s", problem.text.c_str(), problem.at.x, problem.at.z, problem.signal != 0 ? " (signal)" : "", problem.lines ? " (lines)" : ""));
	}
	WriteLog(format("SELFTEST srkcheck: %zu signals, %zu block sections, %zu unread, %zu without a track, %zu problems of the open station, %zu bad", standing.size(), controls.size(), unread, bare, m_tool.m_line_survey.m_problems.size(), bad));
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_fix(std::istringstream & /*Words*/)
{
	// srkfix: the Repair of the check of the open station
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srkfix: no station open");
		return editor_selftest_result::handled;
	}
	m_tool.survey_signals();
	auto const before{m_tool.m_line_survey.m_problems.size()};
	m_tool.m_line_survey.fix_problems();
	WriteLog(format("SELFTEST srkfix: %zu problems before, status %s", before, m_tool.m_status.c_str()));
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_drawing(std::istringstream &Words)
{
	// srkdrawing [track=<circuit>]: the schematic as the plot draws it: the rows of the tracks, the shortened chainage, the points
	// on the rows, on slants, the corners; the points of the paths of the circuit, offset > row
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srkdrawing: no station open");
		return editor_selftest_result::handled;
	}
	auto const &layout{*m_tool.m_layout};
	std::map<int, std::string> rows;
	for (auto const &run : m_tool.m_plot.runs)
		if (false == run.paths.empty())
			rows[run.row] += format(" %s@%.1f", layout.paths[run.paths.front()].track->name().c_str(), run.offset);
	std::string list;
	for (auto const &[row, names] : rows)
		list += format(" |%d:", row) + names;
	WriteLog(format("SELFTEST srkdrawing tracks %zu, rows %zu, spacing %.2f", m_tool.m_plot.runs.size(), rows.size(), m_tool.m_plot.spacing));
	WriteLog("SELFTEST srkdrawing rows" + list);
	WriteLog(format("SELFTEST srkdrawing switches over %.0f..%.0f m drawn %.0f..%.0f, back %.1f..%.1f", layout.from, layout.to, m_tool.m_plot.x(layout.from), m_tool.m_plot.x(layout.to), m_tool.m_plot.chainage(m_tool.m_plot.x(layout.from)), m_tool.m_plot.chainage(m_tool.m_plot.x(layout.to))));
	drawn_points drawn;
	for (std::size_t index = 0; index < layout.paths.size(); ++index)
		if (false == layout.paths[index].approach)
			drawn.count(m_tool.m_plot.paths[index], layout.paths[index].schematic);
	WriteLog(format("SELFTEST srkdrawing points on rows %zu, between %zu, bends %zu, x backwards %zu", drawn.on_rows, drawn.between, drawn.bends, drawn.backwards));
	if (auto const values{selftest_values(Words)}; values.contains("track"))
		for (std::size_t index = 0; index < layout.paths.size(); ++index)
			if (auto const &path{layout.paths[index]}; false == path.track->Isolated.empty() && path.track->Isolated.front()->asName == values.at("track"))
				WriteLog(format("SELFTEST srkdrawing %s/%d:", path.track->name().c_str(), path.index) + schematic_listed(path.schematic, m_tool.m_plot.paths[index]));
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_strip(std::istringstream &Words)
{
	// srkstrip shown=0: the schematic hidden, the 3d view whole for a screenshot
	auto const values{selftest_values(Words)};
	auto const shown{values.find("shown")};
	m_tool.m_schematic.m_strip = shown == values.end() || shown->second != "0";
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_stations(std::istringstream & /*Words*/)
{
	m_tool.load();
	m_tool.m_steps.find_all();
	WriteLog(format("SELFTEST srkstations: %zu", m_tool.m_steps.m_found.size()));
	for (auto const &[name, count] : m_tool.m_steps.m_found)
		WriteLog(format("SELFTEST srkstations %s: %zu switches", name.c_str(), count));
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest_set(std::istringstream &Words)
{
	// srkset code=<code> line=<line> name=<words joined by _>
	if (false == m_tool.m_layout.has_value())
	{
		WriteLog("SELFTEST srkset: no station open");
		return editor_selftest_result::handled;
	}
	auto const values{selftest_values(Words)};
	auto &entry{m_tool.record_of_open()};
	if (auto const code{values.find("code")}; code != values.end())
		entry.code = code->second;
	if (auto const line{values.find("line")}; line != values.end())
		entry.line = line->second;
	if (auto const name{values.find("name")}; name != values.end())
	{
		entry.name = name->second;
		std::ranges::replace(entry.name, '_', ' ');
	}
	if (auto const km{values.find("km")}; km != values.end())
		entry.km = km->second;
	m_tool.m_changed = true;
	WriteLog("SELFTEST srkset: " + m_tool.title());
	return editor_selftest_result::handled;
}

editor_selftest_result srk_tool::selftests::selftest(std::string const &Command, std::istringstream &Words)
{
	using command = editor_selftest_result (srk_tool::selftests::*)(std::istringstream &);
	static constexpr std::array<std::pair<std::string_view, command>, 19> commands{{
		{"srkopen", &srk_tool::selftests::selftest_open},
		{"srknumber", &srk_tool::selftests::selftest_number},
		{"srkcircuits", &srk_tool::selftests::selftest_circuits},
		{"srktrack", &srk_tool::selftests::selftest_track},
		{"srksignals", &srk_tool::selftests::selftest_signals},
		{"srklines", &srk_tool::selftests::selftest_lines},
		{"srklengths", &srk_tool::selftests::selftest_lengths},
		{"srkheads", &srk_tool::selftests::selftest_heads},
		{"srkshp", &srk_tool::selftests::selftest_shp},
		{"srkblock", &srk_tool::selftests::selftest_block},
		{"srkboundary", &srk_tool::selftests::selftest_boundary},
		{"srkfit", &srk_tool::selftests::selftest_fit},
		{"srkdraw", &srk_tool::selftests::selftest_draw},
		{"srkcheck", &srk_tool::selftests::selftest_check},
		{"srkfix", &srk_tool::selftests::selftest_fix},
		{"srkdrawing", &srk_tool::selftests::selftest_drawing},
		{"srkstrip", &srk_tool::selftests::selftest_strip},
		{"srkstations", &srk_tool::selftests::selftest_stations},
		{"srkset", &srk_tool::selftests::selftest_set},
	}};
	for (auto const &[name, run] : commands)
		if (Command == name)
			return (this->*run)(Words);
	return editor_selftest_result::unhandled;
}

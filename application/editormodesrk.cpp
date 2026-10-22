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
#include <limits>
#include <map>
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
double constexpr kBlockShare{1.1}; // of the braking distance, the block sections towards the end of the tracks
double constexpr kPredicted{305.0}; // m, out from the next station to its entry signal, while it has none
// the events of a block section, switching the block signal guarding it
char const *const kControl{"sbl_odstep.inc"};

// m, of a train at the speed, as Ie-4 § 8 takes it: the least block section of the automatic block, a warning disc that far before
// its signal
double braking_distance(double const Speed)
{
	return Speed <= 60.0 ? 400.0 : Speed <= 100.0 ? 700.0 : Speed <= 140.0 ? 1000.0 : 1300.0;
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

// the number of the switch of a unit: of its name once named after the station, of its record, else of its place
std::string unit_number(station::unit const &Unit, std::string const &Stem, std::size_t const Index)
{
	auto const &name{Unit.parts.front()->name()};
	if (name.size() > Stem.size() && name.compare(0, Stem.size(), Stem) == 0)
	{
		auto digits{name.substr(Stem.size())};
		while (false == digits.empty() && std::isalpha(static_cast<unsigned char>(digits.back())))
			digits.pop_back();
		if (false == digits.empty() && std::all_of(digits.begin(), digits.end(), [](char const Digit) { return std::isdigit(static_cast<unsigned char>(Digit)) != 0; }))
			return digits;
	}
	if (auto const record{switch_store::named(*Unit.parts.front())}; record && record->number != "-")
		return record->number;
	return std::to_string(Index + 1);
}

// the number of the station track a circuit is named after: bpd_it5b of track 5; empty for the other names, bpd_ita of the
// circuit before the entry signal A among them
std::string track_of(std::string const &Name, std::string const &Stem)
{
	auto const head{Stem + "it"};
	if (Stem.empty() || Name.size() <= head.size() || Name.compare(0, head.size(), head) != 0)
		return {};
	auto number{Name.substr(head.size())};
	if (std::none_of(number.begin(), number.end(), [](char const Letter) { return std::isdigit(static_cast<unsigned char>(Letter)) != 0; }))
		return {};
	if (number.size() > 1 && std::isalpha(static_cast<unsigned char>(number.back())) && std::isdigit(static_cast<unsigned char>(number[number.size() - 2])))
		number.pop_back();
	return number;
}

// the circuit between an entry signal and the station: bpd_ita
bool entry_circuit(std::string const &Name, std::string const &Stem)
{
	auto const head{Stem + "it"};
	return false == Stem.empty() && Name.size() > head.size() && Name.compare(0, head.size(), head) == 0 &&
	       std::all_of(Name.begin() + head.size(), Name.end(), [](char const Letter) { return std::isalpha(static_cast<unsigned char>(Letter)) != 0; });
}

// the name of a signal as its plate and the schematic show it: bpd_tm3 as Tm3, bpd_f1 as F1
std::string signal_shown(std::string const &Name, std::string const &Stem)
{
	auto text{false == Stem.empty() && Name.size() > Stem.size() && ToLower(Name).compare(0, Stem.size(), Stem) == 0 ? Name.substr(Stem.size()) : Name};
	std::transform(text.begin(), text.end(), text.begin(), [](char const Letter) { return static_cast<char>(std::toupper(static_cast<unsigned char>(Letter))); });
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
	std::transform(tag.begin(), tag.end(), tag.begin(), [](char const Letter) { return static_cast<char>(std::toupper(static_cast<unsigned char>(Letter))); });
	return tag + std::to_string(Track);
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
std::string without_stem(std::string const &Name, std::string const &Stem)
{
	return false == Stem.empty() && Name.size() > Stem.size() && Name.compare(0, Stem.size(), Stem) == 0 ? Name.substr(Stem.size()) : Name;
}

// a limit of a useful length in words, as "from" and "to" take it
std::string limit_text(station::limit const &Limit, std::string const &Stem)
{
	auto const name{without_stem(Limit.name, Stem)};
	switch (Limit.what)
	{
	case station::limit::kind::signal:
		return format(STR_C("signal %s"), name.c_str());
	case station::limit::kind::fouling:
		return format(STR_C("the fouling point of switch %s"), name.c_str());
	case station::limit::kind::start:
		return format(STR_C("the start of switch %s"), name.c_str());
	case station::limit::kind::buffer:
		return STR_C("the buffer stop");
	case station::limit::kind::crossing:
		return STR_C("the level crossing");
	default:
		return name.empty() ? std::string{STR_C("the joint")} : format(STR_C("the joint by switch %s"), name.c_str());
	}
}

// the points of the track from one distance along it to the other
std::vector<glm::dvec3> stretch_of(station::lengths const &Track, double const From, double const To)
{
	auto const at = [&](double const Along) {
		auto const after{std::min<std::size_t>(std::upper_bound(Track.distances.begin(), Track.distances.end(), Along) - Track.distances.begin(), Track.distances.size() - 1)};
		auto const before{after > 0 ? after - 1 : 0};
		auto const span{Track.distances[after] - Track.distances[before]};
		return glm::mix(Track.points[before], Track.points[after], span > 1e-9 ? std::clamp((Along - Track.distances[before]) / span, 0.0, 1.0) : 0.0);
	};
	std::vector<glm::dvec3> result{at(std::min(From, To))};
	for (std::size_t i = 0; i < Track.distances.size(); ++i)
		if (Track.distances[i] > std::min(From, To) + 0.05 && Track.distances[i] < std::max(From, To) - 0.05)
			result.push_back(Track.points[i]);
	result.push_back(at(std::max(From, To)));
	if (From > To)
		std::reverse(result.begin(), result.end());
	return result;
}

glm::dvec3 axis_point(station::layout const &Layout, double const Chainage)
{
	auto const after{std::lower_bound(Layout.chainage.begin(), Layout.chainage.end(), Chainage)};
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
	auto const shown{Speed >= 98.0 ? 100 : Speed >= 58.0 ? 60 : 40};
	return shown >= Line ? 0 : shown;
}

// km/h, a signal slows the trains to with the next one at stop standing that far on, Ie-4 § 8 ust. 5; 140 with a W21 indicator
int shortened_speed(double const Length)
{
	return Length < 400.0 ? 40 : Length < 700.0 ? 60 : Length < 1000.0 ? 100 : 140;
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

head_need need_of(std::vector<station::route> const &Routes, bool const Main, double const Line, double const Braking)
{
	head_need need;
	need.white = Main;
	auto const add = [&](int const Speed) {
		if (Speed > 0 && std::find(need.speeds.begin(), need.speeds.end(), Speed) == need.speeds.end())
			need.speeds.push_back(Speed);
	};
	for (auto const &route : Routes)
	{
		auto const shown{Main ? shown_speed(route.speed, Line) : 0};
		add(shown);
		auto const told{route.next >= 0 && route.warning < 0};
		need.informs = need.informs || told;
		if (false == Main || false == told || route.length >= Braking)
			continue;
		auto const slower{shortened_speed(route.length)};
		if (slower >= (shown > 0 ? shown : Line))
			continue;
		if (slower > 100)
			need.indicator = true;
		else
		{
			add(slower);
			need.shortened = need.shortened == 0 ? slower : std::min(need.shortened, slower);
		}
	}
	std::sort(need.speeds.begin(), need.speeds.end());
	return need;
}

bool has_speed(std::vector<int> const &Speeds, int const Speed)
{
	return std::find(Speeds.begin(), Speeds.end(), Speed) != Speeds.end();
}

// the template has the lights the need takes: a lower orange one for a lower speed, a strip for 60 and 100, Ie-1 § 7, two orange
// ones to show a lower speed with the next signal at stop
bool shows(signal_head const &Head, head_need const &Need)
{
	auto const &lamps{Head.lamps};
	auto const count = [&](char const Lamp) { return std::count(lamps.begin(), lamps.end(), Lamp); };
	if (count('z') == 0 || count('c') == 0 || (Need.white && count('b') == 0))
		return false;
	if (Need.informs && (false == Head.informs || count('p') == 0))
		return false;
	if (Need.speeds.empty())
		return true;
	if (lamps.find('p', lamps.find('c')) == std::string::npos || (Need.informs && count('p') < 2))
		return false;
	return std::all_of(Need.speeds.begin(), Need.speeds.end(), [&](int const Speed) { return has_speed(Head.speeds, Speed); });
}

// the more lights and strips beyond the need, the more
int surplus(signal_head const &Head, head_need const &Need)
{
	auto result{static_cast<int>(Head.lamps.size()) * 4};
	for (auto const speed : {60, 100})
		if (has_speed(Head.speeds, speed) && false == has_speed(Need.speeds, speed))
			++result;
	return result;
}

// of the templates of the kind and the mount that fit the need, the one with the least beyond it, the most like the one named;
// -1 for none
int best_head(std::vector<signal_head> const &Heads, std::string const &Kind, std::string const &Mount, std::string const &Like, head_need const &Need)
{
	auto best{-1};
	std::tuple<int, int, std::size_t> score;
	for (int index = 0; index < static_cast<int>(Heads.size()); ++index)
	{
		auto const &head{Heads[index]};
		if (head.kind != Kind || head.mount != Mount || false == shows(head, Need))
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
		speeds += (speeds.empty() ? "" : ", ") + std::to_string(speed);
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
	if (Signal.track != nullptr && Signal.facing != 0 && false == Signal.track->m_editorremoved)
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
	auto best{std::numeric_limits<double>::max()};
	for (int k = 0; k <= 64; ++k)
		if (auto const distance{geometry::plan_distance(curve.point(k / 64.0), Signal.position)}; distance < best)
		{
			best = distance;
			parameter = k / 64.0;
		}
	auto const tangent{curve.first(parameter)};
	auto const travel{travel_of(Signal.yaw)};
	post.facing = travel.x * tangent.x + travel.y * tangent.z >= 0.0 ? 1 : -1;
	return post;
}

} // namespace

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
			while (words >> word)
			{
				if (word == "line")
					words >> entry.line;
				else if (word == "km")
					words >> entry.km;
				else if (word == "from")
					entry.reversed = (words >> word) && word == "right";
				else if (word == "braking")
					words >> entry.braking;
				else if (word == "anchor")
					words >> entry.anchor;
				else if (word == "name")
				{
					std::getline(words, entry.name);
					entry.name.erase(0, entry.name.find_first_not_of(' '));
					break;
				}
			}
			entry.code = shown(entry.code);
			entry.line = shown(entry.line);
			entry.km = shown(entry.km);
			entry.track = simulation::Paths.find(entry.anchor);
			if (false == entry.anchor.empty())
				m_records.push_back(std::move(entry));
		}
}

void srk_tool::store()
{
	if (false == m_changed || scene::Layers.empty())
		return;
	m_changed = false;
	std::vector<scene::layer_handle> layers{scene::Layers.marked_layers(kMark)};
	for (auto const &entry : m_records)
		if (std::find(layers.begin(), layers.end(), entry.layer) == layers.end())
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
			auto const braking{entry.braking > 0 ? format(" braking %d", entry.braking) : std::string{}};
			lines.push_back(format("station %s line %s km %s from %s anchor %s%s name %s", word_of(entry.code).c_str(), word_of(entry.line).c_str(), word_of(entry.km).c_str(), entry.reversed ? "right" : "left", anchor_of(entry).c_str(), braking.c_str(), entry.name.c_str()));
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
	m_hover = -1;
	m_view_state.exaggeration = 0.0f;
	m_history_revision = m_history.history_revision();
	// the record of the station, by any of its switches
	m_record = -1;
	for (int index = 0; index < static_cast<int>(m_records.size()) && m_record < 0; ++index)
	{
		auto &entry{m_records[index]};
		if (entry.track == nullptr)
			entry.track = simulation::Paths.find(entry.anchor);
		if (std::find(m_layout->switches.begin(), m_layout->switches.end(), entry.track) != m_layout->switches.end())
			m_record = index;
	}
	m_anchor = m_record >= 0 ? m_records[m_record].track : m_layout->switches.front();
	m_reversed = m_record >= 0 && m_records[m_record].reversed;
	survey();
}

void srk_tool::refresh()
{
	m_history_revision = m_history.history_revision();
	if (m_anchor == nullptr || m_anchor->m_editorremoved)
	{
		m_layout.reset();
		return;
	}
	auto const view{m_view_state};
	auto const status{m_status};
	open(*m_anchor);
	if (m_layout.has_value())
	{
		m_view_state = view;
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

void srk_tool::find_all()
{
	m_found.clear();
	for (auto *track : station::stations())
		m_found.emplace_back(track->name(), station::switches_of(*track).size());
	std::sort(m_found.begin(), m_found.end(), [](auto const &A, auto const &B) { return A.second > B.second; });
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
	if ((m_layout.has_value() || m_anchor != nullptr) && m_history.history_revision() != m_history_revision)
		refresh();
	else if (m_layout.has_value() && false == m_signals_known)
		survey_signals();
	m_length_shown = m_length_hover >= 0 ? m_length_hover : m_length_canvas >= 0 ? m_length_canvas : length_of(m_selection.selected_track());
	if (m_length_shown >= static_cast<int>(m_lengths.size()))
		m_length_shown = -1;
	// a track of another station selected opens it
	auto *track{m_selection.selected_track()};
	if (track == nullptr || track == m_picked)
		return;
	m_picked = track;
	if (m_layout.has_value() && std::any_of(m_layout->paths.begin(), m_layout->paths.end(), [&](station::path const &Path) { return Path.track == track && false == Path.approach; }))
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
			for (auto *track : m_layout->switches)
				if (track != Node)
				{
					m_anchor = track;
					break;
				}
	}
	if (m_layout.has_value() && std::any_of(m_layout->paths.begin(), m_layout->paths.end(), [&](station::path const &Path) { return Path.track == Node; }))
	{
		// laid out again from its anchor at the next update
		m_layout.reset();
		m_hover = -1;
		m_history_revision = static_cast<std::size_t>(-1);
	}
}

void srk_tool::draw_overlay() const
{
	if (false == m_open || false == m_layout.has_value())
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	glm::dvec3 const camera{Global.pCamera.Pos};
	auto const near_enough = [&](glm::dvec3 const &Point, double const Range) { return Global.EditorOrtho || glm::distance(Point, camera) < Range; };
	glm::dvec3 const lift{0.0, kRailHead, 0.0};
	// the track circuits: the tracks in their colours, a bar across them at each joint with a post, the station tracks named
	for (std::size_t index = 0; index < m_layout->paths.size() && index < m_circuit_of.size(); ++index)
	{
		if (m_circuit_of[index] < 0)
			continue;
		auto const colour{(m_circuits[m_circuit_of[index]].colour & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 200)};
		auto const &points{m_layout->paths[index].points};
		for (std::size_t i = 1; i < points.size(); ++i)
			if (near_enough(points[i], kOverlayRange))
				projection.line(drawlist, points[i - 1] + lift, points[i] + lift, colour, Global.EditorOrtho ? 6.0f : 3.0f);
	}
	for (auto const &joint : m_joints)
	{
		if (false == near_enough(joint.at, kJointRange))
			continue;
		auto const centre{joint.at + lift};
		projection.line(drawlist, centre - joint.across * 1.7, centre + joint.across * 1.7, kJoint, 4.0f);
		projection.line(drawlist, centre + joint.across * 1.7, centre + joint.across * 1.7 + glm::dvec3{0.0, 1.2, 0.0}, kJoint, 2.0f);
		ImVec2 top;
		if (projection.project(centre + joint.across * 1.7 + glm::dvec3{0.0, 1.2, 0.0}, top))
			drawlist->AddCircleFilled(top, 3.0f, kJoint);
	}
	for (auto const &mark : m_circuits)
	{
		ImVec2 screen;
		if (false == mark.labelled || false == near_enough(mark.world, kLabelRange) || false == projection.project(mark.world + glm::dvec3{0.0, 0.8, 0.0}, screen))
			continue;
		auto const size{ImGui::CalcTextSize(mark.text.c_str())};
		drawlist->AddRectFilled(ImVec2(screen.x - size.x * 0.5f - 3.0f, screen.y - size.y - 2.0f), ImVec2(screen.x + size.x * 0.5f + 3.0f, screen.y + 2.0f), IM_COL32(0, 0, 0, 160), 3.0f);
		drawlist->AddText(ImVec2(screen.x - size.x * 0.5f, screen.y - size.y), mark.colour, mark.text.c_str());
	}
	if (m_hover >= 0 && m_hover < static_cast<int>(m_layout->paths.size()))
	{
		auto const &points{m_layout->paths[m_hover].points};
		for (std::size_t i = 1; i < points.size(); ++i)
			projection.line(drawlist, points[i - 1], points[i], kHovered, 4.0f);
	}
	for (auto const *track : m_layout->switches)
	{
		auto const start{track->m_paths.front().points[segment_data::point::start] + glm::dvec3{0.0, 1.0, 0.0}};
		ImVec2 screen;
		if (false == near_enough(start, kLabelRange) || false == projection.project(start, screen))
			continue;
		auto const text{track->name()};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		drawlist->AddRectFilled(ImVec2(screen.x - 3.0f, screen.y - size.y - 4.0f), ImVec2(screen.x + size.x + 3.0f, screen.y), IM_COL32(0, 0, 0, 160), 3.0f);
		drawlist->AddText(ImVec2(screen.x, screen.y - size.y - 2.0f), kSwitch, text.c_str());
	}
	// the useful lengths of the station track shown, beside it on the right of the trains, a post where they stop
	if (m_length_shown >= 0 && m_length_shown < static_cast<int>(m_lengths.size()))
	{
		auto const &track{m_lengths[m_length_shown]};
		for (auto const &part : track.ways)
		{
			if (part.length <= 0.0)
				continue;
			auto const points{stretch_of(track, part.rear.along, part.head.along)};
			std::vector<glm::dvec3> beside;
			for (std::size_t i = 0; i < points.size(); ++i)
			{
				auto const way{points[std::min(i + 1, points.size() - 1)] - points[i > 0 && i + 1 == points.size() ? i - 1 : i]};
				glm::dvec2 const along{glm::normalize(glm::dvec2{way.x, way.z} + glm::dvec2{1e-12, 0.0})};
				beside.push_back(points[i] + lift + glm::dvec3{-along.y, 0.1, along.x} * 0.9);
			}
			for (std::size_t i = 1; i < beside.size(); ++i)
				if (near_enough(beside[i], kOverlayRange))
					projection.line(drawlist, beside[i - 1], beside[i], kUseful, 3.0f);
			auto const top{beside.back() + glm::dvec3{0.0, 2.5, 0.0}};
			projection.line(drawlist, beside.front(), beside.front() + glm::dvec3{0.0, 1.0, 0.0}, kUseful, 2.0f);
			projection.line(drawlist, beside.back(), top, kUseful, 2.0f);
			ImVec2 screen;
			if (false == near_enough(top, kLabelRange) || false == projection.project(top, screen))
				continue;
			auto const text{format(STR_C("track %s: %.0f m"), track.number.c_str(), part.length)};
			auto const size{ImGui::CalcTextSize(text.c_str())};
			drawlist->AddRectFilled(ImVec2(screen.x - size.x * 0.5f - 4.0f, screen.y - size.y - 4.0f), ImVec2(screen.x + size.x * 0.5f + 4.0f, screen.y), IM_COL32(0, 0, 0, 180), 3.0f);
			drawlist->AddText(ImVec2(screen.x - size.x * 0.5f, screen.y - size.y - 2.0f), kUseful, text.c_str());
		}
	}
	for (auto const &mark : m_signal_marks)
	{
		ImVec2 screen;
		if (false == near_enough(mark.head, kLabelRange) || false == projection.project(mark.head, screen))
			continue;
		auto const text{mark.unfit ? mark.text + " !" : mark.text};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		drawlist->AddRectFilled(ImVec2(screen.x - size.x * 0.5f - 4.0f, screen.y - size.y - 4.0f), ImVec2(screen.x + size.x * 0.5f + 4.0f, screen.y), mark.unfit ? kUnfit : kSignal, 3.0f);
		drawlist->AddText(ImVec2(screen.x - size.x * 0.5f, screen.y - size.y - 2.0f), mark.unfit ? IM_COL32(0, 0, 0, 255) : IM_COL32(255, 255, 255, 255), text.c_str());
	}
	if (m_head_hover >= 0 && m_head_hover < static_cast<int>(m_heads.size()))
	{
		auto const &head{m_heads[m_head_hover]};
		ImVec2 label;
		if (projection.project(head.position + glm::dvec3{0.0, 7.0, 0.0}, label))
		{
			ImVec2 const foot{label.x, label.y - ImGui::GetTextLineHeight() - 8.0f};
			ImVec2 const top{label.x, foot.y - 30.0f};
			drawlist->AddLine(foot, top, kHovered, 3.0f);
			drawlist->AddCircle(ImVec2(top.x, top.y - 9.0f), 9.0f, kHovered, 0, 3.0f);
		}
	}
}

void srk_tool::render_found()
{
	if (main_button(STR_C("Find the stations of the scenery"), ImVec2(-1.0f, tall_button_height())))
		find_all();
	item_tooltip("Each group of switches joined by tracks shorter than 1.5 km is a station");
	if (m_found.empty())
		return;
	ImGui::TextDisabled("%s", format(STR_P("%zu station", "%zu stations", m_found.size()), m_found.size()).c_str());
	if (false == ImGui::BeginChild("##srkfound", ImVec2(0.0f, std::min(300.0f, ImGui::GetTextLineHeightWithSpacing() * (m_found.size() + 1.0f))), true))
	{
		ImGui::EndChild();
		return;
	}
	for (std::size_t index = 0; index < m_found.size(); ++index)
	{
		auto const &[name, count]{m_found[index]};
		auto const described{std::find_if(m_records.begin(), m_records.end(), [&](record const &Entry) { return anchor_of(Entry) == name; })};
		auto label{format(STR_P("%s: %zu switch", "%s: %zu switches", count), name.c_str(), count)};
		if (described != m_records.end() && false == described->name.empty())
			label = described->name + " - " + label;
		if (ImGui::Selectable((label + "##srkfound" + std::to_string(index)).c_str(), false))
			if (auto *track{simulation::Paths.find(name)}; track != nullptr)
			{
				open(*track);
				m_selection.set_current_node(track);
				m_picked = track;
				m_view.focus_track(*track, 0, track->m_paths.front().points[segment_data::point::start]);
			}
	}
	ImGui::EndChild();
}

void srk_tool::render_options()
{
	if (false == m_layout.has_value())
	{
		ImGui::TextDisabled("%s", STR_C("The signalling of a station: its schematic, the numbers of the switches and the tracks, the track circuits and the signals"));
		ImGui::Spacing();
		gesture_rows({{"LMB on a switch", "opens its station"}});
		if (false == m_status.empty())
			ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", m_status.c_str());
		ImGui::Spacing();
		render_found();
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
	if (begin_fields("##srkstation"))
	{
		auto const text = [&](char const *Label, char const *Id, std::string const &Value, char const *Hint, auto const &Set) {
			field_row(Label);
			std::array<char, 128> buffer{};
			Value.copy(buffer.data(), buffer.size() - 1);
			if (ImGui::InputTextWithHint(Id, Hint, buffer.data(), buffer.size()))
			{
				Set(std::string{buffer.data()});
				m_changed = true;
			}
		};
		auto const *entry{m_record >= 0 ? &m_records[m_record] : nullptr};
		text("Name", "##srkname", entry != nullptr ? entry->name : std::string{}, "Biała Podlaska", [&](std::string const &Value) { record_of_open().name = Value; });
		text("Code", "##srkcode", entry != nullptr ? entry->code : std::string{}, "BPd", [&](std::string const &Value) { record_of_open().code = word_of(Value) == "-" ? std::string{} : word_of(Value); });
		item_tooltip("The abbreviation of the station: the names of its switches, track circuits and signals begin with it");
		auto const line_hint{line_of_open()};
		text("Line", "##srkline", entry != nullptr ? entry->line : std::string{}, line_hint.empty() ? "2" : line_hint.c_str(), [&](std::string const &Value) { record_of_open().line = word_of(Value) == "-" ? std::string{} : word_of(Value); });
		item_tooltip("The number of the railway line the station lies on, as the block sections between the stations are named after it");
		double known{0.0};
		if (km_of_line((layout.from + layout.to) * 0.5, known))
		{
			field_row("Km");
			ImGui::TextDisabled("%s", format(STR_C("%s, of the kilometrage of line %s"), kilometrage::text(known).c_str(), line_of_open().c_str()).c_str());
			item_tooltip("The km of the middle of the station, as the Kilometrage tool sets them for the line, the jumps too: the block signals of the lines are numbered after them");
		}
		else
		{
			auto const hint{layout.kilometrage && km_of_open((layout.from + layout.to) * 0.5, known) ? format("%.3f", known) : std::string{"189.563"}};
			text("Km", "##srkkm", entry != nullptr ? entry->km : std::string{}, hint.c_str(), [&](std::string const &Value) {
				auto km{word_of(Value)};
				std::replace(km.begin(), km.end(), ',', '.');
				record_of_open().km = km == "-" ? std::string{} : km;
			});
			item_tooltip("The km of the middle of the station along the line, rising the way the switches are numbered: the block signals of the lines are numbered after the km");
		}
		ImGui::EndTable();
	}
	step_heading(1, STR_C("Switch numbers"));
	int from{m_reversed ? 1 : 0};
	if (segmented("##srkfrom", from,
	              {{0, STR_C("From the left"), layout.kilometrage ? "From the lower km, as PKP PLK numbers them" : "From the west end of the station"},
	               {1, STR_C("From the right"), layout.kilometrage ? "From the higher km" : "From the east end of the station"}}))
	{
		m_reversed = from == 1;
		record_of_open().reversed = m_reversed;
		m_changed = true;
	}
	if (ImGui::Button(STR_C("Number the switches"), ImVec2(-1.0f, 0.0f)))
		number_switches();
	item_tooltip("Numbers the switches 1, 2, 3... along the station and names them after its code, as on the lines of PKP PLK: bpd_1, "
	             "the parts of a slip bpd_15a to bpd_15d. The drives and the objects naming a switch follow it, events written by hand keep the old names. One step of undo");
	step_heading(2, STR_C("Track circuits"));
	if (main_button(STR_C("Set the track circuits"), ImVec2(-1.0f, tall_button_height())))
		set_circuits();
	item_tooltip("Cuts the tracks of the station at the joints of the track circuits and names the circuits, as on the lines of PKP PLK: a "
	             "joint 1.5 m before the start of a switch, 4.2 m beyond the fouling point of each branch, where the axes are 3.5 m apart, as "
	             "Ie-4 gives it, in the middle of a link too short for both. Each switch gets bpd_iz<number>, each station track "
	             "bpd_it<number>: 1 and 2 the main tracks, track 1 on the right along the km, the odd ones beyond it, the even ones on the "
	             "other side. Run again after changes, it keeps the numbers the tracks have. One step of undo");
	step_heading(3, STR_C("Signals"));
	if (ImGui::Button(STR_C("Set the signals"), ImVec2(-1.0f, 0.0f)))
		set_signals();
	item_tooltip("Places the signals of the station, as on the lines of PKP PLK, and names them after its code: an entry signal 300 m out on "
	             "each line, A, B... from the start of the km, Z, Y... from the end, the circuit from 15 m beyond it to the station bpd_ita; "
	             "an exit signal 15 m before the joint at the end of each station track a train is sent out of, F1, G2 towards the start, R1, "
	             "P2 towards the end; a shunting signal Tm1, Tm2... at the end of a shorter track. A signal standing by the place of one gets "
	             "its name, none is doubled. The kinds of signals chosen in the Signals tool. One step of undo");
	step_heading(4, STR_C("Lines"));
	if (ImGui::Button(STR_C("Set the line signals"), ImVec2(-1.0f, 0.0f)))
		set_lines();
	item_tooltip("Places the signals of the lines out of the station, from its entry signals on. A double-track line gets the automatic block: "
	             "block signals evenly to the entry signals of the next station, a section no shorter than the braking distance, numbered "
	             "after the hectometre ahead of them, odd on track 1, even on track 2, and the block sections from 15 m beyond them named as "
	             "on the lines of PKP PLK, 2411-2571@LK2.1. A single-track line gets a warning disc ToA the braking distance before the entry "
	             "signal. The braking distance after the speed of the line, as Ie-4 gives it: 400 m up to 60 km/h, 700 m up to 100, 1000 m up "
	             "to 140, 1300 m above. Needs the line and the km of the station. Run it at the next station too, it keeps what stands. One "
	             "step of undo");
	if (auto const *selected{m_selection.selected_track()}; selected != nullptr && false == selected->Isolated.empty())
		if (auto const number{track_of(selected->Isolated.front()->asName, stem_of_open())}; false == number.empty() && begin_fields("##srktrack"))
		{
			field_row("Track number");
			std::array<char, 16> buffer{};
			number.copy(buffer.data(), buffer.size() - 1);
			if (ImGui::InputText("##srktracknumber", buffer.data(), buffer.size(), ImGuiInputTextFlags_EnterReturnsTrue) || ImGui::IsItemDeactivatedAfterEdit())
				renumber_track(*selected, buffer.data());
			item_tooltip("The number of the selected station track: type another one and its track circuits are renamed, bpd_it7a, bpd_it7b. One step of undo");
			ImGui::EndTable();
		}
	if (false == m_status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", m_status.c_str());
	ImGui::Spacing();
	m_length_hover = -1;
	if (ImGui::CollapsingHeader(format("%s (%zu)###srklengths", STR_C("Track lengths"), m_lengths.size()).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
		render_lengths();
	ImGui::Spacing();
	m_head_hover = -1;
	if (ImGui::CollapsingHeader(format("%s (%zu)###srkheads", STR_C("Signal heads"), static_cast<std::size_t>(std::count_if(m_heads.begin(), m_heads.end(), [](head_check const &Head) { return Head.station; }))).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
		render_heads();
	ImGui::Spacing();
	if (ImGui::CollapsingHeader(format("%s (%zu)###srkswitches", STR_C("Switches"), layout.switches.size()).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::BeginTable("##srkswitchlist", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY, ImVec2(0.0f, std::min(320.0f, ImGui::GetTextLineHeightWithSpacing() * (layout.switches.size() + 1.5f)))))
		{
			ImGui::TableSetupScrollFreeze(0, 1);
			ImGui::TableSetupColumn(STR_C("Switch"), ImGuiTableColumnFlags_WidthStretch, 0.35f);
			ImGui::TableSetupColumn(STR_C("Type"), ImGuiTableColumnFlags_WidthStretch, 0.45f);
			ImGui::TableSetupColumn(STR_C("km"), ImGuiTableColumnFlags_WidthStretch, 0.2f);
			ImGui::TableHeadersRow();
			auto *selected{m_selection.selected_track()};
			for (std::size_t index = 0; index < layout.switches.size(); ++index)
			{
				auto *track{layout.switches[index]};
				auto const start{track->m_paths.front().points[segment_data::point::start]};
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				if (ImGui::Selectable((track->name() + "##srksw" + std::to_string(index)).c_str(), track == selected, ImGuiSelectableFlags_SpanAllColumns))
				{
					m_selection.set_current_node(track);
					m_picked = track;
					m_view.focus_track(*track, 0, start);
				}
				ImGui::TableNextColumn();
				auto const record{switch_store::named(*track)};
				ImGui::TextUnformatted(record ? switch_store::label(*record).c_str() : track->eType == tt_Switch ? STR_C("switch") : track->eType == tt_Cross ? STR_C("crossing") : STR_C("turntable"));
				ImGui::TableNextColumn();
				auto const along{station::project(layout, start).x};
				if (layout.kilometrage)
					ImGui::Text("%.3f", station::km_of(layout, along));
				else
					ImGui::Text("%.0f m", along);
			}
			ImGui::EndTable();
		}
	}
	ImGui::Spacing();
	if (ImGui::Button(STR_C("Another station"), ImVec2(-1.0f, 0.0f)))
	{
		m_layout.reset();
		m_anchor = nullptr;
		m_record = -1;
		m_hover = -1;
	}
	item_tooltip("Leaves this station; LMB on a switch opens another one");
}

void srk_tool::render_lengths()
{
	auto const dimmed = [](char const *Text) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextWrapped("%s", Text);
		ImGui::PopStyleColor();
	};
	if (m_lengths.empty())
	{
		if (stem_of_open().empty())
			dimmed(STR_C("Give the station its code first: its tracks are numbered and measured after it"));
		else if (m_numbered)
			dimmed(STR_C("The station tracks have no ends to be measured between"));
		else
		{
			dimmed(STR_C("Set the track circuits first: the station tracks are measured by their numbers, from their joints, the fouling points and the signals"));
			if (ImGui::Button(STR_C("Set the track circuits"), ImVec2(-1.0f, 0.0f)))
				set_circuits();
		}
		return;
	}
	auto const stem{stem_of_open()};
	char const *const ways[] = {STR_C("To the right"), STR_C("To the left")};
	if (ImGui::BeginTable("##srklengthlist", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY, ImVec2(0.0f, std::min(320.0f, ImGui::GetTextLineHeightWithSpacing() * (m_lengths.size() + 1.5f)))))
	{
		ImGui::TableSetupScrollFreeze(0, 1);
		ImGui::TableSetupColumn(STR_C("Track"), ImGuiTableColumnFlags_WidthStretch, 0.16f);
		ImGui::TableSetupColumn(ways[0], ImGuiTableColumnFlags_WidthStretch, 0.28f);
		ImGui::TableSetupColumn(ways[1], ImGuiTableColumnFlags_WidthStretch, 0.28f);
		ImGui::TableSetupColumn(STR_C("Train up to"), ImGuiTableColumnFlags_WidthStretch, 0.28f);
		ImGui::TableHeadersRow();
		for (std::size_t index = 0; index < m_lengths.size(); ++index)
		{
			auto const &track{m_lengths[index]};
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			if (ImGui::Selectable((track.number + "##srklength" + std::to_string(index)).c_str(), static_cast<int>(index) == m_length_shown, ImGuiSelectableFlags_SpanAllColumns))
			{
				// the track at the middle of its useful length selected, the camera over it
				auto const &way{track.ways[0]};
				auto const middle{stretch_of(track, way.rear.along, way.head.along)};
				auto const at{middle[middle.size() / 2]};
				station::path const *nearest{nullptr};
				auto best{std::numeric_limits<double>::max()};
				for (auto const &path : m_layout->paths)
					for (auto const &point : path.points)
						if (auto const distance{glm::distance(glm::dvec2{point.x, point.z}, glm::dvec2{at.x, at.z})}; false == path.approach && distance < best)
						{
							best = distance;
							nearest = &path;
						}
				if (nearest != nullptr)
				{
					m_selection.set_current_node(nearest->track);
					m_picked = nearest->track;
					m_view.focus_track(*nearest->track, nearest->index, at);
				}
			}
			if (ImGui::IsItemHovered())
			{
				m_length_hover = static_cast<int>(index);
				ImGui::BeginTooltip();
				ImGui::TextUnformatted(format(STR_C("Track %s"), track.number.c_str()).c_str());
				for (int way = 0; way < 2; ++way)
				{
					auto const &part{track.ways[way]};
					ImGui::TextUnformatted(format(STR_C("%s: %.0f m, from %s to %s"), ways[way], part.length, limit_text(part.rear, stem).c_str(), limit_text(part.head, stem).c_str()).c_str());
				}
				ImGui::TextDisabled(STR_C("Construction length %.0f m: between the switches at its ends, the switches within it left out"), track.construction);
				if (std::abs(track.ways[0].length - track.ways[1].length) > kUnequal)
					ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", STR_C("The two ways differ: their trains stop on other stretches of the track, between other switches or signals"));
				ImGui::TextDisabled("%s", STR_C("LMB: show it"));
				ImGui::EndTooltip();
			}
			auto const unequal{std::abs(track.ways[0].length - track.ways[1].length) > kUnequal};
			for (int way = 0; way < 2; ++way)
			{
				ImGui::TableNextColumn();
				auto const &part{track.ways[way]};
				if (unequal)
					ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, part.head.what == station::limit::kind::signal ? 1.0f : 0.6f), "%.0f m", part.length);
				else if (part.head.what == station::limit::kind::signal)
					ImGui::Text("%.0f m", part.length);
				else
					ImGui::TextDisabled("%.0f m", part.length);
			}
			ImGui::TableNextColumn();
			ImGui::Text("%.0f m", std::max(0.0, std::min(track.ways[0].length, track.ways[1].length) - kTrainMargin));
		}
		ImGui::EndTable();
	}
	if (std::none_of(m_lengths.begin(), m_lengths.end(), [](station::lengths const &Track) { return Track.ways[0].head.what == station::limit::kind::signal || Track.ways[1].head.what == station::limit::kind::signal; }))
	{
		dimmed(STR_C("No signal stands by the station tracks yet, the lengths run between the fouling points: set the signals for the useful ones"));
		if (ImGui::Button(STR_C("Set the signals"), ImVec2(-1.0f, 0.0f)))
			set_signals();
	}
	if (ImGui::Button(STR_C("Copy the list of tracks"), ImVec2(-1.0f, 0.0f)))
	{
		ImGui::SetClipboardText(lengths_table().c_str());
		m_status = format(STR_P("The lengths of %zu track copied", "The lengths of %zu tracks copied", m_lengths.size()), m_lengths.size());
	}
	item_tooltip("The lengths of the station tracks as a table to paste into a spreadsheet: the construction length and the useful one each way with its limits, as the list of the tracks of a station gives them");
	dimmed(STR_C("The useful length: where a train stands, from the signal it stops by back to the joint, the fouling point or the level crossing behind it, between those where no signal stands (dimmed). The longest train leaves 25 m of it: 10 m to stop, 10 m to see the signal, 5 m to the joint"));
}

void srk_tool::render_heads()
{
	auto const dimmed = [](char const *Text) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextWrapped("%s", Text);
		ImGui::PopStyleColor();
	};
	if (begin_fields("##srkbraking"))
	{
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
		auto changed{false};
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
		ImGui::EndTable();
		if (changed)
		{
			m_changed = true;
			survey_signals();
		}
	}
	if (false == m_heads_known)
	{
		dimmed(STR_C("Looking for the signal templates..."));
		return;
	}
	if (std::none_of(m_heads.begin(), m_heads.end(), [](head_check const &Head) { return Head.station; }))
	{
		dimmed(STR_C("No main signal stands by the station yet: its lights are checked once one does"));
		if (ImGui::Button((std::string{STR_C("Set the signals")} + "##srkheads").c_str(), ImVec2(-1.0f, 0.0f)))
			set_signals();
		return;
	}
	auto const line{speed_of_open()};
	std::vector<std::size_t> rows;
	std::size_t fine{0};
	for (std::size_t index = 0; index < m_heads.size(); ++index)
	{
		auto const &head{m_heads[index]};
		if (head.station || false == head.fits || false == head.problems.empty())
			rows.push_back(index);
		else
			++fine;
	}
	if (ImGui::BeginTable("##srkheadlist", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY, ImVec2(0.0f, std::min(320.0f, ImGui::GetTextLineHeightWithSpacing() * (rows.size() + 1.5f)))))
	{
		ImGui::TableSetupScrollFreeze(0, 1);
		ImGui::TableSetupColumn(STR_C("Signal"), ImGuiTableColumnFlags_WidthStretch, 0.25f);
		ImGui::TableSetupColumn(STR_C("Speeds"), ImGuiTableColumnFlags_WidthStretch, 0.23f);
		ImGui::TableSetupColumn(STR_C("Lights"), ImGuiTableColumnFlags_WidthStretch, 0.25f);
		ImGui::TableSetupColumn(STR_C("State"), ImGuiTableColumnFlags_WidthStretch, 0.27f);
		ImGui::TableHeadersRow();
		for (auto const index : rows)
		{
			auto const &head{m_heads[index]};
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			if (ImGui::Selectable((head.name + "##srkhead" + std::to_string(index)).c_str(), static_cast<int>(index) == m_head_hover, ImGuiSelectableFlags_SpanAllColumns))
				m_view.fly_to(head.position + glm::dvec3{0.0, 4.0, 0.0}, glm::normalize(glm::dvec3{head.travel.x, -0.3, head.travel.y}), 30.0);
			if (ImGui::IsItemHovered())
			{
				m_head_hover = static_cast<int>(index);
				ImGui::BeginTooltip();
				ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32.0f);
				auto const *kind{head.kind == "main" ? STR_C("Main signal %s") : head.kind == "block" ? STR_C("Block signal %s") : STR_C("Warning disc %s")};
				ImGui::TextUnformatted(format(kind, head.name.c_str()).c_str());
				ImGui::TextDisabled("%s", head.file.c_str());
				for (std::size_t k = 0; k < head.routes.size(); ++k)
				{
					auto const &route{head.routes[k]};
					std::string text;
					if (route.next >= 0)
						text = format(STR_C("To %s: %.0f m"), head.nexts[k].c_str(), route.length);
					else if (route.ended)
						text = format(STR_C("To the end of the tracks: %.0f m"), route.length);
					else
						text = format(STR_C("No signal within %.0f m"), route.length);
					if (head.kind == "main")
						text += ", " + (route.speed > 0.0 && shown_speed(route.speed, line) > 0 ? format(STR_C("%d km/h through the switches"), shown_speed(route.speed, line)) : format(STR_C("%.0f km/h"), line));
					std::string turnouts;
					for (auto const *turnout : route.turnouts)
						turnouts += (turnouts.empty() ? "" : ", ") + without_stem(turnout->name(), stem_of_open());
					if (false == turnouts.empty())
						text += ", " + format(STR_P("off the straight at switch %s", "off the straight at switches %s", route.turnouts.size()), turnouts.c_str());
					if (route.warning >= 0)
						text += ", " + STR("past a warning disc");
					ImGui::BulletText("%s", text.c_str());
				}
				if (head.kind != "warning")
				{
					head_need need;
					need.speeds = head.speeds;
					need.informs = head.informs;
					need.white = head.white;
					ImGui::TextUnformatted(format(STR_C("Needs: %s"), need_text(need).c_str()).c_str());
				}
				if (head.shortened > 0)
					ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", format(STR_C("The next signal stands nearer than the braking distance: with it at stop this one slows the trains to %d km/h, Ie-4 § 8 ust. 5"), head.shortened).c_str());
				if (head.indicator)
					ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", STR_C("Give it a W21 indicator \"14\": 140 km/h with the next signal at stop, Ie-4 § 8 ust. 5"));
				if (head.lamps.empty() && head.kind != "warning")
					ImGui::TextDisabled("%s", STR_C("A semaphore with arms: its lights aren't checked"));
				else if (false == head.fits && head.fit.empty())
					ImGui::TextColored(kWrong, "%s", STR_C("No template of its kind and mount has these lights"));
				else if (false == head.fits)
					ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", format(STR_C("Its lights don't show all of it: %s does"), head.fit.c_str()).c_str());
				else if (head.more)
					ImGui::TextDisabled("%s", format(STR_C("It has more lights than it needs: %s will do"), head.fit.c_str()).c_str());
				for (auto const &problem : head.problems)
					ImGui::TextColored(kWrong, "%s", problem.c_str());
				ImGui::TextDisabled("%s", STR_C("LMB: fly to it"));
				ImGui::PopTextWrapPos();
				ImGui::EndTooltip();
			}
			ImGui::TableNextColumn();
			if (head.kind == "main")
			{
				auto text{format("%.0f", line)};
				for (auto speed{head.speeds.rbegin()}; speed != head.speeds.rend(); ++speed)
					text += ", " + std::to_string(*speed);
				ImGui::TextUnformatted(text.c_str());
			}
			else
				ImGui::TextDisabled("-");
			ImGui::TableNextColumn();
			if (head.lamps.empty())
				ImGui::TextDisabled("%s", head.kind == "warning" ? STR_C("disc") : STR_C("arms"));
			else
				lamp_row(head.lamps, head.own);
			ImGui::TableNextColumn();
			if (head.lamps.empty())
				ImGui::TextDisabled("-");
			else if (false == head.fits)
				ImGui::TextColored(head.fit.empty() ? kWrong : ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", head.fit.empty() ? STR_C("no template") : STR_C("lights missing"));
			else if (head.more)
				ImGui::TextDisabled("%s", STR_C("more than needed"));
			else
				ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f), "%s", STR_C("fits"));
			if (false == head.problems.empty())
			{
				ImGui::SameLine();
				ImGui::TextColored(kWrong, "%s", STR_C("too near"));
			}
		}
		ImGui::EndTable();
	}
	if (fine > 0)
		ImGui::TextDisabled("%s", format(STR_P("%zu signal of the lines fits", "%zu signals of the lines fit", fine), fine).c_str());
	auto const fitting{static_cast<std::size_t>(std::count_if(m_heads.begin(), m_heads.end(), [](head_check const &Head) { return false == Head.fit.empty() && Head.fit != Head.file && (false == Head.fits || Head.more); }))};
	ImGui::BeginDisabled(fitting == 0);
	if (main_button(format("%s (%zu)###srkfitheads", STR_C("Fit the lights"), fitting).c_str(), ImVec2(-1.0f, 0.0f)))
		fit_heads();
	ImGui::EndDisabled();
	item_tooltip("Gives each signal marked the template of its kind and mount with the lights its routes need and the least beyond them, the name, the plate and the place kept, the trains reading it as before. One step of undo");
	dimmed(STR_C("The lights go by the routes of the trains: 40 km/h through the switches 1:9, 60 through 1:12 and 1:14, 100 through 1:18.5, by their radii, for these a lower orange light, with an orange strip for 60, a green one for 100, Ie-1; an upper orange one where the next signal can show stop; the white one of the substitute signal. A signal telling of the next one standing nearer than the braking distance slows the trains with it at stop, Ie-4 § 8 ust. 5"));
}

std::string srk_tool::lengths_table() const
{
	auto const stem{stem_of_open()};
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

int srk_tool::length_of(TTrack const *Track) const
{
	if (Track == nullptr || Track->Isolated.empty())
		return -1;
	auto const number{track_of(Track->Isolated.front()->asName, stem_of_open())};
	for (std::size_t index = 0; index < m_lengths.size() && false == number.empty(); ++index)
		if (m_lengths[index].number == number)
			return static_cast<int>(index);
	return -1;
}

void srk_tool::number_switches()
{
	m_status.clear();
	auto const code{m_record >= 0 ? m_records[m_record].code : std::string{}};
	if (code.empty())
	{
		m_status = STR_C("Give the station its code first: the switches are named after it");
		return;
	}
	if (m_records[m_record].reversed != m_reversed)
	{
		m_records[m_record].reversed = m_reversed;
		m_changed = true;
	}
	auto const units{station::numbering(*m_layout, m_reversed)};
	std::vector<std::pair<TTrack *, std::string>> names;
	std::unordered_set<TTrack const *> own;
	for (std::size_t index = 0; index < units.size(); ++index)
		for (std::size_t part = 0; part < units[index].parts.size(); ++part)
		{
			names.emplace_back(units[index].parts[part], station::switch_name(code, static_cast<int>(index) + 1, part, units[index].parts.size()));
			own.insert(units[index].parts[part]);
		}
	for (auto &[track, name] : names)
	{
		std::string reason;
		TTrack *owner{nullptr};
		if (false == scene::Layers.editable(track, &reason))
		{
			m_status = format(STR_C("%s can't be renamed: %s"), track->name().c_str(), reason.c_str());
			return;
		}
		if (false == editor_track::name_valid(*track, name, reason, &owner) && (owner == nullptr || own.count(owner) == 0))
		{
			m_status = format(STR_C("%s can't be named %s: %s"), track->name().c_str(), name.c_str(), reason.c_str());
			return;
		}
	}
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::map<std::string, std::string> parts;
	std::map<std::string, std::string> bases;
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
	// the objects of the scenery naming a switch, as the lanterns of rainsted/lwzm.inc, follow it
	std::map<std::string, std::string> renamed;
	for (auto const &[track, name] : names)
		if (track->name() != name)
			renamed[ToLower(track->name())] = name;
	std::vector<std::pair<scene::instance_handle, std::vector<std::string>>> following;
	for (auto &standing : m_templates.standing_templates([](std::string const &) { return true; }))
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
	// the names go round among the switches: a switch whose name another one gets gives it up first
	for (auto const &entry : names)
		if (entry.first->name() != entry.second && std::any_of(names.begin(), names.end(), [&](auto const &Other) { return Other.first != entry.first && Other.second == entry.first->name(); }))
			editor_track::rename(*entry.first, entry.first->name() + "~");
	for (auto const &[track, name] : names)
		editor_track::rename(*track, name);
	std::size_t numbered{0};
	for (std::size_t index = 0; index < units.size(); ++index)
		if (auto const record{switch_store::named(*units[index].parts.front())}; record && record->number != std::to_string(index + 1))
		{
			switch_store::number(record->name, std::to_string(index + 1));
			++numbered;
		}
	if (before.empty() && numbered == 0)
	{
		m_status = STR_C("The switches have these numbers already");
		return;
	}
	if (before.empty())
		before.emplace_back(units.front().parts.front(), editor_track::capture(*units.front().parts.front()));
	auto const followed{following.size()};
	if (following.empty())
		m_history.push_track_snapshot(std::move(before));
	else
	{
		std::string error;
		m_signals.signal_batch({{}, {}, {}, std::move(following)}, std::move(before), {}, error);
	}
	m_status = format(STR_P("%zu switch numbered", "%zu switches numbered", units.size()), units.size());
	if (followed > 0)
		m_status += ", " + format(STR_P("%zu object of the scenery follows them", "%zu objects of the scenery follow them", followed), followed);
}

void srk_tool::set_circuits()
{
	m_status.clear();
	auto const code{m_record >= 0 ? m_records[m_record].code : std::string{}};
	if (code.empty())
	{
		m_status = STR_C("Give the station its code first: the track circuits are named after it");
		return;
	}
	auto const units{station::numbering(*m_layout, m_reversed)};
	auto const planned{station::joints(*m_layout, units)};
	for (auto const &entry : planned)
		if (std::string reason; entry.track != nullptr && false == editor_track::can_edit_geometry(*entry.track, reason))
		{
			m_status = format(STR_C("%s can't be cut: %s"), name_of(*entry.track).c_str(), reason.c_str());
			return;
		}
	for (auto const &path : m_layout->paths)
		if (std::string reason; false == path.approach && false == scene::Layers.editable(path.track, &reason))
		{
			m_status = format(STR_C("%s can't be changed: %s"), name_of(*path.track).c_str(), reason.c_str());
			return;
		}
	// the cuts: each track from its end back, so the pieces left to cut stay in the track
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::vector<TTrack *> created;
	std::vector<glm::dvec3> points;
	std::vector<std::pair<TTrack *, std::vector<glm::dvec3>>> cuts;
	for (auto const &entry : planned)
	{
		points.push_back(entry.point);
		if (entry.track == nullptr)
			continue;
		auto found{std::find_if(cuts.begin(), cuts.end(), [&](auto const &Cut) { return Cut.first == entry.track; })};
		if (found == cuts.end())
			found = cuts.insert(cuts.end(), {entry.track, {}});
		found->second.push_back(entry.point);
	}
	for (auto &[track, at] : cuts)
	{
		before.emplace_back(track, editor_track::capture(*track));
		auto *const piece{track};
		std::sort(at.begin(), at.end(), [&](glm::dvec3 const &A, glm::dvec3 const &B) { return editor_track::nearest_parameter(*piece, A) > editor_track::nearest_parameter(*piece, B); });
		for (auto const &point : at)
			if (auto *cut{editor_track::split_path(*track, editor_track::nearest_parameter(*track, point))})
				created.push_back(cut);
	}
	// the circuits of the station as cut
	std::size_t count{0};
	if (auto const found{station::find(*m_anchor)})
	{
		auto const circuits{station::circuits(*found, units, points)};
		auto const numbers{station::track_numbers(*found, circuits)};
		auto const stem{station::prefix(code)};
		std::unordered_set<TTrack const *> captured(created.begin(), created.end());
		for (auto const &[track, state] : before)
			captured.insert(track);
		// a station track keeps the name most of it has, given by hand or so far, unless another one has it too; the others take
		// the numbers left on their side
		std::vector<std::string> names(circuits.size());
		std::map<std::string, std::size_t> held;
		for (std::size_t index = 0; index < circuits.size(); ++index)
		{
			auto const &circuit{circuits[index]};
			if (circuit.unit >= 0)
				continue;
			std::map<std::string, std::size_t> counts;
			for (auto const *track : circuit.tracks)
				if (track->Isolated.size() == 1 && false == track_of(track->Isolated.front()->asName, stem).empty())
					++counts[track->Isolated.front()->asName];
			if (auto const most{std::max_element(counts.begin(), counts.end(), [](auto const &A, auto const &B) { return A.second < B.second; })}; most != counts.end())
				++held[names[index] = most->first];
		}
		std::unordered_set<std::string> taken;
		for (auto &name : names)
			if (false == name.empty() && held[name] > 1)
				name.clear();
			else if (false == name.empty())
				taken.insert(name);
		for (std::size_t index = 0; index < circuits.size(); ++index)
		{
			auto const &circuit{circuits[index]};
			auto &name{names[index]};
			if (circuit.unit >= 0)
				name = stem + "iz" + unit_number(units[circuit.unit], stem, circuit.unit);
			else if (name.empty())
			{
				auto number{numbers[index]};
				while (taken.count(stem + "it" + number) > 0)
				{
					std::size_t digits{0};
					while (digits < number.size() && std::isdigit(static_cast<unsigned char>(number[digits])))
						++digits;
					if (digits == 0)
						break;
					number = std::to_string(std::stoi(number.substr(0, digits)) + 2) + number.substr(digits);
				}
				taken.insert(name = stem + "it" + number);
			}
			for (auto *track : circuit.tracks)
			{
				if (track->Isolated.size() == 1 && track->Isolated.front()->asName == name)
					continue;
				if (captured.insert(track).second)
					before.emplace_back(track, editor_track::capture(*track));
				editor_track::isolated(*track, {name});
			}
		}
		count = circuits.size();
	}
	if (before.empty() && created.empty())
	{
		m_status = STR_C("The track circuits are set already");
		return;
	}
	m_history.push_track_snapshot(std::move(before), created);
	m_status = format(STR_P("%zu track circuit", "%zu track circuits", count), count) + ", " + format(STR_P("%zu cut", "%zu cuts", created.size()), created.size());
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
	std::map<std::string, std::map<std::string, std::vector<TTrack *>>> numbers;
	for (auto const &path : m_layout->paths)
	{
		if (path.track->m_editorremoved || path.track->Isolated.empty() || Inside.count(path.track) > 0)
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

void srk_tool::renumber_track(TTrack const &Track, std::string const &Number)
{
	m_status.clear();
	auto const stem{stem_of_open()};
	auto const from{Track.Isolated.empty() ? std::string{} : track_of(Track.Isolated.front()->asName, stem)};
	std::string to;
	for (auto const letter : Number)
		if (std::isalnum(static_cast<unsigned char>(letter)))
			to += static_cast<char>(std::tolower(static_cast<unsigned char>(letter)));
	if (from.empty() || to.empty() || to == from)
		return;
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::unordered_set<TTrack const *> seen;
	for (auto const &path : m_layout->paths)
	{
		if (path.track->Isolated.empty() || false == seen.insert(path.track).second)
			continue;
		auto const number{track_of(path.track->Isolated.front()->asName, stem)};
		if (number == to)
		{
			m_status = format(STR_C("The station has a track %s already"), to.c_str());
			return;
		}
		if (number == from)
			before.emplace_back(path.track, editor_track::capture(*path.track));
	}
	for (auto &[track, state] : before)
	{
		auto const &name{track->Isolated.front()->asName};
		editor_track::isolated(*track, {stem + "it" + to + name.substr(stem.size() + 2 + from.size())});
	}
	if (before.empty())
		return;
	m_history.push_track_snapshot(std::move(before));
	m_status = format(STR_C("Track %s is track %s now"), from.c_str(), to.c_str());
}

void srk_tool::set_signals()
{
	m_status.clear();
	auto const stem{stem_of_open()};
	if (stem.empty())
	{
		m_status = STR_C("Give the station its code first: the signals are named after it");
		return;
	}
	auto const &layout{*m_layout};
	std::vector<station::numbered> tracks;
	std::unordered_set<TTrack const *> inside;
	if (false == station_tracks(tracks, inside))
	{
		m_status = STR_C("Set the track circuits first: the signals stand at their joints");
		return;
	}
	auto plan{station::signals(layout, tracks, [&](TTrack const &Track) { return inside.count(&Track) > 0; })};
	std::vector<standing_signal> standing;
	m_signals.standing_signals(standing, true);
	standing.erase(std::remove_if(standing.begin(), standing.end(), [&](standing_signal const &Signal) { return gap_to(layout, Signal.position) > 8.0; }), standing.end());
	auto const full = [&](station::signal const &Signal) { return stem + ToLower(Signal.name); };
	// the standing ones by their places, then by their names the ones moved away
	std::vector<int> match(plan.size(), -1);
	std::vector<char> used(standing.size(), 0);
	std::vector<std::tuple<double, std::size_t, std::size_t>> candidates;
	for (std::size_t i = 0; i < plan.size(); ++i)
	{
		auto const &planned{plan[i]};
		glm::dvec2 const travel{glm::normalize(glm::dvec2{planned.travel.x, planned.travel.z} + glm::dvec2{1e-12, 0.0})};
		auto const reach{planned.kind == station::signal::role::entry ? 400.0 : planned.kind == station::signal::role::exit ? 60.0 : 40.0};
		for (std::size_t k = 0; k < standing.size(); ++k)
		{
			auto const &signal{standing[k]};
			if ((signal.kind == "shunt") != (planned.kind == station::signal::role::shunting) || (signal.kind != "shunt" && signal.kind != "main") || glm::dot(travel_of(signal.yaw), travel) < 0.87)
				continue;
			glm::dvec2 const offset{signal.position.x - planned.point.x, signal.position.z - planned.point.z};
			auto const along{glm::dot(offset, travel)};
			auto const across{std::abs(offset.x * travel.y - offset.y * travel.x)};
			if (std::abs(along) < reach && across < 3.0)
				candidates.emplace_back(std::abs(along) + across * 10.0, i, k);
		}
	}
	std::sort(candidates.begin(), candidates.end());
	for (auto const &[score, i, k] : candidates)
		if (match[i] < 0 && 0 == used[k])
		{
			match[i] = static_cast<int>(k);
			used[k] = 1;
		}
	for (std::size_t i = 0; i < plan.size(); ++i)
		for (std::size_t k = 0; k < standing.size() && match[i] < 0; ++k)
			if (0 == used[k] && ToLower(standing[k].name) == full(plan[i]))
			{
				match[i] = static_cast<int>(k);
				used[k] = 1;
			}
	// the circuits from the entry signals to the station: the line cut by each one, all of it checked before any is
	struct cut
	{
		std::size_t signal{0};
		TTrack *track{nullptr};
		double parameter{0.0};
		glm::dvec3 point{0.0};
		bool outward{false};
		std::vector<TTrack *> named; // between the station and the cut
		std::vector<TTrack *> beyond; // out from the cut to the signal and on
	};
	std::vector<cut> cuts;
	for (std::size_t i = 0; i < plan.size(); ++i)
	{
		auto const &planned{plan[i]};
		if (planned.kind != station::signal::role::entry || planned.run.empty())
			continue;
		// the joint the trains pass the signal to, in from it towards the station
		auto const run{station::run_line(planned)};
		double gap{0.0};
		auto const signal_at{station::distance_on(run, match[i] >= 0 ? standing[match[i]].position : planned.point, gap)};
		auto const at{station::point_on(run, std::max(0.0, signal_at - station::signal_drop)).point};
		cut entry;
		entry.signal = i;
		auto best{std::numeric_limits<double>::max()};
		std::size_t nearest{0};
		for (std::size_t k = 0; k < planned.run.size(); ++k)
		{
			auto const t{editor_track::nearest_parameter(*planned.run[k].first, at)};
			auto const point{editor_track::point_at(*planned.run[k].first, t)};
			if (auto const distance{glm::distance(glm::dvec2{point.x, point.z}, glm::dvec2{at.x, at.z})}; distance < best)
			{
				best = distance;
				nearest = k;
				entry.parameter = t;
				entry.point = point;
			}
		}
		for (std::size_t k = 0; k < nearest; ++k)
			entry.named.push_back(planned.run[k].first);
		for (std::size_t k = nearest + 1; k < planned.run.size(); ++k)
			entry.beyond.push_back(planned.run[k].first);
		entry.track = planned.run[nearest].first;
		entry.outward = planned.run[nearest].second;
		auto const &ends{entry.track->m_paths.front().points};
		auto const to_start{glm::distance(entry.point, ends[segment_data::point::start])};
		auto const to_end{glm::distance(entry.point, ends[segment_data::point::end])};
		std::string reason;
		if (std::min(to_start, to_end) <= 0.3)
		{
			// at an end already
			((to_end < to_start) == entry.outward ? entry.named : entry.beyond).push_back(entry.track);
			entry.track = nullptr;
		}
		else if (false == editor_track::can_edit_geometry(*entry.track, reason))
		{
			m_status = format(STR_C("%s can't be cut: %s"), name_of(*entry.track).c_str(), reason.c_str());
			return;
		}
		for (auto *part : entry.named)
			if (false == scene::Layers.editable(part, &reason))
			{
				m_status = format(STR_C("%s can't be changed: %s"), name_of(*part).c_str(), reason.c_str());
				return;
			}
		cuts.push_back(std::move(entry));
	}
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::vector<TTrack *> created;
	std::unordered_set<TTrack const *> captured;
	for (auto &entry : cuts)
	{
		auto &planned{plan[entry.signal]};
		if (entry.track != nullptr)
		{
			captured.insert(entry.track);
			before.emplace_back(entry.track, editor_track::capture(*entry.track));
			if (auto *piece{editor_track::split_path(*entry.track, entry.parameter)})
			{
				created.push_back(piece);
				captured.insert(piece);
				entry.named.push_back(entry.outward ? entry.track : piece);
				entry.beyond.push_back(entry.outward ? piece : entry.track);
				if (planned.track == entry.track)
					planned.track = entry.outward ? piece : entry.track;
			}
		}
		auto const name{stem + "it" + ToLower(planned.name)};
		for (auto *part : entry.named)
		{
			if (part->Isolated.size() == 1 && part->Isolated.front()->asName == name)
				continue;
			if (captured.insert(part).second)
				before.emplace_back(part, editor_track::capture(*part));
			editor_track::isolated(*part, {name});
		}
		// out of the circuit beyond its joint, where an earlier one ended at the signal
		for (auto *part : entry.beyond)
		{
			if (part->Isolated.size() != 1 || ToLower(part->Isolated.front()->asName) != name)
				continue;
			if (captured.insert(part).second)
				before.emplace_back(part, editor_track::capture(*part));
			editor_track::isolated(*part, {});
		}
	}
	std::vector<signal_order> placed;
	std::vector<signal_rename> renamed;
	for (std::size_t i = 0; i < plan.size(); ++i)
	{
		auto const &planned{plan[i]};
		auto const plate{ToLower(planned.name)};
		if (match[i] >= 0)
		{
			if (ToLower(standing[match[i]].name) != full(planned))
				renamed.push_back({standing[match[i]].instance, full(planned), plate});
			continue;
		}
		signal_order order;
		order.track = planned.track;
		order.point = planned.point;
		order.facing = planned.facing;
		order.side = planned.side;
		order.kind = planned.kind == station::signal::role::shunting ? "shunt" : "main";
		order.name = full(planned);
		order.plate = plate;
		placed.push_back(std::move(order));
	}
	// the others keep their names, unless one of these takes it
	for (std::size_t k = 0; k < standing.size(); ++k)
		if (0 == used[k] && std::any_of(plan.begin(), plan.end(), [&](station::signal const &Signal) { return full(Signal) == ToLower(standing[k].name); }))
			renamed.push_back({standing[k].instance, ToLower(standing[k].name) + "_old", signal_shown(standing[k].name, stem) + "_old"});
	if (placed.empty() && renamed.empty() && before.empty() && created.empty())
	{
		m_status = STR_C("The signals are set already");
		return;
	}
	// the lights of the main signals placed after the routes of their trains, to the signals standing and to these
	std::vector<std::string> unfit;
	std::vector<signal_head> heads;
	m_signals.signal_heads(heads, true);
	auto const chosen{std::find_if(heads.begin(), heads.end(), [](signal_head const &Head) { return Head.kind == "main" && Head.chosen; })};
	if (chosen != heads.end())
	{
		std::vector<standing_signal> everywhere;
		m_signals.standing_signals(everywhere, true);
		std::vector<station::post> posts;
		for (auto const &signal : everywhere)
			if (auto const post{post_of(signal)}; post.track != nullptr)
				posts.push_back(post);
		std::vector<std::size_t> from;
		for (auto const &order : placed)
		{
			if (order.kind == "main")
				from.push_back(posts.size());
			posts.push_back({order.track, order.facing, order.point, order.kind == "shunt" ? station::signal::role::shunting : station::signal::role::entry});
		}
		auto const found{station::routes(posts, from, route_reach(braking_of_open()))};
		auto const line{speed_of_open()};
		std::size_t next{0};
		for (auto &order : placed)
		{
			if (order.kind != "main")
				continue;
			auto const need{need_of(found[next++], true, line, braking_of_open())};
			if (auto const best{best_head(heads, "main", chosen->mount, chosen->name, need)}; best >= 0)
				order.file = heads[best].file;
			else
				unfit.push_back(signal_shown(order.name, stem));
		}
	}
	std::string error;
	auto const count{m_signals.signal_batch({placed, renamed, {}, {}, {}}, std::move(before), std::move(created), error)};
	if (false == error.empty())
		m_status = format(STR_C("The signals couldn't be placed: %s"), error.c_str());
	else
		m_status = format(STR_P("%zu signal placed", "%zu signals placed", count), count) + ", " + format(STR_P("%zu renamed", "%zu renamed", renamed.size()), renamed.size());
	if (false == unfit.empty())
	{
		std::string names;
		for (auto const &name : unfit)
			names += (names.empty() ? "" : ", ") + name;
		m_status += "\n" + format(STR_C("No template has the lights %s need, they got the one chosen in the Signals tool: see Signal heads"), names.c_str());
	}
}

void srk_tool::set_lines()
{
	m_status.clear();
	auto const stem{stem_of_open()};
	if (stem.empty())
	{
		m_status = STR_C("Give the station its code first: the signals are named after it");
		return;
	}
	auto const &layout{*m_layout};
	std::vector<station::numbered> tracks;
	std::unordered_set<TTrack const *> inside;
	if (false == station_tracks(tracks, inside))
	{
		m_status = STR_C("Set the track circuits first: the signals stand at their joints");
		return;
	}
	auto const plan{station::signals(layout, tracks, [&](TTrack const &Track) { return inside.count(&Track) > 0; })};
	std::vector<standing_signal> standing;
	m_signals.standing_signals(standing, true);
	// the lines begin at the entry signals standing
	std::vector<station::signal> entries;
	std::vector<std::size_t> entry_standing;
	for (auto const &planned : plan)
	{
		if (planned.kind != station::signal::role::entry)
			continue;
		auto const found{std::find_if(standing.begin(), standing.end(), [&](standing_signal const &Signal) { return Signal.kind == "main" && ToLower(Signal.name) == stem + ToLower(planned.name); })};
		if (found == standing.end())
			continue;
		entries.push_back(planned);
		entry_standing.push_back(static_cast<std::size_t>(found - standing.begin()));
	}
	if (entries.empty())
	{
		m_status = STR_C("Set the signals first: the lines begin at the entry signals");
		return;
	}
	auto lines{station::lines_out(layout, entries, kLineLimit)};
	if (lines.empty())
	{
		m_status = STR_C("No line leads out of the station beyond its entry signals");
		return;
	}
	// the distances from the entry signals as they stand
	for (auto &line : lines)
	{
		double gap{0.0};
		auto const at{station::distance_on(line, standing[entry_standing[line.entry]].position, gap)};
		if (gap < 4.5)
			for (auto &distance : line.distances)
				distance -= at;
	}
	auto const described{line_of_open()};
	double known{0.0};
	if (std::any_of(lines.begin(), lines.end(), [](station::line const &Line) { return Line.number > 0; }) && (false == km_of_open(0.0, known) || described.empty()))
	{
		m_status = STR_C("Give the station its line and its km first: the block signals and the block sections are named after them");
		return;
	}
	auto const middle{(layout.from + layout.to) * 0.5};
	struct planned_signal
	{
		station::signal::role kind{station::signal::role::block};
		std::string name;
		std::string plate;
		std::size_t line{0};
		double distance{0.0}; // out along the line
		std::string linked{"none"};
		int match{-1}; // of the standing signals
	};
	struct far_end
	{
		int signal{-1}; // the entry signal of the next station standing, of the standing signals
		double distance{0.0}; // out to it, or to where the signals of the line end
	};
	std::vector<planned_signal> planned;
	std::vector<far_end> ends(lines.size());
	auto const facing_out = [&](station::line const &Line, standing_signal const &Signal, double const Distance) {
		auto const way{station::point_on(Line, Distance).way};
		return glm::dot(travel_of(Signal.yaw), glm::dvec2{way.x, way.z});
	};
	// the lines of the kilometrage the lines out run along, at a junction another one than the line of the station
	std::vector<std::string> numbers(lines.size(), described);
	for (std::size_t index = 0; index < lines.size(); ++index)
	{
		auto const out{station::point_on(lines[index], std::min(2000.0, lines[index].length() * 0.5)).point};
		double km{0.0};
		if (false == kilometrage::km_at(described, out, km))
			if (auto const found{kilometrage::line_at(out)}; false == found.empty())
				numbers[index] = found;
	}
	auto const km_on = [&](std::size_t const Index, double const Distance) {
		double km{0.0};
		if (kilometrage::km_at(numbers[Index], station::point_on(lines[Index], Distance).point, km))
			return km;
		auto const chainage{station::project(layout, station::point_on(lines[Index], 0.0).point).x};
		km_of_open(chainage < middle ? chainage - Distance : chainage + Distance, km);
		return km;
	};
	// the trains of the line run towards the higher km
	auto const rising = [&](std::size_t const Index) {
		auto const at{station::point_on(lines[Index], 100.0)};
		if (auto const growth{kilometrage::growth_at(numbers[Index], at.point, {at.way.x, at.way.z})}; growth != 0)
			return (growth > 0) != lines[Index].inbound;
		return (km_on(Index, 1000.0) > km_on(Index, 0.0)) != lines[Index].inbound;
	};
	// the braking distance at the highest speed out along the line, at 160 km/h while its tracks have none, unless one is set by hand
	auto const braking = [&](std::size_t const Index) {
		if (m_record >= 0 && m_records[m_record].braking > 0)
			return static_cast<double>(m_records[m_record].braking);
		auto speed{0.0};
		auto const &line{lines[Index]};
		for (std::size_t k = 0; k < line.points.size(); ++k)
			if (auto const set{line.tracks[line.owners[k]]->VelocityGet()}; line.distances[k] >= 0.0 && set <= kFastest)
				speed = std::max(speed, set);
		return braking_distance(speed > 0.0 ? speed : 160.0);
	};
	for (std::size_t index = 0; index < lines.size(); ++index)
	{
		auto const &line{lines[index]};
		auto &end{ends[index]};
		// the entry signal of the next station: the first main signal along the line facing out
		for (std::size_t k = 0; k < standing.size(); ++k)
		{
			if (standing[k].kind != "main")
				continue;
			double gap{0.0};
			auto const distance{station::distance_on(line, standing[k].position, gap)};
			if (gap > 4.5 || distance < 100.0 || distance > line.length() - 1.0 || facing_out(line, standing[k], distance) < 0.87)
				continue;
			if (end.signal < 0 || distance < end.distance)
			{
				end.signal = static_cast<int>(k);
				end.distance = distance;
			}
		}
		if (end.signal < 0)
			end.distance = line.station ? line.length() - kPredicted : line.length();
	}
	for (std::size_t index = 0; index < lines.size(); ++index)
	{
		auto const &line{lines[index]};
		auto const &end{ends[index]};
		auto const &entry{entries[line.entry]};
		auto const least{braking(index)};
		if (line.number > 0)
		{
			// the block signals, Ie-4 § 51: the sections no shorter than the braking distance and no longer than twice it, the line to the
			// next station parted evenly, towards the end of the tracks a little over the braking distance each
			std::vector<double> places;
			if (line.station || end.signal >= 0)
			{
				// as many on both tracks of the line
				auto const reach{line.partner >= 0 ? (end.distance + ends[line.partner].distance) * 0.5 : end.distance};
				auto const sections{std::max(1.0, std::floor(reach / (line.partner >= 0 ? std::max(least, braking(line.partner)) : least)))};
				for (auto k{1.0}; k < sections; k += 1.0)
					places.push_back(end.distance * k / sections);
			}
			else
				for (auto distance{least * kBlockShare}; distance <= end.distance - 100.0; distance += least * kBlockShare)
					places.push_back(distance);
			for (auto const distance : places)
			{
				planned_signal signal;
				signal.kind = station::signal::role::block;
				signal.name = station::block_number(km_on(index, distance), line.number, rising(index));
				signal.plate = signal.name;
				signal.line = index;
				signal.distance = distance;
				planned.push_back(std::move(signal));
			}
		}
		else
		{
			// the warning disc of the entry signal, the braking distance before it
			auto const distance{std::min(least, end.distance - 100.0)};
			if (distance >= 400.0)
			{
				planned_signal signal;
				signal.kind = station::signal::role::warning;
				signal.name = stem + "to" + ToLower(entry.name);
				signal.plate = "to" + ToLower(entry.name);
				signal.line = index;
				signal.distance = distance;
				planned.push_back(std::move(signal));
			}
		}
	}
	// the standing ones by their places, then by their names
	std::vector<char> used(standing.size(), 0);
	for (auto const index : entry_standing)
		used[index] = 1;
	for (auto const &end : ends)
		if (end.signal >= 0)
			used[end.signal] = 1;
	std::vector<std::tuple<double, std::size_t, std::size_t>> candidates;
	std::vector<double> standing_at(standing.size(), 0.0);
	for (std::size_t i = 0; i < planned.size(); ++i)
	{
		auto const &signal{planned[i]};
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
			if (gap < 4.5 && std::abs(distance - signal.distance) < reach && (line.inbound ? -facing : facing) > 0.87)
				candidates.emplace_back(std::abs(distance - signal.distance) + gap * 10.0, i, k);
		}
	}
	std::sort(candidates.begin(), candidates.end());
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
	// a number another signal of the scenery has already gets a letter
	std::unordered_set<std::string> taken;
	for (std::size_t k = 0; k < standing.size(); ++k)
		if (0 == used[k])
			taken.insert(ToLower(standing[k].name));
	for (auto &signal : planned)
	{
		if (signal.kind != station::signal::role::block)
			continue;
		auto const base{signal.name};
		for (char letter = 'a'; taken.count(ToLower(signal.name)) > 0 && letter <= 'z'; ++letter)
			signal.name = base + letter;
		taken.insert(ToLower(signal.name));
		signal.plate = signal.name;
	}
	// the block sections of each track of a double-track line, between the signals in the order the trains pass them
	struct section
	{
		std::string name;
		std::size_t line{0};
		double from{0.0};
		double to{0.0};
		std::string guard; // the block signal at its start, empty for a station one
		glm::dvec3 start{0.0};
	};
	std::vector<section> sections;
	std::vector<std::pair<std::size_t, std::string>> relinked; // standing station signals, the block signal before each
	for (std::size_t index = 0; index < lines.size(); ++index)
	{
		auto const &line{lines[index]};
		if (line.number == 0)
			continue;
		struct stop
		{
			double distance{0.0};
			std::string name;
			int signal{-1}; // of the planned ones, -1 for a station signal
			int standing{-1};
		};
		std::vector<stop> order{{0.0, standing[entry_standing[line.entry]].name, -1, static_cast<int>(entry_standing[line.entry])}};
		for (std::size_t i = 0; i < planned.size(); ++i)
			if (planned[i].line == index)
				order.push_back({planned[i].distance, planned[i].name, static_cast<int>(i), planned[i].match});
		if (ends[index].signal >= 0)
			order.push_back({ends[index].distance, standing[ends[index].signal].name, -1, ends[index].signal});
		std::sort(order.begin(), order.end(), [](stop const &A, stop const &B) { return A.distance < B.distance; });
		if (line.inbound)
			std::reverse(order.begin(), order.end());
		// the joints the trains pass the signals to: on beyond a block signal, in towards the station from an entry one
		auto const joint_of = [&](stop const &Stop) {
			if (Stop.signal >= 0)
				return Stop.distance + (line.inbound ? -station::signal_drop : station::signal_drop);
			return Stop.distance + (Stop.distance > 0.0 ? station::signal_drop : -station::signal_drop);
		};
		for (std::size_t i = 0; i < order.size(); ++i)
		{
			auto const &here{order[i]};
			auto const block_before{i > 0 && order[i - 1].signal >= 0};
			auto const linked{block_before ? order[i - 1].name : std::string{"none"}};
			if (here.signal >= 0)
				planned[here.signal].linked = linked;
			else if (block_before && here.standing >= 0 && false == standing[here.standing].linked.empty() && ToLower(standing[here.standing].linked) != ToLower(linked))
				relinked.emplace_back(here.standing, linked);
			if (i == 0)
				continue;
			auto const &before{order[i - 1]};
			// the lower km first, as on the lines of PKP PLK: 2411-2571@LK2.1
			auto const ascending{rising(index)};
			auto const &low{ascending ? before : here};
			auto const &high{ascending ? here : before};
			section found;
			found.name = ToLower(section_tag(low.name, line.number) + "-" + section_tag(high.name, line.number) + "@lk" + numbers[index] + "." + std::to_string(line.number));
			found.line = index;
			found.from = std::min(joint_of(before), joint_of(here));
			found.to = std::max(joint_of(before), joint_of(here));
			found.guard = before.signal >= 0 ? before.name : std::string{};
			found.start = station::point_on(line, before.distance).point;
			sections.push_back(std::move(found));
		}
	}
	// the tracks cut where the sections meet, all of it checked before any is
	struct cut
	{
		TTrack *track{nullptr};
		glm::dvec3 point{0.0};
	};
	std::vector<cut> cuts;
	auto const add_cut = [&](station::line const &Line, double const Distance) -> bool {
		auto const at{station::point_on(Line, Distance)};
		if (at.track == nullptr || std::any_of(cuts.begin(), cuts.end(), [&](cut const &Cut) { return glm::distance(Cut.point, at.point) < 0.5; }))
			return true;
		auto const &ends_of{at.track->m_paths.front().points};
		if (std::min(glm::distance(at.point, ends_of[segment_data::point::start]), glm::distance(at.point, ends_of[segment_data::point::end])) <= 0.3)
			return true;
		std::string reason;
		if (false == editor_track::can_edit_geometry(*at.track, reason))
		{
			m_status = format(STR_C("%s can't be cut: %s"), name_of(*at.track).c_str(), reason.c_str());
			return false;
		}
		cuts.push_back({at.track, at.point});
		return true;
	};
	for (auto const &found : sections)
		if (false == add_cut(lines[found.line], found.from) || false == add_cut(lines[found.line], found.to))
			return;
	for (auto const &line : lines)
		for (auto *part : line.tracks)
			if (std::string reason; false == scene::Layers.editable(part, &reason) && std::any_of(sections.begin(), sections.end(), [&](section const &Found) { return &lines[Found.line] == &line; }))
			{
				m_status = format(STR_C("%s can't be changed: %s"), name_of(*part).c_str(), reason.c_str());
				return;
			}
	std::vector<std::pair<TTrack *, editor_track::state>> before;
	std::vector<TTrack *> created;
	std::unordered_set<TTrack const *> captured;
	std::unordered_map<TTrack const *, std::vector<TTrack *>> pieces; // of each track cut, itself and the pieces cut off it
	auto const piece_at = [&](TTrack *Track, glm::dvec3 const &Point) {
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
	};
	for (auto const &entry : cuts)
	{
		auto *track{piece_at(entry.track, entry.point)};
		auto const parameter{editor_track::nearest_parameter(*track, entry.point)};
		auto const &ends_of{track->m_paths.front().points};
		if (std::min(glm::distance(entry.point, ends_of[segment_data::point::start]), glm::distance(entry.point, ends_of[segment_data::point::end])) <= 0.3)
			continue;
		if (captured.insert(track).second)
			before.emplace_back(track, editor_track::capture(*track));
		if (auto *piece{editor_track::split_path(*track, parameter)})
		{
			created.push_back(piece);
			captured.insert(piece);
			auto &list{pieces[entry.track]};
			if (list.empty())
				list.push_back(entry.track);
			list.push_back(piece);
		}
	}
	// the sections named, each track by its middle
	for (std::size_t index = 0; index < lines.size(); ++index)
	{
		std::vector<TTrack *> parts;
		for (auto *track : lines[index].tracks)
		{
			auto const found{pieces.find(track)};
			if (found == pieces.end())
				parts.push_back(track);
			else
				parts.insert(parts.end(), found->second.begin(), found->second.end());
		}
		for (auto *part : parts)
		{
			double gap{0.0};
			auto const distance{station::distance_on(lines[index], editor_track::point_at(*part, 0.5), gap)};
			if (gap > 1.0)
				continue;
			auto const found{std::find_if(sections.begin(), sections.end(), [&](section const &Section) { return Section.line == index && distance > Section.from && distance < Section.to; })};
			if (found == sections.end() || (part->Isolated.size() == 1 && ToLower(part->Isolated.front()->asName) == found->name))
				continue;
			if (captured.insert(part).second)
				before.emplace_back(part, editor_track::capture(*part));
			editor_track::isolated(*part, {found->name});
		}
	}
	// the signals placed on the track the trains run on up to them
	signal_changes changes;
	std::size_t blocks{0};
	std::size_t discs{0};
	for (auto const &signal : planned)
	{
		auto const block{signal.kind == station::signal::role::block};
		if (signal.match >= 0)
		{
			auto const &found{standing[signal.match]};
			auto const relink{block && false == found.linked.empty() && ToLower(found.linked) != ToLower(signal.linked)};
			if (ToLower(found.name) != ToLower(signal.name) || relink)
				changes.renamed.push_back({found.instance, signal.name, signal.plate, relink ? signal.linked : std::string{}});
			continue;
		}
		auto const &line{lines[signal.line]};
		auto const inbound{line.inbound};
		auto const at{station::point_on(line, signal.distance)};
		auto const approach{station::point_on(line, signal.distance + (inbound ? 1.0 : -1.0))};
		signal_order order;
		order.track = piece_at(approach.track, approach.point);
		order.point = at.point;
		order.facing = inbound == approach.outward ? -1 : 1;
		order.side = order.facing;
		order.kind = block ? "block" : "warning";
		order.name = signal.name;
		order.plate = signal.plate;
		order.linked = signal.linked;
		changes.placed.push_back(std::move(order));
		++(block ? blocks : discs);
	}
	for (auto const &[index, linked] : relinked)
		changes.renamed.push_back({standing[index].instance, standing[index].name, std::string{}, linked});
	// the block signals switched by the sections they guard
	include_info control;
	std::string missing;
	auto const controlled{editor_includes::load(kControl, control, missing)};
	if (controlled)
	{
		auto const standing_controls{m_templates.standing_templates([](std::string const &File) { return File == kControl; })};
		for (auto const &found : sections)
		{
			if (found.guard.empty())
				continue;
			auto const already{std::find_if(standing_controls.begin(), standing_controls.end(), [&](standing_template const &Control) { return false == Control.values.empty() && ToLower(Control.values[0]) == found.name; })};
			if (already != standing_controls.end())
			{
				if (already->values.size() > 5 && ToLower(already->values[5]) != ToLower(found.guard))
				{
					auto values{already->values};
					values[5] = found.guard;
					changes.values.emplace_back(already->instance, std::move(values));
				}
				continue;
			}
			template_item item;
			item.file = kControl;
			item.location = found.start;
			item.described = true;
			item.values["name"] = found.name;
			item.values["#6"] = found.guard;
			changes.others.push_back(std::move(item));
		}
	}
	if (changes.placed.empty() && changes.renamed.empty() && changes.others.empty() && changes.values.empty() && before.empty() && created.empty())
	{
		m_status = STR_C("The lines are set already");
		return;
	}
	std::string error;
	auto const renamed{changes.renamed.size()};
	m_signals.signal_batch(changes, std::move(before), std::move(created), error);
	if (false == error.empty())
	{
		m_status = format(STR_C("The signals couldn't be placed: %s"), error.c_str());
		return;
	}
	m_status = format(STR_P("%zu block signal placed", "%zu block signals placed", blocks), blocks) + ", " + format(STR_P("%zu warning disc", "%zu warning discs", discs), discs) + ", " +
	           format(STR_P("%zu renamed", "%zu renamed", renamed), renamed) + ", " + format(STR_P("%zu block section", "%zu block sections", sections.size()), sections.size());
	if (false == controlled && std::any_of(sections.begin(), sections.end(), [](section const &Found) { return false == Found.guard.empty(); }))
		m_status += "\n" + format(STR_C("%s isn't in the scenery folder: the block signals won't change their lights"), kControl);
}

void srk_tool::survey_signals()
{
	m_signal_marks.clear();
	std::vector<standing_signal> standing;
	m_signals_known = m_layout.has_value() && m_signals.standing_signals(standing, false);
	if (false == m_signals_known)
	{
		m_heads.clear();
		m_heads_known = false;
		if (false == m_measured)
			measure({});
		return;
	}
	check_heads(standing);
	auto const stem{stem_of_open()};
	std::vector<station::signal> stops;
	for (auto const &signal : standing)
	{
		if (gap_to(*m_layout, signal.position) > 8.0)
			continue;
		auto const travel{travel_of(signal.yaw)};
		auto const at{station::project(*m_layout, signal.position)};
		auto const ahead{station::project(*m_layout, signal.position + glm::dvec3{travel.x, 0.0, travel.y} * 5.0)};
		auto const unfit{std::any_of(m_heads.begin(), m_heads.end(), [&](head_check const &Head) { return Head.instance == signal.instance && (false == Head.fits || false == Head.problems.empty()); })};
		m_signal_marks.push_back({at, ahead.x > at.x, signal_shown(signal.name, stem), signal.position + glm::dvec3{0.0, signal.kind == "shunt" ? 3.0 : 7.0, 0.0}, unfit});
		if (signal.kind != "main" && signal.kind != "shunt")
			continue;
		station::signal stop;
		stop.name = signal_shown(signal.name, stem);
		stop.point = signal.position;
		stop.travel = glm::dvec3{travel.x, 0.0, travel.y};
		stops.push_back(std::move(stop));
	}
	measure(stops);
}

void srk_tool::measure(std::vector<station::signal> const &Signals)
{
	m_measured = true;
	m_lengths.clear();
	std::vector<station::numbered> tracks;
	std::unordered_set<TTrack const *> inside;
	m_numbered = m_layout.has_value() && station_tracks(tracks, inside);
	if (false == m_numbered)
		return;
	m_lengths = station::track_lengths(*m_layout, tracks, Signals);
	std::sort(m_lengths.begin(), m_lengths.end(), [](station::lengths const &A, station::lengths const &B) {
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
	return set <= 0 ? 160.0 : set <= 400 ? 60.0 : set <= 700 ? 100.0 : set <= 1000 ? 140.0 : 160.0;
}

double srk_tool::braking_of_open() const
{
	auto const set{m_record >= 0 ? m_records[m_record].braking : 0};
	return set > 0 ? static_cast<double>(set) : braking_distance(speed_of_open());
}

void srk_tool::check_heads(std::vector<standing_signal> const &Standing)
{
	using role = station::signal::role;
	m_heads.clear();
	std::vector<signal_head> heads;
	m_heads_known = m_layout.has_value() && m_signals.signal_heads(heads, false);
	if (false == m_heads_known)
		return;
	auto const &layout{*m_layout};
	auto const stem{stem_of_open()};
	auto const line{speed_of_open()};
	auto const braking{braking_of_open()};
	auto const middle{axis_point(layout, (layout.from + layout.to) * 0.5)};
	std::vector<station::post> posts;
	std::vector<std::size_t> owners;
	for (std::size_t k = 0; k < Standing.size(); ++k)
		if (geometry::plan_distance(Standing[k].position, middle) <= kLineLimit)
			if (auto const post{post_of(Standing[k])}; post.track != nullptr)
			{
				posts.push_back(post);
				owners.push_back(k);
			}
	// the main signals of the station, the block signals and the warning discs of the lines in and out of it
	std::vector<std::size_t> from;
	std::vector<char> own(posts.size(), 0);
	for (std::size_t index = 0; index < posts.size(); ++index)
	{
		if (posts[index].kind == role::entry && gap_to(layout, posts[index].point) <= 8.0)
			own[index] = 1;
		if (own[index] || posts[index].kind == role::block || posts[index].kind == role::warning)
			from.push_back(index);
	}
	auto const found{station::routes(posts, from, route_reach(braking))};
	std::vector<int> routes_of(posts.size(), -1);
	for (std::size_t k = 0; k < from.size(); ++k)
		routes_of[from[k]] = static_cast<int>(k);
	std::vector<char> checked(own);
	std::vector<std::size_t> order;
	for (std::size_t index = 0; index < posts.size(); ++index)
		if (own[index])
			order.push_back(index);
	for (std::size_t at = 0; at < order.size(); ++at)
		for (auto const &route : found[routes_of[order[at]]])
			for (auto const index : {route.warning, route.next})
				if (index >= 0 && 0 == checked[index] && posts[index].kind != role::entry && routes_of[index] >= 0)
				{
					checked[index] = 1;
					order.push_back(static_cast<std::size_t>(index));
				}
	for (auto grown{true}; grown;)
	{
		grown = false;
		for (std::size_t index = 0; index < posts.size(); ++index)
		{
			if (checked[index] || routes_of[index] < 0 || posts[index].kind == role::entry)
				continue;
			auto const &routes{found[routes_of[index]]};
			if (std::none_of(routes.begin(), routes.end(), [&](station::route const &Route) { return Route.next >= 0 && checked[Route.next]; }))
				continue;
			checked[index] = 1;
			order.push_back(index);
			grown = true;
		}
	}
	std::unordered_map<std::string, std::size_t> by_file;
	for (std::size_t index = 0; index < heads.size(); ++index)
		by_file.emplace(heads[index].file, index);
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
		for (auto const &route : check.routes)
			check.nexts.push_back(route.next >= 0 ? signal_shown(Standing[owners[route.next]].name, stem) : std::string{});
		auto const known{by_file.find(signal.file)};
		auto const *template_of{known != by_file.end() ? &heads[known->second] : nullptr};
		if (template_of != nullptr)
		{
			check.lamps = template_of->lamps;
			check.own = template_of->speeds;
		}
		if (post.kind == role::warning)
		{
			for (std::size_t k = 0; k < check.routes.size(); ++k)
			{
				auto const &route{check.routes[k]};
				if (route.next < 0)
					check.problems.push_back(format(STR_C("No main signal within %.0f m ahead of it to tell of"), route.length));
				else if (route.length < braking - 1.0)
					check.problems.push_back(format(STR_C("It stands %.0f m before %s, less than the braking distance of %.0f m: the trains don't stop by the signal in time"), route.length, check.nexts[k].c_str(), braking));
			}
			m_heads.push_back(std::move(check));
			continue;
		}
		auto const main{post.kind == role::entry};
		auto const need{need_of(check.routes, main, line, braking)};
		check.speeds = need.speeds;
		check.informs = need.informs;
		check.white = need.white;
		check.shortened = need.shortened;
		check.indicator = need.indicator;
		if (false == main)
			for (std::size_t k = 0; k < check.routes.size(); ++k)
			{
				auto const &route{check.routes[k]};
				if (route.next >= 0 && route.warning < 0 && route.length < braking - 1.0)
					check.problems.push_back(format(STR_C("The block section to %s is %.0f m long, shorter than the braking distance of %.0f m, Ie-4 § 8 ust. 1"), check.nexts[k].c_str(), route.length, braking));
			}
		if (template_of != nullptr)
		{
			check.fits = shows(*template_of, need);
			auto const best{best_head(heads, signal.kind, template_of->mount, template_of->name, need)};
			if (best >= 0)
			{
				check.fit = heads[best].file;
				check.more = check.fits && surplus(heads[best], need) < surplus(*template_of, need);
			}
		}
		m_heads.push_back(std::move(check));
	}
	std::stable_sort(m_heads.begin(), m_heads.end(), [&](head_check const &A, head_check const &B) {
		if (A.station != B.station)
			return A.station;
		if (A.station)
			return station::project(layout, A.position).x < station::project(layout, B.position).x;
		return geometry::plan_distance(A.position, middle) < geometry::plan_distance(B.position, middle);
	});
}

void srk_tool::fit_heads()
{
	m_status.clear();
	signal_changes changes;
	for (auto const &head : m_heads)
		if (false == head.fit.empty() && head.fit != head.file && (false == head.fits || head.more))
			changes.files.emplace_back(head.instance, head.fit);
	if (changes.files.empty())
	{
		m_status = std::any_of(m_heads.begin(), m_heads.end(), [](head_check const &Head) { return false == Head.fits; }) ? STR_C("No template has the lights the signals marked need") : STR_C("The lights of the signals fit already");
		return;
	}
	std::string error;
	m_signals.signal_batch(changes, {}, {}, error);
	if (false == error.empty())
	{
		m_status = format(STR_C("The signals couldn't be changed: %s"), error.c_str());
		return;
	}
	std::vector<standing_signal> standing;
	m_signals.standing_signals(standing, true);
	std::size_t changed{0};
	for (auto const &head : m_heads)
		changed += static_cast<std::size_t>(std::count_if(standing.begin(), standing.end(), [&](standing_signal const &Signal) { return Signal.instance == head.instance && Signal.file != head.file; }));
	if (changed == 0)
		m_status = STR_C("The lights of the signals fit already");
	else
		m_status = format(STR_P("%zu signal got the lights it needs, Ctrl+Z takes it back", "%zu signals got the lights they need, Ctrl+Z takes it back", changed), changed);
}

void srk_tool::survey()
{
	m_circuit_of.assign(m_layout.has_value() ? m_layout->paths.size() : 0, -1);
	m_circuits.clear();
	m_ticks.clear();
	m_lengths.clear();
	m_measured = false;
	m_numbered = false;
	if (false == m_layout.has_value())
		return;
	auto const &layout{*m_layout};
	auto const stem{stem_of_open()};
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
		auto found{std::find_if(m_circuits.begin(), m_circuits.end(), [&](circuit_mark const &Mark) { return Mark.text == name; })};
		if (found == m_circuits.end())
		{
			found = m_circuits.insert(m_circuits.end(), {glm::dvec2{0.0}, glm::dvec3{0.0}, name, circuit_colour(name)});
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
		// on its track, nearest the middle of its points
		auto nearest{std::numeric_limits<double>::max()};
		glm::dvec2 on{mark.at};
		for (std::size_t path = 0; path < layout.paths.size(); ++path)
			if (m_circuit_of[path] == static_cast<int>(index))
				for (std::size_t k = 0; k < layout.paths[path].schematic.size(); ++k)
					if (auto const distance{glm::distance(layout.paths[path].schematic[k], mark.at)}; distance < nearest)
					{
						nearest = distance;
						on = layout.paths[path].schematic[k];
						mark.world = layout.paths[path].points[k];
					}
		mark.at = on;
		auto const entry{entry_circuit(mark.text, stem)};
		// bpd_it3 shown as It3
		if (false == stem.empty() && mark.text.compare(0, stem.size(), stem) == 0)
			mark.text.erase(0, stem.size());
		if (false == mark.text.empty())
			mark.text.front() = static_cast<char>(std::toupper(static_cast<unsigned char>(mark.text.front())));
		// bpd_ita, before the entry signal A, as ItA
		if (entry && mark.text.size() > 2)
			std::transform(mark.text.begin() + 2, mark.text.end(), mark.text.begin() + 2, [](char const Letter) { return static_cast<char>(std::toupper(static_cast<unsigned char>(Letter))); });
	}
	// the joints: the ends where paths of different circuits meet
	struct end
	{
		glm::dvec3 point;
		glm::dvec3 way;
		int circuit;
	};
	std::vector<end> ends;
	for (std::size_t index = 0; index < layout.paths.size(); ++index)
	{
		auto const &points{layout.paths[index].points};
		if (points.size() < 2)
			continue;
		auto const circuit{m_circuit_of[index]};
		ends.push_back({points.front(), points[1] - points.front(), circuit});
		ends.push_back({points.back(), points.back() - points[points.size() - 2], circuit});
	}
	m_joints.clear();
	for (std::size_t i = 0; i < ends.size(); ++i)
		for (std::size_t j = i + 1; j < ends.size(); ++j)
			if (ends[i].circuit != ends[j].circuit && (ends[i].circuit >= 0 || ends[j].circuit >= 0) && glm::distance(ends[i].point, ends[j].point) < 0.05 &&
			    std::none_of(m_joints.begin(), m_joints.end(), [&](joint_mark const &Joint) { return glm::distance(Joint.at, ends[i].point) < 0.05; }))
			{
				m_ticks.push_back(station::project(layout, ends[i].point));
				glm::dvec2 const way{glm::normalize(glm::dvec2{ends[i].way.x, ends[i].way.z} + glm::dvec2{1e-12, 0.0})};
				m_joints.push_back({ends[i].point, glm::dvec3{-way.y, 0.0, way.x}});
			}
	survey_signals();
}

void srk_tool::fit_view(float const Width, float const Height)
{
	auto const &layout{*m_layout};
	auto &view{m_view_state};
	// the switches and the signals out on the lines
	auto from{layout.from};
	auto to{layout.to};
	for (auto const &mark : m_signal_marks)
	{
		from = std::min(from, mark.at.x);
		to = std::max(to, mark.at.x);
	}
	auto const span{std::max(50.0, to - from)};
	view.from = from - span * 0.04 - 30.0;
	view.to = to + span * 0.04 + 30.0;
	view.centre = (layout.low + layout.high) * 0.5;
	auto const across{(view.to - view.from) / std::max(1.0f, Width)};
	auto const vertical{(layout.high - layout.low + 20.0) / std::max(1.0f, Height * 0.75f)};
	view.exaggeration = static_cast<float>(std::clamp(across / vertical, 1.0, 500.0));
}

void srk_tool::render_canvas(float const Height)
{
	auto const &layout{*m_layout};
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
	{
		if (io.KeyCtrl)
			view.exaggeration = std::clamp(view.exaggeration * std::pow(1.2f, io.MouseWheel), 1.0f, 500.0f);
		else
		{
			auto const at{view.from + (io.MousePos.x - corner.x) / size.x * (view.to - view.from)};
			auto const factor{std::pow(0.8, static_cast<double>(io.MouseWheel))};
			view.from = at - (at - view.from) * factor;
			view.to = at + (view.to - at) * factor;
		}
	}
	auto const across{(view.to - view.from) / size.x};
	auto const vertical{across / view.exaggeration};
	if (ImGui::IsItemActive() && (ImGui::IsMouseDragging(ImGuiMouseButton_Left, kDragPixels) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle, kDragPixels)))
	{
		m_dragging = true;
		view.from -= io.MouseDelta.x * across;
		view.to -= io.MouseDelta.x * across;
		view.centre += io.MouseDelta.y * vertical;
	}
	auto const x_of = [&](double const Chainage) { return corner.x + static_cast<float>((Chainage - view.from) / across); };
	auto const y_of = [&](double const Offset) { return corner.y + size.y * 0.5f - static_cast<float>((Offset - view.centre) / vertical); };
	auto const screen_of = [&](glm::dvec2 const &Point) { return ImVec2(x_of(Point.x), y_of(Point.y)); };

	// the path under the mouse
	if (hovered && false == m_dragging)
	{
		m_hover = -1;
		auto nearest{7.0f * 7.0f};
		auto const &mouse{io.MousePos};
		for (int index = 0; index < static_cast<int>(layout.paths.size()); ++index)
		{
			auto const &points{layout.paths[index].schematic};
			for (std::size_t i = 1; i < points.size(); ++i)
			{
				auto const a{screen_of(points[i - 1])};
				auto const b{screen_of(points[i])};
				auto const dx{b.x - a.x};
				auto const dy{b.y - a.y};
				auto const length2{dx * dx + dy * dy};
				auto const t{length2 > 0.0f ? std::clamp(((mouse.x - a.x) * dx + (mouse.y - a.y) * dy) / length2, 0.0f, 1.0f) : 0.0f};
				auto const ex{a.x + t * dx - mouse.x};
				auto const ey{a.y + t * dy - mouse.y};
				if (ex * ex + ey * ey < nearest)
				{
					nearest = ex * ex + ey * ey;
					m_hover = index;
				}
			}
		}
	}
	else if (false == hovered && false == ImGui::IsItemActive())
		m_hover = -1;
	m_length_canvas = m_hover >= 0 && false == m_dragging ? length_of(layout.paths[m_hover].track) : -1;
	if (ImGui::IsItemDeactivated())
	{
		if (false == m_dragging && m_hover >= 0)
		{
			auto const &path{layout.paths[m_hover]};
			m_selection.set_current_node(path.track);
			m_picked = path.track;
			m_view.focus_track(*path.track, path.index, path.points[path.points.size() / 2]);
		}
		m_dragging = false;
	}

	auto *draw{ImGui::GetWindowDrawList()};
	ImVec2 const corner_end{corner.x + size.x, corner.y + size.y};
	draw->AddRectFilled(corner, corner_end, IM_COL32(22, 24, 28, 255));
	draw->PushClipRect(corner, corner_end, true);
	// the km, or the chainage, along the top
	{
		auto const metres{across * 90.0};
		double step{10.0};
		for (auto const candidate : {10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0})
			if ((step = candidate) >= metres)
				break;
		auto const origin{layout.kilometrage ? layout.km * 1000.0 : 0.0};
		for (auto mark{std::ceil((view.from + origin) / step) * step}; mark <= view.to + origin; mark += step)
		{
			auto const x{x_of(mark - origin)};
			draw->AddLine(ImVec2(x, corner.y), ImVec2(x, corner_end.y), IM_COL32(255, 255, 255, 16));
			auto const label{layout.kilometrage ? format("%.3f", mark / 1000.0) : format("%.0f m", mark)};
			draw->AddText(ImVec2(x + 3.0f, corner.y + 2.0f), IM_COL32(150, 155, 165, 255), label.c_str());
		}
	}
	auto const *selected{m_selection.selected_track()};
	auto const draw_path = [&](station::path const &Path, ImU32 const Colour, float const Width) {
		for (std::size_t i = 1; i < Path.schematic.size(); ++i)
			draw->AddLine(screen_of(Path.schematic[i - 1]), screen_of(Path.schematic[i]), Colour, Width);
	};
	for (auto const &path : layout.paths)
		if (path.approach)
			draw_path(path, kApproach, 2.0f);
	for (std::size_t index = 0; index < layout.paths.size(); ++index)
		if (auto const &path{layout.paths[index]}; false == path.approach || m_circuit_of[index] >= 0)
			draw_path(path, m_circuit_of[index] >= 0 ? m_circuits[m_circuit_of[index]].colour : is_switch(*path.track) ? kSwitch : kPlain, 2.5f);
	for (auto const &tick : m_ticks)
	{
		auto const at{screen_of(tick)};
		draw->AddLine(ImVec2(at.x, at.y - 5.0f), ImVec2(at.x, at.y + 5.0f), IM_COL32(255, 255, 255, 220), 2.0f);
	}
	for (auto const &path : layout.paths)
		if (path.track == selected)
			draw_path(path, overlay_color::selected, 4.0f);
	if (m_hover >= 0)
		draw_path(layout.paths[m_hover], kHovered, 4.0f);
	// the useful lengths of the station track shown: of the trains to the right over it, to the left under it, an arrowhead where
	// they stop
	std::vector<std::pair<std::string, ImVec2>> useful;
	if (m_length_shown >= 0 && m_length_shown < static_cast<int>(m_lengths.size()))
	{
		auto const &track{m_lengths[m_length_shown]};
		for (int way = 0; way < 2; ++way)
		{
			auto const &part{track.ways[way]};
			if (part.length <= 0.0)
				continue;
			auto const shift{way == 0 ? -7.0f : 7.0f};
			std::vector<ImVec2> line;
			for (auto const &point : stretch_of(track, part.rear.along, part.head.along))
			{
				auto at{screen_of(station::project(layout, point))};
				at.y += shift;
				line.push_back(at);
			}
			draw->AddPolyline(line.data(), static_cast<int>(line.size()), kUseful, ImDrawFlags_None, 2.0f);
			auto const &rear{line.front()};
			auto const &head{line.back()};
			draw->AddLine(ImVec2(rear.x, rear.y - 4.0f), ImVec2(rear.x, rear.y + 4.0f), kUseful, 2.0f);
			auto const way_x{head.x >= line[line.size() > 1 ? line.size() - 2 : 0].x ? 1.0f : -1.0f};
			draw->AddTriangleFilled(ImVec2(head.x - 7.0f * way_x, head.y - 4.5f), ImVec2(head.x - 7.0f * way_x, head.y + 4.5f), ImVec2(head.x + 1.0f * way_x, head.y), kUseful);
			auto const &middle{line[line.size() / 2]};
			useful.emplace_back(format("%.0f m", part.length), ImVec2(middle.x, way == 0 ? middle.y - ImGui::GetTextLineHeight() - 3.0f : middle.y + 3.0f));
		}
	}
	// the labels, none over another: the switches first, a dot at the start and the name over or under it in turn, the number
	// alone once named after the station; then the station tracks under or over them, the longest first
	std::vector<ImVec4> placed;
	auto const label = [&](std::string const &Text, ImU32 const Colour, std::initializer_list<ImVec2> Places) {
		auto const extent{ImGui::CalcTextSize(Text.c_str())};
		for (auto const &place : Places)
		{
			ImVec4 const box{place.x - extent.x * 0.5f - 2.0f, place.y - 1.0f, place.x + extent.x * 0.5f + 2.0f, place.y + extent.y + 1.0f};
			if (std::any_of(placed.begin(), placed.end(), [&](ImVec4 const &Other) { return box.x < Other.z && Other.x < box.z && box.y < Other.w && Other.y < box.w; }))
				continue;
			placed.push_back(box);
			draw->AddRectFilled(ImVec2(box.x, box.y), ImVec2(box.z, box.w), IM_COL32(22, 24, 28, 200), 2.0f);
			draw->AddText(ImVec2(place.x - extent.x * 0.5f, place.y), Colour, Text.c_str());
			return;
		}
	};
	for (auto const &[text, at] : useful)
		label(text, kUseful, {at});
	auto const stem{stem_of_open()};
	auto const height{ImGui::GetTextLineHeight()};
	for (std::size_t index = 0; index < layout.switches.size(); ++index)
	{
		auto const *track{layout.switches[index]};
		auto const at{screen_of(station::project(layout, track->m_paths.front().points[segment_data::point::start]))};
		draw->AddCircleFilled(at, 3.5f, kSwitch);
		auto text{track->name()};
		if (false == stem.empty() && text.size() > stem.size() && text.compare(0, stem.size(), stem) == 0)
			text.erase(0, stem.size());
		ImVec2 const over{at.x, at.y - height - 6.0f};
		ImVec2 const under{at.x, at.y + 6.0f};
		if (index % 2 == 0)
			label(text, kSwitch, {over, under});
		else
			label(text, kSwitch, {under, over});
	}
	// the signals: a flag pointing the way of the trains they face
	for (auto const &mark : m_signal_marks)
	{
		auto const at{screen_of(mark.at)};
		auto const way{mark.ahead ? 1.0f : -1.0f};
		auto const colour{mark.unfit ? kUnfit : kSignal};
		draw->AddTriangleFilled(ImVec2(at.x - 4.0f * way, at.y - 4.0f), ImVec2(at.x - 4.0f * way, at.y + 4.0f), ImVec2(at.x + 4.0f * way, at.y), colour);
		label(mark.unfit ? mark.text + " !" : mark.text, colour, {ImVec2(at.x, at.y - height - 5.0f), ImVec2(at.x, at.y + 5.0f)});
	}
	std::vector<std::size_t> order;
	for (std::size_t index = 0; index < m_circuits.size(); ++index)
		if (m_circuits[index].labelled)
			order.push_back(index);
	std::sort(order.begin(), order.end(), [&](std::size_t A, std::size_t B) { return m_circuits[A].length > m_circuits[B].length; });
	for (auto const index : order)
	{
		auto const &mark{m_circuits[index]};
		auto const at{screen_of(mark.at)};
		label(mark.text, mark.colour, {ImVec2(at.x, at.y + 4.0f), ImVec2(at.x, at.y - height - 4.0f)});
	}
	draw->PopClipRect();
	if (hovered && m_hover >= 0 && false == m_dragging)
	{
		auto const &path{layout.paths[m_hover]};
		auto const along{view.from + (io.MousePos.x - corner.x) * across};
		ImGui::BeginTooltip();
		ImGui::TextUnformatted(path.track->name().empty() ? STR_C("(no name)") : path.track->name().c_str());
		ImGui::TextDisabled("%s, %.1f m", is_switch(*path.track) ? STR_C("switch") : path.approach ? STR_C("line out of the station") : STR_C("track"), path.length);
		if (layout.kilometrage)
			ImGui::TextDisabled("km %.3f", station::km_of(layout, along));
		if (false == path.track->Isolated.empty())
			ImGui::TextDisabled(STR_C("track circuit %s"), path.track->Isolated.front()->asName.c_str());
		if (m_length_canvas >= 0 && m_length_canvas < static_cast<int>(m_lengths.size()))
		{
			auto const &track{m_lengths[m_length_canvas]};
			ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kUseful), STR_C("Track %s, useful length %.0f m to the right, %.0f m to the left"), track.number.c_str(), track.ways[0].length, track.ways[1].length);
		}
		ImGui::TextDisabled("%s", STR_C("LMB: select it"));
		ImGui::EndTooltip();
	}
}

void srk_tool::render_windows()
{
	m_length_canvas = -1;
	if (false == m_open || false == m_strip || false == m_layout.has_value())
		return;
	auto const &io{ImGui::GetIO()};
	// along the bottom of the 3d view, as the vertical profile
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	auto const width{std::max(480.0f, high.x - low.x)};
	auto const height{std::clamp((high.y - low.y) * 0.4f, 220.0f, std::max(220.0f, high.y - low.y - 80.0f))};
	ImGui::SetNextWindowPos(ImVec2(low.x, high.y - height), ImGuiCond_Appearing);
	ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Appearing);
	auto const name{std::string{STR_C("Station schematic")} + "  " + title() + "###srkstrip"};
	if (ImGui::Begin(name.c_str(), nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
	{
		auto const position{ImGui::GetWindowPos()};
		auto const size{ImGui::GetWindowSize()};
		if (false == ImGui::IsWindowDocked() && (position.y + size.y > io.DisplaySize.y + 1.0f || position.x + 80.0f > io.DisplaySize.x || position.x + size.x < 80.0f))
			ImGui::SetWindowPos(ImVec2(std::clamp(position.x, 0.0f, std::max(0.0f, io.DisplaySize.x - size.x)), std::max(0.0f, std::min(position.y, io.DisplaySize.y - size.y))));
		if (ImGui::Button(STR_C("Fit")))
			m_view_state.exaggeration = 0.0f;
		item_tooltip("Shows the whole station");
		ImGui::SameLine();
		ImGui::SetNextItemWidth(140.0f);
		ImGui::SliderFloat("##srkexaggeration", &m_view_state.exaggeration, 1.0f, 500.0f, STR_C("offsets x%.0f"), ImGuiSliderFlags_Logarithmic);
		item_tooltip("How much the offsets of the tracks are stretched against the lengths, also Ctrl+wheel over the plot");
		ImGui::SameLine();
		ImGui::TextDisabled("%s", STR_C("Wheel: zoom, drag: move, LMB on a track: select it"));
		render_canvas(ImGui::GetContentRegionAvail().y);
	}
	ImGui::End();
}

editor_selftest_result srk_tool::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "srkopen")
	{
		// srkopen track=<name>: the station of the track, as a click on it opens it
		auto const values{selftest_values(Words)};
		auto const found{values.find("track")};
		auto *track{found != values.end() ? simulation::Paths.find(found->second) : nullptr};
		m_open = true;
		load();
		if (track == nullptr)
		{
			WriteLog("SELFTEST srkopen: no such track");
			return editor_selftest_result::handled;
		}
		open(*track);
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srkopen: " + m_status);
			return editor_selftest_result::handled;
		}
		auto const &layout{*m_layout};
		std::unordered_set<TTrack const *> tracks;
		std::size_t approach{0};
		for (auto const &path : layout.paths)
			if (path.approach)
				++approach;
			else
				tracks.insert(path.track);
		WriteLog(format("SELFTEST srkopen %s: %zu switches, %zu tracks, %zu paths out, axis %.0f m, switches over %.0f..%.0f, offsets %.1f..%.1f, km %s, record %d", m_anchor->name().c_str(), layout.switches.size(), tracks.size(), approach,
		                layout.chainage.back(), layout.from, layout.to, layout.low, layout.high, layout.kilometrage ? format("%.3f", layout.km).c_str() : "-", m_record));
		WriteLog(format("SELFTEST srkopen lines %zu, main %zu tracks, second %zu tracks", layout.lines.size(), layout.main.size(), layout.second.size()));
		for (auto const *track : layout.switches)
		{
			auto const at{station::project(layout, track->m_paths.front().points[segment_data::point::start])};
			WriteLog(format("SELFTEST srkopen switch %s at %.1f offset %.1f", track->name().c_str(), at.x, at.y));
		}
		return editor_selftest_result::handled;
	}
	if (Command == "srknumber")
	{
		// srknumber [from=right]: the switches of the open station numbered
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srknumber: no station open");
			return editor_selftest_result::handled;
		}
		auto const values{selftest_values(Words)};
		auto const from{values.find("from")};
		m_reversed = from != values.end() && from->second == "right";
		number_switches();
		std::string list;
		for (auto const &unit : station::numbering(*m_layout, m_reversed))
			for (auto const *part : unit.parts)
			{
				auto const record{switch_store::named(*part)};
				list += ' ' + part->name() + '/' + (record ? record->number : std::string{"?"});
			}
		WriteLog("SELFTEST srknumber: " + m_status);
		WriteLog("SELFTEST srknumber names:" + list);
		return editor_selftest_result::handled;
	}
	if (Command == "srkcircuits")
	{
		// srkcircuits: the open station cut into its track circuits
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srkcircuits: no station open");
			return editor_selftest_result::handled;
		}
		set_circuits();
		WriteLog("SELFTEST srkcircuits: " + m_status);
		refresh();
		std::map<std::string, std::pair<std::size_t, double>> found;
		for (auto const &path : m_layout->paths)
			if (false == path.track->Isolated.empty())
			{
				auto &entry{found[path.track->Isolated.front()->asName]};
				++entry.first;
				entry.second += path.length;
			}
		std::string list;
		for (auto const &[name, entry] : found)
			list += format(" %s:%zu/%.0f", name.c_str(), entry.first, entry.second);
		WriteLog(format("SELFTEST srkcircuits %zu named, %zu joints:", found.size(), m_ticks.size()) + list);
		return editor_selftest_result::handled;
	}
	if (Command == "srktrack")
	{
		// srktrack circuit=<name> number=<number>: the station track of the circuit renumbered
		auto const values{selftest_values(Words)};
		auto const circuit{values.find("circuit")};
		auto const number{values.find("number")};
		TTrack const *found{nullptr};
		if (m_layout.has_value() && circuit != values.end())
			for (auto const &path : m_layout->paths)
				if (false == path.track->Isolated.empty() && path.track->Isolated.front()->asName == circuit->second)
					found = path.track;
		if (found == nullptr || number == values.end())
		{
			WriteLog("SELFTEST srktrack: no station open, or no such circuit");
			return editor_selftest_result::handled;
		}
		renumber_track(*found, number->second);
		WriteLog("SELFTEST srktrack: " + m_status);
		return editor_selftest_result::handled;
	}
	if (Command == "srksignals")
	{
		// srksignals: the signals of the open station set
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srksignals: no station open");
			return editor_selftest_result::handled;
		}
		set_signals();
		WriteLog("SELFTEST srksignals: " + m_status);
		refresh();
		std::string list;
		for (auto const &mark : m_signal_marks)
		{
			// and the joint nearest to it, in the plan
			auto nearest{std::numeric_limits<double>::max()};
			for (auto const &joint : m_joints)
				nearest = std::min(nearest, glm::distance(glm::dvec2{joint.at.x, joint.at.z}, glm::dvec2{mark.head.x, mark.head.z}));
			list += format(" %s@%.0f/%.1f%s~%.1f", mark.text.c_str(), mark.at.x, mark.at.y, mark.ahead ? ">" : "<", nearest);
		}
		WriteLog(format("SELFTEST srksignals %zu:", m_signal_marks.size()) + list);
		return editor_selftest_result::handled;
	}
	if (Command == "srklines")
	{
		// srklines: the signals of the lines out of the open station set
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srklines: no station open");
			return editor_selftest_result::handled;
		}
		set_lines();
		WriteLog("SELFTEST srklines: " + m_status);
		refresh();
		std::vector<station::numbered> tracks;
		std::unordered_set<TTrack const *> inside;
		if (false == station_tracks(tracks, inside))
			return editor_selftest_result::handled;
		auto const plan{station::signals(*m_layout, tracks, [&](TTrack const &Track) { return inside.count(&Track) > 0; })};
		std::vector<standing_signal> standing;
		m_signals.standing_signals(standing, true);
		for (auto const &line : station::lines_out(*m_layout, plan, kLineLimit))
		{
			std::vector<std::pair<double, std::string>> along;
			for (auto const &signal : standing)
			{
				double gap{0.0};
				auto const distance{station::distance_on(line, signal.position, gap)};
				if (gap < 4.5 && distance < line.length())
					along.emplace_back(distance, format("%s(%s,%s)", signal.name.c_str(), signal.kind.c_str(), signal.linked.c_str()));
			}
			std::string last;
			for (std::size_t i = 1; i < line.points.size(); ++i)
			{
				auto const *track{line.tracks[line.owners[i]]};
				auto const name{track->Isolated.empty() ? std::string{"-"} : track->Isolated.front()->asName};
				if (name != last)
					along.emplace_back(line.distances[i], "[" + name + "]");
				last = name;
			}
			std::sort(along.begin(), along.end());
			std::string list;
			for (auto const &[distance, text] : along)
				list += format(" %.0f:%s", distance, text.c_str());
			WriteLog(format("SELFTEST srklines %s track %d %s %.0f m%s:", plan[line.entry].name.c_str(), line.number, line.inbound ? "in" : "out", line.length(), line.station ? " to a station" : "") + list);
		}
		return editor_selftest_result::handled;
	}
	if (Command == "srklengths")
	{
		// srklengths [show=<number>]: the lengths of the station tracks of the open station, the one shown drawn
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srklengths: no station open");
			return editor_selftest_result::handled;
		}
		std::vector<standing_signal> standing;
		m_signals.standing_signals(standing, true);
		refresh();
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srklengths: the station is gone");
			return editor_selftest_result::handled;
		}
		auto const values{selftest_values(Words)};
		auto const show{values.find("show")};
		auto const code = [](station::limit const &Limit) {
			char const *const kinds[] = {"S", "J", "F", "St", "B", "X"};
			return std::string{kinds[static_cast<int>(Limit.what)]} + ":" + Limit.name + format("@%.1f", Limit.along);
		};
		for (std::size_t index = 0; index < m_lengths.size(); ++index)
		{
			auto const &track{m_lengths[index]};
			WriteLog(format("SELFTEST srklengths %s: right %.1f (%s .. %s), left %.1f (%s .. %s), construction %.1f", track.number.c_str(), track.ways[0].length, code(track.ways[0].rear).c_str(), code(track.ways[0].head).c_str(), track.ways[1].length,
			                code(track.ways[1].rear).c_str(), code(track.ways[1].head).c_str(), track.construction));
		}
		WriteLog(format("SELFTEST srklengths: %zu tracks", m_lengths.size()));
		if (auto const copy{values.find("copy")}; copy != values.end() && copy->second == "1")
			WriteLog("SELFTEST srklengths table:\n" + lengths_table());
		// the track shown selected
		for (auto const &path : m_layout->paths)
			if (show != values.end() && false == path.track->Isolated.empty() && track_of(path.track->Isolated.front()->asName, stem_of_open()) == show->second)
			{
				m_selection.set_current_node(path.track);
				m_picked = path.track;
				break;
			}
		return editor_selftest_result::handled;
	}
	if (Command == "srkheads")
	{
		// srkheads [braking=<m>] [fit=1]: the lights of the signals of the open station checked, fitted
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srkheads: no station open");
			return editor_selftest_result::handled;
		}
		auto const values{selftest_values(Words)};
		if (auto const braking{values.find("braking")}; braking != values.end())
		{
			record_of_open().braking = std::atoi(braking->second.c_str());
			m_changed = true;
		}
		std::vector<standing_signal> standing;
		m_signals.standing_signals(standing, true);
		std::vector<signal_head> heads;
		m_signals.signal_heads(heads, true);
		refresh();
		if (auto const fit{values.find("fit")}; fit != values.end() && fit->second == "1" && m_layout.has_value())
		{
			fit_heads();
			WriteLog("SELFTEST srkheads fit: " + m_status);
			refresh();
		}
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srkheads: the station is gone");
			return editor_selftest_result::handled;
		}
		WriteLog(format("SELFTEST srkheads: speed %.0f (set %.0f), braking %.0f, %zu checked, templates %zu", speed_of_open(), set_speed_of_open(), braking_of_open(), m_heads.size(), heads.size()));
		auto const list = [](std::vector<int> const &Speeds) {
			std::string text;
			for (auto const speed : Speeds)
				text += (text.empty() ? "" : "/") + std::to_string(speed);
			return text.empty() ? std::string{"-"} : text;
		};
		for (auto const &head : m_heads)
		{
			std::string routes;
			for (std::size_t k = 0; k < head.routes.size(); ++k)
			{
				auto const &route{head.routes[k]};
				routes += format(" %s:%.0f:%.0f%s%s", route.next >= 0 ? head.nexts[k].c_str() : route.ended ? "end" : "none", route.length, route.speed, route.warning >= 0 ? ":to" : "", route.turnouts.empty() ? "" : format(":%zut", route.turnouts.size()).c_str());
			}
			WriteLog(format("SELFTEST srkheads %s %s%s %s lamps=%s own=%s need=%s%s%s short=%d%s fits=%d more=%d fit=%s routes%s problems=%zu", head.name.c_str(), head.kind.c_str(), head.station ? "" : "(line)", head.file.c_str(), head.lamps.c_str(), list(head.own).c_str(), list(head.speeds).c_str(), head.informs ? "+inform" : "",
			                head.white ? "+white" : "", head.shortened, head.indicator ? " w21" : "", head.fits ? 1 : 0, head.more ? 1 : 0, head.fit.empty() ? "-" : head.fit.c_str(), routes.c_str(), head.problems.size()));
			for (auto const &problem : head.problems)
				WriteLog("SELFTEST srkheads   " + problem);
		}
		return editor_selftest_result::handled;
	}
	if (Command == "srkstrip")
	{
		// srkstrip shown=0: the schematic hidden, the 3d view whole for a screenshot
		auto const values{selftest_values(Words)};
		auto const shown{values.find("shown")};
		m_strip = shown == values.end() || shown->second != "0";
		return editor_selftest_result::handled;
	}
	if (Command == "srkstations")
	{
		load();
		find_all();
		WriteLog(format("SELFTEST srkstations: %zu", m_found.size()));
		for (auto const &[name, count] : m_found)
			WriteLog(format("SELFTEST srkstations %s: %zu switches", name.c_str(), count));
		return editor_selftest_result::handled;
	}
	if (Command == "srkset")
	{
		// srkset code=<code> line=<line> name=<words joined by _>
		if (false == m_layout.has_value())
		{
			WriteLog("SELFTEST srkset: no station open");
			return editor_selftest_result::handled;
		}
		auto const values{selftest_values(Words)};
		auto &entry{record_of_open()};
		if (auto const code{values.find("code")}; code != values.end())
			entry.code = code->second;
		if (auto const line{values.find("line")}; line != values.end())
			entry.line = line->second;
		if (auto const name{values.find("name")}; name != values.end())
		{
			entry.name = name->second;
			std::replace(entry.name.begin(), entry.name.end(), '_', ' ');
		}
		if (auto const km{values.find("km")}; km != values.end())
			entry.km = km->second;
		m_changed = true;
		WriteLog("SELFTEST srkset: " + title());
		return editor_selftest_result::handled;
	}
	return editor_selftest_result::unhandled;
}

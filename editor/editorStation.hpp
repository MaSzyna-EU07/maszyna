/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

class TTrack;

// a station as the signalling sees it: its switches, the tracks between them and the lines leading out, and its schematic,
// the tracks laid out along the main track through it
namespace station
{

struct options
{
	double link{1500.0}; // m, switches joined by plain track no longer than this belong to the same station
	double approach{400.0}; // m, of each line leading out, taken beyond the outermost switch
};

struct path
{
	TTrack *track{nullptr};
	int index{0}; // of the path of the track
	std::vector<glm::dvec3> points; // along the path, world space
	std::vector<glm::dvec2> schematic; // the chainage along the axis and the offset to the left of it, as the map shows it
	double length{0.0}; // in the plan
	bool approach{false}; // on a line leading out of the station
};

struct layout
{
	std::vector<TTrack *> switches; // switches, crossings and turntables, by the chainage of their start
	std::vector<path> paths;
	std::vector<glm::dvec3> axis; // the main route through the station
	std::vector<double> chainage; // of the points of the axis
	double from{0.0}; // the chainage over which the switches lie
	double to{0.0};
	double low{0.0}; // the offsets of the outermost tracks of the station
	double high{0.0};
	bool kilometrage{false}; // the chainage runs with the km of the switches
	double km{0.0}; // at chainage 0, when it does
	std::vector<std::vector<TTrack *>> lines; // the lines leading out, their tracks from the station on, as far as the approach
	std::vector<TTrack *> main; // the tracks of the axis
	std::vector<TTrack *> second; // of the other main track of a double-track line, none on a single-track one
};

// what shares a number of the station: a switch, or the switches of a slip, the two of a double switch, the paths of a crossing
struct unit
{
	std::vector<TTrack *> parts;
	std::string record; // the name of its record of the switches, empty without one
	double chainage{0.0}; // of the start of its first part along the station
};

// a joint of the track circuits; Track is the plain track to be cut there, none where two tracks meet already
struct joint
{
	TTrack *track{nullptr};
	glm::dvec3 point{0.0};
};

// a track circuit of the station, once its tracks are cut at the joints: of a switch, or of a station track
struct circuit
{
	std::vector<TTrack *> tracks;
	int unit{-1}; // of the switch, -1 for a station track
	double offset{0.0}; // the mean one of a station track, in the schematic
	double from{0.0}; // the chainage it spans
	double to{0.0};
	double length{0.0};
};

// the station of the switch, or of the plain track near one
std::optional<layout> find(TTrack &From, options const &Options = {});
// the chainage along the axis and the offset to the left of it, as the map shows it
glm::dvec2 project(layout const &Layout, glm::dvec3 const &Point);
// km at the chainage, when the station knows its kilometrage
double km_of(layout const &Layout, double const Chainage);
// one switch of each station of the scenery
std::vector<TTrack *> stations(options const &Options = {});
// the switches of the station by their numbers: along the chainage, or against it when Reversed; turntables have none
std::vector<unit> numbering(layout const &Layout, bool const Reversed);
// the names of the switches and the track circuits of the station begin with it: bpd_
std::string prefix(std::string const &Code);
// <code>_<number>, each part of a unit of several with a letter of its own: bpd_15a
std::string switch_name(std::string const &Code, int const Number, std::size_t const Part, std::size_t const Parts);
// the joints of the track circuits: 1.5 m before the start of a switch, beyond the fouling points of its branches where the axes
// are 4 m apart, in the middle of the links between switches too short for both, and so where the lines leave the station
std::vector<joint> joints(layout const &Layout, std::vector<unit> const &Units, options const &Options = {});
// the track circuits the joints part the station into; the tracks of the lines beyond the joints belong to none
std::vector<circuit> circuits(layout const &Layout, std::vector<unit> const &Units, std::vector<glm::dvec3> const &Joints);
// the numbers of the station tracks as PKP PLK gives them, one for each circuit, empty for those of switches: 1 and 2 the main
// tracks, track 1 on the right along the chainage, the odd ones beyond it on its side, the even ones on the other; a track goes
// on through the switches along their straighter paths, its circuits lettered along the chainage: 5a, 5b
std::vector<std::string> track_numbers(layout const &Layout, std::vector<circuit> const &Circuits);
// the switches of the station of the switch, without laying it out
std::vector<TTrack *> switches_of(TTrack &Switch, options const &Options = {});

// a station track by its number, with its track circuits
struct numbered
{
	std::string number;
	std::vector<std::vector<TTrack *>> circuits;
};

// a signal of the station, as PKP PLK places and names them
// m, from a main signal on to the joint its trains turn it to stop at: Ie-4 § 9, at least 15 m, at most 30 for an entry or a block signal
double constexpr signal_drop{15.0};

struct signal
{
	enum class role
	{
		entry,
		exit,
		shunting,
		block,
		warning
	};
	role kind{role::entry};
	std::string name; // without the code of the station: A, F1, Tm3
	TTrack *track{nullptr};
	int path{0};
	glm::dvec3 point{0.0}; // on the axis
	int facing{1}; // the trains it faces run 1: towards the end of the path, -1: towards its start
	int side{1}; // 1: on the right of the path, -1: on the left
	glm::dvec3 travel{0.0}; // the way the trains it faces run, in the plan
	// of an entry signal: the tracks of its line from the joint of the station out to it, each with the way out along it, true
	// towards the end of the path; a circuit named after the signal runs between the two
	std::vector<std::pair<TTrack *, bool>> run;
};

// the signals of the station: the entry ones 300 m out along the lines beyond the circuits of the station, A, B... from the start
// of the chainage, Z, Y... from its end, the leftmost first; the exit ones facing out at the ends of the longest circuit at least
// 200 m long of each station track, for its longest useful length, a letter for the odd tracks and one for the even ones at each
// end, with the number: F1, G2 towards the start, R1, P2 towards the end; the shunting ones at the ends of the shorter tracks, Tm1,
// Tm2... along the chainage. Each on the right of the trains, unless another track runs too close by there. Inside: a track of the
// circuits of the station
std::vector<signal> signals(layout const &Layout, std::vector<numbered> const &Tracks, std::function<bool(TTrack const &)> const &Inside);

// a track of a line out beyond an entry signal of the station, to the next station or the end of the tracks
struct line
{
	std::size_t entry{0}; // the entry signal standing on it, of the signals given
	int number{0}; // of the track of a double-track line: 1 the one on the right along the chainage, 2 the other; 0 on a single-track line
	bool inbound{true}; // its trains run towards the station: on a double-track line the track on their right
	bool station{false}; // ends at a switch, of the next station, rather than at the end of the tracks
	int partner{-1}; // the other track of a double-track line, of the lines
	std::vector<TTrack *> tracks; // from the entry signal out
	std::vector<bool> outward; // of each track: the line runs out towards the end of its path
	std::vector<glm::dvec3> points; // along the tracks from the entry signal on, a metre or so apart
	std::vector<double> distances; // of the points, from the entry signal: below 0 in from it to the joint of the station
	std::vector<std::size_t> owners; // of the points, the track each lies on
	double length() const { return distances.empty() ? 0.0 : distances.back(); }
};
// where the line runs at the distance out: on what track, the point on its axis, the way out along the line
struct on_line
{
	TTrack *track{nullptr};
	bool outward{true}; // the way out runs towards the end of the path of the track
	glm::dvec3 point{0.0};
	glm::dvec3 way{0.0, 0.0, 1.0};
};

// the lines beyond the entry signals among the signals given, no longer than the limit; two lines out of the same end side by side
// are the tracks of a double-track line
std::vector<line> lines_out(layout const &Layout, std::vector<signal> const &Signals, double const Limit);
// the run of an entry signal as a line, from the joint of the station out, the distances from the joint
line run_line(signal const &Entry);
on_line point_on(line const &Line, double const Distance);
// how far out along the line the point lies, Gap: how far from its axis in the plan
double distance_on(line const &Line, glm::dvec3 const &Point, double &Gap);
// the number of a block signal at the km, as PKP PLK gives them (Ie-4 § 10): the km and the hectometre, the nearest one ahead of the
// signal odd on track 1, even on track 2: 241, 242; Rising: its trains run towards the higher km
std::string block_number(double const Km, int const Track, bool const Rising);

// a limit of the part of a station track a train stands on
struct limit
{
	enum class kind
	{
		signal,
		joint, // of the track circuits
		fouling, // the fouling point of a switch
		start, // of a switch
		buffer, // the end of the tracks, where the buffer stop stands
		crossing // the edge of a road crossing the track on its level
	};
	kind what{kind::joint};
	std::string name; // of the signal, of the switch for the others
	double along{0.0}; // along the track, from its end at the lower chainage
	glm::dvec3 point{0.0};
};
// the part of a station track the trains running one way stand on
struct stand
{
	limit head; // the trains stop by it
	limit rear; // behind them
	double length{0.0};
};
// the lengths of a station track, as the list of the tracks of a station gives them: the rozporządzenie of the minister of
// infrastructure on the railway structures (Dz.U. 2026 poz. 1225) § 2
struct lengths
{
	std::string number;
	double construction{0.0}; // between the ends of the switches at its ends, or to the buffer stop, the switches within it left out
	// the useful one, of the trains running along the chainage, then of those running against it: from the signal they stop by back
	// to the joint, the fouling point or the level crossing behind them, between those where no signal stands
	std::array<stand, 2> ways;
	std::vector<glm::dvec3> points; // along the track from its end at the lower chainage, through the switches within it
	std::vector<double> distances; // of the points, from that end
};
// the lengths of the station tracks, the signals standing by them among the signals given
std::vector<lengths> track_lengths(layout const &Layout, std::vector<numbered> const &Tracks, std::vector<signal> const &Signals);

// a signal by the track the trains read it on
struct post
{
	TTrack *track{nullptr};
	int facing{1}; // the trains it faces run 1: towards the end of the path, -1: towards its start
	glm::dvec3 point{0.0};
	signal::role kind{signal::role::entry}; // entry for any main signal
};
// the way of the trains from a signal on to the next main or block signal facing them
struct route
{
	int next{-1}; // of the posts, -1 for none within the limit
	int warning{-1}; // the warning disc passed on the way, of the posts
	double length{0.0}; // m, in the plan
	double speed{0.0}; // km/h, the least the switches on it allow on their curved paths, by the radius and by the speed set; 0 for no limit
	std::vector<TTrack *> turnouts; // the switches it turns off the straight at
	bool ended{false}; // at the end of the tracks, before the limit
};
// km/h, a train runs the path at by its least radius, 0.65 m/s² of the acceleration left unbalanced without cant; 0 for a straight one
double path_speed(TTrack const &Track, int const Path);
// the routes of the trains from each of the posts given on through the switches, none longer than the limit
std::vector<std::vector<route>> routes(std::vector<post> const &Posts, std::vector<std::size_t> const &From, double const Limit);

} // namespace station

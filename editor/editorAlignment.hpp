/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http:
*/

#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <glm/glm.hpp>

#include "world/Segment.h"

namespace alignment
{

enum class transition_shape
{
	cubic_parabola,
	clothoid
};

struct limits
{
	double gauge{1500.0};
	double unbalanced{0.6};
	double cant_max{150.0};
	double cant_rate{35.0};
	double ramp_factor{10.0};
	double jerk{0.3};
	double vertical_factor{0.4};
	double vertical_min{2000.0};
	double tangent_min_time{0.0};
	double arc_min{20.0};
	double arc_min_factor{0.4};
	bool checked{true};

	bool operator==(limits const &) const = default;
};

struct compound_arc
{
	double radius{600.0};
	double transition{0.0};
	double share{1.0};
	// the cant of the arc; below 0 the cant of the vertex, which is the one of the sharpest arc, in proportion to the curvature
	double cant{-1.0};
};

struct vertex
{
	glm::dvec2 position{0.0};
	double offset{100.0};
	double radius{1000.0};
	double transition_in{0.0};
	double transition_out{0.0};
	double cant{0.0};
	bool reverse_turn{false};
	bool compound{false};
	double share{1.0};
	std::vector<compound_arc> arcs;
	// the cant of the first arc of a compound curve, as the ones of the arcs after it
	double first_cant{-1.0};
	// two transition curves meeting at the curvature of the radius, with no arc between them and no cant: the bend of two straights
	// eased the way the lines are drawn. Their lengths follow from the radius and the deflection, the ones given are left out
	bool spirals{false};
	// the cant ramp at each end of the curve where it doesn't go along the transition curve (ST-T1-A6 9.6 pkt 5 and 11): how far its
	// outer end lies into the curve from the start of the curve, or from its end, below 0 out along the straight, and its length; a
	// length below 0 keeps the ramp along the transition, as the norms recommend
	double ramp_in_offset{0.0};
	double ramp_in_length{-1.0};
	double ramp_out_offset{0.0};
	double ramp_out_length{-1.0};
	bool own_ramps() const { return ramp_in_length >= 0.0 || ramp_out_length >= 0.0; }
};

struct design
{
	glm::dvec3 start{0.0};
	glm::dvec3 end{0.0};
	glm::dvec2 start_direction{0.0, 1.0};
	glm::dvec2 end_direction{0.0, 1.0};
	double start_grade{0.0};
	double end_grade{0.0};
	double start_radius{0.0};
	double end_radius{0.0};
	// length of the straight beyond the start and the end, which the curves can take over
	double start_reserve{0.0};
	double end_reserve{0.0};
	std::vector<vertex> vertices;
	transition_shape shape{transition_shape::cubic_parabola};
	double speed{100.0};
	int transition_pieces{1};
	double arc_piece_angle{10.0};
	limits norms;
};

struct recommendation
{
	double cant_equilibrium{0.0};
	double cant_min{0.0};
	double cant{0.0};
	double radius_min{0.0};
	double transition{0.0};
};
recommendation recommend(double const Speed, double const Radius, limits const &Limits);
double transition_length(double const Speed, double const Radius, double const Cant, limits const &Limits);
// the transition curve from the curvature and the cant of one arc to those of the next one, 0 for a straight; the curvature positive
// turning one way, the cant with the same sign as the curvature it leans into
double transition_between(double const Speed, double const Curvature0, double const Cant0, double const Curvature1, double const Cant1, limits const &Limits);
double unbalanced_acceleration(double const Speed, double const Radius, double const Cant, limits const &Limits);
// the cants of the arcs of a compound curve, the first one's first: each its own, or the one in proportion to its curvature
std::vector<double> compound_cants(vertex const &Vertex);

enum class element_kind
{
	straight,
	transition_in,
	arc,
	transition_out,
	spiral
};

struct element
{
	element_kind kind{element_kind::straight};
	int vertex{-1};
	double chainage{0.0};
	double length{0.0};
	double radius{0.0};
	double transition{0.0};
	int turn{0};
	double cant{0.0};
	double curvature_start{0.0};
	double curvature_end{0.0};
	// the cant of a spiral at its start and at its end, going from the one to the other along it
	double cant_start{0.0};
	double cant_end{0.0};
	glm::dvec2 origin{0.0};
	glm::dvec2 direction{0.0, 1.0};
	glm::dvec2 normal{0.0};
};

struct curve_report
{
	int vertex{-1};
	double radius{0.0};
	std::vector<double> radii;
	std::vector<double> cants; // of the radii of a compound curve
	double transition_in{0.0};
	double transition_out{0.0};
	double deflection{0.0};
	double arc_length{0.0};
	double tangent_in{0.0};
	double tangent_out{0.0};
	double unbalanced{0.0};
	double cant_rate_in{0.0};
	double cant_rate_out{0.0};
	double jerk_in{0.0};
	double jerk_out{0.0};
	recommendation recommended;
	bool spirals{false}; // two transition curves with no arc between them
	double cant{0.0};
};

struct profile_point
{
	double chainage{0.0};
	double elevation{0.0};
	double radius{0.0};
	double tangent{0.0};
};

struct result
{
	bool valid{false};
	transition_shape shape{transition_shape::cubic_parabola};
	double gauge{1500.0};
	std::vector<std::string> errors;
	std::vector<std::string> warnings;
	std::vector<std::string> radius_warnings;
	std::vector<glm::dvec2> vertices;
	std::vector<double> vertex_chainages;
	std::vector<double> vertex_elevations;
	std::vector<element> elements;
	std::vector<curve_report> curves;
	std::vector<profile_point> profile;
	// the cant along the line, mm at the chainages, between them changing evenly; signed with the turn of the curve it belongs to,
	// as the rolls of the paths are
	std::vector<glm::dvec2> cants;
	// the cant ramps at the ends of the curves, where the cant goes from 0 to the one of the curve
	struct cant_ramp
	{
		int vertex{-1};
		double from{0.0};
		double to{0.0};
		double cant{0.0}; // mm, at the end of the ramp in the curve
		bool own{false}; // not along the transition curve
	};
	std::vector<cant_ramp> ramps;
	double length{0.0};
	glm::dvec3 start{0.0};
	glm::dvec3 end{0.0};
	double start_extension{0.0};
	double end_extension{0.0};
};

result compute(design const &Design);
double speed_limit(double const Radius, limits const &Limits);
result compute_recommended(design &Design, bool const Transitions);
bool tangent_intersection(design const &Design, glm::dvec2 &Point);
bool collinear(design const &Design);
bool spiral_tangents(transition_shape const Shape, double const TurnIn, double const TurnOut, double &TangentIn, double &TangentOut);

struct station
{
	glm::dvec3 position{0.0};
	glm::dvec3 direction{0.0};
	double cant{0.0};
	int turn{0};
};
station evaluate(result const &Result, double const Chainage);
double elevation(result const &Result, double const Chainage);
double grade(result const &Result, double const Chainage);

// a point of the plan projected on the alignment, its offset positive to the left; past the ends it's measured from the straights carried on
struct plan_projection
{
	double chainage{0.0};
	double offset{0.0};
};
std::vector<plan_projection> project(result const &Result, std::vector<glm::dvec2> const &Points);

struct trace_fit
{
	bool valid{false};
	vertex fitted;
	double rms{0.0};
	double worst{0.0};
};
// the radii of the curve of the vertex and their shares which follow the points traced along the track best; the tangents, the
// transitions and the cant stay. Radii 0 takes the fewest radii, up to three, which follow the points about as well as more would
trace_fit fit_trace(design const &Design, std::size_t const Vertex, std::vector<glm::dvec2> const &Points, int const Radii);
// the radii of the curve of the vertex and their shares fitted again to the points, as many radii as it has; its transitions, the
// ones between the arcs too, and its cants kept: a curve whose transitions were lengthened keeps to the points it was laid through
trace_fit refit_radii(design const &Design, std::size_t const Vertex, std::vector<glm::dvec2> const &Points);

std::size_t minimum_pieces(result const &Result, design const &Design);
// Breaks: ascending chainages at which pieces have to end. Counts: least number of the pieces between the consecutive breaks,
// one more than the breaks; Intervals receives the number of the interval of each piece
std::vector<segment_data> pieces(result const &Result, design const &Design, std::size_t const Count, std::vector<double> const &Breaks = {}, std::vector<std::size_t> const &Counts = {},
                                 std::vector<std::size_t> *Intervals = nullptr);
// the points of the plan set square onto the line, the chainages of them in their order between From and To; empty where they
// don't follow one another along it
std::vector<double> breaks_at(result const &Result, std::vector<glm::dvec2> const &Points, double const From, double const To);
// the pieces of the line ending at the joints of the old paths set square onto it, so that each old path keeps its stretch of the
// ground: Intervals receives the old path of each piece, or stays empty where the joints don't follow one another along the line
// and the pieces are cut as by pieces() alone
std::vector<segment_data> pieces_at(result const &Result, design const &Design, std::size_t const Count, std::vector<glm::dvec2> const &Joints, std::vector<std::size_t> &Intervals);

struct join_end
{
	glm::dvec3 position{0.0};
	glm::dvec2 direction{0.0, 1.0};
	double roll{0.0};

	bool operator==(join_end const &) const = default;
};

enum class join_kind
{
	none,
	straight,
	arc,
	curve,
	reverse_curve,
	bezier
};

struct join_request
{
	join_end start;
	join_end end;
	double radius{0.0};
	int variant{0};
	bool transitions{false};
	double speed{100.0};
	double kink{0.1};
	transition_shape shape{transition_shape::cubic_parabola};
	limits norms;
};

struct join_result
{
	join_kind kind{join_kind::none};
	std::vector<segment_data> pieces;
	double length{0.0};
	double radius{0.0};
	double cant{0.0};
	double transition{0.0};
	double speed{0.0};
	double kink{0.0};
	int variants{0};
	std::vector<std::string> warnings;
	std::vector<std::string> radius_warnings;
};

join_result join(join_request const &Request);

enum class lay_kind
{
	arc_straight,
	straight_arc,
	arc,
	straight
};

// a track laid on from an end the way the layout lays it, with the transition curves of the norms: from the curvature the end has
// into the arc, between the arcs of a compound curve where both turn the same way, and out of the arc to the straight
struct lay_request
{
	join_end start; // the roll in degrees, as the track goes on
	double curvature{0.0}; // of the end as the track goes on, positive turning from x towards z
	glm::dvec3 target{0.0};
	lay_kind kind{lay_kind::arc_straight};
	double radius{500.0}; // of the arc, but the arc through the target
	double speed{100.0};
	double piece_length{100.0};
	limits norms;

	bool operator==(lay_request const &) const = default;
};

// no pieces where the norms find no room for the transitions, the note says why
struct lay_result
{
	std::vector<segment_data> pieces;
	std::string note;
	double length{0.0};
	double radius{0.0};
	double cant{0.0};
	double entry{0.0}; // the transition out of the curvature of the end
	double exit{0.0};
	double from_radius{0.0}; // of the end, 0 for a straight
	bool compound{false};
};

std::optional<lay_result> lay_on(lay_request const &Request);

// the transitions between the arcs of a compound curve which the norms ask for at the speed and at the cants of the arcs
void recommend_between(vertex &Vertex, double const Speed, limits const &Limits);
// all the transitions of the vertex as long as the norms ask for at its radii and its cants, the cants kept: a cant lowered
// lengthens them, the acceleration across left changing more along them
void norm_transitions(vertex &Vertex, double const Speed, limits const &Limits);

// the length of each of the two transition curves of a vertex with spirals, meeting at the curvature of the radius
double spirals_length(transition_shape const Shape, double const Deflection, double const Radius);
// the curve keeps to the norms at the speed of the design: the radius, the acceleration across, the transitions long enough
bool meets_norms(curve_report const &Curve, design const &Design);

// the radii the bends of the straights are eased with: the two transition curves meet at a radius from High down to Low, or above
// High where they'd be too short for the speed; where they don't fit, an arc with the transitions and the cant of the norms
struct corner_radii
{
	double low{3000.0};
	double high{6000.0};
};
// a stretch of the line traced on the photo: the points clicked along the middle of the track, from the straight Base starts on to
// the one it ends on, both fixed by the points clicked on them
struct trace_stretch
{
	bool valid{false};
	// the curves of the stretch with their places: one, of up to three radii, or two with a straight between them where the points
	// follow those much better; none where the stretch is straight
	std::vector<vertex> vertices;
	std::vector<double> offsets; // of the points from the line laid, positive to the left
	double rms{0.0};
	double worst{0.0};
};
trace_stretch fit_trace_stretch(design const &Base, std::vector<glm::dvec2> const &Points, int const Radii, bool const Straights);
// the fit of a stretch traced, with up to three radii and no straight in it; none while it's being fitted elsewhere
using stretch_fit = std::function<std::optional<trace_stretch>(design const &, std::vector<glm::dvec2> const &)>;

// the vertices marked 1 are given the curves of the bends of straights clicked on the photo, as large as fit between the neighbouring
// ones, these sharing the straights between them; the vertices stay where they are, a curve below the norms is left with its
// warnings. A row of bends turning the same way, clicked so close that the straights between their curves would be shorter than
// the shortest element, is taken for points along a curve: one curve, compound where it has to, is fitted through them, or one for
// each part of the row where the whole doesn't keep to them; such a vertex is marked 0 then. Fit fits it, by fit_trace_stretch
// where none is given
result ease_corners(design &Design, std::vector<char> &Corners, corner_radii const &Radii, stretch_fit const &Fit = {});

// the stretches traced fitted on a thread of their own while the cursor moves: fitting one takes tens of milliseconds. The one asked
// for last is fitted first, the oldest of too many waiting left out, the fits of the last ones kept
class stretch_fitter
{
  public:
	struct request
	{
		std::vector<double> key; // tells the requests apart
		design base;
		std::vector<glm::dvec2> points;
		int radii{0};
		bool straights{true};
	};
	~stretch_fitter();
	void ask(request Request);
	// the fit of the request of the key, once it's done
	bool done(std::vector<double> const &Key, trace_stretch &Fit) const;
	// the fit done last, of whatever request
	bool latest(std::vector<double> &Key, trace_stretch &Fit) const;

  private:
	void work();

	mutable std::mutex m_mutex;
	std::condition_variable m_wake;
	std::vector<request> m_asked;
	std::optional<request> m_fitting;
	std::vector<std::pair<std::vector<double>, trace_stretch>> m_results; // the last done first
	bool m_stop{false};
	std::thread m_thread;
};

}

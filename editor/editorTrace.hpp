/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "editor/editorAlignment.hpp"

// The line traced on the aerial photo: the points clicked one after another along the middle of the track seen on the photo, each with
// the noise of its click, made into the straights and the curves the track runs along, the way the track was built: a straight where the
// points keep to one within the noise, else a curve, a very gentle one too (R 5 - 20 km, "nearly straight"), of one radius or compound,
// with its transitions and its cant as the norms ask for at the speed, and curves the other way with or without a straight between them.
// The line may start at a free end, its heading and its curvature held, or anywhere, inside a curve too.
//
// Each click fits the open end of the line only: a curve with two points clicked on the straight after it is frozen, the straight still
// turning about the end of its last arc until the next curve is frozen, so that what lies behind doesn't change and a click costs as much
// at the end of a long line as at its start. A click's fit takes a budget of evaluations of the line, the same for the same points: where
// it's spent, the fit doesn't converge or the line comes out worse than before the point, the line stays as it was and the point is told
// as one which doesn't fit. A curve sharper than the norms allow at the speed of the line is laid at the speed its radius and its
// transitions allow, the trains slowed down along it; only one below the least radius of the kind of the track is refused.
//
// The domain alone: no scene, no tracks, no user interface; the plan in metres, x and y of the plan (x and z of the scenery).
namespace trace
{

// the noise of a click: two pixels of the view at it, 5 cm at least
double click_sigma(double const MetresPerPixel);

// what the line is held to at its start or its end: the free end of a track it goes on from or into (later the diverging end of a turnout).
// The heading as the line runs, the curvature signed by the side it turns to as the line runs, positive to the left
struct hold
{
	glm::dvec2 position{0.0};
	glm::dvec2 heading{1.0, 0.0};
	double curvature{0.0};
	double cant{0.0}; // mm, of the track there, leaning into its turn
	bool fixed{false}; // the diverging track of a switch: a curve goes on at its radius with no transition, a straight may begin right there
};

struct settings
{
	double speed{100.0}; // km/h of the line
	alignment::limits norms;
	alignment::transition_shape shape{alignment::transition_shape::cubic_parabola};
	double radius_least{0.0}; // m: a curve sharper is refused at any speed, the least radius of the norms for the kind of the track; 0 none
	double speed_step{10.0}; // km/h: a curve below the norms is slowed down by these
	double speed_least{10.0}; // km/h: the slowest a curve is laid for
	std::size_t budget{20000}; // evaluations of the line a click's fit may take

	bool operator==(settings const &) const = default;
};

// a curve of the line traced, one vertex of its design
struct curve
{
	alignment::vertex vertex; // as the design has it, its speed among it where it's laid slower than the line
	std::vector<double> radii; // of its arcs, the first one first
	double speed{0.0}; // km/h it's laid for: the one of the line, or below it where its radius or its transitions don't meet the norms at that
	bool restricted{false}; // slower than the line
	bool refused{false}; // sharper than the least radius of the kind of the track
	bool nearly_straight{false}; // R 5 km and more
	bool open{false}; // the line ends within its last arc
	bool inside{false}; // the line starts within its first arc
	bool frozen{false};
	double from{0.0}; // chainage of its start and of its end along the line laid
	double to{0.0};
	double sharpest() const;
	bool compound() const { return radii.size() > 1; }
};

struct result
{
	bool valid{false}; // a line to lay: two points clicked at least, the design computed
	alignment::design design; // in the plan, the heights 0: its start and end, the vertices of its curves
	alignment::result laid; // computed
	std::vector<curve> curves; // one for each vertex of the design
	std::vector<double> offsets; // of the points clicked from the line laid, positive to the left
	std::vector<double> sigmas; // of the points
	std::vector<char> misfit; // a point the line couldn't be fitted to: the line stayed as it was before it
	std::vector<char> on_straight; // a point on a straight of the line, not on a curve
	double rms{0.0};
	double worst{0.0}; // m, the farthest point
	bool refused{false}; // a curve below the least radius: not to be laid
	std::string error; // why there's no line, or why it's refused
	std::size_t frozen{0}; // curves frozen
	// of the last click
	double milliseconds{0.0};
	std::size_t evaluations{0};
	bool budget_spent{false};
};

// the line built click after click
class tracer
{
  public:
	tracer();
	tracer(settings const &Settings, std::optional<hold> const &Start = {});
	tracer(tracer const &Other);
	tracer &operator=(tracer const &Other);
	~tracer();

	// a point clicked, with the noise of its click in metres
	void add(glm::dvec2 const &Point, double const Sigma);
	// the last point taken back, the line as it was before it
	void pop();
	std::size_t size() const;
	std::vector<glm::dvec2> const &points() const;
	settings const &options() const;
	std::optional<hold> const &start() const;
	// the line of the points clicked so far
	result const &now() const;
	// the same ending at End, a free end the last click reached (the end's position isn't among the points)
	result ended(hold const &End) const;

  private:
	struct impl;
	impl *m_impl{nullptr};
};

// the line of the points clicked one after another, as a tracer makes it
result traced(std::vector<glm::dvec2> const &Points, std::vector<double> const &Sigmas, settings const &Settings, std::optional<hold> const &Start = {});

// how far a run of the points of a line traced keeps from a neighbouring line (points along it, in order), each point measured square
// to the neighbour: the mean spacing, positive to the left of the neighbour as it runs, and how much it varies; a run keeping a constant
// spacing within the noise of its points is parallel
struct spacing
{
	bool parallel{false};
	double mean{0.0};
	double spread{0.0}; // the root mean square of the departures from the mean
	std::size_t points{0};
};
spacing spacing_to(std::vector<glm::dvec2> const &Points, std::vector<double> const &Sigmas, std::vector<glm::dvec2> const &Neighbour);

// the radii of the curve as the labels tell them: "850 m", "600/450 m", "9 km"
std::string radii_text(curve const &Curve);

} // namespace trace

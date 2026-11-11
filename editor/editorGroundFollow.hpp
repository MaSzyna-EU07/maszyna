/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>
#include <glm/glm.hpp>

#include "editor/editorProfile.hpp"
#include "world/Segment.h"

namespace ground_follow
{

using bed_query = std::function<std::vector<double>(std::vector<glm::dvec3> const &, std::vector<glm::dvec2> const &)>;

struct ends
{
	double height0{0.0};
	double grade0{0.0};
	double height1{0.0};
	double grade1{0.0};
};

struct knot_line
{
	std::vector<double> knots;
	std::vector<double> heights;
	std::vector<double> slopes;
	double grade0{0.0};
	double grade1{0.0};

	void refresh();
	double at(double const Chainage, double &Grade) const;
};

// the grade line the track is laid on where there's ground under it: the solver of the grade lines fits it to the ground
struct grading
{
	profile::line settings; // the speed and the norms of the line
	profile::ground_fit fit; // how closely it keeps to the ground
	double reach{100.0}; // m, how far the ground may part from the line and still be followed
};

// the ground under a track, sampled along it, and what the grade line is fitted to
struct request
{
	std::vector<double> chainages;
	std::vector<double> targets; // the height of the bed at each chainage, NaN where there's no ground
	double total{0.0}; // m, the length in plan
	ends heights;
	grading grades;

	bool operator==(request const &Other) const;
};

// the grade lines of the track shown while it's laid, fitted on a thread of their own: fitting one takes some milliseconds,
// tens of them on a long track over the terrain model, and the track follows the cursor frame after frame. The latest one
// asked for is fitted, the ones asked for meanwhile are left out
class fitter
{
  public:
	~fitter();
	// the line of exactly this request, once fitted; graded false: none was found, the heights lead from end to end
	bool done(request const &Request, profile::line &Line, bool &Graded) const;
	// the line fitted last, of whatever request
	bool latest(request &Request, profile::line &Line, bool &Graded) const;
	void ask(request Request);

  private:
	struct result
	{
		request asked;
		profile::line line;
		bool graded{false};
	};
	void work();

	mutable std::mutex m_mutex;
	std::condition_variable m_wake;
	std::optional<request> m_asked;
	std::optional<request> m_fitting;
	std::optional<result> m_result;
	bool m_stop{false};
	std::thread m_thread;
};

// where Bed finds no ground, the heights lead from one end to the other. With a fitter the line it fitted for the same ground
// is taken instead of fitting it again
void follow(std::vector<segment_data> &Pieces, ends const &Ends, bed_query const &Bed, grading const &Grading = {}, fitter const *Fitted = nullptr);
// the same, though the line is fitted by the fitter: until it has it, the track gets the line it fitted last stretched over its
// length, if it went on from the same end, or the heights from end to end
void follow_meanwhile(std::vector<segment_data> &Pieces, ends const &Ends, bed_query const &Bed, grading const &Grading, fitter &Fitter);
// from point to point: the points set square onto the pieces, an even grade between each two of them, the grade breaking only at
// them; from the ends of the heights given, a grade given there led on into the first and the last stretch
void follow_points(std::vector<segment_data> &Pieces, std::vector<glm::dvec3> const &Points, ends const &Ends);

} // namespace ground_follow

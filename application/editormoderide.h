/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editortool.h"

#include <string>

class TTrack;
class TSegment;
class TCamera;

// the place of the camera riding along the tracks
namespace track_ride_walk
{

struct place
{
	TTrack *track{nullptr};
	int path{-1}; // of a switch, -1 on other tracks
	double s{0.0}; // m from Point1 of the path
};

// how the walk ended
enum class halt
{
	none,
	end, // the track ends there, or goes on into a crossroads of roads
};

// the path the place is on
TSegment *segment(place const &Place);
// the point of the path and its unit direction towards the growing distance
bool frame(place const &Place, glm::dvec3 &Point, glm::dvec3 &Forward);
// the path of a switch taken from the blades: 0 the one going on straighter, 1 the other one
int switch_path(TTrack const &Switch, int const Choice);
// moves the place by Distance along the path (negative: towards Point1), over the joints to the next tracks; a switch
// entered from its blades is left by the path Choice gives (switch_path), one entered from the frog by the path it came on;
// the switches aren't thrown and nothing on the tracks learns of it. Flips is how many times the counting of the distance
// turned round on the way, at the joints of tracks running the other way. Facing is set to true when a switch was
// entered from the blades
halt walk(place &Place, double const Distance, int const Choice, int &Flips, bool &Facing);
// the nearest switch ahead to be entered from its blades, moving towards the growing distance (Direction 1) or the other
// way (-1), within Reach; null when there's none
TTrack *facing_switch_ahead(place Place, int Direction, double const Reach, double &Distance);
// the distance along the path nearest to the point, in the plan
double nearest_distance(TSegment const &Segment, glm::dvec3 const &Point);

} // namespace track_ride_walk

// the camera riding along the tracks as a vehicle: the keys drive it over the joints and switches and jump it over to the
// tracks alongside, the mouse looks around as it does when flying
class track_ride final : public editor_tool
{
  public:
	bool active() const { return m_place.track != nullptr; }
	// onto the path at the point nearest to Near, the way ahead the one nearer to Look; Orient: the camera turns to look
	// along the track, else it keeps looking where it did. False where it can't
	bool enter(TTrack *Track, int const Path, glm::dvec3 const &Near, glm::dvec3 const &Look, bool const Orient);
	void leave();
	TTrack const *track() const { return m_place.track; }
	// a key while riding; true when it was taken. Status is a message to show. The keys driving the ride are taken only with
	// Driving, the right button held as it is to fly the camera; without it they're the editor's
	bool key(int const Key, int const Action, int const Mods, glm::dvec3 const &Look, bool const Driving, std::string &Status);
	void wheel(double const Notches);
	// the keys held moved the place by the time passed, the camera is put above the rails there; none is held without Driving
	void advance(TCamera &Camera, double const Deltatime, bool const Driving);
	// to the track nearest the place on the side (-1 left, 1 right) of the way the camera looks; false with none within reach
	bool jump_side(int const Side, glm::dvec3 const &Look);
	// the bar of the ride over the view
	void draw_hud() const;
	void forget(scene::basic_node const *Node) override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;

	static constexpr double default_height{3.0}; // m over the rails

  private:
	// the direction of the growing distance the camera looks
	int look_direction(glm::dvec3 const &Look) const;
	void cycle_choice();
	glm::dvec3 rail_point() const;

	track_ride_walk::place m_place;
	int m_direction{1}; // the growing distance (1) or the other way (-1), which is ahead
	double m_speed{0.0}; // m/s ahead, negative backwards
	double m_height{default_height};
	int m_choice{0}; // at the next switch from its blades
	// keys held
	bool m_forward{false};
	bool m_back{false};
	bool m_up{false};
	bool m_down{false};
	bool m_shift{false};
	bool m_ctrl{false};
	// the camera glides over from where it was, as it enters or jumps over
	glm::dvec3 m_glide_from{0.0};
	double m_glide{1.0}; // 0..1, done at 1
	bool m_glide_pending{false}; // starts from where the camera is on the next step
	bool m_orient_pending{false}; // the camera turns along the track on the next step
	// the switch ahead, for the bar
	TTrack *m_switch{nullptr};
	double m_switch_distance{0.0};
	// kilometrage of the place, worked out now and then
	std::string m_km;
	double m_km_age{1e9};
};

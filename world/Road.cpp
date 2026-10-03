/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/Road.h"

#include "world/Track.h"
#include "simulation/simulation.h"
#include "scene/scene.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"

namespace
{

// dimensions of the markings, in metres
// NOTE: these give a layout resembling an ordinary polish road, they weren't taken from the regulations
constexpr double line_width{0.12};
constexpr double line_spacing{0.12}; // gap between two lines painted side by side
constexpr double line_lift{0.02}; // the markings are raised over the surface, so the two don't fight for depth
constexpr double edge_inset{0.15}; // distance between the edge of the surface and the line running along it
constexpr double dash_period{12.0}; // a third of it is painted, the rest is the gap
constexpr double dash_step{2.0}; // longest straight piece of a painted line
// speed and reach of the paths made to turn the vehicles back at a loose end of the road
constexpr float turn_velocity{10.f};
constexpr double turn_reach{6.0};

glm::dvec3 const up{0.0, 1.0, 0.0};

// placement of a cross-section of the road
struct axis_frame
{
	glm::dvec3 position;
	glm::dvec3 tangent; // direction of the axis
	glm::dvec3 left; // direction across the road, tilted by the roll
	double roll{0.0};
};

// the axis of a road piece; a cubic bezier curve, the same thing the paths of the tracks are made of
class road_axis
{

  public:
	explicit road_axis(segment_data const &Path)
	{
		auto const &start{Path.points[segment_data::point::start]};
		auto const &end{Path.points[segment_data::point::end]};
		auto const &control1{Path.points[segment_data::point::control1]};
		auto const &control2{Path.points[segment_data::point::control2]};
		m_curved = (control1 != glm::dvec3{0.0}) || (control2 != glm::dvec3{0.0});
		m_points[0] = start;
		m_points[3] = end;
		if (m_curved)
		{
			m_points[1] = start + control1;
			m_points[2] = end + control2;
		}
		else
		{
			// a straight gets its control points in thirds of the length, the way the tracks do it
			m_points[1] = start + (end - start) / 3.0;
			m_points[2] = end - (end - start) / 3.0;
		}
		m_rolls = {glm::radians(static_cast<double>(Path.rolls[0])), glm::radians(static_cast<double>(Path.rolls[1]))};
		m_radius = std::abs(Path.radius);
		// distances along the curve for evenly spaced values of its parameter, to convert the former into the latter
		m_stations.resize(samples + 1);
		m_stations[0] = 0.0;
		auto previous{m_points[0]};
		for (int idx = 1; idx <= samples; ++idx)
		{
			auto const current{point(static_cast<double>(idx) / samples)};
			m_stations[idx] = m_stations[idx - 1] + glm::distance(previous, current);
			previous = current;
		}
	}
	bool curved() const
	{
		return m_curved;
	}
	double length() const
	{
		return m_stations.back();
	}
	// control vector of specified end of the curve, relative to that end
	glm::dvec3 control(int const End) const
	{
		return End == 0 ? m_points[1] - m_points[0] : m_points[2] - m_points[3];
	}
	glm::dvec3 point(double const T) const
	{
		auto const u{1.0 - T};
		return u * u * u * m_points[0] + 3.0 * u * u * T * m_points[1] + 3.0 * u * T * T * m_points[2] + T * T * T * m_points[3];
	}
	// curvature of the axis seen from above, positive when it bends to the left
	double curvature(double const T) const
	{
		auto const first{derivative(T)};
		auto const second{6.0 * (1.0 - T) * (m_points[2] - 2.0 * m_points[1] + m_points[0]) + 6.0 * T * (m_points[3] - 2.0 * m_points[2] + m_points[1])};
		auto const speedsquared{first.x * first.x + first.z * first.z};
		if (speedsquared < 1e-9)
		{
			return 0.0;
		}
		return (second.x * first.z - second.z * first.x) / (speedsquared * std::sqrt(speedsquared));
	}
	// cross-section at specified value of the curve parameter
	axis_frame frame(double const T) const
	{
		axis_frame frame;
		frame.position = point(T);
		auto direction{derivative(T)};
		if (glm::length2(direction) < 1e-9)
		{
			// a control point placed right at the end of the curve
			direction = derivative(std::clamp(T, 0.01, 0.99));
		}
		if (glm::length2(direction) < 1e-9)
		{
			direction = m_points[3] - m_points[0];
		}
		if (glm::length2(direction) < 1e-9)
		{
			direction = glm::dvec3{0.0, 0.0, 1.0};
		}
		frame.tangent = glm::normalize(direction);
		// with the y axis up and the vehicle facing along z its left side is at positive x
		glm::dvec3 across{frame.tangent.z, 0.0, -frame.tangent.x};
		across = (glm::length2(across) < 1e-9 ? glm::dvec3{1.0, 0.0, 0.0} : glm::normalize(across));
		// positive roll lowers the right side of the road, same as it does for the tracks
		frame.roll = glm::mix(m_rolls[0], m_rolls[1], T);
		frame.left = across * std::cos(frame.roll) + up * std::sin(frame.roll);
		return frame;
	}
	// cross-section at specified distance from the start of the axis
	axis_frame at(double const Station) const
	{
		return frame(parameter(Station));
	}
	// length of pieces the road is cut into along the axis; follows the rules the tracks use
	double step() const
	{
		if (false == m_curved)
		{
			return 10.0;
		}
		auto const fidelity{std::max(1.0, static_cast<double>(Global.SplineFidelity))};
		return std::clamp(m_radius != 0.0 ? m_radius * 0.02 / fidelity : length() * 0.1, 2.0 / fidelity, 10.0 / fidelity);
	}

  private:
	static constexpr int samples{64};

	glm::dvec3 derivative(double const T) const
	{
		auto const u{1.0 - T};
		return 3.0 * u * u * (m_points[1] - m_points[0]) + 6.0 * u * T * (m_points[2] - m_points[1]) + 3.0 * T * T * (m_points[3] - m_points[2]);
	}
	double parameter(double const Station) const
	{
		if (Station <= 0.0)
		{
			return 0.0;
		}
		if (Station >= length())
		{
			return 1.0;
		}
		// first of the sampled distances past the requested one; it's never the very first sample
		auto const idx{std::distance(m_stations.begin(), std::upper_bound(m_stations.begin(), m_stations.end(), Station))};
		auto const span{m_stations[idx] - m_stations[idx - 1]};
		auto const fraction{span > 0.0 ? (Station - m_stations[idx - 1]) / span : 0.0};
		return (static_cast<double>(idx - 1) + fraction) / samples;
	}

	std::array<glm::dvec3, 4> m_points;
	std::array<double, 2> m_rolls{};
	double m_radius{0.0};
	bool m_curved{false};
	std::vector<double> m_stations;
};

// corner of the cross-section of something laid along the road
struct profile_point
{
	double offset; // distance from the axis, positive to the left. this part of it follows the taper of the road
	double height; // above the surface
	float texture; // texture coordinate across the road
	double shift{0.0}; // further distance from the axis, which the taper leaves alone
};

// how far the taper of a road has got at specified part of its length. it starts and ends level, so the pieces of a road
// meet without a bend in their edges, and it's what a lane running from one width to another comes to on a straight
double taper_progress(double const Fraction)
{
	auto const t{std::clamp(Fraction, 0.0, 1.0)};
	return t * t * (3.0 - 2.0 * t);
}

// stretches provided profile along a part of the axis. the profile points go left to right, each two neighbours make a band of triangles.
// Taper: what the offsets of the profile are multiplied by at the start and at the end of the axis
void loft(std::vector<world_vertex> &Output, road_axis const &Axis, std::vector<profile_point> const &Profile, double const From, double const To, double const Step, double const Texturelength,
          std::array<float, 2> const &Taper)
{
	if (Profile.size() < 2 || To <= From || Step <= 0.0)
	{
		return;
	}
	auto const count{std::max(1, static_cast<int>(std::ceil((To - From) / Step - 0.001)))};
	auto const pointcount{Profile.size()};
	// each band is flat across the road, and follows its bends along it
	auto const normal = [](glm::dvec3 const &Tangent, glm::dvec3 const &Across) {
		auto const result{glm::cross(Tangent, Across)};
		return glm::vec3{glm::length2(result) < 1e-12 ? up : glm::normalize(result)};
	};
	std::vector<glm::dvec3> previous(pointcount), current(pointcount);
	axis_frame previousframe;
	double previousstation{From};

	for (int section = 0; section <= count; ++section)
	{
		auto const station{From + (To - From) * section / count};
		auto const frame{Axis.at(station)};
		auto const scale{Taper[0] == Taper[1] ? static_cast<double>(Taper[0]) : glm::mix(static_cast<double>(Taper[0]), static_cast<double>(Taper[1]), taper_progress(station / Axis.length()))};
		for (std::size_t idx = 0; idx < pointcount; ++idx)
		{
			current[idx] = frame.position + frame.left * (Profile[idx].offset * scale + Profile[idx].shift) + up * Profile[idx].height;
		}
		if (section > 0)
		{
			auto const texturestart{static_cast<float>(previousstation / Texturelength)};
			auto const textureend{static_cast<float>(station / Texturelength)};
			for (std::size_t idx = 0; idx + 1 < pointcount; ++idx)
			{
				auto const normalstart{normal(previousframe.tangent, previous[idx] - previous[idx + 1])};
				auto const normalend{normal(frame.tangent, current[idx] - current[idx + 1])};
				world_vertex const leftstart{previous[idx], normalstart, {Profile[idx].texture, texturestart}};
				world_vertex const rightstart{previous[idx + 1], normalstart, {Profile[idx + 1].texture, texturestart}};
				world_vertex const leftend{current[idx], normalend, {Profile[idx].texture, textureend}};
				world_vertex const rightend{current[idx + 1], normalend, {Profile[idx + 1].texture, textureend}};
				Output.push_back(leftstart);
				Output.push_back(rightstart);
				Output.push_back(leftend);

				Output.push_back(rightstart);
				Output.push_back(rightend);
				Output.push_back(leftend);
			}
		}
		previous.swap(current);
		previousframe = frame;
		previousstation = station;
	}
}

// proportions of the image specified material is painted with
float texture_ratio(std::string const &Material)
{
	auto const material{GfxRenderer->Fetch_Material(Material)};
	if (material == null_handle)
	{
		return 1.f;
	}
	auto const texture{GfxRenderer->Material(material)->GetTexture(0)};
	if (texture == null_handle)
	{
		return 1.f;
	}
	auto const &image{GfxRenderer->Texture(texture)};
	return image.get_height() > 0 ? static_cast<float>(image.get_width()) / static_cast<float>(image.get_height()) : 1.f;
}

} // namespace

// brings the content to a usable form: values within their limits, an entry for each lane and for each pair of neighbours
void road_node::state::normalize()
{
	forward = std::clamp(forward, 0, 8);
	backward = std::clamp(backward, 0, 8);
	if (forward + backward == 0)
	{
		forward = 1;
	}
	lanewidth = std::max(1.f, lanewidth);
	for (auto &end : taper)
	{
		end = std::clamp(end, 0.25f, 4.f);
	}
	if (texturelength < 0.01f)
	{
		texturelength = 4.f;
	}
	if (surface.empty())
	{
		surface = "none";
	}
	// the paths of the lanes are loaded from text, so what goes into it has to be something they understand
	static std::array<std::string, 7> const environments{"flat", "mountains", "mountain", "canyon", "tunnel", "bridge", "bank"};
	if (std::find(environments.begin(), environments.end(), environment) == environments.end())
	{
		environment = "flat";
	}
	auto const count{static_cast<std::size_t>(forward + backward)};
	if (lanes.size() != count)
	{
		lane_data fresh;
		fresh.width = lanewidth;
		fresh.velocity = velocity;
		lanes.assign(count, fresh);
	}
	for (auto &lane : lanes)
	{
		lane.width = std::max(1.f, lane.width);
	}
	if (changes.size() != count - 1)
	{
		changes.resize(count - 1);
		for (std::size_t boundary = 0; boundary < changes.size(); ++boundary)
		{
			changes[boundary] = default_change(boundary);
		}
	}
	for (auto &change : changes)
	{
		change = std::clamp(change, static_cast<int>(change_none), static_cast<int>(change_both));
	}
}

// identifier of a lane: f1..fn go along the axis and b1..bn against it, both counted outwards from where the directions meet
std::string road_node::state::lane_id(std::size_t const Lane) const
{
	auto const against{static_cast<std::size_t>(backward)};
	return Lane < against ? "b" + std::to_string(against - Lane) : "f" + std::to_string(Lane - against + 1);
}

// number of a lane in the left to right order, -1 if there's no lane with such identifier
int road_node::state::lane_index(std::string const &Id) const
{
	if (Id.size() < 2 || Id.size() > 3 || (Id[0] != 'f' && Id[0] != 'b'))
	{
		return -1;
	}
	int number{0};
	for (std::size_t idx = 1; idx < Id.size(); ++idx)
	{
		if (Id[idx] < '0' || Id[idx] > '9')
		{
			return -1;
		}
		number = number * 10 + (Id[idx] - '0');
	}
	if (number < 1 || number > (Id[0] == 'f' ? forward : backward))
	{
		return -1;
	}
	return Id[0] == 'f' ? backward + number - 1 : backward - number;
}

// permission to change the lane a pair of neighbouring lanes gets if the scenery doesn't say otherwise
int road_node::state::default_change(std::size_t const Boundary) const
{
	auto const opposite{backward > 0 && forward > 0 && Boundary + 1 == static_cast<std::size_t>(backward)};
	if (false == opposite)
	{
		return change_both;
	}
	// overtaking across the oncoming lane is left for the simple two lane roads
	return (forward == 1 && backward == 1) ? change_both : change_none;
}

// distance of the middle of a lane from the axis, positive to the left when facing along the axis
double road_node::state::lane_offset(std::size_t const Lane) const
{
	auto offset{0.5 * width()};
	for (std::size_t idx = 0; idx < Lane && idx < lanes.size(); ++idx)
	{
		offset -= lanes[idx].width;
	}
	return Lane < lanes.size() ? offset - 0.5 * lanes[Lane].width : offset;
}

// combined width of the lanes, before the taper
double road_node::state::width() const
{
	double total{0.0};
	for (auto const &lane : lanes)
	{
		total += lane.width;
	}
	return total;
}

// what the widths of the lanes are multiplied by at specified value of the curve parameter
double road_node::state::scale(double const T) const
{
	return glm::mix(static_cast<double>(taper[0]), static_cast<double>(taper[1]), taper_progress(T));
}

// shape of the middle of a lane, laid out in the direction of travel
segment_data road_node::state::lane_path(std::size_t const Lane) const
{
	road_axis const centre{axis};
	// on a road which gets wider or narrower the lane moves away from the axis or towards it along the way
	auto const beginoffset{lane_offset(Lane) * taper[0]};
	auto const endoffset{lane_offset(Lane) * taper[1]};
	auto const begin{centre.frame(0.0)};
	auto const end{centre.frame(1.0)};

	segment_data path;
	path.points[segment_data::point::start] = begin.position + begin.left * beginoffset;
	path.points[segment_data::point::end] = end.position + end.left * endoffset;
	if (centre.curved() || beginoffset != endoffset)
	{
		// a line running beside a bend is shorter or longer than the bend itself, and so are its control vectors.
		// the control vectors go along the axis at both ends, which is what makes the lane meet the next piece without a bend
		path.points[segment_data::point::control1] = centre.control(0) * std::max(0.05, 1.0 - beginoffset * std::cos(begin.roll) * centre.curvature(0.0));
		path.points[segment_data::point::control2] = centre.control(1) * std::max(0.05, 1.0 - endoffset * std::cos(end.roll) * centre.curvature(1.0));
		// if the roll changes along the road its sides rise or fall relative to the axis
		auto const climb{(endoffset * std::sin(end.roll) - beginoffset * std::sin(begin.roll)) / 3.0};
		path.points[segment_data::point::control1].y += climb;
		path.points[segment_data::point::control2].y -= climb;
	}
	path.rolls = axis.rolls;
	if (axis.radius != 0.f)
	{
		// the lanes on the inner side of the bend have it tighter
		auto const turn{centre.curvature(0.5) >= 0.0 ? 1.0 : -1.0};
		path.radius = static_cast<float>(std::max(1.0, std::abs(axis.radius) - turn * 0.5 * (beginoffset + endoffset)));
	}
	if (Lane < static_cast<std::size_t>(backward))
	{
		// the lane goes against the axis
		std::swap(path.points[segment_data::point::start], path.points[segment_data::point::end]);
		std::swap(path.points[segment_data::point::control1], path.points[segment_data::point::control2]);
		path.rolls = {-axis.rolls[1], -axis.rolls[0]};
	}
	if (Global.bRollFix)
	{
		// with this setting on a rolled path gets raised when it's set up, by an amount fit for a railway track.
		// the surface of the road stays where it is, so the lane is lowered beforehand to make up for it
		path.points[segment_data::point::start].y -= std::abs(std::sin(glm::radians(static_cast<double>(path.rolls[0])))) * 0.75;
		path.points[segment_data::point::end].y -= std::abs(std::sin(glm::radians(static_cast<double>(path.rolls[1])))) * 0.75;
	}
	return path;
}

// length of the axis
double road_node::state::length() const
{
	return road_axis{axis}.length();
}

// position on the axis and direction of the axis, for specified value of the curve parameter
glm::dvec3 road_node::state::point(double const T) const
{
	return road_axis{axis}.point(T);
}

glm::dvec3 road_node::state::tangent(double const T) const
{
	return road_axis{axis}.frame(T).tangent;
}

road_node::road_node(scene::node_data const &Nodedata) : basic_node(Nodedata) {}

// restores content of the node from provided input stream; reads up to and including the closing 'endroad'
void road_node::import(cParser &Input, glm::dvec3 const &Offset)
{
	auto const label{m_name.empty() ? std::string{"unnamed road"} : "road \"" + m_name + "\""};

	m_state = state{};
	m_state.axis.deserialize(Input, Offset);

	// properties of the lanes can precede the statement which sets how many of them there are, so they wait until everything is read
	struct lane_property
	{
		std::string lane;
		std::string property;
		float value;
	};
	struct change_property
	{
		std::string first;
		std::string second;
		std::string mode;
	};
	std::vector<lane_property> laneproperties;
	std::vector<change_property> changeproperties;

	auto token{Input.getToken<std::string>()};
	while (false == token.empty() && token != "endroad")
	{
		if (token == "lanes")
		{
			// lanes <along the axis> <against the axis>
			Input.getTokens(2);
			Input >> m_state.forward >> m_state.backward;
		}
		else if (token == "width")
		{
			Input.getTokens();
			Input >> m_state.lanewidth;
		}
		else if (token == "velocity")
		{
			Input.getTokens();
			Input >> m_state.velocity;
		}
		else if (token == "taper")
		{
			// taper <at the start> <at the end>
			Input.getTokens(2);
			Input >> m_state.taper[0] >> m_state.taper[1];
		}
		else if (token == "lane")
		{
			// lane <lane> <width|velocity> <value>
			lane_property property;
			property.lane = Input.getToken<std::string>();
			property.property = Input.getToken<std::string>();
			property.value = 0.f;
			Input.getTokens();
			Input >> property.value;
			laneproperties.emplace_back(property);
		}
		else if (token == "change")
		{
			// change <lane> <neighbouring lane> <both|none|ab|ba>
			change_property property;
			property.first = Input.getToken<std::string>();
			property.second = Input.getToken<std::string>();
			property.mode = Input.getToken<std::string>();
			changeproperties.emplace_back(property);
		}
		else if (token == "surface")
		{
			m_state.surface = Input.getToken<std::string>();
			replace_slashes(m_state.surface);
		}
		else if (token == "texlength")
		{
			Input.getTokens();
			Input >> m_state.texturelength;
		}
		else if (token == "side")
		{
			// side <left|right|both> <none|shoulder|sidewalk> <width> <material>
			auto const where{Input.getToken<std::string>()};
			auto const type{Input.getToken<std::string>()};
			side_data side;
			Input.getTokens();
			Input >> side.width;
			side.material = Input.getToken<std::string>();
			replace_slashes(side.material);
			side.type = (type == "shoulder" ? side_type::shoulder : type == "sidewalk" ? side_type::sidewalk : side_type::none);
			if (type != "shoulder" && type != "sidewalk" && type != "none")
			{
				ErrorLog("Bad road: unknown kind of side \"" + type + "\" defined for " + label);
			}
			if (where == "left" || where == "both")
			{
				m_state.sides[0] = side;
			}
			if (where == "right" || where == "both")
			{
				m_state.sides[1] = side;
			}
			if (where != "left" && where != "right" && where != "both")
			{
				ErrorLog("Bad road: unknown side \"" + where + "\" defined for " + label);
			}
		}
		else if (token == "kerb")
		{
			Input.getTokens();
			Input >> m_state.kerbheight;
		}
		else if (token == "slope")
		{
			// slope <width> <drop>
			Input.getTokens(2);
			Input >> m_state.slope.x >> m_state.slope.y;
		}
		else if (token == "markings")
		{
			auto const colour{Input.getToken<std::string>()};
			m_state.markings = (colour == "white" ? marking_colour::white : colour == "orange" ? marking_colour::orange : marking_colour::none);
			if (colour != "white" && colour != "orange" && colour != "none")
			{
				ErrorLog("Bad road: unknown colour of markings \"" + colour + "\" defined for " + label);
			}
		}
		else if (token == "friction")
		{
			Input.getTokens();
			Input >> m_state.friction;
		}
		else if (token == "environment")
		{
			m_state.environment = Input.getToken<std::string>();
		}
		else
		{
			ErrorLog("Bad road: unknown property: \"" + token + "\" defined for " + label);
		}
		token = Input.getToken<std::string>();
	}

	if (m_state.forward + m_state.backward <= 0)
	{
		ErrorLog("Bad road: " + label + " has no lanes, a single one is used instead");
	}
	auto const environment{m_state.environment};
	m_state.normalize();
	if (environment != m_state.environment)
	{
		ErrorLog("Bad road: unknown environment \"" + environment + "\" defined for " + label);
	}

	for (auto const &property : laneproperties)
	{
		auto const lane{m_state.lane_index(property.lane)};
		if (lane < 0)
		{
			ErrorLog("Bad road: " + label + " has no lane \"" + property.lane + "\"");
			continue;
		}
		if (property.property == "width")
		{
			m_state.lanes[lane].width = std::max(1.f, property.value);
		}
		else if (property.property == "velocity")
		{
			m_state.lanes[lane].velocity = property.value;
		}
		else
		{
			ErrorLog("Bad road: unknown lane property: \"" + property.property + "\" defined for " + label);
		}
	}
	for (auto const &property : changeproperties)
	{
		auto const first{m_state.lane_index(property.first)};
		auto const second{m_state.lane_index(property.second)};
		if (first < 0 || second < 0 || std::abs(first - second) != 1)
		{
			ErrorLog("Bad road: lanes \"" + property.first + "\" and \"" + property.second + "\" aren't neighbours on " + label);
			continue;
		}
		// the permissions are stored for the pair taken left to right
		auto const firstonleft{first < second};
		auto &change{m_state.changes[std::min(first, second)]};
		if (property.mode == "both")
		{
			change = change_both;
		}
		else if (property.mode == "none")
		{
			change = change_none;
		}
		else if (property.mode == "ab")
		{
			change = (firstonleft ? change_toright : change_toleft);
		}
		else if (property.mode == "ba")
		{
			change = (firstonleft ? change_toleft : change_toright);
		}
		else
		{
			ErrorLog("Bad road: unknown lane change mode: \"" + property.mode + "\" defined for " + label);
		}
	}

	location(m_state.point(0.5));
}

// replaces what the node knows about the road
void road_node::define(state const &State)
{
	m_state = State;
	m_state.normalize();
	location(m_state.point(0.5));
	m_area.radius = -1.f;
}

// creates paths of the lanes and hands them over to the simulation
void road_node::create_lanes()
{
	m_tracks.assign(m_state.lanes.size(), nullptr);
	for (std::size_t lane = 0; lane < m_state.lanes.size(); ++lane)
	{
		m_tracks[lane] = create_track(m_name.empty() ? std::string{} : m_name + ":" + m_state.lane_id(lane), m_state.lane_path(lane), m_state.lanes[lane].width, m_state.lanes[lane].velocity);
	}
}

// links lanes of opposite directions at an end of the road which leads nowhere, so the vehicles can turn back
void road_node::close_ends()
{
	// NOTE: without the link a vehicle reaching the end would be sent back on the lane it came by, against its direction
	if (m_tracks.size() != m_state.lanes.size())
	{
		return;
	}
	road_axis const axis{m_state.axis};
	for (int end = 0; end < 2; ++end)
	{
		// the lanes going along the axis leave the road at its end, the ones going against it leave at its start
		auto const outwards{end == 1 ? axis.frame(1.0).tangent : -axis.frame(0.0).tangent};
		for (int pair = 1; pair <= std::min(m_state.forward, m_state.backward); ++pair)
		{
			auto const forward{static_cast<std::size_t>(m_state.backward + pair - 1)};
			auto const backward{static_cast<std::size_t>(m_state.backward - pair)};
			auto *exit{m_tracks[end == 1 ? forward : backward]};
			auto *entry{m_tracks[end == 1 ? backward : forward]};
			if (exit == nullptr || entry == nullptr || exit->trNext != nullptr || entry->trPrev != nullptr)
			{
				continue;
			}
			segment_data path;
			path.points[segment_data::point::start] = exit->CurrentSegment()->FastGetPoint_1();
			path.points[segment_data::point::end] = entry->CurrentSegment()->FastGetPoint_0();
			auto const span{glm::distance(path.points[segment_data::point::start], path.points[segment_data::point::end])};
			path.points[segment_data::point::control1] = outwards * std::max(turn_reach, 1.5 * span);
			path.points[segment_data::point::control2] = outwards * std::max(turn_reach, 1.5 * span);
			path.radius = static_cast<float>(std::max(2.0, 0.5 * span));
			auto *turn{create_track(m_name.empty() ? std::string{} : m_name + ":" + (end == 1 ? "endturn" : "startturn") + std::to_string(pair), path, m_state.lanes[forward].width, turn_velocity)};
			exit->ConnectNextPrev(turn, 0);
			turn->ConnectNextPrev(entry, 0);
			m_turns.emplace_back(turn);
		}
	}
}

// gives up the paths of the lanes
std::vector<TTrack *> road_node::release_lanes()
{
	std::vector<TTrack *> released;
	released.swap(m_tracks);
	released.erase(std::remove(released.begin(), released.end(), nullptr), released.end());
	return released;
}

// gives up the paths made to close the loose ends
std::vector<TTrack *> road_node::release_turns()
{
	std::vector<TTrack *> released;
	released.swap(m_turns);
	return released;
}

// true if there's a vehicle on any path of the road
bool road_node::occupied() const
{
	for (auto const *tracks : {&m_tracks, &m_turns})
	{
		for (auto const *track : *tracks)
		{
			if (track != nullptr && false == track->Dynamics.empty())
			{
				return true;
			}
		}
	}
	return false;
}

// generates geometry of the surface, the sides and the markings
std::vector<scene::shape_node> road_node::create_shapes() const
{
	// triangles are gathered separately for each look, so the lot can be drawn with as few shapes as possible
	struct batch
	{
		std::string material;
		lighting_data lighting;
		std::vector<world_vertex> vertices;
	};
	std::vector<batch> batches;
	auto const vertices = [&batches](std::string const &Material, lighting_data const &Lighting) -> std::vector<world_vertex> & {
		for (auto &entry : batches)
		{
			if (entry.material == Material && entry.lighting == Lighting)
			{
				return entry.vertices;
			}
		}
		batches.emplace_back();
		batches.back().material = Material;
		batches.back().lighting = Lighting;
		return batches.back().vertices;
	};

	auto const &road{m_state};
	road_axis const axis{road.axis};
	auto const length{axis.length()};
	auto const step{axis.step()};
	auto const halfwidth{0.5 * road.width()};
	double const texturelength{road.texturelength};

	// surface. the image is centered on the axis, and repeated if the road is wider
	if (road.surface != "none")
	{
		auto const tile{static_cast<double>(texture_ratio(road.surface)) * texturelength};
		loft(vertices(road.surface, lighting_data{}), axis, {{halfwidth, 0.0, static_cast<float>(0.5 + halfwidth / tile)}, {-halfwidth, 0.0, static_cast<float>(0.5 - halfwidth / tile)}}, 0.0, length, step, texturelength, road.taper);
	}

	// sides
	for (int side = 0; side < 2; ++side)
	{
		auto const &data{road.sides[side]};
		if (data.type == side_type::none || data.width <= 0.f || data.material.empty() || data.material == "none")
		{
			continue;
		}
		auto const direction{side == 0 ? 1.0 : -1.0};
		// corners of the profile, from the edge of the surface outwards
		std::vector<profile_point> profile;
		if (data.type == side_type::shoulder)
		{
			// the image is laid out like the one of the legacy roads: the edge of the surface on its right, the bank on its left half
			profile.push_back({direction * halfwidth, 0.0, 1.f});
			profile.push_back({direction * halfwidth, 0.0, 0.5f, direction * data.width});
			if (road.slope.x > 0.f)
			{
				profile.push_back({direction * halfwidth, -road.slope.y, 0.f, direction * (data.width + road.slope.x)});
			}
		}
		else
		{
			// the image starts at the foot of the kerb, and is laid out in its own proportions
			auto const tile{static_cast<double>(texture_ratio(data.material)) * texturelength};
			profile.push_back({direction * halfwidth, 0.0, 0.f});
			profile.push_back({direction * halfwidth, road.kerbheight, static_cast<float>(road.kerbheight / tile)});
			profile.push_back({direction * halfwidth, road.kerbheight, static_cast<float>((road.kerbheight + data.width) / tile), direction * data.width});
		}
		if (side == 0)
		{
			// the loft takes the corners left to right
			std::reverse(profile.begin(), profile.end());
		}
		loft(vertices(data.material, lighting_data{}), axis, profile, 0.0, length, step, texturelength, road.taper);
	}

	// markings. these are painted with plain colour instead of an image
	if (road.markings != marking_colour::none)
	{
		lighting_data paint;
		paint.diffuse = (road.markings == marking_colour::white ? glm::vec4{0.92f, 0.92f, 0.92f, 1.f} : glm::vec4{0.95f, 0.5f, 0.08f, 1.f});
		paint.ambient = paint.diffuse;
		auto &lines{vertices("colored", paint)};

		// Offset: where the line runs on a road as wide as its lanes say, Shift: how far from there it's kept whatever the width
		auto const solid = [&](double const Offset, double const Shift) {
			loft(lines, axis, {{Offset, line_lift, 0.f, Shift + 0.5 * line_width}, {Offset, line_lift, 1.f, Shift - 0.5 * line_width}}, 0.0, length, std::min(step, dash_step), texturelength, road.taper);
		};
		// the dashes are fitted so the gaps at both ends of the road are a half of the regular one, which makes them match the next piece
		auto const dashed = [&](double const Offset, double const Shift) {
			auto const count{std::max(1, static_cast<int>(std::lround(length / dash_period)))};
			auto const period{length / count};
			for (int dash = 0; dash < count; ++dash)
			{
				auto const from{(dash + 1.0 / 3.0) * period};
				loft(lines, axis, {{Offset, line_lift, 0.f, Shift + 0.5 * line_width}, {Offset, line_lift, 1.f, Shift - 0.5 * line_width}}, from, from + period / 3.0, dash_step, texturelength, road.taper);
			}
		};

		// edges of the surface
		auto const inset{edge_inset + 0.5 * line_width};
		if (halfwidth * std::min(road.taper[0], road.taper[1]) - inset > 0.5)
		{
			solid(halfwidth, -inset);
			solid(-halfwidth, inset);
		}
		// lines between the lanes. a lane can be left across a dashed line, or across a pair of lines if the dashed one is on its side
		auto offset{halfwidth};
		for (std::size_t boundary = 0; boundary < road.changes.size() && boundary < road.lanes.size(); ++boundary)
		{
			offset -= road.lanes[boundary].width;
			auto const opposite{road.backward > 0 && boundary + 1 == static_cast<std::size_t>(road.backward)};
			auto const pair{0.5 * (line_width + line_spacing)};
			switch (road.changes[boundary])
			{
			case change_both:
			{
				dashed(offset, 0.0);
				break;
			}
			case change_toright:
			{
				dashed(offset, pair);
				solid(offset, -pair);
				break;
			}
			case change_toleft:
			{
				solid(offset, pair);
				dashed(offset, -pair);
				break;
			}
			default:
			{
				if (opposite)
				{
					// traffic going opposite ways is kept apart with a double line
					solid(offset, pair);
					solid(offset, -pair);
				}
				else
				{
					solid(offset, 0.0);
				}
				break;
			}
			}
		}
	}

	std::vector<scene::shape_node> shapes;
	for (auto &entry : batches)
	{
		if (entry.vertices.empty())
		{
			continue;
		}
		scene::shape_node shape;
		shape.make_terrain(GfxRenderer->Fetch_Material(entry.material), std::move(entry.vertices), glm::dvec3{0.0});
		shape.lighting(entry.lighting);
		shapes.emplace_back(std::move(shape));
	}
	return shapes;
}

namespace
{

// banks holding geometry put in the scene by the owned shapes
std::unordered_set<std::uint32_t> ownedbanks;

double const junction_reach{0.25}; // an end of a road this close to an arm is attached to it
double const junction_fit{0.05}; // a lane has to end this close to where the junction expects it
int const corner_steps{6}; // pieces a rounded corner is made of

// direction on the ground as a vector in space
glm::dvec3 flat(glm::dvec2 const &Direction)
{
	return glm::dvec3{Direction.x, 0.0, Direction.y};
}

// direction to the left of the provided one, on the ground
glm::dvec2 left_of(glm::dvec2 const &Direction)
{
	return glm::dvec2{Direction.y, -Direction.x};
}

// angle of the turn from one direction to another, positive to the left
double turn_angle(glm::dvec2 const &From, glm::dvec2 const &To)
{
	return std::atan2(glm::dot(To, left_of(From)), glm::dot(To, From));
}

// slope a lane has at one of its ends, in the direction of travel
double lane_grade(TTrack const &Lane, bool const Atend)
{
	if (Lane.m_paths.empty())
	{
		return 0.0;
	}
	auto const &path{Lane.m_paths.front()};
	auto const &control{path.points[Atend ? segment_data::point::control2 : segment_data::point::control1]};
	auto const direction{control != glm::dvec3{0.0} ? (Atend ? -control : control) : path.points[segment_data::point::end] - path.points[segment_data::point::start]};
	auto const run{glm::length(glm::dvec2{direction.x, direction.z})};
	return run > 1e-6 ? direction.y / run : 0.0;
}

} // namespace

// puts provided shapes in the section of the scene holding specified point
void owned_shapes::show(std::vector<scene::shape_node> Shapes, glm::dvec3 const &Location)
{
	if (m_section != nullptr || Shapes.empty() || simulation::Region == nullptr || false == simulation::Region->point_inside(Location))
	{
		return;
	}
	auto &section{simulation::Region->section(Location)};
	// the rest of the section has to be built by now, or it'd count our shapes as waiting for it too
	section.create_geometry();
	if (m_bank.bank == 0 && m_bank.chunk == 0)
	{
		m_bank = GfxRenderer->Create_Bank();
		ownedbanks.emplace(m_bank.bank);
	}
	for (auto &shape : Shapes)
	{
		// shapes held by a section are drawn relative to its centre
		shape.origin(section.m_area.center);
		auto const centre{shape.data().area.center};
		auto const radius{shape.radius()};
		shape.create_geometry(m_bank);
		m_geometry.emplace_back(shape.data().geometry);
		section.m_shapes.emplace_back(std::move(shape));
		// the section may need to reach further to keep the shape from being culled at its edges
		section.m_area.radius = std::max(section.m_area.radius, static_cast<float>(glm::length(section.m_area.center - centre) + radius));
	}
	m_section = &section;
}

// takes the shapes back
void owned_shapes::hide()
{
	if (m_section == nullptr)
	{
		return;
	}
	// our shapes are told from the others by their geometry. the renderer reclaims the chunks once they're no longer drawn
	auto &shapes{m_section->m_shapes};
	shapes.erase(std::remove_if(shapes.begin(), shapes.end(),
	                            [this](scene::shape_node const &Shape) {
		                            auto const handle{Shape.data().geometry};
		                            return std::any_of(m_geometry.begin(), m_geometry.end(), [&handle](gfx::geometry_handle const &Own) { return Own.bank == handle.bank && Own.chunk == handle.chunk; });
	                            }),
	             shapes.end());
	m_geometry.clear();
	m_section = nullptr;
}

// true if specified geometry was put in the scene by any object of this class
bool owned_shapes::owns(gfx::geometry_handle const &Geometry)
{
	return Geometry.bank != 0 && ownedbanks.count(Geometry.bank) != 0;
}

// puts geometry of the road in the scene as shapes of its own, which can be taken back
void road_node::show()
{
	if (m_merged || m_shapes.shown())
	{
		return;
	}
	m_shapes.show(create_shapes(), location());
}

// takes back geometry put in the scene by show()
void road_node::hide()
{
	m_shapes.hide();
}

// brings the content to a usable form
void junction_node::state::normalize()
{
	for (auto &arm : arms)
	{
		arm.incoming = std::clamp(arm.incoming, 0, 8);
		arm.outgoing = std::clamp(arm.outgoing, 0, 8);
		arm.width = std::max(1.f, arm.width);
		arm.direction = (glm::length(arm.direction) > 1e-6 ? glm::normalize(arm.direction) : glm::dvec2{0.0, 1.0});
	}
	if (texturelength < 0.01f)
	{
		texturelength = 4.f;
	}
	if (surface.empty())
	{
		surface = "none";
	}
	static std::array<std::string, 7> const environments{"flat", "mountains", "mountain", "canyon", "tunnel", "bridge", "bank"};
	if (std::find(environments.begin(), environments.end(), environment) == environments.end())
	{
		environment = "flat";
	}
}

// width of the road at specified arm
double junction_node::state::arm_width(std::size_t const Arm) const
{
	return Arm < arms.size() ? static_cast<double>(arms[Arm].incoming + arms[Arm].outgoing) * arms[Arm].width : 0.0;
}

// where a lane of an arm meets the junction; the lanes of each kind are counted from the middle of the road, starting with 1
glm::dvec3 junction_node::state::lane_point(std::size_t const Arm, bool const Incoming, int const Lane) const
{
	auto const &arm{arms[Arm]};
	// seen from the junction the traffic coming in is on the left side of the road, and the one going out on the right
	auto const divide{0.5 * arm_width(Arm) - static_cast<double>(arm.incoming) * arm.width};
	auto const offset{Incoming ? divide + (Lane - 0.5) * arm.width : divide - (Lane - 0.5) * arm.width};
	return arm.position + flat(left_of(arm.direction)) * offset;
}

// arms in the order they're met going around the junction
std::vector<std::size_t> junction_node::state::arm_order() const
{
	std::vector<std::size_t> order(arms.size());
	for (std::size_t idx = 0; idx < order.size(); ++idx)
	{
		order[idx] = idx;
	}
	// the order goes from the z axis towards the x axis, which makes triangles spanned from the centre face up
	std::sort(order.begin(), order.end(), [this](std::size_t const Left, std::size_t const Right) { return std::atan2(arms[Left].direction.x, arms[Left].direction.y) < std::atan2(arms[Right].direction.x, arms[Right].direction.y); });
	return order;
}

// edge of the surface: ends of the arms joined with rounded corners, going around the junction
std::vector<glm::dvec3> junction_node::state::outline(std::vector<std::vector<glm::dvec3>> *Corners) const
{
	std::vector<glm::dvec3> points;
	auto const order{arm_order()};
	for (std::size_t idx = 0; idx < order.size(); ++idx)
	{
		auto const &arm{arms[order[idx]]};
		auto const &next{arms[order[(idx + 1) % order.size()]]};
		auto const halfwidth{0.5 * arm_width(order[idx])};
		auto const nexthalfwidth{0.5 * arm_width(order[(idx + 1) % order.size()])};
		// the end of the arm, then the corner leading to the next one
		auto const right{arm.position - flat(left_of(arm.direction)) * halfwidth};
		auto const left{arm.position + flat(left_of(arm.direction)) * halfwidth};
		auto const nextright{next.position - flat(left_of(next.direction)) * nexthalfwidth};
		points.emplace_back(right);
		points.emplace_back(left);
		// the corner is bent towards the point where the edges of both roads would meet, if there's a reasonable one
		auto control{0.5 * (left + nextright)};
		auto const cross{arm.direction.x * next.direction.y - arm.direction.y * next.direction.x};
		if (std::abs(cross) > 0.05)
		{
			// left - arm.direction * t == nextright - next.direction * s
			glm::dvec2 const delta{nextright.x - left.x, nextright.z - left.z};
			auto const t{-(delta.x * next.direction.y - delta.y * next.direction.x) / cross};
			auto const s{-(delta.x * arm.direction.y - delta.y * arm.direction.x) / cross};
			if (t > 0.0 && s > 0.0 && t < 100.0 && s < 100.0)
			{
				control = left - flat(arm.direction) * t;
				control.y = 0.5 * (left.y + nextright.y);
			}
		}
		std::vector<glm::dvec3> corner{left};
		for (int step = 1; step < corner_steps; ++step)
		{
			auto const t{static_cast<double>(step) / corner_steps};
			auto const point{(1.0 - t) * (1.0 - t) * left + 2.0 * (1.0 - t) * t * control + t * t * nextright};
			points.emplace_back(point);
			corner.emplace_back(point);
		}
		corner.emplace_back(nextright);
		if (Corners != nullptr)
		{
			Corners->emplace_back(std::move(corner));
		}
	}
	return points;
}

junction_node::junction_node(scene::node_data const &Nodedata) : basic_node(Nodedata) {}

// restores content of the node from provided input stream; reads up to and including the closing 'endjunction'
void junction_node::import(cParser &Input, glm::dvec3 const &Offset)
{
	auto const label{m_name.empty() ? std::string{"unnamed junction"} : "junction \"" + m_name + "\""};

	m_state = state{};
	Input.getTokens(3);
	Input >> m_state.centre.x >> m_state.centre.y >> m_state.centre.z;
	m_state.centre += Offset;

	auto token{Input.getToken<std::string>()};
	while (false == token.empty() && token != "endjunction")
	{
		if (token == "arm")
		{
			// arm <position> <direction x z> <lanes in> <lanes out> <lane width>
			arm_data arm;
			Input.getTokens(3);
			Input >> arm.position.x >> arm.position.y >> arm.position.z;
			arm.position += Offset;
			Input.getTokens(2);
			Input >> arm.direction.x >> arm.direction.y;
			Input.getTokens(3);
			Input >> arm.incoming >> arm.outgoing >> arm.width;
			m_state.arms.emplace_back(arm);
		}
		else if (token == "surface")
		{
			m_state.surface = Input.getToken<std::string>();
			replace_slashes(m_state.surface);
		}
		else if (token == "texlength")
		{
			Input.getTokens();
			Input >> m_state.texturelength;
		}
		else if (token == "markings")
		{
			auto const colour{Input.getToken<std::string>()};
			m_state.markings = (colour == "white" ? road_node::marking_colour::white : colour == "orange" ? road_node::marking_colour::orange : road_node::marking_colour::none);
		}
		else if (token == "velocity")
		{
			Input.getTokens();
			Input >> m_state.velocity;
		}
		else if (token == "friction")
		{
			Input.getTokens();
			Input >> m_state.friction;
		}
		else if (token == "environment")
		{
			m_state.environment = Input.getToken<std::string>();
		}
		else
		{
			ErrorLog("Bad junction: unknown property: \"" + token + "\" defined for " + label);
		}
		token = Input.getToken<std::string>();
	}
	if (m_state.arms.size() < 2 || m_state.arms.size() > 4)
	{
		ErrorLog("Bad junction: " + label + " has " + std::to_string(m_state.arms.size()) + " arms, it takes 2 to 4");
		if (m_state.arms.size() > 4)
		{
			m_state.arms.resize(4);
		}
	}
	m_state.normalize();
	location(m_state.centre);
}

void junction_node::define(state const &State)
{
	m_state = State;
	m_state.normalize();
	location(m_state.centre);
	m_area.radius = -1.f;
}

// makes paths leading through the junction, between the lanes of the roads attached to it at the moment
void junction_node::create_links()
{
	m_movements.clear();
	auto const &arms{m_state.arms};
	// lanes of the roads attached to the arms; a road is attached if one of its ends is where the arm is
	struct arm_lanes
	{
		std::vector<TTrack *> incoming;
		std::vector<TTrack *> outgoing;
	};
	std::vector<arm_lanes> lanes(arms.size());
	for (std::size_t arm = 0; arm < arms.size(); ++arm)
	{
		lanes[arm].incoming.assign(static_cast<std::size_t>(arms[arm].incoming), nullptr);
		lanes[arm].outgoing.assign(static_cast<std::size_t>(arms[arm].outgoing), nullptr);
		for (auto const *road : simulation::Roads.sequence())
		{
			if (road == nullptr || road->m_editorremoved || road->tracks().size() != road->definition().lanes.size())
			{
				continue;
			}
			auto const &layout{road->definition()};
			for (auto const atend : {false, true})
			{
				if (glm::distance(layout.axis.points[atend ? segment_data::point::end : segment_data::point::start], arms[arm].position) > junction_reach)
				{
					continue;
				}
				// the lanes heading for this end of the road lead into the junction, the others lead out of it
				for (int lane = 1; lane <= arms[arm].incoming; ++lane)
				{
					auto const index{layout.lane_index((atend ? "f" : "b") + std::to_string(lane))};
					if (index >= 0 && road->tracks()[index] != nullptr && glm::distance(glm::dvec3{road->tracks()[index]->CurrentSegment()->FastGetPoint_1()}, m_state.lane_point(arm, true, lane)) < junction_fit)
					{
						lanes[arm].incoming[lane - 1] = road->tracks()[index];
					}
				}
				for (int lane = 1; lane <= arms[arm].outgoing; ++lane)
				{
					auto const index{layout.lane_index((atend ? "b" : "f") + std::to_string(lane))};
					if (index >= 0 && road->tracks()[index] != nullptr && glm::distance(glm::dvec3{road->tracks()[index]->CurrentSegment()->FastGetPoint_0()}, m_state.lane_point(arm, false, lane)) < junction_fit)
					{
						lanes[arm].outgoing[lane - 1] = road->tracks()[index];
					}
				}
			}
		}
	}

	enum class way
	{
		left,
		straight,
		right
	};
	struct exit_data
	{
		std::size_t arm;
		double turn;
		way kind;
		TTrack *track;
	};
	std::ostringstream text;
	text.precision(std::numeric_limits<double>::digits10);
	auto const write_point = [&text](glm::dvec3 const &Point) { text << Point.x << ' ' << Point.y << ' ' << Point.z << ' '; };

	for (std::size_t arm = 0; arm < arms.size(); ++arm)
	{
		auto const heading{-arms[arm].direction};
		// the other arms with a road to leave by, from the leftmost to the rightmost
		std::vector<exit_data> exits;
		for (std::size_t other = 0; other < arms.size(); ++other)
		{
			if (other == arm || std::all_of(lanes[other].outgoing.begin(), lanes[other].outgoing.end(), [](TTrack const *Track) { return Track == nullptr; }))
			{
				continue;
			}
			exits.push_back({other, turn_angle(heading, arms[other].direction), way::straight, nullptr});
		}
		std::sort(exits.begin(), exits.end(), [](exit_data const &Left, exit_data const &Right) { return Left.turn > Right.turn; });
		if (exits.size() == 3)
		{
			exits[0].kind = way::left;
			exits[2].kind = way::right;
		}
		else
		{
			auto const sideways{glm::radians(50.0)};
			for (auto &exit : exits)
			{
				exit.kind = (exit.turn > sideways ? way::left : exit.turn < -sideways ? way::right : way::straight);
			}
			if (exits.size() == 2 && exits[0].kind == exits[1].kind)
			{
				// two ways of the same kind are told apart by which one is more to the left
				if (exits[0].kind == way::right)
				{
					exits[0].kind = way::straight;
				}
				else if (exits[0].kind == way::left)
				{
					exits[1].kind = way::straight;
				}
				else
				{
					exits[0].kind = way::left;
					exits[1].kind = way::right;
				}
			}
		}
		auto const lanecount{static_cast<int>(lanes[arm].incoming.size())};
		for (int lane = 1; lane <= lanecount; ++lane)
		{
			auto *entry{lanes[arm].incoming[lane - 1]};
			if (entry == nullptr)
			{
				continue;
			}
			// with more lanes than one the inner lane takes the left turns, the outer one the right turns, and all go straight
			std::vector<exit_data> ways;
			for (auto const &exit : exits)
			{
				auto const allowed{lanecount == 1 || exit.kind == way::straight || (exit.kind == way::left && lane == 1) || (exit.kind == way::right && lane == lanecount)};
				if (allowed)
				{
					ways.emplace_back(exit);
				}
			}
			if (ways.empty())
			{
				ways = exits;
			}
			for (auto &exit : ways)
			{
				// the lane of the same number on the road to leave by, or the nearest one there is
				auto const &candidates{lanes[exit.arm].outgoing};
				for (int shift = 0; shift < static_cast<int>(candidates.size()) && exit.track == nullptr; ++shift)
				{
					for (auto const index : {lane - 1 - shift, lane - 1 + shift})
					{
						if (index >= 0 && index < static_cast<int>(candidates.size()) && candidates[index] != nullptr)
						{
							exit.track = candidates[index];
							break;
						}
					}
				}
				if (exit.track == nullptr)
				{
					exit.track = candidates.back();
				}
			}
			ways.erase(std::remove_if(ways.begin(), ways.end(), [](exit_data const &Exit) { return Exit.track == nullptr; }), ways.end());
			if (ways.empty())
			{
				continue;
			}
			if (ways.size() == 3)
			{
				// the legacy crossroads has the way straight ahead at its second point, the left one at the third and the right one at the fourth
				std::swap(ways[0], ways[1]);
			}
			if (exits.size() == 1 && lane == lanecount)
			{
				// where the only road to leave by has more lanes than the one coming in, the road gets wider: the lanes it gains
				// are entered from the outermost lane. a path has up to three ways out, so that's two lanes gained at most
				auto const &candidates{lanes[exits[0].arm].outgoing};
				for (auto extra = static_cast<std::size_t>(lanecount); extra < candidates.size() && ways.size() < 3; ++extra)
				{
					if (candidates[extra] != nullptr && candidates[extra] != ways.front().track)
					{
						auto way{ways.front()};
						way.track = candidates[extra];
						ways.emplace_back(way);
					}
				}
			}

			// shape of the ways: each leaves the lane coming in the way that lane goes, and joins the lane going out the way that one goes
			glm::dvec3 const start{entry->CurrentSegment()->FastGetPoint_1()};
			std::vector<glm::dvec3> ends;
			std::vector<glm::dvec3> endcontrols;
			double reach{0.0};
			for (auto const &exit : ways)
			{
				glm::dvec3 const end{exit.track->CurrentSegment()->FastGetPoint_0()};
				auto const span{glm::distance(start, end)};
				ends.emplace_back(end);
				// the slopes of the lanes are carried on, so a junction on a hill doesn't make a step
				endcontrols.emplace_back(-(flat(arms[exit.arm].direction) + up * lane_grade(*exit.track, false)) * (0.39 * span));
				reach += 0.39 * span / ways.size();
			}
			auto const startcontrol{(flat(heading) + up * lane_grade(*entry, true)) * reach};
			for (std::size_t idx = 0; idx < ways.size(); ++idx)
			{
				segment_data movement;
				movement.points[segment_data::point::start] = start;
				movement.points[segment_data::point::control1] = startcontrol;
				movement.points[segment_data::point::control2] = endcontrols[idx];
				movement.points[segment_data::point::end] = ends[idx];
				m_movements.emplace_back(movement);
			}

			text.str("");
			text << (ways.size() == 1 ? "road " : "cross ") << glm::distance(start, ends[0]) << ' ' << arms[arm].width << ' ' << m_state.friction << ' ' << m_state.sounddistance << ' ' << m_state.quality << " 0 " << m_state.environment << " unvis ";
			// first path: from the lane coming in to the first way out
			write_point(start);
			text << "0 ";
			write_point(startcontrol);
			write_point(endcontrols[0]);
			write_point(ends[0]);
			text << "0 0 ";
			if (ways.size() == 2)
			{
				// second path of a crossroads of three roads starts where the first one does
				write_point(start);
				text << "0 ";
				write_point(startcontrol);
				write_point(endcontrols[1]);
				write_point(ends[1]);
				text << "0 0 ";
			}
			else if (ways.size() == 3)
			{
				// second path of a crossroads of four roads runs between its third and fourth point
				write_point(ends[1]);
				text << "0 ";
				write_point(endcontrols[1]);
				write_point(endcontrols[2]);
				write_point(ends[2]);
				text << "0 0 ";
			}
			if (m_state.velocity > 0.f)
			{
				text << "velocity " << m_state.velocity << ' ';
			}
			text << "endtrack";
			auto *link{create_track(text.str(), m_links.size())};

			// the path is tied to the lanes by hand: the lanes going out are shared by the paths from all the other arms,
			// which is more than the automatic joining of path ends can express
			if (ways.size() == 1)
			{
				entry->ConnectNextPrev(link, 0);
				link->trNext = ways[0].track;
				link->iNextDirection = 0;
				if (ways[0].track->trPrev == nullptr)
				{
					ways[0].track->trPrev = link;
					ways[0].track->iPrevDirection = 1;
				}
			}
			else if (link->SwitchExtension != nullptr)
			{
				auto &extension{*link->SwitchExtension};
				// the points of the crossroads are stored as: first, second, fourth, third
				extension.pPrevs[0] = entry;
				extension.iPrevDirection[0] = 1;
				extension.pNexts[0] = ways[0].track;
				extension.iNextDirection[0] = 0;
				extension.pPrevs[1] = (ways.size() == 3 ? ways[2].track : ways[1].track);
				extension.iPrevDirection[1] = 0;
				extension.pNexts[1] = (ways.size() == 3 ? ways[1].track : nullptr);
				extension.iNextDirection[1] = 0;
				link->Switch(0);
				entry->trNext = link;
				entry->iNextDirection = 0;
				// the lanes going out get to know one of the paths leading to them, for whatever goes backwards
				int const ends2[]{1, 2};
				int const ends3[]{1, 3, 2};
				for (std::size_t idx = 0; idx < ways.size(); ++idx)
				{
					if (ways[idx].track->trPrev == nullptr)
					{
						ways[idx].track->trPrev = link;
						ways[idx].track->iPrevDirection = (ways.size() == 3 ? ends3[idx] : ends2[idx]);
					}
				}
			}
			m_links.emplace_back(link);
		}
	}
}

// gives up the paths leading through the junction
std::vector<TTrack *> junction_node::release_links()
{
	std::vector<TTrack *> released;
	released.swap(m_links);
	m_movements.clear();
	return released;
}

// true if there's a vehicle on any path of the junction
bool junction_node::occupied() const
{
	return std::any_of(m_links.begin(), m_links.end(), [](TTrack const *Track) { return Track != nullptr && false == Track->Dynamics.empty(); });
}

// generates geometry of the surface
std::vector<scene::shape_node> junction_node::create_shapes() const
{
	std::vector<scene::shape_node> shapes;
	std::vector<std::vector<glm::dvec3>> corners;
	auto const points{m_state.outline(&corners)};
	if (points.size() < 3)
	{
		return shapes;
	}
	auto const &centre{m_state.centre};
	if (m_state.surface != "none")
	{
		// a fan of triangles spanned from the centre, with the image laid out flat over the ground
		auto const tile{static_cast<double>(texture_ratio(m_state.surface)) * m_state.texturelength};
		auto const vertex = [&](glm::dvec3 const &Point, glm::vec3 const &Normal) {
			return world_vertex{Point, Normal, {static_cast<float>(0.5 + (Point.x - centre.x) / tile), static_cast<float>(0.5 + (Point.z - centre.z) / m_state.texturelength)}};
		};
		std::vector<world_vertex> vertices;
		for (std::size_t idx = 0; idx < points.size(); ++idx)
		{
			auto const &current{points[idx]};
			auto const &next{points[(idx + 1) % points.size()]};
			auto normal{glm::cross(current - centre, next - centre)};
			if (glm::length2(normal) < 1e-12)
			{
				continue;
			}
			glm::vec3 const facing{glm::normalize(normal)};
			vertices.emplace_back(vertex(centre, facing));
			vertices.emplace_back(vertex(current, facing));
			vertices.emplace_back(vertex(next, facing));
		}
		if (false == vertices.empty())
		{
			scene::shape_node shape;
			shape.make_terrain(GfxRenderer->Fetch_Material(m_state.surface), std::move(vertices), glm::dvec3{0.0});
			shapes.emplace_back(std::move(shape));
		}
	}
	if (m_state.markings != road_node::marking_colour::none)
	{
		// edge lines carried around the corners
		std::vector<world_vertex> vertices;
		glm::dvec3 const lift{0.0, line_lift, 0.0};
		glm::vec3 const upwards{0.f, 1.f, 0.f};
		auto const &arms{m_state.arms};
		if (arms.size() == 2 && arms[0].incoming > 0 && arms[0].outgoing > 0 && arms[1].incoming > 0 && arms[1].outgoing > 0)
		{
			// a junction of two roads is where a road changes its lanes. the double line keeping the directions apart is led through it
			auto const divide = [&](std::size_t const Arm) { return arms[Arm].position + flat(left_of(arms[Arm].direction)) * (0.5 * m_state.arm_width(Arm) - static_cast<double>(arms[Arm].incoming) * arms[Arm].width); };
			auto const p0{divide(0)};
			auto const p3{divide(1)};
			auto const reach{0.39 * glm::distance(p0, p3)};
			auto const p1{p0 - flat(arms[0].direction) * reach};
			auto const p2{p3 - flat(arms[1].direction) * reach};
			auto const pair{0.5 * (line_width + line_spacing)};
			int const steps{12};
			glm::dvec3 previous{p0};
			glm::dvec3 previousleft{flat(left_of(-arms[0].direction))};
			for (int step = 1; step <= steps; ++step)
			{
				auto const t{static_cast<double>(step) / steps};
				auto const u{1.0 - t};
				auto const current{u * u * u * p0 + 3.0 * u * u * t * p1 + 3.0 * u * t * t * p2 + t * t * t * p3};
				auto const direction{3.0 * u * u * (p1 - p0) + 6.0 * u * t * (p2 - p1) + 3.0 * t * t * (p3 - p2)};
				glm::dvec2 const heading{direction.x, direction.z};
				auto const left{glm::length(heading) > 1e-9 ? flat(left_of(glm::normalize(heading))) : previousleft};
				for (auto const side : {-1.0, 1.0})
				{
					auto const leftstart{previous + previousleft * (side * pair + 0.5 * line_width) + lift};
					auto const rightstart{previous + previousleft * (side * pair - 0.5 * line_width) + lift};
					auto const leftend{current + left * (side * pair + 0.5 * line_width) + lift};
					auto const rightend{current + left * (side * pair - 0.5 * line_width) + lift};
					vertices.push_back({leftstart, upwards, {0.f, 0.f}});
					vertices.push_back({rightstart, upwards, {1.f, 0.f}});
					vertices.push_back({leftend, upwards, {0.f, 1.f}});
					vertices.push_back({rightstart, upwards, {1.f, 0.f}});
					vertices.push_back({rightend, upwards, {1.f, 1.f}});
					vertices.push_back({leftend, upwards, {0.f, 1.f}});
				}
				previous = current;
				previousleft = left;
			}
		}
		for (auto const &corner : corners)
		{
			for (std::size_t idx = 0; idx + 1 < corner.size(); ++idx)
			{
				// the line keeps its distance from the edge, on the side of the centre
				auto const inwards = [&](glm::dvec3 const &Point) {
					auto direction{centre - Point};
					direction.y = 0.0;
					return glm::length2(direction) > 1e-12 ? glm::normalize(direction) : glm::dvec3{0.0};
				};
				auto const outer0{corner[idx] + inwards(corner[idx]) * edge_inset + lift};
				auto const inner0{corner[idx] + inwards(corner[idx]) * (edge_inset + line_width) + lift};
				auto const outer1{corner[idx + 1] + inwards(corner[idx + 1]) * edge_inset + lift};
				auto const inner1{corner[idx + 1] + inwards(corner[idx + 1]) * (edge_inset + line_width) + lift};
				// the corners are listed going around the junction the way which makes these face up
				vertices.push_back({inner0, upwards, {0.f, 0.f}});
				vertices.push_back({outer0, upwards, {1.f, 0.f}});
				vertices.push_back({outer1, upwards, {1.f, 1.f}});
				vertices.push_back({inner0, upwards, {0.f, 0.f}});
				vertices.push_back({outer1, upwards, {1.f, 1.f}});
				vertices.push_back({inner1, upwards, {0.f, 1.f}});
			}
		}
		if (false == vertices.empty())
		{
			lighting_data paint;
			paint.diffuse = (m_state.markings == road_node::marking_colour::white ? glm::vec4{0.92f, 0.92f, 0.92f, 1.f} : glm::vec4{0.95f, 0.5f, 0.08f, 1.f});
			paint.ambient = paint.diffuse;
			scene::shape_node shape;
			shape.make_terrain(GfxRenderer->Fetch_Material("colored"), std::move(vertices), glm::dvec3{0.0});
			shape.lighting(paint);
			shapes.emplace_back(std::move(shape));
		}
	}
	return shapes;
}

void junction_node::show()
{
	if (m_merged || m_shapes.shown())
	{
		return;
	}
	m_shapes.show(create_shapes(), location());
}

void junction_node::hide()
{
	m_shapes.hide();
}

// creates a path through the junction from provided definition and registers it with the simulation
TTrack *junction_node::create_track(std::string const &Definition, std::size_t const Index)
{
	cParser parser(Definition, cParser::buffer_TEXT);
	scene::node_data nodedata;
	nodedata.range_max = -1.0;
	nodedata.name = (m_name.empty() ? std::string{} : m_name + ":way" + std::to_string(Index + 1));
	nodedata.type = "track";
	auto *track{new TTrack(nodedata)};
	track->Load(&parser, glm::dvec3{0.0});
	track->m_road = this;
	// NOTE: the table points the name at the newest path, which is what a junction made anew in the editor needs
	simulation::Paths.insert(track);
	simulation::Region->insert_and_register(track);
	return track;
}

float junction_node::radius_()
{
	float radius{0.f};
	for (std::size_t arm = 0; arm < m_state.arms.size(); ++arm)
	{
		radius = std::max(radius, static_cast<float>(glm::distance(m_state.arms[arm].position, m_state.centre) + 0.5 * m_state.arm_width(arm)));
	}
	return radius;
}

void junction_node::serialize_(std::ostream &Output) const
{
	// TODO: implement
}

void junction_node::deserialize_(std::istream &Input)
{
	// TODO: implement
}

// export() subclass details, sends basic content of the class in legacy (text) format to provided stream
void junction_node::export_as_text_(std::ostream &Output) const
{
	Output << "junction ";
	auto const precision{Output.precision(std::numeric_limits<double>::digits10)};
	Output << m_state.centre.x << ' ' << m_state.centre.y << ' ' << m_state.centre.z << ' ';
	for (auto const &arm : m_state.arms)
	{
		Output << "arm " << arm.position.x << ' ' << arm.position.y << ' ' << arm.position.z << ' ' << arm.direction.x << ' ' << arm.direction.y << ' ' << arm.incoming << ' ' << arm.outgoing << ' ' << arm.width << ' ';
	}
	Output.precision(precision);
	Output << "surface " << m_state.surface << ' ' << "texlength " << m_state.texturelength << ' ' << "markings "
	       << (m_state.markings == road_node::marking_colour::white ? "white" : m_state.markings == road_node::marking_colour::orange ? "orange" : "none") << ' ' << "velocity " << m_state.velocity << ' ' << "friction " << m_state.friction << ' '
	       << "environment " << m_state.environment << ' ';
	Output << "endjunction"
	       << "\n";
}

// creates a path for the vehicles and registers it with the simulation
TTrack *road_node::create_track(std::string const &Name, segment_data const &Path, float const Width, float const Velocity)
{
	auto const &start{Path.points[segment_data::point::start]};
	auto const &control1{Path.points[segment_data::point::control1]};
	auto const &control2{Path.points[segment_data::point::control2]};
	auto const &end{Path.points[segment_data::point::end]};
	// the path is put together the way the scenery defines an invisible road, to have it set up by the code which loads these
	std::ostringstream text;
	text.precision(std::numeric_limits<double>::digits10);
	text << "road " << glm::distance(start, end) << ' ' << Width << ' ' << m_state.friction << ' ' << m_state.sounddistance << ' ' << m_state.quality << " 0 " << m_state.environment << " unvis " << start.x << ' ' << start.y << ' ' << start.z << ' ' << Path.rolls[0] << ' '
	     << control1.x << ' ' << control1.y << ' ' << control1.z << ' ' << control2.x << ' ' << control2.y << ' ' << control2.z << ' ' << end.x << ' ' << end.y << ' ' << end.z << ' ' << Path.rolls[1] << ' ' << Path.radius << ' ';
	if (Velocity > 0.f)
	{
		text << "velocity " << Velocity << ' ';
	}
	text << "endtrack";

	cParser parser(text.str(), cParser::buffer_TEXT);
	scene::node_data nodedata;
	nodedata.range_max = -1.0;
	nodedata.name = Name;
	nodedata.type = "track";
	auto *track{new TTrack(nodedata)};
	track->Load(&parser, glm::dvec3{0.0});
	track->m_road = this;
	if (false == simulation::Paths.insert(track))
	{
		// NOTE: the table points the name at the newest path, which is what a road made anew in the editor needs
		if (false == m_editorremoved && false == dirty())
		{
			ErrorLog("Bad scenario: duplicate track name \"" + track->name() + "\" generated for a road");
		}
	}
	simulation::Region->insert_and_register(track);
	return track;
}

// radius() subclass details, calculates node's bounding radius
float road_node::radius_()
{
	return static_cast<float>(0.5 * m_state.length() + 0.5 * m_state.width() * std::max(m_state.taper[0], m_state.taper[1]) + std::max(m_state.sides[0].width, m_state.sides[1].width) + m_state.slope.x);
}

// serialize() subclass details, sends content of the subclass to provided stream
void road_node::serialize_(std::ostream &Output) const
{
	// TODO: implement
}

// deserialize() subclass details, restores content of the subclass from provided stream
void road_node::deserialize_(std::istream &Input)
{
	// TODO: implement
}

// export() subclass details, sends basic content of the class in legacy (text) format to provided stream
void road_node::export_as_text_(std::ostream &Output) const
{
	auto const &road{m_state};
	Output << "road ";
	// axis
	auto const precision{Output.precision(std::numeric_limits<double>::digits10)};
	Output << road.axis.points[segment_data::point::start].x << ' ' << road.axis.points[segment_data::point::start].y << ' ' << road.axis.points[segment_data::point::start].z << ' ' << road.axis.rolls[0] << ' '
	       << road.axis.points[segment_data::point::control1].x << ' ' << road.axis.points[segment_data::point::control1].y << ' ' << road.axis.points[segment_data::point::control1].z << ' '
	       << road.axis.points[segment_data::point::control2].x << ' ' << road.axis.points[segment_data::point::control2].y << ' ' << road.axis.points[segment_data::point::control2].z << ' '
	       << road.axis.points[segment_data::point::end].x << ' ' << road.axis.points[segment_data::point::end].y << ' ' << road.axis.points[segment_data::point::end].z << ' ' << road.axis.rolls[1] << ' '
	       << road.axis.radius << ' ';
	Output.precision(precision);
	// lanes
	Output << "lanes " << road.forward << ' ' << road.backward << ' ' << "width " << road.lanewidth << ' ';
	if (road.taper[0] != 1.f || road.taper[1] != 1.f)
	{
		Output << "taper " << road.taper[0] << ' ' << road.taper[1] << ' ';
	}
	if (road.velocity > 0.f)
	{
		Output << "velocity " << road.velocity << ' ';
	}
	for (std::size_t lane = 0; lane < road.lanes.size(); ++lane)
	{
		if (road.lanes[lane].width != road.lanewidth)
		{
			Output << "lane " << road.lane_id(lane) << " width " << road.lanes[lane].width << ' ';
		}
		if (road.lanes[lane].velocity != road.velocity)
		{
			Output << "lane " << road.lane_id(lane) << " velocity " << road.lanes[lane].velocity << ' ';
		}
	}
	for (std::size_t boundary = 0; boundary < road.changes.size(); ++boundary)
	{
		if (road.changes[boundary] == road.default_change(boundary))
		{
			continue;
		}
		Output << "change " << road.lane_id(boundary) << ' ' << road.lane_id(boundary + 1) << ' ' << (road.changes[boundary] == change_both ? "both" : road.changes[boundary] == change_toright ? "ab" : road.changes[boundary] == change_toleft ? "ba" : "none") << ' ';
	}
	// appearance
	Output << "surface " << road.surface << ' ' << "texlength " << road.texturelength << ' ';
	char const *sidenames[]{"left", "right"};
	for (int side = 0; side < 2; ++side)
	{
		auto const &data{road.sides[side]};
		if (data.type == side_type::none)
		{
			continue;
		}
		Output << "side " << sidenames[side] << ' ' << (data.type == side_type::shoulder ? "shoulder" : "sidewalk") << ' ' << data.width << ' ' << (data.material.empty() ? "none" : data.material) << ' ';
	}
	Output << "kerb " << road.kerbheight << ' ' << "slope " << road.slope.x << ' ' << road.slope.y << ' ' << "markings " << (road.markings == marking_colour::white ? "white" : road.markings == marking_colour::orange ? "orange" : "none") << ' ';
	// properties passed on to the lanes
	Output << "friction " << road.friction << ' ' << "environment " << road.environment << ' ';
	// footer
	Output << "endroad"
	       << "\n";
}

// legacy style initialization, to be performed when the tracks are already joined
void road_table::InitRoads()
{
	for (auto *road : m_items)
	{
		if (road != nullptr)
		{
			road->close_ends();
		}
	}
}

// generates geometry of the roads and puts it in the scene
void road_table::create_geometry(scene::scratch_data &Scratchpad)
{
	if (simulation::Region == nullptr)
	{
		return;
	}
	for (auto *road : m_items)
	{
		if (road == nullptr)
		{
			continue;
		}
		if (Global.editor_session && false == Global.NvRenderer)
		{
			// scenery opened for editing: the road keeps its geometry apart, so it can be changed later.
			// NOTE: the experimental renderer draws only what the scenery had when it was loaded
			road->show();
			continue;
		}
		// otherwise the geometry joins the rest of the scenery, which lets it be drawn along with it at no extra cost
		for (auto &shape : road->create_shapes())
		{
			// it's already where it belongs, so the placement set by the scenery isn't to be applied to it
			simulation::Region->insert(shape, Scratchpad, false);
		}
		road->merged(true);
	}
}

// legacy style initialization, to be performed when the tracks are joined, ahead of the roads
void junction_table::InitJunctions()
{
	for (auto *junction : m_items)
	{
		if (junction != nullptr)
		{
			junction->create_links();
		}
	}
}

// generates geometry of the junctions and puts it in the scene
void junction_table::create_geometry(scene::scratch_data &Scratchpad)
{
	if (simulation::Region == nullptr)
	{
		return;
	}
	for (auto *junction : m_items)
	{
		if (junction == nullptr)
		{
			continue;
		}
		if (Global.editor_session && false == Global.NvRenderer)
		{
			junction->show();
			continue;
		}
		for (auto &shape : junction->create_shapes())
		{
			simulation::Region->insert(shape, Scratchpad, false);
		}
		junction->merged(true);
	}
}

//---------------------------------------------------------------------------

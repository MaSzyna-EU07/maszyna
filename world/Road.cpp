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
	double offset; // distance from the axis, positive to the left
	double height; // above the surface
	float texture; // texture coordinate across the road
};

// stretches provided profile along a part of the axis. the profile points go left to right, each two neighbours make a band of triangles.
void loft(std::vector<world_vertex> &Output, road_axis const &Axis, std::vector<profile_point> const &Profile, double const From, double const To, double const Step, double const Texturelength)
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
		for (std::size_t idx = 0; idx < pointcount; ++idx)
		{
			current[idx] = frame.position + frame.left * Profile[idx].offset + up * Profile[idx].height;
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

// combined width of the lanes
double road_node::state::width() const
{
	double total{0.0};
	for (auto const &lane : lanes)
	{
		total += lane.width;
	}
	return total;
}

// shape of the middle of a lane, laid out in the direction of travel
segment_data road_node::state::lane_path(std::size_t const Lane) const
{
	road_axis const centre{axis};
	auto const offset{lane_offset(Lane)};
	auto const begin{centre.frame(0.0)};
	auto const end{centre.frame(1.0)};

	segment_data path;
	path.points[segment_data::point::start] = begin.position + begin.left * offset;
	path.points[segment_data::point::end] = end.position + end.left * offset;
	if (centre.curved())
	{
		// a line running beside a bend is shorter or longer than the bend itself, and so are its control vectors
		path.points[segment_data::point::control1] = centre.control(0) * std::max(0.05, 1.0 - offset * std::cos(begin.roll) * centre.curvature(0.0));
		path.points[segment_data::point::control2] = centre.control(1) * std::max(0.05, 1.0 - offset * std::cos(end.roll) * centre.curvature(1.0));
		// if the roll changes along the road its sides rise or fall relative to the axis
		auto const climb{offset * (std::sin(end.roll) - std::sin(begin.roll)) / 3.0};
		path.points[segment_data::point::control1].y += climb;
		path.points[segment_data::point::control2].y -= climb;
	}
	path.rolls = axis.rolls;
	if (axis.radius != 0.f)
	{
		// the lanes on the inner side of the bend have it tighter
		auto const turn{centre.curvature(0.5) >= 0.0 ? 1.0 : -1.0};
		path.radius = static_cast<float>(std::max(1.0, std::abs(axis.radius) - turn * offset));
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
		loft(vertices(road.surface, lighting_data{}), axis, {{halfwidth, 0.0, static_cast<float>(0.5 + halfwidth / tile)}, {-halfwidth, 0.0, static_cast<float>(0.5 - halfwidth / tile)}}, 0.0, length, step, texturelength);
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
			profile.push_back({direction * (halfwidth + data.width), 0.0, 0.5f});
			if (road.slope.x > 0.f)
			{
				profile.push_back({direction * (halfwidth + data.width + road.slope.x), -road.slope.y, 0.f});
			}
		}
		else
		{
			// the image starts at the foot of the kerb, and is laid out in its own proportions
			auto const tile{static_cast<double>(texture_ratio(data.material)) * texturelength};
			profile.push_back({direction * halfwidth, 0.0, 0.f});
			profile.push_back({direction * halfwidth, road.kerbheight, static_cast<float>(road.kerbheight / tile)});
			profile.push_back({direction * (halfwidth + data.width), road.kerbheight, static_cast<float>((road.kerbheight + data.width) / tile)});
		}
		if (side == 0)
		{
			// the loft takes the corners left to right
			std::reverse(profile.begin(), profile.end());
		}
		loft(vertices(data.material, lighting_data{}), axis, profile, 0.0, length, step, texturelength);
	}

	// markings. these are painted with plain colour instead of an image
	if (road.markings != marking_colour::none)
	{
		lighting_data paint;
		paint.diffuse = (road.markings == marking_colour::white ? glm::vec4{0.92f, 0.92f, 0.92f, 1.f} : glm::vec4{0.95f, 0.5f, 0.08f, 1.f});
		paint.ambient = paint.diffuse;
		auto &lines{vertices("colored", paint)};

		auto const solid = [&](double const Offset) {
			loft(lines, axis, {{Offset + 0.5 * line_width, line_lift, 0.f}, {Offset - 0.5 * line_width, line_lift, 1.f}}, 0.0, length, std::min(step, dash_step), texturelength);
		};
		// the dashes are fitted so the gaps at both ends of the road are a half of the regular one, which makes them match the next piece
		auto const dashed = [&](double const Offset) {
			auto const count{std::max(1, static_cast<int>(std::lround(length / dash_period)))};
			auto const period{length / count};
			for (int dash = 0; dash < count; ++dash)
			{
				auto const from{(dash + 1.0 / 3.0) * period};
				loft(lines, axis, {{Offset + 0.5 * line_width, line_lift, 0.f}, {Offset - 0.5 * line_width, line_lift, 1.f}}, from, from + period / 3.0, dash_step, texturelength);
			}
		};

		// edges of the surface
		auto const edge{halfwidth - edge_inset - 0.5 * line_width};
		if (edge > 0.5)
		{
			solid(edge);
			solid(-edge);
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
				dashed(offset);
				break;
			}
			case change_toright:
			{
				dashed(offset + pair);
				solid(offset - pair);
				break;
			}
			case change_toleft:
			{
				solid(offset + pair);
				dashed(offset - pair);
				break;
			}
			default:
			{
				if (opposite)
				{
					// traffic going opposite ways is kept apart with a double line
					solid(offset + pair);
					solid(offset - pair);
				}
				else
				{
					solid(offset);
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

// puts geometry of the road in the scene as shapes of its own, which can be taken back
void road_node::show()
{
	if (m_section != nullptr || m_merged || simulation::Region == nullptr || false == simulation::Region->point_inside(location()))
	{
		return;
	}
	auto shapes{create_shapes()};
	if (shapes.empty())
	{
		return;
	}
	auto &section{simulation::Region->section(location())};
	// the rest of the section has to be built by now, or it'd count our shapes as waiting for it too
	section.create_geometry();
	if (m_bank.bank == 0 && m_bank.chunk == 0)
	{
		m_bank = GfxRenderer->Create_Bank();
	}
	for (auto &shape : shapes)
	{
		// shapes held by a section are drawn relative to its centre
		shape.origin(section.m_area.center);
		auto const centre{shape.data().area.center};
		auto const radius{shape.radius()};
		shape.create_geometry(m_bank);
		m_geometry.emplace_back(shape.data().geometry);
		section.m_shapes.emplace_back(std::move(shape));
		// the section may need to reach further to keep the road from being culled at its edges
		section.m_area.radius = std::max(section.m_area.radius, static_cast<float>(glm::length(section.m_area.center - centre) + radius));
	}
	m_section = &section;
}

// takes back geometry put in the scene by show()
void road_node::hide()
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
	return static_cast<float>(0.5 * m_state.length() + 0.5 * m_state.width() + std::max(m_state.sides[0].width, m_state.sides[1].width) + m_state.slope.x);
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

//---------------------------------------------------------------------------

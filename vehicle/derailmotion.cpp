/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/derailmotion.h"

#include "McZapkie/derailment.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace derail_motion {

namespace {

glm::dvec3 const world_up{0.0, 1.0, 0.0};

// the vector level, of unit length; the fallback for one standing up
glm::dvec3 level(glm::dvec3 const &Vector, glm::dvec3 const &Fallback = {0.0, 0.0, 1.0}) {
	glm::dvec3 const flat{Vector.x, 0.0, Vector.z};
	auto const length{glm::length(flat)};
	return length > 1e-9 ? flat / length : Fallback;
}

// to the left of a level heading, as TDynamicObject has it: up x front
glm::dvec3 left_of(glm::dvec3 const &Front) {
	return glm::dvec3{Front.z, 0.0, -Front.x};
}

// the vector turned about the axis by the angle, right-handed
glm::dvec3 turned(glm::dvec3 const &Vector, glm::dvec3 const &Axis, double const Angle) {
	auto const c{std::cos(Angle)};
	auto const s{std::sin(Angle)};
	return Vector * c + glm::cross(Axis, Vector) * s + Axis * (glm::dot(Axis, Vector) * (1.0 - c));
}

double sign_of(double const Value) {
	if (Value > 0.0) {
		return 1.0;
	}
	return Value < 0.0 ? -1.0 : 0.0;
}

// the height of the triangle over the point, if it is over it (in the plan)
std::optional<double> height_in(mesh_ground::triangle const &Triangle, double const X, double const Z) {
	auto const &a{Triangle[0]};
	auto const &b{Triangle[1]};
	auto const &c{Triangle[2]};
	auto const determinant{(b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z)};
	if (std::abs(determinant) < 1e-12) {
		// standing on its edge, a wall
		return std::nullopt;
	}
	auto const u{((b.z - c.z) * (X - c.x) + (c.x - b.x) * (Z - c.z)) / determinant};
	auto const v{((c.z - a.z) * (X - c.x) + (a.x - c.x) * (Z - c.z)) / determinant};
	auto const w{1.0 - u - v};
	constexpr double edge{-1e-9};
	if (u < edge || v < edge || w < edge) {
		return std::nullopt;
	}
	return u * a.y + v * b.y + w * c.y;
}

} // namespace

namespace {

// the angle [rad] from the first level direction to the second, to the left positive
double angle_between(glm::dvec3 const &From, glm::dvec3 const &To) {
	return std::atan2(glm::dot(To, left_of(From)), glm::dot(To, From));
}

// the beams of two ends further apart across than this have gone past each other [m]
constexpr double beam_reach{2.0};
// nor do they meet pressed into each other deeper than this: the buffers and the ends behind them crushed [m]
constexpr double beam_depth{1.5};
// the buffers of two ends this far apart along the line between them are apart, the coupling straight between [m]
constexpr double buffer_reach{0.1};

} // namespace

double sliding_friction(double const Tipping, bool const Guided) {
	if (Guided) {
		return guided_friction;
	}
	auto const share{std::clamp(std::abs(Tipping) / body::half_turn, 0.0, 1.0)};
	return std::lerp(derailment::off_rails_friction, derailment::overturned_friction, share);
}

double departure_angle(cause const Cause, double const Speed, double const Acceleration) {
	if (Cause == cause::end_of_track || Cause == cause::collision) {
		return 0.0;
	}
	auto const sideways{std::sqrt(2.0 * std::abs(Acceleration) * climb_width)};
	return std::min(most_departure, std::atan2(sideways, std::max(std::abs(Speed), 1.0)));
}

scatter draw_scatter(std::uint32_t const Seed) {
	// the raw numbers of the engine are the same with every library, the distributions of <random> aren't
	std::mt19937 engine{Seed};
	auto const uniform = [&engine](double const Low, double const High) { return Low + (High - Low) * ((static_cast<double>(engine()) + 0.5) / 4294967296.0); };
	scatter result;
	result.departure = uniform(-0.01, 0.01);
	result.yaw = uniform(-0.02, 0.02);
	result.yaw_rate = uniform(-0.02, 0.02);
	result.friction_front = uniform(0.85, 1.15);
	result.friction_rear = uniform(0.85, 1.15);
	auto const bites{uniform(0.0, 1.0) < 2.0 / 3.0};
	auto const when{uniform(0.2, 1.5)};
	auto const howlong{uniform(0.3, 0.8)};
	auto const how{uniform(2.0, 4.0)};
	auto const side{uniform(0.0, 1.0) < 0.5 ? -1 : 1};
	if (bites) {
		result.dig_in = when;
		result.dig_in_time = howlong;
		result.dig_in_friction = how;
		result.dig_in_side = side;
	}
	return result;
}

std::uint32_t seed_of(std::uint32_t const Run, std::string_view const Name) {
	// FNV-1a of the name, mixed with the seed of the run
	std::uint32_t hash{2166136261u};
	for (auto const character : Name) {
		hash = (hash ^ static_cast<std::uint8_t>(character)) * 16777619u;
	}
	hash ^= Run + 0x9e3779b9u + (hash << 6) + (hash >> 2);
	return hash;
}

meeting meet(end_point const &First, end_point const &Second) {
	meeting result;
	auto const outward{level(First.outward)};
	auto const otherward{level(Second.outward, -outward)};
	result.point = First.point;
	result.push = level(outward - otherward, outward);
	result.bend = angle_between(outward, -otherward);
	glm::dvec3 const apart{Second.point.x - First.point.x, 0.0, Second.point.z - First.point.z};
	auto const along{glm::dot(apart, result.push)};
	auto const across{glm::dot(apart, left_of(result.push))};
	auto const distance{glm::length(apart)};
	result.pull = distance > 0.05 ? apart / distance : result.push;
	// the ends face each other (bent by less than a right angle) and meet within the depth of the buffers and the ends
	// crushed behind them
	auto const facing{glm::dot(outward, otherward) < 0.0};
	if (along <= 0.0 && facing && along > -beam_depth && std::abs(across) < beam_reach) {
		// the buffers press into each other
		result.gap = along;
	}
	else if (along <= 0.0) {
		// the ends went past each other, or one body turned about: only the coupling holds them, stretched
		result.gap = distance;
	}
	else {
		// apart, the coupling between the middles of the beams; as they come together, the buffers in line
		result.gap = std::hypot(along, across) - std::abs(across) * std::max(0.0, 1.0 - along / buffer_reach);
	}
	// the buffers of the outer side of a body rolled against the other one lift or drop by its roll
	auto const apartinheight{std::abs(Second.point.y - First.point.y) + buffer_half_spacing * std::abs(std::sin(Second.roll - First.roll))};
	result.overlap = std::clamp(1.0 - apartinheight / buffer_head, 0.0, 1.0);
	return result;
}

end_force force_on(meeting const &Meeting, double const Compression) {
	end_force result;
	result.point = Meeting.point;
	if (Compression > 0.0) {
		// the inner buffer of the bend takes more of the push, all of it past the play of the buffers; as far as the
		// buffers meet in height, and as much of it as a buffer takes, the rest at the middle of the beam
		auto const holding{buffer_half_spacing * Meeting.overlap * std::min(1.0, buffer_limit / Compression)};
		result.point += left_of(Meeting.push) * (holding * std::clamp(Meeting.bend / buffer_lock, -1.0, 1.0));
		result.force = -Meeting.push * Compression;
	}
	else {
		result.force = Meeting.pull * (-Compression);
	}
	return result;
}

footprint footprint_of(pose const &Pose, double const Length, double const Width, double const Height) {
	footprint result;
	result.axis = level(Pose.front);
	auto const left{left_of(result.axis)};
	result.half_length = std::max(0.5 * Length - buffer_length, 0.5) * std::max(std::hypot(Pose.front.x, Pose.front.z), 0.1);
	// the corners of its cross section, from the rails to the roof, as it is rolled
	auto low{std::numeric_limits<double>::max()};
	auto high{std::numeric_limits<double>::lowest()};
	auto mostleft{std::numeric_limits<double>::lowest()};
	auto mostright{std::numeric_limits<double>::max()};
	for (auto const across : {-0.5 * Width, 0.5 * Width}) {
		for (auto const up : {0.0, Height}) {
			auto const corner{Pose.position + Pose.left * across + Pose.up * up};
			auto const aside{glm::dot(corner - Pose.position, left)};
			mostleft = std::max(mostleft, aside);
			mostright = std::min(mostright, aside);
			low = std::min(low, corner.y);
			high = std::max(high, corner.y);
		}
	}
	result.half_width = 0.5 * (mostleft - mostright);
	result.centre = glm::dvec3{Pose.position.x, 0.0, Pose.position.z} + left * (0.5 * (mostleft + mostright));
	// pitched, the ends rise and drop by the slope over half its length
	auto const rise{std::abs(Pose.front.y) / std::max(std::hypot(Pose.front.x, Pose.front.z), 0.1) * result.half_length};
	result.low = low - rise;
	result.high = high + rise;
	return result;
}

namespace {

// the corners of the footprint, in turn round it
std::array<glm::dvec3, 4> corners_of(footprint const &Print) {
	auto const along{Print.axis * Print.half_length};
	auto const across{left_of(Print.axis) * Print.half_width};
	return {Print.centre + along + across, Print.centre + along - across, Print.centre - along - across, Print.centre - along + across};
}

// whether the point is inside the footprint (in the plan)
bool inside(footprint const &Print, glm::dvec3 const &Point) {
	glm::dvec3 const offset{Point.x - Print.centre.x, 0.0, Point.z - Print.centre.z};
	return std::abs(glm::dot(offset, Print.axis)) <= Print.half_length && std::abs(glm::dot(offset, left_of(Print.axis))) <= Print.half_width;
}

double cross_up(glm::dvec3 const &Arm, glm::dvec3 const &Force) {
	// about the vertical, to the left positive
	return Arm.z * Force.x - Arm.x * Force.z;
}

// the inverse of the mass the body moves with at the point the way of the direction
double inverse_mass(contact_body const &Body, glm::dvec3 const &Point, glm::dvec3 const &Direction) {
	if (Body.mass <= 0.0) {
		return 0.0;
	}
	if (Body.guided) {
		auto const along{glm::dot(Direction, Body.guide)};
		return along * along / Body.mass;
	}
	glm::dvec3 const arm{Point.x - Body.base.x, 0.0, Point.z - Body.base.z};
	auto const turning{cross_up(arm, Direction)};
	return (1.0 + turning * turning / std::max(Body.gyration, 1e-3)) / Body.mass;
}

} // namespace

std::optional<contact> collide(footprint const &First, footprint const &Second) {
	if (std::min(First.high, Second.high) - std::max(First.low, Second.low) < height_overlap) {
		// one goes over the other
		return std::nullopt;
	}
	// the separating axes: the sides of both rectangles; the one they overlap least across is the way out
	auto const first{corners_of(First)};
	auto const second{corners_of(Second)};
	glm::dvec3 const apart{Second.centre.x - First.centre.x, 0.0, Second.centre.z - First.centre.z};
	contact result;
	result.depth = std::numeric_limits<double>::max();
	for (auto const &axis : {First.axis, left_of(First.axis), Second.axis, left_of(Second.axis)}) {
		auto spread = [&axis](std::array<glm::dvec3, 4> const &Corners) {
			auto low{std::numeric_limits<double>::max()};
			auto high{std::numeric_limits<double>::lowest()};
			for (auto const &corner : Corners) {
				auto const along{glm::dot(corner, axis)};
				low = std::min(low, along);
				high = std::max(high, along);
			}
			return std::pair{low, high};
		};
		auto const [low1, high1]{spread(first)};
		auto const [low2, high2]{spread(second)};
		auto const overlap{std::min(high1, high2) - std::max(low1, low2)};
		if (overlap <= 0.0) {
			return std::nullopt;
		}
		if (overlap < result.depth) {
			result.depth = overlap;
			result.normal = glm::dot(apart, axis) >= 0.0 ? axis : -axis;
		}
	}
	// the point: the middle of the corners of each inside the other, or of where the overlap is
	glm::dvec3 sum{0.0};
	auto count{0};
	for (auto const &corner : first) {
		if (inside(Second, corner)) {
			sum += corner;
			++count;
		}
	}
	for (auto const &corner : second) {
		if (inside(First, corner)) {
			sum += corner;
			++count;
		}
	}
	if (count > 0) {
		result.point = sum / static_cast<double>(count);
	}
	else {
		// crossed without a corner inside the other (two long sides across each other): between the middles
		result.point = 0.5 * (First.centre + Second.centre);
	}
	result.point.y = 0.0;
	return result;
}

glm::dvec3 velocity_at(contact_body const &Body, glm::dvec3 const &Point) {
	if (Body.guided) {
		return Body.velocity;
	}
	// turning about the vertical at the yaw rate: up x arm
	glm::dvec3 const arm{Point.x - Body.base.x, 0.0, Point.z - Body.base.z};
	return Body.velocity + Body.yaw_rate * glm::dvec3{arm.z, 0.0, -arm.x};
}

glm::dvec3 contact_impulse(contact const &Contact, contact_body const &First, contact_body const &Second) {
	auto const &normal{Contact.normal};
	auto const relative{velocity_at(Second, Contact.point) - velocity_at(First, Contact.point)};
	auto const closing{glm::dot(relative, normal)};
	if (closing >= 0.0) {
		// parting already
		return glm::dvec3{0.0};
	}
	auto const inverse{inverse_mass(First, Contact.point, normal) + inverse_mass(Second, Contact.point, normal)};
	if (inverse <= 0.0) {
		return glm::dvec3{0.0};
	}
	auto const push{-(1.0 + contact_restitution) * closing / inverse};
	glm::dvec3 result{normal * push};
	// the friction of the sides sliding along each other, no more than stops the sliding
	auto const sliding{relative - normal * closing};
	auto const slide{glm::length(sliding)};
	if (slide > 1e-6) {
		auto const along{sliding / slide};
		auto const alonginverse{inverse_mass(First, Contact.point, along) + inverse_mass(Second, Contact.point, along)};
		if (alonginverse > 0.0) {
			result -= along * std::min(contact_friction * push, slide / alonginverse);
		}
	}
	return result;
}

std::pair<glm::dvec3, double> impulse_change(contact_body const &Body, glm::dvec3 const &Impulse, glm::dvec3 const &Point) {
	if (Body.mass <= 0.0) {
		return {glm::dvec3{0.0}, 0.0};
	}
	if (Body.guided) {
		return {Body.guide * (glm::dot(Impulse, Body.guide) / Body.mass), 0.0};
	}
	glm::dvec3 const arm{Point.x - Body.base.x, 0.0, Point.z - Body.base.z};
	return {glm::dvec3{Impulse.x, 0.0, Impulse.z} / Body.mass, cross_up(arm, Impulse) / (Body.mass * std::max(Body.gyration, 1e-3))};
}

std::optional<double> stopping_distance(double const Speed, double const Friction, double const Grade) {
	auto const angle{std::atan(Grade)};
	auto const deceleration{gravity * (Friction * std::cos(angle) + std::sin(angle))};
	if (deceleration <= 0.0) {
		return std::nullopt;
	}
	return Speed * Speed / (2.0 * deceleration);
}

double pull_ratio(double const Force, double const Angle, double const Mass, int const Axles) {
	auto const axles{std::max(Axles, 2)};
	auto const load{Mass * gravity / (2.0 * axles)};
	if (load <= 0.0) {
		return 0.0;
	}
	auto const across{std::abs(Force * std::sin(Angle))};
	return across / (0.5 * axles) / load;
}

// ---- the ground

std::optional<double> flat_ground::height(double, double, double) const {
	return m_level;
}

mesh_ground::mesh_ground(std::vector<triangle> const &Triangles, glm::dvec2 const &Min, glm::dvec2 const &Max, double const Cell) : m_min{Min}, m_max{glm::max(Max, Min + glm::dvec2{1.0})}, m_cell{std::max(Cell, 1.0)} {
	auto const size{m_max - m_min};
	// a million cells at most
	m_cell = std::max(m_cell, std::sqrt(size.x * size.y / 1e6));
	m_columns = static_cast<std::size_t>(std::ceil(size.x / m_cell));
	m_rows = static_cast<std::size_t>(std::ceil(size.y / m_cell));
	m_cells.resize(m_columns * m_rows);
	auto const column = [this](double const X) { return static_cast<std::size_t>(std::clamp(std::floor((X - m_min.x) / m_cell), 0.0, static_cast<double>(m_columns - 1))); };
	auto const row = [this](double const Z) { return static_cast<std::size_t>(std::clamp(std::floor((Z - m_min.y) / m_cell), 0.0, static_cast<double>(m_rows - 1))); };
	for (auto const &triangle : Triangles) {
		auto const low{glm::min(glm::min(triangle[0], triangle[1]), triangle[2])};
		auto const high{glm::max(glm::max(triangle[0], triangle[1]), triangle[2])};
		if (high.x < m_min.x || low.x > m_max.x || high.z < m_min.y || low.z > m_max.y) {
			continue;
		}
		auto const index{m_triangles.size()};
		m_triangles.push_back(triangle);
		for (auto r = row(low.z); r <= row(high.z); ++r) {
			for (auto c = column(low.x); c <= column(high.x); ++c) {
				m_cells[r * m_columns + c].push_back(index);
			}
		}
	}
}

bool mesh_ground::covers(double const X, double const Z) const {
	return X >= m_min.x && X <= m_max.x && Z >= m_min.y && Z <= m_max.y;
}

std::optional<std::size_t> mesh_ground::cell_of(double const X, double const Z) const {
	if (false == covers(X, Z) || m_cells.empty()) {
		return std::nullopt;
	}
	auto const column{std::min(static_cast<std::size_t>((X - m_min.x) / m_cell), m_columns - 1)};
	auto const row{std::min(static_cast<std::size_t>((Z - m_min.y) / m_cell), m_rows - 1)};
	return row * m_columns + column;
}

std::optional<double> mesh_ground::height(double const X, double const Z, double const Ceiling) const {
	auto const cell{cell_of(X, Z)};
	if (false == cell.has_value()) {
		return std::nullopt;
	}
	std::optional<double> highest;
	for (auto const index : m_cells[*cell]) {
		auto const found{height_in(m_triangles[index], X, Z)};
		if (found.has_value() && *found <= Ceiling && (false == highest.has_value() || *found > *highest)) {
			highest = found;
		}
	}
	return highest;
}

double formation::height(double const X, double const Z) const {
	auto const across{std::abs(glm::dot(glm::dvec3{X, 0.0, Z} - glm::dvec3{point.x, 0.0, point.z}, left_of(level(direction))))};
	auto const top{point.y - depth};
	if (across <= half_width) {
		return top;
	}
	return top - (across - half_width) / std::max(slope, 0.1);
}

std::optional<double> site::height(double const X, double const Z, double const Ceiling) const {
	auto const bed{m_bed.height(X, Z)};
	std::optional<double> result;
	if (m_terrain == nullptr) {
		// no terrain: level with the top of the ballast
		result = m_bed.point.y - m_bed.depth;
	}
	else {
		result = m_terrain->height(X, Z, Ceiling);
	}
	if (bed <= Ceiling && (false == result.has_value() || bed > *result)) {
		result = bed;
	}
	return result;
}

// ---- the vehicle

void body::begin(start const &Start, vehicle_shape const &Shape, ground const &Ground, scatter const &Scatter) {
	*this = body{};
	m_active = true;
	m_reason = Start.reason;
	m_shape = Shape;
	m_scatter = Scatter;
	// the body a box turning about its middle
	m_yawgyration = std::max((Shape.length * Shape.length + Shape.width * Shape.width) / 12.0, 1.0);

	m_pivot = 0.5 * std::max(Shape.contact_spacing, 0.5);
	auto const centre{std::max(Shape.centre_of_gravity, 0.3)};
	m_lever = centre + Shape.flexibility * std::max(0.0, centre - Shape.roll_centre);
	m_tipangle = std::atan2(m_pivot, m_lever);
	m_reach = std::hypot(m_pivot, m_lever);
	// the body a box rolling about its middle, carried over to the wheels it tips about
	m_gyration = m_reach * m_reach + (Shape.width * Shape.width + Shape.height * Shape.height) / 12.0;

	m_position = Start.position;
	m_origin = Start.position;
	m_front = level(Start.front);
	m_line = level(Start.line, m_front);
	m_speed = Start.speed;
	// out of the curve, the way it travels
	auto const direction{Start.speed < 0.0 ? -1.0 : 1.0};
	auto turn{Start.outward * direction * departure_angle(Start.reason, Start.speed, Start.acceleration)};
	if (Start.reason != cause::end_of_track && Start.reason != cause::collision) {
		turn += Scatter.departure;
	}
	m_front = level(turned(m_front, world_up, turn), m_front);
	m_axis = level(turned(m_front, world_up, Scatter.yaw), m_front);

	if (Start.reason == cause::overturned) {
		m_guided = true;
		m_outward = Start.outward != 0 ? Start.outward : 1;
		m_guideacceleration = std::abs(Start.acceleration);
		m_guidespeed = std::max(std::abs(Start.speed), 0.1);
	}
	else {
		m_yawrate = Scatter.yaw_rate;
	}
	auto const foot{find_footing(Ground)};
	m_pitch = foot.grade;
	m_grade = foot.grade;
	m_cross = foot.cross;
	place();
}

void body::step(double const Dt, double const Distance, double const Speed, ground const &Ground, std::array<end_force, 2> const &Ends) {
	if (false == m_active || Dt <= 0.0) {
		return;
	}
	// where the forces of the couplings act on the body, which turns under them through the step
	for (std::size_t end = 0; end < Ends.size(); ++end) {
		glm::dvec3 const arm{Ends[end].point.x - m_position.x, 0.0, Ends[end].point.z - m_position.z};
		m_ends[end].force = glm::dvec3{Ends[end].force.x, 0.0, Ends[end].force.z};
		m_ends[end].along = glm::dot(arm, m_axis);
		m_ends[end].across = glm::dot(arm, left_of_heading());
	}
	auto const steps{std::max(1, static_cast<int>(std::ceil(Dt / substep - 1e-9)))};
	auto const dt{Dt / steps};
	auto const startspeed{m_speed};
	// the heading turned about within the step goes the other way along with the rest of the step
	auto direction{1.0};
	for (int index = 1; index <= steps; ++index) {
		auto const speed{std::lerp(startspeed, direction * Speed, static_cast<double>(index) / steps)};
		auto const reversals{m_reversals};
		advance(dt, direction * Distance / steps, speed, Ground);
		if (m_reversals != reversals) {
			direction = -direction;
		}
	}
	m_speed = direction * Speed;
	place();
}

void body::advance(double const Dt, double const Distance, double const Speed, ground const &Ground) {
	m_position += m_front * Distance + left_of_heading() * (m_lateral * Dt);
	m_travelled += std::hypot(Distance, m_lateral * Dt);
	m_time += Dt;
	auto const foot{find_footing(Ground)};
	m_pitch = foot.grade;
	// along the way it goes: of the slope along the body and across it
	m_grade = foot.grade * glm::dot(m_front, m_axis) + foot.cross * glm::dot(m_front, left_of_heading());
	m_cross = foot.cross;
	fall(Dt, foot.level, std::hypot(Distance, m_lateral * Dt));
	slide(Dt, foot.cross);
	tip(Dt, foot.cross, Speed);
	turn(Dt, Speed);
	if (m_guided) {
		m_guidetime += Dt;
		if (m_tipping >= m_tipangle || std::abs(Speed) < 0.5 || m_guidetime > guidance_time) {
			m_guided = false;
		}
	}
}

body::footing body::find_footing(ground const &Ground) const {
	auto const left{left_of_heading()};
	// a step up under the middle, and as much more under a support away from it as a slope of 2 in 1 rises: what is
	// higher is over the vehicle, not under it (a bridge, a roof), and the ground under the slope of a bank isn't the
	// ground the support stands on
	constexpr double steepest{2.0};
	auto const sample = [&Ground, this](glm::dvec3 const &Point) {
		auto const reach{std::hypot(Point.x - m_position.x, Point.z - m_position.z)};
		return Ground.height(Point.x, Point.z, m_position.y + step_up + steepest * reach).value_or(m_position.y);
	};
	auto const ahead{m_axis * (0.5 * std::max(m_shape.base, 1.0))};
	// upright it stands on its wheels; tipping over, it comes to rest on the side of its body, which reaches out
	// from the wheels it tips about as far as the body is high
	auto const lean{std::sin(m_tipping)};
	auto const middle{left * (m_side * lean * 0.5 * m_shape.height)};
	auto const span{m_pivot + lean * std::max(0.0, 0.5 * m_shape.height - m_pivot)};
	auto const aside{left * span};
	auto const frontleft{sample(m_position + middle + ahead + aside)};
	auto const frontright{sample(m_position + middle + ahead - aside)};
	auto const rearleft{sample(m_position + middle - ahead + aside)};
	auto const rearright{sample(m_position + middle - ahead - aside)};
	footing result;
	result.level = std::max(0.25 * (frontleft + frontright + rearleft + rearright), sample(m_position));
	// a step under one support is no slope steeper than 2 in 1
	result.grade = std::clamp((frontleft + frontright - rearleft - rearright) / (2.0 * std::max(m_shape.base, 1.0)), -steepest, steepest);
	result.cross = std::clamp((frontleft + rearleft - frontright - rearright) / (4.0 * span), -steepest, steepest);
	return result;
}

void body::fall(double const Dt, double const Level, double const Travel) {
	// on the ground it follows it down a slope as steep as 1:1.5; off a steeper drop it falls
	constexpr double follows{1.0 / 1.5};
	if (m_grounded && m_position.y - Level <= std::max(0.02, follows * Travel)) {
		m_position.y = Level;
		return;
	}
	m_fallspeed -= gravity * Dt;
	auto const height{m_position.y + m_fallspeed * Dt};
	m_grounded = (height <= Level);
	if (m_grounded) {
		m_position.y = Level;
		m_fallspeed = 0.0;
	}
	else {
		m_position.y = height;
	}
}

void body::slide(double const Dt, double const Cross) {
	if (false == m_grounded) {
		// in the air nothing holds it, nor pushes it
		return;
	}
	auto const angle{std::atan(Cross)};
	// down the slope, to the left positive; held by the friction of its weight on the slope
	m_lateral -= gravity * std::sin(angle) * Dt;
	auto const held{lateral_friction * gravity * std::cos(angle) * Dt};
	if (std::abs(m_lateral) <= held) {
		m_lateral = 0.0;
	}
	else {
		m_lateral -= sign_of(m_lateral) * held;
	}
}

double body::lateral_load(double const Cross, double const Speed) const {
	auto const angle{std::atan(Cross)};
	auto load{0.0};
	if (m_grounded) {
		if (m_lateral != 0.0) {
			// sliding across, the friction under its wheels or its side trips it the way it slides
			load = sign_of(m_lateral) * lateral_friction * gravity * std::cos(angle);
		}
		else {
			// standing on the slope, the weight pulls it down the slope, the friction holds its feet
			load = -gravity * std::sin(angle);
		}
		// going askew, the friction across it trips it too
		load += m_trip;
	}
	if (m_guided) {
		auto const share{Speed / m_guidespeed};
		load += m_outward * m_guideacceleration * share * share;
	}
	return load;
}

void body::tip(double const Dt, double const Cross, double const Speed) {
	if (lying()) {
		return;
	}
	auto const normal{gravity * std::cos(std::atan(Cross))};
	auto const load{lateral_load(Cross, Speed)};
	if (m_side == 0) {
		// upright: it starts to tip once the load lifts the wheels of the other side
		if (std::abs(load) * std::cos(m_tipangle) - normal * std::sin(m_tipangle) <= 0.0) {
			return;
		}
		m_side = load > 0.0 ? 1 : -1;
	}
	auto const angle{m_tipping - m_tipangle};
	auto const acceleration{m_reach / m_gyration * (m_side * load * std::cos(angle) + normal * std::sin(angle))};
	m_tiprate += acceleration * Dt;
	m_tipping += m_tiprate * Dt;
	if (m_tipping <= 0.0) {
		// back on its wheels
		m_tipping = 0.0;
		m_tiprate = 0.0;
		m_side = 0;
	}
	else if (m_tipping >= half_turn) {
		// on its side, where it stays
		m_tipping = half_turn;
		m_tiprate = 0.0;
	}
}

glm::dvec3 body::left_of_heading() const {
	return left_of(m_axis);
}

double body::friction_along() const {
	return sliding_friction(m_tipping, m_guided);
}

double body::friction_across() const {
	return m_guided ? guided_friction : std::max(lateral_friction, friction_along());
}

double body::friction_share(bool const Front) const {
	return Front ? m_scatter.friction_front : m_scatter.friction_rear;
}

int body::biting() const {
	if (m_scatter.dig_in < 0.0 || m_guided || false == m_grounded || m_time < m_scatter.dig_in || m_time >= m_scatter.dig_in + m_scatter.dig_in_time) {
		return 0;
	}
	// the end it goes towards
	auto const going{m_speed < 0.0 ? -m_front : m_front};
	return glm::dot(going, m_axis) >= 0.0 ? 1 : -1;
}

double body::friction() const {
	auto const along{glm::dot(m_front, m_axis)};
	auto const across{1.0 - along * along};
	auto result{0.5 * (friction_share(true) + friction_share(false)) * (friction_along() * along * along + friction_across() * across)};
	if (biting() != 0) {
		// under half of its weight
		result += 0.5 * (m_scatter.dig_in_friction - 1.0) * friction_along();
	}
	return result;
}

glm::dvec3 body::velocity() const {
	return m_front * m_speed + left_of_heading() * m_lateral;
}

void body::kick(glm::dvec3 const &Velocity, double const YawRate) {
	if (false == m_active) {
		return;
	}
	auto const moving{m_front * m_speed + glm::dvec3{Velocity.x, 0.0, Velocity.z}};
	auto const speed{glm::length(moving)};
	if (speed > 1e-6) {
		m_front = moving / speed;
		m_speed = speed;
		if (glm::dot(m_front, m_axis) < 0.0) {
			// it goes rear first
			m_front = -m_front;
			m_speed = -m_speed;
			++m_reversals;
		}
	}
	else {
		m_speed = 0.0;
	}
	m_yawrate = std::clamp(m_yawrate + YawRate, -most_yaw_rate, most_yaw_rate);
}

void body::nudge(glm::dvec3 const &Offset) {
	if (false == m_active) {
		return;
	}
	m_position += glm::dvec3{Offset.x, 0.0, Offset.z};
	place();
}

double body::yaw() const {
	return angle_between(m_line, m_axis);
}

body::drag body::friction_under(double const Speed, double const Yawrate) const {
	drag result;
	if (false == m_grounded) {
		// nothing holds it in the air
		return result;
	}
	auto const left{left_of_heading()};
	auto const along{friction_along()};
	auto const across{friction_across()};
	// upright it stands on its bogies (the ends of its base), on its side it lies along all of its length
	auto const share{std::clamp(m_tipping / half_turn, 0.0, 1.0)};
	auto const base{std::max(m_shape.base, 1.0)};
	auto const span{std::max(m_shape.length, base)};
	auto const velocity{m_front * Speed};
	auto const point = [&](double const Along, double const Across, double const Weight) {
		// the friction under a point of the body: against the way it slides there, along the body and across it as each
		// holds; slower than the creep, as much as it slides
		auto const motion{velocity + left * (Yawrate * Along) - m_axis * (Yawrate * Across)};
		auto const speed{std::max(glm::length(motion), creep_speed)};
		auto const force{-(gravity * Weight) * (along * glm::dot(motion, m_axis) * m_axis + across * glm::dot(motion, left) * left) / speed};
		result.force += force;
		// about the vertical, to the left positive
		result.moment += Along * glm::dot(force, left) - Across * glm::dot(force, m_axis);
	};
	if (share < 1.0) {
		for (auto const end : {1.0, -1.0}) {
			point(end * 0.5 * base, 0.0, (1.0 - share) * 0.5 * friction_share(end > 0.0));
		}
	}
	if (share > 0.0) {
		for (int index = 0; index < friction_points; ++index) {
			auto const along_body{span * ((index + 0.5) / friction_points - 0.5)};
			point(along_body, 0.0, share * friction_share(along_body > 0.0) / friction_points);
		}
	}
	if (auto const end{biting()}; end != 0) {
		// the leading end bites into the ballast at one of its wheels, under half of its weight; on its side, at a corner
		// of its body
		point(end * 0.5 * std::lerp(base, 0.5 * span, share), m_scatter.dig_in_side * std::lerp(m_pivot, 0.5 * m_shape.height, share), 0.5 * (m_scatter.dig_in_friction - 1.0));
	}
	// the way it slides across, the friction trips it
	result.trip = -glm::dot(result.force, left);
	return result;
}

void body::turn(double const Dt, double const Speed) {
	// the forces of the couplings, per the mass
	auto const left{left_of_heading()};
	auto moment{0.0};
	glm::dvec3 push{0.0};
	if (m_shape.mass > 0.0) {
		for (auto const &end : m_ends) {
			auto const arm{m_axis * end.along + left * end.across};
			moment += (arm.z * end.force.x - arm.x * end.force.z) / m_shape.mass;
			push += end.force / m_shape.mass;
		}
	}
	if (m_guided) {
		// on its outer wheels on the rail it goes along the rail
		m_yawrate = 0.0;
		m_trip = 0.0;
		return;
	}
	// the body: the couplings turn it, and the friction under it where it differs between the ends of a body going
	// askew; the friction holds it against turning, never turning it back the other way
	auto const driving{friction_under(Speed, 0.0).moment};
	auto rate{m_yawrate + (moment + driving) / m_yawgyration * Dt};
	auto const change{(friction_under(Speed, rate).moment - driving) / m_yawgyration * Dt};
	rate = ((rate + change) * rate <= 0.0 ? 0.0 : rate + change);
	if (std::abs(Speed) < creep_speed && m_lateral == 0.0 && std::abs(rate) < 0.01 && m_grounded) {
		// at a standstill it doesn't turn as long as the friction under it holds against the couplings
		// on its bogies at half the base from the middle, lying along the length a quarter of it on average
		auto const share{std::clamp(m_tipping / half_turn, 0.0, 1.0)};
		auto const arm{std::lerp(0.5 * std::max(m_shape.base, 1.0), 0.25 * std::max(m_shape.length, m_shape.base), share)};
		if (std::abs(moment) <= gravity * friction_across() * arm) {
			rate = 0.0;
		}
	}
	m_yawrate = std::clamp(rate, -most_yaw_rate, most_yaw_rate);
	m_axis = level(turned(m_axis, world_up, m_yawrate * Dt), m_axis);

	// the way it travels: the friction of the body going askew turns it towards the body (no further), the couplings
	// pushing across it turn it their way; the faster it goes, the less
	auto const drag{friction_under(Speed, m_yawrate)};
	m_trip = drag.trip;
	if (m_grounded && std::abs(Speed) >= turning_speed) {
		auto const across{left_of(m_front)};
		auto const off{angle_between(m_front, glm::dot(m_front, m_axis) >= 0.0 ? m_axis : -m_axis)};
		auto byfriction{glm::dot(drag.force, across) * Dt / Speed};
		byfriction = (byfriction * off <= 0.0 ? 0.0 : sign_of(off) * std::min(std::abs(byfriction), std::abs(off)));
		auto const bypush{glm::dot(push, across) * Dt / Speed};
		auto const turning{std::clamp(byfriction + bypush, -most_yaw_rate * Dt, most_yaw_rate * Dt)};
		m_front = level(turned(m_front, world_up, turning), m_front);
	}
	if (glm::dot(m_front, m_axis) < 0.0) {
		// the body turned past a right angle off the way it goes: it goes rear first, the heading the other way
		m_front = -m_front;
		m_speed = -m_speed;
		++m_reversals;
	}
}

double body::roll() const {
	// the ground rising to the left leans it to the right
	return tipping() - std::atan(m_cross);
}

double body::along() const {
	return glm::dot(m_position - m_origin, m_line);
}

double body::across() const {
	return glm::dot(m_position - m_origin, left_of(m_line));
}

bool body::at_rest() const {
	return m_active && m_grounded && std::abs(m_speed) < 0.01 && m_lateral == 0.0 && m_tiprate == 0.0 && m_yawrate == 0.0 && false == m_guided;
}

void body::place() {
	// the plane of the ground under the supports
	auto const front{glm::normalize(m_axis + world_up * m_pitch)};
	auto const flatleft{glm::normalize(left_of_heading() + world_up * m_cross)};
	auto const up{glm::normalize(glm::cross(front, flatleft))};
	auto const left{glm::cross(up, front)};
	// tipped about the wheels of its side: turned about the front, the up going over to that side
	auto const angle{-m_side * m_tipping};
	auto const pivot{m_position + left * (m_side * m_pivot)};
	auto origin{pivot + turned(m_position - pivot, front, angle)};
	auto const tippedleft{turned(left, front, angle)};
	auto const tippedup{turned(up, front, angle)};
	// the body doesn't go into the ground: the lowest of the edges of its box on it, from the rails to its roof
	auto lowest{std::numeric_limits<double>::max()};
	for (auto const across : {-0.5 * m_shape.width, 0.5 * m_shape.width}) {
		for (auto const height : {0.0, m_shape.height}) {
			lowest = std::min(lowest, glm::dot(origin + tippedleft * across + tippedup * height - m_position, up));
		}
	}
	if (lowest < 0.0) {
		origin -= up * lowest;
	}
	m_pose.position = origin;
	m_pose.front = front;
	m_pose.left = tippedleft;
	m_pose.up = tippedup;
}

} // namespace derail_motion

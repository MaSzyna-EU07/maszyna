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

void body::begin(start const &Start, vehicle_shape const &Shape, ground const &Ground) {
	*this = body{};
	m_active = true;
	m_reason = Start.reason;
	m_shape = Shape;

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
	auto const turn{Start.outward * direction * departure_angle(Start.reason, Start.speed, Start.acceleration)};
	m_front = level(turned(m_front, world_up, turn), m_front);

	if (Start.reason == cause::overturned) {
		m_guided = true;
		m_outward = Start.outward != 0 ? Start.outward : 1;
		m_guideacceleration = std::abs(Start.acceleration);
		m_guidespeed = std::max(std::abs(Start.speed), 0.1);
	}
	auto const foot{find_footing(Ground)};
	m_grade = foot.grade;
	m_cross = foot.cross;
	place();
}

void body::step(double const Dt, double const Distance, double const Speed, ground const &Ground) {
	if (false == m_active || Dt <= 0.0) {
		return;
	}
	auto const steps{std::max(1, static_cast<int>(std::ceil(Dt / substep - 1e-9)))};
	auto const dt{Dt / steps};
	auto const startspeed{m_speed};
	for (int index = 1; index <= steps; ++index) {
		auto const speed{std::lerp(startspeed, Speed, static_cast<double>(index) / steps)};
		advance(dt, Distance / steps, speed, Ground);
	}
	m_speed = Speed;
	place();
}

void body::advance(double const Dt, double const Distance, double const Speed, ground const &Ground) {
	m_position += m_front * Distance + left_of_heading() * (m_lateral * Dt);
	m_travelled += std::hypot(Distance, m_lateral * Dt);
	auto const foot{find_footing(Ground)};
	m_grade = foot.grade;
	m_cross = foot.cross;
	fall(Dt, foot.level, std::hypot(Distance, m_lateral * Dt));
	slide(Dt, foot.cross);
	tip(Dt, foot.cross, Speed);
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
	auto const ahead{m_front * (0.5 * std::max(m_shape.base, 1.0))};
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
	return left_of(m_front);
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
	return m_active && m_grounded && std::abs(m_speed) < 0.01 && m_lateral == 0.0 && m_tiprate == 0.0 && false == m_guided;
}

void body::place() {
	// the plane of the ground under the supports
	auto const front{glm::normalize(m_front + world_up * m_grade)};
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

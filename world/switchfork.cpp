/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/switchfork.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace switch_fork {

int leaving_path(int const Path, int const Set) {
	return (Path == 0 || Path == 1) && Path != (Set & 1) ? Path : -1;
}

void path_memory::record(void const *Switch, int const Path) {
	// the older record of the switch goes, the ones after it move up
	auto const end{m_entries.begin() + m_count};
	auto const found{std::find_if(m_entries.begin(), end, [Switch](auto const &Entry) { return Entry.first == Switch; })};
	if (found != end) {
		std::rotate(found, found + 1, end);
		--m_count;
	}
	if (m_count == size) {
		// the oldest goes
		std::rotate(m_entries.begin(), m_entries.begin() + 1, m_entries.end());
		--m_count;
	}
	m_entries[m_count++] = {Switch, Path};
}

std::optional<int> path_memory::path_of(void const *Switch) const {
	for (auto i = m_count; i > 0; --i) {
		if (m_entries[i - 1].first == Switch) {
			return m_entries[i - 1].second;
		}
	}
	return std::nullopt;
}

bool forked(int const Path, std::optional<int> const Ahead) {
	return Ahead.has_value() && *Ahead >= 0 && Path >= 0 && *Ahead != Path;
}

double bogie_limit(double const Pivots, double const Radius) {
	if (Radius <= 0.0) {
		return 0.0;
	}
	return std::asin(std::clamp(std::abs(Pivots) / (2.0 * Radius), 0.0, 1.0));
}

double turn(double const TrackHeading, double const BodyHeading) {
	// the difference of the lines, into (-pi/2, pi/2]
	auto angle{std::remainder(TrackHeading - BodyHeading, std::numbers::pi)};
	if (angle <= -0.5 * std::numbers::pi) {
		angle += std::numbers::pi;
	}
	return angle;
}

double heading(double const X, double const Z) {
	return std::atan2(-X, Z);
}

double end_offset(double const X, double const Z, double const Heading, double const OtherX, double const OtherZ, double const OtherHeading) {
	// the mean of the two lines: the other heading brought within a quarter turn of the first one
	auto const mean{Heading + 0.5 * turn(OtherHeading, Heading)};
	// across it: the unit vector of the heading turned by a quarter, (-sin h, cos h) to (cos h, sin h)
	auto const acrossx{std::cos(mean)};
	auto const acrossz{std::sin(mean)};
	return std::abs((OtherX - X) * acrossx + (OtherZ - Z) * acrossz);
}

state bogies(double const Front, double const Rear, double const Pivots, double const Radius) {
	return {std::max(std::abs(Front), std::abs(Rear)), bogie_limit(Pivots, Radius)};
}

state coupling(double const Offset) {
	return {std::abs(Offset), buffer_head_width};
}

} // namespace switch_fork

/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string_view>

// smooth regulation of the universal cab controls (MaSzyna-EU07/maszyna#62): a universal whose definition in the
// .mmd has "drag: <direction>" can be dragged with the mouse to any position between its ends (0 and 1), and stays
// where it is let go, like a window opened a crack or a sun blind pulled half way down. a click without dragging and
// the keys still move it to its ends, as they do any universal. universals without the key work as they always did
namespace universal_drag {

// the way the mouse moves to take the control towards its end 1 (to open it); none: the control can't be dragged
enum class direction { none, up, down, left, right };

// the direction the value of the drag: key gives, nothing for a value it doesn't know
inline std::optional<direction> parse(std::string_view const Value) {
	if (Value == "yes" || Value == "true" || Value == "1" || Value == "up") {
		return direction::up;
	}
	if (Value == "down") {
		return direction::down;
	}
	if (Value == "left") {
		return direction::left;
	}
	if (Value == "right") {
		return direction::right;
	}
	if (Value == "no" || Value == "false" || Value == "0" || Value == "none") {
		return direction::none;
	}
	return std::nullopt;
}

// a push (impulse) universal springs back when it is let go, so it can't be left anywhere in between
inline direction effective(direction const Drag, bool const Push) {
	return Push ? direction::none : Drag;
}

// the position of the control, kept between its ends
inline float clamp(double const Position) {
	return static_cast<float>(std::clamp(Position, 0.0, 1.0));
}

// how far the cursor moving by Dx, Dy pixels (y grows downwards, as the cursor's does) takes the control along the
// Drag direction, as a fraction of its full travel; Size is the move in pixels which takes it from one end to the other
inline double along(direction const Drag, double const Dx, double const Dy, double const Size) {
	if (Size <= 0.0) {
		return 0.0;
	}
	switch (Drag) {
	case direction::up:
		return -Dy / Size;
	case direction::down:
		return Dy / Size;
	case direction::left:
		return -Dx / Size;
	case direction::right:
		return Dx / Size;
	default:
		return 0.0;
	}
}

// the position the keys and a click move the control to: the end farther from the position it is set to, as for
// the two-state universals: a control open less than half way opens fully, one open half way or more closes
inline float toggled(float const Desired) {
	return Desired < 0.5f ? 1.f : 0.f;
}

// the travel of the cursor along the direction of the drag, as a fraction of the full travel of the control, under
// which letting the button go counts as a click rather than a drag
inline constexpr double click_travel{0.02};

// one drag of a control, from the press of the mouse button to its release
class drag {

  public:
	drag(direction const Drag, float const Position) : m_direction{Drag}, m_position{clamp(Position)}, m_start{m_position} {}

	// the cursor moved to X, Y; Size is the move in pixels which takes the control from one end to the other.
	// the first position of the cursor is only where the drag starts from: when the cursor gets hidden for the drag
	// it can be put elsewhere. returns whether the position of the control changed
	bool move(double const X, double const Y, double const Size) {
		if (false == m_started) {
			m_started = true;
			m_x = X;
			m_y = Y;
			return false;
		}
		auto const step{along(m_direction, X - m_x, Y - m_y, Size)};
		m_x = X;
		m_y = Y;
		m_travel += std::abs(step);
		// the position follows each move from where it is, so after the control reached its end, moving the cursor
		// back takes it back at once rather than after the overshoot is undone
		auto const position{clamp(m_position + step)};
		if (position == m_position) {
			return false;
		}
		m_position = position;
		return true;
	}
	// the position the control is dragged to
	float position() const {
		return m_position;
	}
	// the position the control was in when the drag started
	float start() const {
		return m_start;
	}
	// whether letting the button go now is a click rather than a drag: the cursor barely moved along the direction
	bool is_click() const {
		return m_travel < click_travel;
	}

  private:
	direction m_direction;
	float m_position;
	float m_start;
	bool m_started{false};
	double m_x{0.0};
	double m_y{0.0};
	double m_travel{0.0}; // the moves of the cursor along the direction, either way, as fractions of the full travel
};

} // namespace universal_drag

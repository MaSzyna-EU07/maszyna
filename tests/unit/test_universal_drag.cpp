#include "stdafx.h"
#include "doctest.h"

#include "vehicle/universaldrag.h"

// smooth regulation of the universal cab controls (MaSzyna-EU07/maszyna#62): a universal with "drag: <direction>"
// in its definition is dragged with the mouse to any position between its ends and stays there when let go; a click
// and the keys move it to its ends. the mouse and TGauge are not part of the core library, the rules they follow are

using universal_drag::direction;

TEST_CASE("universal drag: the values of the drag key") {
	CHECK(universal_drag::parse("up") == direction::up);
	CHECK(universal_drag::parse("down") == direction::down);
	CHECK(universal_drag::parse("left") == direction::left);
	CHECK(universal_drag::parse("right") == direction::right);
	// yes drags it up, as the sliders of the controllers
	CHECK(universal_drag::parse("yes") == direction::up);
	CHECK(universal_drag::parse("true") == direction::up);
	CHECK(universal_drag::parse("1") == direction::up);
	CHECK(universal_drag::parse("no") == direction::none);
	CHECK(universal_drag::parse("false") == direction::none);
	CHECK(universal_drag::parse("0") == direction::none);
	CHECK(universal_drag::parse("none") == direction::none);
	// a value it doesn't know is reported, and the control can't be dragged
	CHECK_FALSE(universal_drag::parse("sideways").has_value());
	CHECK_FALSE(universal_drag::parse("").has_value());
	CHECK_FALSE(universal_drag::parse("Up").has_value());
}

TEST_CASE("universal drag: a push universal can't be dragged, it springs back when let go") {
	CHECK(universal_drag::effective(direction::up, true) == direction::none);
	CHECK(universal_drag::effective(direction::left, true) == direction::none);
	CHECK(universal_drag::effective(direction::up, false) == direction::up);
	CHECK(universal_drag::effective(direction::none, false) == direction::none);
}

TEST_CASE("universal drag: the move of the cursor along the direction") {
	auto const size{600.0};
	// the cursor's y grows downwards
	CHECK(universal_drag::along(direction::up, 0.0, -60.0, size) == doctest::Approx(0.1));
	CHECK(universal_drag::along(direction::up, 0.0, 60.0, size) == doctest::Approx(-0.1));
	CHECK(universal_drag::along(direction::down, 0.0, 60.0, size) == doctest::Approx(0.1));
	CHECK(universal_drag::along(direction::left, -150.0, 0.0, size) == doctest::Approx(0.25));
	CHECK(universal_drag::along(direction::right, -150.0, 0.0, size) == doctest::Approx(-0.25));
	// the move across the direction doesn't count
	CHECK(universal_drag::along(direction::up, 300.0, 0.0, size) == 0.0);
	CHECK(universal_drag::along(direction::left, 0.0, 300.0, size) == 0.0);
	// nothing to drag
	CHECK(universal_drag::along(direction::none, 100.0, 100.0, size) == 0.0);
	CHECK(universal_drag::along(direction::up, 0.0, -60.0, 0.0) == 0.0);
}

TEST_CASE("universal drag: the position is kept between the ends") {
	CHECK(universal_drag::clamp(-0.5) == 0.f);
	CHECK(universal_drag::clamp(1.5) == 1.f);
	CHECK(universal_drag::clamp(0.25) == doctest::Approx(0.25f));
}

TEST_CASE("universal drag: the keys and a click move the control to the end farther from its position") {
	CHECK(universal_drag::toggled(0.f) == 1.f);
	CHECK(universal_drag::toggled(1.f) == 0.f);
	// a control left in between
	CHECK(universal_drag::toggled(0.3f) == 1.f);
	CHECK(universal_drag::toggled(0.7f) == 0.f);
	// half way, as the two-state universals always did
	CHECK(universal_drag::toggled(0.5f) == 0.f);
}

TEST_CASE("universal drag: a drag takes the control to a position between its ends, where it stays") {
	auto const size{600.0};
	universal_drag::drag drag{direction::up, 0.f};
	// the first position of the cursor is where the drag starts from, even if the hidden cursor was moved
	CHECK_FALSE(drag.move(400.0, 500.0, size));
	CHECK(drag.position() == 0.f);
	CHECK(drag.is_click());
	// up by a quarter of the size
	CHECK(drag.move(400.0, 350.0, size));
	CHECK(drag.position() == doctest::Approx(0.25f));
	// sideways it stays
	CHECK_FALSE(drag.move(700.0, 350.0, size));
	CHECK(drag.position() == doctest::Approx(0.25f));
	// back down a little
	CHECK(drag.move(700.0, 380.0, size));
	CHECK(drag.position() == doctest::Approx(0.2f));
	CHECK_FALSE(drag.is_click());
	// let go: the position is what the drag left, the control keeps it
	CHECK(drag.start() == 0.f);
	CHECK(drag.position() == doctest::Approx(0.2f));
}

TEST_CASE("universal drag: the control stops at its ends, and moving back takes it back at once") {
	auto const size{100.0};
	universal_drag::drag drag{direction::right, 0.9f};
	drag.move(0.0, 0.0, size);
	CHECK(drag.move(50.0, 0.0, size));
	CHECK(drag.position() == 1.f);
	// further on it stays at the end
	CHECK_FALSE(drag.move(200.0, 0.0, size));
	CHECK(drag.position() == 1.f);
	// back: from the end, not from where the cursor went past it
	CHECK(drag.move(190.0, 0.0, size));
	CHECK(drag.position() == doctest::Approx(0.9f));
	// and the other end
	CHECK(drag.move(0.0, 0.0, size));
	CHECK(drag.position() == 0.f);
}

TEST_CASE("universal drag: a press let go with the cursor barely moved is a click") {
	auto const size{600.0};
	universal_drag::drag drag{direction::down, 0.4f};
	drag.move(10.0, 10.0, size);
	// a jitter of the hand, under the travel of a click
	drag.move(11.0, 13.0, size);
	drag.move(10.0, 11.0, size);
	CHECK(drag.is_click());
	// the click moves it to the end farther from it, as the key does
	CHECK(universal_drag::toggled(drag.position()) == 1.f);
	// a drag there and back is a drag, though it ends where it started
	universal_drag::drag back{direction::down, 0.4f};
	back.move(0.0, 0.0, size);
	back.move(0.0, 60.0, size);
	back.move(0.0, 0.0, size);
	CHECK_FALSE(back.is_click());
	CHECK(back.position() == doctest::Approx(0.4f));
}

TEST_CASE("universal drag: a drag starts from the position the control is set to") {
	universal_drag::drag drag{direction::up, 0.6f};
	CHECK(drag.start() == doctest::Approx(0.6f));
	CHECK(drag.position() == doctest::Approx(0.6f));
	// a position out of the range is brought into it
	CHECK(universal_drag::drag{direction::up, 3.f}.position() == 1.f);
	CHECK(universal_drag::drag{direction::up, -1.f}.position() == 0.f);
}

TEST_CASE("universal drag: a control which can't be dragged doesn't move") {
	universal_drag::drag drag{direction::none, 0.f};
	drag.move(0.0, 0.0, 600.0);
	CHECK_FALSE(drag.move(300.0, -300.0, 600.0));
	CHECK(drag.position() == 0.f);
	CHECK(drag.is_click());
}

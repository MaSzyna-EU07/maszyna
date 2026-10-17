#include "stdafx.h"
#include "doctest.h"

#include "vehicle/mirrors.h"

// the sounds of the mirrors (MaSzyna-EU07/maszyna#84, eu07/maszyna#322): a mirror sounds as it sets off, unfolding
// or folding, and not again while it keeps moving the same way

using mirror_moves::event;

TEST_CASE("mirrors: a mirror sounds once as it sets off") {
	mirror_moves::motion mirror;
	CHECK(mirror.update(0.0) == event::none); // standing folded
	CHECK(mirror.update(0.01) == event::unfolding);
	CHECK(mirror.update(0.01) == event::none); // still unfolding
	CHECK(mirror.update(0.0) == event::none); // unfolded
	CHECK(mirror.update(-0.01) == event::folding);
	CHECK(mirror.update(-0.01) == event::none);
}

TEST_CASE("mirrors: a mirror turned back halfway sounds the other way") {
	mirror_moves::motion mirror;
	REQUIRE(mirror.update(0.01) == event::unfolding);
	CHECK(mirror.update(-0.01) == event::folding); // the vehicle got too fast while it was unfolding
}

TEST_CASE("mirrors: a mirror setting off again the same way sounds again") {
	mirror_moves::motion mirror;
	REQUIRE(mirror.update(0.01) == event::unfolding);
	REQUIRE(mirror.update(0.0) == event::none); // stopped, the doors lost their permit say
	CHECK(mirror.update(0.01) == event::unfolding);
}

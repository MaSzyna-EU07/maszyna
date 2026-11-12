#include "stdafx.h"
#include "doctest.h"
#include "scratch.h"

#include "scripting/ladderlogic.h"

// the programmable controller of a vehicle (scripting/ladderlogic.cpp): a program of ladder logic in the instruction
// list form (ld, and, or, out, set, rst...), run once in every step of the vehicle, with variables, timers and
// counters told apart by their names. nothing in the game loads a program into it yet (see COVERAGE.md), so no game
// vehicle has one: the programs here are made up, and the tests record how the controller runs them now

namespace {

// a controller, and a folder for the file of its program
class controller_fixture {
  public:
	explicit controller_fixture(std::string const &Name) : m_folder{"plc_" + Name} {}

	bool load(std::string_view const Program) { return controller.load(m_folder.write("program.plc", Program)); }
	// the element of that name, made as a variable if neither the program nor the test used the name yet
	int &in(std::string const &Name) { return controller.input(controller.find_or_insert(Name)); }
	int out(std::string const &Name) { return controller.output(controller.find_or_insert(Name)); }
	// runs the program, a tenth of a second for every cycle: one step of the timers
	void cycles(int const Count = 1) {
		for (auto cycle{0}; cycle < Count; ++cycle) {
			controller.update(0.1);
		}
	}
	// a cycle with the element on, then one with it off
	void pulse(std::string const &Name) {
		in(Name) = 1;
		cycles();
		in(Name) = 0;
		cycles();
	}
	std::string missing_file() const { return m_folder.path("nothing.plc"); }

	plc::basic_controller controller;

  private:
	test::scratch_folder m_folder;
};

} // namespace

// loading -------------------------------------------------------------------------------------------------------------

TEST_CASE("plc: a program is read from a file, an operation a line, and run with every update") {
	controller_fixture plc{"load"};
	REQUIRE(plc.load("ld start\nout lamp\n"));
	plc.in("start") = 1;
	CHECK(plc.controller.update(0.1) == 0);
	CHECK(plc.out("lamp") == 1);
	plc.in("start") = 0;
	CHECK(plc.controller.update(0.1) == 0);
	CHECK(plc.out("lamp") == 0);
}

TEST_CASE("plc: a controller without a program runs nothing") {
	plc::basic_controller controller;
	auto const lamp{controller.find_or_insert("lamp")};
	controller.input(lamp) = 1;
	CHECK(controller.update(0.1) == 0);
	CHECK(controller.output(lamp) == 1);
}

TEST_CASE("plc: the commands are read in any case, with tabs, comments and empty lines around them") {
	controller_fixture plc{"layout"};
	REQUIRE(plc.load("// the lamp follows the button\r\n\r\nLD\tStart\r\n  OUT   Lamp // done\r\n\r\n"));
	// the names of the elements are put in lower case
	plc.in("start") = 1;
	plc.cycles();
	CHECK(plc.out("lamp") == 1);
	// and looked up as they are given: this one is another element
	CHECK(plc.controller.find_or_insert("Start") != plc.controller.find_or_insert("start"));
}

TEST_CASE("plc: a file which isn't there, or has no line to read, isn't loaded") {
	controller_fixture plc{"missing"};
	CHECK_FALSE(plc.controller.load(plc.missing_file()));
	CHECK_FALSE(plc.load(""));
	// a comment alone is no line
	CHECK_FALSE(plc.load("// nothing but a comment\n"));
	// any line is enough, also one which is no operation
	CHECK(plc.load("end\n"));
	CHECK(plc.load("blink lamp\n"));
}

TEST_CASE("plc: a failed load leaves the controller without a program") {
	controller_fixture plc{"unloaded"};
	REQUIRE(plc.load("ld start\nout lamp\n"));
	plc.in("start") = 1;
	plc.cycles();
	REQUIRE(plc.out("lamp") == 1);
	CHECK_FALSE(plc.controller.load(plc.missing_file()));
	// nothing drives the lamp any more
	plc.in("start") = 0;
	plc.cycles();
	CHECK(plc.out("lamp") == 1);
}

TEST_CASE("plc: an unknown command is passed over, and end doesn't end the program") {
	controller_fixture plc{"unknown"};
	REQUIRE(plc.load("ld a\nblink y\nout y\nend\nld b\nout z\n"));
	plc.in("a") = 1;
	plc.in("b") = 1;
	plc.cycles();
	CHECK(plc.out("y") == 1);
	CHECK(plc.out("z") == 1);
}

TEST_CASE("plc: another program loaded keeps the elements of the one before, and their states") {
	controller_fixture plc{"reload"};
	REQUIRE(plc.load("ld a\nout b\n"));
	plc.in("a") = 1;
	plc.cycles();
	REQUIRE(plc.out("b") == 1);
	CHECK(plc.controller.find_or_insert("a") == 1);
	CHECK(plc.controller.find_or_insert("b") == 2);

	REQUIRE(plc.load("ld b\nout d\n"));
	CHECK(plc.controller.find_or_insert("b") == 2);
	CHECK(plc.controller.find_or_insert("d") == 3);
	plc.cycles();
	CHECK(plc.out("d") == 1);
	// nothing drives b any more, it stays as the first program left it
	plc.in("a") = 0;
	plc.cycles();
	CHECK(plc.out("b") == 1);
	CHECK(plc.out("d") == 1);
}

// elements ------------------------------------------------------------------------------------------------------------

TEST_CASE("plc: an element is found by its name, or made on its first use") {
	plc::basic_controller controller;
	auto const first{controller.find_or_insert("a")};
	CHECK(first == 1);
	CHECK(controller.find_or_insert("b") == 2);
	CHECK(controller.find_or_insert("a") == 1);
	// a variable gives what was put in it
	CHECK(controller.output(first) == 0);
	controller.input(first) = 5;
	CHECK(controller.output(first) == 5);
	// it can be made with a value
	auto const preset{controller.find_or_insert("preset", plc::basic_element::type_e::variable, 1)};
	CHECK(controller.output(preset) == 1);
	// the kind asked for doesn't change an element which is there already
	CHECK(controller.find_or_insert("a", plc::basic_element::type_e::counter) == 1);
	CHECK(controller.output(first) == 5);
}

TEST_CASE("plc: a new timer without a time passes its input on, a new counter without a limit is done") {
	plc::basic_controller controller;
	auto const timer{controller.find_or_insert("delay", plc::basic_element::type_e::timer)};
	CHECK(controller.output(timer) == 0);
	controller.input(timer) = 3;
	CHECK(controller.output(timer) == 3);
	auto const counter{controller.find_or_insert("laps", plc::basic_element::type_e::counter)};
	CHECK(controller.output(counter) == 1);
	controller.input(counter) = 0;
	CHECK(controller.output(counter) == 1);
}

TEST_CASE("plc: the kind of an element the program names is told by the letters before its number") {
	controller_fixture plc{"kinds"};
	REQUIRE(plc.load("ld x\nout c1\nout c\nout counter.laps\nout cab1\nout count\nout t1 k200\nout ton2 k200\nout timer.delay k200\nout t k200\nout temp k200\n"));
	// before the first cycle: a counter has no limit yet and is done, the others are off
	for (auto const *counter : {"c1", "c", "counter.laps"}) {
		CAPTURE(counter);
		CHECK(plc.out(counter) == 1);
	}
	for (auto const *other : {"cab1", "count", "t1", "ton2", "timer.delay", "t", "temp"}) {
		CAPTURE(other);
		CHECK(plc.out(other) == 0);
	}
	// switched on, a variable is on at once and a timer waits for its time
	plc.in("x") = 1;
	plc.cycles();
	for (auto const *variable : {"cab1", "count", "temp"}) {
		CAPTURE(variable);
		CHECK(plc.out(variable) == 1);
	}
	for (auto const *timer : {"t1", "ton2", "timer.delay", "t"}) {
		CAPTURE(timer);
		CHECK(plc.out(timer) == 0);
	}
	plc.cycles(2);
	for (auto const *timer : {"t1", "ton2", "timer.delay", "t"}) {
		CAPTURE(timer);
		CHECK(plc.out(timer) == 1);
	}
}

TEST_CASE("plc: a name with a hyphen can't be loaded") {
	// the hyphen is taken for the start of the number of the element, and isn't one
	controller_fixture plc{"hyphen"};
	CHECK_THROWS_AS(plc.load("ld door-open\nout lamp\n"), std::invalid_argument);
}

// logic ---------------------------------------------------------------------------------------------------------------

TEST_CASE("plc: and, or and their inverted forms combine the elements of a rung") {
	controller_fixture plc{"logic"};
	REQUIRE(plc.load("ld a\nand b\nout both\n"
	                 "ld a\nor b\nout either\n"
	                 "ldi a\nout nota\n"
	                 "ld a\nani b\nout aonly\n"
	                 "ld a\nori b\nout aornotb\n"));
	for (auto const a : {0, 1}) {
		for (auto const b : {0, 1}) {
			CAPTURE(a);
			CAPTURE(b);
			plc.in("a") = a;
			plc.in("b") = b;
			plc.cycles();
			CHECK(plc.out("both") == (a & b));
			CHECK(plc.out("either") == (a | b));
			CHECK(plc.out("nota") == (a == 0 ? 1 : 0));
			CHECK(plc.out("aonly") == (a == 1 && b == 0 ? 1 : 0));
			CHECK(plc.out("aornotb") == (a == 1 || b == 0 ? 1 : 0));
		}
	}
}

TEST_CASE("plc: a second ld starts a block, which anb and orb combine with the one before") {
	// the elements can't be named a, b, c and d: c alone is the name of a counter
	controller_fixture plc{"blocks"};
	REQUIRE(plc.load("ld p\nand q\nld r\nand s\norb\nout anypair\n"
	                 "ld p\nor q\nld r\nor s\nanb\nout bothsides\n"));
	for (auto bits{0}; bits < 16; ++bits) {
		auto const p{bits & 1};
		auto const q{(bits >> 1) & 1};
		auto const r{(bits >> 2) & 1};
		auto const s{(bits >> 3) & 1};
		CAPTURE(bits);
		plc.in("p") = p;
		plc.in("q") = q;
		plc.in("r") = r;
		plc.in("s") = s;
		plc.cycles();
		CHECK(plc.out("anypair") == ((p & q) | (r & s)));
		CHECK(plc.out("bothsides") == ((p | q) & (r | s)));
	}
}

TEST_CASE("plc: a rung drives every element it has an out for, and the ld after them starts a new rung") {
	controller_fixture plc{"outputs"};
	REQUIRE(plc.load("ld a\nout x\nout y\nld b\nout z\n"));
	plc.in("a") = 1;
	plc.in("b") = 0;
	plc.cycles();
	CHECK(plc.out("x") == 1);
	CHECK(plc.out("y") == 1);
	CHECK(plc.out("z") == 0);
}

TEST_CASE("plc: set and rst latch an element") {
	controller_fixture plc{"latch"};
	REQUIRE(plc.load("ld start\nset run\nld stop\nrst run\nld run\nout lamp\n"));
	plc.cycles();
	CHECK(plc.out("lamp") == 0);
	plc.pulse("start");
	CHECK(plc.out("lamp") == 1);
	plc.cycles(3);
	CHECK(plc.out("lamp") == 1);
	plc.pulse("stop");
	CHECK(plc.out("lamp") == 0);
	// with both on, the one later in the program has the last word
	plc.in("start") = 1;
	plc.in("stop") = 1;
	plc.cycles();
	CHECK(plc.out("lamp") == 0);
}

TEST_CASE("plc: an element holds itself on through its own state") {
	controller_fixture plc{"holding"};
	REQUIRE(plc.load("ld start\nor run\nani stop\nout run\n"));
	plc.pulse("start");
	CHECK(plc.out("run") == 1);
	plc.cycles(3);
	CHECK(plc.out("run") == 1);
	plc.pulse("stop");
	CHECK(plc.out("run") == 0);
}

TEST_CASE("plc: the states are numbers, combined bit by bit") {
	controller_fixture plc{"numbers"};
	REQUIRE(plc.load("ld a\nout same\nld a\nand b\nout both\nld a\nor b\nout either\nldi a\nout nota\n"));
	plc.in("a") = 2;
	plc.in("b") = 1;
	plc.cycles();
	// an out passes the number on
	CHECK(plc.out("same") == 2);
	// two elements which are both on, and no common bit
	CHECK(plc.out("both") == 0);
	CHECK(plc.out("either") == 3);
	// the inverted forms know only on and off
	CHECK(plc.out("nota") == 0);
}

TEST_CASE("plc: an operation with nothing to work on is passed over") {
	controller_fixture plc{"empty"};
	REQUIRE(plc.load("and a\nani a\nor a\nori a\nanb\norb\nout y\nset y\nrst y\nld b\nanb\norb\nout z\n"));
	plc.in("a") = 1;
	plc.in("b") = 1;
	plc.in("y") = 7;
	plc.cycles();
	// nothing was loaded for the first rung to give out
	CHECK(plc.out("y") == 7);
	// a single block has nothing to be combined with
	CHECK(plc.out("z") == 1);
}

// timers --------------------------------------------------------------------------------------------------------------

TEST_CASE("plc: a timer is on once its input was on for its time") {
	controller_fixture plc{"timer"};
	REQUIRE(plc.load("ld start\nout t1 k300\nld t1\nout lamp\n"));
	plc.in("start") = 1;
	// the first cycle switches it on, the three after it count 300 ms
	plc.cycles(3);
	CHECK(plc.out("lamp") == 0);
	plc.cycles();
	CHECK(plc.out("lamp") == 1);
	plc.cycles(10);
	CHECK(plc.out("lamp") == 1);
	// it goes off with its input, in the same cycle
	plc.in("start") = 0;
	plc.cycles();
	CHECK(plc.out("lamp") == 0);
	// and counts from the start the next time
	plc.in("start") = 1;
	plc.cycles(3);
	CHECK(plc.out("lamp") == 0);
	plc.cycles();
	CHECK(plc.out("lamp") == 1);
}

TEST_CASE("plc: the time of a timer is the number in its parameter, with letters before it or without") {
	controller_fixture plc{"timer_time"};
	// a semicolon parts the words of an operation like a space does
	REQUIRE(plc.load("ld start\nout t1 200\nout t2\nout t3;k200\nld t1\nout slow\nld t2\nout fast\nld t3\nout third\n"));
	plc.in("start") = 1;
	plc.cycles();
	// without a time it's on with its input
	CHECK(plc.out("fast") == 1);
	CHECK(plc.out("slow") == 0);
	CHECK(plc.out("third") == 0);
	plc.cycles();
	CHECK(plc.out("slow") == 0);
	CHECK(plc.out("third") == 0);
	plc.cycles();
	CHECK(plc.out("slow") == 1);
	CHECK(plc.out("third") == 1);
}

TEST_CASE("plc: a timer passes the state of its input on") {
	controller_fixture plc{"timer_state"};
	REQUIRE(plc.load("ld start\nout t1 k100\nld t1\nout lamp\n"));
	plc.in("start") = 5;
	plc.cycles(2);
	CHECK(plc.out("lamp") == 5);
}

TEST_CASE("plc: the timers go on in steps of a tenth of a second, however long the updates are") {
	SUBCASE("short updates add up to a step") {
		controller_fixture plc{"timer_short"};
		REQUIRE(plc.load("ld start\nout t1 k100\nld t1\nout lamp\n"));
		plc.in("start") = 1;
		plc.controller.update(0.04);
		CHECK(plc.out("lamp") == 0);
		plc.controller.update(0.04);
		CHECK(plc.out("lamp") == 0);
		plc.controller.update(0.04);
		CHECK(plc.out("lamp") == 1);
	}
	SUBCASE("a long update is many steps") {
		controller_fixture plc{"timer_long"};
		REQUIRE(plc.load("ld start\nout t1 k900\nld t1\nout lamp\n"));
		plc.in("start") = 1;
		plc.controller.update(0.1);
		CHECK(plc.out("lamp") == 0);
		plc.controller.update(1.0);
		CHECK(plc.out("lamp") == 1);
	}
}

// counters ------------------------------------------------------------------------------------------------------------

TEST_CASE("plc: a counter is done once its input went on as many times as its limit") {
	controller_fixture plc{"counter"};
	REQUIRE(plc.load("ld pulse\nout c1 k3\nld c1\nout done\nld clear\nrst c1\n"));
	plc.cycles();
	CHECK(plc.out("done") == 0);
	plc.pulse("pulse");
	plc.pulse("pulse");
	CHECK(plc.out("done") == 0);
	// an input held on counts once
	plc.in("pulse") = 1;
	plc.cycles(5);
	CHECK(plc.out("done") == 1);
	plc.in("pulse") = 0;
	plc.cycles();
	CHECK(plc.out("done") == 1);
	// more of them change nothing
	plc.pulse("pulse");
	CHECK(plc.out("done") == 1);

	// rst starts the count again
	plc.in("clear") = 1;
	plc.cycles(2);
	plc.in("clear") = 0;
	plc.cycles();
	CHECK(plc.out("done") == 0);
	plc.pulse("pulse");
	plc.pulse("pulse");
	CHECK(plc.out("done") == 0);
	plc.pulse("pulse");
	CHECK(plc.out("done") == 1);
}

TEST_CASE("plc: set on a counter makes it done for as long as it's held") {
	controller_fixture plc{"counter_set"};
	REQUIRE(plc.load("ld pulse\nout c1 k2\nld force\nset c1\nld c1\nout done\n"));
	plc.cycles();
	CHECK(plc.out("done") == 0);
	// the limit becomes the count the counter has
	plc.in("force") = 1;
	plc.cycles();
	CHECK(plc.out("done") == 1);
	plc.cycles(3);
	CHECK(plc.out("done") == 1);
	// the out of the next cycle puts the limit of the program back
	plc.in("force") = 0;
	plc.cycles();
	CHECK(plc.out("done") == 0);
}

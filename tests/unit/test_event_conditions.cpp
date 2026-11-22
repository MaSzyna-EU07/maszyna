#include "stdafx.h"
#include "doctest.h"

#include "utilities/comparison.h"
#include "utilities/parser.h"
#include "world/Event.h"

#include <sstream>

// the condition block of the multiple and updatevalues events (world/eventconditions.cpp): the scenery editor writes
// the conditions of the events back with export_as_text(), and what it writes has to read back as the same conditions

namespace {

// the conditions are a part of the event, open to its kinds only
struct event_kind : basic_event {
	using basic_event::event_conditions;
};
using conditions = event_kind::event_conditions;

// the conditions of an event text, read the way the events read them: the parser stands on the word 'condition'
conditions read(std::string const &Text) {
	cParser parser(Text, cParser::buffer_TEXT);
	parser.getTokens();
	REQUIRE(parser.peek() == "condition");
	conditions result;
	result.deserialize(parser, "test");
	// the reading stops at the next keyword of the event and leaves it for the event
	CHECK(parser.peek() == "endevent");
	return result;
}

std::string write(conditions const &Conditions) {
	std::ostringstream output;
	Conditions.export_as_text(output);
	return output.str();
}

// the conditions written and read again
conditions write_and_read(conditions const &Conditions) {
	return read(write(Conditions) + "endevent");
}

void check_same(conditions const &Read, conditions const &Expected) {
	CHECK(Read.flags == Expected.flags);
	CHECK(Read.probability == Expected.probability);
	CHECK(Read.memcompare_pass == Expected.memcompare_pass);
	CHECK(Read.memcompare_text == Expected.memcompare_text);
	CHECK(Read.memcompare_text_operator == Expected.memcompare_text_operator);
	CHECK(Read.memcompare_value1 == Expected.memcompare_value1);
	CHECK(Read.memcompare_value1_operator == Expected.memcompare_value1_operator);
	CHECK(Read.memcompare_value2 == Expected.memcompare_value2);
	CHECK(Read.memcompare_value2_operator == Expected.memcompare_value2_operator);
}

comparison_operator const operators[]{comparison_operator::equal, comparison_operator::not_equal, comparison_operator::lt,
                                      comparison_operator::gt, comparison_operator::lt_eq, comparison_operator::gt_eq};
comparison_pass const passes[]{comparison_pass::all, comparison_pass::any, comparison_pass::none};

} // namespace

TEST_CASE("event conditions: each comparison operator and pass is written as the word that reads back as it") {
	for (auto const op : operators) {
		CAPTURE(to_string(op));
		CHECK(comparison_operator_from_string(to_string(op)) == op);
	}
	CHECK(to_string(comparison_operator::gt_eq) == ">=");
	for (auto const pass : passes) {
		CAPTURE(to_string(pass));
		CHECK(comparison_pass_from_string(to_string(pass)) == pass);
	}
}

TEST_CASE("event conditions: memcompareex is written as the pass, then the operator before each compared value") {
	auto const conditions{read("condition memcompareex any != Blokada >= 1 < 2.5 endevent")};
	CHECK(conditions.memcompare_pass == comparison_pass::any);
	CHECK(conditions.memcompare_text == "Blokada");
	CHECK(conditions.memcompare_text_operator == comparison_operator::not_equal);
	CHECK(conditions.memcompare_value1_operator == comparison_operator::gt_eq);
	CHECK(conditions.memcompare_value2_operator == comparison_operator::lt);
	CHECK(write(conditions) == "condition memcompareex any != Blokada >= 1.000000 < 2.500000 ");
	check_same(write_and_read(conditions), conditions);
}

TEST_CASE("event conditions: memcompareex reads back the same for every pass, operator and left out value") {
	for (auto const pass : passes) {
		for (auto const op : operators) {
			for (unsigned int used{0}; used < 8; ++used) {
				std::string text{"condition memcompareex " + to_string(pass)};
				text += (used & 1) ? " " + to_string(op) + " Zajety_tor" : std::string{" *"};
				text += (used & 2) ? " " + to_string(op) + " -3.25" : std::string{" *"};
				text += (used & 4) ? " " + to_string(op) + " 1250" : std::string{" *"};
				text += " endevent";
				CAPTURE(text);
				auto const conditions{read(text)};
				CHECK(conditions.flags == used);
				if (used == 0) {
					// nothing compared is no condition, and no condition is written
					CHECK(write(conditions).empty());
					continue;
				}
				check_same(write_and_read(conditions), conditions);
			}
		}
	}
}

TEST_CASE("event conditions: the old memcompare is written as memcompareex all with == and reads back the same") {
	auto const conditions{read("condition memcompare Wolny * 3 endevent")};
	CHECK(write(conditions) == "condition memcompareex all == Wolny * == 3.000000 ");
	check_same(write_and_read(conditions), conditions);

	auto const all{read("condition memcompare Wolny 1 0 endevent")};
	check_same(write_and_read(all), all);
}

TEST_CASE("event conditions: the track and probability conditions read back along with the comparison") {
	auto const conditions{read("condition trackoccupied trackfree probability 0.25 memcompareex none <= 3 * > -1.5 endevent")};
	CHECK(conditions.flags == (basic_event::flags::track_busy | basic_event::flags::track_free | basic_event::flags::probability |
	                           basic_event::flags::text | basic_event::flags::value2));
	check_same(write_and_read(conditions), conditions);

	auto const probability{read("condition probability 0.5 endevent")};
	CHECK(write(probability) == "condition probability 0.5 ");
	check_same(write_and_read(probability), probability);
}

TEST_CASE("event conditions: an event without conditions writes none") {
	CHECK(write(conditions{}).empty());
}

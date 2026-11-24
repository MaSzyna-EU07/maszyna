// the generator of the tracks of the simulator from the lines of the track geometry (editor/layout, editor-tracks-domain.md §5): the
// Bezier pieces within a millimetre of the elements they follow, the clothoid, the Bloss curve, the cubic parabola, the arcs from 190 m to
// 5 km, the vertical curves; the tracks written and read back by the game's own TTrack::Load(): the ends at the nodes the same to the bit
// on both sides, the curve as the simulator draws it within a millimetre of the line, the radius of the element; the names from the ids

#include "stdafx.h"
#include "doctest.h"
#include "layout_fixture.h"
#include "track_text.h"
#include "editor/layout/layoutGenerator.hpp"
#include "editor/project/projectFile.hpp"

#include <algorithm>
#include <set>
#include <sstream>

namespace {

using namespace test::layout_fixture;
using layout::line_id;

double constexpr kMillimetre{0.001};

// the farthest the line lies from the curve drawn: each of its points against the nearest point of the curve, searched on its own
// (the other way round from layout::deviation, which sets the points of the curve square onto the line)
double line_from_curve(layout::solved_line const &Line, layout::drawn_curve const &Curve, double const From, double const To) {
	double worst{0.0};
	int constexpr samples{40};
	for (int i = 0; i <= samples; ++i) {
		auto const s{From + (To - From) * i / samples};
		auto const target{Line.point(s)};
		// coarse, then golden section around the best
		int constexpr coarse{400};
		double best{0.0};
		double nearest{1e9};
		for (int k = 0; k <= coarse; ++k) {
			auto const t{static_cast<double>(k) / coarse};
			auto const d{glm::distance(Curve.point(t), target)};
			if (d < nearest) {
				nearest = d;
				best = t;
			}
		}
		double low{std::max(0.0, best - 1.0 / coarse)};
		double high{std::min(1.0, best + 1.0 / coarse)};
		for (int k = 0; k < 60; ++k) {
			auto const a{high - (high - low) * 0.618};
			auto const b{low + (high - low) * 0.618};
			if (glm::distance(Curve.point(a), target) < glm::distance(Curve.point(b), target)) {
				high = b;
			}
			else {
				low = a;
			}
		}
		worst = std::max(worst, glm::distance(Curve.point(0.5 * (low + high)), target));
	}
	return worst;
}

// the radius of the pieces: of the sharpest curvature of the elements over it
float element_radius(layout::solved_line const &Line, double const From, double const To) {
	double sharpest{0.0};
	for (int i = 0; i <= 64; ++i) {
		sharpest = std::max(sharpest, std::abs(Line.curvature(std::clamp(From + (To - From) * i / 64.0, From + 1e-9, To - 1e-9))));
	}
	return sharpest > 1e-9 ? static_cast<float>(1.0 / sharpest) : 0.f;
}

layout::model laid(layout::create_line const &Line) {
	layout::model model;
	auto const made{model.execute(Line)};
	CAPTURE(reasons_of(made));
	REQUIRE(made.done);
	return model;
}

} // namespace

TEST_CASE("layout generator: the pieces of clothoids, Bloss curves and cubic parabolas, arcs of 190 m to 5 km and vertical curves are within a millimetre") {
	struct sample {
		char const *name;
		layout::create_line line;
	};
	std::vector<sample> samples;
	for (auto const shape : {layout::transition_shape::clothoid, layout::transition_shape::bloss, layout::transition_shape::cubic_parabola}) {
		samples.push_back({shape == layout::transition_shape::clothoid ? "clothoid" : shape == layout::transition_shape::bloss ? "bloss" : "cubic parabola", three_curves(shape)});
	}
	// tight and wide arcs: a slow siding round R 190 m, a gentle bend of R 5 km, vertical curves on the straights
	auto tight{three_curves()};
	tight.category.vmax = 40.0;
	tight.category.track = norms::track_kind::side;
	tight.end = {420.0, 100.0, 420.0};
	tight.vertices = {curve({300.0, 0.0}, 190.0)};
	tight.vertices.front().cant = 40.0;
	tight.vertices.front().transition_in = tight.vertices.front().transition_out = 30.0;
	tight.grades.clear();
	samples.push_back({"R 190", tight});
	auto wide{three_curves()};
	wide.end = {3000.0, 100.0, 300.0};
	wide.vertices = {curve({1500.0, 0.0}, 5000.0)};
	wide.grades = {{700.0, 106.0, 5000.0}, {2200.0, 98.0, 8000.0}};
	samples.push_back({"R 5000 and vertical curves", wide});
	for (auto const &[name, line] : samples) {
		CAPTURE(name);
		auto const model{laid(line)};
		auto const id{model.state().lines().begin()->first};
		auto const &solved{model.state().find(id)->solved};
		auto const made{layout::generate(model.state(), id)};
		CHECK(made.errors.empty());
		REQUIRE_FALSE(made.pieces.empty());
		double worst{0.0};
		double covered{0.0};
		for (auto const &piece : made.pieces) {
			CAPTURE(piece.name);
			auto const curve{layout::drawn(piece.path)};
			auto const there{layout::deviation(solved, curve, piece.from, piece.to)};
			auto const back{line_from_curve(solved, curve, piece.from, piece.to)};
			CHECK(there < kMillimetre);
			CHECK(back < kMillimetre);
			CHECK(piece.deviation == doctest::Approx(there));
			worst = std::max({worst, there, back});
			covered += piece.to - piece.from;
			CHECK(piece.to - piece.from <= geometry::longest_path + 1e-9);
		}
		CHECK(covered == doctest::Approx(solved.length).epsilon(1e-12));
		MESSAGE(name << ": " << made.pieces.size() << " pieces, the farthest " << worst * 1000.0 << " mm from the line");
	}
}

TEST_CASE("layout generator: the tracks read back by TTrack::Load end at the nodes to the bit, within a millimetre of the line, with its radii") {
	test::track_reading const reading;
	layout::model model;
	REQUIRE(model.execute(three_curves()).done);
	auto const first{model.state().lines().begin()->first};
	// split on a straight and inside the arc of the second curve: nodes holding two ends, one of them in a curve
	REQUIRE(model.execute(layout::split_line{first, 700.0}).done);
	auto const second{std::prev(model.state().lines().end())->first};
	double inside{0.0};
	for (auto const &e : model.state().find(second)->solved.elements) {
		if (e.role == layout::element_role::arc) {
			inside = e.chainage + 0.5 * e.length;
			break;
		}
	}
	REQUIRE(model.execute(layout::split_line{second, inside}).done);
	auto const &state{model.state()};
	REQUIRE(state.lines().size() == 3);
	std::map<line_id, std::vector<std::unique_ptr<TTrack>>> tracks;
	std::set<std::string> names;
	for (auto const &[id, line] : state.lines()) {
		CAPTURE(layout::number(id));
		auto const made{layout::generate(state, id)};
		REQUIRE(made.errors.empty());
		for (auto const &piece : made.pieces) {
			CAPTURE(piece.text);
			auto track{test::read_track(piece.text)};
			REQUIRE(track->m_paths.size() == 1);
			auto const &path{track->m_paths.front()};
			// as written
			CHECK(path.points == piece.path.points);
			CHECK(path.rolls == piece.path.rolls);
			CHECK(track->name() == piece.name);
			CHECK(names.insert(piece.name).second);
			// as the simulator draws it, within a millimetre of the line
			auto const curve{layout::drawn(path)};
			CHECK(layout::deviation(line.solved, curve, piece.from, piece.to) < kMillimetre);
			CHECK(line_from_curve(line.solved, curve, piece.from, piece.to) < kMillimetre);
			// the radius of the element, the arcs to the centimetre
			auto const expected{element_radius(line.solved, piece.from, piece.to)};
			CHECK(path.radius == doctest::Approx(expected).epsilon(1e-6));
			auto const &e{line.solved.elements[line.solved.element_at(0.5 * (piece.from + piece.to))]};
			if (e.kind == layout::element_kind::arc && e.chainage <= piece.from + 1e-9 && e.end_chainage() >= piece.to - 1e-9) {
				CHECK(std::abs(path.radius - 1.0 / std::abs(e.curvature_start)) < 0.01);
			}
			if (e.kind == layout::element_kind::straight && e.chainage <= piece.from + 1e-9 && e.end_chainage() >= piece.to - 1e-9) {
				CHECK(path.radius == 0.f);
			}
			CHECK(track->VelocityGet() == doctest::Approx(80.0));
			tracks[id].push_back(std::move(track));
		}
		// the pieces end where the next ones start, to the bit
		auto const &pieces{tracks[id]};
		for (std::size_t i = 1; i < pieces.size(); ++i) {
			CHECK(pieces[i - 1]->m_paths.front().points[segment_data::point::end] == pieces[i]->m_paths.front().points[segment_data::point::start]);
			CHECK(pieces[i - 1]->m_paths.front().rolls[1] == pieces[i]->m_paths.front().rolls[0]);
		}
		// and the line starts and ends at its nodes, to the bit
		CHECK(pieces.front()->m_paths.front().points[segment_data::point::start] == state.find(line.start)->position);
		CHECK(pieces.back()->m_paths.front().points[segment_data::point::end] == state.find(line.end)->position);
	}
	// at each node holding two ends the tracks on both sides have the same numbers: the place, the roll, the heading of the control
	// vectors
	for (auto const &[id, node] : state.nodes()) {
		if (node.ends.size() != 2) {
			continue;
		}
		CAPTURE(layout::number(id));
		auto const &before{tracks[node.ends[0].line].back()->m_paths.front()};
		auto const &after{tracks[node.ends[1].line].front()->m_paths.front()};
		REQUIRE(node.ends[0].end == layout::line_end::end);
		REQUIRE(node.ends[1].end == layout::line_end::start);
		CHECK(before.points[segment_data::point::end] == after.points[segment_data::point::start]);
		CHECK(before.rolls[1] == after.rolls[0]);
		auto const out{glm::normalize(geometry::plan_of(-before.points[segment_data::point::control2]))};
		auto const on{glm::normalize(geometry::plan_of(after.points[segment_data::point::control1]))};
		CHECK(std::abs(geometry::signed_angle(out, on)) < 1e-12);
	}
	// the node inside the arc has its cant on both sides
	auto const curved{std::ranges::find_if(state.nodes(), [](auto const &Each) { return Each.second.curvature != 0.0; })};
	REQUIRE(curved != state.nodes().end());
	CHECK(tracks[curved->second.ends[1].line].front()->m_paths.front().rolls[0] != 0.f);
}

TEST_CASE("layout generator: a line run the other way through its nodes meets the tracks on both sides with the same numbers") {
	test::track_reading const reading;
	layout::model model;
	REQUIRE(model.execute(three_curves()).done);
	auto const first{model.state().lines().begin()->second};
	auto const at = [&](layout::line const &Line, unsigned const Vertex) {
		for (auto const &e : Line.solved.elements) {
			if (e.role == layout::element_role::straight && e.vertex == Vertex) {
				return e.chainage + 0.5 * e.length;
			}
		}
		return 0.0;
	};
	// a middle part between two straights taken out and laid again from its end back to its start: the line runs against the
	// headings of its nodes
	auto const cut{model.execute(layout::split_line{first.id, at(first, layout::number(first.shape.vertices[1].id))})};
	REQUIRE(cut.done);
	auto const &middle{*model.state().find(cut.lines.front())};
	auto const tail{model.execute(layout::split_line{middle.id, at(middle, 0)})};
	REQUIRE(tail.done);
	auto const from{model.state().find(cut.lines.front())->start};
	auto const to{model.state().find(cut.lines.front())->end};
	REQUIRE(model.execute(layout::delete_line{cut.lines.front()}).done);
	auto const back{model.execute(layout::join_ends{to, from})};
	CAPTURE(reasons_of(back));
	REQUIRE(back.done);
	auto const &state{model.state()};
	check_invariants(state);
	auto const reversed{back.lines.front()};
	auto const made{layout::generate(state, reversed)};
	REQUIRE(made.errors.empty());
	auto const loaded_first{test::read_track(made.pieces.front().text)};
	auto const loaded_last{test::read_track(made.pieces.back().text)};
	for (auto const node : {to, from}) {
		CAPTURE(layout::number(node));
		auto const &held{state.find(node)->ends};
		REQUIRE(held.size() == 2);
		auto const &other{held[0].line == reversed ? held[1] : held[0]};
		auto const &mine{held[0].line == reversed ? held[0] : held[1]};
		CHECK(mine.sense == -1);
		auto const theirs{layout::generate(state, other.line)};
		REQUIRE(theirs.errors.empty());
		auto const their_track{test::read_track(other.end == layout::line_end::start ? theirs.pieces.front().text : theirs.pieces.back().text)};
		auto const &their_path{their_track->m_paths.front()};
		auto const &my_path{(mine.end == layout::line_end::start ? loaded_first : loaded_last)->m_paths.front()};
		auto const their_index{other.end == layout::line_end::start ? segment_data::point::start : segment_data::point::end};
		auto const my_index{mine.end == layout::line_end::start ? segment_data::point::start : segment_data::point::end};
		CHECK(their_path.points[their_index] == my_path.points[my_index]);
		CHECK(their_path.points[their_index] == state.find(node)->position);
		// the roll of each as it runs: the same cant leaning the same way, of the other sign run the other way
		auto const their_roll{their_path.rolls[other.end == layout::line_end::start ? 0 : 1]};
		auto const my_roll{my_path.rolls[mine.end == layout::line_end::start ? 0 : 1]};
		CHECK(my_roll == -their_roll);
	}
}

TEST_CASE("layout generator: the names come from the ids of the line and its vertices, a change of one curve leaves the others' names") {
	layout::model model;
	REQUIRE(model.execute(three_curves()).done);
	auto const &line{model.state().lines().begin()->second};
	auto const id{line.id};
	auto const before{layout::generate(model.state(), id)};
	REQUIRE(before.errors.empty());
	auto const prefix{"l" + std::to_string(layout::number(id)) + "_"};
	for (auto const &piece : before.pieces) {
		CHECK(piece.name.starts_with(prefix));
	}
	// a new cant and transitions of the last curve: the pieces before its straight keep their names
	auto const last{line.shape.vertices.back().id};
	auto const tightened{layout::preferred_curve(norms::profile_id::plk, line_class(), 700.0)};
	REQUIRE(model.execute(layout::set_vertex{id, last, 700.0, tightened.transition, tightened.transition, tightened.cant}).done);
	auto const after{layout::generate(model.state(), id)};
	REQUIRE(after.errors.empty());
	auto const key{"s" + std::to_string(layout::number(last))};
	std::vector<std::string> kept_before;
	std::vector<std::string> kept_after;
	for (auto const *made : {&before, &after}) {
		auto &kept{made == &before ? kept_before : kept_after};
		for (auto const &piece : made->pieces) {
			if (piece.name.find(key) != std::string::npos) {
				break;
			}
			kept.push_back(piece.name);
		}
	}
	CHECK_FALSE(kept_before.empty());
	CHECK(kept_before == kept_after);
	// the same network made twice gives the same text
	CHECK(layout::generate(model.state(), id).text() == after.text());
}

TEST_CASE("layout generator: the joints of old tracks as hard points of the pieces, their names, attributes and the way they ran kept") {
	auto const model{laid(three_curves())};
	auto const id{model.state().lines().begin()->first};
	auto network{model.state()};
	auto &line{const_cast<layout::line &>(*network.find(id))};
	auto const length{line.solved.length};
	// three old tracks: one of a name with its events and circuit, one of none, one of a name which ran against the line
	layout::old_track first{"tor_a", "1.435 0.25 25.0 20 0 flat vis rail 4 bed 0.2 0.5 1.1", "velocity 60 event1 ev_a isolated obw_a", false, false};
	layout::old_track second{"", "", "velocity 70 event2 ev_b", false, false};
	layout::old_track third{"tor_c", "", "velocity 80 event1 ev_c", true, false};
	line.hard_splits = {{0.0, first}, {350.0, second}, {length - 100.0, third}};
	auto const made{layout::generate(network, id)};
	REQUIRE(made.errors.empty());
	// the pieces end at the joints
	auto const ends_at = [&](double const Chainage) { return std::ranges::any_of(made.pieces, [&](layout::generated_piece const &Piece) { return std::abs(Piece.to - Chainage) < 1e-9; }); };
	CHECK(ends_at(350.0));
	CHECK(ends_at(length - 100.0));
	std::map<std::string, std::string> texts;
	for (auto const &piece : made.pieces) {
		CHECK(texts.emplace(piece.name, piece.text).second);
	}
	// the first old track: its name on its first piece with all of its attributes, the others numbered on without its events
	REQUIRE(texts.contains("tor_a"));
	CHECK(texts["tor_a"].find("1.435 0.25 25.0 20 0 flat vis rail 4 bed 0.2 0.5 1.1") != std::string::npos);
	CHECK(texts["tor_a"].ends_with(" velocity 60 event1 ev_a isolated obw_a endtrack\n"));
	for (auto const &[name, text] : texts) {
		if (name.starts_with("tor_a_")) {
			CHECK(text.ends_with(" velocity 60 isolated obw_a endtrack\n"));
		}
	}
	// of no name: named by the generator, its events on the first piece only
	auto const events_b{std::ranges::count_if(texts, [](auto const &Each) { return Each.second.find("event2 ev_b") != std::string::npos; })};
	CHECK(events_b == 1);
	// the last one ran against the line: its piece written from the end of the line, the named one where it started
	REQUIRE(texts.contains("tor_c"));
	auto const named_c{std::ranges::find_if(made.pieces, [](layout::generated_piece const &Piece) { return Piece.name == "tor_c"; })};
	CHECK(std::abs(named_c->to - length) < 1e-9);
	CHECK(named_c->path.points[segment_data::point::start] == network.find(line.end)->position);
	// written and read back as the line has them
	project::document project;
	project.network = network;
	auto const read{project::read(project::write(project))};
	REQUIRE(read.project.has_value());
	CHECK(read.project->network.find(id)->hard_splits == line.hard_splits);
}

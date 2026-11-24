// The takeover of the old tracks of a scenery (editor-tracks-domain.md §10, editor-tracks-tests.md §4 "E6", G): data/takeover/old.scn,
// written by make_old.py, read with no scene; its plain tracks taken over by one command of the model, the lines within the tolerance of
// the old tracks measured densely, the switch left as it is holding the lines at its legs; the tracks made again ending at the old joints
// with the old names, events and circuits, also after the line was changed; the anchors of the lineside objects with their confidence.

#include "stdafx.h"
#include "doctest.h"
#include "editor/editorGeometry.hpp"
#include "editor/layout/layoutAnchors.hpp"
#include "editor/layout/layoutGenerator.hpp"
#include "editor/layout/layoutLegacy.hpp"
#include "editor/project/legacyScenery.hpp"
#include "editor/project/projectFile.hpp"

#include <algorithm>
#include <filesystem>
#include <format>
#include <map>

namespace {

std::filesystem::path folder() {
	return std::filesystem::path{EU07_UNIT_TESTS_DATA} / "takeover";
}

project::legacy_scenery old_scenery() {
	auto result{project::read_legacy(folder(), "old.scn")};
	REQUIRE(result.problems.empty());
	return result;
}

std::vector<std::size_t> plain_tracks(project::legacy_scenery const &Scenery) {
	std::vector<std::size_t> result;
	for (std::size_t i = 0; i < Scenery.tracks.size(); ++i) {
		result.push_back(i);
	}
	return result;
}

layout::taken_line const *line_with(layout::takeover_plan const &Plan, project::legacy_scenery const &Scenery, std::string const &Name) {
	for (auto const &line : Plan.lines) {
		for (auto const &link : line.chain.links) {
			if (Scenery.tracks[link.track].name == Name) {
				return &line;
			}
		}
	}
	return nullptr;
}

std::string first_reason(layout::outcome const &Outcome) {
	return Outcome.reasons.empty() ? std::string{} : Outcome.reasons.front();
}

std::string deviation_text(layout::measured_deviation const &Deviation) {
	return std::format("plan {:.4f} m, height {:.4f} m, cant {:.2f} mm", Deviation.plan, Deviation.height, Deviation.cant);
}

// the line of the network the old track became
layout::line_id line_of(layout::network const &Network, std::string const &Name) {
	for (auto const &[id, line] : Network.lines()) {
		if (std::ranges::any_of(line.hard_splits, [&](layout::hard_split const &Split) { return Split.track.name == Name; })) {
			return id;
		}
	}
	return layout::line_id::none;
}

} // namespace

TEST_CASE("takeover scenery: the old tracks read with no scene, a switch among them, the objects along them") {
	auto const scenery{old_scenery()};
	auto const switches{std::ranges::count_if(scenery.tracks, [](layout::legacy_track const &Track) { return Track.is_switch; })};
	CHECK(switches == 1);
	CHECK(scenery.tracks.size() == 32);
	auto const &a3{*std::ranges::find_if(scenery.tracks, [](layout::legacy_track const &Track) { return Track.name == "a_3"; })};
	CHECK(a3.attributes == "velocity 100 event1 sem_a_set");
	CHECK(a3.surface == "1.435 0.25 25.0 20 0 flat vis rail_screw_used1 4 tpbps-new2 0.2 0.5 1.1");
	CHECK(a3.velocity() == 100.0);
	// the signal named by the events of a_2 (lights) and its memory cell by those of a_3 (updatevalues)
	auto const object = [&](std::string const &Name) { return *std::ranges::find_if(scenery.objects, [&](layout::lineside_object const &Each) { return Each.name == Name; }); };
	CHECK(object("sem_a").tracks == std::vector<std::string>{"a_2"});
	CHECK(object("sem_a").group == infra::category::signal);
	CHECK(object("sem_a_mem").tracks == std::vector<std::string>{"a_3"});
	CHECK(object("slup_1").group == infra::category::pole);
	auto const wires{std::ranges::count_if(scenery.objects, [](layout::lineside_object const &Each) { return Each.kind == "traction"; })};
	CHECK(wires > 10);
}

TEST_CASE("takeover scenery: the plain tracks taken over as three lines within the tolerance, the switch holding them, one step to undo") {
	auto const scenery{old_scenery()};
	auto const plan{layout::plan_takeover(scenery.tracks, plain_tracks(scenery))};
	REQUIRE(plan.lines.size() == 3);
	CAPTURE(deviation_text(plan.worst));
	CHECK(plan.silent());
	CHECK(plan.worst.within(plan.tolerance));
	REQUIRE(plan.notes.size() == 1);
	CHECK(plan.notes.front().find("zwr_1") != std::string::npos);
	// line a: the curve of R 500 m, its clothoids of 60 m and 80 mm read back; held at the switch, in its heading
	auto const *a{line_with(plan, scenery, "a_1")};
	REQUIRE(a != nullptr);
	CHECK(a->chain.links.size() == 26); // NOSONAR: REQUIRE above ends the test on none
	CHECK(a->chain.end.kind == layout::legacy_end_kind::held);
	CHECK(a->chain.start.kind == layout::legacy_end_kind::free);
	REQUIRE(a->fit.shape.vertices.size() == 1);
	auto const &curve{a->fit.shape.vertices.front()};
	CHECK(curve.radius == doctest::Approx(500.0).epsilon(0.01));
	CHECK(curve.transition_in == doctest::Approx(60.0).epsilon(0.05));
	CHECK(curve.transition_out == doctest::Approx(60.0).epsilon(0.05));
	CHECK(curve.cant == doctest::Approx(80.0).epsilon(0.05));
	// the grade line: level, then 5 per mille from the break at 300 m with no vertical curve, so the sharpest radius which keeps within
	REQUIRE_FALSE(a->fit.shape.grades.empty());
	CHECK(a->category.vmax == 100.0);
	// lines b and c start at the legs of the switch, held there
	auto const *b{line_with(plan, scenery, "b_1")};
	auto const *c{line_with(plan, scenery, "c_1")};
	REQUIRE(b != nullptr);
	REQUIRE(c != nullptr);
	CHECK(b->chain.start.kind == layout::legacy_end_kind::held);
	CHECK(c->chain.start.kind == layout::legacy_end_kind::held);
	CHECK(b->category.vmax == 60.0);

	layout::model model;
	auto const taken{layout::take_over_tracks(model, plan)};
	REQUIRE_MESSAGE(taken.made.done, first_reason(taken.made));
	CHECK(taken.silent);
	CHECK(taken.deviations.size() == 3);
	CHECK(model.state().lines().size() == 3);
	CHECK(model.state().nodes().size() == 6);
	CHECK(model.undoable() == 1);
	// the end of line a at the start of the switch, in the heading of its main path
	auto const &network{model.state()};
	auto const *line_a{network.find(line_of(network, "a_1"))};
	REQUIRE(line_a != nullptr);
	auto const switch_track{std::ranges::find_if(scenery.tracks, [](layout::legacy_track const &Track) { return Track.is_switch; })};
	auto const &main_path{switch_track->paths.front()};
	CHECK(network.find(line_a->end)->position == main_path.points[segment_data::point::start]);
	auto const along{glm::normalize(geometry::plan_of(main_path.points[segment_data::point::end] - main_path.points[segment_data::point::start]))};
	CHECK(std::abs(geometry::signed_angle(line_a->solved.end.heading, along)) < 1e-9);
	// undone whole, done again the same
	auto const after{model.state()};
	REQUIRE(model.undo());
	CHECK(model.state().lines().empty());
	CHECK(model.state().nodes().empty());
	REQUIRE(model.redo());
	CHECK(model.state().same(after));
}

TEST_CASE("takeover scenery: the tracks made again end at the old joints with the old names, events and circuits, a change of the line kept them") {
	auto const scenery{old_scenery()};
	auto const plan{layout::plan_takeover(scenery.tracks, plain_tracks(scenery))};
	layout::model model;
	REQUIRE(layout::take_over_tracks(model, plan).made.done);
	auto const check_made = [&](layout::network const &Network) {
		std::map<std::string, std::size_t> names;
		std::vector<std::string> errors;
		for (auto const &[id, line] : Network.lines()) {
			auto const made{layout::generate(Network, id)};
			errors.insert(errors.end(), made.errors.begin(), made.errors.end());
			for (auto const &piece : made.pieces) {
				++names[piece.name];
			}
			// each old track's stretch begins at a piece's end
			for (auto const &split : line.hard_splits) {
				auto const at_joint{std::ranges::any_of(made.pieces, [&](layout::generated_piece const &Piece) { return std::abs(Piece.from - split.chainage) < 1e-9; })};
				CHECK(at_joint);
			}
		}
		CHECK(errors.empty());
		// each named old track once, the events and the circuit on its piece
		for (auto const &track : scenery.tracks) {
			if (track.is_switch || track.name == "none") {
				continue;
			}
			CAPTURE(track.name);
			CHECK(names[track.name] == 1);
		}
		auto const text{layout::generate_all(Network, {}, errors)};
		CHECK(text.find("node -1 0 a_3 track normal") != std::string::npos);
		CHECK(text.find(" velocity 100 event1 sem_a_set endtrack") != std::string::npos);
		CHECK(text.find(" velocity 100 event2 sem_a_light endtrack") != std::string::npos);
		CHECK(text.find("event1 sem_a_set", text.find("event1 sem_a_set") + 1) == std::string::npos);
		CHECK(std::ranges::count(names, std::string{"none"}, &std::map<std::string, std::size_t>::value_type::first) == 0);
	};
	check_made(model.state());
	// the old track a_3 ran against the line: its piece starts where it started
	auto const &network{model.state()};
	auto const id{line_of(network, "a_1")};
	auto const made{layout::generate(network, id)};
	auto const &a3{*std::ranges::find_if(scenery.tracks, [](layout::legacy_track const &Track) { return Track.name == "a_3"; })};
	auto const piece{std::ranges::find_if(made.pieces, [](layout::generated_piece const &Each) { return Each.name == "a_3"; })};
	REQUIRE(piece != made.pieces.end());
	CHECK(geometry::plan_distance(piece->path.points[segment_data::point::start], a3.paths.front().points[segment_data::point::start]) < 0.02);
	// the curve made wider by 20 m: the lines laid again, the old tracks where they were on the ground, each joint at the point of the
	// new line nearest to where it was; the line shorter, none of them past its end
	auto const before{network.find(id)->solved};
	auto const splits{network.find(id)->hard_splits};
	auto const &vertex{network.find(id)->shape.vertices.front()};
	auto const changed{model.execute(layout::set_vertex{id, vertex.id, vertex.radius + 20.0, {}, {}, {}}, layout::edit_policy::no_new_block)};
	REQUIRE_MESSAGE(changed.done, first_reason(changed));
	auto const &widened{*model.state().find(id)};
	REQUIRE(widened.hard_splits.size() == splits.size());
	for (std::size_t i = 0; i < splits.size(); ++i) {
		CAPTURE(i);
		CHECK(widened.hard_splits[i].track == splits[i].track);
		CHECK(widened.hard_splits[i].chainage <= widened.solved.length);
		CHECK(widened.hard_splits[i].chainage == doctest::Approx(widened.solved.nearest(before.position(splits[i].chainage))));
	}
	check_made(model.state());
	// written into the project file and read back the same
	project::document document;
	document.network = model.state();
	auto const read{project::read(project::write(document))};
	REQUIRE(read.project.has_value());
	CHECK(read.project->network.find(id)->hard_splits == widened.hard_splits);
}

TEST_CASE("takeover scenery: the anchors of the lineside objects, with their confidence, follow a change of their line") {
	auto const scenery{old_scenery()};
	auto const plan{layout::plan_takeover(scenery.tracks, plain_tracks(scenery))};
	layout::model model;
	auto const taken{layout::take_over_tracks(model, plan, scenery.objects)};
	REQUIRE(taken.made.done);
	auto const &network{model.state()};
	auto anchors{taken.anchors};
	auto const of = [&](std::string const &Name) { return *std::ranges::find_if(anchors, [&](layout::anchor const &Each) { return Each.name == Name; }); };
	auto const line_a{line_of(network, "a_1")};
	// the signal named by a track, 3 m to its right; its memory cell
	auto const signal{of("sem_a")};
	CHECK(signal.line == line_a);
	CHECK(signal.confidence >= 0.9);
	CHECK(signal.at.offset == doctest::Approx(-3.0).epsilon(0.01));
	CHECK(signal.at.chainage == doctest::Approx(150.0).epsilon(0.001));
	CHECK(std::abs(signal.turn) < 0.5);
	CHECK(of("sem_a_mem").confidence >= 0.9);
	// the poles 3.5 m to the left, the wires over the track hanging on them
	auto const pole{of("slup_3")};
	CHECK(pole.line == line_a);
	CHECK(pole.confidence == doctest::Approx(1.0));
	CHECK(pole.at.offset == doctest::Approx(3.5).epsilon(0.01));
	for (auto const &wire : anchors) {
		if (wire.kind != "traction") {
			continue;
		}
		CHECK(wire.poles.size() == 2);
		CHECK(wire.confidence >= 0.9);
		REQUIRE(wire.points.size() == 4);
		CHECK(wire.points[0].height == doctest::Approx(5.6).epsilon(0.01));
	}
	// the doubtful ones: the tree 25 m off, the cabinet as near to line b as to line c
	std::vector<std::string> doubtful;
	std::string why;
	for (auto const &anchor : anchors) {
		if (anchor.doubtful()) {
			doubtful.push_back(anchor.key);
			why += std::format("{} {} {:.2f}:", anchor.key, anchor.name, anchor.confidence);
			for (auto const &reason : anchor.reasons) {
				why += " " + reason + ";";
			}
			why += " / ";
		}
	}
	CAPTURE(why);
	CHECK(doubtful == std::vector<std::string>{"drzewa/tree1.t3d", "ludzie/szafa_kablowa.t3d"});
	CHECK(taken.doubtful().size() == 2);
	// the curve made wider: the objects along it follow, at their chainages and offsets; those on lines b and c stay
	auto const before{model.state()};
	auto const &vertex{before.find(line_a)->shape.vertices.front()};
	REQUIRE(model.execute(layout::set_vertex{line_a, vertex.id, vertex.radius + 20.0, {}, {}, {}}, layout::edit_policy::no_new_block).done);
	auto const moved{layout::follow(before, model.state(), anchors)};
	CHECK_FALSE(moved.empty());
	for (auto const &each : moved) {
		auto const &anchor{anchors[each.anchor]};
		CHECK(anchor.line == line_a);
		auto const placed{layout::place(model.state(), anchor)};
		REQUIRE(placed.has_value());
		CHECK(glm::distance(placed->location, each.to.location) < 1e-9);
	}
	// the signal on the straight before the curve doesn't move; a pole in the curve does, 3.5 m off the new curve
	CHECK(std::ranges::none_of(moved, [&](layout::moved const &Each) { return anchors[Each.anchor].name == "sem_a"; }));
	auto const in_curve{std::ranges::find_if(moved, [&](layout::moved const &Each) { return anchors[Each.anchor].name == "slup_7"; })};
	REQUIRE(in_curve != moved.end());
	auto const &solved{model.state().find(line_a)->solved};
	auto const s{anchors[in_curve->anchor].at.chainage};
	CHECK(glm::distance(geometry::plan_of(in_curve->to.location), solved.position(s)) == doctest::Approx(anchors[in_curve->anchor].at.offset).epsilon(1e-9));
	// written into the project file and read back the same
	project::document document;
	document.network = model.state();
	document.anchors = anchors;
	auto const read{project::read(project::write(document))};
	REQUIRE(read.project.has_value());
	CHECK(read.project->anchors.size() == anchors.size());
	CHECK(project::write(*read.project) == project::write(document));
}

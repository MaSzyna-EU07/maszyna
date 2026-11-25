#include "stdafx.h"
#include "doctest.h"

#include <cmath>
#include <numbers>

#include "vehicle/bones.h"

// the bones of the vehicle models (vehicle/bones.h): the rods of a steam locomotive pinned to the cranks of its
// wheels and to the crossheads, and the couplers following the next vehicle. the figures are those of the rods of the
// Ty2: a crank of 330 mm, a connecting rod of about 3 m

namespace {

double const radius{0.33};
glm::dvec3 const axle{0.0, 0.0, 0.7};

bones::rig rods() {
	bones::rig rig;
	REQUIRE(bones::add_point(rig, {"pin", "crank", "wheel3", "-1.2", "0.0", "1.03"}).empty());
	REQUIRE(bones::add_point(rig, {"cross", "slide", "pin", "-1.2", "3.0", "0.7", "0", "1", "0"}).empty());
	REQUIRE(bones::add_bone(rig, {"korbowod", "pin", "cross"}).empty());
	REQUIRE(bones::add_bone(rig, {"wiazar", "pin", "none"}).empty());
	REQUIRE(bones::add_bone(rig, {"wodzik", "cross", "none"}).empty());
	return rig;
}

bones::drive turned(bones::rig const &Rig, double Angle) {
	bones::drive drive;
	drive.wheels.resize(Rig.points.size());
	drive.couplers.resize(Rig.points.size());
	drive.wheels[0] = {glm::dvec3{-1.2, 0.0, 0.7}, glm::dvec3{1.0, 0.0, 0.0}, Angle};
	return drive;
}

// where the point of the submodel, given as it stands in the model, goes with the animation
glm::dvec3 animated(bones::animation const &Animation, glm::dvec3 Origin, glm::dmat3 Rotation, glm::dvec3 Point) {
	auto const local{glm::transpose(Rotation) * (Point - Origin)};
	return Origin + Rotation * glm::dvec3(glm::dmat4(Animation.matrix) * glm::dvec4(local, 1.0));
}

} // namespace

TEST_CASE("bones: a crank pin goes round with its wheel, the crosshead slides keeping the rod's length") {
	auto const rig{rods()};
	auto const length{glm::length(rig.points[1].rest - rig.points[0].rest)};
	auto lowest{1e9};
	auto highest{-1e9};
	for (auto degrees{0}; degrees < 360; degrees += 5) {
		auto const angle{static_cast<double>(degrees)};
		CAPTURE(angle);
		auto const points{bones::pose(rig, turned(rig, angle))};
		// the pin on its circle
		CHECK(glm::length(points[0] - glm::dvec3{-1.2, 0.0, 0.7}) == doctest::Approx(radius));
		// the crosshead on its guide, the rod's length kept
		CHECK(points[1].x == doctest::Approx(-1.2));
		CHECK(points[1].z == doctest::Approx(0.7));
		CHECK(glm::length(points[1] - points[0]) == doctest::Approx(length));
		lowest = std::min(lowest, points[1].y);
		highest = std::max(highest, points[1].y);
	}
	// the stroke of the piston: twice the crank
	CHECK(highest - lowest == doctest::Approx(2.0 * radius).epsilon(0.01));
	// a quarter turn about x takes the pin from over the axle to behind it
	auto const quarter{bones::crank(glm::dvec3{0.0, 0.0, 1.0}, {glm::dvec3{0.0}, glm::dvec3{1.0, 0.0, 0.0}, 90.0})};
	CHECK(quarter.y == doctest::Approx(-1.0));
	CHECK(quarter.z == doctest::Approx(0.0).scale(1.0));
}

TEST_CASE("bones: the connecting rod keeps its ends on the pin and the crosshead, whatever its submodel's frame") {
	auto const rig{rods()};
	// the submodel with its origin in the middle of the rod, turned half round about z the way some models are made
	glm::dvec3 const origin{-1.2, 1.5, 0.86};
	auto const rotation{glm::dmat3(glm::rotate(glm::dmat4(1.0), std::numbers::pi, glm::dvec3{0.0, 0.0, 1.0}))};
	auto const pinrest{rig.points[0].rest};
	auto const crossrest{rig.points[1].rest};
	for (auto angle : {0.0, 37.0, 90.0, 180.0, 271.0}) {
		CAPTURE(angle);
		auto const points{bones::pose(rig, turned(rig, angle))};
		auto const motion{bones::bone_motion(pinrest, points[0], crossrest, points[1])};
		auto const animation{bones::localise(motion, origin, rotation)};
		auto const pin{animated(animation, origin, rotation, pinrest)};
		auto const cross{animated(animation, origin, rotation, crossrest)};
		CHECK(glm::length(pin - points[0]) == doctest::Approx(0.0).scale(1.0));
		CHECK(glm::length(cross - points[1]) == doctest::Approx(0.0).scale(1.0));
		// the angles do what TSubModel::RaAnimation() does with them after the translation
		auto matrix{glm::translate(glm::mat4(1.f), animation.translation)};
		matrix = glm::rotate(matrix, glm::radians(animation.angles.x), glm::vec3(1.f, 0.f, 0.f));
		matrix = glm::rotate(matrix, glm::radians(animation.angles.y), glm::vec3(0.f, 1.f, 0.f));
		matrix = glm::rotate(matrix, glm::radians(animation.angles.z), glm::vec3(0.f, 0.f, 1.f));
		for (auto column{0}; column < 4; ++column) {
			for (auto row{0}; row < 4; ++row) {
				CHECK(matrix[column][row] == doctest::Approx(animation.matrix[column][row]).scale(1.0).epsilon(1e-4));
			}
		}
	}
}

TEST_CASE("bones: a coupling rod moves with its pin without turning") {
	auto const rig{rods()};
	auto const points{bones::pose(rig, turned(rig, 120.0))};
	auto const motion{bones::bone_motion(rig.points[0].rest, points[0], std::nullopt, std::nullopt)};
	auto const animation{bones::localise(motion, glm::dvec3{-1.2, -1.65, 1.03}, glm::dmat3{1.0})};
	CHECK(animation.angles.x == doctest::Approx(0.0).scale(1.0));
	CHECK(glm::length(glm::dvec3(animation.translation) - (points[0] - rig.points[0].rest)) == doctest::Approx(0.0).scale(1.0));
}

TEST_CASE("bones: a coupler faces the coupler of the next vehicle as it moves") {
	bones::rig rig;
	REQUIRE(bones::add_point(rig, {"hinge", "fixed", "0", "6.2", "1.06"}).empty());
	REQUIRE(bones::add_point(rig, {"hook", "coupler", "front", "0", "6.9", "1.06"}).empty());
	REQUIRE(bones::add_bone(rig, {"sprzeg", "hinge", "hook"}).empty());
	bones::drive drive;
	drive.wheels.resize(2);
	drive.couplers = {std::nullopt, glm::dvec3{0.2, -0.05, 0.0}};
	auto const points{bones::pose(rig, drive)};
	CHECK(points[1].x == doctest::Approx(0.2));
	auto const motion{bones::bone_motion(rig.points[0].rest, points[0], rig.points[1].rest, points[1])};
	auto const facing{motion.rotation * (rig.points[1].rest - rig.points[0].rest)};
	CHECK(glm::length(glm::normalize(facing) - glm::normalize(points[1] - points[0])) == doctest::Approx(0.0).scale(1.0));
	// not coupled: as made
	drive.couplers[1] = std::nullopt;
	CHECK(bones::pose(rig, drive)[1] == rig.points[1].rest);
}

TEST_CASE("bones: what the .mmd can get wrong") {
	bones::rig rig;
	CHECK_FALSE(bones::add_point(rig, {"p", "wobble", "1", "2", "3"}).empty());
	CHECK_FALSE(bones::add_point(rig, {"p", "fixed", "1", "2"}).empty());
	CHECK_FALSE(bones::add_point(rig, {"p", "fixed", "1", "x", "3"}).empty());
	// a slide driven by a point defined later
	CHECK_FALSE(bones::add_point(rig, {"c", "slide", "pin", "0", "3", "0.7", "0", "1", "0"}).empty());
	CHECK_FALSE(bones::add_point(rig, {"c", "coupler", "middle", "0", "3", "0.7"}).empty());
	CHECK(bones::add_point(rig, {"p", "fixed", "1", "2", "3"}).empty());
	CHECK_FALSE(bones::add_point(rig, {"p", "fixed", "1", "2", "3"}).empty());
	CHECK_FALSE(bones::add_bone(rig, {"rod", "q", "none"}).empty());
	CHECK_FALSE(bones::add_bone(rig, {"rod", "p", "q"}).empty());
	CHECK(bones::add_bone(rig, {"rod", "p", "none"}).empty());
	CHECK(rig.points.size() == 1);
	CHECK(rig.bones.size() == 1);
}

namespace {

// a hose of 1.3 m hanging between two points 1 m apart, in the frame of the model file (z up)
bones::rig hose() {
	bones::rig rig;
	REQUIRE(bones::add_point(rig, {"kurek", "fixed", "0", "0", "0"}).empty());
	REQUIRE(bones::add_point(rig, {"zlaczka", "fixed", "1", "0", "0"}).empty());
	REQUIRE(bones::add_chain(rig, {"waz", "kurek", "zlaczka", "3", "0.25", "0", "-0.25", "0.5", "0", "-0.35", "0.75", "0", "-0.25"}).empty());
	return rig;
}

bones::chain_physics downwards() {
	bones::chain_physics physics;
	physics.gravity = glm::dvec3{0.0, 0.0, -9.81};
	return physics;
}

std::vector<glm::dvec3> rests(bones::rig const &Rig) {
	std::vector<glm::dvec3> nodes;
	for (auto const node : Rig.chains[0].nodes) {
		nodes.push_back(Rig.points[node].rest);
	}
	return nodes;
}

void settle(bones::rig const &Rig, bones::chain_state &State, glm::dvec3 Start, glm::dvec3 End, double Seconds) {
	for (auto step{0L}; step < std::lround(Seconds / 0.02); ++step) {
		bones::simulate(Rig.chains[0].lengths, State, Start, End, 0.02, downwards());
	}
}

} // namespace

TEST_CASE("bones: a chain hangs between its ends, its pieces keeping their lengths") {
	auto const rig{hose()};
	REQUIRE(rig.chains.size() == 1);
	CHECK(rig.find("waz.2") >= 0);
	CHECK(rig.chains[0].lengths.size() == 4);
	bones::chain_state state;
	glm::dvec3 const start{0.0};
	glm::dvec3 const end{1.0, 0.0, 0.0};
	bones::start_chain(state, rests(rig), start, end);
	settle(rig, state, start, end, 20.0);
	std::vector<glm::dvec3> points{start};
	points.insert(points.end(), state.now.begin(), state.now.end());
	points.push_back(end);
	for (std::size_t piece{0}; piece < rig.chains[0].lengths.size(); ++piece) {
		CAPTURE(piece);
		CHECK(glm::length(points[piece + 1] - points[piece]) == doctest::Approx(rig.chains[0].lengths[piece]).epsilon(0.01));
	}
	// hanging below its ends, the middle the lowest, and at rest
	CHECK(state.now[1].z < state.now[0].z);
	CHECK(state.now[1].z < -0.3);
	CHECK(state.now[1].x == doctest::Approx(0.5).epsilon(0.01));
	CHECK(glm::length(state.now[1] - state.before[1]) < 1e-5);
}

TEST_CASE("bones: a chain jerked by its ends swings after them and settles") {
	auto const rig{hose()};
	bones::chain_state state;
	glm::dvec3 const start{0.0};
	glm::dvec3 const end{1.0, 0.0, 0.0};
	bones::start_chain(state, rests(rig), start, end);
	settle(rig, state, start, end, 20.0);
	auto const hanging{state.now[1]};
	// both ends 0.4 m on in a tenth of a second, as the vehicle jerked by its coupler
	glm::dvec3 const shift{0.0, 0.4, 0.0};
	bones::simulate(rig.chains[0].lengths, state, start + shift, end + shift, 0.1, downwards());
	// the middle lags behind, swinging
	CHECK(state.now[1].y - hanging.y < 0.35);
	auto farthest{0.0};
	for (auto step{0L}; step < std::lround(2.0 / 0.02); ++step) {
		bones::simulate(rig.chains[0].lengths, state, start + shift, end + shift, 0.02, downwards());
		farthest = std::max(farthest, state.now[1].y - hanging.y);
	}
	// and overshoots the ends before it comes back
	CHECK(farthest > 0.4);
	settle(rig, state, start + shift, end + shift, 20.0);
	CHECK(state.now[1].y - hanging.y == doctest::Approx(0.4).epsilon(0.01));
}

TEST_CASE("bones: the hoses of two vehicles meet halfway between their cocks") {
	bones::rig rig;
	REQUIRE(bones::add_point(rig, {"kurek", "fixed", "0.45", "7.6", "0.95"}).empty());
	REQUIRE(bones::add_point(rig, {"kurek_sasiada", "coupler", "front", "0.45", "8.3", "0.95"}).empty());
	REQUIRE(bones::add_point(rig, {"spotkanie", "mid", "kurek", "kurek_sasiada"}).empty());
	CHECK(rig.points[2].rest.y == doctest::Approx(7.95));
	bones::drive drive;
	drive.wheels.resize(3);
	drive.couplers = {std::nullopt, glm::dvec3{0.1, -0.2, 0.0}, std::nullopt};
	auto const points{bones::pose(rig, drive)};
	CHECK(points[2].x == doctest::Approx(0.5));
	CHECK(points[2].y == doctest::Approx(7.85));
	// a chain's node doesn't drive other points
	REQUIRE(bones::add_chain(rig, {"waz", "kurek", "spotkanie", "1", "0.45", "7.8", "0.6"}).empty());
	CHECK_FALSE(bones::add_point(rig, {"x", "mid", "kurek", "waz.1"}).empty());
	CHECK_FALSE(bones::add_chain(rig, {"waz2", "kurek", "nic", "1", "0", "0", "0"}).empty());
	CHECK_FALSE(bones::add_chain(rig, {"waz3", "kurek", "spotkanie", "2", "0", "0", "0"}).empty());
}

TEST_CASE("bones: a hose hung between two vehicles is the same chain from either end") {
	// each vehicle hangs it from its own cock to the other's: the coupling heads meet only if the two come out the same
	std::vector<double> const lengths{0.78, 0.78};
	glm::dvec3 const ours{-0.57, 7.47, 1.07};
	glm::dvec3 const theirs{-0.57, 8.445, 1.07};
	bones::chain_state forwards;
	bones::chain_state backwards;
	bones::start_chain(forwards, {glm::dvec3{-0.57, 7.9575, 0.49}}, ours, theirs);
	bones::start_chain(backwards, {glm::dvec3{-0.57, 7.9575, 0.49}}, theirs, ours);
	for (auto step{0}; step < 300; ++step) {
		// the two rocking and jerking against each other
		auto const sway{glm::dvec3{0.05 * std::sin(step * 0.13), 0.03 * std::sin(step * 0.07), 0.04 * std::cos(step * 0.11)}};
		bones::simulate(lengths, forwards, ours, theirs + sway, 0.02, downwards());
		bones::simulate(lengths, backwards, theirs + sway, ours, 0.02, downwards());
		REQUIRE(forwards.now[0] == backwards.now[0]);
	}
	// and it hangs, the pieces about their lengths
	CHECK(forwards.now[0].z < 0.6);
	CHECK(glm::length(forwards.now[0] - ours) == doctest::Approx(0.78).epsilon(0.02));
}

TEST_CASE("bones: the skin of a bent submodel follows its spine smoothly") {
	std::vector<glm::dvec3> const rest{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 2.0, 0.0}};
	// the spine bent a quarter turn at its middle
	std::vector<glm::dvec3> const now{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}};
	std::vector<glm::dvec3> const skin{{0.1, 0.0, 0.0}, {0.1, 1.0, 0.0}, {0.1, 2.0, 0.0}, {0.0, 1.5, 0.1}};
	auto const binds{bones::bind_skin(rest, skin)};
	REQUIRE(binds.size() == skin.size());
	auto const turns{bones::spine_turns(rest, now)};
	REQUIRE(turns.size() == 3);
	auto const at = [&binds, &now, &turns](std::size_t Index) { return bones::pose_skin(binds[Index], now, turns).position; };
	// the start as it was
	CHECK(glm::length(at(0) - glm::dvec3{0.1, 0.0, 0.0}) == doctest::Approx(0.0).scale(1.0));
	// the end turned with the last piece
	CHECK(glm::length(at(2) - glm::dvec3{1.0, 0.9, 0.0}) == doctest::Approx(0.0).scale(1.0));
	// at the bend halfway between the turns of the two pieces, as far off the spine as it was
	CHECK(glm::length(at(1) - glm::dvec3{0.1 * std::sqrt(0.5), 1.0 - 0.1 * std::sqrt(0.5), 0.0}) == doctest::Approx(0.0).scale(1.0));
	// a point off the plane of the bend stays off it
	CHECK(at(3).z == doctest::Approx(0.1));
	CHECK(at(3).x == doctest::Approx(0.5));
	// a bent submodel needs its spine
	bones::rig rig;
	REQUIRE(bones::add_point(rig, {"a", "fixed", "0", "0", "0"}).empty());
	REQUIRE(bones::add_point(rig, {"b", "fixed", "0", "1", "0"}).empty());
	CHECK(bones::add_bend(rig, {"waz", "a", "b"}).empty());
	CHECK_FALSE(bones::add_bend(rig, {"waz", "a"}).empty());
	CHECK_FALSE(bones::add_bend(rig, {"waz", "a", "c"}).empty());
	CHECK_FALSE(rig.empty());
}

TEST_CASE("bones: a hose on a running vehicle hangs as it does standing, its swing against the vehicle damped") {
	auto const rig{hose()};
	glm::dvec3 const start{0.0};
	glm::dvec3 const end{1.0, 0.0, 0.0};
	bones::chain_state standing;
	bones::start_chain(standing, rests(rig), start, end);
	for (auto step{0L}; step < std::lround(20.0 / 0.02); ++step) {
		bones::simulate(rig.chains[0].lengths, rig.chains[0].bends, standing, start, end, 0.02, downwards());
	}
	// the same at 50 km/h along the track, the frame going with it
	bones::chain_state running;
	bones::start_chain(running, rests(rig), start, end);
	glm::dvec3 const velocity{0.0, 13.9, 0.0};
	glm::dvec3 travelled{0.0};
	for (auto step{0L}; step < std::lround(20.0 / 0.02); ++step) {
		travelled += velocity * 0.02;
		bones::simulate(rig.chains[0].lengths, rig.chains[0].bends, running, start + travelled, end + travelled, 0.02, downwards());
	}
	for (std::size_t node{0}; node < standing.now.size(); ++node) {
		CAPTURE(node);
		CHECK(glm::length((running.now[node] - travelled) - standing.now[node]) < 0.01);
	}
	// stiff, it keeps more of its curve than a limp one: the middle hangs less far below where it was made
	bones::chain_state limp;
	bones::start_chain(limp, rests(rig), start, end);
	for (auto step{0L}; step < std::lround(20.0 / 0.02); ++step) {
		bones::simulate(rig.chains[0].lengths, limp, start, end, 0.02, downwards());
	}
	auto const made{rig.points[rig.chains[0].nodes[1]].rest};
	CHECK(glm::length(standing.now[1] - made) <= glm::length(limp.now[1] - made) + 1e-6);
}

TEST_CASE("bones: a hose on a running vehicle keeps still through frames of changing length, from its start") {
	auto const rig{hose()};
	glm::dvec3 const start{0.0};
	glm::dvec3 const end{1.0, 0.0, 0.0};
	bones::chain_state standing;
	bones::start_chain(standing, rests(rig), start, end);
	for (auto step{0L}; step < std::lround(20.0 / 0.02); ++step) {
		bones::simulate(rig.chains[0].lengths, rig.chains[0].bends, standing, start, end, 0.02, downwards());
	}
	// started at 50 km/h, with frames now short, now long: the substeps change their length from frame to frame
	bones::chain_state running;
	bones::start_chain(running, rests(rig), start, end);
	glm::dvec3 const velocity{0.0, 13.9, 0.0};
	glm::dvec3 travelled{0.0};
	auto firstsecond{0.0};
	auto settled{0.0};
	auto time{0.0};
	for (auto frame{0}; time < 20.0; ++frame) {
		auto const deltatime{frame % 2 == 0 ? 0.005 : 0.035};
		time += deltatime;
		travelled += velocity * deltatime;
		bones::simulate(rig.chains[0].lengths, rig.chains[0].bends, running, start + travelled, end + travelled, deltatime, downwards());
		for (std::size_t node{0}; node < standing.now.size(); ++node) {
			auto const off{glm::length((running.now[node] - travelled) - standing.now[node])};
			if (time < 1.0) {
				firstsecond = std::max(firstsecond, glm::length((running.now[node] - travelled) - rig.points[rig.chains[0].nodes[node]].rest));
			}
			else if (time > 10.0) {
				settled = std::max(settled, off);
			}
		}
	}
	// no whip at the start: the hose doesn't fly past where it hangs, left behind by the vehicle
	CHECK(firstsecond < 0.5);
	CHECK(settled < 0.005);
}

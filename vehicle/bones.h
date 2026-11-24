/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

class TSubModel;

// bones of a vehicle model: submodels pinned to points that go round with a wheel (the crank pins of the rods of a
// steam locomotive), slide along a guide (the crossheads), or follow the coupler of the next vehicle, and turned to
// keep facing another such point. a vehicle's .mmd defines them in its models: section, the points before the bones
// and before the points sliding with them:
//   bonepoint: <name> fixed <x> <y> <z>
//   bonepoint: <name> crank <wheel submodel> <x> <y> <z>
//   bonepoint: <name> slide <driving point> <x> <y> <z> <guide dx> <dy> <dz>
//   bonepoint: <name> coupler <front|rear> <x> <y> <z>
//   bonepoint: <name> mid <point> <point>
//   bonechain: <name> <start point> <end point> <nodes between> <x> <y> <z> (of each node)
//   bone: <submodel> <anchor point> <point it faces, or none>
// the coordinates are those of the model file (x across, y along the vehicle, z up), and the positions are those of
// the model as it is made: the wheels at their angle 0, the next vehicle standing straight behind. a sliding point
// keeps its distance from the point driving it (the length of the connecting rod), along its guide. a chain hangs
// between its two points: its nodes, the points <name>.1, <name>.2 and so on, swing with gravity and their inertia
// (see simulate()), the lengths between them kept: a hose, its pieces bones from node to node
namespace bones {

enum class point_kind
{
	fixed,
	crank,
	slide,
	coupler,
	middle, // halfway between two points
	node // of a chain
};

struct point {
	std::string name;
	point_kind kind{point_kind::fixed};
	glm::dvec3 rest{0.0};
	std::string wheel; // crank: the wheel submodel it goes round with
	int driver{-1}; // slide: the point driving it
	glm::dvec3 guide{0.0, 1.0, 0.0}; // slide: the direction of its guide
	int end{0}; // coupler: the end of the vehicle, 0 front, 1 rear
	int second{-1}; // middle: the other point, driver the first
};

// a chain of nodes hanging between two points
struct chain {
	std::string name;
	int start{-1};
	int end{-1};
	std::vector<int> nodes; // the points of its nodes, from the start
	std::vector<double> lengths; // of its pieces, from the start point through the nodes to the end point
};

struct bone {
	std::string submodel;
	int anchor{-1};
	int aim{-1}; // -1: the bone only moves with its anchor
};

struct rig {
	std::vector<point> points;
	std::vector<bone> bones;
	std::vector<chain> chains;
	// the index of the named point, -1 for none
	int find(std::string_view Name) const;
	bool empty() const { return bones.empty(); }
};

// adds what follows bonepoint: or bone: in the .mmd; returns why it couldn't, empty when it did
std::string add_point(rig &Rig, std::vector<std::string> const &Words);
std::string add_bone(rig &Rig, std::vector<std::string> const &Words);
// the words after bonechain: are the name, the two points, the count of the nodes and three numbers for each
std::string add_chain(rig &Rig, std::vector<std::string> const &Words);
// how many words follow bonepoint: of the kind, after the name and the kind; 0 for an unknown kind
int point_words(std::string_view Kind);

// a wheel turned by its angle about its axle
struct wheel_turn {
	glm::dvec3 centre{0.0}; // of the axle
	glm::dvec3 axis{1.0, 0.0, 0.0}; // the wheel turns about, as the wheel animation turns it
	double angle{0.0}; // degrees
};

// what moves the points: for each point, the turn of its wheel (crank points) or the shift of the next vehicle's
// coupler from where it stands in the model (coupler points, none when there's no vehicle coupled)
struct drive {
	std::vector<wheel_turn> wheels;
	std::vector<std::optional<glm::dvec3>> couplers;
};

// where the points are
std::vector<glm::dvec3> pose(rig const &Rig, drive const &Drive);

glm::dvec3 crank(glm::dvec3 const &Rest, wheel_turn const &Wheel);
// the point on its guide at the distance from the driving point it has standing as made
glm::dvec3 slide(glm::dvec3 const &Rest, glm::dvec3 const &Guide, glm::dvec3 const &DriverRest, glm::dvec3 const &DriverNow);

// a chain in motion: its nodes now and a step before, in a frame of the world (Verlet integration)
struct chain_state {
	std::vector<glm::dvec3> now;
	std::vector<glm::dvec3> before;
	glm::dvec3 start{0.0};
	glm::dvec3 end{0.0};
	bool started{false};
};
struct chain_physics {
	glm::dvec3 gravity{0.0, -9.81, 0.0}; // in the frame of the simulation
	double damping{1.5}; // 1/s: how fast the swing dies down
	double step{1.0 / 240.0}; // s, the longest substep
	int iterations{12}; // of keeping the lengths, a substep
};
// starts the chain at rest with its nodes where given
void start_chain(chain_state &State, std::vector<glm::dvec3> const &Nodes, glm::dvec3 const &Start, glm::dvec3 const &End);
// moves the chain by the time step, its ends going over to where they are now; the ends are pinned, the nodes swing
void simulate(std::vector<double> const &Lengths, chain_state &State, glm::dvec3 const &Start, glm::dvec3 const &End, double Deltatime,
              chain_physics const &Physics = {});

// the move of a bone: turned about its anchor to face its aim again, its anchor moved
struct motion {
	glm::dmat3 rotation{1.0};
	glm::dvec3 anchor_rest{0.0};
	glm::dvec3 anchor_now{0.0};
};
motion bone_motion(glm::dvec3 const &AnchorRest, glm::dvec3 const &AnchorNow, std::optional<glm::dvec3> const &AimRest, std::optional<glm::dvec3> const &AimNow);

// the animation of a submodel making the motion, in the submodel's own frame: the submodel has its origin at the
// Origin and its axes turned by the Rotation (both in the model file's frame). the translation and the angles (XYZ,
// degrees) are those TSubModel::SetRotateXYZ() applies after TSubModel::SetTranslate(), the matrix the same
struct animation {
	glm::vec3 translation{0.f};
	glm::vec3 angles{0.f};
	glm::mat4 matrix{1.f};
};
animation localise(motion const &Motion, glm::dvec3 const &Origin, glm::dmat3 const &Rotation);

// a bone's submodel, where it is and how it's turned in the frame of the model file
struct binding {
	TSubModel *submodel{nullptr};
	glm::dvec3 origin{0.0};
	glm::dmat3 rotation{1.0};
};

// the frame of the model file from the frame of the loaded model: x the other way, y and z swapped
// (float4x4::InitialRotate()); it is its own inverse
glm::dmat3 model_file_frame();

} // namespace bones

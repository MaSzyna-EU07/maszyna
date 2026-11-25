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
#include <glm/gtc/quaternion.hpp>

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
//   bonebend: <submodel> <point> <point> ... [head <point> <point>] end
// the coordinates are those of the model file (x across, y along the vehicle, z up), and the positions are those of
// the model as it is made: the wheels at their angle 0, the next vehicle standing straight behind. a sliding point
// keeps its distance from the point driving it (the length of the connecting rod), along its guide. a chain hangs
// between its two points: its nodes, the points <name>.1, <name>.2 and so on, swing with gravity and their inertia
// (see simulate()), the lengths between them kept: a hose, its pieces bones from node to node. a bent submodel is a
// hose of one piece bending along its spine, the points given from its one end to the other: its vertices follow the
// spine as it bends (bind_skin(), pose_skin()), in a copy of it of the vehicle's own. its head, past the last point of
// the spine, can be held to the line between two points instead of turning with the last piece: the coupling heads of the
// hoses of two vehicles, both turned to the line between them, keep the faces they couple with together
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
	std::vector<double> bends; // between every other point, from the start point: what keeps a hose from folding
};

struct bone {
	std::string submodel;
	int anchor{-1};
	int aim{-1}; // -1: the bone only moves with its anchor
};

// a submodel bent along the points of its spine
struct bend {
	std::string submodel;
	std::vector<int> spine;
	int head_from{-1}; // the line the head is held to, none for a head turning with the last piece
	int head_to{-1};
};

struct rig {
	std::vector<point> points;
	std::vector<bone> bones;
	std::vector<chain> chains;
	std::vector<bend> bends;
	// the index of the named point, -1 for none
	int find(std::string_view Name) const;
	bool empty() const { return bones.empty() && bends.empty(); }
};

// adds what follows bonepoint: or bone: in the .mmd; returns why it couldn't, empty when it did
std::string add_point(rig &Rig, std::vector<std::string> const &Words);
std::string add_bone(rig &Rig, std::vector<std::string> const &Words);
// the words after bonebend: are the submodel and the points of its spine, two at least, then optionally head and the two
// points of the line its head is held to
std::string add_bend(rig &Rig, std::vector<std::string> const &Words);
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
	double step{0.0}; // s, the substep that took the nodes from before to now
	bool started{false};
	bool moving{false}; // whether the nodes have taken on the motion of the ends yet
};
struct chain_physics {
	glm::dvec3 gravity{0.0, -9.81, 0.0}; // in the frame of the simulation
	double damping{6.0}; // 1/s: how fast the swing against the vehicle dies down, not the vehicle's own motion
	double bending{0.85}; // of the difference between every other point made up in a pass: how stiff the hose is
	double step{1.0 / 240.0}; // s, the longest substep
	int iterations{40}; // of keeping the lengths, a substep
};
// starts the chain at rest with its nodes where given
void start_chain(chain_state &State, std::vector<glm::dvec3> const &Nodes, glm::dvec3 const &Start, glm::dvec3 const &End);
// moves the chain by the time step, its ends going over to where they are now; the ends are pinned, the nodes swing
void simulate(std::vector<double> const &Lengths, std::vector<double> const &Bends, chain_state &State, glm::dvec3 const &Start, glm::dvec3 const &End,
              double Deltatime, chain_physics const &Physics = {});
// a chain with no stiffness against bending
void simulate(std::vector<double> const &Lengths, chain_state &State, glm::dvec3 const &Start, glm::dvec3 const &End, double Deltatime,
              chain_physics const &Physics = {});

// a point of the skin of a bent submodel, bound to the piece of its spine nearest to it: where along the piece, and
// how far off it, as the spine was made
struct skin_bind {
	std::size_t piece{0};
	double along{0.0};
	glm::dvec3 offset{0.0};
};
std::vector<skin_bind> bind_skin(std::vector<glm::dvec3> const &Spine, std::vector<glm::dvec3> const &Points);
// how each node of the spine turned from where it was made to where it is now: the turn of each piece, the nodes
// between two pieces halfway between their turns
std::vector<glm::dquat> spine_turns(std::vector<glm::dvec3> const &Rest, std::vector<glm::dvec3> const &Now);
// the turn of a head held to the line between two points: the line as made turned to the line as it is now
glm::dquat line_turn(glm::dvec3 const &FromRest, glm::dvec3 const &ToRest, glm::dvec3 const &FromNow, glm::dvec3 const &ToNow);
// the point of the skin on the spine as it is now, and its turn, for its normals: both going smoothly along the piece
// from the turn of one node to the other's, the skin bends rather than breaks at the nodes
struct skin_pose {
	glm::dvec3 position{0.0};
	glm::dquat turn{1.0, 0.0, 0.0, 0.0};
};
skin_pose pose_skin(skin_bind const &Bind, std::vector<glm::dvec3> const &Now, std::vector<glm::dquat> const &Turns);

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

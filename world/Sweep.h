/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "utilities/Classes.h"
#include "utilities/Names.h"
#include "scene/scenenode.h"
#include "world/Road.h"
#include "world/Segment.h"
#include "vehicle/DynObj.h"

namespace scene
{
struct scratch_data;
}

// a model laid along a curve: copies of it bent to follow the curve end to end, or set along it at even steps.
// the node isn't drawn itself, it makes ordinary shapes of the copies, so the renderers deal with it as with the rest
// of the scenery. animations, lights and other submodels which aren't plain triangles are left out of the copies.
// scenery entry:
// node <max> <min> <name> sweep <model> <skin> [<property> <values>]... piece <laid out like a path of a track>... endsweep
// properties: mode <bend|repeat>, step <length of a copy along the curve, 0: the length of the model>,
// range <from> <to> (m along the curve, to below 0: up to its end), offset <sideways, + to the right> <up>,
// anchor <origin|near|far|centre> <origin|bottom|top> (what of the model the offset puts there: sideways its origin, its side
// facing the curve, the other side or its middle; up its origin, its bottom or its top),
// axis <x|z> (axis of the model laid along the curve), flip (the model goes the other way), mirror (its sides swapped),
// widen (the offset grows by the widening of the structure gauge in the curves, the way the platforms are set back),
// noface (the model isn't turned to have its -x side, or -z for one laid along x, facing the curve),
// parameters <count> <values>... (of a template: the model can be an *.inc file, its long models are bent, the others stand
// upright at their places along the curve),
// anchor sides: origin, near, far, centre; heights: origin, bottom, top, edge (the top of the side facing the curve),
// tilt (bent copies lean with the cant of the curve; otherwise they stay upright, as the platforms and the walls by a track do,
// at the height the plane of the cant has at their sideways offset)
class sweep_node : public scene::basic_node
{

  public:
	struct state
	{
		std::string model;
		std::string skin{"none"};
		bool bend{true};
		double step{0.0};
		double from{0.0};
		double to{-1.0};
		double lateral{0.0};
		double height{0.0};
		bool along_x{false};
		bool flip{false};
		bool mirror{false};
		bool tilt{false};
		bool face{true}; // the model is turned so its -x side (-z for a model laid along x) faces the curve
		bool widen{false}; // moved away from the curve by the widening of the structure gauge in the curves
		std::vector<std::string> parameters; // of a template (*.inc) laid along the curve
		int side_anchor{0}; // 0: origin, 1: the side facing the curve, 2: the far side, 3: the middle
		int height_anchor{0}; // 0: origin, 1: bottom, 2: top, 3: top of the side facing the curve
		std::vector<segment_data> pieces;

		bool operator==(state const &Other) const;
	};
	// point of the curve, with the directions the copies are laid out by
	struct frame
	{
		glm::dvec3 position{0.0};
		glm::dvec3 forward{0.0, 0.0, 1.0};
		glm::dvec3 left{-1.0, 0.0, 0.0};
		glm::dvec3 up{0.0, 1.0, 0.0};
		double roll{0.0}; // degrees, of the curve there
		std::array<double, 2> widening{}; // of the structure gauge, below the platforms: right and left side, metres
		std::array<double, 2> cant{}; // metres, positive on the inner side of the curve
	};

	explicit sweep_node(scene::node_data const &Nodedata);

	void import(cParser &Input, glm::dvec3 const &Offset);
	void define(state const &State);
	state const &definition() const
	{
		return m_state;
	}
	// length of the curve, in the plan
	double length() const;
	frame frame_at(double const Station) const;
	// station of the point of the curve nearest to the point in the plan, and how far to the right of the curve it is
	double project(glm::dvec3 const &Point, double &Lateral) const;
	// stretch of the curve the copies take
	double start() const;
	double end() const;
	// length of a copy along the curve, as the model has it
	double model_length() const;
	std::vector<scene::shape_node> create_shapes() const;
	// a model of what's laid along the curve, placed in its space
	struct item
	{
		TModel3d *model{nullptr};
		material_data skins;
		glm::dmat4 transform{1.0};
	};
	std::vector<item> items() const;
	// height of the head of the rail on the side of the offset over the points of the track, less the height of the rail
	static double rail_head(double const Roll, double const Lateral);
	// puts the copies in the scene as shapes of their own, which can be taken back
	void show();
	void hide();
	void merged(bool const Merged)
	{
		m_merged = Merged;
	}
	bool merged() const
	{
		return m_merged;
	}

	bool m_editorremoved{false};

  private:
	void rebuild_samples();
	float radius_() override;
	void serialize_(std::ostream &Output) const override;
	void deserialize_(std::istream &Input) override;
	void export_as_text_(std::ostream &Output) const override;

	struct sample
	{
		double station;
		glm::dvec3 position;
		glm::dvec3 tangent;
		double roll; // degrees
		std::array<double, 2> widening;
		std::array<double, 2> cant;
	};
	state m_state;
	std::vector<sample> m_samples;
	bool m_merged{false};
	owned_shapes m_shapes;
};

// collection of the models laid along curves present in the scene
class sweep_table : public basic_table<sweep_node>
{

  public:
	// generates geometry of the copies and puts it in the scene
	void create_geometry(scene::scratch_data &Scratchpad);
};

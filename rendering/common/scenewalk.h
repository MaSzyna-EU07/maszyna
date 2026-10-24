/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the walk of the scene shared by the renderers (stage N6 of claude/vulkan14-renderer-plan.md, decision K11): the decisions of
// what is drawn in a pass (the sections, cells, tracks, shapes, models, instances, vehicles, the cab, the translucent parts),
// in which order, with which material, transform and level of detail. a renderer derives from it and gets the draws through
// the walk_* methods: the opengl renderer makes them at once (the same gl calls in the same order as before the walk was
// shared), the vulkan renderer records them. what the backend may draw itself (the gpu culling of the opengl renderer) is
// asked about with the walk_* methods which do nothing by default.
//
// the walk keeps the state of the draw in model_ubs (the block of the model of the shaders) and the transform on the matrix
// stack of OpenGLMatrices (a cpu stack, despite the name)

#pragma once

#include "rendering/geometrybank.h"
#include "rendering/openglcamera.h"
#include "scene/scene.h"
#include "world/Track.h"
#include "world/TrackSleepers.h"
#include "model/material.h"
#include "gl/ubo.h"

#include <unordered_map>
#include <utility>
#include <vector>

class TAnimModel;
class TDynamicObject;
class TModel3d;
class TSubModel;
class TTraction;
class TMemCell;
struct material_data;
struct lighting_data;

namespace gfx
{

class scene_walk
{
  public:
	virtual ~scene_walk() = default;

  protected:
	// types
	enum class rendermode
	{
		none,
		color,
		shadows,
		reflections,
		pickcontrols,
		pickscenery
	};

	struct debug_stats
	{
		int dynamics{0};
		int models{0};
		int submodels{0};
		int paths{0};
		int traction{0};
		int shapes{0};
		int lines{0};
		int particles{0};
		int drawcalls{0};
		int instanced_drawcalls{0}; // drawcalls issued via instanced render path (one per TModel3d batch)
		int instances{0}; // total instances drawn via instanced render path
		int triangles{0};

		debug_stats &operator+=(const debug_stats &Right)
		{
			dynamics += Right.dynamics;
			models += Right.models;
			submodels += Right.submodels;
			paths += Right.paths;
			traction += Right.traction;
			shapes += Right.shapes;
			lines += Right.lines;
			particles += Right.particles;
			drawcalls += Right.drawcalls;
			instanced_drawcalls += Right.instanced_drawcalls;
			instances += Right.instances;
			return *this;
		}
	};

	using section_sequence = std::vector<scene::basic_section *>;
	using distancecell_pair = std::pair<double, scene::basic_cell *>;
	using cell_sequence = std::vector<distancecell_pair>;

	struct renderpass_config
	{
		opengl_camera pass_camera;
		opengl_camera viewport_camera;
		rendermode draw_mode{rendermode::none};
		float draw_range{0.0f};
		// depth range of the projection of a color pass; the far plane is only the end of the range which is drawn,
		// as the projection matrix itself has none
		float znear{0.0f};
		float zfar{0.0f};
		debug_stats draw_stats;
	};

	// an instance of a bucket of a scene cell, queued to be drawn by Render_Instanced()
	struct queued_instance
	{
		scene::basic_cell::instance_bucket const *bucket;
		std::size_t index; // of the instance and its record in the bucket
	};

	// the programs of the draws which aren't drawn with the programs of their materials
	enum class walk_shader
	{
		pick, // a colour for the picking of the scenery or the controls
		pick_surface, // the gradient of a touch screen for the picking of the controls
		line, // the wires of the traction and the lines
		billboard, // the glare of a lamp
		freespot // the point of light of a lamp
	};

	// how far the reflection pass draws the simplified versions of things, as if they were farther away
	static int constexpr EU07_REFLECTIONFIDELITYOFFSET{250};

	// the walk
	void Render(scene::basic_region *Region);
	void Render(section_sequence::iterator First, section_sequence::iterator Last);
	void Render(cell_sequence::iterator First, cell_sequence::iterator Last);
	void Render(scene::shape_node const &Shape, bool const Ignorerange);
	void Render(TAnimModel *Instance);
	// batched render path for many TAnimModel instances that share the same TModel3d.
	// Caller guarantees every instance was instanceable when bucketed (no per-instance lights,
	// no submodel animation, same replacable skins); the records of the instances tell which
	// ones still are. Instances are individually frustum/distance culled by their records, the
	// TAnimModel itself is read only for the first instance drawn. Each lod band drawn counts as
	// one instanced_drawcall in draw_stats; the instances drawn are added to the instances counter.
	// Bandcounts, when given, gets the number of instances drawn in each lod band.
	void Render_Instanced(TModel3d *Model, std::vector<queued_instance> const &Instances, std::vector<std::uint32_t> *Bandcounts = nullptr);
	// adds the bucketed instances of a scene cell to the frame-level buckets drawn by Render_Instanced(),
	// less the ones their records show to be out of sight in the current render pass
	void Queue_Instances(scene::basic_cell const *Cell);
	// makes the records of the instances held by a bucket of a scene cell, with locations relative to Origin
	void Update_Instance_Records(TModel3d const *Model, scene::basic_cell::instance_bucket const &Bucket, glm::dvec3 const &Origin);
	// whether none of the instances of a bucket can pass the culling of the current pass, by the bounds of their records
	bool Bucket_Culled(scene::basic_cell::instance_bucket const &Bucket) const;
	// the sphere around the locations of the visible instances of a bucket, with a margin for the precision of the records
	static scene::bounding_area Bucket_Area(scene::basic_cell::instance_bucket const &Bucket);
	bool Render(TDynamicObject *Dynamic);
	bool Render(TModel3d *Model, material_data const *Material, float const Squaredistance, glm::dvec3 const &Position, glm::vec3 const &Angle);
	bool Render(TModel3d *Model, material_data const *Material, float const Squaredistance);
	void Render(TSubModel *Submodel);
	void Render(TTrack *Track);
	void Render(scene::basic_cell::path_sequence::const_iterator First, scene::basic_cell::path_sequence::const_iterator Last);
	// renders the per-track sleeper instances (TTrack::m_sleeper_local_transforms) via GPU instancing.
	// caller must already have the camera-relative world-space transform set on the matrix stack.
	// no-op if the track has no sleepermodel, Global.SleeperDistance is 0, or the camera is beyond
	// Global.SleeperDistance from the track origin.
	void Render_Sleepers(TTrack *Track);
	// renders the sleepers and switch timbers of the tracks in range (TTrack::m_sleepers3d), laying them first where they're out of date.
	// the parts of the model are drawn in instanced batches for the whole run of tracks, the runs of beams out of sight left out
	void Render_Timbers(scene::basic_cell::path_sequence::const_iterator First, scene::basic_cell::path_sequence::const_iterator Last);
	bool Render_cab(TDynamicObject const *Dynamic, float const Lightlevel, bool const Alpha = false);
	bool Render_interior(bool const Alpha = false);
	bool Render_lowpoly(TDynamicObject *Dynamic, float const Squaredistance, bool const Setup, bool const Alpha = false);
	bool Render_coupler_adapter(TDynamicObject *Dynamic, float const Squaredistance, int const End, bool const Alpha = false);
	void Render(TMemCell *Memcell);
	void Render_Alpha(scene::basic_region *Region);
	void Render_Alpha(cell_sequence::reverse_iterator First, cell_sequence::reverse_iterator Last);
	void Render_Alpha(TAnimModel *Instance);
	void Render_Alpha(TTraction *Traction);
	void Render_Alpha(scene::lines_node const &Lines);
	bool Render_Alpha(TDynamicObject *Dynamic);
	bool Render_Alpha(TModel3d *Model, material_data const *Material, float const Squaredistance, glm::dvec3 const &Position, glm::vec3 const &Angle);
	bool Render_Alpha(TModel3d *Model, material_data const *Material, float const Squaredistance);
	void Render_Alpha(TSubModel *Submodel);
	void Render_Alpha_text(TSubModel *Submodel); // characters of a text display

	// the colour of the picking of the item of the index, and the index of a colour read back
	static glm::vec3 pick_color(std::size_t const Index);
	static std::size_t pick_index(glm::ivec3 const &Color);
	// the range the top view of the editor draws the tracks to, and the parts of the scenery it leaves out
	static float editor_track_view_range(float const Range, bool const Apply);
	static bool editor_ballast_hidden(TTrack const *Track, bool const Colour);
	static bool editor_far_tracks_only(bool const Apply, scene::basic_cell const *Cell, glm::dvec3 const &Camera, double const SceneryLimit);
	// material assigned to the submodel, resolving replacable skins. null_handle if no skin set is bound
	static material_handle submodel_material(TSubModel const *Submodel);

	// what the backend does with the draws of the walk
	// binds the material for the colour passes: its program, its textures and its parameters in model_ubs (opacity, alpha_mult,
	// the params of the submodel or of the lighting of a shape)
	virtual void walk_material(material_handle const Material, TSubModel const *Submodel = nullptr, lighting_data const *Lighting = nullptr) = 0;
	// the same for the shadow passes: the program of the shadows, with the alpha test of the diffuse texture when there is one
	virtual void walk_shadow_material(material_handle const Material) = 0;
	virtual opengl_material const &walk_material_data(material_handle const Material) const = 0;
	virtual opengl_material const &walk_material_data(TSubModel const *Submodel) const = 0;
	virtual void walk_program(walk_shader const Program) = 0;
	virtual void walk_texture(std::size_t const Unit, texture_handle const Texture) = 0;
	// what follows is translucent (blended, without the depth written) or opaque, culled as the pass culls
	virtual void walk_blending(bool const Alpha) = 0;
	// the strength of the sun and the ambient light, for things in the shade (tunnels, vehicles in them), 1: the full light
	virtual void walk_sunlight(float const Factor = 1.f) = 0;
	virtual void walk_environment(TEnvironmentType const Environment = e_flat) = 0;
	// the ambient light of the cab: kept, added to (the lamp of the cab), put back
	virtual void walk_ambient_save() = 0;
	virtual void walk_ambient_add(glm::vec3 const &Addition) = 0;
	virtual void walk_ambient_restore() = 0;
	// the ambient light of the sun, which lights the lines
	virtual glm::vec4 walk_sun_ambient() const = 0;
	// a draw of the chunk with the state of the walk: model_ubs with the modelview of OpenGLMatrices, and the instances of
	// m_current_instance_count / m_current_instance_base when there are any. the second draws the chunks together
	virtual void walk_draw(gfx::geometry_handle const &Geometry) = 0;
	virtual void walk_draw(std::vector<gfx::geometrybank_handle>::iterator First, std::vector<gfx::geometrybank_handle>::iterator Last) = 0;
	// places the modelview matrices of instances in the instance buffer of the frame. returns: base instance of the first of them
	virtual std::uint32_t walk_instances(glm::mat4 const *Modelviews, std::size_t const Count) = 0;
	virtual void walk_cull_face(bool const Enabled) = 0;
	// the glare of a lamp: drawn without the depth test, added to what is there; drawn only where the point of the light was
	// seen (walk_light_query() of the last frame, when the backend has it)
	virtual void walk_glare(bool const Enabled) = 0;
	virtual void walk_glare_draw(TSubModel *Submodel, gfx::geometry_handle const &Geometry) = 0;
	virtual void walk_light_query(TSubModel *Submodel, bool const Begin) = 0;
	// what the backend may draw itself (the gpu culling of the opengl renderer); by default the walk draws everything
	// the start of the walk of a region
	virtual void walk_region_begin() {}
	// the opaque ground of the colour pass is drawn, the editor takes the position of the cursor from the depth
	virtual void walk_editor_cursor() {}
	// a bucket of instances of a cell: false when the backend took the instances (the walk leaves them out); Area is the cell's
	virtual bool walk_bucket(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, bool const Bucketrange, scene::bounding_area const &Area)
	{
		return true;
	}
	// a bucket left out of the pass as wholly out of sight
	virtual void walk_bucket_culled(scene::basic_cell::instance_bucket const &Bucket) {}
	// a model drawn on its own: true when the backend took it. Inrange: the cell of the instance lies within the range of the pass
	virtual bool walk_queue_model(TAnimModel *Instance, bool const Inrange)
	{
		return false;
	}
	// the sleepers of a track: true when the backend took them
	virtual bool walk_queue_sleepers(TTrack *Track)
	{
		return false;
	}
	// the sleepers the walk drew for a track, by their squared distances, and the lod bounds of their model
	virtual void walk_sleepers_drawn(TTrack const &Track, std::vector<float> const &Lodbounds, std::vector<std::pair<float, glm::mat4>> const &Drawn) {}
	// the instances of the cells are about to be drawn: whether the walk reports what it drew of each bucket (walk_instances_compare())
	virtual bool walk_instances_begin()
	{
		return false;
	}
	virtual void walk_instances_compare(scene::basic_cell::instance_bucket_key const &Key, std::vector<std::uint32_t> const &Bandcounts) {}
	virtual void walk_instances_end() {}

	// members
	renderpass_config m_renderpass; // parameters for current render pass
	section_sequence m_sectionqueue; // list of sections in current render pass
	cell_sequence m_cellqueue;
	// frame-level accumulation of per-cell opaque instance buckets. Each visited
	// cell's buckets are merged here keyed by (TModel3d*, skins), so that
	// Render_Instanced() runs once per unique model across the whole pass instead
	// of once per cell -- collapsing many tiny instanced draws into a few large
	// batches. Reused every pass; the vectors are emptied (not freed) at the top of Render(scene::basic_region*).
	std::unordered_map<scene::basic_cell::instance_bucket_key, std::vector<queued_instance>, scene::basic_cell::instance_bucket_key_hash> m_frame_instance_buckets;
	// geometry of neighbouring tracks of a cell which share the material, gathered to be drawn together. reused by each Render(path_sequence)
	std::vector<gfx::geometrybank_handle> m_pathbatch;
	// instance records kept by the buckets of scene cells are good as long as they carry this number.
	// it changes with each frame drawn with the scenery editor active, as the editor moves and resizes instances in place,
	// and whenever scene::node_state_version does, as instances are shown, hidden or drawn on their own
	unsigned int m_instancerecordsversion{1};
	// the model block of the draw being made
	gl::model_ubs model_ubs;
	unsigned int m_framestamp{0}; // id of currently rendered gfx frame
	unsigned int m_timbersweepframe{0}; // the frame the beams of the tracks far away were last let go in
	float m_fogrange{2000.0f};
	// when > 0, the walk is inside a Render_Instanced() call: the draws are made for this many instances from
	// m_current_instance_base onwards. The vertex shader reads per-instance modelview from the instance buffer
	std::size_t m_current_instance_count{0};
	std::uint32_t m_current_instance_base{0};
	// camera-space transforms of the sleepers and timbers drawn by Render_Timbers, by the model and its part or its level of
	// detail; kept from frame to frame to reuse their allocations
	struct timber_bucket
	{
		track_sleepers::beam_model const *model{nullptr};
		int part{-1}; // of a model put together from parts; -1: the whole model
		float band{0.f}; // the squared distance the level of detail of the whole model is seen from
		std::vector<glm::mat4> instances;
	};
	std::vector<timber_bucket> m_timberbuckets;
	// persistent scratch buffer for Render_Instanced(): holds the per-instance
	// camera-space root modelview matrices for the batch currently being built.
	// Kept as a member rather than a function-local so its heap allocation is
	// reused across calls -- clear() retains capacity, so once it has grown to
	// the largest batch seen, steady-state frames perform no allocation here.
	std::vector<glm::mat4> m_instance_modelviews;
	// further persistent scratch buffers for Render_Instanced(): culled instances paired with their
	// squared distance, and the lod bounds of the drawn model which split them into lod bands
	struct instance_survivor
	{
		glm::mat4 modelview;
		float distancesquared;
		std::uint32_t band;
	};
	std::vector<instance_survivor> m_instance_survivors;
	std::vector<float> m_instance_lodbounds;
	std::vector<TSubModel const *> m_instance_lodpending;
	// per sort key (lod band and distance slot): first index in m_instance_modelviews (with end sentinel)
	// and fill cursor; per lod band: nearest distance
	std::vector<std::size_t> m_instance_bandstarts;
	std::vector<std::size_t> m_instance_bandcursors;
	std::vector<float> m_instance_banddistances;
	std::vector<std::uint32_t> m_instance_bandcounts; // reported by Render_Instanced() for the comparison
	// comparison of the gpu culling: the key of the model Render( TAnimModel * ) is called for, and the models it drew in
	// the pass with their scaled squared distances
	scene::basic_cell::instance_bucket_key const *m_modelcomparekey{nullptr};
	std::vector<std::pair<scene::basic_cell::instance_bucket_key, float>> m_modelcpudistances;
	bool m_indirectchanging{false}; // the walk under way met a submodel which may look different in the next frame
	// a vehicle drawn in the colour pass of this frame moved (motion blur)
	bool m_vehiclesmoving{false};
	// set while a preview is drawn: submodels faded out entirely are skipped, instead of being drawn invisible over the stencil
	bool m_previewing{false};
	// the last lighting of the pass placed the cab lamp among the lights: the cab is lit by it, not by an addition to the ambient light
	bool m_cablamplit{false};
	// the shadow pass draws the kept scenery part of the map of a lamp: no vehicles, and not the model of the lamp itself
	bool m_cubescenery{false};
	TAnimModel const *m_cubeskipinstance{nullptr};
	// the shadow pass leaves this vehicle out (the one the headlights of the map being drawn belong to)
	TDynamicObject const *m_shadowskipvehicle{nullptr};
	// what the picking passes drew, by the index of their colours
	std::vector<TSubModel const *> m_pickcontrolsitems;
	std::vector<TSubModel const *> m_picksurfaceitems;
	std::vector<scene::basic_node *> m_picksceneryitems;
	// the quad of the glare of lamps and its texture
	gfx::geometry_handle m_billboardgeometry{0, 0};
	texture_handle m_glaretexture{-1};

  private:
	static float editor_ortho_reach();
};

} // namespace gfx

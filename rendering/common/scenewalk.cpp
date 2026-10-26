/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the walk of the scene shared by the renderers (stage N6 of claude/vulkan14-renderer-plan.md): what is drawn in a pass, in
// which order, with which material and transform. the backends get the draws through the walk_* methods, the opengl renderer
// makes them at once, the vulkan renderer records them. moved out of opengl46renderer.cpp without changes to what it does

#include "stdafx.h"
#include "rendering/common/scenewalk.h"
#include "utilities/color.h"
#include "utilities/Globals.h"
#include "vehicle/Train.h"
#include "vehicle/Camera.h"
#include "vehicle/DynObj.h"
#include "simulation/simulation.h"
#include "world/Track.h"
#include "world/Traction.h"
#include "world/catenary_span.h"
#include "world/MemCell.h"
#include "world/TrackSleepers.h"
#include "model/AnimModel.h"
#include "model/Model3d.h"

namespace gfx
{

// the top view of the editor shows everything this close to the camera, the corners of the screen included
float scene_walk::editor_ortho_reach()
{
	auto const aspect{Global.window_size.x / std::max(1.f, static_cast<float>(Global.window_size.y))};
	return Global.EditorOrthoExtent * std::sqrt(1.f + aspect * aspect) + 500.f;
}

float scene_walk::editor_track_view_range(float const Range, bool const Apply)
{
	if (false == Apply || false == Global.editor_tracks)
		return Range;
	float extra = 50000.0f;
	if (Global.EditorOrtho)
		extra = editor_ortho_reach();
	return Range > extra ? Range : extra;
}

// the top view of the editor may leave the ballast of the railway out, for the ground under the tracks to show
bool scene_walk::editor_ballast_hidden(TTrack const *Track, bool const Colour)
{
	return Colour && EditorModeFlag && Global.EditorOrtho && false == Global.EditorOrthoBallast && (Track->iCategoryFlag & 15) == 1;
}

// the top view draws the scenery as far as the tracks; its models keep their own ranges
bool scene_walk::editor_far_tracks_only(bool const Apply, scene::basic_cell const *Cell, glm::dvec3 const &Camera, double const SceneryLimit)
{
	if (false == Apply || Global.EditorOrtho)
		return false;
	double nearest = glm::length(Cell->m_area.center - Camera) - static_cast<double>(Cell->m_area.radius);
	if (nearest < 0.0)
		nearest = 0.0;
	return nearest > SceneryLimit;
}

material_handle scene_walk::submodel_material( TSubModel const *Submodel ) {

    if( Submodel->m_material >= 0 ) {
        return Submodel->m_material;
    }
    return (
        TSubModel::ReplacableSkinId != nullptr ?
            TSubModel::ReplacableSkinId[ -Submodel->m_material ] :
            null_handle );
}

glm::vec3 scene_walk::pick_color(std::size_t const Index)
{
	return glm::vec3{((Index & 0xff0000) >> 16) / 255.0f, ((Index & 0x00ff00) >> 8) / 255.0f, (Index & 0x0000ff) / 255.0f};
}

std::size_t scene_walk::pick_index(glm::ivec3 const &Color)
{
	return Color.b + (Color.g * 256) + (Color.r * 256 * 256);
}

bool scene_walk::Render_interior( bool const Alpha ) {
    // TODO: early frustum based cull, camera might be pointing elsewhere
    std::vector< std::pair<float, TDynamicObject *> > dynamics;
    auto *dynamic { simulation::Train->Dynamic() };
    // draw interiors of the occupied vehicle, and all vehicles behind it with permanent coupling, in case they're open-ended
    while( dynamic != nullptr ) {

        glm::dvec3 const originoffset { dynamic->vPosition - m_renderpass.pass_camera.position() };
        float const squaredistance{ glm::length2( glm::vec3{ originoffset } / Global.ZoomFactor ) };
        dynamics.emplace_back( squaredistance, dynamic );
        dynamic = dynamic->Next( coupling::permanent );
    }
    // draw also interiors of permanently coupled vehicles in front, if there's any
    dynamic = simulation::Train->Dynamic()->Prev( coupling::permanent );
    while( dynamic != nullptr ) {

        glm::dvec3 const originoffset { dynamic->vPosition - m_renderpass.pass_camera.position() };
        float const squaredistance{ glm::length2( glm::vec3{ originoffset } / Global.ZoomFactor ) };
        dynamics.emplace_back( squaredistance, dynamic );
        dynamic = dynamic->Prev( coupling::permanent );
    }
    if( Alpha ) {
        std::sort(
            std::begin( dynamics ), std::end( dynamics ),
            []( std::pair<float, TDynamicObject *> const &Left, std::pair<float, TDynamicObject *> const &Right ) {
               return ( Left.first ) > ( Right.first ); } );
    }
    for( auto &dynamic : dynamics ) {
        Render_lowpoly( dynamic.second, dynamic.first, true, Alpha );
    }

    return true;
}

bool scene_walk::Render_lowpoly( TDynamicObject *Dynamic, float const Squaredistance, bool const Setup, bool const Alpha ) {

    if( Dynamic->mdLowPolyInt == nullptr ) { return false; }

    // low poly interior
    if( Setup ) {

        TSubModel::iInstance = reinterpret_cast<std::uintptr_t>( Dynamic ); //żeby nie robić cudzych animacji
        TSubModel::iVariantSeed = Dynamic->variant_seed();
        glm::dvec3 const originoffset{ Dynamic->vPosition - m_renderpass.pass_camera.position() };

        Dynamic->ABuLittleUpdate( Squaredistance ); // ustawianie zmiennych submodeli dla wspólnego modelu

        glm::mat4 mv = OpenGLMatrices.data( GL_MODELVIEW );

        ::glPushMatrix();
        ::glTranslated( originoffset.x, originoffset.y, originoffset.z );
		::glMultMatrixd( glm::value_ptr(Dynamic->mMatrix) );
    }
    // HACK: reduce light level for vehicle interior if there's strong global lighting source
    if( false == Alpha ) {
/*
        auto const luminance { static_cast<float>( std::min(
            ( Dynamic->fShade > 0.0 ? Dynamic->fShade * Global.fLuminance : Global.fLuminance ),
            Global.fLuminance - 0.5 * ( std::max( 0.3, Global.fLuminance - Global.Overcast ) ) ) ) };
*/
        auto const luminance{ static_cast<float>(
            0.5
            * ( std::max(
                0.3,
                ( Global.fLuminance - Global.Overcast )
                * ( Dynamic->fShade > 0.0 ?
                    Dynamic->fShade :
                    1.0 ) ) ) ) };
        walk_sunlight(
            std::clamp( (
                Dynamic->fShade > 0.f ?
                    Dynamic->fShade :
                    1.f )
                - luminance,
                0.f, 1.f ) );
        Render( Dynamic->mdLowPolyInt, Dynamic->Material(), Squaredistance );
        // HACK: if the model has low poly interior, we presume the load is placed inside and also affected by reduced light level
        if( Dynamic->mdLoad ) {
            // renderowanie nieprzezroczystego ładunku
            Render( Dynamic->mdLoad, Dynamic->Material(), Squaredistance, { 0.f, Dynamic->LoadOffset, 0.f }, {} );
        }
        walk_sunlight( Dynamic->fShade > 0.f ? Dynamic->fShade : 1.f );
    }
    else {
//        Render_Alpha( Dynamic->mdLowPolyInt, Dynamic->Material(), Squaredistance );
        // HACK: some models have windows included as part of the main model instead of lowpoly
        if( Dynamic->mdModel ) {
            // main model
            Render_Alpha( Dynamic->mdModel, Dynamic->Material(), Squaredistance );
        }
    }

    if( Setup ) {

        if( Dynamic->fShade > 0.0f ) {
            // restore regular light level
            walk_sunlight();
        }

        ::glPopMatrix();

        // TODO: check if this reset is needed. In theory each object should render all parts based on its own instance data anyway?
        if( Dynamic->btnOn ) {
            Dynamic->TurnOff(); // przywrócenie domyślnych pozycji submodeli
        }
    }

    return true;
}

bool scene_walk::Render_coupler_adapter( TDynamicObject *Dynamic, float const Squaredistance, int const End, bool const Alpha ) {

    if( Dynamic->m_coupleradapters[ End ] == nullptr ) { return false; }

    auto const position { glm::dvec3 {
        0.f,
        Dynamic->MoverParameters->Couplers[ End ].adapter_height,
        ( Dynamic->MoverParameters->Couplers[ End ].adapter_length + Dynamic->MoverParameters->Dim.L * 0.5 ) * ( End == end::front ? 1 : -1 ) } };

    auto const angle { glm::vec3{
        0,
        ( End == end::front ? 0 : 180 ),
        0 } };

    if( Alpha ) {
        Render_Alpha( Dynamic->m_coupleradapters[ End ], Dynamic->Material(), Squaredistance, position, angle );
    }
    else {
        Render( Dynamic->m_coupleradapters[ End ], Dynamic->Material(), Squaredistance, position, angle );
    }

    return true;
}

void scene_walk::Render(scene::basic_region *Region)
{

	m_sectionqueue.clear();
	m_cellqueue.clear();
	// discard last pass's accumulated instance buckets before this pass starts.
	// the vectors themselves are kept, so their allocations get reused by the following passes
	for( auto &bucket : m_frame_instance_buckets ) {
		bucket.second.clear();
	}
	// the backend gets ready for the pass (the gpu culling of the opengl renderer)
	walk_region_begin();
	m_modelcpudistances.clear();
	// build a list of region sections to render
	glm::vec3 const cameraposition{m_renderpass.pass_camera.position()};
	auto const camerax = static_cast<int>(std::floor(cameraposition.x / scene::EU07_SECTIONSIZE + scene::EU07_REGIONSIDESECTIONCOUNT / 2));
	auto const cameraz = static_cast<int>(std::floor(cameraposition.z / scene::EU07_SECTIONSIZE + scene::EU07_REGIONSIDESECTIONCOUNT / 2));
	float section_range = m_renderpass.draw_range * Global.fDistanceFactor;
	section_range = editor_track_view_range(section_range, EditorModeFlag && (m_renderpass.draw_mode == rendermode::color || m_renderpass.draw_mode == rendermode::pickscenery));
	int const segmentcount = 2 * static_cast<int>(std::ceil(section_range / scene::EU07_SECTIONSIZE));
	int const originx = camerax - segmentcount / 2;
	int const originz = cameraz - segmentcount / 2;

	for (int row = originz; row <= originz + segmentcount; ++row)
	{
		if (row < 0)
		{
			continue;
		}
		if (row >= scene::EU07_REGIONSIDESECTIONCOUNT)
		{
			break;
		}
		for (int column = originx; column <= originx + segmentcount; ++column)
		{
			if (column < 0)
			{
				continue;
			}
			if (column >= scene::EU07_REGIONSIDESECTIONCOUNT)
			{
				break;
			}
			auto *section{Region->m_sections[row * scene::EU07_REGIONSIDESECTIONCOUNT + column]};
			if ((section != nullptr) && (m_renderpass.pass_camera.visible(section->m_area)))
			{
				m_sectionqueue.emplace_back(section);
			}
		}
	}

	switch (m_renderpass.draw_mode)
	{
	case rendermode::color:
	{
		Render(std::begin(m_sectionqueue), std::end(m_sectionqueue));
		// draw queue is filled while rendering sections
		if (EditorModeFlag)
		{
			// when editor mode is active calculate world position of the cursor
			// at this stage the z-buffer is filled with only ground geometry
			walk_editor_cursor();
		}
		// draw opaque cells front-to-back: with the depth test enabled this
		// lets the GPU reject hidden fragments early, before the (expensive)
		// lit fragment shader runs on them. Order is irrelevant to the final
		// image for opaque geometry, so this is purely a fill-rate win.
		std::sort( std::begin( m_cellqueue ), std::end( m_cellqueue ),
			[]( distancecell_pair const &Left, distancecell_pair const &Right ) { return Left.first < Right.first; } );
		Render(std::begin(m_cellqueue), std::end(m_cellqueue));
		break;
	}
	case rendermode::shadows:
    case rendermode::pickscenery:
	{
		// these render modes don't bother with lights
		Render(std::begin(m_sectionqueue), std::end(m_sectionqueue));
		// they can also skip queue sorting, as they only deal with opaque geometry
		// NOTE: there's benefit from rendering front-to-back, but is it significant enough? TODO: investigate
		Render(std::begin(m_cellqueue), std::end(m_cellqueue));
		break;
	}
	case rendermode::reflections:
	{
		Render(std::begin(m_sectionqueue), std::end(m_sectionqueue));
        if( Global.reflectiontune.fidelity >= 1 ) {
            Render(std::begin(m_cellqueue), std::end(m_cellqueue));
        }
		break;
	}
	case rendermode::pickcontrols:
	default:
	{
		// no need to render anything ourside of the cab in control picking mode
		break;
	}
	}
}

void scene_walk::Render(section_sequence::iterator First, section_sequence::iterator Last)
{

	switch (m_renderpass.draw_mode)
	{
	case rendermode::color:
	case rendermode::reflections:
    case rendermode::shadows:
		break;
	case rendermode::pickscenery:
	{
		// non-interactive scenery elements get neutral colour
		model_ubs.param[0] = colors::none;
		break;
	}
	default:
		break;
	}

	while (First != Last)
	{

		auto *section = *First;
		section->create_geometry();

		// render shapes held by the section
		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		case rendermode::reflections:
		case rendermode::shadows:
        case rendermode::pickscenery:
		{
			if (false == section->m_shapes.empty())
			{
				// since all shapes of the section share center point we can optimize out a few calls here
				::glPushMatrix();
				auto const originoffset{section->m_area.center - m_renderpass.pass_camera.position()};
				::glTranslated(originoffset.x, originoffset.y, originoffset.z);
				// render
				for (auto const &shape : section->m_shapes)
				{
					Render(shape, true);
				}
				// post-render cleanup
				::glPopMatrix();
			}
			break;
		}
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}

		// add the section's cells to the cell queue
		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		case rendermode::shadows:
        case rendermode::pickscenery:
		{
            // TBD, TODO: refactor into a method to reuse below?
			for (auto &cell : section->m_cells)
			{
				if ((true == cell.m_active) && (m_renderpass.pass_camera.visible(cell.m_area)))
				{
					// store visible cells with content as well as their current distance, for sorting later
					m_cellqueue.emplace_back(glm::length2(m_renderpass.pass_camera.position() - cell.m_area.center), &cell);
				}
			}
			break;
		}
		case rendermode::reflections:
        {
            // we can skip filling the cell queue if reflections pass isn't going to use it
            if( Global.reflectiontune.fidelity == 0 ) { break; }
			for (auto &cell : section->m_cells)
			{
				if ((true == cell.m_active) && (m_renderpass.pass_camera.visible(cell.m_area)))
				{
					// store visible cells with content as well as their current distance, for sorting later
					m_cellqueue.emplace_back(glm::length2(m_renderpass.pass_camera.position() - cell.m_area.center), &cell);
				}
			}
			break;
        }
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}
		// proceed to next section
		++First;
	}

	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
    {
		break;
	}
	default:
	{
		break;
	}
	}
}

void scene_walk::Render(cell_sequence::iterator First, cell_sequence::iterator Last)
{

	// cache initial iterator for the second sweep
	auto first{First};
	auto const scenery_limit{static_cast<double>(Global.BaseDrawRange) * Global.fDistanceFactor + 250.0};
	bool const far_paths{EditorModeFlag && Global.editor_tracks && (m_renderpass.draw_mode == rendermode::color || m_renderpass.draw_mode == rendermode::pickscenery)};
	glm::dvec3 const camera{m_renderpass.pass_camera.position()};
	// first pass draws elements which we know are located in section banks, to reduce vbo switching
	while (First != Last)
	{

		auto *cell = First->second;
		// przeliczenia animacji torów w sektorze
		cell->RaAnimate(m_framestamp);
		bool const tracks_only{editor_far_tracks_only(far_paths, cell, camera, scenery_limit)};

		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		{
			// since all shapes of the section share center point we can optimize out a few calls here
			::glPushMatrix();
			auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
			::glTranslated(originoffset.x, originoffset.y, originoffset.z);

			// render
			// opaque non-instanced shapes
			if (false == tracks_only)
			{
				for (auto const &shape : cell->m_shapesopaque)
				{
					Render(shape, false);
				}
			}
			// tracks
			// TODO: update after path node refactoring
			Render(std::begin(cell->m_paths), std::end(cell->m_paths));
			// post-render cleanup
			::glPopMatrix();

			break;
		}
        case rendermode::reflections:
        {
			// since all shapes of the section share center point we can optimize out a few calls here
			::glPushMatrix();
			auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
			::glTranslated(originoffset.x, originoffset.y, originoffset.z);

			// render
			// opaque non-instanced shapes
			for (auto const &shape : cell->m_shapesopaque)
				Render(shape, false);
			// tracks: the ray traced reflections (the one user of this mode) show the trackbed and the platforms made of roads
			Render(std::begin(cell->m_paths), std::end(cell->m_paths));

			// post-render cleanup
			::glPopMatrix();

			break;
        }
		case rendermode::shadows:
		{
			// since all shapes of the section share center point we can optimize out a few calls here
			::glPushMatrix();
			auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
			::glTranslated(originoffset.x, originoffset.y, originoffset.z);

			// render
			// opaque non-instanced shapes
			for (auto const &shape : cell->m_shapesopaque)
				Render(shape, false);
			// tracks
			Render(std::begin(cell->m_paths), std::end(cell->m_paths));

			// post-render cleanup
			::glPopMatrix();

			break;
		}
		case rendermode::pickscenery:
		{
			// same procedure like with regular render, but editor-enabled nodes receive custom colour used for picking
			// since all shapes of the section share center point we can optimize out a few calls here
			::glPushMatrix();
			auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
			::glTranslated(originoffset.x, originoffset.y, originoffset.z);
			// render
			// opaque non-instanced shapes
			// non-interactive scenery elements get neutral colour
			if (false == tracks_only)
			{
				model_ubs.param[0] = colors::none;
				for (auto const &shape : cell->m_shapesopaque)
					Render(shape, false);
			}
			// tracks
			for (auto *path : cell->m_paths)
			{
				model_ubs.param[0] = glm::vec4(pick_color(m_picksceneryitems.size() + 1), 1.0f);
				Render(path);
			}
			// post-render cleanup
			::glPopMatrix();
			break;
		}
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}

		++First;
	}
	// second pass draws elements with their own vbos
	while (first != Last)
	{

		auto const *cell = first->second;

		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
        case rendermode::shadows:
        {
			if (editor_far_tracks_only(far_paths, cell, camera, scenery_limit))
				break;
            // TBD, TODO: refactor in to a method to reuse in branch below?
			// opaque parts of instanced models -- accumulate this cell's buckets
			// into the frame-level map; the actual Render_Instanced() calls are
			// issued once per unique model after the cell loop (see flush below).
			//
			// Cell-level far-distance pre-cull: when the whole cell lies beyond
			// the instance draw distance, every instance in it would fail the
			// per-instance drawdistancethreshold test inside Render_Instanced(),
			// so there is no point merging its buckets into the frame map. The
			// test reproduces Render_Instanced()'s distance maths exactly -- same
			// ZoomFactor / fDistanceFactor scaling, same +250 margin -- applied to
			// the nearest point of the cell's bounding sphere. basic_cell::enclose_area
			// guarantees m_area.radius >= |m_area.center - instance.location()| for
			// every contained instance, so the nearest-point distance is a true
			// lower bound on every instance's distance: the cull can never drop a
			// cell that still holds a drawable instance. Shadows measure from the
			// real (viewport) camera, matching Render_Instanced()'s shadow branch;
			// every other gated mode measures from the pass camera. Non-instanced
			// scenery drawn by the cpu and vehicles below keep their own per-node
			// culling and are intentionally left untouched; the non-instanced models
			// culled on the gpu (Queue_Model()) are left out with the cell the same way.
			{
				auto const &distancecamera = (
					m_renderpass.draw_mode == rendermode::shadows
						? m_renderpass.viewport_camera
						: m_renderpass.pass_camera );
				auto const cellcenterdistance { glm::length( cell->m_area.center - distancecamera.position() ) };
				auto const cellnearestdistance { std::max( 0.0, cellcenterdistance - cell->m_area.radius ) };
				auto const cellnearestdistancesquared {
					( cellnearestdistance * cellnearestdistance )
					/ ( static_cast<double>( Global.ZoomFactor ) * static_cast<double>( Global.ZoomFactor ) )
					/ static_cast<double>( Global.fDistanceFactor ) };
				auto const cellinrange { cellnearestdistancesquared <= sq( static_cast<double>( m_renderpass.draw_range ) + 250.0 ) };
				if( cellinrange ) {
					Queue_Instances( cell );
				}
				// remaining (non-instanceable) opaque instance nodes go through the per-node path, or with the instances
				// to the gpu culling, each on its own
				for (auto *instance : cell->m_instancesopaque)
				{
					if( instance->m_instanceable ) { continue; } // already handled via bucket
					// the model of a lamp would shadow everything in the map of the lamp
					if( instance == m_cubeskipinstance ) { continue; }
					if( walk_queue_model( instance, cellinrange ) ) { continue; }
					Render(instance);
					m_modelcomparekey = nullptr;
				}
			}
			// opaque parts of vehicles. the kept scenery part of the map of a lamp is drawn without them
			if( false == m_cubescenery ) {
				for (auto *path : cell->m_paths)
				{
					for (auto *dynamic : path->Dynamics)
					{
						// the vehicle of the headlights a map is drawn for would cover all of it
						if (dynamic == m_shadowskipvehicle)
							continue;
						Render(dynamic);
					}
				}
			}
			break;
		}
        case rendermode::reflections:
        {
            // a cell lying whole beyond the range the reflections draw model instances at is left out, as each
            // of its instances would fail the range test done for it in Render_Instanced() and Render( TAnimModel * ).
            // like the cull of the regular passes above, the test goes by the nearest point of the bounding sphere
            // of the cell, which holds the locations of all its instances
            auto const cellnearestdistance { std::max( 0.0, glm::length( cell->m_area.center - m_renderpass.pass_camera.position() ) - cell->m_area.radius ) };
            if( ( Global.reflectiontune.fidelity >= 1 )
             && ( cellnearestdistance <= Global.reflectiontune.range_instances ) ) {
                // opaque parts of instanced models -- accumulate into the
                // frame-level map; flushed once per unique model after the loop.
                Queue_Instances( cell );
                for( auto *instance : cell->m_instancesopaque ) {
                    if( instance->m_instanceable ) { continue; }
                    if( walk_queue_model( instance, true ) ) { continue; }
                    Render( instance );
                    m_modelcomparekey = nullptr;
                }
            }
            if( Global.reflectiontune.fidelity >= 2 ) {
                // opaque parts of vehicles, but the one the backend leaves out (the vehicle of the cab, seen from the inside)
                for( auto *path : cell->m_paths ) {
                    for( auto *dynamic : path->Dynamics ) {
                        if( dynamic == m_shadowskipvehicle ) { continue; }
                        Render( dynamic );
                    }
                }
            }
			break;
        }
        case rendermode::pickscenery:
		{
			if (editor_far_tracks_only(far_paths, cell, camera, scenery_limit))
				break;
			// opaque parts of instanced models
			// same procedure like with regular render, but each node receives custom colour used for picking.
			// picking always uses the per-instance path because each instance needs a unique pick colour;
			// batching would assign the same colour to every instance in the bucket.
			for (auto *instance : cell->m_instancesopaque)
			{
				model_ubs.param[0] = glm::vec4(pick_color(m_picksceneryitems.size() + 1), 1.0f);
				Render(instance);
			}
			// vehicles aren't included in scenery picking for the time being
			break;
		}
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}

		++first;
	}

	// flush accumulated instance buckets: issue one Render_Instanced() per unique
	// (TModel3d*, skins) key across every cell visited in this pass, instead of
	// one call per cell. All per-instance frustum/distance/pixel-area culling
	// still happens inside Render_Instanced(), so correctness is unchanged -- only
	// the number of calls and glBufferSubData round-trips drops sharply.
	// (For pickscenery/pickcontrols modes the map stays empty, so this is a no-op.)
	// what the backend draws of the instances itself (the gpu culling of the opengl renderer), and the comparison with it
	auto const compare { walk_instances_begin() };
	for( auto &bucket : m_frame_instance_buckets ) {
		if( bucket.second.empty() ) { continue; }
		Render_Instanced( bucket.first.pModel, bucket.second, ( compare ? &m_instance_bandcounts : nullptr ) );
		if( compare ) {
			walk_instances_compare( bucket.first, m_instance_bandcounts );
		}
		bucket.second.clear();
	}
	walk_instances_end();
	m_modelcpudistances.clear();
}

void scene_walk::Render(scene::shape_node const &Shape, bool const Ignorerange)
{
	auto const &data{Shape.data()};

	// a shape lying wholly outside the view of the pass draws nothing. the section and the cell holding it were tested as a
	// whole, while a section holds the chunks of the heightmap terrain (250 m each, 16 of them) and the large shapes of the
	// scenery, drawn in every pass the section is seen in: in a cascade of the sun shadows or a face of the environment map
	// most of them are out of view. a shape merged from others has its radius invalidated (-1) and keeps being drawn
	if ((data.area.radius > 0.f) && (false == m_renderpass.pass_camera.visible(data.area)))
	{
		return;
	}

	if (false == Ignorerange)
	{
		double distancesquared;
		switch (m_renderpass.draw_mode)
		{
		case rendermode::shadows:
        {
			// 'camera' for the light pass is the light source, but we need to draw what the 'real' camera sees
			distancesquared = glm::length2((data.area.center - m_renderpass.viewport_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
			break;
		}
        case rendermode::reflections:
        {
            // reflection mode draws simplified version of the shapes, by artificially increasing view range
            distancesquared =
                // TBD, TODO: bind offset value with setting variable?
                sq(EU07_REFLECTIONFIDELITYOFFSET)
                + glm::length2( ( data.area.center - m_renderpass.pass_camera.position() ) );
/*
                // TBD: take into account distance multipliers?
                / Global.fDistanceFactor;
*/
            break;
        }
		default:
		{
			distancesquared = glm::length2((data.area.center - m_renderpass.pass_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
			break;
		}
		}
		if ((distancesquared < data.rangesquared_min) || (distancesquared >= data.rangesquared_max))
		{
			return;
		}
	}

	// setup
	switch (m_renderpass.draw_mode)
	{
	case rendermode::color:
	case rendermode::reflections:
		walk_material(data.material, nullptr, &Shape.data().lighting );
		break;
	case rendermode::shadows:
		// skip if the shadow caster rank is too low for currently set threshold
		if( walk_material_data( data.material ).shadow_rank > Global.gfx_shadow_rank_cutoff )
			return;
		walk_shadow_material(data.material);
		break;
	case rendermode::pickscenery:
	case rendermode::pickcontrols:
		walk_program( walk_shader::pick );
		break;
	default:
		break;
	}
	// render

	walk_draw(data.geometry);
	// debug data
	++m_renderpass.draw_stats.shapes;
	++m_renderpass.draw_stats.drawcalls;
}

void scene_walk::Render(TAnimModel *Instance)
{

	if (false == Instance->m_visible) { return; }
    if( false == m_renderpass.pass_camera.visible( Instance->m_area ) ) { return; }

    double distancesquared;
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
    {
        // 'camera' for the light pass is the light source, but we need to draw what the 'real' camera sees
		distancesquared = glm::length2((Instance->location() - m_renderpass.viewport_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
		break;
	}
    case rendermode::reflections:
    {
        // reflection mode draws simplified version of the shapes, by artificially increasing view range
        // it also ignores zoom settings and distance multipliers
        // TBD: take into account distance multipliers?
        distancesquared = glm::length2((Instance->location() - m_renderpass.pass_camera.position())) /* / Global.fDistanceFactor */;
        // NOTE: arbitrary draw range limit
        if( distancesquared > Global.reflectiontune.range_instances * Global.reflectiontune.range_instances ) {
            return;
        }
        // TBD, TODO: bind offset value with setting variable?
        distancesquared += sq(EU07_REFLECTIONFIDELITYOFFSET);
        break;
    }
	default:
	{
		distancesquared = glm::length2((Instance->location() - m_renderpass.pass_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
		break;
	}
	}
	if ((distancesquared < Instance->m_rangesquaredmin) || (distancesquared >= Instance->m_rangesquaredmax))
	{
		return;
	}
     // crude way to reject early items too far to affect the output (mostly relevant for shadow passes)
    auto const drawdistancethreshold{ m_renderpass.draw_range + 250 };
    if( distancesquared > sq(drawdistancethreshold) ) {
        return;
    }
   // second stage visibility cull, reject modelstoo far away to be noticeable
    auto const radiussquared { Instance->radius() * Instance->radius() };
    if( radiussquared * Global.ZoomFactor / distancesquared < 0.003 * 0.003 ) {
        return;
    }
    if( m_modelcomparekey != nullptr ) {
        // queued in the gpu culling as well, to compare with (Queue_Model())
        m_modelcpudistances.emplace_back( *m_modelcomparekey, static_cast<float>( distancesquared ) );
    }

	switch (m_renderpass.draw_mode)
	{
	case rendermode::pickscenery:
	{
		// add the node to the pick list
		m_picksceneryitems.emplace_back(Instance);
		break;
	}
	default:
	{
		break;
	}
	}

	Instance->RaAnimate(m_framestamp); // jednorazowe przeliczenie animacji
	Instance->RaPrepare();
	if (Instance->pModel)
	{
		// per-instance uniform scale: applied between the rotation and the
		// submodel-local transform chain. Pushing it on the matrix stack here
		// means Render(TModel3d, ..., position, angle) doesn't need a new
		// signature and the same scale flows naturally into Render_Alpha and
		// any other recursive callers.
		auto const &scale = Instance->Scale();
		bool const scaled = ( scale.x != 1.0f || scale.y != 1.0f || scale.z != 1.0f );
		if( scaled ) {
			// Per-axis scale applied between rotation and the submodel-local
			// chain: build the per-instance transform here so we can inject
			// glScalef before the submodel walk.
			::glPushMatrix();
			auto const Position = Instance->location() - m_renderpass.pass_camera.position();
			auto const Angle = Instance->vAngle;
			::glTranslated( Position.x, Position.y, Position.z );
			if( Angle.y != 0.0 ) ::glRotated( Angle.y, 0.f, 1.f, 0.f );
			if( Angle.x != 0.0 ) ::glRotated( Angle.x, 1.f, 0.f, 0.f );
			if( Angle.z != 0.0 ) ::glRotated( Angle.z, 0.f, 0.f, 1.f );
			::glScalef( scale.x, scale.y, scale.z );
			Render( Instance->pModel, Instance->Material(), distancesquared );
			::glPopMatrix();
		}
		else {
			// fast path for unscaled (the common case): existing behaviour
			Render(Instance->pModel, Instance->Material(), distancesquared, Instance->location() - m_renderpass.pass_camera.position(), Instance->vAngle);
		}
	    // debug data
	    ++m_renderpass.draw_stats.models;
	}
}

// adds the bucketed instances of a scene cell to the frame-level buckets, for Render_Instanced() to draw.
// Render_Instanced() culls each instance it's given by its record. the tests it makes are repeated here, with some slack
// for the single precision of the records, and the instances which fail are left out of the queue
void scene_walk::Queue_Instances( scene::basic_cell const *Cell )
{
	// the editor moves and resizes the instances in place. the records are made anew for each frame then,
	// but aren't relied on to leave instances out of the queue
	auto const userecords { false == EditorModeFlag };
	auto const reflections { m_renderpass.draw_mode == rendermode::reflections };
	// same choices as in Render_Instanced(): shadows measure the distance from the real camera, reflections
	// ignore zoom and distance factor, test the plain distance against their own range and add the fidelity offset
	auto const &distancecamera { m_renderpass.draw_mode == rendermode::shadows ? m_renderpass.viewport_camera : m_renderpass.pass_camera };
	glm::dvec3 const cameraposition { distancecamera.position() };
	auto const distancescale { reflections ? 1.0 : 1.0 / ( static_cast<double>( Global.ZoomFactor ) * static_cast<double>( Global.ZoomFactor ) * static_cast<double>( Global.fDistanceFactor ) ) };
	auto const distanceoffset { reflections ? sq( static_cast<double>( EU07_REFLECTIONFIDELITYOFFSET ) ) : 0.0 };
	auto const slack { 1.001 };
	auto const distancelimit { slack * ( reflections ? sq( static_cast<double>( Global.reflectiontune.range_instances ) ) : sq( static_cast<double>( m_renderpass.draw_range ) + 250.0 ) ) };
	// up close the rounding of the records is too large a part of the distance for the slack to cover it
	auto const distancetested { sq( 64.0 ) };
	// a bucket whose visible instances all fail the tests of the culling by the bounds of their records (Bucket_Culled()) is left
	// out of the pass before the culling. the bounds are made with the records, which the editor doesn't let go by
	auto const buckettests { Global.gfx_gpucull_bucketrange && userecords };

	for( auto const &bucket : Cell->m_instancebuckets_opaque ) {
		auto const &instancebucket { bucket.second };
		if( ( instancebucket.recordsversion != m_instancerecordsversion )
		 || ( instancebucket.records.size() != instancebucket.instances.size() ) ) {
			Update_Instance_Records( bucket.first.pModel, instancebucket, Cell->m_area.center );
		}
		if( buckettests && Bucket_Culled( instancebucket ) ) {
			walk_bucket_culled( instancebucket );
			continue;
		}
		// the backend may draw the instances of the bucket itself (the gpu culling of the opengl renderer); the lod bands drawn go
		// by the distances the locations of the instances can be at: of their own bounds, or of the cell, which holds them too
		if( false == walk_bucket( bucket.first, instancebucket, buckettests, Cell->m_area ) ) { continue; }
		auto const &records { instancebucket.records };
		std::vector<queued_instance> *dest { nullptr }; // looked up when there's something to put in it
		for( std::size_t idx = 0; idx < records.size(); ++idx ) {
			if( userecords ) {
				auto const &record { records[ idx ] };
				auto const center { instancebucket.recordorigin + glm::dvec3{ record.position } };
				auto const distancesquared { glm::length2( center - cameraposition ) };
				if( distancesquared > distancetested ) {
					auto const scaleddistancesquared { distancesquared * distancescale };
					if( scaleddistancesquared > distancelimit ) { continue; }
					if( scaleddistancesquared + distanceoffset >= slack * std::min( instancebucket.lodrangesquared, record.rangesquaredmax ) ) { continue; }
				}
				if( false == m_renderpass.pass_camera.visible( scene::bounding_area{ center, record.radius } ) ) { continue; }
			}
			if( dest == nullptr ) {
				dest = &m_frame_instance_buckets[ bucket.first ];
			}
			dest->push_back( { &instancebucket, idx } );
		}
	}
}

// makes the records of the instances held by a bucket of a scene cell, with locations relative to Origin
void scene_walk::Update_Instance_Records( TModel3d const *Model, scene::basic_cell::instance_bucket const &Bucket, glm::dvec3 const &Origin )
{
	// a submodel is drawn only closer than its fSquareMaxDist, see Render( TSubModel * ). past the largest of these
	// there's nothing of the model left to draw, which makes it another limit of the range of its instances
	auto lodrangesquared { ( Model != nullptr && Model->Root != nullptr ) ? 0.f : std::numeric_limits<float>::max() };
	m_instance_lodpending.clear();
	m_instance_lodpending.push_back( Model != nullptr ? Model->Root : nullptr );
	while( false == m_instance_lodpending.empty() ) {
		auto const *submodel = m_instance_lodpending.back();
		m_instance_lodpending.pop_back();
		if( submodel == nullptr ) { continue; }
		lodrangesquared = std::max( lodrangesquared, submodel->fSquareMaxDist );
		m_instance_lodpending.push_back( submodel->Child );
		m_instance_lodpending.push_back( submodel->Next );
	}
	Bucket.lodrangesquared = lodrangesquared;
	Bucket.recordorigin = Origin;

	auto const tofloat = []( double const Value ) {
		return static_cast<float>( std::min( Value, static_cast<double>( std::numeric_limits<float>::max() ) ) ); };

	Bucket.records.clear();
	Bucket.records.reserve( Bucket.instances.size() );
	// the range of the bucket: past it none of its instances is drawn, by its own range or by the submodels of the model
	auto rangesquared { 0.f };
	for( auto *instance : Bucket.instances ) {
		scene::basic_cell::instance_record record;
		if( instance != nullptr ) {
			record.position = packed_vec3{ instance->location() - Origin };
			record.radius = instance->radius();
			record.rangesquaredmin = tofloat( instance->m_rangesquaredmin );
			record.rangesquaredmax = tofloat( instance->m_rangesquaredmax );
			rangesquared = std::max( rangesquared, record.rangesquaredmax );
			record.flags =
				( instance->visible() ? scene::basic_cell::instance_visible : 0u )
				| ( instance->m_instanceable ? scene::basic_cell::instance_instanceable : 0u );
			auto const &rotationscale { instance->rotation_scale() };
			for( int column = 0; column < 3; ++column ) {
				record.rotationscale[ column ] = rotationscale[ column ];
			}
		}
		// an empty slot keeps its record without flags, and isn't drawn
		Bucket.records.emplace_back( record );
	}
	Bucket.rangesquared = std::min( rangesquared, lodrangesquared );

	// bounds of the visible instances, for Bucket_Culled(): the centre of the box around their locations, and the spheres
	// around it which hold the locations and the instances whole. the gpu leaves out the ones which aren't visible
	glm::dvec3 low { std::numeric_limits<double>::max() };
	glm::dvec3 high { std::numeric_limits<double>::lowest() };
	auto radiusmax { 0.f };
	for( auto const &record : Bucket.records ) {
		if( ( record.flags & scene::basic_cell::instance_visible ) == 0 ) { continue; }
		glm::dvec3 const position { record.position };
		low = glm::min( low, position );
		high = glm::max( high, position );
		radiusmax = std::max( radiusmax, record.radius );
	}
	Bucket.recordlocationradius = 0.f;
	Bucket.recordradius = -1.f;
	Bucket.recordradiusmax = radiusmax;
	if( low.x <= high.x ) {
		auto const center { 0.5 * ( low + high ) };
		auto locationradius { 0.0 };
		auto radius { 0.0 };
		for( auto const &record : Bucket.records ) {
			if( ( record.flags & scene::basic_cell::instance_visible ) == 0 ) { continue; }
			auto const distance { glm::length( glm::dvec3{ record.position } - center ) };
			locationradius = std::max( locationradius, distance );
			radius = std::max( radius, distance + record.radius );
		}
		Bucket.recordcenter = Origin + center;
		Bucket.recordlocationradius = static_cast<float>( locationradius );
		Bucket.recordradius = static_cast<float>( radius );
	}
	else {
		Bucket.recordcenter = Origin;
	}
	Bucket.recordsversion = m_instancerecordsversion;
	Bucket.gpustale = true;
}

// whether none of the instances of a bucket can pass the tests of the culling of the pass (Render_Instanced(), on the gpu
// instance_cull.comp): by the bounds of their records the visible ones lie wholly out of the frustum of the pass, beyond their
// range or the draw range, or are too small on the screen. the tests are those of the culling with the nearest location and
// the largest instance of the bucket, with a margin for the single precision of the records and of the gpu, so they leave out
// only buckets which the culling would leave out whole
bool scene_walk::Bucket_Culled( scene::basic_cell::instance_bucket const &Bucket ) const
{
	if( Bucket.recordradius < 0.f ) { return true; }

	auto const slack { 1.001 };
	auto const margin { 0.5 }; // metres
	// the planes of the frustum in world coordinates, the ones the gpu culling moves to the camera of the pass
	auto const &planes { m_renderpass.pass_camera.frustum().planes() };
	auto const &center { Bucket.recordcenter };
	auto const radius { slack * Bucket.recordradius + margin };
	for( auto const *plane : planes ) {
		if( plane[ 0 ] * center.x + plane[ 1 ] * center.y + plane[ 2 ] * center.z + plane[ 3 ] <= -radius ) { return true; }
	}

	// the same choices as in Render_Instanced(): shadows measure the distance from the real camera, reflections ignore zoom and
	// distance factor, test the plain distance against their own range and add the fidelity offset
	auto const reflections { m_renderpass.draw_mode == rendermode::reflections };
	auto const &distancecamera { m_renderpass.draw_mode == rendermode::shadows ? m_renderpass.viewport_camera : m_renderpass.pass_camera };
	auto const nearest { std::max( 0.0, glm::length( center - distancecamera.position() ) - ( slack * Bucket.recordlocationradius + margin ) ) };
	auto const zoom { static_cast<double>( Global.ZoomFactor ) };
	auto const scaledsquared { nearest * nearest * ( reflections ? 1.0 : 1.0 / ( zoom * zoom * static_cast<double>( Global.fDistanceFactor ) ) ) };
	if( reflections && ( scaledsquared > slack * sq( static_cast<double>( Global.reflectiontune.range_instances ) ) ) ) { return true; }
	auto const distancesquared { scaledsquared + ( reflections ? sq( static_cast<double>( EU07_REFLECTIONFIDELITYOFFSET ) ) : 0.0 ) };
	if( distancesquared >= slack * Bucket.rangesquared ) { return true; }
	if( distancesquared > slack * sq( static_cast<double>( m_renderpass.draw_range ) + 250.0 ) ) { return true; }
	// the largest instance at the nearest location takes the largest part of the view
	auto const radiusmax { static_cast<double>( Bucket.recordradiusmax ) };
	if( radiusmax * radiusmax * zoom * slack < 0.003 * 0.003 * distancesquared ) { return true; }

	return false;
}

// the sphere around the locations of the visible instances of a bucket, with the margin of Bucket_Culled(): their distances
// from it set the lod bands the gpu culling can put them in
scene::bounding_area scene_walk::Bucket_Area( scene::basic_cell::instance_bucket const &Bucket )
{
	return scene::bounding_area{ Bucket.recordcenter, static_cast<float>( 1.001 * Bucket.recordlocationradius + 0.5 ) };
}

// True GPU-instanced render path for a group of TAnimModel instances sharing the same
// TModel3d. The submodel tree is walked ONCE per lod band; at every submodel that draws
// geometry we issue a single glDrawElementsInstancedBaseVertexBaseInstance(N) covering all
// visible instances of the band. The vertex shader multiplies effective_modelview =
// instance_modelview[gl_BaseInstance + gl_InstanceID] * model_ubs.modelview, where
// instance_modelview[i] is the camera-space root transform of an instance (precomputed below)
// and model_ubs.modelview is the submodel-local chain (accumulated by the matrix stack starting
// from identity, NOT the camera/instance frame).
void scene_walk::Render_Instanced( TModel3d *Model, std::vector<queued_instance> const &Instances, std::vector<std::uint32_t> *Bandcounts )
{
	if( Bandcounts != nullptr ) { Bandcounts->clear(); }
	if( Model == nullptr ) { return; }
	if( Instances.empty() ) { return; }

	// LOD: a submodel is drawn only while fSquareDist lies inside its [fSquareMinDist, fSquareMaxDist)
	// range, so the distinct range bounds split distance into bands with constant LOD. The bucket
	// collects instances from every visible cell, so a single fSquareDist for all of them would draw
	// far instances at the LOD of the nearest one. Each instance is assigned to its band and every
	// band is drawn with its own fSquareDist, like in Render_Sleepers(). The grouping is a counting
	// sort, linear in the instance count.
	// the same walk also checks whether any piece of the model can get into the shadow map. Models
	// whose materials are all excluded by the shadow rank cutoff (e.g. grass) would otherwise go through
	// the whole per-instance culling and the batched draws in every shadow pass, only for every submodel
	// to be skipped in Render(TSubModel*)
	auto const *frontinstance { Instances.front().bucket->instances[ Instances.front().index ] };
	auto const *skins { frontinstance != nullptr && frontinstance->Material() != nullptr ? frontinstance->Material()->replacable_skins : nullptr };
	bool castsshadow { m_renderpass.draw_mode != rendermode::shadows };
	m_instance_lodbounds.clear();
	m_instance_lodpending.clear();
	m_instance_lodpending.push_back( Model->Root );
	while( false == m_instance_lodpending.empty() ) {
		auto const *submodel = m_instance_lodpending.back();
		m_instance_lodpending.pop_back();
		if( submodel == nullptr ) { continue; }
		if( submodel->fSquareMinDist > 0.f ) { m_instance_lodbounds.emplace_back( submodel->fSquareMinDist ); }
		if( submodel->fSquareMaxDist < std::numeric_limits<float>::max() ) { m_instance_lodbounds.emplace_back( submodel->fSquareMaxDist ); }
		if( ( false == castsshadow ) && ( submodel->eType < TP_ROTATOR ) ) {
			auto const material { submodel->m_material >= 0 ? submodel->m_material : ( skins != nullptr ? skins[ -submodel->m_material ] : null_handle ) };
			castsshadow = ( walk_material_data( material ).shadow_rank <= Global.gfx_shadow_rank_cutoff );
		}
		m_instance_lodpending.push_back( submodel->Child );
		m_instance_lodpending.push_back( submodel->Next );
	}
	if( false == castsshadow ) { return; }
	std::sort( m_instance_lodbounds.begin(), m_instance_lodbounds.end() );
	m_instance_lodbounds.erase( std::unique( m_instance_lodbounds.begin(), m_instance_lodbounds.end() ), m_instance_lodbounds.end() );
	auto const bandcount { m_instance_lodbounds.size() + 1 };
	if( Bandcounts != nullptr ) { Bandcounts->assign( bandcount, 0 ); }

	// 1. Visibility / distance cull, by the records of the instances. Collect surviving instances with
	// their precomputed camera-space root modelview matrices and lod bands. m_instance_survivors and the
	// other scratch vectors are persistent members reused across every Render_Instanced() call:
	// clear() drops the contents but keeps the allocated capacity, so after the first few
	// frames this stops calling malloc/free entirely.
	m_instance_survivors.clear();
	m_instance_survivors.reserve( Instances.size() );

	// Pull the current pass camera/view transform once. We use the current GL
	// modelview matrix as the view matrix because at the point Render_Instanced
	// is called, OpenGLMatrices is set to the camera view (scene root) — no
	// per-instance transform has been pushed yet.
	glm::mat4 const view_matrix = OpenGLMatrices.data( GL_MODELVIEW );

	TAnimModel *batch_instance { nullptr }; // the first instance drawn; its RaPrepare() and material serve the whole bucket
	material_data const *batch_material { nullptr };
	float farthest_distancesquared { 0.f };

	for( auto const &queued : Instances ) {
		auto const &record { queued.bucket->records[ queued.index ] };
		// instances which lost their instanceable status after being bucketed (animation or skin
		// assigned by an event) are drawn by the per-node path
		if( ( record.flags & scene::basic_cell::instance_instanceable ) == 0 ) { continue; }
		if( ( record.flags & scene::basic_cell::instance_visible ) == 0 ) { continue; }
		glm::dvec3 const location { queued.bucket->recordorigin + glm::dvec3{ record.position } };
		if( false == m_renderpass.pass_camera.visible( scene::bounding_area{ location, record.radius } ) ) { continue; }

		double distancesquared;
		switch( m_renderpass.draw_mode ) {
		case rendermode::shadows:
			distancesquared = glm::length2( ( location - m_renderpass.viewport_camera.position() ) / (double)Global.ZoomFactor ) / Global.fDistanceFactor;
			break;
		case rendermode::reflections:
			distancesquared = glm::length2( ( location - m_renderpass.pass_camera.position() ) );
			if( distancesquared > Global.reflectiontune.range_instances * Global.reflectiontune.range_instances ) { continue; }
			distancesquared += sq( EU07_REFLECTIONFIDELITYOFFSET );
			break;
		default:
			distancesquared = glm::length2( ( location - m_renderpass.pass_camera.position() ) / (double)Global.ZoomFactor ) / Global.fDistanceFactor;
			break;
		}
		if( ( distancesquared < record.rangesquaredmin )
		 || ( distancesquared >= record.rangesquaredmax ) ) { continue; }

		auto const drawdistancethreshold { m_renderpass.draw_range + 250 };
		if( distancesquared > sq( drawdistancethreshold ) ) { continue; }

		auto const radiussquared { record.radius * record.radius };
		if( radiussquared * Global.ZoomFactor / distancesquared < 0.003 * 0.003 ) { continue; }

		if( batch_instance == nullptr ) {
			batch_instance = queued.bucket->instances[ queued.index ];
			batch_instance->RaPrepare();
			batch_material = batch_instance->Material();
		}

		// Build the camera-relative root modelview for this instance:
		//   mv = view * translate(instance_pos - camera_pos) * rotate(instance_angles) * scale(m_scale)
		// The rotation and scale part comes from the record, the translation goes into its last
		// column. The scale is folded in so the GPU-instanced path produces visually identical output
		// to the regular per-instance path; the shader applies it via effective_modelview = instance_mv * model_local.
		glm::mat4 const root {
			record.rotationscale[ 0 ],
			record.rotationscale[ 1 ],
			record.rotationscale[ 2 ],
			glm::vec4( glm::vec3( location - m_renderpass.pass_camera.position() ), 1.f ) };

		auto const distance { static_cast<float>( distancesquared ) };
		farthest_distancesquared = std::max( farthest_distancesquared, distance );
		m_instance_survivors.push_back( {
			view_matrix * root,
			distance,
			static_cast<std::uint32_t>( std::upper_bound( m_instance_lodbounds.begin(), m_instance_lodbounds.end(), distance ) - m_instance_lodbounds.begin() ) } );
	}

	if( m_instance_survivors.empty() ) { return; }

	// group the modelviews by lod band into a contiguous array, for the upload. every distance in
	// a band selects the same lod; the nearest one represents the band.
	// within a band the instances are additionally ordered roughly near-to-far, in distance slots. dense
	// alpha tested foliage then gets the most out of the early depth test, as the nearest instances fill
	// the depth buffer first and hide the fragments of those behind them before they get shaded.
	// both levels are done with a single counting sort, linear in the instance count
	auto const distanceslots { 16 };
	auto const keycount { bandcount * distanceslots };
	auto const slotscale { distanceslots / std::sqrt( std::max( farthest_distancesquared, 1.f ) ) };
	m_instance_bandstarts.assign( keycount + 1, 0 );
	m_instance_banddistances.assign( bandcount, std::numeric_limits<float>::max() );
	for( auto &survivor : m_instance_survivors ) {
		m_instance_banddistances[ survivor.band ] = std::min( m_instance_banddistances[ survivor.band ], survivor.distancesquared );
		auto const slot { std::min( distanceslots - 1, static_cast<int>( std::sqrt( survivor.distancesquared ) * slotscale ) ) };
		survivor.band = static_cast<std::uint32_t>( survivor.band * distanceslots + slot ); // from now on it's the sort key
		++m_instance_bandstarts[ survivor.band + 1 ];
	}
	for( std::size_t key = 1; key <= keycount; ++key ) {
		m_instance_bandstarts[ key ] += m_instance_bandstarts[ key - 1 ];
	}
	m_instance_bandcursors.assign( m_instance_bandstarts.begin(), m_instance_bandstarts.end() - 1 );
	m_instance_modelviews.resize( m_instance_survivors.size() );
	for( auto const &survivor : m_instance_survivors ) {
		m_instance_modelviews[ m_instance_bandcursors[ survivor.band ]++ ] = survivor.modelview;
	}
	if( Bandcounts != nullptr ) {
		for( std::size_t band = 0; band < bandcount; ++band ) {
			( *Bandcounts )[ band ] = static_cast<std::uint32_t>( m_instance_bandstarts[ ( band + 1 ) * distanceslots ] - m_instance_bandstarts[ band * distanceslots ] );
		}
	}
	// all bands go to the instance buffer at once, each band is drawn from its own base instance
	auto const baseinstance { walk_instances( m_instance_modelviews.data(), m_instance_modelviews.size() ) };

	auto alpha = ( batch_material != nullptr ? batch_material->textures_alpha : 0x30300030 );
	alpha ^= 0x0F0F000F;

	// 2. Walk the submodel tree once per lod band. The submodel-local matrix
	// stack starts at identity; the per-instance camera transform comes from
	// instance_modelview[gl_BaseInstance + gl_InstanceID] in the shader.
	for( std::size_t band = 0; band < bandcount; ++band ) {

		std::size_t const band_start = m_instance_bandstarts[ band * distanceslots ];
		std::size_t const band_end = m_instance_bandstarts[ ( band + 1 ) * distanceslots ];
		if( band_start == band_end ) { continue; }

		// 2a. Push identity onto the matrix stack so submodel transforms
		// accumulate from identity, not from camera/instance space.
		::glPushMatrix();
		::glLoadIdentity();

		// 2b. Configure the existing TModel3d/TSubModel render path. Setting
		// m_current_instance_count routes walk_draw(handle) calls to draw_instanced.
		m_current_instance_count = band_end - band_start;
		m_current_instance_base = baseinstance + static_cast<std::uint32_t>( band_start );

		Model->Root->fSquareDist = m_instance_banddistances[ band ]; // shared global, used by submodel LOD
		Model->Root->ReplacableSet( ( batch_material != nullptr ? batch_material->replacable_skins : nullptr ), alpha );
		Model->Root->pRoot = Model;

		Render( Model->Root );

		m_current_instance_count = 0;
		m_current_instance_base = 0;

		::glPopMatrix();

		++m_renderpass.draw_stats.instanced_drawcalls;
	}

	m_renderpass.draw_stats.instances += static_cast<int>( m_instance_survivors.size() );
	m_renderpass.draw_stats.models += static_cast<int>( m_instance_survivors.size() );
}

void scene_walk::Render_Sleepers( TTrack *Track )
{
	if( Track == nullptr ) { return; }
	if( false == Track->m_sleeper_enabled ) { return; }
	if( Track->m_sleeper_model == nullptr ) { return; }
	if( Track->m_sleeper_local_transforms.empty() ) { return; }
	if( Global.SleeperDistance <= 0.f ) { return; }

	// only the color and reflection passes draw sleepers; shadow/pick skip them on purpose
	// (sleeper shadows would mostly fall back under the trackbed and pick already operates on
	// the track itself).
	switch( m_renderpass.draw_mode ) {
	case rendermode::color:
	case rendermode::reflections:
		break;
	default:
		return;
	}

	// distance gate -- compare against Globals.SleeperDistance squared to avoid the sqrt
	auto const camerapos = m_renderpass.pass_camera.position();
	auto const trackpos  = Track->location();
	auto const distsq    = glm::length2( trackpos - camerapos );
	auto const cutoffsq  = static_cast<double>( Global.SleeperDistance ) * static_cast<double>( Global.SleeperDistance );
	if( distsq > cutoffsq ) { return; }

	// build camera-space modelview matrices.
	// each sleeper's stored matrix is in track-local space (relative to Track->m_origin).
	// Render_Sleepers is called from inside the per-cell origin push -- the cell's center
	// already equals Track->m_origin (see basic_cell::insert), so the current GL_MODELVIEW
	// is already view * translate(m_origin - camera). We just need to compose with each
	// per-sleeper local transform to get the final modelview.
	glm::mat4 const origin_mv = OpenGLMatrices.data( GL_MODELVIEW );

	// per-sleeper frustum cull + LOD selection. The whole-track SleeperDistance
	// gate above keeps or drops the track as a unit; everything from here on
	// operates on individual sleepers.
	//
	// Each m_sleeper_local_transforms entry is
	//   translate(world_pos - m_origin) * rotate(...) * translate(local_offset)
	// so its translation column is the sleeper position relative to m_origin;
	// adding m_origin back yields the sleeper's world-space position. The model's
	// bounding radius (floored to a small minimum, in case it was never measured)
	// serves both as the frustum test sphere and -- via the distance to that
	// world position -- as the per-sleeper LOD distance.
	auto const sleeperradius = std::max( Track->m_sleeper_model->bounding_radius(), 2.0f );

	// Phase 1 -- frustum cull. For every sleeper that survives, store its
	// camera-space modelview paired with the squared distance from the camera
	// to ITS OWN world position (not the track / segment origin). That
	// per-sleeper distance is what drives LOD selection below. A sleeper whose
	// origin sits just off screen while its geometry still reaches into view is
	// kept rather than wrongly culled.
	std::vector<std::pair<float, glm::mat4>> survivors;
	survivors.reserve( Track->m_sleeper_local_transforms.size() );
	for( auto const &local : Track->m_sleeper_local_transforms ) {
		glm::dvec3 const sleeperworldpos {
			Track->m_origin + glm::dvec3( local[ 3 ].x, local[ 3 ].y, local[ 3 ].z ) };
		if( false == m_renderpass.pass_camera.visible( scene::bounding_area{ sleeperworldpos, sleeperradius } ) ) {
			continue;
		}
		auto const sleeperdistancesquared = static_cast<float>( glm::length2( sleeperworldpos - camerapos ) );
		survivors.emplace_back( sleeperdistancesquared, origin_mv * local );
	}

	// every sleeper of this track was frustum-culled -- nothing left to draw
	if( survivors.empty() ) { return; }

	// Phase 2 -- sort survivors near-to-far. A submodel is drawn only while
	// fSquareDist lies inside its [fSquareMinDist, fSquareMaxDist) range, so
	// every distance between two consecutive range bounds selects an identical
	// set of submodels -- i.e. the same LOD. Sorting turns each such LOD band
	// into a contiguous run, letting the draw loop emit one instanced batch per
	// band instead of one track-wide batch at a single distance.
	std::sort( survivors.begin(), survivors.end(),
		[]( std::pair<float, glm::mat4> const &Left, std::pair<float, glm::mat4> const &Right ) {
			return Left.first < Right.first; } );

	// contiguous copy of the sorted modelview matrices, for the upload.
	// Per-sleeper distances stay available as survivors[i].first, in lockstep.
	std::vector<glm::mat4> instance_modelviews;
	instance_modelviews.reserve( survivors.size() );
	for( auto const &survivor : survivors ) {
		instance_modelviews.emplace_back( survivor.second );
	}

	// collect the model's distinct LOD distance bounds. The sorted, de-duplicated
	// set of every submodel's fSquareMinDist / fSquareMaxDist partitions distance
	// into bands within which the selected LOD is constant. A model with no LOD
	// yields a single band, and the draw loop below then behaves exactly like a
	// single plain batched draw.
	std::vector<float> lodbounds;
	{
		std::vector<TSubModel const *> pending;
		if( Track->m_sleeper_model->Root != nullptr ) {
			pending.push_back( Track->m_sleeper_model->Root );
		}
		while( false == pending.empty() ) {
			auto const *submodel = pending.back();
			pending.pop_back();
			lodbounds.emplace_back( submodel->fSquareMinDist );
			lodbounds.emplace_back( submodel->fSquareMaxDist );
			if( submodel->Child != nullptr ) { pending.push_back( submodel->Child ); }
			if( submodel->Next  != nullptr ) { pending.push_back( submodel->Next ); }
		}
	}
	std::sort( lodbounds.begin(), lodbounds.end() );
	lodbounds.erase( std::unique( lodbounds.begin(), lodbounds.end() ), lodbounds.end() );

	// optional replacable skin: build a transient material_data so we can drive ReplacableSet
	// the same way Render_Instanced does. when no skin is set we fall back to the model defaults.
	material_data sleeper_material {};
	bool const has_skin = ( Track->m_sleeper_skin != null_handle );
	if( has_skin ) {
		sleeper_material.replacable_skins[ 1 ] = Track->m_sleeper_skin;
	}

	// Phase 3 -- draw. Walk the sorted survivors one LOD band at a time. Each
	// band is submitted as one instanced draw from its own base instance; every
	// draw uses an in-band fSquareDist, so each sleeper renders at the LOD its own
	// distance selects while instancing stays fully in effect.
	auto *Model = Track->m_sleeper_model;
	std::size_t const total = instance_modelviews.size();
	auto const baseinstance { walk_instances( instance_modelviews.data(), total ) };
	std::size_t band_start = 0;
	while( band_start < total ) {
		// the band ends at the first sleeper distance that reaches the next LOD
		// bound above the band's starting distance.
		auto const upperbound = std::upper_bound(
			lodbounds.begin(), lodbounds.end(), survivors[ band_start ].first );
		float const band_limit = ( upperbound == lodbounds.end()
			? std::numeric_limits<float>::max()
			: *upperbound );
		std::size_t band_end = band_start;
		while( ( band_end < total ) && ( survivors[ band_end ].first < band_limit ) ) {
			++band_end;
		}
		// every distance in the band selects the same LOD; the band's nearest
		// sleeper is used as the representative fSquareDist.
		float const band_distancesquared = survivors[ band_start ].first;

		::glPushMatrix();
		::glLoadIdentity();

		m_current_instance_count = band_end - band_start;
		m_current_instance_base = baseinstance + static_cast<std::uint32_t>( band_start );

		Model->Root->fSquareDist = band_distancesquared;
		auto alpha = ( has_skin ? sleeper_material.textures_alpha : 0x30300030 );
		alpha ^= 0x0F0F000F;
		Model->Root->ReplacableSet( ( has_skin ? sleeper_material.replacable_skins : nullptr ), alpha );
		Model->Root->pRoot = Model;

		Render( Model->Root );

		m_current_instance_count = 0;
		m_current_instance_base = 0;

		::glPopMatrix();

		++m_renderpass.draw_stats.instanced_drawcalls;

		band_start = band_end;
	}

	m_renderpass.draw_stats.instances += static_cast<int>( total );
	m_renderpass.draw_stats.models    += static_cast<int>( total );

	// the comparison with the gpu culling of the opengl renderer
	walk_sleepers_drawn( *Track, lodbounds, survivors );
}

// Renders the sleepers and switch timbers of a run of tracks of a cell. Each beam is made of instances of the parts of a model or
// of a whole model, kept by its track relative to the origin of the track. The runs of beams in range and in sight are gathered
// for the whole run of tracks by the model and its part or its level of detail, and each is drawn in one instanced batch.
// In the shadow pass only those near the camera are drawn, their shadows falling where they can be seen.
void scene_walk::Render_Timbers( scene::basic_cell::path_sequence::const_iterator First, scene::basic_cell::path_sequence::const_iterator Last )
{
	if( Global.SleeperDistance <= 0.f ) { return; }

	auto const shadows = ( m_renderpass.draw_mode == rendermode::shadows );
	// the distance is taken from the camera of the view, the shadow pass culling against its own frustum
	auto const camera = ( shadows ? m_renderpass.viewport_camera.position() : m_renderpass.pass_camera.position() );
	auto const range = static_cast<double>( shadows ? std::min( Global.SleeperDistance, 50.f ) : Global.SleeperDistance );
	if( m_renderpass.draw_mode == rendermode::color && m_timbersweepframe != m_framestamp ) {
		// once a frame the beams of the tracks well past where they're laid and worked out ahead are let go
		m_timbersweepframe = m_framestamp;
		track_sleepers::release_far( camera, range + 600.0 );
	}
	glm::mat4 const originmodelview = OpenGLMatrices.data( GL_MODELVIEW );
	for( auto &bucket : m_timberbuckets ) {
		bucket.instances.clear();
	}
	auto const bucket_for = [&]( track_sleepers::beam_model const *Model, int const Part, float const Band ) -> std::vector<glm::mat4> & {
		for( auto &bucket : m_timberbuckets ) {
			if( bucket.model == Model && bucket.part == Part && bucket.band == Band ) {
				return bucket.instances;
			}
		}
		m_timberbuckets.push_back( { Model, Part, Band, {} } );
		return m_timberbuckets.back().instances; };
	// the level of detail of the whole model a beam is seen at, from its distance
	auto const band_of = []( track_sleepers::beam_model const &Model, double const Distance ) {
		auto const squaredistance { static_cast<float>( Distance * Distance ) };
		auto band { Model.bands.empty() ? 0.f : Model.bands.front() };
		for( auto const start : Model.bands ) {
			if( squaredistance >= start ) {
				band = start;
			}
		}
		return band; };
	// the beams of a run made of a whole model, each at the level of detail of its own distance; the run is culled as one
	auto const put_by_band = [&]( track_sleepers::beam_model const &Model, std::vector<glm::mat4> const &Transforms, std::uint32_t const From, std::uint32_t const To, glm::dvec3 const &Origin ) {
		std::vector<glm::mat4> *instances { nullptr };
		auto current { 0.f };
		for( auto index = From; index < To; ++index ) {
			auto const &local = Transforms[ index ];
			auto const band = band_of( Model, glm::length( Origin + glm::dvec3( glm::vec3( local[ 3 ] ) ) - camera ) );
			if( instances == nullptr || band != current ) {
				// looked up again only as the band changes; a bucket added on the way moves the others, so the pointer goes with it
				instances = &bucket_for( &Model, -1, band );
				current = band;
			}
			instances->emplace_back( originmodelview * local );
		}
	};
	for( auto first { First }; first != Last; ++first ) {
		auto *track = *first;
		if( false == track->m_visible || false == track->m_sleepers3d.active ) { continue; }
		if( glm::length( track->location() - camera ) > range + track->radius() ) {
			// worked out ahead, in the background, before it comes in range
			if( false == shadows && track->m_sleepers3d.dirty && glm::length( track->location() - camera ) < range + track->radius() + 300.0 ) {
				track_sleepers::prefetch( *track );
			}
			continue;
		}
		if( track->m_sleepers3d.dirty ) {
			track_sleepers::lay( *track );
		}
		auto const &laid = track->m_sleepers3d;
		if( laid.timbers == nullptr ) { continue; }
		// the transforms of the beams are relative to the origin of the track, which the modelview is translated to already
		auto const &trackmodelview = originmodelview;
		for( auto const &chunk : laid.chunks ) {
			auto const centre = track->m_origin + glm::dvec3( chunk.centre );
			auto const distance = glm::length( centre - camera );
			if( distance > range + chunk.radius ) { continue; }
			if( false == m_renderpass.pass_camera.visible( scene::bounding_area{ centre, chunk.radius } ) ) { continue; }
			if( laid.timbers->stretchable ) {
				for( int part = 0; part < track_sleepers::part_count; ++part ) {
					auto &instances = bucket_for( laid.timbers, part, 0.f );
					for( auto index = chunk.first[ part ]; index < chunk.last[ part ]; ++index ) {
						instances.emplace_back( trackmodelview * laid.instances[ part ][ index ] );
					}
				}
			}
			if( chunk.wholelast > chunk.wholefirst ) {
				put_by_band( *laid.timbers, laid.whole, chunk.wholefirst, chunk.wholelast, track->m_origin );
			}
			if( laid.plains != nullptr && chunk.plainlast > chunk.plainfirst ) {
				put_by_band( *laid.plains, laid.plain, chunk.plainfirst, chunk.plainlast, track->m_origin );
			}
		}
	}

	auto const alpha = ( 0x30300030 ^ 0x0F0F000F );
	std::size_t total { 0 };
	for( auto &bucket : m_timberbuckets ) {
		auto const &instances = bucket.instances;
		if( instances.empty() ) { continue; }
		total += instances.size();
		auto const &beammodel = *bucket.model;
		auto *model = beammodel.model;
		if( bucket.part >= 0 ) {
			// only the part of the model is visible, its siblings are skipped as the model is walked
			for( int part = 0; part < track_sleepers::part_count; ++part ) {
				beammodel.parts[ part ]->iVisible = ( part == bucket.part ? 1 : 0 );
			}
		}
		material_data skins {};
		if( beammodel.skin != null_handle ) {
			skins.replacable_skins[ 1 ] = beammodel.skin;
		}
		// the instances go to the instance stream of the frame, as in Render_Sleepers(), which the recorded walk of the depth
		// pass keeps for its replay as well
		auto const baseinstance { walk_instances( instances.data(), instances.size() ) };
		::glPushMatrix();
		::glLoadIdentity();
		m_current_instance_count = instances.size();
		m_current_instance_base = baseinstance;
		// inside the range of the level of detail the beams are at
		model->Root->fSquareDist = ( bucket.part >= 0 ? 0.f : bucket.band + 1.f );
		model->Root->ReplacableSet( ( beammodel.skin != null_handle ? skins.replacable_skins : nullptr ), alpha );
		model->Root->pRoot = model;
		Render( model->Root );
		m_current_instance_count = 0;
		m_current_instance_base = 0;
		::glPopMatrix();
		++m_renderpass.draw_stats.instanced_drawcalls;
		if( bucket.part >= 0 ) {
			for( int part = 0; part < track_sleepers::part_count; ++part ) {
				beammodel.parts[ part ]->iVisible = 1;
			}
		}
	}
	m_renderpass.draw_stats.instances += static_cast<int>( total );
}

bool scene_walk::Render(TDynamicObject *Dynamic)
{
	glDebug("Render TDynamicObject");

	if (!Global.render_cab && Global.pCamera.m_owner == Dynamic)
		return false;

    Dynamic->renderme = (
        ( Global.pCamera.m_owner == Dynamic && !FreeFlyModeFlag )
     || ( m_renderpass.pass_camera.visible( Dynamic ) ) );

    if (false == Dynamic->renderme) { return false; }

	// setup
	TSubModel::iInstance = reinterpret_cast<std::uintptr_t>(Dynamic); //żeby nie robić cudzych animacji
	TSubModel::iVariantSeed = Dynamic->variant_seed();
	glm::dvec3 const originoffset = Dynamic->vPosition - m_renderpass.pass_camera.position();
	// lod visibility ranges are defined for base (x 1.0) viewing distance. for render we adjust them for actual range multiplier and zoom
	float squaredistance;
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
	{
		squaredistance = glm::length2(glm::vec3{glm::dvec3{Dynamic->vPosition - m_renderpass.viewport_camera.position()}} / Global.ZoomFactor);
        if( false == FreeFlyModeFlag ) {
            // filter out small details if we're in vehicle cab
            squaredistance = std::max( 100.f * 100.f, squaredistance );
        }
		break;
	}
    case rendermode::reflections:
    {
        // reflection mode draws simplified version of the shapes, by artificially increasing view range
        // it also ignores zoom settings and distance multipliers
        squaredistance =
            std::max(
                100.f * 100.f,
                // TBD: take into account distance multipliers?
                glm::length2( glm::vec3{ originoffset } ) /* / Global.fDistanceFactor */ );
        // NOTE: arbitrary draw range limit
        if( squaredistance > Global.reflectiontune.range_vehicles * Global.reflectiontune.range_vehicles ) {
            return false;
        }
        // TBD, TODO: bind offset value with setting variable?
        // NOTE: combined 'squared' distance doesn't equal actual squared (distance + offset) but, eh
        squaredistance += sq(EU07_REFLECTIONFIDELITYOFFSET);
        break;
    }
	default:
	{
		squaredistance = glm::length2(glm::vec3{originoffset} / Global.ZoomFactor);
        // TODO: filter out small details based on fidelity setting
		break;
	}
	}
    // crude way to reject early items too far to affect the output (mostly relevant for shadow and reflection passes)
    auto const drawdistancethreshold{ m_renderpass.draw_range + 250 };
    if( squaredistance > sq(drawdistancethreshold) ) {
        return false;
    }
    // second stage visibility cull, reject vehicles too far away to be noticeable
    auto const squareradius{ Dynamic->radius() * Dynamic->radius() };
    Dynamic->renderme = ( squareradius * Global.ZoomFactor / squaredistance > 0.003 * 0.003 );
    if( false == Dynamic->renderme ) {
        return false;
    }

    // debug data
    ++m_renderpass.draw_stats.dynamics;

	Dynamic->ABuLittleUpdate(squaredistance); // ustawianie zmiennych submodeli dla wspólnego modelu

	glm::mat4 future_stack = model_ubs.future;

	glm::mat4 mv = OpenGLMatrices.data(GL_MODELVIEW);
	model_ubs.future *= glm::translate(mv, glm::vec3(m_motionhistory ? -Dynamic->get_future_movement() : Dynamic->get_future_movement())) * glm::inverse(mv);
	// the velocity it leaves in the colour pass keeps the motion blur on (Render_pass())
	if ((m_renderpass.draw_mode == rendermode::color) && (Dynamic->get_future_movement() != glm::dvec3{0.0}))
		m_vehiclesmoving = true;

	::glPushMatrix();
	::glTranslated(originoffset.x, originoffset.y, originoffset.z);
	::glMultMatrixd(glm::value_ptr(Dynamic->mMatrix));

	switch (m_renderpass.draw_mode)
	{

	case rendermode::color:
    case rendermode::reflections:
	{
		if (Dynamic->fShade > 0.0f)
		{
			// change light level based on light level of the occupied track
			walk_sunlight(Dynamic->fShade);
        }

		// render
        if( Dynamic->mdLowPolyInt ) {
            // low poly interior
            Render_lowpoly( Dynamic, squaredistance, false );
        }
        else {
            // HACK: if the model lacks low poly interior, we presume the load is placed outside
            if( Dynamic->mdLoad ) {
                // renderowanie nieprzezroczystego ładunku
                Render( Dynamic->mdLoad, Dynamic->Material(), squaredistance, { 0.f, Dynamic->LoadOffset, 0.f }, {} );
            }
        }
        if( Dynamic->mdModel ) {
            // main model
            Render( Dynamic->mdModel, Dynamic->Material(), squaredistance );
        }
        // optional attached models
        for( auto *attachment : Dynamic->mdAttachments ) {
            Render( attachment, Dynamic->Material(), squaredistance );
        }
        // optional coupling adapters
        Render_coupler_adapter( Dynamic, squaredistance, end::front );
        Render_coupler_adapter( Dynamic, squaredistance, end::rear );

        // post-render cleanup
		if (Dynamic->fShade > 0.0f)
		{
			// restore regular light level
			walk_sunlight();
        }
		break;
	}
	case rendermode::shadows:
	{
        if( Dynamic->mdLowPolyInt ) {
            // low poly interior
            Render( Dynamic->mdLowPolyInt, Dynamic->Material(), squaredistance );
        }
        if( Dynamic->mdModel ) {
            // main model
            Render( Dynamic->mdModel, Dynamic->Material(), squaredistance );
        }
        // optional attached models
        for( auto *attachment : Dynamic->mdAttachments ) {
            Render( attachment, Dynamic->Material(), squaredistance );
        }
        // optional coupling adapters
        Render_coupler_adapter( Dynamic, squaredistance, end::front );
        Render_coupler_adapter( Dynamic, squaredistance, end::rear );
        if( Dynamic->mdLoad ) {
            // renderowanie nieprzezroczystego ładunku
            Render( Dynamic->mdLoad, Dynamic->Material(), squaredistance, { 0.f, Dynamic->LoadOffset, 0.f }, {} );
        }
		// post-render cleanup
		break;
	}
	case rendermode::pickcontrols:
	{
		if (Dynamic->mdLowPolyInt) {
			// low poly interior
			Render(Dynamic->mdLowPolyInt, Dynamic->Material(), squaredistance);
		}
		break;
	}
	case rendermode::pickscenery:
	default:
	{
		break;
	}
	}

	::glPopMatrix();

	model_ubs.future = future_stack;

	// TODO: check if this reset is needed. In theory each object should render all parts based on its own instance data anyway?
	if (Dynamic->btnOn)
		Dynamic->TurnOff(); // przywrócenie domyślnych pozycji submodeli

	return true;
}

// rendering kabiny gdy jest oddzielnym modelem i ma byc wyswietlana
bool scene_walk::Render_cab(TDynamicObject const *Dynamic, float const Lightlevel, bool const Alpha)
{

	if (Dynamic == nullptr)
	{

		TSubModel::iInstance = 0;
		TSubModel::iVariantSeed = 0;
		return false;
	}

	TSubModel::iInstance = reinterpret_cast<std::uintptr_t>(Dynamic);
	TSubModel::iVariantSeed = Dynamic->variant_seed();

	if ((true == FreeFlyModeFlag) || (false == Dynamic->bDisplayCab) || (Dynamic->mdKabina == Dynamic->mdModel))
	{
		// ABu: Rendering kabiny jako ostatniej, zeby bylo widac przez szyby, tylko w widoku ze srodka
		return false;
	}

	if (Dynamic->mdKabina)
	{ // bo mogła zniknąć przy przechodzeniu do innego pojazdu
		// setup shared by all render paths
		::glPushMatrix();

		auto const originoffset = Dynamic->GetPosition() - m_renderpass.pass_camera.position();
		::glTranslated(originoffset.x, originoffset.y, originoffset.z);
		::glMultMatrixd(glm::value_ptr(Dynamic->mMatrix));

        switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		{
			// render path specific setup:
			if (Dynamic->fShade > 0.0f)
			{
				// change light level based on light level of the occupied track
				walk_sunlight(Dynamic->fShade);
            }

            walk_ambient_save();
            auto const luminance { Global.fLuminance * ( Dynamic->fShade > 0.0f ? Dynamic->fShade : 1.0f ) };
            if( Lightlevel > 0.f ) {
                // crude way to light the cabin, until we have something more complete in place. with the lamp of the cab among the
                // lights a share of it is kept for the light which bounces around the cab
                auto const share { m_cablamplit ? Global.gfx_lights_cab_bounce : 1.f };
                walk_ambient_add( ( Dynamic->InteriorLight * Lightlevel ) * std::clamp( 1.25f - (float)luminance, 0.f, 1.f ) * share );
            }

			// render
			if (true == Alpha)
			{
				// translucent parts
				Render_Alpha(Dynamic->mdKabina, Dynamic->Material(), 0.0);
			}
			else
			{
				// opaque parts
				Render(Dynamic->mdKabina, Dynamic->Material(), 0.0);
			}
			// post-render restore
            walk_ambient_restore();
            walk_sunlight();

			break;
		}
        case rendermode::shadows:
        {
            if( true == Alpha ) {
                // translucent parts
                Render_Alpha( Dynamic->mdKabina, Dynamic->Material(), 0.0 );
            }
            else {
                // opaque parts
                Render( Dynamic->mdKabina, Dynamic->Material(), 0.0 );
            }
            break;
        }
		case rendermode::pickcontrols:
		{
			Render(Dynamic->mdKabina, Dynamic->Material(), 0.0);
			break;
		}
		default:
		{
			break;
		}
		}
		// post-render restore
		::glPopMatrix();
	}

	return true;
}

bool scene_walk::Render(TModel3d *Model, material_data const *Material, float const Squaredistance)
{

	auto alpha = (Material != nullptr ? Material->textures_alpha : 0x30300030);
	alpha ^= 0x0F0F000F; // odwrócenie flag tekstur, aby wyłapać nieprzezroczyste
	if (0 == (alpha & Model->iFlags & 0x1F1F001F))
	{
		// czy w ogóle jest co robić w tym cyklu?
		return false;
	}

	Model->Root->fSquareDist = Squaredistance; // zmienna globalna!

	// setup
	Model->Root->ReplacableSet((Material != nullptr ? Material->replacable_skins : nullptr), alpha);

	Model->Root->pRoot = Model;

	// render
	Render(Model->Root);

	// post-render cleanup

	return true;
}

bool scene_walk::Render(TModel3d *Model, material_data const *Material, float const Squaredistance, glm::dvec3 const &Position, glm::vec3 const &Angle)
{
	::glPushMatrix();
	::glTranslated(Position.x, Position.y, Position.z);
	if (Angle.y != 0.0)
		::glRotated(Angle.y, 0.f, 1.f, 0.f);
	if (Angle.x != 0.0)
		::glRotated(Angle.x, 1.f, 0.f, 0.f);
	if (Angle.z != 0.0)
		::glRotated(Angle.z, 0.f, 0.f, 1.f);

	auto const result = Render(Model, Material, Squaredistance);

	::glPopMatrix();

	return result;
}

void scene_walk::Render(TSubModel *Submodel)
{
	glDebug("Render TSubModel");

	if ((Submodel->iVisible) && (TSubModel::fSquareDist >= Submodel->fSquareMinDist) && (TSubModel::fSquareDist < Submodel->fSquareMaxDist) && (false == m_previewing || Submodel->fVisible > 0.f))
	{
		glm::mat4 future_stack = model_ubs.future;

		if (Submodel->iFlags & 0xC000)
		{
			::glPushMatrix();
			if (Submodel->fMatrix)
				::glMultMatrixf(Submodel->fMatrix->readArray());
			if (Submodel->b_aAnim != TAnimType::at_None)
			{
				Submodel->RaAnimation(Submodel->b_aAnim);

				glm::mat4 mv = OpenGLMatrices.data(GL_MODELVIEW);
				model_ubs.future *= (mv * (m_motionhistory ? glm::inverse(Submodel->future_transform) : Submodel->future_transform)) * glm::inverse(mv);
				// the placement may be different in the next frame: the walk isn't kept (Record_Indirect_Draws())
				m_indirectchanging = true;
			}
		}

		if (Submodel->eType < TP_ROTATOR)
		{
			// renderowanie obiektów OpenGL
			if (Submodel->iAlpha & Submodel->iFlags & 0x1F)
			{
				// rysuj gdy element nieprzezroczysty

		        // debug data
		        ++m_renderpass.draw_stats.submodels;
		        ++m_renderpass.draw_stats.drawcalls;

				switch (m_renderpass.draw_mode)
				{
				case rendermode::color:
				case rendermode::reflections:
				{
					// material configuration:
					// transparency hack
					if (Submodel->fVisible < 1.0f)
						walk_blending(true);

					// textures...
					if (Submodel->m_material < 0)
					{ // zmienialne skóry
						walk_material(submodel_material(Submodel), Submodel);
					}
					else
					{
						// również 0
						walk_material(Submodel->m_material, Submodel);
					}

					// ...luminance
					auto const isemissive { ( Submodel->f4Emision.a > 0.f ) && ( Global.fLuminance < Submodel->fLight ) };
					if( ( Submodel->f4Emision.a > 0.f ) && ( Submodel->fLight > -1.f ) )
					{
						// lit or not by the luminance of the moment: the walk isn't kept (Record_Indirect_Draws())
						m_indirectchanging = true;
					}
					if (isemissive)
						model_ubs.emission = Submodel->f4Emision.a;

					// main draw call
					walk_draw(Submodel->m_geometry.handle);

					// post-draw reset
					model_ubs.emission = 0.0f;
					if (Submodel->fVisible < 1.0f)
						walk_blending(false);

					break;
				}
				case rendermode::shadows:
				{
                    // skip if the shadow caster rank is too low for currently set threshold
                    // or if the submodel is hidden (colour pass only fades it out, e.g. inactive seasonal variant)
                    if( ( walk_material_data( Submodel ).shadow_rank > Global.gfx_shadow_rank_cutoff )
                     || ( Submodel->fVisible <= 0.f ) )
                    {
                        --m_renderpass.draw_stats.submodels;
                        --m_renderpass.draw_stats.drawcalls;
                        break;
                    }

                    if (Submodel->m_material < 0)
					{ // zmienialne skóry
						walk_shadow_material(submodel_material(Submodel));
					}
					else
					{
						// również 0
						walk_shadow_material(Submodel->m_material);
					}
                    walk_draw(Submodel->m_geometry.handle);
                    break;
				}
				case rendermode::pickscenery:
				{
					walk_program( walk_shader::pick );
					walk_draw(Submodel->m_geometry.handle);
					break;
				}
				case rendermode::pickcontrols:
				{
				    if (Submodel->screen_touch_list) {
                        // touch screen gradient
                        walk_program( walk_shader::pick_surface );
                        model_ubs.param[0] = glm::vec4(1.0f - m_picksurfaceitems.size() / 255.0f, 0.0f, 0.0f, 1.0f);
                        m_picksurfaceitems.emplace_back(Submodel);
                    }
                    else {
                        // control picking applies individual colour for each submodel
                        walk_program( walk_shader::pick );
                        m_pickcontrolsitems.emplace_back(Submodel);
                        model_ubs.param[0] = glm::vec4(pick_color(m_pickcontrolsitems.size()), 1.0f);
                    }

					walk_draw(Submodel->m_geometry.handle);
					break;
				}
				default:
				{
					break;
				}
				}
			}
		}
		else if (Submodel->eType == TP_STARS)
		{

			switch (m_renderpass.draw_mode)
			{
			// colour points are only rendered in colour mode(s)
			case rendermode::color:
			case rendermode::reflections:
			{
				m_indirectchanging = true;
				if (Global.fLuminance < Submodel->fLight)
				{
					walk_material(Submodel->m_material, Submodel);

					// main draw call
                    model_ubs.param[1].x = 8.0f;

					walk_draw(Submodel->m_geometry.handle);
				}
				break;
			}
			default:
			{
				break;
			}
			}
		}
		if (Submodel->Child != nullptr)
			if (Submodel->iAlpha & Submodel->iFlags & 0x001F0000)
				Render(Submodel->Child);

		if (Submodel->iFlags & 0xC000)
		{
			model_ubs.future = future_stack;
			::glPopMatrix();
		}
	}
	/*
	    if( Submodel->b_Anim < at_SecondsJump )
	        Submodel->b_Anim = at_None; // wyłączenie animacji dla kolejnego użycia subm
	*/
	if (Submodel->Next)
		if (Submodel->iAlpha & Submodel->iFlags & 0x1F000000)
			Render(Submodel->Next); // dalsze rekurencyjnie
}

void scene_walk::Render(TTrack *Track)
{
	if ((Track->m_material1 == 0) && (Track->m_material2 == 0) && (Track->eType != tt_Switch || Track->SwitchExtension->m_material3 == 0))
	{
		return;
	}
	if (false == Track->m_visible)
	{
		return;
	}

	++m_renderpass.draw_stats.paths;
	++m_renderpass.draw_stats.drawcalls;

	switch (m_renderpass.draw_mode)
	{
	// single path pieces are rendererd in pick scenery mode only
	case rendermode::pickscenery:
	{
        walk_program( walk_shader::pick );
        m_picksceneryitems.emplace_back( Track );

		walk_draw(std::begin(Track->Geometry1), std::end(Track->Geometry1));
		walk_draw(std::begin(Track->Geometry2), std::end(Track->Geometry2));
		for (auto &tread : Track->GeometryTreads)
			walk_draw(std::begin(tread.geometry), std::end(tread.geometry));
		if (Track->eType == tt_Switch)
			walk_draw(Track->SwitchExtension->Geometry3);
		break;
	}
	default:
	{
		break;
	}
	}
}

// experimental, does track rendering in two passes, to take advantage of reduced texture switching
void scene_walk::Render(scene::basic_cell::path_sequence::const_iterator First, scene::basic_cell::path_sequence::const_iterator Last)
{

	// setup
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
	{
		// NOTE: roads-based platforms tend to miss parts of shadows if rendered with either back or front culling
		walk_cull_face(false);
		break;
	}
	default:
	{
		break;
	}
	}

	// geometry of neighbouring tracks which share the material is gathered and drawn together.
	// only neighbours are combined, the tracks are still drawn in their order in the cell
	material_handle batchmaterial{null_handle};
	auto const flush_batch = [&]()
	{
		if (false == m_pathbatch.empty())
		{
			walk_draw(std::begin(m_pathbatch), std::end(m_pathbatch));
			m_pathbatch.clear();
		}
		batchmaterial = null_handle;
	};
	// returns: true if the material starts a new batch, false if it continues the current one
	auto const begin_batch = [&](material_handle const Material)
	{
		if (Material == batchmaterial)
		{
			return false;
		}
		flush_batch();
		walk_material(Material);
		batchmaterial = Material;
		return true;
	};

	// TODO: render auto generated trackbeds together with regular trackbeds in pass 1, and all rails in pass 2
	// first pass, material 1
	for (auto first{First}; first != Last; ++first)
	{

		auto const track{*first};

		if (track->m_material1 == 0)
		{
			continue;
		}
		if (false == track->m_visible)
		{
			continue;
		}

		++m_renderpass.draw_stats.paths;
		++m_renderpass.draw_stats.drawcalls;

		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		case rendermode::reflections:
		{
			if (track->eEnvironment != e_flat)
			{
				// track with light level of its own is drawn on its own
				flush_batch();
				walk_environment(track->eEnvironment);
				walk_material(track->m_blendmaterial1 != null_handle ? track->m_blendmaterial1 : track->m_material1);
				walk_draw(std::begin(track->Geometry1), std::end(track->Geometry1));
				// restore default lighting
				walk_environment();
				break;
			}
			if (false == begin_batch(track->m_blendmaterial1 != null_handle ? track->m_blendmaterial1 : track->m_material1))
			{
				// the track is drawn with the call of its predecessor
				--m_renderpass.draw_stats.drawcalls;
			}
			m_pathbatch.insert(std::end(m_pathbatch), std::begin(track->Geometry1), std::end(track->Geometry1));
			break;
		}
		case rendermode::shadows:
        {
			if ((std::abs(track->fTexHeight1) < 0.35f) || (track->iCategoryFlag != 2))
			{
				// shadows are only calculated for high enough roads, typically meaning track platforms
				--m_renderpass.draw_stats.paths;
				--m_renderpass.draw_stats.drawcalls;
				continue;
			}
            if( walk_material_data( track->m_material1 ).shadow_rank > Global.gfx_shadow_rank_cutoff ) {
                // skip if the shadow caster rank is too low for currently set threshold
                --m_renderpass.draw_stats.paths;
                --m_renderpass.draw_stats.drawcalls;
                continue;
            }
			walk_shadow_material(track->m_material1);
			walk_draw(std::begin(track->Geometry1), std::end(track->Geometry1));
			break;
		}
		case rendermode::pickscenery: // pick scenery mode uses piece-by-piece approach
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}
	}
	flush_batch();
	// second pass, material 2
	for (auto first{First}; first != Last; ++first)
	{
		auto const track{*first};

		if (track->m_material2 == 0)
		{
			continue;
		}
		if (false == track->m_visible)
		{
			continue;
		}
		if (track->eType == tt_Normal && editor_ballast_hidden(track, m_renderpass.draw_mode == rendermode::color))
		{
			continue;
		}

		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		case rendermode::reflections:
		{
			if (track->eEnvironment != e_flat)
			{
				flush_batch();
				walk_environment(track->eEnvironment);
				walk_material(track->m_blendmaterial2 != null_handle ? track->m_blendmaterial2 : track->m_material2);
				walk_draw(std::begin(track->Geometry2), std::end(track->Geometry2));
				// restore default lighting
				walk_environment();
				break;
			}
			begin_batch(track->m_blendmaterial2 != null_handle ? track->m_blendmaterial2 : track->m_material2);
			m_pathbatch.insert(std::end(m_pathbatch), std::begin(track->Geometry2), std::end(track->Geometry2));
			break;
		}
		case rendermode::shadows:
        {
			if ((std::abs(track->fTexHeight1) < 0.35f) || ((track->iCategoryFlag == 1) && (track->eType != tt_Normal)))
			{
				// shadows are only calculated for high enough trackbeds
				continue;
			}
            if( walk_material_data( track->m_material2 ).shadow_rank > Global.gfx_shadow_rank_cutoff ) {
                // skip if the shadow caster rank is too low for currently set threshold
                continue;
            }
			walk_shadow_material(track->m_material2);
			walk_draw(std::begin(track->Geometry2), std::end(track->Geometry2));
			break;
		}
		case rendermode::pickscenery: // pick scenery mode uses piece-by-piece approach
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}
	}
	flush_batch();

	// third pass, material 3
	for (auto first{First}; first != Last; ++first)
	{

		auto const track{*first};

		if (track->eType != tt_Switch)
		{
			continue;
		}
		if (track->SwitchExtension->m_material3 == 0)
		{
			continue;
		}
		if (false == track->m_visible)
		{
			continue;
		}
		if (editor_ballast_hidden(track, m_renderpass.draw_mode == rendermode::color))
		{
			continue;
		}

		switch (m_renderpass.draw_mode)
		{
		case rendermode::color:
		case rendermode::reflections:
		{
			if (track->eEnvironment != e_flat)
			{
				flush_batch();
				walk_environment(track->eEnvironment);
				walk_material(track->m_blendmaterial3 != null_handle ? track->m_blendmaterial3 : track->SwitchExtension->m_material3);
				walk_draw(track->SwitchExtension->Geometry3);
				// restore default lighting
				walk_environment();
				break;
			}
			begin_batch(track->m_blendmaterial3 != null_handle ? track->m_blendmaterial3 : track->SwitchExtension->m_material3);
			m_pathbatch.emplace_back(track->SwitchExtension->Geometry3);
			break;
		}
		case rendermode::shadows:
        {
			if ((std::abs(track->fTexHeight1) < 0.35f) || ((track->iCategoryFlag == 1) && (track->eType != tt_Normal)))
			{
				// shadows are only calculated for high enough trackbeds
				continue;
			}
            if( walk_material_data( track->m_material2 ).shadow_rank > Global.gfx_shadow_rank_cutoff ) {
                // skip if the shadow caster rank is too low for currently set threshold
                continue;
            }
			walk_shadow_material(track->SwitchExtension->m_material3);
			walk_draw(track->SwitchExtension->Geometry3);
			break;
		}
		case rendermode::pickscenery: // pick scenery mode uses piece-by-piece approach
		case rendermode::pickcontrols:
		default:
		{
			break;
		}
		}
	}
	flush_batch();

	// cut faces of rails, in matte variants of the rail materials
	if ((m_renderpass.draw_mode == rendermode::color) || (m_renderpass.draw_mode == rendermode::reflections))
	{
		for (auto first{First}; first != Last; ++first)
		{
			auto const track{*first};

			if (false == track->m_visible)
			{
				continue;
			}
			std::vector<std::pair<material_handle, TTrack::geometryhandle_sequence *>> parts{{track->m_materialcut1, &track->GeometryCut1}, {track->m_materialcut2, &track->GeometryCut2}};
			for (auto &tread : track->GeometryTreads)
			{
				parts.emplace_back(tread.material, &tread.geometry);
				parts.emplace_back(tread.matte, &tread.cuts);
			}
			for (auto const &cuts : parts)
			{
				if ((cuts.first == null_handle) || cuts.second->empty())
				{
					continue;
				}
				if (track->eEnvironment != e_flat)
				{
					flush_batch();
					walk_environment(track->eEnvironment);
					walk_material(cuts.first);
					walk_draw(std::begin(*cuts.second), std::end(*cuts.second));
					walk_environment();
					continue;
				}
				begin_batch(cuts.first);
				m_pathbatch.insert(std::end(m_pathbatch), std::begin(*cuts.second), std::end(*cuts.second));
			}
		}
		flush_batch();
	}

	// fourth pass: per-track sleeper models (sleepermodel optional directive).
	// drawn after rails/trackbeds so depth pre-pass culling is favourable, and only in passes
	// where Render_Sleepers actually does work (it gates itself on draw mode / distance).
	switch( m_renderpass.draw_mode ) {
	case rendermode::color:
	case rendermode::reflections: {
		for( auto first { First }; first != Last; ++first ) {
			auto *track = *first;
			if( false == track->m_visible )       { continue; }
			if( false == track->m_sleeper_enabled ) { continue; }
			// a track with the sleepers and timbers of eu07.ini lays its sleeper model there
			if( track->m_sleepers3d.active )      { continue; }
			// the backend may draw them itself (the gpu culling of the opengl renderer)
			if( walk_queue_sleepers( track ) ) { continue; }
			Render_Sleepers( track );
		}
		if( m_renderpass.draw_mode == rendermode::color ) {
			Render_Timbers( First, Last );
		}
		break;
	}
	case rendermode::shadows: {
		Render_Timbers( First, Last );
		break;
	}
	default:
		break;
	}

	// post-render reset
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
    {
		// restore standard face cull mode
		walk_cull_face(true);
		break;
	}
	default:
	{
		break;
	}
	}
}

void scene_walk::Render(TMemCell *Memcell)
{

	::glPushMatrix();
	auto const position = Memcell->location() - m_renderpass.pass_camera.position();
	::glTranslated(position.x, position.y + 0.5, position.z);

	switch (m_renderpass.draw_mode)
	{
	case rendermode::color:
	{
		break;
	}
	case rendermode::shadows:
    case rendermode::pickscenery:
	{
		break;
	}
	case rendermode::reflections:
	case rendermode::pickcontrols:
	{
		break;
	}
	default:
	{
		break;
	}
	}

	::glPopMatrix();
}

void scene_walk::Render_Alpha(scene::basic_region *Region)
{

	// sort the nodes based on their distance to viewer
	std::sort(std::begin(m_cellqueue), std::end(m_cellqueue), [](distancecell_pair const &Left, distancecell_pair const &Right) { return (Left.first) < (Right.first); });

	Render_Alpha(std::rbegin(m_cellqueue), std::rend(m_cellqueue));
}

void scene_walk::Render_Alpha(cell_sequence::reverse_iterator First, cell_sequence::reverse_iterator Last)
{

	// NOTE: this method is launched only during color pass therefore we don't bother with mode test here
	// first pass draws elements which we know are located in section banks, to reduce vbo switching
	{
		auto first{First};
		while (first != Last)
		{

			auto const *cell = first->second;

			if (false == cell->m_shapestranslucent.empty())
			{
				// since all shapes of the cell share center point we can optimize out a few calls here
				::glPushMatrix();
				auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
				::glTranslated(originoffset.x, originoffset.y, originoffset.z);
				// render
				// NOTE: we can reuse the method used to draw opaque geometry
				for (auto const &shape : cell->m_shapestranslucent)
				{
					Render(shape, false);
				}
				// post-render cleanup
				::glPopMatrix();
			}

			++first;
		}
	}
	// second pass draws elements with their own vbos
	{
		auto first{First};
		while (first != Last)
		{

			auto const *cell = first->second;

			// translucent parts of instanced models
			for (auto *instance : cell->m_instancetranslucent)
			{
				Render_Alpha(instance);
			}
			// translucent parts of vehicles
			for (auto *path : cell->m_paths)
			{
				for (auto *dynamic : path->Dynamics)
				{
					Render_Alpha(dynamic);
				}
			}

			++first;
		}
	}
	// third pass draws the wires;
	// wires use section vbos, but for the time being we want to draw them at the very end
	{
		auto first{First};
		while (first != Last)
		{

			auto const *cell = first->second;

			if ((false == cell->m_traction.empty() || (false == cell->m_lines.empty())))
			{
				// since all shapes of the cell share center point we can optimize out a few calls here
				::glPushMatrix();
				auto const originoffset{cell->m_area.center - m_renderpass.pass_camera.position()};
				::glTranslated(originoffset.x, originoffset.y, originoffset.z);
				walk_material(null_handle);
				// render
				for (auto *traction : cell->m_traction)
				{
					Render_Alpha(traction);
				}
				for (auto &lines : cell->m_lines)
				{
					Render_Alpha(lines);
				}
				// post-render cleanup
				::glPopMatrix();
			}

			++first;
		}
	}
}

void scene_walk::Render_Alpha(TAnimModel *Instance)
{

	if (false == Instance->m_visible) { return; }
    if( false == m_renderpass.pass_camera.visible( Instance->m_area ) ) { return; }

    double distancesquared;
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
    {
        // 'camera' for the light pass is the light source, but we need to draw what the 'real' camera sees
		distancesquared = glm::length2((Instance->location() - m_renderpass.viewport_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
		break;
	}
	default:
	{
		distancesquared = glm::length2((Instance->location() - m_renderpass.pass_camera.position()) / (double)Global.ZoomFactor) / Global.fDistanceFactor;
		break;
	}
	}
	if ((distancesquared < Instance->m_rangesquaredmin) || (distancesquared >= Instance->m_rangesquaredmax))
	{
		return;
	}
     // crude way to reject early items too far to affect the output (mostly relevant for shadow passes)
    auto const drawdistancethreshold{ m_renderpass.draw_range + 250 };
    if( distancesquared > sq(drawdistancethreshold) ) {
        return;
    }
   // second stage visibility cull, reject modelstoo far away to be noticeable
    auto const radiussquared { Instance->radius() * Instance->radius() };
    if( radiussquared * Global.ZoomFactor / distancesquared < 0.003 * 0.003 ) {
        return;
    }

	Instance->RaPrepare();
	if (Instance->pModel)
	{
		auto const &scale = Instance->Scale();
		bool const scaled = ( scale.x != 1.0f || scale.y != 1.0f || scale.z != 1.0f );
		if( scaled ) {
			// scaled instance: build the per-instance transform locally and call
			// the distance-only Render_Alpha so we can inject glScalef before
			// the submodel walk. Mirror of the opaque path in Render(TAnimModel*).
			::glPushMatrix();
			auto const Position = Instance->location() - m_renderpass.pass_camera.position();
			auto const Angle = Instance->vAngle;
			::glTranslated( Position.x, Position.y, Position.z );
			if( Angle.y != 0.0 ) ::glRotated( Angle.y, 0.f, 1.f, 0.f );
			if( Angle.x != 0.0 ) ::glRotated( Angle.x, 1.f, 0.f, 0.f );
			if( Angle.z != 0.0 ) ::glRotated( Angle.z, 0.f, 0.f, 1.f );
			::glScalef( scale.x, scale.y, scale.z );
			Render_Alpha( Instance->pModel, Instance->Material(), distancesquared );
			::glPopMatrix();
		}
		else {
			Render_Alpha(Instance->pModel, Instance->Material(), distancesquared, Instance->location() - m_renderpass.pass_camera.position(), Instance->vAngle);
		}
	}
}

void scene_walk::Render_Alpha(TTraction *Traction)
{
	glDebug("Render_Alpha TTraction");

	auto const distancesquared { glm::length2( ( Traction->location() - m_renderpass.pass_camera.position() ) / (double)Global.ZoomFactor ) / Global.fDistanceFactor };
	if ((distancesquared < Traction->m_rangesquaredmin) || (distancesquared >= Traction->m_rangesquaredmax))
	{
		return;
	}

	if (false == Traction->m_visible)
	{
		return;
	}
	// rysuj jesli sa druty i nie zerwana
	if ((Traction->Wires == 0) || (true == TestFlag(Traction->DamageFlag, 128)))
	{
		return;
	}
	// setup
	auto const distance{static_cast<float>(std::sqrt(distancesquared))};
	auto const linealpha = 20.f * Traction->WireThickness / std::max(0.5f * Traction->radius() + 1.f, distance - (0.5f * Traction->radius()));

	// render

	// McZapkie-261102: kolor zalezy od materialu i zasniedzenia
	model_ubs.param[0] = glm::vec4(Traction->wire_color(), glm::min(1.0f, linealpha));

	if (m_renderpass.draw_mode == rendermode::shadows)
		walk_shadow_material(null_handle);
	else
		walk_program( walk_shader::line );

	walk_draw(Traction->m_geometry);

	// up close also the solids of a span drawn by the type of the catenary: the wires and ropes in their real thickness and the fittings.
	// they're opaque, so they're drawn without blending and write the depth; otherwise they'd cover one another by the order they're drawn in
	if (auto const solid{catenary_span_solid(*Traction, Global.pCamera.Pos)}; solid.wires != null_handle)
	{
		auto const blending{walk_blending_enabled()};
		if (m_renderpass.draw_mode != rendermode::shadows)
			walk_blending(false);
		model_ubs.param[0] = glm::vec4(Traction->wire_color(), 1.0f);
		walk_draw(solid.wires);
		++m_renderpass.draw_stats.drawcalls;
		if (solid.fittings != null_handle)
		{
			walk_draw(solid.fittings);
			++m_renderpass.draw_stats.drawcalls;
		}
		// the rope of the tensioning device at the end of the span (steel)
		if (solid.rope != null_handle)
		{
			auto const &steel{catenary_rope_metal()};
			model_ubs.param[0] = glm::vec4(steel.color[0], steel.color[1], steel.color[2], 1.0f);
			walk_draw(solid.rope);
			++m_renderpass.draw_stats.drawcalls;
		}
		if (m_renderpass.draw_mode != rendermode::shadows)
			walk_blending(blending);
	}

	// debug data
	++m_renderpass.draw_stats.traction;
	++m_renderpass.draw_stats.drawcalls;
}

void scene_walk::Render_Alpha(scene::lines_node const &Lines)
{
	glDebug("Render_Alpha scene::lines_node");

	auto const &data{Lines.data()};

	auto const distancesquared { glm::length2( ( data.area.center - m_renderpass.pass_camera.position() ) / (double)Global.ZoomFactor ) / Global.fDistanceFactor };
	if ((distancesquared < data.rangesquared_min) || (distancesquared >= data.rangesquared_max))
	{
		return;
	}
	// setup
	auto const distance{static_cast<float>(std::sqrt(distancesquared))};
	auto const linealpha =
	    (data.line_width > 0.f ? 10.f * data.line_width / std::max(0.5f * data.area.radius + 1.f, distance - (0.5f * data.area.radius)) : 1.f); // negative width means the lines are always opague

	model_ubs.param[0] = glm::vec4(glm::vec3(data.lighting.diffuse * walk_sun_ambient()), glm::min(1.0f, linealpha));

	if (m_renderpass.draw_mode == rendermode::shadows)
		walk_shadow_material(null_handle);
	else
		walk_program( walk_shader::line );
	walk_draw(data.geometry);

	++m_renderpass.draw_stats.lines;
	++m_renderpass.draw_stats.drawcalls;
}

bool scene_walk::Render_Alpha(TDynamicObject *Dynamic)
{
	if (!Global.render_cab && Global.pCamera.m_owner == Dynamic)
		return false;

	if (false == Dynamic->renderme)
	{
		return false;
	}

	// setup
	TSubModel::iInstance = (size_t)Dynamic; //żeby nie robić cudzych animacji
	TSubModel::iVariantSeed = Dynamic->variant_seed();
	glm::dvec3 const originoffset = Dynamic->vPosition - m_renderpass.pass_camera.position();
	// lod visibility ranges are defined for base (x 1.0) viewing distance. for render we adjust them for actual range multiplier and zoom
	float squaredistance;
	switch (m_renderpass.draw_mode)
	{
	case rendermode::shadows:
	default:
	{
		squaredistance = glm::length2(glm::vec3{originoffset} / Global.ZoomFactor);
		break;
	}
	}
	Dynamic->ABuLittleUpdate(squaredistance); // ustawianie zmiennych submodeli dla wspólnego modelu

	glm::mat4 future_stack = model_ubs.future;

	glm::mat4 mv = OpenGLMatrices.data(GL_MODELVIEW);
	model_ubs.future *= glm::translate(mv, glm::vec3(m_motionhistory ? -Dynamic->get_future_movement() : Dynamic->get_future_movement())) * glm::inverse(mv);
	// the velocity it leaves in the colour pass keeps the motion blur on (Render_pass())
	if ((m_renderpass.draw_mode == rendermode::color) && (Dynamic->get_future_movement() != glm::dvec3{0.0}))
		m_vehiclesmoving = true;

	::glPushMatrix();

	::glTranslated(originoffset.x, originoffset.y, originoffset.z);
	::glMultMatrixd(glm::value_ptr(Dynamic->mMatrix));

	if (Dynamic->fShade > 0.0f)
	{
		// change light level based on light level of the occupied track
		walk_sunlight(Dynamic->fShade);
	}

    // render
    if( Dynamic->mdLowPolyInt ) {
        // low poly interior
        Render_Alpha( Dynamic->mdLowPolyInt, Dynamic->Material(), squaredistance );
    }
    if( Dynamic->mdModel ) {
        // main model
        Render_Alpha( Dynamic->mdModel, Dynamic->Material(), squaredistance );
    }
    // optional attached models
    for( auto *attachment : Dynamic->mdAttachments ) {
        Render_Alpha( attachment, Dynamic->Material(), squaredistance );
    }
    // optional coupling adapters
    Render_coupler_adapter( Dynamic, squaredistance, end::front, true );
    Render_coupler_adapter( Dynamic, squaredistance, end::rear, true );
    if( Dynamic->mdLoad ) {
        // renderowanie nieprzezroczystego ładunku
        Render_Alpha( Dynamic->mdLoad, Dynamic->Material(), squaredistance, { 0.f, Dynamic->LoadOffset, 0.f }, {} );
    }

	// post-render cleanup
	if (Dynamic->fShade > 0.0f)
	{
		// restore regular light level
		walk_sunlight();
	}

	::glPopMatrix();

	model_ubs.future = future_stack;

	if (Dynamic->btnOn)
		Dynamic->TurnOff(); // przywrócenie domyślnych pozycji submodeli

	return true;
}

bool scene_walk::Render_Alpha(TModel3d *Model, material_data const *Material, float const Squaredistance)
{

	auto alpha = (Material != nullptr ? Material->textures_alpha : 0x30300030);

	if (0 == (alpha & Model->iFlags & 0x2F2F002F))
	{
		// nothing to render
		return false;
	}

	Model->Root->fSquareDist = Squaredistance; // zmienna globalna!

	// setup
	Model->Root->ReplacableSet((Material != nullptr ? Material->replacable_skins : nullptr), alpha);

	Model->Root->pRoot = Model;

	// render
	Render_Alpha(Model->Root);

	// post-render cleanup

	return true;
}

bool scene_walk::Render_Alpha(TModel3d *Model, material_data const *Material, float const Squaredistance, glm::dvec3 const &Position, glm::vec3 const &Angle)
{
	::glPushMatrix();
	::glTranslated(Position.x, Position.y, Position.z);
	if (Angle.y != 0.0)
		::glRotated(Angle.y, 0.f, 1.f, 0.f);
	if (Angle.x != 0.0)
		::glRotated(Angle.x, 1.f, 0.f, 0.f);
	if (Angle.z != 0.0)
		::glRotated(Angle.z, 0.f, 0.f, 1.f);

	auto const result = Render_Alpha(Model, Material, Squaredistance); // position is effectively camera offset

	::glPopMatrix();

	return result;
}

// characters of a text display, laid out in a row. the text can be encoded in utf-8 or in windows-1250
void scene_walk::Render_Alpha_text(TSubModel *Submodel)
{
	if (TSubModel::pasText == nullptr)
	{
		return;
	}
	for (auto *p : Submodel->text_letters(*TSubModel::pasText))
	{ // translucent only, for the time being
		Render_Alpha(p);
		if (p->fMatrix)
			::glMultMatrixf(p->fMatrix->readArray()); // move on to the place of the next character
	}
}

void scene_walk::Render_Alpha(TSubModel *Submodel)
{
	// renderowanie przezroczystych przez DL
	if ((Submodel->iVisible) && (TSubModel::fSquareDist >= Submodel->fSquareMinDist) && (TSubModel::fSquareDist < Submodel->fSquareMaxDist) && (false == m_previewing || Submodel->fVisible > 0.f))
	{
		glm::mat4 future_stack = model_ubs.future;

		if (Submodel->iFlags & 0xC000)
		{
			::glPushMatrix();
			if (Submodel->fMatrix)
				::glMultMatrixf(Submodel->fMatrix->readArray());
			if (Submodel->b_aAnim != TAnimType::at_None)
			{
				Submodel->RaAnimation(Submodel->b_aAnim);

				glm::mat4 mv = OpenGLMatrices.data(GL_MODELVIEW);
				model_ubs.future *= (mv * (m_motionhistory ? glm::inverse(Submodel->future_transform) : Submodel->future_transform)) * glm::inverse(mv);
			}
		}

		if (Submodel->eType < TP_ROTATOR)
		{
			// renderowanie obiektów OpenGL
			if (Submodel->iAlpha & Submodel->iFlags & 0x2F)
			{
                // rysuj gdy element przezroczysty

                // debug data
		        ++m_renderpass.draw_stats.submodels;
		        ++m_renderpass.draw_stats.drawcalls;

				switch (m_renderpass.draw_mode)
				{
				case rendermode::color:
				{
					// material configuration:
					// textures...
					if (Submodel->m_material < 0)
					{ // zmienialne skóry
						walk_material(submodel_material(Submodel), Submodel);
					}
					else
					{
						walk_material(Submodel->m_material, Submodel);
					}
					// ...luminance

					auto const isemissive { ( Submodel->f4Emision.a > 0.f ) && ( Global.fLuminance < Submodel->fLight ) };
					if (isemissive)
						model_ubs.emission = Submodel->f4Emision.a;

					// main draw call
					walk_draw(Submodel->m_geometry.handle);

					model_ubs.emission = 0.0f;
					break;
				}
				case rendermode::shadows:
				{
                    // skip if the shadow caster rank is too low for currently set threshold
                    // or if the submodel is hidden (colour pass only fades it out, e.g. inactive seasonal variant)
                    if( ( walk_material_data( Submodel ).shadow_rank > Global.gfx_shadow_rank_cutoff )
                     || ( Submodel->fVisible <= 0.f ) )
                    {
                        --m_renderpass.draw_stats.submodels;
                        --m_renderpass.draw_stats.drawcalls;
                        break;
                    }

					if (Submodel->m_material < 0)
					{ // zmienialne skóry
						walk_shadow_material(submodel_material(Submodel));
					}
					else
					{
						walk_shadow_material(Submodel->m_material);
					}
					walk_draw(Submodel->m_geometry.handle);
					break;
				}
				default:
				{
					break;
				}
				}
			}
		}
		else if (Submodel->eType == TP_FREESPOTLIGHT)
		{
			// NOTE: we're forced here to redo view angle calculations etc, because this data isn't instanced but stored along with the single mesh
			// TODO: separate instance data from reusable geometry
			auto const &modelview = OpenGLMatrices.data(GL_MODELVIEW);
			auto const lightcenter = modelview * glm::mix(glm::vec4(0.f, 0.f, -0.05f, 1.f), glm::vec4(0.f, 0.f, -0.25f, 1.f),
			                                                 static_cast<float>(TSubModel::fSquareDist / Submodel->fSquareMaxDist)); // pozycja punktu świecącego względem kamery
			Submodel->fCosViewAngle = glm::dot(glm::normalize(modelview * glm::vec4(0.f, 0.f, -1.f, 1.f) - lightcenter), glm::normalize(-lightcenter));

			if (Global.fLuminance < Submodel->fLight || Global.Overcast > 1.0f)
			{
				if (Submodel->fCosViewAngle > Submodel->fCosFalloffAngle)
				{
					// only bother if the viewer is inside the visibility cone
					// luminosity at night is at level of ~0.1, so the overall resulting transparency in clear conditions is ~0.5 at full 'brightness'
					auto glarelevel{std::clamp(std::max<float>(0.6f - Global.fLuminance, // reduce the glare in bright daylight
						                                  Global.Overcast - 1.f), // ensure some glare in rainy/foggy conditions
						                  0.f, 1.f)};
					// view angle attenuation
					float const anglefactor{std::clamp((Submodel->fCosViewAngle - Submodel->fCosFalloffAngle) / (Submodel->fCosHotspotAngle - Submodel->fCosFalloffAngle), 0.f, 1.f)};

					glarelevel *= anglefactor;

					if (glarelevel > 0.0f)
					{
						walk_glare(true);

						::glPushMatrix();
						::glLoadIdentity(); // macierz jedynkowa
						::glTranslatef(lightcenter.x, lightcenter.y, lightcenter.z); // początek układu zostaje bez zmian
						::glRotated(std::atan2(lightcenter.x, lightcenter.z) * 180.0 / M_PI, 0.0, 1.0, 0.0); // jedynie obracamy w pionie o kąt

						auto const lightcolor = glm::vec3(Submodel->DiffuseOverride.r < 0.f ? // -1 indicates no override
						                           Submodel->f4Diffuse :
						                           Submodel->DiffuseOverride);

						walk_program( walk_shader::billboard );
						walk_texture(0, m_glaretexture);
						model_ubs.param[0] = glm::vec4(glm::vec3(lightcolor), Submodel->fVisible * glarelevel);

						// main draw call
						// drawn where the light itself was seen in the last frame (an occlusion query of the opengl renderer)
						walk_glare_draw(Submodel, m_billboardgeometry);

						walk_glare(false);

						::glPopMatrix();
					}
				}
			}

			if (Submodel->fCosViewAngle > Submodel->fCosFalloffAngle)
			{
				// kąt większy niż maksymalny stożek swiatła
				float lightlevel = 1.f; // TODO, TBD: parameter to control light strength
				// view angle attenuation
				float const anglefactor = std::clamp((Submodel->fCosViewAngle - Submodel->fCosFalloffAngle) / (Submodel->fCosHotspotAngle - Submodel->fCosFalloffAngle), 0.f, 1.f);
				lightlevel *= anglefactor;

				// distance attenuation. NOTE: since it's fixed pipeline with built-in gamma correction we're using linear attenuation
				// we're capping how much effect the distance attenuation can have, otherwise the lights get too tiny at regular distances
				float const distancefactor{std::max(0.5f, (Submodel->fSquareMaxDist - TSubModel::fSquareDist) / Submodel->fSquareMaxDist)};
                auto const pointsize{std::max(3.f, 5.f * distancefactor * anglefactor)};
				// additionally reduce light strength for farther sources in rain or snow
				if (Global.Overcast > 0.75f)
				{
					float const precipitationfactor{std::lerp(std::lerp(1.f, 0.25f, std::clamp(Global.Overcast * 0.75f - 0.5f, 0.f, 1.f)), 1.f, distancefactor)};
					lightlevel *= precipitationfactor;
				}

				if (lightlevel > 0.f)
				{
					::glPushMatrix();
					::glLoadIdentity();
					::glTranslatef(lightcenter.x, lightcenter.y, lightcenter.z); // początek układu zostaje bez zmian

					// material configuration:
					// limit impact of dense fog on the lights
					auto const lightrange { std::max<float>( 500, m_fogrange * 2 ) }; // arbitrary, visibility at least 500m
					model_ubs.fog_density = 1.0 / lightrange;

					// main draw call
					model_ubs.emission = 1.0f;

					auto lightcolor = glm::vec3(Submodel->DiffuseOverride.r < 0.f ? // -1 indicates no override
                                            Submodel->f4Diffuse :
                                            Submodel->DiffuseOverride);

					walk_program( walk_shader::freespot );

					if (Global.Overcast > 1.0f)
					{
						// fake fog halo
						float const fogfactor{std::lerp(1.5f, 1.f, std::clamp(Global.fFogEnd / 2000, 0.f, 1.f)) * std::max(1.f, Global.Overcast)};
                        model_ubs.param[1].x = pointsize * fogfactor * 4.0f;
						model_ubs.param[0] = glm::vec4(glm::vec3(lightcolor), Submodel->fVisible * std::min(1.f, lightlevel) * 0.5f);

						walk_draw(Submodel->m_geometry.handle);
					}
                    model_ubs.param[1].x = pointsize * 4.0f;
					model_ubs.param[0] = glm::vec4(glm::vec3(lightcolor), Submodel->fVisible * std::min(1.f, lightlevel));

                    walk_light_query(Submodel, true);

					walk_draw(Submodel->m_geometry.handle);

                    walk_light_query(Submodel, false);

					// post-draw reset
					model_ubs.emission = 0.0f;
					model_ubs.fog_density = 1.0f / m_fogrange;

					::glPopMatrix();
				}
			}
		}

		if (Submodel->Child != nullptr)
		{
			if (Submodel->eType == TP_TEXT)
			{ // tekst renderujemy w specjalny sposób, zamiast submodeli z łańcucha Child
				Render_Alpha_text(Submodel);
			}
			else if (Submodel->iAlpha & Submodel->iFlags & 0x002F0000)
				Render_Alpha(Submodel->Child);
		}

		if (Submodel->iFlags & 0xC000)
		{
			model_ubs.future = future_stack;
			::glPopMatrix();
		}
	}
	/*
	    if( Submodel->b_aAnim < at_SecondsJump )
	        Submodel->b_aAnim = at_None; // wyłączenie animacji dla kolejnego użycia submodelu
	*/
	if (Submodel->Next != nullptr)
		if (Submodel->iAlpha & Submodel->iFlags & 0x2F000000)
			Render_Alpha(Submodel->Next);
};

} // namespace gfx

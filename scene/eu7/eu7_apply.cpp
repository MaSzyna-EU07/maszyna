/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/eu7/eu7_apply.h"

#include "scene/eu7/eu7_loader.h"
#include "scene/eu7/eu7_load_stats.h"
#include "scene/eu7/eu7_transform.h"
#include "simulation/simulation.h"
#include "simulation/simulationstateserializer.h"
#include "utilities/Logs.h"

namespace scene::eu7 {

namespace {

void
insert_mesh_shapes(
    basic_region &region,
    Eu7Module const &module,
    scene::scratch_data &scratch ) {
    for( auto const &source : module.scene.shapes ) {
        shape_node shape { build_shape_node( source ) };
        region.insert( shape, scratch, true );
    }
}

void
insert_line_shapes(
    basic_region &region,
    Eu7Module const &module,
    scene::scratch_data &scratch ) {
    for( auto const &source : module.scene.lines ) {
        lines_node lines { build_lines_node( source ) };
        region.insert( lines, scratch );
    }
}

void
transform_segment_path( Eu7SegmentPath &path, Eu7TransformContext const &prefix ) {
    path.p_start = transform_point( path.p_start, prefix );
    path.cp_out = transform_vector( path.cp_out, prefix );
    path.cp_in = transform_vector( path.cp_in, prefix );
    path.p_end = transform_point( path.p_end, prefix );
}

void
transform_shape_vertices( std::vector<Eu7WorldVertex> &vertices, Eu7TransformContext const &prefix ) {
    for( auto &vertex : vertices ) {
        vertex.position = transform_point( vertex.position, prefix );
    }
}

} // namespace

bool
is_model_only_module( Eu7Module const &module ) {
    if( module.has_terrain_chunk ) {
        return false;
    }

    auto const &scene { module.scene };
    return (
        false == scene.models.empty() &&
        scene.tracks.empty() &&
        scene.traction.empty() &&
        scene.power_sources.empty() &&
        scene.shapes.empty() &&
        scene.terrain_shapes.empty() &&
        scene.lines.empty() &&
        scene.memcells.empty() &&
        scene.event_launchers.empty() &&
        scene.dynamics.empty() &&
        scene.sounds.empty() &&
        scene.trainsets.empty() &&
        scene.events.empty() &&
        scene.first_init_count == 0 );
}

void
compose_models_with_prefix( std::vector<Eu7Model> &models, Eu7TransformContext const &prefix ) {
    if( transform_is_empty( prefix ) ) {
        return;
    }

    for( auto &model : models ) {
        model.location = transform_point( model.location, prefix );
        model.angles.x += prefix.rotation.x;
        model.angles.y += prefix.rotation.y;
        model.angles.z += prefix.rotation.z;
        compose_node_transform( model.node.transform, prefix );
        model.node.area.center = model.location;
    }
}

void
compose_scene_with_include_prefix( Eu7Scene &scene, Eu7TransformContext const &prefix ) {
    if( transform_is_empty( prefix ) ) {
        return;
    }

    for( auto &track : scene.tracks ) {
        for( auto &path : track.paths ) {
            transform_segment_path( path, prefix );
        }
        compose_node_transform( track.node.transform, prefix );
    }

    for( auto &traction : scene.traction ) {
        traction.wire_p1 = transform_point( traction.wire_p1, prefix );
        traction.wire_p2 = transform_point( traction.wire_p2, prefix );
        traction.wire_p3 = transform_point( traction.wire_p3, prefix );
        traction.wire_p4 = transform_point( traction.wire_p4, prefix );
        compose_node_transform( traction.node.transform, prefix );
    }

    for( auto &source : scene.power_sources ) {
        source.position = transform_point( source.position, prefix );
        compose_node_transform( source.node.transform, prefix );
    }

    for( auto &shape : scene.shapes ) {
        shape.origin = transform_point( shape.origin, prefix );
        transform_shape_vertices( shape.vertices, prefix );
        compose_node_transform( shape.node.transform, prefix );
    }

    for( auto &shape : scene.terrain_shapes ) {
        shape.origin = transform_point( shape.origin, prefix );
        transform_shape_vertices( shape.vertices, prefix );
        compose_node_transform( shape.node.transform, prefix );
    }

    for( auto &lines : scene.lines ) {
        lines.origin = transform_point( lines.origin, prefix );
        transform_shape_vertices( lines.vertices, prefix );
        compose_node_transform( lines.node.transform, prefix );
    }

    compose_models_with_prefix( scene.models, prefix );

    for( auto &cell : scene.memcells ) {
        cell.node.area.center = transform_point( cell.node.area.center, prefix );
        compose_node_transform( cell.node.transform, prefix );
    }

    for( auto &launcher : scene.event_launchers ) {
        launcher.location = transform_point( launcher.location, prefix );
        compose_node_transform( launcher.node.transform, prefix );
        launcher.node.area.center = launcher.location;
    }

    for( auto &vehicle : scene.dynamics ) {
        compose_node_transform( vehicle.node.transform, prefix );
    }

    for( auto &sound : scene.sounds ) {
        sound.location = transform_point( sound.location, prefix );
        compose_node_transform( sound.node.transform, prefix );
        sound.node.area.center = sound.location;
    }
}

void
apply_module(
    Eu7Module const &module,
    simulation::state_serializer &serializer,
    scene::scratch_data &scratch ) {
    if( simulation::Region != nullptr ) {
        {
            ScopedTimer const timer { load_stats().terr_ms };
            insert_terrain_shapes( *simulation::Region, module );
        }
        {
            ScopedTimer const timer { load_stats().mesh_ms };
            insert_mesh_shapes( *simulation::Region, module, scratch );
        }
        {
            ScopedTimer const timer { load_stats().line_ms };
            insert_line_shapes( *simulation::Region, module, scratch );
        }
    }

    serializer.apply_eu7_scene( module.scene, scratch );
}

} // namespace scene::eu7

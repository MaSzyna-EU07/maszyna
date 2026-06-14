/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "simulation/simulationstateserializer.h"

#include <unordered_set>

#include "utilities/Globals.h"
#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "simulation/simulationsounds.h"
#include "simulation/simulationenvironment.h"
#include "scene/scenenodegroups.h"
#include "rendering/particles.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "model/AnimModel.h"
#include "model/MdlMngr.h"
#include "rendering/lightarray.h"
#include "world/TractionPower.h"
#include "application/application.h"
#include "rendering/renderer.h"
#include "utilities/Logs.h"
#include "scene/eu7/eu7_bake.h"
#include "scene/eu7/eu7_loader.h"
#include "scene/eu7/eu7_load_stats.h"
#include "scene/eu7/eu7_pack_bench.h"
#include "scene/eu7/eu7_transform.h"
#include "world/Track.h"
#include "world/Traction.h"
#include "audio/sound.h"

#include <chrono>
#include <deque>

namespace simulation {

namespace {

constexpr double kDeferTrainsetHorizDistM { 4000.0 };
constexpr double kDeferTrainsetHorizDistSq {
    kDeferTrainsetHorizDistM * kDeferTrainsetHorizDistM };

struct DeferredEu7Trainset {
    scene::eu7::Eu7Trainset trainset;
    std::vector<scene::eu7::Eu7Dynamic> vehicles;
};

std::deque<DeferredEu7Trainset> g_deferred_eu7_trainsets;

struct Eu7TransformState {
    std::size_t group_depth { 0 };
};

void
clear_deferred_eu7_trainsets() {
    g_deferred_eu7_trainsets.clear();
}

[[nodiscard]] glm::dvec3
eu7_trainset_spawn_origin() {
    auto const &saved_camera { Global.FreeCameraInit[ 0 ] };
    if( saved_camera.x != 0.0 || saved_camera.y != 0.0 || saved_camera.z != 0.0 ) {
        return saved_camera;
    }

    if(
        false == Global.local_start_vehicle.empty() &&
        Global.local_start_vehicle != "ghostview" ) {
        if( auto *vehicle { simulation::Vehicles.find( Global.local_start_vehicle ) };
            vehicle != nullptr ) {
            return vehicle->GetPosition();
        }
    }

    for( auto *vehicle : simulation::Vehicles.sequence() ) {
        if( vehicle != nullptr ) {
            return vehicle->GetPosition();
        }
    }

    return {};
}

[[nodiscard]] bool
eu7_trainset_is_player(
    scene::eu7::Eu7Trainset const &trainset,
    scene::eu7::Eu7Scene const &scene ) {
    if(
        Global.local_start_vehicle.empty() ||
        Global.local_start_vehicle == "ghostview" ) {
        return false;
    }

    for( auto const index : trainset.vehicle_indices ) {
        if(
            index < scene.dynamics.size() &&
            scene.dynamics[ index ].node.name == Global.local_start_vehicle ) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] double
eu7_trainset_horiz_dist_sq( scene::eu7::Eu7Trainset const &trainset ) {
    auto *path { simulation::Paths.find( trainset.track ) };
    if( path == nullptr ) {
        return 0.0;
    }

    auto const spawn { eu7_trainset_spawn_origin() };
    if( spawn.x == 0.0 && spawn.y == 0.0 && spawn.z == 0.0 ) {
        return 0.0;
    }

    auto const track_pos { glm::dvec3 { path->location() } };
    auto const dx { track_pos.x - spawn.x };
    auto const dz { track_pos.z - spawn.z };
    return dx * dx + dz * dz;
}

[[nodiscard]] bool
eu7_should_load_trainset_now(
    scene::eu7::Eu7Trainset const &trainset,
    scene::eu7::Eu7Scene const &scene ) {
    if( eu7_trainset_is_player( trainset, scene ) ) {
        return true;
    }
    if( eu7_trainset_horiz_dist_sq( trainset ) <= kDeferTrainsetHorizDistSq ) {
        return true;
    }
    return false;
}

void
eu7_queue_deferred_trainset(
    scene::eu7::Eu7Trainset const &trainset,
    scene::eu7::Eu7Scene const &scene ) {
    DeferredEu7Trainset job;
    job.trainset = trainset;
    job.vehicles.reserve( trainset.vehicle_indices.size() );
    for( auto const index : trainset.vehicle_indices ) {
        if( index < scene.dynamics.size() ) {
            job.vehicles.push_back( scene.dynamics[ index ] );
        }
    }
    g_deferred_eu7_trainsets.push_back( std::move( job ) );
}

[[nodiscard]] glm::dvec3
origin_push_delta( std::vector<glm::dvec3> const &stack, std::size_t const index ) {
    auto const &cumulative { stack[ index ] };
    if( index == 0 ) {
        return cumulative;
    }
    auto const &parent { stack[ index - 1 ] };
    return { cumulative.x - parent.x, cumulative.y - parent.y, cumulative.z - parent.z };
}

[[nodiscard]] glm::vec3
scale_push_factor( std::vector<glm::dvec3> const &stack, std::size_t const index ) {
    auto const &cumulative { stack[ index ] };
    auto const parent { index == 0 ? glm::dvec3{ 1.0, 1.0, 1.0 } : stack[ index - 1 ] };
    return {
        parent.x != 0.0 ? static_cast<float>( cumulative.x / parent.x ) : static_cast<float>( cumulative.x ),
        parent.y != 0.0 ? static_cast<float>( cumulative.y / parent.y ) : static_cast<float>( cumulative.y ),
        parent.z != 0.0 ? static_cast<float>( cumulative.z / parent.z ) : static_cast<float>( cumulative.z ) };
}

void
sync_scratch_transform(
    scene::scratch_data &scratchpad,
    Eu7TransformState &state,
    scene::eu7::Eu7TransformContext const &target ) {
    while( state.group_depth > target.group_depth ) {
        scene::Groups.close();
        --state.group_depth;
    }
    while( scratchpad.location.scale.size() > target.scale_stack.size() ) {
        scratchpad.location.scale.pop();
    }
    while( scratchpad.location.offset.size() > target.origin_stack.size() ) {
        scratchpad.location.offset.pop();
    }

    while( scratchpad.location.offset.size() < target.origin_stack.size() ) {
        auto const index { scratchpad.location.offset.size() };
        auto const delta { origin_push_delta( target.origin_stack, index ) };
        scratchpad.location.offset.push(
            delta + (
                scratchpad.location.offset.empty() ?
                    glm::dvec3{} :
                    scratchpad.location.offset.top() ) );
    }

    while( scratchpad.location.scale.size() < target.scale_stack.size() ) {
        auto const index { scratchpad.location.scale.size() };
        auto const &cumulative { target.scale_stack[ index ] };
        scratchpad.location.scale.push( glm::vec3(
            static_cast<float>( cumulative.x ),
            static_cast<float>( cumulative.y ),
            static_cast<float>( cumulative.z ) ) );
    }

    while( state.group_depth < target.group_depth ) {
        scene::Groups.create();
        ++state.group_depth;
    }

    scratchpad.location.rotation = glm::vec3(
        static_cast<float>( target.rotation.x ),
        static_cast<float>( target.rotation.y ),
        static_cast<float>( target.rotation.z ) );
}

[[nodiscard]] scene::node_data
node_data_from_eu7( scene::eu7::Eu7BasicNode const &node ) {
    scene::node_data nodedata;
    if( node.range_squared_max >= std::numeric_limits<double>::max() * 0.5 ) {
        nodedata.range_max = -1.0;
    }
    else {
        nodedata.range_max = std::sqrt( node.range_squared_max );
    }
    nodedata.range_min = std::sqrt( node.range_squared_min );
    nodedata.name = node.name;
    nodedata.type = node.node_type;
    return nodedata;
}

void
preload_unique_pack_meshes(
    std::vector<scene::eu7::Eu7Model> const &models,
    std::size_t const offset,
    std::size_t const end ) {
    std::unordered_set<std::string> unique_files;
    unique_files.reserve( end - offset );

    for( std::size_t i { offset }; i < end; ++i ) {
        auto model_file { models[ i ].model_file };
        if( model_file.empty() || model_file == "notload" ) {
            continue;
        }
        replace_slashes( model_file );
        if( unique_files.insert( model_file ).second ) {
            if( auto *const model { TModelsManager::GetModel( model_file, false, false ) } ) {
                TAnimModel::warm_instanceable_cache( model );
            }
        }
    }
}

[[nodiscard]] bool
pack_model_needs_full_load( scene::eu7::Eu7Model const &model ) {
    return false == model.light_states.empty()
        || false == model.light_colors.empty()
        || false == model.transition;
}

[[nodiscard]] std::string
pack_nodedata_cache_key( scene::eu7::Eu7Model const &model ) {
    return model.model_file + '\x1f' + model.texture_file + '\x1f'
        + std::to_string( model.node.range_squared_min ) + '\x1f'
        + std::to_string( model.node.range_squared_max ) + '\x1f'
        + ( model.is_terrain ? '1' : '0' );
}

[[nodiscard]] scene::node_data const &
pack_nodedata_cached(
    scene::eu7::Eu7Model const &model,
    std::unordered_map<std::string, scene::node_data> &cache ) {
    auto const key { pack_nodedata_cache_key( model ) };
    auto const found { cache.find( key ) };
    if( found != cache.end() ) {
        return found->second;
    }
    scene::node_data nodedata;
    if( model.is_terrain ) {
        nodedata.range_max = -1.0;
        nodedata.range_min = 0.0;
        nodedata.name = model.node.name;
        nodedata.type = "model";
    }
    else {
        nodedata = node_data_from_eu7( model.node );
    }
    auto const inserted { cache.emplace( key, std::move( nodedata ) ) };
    return inserted.first->second;
}

[[nodiscard]] std::string
join_event_targets( std::vector<std::string> const &targets ) {
    std::string joined;
    for( std::size_t i { 0 }; i < targets.size(); ++i ) {
        if( i > 0 ) {
            joined += '|';
        }
        joined += targets[ i ];
    }
    return joined.empty() ? "none" : joined;
}

} // namespace

struct state_serializer::eu7_transform_state {
    Eu7TransformState state;
};

void
state_serializer::insert_eu7_models(
    std::vector<scene::eu7::Eu7Model> const &models,
    scene::scratch_data &scratchpad,
    eu7_transform_state &transform_state ) {
    scene::eu7::ScopedTimer const model_timer { scene::eu7::load_stats().model_ms };
    scene::eu7::load_stats().models += models.size();

    auto const apply_node_transform { [&]( scene::eu7::Eu7BasicNode const &node ) {
        sync_scratch_transform( scratchpad, transform_state.state, node.transform );
    } };

    for( auto const &model : models ) {
        apply_node_transform( model.node );
        scene::node_data nodedata;
        if( model.is_terrain ) {
            nodedata.range_max = -1.0;
            nodedata.range_min = 0.0;
            nodedata.name = model.node.name;
            nodedata.type = "model";
        }
        else {
            nodedata = node_data_from_eu7( model.node );
        }

        auto const local_location {
            scene::eu7::inverse_transform_point( model.location, model.node.transform ) };
        auto const local_rotation_y { model.angles.y - model.node.transform.rotation.y };
        auto *instance { new TAnimModel( nodedata ) };
        instance->Angles( scratchpad.location.rotation + glm::vec3( 0.f, static_cast<float>( local_rotation_y ), 0.f ) );
        if( false == scratchpad.location.scale.empty() ) {
            instance->Scale( scratchpad.location.scale.top() );
        }
        if( model.scale.x != 1.0 || model.scale.y != 1.0 || model.scale.z != 1.0 ) {
            instance->Scale( instance->Scale() * glm::vec3(
                static_cast<float>( model.scale.x ),
                static_cast<float>( model.scale.y ),
                static_cast<float>( model.scale.z ) ) );
        }

        auto model_file { model.model_file };
        auto texture_file { model.texture_file };
        replace_slashes( model_file );
        replace_slashes( texture_file );
        std::string load_tokens { model_file + " " + texture_file };
        if( false == model.light_states.empty() ) {
            load_tokens += " lights";
            for( auto const state : model.light_states ) {
                load_tokens += ' ' + std::to_string( state );
            }
        }
        if( false == model.light_colors.empty() ) {
            load_tokens += " lightcolors";
            for( auto const color : model.light_colors ) {
                load_tokens += ' ' + std::to_string( color );
            }
        }
        if( false == model.transition ) {
            load_tokens += " notransition";
        }
        load_tokens += " endmodel";

        cParser model_parser( load_tokens, cParser::buffer_TEXT, "", false );
        if( false == instance->Load( &model_parser, nodedata.range_min < 0.0 ) ) {
            SafeDelete( instance );
            continue;
        }
        instance->location( transform( local_location, scratchpad ) );

        if( nodedata.range_min < 0.0 ) {
            if( false == scratchpad.binary.terrain ) {
                auto const cellcount { instance->TerrainCount() + 1 };
                for( auto i = 1; i < cellcount; ++i ) {
                    auto *submodel { instance->TerrainSquare( i - 1 ) };
                    simulation::Region->insert(
                        scene::shape_node().convert( submodel ),
                        scratchpad,
                        false );
                    submodel = submodel->ChildGet();
                    while( submodel != nullptr ) {
                        simulation::Region->insert(
                            scene::shape_node().convert( submodel ),
                            scratchpad,
                            false );
                        submodel = submodel->NextGet();
                    }
                }
            }
            delete instance;
            continue;
        }

        if( instance->Model() != nullptr ) {
            for( auto const &smokesource : instance->Model()->smoke_sources() ) {
                Particles.insert( smokesource.first, instance, smokesource.second );
            }
        }
        if( false == simulation::Instances.insert( instance ) ) {
            ErrorLog( "Bad EU7: duplicate model name \"" + instance->name() + "\"" );
        }
        scene::Groups.insert( scene::Groups.handle(), instance );
        simulation::Region->insert( instance );
        if( auto *hierarchy_node = static_cast<scene::basic_node *>( instance ) ) {
            scene::Hierarchy[ hierarchy_node->uuid.to_string() ] = hierarchy_node;
        }
    }
}

void
state_serializer::insert_eu7_pack_models(
    scene::eu7::Eu7Model const *models,
    std::size_t const count,
    scene::scratch_data &scratchpad,
    eu7_pack_apply_session const *const session ) {
    if( models == nullptr || count == 0 ) {
        return;
    }

    scene::eu7::ScopedTimer const model_timer { scene::eu7::load_stats().model_ms };
    scene::eu7::load_stats().models += count;

    std::unordered_map<std::string, scene::node_data> local_nodedata_cache;
    local_nodedata_cache.reserve( std::min( count, std::size_t { 64 } ) );
    std::unordered_map<std::string, TModel3d *> local_mesh_cache;
    local_mesh_cache.reserve( std::min( count, std::size_t { 64 } ) );

    auto &nodedata_cache {
        session != nullptr && session->nodedata_cache != nullptr ?
            *session->nodedata_cache :
            local_nodedata_cache };
    auto &mesh_cache {
        session != nullptr && session->mesh_cache != nullptr ?
            *session->mesh_cache :
            local_mesh_cache };

    for( std::size_t i { 0 }; i < count; ++i ) {
        auto const &model { models[ i ] };
        auto const &nodedata { pack_nodedata_cached( model, nodedata_cache ) };

        auto *instance { TAnimModel::acquire_pack_instance( nodedata ) };
        instance->Angles( glm::vec3(
            static_cast<float>( model.angles.x ),
            static_cast<float>( model.angles.y ),
            static_cast<float>( model.angles.z ) ) );
        if( model.scale.x != 1.0 || model.scale.y != 1.0 || model.scale.z != 1.0 ) {
            instance->Scale( glm::vec3(
                static_cast<float>( model.scale.x ),
                static_cast<float>( model.scale.y ),
                static_cast<float>( model.scale.z ) ) );
        }

        instance->location( glm::vec3(
            static_cast<float>( model.location.x ),
            static_cast<float>( model.location.y ),
            static_cast<float>( model.location.z ) ) );

        if( nodedata.range_min < 0.0 ) {
            if( false == instance->LoadEu7(
                    model.model_file,
                    model.texture_file,
                    model.light_states,
                    model.light_colors,
                    model.transition,
                    true ) ) {
                TAnimModel::release_pack_instance( instance );
                continue;
            }
            if( false == scratchpad.binary.terrain ) {
                auto const cellcount { instance->TerrainCount() + 1 };
                for( auto i = 1; i < cellcount; ++i ) {
                    auto *submodel { instance->TerrainSquare( i - 1 ) };
                    simulation::Region->insert(
                        scene::shape_node().convert( submodel ),
                        scratchpad,
                        false );
                    submodel = submodel->ChildGet();
                    while( submodel != nullptr ) {
                        simulation::Region->insert(
                            scene::shape_node().convert( submodel ),
                            scratchpad,
                            false );
                        submodel = submodel->NextGet();
                    }
                }
            }
            TAnimModel::release_pack_instance( instance );
            continue;
        }

        bool const needs_full_load { pack_model_needs_full_load( model ) };
        bool loaded { false };
        if( needs_full_load ) {
            scene::eu7::PackBenchTimer const load_timer {
                &scene::eu7::Eu7PackBench::main_load_eu7_full_ms };
            loaded = instance->LoadEu7(
                model.model_file,
                model.texture_file,
                model.light_states,
                model.light_colors,
                model.transition,
                false );
            if( loaded ) {
                scene::eu7::pack_bench_inc( &scene::eu7::Eu7PackBench::main_pack_full_loads );
            }
        }
        else {
            scene::eu7::PackBenchTimer const load_timer {
                &scene::eu7::Eu7PackBench::main_load_eu7_pack_ms };
            auto model_file { model.model_file };
            auto texture_file { model.texture_file };
            replace_slashes( model_file );
            replace_slashes( texture_file );
            TModel3d *mesh { nullptr };
            if( false == model_file.empty() && model_file != "notload" ) {
                auto const found { mesh_cache.find( model_file ) };
                if( found != mesh_cache.end() ) {
                    mesh = found->second;
                }
                else {
                    mesh = TModelsManager::GetModel( model_file, false, false );
                    mesh_cache.emplace( model_file, mesh );
                }
            }
            loaded = instance->LoadEu7PackWarm( mesh, texture_file );
            if( loaded ) {
                scene::eu7::pack_bench_inc( &scene::eu7::Eu7PackBench::main_pack_fast_loads );
            }
        }
        if( false == loaded ) {
            TAnimModel::release_pack_instance( instance );
            continue;
        }

        if( auto *const mesh { instance->Model() } ) {
            if( false == mesh->smoke_sources().empty() ) {
                for( auto const &smokesource : mesh->smoke_sources() ) {
                    Particles.insert( smokesource.first, instance, smokesource.second );
                }
            }
        }
        {
            scene::eu7::PackBenchTimer const region_timer {
                &scene::eu7::Eu7PackBench::main_region_insert_ms };
            simulation::Region->insert( instance );
            scene::eu7::pack_bench_inc( &scene::eu7::Eu7PackBench::main_region_inserts );
            scene::eu7::pack_bench_inc( &scene::eu7::Eu7PackBench::main_instances_applied );
        }
    }
}

void
state_serializer::insert_eu7_pack_models(
    std::vector<scene::eu7::Eu7Model> const &models,
    scene::scratch_data &scratchpad,
    std::size_t const offset,
    std::size_t const count,
    eu7_pack_apply_session const *const session ) {
    if( offset >= models.size() || count == 0 ) {
        return;
    }

    auto const end { std::min( offset + count, models.size() ) };
    insert_eu7_pack_models( models.data() + offset, end - offset, scratchpad, session );
}

void
state_serializer::insert_eu7_pack_models(
    std::vector<scene::eu7::Eu7Model> const &models,
    scene::scratch_data &scratchpad ) {
    insert_eu7_pack_models( models, scratchpad, 0, models.size(), nullptr );
}

std::shared_ptr<deserializer_state>
state_serializer::deserialize_begin( std::string const &Scenariofile ) {

    crashreport_add_info("scenario", Scenariofile);

    // TODO: move initialization to separate routine so we can reuse it
    SafeDelete( Region );
    Region = new scene::basic_region();

    simulation::State.init_scripting_interface();

    scene::eu7::begin_load_session();
    clear_deferred_eu7_trainsets();

    auto const resolved_scenario { scene::eu7::resolve_scenery_path( Scenariofile ) };
    bool is_pure_eu7_scenario { scene::eu7::probe_file( resolved_scenario ) };

    if( false == is_pure_eu7_scenario ) {
        auto const bake { scene::eu7::ensure_scenario_eu7( Scenariofile ) };
        if( false == bake.ok ) {
            ErrorLog( "EU7: nie udalo sie przygotowac modulu: " + bake.message );
            throw invalid_scenery_exception();
        }
        if( bake.regenerated ) {
            WriteLog( "EU7: wygenerowano modul scenariusza (" + bake.message + ")" );
        }
        is_pure_eu7_scenario = scene::eu7::probe_file( resolved_scenario );
    }

    bool const use_eu7_scenario {
        is_pure_eu7_scenario || scene::eu7::probe_baked_scenario( Scenariofile ) };
    auto const eu7_load_path {
        is_pure_eu7_scenario ?
            resolved_scenario :
            scene::eu7::resolve_scenery_path( scene::eu7::binary_path( Scenariofile ) ) };

    // Scenariusz EU7B: nie parsujemy binarki jako tekstu SCM.
    std::shared_ptr<deserializer_state> state;
    if( use_eu7_scenario ) {
        state = std::make_shared<deserializer_state>(
            std::string{}, cParser::buffer_TEXT, Global.asCurrentSceneryPath, Global.bLoadTraction );
        state->scenariofile = Scenariofile;
    }
    else {
        state = std::make_shared<deserializer_state>(
            Scenariofile, cParser::buffer_FILE, Global.asCurrentSceneryPath, Global.bLoadTraction );
    }

	state->scratchpad.name = Scenariofile;

    if( use_eu7_scenario ) {
        Global.file_binary_terrain_state = true;
        state->scratchpad.binary.terrain = true;
        state->scratchpad.binary.terrain_included = true;
        WriteLog( "EU7 scenario: " + eu7_load_path );
        if( false == scene::eu7::load_module( eu7_load_path, *this ) ) {
            throw invalid_scenery_exception();
        }
    }
    else if( ( true == Global.file_binary_terrain )
          && ( Scenariofile != "$.scn" ) ) {
        // EU7 ma pierwszenstwo przed SBT przy tym samym stemie.
        if( scene::eu7::is_scenario_terrain( Scenariofile ) ) {
            state->scratchpad.binary.terrain = true;
            Global.file_binary_terrain_state = true;
            WriteLog( "Default EU7 terrain present" );
        }
        else if( Region->is_scene( Scenariofile ) ) {
            state->scratchpad.binary.terrain = true;
            Global.file_binary_terrain_state = true;
            WriteLog( "Default SBT present" );
        }
        else {
            Global.file_binary_terrain_state = false;
            WriteLog( "Default binary terrain absent" );
        }
    }
    else {
        Global.file_binary_terrain_state = false;
        WriteLog( "Default binary terrain absent" );
    }

    scene::Groups.create();

	if( false == state->input.ok() )
		throw invalid_scenery_exception();

	populate_deserialize_functionmap( state->functionmap, state->input, state->scratchpad );

    if (!Global.prepend_scn.empty()) {
        state->input.injectString(Global.prepend_scn);
    }

	return state;
}

// continues deserialization for given context, amount limited by time, returns true if needs to be called again
bool
state_serializer::deserialize_continue(std::shared_ptr<deserializer_state> state) {
	cParser &Input = state->input;
	scene::scratch_data &Scratchpad = state->scratchpad;

    // deserialize content from the provided input
	auto timelast { std::chrono::steady_clock::now() };
    std::string token { Input.getToken<std::string>() };
    while( false == token.empty() ) {

		auto lookup = state->functionmap.find( token );
		if( lookup != state->functionmap.end() ) {
            lookup->second();
        }
        else {
            ErrorLog( "Bad scenario: unexpected token \"" + token + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() - 1 ) + ")" );
        }

		auto timenow = std::chrono::steady_clock::now();
        if( std::chrono::duration_cast<std::chrono::milliseconds>( timenow - timelast ).count() >= 200 ) {
            Application.set_progress( Input.getProgress(), Input.getFullProgress() );
			return true;
        }

        token = Input.getToken<std::string>();
    }

    if( false == Scratchpad.initialized ) {
        // manually perform scenario initialization
        deserialize_firstinit( Input, Scratchpad );
    }

    scene::Groups.close();

	scene::Groups.update_map();
	Region->create_map_geometry();

	return false;
}

void
state_serializer::populate_deserialize_functionmap(
    std::unordered_map<std::string, deserializer_state::deserializefunctionbind> &functionmap,
    cParser &input,
    scene::scratch_data &scratchpad ) {
    using deserializefunction = void( state_serializer::*)( cParser &, scene::scratch_data & );
    std::vector<std::pair<std::string, deserializefunction>> const functionlist {
        { "area",        &state_serializer::deserialize_area },
        { "isolated",    &state_serializer::deserialize_isolated },
        { "assignment",  &state_serializer::deserialize_assignment },
        { "atmo",        &state_serializer::deserialize_atmo },
        { "camera",      &state_serializer::deserialize_camera },
        { "config",      &state_serializer::deserialize_config },
        { "description", &state_serializer::deserialize_description },
        { "event",       &state_serializer::deserialize_event },
        { "lua",         &state_serializer::deserialize_lua },
        { "firstinit",   &state_serializer::deserialize_firstinit },
        { "group",       &state_serializer::deserialize_group },
        { "endgroup",    &state_serializer::deserialize_endgroup },
        { "light",       &state_serializer::deserialize_light },
        { "node",        &state_serializer::deserialize_node },
        { "origin",      &state_serializer::deserialize_origin },
        { "endorigin",   &state_serializer::deserialize_endorigin },
        { "scale",       &state_serializer::deserialize_scale },
        { "endscale",    &state_serializer::deserialize_endscale },
        { "rotate",      &state_serializer::deserialize_rotate },
        { "sky",         &state_serializer::deserialize_sky },
        { "test",        &state_serializer::deserialize_test },
        { "time",        &state_serializer::deserialize_time },
        { "trainset",    &state_serializer::deserialize_trainset },
        { "terrain",     &state_serializer::deserialize_terrain },
        { "endtrainset", &state_serializer::deserialize_endtrainset } };

    functionmap.clear();
    for( auto const &function : functionlist ) {
        functionmap.emplace(
            function.first,
            std::bind( function.second, this, std::ref( input ), std::ref( scratchpad ) ) );
    }
}

void
state_serializer::deserialize_parser_tokens(
    cParser &input,
    scene::scratch_data &scratchpad,
    std::string const &source_name ) {
    std::unordered_map<std::string, deserializer_state::deserializefunctionbind> functionmap;
    populate_deserialize_functionmap( functionmap, input, scratchpad );

    std::string token { input.getToken<std::string>() };
    while( false == token.empty() ) {
        auto const lookup { functionmap.find( token ) };
        if( lookup != functionmap.end() ) {
            lookup->second();
        }
        else {
            ErrorLog(
                "Bad EU7 module: unexpected token \"" + token + "\" in \"" + source_name +
                "\" (line " + std::to_string( input.Line() - 1 ) + ")" );
        }
        token = input.getToken<std::string>();
    }
}

void
state_serializer::deserialize_module_text(
    std::string const &text,
    std::string const &source_name,
    scene::scratch_data &scratchpad ) {
    // mPath musi byc katalogiem scenerii (jak przy buffer_FILE), nie pelna sciezka .eu7 —
    // inaczej include "td/foo.scm" skleja sie w "scenery/td.eu7td/foo.scm".
    cParser input( text, cParser::buffer_TEXT, Global.asCurrentSceneryPath, Global.bLoadTraction );
    // Dzieci INCL sa juz zaladowane w load_module_recursive — bez ponownego include→EU7.
    input.expandIncludes = false;

    deserialize_parser_tokens( input, scratchpad, source_name );
}

void
state_serializer::deserialize_include_file(
    std::string const &include_reference,
    std::string const &current_relative_file,
    std::vector<std::string> const &parameters,
    scene::scratch_data &scratchpad ) {
    cParser input(
        include_reference,
        cParser::buffer_FILE,
        Global.asCurrentSceneryPath,
        Global.bLoadTraction,
        parameters );
    input.expandIncludes = true;

    deserialize_parser_tokens( input, scratchpad, current_relative_file + " -> " + include_reference );
}

void
state_serializer::apply_eu7_models(
    std::vector<scene::eu7::Eu7Model> const &models,
    scene::scratch_data &scratchpad ) {
    eu7_transform_state transform_state;
    insert_eu7_models( models, scratchpad, transform_state );
}

void
state_serializer::apply_eu7_pack_models(
    std::vector<scene::eu7::Eu7Model> const &models,
    scene::scratch_data &scratchpad ) {
    preload_unique_pack_meshes( models, 0, models.size() );
    insert_eu7_pack_models( models, scratchpad );
}

void
state_serializer::apply_eu7_pack_models(
    std::vector<scene::eu7::Eu7Model> const &models,
    scene::scratch_data &scratchpad,
    std::size_t const offset,
    std::size_t const count,
    eu7_pack_apply_session const *const session ) {
    if( session == nullptr && offset < models.size() && count > 0 ) {
        auto const end { std::min( offset + count, models.size() ) };
        preload_unique_pack_meshes( models, offset, end );
    }
    if( offset >= models.size() || count == 0 ) {
        return;
    }
    auto const end { std::min( offset + count, models.size() ) };
    insert_eu7_pack_models( models.data() + offset, end - offset, scratchpad, session );
}

void
state_serializer::apply_eu7_pack_models(
    scene::eu7::Eu7Model const *models,
    std::size_t const count,
    scene::scratch_data &scratchpad,
    eu7_pack_apply_session const *const session ) {
    insert_eu7_pack_models( models, count, scratchpad, session );
}

void
state_serializer::apply_eu7_pack_models(
    scene::eu7::Eu7Model const *models,
    std::size_t const offset,
    std::size_t const count,
    scene::scratch_data &scratchpad,
    eu7_pack_apply_session const *const session ) {
    if( models == nullptr || count == 0 ) {
        return;
    }
    insert_eu7_pack_models( models + offset, count, scratchpad, session );
}

void
state_serializer::apply_eu7_scene(
    scene::eu7::Eu7Scene const &scene,
    scene::scratch_data &scratchpad ) {
    eu7_transform_state transform_state;

    auto const scratch_offset { [&]() -> glm::dvec3 {
        return (
            scratchpad.location.offset.empty() ?
                glm::dvec3{} :
                scratchpad.location.offset.top() );
    } };

    auto const apply_node_transform { [&]( scene::eu7::Eu7BasicNode const &node ) {
        sync_scratch_transform( scratchpad, transform_state.state, node.transform );
    } };

    {
        scene::eu7::ScopedTimer const timer { scene::eu7::load_stats().trak_ms };
        scene::eu7::load_stats().tracks += scene.tracks.size();
        for( auto const &track : scene.tracks ) {
            apply_node_transform( track.node );
            auto const nodedata { node_data_from_eu7( track.node ) };
            auto *path { new TTrack( nodedata ) };
            path->LoadFromEu7( track );
            if( false == simulation::Paths.insert( path ) ) {
                ErrorLog( "Bad EU7: duplicate track name \"" + path->name() + "\"" );
            }
            scene::Groups.insert( scene::Groups.handle(), path );
            simulation::Region->insert_and_register( path );
        }
    }

    {
        scene::eu7::ScopedTimer const timer { scene::eu7::load_stats().trac_ms };
        scene::eu7::load_stats().traction += scene.traction.size();
        for( auto const &traction : scene.traction ) {
        if( false == Global.bLoadTraction ) {
            continue;
        }
        apply_node_transform( traction.node );
        auto const nodedata { node_data_from_eu7( traction.node ) };
        auto *piece { new TTraction( nodedata ) };
        auto const origin { scratch_offset() };
        auto const local_p1 { scene::eu7::subtract_origin_offset( traction.wire_p1, traction.node.transform ) + origin };
        auto const local_p2 { scene::eu7::subtract_origin_offset( traction.wire_p2, traction.node.transform ) + origin };
        auto const local_p3 { scene::eu7::subtract_origin_offset( traction.wire_p3, traction.node.transform ) + origin };
        auto const local_p4 { scene::eu7::subtract_origin_offset( traction.wire_p4, traction.node.transform ) + origin };

        piece->asPowerSupplyName = traction.power_supply_name;
        piece->NominalVoltage = traction.nominal_voltage;
        piece->MaxCurrent = traction.max_current;
        piece->fResistivity = (
            traction.resistivity_legacy != 0.0 ?
                static_cast<float>( traction.resistivity_legacy ) :
                traction.resistivity_ohm_per_m / 0.001f );
        if( piece->fResistivity == 0.01f ) {
            piece->fResistivity = 0.075f;
        }
        piece->fResistivity *= 0.001f;

        auto const material { (
            traction.material_raw.empty() ?
                ( traction.material == scene::eu7::Eu7TractionWireMaterial::Aluminium ? "al" :
                  traction.material == scene::eu7::Eu7TractionWireMaterial::None ? "none" : "cu" ) :
                traction.material_raw ) };
             if( material == "none" ) { piece->Material = 0; }
        else if( material == "al" )   { piece->Material = 2; }
        else                          { piece->Material = 1; }

        piece->WireThickness = traction.wire_thickness;
        piece->DamageFlag = traction.damage_flag;
        piece->pPoint1 = local_p1;
        piece->pPoint2 = local_p2;
        piece->pPoint3 = local_p3;
        piece->pPoint4 = local_p4;
        piece->fHeightDifference = ( local_p3.y - local_p1.y + local_p4.y - local_p2.y ) * 0.5 - traction.min_height;
        piece->iNumSections = (
            traction.segment_length ?
                static_cast<int>( glm::length( local_p1 - local_p2 ) / traction.segment_length ) :
                0 );
        piece->Wires = traction.wire_count;
        piece->WireOffset = traction.wire_offset;
        piece->m_visible = traction.node.visible;
        if( traction.parallel_name ) {
            piece->asParallel = *traction.parallel_name;
        }
        piece->Init();
        piece->location( glm::mix( local_p2, local_p1, 0.5 ) );

        if( false == simulation::Traction.insert( piece ) ) {
            ErrorLog( "Bad EU7: duplicate traction name \"" + piece->name() + "\"" );
        }
        scene::Groups.insert( scene::Groups.handle(), piece );
        simulation::Region->insert_and_register( piece );
        }
    }

    {
        scene::eu7::ScopedTimer const timer { scene::eu7::load_stats().power_ms };
        scene::eu7::load_stats().power_sources += scene.power_sources.size();
        for( auto const &source : scene.power_sources ) {
        if( false == Global.bLoadTraction ) {
            continue;
        }
        apply_node_transform( source.node );
        auto const nodedata { node_data_from_eu7( source.node ) };
        auto const local_position {
            scene::eu7::inverse_transform_point( source.position, source.node.transform ) };
        auto const internal_res { (
            source.internal_resistance_legacy != 0.2 ?
                source.internal_resistance_legacy :
                static_cast<double>( source.internal_resistance ) ) };

        std::ostringstream power_body;
        power_body
            << local_position.x << ' ' << local_position.y << ' ' << local_position.z << ' '
            << source.nominal_voltage << ' ' << source.voltage_frequency << ' '
            << internal_res << ' ' << source.max_output_current << ' '
            << source.fast_fuse_timeout << ' ' << source.fast_fuse_repetition << ' '
            << source.slow_fuse_timeout << ' ';
        if( source.modifier == scene::eu7::Eu7PowerSourceModifier::Recuperation ) {
            power_body << "recuperation ";
        }
        else if( source.modifier == scene::eu7::Eu7PowerSourceModifier::Section ) {
            power_body << "section ";
        }
        power_body << "end";

        cParser power_parser( power_body.str(), cParser::buffer_TEXT, "", false );
        auto *powersource { deserialize_tractionpowersource( power_parser, scratchpad, nodedata ) };
        if( powersource == nullptr ) {
            continue;
        }

        if( false == simulation::Powergrid.insert( powersource ) ) {
            ErrorLog( "Bad EU7: duplicate power source name \"" + powersource->name() + "\"" );
        }
        }
    }

    if( false == scene.models.empty() ) {
        if( scene::eu7::pack_scenery_active() ) {
            scene::eu7::load_stats().pack_skipped_models += scene.models.size();
        }
        else {
            insert_eu7_models( scene.models, scratchpad, transform_state );
        }
    }

    {
        scene::eu7::ScopedTimer const timer { scene::eu7::load_stats().memcell_ms };
        scene::eu7::load_stats().memcells += scene.memcells.size();
        for( auto const &cell : scene.memcells ) {
        apply_node_transform( cell.node );
        auto const nodedata { node_data_from_eu7( cell.node ) };
        auto const local_position {
            scene::eu7::inverse_transform_point( cell.node.area.center, cell.node.transform ) };

        std::ostringstream memcell_body;
        memcell_body
            << local_position.x << ' ' << local_position.y << ' ' << local_position.z << ' '
            << cell.text << ' ' << cell.value1 << ' ' << cell.value2 << ' '
            << ( cell.track_name ? *cell.track_name : "none" )
            << " endmemcell";

        cParser memcell_parser( memcell_body.str(), cParser::buffer_TEXT, "", false );
        auto *memorycell { deserialize_memorycell( memcell_parser, scratchpad, nodedata ) };
        if( memorycell == nullptr ) {
            continue;
        }

        if( false == simulation::Memory.insert( memorycell ) ) {
            ErrorLog( "Bad EU7: duplicate memcell name \"" + memorycell->name() + "\"" );
        }
        scene::Groups.insert( scene::Groups.handle(), memorycell );
        simulation::Region->insert( memorycell );
        }
    }

    {
        scene::eu7::ScopedTimer const timer { scene::eu7::load_stats().launcher_ms };
        scene::eu7::load_stats().launchers += scene.event_launchers.size();
        for( auto const &launcher : scene.event_launchers ) {
        apply_node_transform( launcher.node );
        auto const nodedata { node_data_from_eu7( launcher.node ) };
        auto const local_location {
            scene::eu7::inverse_transform_point( launcher.location, launcher.node.transform ) };

        std::string const time_token { (
            launcher.launch_hour != 0 || launcher.launch_minute != 0 ) ?
                std::to_string( launcher.launch_hour * 100 + launcher.launch_minute ) :
            ( launcher.delta_time != 0.0 ) ?
                std::to_string( -launcher.delta_time ) :
                "0" };

        std::ostringstream launcher_body;
        launcher_body
            << std::sqrt( launcher.radius_squared ) << ' '
            << launcher.activation_key_raw << ' '
            << time_token << ' '
            << launcher.event1_name << ' ';
        if( false == launcher.event2_name.empty() && launcher.event2_name != "none" &&
            launcher.event2_name != "endeventlauncher" ) {
            launcher_body << launcher.event2_name << ' ';
        }
        if( launcher.condition ) {
            launcher_body
                << "condition "
                << launcher.condition->memcell_name << ' '
                << launcher.condition->compare_text << ' ';
            if( launcher.condition->check_mask & 2 ) {
                launcher_body << launcher.condition->compare_value1 << ' ';
            }
            else {
                launcher_body << "* ";
            }
            if( launcher.condition->check_mask & 4 ) {
                launcher_body << launcher.condition->compare_value2 << ' ';
            }
            else {
                launcher_body << "* ";
            }
        }
        if( launcher.train_triggered ) {
            launcher_body << "traintriggered ";
        }
        launcher_body << "endeventlauncher";

        cParser launcher_parser( launcher_body.str(), cParser::buffer_TEXT, "", false );
        auto *eventlauncher { new TEventLauncher( nodedata ) };
        eventlauncher->Load( &launcher_parser );
        eventlauncher->location( transform( local_location, scratchpad ) );

        if( false == simulation::Events.insert( eventlauncher ) ) {
            ErrorLog( "Bad EU7: duplicate event launcher name \"" + eventlauncher->name() + "\"" );
            continue;
        }
        if( true == eventlauncher->IsGlobal() ) {
            simulation::Events.queue( eventlauncher );
        }
        else {
            scene::Groups.insert( scene::Groups.handle(), eventlauncher );
            if( false == eventlauncher->IsRadioActivated() ) {
                simulation::Region->insert( eventlauncher );
            }
        }
        }
    }

    std::vector<bool> used_in_trainset( scene.dynamics.size(), false );
    for( auto const &trainset : scene.trainsets ) {
        for( auto const index : trainset.vehicle_indices ) {
            if( index < used_in_trainset.size() ) {
                used_in_trainset[ index ] = true;
            }
        }
    }

    auto const apply_dynamic { [&]( scene::eu7::Eu7Dynamic const &vehicle, bool const in_trainset ) {
        apply_node_transform( vehicle.node );
        auto const nodedata { node_data_from_eu7( vehicle.node ) };
        if( false == in_trainset ) {
            scratchpad.trainset = scene::scratch_data::trainset_data();
        }

        auto datafolder { vehicle.data_folder };
        auto skinfile { vehicle.skin_file };
        auto mmdfile { vehicle.mmd_file };
        replace_slashes( datafolder );
        replace_slashes( skinfile );
        replace_slashes( mmdfile );

        auto const pathname { (
            in_trainset ?
                scratchpad.trainset.track :
                vehicle.track_name ) };
        auto const offset { vehicle.offset };
        auto const drivertype { vehicle.driver_type };
        auto const couplingdata { (
            in_trainset ?
                ( vehicle.coupling_raw.empty() ? std::to_string( vehicle.coupling ) : vehicle.coupling_raw ) :
                "3" ) };
        auto const velocity { (
            in_trainset ?
                scratchpad.trainset.velocity :
                vehicle.velocity ) };

        auto const couplingdatawithparams { couplingdata.find( '.' ) };
        auto coupling { (
            couplingdatawithparams != std::string::npos ?
                std::atoi( couplingdata.substr( 0, couplingdatawithparams ).c_str() ) :
                std::atoi( couplingdata.c_str() ) ) };
        if( coupling < 0 ) {
            coupling = ( -coupling ) | coupling::permanent;
        }
        if( ( offset != -1.0 ) && ( std::abs( offset ) > 0.5 ) ) {
            coupling = coupling::faux;
        }
        auto const params { (
            couplingdatawithparams != std::string::npos ?
                couplingdata.substr( couplingdatawithparams + 1 ) :
                "" ) };

        auto loadcount { vehicle.load_count };
        auto loadtype { vehicle.load_type };

        auto *path { simulation::Paths.find( pathname ) };
        if( path == nullptr ) {
            ErrorLog( "Bad EU7: vehicle \"" + nodedata.name + "\" on missing track \"" + pathname + "\"" );
            return;
        }

        if( ( true == scratchpad.trainset.vehicles.empty() )
         && ( false == path->m_events0.empty() )
         && ( std::abs( velocity ) <= 1.f )
         && ( scratchpad.trainset.offset >= 0.0 )
         && ( scratchpad.trainset.offset < 8.0 ) ) {
            scratchpad.trainset.offset = 8.0f;
        }

        auto *dyn { new TDynamicObject() };
        auto const length { dyn->Init(
            nodedata.name,
            datafolder, skinfile, mmdfile,
            path,
            ( offset == -1.0 ?
                scratchpad.trainset.offset :
                scratchpad.trainset.offset - static_cast<float>( offset ) ),
            drivertype,
            velocity,
            scratchpad.trainset.name,
            loadcount, loadtype,
            ( offset == -1.0 ),
            params ) };

        if( length != 0.0 ) {
            scratchpad.trainset.offset -= static_cast<float>( length );
            if( ( coupling != 0 )
             && ( dyn->MoverParameters->Couplers[ ( offset == -1.0 ? end::front : end::rear ) ].AllowedFlag & coupling::permanent ) ) {
                coupling |= coupling::permanent;
            }
            if( in_trainset ) {
                scratchpad.trainset.vehicles.emplace_back( dyn );
                scratchpad.trainset.couplings.emplace_back( coupling );
            }
        }
        else {
            if( dyn->MyTrack != nullptr ) {
                dyn->MyTrack->RemoveDynamicObject( dyn );
            }
            delete dyn;
            return;
        }

        if( vehicle.destination ) {
            dyn->asDestination = *vehicle.destination;
        }

        if( dyn->mdModel != nullptr ) {
            for( auto const &smokesource : dyn->mdModel->smoke_sources() ) {
                Particles.insert( smokesource.first, dyn, smokesource.second );
            }
        }

        if( false == in_trainset ) {
            if( false == simulation::Vehicles.insert( dyn ) ) {
                ErrorLog( "Bad EU7: duplicate vehicle name \"" + dyn->name() + "\"" );
            }
            if( ( dyn->MoverParameters->CategoryFlag == 1 )
             && ( ( ( dyn->LightList( end::front ) & ( light::headlight_left | light::headlight_right | light::headlight_upper ) ) != 0 )
               || ( ( dyn->LightList( end::rear ) & ( light::headlight_left | light::headlight_right | light::headlight_upper ) ) != 0 ) ) ) {
                simulation::Lights.insert( dyn );
            }
        }
    } };

    {
        scene::eu7::ScopedTimer const timer { scene::eu7::load_stats().dynamic_ms };
        scene::eu7::load_stats().dynamics += scene.dynamics.size();
        for( std::size_t i { 0 }; i < scene.dynamics.size(); ++i ) {
            if( false == used_in_trainset[ i ] ) {
                apply_dynamic( scene.dynamics[ i ], false );
            }
        }
    }

    {
        scene::eu7::ScopedTimer const timer { scene::eu7::load_stats().sound_ms };
        scene::eu7::load_stats().sounds += scene.sounds.size();
        for( auto const &sound : scene.sounds ) {
        apply_node_transform( sound.node );
        auto const nodedata { node_data_from_eu7( sound.node ) };
        auto const location { transform(
            scene::eu7::inverse_transform_point( sound.location, sound.node.transform ),
            scratchpad ) };
        auto *snd { new sound_source( sound_placement::external, static_cast<float>( nodedata.range_max ) ) };
        snd->offset( location );
        snd->name( nodedata.name );
        snd->deserialize( sound.wav_file, sound_type::single );
        if( false == simulation::Sounds.insert( snd ) ) {
            ErrorLog( "Bad EU7: duplicate sound name \"" + snd->name() + "\"" );
        }
        simulation::Region->insert( snd );
        }
    }

    {
        scene::eu7::ScopedTimer const timer { scene::eu7::load_stats().event_ms };
        scene::eu7::load_stats().events += scene.events.size();
        for( auto const &event : scene.events ) {
        auto *ev { make_event_from_eu7( event ) };
        if( ev == nullptr ) {
            continue;
        }

        std::ostringstream body;
        body << event.delay << ' ' << join_event_targets( event.targets ) << ' ';
        for( auto const &[key, value] : event.payload ) {
            if( false == key.empty() ) {
                body << key << ' ';
            }
            if( false == value.empty() ) {
                body << value << ' ';
            }
        }
        if( event.delay_random != 0.0 ) {
            body << "randomdelay " << event.delay_random << ' ';
        }
        if( event.delay_departure != 0.0 ) {
            body << "departuredelay " << event.delay_departure << ' ';
        }
        if( event.passive ) {
            body << "passive ";
        }
        body << "endevent";

        cParser parser( body.str(), cParser::buffer_TEXT, "", false );
        ev->deserialize( parser, scratchpad );

        if( true == simulation::Events.insert( ev ) ) {
            scene::Groups.insert( scene::Groups.handle(), ev );
        }
        else {
            delete ev;
        }
        }
    }

    {
        scene::eu7::ScopedTimer const timer { scene::eu7::load_stats().first_init_ms };
        for( std::uint32_t i { 0 }; i < scene.first_init_count; ++i ) {
        if( true == scratchpad.initialized ) {
            continue;
        }
        if( true == scratchpad.binary.terrain ) {
            if( false == scratchpad.binary.terrain_included ) {
                if( false == scene::eu7::try_load_scenario_terrain( *Region, scratchpad.name ) ) {
                    Region->deserialize( scratchpad.name );
                }
            }
        }
        simulation::Paths.InitTracks();
        simulation::Traction.InitTraction();
        simulation::Events.InitEvents();
        simulation::Events.InitLaunchers();
        simulation::Memory.InitCells();
        if( false == scratchpad.time_initialized ) {
            init_time();
        }
        scratchpad.initialized = true;
        }
    }

    cParser dummy_parser( "", cParser::buffer_TEXT, "", false );

    auto const load_eu7_trainset { [&](
        scene::eu7::Eu7Trainset const &trainset,
        std::vector<scene::eu7::Eu7Dynamic> const &vehicles ) {
        if( true == scratchpad.trainset.is_open ) {
            deserialize_endtrainset( dummy_parser, scratchpad );
            ErrorLog( "Bad EU7: nested trainset definitions" );
        }

        scratchpad.trainset = scene::scratch_data::trainset_data();
        scratchpad.trainset.is_open = true;
        scratchpad.trainset.name = trainset.name;
        scratchpad.trainset.track = trainset.track;
        scratchpad.trainset.offset = trainset.offset;
        scratchpad.trainset.velocity = trainset.velocity;
        scratchpad.trainset.assignment = trainset.assignment;

        std::size_t vehicle_slot { 0 };
        for( auto const &vehicle_source : vehicles ) {
            auto vehicle { vehicle_source };
            if( vehicle_slot < trainset.couplings.size() ) {
                vehicle.coupling_raw = std::to_string( trainset.couplings[ vehicle_slot ] );
            }
            apply_dynamic( vehicle, true );
            ++vehicle_slot;
        }

        deserialize_endtrainset( dummy_parser, scratchpad );
    } };

    {
        scene::eu7::ScopedTimer const timer { scene::eu7::load_stats().trainset_ms };
        std::size_t loaded_now { 0 };
        for( auto const &trainset : scene.trainsets ) {
            if( eu7_should_load_trainset_now( trainset, scene ) ) {
                std::vector<scene::eu7::Eu7Dynamic> vehicles;
                vehicles.reserve( trainset.vehicle_indices.size() );
                for( auto const index : trainset.vehicle_indices ) {
                    if( index < scene.dynamics.size() ) {
                        vehicles.push_back( scene.dynamics[ index ] );
                    }
                }
                load_eu7_trainset( trainset, vehicles );
                ++loaded_now;
            }
            else {
                eu7_queue_deferred_trainset( trainset, scene );
            }
        }
        scene::eu7::load_stats().trainsets += loaded_now;
        if( false == g_deferred_eu7_trainsets.empty() ) {
            WriteLog(
                "EU7: odlozono " + std::to_string( g_deferred_eu7_trainsets.size() ) +
                " skladow poza " + std::to_string( static_cast<int>( kDeferTrainsetHorizDistM ) ) +
                "m (ladowanie w tle)" );
        }
    }
}

void
state_serializer::drain_deferred_eu7_trainsets( double const max_ms ) {
    if( g_deferred_eu7_trainsets.empty() ) {
        return;
    }

    auto const deadline {
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>( max_ms ) ) };

    cParser dummy_parser( "", cParser::buffer_TEXT, "", false );
    scene::scratch_data scratchpad;

    while(
        false == g_deferred_eu7_trainsets.empty() &&
        std::chrono::steady_clock::now() < deadline ) {
        auto job { std::move( g_deferred_eu7_trainsets.front() ) };
        g_deferred_eu7_trainsets.pop_front();

        scene::eu7::ScopedTimer const timer { scene::eu7::load_stats().trainset_ms };
        ++scene::eu7::load_stats().trainsets;

        if( true == scratchpad.trainset.is_open ) {
            deserialize_endtrainset( dummy_parser, scratchpad );
            scratchpad.trainset.is_open = false;
        }

        scratchpad.trainset = scene::scratch_data::trainset_data();
        scratchpad.trainset.is_open = true;
        scratchpad.trainset.name = job.trainset.name;
        scratchpad.trainset.track = job.trainset.track;
        scratchpad.trainset.offset = job.trainset.offset;
        scratchpad.trainset.velocity = job.trainset.velocity;
        scratchpad.trainset.assignment = job.trainset.assignment;

        std::size_t vehicle_slot { 0 };
        for( auto const &vehicle_source : job.vehicles ) {
            auto vehicle { vehicle_source };
            if( vehicle_slot < job.trainset.couplings.size() ) {
                vehicle.coupling_raw = std::to_string( job.trainset.couplings[ vehicle_slot ] );
            }

            auto datafolder { vehicle.data_folder };
            auto skinfile { vehicle.skin_file };
            auto mmdfile { vehicle.mmd_file };
            replace_slashes( datafolder );
            replace_slashes( skinfile );
            replace_slashes( mmdfile );

            auto const pathname { scratchpad.trainset.track };
            auto const offset { vehicle.offset };
            auto const drivertype { vehicle.driver_type };
            auto const couplingdata { (
                vehicle.coupling_raw.empty() ?
                    std::to_string( vehicle.coupling ) :
                    vehicle.coupling_raw ) };
            auto const velocity { scratchpad.trainset.velocity };

            auto const couplingdatawithparams { couplingdata.find( '.' ) };
            auto coupling { (
                couplingdatawithparams != std::string::npos ?
                    std::atoi( couplingdata.substr( 0, couplingdatawithparams ).c_str() ) :
                    std::atoi( couplingdata.c_str() ) ) };
            if( coupling < 0 ) {
                coupling = ( -coupling ) | coupling::permanent;
            }
            if( ( offset != -1.0 ) && ( std::abs( offset ) > 0.5 ) ) {
                coupling = coupling::faux;
            }
            auto const params { (
                couplingdatawithparams != std::string::npos ?
                    couplingdata.substr( couplingdatawithparams + 1 ) :
                    "" ) };

            auto loadcount { vehicle.load_count };
            auto loadtype { vehicle.load_type };

            auto *path { simulation::Paths.find( pathname ) };
            if( path == nullptr ) {
                ++vehicle_slot;
                continue;
            }

            if(
                scratchpad.trainset.vehicles.empty() &&
                false == path->m_events0.empty() &&
                std::abs( velocity ) <= 1.f &&
                scratchpad.trainset.offset >= 0.0 &&
                scratchpad.trainset.offset < 8.0 ) {
                scratchpad.trainset.offset = 8.0f;
            }

            auto const nodedata { node_data_from_eu7( vehicle.node ) };
            auto *dyn { new TDynamicObject() };
            auto const length { dyn->Init(
                nodedata.name,
                datafolder, skinfile, mmdfile,
                path,
                ( offset == -1.0 ?
                    scratchpad.trainset.offset :
                    scratchpad.trainset.offset - static_cast<float>( offset ) ),
                drivertype,
                velocity,
                scratchpad.trainset.name,
                loadcount, loadtype,
                ( offset == -1.0 ),
                params ) };

            if( length != 0.0 ) {
                scratchpad.trainset.offset -= static_cast<float>( length );
                if(
                    ( coupling != 0 ) &&
                    ( dyn->MoverParameters->Couplers[ ( offset == -1.0 ? end::front : end::rear ) ].AllowedFlag &
                      coupling::permanent ) ) {
                    coupling |= coupling::permanent;
                }
                scratchpad.trainset.vehicles.emplace_back( dyn );
                scratchpad.trainset.couplings.emplace_back( coupling );
            }
            else {
                if( dyn->MyTrack != nullptr ) {
                    dyn->MyTrack->RemoveDynamicObject( dyn );
                }
                delete dyn;
            }

            if( vehicle.destination ) {
                if( false == scratchpad.trainset.vehicles.empty() ) {
                    scratchpad.trainset.vehicles.back()->asDestination = *vehicle.destination;
                }
            }

            if(
                false == scratchpad.trainset.vehicles.empty() &&
                scratchpad.trainset.vehicles.back()->mdModel != nullptr ) {
                for( auto const &smokesource : scratchpad.trainset.vehicles.back()->mdModel->smoke_sources() ) {
                    Particles.insert(
                        smokesource.first,
                        scratchpad.trainset.vehicles.back(),
                        smokesource.second );
                }
            }

            ++vehicle_slot;
        }

        deserialize_endtrainset( dummy_parser, scratchpad );
    }
}

void
state_serializer::deserialize_isolated( cParser &Input, scene::scratch_data &Scratchpad ) {
    // first parameter specifies name of parent piece...
    auto token { Input.getToken<std::string>() };
    auto *groupowner { TIsolated::Find( token ) };
    // ...followed by list of its tracks
    while( ( false == ( token = Input.getToken<std::string>() ).empty() )
        && ( token != "endisolated" ) ) {
        auto *track { simulation::Paths.find( token ) };
        if( track != nullptr )
            track->AddIsolated( groupowner );
        else
            ErrorLog( "Bad scenario: track \"" + token + "\" not found" );
    }
}

void
state_serializer::deserialize_area( cParser &Input, scene::scratch_data &Scratchpad ) {
    // first parameter specifies name of parent piece...
    auto token { Input.getToken<std::string>() };
    auto *groupowner { TIsolated::Find( token ) };
    // ...followed by list of its children
    while( ( false == ( token = Input.getToken<std::string>() ).empty() )
        && ( token != "endarea" ) ) {
        // bind the children with their parent
        auto *isolated { TIsolated::Find( token ) };
        isolated->parent( groupowner );
    }
}

void
state_serializer::deserialize_assignment( cParser &Input, scene::scratch_data &Scratchpad ) {

    std::string token { Input.getToken<std::string>() };
    while( ( false == token.empty() )
        && ( token != "endassignment" ) ) {
        // assignment is expected to come as string pairs: language id and the actual assignment enclosed in quotes to form a single token
        auto assignment{ Input.getToken<std::string>() };
        win1250_to_ascii( assignment );
        Scratchpad.trainset.assignment.emplace( token, assignment );
        token = Input.getToken<std::string>();
    }
}

void
state_serializer::deserialize_atmo( cParser &Input, scene::scratch_data &Scratchpad ) {

    // NOTE: parameter system needs some decent replacement, but not worth the effort if we're moving to built-in editor
    // atmosphere color; legacy parameter, no longer used
    Input.getTokens( 3 );
    // fog range
    {
        double fograngestart, fograngeend;
        Input.getTokens( 2 );
        Input
            >> fograngestart
            >> fograngeend;

        if( Global.fFogEnd != 0.0 ) {
            // fog colour; optional legacy parameter, no longer used
            Input.getTokens( 3 );
        }

        Global.fFogEnd =
            std::clamp(
                Random( std::min( fograngestart, fograngeend ), std::max( fograngestart, fograngeend ) ),
                10.0, 25000.0 );
    }

    std::string token { Input.getToken<std::string>() };
    if( token != "endatmo" ) {
        // optional overcast parameter
        Global.Overcast = std::stof( token );
        if( Global.Overcast < 0.f ) {
            // negative overcast means random value in range 0-abs(specified range)
            Global.Overcast =
                Random(
                    std::clamp(
                        std::abs( Global.Overcast ),
                        0.f, 2.f ) );
        }
        // overcast drives weather so do a calculation here
        // NOTE: ugly, clean it up when we're done with world refactoring
        simulation::Environment.compute_weather();
    }
    while( ( false == token.empty() )
        && ( token != "endatmo" ) ) {
        // anything else left in the section has no defined meaning
        token = Input.getToken<std::string>();
    }
}

void
state_serializer::deserialize_camera( cParser &Input, scene::scratch_data &Scratchpad ) {

    glm::dvec3 xyz, abc;
    int i = -1, into = -1; // do której definicji kamery wstawić
    std::string token;
    do { // opcjonalna siódma liczba określa numer kamery, a kiedyś były tylko 3
        Input.getTokens();
        Input >> token;
        switch( ++i ) { // kiedyś camera miało tylko 3 współrzędne
            case 0: { xyz.x = atof( token.c_str() ); break; }
            case 1: { xyz.y = atof( token.c_str() ); break; }
            case 2: { xyz.z = atof( token.c_str() ); break; }
            case 3: { abc.x = atof( token.c_str() ); break; }
            case 4: { abc.y = atof( token.c_str() ); break; }
            case 5: { abc.z = atof( token.c_str() ); break; }
            case 6: { into = atoi( token.c_str() ); break; } // takie sobie, bo można wpisać -1
            default: { break; }
        }
    } while( token.compare( "endcamera" ) != 0 );
    if( into < 0 )
        into = ++Global.iCameraLast;
    if( into < 10 ) { // przepisanie do odpowiedniego miejsca w tabelce
        Global.FreeCameraInit[ into ] = xyz;
        Global.FreeCameraInitAngle[ into ] =
            glm::dvec3(
                glm::radians( abc.x ),
                glm::radians( abc.y ),
                glm::radians( abc.z ) );
        Global.iCameraLast = into; // numer ostatniej
    }
/*
    // cleaned up version of the above.
    // NOTE: no longer supports legacy mode where some parameters were optional
    Input.getTokens( 7 );
    glm::vec3
        position,
        rotation;
    int index;
    Input
        >> position.x
        >> position.y
        >> position.z
        >> rotation.x
        >> rotation.y
        >> rotation.z
        >> index;

    skip_until( Input, "endcamera" );

    // TODO: finish this
*/
}

void
state_serializer::deserialize_config( cParser &Input, scene::scratch_data &Scratchpad ) {

    // config parameters (re)definition
    Global.ConfigParse( Input );
}

void
state_serializer::deserialize_description( cParser &Input, scene::scratch_data &Scratchpad ) {

    // legacy section, never really used;
    skip_until( Input, "enddescription" );
}

void
state_serializer::deserialize_event( cParser &Input, scene::scratch_data &Scratchpad ) {

    // TODO: refactor event class and its de/serialization. do offset and rotation after deserialization is done
    auto *event = make_event( Input, Scratchpad );
    if( event == nullptr ) {
        // something went wrong at initial stage, move on
        skip_until( Input, "endevent" );
        return;
    }

    event->deserialize( Input, Scratchpad );

    if( true == simulation::Events.insert( event ) ) {
        scene::Groups.insert( scene::Groups.handle(), event );
    }
    else {
        delete event;
    }
}

void state_serializer::deserialize_lua( cParser &Input, scene::scratch_data &Scratchpad )
{
       Input.getTokens(1, false);
       std::string file;
       Input >> file;
#ifdef WITH_LUA
       simulation::Lua.interpret(Global.asCurrentSceneryPath + file);
#else
       ErrorLog(file + ": lua scripts not supported in this build.");
#endif
}

void
state_serializer::deserialize_firstinit( cParser &Input, scene::scratch_data &Scratchpad ) {

    if( true == Scratchpad.initialized ) { return; }

    if( true == Scratchpad.binary.terrain ) {
        // at this stage it should be safe to import terrain from the binary scene file
        // TBD: postpone loading furter and only load required blocks during the simulation?
		if (false == Scratchpad.binary.terrain_included)
		{
            if( false == scene::eu7::try_load_scenario_terrain( *Region, Scratchpad.name ) ) {
                Region->deserialize( Scratchpad.name );
            }
		}
			
    }

    simulation::Paths.InitTracks();
    simulation::Traction.InitTraction();
    simulation::Events.InitEvents();
    simulation::Events.InitLaunchers();
    simulation::Memory.InitCells();

	if (!Scratchpad.time_initialized)
		init_time();

    Scratchpad.initialized = true;
}

void state_serializer::init_time() {
	auto &time = simulation::Time.data();
	if( true == Global.ScenarioTimeCurrent ) {
		// calculate time shift required to match scenario time with local clock
		auto const *localtime = std::gmtime( &Global.starting_timestamp );
		Global.ScenarioTimeOffset = ( ( localtime->tm_hour * 60 + localtime->tm_min ) - ( time.wHour * 60 + time.wMinute ) ) / 60.f;
	}
	else if( false == std::isnan( Global.ScenarioTimeOverride ) ) {
		// scenario time override takes precedence over scenario time offset
		Global.ScenarioTimeOffset = ( ( Global.ScenarioTimeOverride * 60 ) - ( time.wHour * 60 + time.wMinute ) ) / 60.f;
	}
}

void
state_serializer::deserialize_group( cParser &Input, scene::scratch_data &Scratchpad ) {

    scene::Groups.create();
}

void
state_serializer::deserialize_endgroup( cParser &Input, scene::scratch_data &Scratchpad ) {

    scene::Groups.close();
}

void
state_serializer::deserialize_light( cParser &Input, scene::scratch_data &Scratchpad ) {

    // legacy section, no longer used nor supported;
    skip_until( Input, "endlight" );
}

void
state_serializer::deserialize_node( cParser &Input, scene::scratch_data &Scratchpad ) {

    auto const inputline = Input.Line(); // cache in case we need to report error

    scene::node_data nodedata;
    // common data and node type indicator
    Input.getTokens( 4 );
    Input
        >> nodedata.range_max
        >> nodedata.range_min
        >> nodedata.name
        >> nodedata.type;
    if( nodedata.name == "none" ) { nodedata.name.clear(); }
    // type-based deserialization. not elegant but it'll do
    if( nodedata.type == "dynamic" ) {

        auto *vehicle { deserialize_dynamic( Input, Scratchpad, nodedata ) };
        // vehicle import can potentially fail
        if( vehicle == nullptr ) { return; }

        //
        if( vehicle->mdModel != nullptr ) {
            for( auto const &smokesource : vehicle->mdModel->smoke_sources() ) {
                Particles.insert(
                    smokesource.first,
                    vehicle,
                    smokesource.second );
            }
        }

        if( false == simulation::Vehicles.insert( vehicle ) ) {

            ErrorLog( "Bad scenario: duplicate vehicle name \"" + vehicle->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        }

        if( ( vehicle->MoverParameters->CategoryFlag == 1 ) // trains only
         && ( ( ( vehicle->LightList( end::front ) & ( light::headlight_left | light::headlight_right | light::headlight_upper ) ) != 0 )
           || ( ( vehicle->LightList( end::rear )  & ( light::headlight_left | light::headlight_right | light::headlight_upper ) ) != 0 ) ) ) {
            simulation::Lights.insert( vehicle );
        }
    }
    else if( nodedata.type == "track" ) {

        auto *path { deserialize_path( Input, Scratchpad, nodedata ) };
        // duplicates of named tracks are currently experimentally allowed
        if( false == simulation::Paths.insert( path ) ) {
            ErrorLog( "Bad scenario: duplicate track name \"" + path->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
/*
            delete path;
            delete pathnode;
*/
        }
        scene::Groups.insert( scene::Groups.handle(), path );
        simulation::Region->insert_and_register( path );
    }
    else if( nodedata.type == "traction" ) {

        auto *traction { deserialize_traction( Input, Scratchpad, nodedata ) };
        // traction loading is optional
        if( traction == nullptr ) { return; }

        if( false == simulation::Traction.insert( traction ) ) {
            ErrorLog( "Bad scenario: duplicate traction piece name \"" + traction->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        }
        scene::Groups.insert( scene::Groups.handle(), traction );
        simulation::Region->insert_and_register( traction );
    }
    else if( nodedata.type == "tractionpowersource" ) {

        auto *powersource { deserialize_tractionpowersource( Input, Scratchpad, nodedata ) };
        // traction loading is optional
        if( powersource == nullptr ) { return; }

        if( false == simulation::Powergrid.insert( powersource ) ) {
            ErrorLog( "Bad scenario: duplicate power grid source name \"" + powersource->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        }
/*
        // TODO: implement this
        simulation::Region.insert_powersource( powersource, Scratchpad );
*/
    }
    else if( nodedata.type == "model" ) {

        if( nodedata.range_min < 0.0 ) {
            // 3d terrain
            if( false == Scratchpad.binary.terrain ) {
                // if we're loading data from text .scn file convert and import
                auto *instance = deserialize_model( Input, Scratchpad, nodedata );
                // model import can potentially fail
                if( instance == nullptr ) { return; }
                // go through submodels, and import them as shapes
                auto const cellcount = instance->TerrainCount() + 1; // zliczenie submodeli
                for( auto i = 1; i < cellcount; ++i ) {
                    auto *submodel = instance->TerrainSquare( i - 1 );
                    simulation::Region->insert(
                        scene::shape_node().convert( submodel ),
                        Scratchpad,
                        false );
                    // if there's more than one group of triangles in the cell they're held as children of the primary submodel
                    submodel = submodel->ChildGet();
                    while( submodel != nullptr ) {
                        simulation::Region->insert(
                            scene::shape_node().convert( submodel ),
                            Scratchpad,
                            false );
                        submodel = submodel->NextGet();
                    }
                }
                // with the import done we can get rid of the source model
                delete instance;
            }
            else {
                // if binary terrain file was present, we already have this data
                skip_until( Input, "endmodel" );
            }
        }
        else {
            // regular instance of 3d mesh
            auto *instance { deserialize_model( Input, Scratchpad, nodedata ) };
            // model import can potentially fail
            if( instance == nullptr ) { return; }

            if( instance->Model() != nullptr ) {
                for( auto const &smokesource : instance->Model()->smoke_sources() ) {
                    Particles.insert(
                        smokesource.first,
                        instance,
                        smokesource.second );
                }
            }

            if( false == simulation::Instances.insert( instance ) ) {
                ErrorLog( "Bad scenario: duplicate 3d model instance name \"" + instance->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
            }
            scene::Groups.insert( scene::Groups.handle(), instance );
            simulation::Region->insert( instance );
            scene::basic_node *hierarchy_node = instance;
            if (hierarchy_node)
            {   scene::Hierarchy[hierarchy_node->uuid.to_string()] = hierarchy_node;
            }
        }
    }
    else if( ( nodedata.type == "triangles" )
          || ( nodedata.type == "triangle_strip" )
          || ( nodedata.type == "triangle_fan" ) ) {

        auto const skip {
            // all shapes will be loaded from the binary version of the file
            ( true == Scratchpad.binary.terrain )
            // crude way to detect fixed switch trackbed geometry
         || ( ( true == Global.CreateSwitchTrackbeds )
           && ( Input.Name().size() >= 15 )
           && Input.Name().starts_with("scenery/zwr")
           && Input.Name().ends_with(".inc") ) };

        if( false == skip ) {

            simulation::Region->insert(
                scene::shape_node().import(
                    Input, nodedata ),
                Scratchpad,
                true );
        }
        else {
            skip_until( Input, "endtri" );
        }
    }
    else if( ( nodedata.type == "lines" )
          || ( nodedata.type == "line_strip" )
          || ( nodedata.type == "line_loop" ) ) {

        if( false == Scratchpad.binary.terrain ) {

            simulation::Region->insert(
                scene::lines_node().import(
                    Input, nodedata ),
                Scratchpad );
        }
        else {
            // all lines were already loaded from the binary version of the file
            skip_until( Input, "endline" );
        }
    }
    else if( nodedata.type == "memcell" ) {

        auto *memorycell { deserialize_memorycell( Input, Scratchpad, nodedata ) };
        if( false == simulation::Memory.insert( memorycell ) ) {
            ErrorLog( "Bad scenario: duplicate memory cell name \"" + memorycell->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        }
        scene::Groups.insert( scene::Groups.handle(), memorycell );
        simulation::Region->insert( memorycell );
    }
    else if( nodedata.type == "eventlauncher" ) {

        auto *eventlauncher { deserialize_eventlauncher( Input, Scratchpad, nodedata ) };
        if( false == simulation::Events.insert( eventlauncher ) ) {
            ErrorLog( "Bad scenario: duplicate event launcher name \"" + eventlauncher->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        }
            // event launchers can be either global, or local with limited range of activation
            // each gets assigned different caretaker
        if( true == eventlauncher->IsGlobal() ) {
            simulation::Events.queue( eventlauncher );
        }
        else {
            scene::Groups.insert( scene::Groups.handle(), eventlauncher );
            if( false == eventlauncher->IsRadioActivated() ) {
                // NOTE: radio-activated launchers due to potentially large activation radius are resolved on global level rather than put in a region cell
                simulation::Region->insert( eventlauncher );
            }
        }
    }
    else if( nodedata.type == "sound" ) {

        auto *sound { deserialize_sound( Input, Scratchpad, nodedata ) };
        if( false == simulation::Sounds.insert( sound ) ) {
            ErrorLog( "Bad scenario: duplicate sound node name \"" + sound->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        }
        simulation::Region->insert( sound );
    }

}

void
state_serializer::deserialize_origin( cParser &Input, scene::scratch_data &Scratchpad ) {

    glm::dvec3 offset;
    Input.getTokens( 3 );
    Input
        >> offset.x
        >> offset.y
        >> offset.z;
    // sumowanie całkowitego przesunięcia
    Scratchpad.location.offset.emplace(
        offset + (
            Scratchpad.location.offset.empty() ?
                glm::dvec3() :
                Scratchpad.location.offset.top() ) );
}

void
state_serializer::deserialize_endorigin( cParser &Input, scene::scratch_data &Scratchpad ) {

    if( false == Scratchpad.location.offset.empty() ) {
        Scratchpad.location.offset.pop();
    }
    else {
        ErrorLog( "Bad origin: endorigin instruction with empty origin stack in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() - 1 ) + ")" );
    }
}

void
state_serializer::deserialize_scale( cParser &Input, scene::scratch_data &Scratchpad ) {
    // Syntax: `scale <x> <y> <z>` (three tokens, mirroring `rotate`/`angles`).
    // For uniform scaling write the same value three times (e.g. `scale 2 2 2`).
    // Affects both:
    //   1. positions of nodes inside the block (transform() multiplies offset by scale)
    //   2. the per-instance m_scale stamped onto each TAnimModel created inside the block
    // The two together let you scale a multi-node-model group built around a common
    // origin: positions of the parts spread out by the factor AND each part is itself
    // scaled by the same factor, preserving the visual shape of the assembly.
    glm::vec3 factor;
    Input.getTokens( 3 );
    Input >> factor.x >> factor.y >> factor.z;
    if( factor.x <= 0.0f || factor.y <= 0.0f || factor.z <= 0.0f ) {
        ErrorLog( "Bad scale: non-positive scale factor in file \""
                + Input.Name() + "\" (line " + std::to_string( Input.Line() - 1 ) + "); scale (1,1,1) used" );
        factor = glm::vec3( 1.0f );
    }
    // scales compose component-wise, mirroring how origin offsets compose additively.
    glm::vec3 const parent = (
        Scratchpad.location.scale.empty() ?
            glm::vec3( 1.0f ) :
            Scratchpad.location.scale.top() );
    Scratchpad.location.scale.emplace( factor * parent );
}

void
state_serializer::deserialize_endscale( cParser &Input, scene::scratch_data &Scratchpad ) {

    if( false == Scratchpad.location.scale.empty() ) {
        Scratchpad.location.scale.pop();
    }
    else {
        ErrorLog( "Bad scale: endscale instruction with empty scale stack in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() - 1 ) + ")" );
    }
}

void
state_serializer::deserialize_rotate( cParser &Input, scene::scratch_data &Scratchpad ) {

    Input.getTokens( 3 );
    Input
        >> Scratchpad.location.rotation.x
        >> Scratchpad.location.rotation.y
        >> Scratchpad.location.rotation.z;
}

void
state_serializer::deserialize_sky( cParser &Input, scene::scratch_data &Scratchpad ) {

    // sky model
    Input.getTokens( 1 );
    Input
        >> Global.asSky;
    // anything else left in the section has no defined meaning
    skip_until( Input, "endsky" );
}

void
state_serializer::deserialize_test( cParser &Input, scene::scratch_data &Scratchpad ) {

    // legacy section, no longer supported;
    skip_until( Input, "endtest" );
}

void
state_serializer::deserialize_time( cParser &Input, scene::scratch_data &Scratchpad ) {

    // current scenario time
    cParser timeparser( Input.getToken<std::string>() );
    timeparser.getTokens( 2, false, ":" );
    auto &time = simulation::Time.data();
    timeparser
        >> time.wHour
        >> time.wMinute;

    // remaining sunrise and sunset parameters are no longer used, as they're now calculated dynamically
    // anything else left in the section has no defined meaning
    skip_until( Input, "endtime" );

	if (!Scratchpad.time_initialized)
		Scratchpad.time_initialized = true;

	init_time();
}

void
state_serializer::deserialize_trainset( cParser &Input, scene::scratch_data &Scratchpad ) {

	int line = Input.LineMain();
	if (line != -1) {
		auto it = Global.trainset_overrides.find(line);
		if (it != Global.trainset_overrides.end()) {
			skip_until(Input, "endtrainset");
			Input.injectString(it->second);
			return;
		}
	}

    if( true == Scratchpad.trainset.is_open ) {
        // shouldn't happen but if it does wrap up currently open trainset and report an error
        deserialize_endtrainset( Input, Scratchpad );
        ErrorLog( "Bad scenario: encountered nested trainset definitions in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() ) + ")" );
    }

	Scratchpad.trainset = scene::scratch_data::trainset_data();
	Scratchpad.trainset.is_open = true;

    Input.getTokens( 4 );
    Input
        >> Scratchpad.trainset.name
        >> Scratchpad.trainset.track
        >> Scratchpad.trainset.offset
        >> Scratchpad.trainset.velocity;
}

void 
state_serializer::deserialize_terrain(cParser &Input, scene::scratch_data &Scratchpad)
{
	std::string line;
	Input.getTokens(1);
	Input >> line;
	if ( Global.file_binary_terrain
     && ( line.ends_with( ".sbt" ) || line.ends_with( ".eu7" ) ) )
	{
        auto const eu7path { scene::eu7::terrain_binary_path( line ) };
        if( scene::eu7::probe_terrain_file( eu7path ) ) {
            Scratchpad.binary.terrain = true;
            Global.file_binary_terrain_state = true;
            Scratchpad.binary.terrain_included = true;
            Scratchpad.terrain_name = line;
            WriteLog( "Included EU7 terrain: " + eu7path );
            scene::eu7::load_terrain( *Region, eu7path );
        }
        else if( scene::eu7::probe_file( eu7path ) ) {
            Scratchpad.binary.terrain = true;
            Global.file_binary_terrain_state = true;
            Scratchpad.binary.terrain_included = true;
            Scratchpad.terrain_name = line;
            WriteLog( "Included EU7 module (SBT skipped): " + eu7path );
            scene::eu7::load_module( eu7path, *this );
        }
        else if( Region->is_scene( line ) ) {
            Scratchpad.binary.terrain = true;
            Global.file_binary_terrain_state = true;
            Scratchpad.binary.terrain_included = true;
            Scratchpad.terrain_name = line;
            WriteLog( "Included SBT file: " + line );
            Region->deserialize( Scratchpad.terrain_name );
        }
    }

    skip_until(Input, "endterrain");
	
}

void
state_serializer::deserialize_endtrainset( cParser &Input, scene::scratch_data &Scratchpad ) {

    if( ( false == Scratchpad.trainset.is_open )
     || ( true == Scratchpad.trainset.vehicles.empty() ) ) {
        // not bloody likely but we better check for it just the same
        ErrorLog( "Bad trainset: empty trainset defined in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() - 1 ) + ")" );
        Scratchpad.trainset.is_open = false;
        return;
    }

    std::size_t vehicleindex { 0 };
    for( auto *vehicle : Scratchpad.trainset.vehicles ) {
        // go through list of vehicles in the trainset, coupling them together and checking for potential driver
        if( ( vehicle->Mechanik != nullptr )
         && ( vehicle->Mechanik->primary() ) ) {
            // primary driver will receive the timetable for this trainset
            Scratchpad.trainset.driver = vehicle;
            // they'll also receive assignment data if there's any
            auto const lookup { Scratchpad.trainset.assignment.find( Global.asLang ) };
            if( lookup != Scratchpad.trainset.assignment.end() ) {
                vehicle->Mechanik->assignment() = lookup->second;
            }
        }
        if( vehicleindex > 0 ) {
            // from second vehicle on couple it with the previous one
            Scratchpad.trainset.vehicles[ vehicleindex - 1 ]->AttachNext(
                vehicle,
                Scratchpad.trainset.couplings[ vehicleindex - 1 ] );
        }
        ++vehicleindex;
    }

    if( Scratchpad.trainset.driver != nullptr ) {
        // if present, send timetable to the driver
        // wysłanie komendy "Timetable" ustawia odpowiedni tryb jazdy
        auto *controller = Scratchpad.trainset.driver->Mechanik;
            controller->DirectionInitial();
            controller->PutCommand(
                "Timetable:" + Scratchpad.trainset.name,
                Scratchpad.trainset.velocity,
                0,
                nullptr );
    }
    if( Scratchpad.trainset.couplings.back() == coupling::faux ) {
        // jeśli ostatni pojazd ma sprzęg 0 to założymy mu końcówki blaszane (jak AI się odpali, to sobie poprawi)
        // place end signals only on trains without a driver, activate markers otherwise
        Scratchpad.trainset.vehicles.back()->RaLightsSet(
            -1,
            ( Scratchpad.trainset.driver != nullptr ?
                light::redmarker_left | light::redmarker_right | light::rearendsignals :
                light::rearendsignals ) );
    }

    for( auto *vehicle : Scratchpad.trainset.vehicles ) {
        if( false == simulation::Vehicles.insert( vehicle ) ) {
            ErrorLog( "Bad trainset: duplicate vehicle name \"" + vehicle->name() + "\"" );
        }
        if( ( vehicle->MoverParameters->CategoryFlag == 1 )
         && ( ( ( vehicle->LightList( end::front ) & ( light::headlight_left | light::headlight_right | light::headlight_upper ) ) != 0 )
           || ( ( vehicle->LightList( end::rear ) & ( light::headlight_left | light::headlight_right | light::headlight_upper ) ) != 0 ) ) ) {
            simulation::Lights.insert( vehicle );
        }
    }

    // all done
    Scratchpad.trainset.is_open = false;
}

// creates path and its wrapper, restoring class data from provided stream
TTrack *
state_serializer::deserialize_path( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata ) {

    // TODO: refactor track and wrapper classes and their de/serialization. do offset and rotation after deserialization is done
    auto *track = new TTrack( Nodedata );
    auto const offset { (
        Scratchpad.location.offset.empty() ?
            glm::dvec3 { 0.0 } :
            glm::dvec3 {
                Scratchpad.location.offset.top().x,
                Scratchpad.location.offset.top().y,
                Scratchpad.location.offset.top().z } ) };
    track->Load( &Input, offset );

    return track;
}

TTraction *
state_serializer::deserialize_traction( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata ) {

    if( false == Global.bLoadTraction ) {
        skip_until( Input, "endtraction" );
        return nullptr;
    }
    // TODO: refactor track and wrapper classes and their de/serialization. do offset and rotation after deserialization is done
    auto *traction = new TTraction( Nodedata );
    auto offset = (
        Scratchpad.location.offset.empty() ?
            glm::dvec3() :
            Scratchpad.location.offset.top() );
    traction->Load( &Input, offset );

    return traction;
}

TTractionPowerSource *
state_serializer::deserialize_tractionpowersource( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata ) {

    if( false == Global.bLoadTraction ) {
        skip_until( Input, "end" );
        return nullptr;
    }

    auto *powersource = new TTractionPowerSource( Nodedata );
    powersource->Load( &Input );
    // adjust location
    powersource->location( transform( powersource->location(), Scratchpad ) );

    return powersource;
}

TMemCell *
state_serializer::deserialize_memorycell( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata ) {

    auto *memorycell = new TMemCell( Nodedata );
    memorycell->Load( &Input );
    // adjust location
    memorycell->location( transform( memorycell->location(), Scratchpad ) );

    return memorycell;
}

TEventLauncher *
state_serializer::deserialize_eventlauncher( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata ) {

    glm::dvec3 location;
    Input.getTokens( 3 );
    Input
        >> location.x
        >> location.y
        >> location.z;

    auto *eventlauncher = new TEventLauncher( Nodedata );
    eventlauncher->Load( &Input );
    eventlauncher->location( transform( location, Scratchpad ) );

    return eventlauncher;
}

TAnimModel *
state_serializer::deserialize_model( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata ) {

    glm::dvec3 location;
    glm::vec3 rotation;
    Input.getTokens( 4 );
    Input
        >> location.x
        >> location.y
        >> location.z
        >> rotation.y;

    auto *instance = new TAnimModel( Nodedata );
    instance->Angles( Scratchpad.location.rotation + rotation ); // dostosowanie do pochylania linii
    // pick up the scale active at this point in the scenario stream — outer
    // `scale`/`endscale` blocks compose multiplicatively in the scratchpad.
    // Load() may further multiply this by an inline `scale <factor>` token.
    if( false == Scratchpad.location.scale.empty() ) {
        instance->Scale( Scratchpad.location.scale.top() );
    }

    if( instance->Load( &Input, false ) ) {
        instance->location( transform( location, Scratchpad ) );
    }
    else {
        // model nie wczytał się - ignorowanie node
        SafeDelete( instance );
    }

    return instance;
}

TDynamicObject *
state_serializer::deserialize_dynamic( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata ) {

    if( false == Scratchpad.trainset.is_open ) {
        // part of trainset data is used when loading standalone vehicles, so clear it just in case
        Scratchpad.trainset = scene::scratch_data::trainset_data();
    }
    auto const inputline { Input.Line() }; // cache in case of errors
    // basic attributes
    auto datafolder { Input.getToken<std::string>() };
    auto skinfile { Input.getToken<std::string>() };
    auto mmdfile { Input.getToken<std::string>() };

	replace_slashes(datafolder);
	replace_slashes(skinfile);
	replace_slashes(mmdfile);

    auto const pathname = (
        Scratchpad.trainset.is_open ?
            Scratchpad.trainset.track :
            Input.getToken<std::string>() );
    auto const offset { Input.getToken<double>( false ) };
    auto const drivertype { Input.getToken<std::string>() };
    auto const couplingdata = (
        Scratchpad.trainset.is_open ?
            Input.getToken<std::string>() :
            "3" );
    auto const velocity = (
        Scratchpad.trainset.is_open ?
            Scratchpad.trainset.velocity :
            Input.getToken<float>( false ) );
    // extract coupling type and optional parameters
    auto const couplingdatawithparams = couplingdata.find( '.' );
    auto coupling = (
        couplingdatawithparams != std::string::npos ?
            std::atoi( couplingdata.substr( 0, couplingdatawithparams ).c_str() ) :
            std::atoi( couplingdata.c_str() ) );
    if( coupling < 0 ) {
        // sprzęg zablokowany (pojazdy nierozłączalne przy manewrach)
        coupling = ( -coupling ) | coupling::permanent;
    }
    if( ( offset != -1.0 )
     && ( std::abs( offset ) > 0.5 ) ) { // maksymalna odległość między sprzęgami - do przemyślenia
        // likwidacja sprzęgu, jeśli odległość zbyt duża - to powinno być uwzględniane w fizyce sprzęgów...
        coupling = coupling::faux; 
    }
    auto const params = (
        couplingdatawithparams != std::string::npos ?
            couplingdata.substr( couplingdatawithparams + 1 ) :
            "" );
    // load amount and type
    auto loadcount { Input.getToken<int>( false ) };
    auto loadtype = (
        loadcount ?
            Input.getToken<std::string>() :
            "" );
    if( loadtype == "enddynamic" ) {
        // idiotoodporność: ładunek bez podanego typu nie liczy się jako ładunek
        loadcount = 0;
        loadtype = "";
    }

    auto *path = simulation::Paths.find( pathname );
    if( path == nullptr ) {

        ErrorLog( "Bad scenario: vehicle \"" + Nodedata.name + "\" placed on nonexistent path \"" + pathname + "\" in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        skip_until( Input, "enddynamic" );
        return nullptr;
    }

    if( ( true == Scratchpad.trainset.vehicles.empty() ) // jeśli pierwszy pojazd,
     && ( false == path->m_events0.empty() ) // tor ma Event0
     && ( std::abs( velocity ) <= 1.f ) // a skład stoi
     && ( Scratchpad.trainset.offset >= 0.0 ) // ale może nie sięgać na owy tor
     && ( Scratchpad.trainset.offset <  8.0 ) ) { // i raczej nie sięga
        // przesuwamy około pół EU07 dla wstecznej zgodności
        Scratchpad.trainset.offset = 8.0;
    }

    auto *vehicle = new TDynamicObject();
    
    auto const length =
        vehicle->Init(
            Nodedata.name,
            datafolder, skinfile, mmdfile,
            path,
            ( offset == -1.0 ?
                Scratchpad.trainset.offset :
                Scratchpad.trainset.offset - offset ),
            drivertype,
            velocity,
            Scratchpad.trainset.name,
            loadcount, loadtype,
            ( offset == -1.0 ),
            params );

    if( length != 0.0 ) { // zero oznacza błąd
        // przesunięcie dla kolejnego, minus bo idziemy w stronę punktu 1
        Scratchpad.trainset.offset -= length;
        // automatically establish permanent connections for couplers which specify them in their definitions
        if( ( coupling != 0 )
         && ( vehicle->MoverParameters->Couplers[ ( offset == -1.0 ? end::front : end::rear ) ].AllowedFlag & coupling::permanent ) ) {
            coupling |= coupling::permanent;
        }
        if( true == Scratchpad.trainset.is_open ) {
            Scratchpad.trainset.vehicles.emplace_back( vehicle );
            Scratchpad.trainset.couplings.emplace_back( coupling );
        }
    }
    else {
        if( vehicle->MyTrack != nullptr ) {
            // rare failure case where vehicle with length of 0 is added to the track,
            // treated as error code and consequently deleted, but still remains on the track
            vehicle->MyTrack->RemoveDynamicObject( vehicle );
        }
        delete vehicle;
        skip_until( Input, "enddynamic" );
        return nullptr;
    }

    auto const destination { Input.getToken<std::string>() };
    if( destination != "enddynamic" ) {
        // optional vehicle destination parameter
        vehicle->asDestination = Input.getToken<std::string>();
        skip_until( Input, "enddynamic" );
    }

    return vehicle;
}

sound_source *
state_serializer::deserialize_sound( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data const &Nodedata ) {

    glm::dvec3 location;
    Input.getTokens( 3 );
    Input
        >> location.x
        >> location.y
        >> location.z;
    // adjust location
    location = transform( location, Scratchpad );

    auto *sound = new sound_source( sound_placement::external, Nodedata.range_max );
    sound->offset( location );
    sound->name( Nodedata.name );
    sound->deserialize( Input, sound_type::single );

    skip_until( Input, "endsound" );

    return sound;
}

// skips content of stream until specified token
void
state_serializer::skip_until( cParser &Input, std::string const &Token ) {

    std::string token { Input.getToken<std::string>() };
    while( ( false == token.empty() )
        && ( token != Token ) ) {

        token = Input.getToken<std::string>();
    }
}

// transforms provided location by specifed rotation, scale and offset
glm::dvec3
state_serializer::transform( glm::dvec3 Location, scene::scratch_data const &Scratchpad ) {

    if( Scratchpad.location.rotation != glm::vec3( 0, 0, 0 ) ) {
        auto const rotation = glm::radians( Scratchpad.location.rotation );
        Location = glm::rotateY<double>( Location, rotation.y ); // Ra 2014-11: uwzględnienie rotacji
    }
    // Scale applies in local origin space — positions inside a `scale 2 2 2` block
    // are pushed twice as far from the local origin along each axis, so a
    // multi-node-model group (e.g. a building made of separate node models built
    // around a shared origin) ends up looking uniformly scaled rather than just
    // having one piece grow. Per-axis values stretch the assembly anisotropically.
    if( false == Scratchpad.location.scale.empty() ) {
        auto const &s = Scratchpad.location.scale.top();
        Location.x *= static_cast<double>( s.x );
        Location.y *= static_cast<double>( s.y );
        Location.z *= static_cast<double>( s.z );
    }
    if( false == Scratchpad.location.offset.empty() ) {
        Location += Scratchpad.location.offset.top();
    }
    return Location;
}

/*
// stores class data in specified file, in legacy (text) format
void
state_serializer::export_as_text(std::string const &Scenariofile) const {

    if( Scenariofile == "$.scn" ) {
        ErrorLog( "Bad file: scenery export not supported for file \"$.scn\"" );
    }
    else {
        WriteLog( "Scenery data export in progress..." );
    }

	auto filename { Scenariofile };
	while( filename[ 0 ] == '$' ) {
        // trim leading $ char rainsted utility may add to the base name for modified .scn files
		filename.erase( 0, 1 );
    }
	erase_extension( filename );
	auto absfilename = Global.asCurrentSceneryPath + filename + "_export";

	std::ofstream scmdirtyfile { absfilename + "_dirty.scm" };
	export_nodes_to_stream(scmdirtyfile, true);

	std::ofstream scmfile { absfilename + ".scm" };
	export_nodes_to_stream(scmfile, false);

	// sounds
	// NOTE: sounds currently aren't included in groups
	scmfile << "// sounds\n";
	Region->export_as_text( scmfile );

	scmfile << "// modified objects\ninclude " << filename << "_export_dirty.scm\n";

	std::ofstream ctrfile { absfilename + ".ctr" };
	// mem cells
	ctrfile << "// memory cells\n";
	for( auto const *memorycell : Memory.sequence() ) {
		if( ( true == memorycell->is_exportable )
		 && ( memorycell->group() == null_handle ) ) {
			memorycell->export_as_text( ctrfile );
		}
	}

	// events
	Events.export_as_text( ctrfile );

    WriteLog( "Scenery data export done." );
}
*/
void
state_serializer::export_as_text(std::string const &Scenariofile) const {

    if( Scenariofile == "$.scn" ) {
        ErrorLog( "Bad file: scenery export not supported for file \"$.scn\"" );
    }
    else {
        WriteLog( "Scenery data export in progress..." );
    }

	auto filename { Scenariofile };
	while( filename[ 0 ] == '$' ) {
        // trim leading $ char rainsted utility may add to the base name for modified .scn files
		filename.erase( 0, 1 );
    }
	erase_extension( filename );
	auto absfilename = Global.asCurrentSceneryPath + filename + "_export";

	std::ofstream scmdirtyfile { absfilename + "_dirty.scm" };
	export_nodes_to_stream(scmdirtyfile, true);

	std::ofstream scmfile { absfilename + ".scm" };
	export_nodes_to_stream(scmfile, false);

	// sounds
	// NOTE: sounds currently aren't included in groups
	scmfile << "// sounds\n";
	Region->export_as_text( scmfile );

	scmfile << "// modified objects\ninclude " << filename << "_export_dirty.scm\n";

	std::ofstream ctrfile { absfilename + ".ctr" };
	// mem cells
	ctrfile << "// memory cells\n";
	for( auto const *memorycell : Memory.sequence() ) {
		if( ( true == memorycell->is_exportable )
		 && ( memorycell->group() == null_handle ) ) {
			memorycell->export_as_text( ctrfile );
		}
	}

	// events
	Events.export_as_text( ctrfile );

    WriteLog( "Scenery data export done." );
}

void
state_serializer::export_nodes_to_stream(std::ostream &scmfile, bool Dirty) const {
	// groups
	scmfile << "// groups\n";
	scene::Groups.export_as_text( scmfile, Dirty );

	// tracks
	scmfile << "// paths\n";
	for( auto const *path : Paths.sequence() ) {
		if( path->dirty() == Dirty && path->group() == null_handle ) {
			path->export_as_text( scmfile );
		}
	}
	// traction
	scmfile << "// traction\n";
	for( auto const *traction : Traction.sequence() ) {
		if( traction->dirty() == Dirty && traction->group() == null_handle ) {
			traction->export_as_text( scmfile );
		}
	}
	// power grid
	scmfile << "// traction power sources\n";
	for( auto const *powersource : Powergrid.sequence() ) {
		if( powersource->dirty() == Dirty && powersource->group() == null_handle ) {
			powersource->export_as_text( scmfile );
		}
	}
	// models
	scmfile << "// instanced models\n";
	for( auto const *instance : Instances.sequence() ) {
		if( instance && instance->dirty() == Dirty && instance->group() == null_handle ) {
			instance->export_as_text( scmfile );
		}
	}
}

TAnimModel *state_serializer::create_model(const std::string &src, const std::string &name, const glm::dvec3 &position) {
	cParser parser(src);
	parser.getTokens(); // "node"
	parser.getTokens(2); // ranges

	scene::node_data nodedata;
	parser >> nodedata.range_max >> nodedata.range_min;

	parser.getTokens(2); // name, type
	nodedata.name = name;
	nodedata.type = "model";

	scene::scratch_data scratch;

	TAnimModel *cloned = deserialize_model(parser, scratch, nodedata);

	if (!cloned)
		return nullptr;

	cloned->mark_dirty();
	cloned->location(position);
	simulation::Instances.insert(cloned);
	simulation::Region->insert(cloned);

	return cloned;
}

TEventLauncher *state_serializer::create_eventlauncher(const std::string &src, const std::string &name, const glm::dvec3 &position) {
	cParser parser(src);
	parser.getTokens(); // "node"
	parser.getTokens(2); // ranges

	scene::node_data nodedata;
	parser >> nodedata.range_max >> nodedata.range_min;

	parser.getTokens(2); // name, type
	nodedata.name = name;
	nodedata.type = "eventlauncher";

	scene::scratch_data scratch;

	TEventLauncher *launcher = deserialize_eventlauncher(parser, scratch, nodedata);

	if (!launcher)
		return nullptr;

	launcher->Event1 = simulation::Events.FindEvent( launcher->asEvent1Name );
	launcher->location(position);
	simulation::Events.insert(launcher);
	simulation::Region->insert(launcher);

	return launcher;
}

} // simulation

  //---------------------------------------------------------------------------

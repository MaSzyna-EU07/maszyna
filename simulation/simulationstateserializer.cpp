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

#include "utilities/Globals.h"
#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "simulation/simulationsounds.h"
#include "simulation/simulationenvironment.h"
#include "scene/scenenodegroups.h"
#include "scene/scenelayers.h"
#include "scene/sceneterrain.h"
#include "scene/scenemodelentries.h"
#include "rendering/particles.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "model/AnimModel.h"
#include "rendering/lightarray.h"
#include "world/TractionPower.h"
#include "world/Road.h"
#include "world/Sweep.h"
#include "world/RoadPoint.h"
#include "application/application.h"
#include "rendering/renderer.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "editor/editorTerrainStreamer.hpp"


namespace simulation {

namespace {

double vehicle_length( std::string const &Folder, std::string const &Type ) {

    TMoverParameters probe( 0.0, Type, Type, 0 );
    return probe.LoadFIZ( paths::dynamic + Folder + "/" ) ? probe.Dim.L : 0.0;
}

} // namespace

std::shared_ptr<deserializer_state>
state_serializer::deserialize_begin( std::string const &Scenariofile ) {

    crashreport_add_info("scenario", Scenariofile);

    // drop any streamed editor terrain from a previously loaded scenery before the old region (and
    // its sections, which those chunks referenced) is destroyed below
    EditorTerrain.reset();
    scene::terrain_file::references().clear();

    // TODO: move initialization to separate routine so we can reuse it
    SafeDelete( Region );
    Region = new scene::basic_region();

    simulation::State.init_scripting_interface();

	// NOTE: for the time being import from text format is a given, since we don't have full binary serialization
	std::shared_ptr<deserializer_state> state =
	        std::make_shared<deserializer_state>(Scenariofile, cParser::buffer_FILE, Global.asCurrentSceneryPath, Global.bLoadTraction);

    // TODO: check first for presence of serialized binary files
    // if this fails, fall back on the legacy text format
	state->scratchpad.name = Scenariofile;
    if( true == Global.file_binary_terrain
     && Scenariofile != "$.scn" ) {
        // compilation to binary file isn't supported for rainsted-created overrides
        // NOTE: we postpone actual loading of the scene until we process time, season and weather data
		state->scratchpad.binary.terrain = Region->is_scene( Scenariofile ) ;
		state->scratchpad.binary.terrain_default = state->scratchpad.binary.terrain;
    }
	Global.file_binary_terrain_skipped = 0;

	if (false != state->scratchpad.binary.terrain)
	{
		Global.file_binary_terrain_state = true;
		WriteLog("Default SBT present");
    }
	else
	{
		Global.file_binary_terrain_state = false;
		WriteLog("Default SBT absent");
    }
    scene::Groups.create();

    scene::Layers.clear();
    if( true == Global.editor_session ) {
        // scenery opened for editing: keep track of which scenery file defines each node.
        // the scenario file itself is the root layer, and the default target for nodes created in the editor
        state->input.sceneryLayers = true;
        scene::Layers.active( scene::Layers.open( Scenariofile ) );
    }

	if( false == state->input.ok() )
		throw invalid_scenery_exception();

	// prepare deserialization function table
	// since all methods use the same objects, we can have simple, hard-coded binds or lambdas for the task
	using deserializefunction = void( state_serializer::*)(cParser &, scene::scratch_data &);
	std::vector<
	    std::pair<
	        std::string,
	        deserializefunction> > functionlist = {
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
	            { "heightmap_terrain", &state_serializer::deserialize_heightmapterrain },
	            { "editorterrain", &state_serializer::deserialize_editorterrain },
	            { "endtrainset", &state_serializer::deserialize_endtrainset },
	            { "reversed", &state_serializer::deserialize_reversed } };

	for( auto &function : functionlist ) {
		state->functionmap.emplace( function.first, std::bind( function.second, this, std::ref( state->input ), std::ref( state->scratchpad ) ) );
	}

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
    scene::Layers.close();

	scene::Groups.update_map();
	Region->create_map_geometry();

	if( true == Global.file_binary_terrain
     && false == state->scratchpad.binary.terrain
     && true == state->scratchpad.binary.terrain_textfiles.empty()
	 && state->scenariofile != "$.scn" ) {
		// if we didn't find usable binary version of the scenario files, create them now for future use
		// as long as the scenario file wasn't rainsted-created base file override.
		// NOTE: this is done only for the sceneries which don't refer to terrain files (.txtf, .btf). these get binary
		// versions of their terrain files instead, and a file holding all geometry of the scenery would only get in their way
		Region->serialize( state->scenariofile );
	}

	// geometry of the roads is generated on each load instead of being kept in the binary terrain file,
	// so it's inserted in the region only after that file had its chance to be written
	simulation::Roads.create_geometry( Scratchpad );
	simulation::Junctions.create_geometry( Scratchpad );
	simulation::Roadpoints.create_geometry();
	simulation::Sweeps.create_geometry( Scratchpad );

	return false;
}

void
state_serializer::deserialize_isolated( cParser &Input, scene::scratch_data &Scratchpad ) {
    // first parameter specifies name of parent piece...
    auto token { Input.getToken<std::string>() };
    auto *groupowner { TIsolated::Find( token ) };
    // ...followed by list of its tracks
    while( false == (token = Input.getToken<std::string>()).empty()
        && token != "endisolated" ) {
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
    while( false == (token = Input.getToken<std::string>()).empty()
        && token != "endarea" ) {
        // bind the children with their parent
        auto *isolated { TIsolated::Find( token ) };
        isolated->parent( groupowner );
    }
}

void
state_serializer::deserialize_assignment( cParser &Input, scene::scratch_data &Scratchpad ) {

    std::string token { Input.getToken<std::string>() };
    while( false == token.empty()
        && token != "endassignment" ) {
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
    while( false == token.empty()
        && token != "endatmo" ) {
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
        scene::Layers.count( scene::Layers.handle(), scene::layer_item::event );
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

    // scenery opened for editing: what the editor adds to the scenery files is kept ahead of the initialization
    scene::Layers.initialization( { Input.TokenBegin(), Input.TokenEnd() }, Input.InLayerFile() );

    if( true == Scratchpad.binary.terrain ) {
        // at this stage it should be safe to import terrain from the binary scene file
        // TBD: postpone loading furter and only load required blocks during the simulation?
		if (false == Scratchpad.binary.terrain_included)
		{
			Region->deserialize(Scratchpad.name);
		}
		else
		{
			// files named by terrain directives are loaded here rather than on the spot, as by this point
			// it's known whether all of them can be used. the list holds each file once
			for (auto const &terrainfile : Scratchpad.binary.terrain_files)
			{
				Region->deserialize(terrainfile);
			}
		}
			
    }
    // binary versions of terrain files only announce what they hold at this point, the sections of the scene load it when they need it.
    // scenery opened for editing gets it all at once, the editor works with complete geometry
    for( auto const &terrainfile : Scratchpad.binary.terrain_binaryfiles ) {
        scene::terrain_file::attach( terrainfile, *Region, false == Global.editor_session );
    }
    Scratchpad.binary.terrain_binaryfiles.clear();

    simulation::Paths.InitTracks();
    // the roads tie up their own lanes where they gain or lose some, the junctions tie the lanes of the roads together,
    // what's left loose after that gets closed by the roads
    simulation::Roads.InitLanes();
    simulation::Junctions.InitJunctions();
    simulation::Roads.InitRoads();
    // crossings and traffic points go by the lanes, which are complete at this point
    simulation::Roadpoints.InitRoadpoints();
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
		auto const localtime = utc_tm( Global.starting_timestamp );
		Global.ScenarioTimeOffset = ( localtime.tm_hour * 60 + localtime.tm_min - ( time.wHour * 60 + time.wMinute ) ) / 60.f;
	}
	else if( false == std::isnan( Global.ScenarioTimeOverride ) ) {
		// scenario time override takes precedence over scenario time offset
		Global.ScenarioTimeOffset = (Global.ScenarioTimeOverride * 60 - ( time.wHour * 60 + time.wMinute ) ) / 60.f;
	}
}

void
state_serializer::deserialize_group( cParser &Input, scene::scratch_data &Scratchpad ) {

    // a group of the scenery stands by itself, rather than being a part of the file it's in (e.g. made in the editor)
    scene::Groups.create( true );
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
    auto const sourcebegin = Input.TokenBegin(); // location of the node definition, for scenery opened for editing

    scene::node_data nodedata;
    nodedata.layer = scene::Layers.handle();
    if( ( nodedata.layer != null_handle ) && ( false == Input.InLayerFile() ) ) {
        // defined by a template, or by something else which isn't a scenery layer file
        nodedata.instance = ( scene::Layers.instance() != 0 ? scene::Layers.instance() : scene::untracked_instance );
    }
    // a run of plain model instances is taken straight from the text, instead of token by token
    if( true == deserialize_models( Input, Scratchpad, nodedata, inputline, sourcebegin ) ) { return; }
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

        if( vehicle->MoverParameters->CategoryFlag == 1 // trains only
         && ( (vehicle->LightList(end::front) & (light::headlight_left | light::headlight_right | light::headlight_upper)) != 0
           || (vehicle->LightList(end::rear) & (light::headlight_left | light::headlight_right | light::headlight_upper)) != 0 ) ) {
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
        scene::Layers.track( path, { sourcebegin, Input.TokenEnd() } );
    }
    else if( nodedata.type == "road" ) {

        auto *road { new road_node( nodedata ) };
        road->import(
            Input,
            ( Scratchpad.location.offset.empty() ?
                glm::dvec3 { 0.0 } :
                glm::dvec3 { Scratchpad.location.offset.top() } ) );
        if( false == simulation::Roads.insert( road ) ) {
            ErrorLog( "Bad scenario: duplicate road name \"" + road->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        }
        scene::Groups.insert( scene::Groups.handle(), road );
        // the lanes are regular paths, registered right away so they get joined with their neighbours along with the tracks
        road->create_lanes();
        scene::Layers.track( road, { sourcebegin, Input.TokenEnd() } );
    }
    else if( nodedata.type == "sweep" ) {

        auto *sweep { new sweep_node( nodedata ) };
        sweep->import(
            Input,
            ( Scratchpad.location.offset.empty() ?
                glm::dvec3 { 0.0 } :
                glm::dvec3 { Scratchpad.location.offset.top() } ) );
        if( false == simulation::Sweeps.insert( sweep ) ) {
            ErrorLog( "Bad scenario: duplicate sweep name \"" + sweep->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        }
        scene::Groups.insert( scene::Groups.handle(), sweep );
        scene::Layers.track( sweep, { sourcebegin, Input.TokenEnd() } );
    }
    else if( nodedata.type == "junction" ) {

        auto *junction { new junction_node( nodedata ) };
        junction->import(
            Input,
            ( Scratchpad.location.offset.empty() ?
                glm::dvec3 { 0.0 } :
                glm::dvec3 { Scratchpad.location.offset.top() } ) );
        road_node::state stretch;
        if( junction->as_road( stretch ) ) {
            // a junction of two roads is where a road changes its lanes, which used to take a junction and is done by a road now.
            // it's loaded as the road it would be, and marked as changed so it's written as one when the scenery is saved
            delete junction;
            nodedata.type = "road";
            auto *road { new road_node( nodedata ) };
            road->define( stretch );
            if( false == simulation::Roads.insert( road ) ) {
                ErrorLog( "Bad scenario: duplicate road name \"" + road->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
            }
            scene::Groups.insert( scene::Groups.handle(), road );
            road->create_lanes();
            road->mark_dirty();
            scene::Layers.track( road, { sourcebegin, Input.TokenEnd() } );
        }
        else {
            if( false == simulation::Junctions.insert( junction ) ) {
                ErrorLog( "Bad scenario: duplicate junction name \"" + junction->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
            }
            scene::Groups.insert( scene::Groups.handle(), junction );
            scene::Layers.track( junction, { sourcebegin, Input.TokenEnd() } );
        }
    }
    else if( roadpoint_node::is_keyword( nodedata.type ) ) {
        // level crossing, or a point where road vehicles appear or are taken away
        auto *point { new roadpoint_node( nodedata ) };
        point->import(
            Input,
            ( Scratchpad.location.offset.empty() ?
                glm::dvec3 { 0.0 } :
                glm::dvec3 { Scratchpad.location.offset.top() } ) );
        if( false == simulation::Roadpoints.insert( point ) ) {
            ErrorLog( "Bad scenario: duplicate road point name \"" + point->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        }
        scene::Groups.insert( scene::Groups.handle(), point );
        scene::Layers.track( point, { sourcebegin, Input.TokenEnd() } );
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
        scene::Layers.track( traction, { sourcebegin, Input.TokenEnd() } );
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
                ++Scratchpad.binary.geometry_imported;
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
                ++Scratchpad.binary.geometry_skipped;
                skip_until( Input, "endmodel" );
            }
        }
        else {
            // regular instance of 3d mesh
            auto *instance { deserialize_model( Input, Scratchpad, nodedata ) };
            // model import can potentially fail
            if( instance == nullptr ) { return; }

            insert_model( instance, Input, inputline, { sourcebegin, Input.TokenEnd() } );
        }
    }
    else if( nodedata.type == "triangles"
          || nodedata.type == "triangle_strip"
          || nodedata.type == "triangle_fan" ) {

        auto const skip {
            // all shapes will be loaded from the binary version of the file
            true == Scratchpad.binary.terrain
            // crude way to detect fixed switch trackbed geometry
         || ( true == Global.CreateSwitchTrackbeds
           && Input.Name().size() >= 15
           && Input.Name().starts_with("scenery/zwr")
           && Input.Name().ends_with(".inc") ) };

        material_handle material { null_handle };
        if( false == skip ) {

            ++Scratchpad.binary.geometry_imported;
            auto shape { scene::shape_node().import( Input, nodedata ) };
            material = shape.data().material;
            if( true == Global.editor_session ) {
                shape.source_file( scene::terrain_file::reference_of( Input.Name() ) );
            }
            simulation::Region->insert(
                std::move( shape ),
                Scratchpad,
                true );
        }
        else {
            if( true == Scratchpad.binary.terrain ) {
                ++Scratchpad.binary.geometry_skipped;
            }
            if( nodedata.layer != null_handle && Input.InLayerFile() ) {
                // the editor wants to know the material even of the shapes which come from the binary terrain
                auto token { Input.getToken<std::string>() };
                if( token == "material" ) {
                    skip_until( Input, "endmaterial" );
                    token = Input.getToken<std::string>();
                }
                replace_slashes( token );
                material = GfxRenderer->Fetch_Material( token );
            }
            skip_until( Input, "endtri" );
        }
        if( nodedata.layer != null_handle && Input.InLayerFile() ) {
            scene::Layers.shape( material, { sourcebegin, Input.TokenEnd() } );
        }
    }
    else if( nodedata.type == "lines"
          || nodedata.type == "line_strip"
          || nodedata.type == "line_loop" ) {

        if( false == Scratchpad.binary.terrain ) {

            ++Scratchpad.binary.geometry_imported;
            simulation::Region->insert(
                scene::lines_node().import(
                    Input, nodedata ),
                Scratchpad );
        }
        else {
            // all lines were already loaded from the binary version of the file
            ++Scratchpad.binary.geometry_skipped;
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
        scene::Layers.track( memorycell, { sourcebegin, Input.TokenEnd() } );
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
        scene::Layers.track( eventlauncher, { sourcebegin, Input.TokenEnd() } );
    }
    else if( nodedata.type == "sound" ) {

        auto *sound { deserialize_sound( Input, Scratchpad, nodedata ) };
        if( false == simulation::Sounds.insert( sound ) ) {
            ErrorLog( "Bad scenario: duplicate sound node name \"" + sound->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        }
        simulation::Region->insert( sound );
    }

    if( nodedata.layer != null_handle ) {
        // scenery opened for editing: keep count of what the scenery file contains
        // NOTE: node types which failed to load bail out earlier and aren't counted
        static std::unordered_map<std::string, scene::layer_item> const itemtypes {
            { "dynamic", scene::layer_item::vehicle },
            { "track", scene::layer_item::track },
            { "road", scene::layer_item::track },
            { "junction", scene::layer_item::track },
            { "crossing", scene::layer_item::track },
            { "spawn", scene::layer_item::track },
            { "despawn", scene::layer_item::track },
            { "crosswalk", scene::layer_item::track },
            { "traction", scene::layer_item::traction },
            { "tractionpowersource", scene::layer_item::powersource },
            { "model", scene::layer_item::model },
            { "triangles", scene::layer_item::shape },
            { "triangle_strip", scene::layer_item::shape },
            { "triangle_fan", scene::layer_item::shape },
            { "lines", scene::layer_item::lines },
            { "line_strip", scene::layer_item::lines },
            { "line_loop", scene::layer_item::lines },
            { "memcell", scene::layer_item::memcell },
            { "eventlauncher", scene::layer_item::launcher },
            { "sound", scene::layer_item::sound } };
        auto const lookup { itemtypes.find( nodedata.type ) };
        if( lookup != itemtypes.end() ) {
            scene::Layers.count(
                nodedata.layer,
                // models with negative minimum range are 3d terrain, converted to shapes
                ( lookup->second == scene::layer_item::model && nodedata.range_min < 0.0 ) ?
                    scene::layer_item::shape :
                    lookup->second );
        }
    }
}

// makes model instance defined by a scenery file a part of the simulation
void
state_serializer::insert_model( TAnimModel *Instance, cParser const &Input, std::size_t const Line, scene::source_span const &Span ) {

    if( Instance->Model() != nullptr ) {
        for( auto const &smokesource : Instance->Model()->smoke_sources() ) {
            Particles.insert(
                smokesource.first,
                Instance,
                smokesource.second );
        }
    }

    if( false == simulation::Instances.insert( Instance ) ) {
        ErrorLog( "Bad scenario: duplicate 3d model instance name \"" + Instance->name() + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( Line ) + ")" );
    }
    scene::Groups.insert( scene::Groups.handle(), Instance );
    simulation::Region->insert( Instance );
    scene::Layers.track( Instance, Span, Instance->Angles(), Instance->Scale() );
    // the lookup by uuid serves the scenery editor. it's filled only for a scenery opened for editing, as with a lot
    // of instances it takes more time than everything else done for an instance put together
    if( true == Global.editor_session ) {
        scene::Hierarchy[ Instance->uuid.to_string() ] = Instance;
    }
}

// loads a run of model instances straight from the text of the scenery file, starting with the instance whose node keyword
// was just read. the text is taken apart by a reader made for these definitions, with a few threads sharing the work; the
// instances are then created here one by one, in the order of the text. returns: true if any instances were loaded, false
// if the definition at hand has to go through the parser
bool
state_serializer::deserialize_models( cParser &Input, scene::scratch_data &Scratchpad, scene::node_data &Nodedata, std::size_t const Line, std::streamoff const Sourcebegin ) {

    auto const text { Input.remainingText() };
    if( true == text.empty() ) { return false; }

    auto const run { scene::read_model_entries( text, true ) };
    if( true == run.blocks.empty() ) { return false; }
    // NOTE: the parser moves on but the text stays where it is, the definitions refer to it
    Input.skipText( run.length, run.linebreaks, std::string_view { "endmodel" }.size() );
    auto const textbegin { Input.TokenEnd() - static_cast<std::streamoff>( run.length ) };

    Nodedata.type = "model";
    auto loaded { 0 };
    for( auto const &block : run.blocks ) {
        // instances of the same model with the same skin are set up after the first one of them
        std::vector<TAnimModel const *> twins( block.appearances.size(), nullptr );
        for( auto const &entry : block.entries ) {

            Nodedata.range_max = entry.range_max;
            Nodedata.range_min = entry.range_min;
            // the parser supplies the names in lower case, which goes only for the ascii letters
            Nodedata.name.assign( entry.name );
            for( auto &character : Nodedata.name ) {
                if( character >= 'A' && character <= 'Z' ) {
                    character = static_cast<char>( character - 'A' + 'a' );
                }
            }
            if( Nodedata.name == "none" ) { Nodedata.name.clear(); }

            // what follows is what deserialize_model() does
            auto *instance = new TAnimModel( Nodedata );
            instance->Angles( Scratchpad.location.rotation + glm::vec3 { 0.f, entry.angle, 0.f } );
            if( false == Scratchpad.location.scale.empty() ) {
                instance->Scale( Scratchpad.location.scale.top() );
            }
            auto const &appearance { block.appearances[ entry.appearance ] };
            auto &twin { twins[ entry.appearance ] };
            instance->Load(
                appearance.model, appearance.texture, twin,
                ( entry.has_angles ? &entry.angles : nullptr ),
                ( entry.has_scale ? &entry.scale : nullptr ),
                false == entry.notransition );
            if( twin == nullptr ) { twin = instance; }
            instance->location( transform( entry.location, Scratchpad ) );

            // location of the first definition is known from its node keyword, which went through the parser
            auto const first { loaded == 0 };
            insert_model(
                instance, Input,
                ( first ? Line : Line + entry.line ),
                { ( first ? Sourcebegin : textbegin + static_cast<std::streamoff>( entry.begin ) ), textbegin + static_cast<std::streamoff>( entry.end ) } );
            ++loaded;
        }
    }
    if( Nodedata.layer != null_handle ) {
        // scenery opened for editing: keep count of what the scenery file contains
        scene::Layers.count( Nodedata.layer, scene::layer_item::model, loaded );
    }

    return true;
}

namespace {

// passes the placement in effect to the layer bookkeeping of scenery opened for editing
void
sync_layer_context( scene::scratch_data const &Scratchpad ) {

    if( true == scene::Layers.empty() ) { return; }

    scene::Layers.context( {
        ( Scratchpad.location.offset.empty() ? glm::dvec3{ 0.0 } : Scratchpad.location.offset.top() ),
        Scratchpad.location.rotation,
        ( Scratchpad.location.scale.empty() ? glm::vec3{ 1.f } : Scratchpad.location.scale.top() ) } );
}

} // namespace

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
    sync_layer_context( Scratchpad );
}

void
state_serializer::deserialize_endorigin( cParser &Input, scene::scratch_data &Scratchpad ) {

    if( false == Scratchpad.location.offset.empty() ) {
        Scratchpad.location.offset.pop();
    }
    else {
        ErrorLog( "Bad origin: endorigin instruction with empty origin stack in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() - 1 ) + ")" );
    }
    sync_layer_context( Scratchpad );
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
    glm::vec3 const parent = Scratchpad.location.scale.empty() ? glm::vec3(1.0f) : Scratchpad.location.scale.top();
    Scratchpad.location.scale.emplace( factor * parent );
    sync_layer_context( Scratchpad );
}

void
state_serializer::deserialize_endscale( cParser &Input, scene::scratch_data &Scratchpad ) {

    if( false == Scratchpad.location.scale.empty() ) {
        Scratchpad.location.scale.pop();
    }
    else {
        ErrorLog( "Bad scale: endscale instruction with empty scale stack in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() - 1 ) + ")" );
    }
    sync_layer_context( Scratchpad );
}

void
state_serializer::deserialize_rotate( cParser &Input, scene::scratch_data &Scratchpad ) {

    Input.getTokens( 3 );
    Input
        >> Scratchpad.location.rotation.x
        >> Scratchpad.location.rotation.y
        >> Scratchpad.location.rotation.z;
    sync_layer_context( Scratchpad );
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

namespace {

// processes binary terrain file named by a terrain directive.
// a binary terrain file holds all static geometry of the scenery it was written for, and while one is in use
// the text definitions of that geometry are skipped, wherever they are. the two can't be mixed without some
// of the geometry showing up twice or not at all. thus the named files are used only if nothing was read from
// the text yet and every one of them is usable, each is loaded once, and if that can't be done the scenery
// falls back on the default binary file of the scenario, or on the text.
void
include_binary_terrain( std::string File, scene::scratch_data &Scratchpad ) {

    auto &binary { Scratchpad.binary };

    replace_slashes( File );
    if( false == binary.terrain_textfiles.empty() ) {
        // the scenery keeps its terrain in files of the current format, which don't mix with a file holding all of its geometry
        WriteLog( "Included SBT file: " + File + " ignored, the scenery uses terrain files" );
        return;
    }
    if( std::find( binary.terrain_files.begin(), binary.terrain_files.end(), File ) != binary.terrain_files.end() ) {
        WriteLog( "Included SBT file: " + File + " is already in use, ignored" );
        return;
    }

    if( false == simulation::Region->is_scene( File ) ) {
        // the file is missing, or of a type or version we can't read
        if( true == binary.terrain_files.empty() ) {
            // nothing changes: the default file if there's one, or the text otherwise, remains the source of the geometry
            WriteLog( "Included SBT file: " + File + " can't be used, ignored" );
        }
        else if( true == Scratchpad.initialized ) {
            // the other named files are loaded already, it's too late to give up on them
            ErrorLog( "Bad scenario: included SBT file \"" + File + "\" can't be used, the geometry it should provide will be missing" );
        }
        else if( true == binary.terrain_default ) {
            // the default file was made out of the complete text of this scenario, so it can stand in for the incomplete set
            binary.terrain_files.clear();
            binary.terrain_included = false;
            WriteLog( "Included SBT file: " + File + " can't be used, falling back on the default SBT" );
        }
        else if( ( binary.geometry_skipped == 0 ) && ( Global.file_binary_terrain_skipped == 0 ) ) {
            // none of the text definitions was skipped so far, so the whole scenery can still be read from the text
            binary.terrain_files.clear();
            binary.terrain_included = false;
            binary.terrain = false;
            Global.file_binary_terrain_state = false;
            WriteLog( "Included SBT file: " + File + " can't be used, the scenery will be loaded from the text files" );
        }
        else {
            ErrorLog( "Bad scenario: included SBT file \"" + File + "\" can't be used, the geometry it should provide will be missing" );
        }
        return;
    }

    if( binary.geometry_imported > 0 ) {
        // part of the geometry was read from the text already, and the binary file would bring it in for the second time
        WriteLog( "Included SBT file: " + File + " ignored, the scenery geometry is already being loaded from the text files" );
        return;
    }
    if( ( true == Scratchpad.initialized ) && ( true == binary.terrain ) && ( false == binary.terrain_included ) ) {
        // likewise if the default file was loaded by now
        WriteLog( "Included SBT file: " + File + " ignored, the scenery geometry is already loaded from the default SBT" );
        return;
    }

    binary.terrain_files.emplace_back( File );
    binary.terrain_included = true;
    binary.terrain = true;
    Global.file_binary_terrain_state = true;
    Scratchpad.terrain_name = File;
    WriteLog( "Included SBT file: " + File );
    if( true == Scratchpad.initialized ) {
        // past the initialization there's nothing to wait for
        simulation::Region->deserialize( File );
    }
}

// processes terrain file named by a terrain directive: static geometry kept as text (.txtf), with binary version (.btf)
// made out of it and loaded in its place. unlike the legacy binary terrain above, the binary file stands in
// only for the text file of the same name, and has no bearing on how the rest of the scenery is loaded.
// a file which is to be read as text is added to Includes, for the caller to pass to the parser
void
include_terrain_file( std::string File, std::string &Includes, scene::scratch_data &Scratchpad ) {

    auto &binary { Scratchpad.binary };

    replace_slashes( File );
    erase_extension( File );
    if( std::find( binary.terrain_textfiles.begin(), binary.terrain_textfiles.end(), File ) != binary.terrain_textfiles.end() ) {
        WriteLog( "Terrain file: " + File + " is already in use, ignored" );
        return;
    }
    binary.terrain_textfiles.emplace_back( File );
    // the editor converts the triangles of the terrain files to heightmap terrain, it has to know them and where they're placed
    scene::terrain_file::references().push_back( {
        File,
        ( Scratchpad.location.offset.empty() ? glm::dvec3( 0.0 ) : Scratchpad.location.offset.top() ),
        Scratchpad.location.rotation } );

    auto const textfile { Global.asCurrentSceneryPath + File + ".txtf" };
    auto const binaryfile { Global.asCurrentSceneryPath + File + ".btf" };
    auto const textpresent { FileExists( textfile ) };
    // binary file holds the geometry where the text alone puts it, it can't stand in for a file placed with an offset or rotation
    auto const relocated {
        ( false == Scratchpad.location.offset.empty() && Scratchpad.location.offset.top() != glm::dvec3( 0.0 ) )
     || ( Scratchpad.location.rotation != glm::vec3( 0.f ) ) };

    auto state { scene::terrain_file::state::text };
    if( true == relocated ) {
        if( false == textpresent ) {
            ErrorLog( "Bad scenario: terrain file \"" + File + "\" placed with an offset or rotation can be loaded only from the text, which is missing" );
            return;
        }
    }
    else if( ( false == textpresent )
          || ( ( false == Global.editor_session ) && ( true == Global.file_binary_terrain ) ) ) {
        // NOTE: scenery opened for editing works on its text files, the text is left as is also if binary terrain is turned off.
        // a binary file which is all there is gets loaded regardless
        state = scene::terrain_file::prepare( textfile, binaryfile );
    }

    switch( state ) {
        case scene::terrain_file::state::binary: {
            if( true == binary.terrain ) {
                // legacy binary terrain file is in use, expected to hold all static geometry of the scenery, this terrain likely included
                if( ( false == Scratchpad.initialized )
                 && ( binary.geometry_skipped == 0 )
                 && ( Global.file_binary_terrain_skipped == 0 ) ) {
                    // nothing was left out on account of that file yet, so the scenery can still do without it
                    binary.terrain = false;
                    binary.terrain_included = false;
                    binary.terrain_default = false;
                    binary.terrain_files.clear();
                    Global.file_binary_terrain_state = false;
                    WriteLog( "Terrain file: " + File + " in use, SBT of the scenery is ignored" );
                }
                else {
                    ErrorLog( "Bad scenario: terrain file \"" + File + "\" ignored, the scenery geometry already comes from an SBT file. Remove the SBT file to have the terrain file used" );
                    return;
                }
            }
            if( true == Scratchpad.initialized ) {
                scene::terrain_file::attach( binaryfile, *simulation::Region, false == Global.editor_session );
            }
            else {
                binary.terrain_binaryfiles.emplace_back( binaryfile );
            }
            Includes += scene::terrain_file::extra( binaryfile ) + ' ';
            break;
        }
        case scene::terrain_file::state::text: {
            // read the way any other scenery file is
            WriteLog( "Terrain file: " + File + " loaded as text" );
            Includes += "include \"" + textfile + "\" end ";
            break;
        }
        default: {
            ErrorLog( "Bad scenario: terrain file \"" + File + "\" not found" );
            break;
        }
    }
}

} // namespace

void
state_serializer::deserialize_reversed( cParser &Input, scene::scratch_data &Scratchpad ) {

    if( false == Scratchpad.trainset.is_open
     || false == Scratchpad.trainset.vehicles.empty() ) {
        ErrorLog( "Bad trainset: \"reversed\" has to follow the trainset header, ahead of its vehicles, in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() - 1 ) + ")" );
        return;
    }
    Scratchpad.trainset.reversed = true;
}

void 
state_serializer::deserialize_terrain(cParser &Input, scene::scratch_data &Scratchpad)
{
	// the directive names a terrain file, or a number of them
	// NOTE: the directive is read to its end first, as processing of a terrain file can leave content for the parser to go through next
	std::vector<std::string> files;
	std::string token;
	while (false == (token = Input.getToken<std::string>()).empty() && token != "endterrain")
	{
		files.emplace_back(token);
	}

	std::string includes; // terrain files to be read as text, in the order the directive names them
	for (auto const &file : files)
	{
		if (file.ends_with(".txtf") || file.ends_with(".btf"))
		{
			include_terrain_file(file, includes, Scratchpad);
		}
		else if (Global.file_binary_terrain && file.ends_with(".sbt"))
		{
			include_binary_terrain(file, Scratchpad);
		}
	}
	if (false == includes.empty())
	{
		Input.injectString(includes);
	}
}

void
state_serializer::deserialize_heightmapterrain(cParser &Input, scene::scratch_data &Scratchpad)
{
	// heightmap terrain, kept in terrain/<name>/ of the simulator. format:
	//   heightmap_terrain [name] endheightmap_terrain
	// without the name the terrain is named after the scenery file. the chunks are loaded around the camera in every mode
	std::string name;
	while( true ) {
		auto const token { Input.getToken<std::string>( false ) };
		if( token.empty() || token == "endheightmap_terrain" ) {
			break;
		}
		if( name.empty() ) {
			name = token;
		}
	}
	if( name.empty() ) {
		name = Global.SceneryFile;
		auto const slash { name.find_last_of( "/\\" ) };
		if( slash != std::string::npos ) {
			name.erase( 0, slash + 1 );
		}
		erase_extension( name );
	}
	scene::Layers.terrain_directive(true);
	if( false == name.empty() ) {
		EditorTerrain.open( name );
	}
}

void
state_serializer::deserialize_editorterrain(cParser &Input, scene::scratch_data &Scratchpad)
{
	// the chunk files of the first editor terrain were replaced with the heightmap terrain before they were used in sceneries
	skip_until(Input, "endeditorterrain");
	WriteLog("Bad scenario: obsolete \"editorterrain\" directive ignored, the editor terrain is \"heightmap_terrain\" now", logtype::generic);
}

void
state_serializer::deserialize_endtrainset( cParser &Input, scene::scratch_data &Scratchpad ) {

    if( false == Scratchpad.trainset.is_open
     || true == Scratchpad.trainset.vehicles.empty() ) {
        // not bloody likely but we better check for it just the same
        ErrorLog( "Bad trainset: empty trainset defined in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() - 1 ) + ")" );
        Scratchpad.trainset.is_open = false;
        return;
    }

    std::size_t vehicleindex { 0 };
    for( auto *vehicle : Scratchpad.trainset.vehicles ) {
        // go through list of vehicles in the trainset, coupling them together and checking for potential driver
        if( vehicle->Mechanik != nullptr
         && vehicle->Mechanik->primary() ) {
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
            if( Scratchpad.trainset.reversed ) {
                vehicle->AttachNext(
                    Scratchpad.trainset.vehicles[ vehicleindex - 1 ],
                    Scratchpad.trainset.couplings[ vehicleindex - 1 ] );
            }
            else {
                Scratchpad.trainset.vehicles[ vehicleindex - 1 ]->AttachNext(
                    vehicle,
                    Scratchpad.trainset.couplings[ vehicleindex - 1 ] );
            }
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
            Scratchpad.trainset.driver != nullptr ? light::redmarker_left | light::redmarker_right | light::rearendsignals : light::rearendsignals );
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
    auto offset = Scratchpad.location.offset.empty() ? glm::dvec3() : Scratchpad.location.offset.top();
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

    auto const pathname = Scratchpad.trainset.is_open ? Scratchpad.trainset.track : Input.getToken<std::string>();
    auto const offset { Input.getToken<double>( false ) };
    auto const drivertype { Input.getToken<std::string>() };
    auto const couplingdata = Scratchpad.trainset.is_open ? Input.getToken<std::string>() : "3";
    auto const velocity = Scratchpad.trainset.is_open ? Scratchpad.trainset.velocity : Input.getToken<float>(false);
    // extract coupling type and optional parameters
    auto const couplingdatawithparams = couplingdata.find( '.' );
    auto coupling = couplingdatawithparams != std::string::npos ? std::atoi(couplingdata.substr(0, couplingdatawithparams).c_str()) : std::atoi(couplingdata.c_str());
    if( coupling < 0 ) {
        // sprzęg zablokowany (pojazdy nierozłączalne przy manewrach)
        coupling = -coupling | coupling::permanent;
    }
    if( offset != -1.0
     && std::abs(offset) > 0.5 ) { // maksymalna odległość między sprzęgami - do przemyślenia
        // likwidacja sprzęgu, jeśli odległość zbyt duża - to powinno być uwzględniane w fizyce sprzęgów...
        coupling = coupling::faux; 
    }
    auto const params = couplingdatawithparams != std::string::npos ? couplingdata.substr(couplingdatawithparams + 1) : "";
    // load amount and type
    auto loadcount { Input.getToken<int>( false ) };
    auto loadtype = loadcount ? Input.getToken<std::string>() : "";
    if( loadtype == "enddynamic" ) {
        // idiotoodporność: ładunek bez podanego typu nie liczy się jako ładunek
        loadcount = 0;
        loadtype = "";
    }

    auto *path = Scratchpad.trainset.path != nullptr ? Scratchpad.trainset.path : simulation::Paths.find( pathname );
    if( path == nullptr ) {

        ErrorLog( "Bad scenario: vehicle \"" + Nodedata.name + "\" placed on nonexistent path \"" + pathname + "\" in file \"" + Input.Name() + "\" (line " + std::to_string( inputline ) + ")" );
        skip_until( Input, "enddynamic" );
        return nullptr;
    }

    auto const reversedset { Scratchpad.trainset.is_open && Scratchpad.trainset.reversed };
    if( false == reversedset
     && true == Scratchpad.trainset.vehicles.empty() // jeśli pierwszy pojazd,
     && false == path->m_events0.empty() // tor ma Event0
     && std::abs(velocity) <= 1.f // a skład stoi
     && Scratchpad.trainset.offset >= 0.0 // ale może nie sięgać na owy tor
     && Scratchpad.trainset.offset < 8.0 ) { // i raczej nie sięga
        // przesuwamy około pół EU07 dla wstecznej zgodności
        Scratchpad.trainset.offset = 8.0;
    }

    auto *vehicle = new TDynamicObject();

    auto const gap { offset == -1.0 ? 0.0 : offset };
    auto const turned { ( offset == -1.0 ) != reversedset };
    auto const expected { reversedset ? vehicle_length( datafolder, mmdfile ) : 0.0 };
    auto const length =
        vehicle->Init(
            Nodedata.name,
            datafolder, skinfile, mmdfile,
            path,
            reversedset ? Scratchpad.trainset.offset + gap + expected : Scratchpad.trainset.offset - gap,
            drivertype,
            velocity,
            Scratchpad.trainset.name,
            loadcount, loadtype,
            turned,
            params );

    if( length != 0.0 && reversedset ) {
        if( std::abs( length - expected ) > 0.01 ) {
            vehicle->place_on_track( path, Scratchpad.trainset.offset + gap + length, turned );
        }
        Scratchpad.trainset.offset += length;
    }
    if( length != 0.0 ) { // zero oznacza błąd
        // przesunięcie dla kolejnego, minus bo idziemy w stronę punktu 1
        if( false == reversedset ) {
            Scratchpad.trainset.offset -= length;
        }
        // automatically establish permanent connections for couplers which specify them in their definitions
        if( coupling != 0
         && vehicle->MoverParameters->Couplers[(offset == -1.0 ? end::front : end::rear)].AllowedFlag & coupling::permanent ) {
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
    while( false == token.empty()
        && token != Token ) {

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

	// heightmap terrain: the directive which opens its folder, so the scenery streams it on load (in every mode)
	if( EditorTerrain.active() ) {
		scmfile << "// heightmap terrain\nheightmap_terrain " << EditorTerrain.name() << " endheightmap_terrain\n";
	}

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

	// heightmap terrain: the directive which opens its folder, so the scenery streams it on load (in every mode)
	if( EditorTerrain.active() ) {
		scmfile << "// heightmap terrain\nheightmap_terrain " << EditorTerrain.name() << " endheightmap_terrain\n";
	}

	scmfile << "// modified objects\ninclude " << filename << "_export_dirty.scm\n";

	std::ofstream ctrfile { absfilename + ".ctr" };
	// mem cells
	ctrfile << "// memory cells\n";
	for( auto const *memorycell : Memory.sequence() ) {
		if( true == memorycell->is_exportable
		 && memorycell->group() == null_handle) {
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
		if( path == nullptr || path->m_editorremoved ) {
			continue;
		}
		if( path->m_road != nullptr ) {
			// lanes are generated anew from their road
			continue;
		}
		if( path->dirty() == Dirty && path->group() == null_handle ) {
			path->export_as_text( scmfile );
		}
	}
	// roads
	scmfile << "// roads\n";
	for( auto const *road : Roads.sequence() ) {
		if( road != nullptr && false == road->m_editorremoved && road->dirty() == Dirty && road->group() == null_handle ) {
			road->export_as_text( scmfile );
		}
	}
	for( auto const *junction : Junctions.sequence() ) {
		if( junction != nullptr && false == junction->m_editorremoved && junction->dirty() == Dirty && junction->group() == null_handle ) {
			junction->export_as_text( scmfile );
		}
	}
	for( auto const *point : Roadpoints.sequence() ) {
		if( point != nullptr && false == point->m_editorremoved && point->dirty() == Dirty && point->group() == null_handle ) {
			point->export_as_text( scmfile );
		}
	}
	for( auto const *sweep : Sweeps.sequence() ) {
		if( sweep != nullptr && false == sweep->m_editorremoved && sweep->dirty() == Dirty && sweep->group() == null_handle ) {
			sweep->export_as_text( scmfile );
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
	nodedata.layer = scene::Layers.active(); // null_handle unless the scenery was opened for editing

	scene::scratch_data scratch;

	TAnimModel *cloned = deserialize_model(parser, scratch, nodedata);

	if (!cloned)
		return nullptr;

	cloned->mark_dirty();
	cloned->location(position);
	simulation::Instances.insert(cloned);
	simulation::Region->insert(cloned);
	scene::Layers.count(cloned->layer(), scene::layer_item::model);

	return cloned;
}

std::vector<TDynamicObject *> state_serializer::insert_trainset(std::string const &Name, TTrack *Path, double const Offset, std::string const &Vehicles, bool const Reversed) {
	scene::scratch_data scratch;
	scratch.trainset.is_open = true;
	scratch.trainset.name = Name;
	scratch.trainset.track = Path != nullptr ? Path->name() : std::string{};
	scratch.trainset.path = Path;
	scratch.trainset.offset = static_cast<float>(Offset);
	scratch.trainset.reversed = Reversed;
	cParser parser(Vehicles, cParser::buffer_TEXT, Global.asCurrentSceneryPath, Global.bLoadTraction);
	auto token { parser.getToken<std::string>() };
	while (false == token.empty()) {
		if (token == "node") { deserialize_node(parser, scratch); }
		token = parser.getToken<std::string>();
	}
	auto const vehicles { scratch.trainset.vehicles };
	if (false == vehicles.empty()) { deserialize_endtrainset(parser, scratch); }
	return vehicles;
}

std::pair<int, int> state_serializer::preview_include(std::string const &Directive, scene::layer_context const &Context, scene::layer_handle Layer, scene::instance_handle Instance) {
	// statements which take more than a single token, with the tokens ending them
	static std::unordered_map<std::string, std::string> const nodeends {
	    { "dynamic", "enddynamic" }, { "track", "endtrack" }, { "road", "endroad" }, { "junction", "endjunction" }, { "crossing", "endcrossing" }, { "spawn", "endspawn" }, { "despawn", "enddespawn" },
	    { "crosswalk", "endcrosswalk" }, { "traction", "endtraction" }, { "tractionpowersource", "end" }, { "model", "endmodel" },
	    { "triangles", "endtri" }, { "triangle_strip", "endtri" }, { "triangle_fan", "endtri" }, { "lines", "endline" }, { "line_strip", "endline" }, { "line_loop", "endline" },
	    { "memcell", "endmemcell" }, { "eventlauncher", "end" }, { "sound", "endsound" } };
	static std::unordered_map<std::string, std::string> const statementends {
	    { "event", "endevent" }, { "trainset", "endtrainset" }, { "isolated", "endisolated" }, { "area", "endarea" }, { "assignment", "endassignment" },
	    { "atmo", "endatmo" }, { "camera", "endcamera" }, { "config", "endconfig" }, { "description", "enddescription" }, { "light", "endlight" },
	    { "sky", "endsky" }, { "test", "endtest" }, { "time", "endtime" }, { "terrain", "endterrain" }, { "heightmap_terrain", "endheightmap_terrain" }, { "editorterrain", "endeditorterrain" } };

	cParser parser(Directive, cParser::buffer_TEXT, Global.asCurrentSceneryPath, Global.bLoadTraction);
	// the template is processed with the placement its directive is going to be loaded with
	scene::scratch_data scratch;
	scratch.location.offset.emplace(Context.offset);
	scratch.location.rotation = Context.rotation;
	scratch.location.scale.emplace(Context.scale);

	auto created { 0 };
	auto skipped { 0 };
	auto token { parser.getToken<std::string>() };
	while (false == token.empty()) {
		if (token == "origin") { deserialize_origin(parser, scratch); }
		else if (token == "endorigin") { deserialize_endorigin(parser, scratch); }
		else if (token == "scale") { deserialize_scale(parser, scratch); }
		else if (token == "endscale") { deserialize_endscale(parser, scratch); }
		else if (token == "rotate") { deserialize_rotate(parser, scratch); }
		else if (token == "node") {
			scene::node_data nodedata;
			parser.getTokens(4);
			parser >> nodedata.range_max >> nodedata.range_min >> nodedata.name >> nodedata.type;
			if (nodedata.name == "none") { nodedata.name.clear(); }
			nodedata.layer = Layer;
			nodedata.instance = Instance;
			auto *instance { (nodedata.type == "model" && nodedata.range_min >= 0.0) ? deserialize_model(parser, scratch, nodedata) : nullptr };
			if (instance != nullptr) {
				// NOTE: unlike for a model loaded with the scenery, smoke sources of the model aren't set up.
				// they'd be left with a dangling owner when the model is replaced after a change of the include
				instance->m_preview = true;
				simulation::Instances.insert(instance);
				simulation::Region->insert(instance);
				scene::Hierarchy[instance->uuid.to_string()] = instance;
				scene::Layers.count(Layer, scene::layer_item::model);
				++created;
			}
			else if (nodedata.type != "model" || nodedata.range_min < 0.0) {
				// NOTE: a model which failed to load is consumed up to its end already
				auto const lookup { nodeends.find(nodedata.type) };
				if (lookup != nodeends.end()) { skip_until(parser, lookup->second); }
				++skipped;
			}
		}
		else if (token == "lua") {
			parser.getTokens(1, false);
			++skipped;
		}
		else {
			auto const lookup { statementends.find(token) };
			if (lookup != statementends.end()) {
				skip_until(parser, lookup->second);
				++skipped;
			}
			// anything else is a single token with no parameters, or something the loader wouldn't recognize either
		}
		token = parser.getToken<std::string>();
	}
	return { created, skipped };
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

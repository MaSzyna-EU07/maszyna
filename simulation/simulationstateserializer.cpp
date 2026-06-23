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
#include "scene/scenerybinary.h"
#include "rendering/particles.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "model/AnimModel.h"
#include "rendering/lightarray.h"
#include "world/TractionPower.h"
#include "application/application.h"
#include "rendering/renderer.h"
#include "utilities/Logs.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <limits>

namespace simulation {

namespace {
// camera-distance rings for nearest-first visual streaming: the squared outer radius of
// each ring. the visual pass is replayed once per ring (nearest first), building only the
// nodes whose squared distance to the camera falls in [inner, outer); the last ring is
// unbounded so every remaining node is built exactly once.
constexpr double RING_OUTER2[] = {
    500.0 * 500.0,
    1500.0 * 1500.0,
    4000.0 * 4000.0,
    std::numeric_limits<double>::infinity() };
constexpr int RING_COUNT { static_cast<int>( sizeof( RING_OUTER2 ) / sizeof( RING_OUTER2[ 0 ] ) ) };
inline int ring_lastindex() { return RING_COUNT - 1; }
inline double ring_min2( int const K ) { return ( K <= 0 ? 0.0 : RING_OUTER2[ K - 1 ] ); }
inline double ring_max2( int const K ) { return RING_OUTER2[ ( K < RING_COUNT ? K : RING_COUNT - 1 ) ]; }
} // anonymous namespace

std::shared_ptr<deserializer_state>
state_serializer::deserialize_begin( std::string const &Scenariofile ) {

    crashreport_add_info("scenario", Scenariofile);

    // TODO: move initialization to separate routine so we can reuse it
    SafeDelete( Region );
    Region = new scene::basic_region();

    simulation::State.init_scripting_interface();

    // open the scenario file. binary scenery twins (.scnb/.incb/.scmb) are handled
    // transparently inside cParser: if a twin exists it is replayed instead of the
    // text, otherwise the text is parsed and a twin compiled alongside it.
    std::shared_ptr<deserializer_state> state =
            std::make_shared<deserializer_state>( Scenariofile, cParser::buffer_FILE, Global.asCurrentSceneryPath, Global.bLoadTraction );
    state->scenariofile = Scenariofile;
    state->scratchpad.name = Scenariofile;
    // first pass loads infrastructure (tracks/traction/events/memcells/sounds + directives);
    // visual nodes are skipped by the reader and loaded in a second pass. this two-pass split
    // is only valid when the top-level file is itself a replayable twin, because the visual
    // pass is started via restartReplay() which needs a top-level reader. for a text/compile
    // load (no top twin) we MUST stay in a single 'all' pass and load everything at once;
    // otherwise visual nodes served by included twins (.incb) would be skipped in the infra
    // pass and never rebuilt (restartReplay returns false), and all those models go missing.
    if( true == state->input.isReplaying() ) {
        state->input.setReplayPass( scene::scenery_load_pass::infrastructure );
    }
    scene::Groups.create();

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
	            { "endtrainset", &state_serializer::deserialize_endtrainset } };

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

    // mirror the camera-ring state so deserialize_model()/deserialize_node() can ring-test
    // each node by distance: in the visual phase the twin is replayed once per ring and only
    // the nodes in the current ring are built, the rest skipped in O(1)
    m_ringactive = state->visualphase;
    if( true == m_ringactive ) {
        m_ringindex = state->ringindex;
        m_ringeye = state->ringeye;
        m_ringmin2 = ring_min2( state->ringindex );
        m_ringmax2 = ring_max2( state->ringindex );
    }

    // stateful directives that build objects/lists; on the visual (second) pass they are
    // skipped wholesale so their side effects (trainsets, events, cameras, ...) don't
    // duplicate. transform/group directives (origin/rotate/scale/group) and idempotent
    // setters are re-run, so deferred visual nodes get the correct placement.
    static std::unordered_map<std::string, std::string> const visualskip {
        { "trainset",    "endtrainset" },
        { "event",       "endevent" },
        { "camera",      "endcamera" },
        { "light",       "endlight" },
        { "description", "enddescription" },
        { "test",        "endtest" },
        { "sky",         "endsky" },
        { "time",        "endtime" },
        { "terrain",     "endterrain" },
    };

    // deserialize content from the provided input
	auto timelast { std::chrono::steady_clock::now() };
    std::string token { Input.getToken<std::string>() };
    while( false == token.empty() ) {

        if( state->visualphase ) {
            auto const skip = visualskip.find( token );
            if( skip != visualskip.end() ) {
                // consume the stateful directive without running its handler
                skip_until( Input, skip->second );
                token = Input.getToken<std::string>();
                continue;
            }
        }

		auto lookup = state->functionmap.find( token );
		if( lookup != state->functionmap.end() ) {
            lookup->second();
        }
        else {
            ErrorLog( "Bad scenario: unexpected token \"" + token + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() - 1 ) + ")" );
        }

		auto timenow = std::chrono::steady_clock::now();
        // small per-frame budget while streaming visuals in the driver (avoid stutter),
        // generous budget while the loading screen is up (infrastructure pass)
        auto const budget = ( state->visualphase ? 8 : 200 );
        if( std::chrono::duration_cast<std::chrono::milliseconds>( timenow - timelast ).count() >= budget ) {
            Application.set_progress( Input.getProgress(), Input.getFullProgress() );
			return true;
        }

        token = Input.getToken<std::string>();
    }

    if( false == Scratchpad.initialized ) {
        // manually perform scenario initialization
        deserialize_firstinit( Input, Scratchpad );
    }

    // helper: reset the transform stack before each replay pass. the directives
    // (origin/rotate/scale) are replayed in order, so resetting here reproduces the
    // single-pass placement exactly; without it an unbalanced origin left on the stack would
    // be applied again and shift every deferred visual node ("terrain dumped beside the tracks").
    auto const resettransform = [ &Scratchpad ]() {
        Scratchpad.location.offset = {};
        Scratchpad.location.scale = {};
        Scratchpad.location.rotation = glm::vec3{}; };

    // first (infrastructure) pass finished: the scenario is now playable (tracks, events,
    // signals, the player train are all loaded). hand control back so the loader can switch
    // to the driver; the visual nodes load progressively from the driver, replayed once per
    // camera-distance ring (nearest first). the camera is sampled once here so the ring
    // partition stays stable across passes. only possible when replaying a binary twin -- a
    // text/compile load did everything in one pass (restartReplay returns false).
    if( ( false == state->visualphase )
     && ( true == Input.restartReplay( scene::scenery_load_pass::visual ) ) ) {
        state->visualphase = true;
        state->ringindex = 0;
        state->ringeye = Global.pCamera.Pos;
        resettransform();
        WriteLog( "Progressive visual load: streaming deferred nodes nearest-camera first (" + std::to_string( RING_COUNT ) + " rings)" );
        return false; // infrastructure ready -> go to driver; visuals continue there
    }

    // a ring pass finished: advance to the next (farther) ring and replay again, until the
    // outermost ring has been built
    if( ( true == state->visualphase )
     && ( state->ringindex < ring_lastindex() )
     && ( true == Input.restartReplay( scene::scenery_load_pass::visual ) ) ) {
        ++state->ringindex;
        resettransform();
        return true; // more rings to build
    }

    scene::Groups.close();

	scene::Groups.update_map();
	Region->create_map_geometry();

	// all nodes (including visual model instances) are now loaded, so initialise the
	// events that bind to model instances; in a single-pass (text/compile) load nothing
	// was deferred, but InitEvents() still skipped them, so do it here too
	simulation::Events.InitInstanceEvents();

	// loading finished: flush the top-level scenario's binary twin now rather than
	// waiting for the parser to be destroyed (the loader keeps the state around)
	Input.flushBinaryTwin();
	// wait out any background twin writes (includes) so they are complete and logged
	// before we report the scenario as loaded
	scene::scenerybinary_wait_all();

	state->done = true;
	return false;
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
            // 3d terrain: convert the model's submodels into region shapes
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

        // explicit shapes have no single position to ring-test by, so build them only in the
        // nearest ring pass (ring 0) and skip them in O(1) on the farther passes
        if( ( true == m_ringactive ) && ( m_ringindex > 0 ) ) {
            if( false == Input.skipReplayNode() ) { skip_until( Input, "endtri" ); }
            return;
        }

        auto const skip {
            // crude way to detect fixed switch trackbed geometry
            ( ( true == Global.CreateSwitchTrackbeds )
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

        // see the triangles branch: explicit shapes build in ring 0 only
        if( ( true == m_ringactive ) && ( m_ringindex > 0 ) ) {
            if( false == Input.skipReplayNode() ) { skip_until( Input, "endline" ); }
            return;
        }

        simulation::Region->insert(
            scene::lines_node().import(
                Input, nodedata ),
            Scratchpad );
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
	// legacy directive; the SBT terrain blob has been retired and terrain now loads
	// as ordinary scenery content, so the block is simply consumed.
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

    // camera-ring visual streaming: build this model only if it falls in the ring currently
    // being streamed; otherwise skip the rest of its body in O(1) and let a later (farther)
    // ring pass pick it up. covers terrain models (range_min<0) too -- they also have X Y Z.
    if( true == m_ringactive ) {
        auto const world { transform( location, Scratchpad ) };
        auto const d { world - m_ringeye };
        auto const d2 { d.x * d.x + d.y * d.y + d.z * d.z };
        if( ( d2 < m_ringmin2 ) || ( d2 >= m_ringmax2 ) ) {
            if( false == Input.skipReplayNode() ) { skip_until( Input, "endmodel" ); }
            return nullptr;
        }
    }

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

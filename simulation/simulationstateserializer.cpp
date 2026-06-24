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
// camera-following visual streaming: visual nodes are built only for region sections within
// this radius (m) of the camera, and more are built as the camera moves into new sections.
// should comfortably cover the model render range so nothing visibly pops in at the edge.
constexpr double STREAM_RADIUS { 2000.0 };
// per-frame time budget (ms) the driver spends streaming visual nodes. larger = the
// surroundings fill in faster but the frame it runs on is longer; on a heavy scene (low fps)
// a too-small budget is a tiny duty cycle, so streaming a million flora instances drags.
constexpr int VISUAL_BUDGET_MS { 24 };

// --- load profiler: where the load time actually goes, so we optimise the real bottleneck ---
struct load_profile {
    std::unordered_map<std::string, double> typetime;     // seconds building each node type (inside deserialize_node)
    std::unordered_map<std::string, long>  typecount;     // how many of each node type
    std::unordered_map<std::string, double> dispatchtime; // seconds per top-level token (node/event/trainset/...)
    long   tokens { 0 };    // top-level getToken() dispatches (gauges the cParser scan cost)
    double finalize { 0 };  // create_map_geometry + InitInstanceEvents
    void reset() { typetime.clear(); typecount.clear(); dispatchtime.clear(); tokens = 0; finalize = 0; }
    static void dump( std::unordered_map<std::string, double> const &M, char const *Tag, std::unordered_map<std::string, long> const *C ) {
        std::vector<std::pair<std::string, double>> v( M.begin(), M.end() );
        std::sort( v.begin(), v.end(), []( auto const &a, auto const &b ) { return a.second > b.second; } );
        for( auto const &p : v ) {
            if( p.second < 0.05 ) { break; }
            WriteLog( std::string( "    " ) + Tag + " " + p.first + ": " + std::to_string( p.second ) + "s"
                + ( C ? "  x" + std::to_string( ( *const_cast<std::unordered_map<std::string, long>*>( C ) )[ p.first ] ) : "" ) );
        }
    }
    void log( char const *Phase ) {
        double dtotal = finalize; for( auto const &p : dispatchtime ) { dtotal += p.second; }
        WriteLog( "=== load profile [" + std::string( Phase ) + "]: dispatch " + std::to_string( dtotal )
            + "s, getToken " + std::to_string( tokens ) + ", finalize " + std::to_string( finalize ) + "s ===" );
        dump( dispatchtime, "[top]", nullptr );
        dump( typetime, "[node]", &typecount );
    }
};
load_profile g_profile;

// RAII: add the elapsed time to Acc on scope exit (handles deserialize_node's many returns)
struct scoped_accum {
    std::chrono::steady_clock::time_point t0 { std::chrono::steady_clock::now() };
    double &acc;
    explicit scoped_accum( double &Acc ) : acc( Acc ) {}
    ~scoped_accum() { acc += std::chrono::duration<double>( std::chrono::steady_clock::now() - t0 ).count(); }
};

// fills Tobuild with the region-section indices within STREAM_RADIUS of Eye that are not yet
// in Built. mirrors basic_region::section() indexing (clamped to the grid). returns count.
std::size_t wanted_sections( glm::dvec3 const &Eye, std::unordered_set<int> const &Built, std::unordered_set<int> &Tobuild ) {
    Tobuild.clear();
    int const N { scene::EU07_REGIONSIDESECTIONCOUNT };
    int const ccol { static_cast<int>( std::floor( Eye.x / scene::EU07_SECTIONSIZE + N / 2 ) ) };
    int const crow { static_cast<int>( std::floor( Eye.z / scene::EU07_SECTIONSIZE + N / 2 ) ) };
    int const span { static_cast<int>( std::ceil( STREAM_RADIUS / scene::EU07_SECTIONSIZE ) ) };
    for( int r = crow - span; r <= crow + span; ++r ) {
        for( int c = ccol - span; c <= ccol + span; ++c ) {
            int const idx { std::clamp( r, 0, N - 1 ) * N + std::clamp( c, 0, N - 1 ) };
            if( 0 == Built.count( idx ) ) { Tobuild.insert( idx ); }
        }
    }
    return Tobuild.size();
}
} // anonymous namespace

// region-section index enclosing a world position (row-major, clamped) -- matches
// basic_region::section() so a node buckets into the same section it inserts into.
int
state_serializer::section_index( glm::dvec3 const &World ) {
    int const N { scene::EU07_REGIONSIDESECTIONCOUNT };
    int const col { static_cast<int>( std::floor( World.x / scene::EU07_SECTIONSIZE + N / 2 ) ) };
    int const row { static_cast<int>( std::floor( World.z / scene::EU07_SECTIONSIZE + N / 2 ) ) };
    return std::clamp( row, 0, N - 1 ) * N + std::clamp( col, 0, N - 1 );
}

std::shared_ptr<deserializer_state>
state_serializer::deserialize_begin( std::string const &Scenariofile ) {

    crashreport_add_info("scenario", Scenariofile);
    cParser::clearInfraSkipCache(); // fresh per-load cache of pure-visual leaf includes

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

    // reset the transform stack before each replay pass. the directives (origin/rotate/scale)
    // are replayed in order, so resetting here reproduces the single-pass placement exactly;
    // without it an unbalanced origin left on the stack would be applied again and shift nodes.
    auto const resettransform = [ &Scratchpad ]() {
        // a well-formed pass ends with a balanced (empty) transform stack; a leftover means an
        // origin/scale was pushed but never popped -- e.g. a node whose binary marker span
        // over-ran its terminator and skipped the following endorigin. warn rather than let it
        // silently accumulate into the next pass.
        if( false == Scratchpad.location.offset.empty() ) {
            WriteLog( "Bad scenery: " + std::to_string( Scratchpad.location.offset.size() ) + " unbalanced origin(s) left on the stack at end of a load pass" );
        }
        Scratchpad.location.offset = {};
        Scratchpad.location.scale = {};
        Scratchpad.location.rotation = glm::vec3{}; };

    // mirror the visual-streaming state so deserialize_model()/deserialize_node() can decide
    // whether a node belongs to the section set being built this cycle (or, in ringall, build
    // everything). inactive (builds everything) outside the visual phase.
    m_ringactive = state->visualphase;
    if( true == m_ringactive ) {
        if( false == state->ringeye_valid ) {
            // the camera centre decides spawn-area-first streaming; the camera isn't positioned
            // during load (especially ghostview), so prefer the player vehicle, then the camera,
            // then the scenery's first camera directive. wait a few frames if nothing is ready.
            auto const iszero = []( glm::dvec3 const &V ) { return ( V.x == 0.0 ) && ( V.y == 0.0 ) && ( V.z == 0.0 ); };
            glm::dvec3 eye = Global.pCamera.Pos;
            char const *src = "camera";
            if( true == iszero( eye ) ) {
                auto *player = simulation::Vehicles.find( Global.local_start_vehicle );
                if( player != nullptr ) { eye = player->GetPosition(); src = "player vehicle"; }
            }
            if( ( true == iszero( eye ) ) && ( false == simulation::Vehicles.sequence().empty() ) ) {
                // no designated player (e.g. ghostview), but the scenery has consists -- centre on
                // the first one; it sits on the network, near where the action is.
                eye = simulation::Vehicles.sequence().front()->GetPosition();
                src = "first vehicle";
            }
            if( true == iszero( eye ) ) { eye = Global.FreeCameraInit[ 0 ]; src = "camera directive"; }
            if( ( true == iszero( eye ) ) && ( state->ringeye_waits < 120 ) ) {
                ++state->ringeye_waits;
                return true; // nothing positioned yet; try again next frame
            }
            state->ringeye = eye;
            state->ringeye_valid = true;
            // no spawn/camera to centre on (e.g. ghostview at the origin): camera-following is
            // meaningless, so build every visual node in one pass. otherwise stream by section.
            state->ringall = iszero( eye );
            state->sectionmode = ( false == state->ringall );
            WriteLog( std::string( "Progressive visual load: " )
                + ( state->ringall ?
                    "no camera centre -- building all visual nodes in one pass" :
                    "streaming sections within " + std::to_string( static_cast<int>( STREAM_RADIUS ) ) + "m of the camera (from " + src + ")" ) );
        }
        m_ringall = state->ringall;
        m_sectionmode = state->sectionmode;
        m_shapes_built = state->shapes_built;
        m_ringeye = state->ringeye;
        m_tobuild = &state->tobuild;

        // section streaming. the first pass replays the whole twin once, indexing every deferred
        // node under its section while building the spawn area. afterwards (state->indexed) the
        // sections the camera moves into are rebuilt by seeking straight to their nodes -- no more
        // whole-twin re-scans, which is what was tanking fps / dragging on a million-node scenery.
        if( true == state->sectionmode ) {
            if( true == state->indexed ) {
                if( true == state->tobuild.empty() ) {
                    if( 0 == wanted_sections( Global.pCamera.Pos, state->built, state->tobuild ) ) {
                        return true; // surroundings already built; stay alive for camera moves
                    }
                }
                m_rebuilding = true;
                m_state = state.get();
                auto streamstart { std::chrono::steady_clock::now() };
                while( false == state->tobuild.empty() ) {
                    int const sec = *state->tobuild.begin();
                    state->tobuild.erase( state->tobuild.begin() );
                    rebuild_section( *state, sec );
                    state->built.insert( sec );
                    if( std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - streamstart ).count() >= VISUAL_BUDGET_MS ) { break; }
                }
                m_rebuilding = false;
                return true;
            }
            // first pass: index every deferred node while building the spawn area
            m_indexing = true;
            m_state = state.get();
            if( false == state->pass_active ) {
                wanted_sections( Global.pCamera.Pos, state->built, state->tobuild );
                Input.restartReplay( scene::scenery_load_pass::visual );
                resettransform();
                state->pass_active = true;
            }
        }
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

    // deserialize content from the provided input. modest budget while streaming visuals in the
    // driver (rendering is live, so a big slice would tank fps), generous budget while the loading
    // screen is up (infrastructure pass, nothing rendering yet).
    int const budget { state->visualphase ? VISUAL_BUDGET_MS : 200 };
	auto timelast { std::chrono::steady_clock::now() };
    std::string token { Input.getToken<std::string>() };
    while( false == token.empty() ) {
        ++g_profile.tokens; // profile: gauge the cParser scan cost

        if( state->visualphase ) {
            auto const skip = visualskip.find( token );
            if( skip != visualskip.end() ) {
                // consume the stateful directive without running its handler
                skip_until( Input, skip->second );
                token = Input.getToken<std::string>();
                continue;
            }
            // fast section skip: a visual model node carries its local position in its v7
            // marker, so we can section-test it and drop it in O(1) -- without deserialize_node
            // decoding any of its tokens. this is what keeps the per-cycle replay cheap when a
            // scenery has a million flora instances. (shapes/older twins have no marker position;
            // they fall through to deserialize_node, which section-tests them itself.)
            if( ( true == m_sectionmode ) && ( token == "node" ) ) {
                double x, y, z, range;
                if( true == Input.currentNodePosition( x, y, z, range ) ) {
                    auto const world { transform( glm::dvec3{ x, y, z }, Scratchpad ) };
                    // a model visible from beyond the stream radius (large/unlimited range_max) is
                    // built once, up front, regardless of section -- otherwise it would vanish at
                    // distance. the rest are indexed and built when their section comes into range.
                    bool const eager { ( range < 0.0 ) || ( range > STREAM_RADIUS ) };
                    if( ( true == m_indexing ) && ( false == eager ) ) { capture_node( Input, Scratchpad, world ); }
                    bool const wanted {
                        eager ?
                            ( false == m_shapes_built ) :
                            ( 0 != m_tobuild->count( section_index( world ) ) ) };
                    if( ( false == wanted )
                     && ( true == Input.skipReplayNode() ) ) {
                        auto timenow = std::chrono::steady_clock::now();
                        if( std::chrono::duration_cast<std::chrono::milliseconds>( timenow - timelast ).count() >= budget ) {
                            Application.set_progress( Input.getProgress(), Input.getFullProgress() );
                            return true;
                        }
                        token = Input.getToken<std::string>();
                        continue;
                    }
                }
            }
        }

		auto lookup = state->functionmap.find( token );
		if( lookup != state->functionmap.end() ) {
            scoped_accum const dispatchguard { g_profile.dispatchtime[ token ] };
            lookup->second();
        }
        else {
            ErrorLog( "Bad scenario: unexpected token \"" + token + "\" defined in file \"" + Input.Name() + "\" (line " + std::to_string( Input.Line() - 1 ) + ")" );
        }

		auto timenow = std::chrono::steady_clock::now();
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

    // helper: make the scenario playable / persist the twin. the map, instance-bound events and
    // twin flush are done once, after the first cycle (or the single build-all pass). the active
    // group stack is left open on purpose in section mode -- later cycles keep inserting into it
    // (update_map reads the persistent group map, not the stack, so it works either way).
    auto const finalize = [ & ]( bool const Closegroups ) {
        scoped_accum const fg { g_profile.finalize };
        if( true == Closegroups ) { scene::Groups.close(); }
        scene::Groups.update_map();
        Region->create_map_geometry();
        simulation::Events.InitInstanceEvents();
        Input.flushBinaryTwin();
        scene::scenerybinary_wait_all(); };

    // first (infrastructure) pass finished: the scenario is now playable (tracks, events,
    // signals, the player train are all loaded). hand control back so the loader can switch to
    // the driver; the visual nodes stream in from the driver. the camera centre / mode are
    // resolved later, on the first driver pass (the camera isn't positioned here yet). only
    // possible when replaying a binary twin -- a text/compile load did everything in one pass.
    if( ( false == state->visualphase )
     && ( true == Input.restartReplay( scene::scenery_load_pass::visual ) ) ) {
        state->visualphase = true;
        resettransform();
        g_profile.log( "infrastructure" );
        g_profile.reset(); // measure the visual phase separately
        WriteLog( "Progressive visual load: infrastructure ready, streaming visuals from the driver" );
        return false; // infrastructure ready -> go to driver; visuals continue there
    }

    // section streaming: a build cycle's replay pass just finished. mark its sections built so
    // they aren't rebuilt, finalize once after the first cycle, and stay alive so the next call
    // can pick up sections the camera has since moved into. the load never reports "done".
    if( ( true == state->visualphase ) && ( true == state->sectionmode ) ) {
        if( true == state->pass_active ) {
            state->built.insert( std::begin( state->tobuild ), std::end( state->tobuild ) );
            state->tobuild.clear();
            state->pass_active = false;
            state->shapes_built = true; // explicit shapes + eager models are built in the first pass
            m_indexing = false;
            if( false == state->initial_done ) {
                finalize( /*Closegroups*/ false ); // keep groups open for later cycles
                state->initial_done = true;
                state->indexed = true; // the first pass has indexed every deferred node by section
                std::size_t refs = 0; for( auto const &s : state->index ) { refs += s.second.size(); }
                WriteLog( "Progressive visual load: spawn ready (" + std::to_string( simulation::Instances.sequence().size() )
                    + " instances), " + std::to_string( refs ) + " nodes indexed across " + std::to_string( state->index.size() ) + " sections" );
                g_profile.log( "visual first pass" );
                g_profile.reset();
            }
        }
        return true; // keep streaming alive; sections the camera enters are served from the index
    }

    // build-all (no camera centre, e.g. ghostview): everything was built in this single pass.
    finalize( /*Closegroups*/ true );
    state->done = true;
    g_profile.log( "visual build-all" );
    return false;
}

int
state_serializer::twin_id( deserializer_state &State, std::string const &File, std::string const &Path ) {
    std::string key = Path + "|" + File;
    auto const it = State.twinids.find( key );
    if( it != State.twinids.end() ) { return it->second; }
    int const id = static_cast<int>( State.twins.size() );
    State.twins.emplace_back( File, Path );
    State.twinids.emplace( std::move( key ), id );
    return id;
}

void
state_serializer::capture_node( cParser &Input, scene::scratch_data const &Scratchpad, glm::dvec3 const &World ) {
    // record where the node lives and the context it needs, so rebuild_section() can place it
    // identically later without re-scanning the twin. only small-range nodes are indexed; large/
    // unlimited-range ("eager") ones are built once up front and never streamed again.
    if( m_state == nullptr ) { return; }
    visual_ref ref;
    ref.twin = twin_id( *m_state, Input.currentReplayFile(), Input.currentReplayPath() );
    ref.offset = Input.currentReplayOffset();
    ref.has_offset = ( false == Scratchpad.location.offset.empty() );
    if( true == ref.has_offset ) { ref.t_offset = Scratchpad.location.offset.top(); }
    ref.has_scale = ( false == Scratchpad.location.scale.empty() );
    if( true == ref.has_scale ) { ref.t_scale = Scratchpad.location.scale.top(); }
    ref.t_rotation = Scratchpad.location.rotation;
    ref.params = Input.currentReplayParams();
    m_state->index[ section_index( World ) ].emplace_back( std::move( ref ) );
}

void
state_serializer::rebuild_section( deserializer_state &State, int Section ) {
    auto const it = State.index.find( Section );
    if( it == State.index.end() ) { return; }
    scene::scratch_data &Scratchpad = State.scratchpad;
    for( auto &ref : it->second ) {
        // reuse one parser per twin across the whole stream (re-opening an .inc is not free)
        auto pit = State.rebuild_parsers.find( ref.twin );
        if( pit == State.rebuild_parsers.end() ) {
            auto const &tw = State.twins[ ref.twin ];
            pit = State.rebuild_parsers.emplace(
                ref.twin,
                std::make_unique<cParser>( tw.first, cParser::buffer_FILE, tw.second, Global.bLoadTraction ) ).first;
        }
        cParser &cp = *pit->second;
        cp.seekReplayNode( ref.offset );
        cp.setReplayParams( ref.params );
        // restore the transform context captured for this node
        Scratchpad.location.offset = {};
        if( true == ref.has_offset ) { Scratchpad.location.offset.push( ref.t_offset ); }
        Scratchpad.location.scale = {};
        if( true == ref.has_scale ) { Scratchpad.location.scale.push( ref.t_scale ); }
        Scratchpad.location.rotation = ref.t_rotation;
        auto const tok = cp.getToken<std::string>();
        if( tok == "node" ) { deserialize_node( cp, Scratchpad ); }
    }
    // the section is built; release its index entries
    std::vector<visual_ref>().swap( it->second );
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
    // profile: attribute this node's build time to its type (see g_profile log at pass boundaries)
    scoped_accum const profileguard { g_profile.typetime[ nodedata.type ] };
    ++g_profile.typecount[ nodedata.type ];
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

        // origin-placed shapes (e.g. flora includes) carry their world position in the active
        // origin, so they section-stream like models: indexed on the first pass, rebuilt per
        // section after. absolute shapes (terrain, no origin) have no single position -> built
        // once in the first pass. (m_rebuilding: chosen from the index -> build unconditionally.)
        if( ( true == m_sectionmode ) && ( false == m_rebuilding ) && ( nullptr != m_tobuild ) ) {
            if( false == Scratchpad.location.offset.empty() ) {
                glm::dvec3 const world { Scratchpad.location.offset.top() };
                if( true == m_indexing ) { capture_node( Input, Scratchpad, world ); }
                if( 0 == m_tobuild->count( section_index( world ) ) ) {
                    if( false == Input.skipReplayNode() ) { skip_until( Input, "endtri" ); }
                    return;
                }
            }
            else if( true == m_shapes_built ) {
                if( false == Input.skipReplayNode() ) { skip_until( Input, "endtri" ); }
                return;
            }
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

        // see the triangles branch: origin-placed lines section-stream, absolute ones build once.
        if( ( true == m_sectionmode ) && ( false == m_rebuilding ) && ( nullptr != m_tobuild ) ) {
            if( false == Scratchpad.location.offset.empty() ) {
                glm::dvec3 const world { Scratchpad.location.offset.top() };
                if( true == m_indexing ) { capture_node( Input, Scratchpad, world ); }
                if( 0 == m_tobuild->count( section_index( world ) ) ) {
                    if( false == Input.skipReplayNode() ) { skip_until( Input, "endline" ); }
                    return;
                }
            }
            else if( true == m_shapes_built ) {
                if( false == Input.skipReplayNode() ) { skip_until( Input, "endline" ); }
                return;
            }
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

    // camera-following visual streaming: build this model only if its region section is in the
    // set being built this cycle; otherwise skip the rest of its body in O(1) and let a later
    // cycle (once the camera is near) pick it up. covers terrain models (range_min<0) too -- they
    // also have X Y Z. most out-of-range models are already dropped O(1) at the dispatch loop via
    // their v7 marker; this is the fallback for nodes that reached here (in-range, or no marker).
    // use the marker position when present so this decision matches the dispatch one exactly.
    // m_rebuilding: this node was chosen from the section index, so build it unconditionally.
    if( ( true == m_sectionmode ) && ( false == m_rebuilding ) && ( nullptr != m_tobuild ) ) {
        // models visible from beyond the stream radius build once (first cycle); the rest build
        // when their section is in range. mirrors the dispatch-loop fast path exactly.
        bool const eager { ( Nodedata.range_max < 0.0 ) || ( Nodedata.range_max > STREAM_RADIUS ) };
        bool wanted;
        if( true == eager ) {
            wanted = ( false == m_shapes_built );
        }
        else {
            glm::dvec3 modellocal { location };
            double mx, my, mz, mr;
            if( true == Input.currentNodePosition( mx, my, mz, mr ) ) { modellocal = glm::dvec3{ mx, my, mz }; }
            wanted = ( 0 != m_tobuild->count( section_index( transform( modellocal, Scratchpad ) ) ) );
        }
        if( false == wanted ) {
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

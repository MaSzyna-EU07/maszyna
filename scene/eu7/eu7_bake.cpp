/*

This Source Code Form is subject to the

terms of the Mozilla Public License, v.

2.0. If a copy of the MPL was not

distributed with this file, You can

obtain one at

http://mozilla.org/MPL/2.0/.

*/



#include "stdafx.h"

#include "scene/eu7/eu7_bake.h"



#include "scene/eu7/eu7_loader.h"

#include "utilities/Globals.h"

#include "utilities/Logs.h"

#include "utilities/utilities.h"



#include <thread>



#ifdef WITH_EU7_PARSER

#include "scene/eu7/eu7_bake_parser.h"

#endif



namespace scene::eu7 {

namespace {



[[nodiscard]] unsigned

bake_thread_count() {

    if( Global.eu7_bake_threads > 0 ) {

        return static_cast<unsigned>( Global.eu7_bake_threads );

    }

    auto const hardware { std::thread::hardware_concurrency() };

    return hardware > 0 ? hardware : 1u;

}



#ifdef WITH_EU7_PARSER

[[nodiscard]] bool

run_scenario_tree_bake(

    std::string const &text_root,

    std::string const &eu7_path,

    Eu7BakeOutcome &outcome ) {

    WriteLog(

        "EU7 bake: generowanie modulow z \"" + text_root + "\" (watek=" +

        std::to_string( bake_thread_count() ) + ")" );



    std::string error;

    if( false == bake_parser::bake_scenario_tree( text_root, bake_thread_count(), error ) ) {

        if( probe_file( eu7_path ) ) {

            ErrorLog(

                "EU7 bake nieudany, uzywam istniejacego modulu: " + error );

            outcome.ok = true;

            outcome.message = "fallback na stary .eu7 po bledzie bake";

            return true;

        }

        outcome.message = error;

        return false;

    }



    if( false == probe_file( eu7_path ) ) {

        outcome.message = "bake zakonczony, ale brak pliku: " + eu7_path;

        return false;

    }



    outcome.ok = true;

    outcome.regenerated = true;

    outcome.message = eu7_path;

    WriteLog( "EU7 bake: zapisano " + eu7_path );

    return true;

}

#endif



} // namespace



bool

scenario_needs_eu7_regen( std::string const &scenario_file ) {

#ifdef WITH_EU7_PARSER

    if( false == Global.eu7_auto_bake ) {

        return false;

    }



    auto const resolved { resolve_scenery_path( scenario_file ) };



    if( probe_file( resolved ) ) {

        return false;

    }



    if( false == is_text_module_extension( resolved ) || false == FileExists( resolved ) ) {

        return false;

    }

    return false == probe_baked_scenario( scenario_file );

#else

    (void)scenario_file;

    return false;

#endif

}



Eu7BakeOutcome

ensure_scenario_eu7( std::string const &scenario_file ) {

    Eu7BakeOutcome outcome;

    auto const resolved { resolve_scenery_path( scenario_file ) };

    auto const eu7_path { resolve_scenery_path( binary_path( scenario_file ) ) };



    if( probe_file( resolved ) ) {

        outcome.ok = true;

        outcome.message = "uzywam .eu7: " + resolved;

        WriteLog( "EU7: uzywam .eu7: " + resolved );

        return outcome;

    }



#ifdef WITH_EU7_PARSER

    if( false == Global.eu7_auto_bake ) {

        if( probe_baked_scenario( scenario_file ) ) {

            outcome.ok = true;

            outcome.message = "eu7_auto_bake wylaczone, uzywam istniejacego .eu7";

            return outcome;

        }

        outcome.ok = true;

        outcome.message = "eu7_auto_bake wylaczone, ladowanie SCM";

        return outcome;

    }



    if( false == is_text_module_extension( resolved ) ) {

        outcome.ok = true;

        outcome.message = "nie jest scenariuszem tekstowym";

        return outcome;

    }

    if( false == FileExists( resolved ) ) {

        outcome.message = "brak pliku scenariusza: " + resolved;

        return outcome;

    }



    if( probe_baked_scenario( scenario_file ) ) {

        outcome.ok = true;

        outcome.message = "uzywam istniejacego .eu7: " + eu7_path;

        WriteLog( "EU7: uzywam .eu7: " + eu7_path );

        return outcome;

    }



    run_scenario_tree_bake( resolved, eu7_path, outcome );

    return outcome;

#else

    if( probe_baked_scenario( scenario_file ) ) {

        outcome.ok = true;

        outcome.message = "uzywam istniejacego .eu7 (parser niedostepny w buildzie)";

        return outcome;

    }

    outcome.ok = true;

    outcome.message = "parser niedostepny, ladowanie SCM";

    return outcome;

#endif

}



} // namespace scene::eu7

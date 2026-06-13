/*

This Source Code Form is subject to the

terms of the Mozilla Public License, v.

2.0. If a copy of the MPL was not

distributed with this file, You can

obtain one at

http://mozilla.org/MPL/2.0/.

*/



#include <eu07/scene/bake/bake_tree.hpp>



#include <filesystem>

#include <mutex>

#include <string>



namespace scene::eu7::bake_parser {

namespace {



namespace fs = std::filesystem;



std::mutex g_progress_log_mutex;



void

log_bake_progress( eu07::scene::bake::BakeProgress const &progress ) {

    std::lock_guard<std::mutex> lock { g_progress_log_mutex };

    auto const path { progress.path.u8string() };

    switch( progress.phase ) {

    case eu07::scene::bake::BakeProgressPhase::Starting:

    case eu07::scene::bake::BakeProgressPhase::Start:

        break;

    case eu07::scene::bake::BakeProgressPhase::PackModels:

    case eu07::scene::bake::BakeProgressPhase::PackCompose:

    case eu07::scene::bake::BakeProgressPhase::PackComposeDone:

    case eu07::scene::bake::BakeProgressPhase::Bake:

    case eu07::scene::bake::BakeProgressPhase::Done:

        break;

    }

    (void)path;

}



[[nodiscard]] bool

is_text_scenery_path( fs::path const &path ) {

    auto const ext { path.extension().string() };

    return ext == ".scn" || ext == ".scm" || ext == ".inc" || ext == ".sbt";

}



} // namespace



bool

bake_scenario_tree(

    std::string const &text_scenario_path,

    unsigned const max_threads,

    std::string &error_out ) {

    error_out.clear();



    fs::path const input { text_scenario_path };

    if( false == fs::exists( input ) ) {

        error_out = "brak pliku: " + text_scenario_path;

        return false;

    }

    if( false == is_text_scenery_path( input ) ) {

        error_out = "nie jest plikiem SCM/SCN/INC: " + text_scenario_path;

        return false;

    }



    try {

        eu07::scene::bake::BakeTreeOptions options;

        options.maxThreads = max_threads;

        options.onProgress = log_bake_progress;



        eu07::scene::bake::BakeTreeStats stats;

        (void)eu07::scene::bake::bakeModuleTree( input, &stats, options );

        return true;

    }

    catch( std::exception const &ex ) {

        error_out = ex.what();

        return false;

    }

    catch( ... ) {

        error_out = "nieznany wyjatek bake";

        return false;

    }

}



} // namespace scene::eu7::bake_parser


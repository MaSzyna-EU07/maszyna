/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <sstream>
#include <string>
#include <vector>
#include "utilities/Globals_macros.h"

module eu07.scene.prefab;
import eu07.utilities.parser;
import eu07.utilities.logs;
import eu07.utilities.globals;

namespace scene {

namespace {

// a light pattern as a lights event writes it: one value per channel, each preceded by a
// space, channels the given values do not reach set to Fill
std::string
pattern( std::vector<int> const &Lights, std::size_t const Chambers, int const Fill = 0 ) {

    std::string result;
    for( std::size_t chamber = 0; chamber < Chambers; ++chamber ) {
        result += ' ' + std::to_string( chamber < Lights.size() ? Lights[ chamber ] : Fill );
    }
    return result;
}

void
parse_aspect( cParser &Input, prefab_definition const &Definition, prefab_aspect &Aspect ) {

    auto token { Input.getToken<std::string>() };
    while( ( false == token.empty() ) && ( token != "endaspect" ) ) {

        if( token == "lights" ) {
            for( std::size_t chamber = 0; chamber < Definition.chambers; ++chamber ) {
                Aspect.lights.emplace_back( Input.getToken<int>() );
            }
        }
        else if( token == "lightdelay" ) {
            Aspect.lightdelay = Input.getToken<std::string>();
        }
        else if( token == "velocity" ) {
            Aspect.velocity = Input.getToken<std::string>();
            Aspect.velocitynext = Input.getToken<std::string>();
        }
        else if( token == "animate" ) {
            prefab_animation animation;
            animation.kind = Input.getToken<std::string>();
            animation.submodel = Input.getToken<std::string>( false );
            animation.x = Input.getToken<std::string>();
            animation.y = Input.getToken<std::string>();
            animation.z = Input.getToken<std::string>();
            animation.speed = Input.getToken<std::string>();
            Aspect.animations.emplace_back( animation );
        }
        else {
            ErrorLog( "Prefab \"" + Definition.name + "\": unexpected \"" + token + "\" in aspect \"" + Aspect.name + "\"" );
        }
        token = Input.getToken<std::string>();
    }
}

} // anonymous namespace

bool
prefab_registry::load( std::string const &Name ) {

    auto const path { Global.asCurrentSceneryPath + Name + ".pfb" };
    cParser input( path, cParser::buffer_FILE );
    if( false == input.ok() ) {
        ErrorLog( "Prefab \"" + Name + "\" not found (" + path + ")" );
        return false;
    }

    prefab_definition definition;
    definition.name = Name;

    input.expectToken( "prefab" );
    auto token { input.getToken<std::string>() };
    while( ( false == token.empty() ) && ( token != "endprefab" ) ) {

        if( token == "model" ) {
            definition.model = input.getToken<std::string>( false );
        }
        else if( token == "modelangle" ) {
            definition.modelangle = input.getToken<std::string>();
        }
        else if( token == "attach" ) {
            prefab_attachment attachment;
            attachment.rangemax = input.getToken<std::string>();
            attachment.rangemin = input.getToken<std::string>();
            attachment.model = input.getToken<std::string>( false );
            attachment.texture = input.getToken<std::string>( false );
            definition.attachments.emplace_back( attachment );
        }
        else if( token == "chambers" ) {
            definition.chambers = input.getToken<int>();
        }
        else if( token == "initial" ) {
            definition.initial = input.getToken<std::string>();
        }
        else if( token == "initiallights" ) {
            for( std::size_t chamber = 0; chamber < definition.chambers; ++chamber ) {
                definition.initiallights.emplace_back( input.getToken<int>() );
            }
        }
        else if( token == "sound" ) {
            definition.sound = input.getToken<std::string>( false );
        }
        else if( token == "lightmodes" ) {
            definition.lightmodes = true;
        }
        else if( token == "aspect" ) {
            prefab_aspect aspect;
            aspect.name = input.getToken<std::string>();
            parse_aspect( input, definition, aspect );
            definition.aspects.emplace_back( aspect );
        }
        else {
            ErrorLog( "Prefab \"" + Name + "\": unexpected \"" + token + "\"" );
        }
        token = input.getToken<std::string>();
    }

    if( definition.chambers == 0 ) {
        // the chamber count has to come before the aspects, since it says how many light
        // values each of them reads. an aspect list with no lights in it means it did not
        ErrorLog( "Prefab \"" + Name + "\" declares no chambers" );
        return false;
    }

    m_definitions.emplace( Name, definition );
    WriteLog( "Prefab \"" + Name + "\": " + std::to_string( definition.aspects.size() ) + " aspects, " + std::to_string( definition.chambers ) + " chambers" );
    return true;
}

prefab_definition const *
prefab_registry::definition( std::string const &Name ) {

    auto const lookup { m_definitions.find( Name ) };
    if( lookup != m_definitions.end() ) {
        return &( lookup->second );
    }
    if( true == m_missing[ Name ] ) {
        return nullptr;
    }
    if( false == load( Name ) ) {
        m_missing[ Name ] = true;
        return nullptr;
    }
    return &( m_definitions.find( Name )->second );
}

std::string
prefab_registry::expand( std::string const &Name, prefab_placement const &Placement ) {

    auto const *definition { this->definition( Name ) };
    if( definition == nullptr ) {
        return {};
    }

    auto const &instance { Placement.name };
    auto const position {
        std::to_string( Placement.x ) + " "
        + std::to_string( Placement.y ) + " "
        + std::to_string( Placement.z ) };
    auto const texture { Placement.texture.empty() ? std::string( "none" ) : Placement.texture };

    // what the signal shows until something sets it: the node's own pattern if the
    // definition gives one, otherwise the initial aspect's, otherwise dark
    auto const *initiallights { &definition->initiallights };
    if( true == initiallights->empty() ) {
        for( auto const &aspect : definition->aspects ) {
            if( aspect.name == definition->initial ) {
                initiallights = &aspect.lights;
            }
        }
    }
    auto const initial { pattern( *initiallights, definition->chambers ) };

    std::ostringstream scenery;
    scenery
        << "origin " << position << "\n"
        << "rotate 0 " << Placement.yaw << " 0\n"
        << "node 1000 0 " << instance << " model 0 0 0 " << definition->modelangle << " "
        << definition->model << " " << texture << " lights" << initial << " endmodel\n";

    for( auto const &attachment : definition->attachments ) {
        scenery
            << "node " << attachment.rangemax << " " << attachment.rangemin
            << " none model 0 0 0 " << definition->modelangle << " " << attachment.model << " "
            << ( attachment.texture == "instance" ? texture : attachment.texture ) << " endmodel\n";
    }

    scenery
        << "rotate 0 0 0\n"
        << "endorigin\n"
        // the speed the driver reads off the signal. the aspect events write it, the track
        // event beside the signal reads it
        << "node -1 0 " << instance << "_sem_mem memcell " << position << " SetVelocity 0.0 0.0 none endmemcell\n"
        << "event " << instance << "_sem_info getvalues 0 " << instance << "_sem_mem endevent\n";

    if( false == definition->sound.empty() ) {
        scenery
            << "node 50 0 " << instance << "_dzwiek sound " << position << " " << definition->sound << " endsound\n"
            << "event " << instance << "_dzwiek sound 0.0 " << instance << "_dzwiek 1 endevent\n";
    }

    // one state event per aspect, and that is the whole public surface of the instance:
    // whatever sets the signal launches <name>_<aspect>, and the lights, speeds, animation
    // and sound behind it belong to the prefab
    for( auto const &aspect : definition->aspects ) {
        scenery
            << "event " << instance << "_" << aspect.name << " multiple 0 none "
            << instance << "_sem_ligh_" << aspect.name << " "
            << instance << "_sem_vel_" << aspect.name;
        if( false == aspect.animations.empty() ) {
            scenery << " " << instance << "_sem_anim_" << aspect.name;
        }
        if( false == definition->sound.empty() ) {
            scenery << " " << instance << "_dzwiek";
        }
        scenery
            << " endevent\n"
            << "event " << instance << "_sem_ligh_" << aspect.name << " lights " << aspect.lightdelay << " "
            << instance << pattern( aspect.lights, definition->chambers ) << " endevent\n"
            << "event " << instance << "_sem_vel_" << aspect.name << " updatevalues 0 "
            << instance << "_sem_mem SetVelocity " << aspect.velocity << " " << aspect.velocitynext << " endevent\n";

        if( false == aspect.animations.empty() ) {
            // the templates split this into groups of eight, which a multi_event has not
            // needed for a long time - it holds a vector. one list, one event
            scenery << "event " << instance << "_sem_anim_" << aspect.name << " multiple 0 none";
            for( std::size_t channel = 0; channel < aspect.animations.size(); ++channel ) {
                scenery << " " << instance << "_sem_anim_" << aspect.name << "_" << channel;
            }
            scenery << " endevent\n";
            std::size_t channel { 0 };
            for( auto const &animation : aspect.animations ) {
                scenery
                    << "event " << instance << "_sem_anim_" << aspect.name << "_" << channel
                    << " animation 0 " << instance << " " << animation.kind << " " << animation.submodel << " "
                    << animation.x << " " << animation.y << " " << animation.z << " " << animation.speed << " endevent\n";
                ++channel;
            }
        }
    }

    // a failed signal is dark and stops what comes at it
    scenery
        << "event " << instance << "_uszk multiple 0 none "
        << instance << "_sem_ligh0 " << instance << "_sem_vel_stop endevent\n"
        << "event " << instance << "_sem_ligh0 lights 0.0 " << instance << pattern( {}, definition->chambers ) << " endevent\n"
        << "event " << instance << "_sem_vel_stop updatevalues 0 "
        << instance << "_sem_mem SetVelocity 0.0 0.0 endevent\n";

    if( true == definition->lightmodes ) {
        // channel zero picks day, night or automatic lighting; the rest is left alone
        auto const untouched { pattern( {}, definition->chambers > 0 ? definition->chambers - 1 : 0, -1 ) };
        scenery
            << "event " << instance << "_dzienny lights 0.0 " << instance << " 0" << untouched << " endevent\n"
            << "event " << instance << "_nocny lights 0.0 " << instance << " 1" << untouched << " endevent\n"
            << "event " << instance << "_auto lights 0.0 " << instance << " 3" << untouched << " endevent\n";
    }

    return scenery.str();
}

} // namespace scene

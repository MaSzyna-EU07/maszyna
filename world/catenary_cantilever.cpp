#include "world/catenary_cantilever.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <locale>
#include <set>
#include <sstream>

namespace catenary {

namespace {

bool to_number( std::string const &Text, double &Value ) {
    std::istringstream stream { Text };
    stream.imbue( std::locale::classic() );
    stream >> Value;
    return false == stream.fail() && stream.eof();
}

std::string to_text( double const Value, int const Precision = 3 ) {
    std::ostringstream stream;
    stream.imbue( std::locale::classic() );
    stream.setf( std::ios::fixed );
    stream.precision( Precision );
    stream << Value;
    return stream.str();
}

// wektor w płaszczyźnie wysięgnika
struct vec2 {
    double y { 0.0 };
    double z { 0.0 };
};
vec2 operator+( vec2 const &Left, vec2 const &Right ) { return { Left.y + Right.y, Left.z + Right.z }; }
vec2 operator-( vec2 const &Left, vec2 const &Right ) { return { Left.y - Right.y, Left.z - Right.z }; }
vec2 operator*( vec2 const &Vector, double const Factor ) { return { Vector.y * Factor, Vector.z * Factor }; }
double dot( vec2 const &Left, vec2 const &Right ) { return Left.y * Right.y + Left.z * Right.z; }
double length( vec2 const &Vector ) { return std::sqrt( dot( Vector, Vector ) ); }
vec2 unit( vec2 const &Vector ) { auto const size { length( Vector ) }; return ( size > 1e-9 ? Vector * ( 1.0 / size ) : vec2 { -1.0, 0.0 } ); }
// prostopadła skierowana w górę
vec2 upward_normal( vec2 const &Direction ) {
    vec2 const normal { Direction.z, -Direction.y };
    return ( normal.z >= 0.0 ? normal : normal * -1.0 );
}
// obrót w płaszczyźnie wysięgnika, wokół osi X
vec2 rotate( vec2 const &Vector, double const Angle ) {
    auto const cosine { std::cos( Angle ) };
    auto const sine { std::sin( Angle ) };
    return { Vector.y * cosine - Vector.z * sine, Vector.y * sine + Vector.z * cosine };
}
vec2 to_vec( cantilever_point const &Point ) { return { Point.y, Point.z }; }

// kierunki spoczynkowe części: tak część jest ułożona w swoim modelu
vec2 const toward_track { -1.0, 0.0 };
vec2 const toward_support { 1.0, 0.0 };
vec2 const downward { 0.0, -1.0 };
vec2 const upward { 0.0, 1.0 };

// część leżąca w płaszczyźnie wysięgnika: zero w podanym punkcie, obrócona wokół osi X (wzdłuż toru) tak,
// żeby kierunek spoczynkowy Rest z modelu pokrył się z kierunkiem Direction
cantilever_placement place( std::string const &Role, std::string const &Part, vec2 const &Origin, vec2 const &Direction, vec2 const &Rest ) {
    cantilever_placement placement;
    placement.role = Role;
    placement.part = Part;
    placement.position[ 1 ] = Origin.y;
    placement.position[ 2 ] = Origin.z;
    auto const angle { std::atan2( Direction.z, Direction.y ) - std::atan2( Rest.z, Rest.y ) };
    auto const cosine { std::cos( angle ) };
    auto const sine { std::sin( angle ) };
    placement.rotation[ 4 ] = cosine; placement.rotation[ 5 ] = -sine;
    placement.rotation[ 7 ] = sine;   placement.rotation[ 8 ] = cosine;
    return placement;
}

// obrót części o 90 stopni wokół jej własnej osi podłużnej (lokalnej Y), po ustawieniu w płaszczyźnie wysięgnika
void roll_quarter( cantilever_placement &Placement ) {
    auto const cosine { Placement.rotation[ 4 ] };
    auto const sine { Placement.rotation[ 7 ] };
    // kolumny: X = -( dawne Z ), Y bez zmian, Z = dawne X
    double const rolled[ 9 ] { 0.0, 0.0, 1.0,  sine, cosine, 0.0,  -cosine, sine, 0.0 };
    std::copy( std::begin( rolled ), std::end( rolled ), std::begin( Placement.rotation ) );
}

std::set<std::string> const block_ends { "endpart", "endregistration", "endscheme", "endrules", "endpole", "endmount", "endassembly" };
std::set<std::string> const block_starts { "part", "registration", "scheme", "rules", "models", "pole", "mount", "assembly" };

// opis argumentów klocka sposobu składania; nullptr dla nieznanego
std::string const *step_signature( std::string const &Op );
// sprawdzenie zapisu argumentów klocka
bool valid_step( std::string const &Op, std::vector<std::string> const &Words, std::string &Reason );

} // namespace

void cantilever_catalog::parse( std::vector<std::string> const &Tokens, std::string const &Source, std::vector<std::string> &Errors ) {

    auto const count { Tokens.size() };
    std::size_t index { 0 };
    auto const report = [ & ]( std::string const &Entry, std::string const &Reason ) {
        Errors.push_back( "Bad cantilever file \"" + Source + "\"" + ( Entry.empty() ? "" : ", " + Entry ) + ": " + Reason );
    };
    // kolejna liczba wpisu; brak albo zły zapis psują wpis
    auto const number = [ & ]( double &Value, bool &Good ) {
        if( index >= count || false == to_number( Tokens[ index ], Value ) ) { Good = false; return; }
        ++index;
    };
    // pominięcie reszty zepsutego wpisu
    auto const skip = [ & ]() {
        while( index < count && block_ends.find( Tokens[ index ] ) == block_ends.end() && block_starts.find( Tokens[ index ] ) == block_starts.end() ) { ++index; }
        if( index < count && block_ends.find( Tokens[ index ] ) != block_ends.end() ) { ++index; }
    };

    while( index < count ) {
        auto const token { Tokens[ index++ ] };

        if( token == "part" ) {
            if( index >= count ) { report( "", "\"part\" without a name" ); break; }
            cantilever_part part;
            part.name = Tokens[ index++ ];
            auto const entry { "part \"" + part.name + "\"" };
            auto good { true };
            auto closed { false };
            while( index < count && good ) {
                auto const key { Tokens[ index++ ] };
                if( key == "endpart" ) { closed = true; break; }
                if( key == "kind" ) { if( index < count ) { part.kind = Tokens[ index++ ]; } else { good = false; } }
                else if( key == "length" ) { number( part.length, good ); }
                else if( key == "clearance" ) { number( part.clearance, good ); }
                else if( key == "pin" ) { number( part.pin, good ); }
                else if( key == "radius" ) { number( part.radius, good ); }
                else if( key == "model" ) { if( index < count ) { part.model = Tokens[ index++ ]; } else { good = false; } }
                else if( key == "girth" ) { number( part.girth, good ); }
                else if( key == "eye" ) { number( part.eye, good ); }
                else if( key == "hook" ) { number( part.hook[ 0 ], good ); number( part.hook[ 1 ], good ); }
                else if( key == "end" ) { number( part.end[ 0 ], good ); number( part.end[ 1 ], good ); }
                else if( key == "point" ) {
                    if( index < count ) {
                        auto const name { Tokens[ index++ ] };
                        double y { 0.0 }, z { 0.0 };
                        number( y, good ); number( z, good );
                        part.points[ name ] = { y, z };
                    }
                    else { good = false; }
                }
                else if( key == "pointx" ) {
                    // składowa X punktu (wzdłuż toru): pointx <nazwa> <x>
                    if( index < count ) {
                        auto const name { Tokens[ index++ ] };
                        double x { 0.0 };
                        number( x, good );
                        part.points_x[ name ] = x;
                    }
                    else { good = false; }
                }
                else if( key == "member" ) {
                    cantilever_member member;
                    if( index < count ) { member.part = Tokens[ index++ ]; } else { good = false; }
                    for( auto &value : member.position ) { number( value, good ); }
                    number( member.yaw, good );
                    if( good ) { part.members.push_back( member ); }
                }
                else if( key == "holes" ) {
                    double value { 0.0 };
                    while( index < count && to_number( Tokens[ index ], value ) ) { part.holes.push_back( value ); ++index; }
                    if( part.holes.empty() ) { good = false; }
                }
                else { report( entry, "unknown keyword \"" + key + "\"" ); good = false; --index; }
            }
            if( false == good || false == closed ) {
                if( good ) { report( entry, "missing \"endpart\"" ); } else if( Errors.empty() || Errors.back().find( entry ) == std::string::npos ) { report( entry, "bad or missing value" ); }
                skip();
                continue;
            }
            if( m_parts.find( part.name ) != m_parts.end() ) { report( entry, "defined more than once, the first definition stays" ); continue; }
            m_parts.emplace( part.name, part );
            continue;
        }

        if( token == "registration" ) {
            if( index >= count ) { report( "", "\"registration\" without a name" ); break; }
            cantilever_registration registration;
            registration.name = Tokens[ index++ ];
            auto const entry { "registration \"" + registration.name + "\"" };
            auto good { true };
            auto closed { false };
            while( index < count && good ) {
                auto const key { Tokens[ index++ ] };
                if( key == "endregistration" ) { closed = true; break; }
                if( key == "tube" ) { number( registration.tube, good ); }
                else if( key == "clamp" ) { number( registration.clamp, good ); }
                else if( key == "item" ) {
                    cantilever_item item;
                    if( index < count ) { item.part = Tokens[ index++ ]; } else { good = false; }
                    for( auto &value : item.position ) { number( value, good ); }
                    for( auto &value : item.rotation ) { number( value, good ); }
                    if( good ) { registration.items.push_back( item ); }
                }
                else { report( entry, "unknown keyword \"" + key + "\"" ); good = false; --index; }
            }
            if( false == good || false == closed ) {
                if( good ) { report( entry, "missing \"endregistration\"" ); } else if( Errors.empty() || Errors.back().find( entry ) == std::string::npos ) { report( entry, "bad or missing value" ); }
                skip();
                continue;
            }
            if( m_registrations.find( registration.name ) != m_registrations.end() ) { report( entry, "defined more than once, the first definition stays" ); continue; }
            m_registrations.emplace( registration.name, registration );
            continue;
        }

        if( token == "scheme" ) {
            cantilever_scheme scheme;
            auto good { true };
            double card { 0.0 }, execution { 0.0 };
            auto const name { ( index < count ? Tokens[ index ] : std::string() ) + ( index + 1 < count ? " " + Tokens[ index + 1 ] : std::string() ) };
            auto const entry { "scheme \"" + name + "\"" };
            number( card, good );
            number( execution, good );
            scheme.card = static_cast<int>( card );
            scheme.execution = static_cast<int>( execution );
            auto closed { false };
            while( index < count && good ) {
                auto const key { Tokens[ index++ ] };
                if( key == "endscheme" ) { closed = true; break; }
                if( key == "reach" ) { number( scheme.reach_min, good ); number( scheme.reach_max, good ); }
                else if( key == "height" ) { number( scheme.height, good ); }
                else if( key == "registration" ) { if( index < count ) { scheme.registration = Tokens[ index++ ]; } else { good = false; } }
                else if( key == "assembly" ) { if( index < count ) { scheme.assembly = Tokens[ index++ ]; } else { good = false; } }
                else if( key == "tube_steps" ) {
                    double value { 0.0 };
                    while( index < count && to_number( Tokens[ index ], value ) ) { scheme.tube_steps.push_back( value ); ++index; }
                    if( scheme.tube_steps.empty() ) { good = false; }
                    std::sort( scheme.tube_steps.begin(), scheme.tube_steps.end() );
                }
                else if( key == "tube_prefer" ) { number( scheme.tube_prefer, good ); }
                else if( key == "mount" ) {
                    auto const which { index < count ? Tokens[ index++ ] : std::string() };
                    if( which == "stay" ) { number( scheme.mount_stay, good ); }
                    else if( which == "strut" ) { number( scheme.mount_strut, good ); }
                    else if( which == "steady" ) { number( scheme.mount_steady, good ); scheme.steady_mounted = true; }
                    else { report( entry, "unknown mount \"" + which + "\"" ); good = false; }
                }
                else if( key == "role" ) {
                    if( index + 1 < count ) { scheme.roles[ Tokens[ index ] ] = Tokens[ index + 1 ]; index += 2; } else { good = false; }
                }
                else { report( entry, "unknown keyword \"" + key + "\"" ); good = false; --index; }
            }
            if( false == good || false == closed ) {
                if( good ) { report( entry, "missing \"endscheme\"" ); } else if( Errors.empty() || Errors.back().find( entry ) == std::string::npos ) { report( entry, "bad or missing value" ); }
                skip();
                continue;
            }
            auto const key { std::make_pair( scheme.card, scheme.execution ) };
            if( m_schemes.find( key ) != m_schemes.end() ) { report( entry, "defined more than once, the first definition stays" ); continue; }
            m_schemes.emplace( key, scheme );
            continue;
        }

        if( token == "pole" || token == "mount" ) {
            auto const ispole { token == "pole" };
            if( index >= count ) { report( "", "\"" + token + "\" without a name" ); break; }
            cantilever_pole pole;
            cantilever_mount mount;
            auto const name { Tokens[ index++ ] };
            pole.name = name;
            mount.name = name;
            auto const entry { token + " \"" + name + "\"" };
            auto const ending { ispole ? std::string( "endpole" ) : std::string( "endmount" ) };
            auto good { true };
            auto closed { false };
            auto const text = [ & ]( std::string &Value ) { if( index < count ) { Value = Tokens[ index++ ]; } else { good = false; } };
            while( index < count && good ) {
                auto const key { Tokens[ index++ ] };
                if( key == ending ) { closed = true; break; }
                if( ispole ) {
                    if( key == "model" ) { text( pole.model ); std::replace( pole.model.begin(), pole.model.end(), '\\', '/' ); }
                    else if( key == "foundation" ) { text( pole.foundation ); std::replace( pole.foundation.begin(), pole.foundation.end(), '\\', '/' ); }
                    else if( key == "top" ) { number( pole.top, good ); }
                    else if( key == "length" ) { number( pole.length, good ); }
                    else if( key == "depth" ) { number( pole.depth_top, good ); number( pole.depth_base, good ); }
                    else if( key == "mount_max" ) { number( pole.mount_max, good ); }
                    else if( key == "mount_min" ) { number( pole.mount_min, good ); pole.has_mount_min = true; }
                    else if( key == "turn" ) { number( pole.turn, good ); }
                    else if( key == "drop" ) { text( pole.drop ); }
                    else if( key == "strut" ) { text( pole.strut ); }
                    else if( key == "stay" ) { text( pole.stay ); }
                    else if( key == "steady" ) { text( pole.steady ); }
                    else { report( entry, "unknown keyword \"" + key + "\"" ); good = false; --index; }
                }
                else {
                    if( key == "back" ) { text( mount.back ); }
                    else if( key == "hinge" ) { text( mount.hinge ); number( mount.hinge_eye[ 0 ], good ); number( mount.hinge_eye[ 1 ], good ); }
                    else if( key == "rod" ) { text( mount.rod ); }
                    else if( key == "twin" ) { text( mount.twin ); }
                    else if( key == "axis" || key == "eye" ) { number( mount.point[ 0 ], good ); number( mount.point[ 1 ], good ); }
                    else { report( entry, "unknown keyword \"" + key + "\"" ); good = false; --index; }
                }
            }
            if( good && closed && ispole && ( pole.length <= 0.0 || pole.depth_top <= 0.0 || pole.depth_base <= 0.0 ) ) {
                report( entry, "needs \"length\" and both \"depth\" values above zero" ); continue;
            }
            if( false == good || false == closed ) {
                if( good ) { report( entry, "missing \"" + ending + "\"" ); } else if( Errors.empty() || Errors.back().find( entry ) == std::string::npos ) { report( entry, "bad or missing value" ); }
                skip();
                continue;
            }
            if( ispole ) {
                if( m_poles.find( name ) != m_poles.end() ) { report( entry, "defined more than once, the first definition stays" ); continue; }
                m_poles.emplace( name, pole );
            }
            else {
                if( m_mounts.find( name ) != m_mounts.end() ) { report( entry, "defined more than once, the first definition stays" ); continue; }
                m_mounts.emplace( name, mount );
            }
            continue;
        }

        if( token == "mast" || token == "beam" ) {
            // konstrukcje bramek: mast <nazwa> model … endmast, beam <nazwa> model … endbeam
            auto const ismast { token == "mast" };
            if( index >= count ) { report( "", "\"" + token + "\" without a name" ); break; }
            gantry_mast mast;
            gantry_beam beam;
            auto const name { Tokens[ index++ ] };
            mast.name = beam.name = name;
            auto const entry { token + " \"" + name + "\"" };
            auto const ending { ismast ? std::string( "endmast" ) : std::string( "endbeam" ) };
            auto good { true };
            auto closed { false };
            auto const text = [ & ]( std::string &Value ) { if( index < count ) { Value = Tokens[ index++ ]; } else { good = false; } };
            while( index < count && good ) {
                auto const key { Tokens[ index++ ] };
                if( key == ending ) { closed = true; break; }
                if( key == "model" ) { text( ismast ? mast.model : beam.model ); auto &model { ismast ? mast.model : beam.model }; std::replace( model.begin(), model.end(), '\\', '/' ); }
                else if( ismast && key == "width" ) { number( mast.width, good ); }
                else if( ismast && key == "length" ) { number( mast.length, good ); }
                else if( ismast && key == "height" ) { number( mast.height, good ); }
                else if( ismast && key == "mount" ) { number( mast.mount_low, good ); number( mast.mount_high, good ); }
                else if( ismast && key == "offset" ) { number( mast.offset, good ); }
                else if( ismast && key == "through" ) { mast.through = true; }
                else if( ismast && key == "end" ) { mast.through = false; }
                else if( ismast && key == "strut" ) { text( mast.strut ); }
                else if( ismast && key == "stay" ) { text( mast.stay ); }
                else if( ismast && key == "steady" ) { text( mast.steady ); }
                else if( ismast && key == "below_beam" ) { number( mast.below_beam, good ); }
                else if( false == ismast && key == "span" ) { number( beam.span, good ); }
                else if( false == ismast && key == "drop" ) { text( beam.drop ); }
                else if( false == ismast && key == "single" ) { beam.single = true; }
                else { report( entry, "unknown keyword \"" + key + "\"" ); good = false; --index; }
            }
            if( false == good || false == closed ) {
                if( good ) { report( entry, "missing \"" + ending + "\"" ); } else { report( entry, "bad or missing value" ); }
                skip();
                continue;
            }
            if( ismast ) {
                if( mast.model.empty() || mast.mount_low <= 0.0 ) { report( entry, "needs \"model\" and \"mount\"" ); continue; }
                if( false == m_masts.emplace( name, mast ).second ) { report( entry, "defined more than once, the first definition stays" ); continue; }
                // słup bramki z mocowaniami wysięgnika to też słup dla wysięgników (pod tą samą nazwą): bez modelu - stawia go
                // bramka; głębokość w stronę toru to wymiar wzdłuż dźwigara, mocowania najwyżej pod dźwigarem dolnego poziomu
                if( false == mast.strut.empty() && false == mast.stay.empty() ) {
                    cantilever_pole pole;
                    pole.name = name;
                    pole.gantry = true;
                    pole.top = mast.height;
                    pole.length = mast.height;
                    // ściana słupa od strony toru tam, gdzie kończy się dźwigar (gantry_beam_ends: oś + offset) - zgłoszenie 2026-10-22:
                    // z length (0,60) mocowania stały ~0,18 m przed ścianą słupa
                    pole.depth_top = pole.depth_base = mast.offset > 0.0 ? 2.0 * mast.offset : mast.length;
                    pole.mast_low = mast.mount_low;
                    pole.mast_high = mast.mount_high > 0.0 ? mast.mount_high : mast.mount_low;
                    pole.below_beam = mast.below_beam;
                    pole.mount_max = mast.mount_low - mast.below_beam;
                    pole.strut = mast.strut;
                    pole.stay = mast.stay;
                    pole.steady = mast.steady;
                    if( false == m_poles.emplace( name, pole ).second ) { report( entry, "has the name of a pole, it can't carry cantilevers" ); }
                }
            }
            else {
                if( beam.model.empty() || beam.span <= 0.0 ) { report( entry, "needs \"model\" and \"span\"" ); continue; }
                if( false == m_beams.emplace( name, beam ).second ) { report( entry, "defined more than once, the first definition stays" ); }
            }
            continue;
        }

        if( token == "assembly" ) {
            if( index >= count ) { report( "", "\"assembly\" without a name" ); break; }
            cantilever_assembly assembly;
            assembly.name = Tokens[ index++ ];
            auto const entry { "assembly \"" + assembly.name + "\"" };
            auto good { true };
            auto closed { false };
            std::vector<std::size_t> open;
            while( index < count && good ) {
                cantilever_step step;
                step.op = Tokens[ index++ ];
                if( step.op == "endassembly" ) { closed = true; break; }
                // liczba argumentów z opisu klocka; "line" ma po punkcie początkowym jeden z trzech sposobów
                std::size_t wanted { 0 };
                if( step.op == "line" ) {
                    wanted = 3;
                    if( index + 2 < count ) {
                        auto const &how { Tokens[ index + 2 ] };
                        wanted += ( how == "dir" ? 2 : ( how == "toward" || how == "along" ? 1 : 0 ) );
                    }
                }
                else {
                    auto const signature { step_signature( step.op ) };
                    if( signature == nullptr ) { report( entry, "unknown step \"" + step.op + "\"" ); good = false; break; }
                    wanted = signature->size();
                }
                for( std::size_t word = 0; word < wanted && index < count; ++word ) { step.words.push_back( Tokens[ index++ ] ); }
                std::string reason;
                if( step.words.size() != wanted || false == valid_step( step.op, step.words, reason ) ) {
                    report( entry, "step \"" + step.op + "\": " + ( reason.empty() ? std::string( "missing arguments" ) : reason ) );
                    good = false; break;
                }
                if( step.op == "repeat" ) { open.push_back( assembly.steps.size() ); }
                if( step.op == "endrepeat" ) {
                    if( open.empty() ) { report( entry, "\"endrepeat\" without \"repeat\"" ); good = false; break; }
                    assembly.steps[ open.back() ].end = assembly.steps.size() + 1;
                    open.pop_back();
                }
                assembly.steps.push_back( step );
            }
            if( good && closed && false == open.empty() ) { report( entry, "\"repeat\" without \"endrepeat\"" ); continue; }
            if( false == good || false == closed ) {
                if( good ) { report( entry, "missing \"endassembly\"" ); }
                skip();
                continue;
            }
            if( m_assemblies.find( assembly.name ) != m_assemblies.end() ) { report( entry, "defined more than once, the first definition stays" ); continue; }
            m_assemblies.emplace( assembly.name, assembly );
            continue;
        }

        if( token == "models" ) {
            if( index >= count ) { report( "", "\"models\" without a path" ); break; }
            m_models = Tokens[ index++ ];
            std::replace( m_models.begin(), m_models.end(), '\\', '/' );
            if( m_models.back() != '/' ) { m_models += '/'; }
            continue;
        }

        if( token == "rules" ) {
            auto good { true };
            auto closed { false };
            auto changed { rules };
            while( index < count && good ) {
                auto const key { Tokens[ index++ ] };
                if( key == "endrules" ) { closed = true; break; }
                if( key == "stay_gap" ) { number( changed.stay_gap_min, good ); number( changed.stay_gap, good ); number( changed.stay_gap_max, good ); }
                else if( key == "clamp_margin" ) { number( changed.clamp_margin, good ); }
                else if( key == "cord_clamp_inset" ) { number( changed.cord_clamp_inset, good ); }
                else if( key == "umbrella_spread" ) { number( changed.umbrella_spread, good ); }
                else { report( "rules", "unknown keyword \"" + key + "\"" ); good = false; --index; }
            }
            if( false == good || false == closed ) {
                if( good ) { report( "rules", "missing \"endrules\"" ); } else if( Errors.empty() || Errors.back().find( "rules" ) == std::string::npos ) { report( "rules", "bad or missing value" ); }
                skip();
                continue;
            }
            rules = changed;
            continue;
        }

        report( "", "unexpected token \"" + token + "\"" );
    }

    // spójność: schemat ma wskazywać istniejące części i zestaw ramion
    for( auto const &scheme : m_schemes ) {
        auto const entry { "scheme \"" + std::to_string( scheme.second.card ) + " " + std::to_string( scheme.second.execution ) + "\"" };
        if( false == scheme.second.registration.empty() && m_registrations.find( scheme.second.registration ) == m_registrations.end() ) { report( entry, "unknown registration \"" + scheme.second.registration + "\"" ); }
        if( m_assemblies.find( scheme.second.assembly ) == m_assemblies.end() ) { report( entry, "unknown assembly \"" + scheme.second.assembly + "\"" ); }
        for( auto const &role : scheme.second.roles ) {
            if( m_parts.find( role.second ) == m_parts.end() ) { report( entry, "role \"" + role.first + "\" uses unknown part \"" + role.second + "\"" ); }
        }
    }
    for( auto const &part : m_parts ) {
        for( auto const &member : part.second.members ) {
            if( m_parts.find( member.part ) == m_parts.end() ) { report( "part \"" + part.first + "\"", "member uses unknown part \"" + member.part + "\"" ); }
        }
    }
}

bool cantilever_catalog::load( std::string const &Filename, std::vector<std::string> &Errors ) {

    std::ifstream file { Filename };
    if( false == file.is_open() ) { return false; }
    std::vector<std::string> tokens;
    std::string line;
    while( std::getline( file, line ) ) {
        auto const comment { line.find( "//" ) };
        if( comment != std::string::npos ) { line.erase( comment ); }
        std::istringstream words { line };
        std::string word;
        while( words >> word ) { tokens.push_back( word ); }
    }
    parse( tokens, Filename, Errors );
    return true;
}

cantilever_part const *cantilever_catalog::part( std::string const &Name ) const {
    auto const lookup { m_parts.find( Name ) };
    return ( lookup != m_parts.end() ? &lookup->second : nullptr );
}

cantilever_registration const *cantilever_catalog::registration( std::string const &Name ) const {
    auto const lookup { m_registrations.find( Name ) };
    return ( lookup != m_registrations.end() ? &lookup->second : nullptr );
}

cantilever_scheme const *cantilever_catalog::scheme( int const Card, int const Execution ) const {
    auto const lookup { m_schemes.find( { Card, Execution } ) };
    return ( lookup != m_schemes.end() ? &lookup->second : nullptr );
}

cantilever_assembly const *cantilever_catalog::assembly( std::string const &Name ) const {
    auto const lookup { m_assemblies.find( Name ) };
    return ( lookup != m_assemblies.end() ? &lookup->second : nullptr );
}

cantilever_pole const *cantilever_catalog::pole( std::string const &Name ) const {
    auto const lookup { m_poles.find( Name ) };
    return ( lookup != m_poles.end() ? &lookup->second : nullptr );
}

cantilever_mount const *cantilever_catalog::mount( std::string const &Name ) const {
    auto const lookup { m_mounts.find( Name ) };
    return ( lookup != m_mounts.end() ? &lookup->second : nullptr );
}

double cantilever_pole::depth( double const Height ) const {
    // zbieżność liczona od góry słupa; nad górą wymiar górny
    auto const below { std::max( 0.0, top - Height ) };
    return depth_top + ( depth_base - depth_top ) * below / length;
}

cantilever_scheme const *cantilever_catalog::scheme_for_reach( int const Card, int const Execution, double const Reach ) const {
    auto const decade { Card / 10 };
    for( auto lookup { m_schemes.lower_bound( { decade * 10, 0 } ) }; lookup != m_schemes.end() && lookup->first.first / 10 == decade; ++lookup ) {
        auto const &scheme { lookup->second };
        if( scheme.execution == Execution && Reach >= scheme.reach_min && Reach < scheme.reach_max ) { return &scheme; }
    }
    return nullptr;
}

gantry_mast const *cantilever_catalog::mast( std::string const &Name ) const {
    auto const lookup { m_masts.find( Name ) };
    return ( lookup != m_masts.end() ? &lookup->second : nullptr );
}

gantry_beam const *cantilever_catalog::beam( std::string const &Name ) const {
    auto const lookup { m_beams.find( Name ) };
    return ( lookup != m_beams.end() ? &lookup->second : nullptr );
}

cantilever_input cantilever_nominal_input( cantilever_scheme const &Scheme, double const Reach ) {
    cantilever_input input;
    input.stay_mount = { Reach, Scheme.mount_stay };
    input.strut_mount = { Reach, Scheme.mount_strut };
    input.steady_mount = { Reach, Scheme.mount_steady };
    input.messenger = { 0.0, Scheme.height };
    return input;
}

cantilever_placement const *cantilever_result::find( std::string const &Role ) const {
    for( auto const &placement : parts ) { if( placement.role == Role ) { return &placement; } }
    return nullptr;
}

namespace {

// ---- sposób składania: wyrażenia, odwołania do punktów, kierunki

// znane pola części dostępne w wyrażeniach jako <rola>.<pole>
bool part_field( cantilever_part const &Part, std::string const &Field, double &Value ) {
    if( Field == "length" ) { Value = Part.length; }
    else if( Field == "clearance" ) { Value = Part.clearance; }
    else if( Field == "pin" ) { Value = Part.pin; }
    else if( Field == "radius" ) { Value = Part.radius; }
    else if( Field == "eye" ) { Value = Part.eye; }
    else if( Field == "hook.y" ) { Value = Part.hook[ 0 ]; }
    else if( Field == "hook.z" ) { Value = Part.hook[ 1 ]; }
    else if( Field == "end.y" ) { Value = Part.end[ 0 ]; }
    else if( Field == "end.z" ) { Value = Part.end[ 1 ]; }
    else { return false; }
    return true;
}
bool is_part_field( std::string const &Field ) {
    cantilever_part const dummy;
    double value;
    return part_field( dummy, Field, value );
}

// wyrażenie: składniki rozdzielone znakami + i -, bez spacji, np. "strut_insulator.length+rules.clamp_margin", "-steady.hook.y"
struct term { double sign { 1.0 }; std::string text; double number { 0.0 }; bool constant { false }; };

bool split_expression( std::string const &Text, std::vector<term> &Terms ) {
    Terms.clear();
    if( Text.empty() ) { return false; }
    std::size_t start { 0 };
    double sign { 1.0 };
    if( Text[ 0 ] == '+' || Text[ 0 ] == '-' ) { sign = ( Text[ 0 ] == '-' ? -1.0 : 1.0 ); start = 1; }
    while( start <= Text.size() ) {
        // koniec składnika: następny znak + albo -, który nie jest częścią zapisu wykładnika liczby
        auto end { start };
        while( end < Text.size() ) {
            auto const c { Text[ end ] };
            if( ( c == '+' || c == '-' ) && end > start && false == ( ( Text[ end - 1 ] == 'e' || Text[ end - 1 ] == 'E' ) && std::isdigit( static_cast<unsigned char>( Text[ start ] ) ) ) ) { break; }
            ++end;
        }
        term item;
        item.sign = sign;
        item.text = Text.substr( start, end - start );
        if( item.text.empty() ) { return false; }
        item.constant = to_number( item.text, item.number );
        if( false == item.constant && ( std::isdigit( static_cast<unsigned char>( item.text[ 0 ] ) ) || item.text[ 0 ] == '.' ) ) { return false; }
        Terms.push_back( item );
        if( end >= Text.size() ) { break; }
        sign = ( Text[ end ] == '-' ? -1.0 : 1.0 );
        start = end + 1;
        if( start >= Text.size() ) { return false; }
    }
    return false == Terms.empty();
}

bool is_name( std::string const &Text ) {
    if( Text.empty() ) { return false; }
    for( auto const c : Text ) {
        if( false == ( std::isalnum( static_cast<unsigned char>( c ) ) || c == '_' || c == '.' ) ) { return false; }
    }
    return true;
}

// rodzaje argumentów klocków: n nazwa nowego symbolu, p punkt (nazwa albo <człon>@<wyrażenie>), l człon, e wyrażenie,
// E wyrażenie albo "-", r rola (z ? na początku: opcjonalna), R rola albo "-", d kierunek, a oś spoczynkowa części, i liczba całkowita
std::map<std::string, std::string> const step_arguments {
    { "point", "np" }, { "shift", "npee" }, { "side", "nlee" }, { "line", "np" }, { "hang", "rpp" },
    { "tangent", "nppen" }, { "level", "lnpe" }, { "project", "lnpee" }, { "limit", "lnee" }, { "window", "lnEE" },
    { "chain", "lnpeernEEE" }, { "place", "rpda" }, { "roll", "rR" }, { "stretch", "rppa" }, { "registration", "" },
    { "repeat", "ip" }, { "endrepeat", "" }, { "vee", "rppea" }, { "hangarms", "rrrlp" }, { "anchor", "nper" },
    { "flip", "r" }, { "snap", "lnrn" }, { "platform", "ppp" }, { "armcords", "rrpe" }, { "setarms", "rp" }, { "atleast", "lne" }, { "export", "p" }, { "hangat", "rp" }, { "across", "lnpe" }, { "hangto", "rplne" }, { "armropes", "rp" }, { "tee", "ppp" }, { "tee_bar", "pppn" } };

bool valid_argument( char const Kind, std::string const &Word ) {
    std::vector<term> terms;
    switch( Kind ) {
        case 'n': case 'l': return is_name( Word );
        case 'e': return split_expression( Word, terms );
        case 'E': return Word == "-" || split_expression( Word, terms );
        case 'r': return is_name( Word[ 0 ] == '?' ? Word.substr( 1 ) : Word );
        case 'R': return Word == "-" || is_name( Word );
        case 'p': {
            auto const at { Word.find( '@' ) };
            if( at == std::string::npos ) { return is_name( Word ); }
            return is_name( Word.substr( 0, at ) ) && split_expression( Word.substr( at + 1 ), terms );
        }
        case 'd': {
            auto name { Word[ 0 ] == '-' ? Word.substr( 1 ) : Word };
            return is_name( name );
        }
        case 'a': return Word == "+y" || Word == "-y" || Word == "+z" || Word == "-z";
        case 'i': { double value; return to_number( Word, value ) && value >= 1.0; }
        default: return false;
    }
}

std::string const *step_signature( std::string const &Op ) {
    auto const lookup { step_arguments.find( Op ) };
    return ( lookup != step_arguments.end() ? &lookup->second : nullptr );
}

bool valid_step( std::string const &Op, std::vector<std::string> const &Words, std::string &Reason ) {
    auto signature { *step_signature( Op ) };
    if( Op == "line" ) {
        // line <nazwa> <początek> toward <punkt> | dir <dy> <dz> | along <człon>
        auto const &how { Words[ 2 ] };
        if( how == "toward" ) { signature = "npkp"; }
        else if( how == "dir" ) { signature = "npkee"; }
        else if( how == "along" ) { signature = "npkl"; }
        else { Reason = "expected \"toward\", \"dir\" or \"along\" instead of \"" + how + "\""; return false; }
    }
    for( std::size_t index = 0; index < Words.size(); ++index ) {
        if( signature[ index ] == 'k' ) { continue; }
        if( false == valid_argument( signature[ index ], Words[ index ] ) ) { Reason = "bad argument \"" + Words[ index ] + "\""; return false; }
    }
    return true;
}

vec2 rest_axis( std::string const &Axis ) {
    if( Axis == "+y" ) { return toward_support; }
    if( Axis == "-y" ) { return toward_track; }
    if( Axis == "+z" ) { return upward; }
    return downward;
}

// prosta w płaszczyźnie wysięgnika: oś członu albo kierunek pomocniczy, z nazwanymi położeniami wzdłuż niej
struct line {
    vec2 origin;
    vec2 direction;
    vec2 normal;                        // prostopadła skierowana w górę
    std::map<std::string, double> stations;
    vec2 at( double const Distance ) const { return origin + direction * Distance; }
};

// stan składania jednego wysięgnika
class assembler {
public:
    assembler( cantilever_catalog const &Catalog, cantilever_scheme const &Scheme, cantilever_input const &Input, cantilever_registration const *Registration, cantilever_result &Result )
        : m_catalog( Catalog ), m_scheme( Scheme ), m_input( Input ), m_registration( Registration ), m_result( Result ),
          m_label( std::to_string( Scheme.card ) + "-" + std::to_string( Scheme.execution ) ) {
        m_points[ "wire" ] = { 0.0, 0.0 };
        m_points[ "messenger" ] = to_vec( Input.messenger );
        m_points[ "strut_mount" ] = to_vec( Input.strut_mount );
        m_points[ "stay_mount" ] = to_vec( Input.stay_mount );
        m_points[ "steady_mount" ] = to_vec( Input.steady_mount );
        if( Registration != nullptr ) { m_points[ "registration" ] = { Registration->clamp, Registration->tube }; }
        for( auto const &point : Input.points ) { m_points[ point.first ] = to_vec( point.second ); }
    }

    bool run( std::vector<cantilever_step> const &Steps ) {
        return run( Steps, 0, Steps.size() );
    }

private:
    // błąd przerywający składanie
    struct failure {};

    bool run( std::vector<cantilever_step> const &Steps, std::size_t const Begin, std::size_t const End ) {
        try {
            for( auto index { Begin }; index < End; ) {
                auto const &step { Steps[ index ] };
                if( step.op == "repeat" ) {
                    // powtarzanie do ustalenia się punktu; ostrzeżenia zostają tylko z ostatniego przebiegu
                    auto const passes { static_cast<int>( number( step.words[ 0 ] ) ) };
                    auto const warnings { m_result.warnings.size() };
                    for( int pass = 0; pass < passes; ++pass ) {
                        m_result.warnings.resize( warnings );
                        auto const before { point( step.words[ 1 ] ) };
                        if( false == run( Steps, index + 1, step.end - 1 ) ) { return false; }
                        if( length( point( step.words[ 1 ] ) - before ) < 1e-6 ) { break; }
                    }
                    index = step.end;
                    continue;
                }
                execute( step );
                ++index;
            }
        }
        catch( failure const & ) { return false; }
        return true;
    }

    [[noreturn]] void fail( std::string const &Reason ) {
        m_result.warnings.push_back( "cantilever " + m_label + ": " + Reason );
        throw failure();
    }
    void warn( std::string const &Text ) { m_result.warnings.push_back( "cantilever " + m_label + ": " + Text ); }

    // nazwa części przypisanej roli; pusta, gdy schemat jej nie ma
    std::string part_name( std::string const &Role ) const {
        auto const replacement { m_input.overrides.find( Role ) };
        if( replacement != m_input.overrides.end() ) { return replacement->second; }
        auto const lookup { m_scheme.roles.find( Role ) };
        return ( lookup != m_scheme.roles.end() ? lookup->second : std::string() );
    }
    bool has_role( std::string const &Role ) const { return false == part_name( Role ).empty(); }
    cantilever_part const &part( std::string const &Role ) {
        auto const name { part_name( Role ) };
        if( name.empty() ) { fail( "the scheme has no part for the role \"" + Role + "\"" ); }
        auto const *found { m_catalog.part( name ) };
        if( found == nullptr ) { fail( "unknown part \"" + name + "\" for the role \"" + Role + "\"" ); }
        return *found;
    }
    // rola z nazwą części do komunikatów
    std::string described( std::string const &Role ) const {
        auto const name { part_name( Role ) };
        return Role + ( name.empty() ? std::string() : " \"" + name + "\"" );
    }

    line &line_named( std::string const &Name ) {
        auto const lookup { m_lines.find( Name ) };
        if( lookup == m_lines.end() ) { fail( "unknown line \"" + Name + "\"" ); }
        return lookup->second;
    }
    double station( std::string const &Line, std::string const &Station ) {
        auto const &found { line_named( Line ) };
        auto const lookup { found.stations.find( Station ) };
        if( lookup == found.stations.end() ) { fail( "line \"" + Line + "\" has no position \"" + Station + "\"" ); }
        return lookup->second;
    }

    // wartość jednego składnika wyrażenia
    double value( std::string const &Text ) {
        double result;
        if( to_number( Text, result ) ) { return result; }
        auto const dot { Text.find( '.' ) };
        if( dot != std::string::npos ) {
            auto const head { Text.substr( 0, dot ) };
            auto const tail { Text.substr( dot + 1 ) };
            if( head == "rules" ) {
                auto const &rules { m_catalog.rules };
                if( tail == "stay_gap_min" ) { return rules.stay_gap_min; }
                if( tail == "stay_gap" ) { return rules.stay_gap; }
                if( tail == "stay_gap_max" ) { return rules.stay_gap_max; }
                if( tail == "clamp_margin" ) { return rules.clamp_margin; }
                if( tail == "cord_clamp_inset" ) { return rules.cord_clamp_inset; }
                fail( "unknown rule \"" + tail + "\"" );
            }
            auto const lineat { m_lines.find( head ) };
            if( lineat != m_lines.end() ) {
                auto const stationat { lineat->second.stations.find( tail ) };
                if( stationat != lineat->second.stations.end() ) { return stationat->second; }
            }
            if( is_part_field( tail ) ) {
                part_field( part( head ), tail, result );
                return result;
            }
            // nazwany punkt modelu części: <rola>.<punkt>.y albo .z
            if( has_role( head ) ) {
                auto const last { tail.rfind( '.' ) };
                if( last != std::string::npos ) {
                    auto const &named { part( head ).points };
                    auto const found { named.find( tail.substr( 0, last ) ) };
                    if( found != named.end() ) {
                        if( tail.substr( last + 1 ) == "y" ) { return found->second.first; }
                        if( tail.substr( last + 1 ) == "z" ) { return found->second.second; }
                    }
                }
            }
            auto const valueat { m_result.values.find( Text ) };
            if( valueat != m_result.values.end() ) { return valueat->second; }
            // współrzędna punktu: <punkt>.y albo <punkt>.z
            auto const last { Text.rfind( '.' ) };
            auto const pointat { m_points.find( Text.substr( 0, last ) ) };
            if( pointat != m_points.end() ) {
                if( Text.substr( last + 1 ) == "y" ) { return pointat->second.y; }
                if( Text.substr( last + 1 ) == "z" ) { return pointat->second.z; }
            }
        }
        fail( "unknown value \"" + Text + "\"" );
    }
    double number( std::string const &Text ) {
        std::vector<term> terms;
        if( false == split_expression( Text, terms ) ) { fail( "bad expression \"" + Text + "\"" ); }
        double result { 0.0 };
        auto first { true };
        for( auto const &item : terms ) {
            auto const part { item.constant ? item.number : value( item.text ) };
            // pierwszy składnik bez dodawania do zera, żeby wynik był co do bitu taki, jak przy zapisie wprost
            if( first ) { result = ( item.sign < 0.0 ? -part : part ); first = false; }
            else if( item.sign < 0.0 ) { result = result - part; }
            else { result = result + part; }
        }
        return result;
    }
    vec2 point( std::string const &Text ) {
        auto const at { Text.find( '@' ) };
        if( at != std::string::npos ) {
            auto const &found { line_named( Text.substr( 0, at ) ) };
            return found.at( number( Text.substr( at + 1 ) ) );
        }
        auto const lookup { m_points.find( Text ) };
        if( lookup == m_points.end() ) { fail( "unknown point \"" + Text + "\"" ); }
        return lookup->second;
    }
    // kierunek: <człon>, -<człon>, <człon>.up, <człon>.down (prostopadła w górę / w dół)
    vec2 direction( std::string const &Text ) {
        auto const reverse { Text[ 0 ] == '-' };
        auto name { reverse ? Text.substr( 1 ) : Text };
        auto side { 0 };
        auto const suffix = [ & ]( std::string const &Ending ) {
            return name.size() > Ending.size() && name.compare( name.size() - Ending.size(), Ending.size(), Ending ) == 0 && m_lines.count( name ) == 0;
        };
        if( suffix( ".up" ) ) { side = 1; name.erase( name.size() - 3 ); }
        else if( suffix( ".down" ) ) { side = -1; name.erase( name.size() - 5 ); }
        auto const &found { line_named( name ) };
        auto const result { side == 0 ? found.direction : ( side > 0 ? found.normal : found.normal * -1.0 ) };
        return ( reverse ? result * -1.0 : result );
    }
    line &make_line( std::string const &Name, vec2 const &Origin, vec2 const &Direction ) {
        auto &made { m_lines[ Name ] };
        made.origin = Origin;
        made.direction = Direction;
        made.normal = upward_normal( Direction );
        made.stations.clear();
        return made;
    }
    // zestaw ramion stawiany zerem w punkcie przewodu "wire"; Flip: zestaw od słupa (obrót o 180 stopni wokół pionu).
    // Zestaw ze składnikami (member) stawia składniki zamiast siebie
    void put_arm_set( std::string const &setrole, cantilever_part const &set, bool const flip ) {
        auto const side { flip ? -1.0 : 1.0 };
        auto const turn = [ & ]( cantilever_placement &Placement ) {
            if( flip ) { for( int column : { 0, 1 } ) { for( int row = 0; row < 3; ++row ) { Placement.rotation[ row * 3 + column ] = -Placement.rotation[ row * 3 + column ]; } } }
        };
        if( set.members.empty() ) {
            auto placement { place( setrole, set.name, m_points[ "wire" ], toward_support, toward_support ) };
            turn( placement );
            put( placement );
        }
        else {
            // zestaw z osobnych modeli: każdy składnik w swoim położeniu i obrocie wokół pionu, cały zestaw obracany jak zwykły
            auto const &wire { m_points[ "wire" ] };
            for( std::size_t index = 0; index < set.members.size(); ++index ) {
                auto const &member { set.members[ index ] };
                cantilever_placement placement;
                placement.role = setrole + "." + std::to_string( index + 1 );
                placement.part = member.part;
                placement.position[ 0 ] = side * member.position[ 0 ];
                placement.position[ 1 ] = wire.y + side * member.position[ 1 ];
                placement.position[ 2 ] = wire.z + member.position[ 2 ];
                auto const angle { member.yaw * 3.14159265358979323846 / 180.0 };
                auto const cosine { std::cos( angle ) };
                auto const sine { std::sin( angle ) };
                double const rotation[ 9 ] { cosine, -sine, 0.0,  sine, cosine, 0.0,  0.0, 0.0, 1.0 };
                std::copy( std::begin( rotation ), std::end( rotation ), std::begin( placement.rotation ) );
                turn( placement );
                put( placement );
                // zaczep przewodu każdego ramienia zestawu (zero modelu ramienia) - dane dla edytora
                auto const *memberpart { m_catalog.part( member.part ) };
                if( memberpart != nullptr && memberpart->kind == "arms" ) {
                    m_result.values[ placement.role + ".wire.x" ] = placement.position[ 0 ];
                    m_result.values[ placement.role + ".wire.y" ] = placement.position[ 1 ];
                    m_result.values[ placement.role + ".wire.z" ] = placement.position[ 2 ];
                }
            }
        }
    }

    // część ustawiona drugi raz (w kolejnym przebiegu "repeat") zastępuje poprzednie ustawienie
    void put( cantilever_placement const &Input ) {
        auto Placement { Input };
        // część z innym plikiem modelu (lina 9832-2 z modelu linki 4490-00)
        if( Placement.model.empty() ) {
            auto const *known { m_catalog.part( Placement.part ) };
            if( known != nullptr && false == known->model.empty() ) { Placement.model = m_catalog.models() + known->model; }
        }
        for( auto &placed : m_result.parts ) {
            if( placed.role == Placement.role && Placement.role != "registration" ) { placed = Placement; return; }
        }
        m_result.parts.push_back( Placement );
    }
    cantilever_placement *placed( std::string const &Role ) {
        for( auto &placement : m_result.parts ) { if( placement.role == Role ) { return &placement; } }
        return nullptr;
    }

    // część w dowolnym kierunku w przestrzeni: oś spoczynkowa Axis (+y -y +z -z) wzdłuż Direction (wektor jednostkowy X Y Z),
    // oś X części możliwie wzdłuż toru
    cantilever_placement place_3d( std::string const &Role, std::string const &Part, vec2 const &Origin, double const ( &Direction )[ 3 ], std::string const &Axis ) {
        cantilever_placement placement;
        placement.role = Role;
        placement.part = Part;
        placement.position[ 1 ] = Origin.y;
        placement.position[ 2 ] = Origin.z;
        auto const sign { Axis[ 0 ] == '-' ? -1.0 : 1.0 };
        double const main[ 3 ] { Direction[ 0 ] * sign, Direction[ 1 ] * sign, Direction[ 2 ] * sign };
        // X części: oś toru bez składowej wzdłuż głównej osi
        double x[ 3 ] { 1.0 - main[ 0 ] * main[ 0 ], -main[ 0 ] * main[ 1 ], -main[ 0 ] * main[ 2 ] };
        auto const size { std::sqrt( x[ 0 ] * x[ 0 ] + x[ 1 ] * x[ 1 ] + x[ 2 ] * x[ 2 ] ) };
        for( auto &value : x ) { value /= size; }
        double y[ 3 ], z[ 3 ];
        if( Axis[ 1 ] == 'z' ) {
            std::copy( std::begin( main ), std::end( main ), std::begin( z ) );
            y[ 0 ] = z[ 1 ] * x[ 2 ] - z[ 2 ] * x[ 1 ]; y[ 1 ] = z[ 2 ] * x[ 0 ] - z[ 0 ] * x[ 2 ]; y[ 2 ] = z[ 0 ] * x[ 1 ] - z[ 1 ] * x[ 0 ];
        }
        else {
            std::copy( std::begin( main ), std::end( main ), std::begin( y ) );
            z[ 0 ] = x[ 1 ] * y[ 2 ] - x[ 2 ] * y[ 1 ]; z[ 1 ] = x[ 2 ] * y[ 0 ] - x[ 0 ] * y[ 2 ]; z[ 2 ] = x[ 0 ] * y[ 1 ] - x[ 1 ] * y[ 0 ];
        }
        for( int row = 0; row < 3; ++row ) {
            placement.rotation[ row * 3 + 0 ] = x[ row ];
            placement.rotation[ row * 3 + 1 ] = y[ row ];
            placement.rotation[ row * 3 + 2 ] = z[ row ];
        }
        return placement;
    }

    void execute( cantilever_step const &Step ) {
        auto const &op { Step.op };
        auto const &w { Step.words };

        if( op == "point" ) { m_points[ w[ 0 ] ] = point( w[ 1 ] ); return; }

        if( op == "shift" ) {
            auto const base { point( w[ 1 ] ) };
            m_points[ w[ 0 ] ] = base + vec2 { number( w[ 2 ] ), number( w[ 3 ] ) };
            return;
        }

        if( op == "side" ) {
            // punkt na członie, odsunięty od jego osi wzdłuż prostopadłej skierowanej w górę
            auto const &found { line_named( w[ 1 ] ) };
            m_points[ w[ 0 ] ] = found.at( number( w[ 2 ] ) ) + found.normal * number( w[ 3 ] );
            return;
        }

        if( op == "line" ) {
            auto const origin { point( w[ 1 ] ) };
            auto const &how { w[ 2 ] };
            if( how == "toward" ) { make_line( w[ 0 ], origin, unit( point( w[ 3 ] ) - origin ) ); }
            else if( how == "dir" ) { make_line( w[ 0 ], origin, unit( vec2 { number( w[ 3 ] ), number( w[ 4 ] ) } ) ); }
            else { make_line( w[ 0 ], origin, line_named( w[ 3 ] ).direction ); }
            return;
        }

        if( op == "hang" ) {
            // część wisząca w jednej linii z linką: zero w punkcie zawieszenia, prosta od punktu "end" do "hook" części
            // idzie wzdłuż kierunku od punktu zaczepu linki do punktu zawieszenia
            auto const &role { w[ 0 ] };
            auto const &hung { part( role ) };
            vec2 const hook { hung.hook[ 0 ], hung.hook[ 1 ] };
            vec2 const end { hung.end[ 0 ], hung.end[ 1 ] };
            if( length( hook - end ) < 1e-4 ) { fail( "part \"" + hung.name + "\" has no \"hook\" and \"end\" points" ); }
            auto const axis { unit( hook - end ) };
            auto const origin { point( w[ 1 ] ) };
            auto const pull { unit( origin - point( w[ 2 ] ) ) };
            auto const turn { std::atan2( pull.z, pull.y ) - std::atan2( axis.z, axis.y ) };
            m_points[ role + ".hook" ] = origin + rotate( hook, turn );
            m_points[ role + ".end" ] = origin + rotate( end, turn );
            make_line( role, origin, pull );
            put( place( role, hung.name, origin, pull, axis ) );
            return;
        }

        if( op == "hangat" ) {
            // uchwyt liny wiszący pionowo hakiem w punkcie Hook (drugi uchwyt liny na ukośniku, 102x); zero (oś liny) trafia
            // do punktu <rola>, haczyk i koniec do <rola>.hook, <rola>.end
            auto const &role { w[ 0 ] };
            auto const &hung { part( role ) };
            vec2 const hook { hung.hook[ 0 ], hung.hook[ 1 ] };
            vec2 const end { hung.end[ 0 ], hung.end[ 1 ] };
            if( length( hook - end ) < 1e-4 ) { fail( "part \"" + hung.name + "\" has no \"hook\" and \"end\" points" ); }
            auto const axis { unit( hook - end ) };
            auto const turn { std::atan2( 1.0, 0.0 ) - std::atan2( axis.z, axis.y ) };
            auto const origin { point( w[ 1 ] ) - rotate( hook, turn ) };
            m_points[ role ] = origin;
            m_points[ role + ".hook" ] = origin + rotate( hook, turn );
            m_points[ role + ".end" ] = origin + rotate( end, turn );
            make_line( role, origin, upward );
            put( place( role, hung.name, origin, upward, axis ) );
            return;
        }

        if( op == "hangto" ) {
            // uchwyt liny z zerem w punkcie Rope, zawieszony hakiem na uchu uchwytu stawianego na członie Line (ucho odsunięte o Side
            // wzdłuż prostopadłej): ucho tam, gdzie jego odległość od liny równa się odległości haka od zera uchwytu (wyżej z dwóch
            // rozwiązań); położenie ucha na członie trafia do stacji Station. Drugi uchwyt liny na ukośniku (102x)
            auto const &role { w[ 0 ] };
            auto const &hung { part( role ) };
            vec2 const hook { hung.hook[ 0 ], hung.hook[ 1 ] };
            vec2 const end { hung.end[ 0 ], hung.end[ 1 ] };
            if( length( hook - end ) < 1e-4 ) { fail( "part \"" + hung.name + "\" has no \"hook\" and \"end\" points" ); }
            auto const rope { point( w[ 1 ] ) };
            auto &found { line_named( w[ 2 ] ) };
            auto const side { number( w[ 4 ] ) };
            auto const base { found.origin + found.normal * side - rope };
            auto const radius { length( hook ) };
            // |base + dir*s|^2 = r^2
            auto const b { dot( base, found.direction ) };
            auto const c { dot( base, base ) - radius * radius };
            auto const delta { b * b - c };
            if( delta < 0.0 ) { fail( "the " + role + " can't reach the " + w[ 2 ] + " from the rope" ); }
            auto const s1 { -b - std::sqrt( delta ) };
            auto const s2 { -b + std::sqrt( delta ) };
            auto const e1 { found.at( s1 ) + found.normal * side };
            auto const e2 { found.at( s2 ) + found.normal * side };
            auto const pick { e1.z >= e2.z ? s1 : s2 };
            auto const eye { e1.z >= e2.z ? e1 : e2 };
            found.stations[ w[ 3 ] ] = pick;
            auto const turn { std::atan2( ( eye - rope ).z, ( eye - rope ).y ) - std::atan2( hook.z, hook.y ) };
            auto const axis { unit( hook - end ) };
            auto const pull { rotate( axis, turn ) };
            m_points[ role ] = rope;
            m_points[ role + ".hook" ] = eye;
            m_points[ role + ".end" ] = rope + rotate( end, turn );
            make_line( role, rope, pull );
            put( place( role, hung.name, rope, pull, axis ) );
            return;
        }

        if( op == "tangent" ) {
            // człon z punktu, styczny do okręgu o promieniu Radius wokół punktu Center; okrąg leży pod członem.
            // położenie Station to punkt styczności (rzut środka okręgu na oś członu)
            auto const from { point( w[ 1 ] ) };
            auto const center { point( w[ 2 ] ) };
            auto const radius { number( w[ 3 ] ) };
            auto const toward { center - from };
            auto const distance { length( toward ) };
            if( distance <= radius + 1e-6 ) { fail( "line \"" + w[ 0 ] + "\" starts inside the circle it should touch" ); }
            auto const along { std::sqrt( distance * distance - radius * radius ) };
            auto const base { std::atan2( toward.z, toward.y ) };
            auto const offset { std::asin( radius / distance ) };
            vec2 heading, normal;
            for( auto const sign : { 1.0, -1.0 } ) {
                auto const angle { base + sign * offset };
                heading = { std::cos( angle ), std::sin( angle ) };
                normal = upward_normal( heading );
                if( length( from + heading * along - normal * radius - center ) < 1e-6 ) { break; }
            }
            make_line( w[ 0 ], from, heading ).stations[ w[ 4 ] ] = along;
            return;
        }

        if( op == "level" ) {
            // położenie na członie, w którym punkt odsunięty o Side wzdłuż prostopadłej wypada na wysokości punktu Target
            auto &found { line_named( w[ 0 ] ) };
            if( std::abs( found.direction.z ) < 1e-6 ) { fail( "line \"" + w[ 0 ] + "\" is level, nothing on it can be set to a height" ); }
            auto const side { number( w[ 3 ] ) };
            found.stations[ w[ 1 ] ] = ( point( w[ 2 ] ).z - found.origin.z - side * found.normal.z ) / found.direction.z;
            return;
        }

        if( op == "across" ) {
            // położenie na członie, w którym punkt odsunięty o Side wzdłuż prostopadłej wypada w poziomie (Y) punktu Target
            auto &found { line_named( w[ 0 ] ) };
            if( std::abs( found.direction.y ) < 1e-6 ) { fail( "line \"" + w[ 0 ] + "\" is vertical, nothing on it can be set across" ); }
            auto const side { number( w[ 3 ] ) };
            found.stations[ w[ 1 ] ] = ( point( w[ 2 ] ).y - found.origin.y - side * found.normal.y ) / found.direction.y;
            return;
        }

        if( op == "project" ) {
            // rzut punktu na człon, najdalej do End - Inset; wystawanie członu za tym położeniem trafia do <człon>.overhang,
            // a brakujący kawałek do <człon>.short
            auto &found { line_named( w[ 0 ] ) };
            auto const nominal { dot( point( w[ 2 ] ) - found.origin, found.direction ) };
            auto const end { number( w[ 3 ] ) };
            auto const inset { number( w[ 4 ] ) };
            auto const along { std::min( nominal, end - inset ) };
            found.stations[ w[ 1 ] ] = along;
            m_result.values[ w[ 0 ] + ".overhang" ] = end - along;
            m_result.values[ w[ 0 ] + ".short" ] = std::max( 0.0, nominal - ( end - inset ) );
            // gdy człon nie sięga położenia nominalnego, część staje przy jego końcu (z zapasem Inset); katalog nie podaje
            // sztywnej odległości, więc to nie jest uwaga do logu (Jan, 2026-10-13: obejma 4210)
            if( nominal < 0.0 ) { warn( described( w[ 0 ] ) + " starts beyond the " + w[ 1 ] ); }
            return;
        }

        if( op == "limit" ) {
            // za położeniem Station musi zostać co najmniej Margin członu, który kończy się w End
            auto const at { station( w[ 0 ], w[ 1 ] ) };
            auto const overhang { number( w[ 2 ] ) - at };
            auto const margin { number( w[ 3 ] ) };
            m_result.values[ w[ 0 ] + ".overhang" ] = overhang;
            if( overhang < margin ) { warn( described( w[ 0 ] ) + " is " + to_text( margin - overhang ) + " m too short for the " + w[ 1 ] ); }
            return;
        }

        if( op == "export" ) {
            // punkt do wyniku (values <punkt>.y, <punkt>.z) - dane dla edytora, np. zaczep drugiego przewodu na 4120 (117x/118x)
            auto const at { point( w[ 0 ] ) };
            m_result.values[ w[ 0 ] + ".y" ] = at.y;
            m_result.values[ w[ 0 ] + ".z" ] = at.z;
            return;
        }

        if( op == "atleast" ) {
            // położenie na członie nie bliżej początku niż Min (uchwyt liny ramion za izolatorem ukośnika, 105x/125x)
            auto &found { line_named( w[ 0 ] ) };
            auto const at { station( w[ 0 ], w[ 1 ] ) };
            found.stations[ w[ 1 ] ] = std::max( at, number( w[ 2 ] ) );
            return;
        }

        if( op == "window" ) {
            auto const at { station( w[ 0 ], w[ 1 ] ) };
            if( w[ 2 ] != "-" && at < number( w[ 2 ] ) ) { warn( "the " + w[ 1 ] + " falls too close to the start of the " + w[ 0 ] ); }
            if( w[ 3 ] != "-" && at > number( w[ 3 ] ) ) { warn( "the " + w[ 1 ] + " falls too close to the end of the " + w[ 0 ] ); }
            return;
        }

        if( op == "anchor" ) {
            // zaczep przesuwany w pionie tak, żeby łańcuch o długości Length + jeden z otworów części Holes sięgnął punktu
            // Target; wybierany otwór z najmniejszym przesunięciem (zaczep odciągu na słupie jest regulowany, "~1,20 m")
            auto const found { m_points.find( w[ 0 ] ) };
            if( found == m_points.end() ) { fail( "unknown point \"" + w[ 0 ] + "\"" ); }
            auto const target { point( w[ 1 ] ) };
            auto const base { number( w[ 2 ] ) };
            auto const &holes { part( w[ 3 ] ) };
            if( holes.holes.empty() ) { fail( "part \"" + holes.name + "\" has no adjustment holes" ); }
            auto const old { found->second };
            auto const dy { target.y - old.y };
            auto best { -1 };
            auto bestz { old.z };
            for( std::size_t hole = 0; hole < holes.holes.size(); ++hole ) {
                auto const reach { base + holes.holes[ hole ] };
                if( reach <= std::abs( dy ) ) { continue; }
                auto const rise { std::sqrt( reach * reach - dy * dy ) };
                for( auto const z : { target.z + rise, target.z - rise } ) {
                    if( best < 0 || std::abs( z - old.z ) < std::abs( bestz - old.z ) ) { best = static_cast<int>( hole ); bestz = z; }
                }
            }
            if( best < 0 ) {
                // brak do 2 cm: odciąg w poziomie na ostatnim otworze (luz na sworzniu, na oko niewidoczne; 0738/0748 na granicy)
                auto const longest { base + holes.holes.back() };
                if( std::abs( dy ) - longest > 0.02 ) {
                    warn( described( w[ 3 ] ) + " can't make the chain from the " + w[ 0 ] + " reach the " + w[ 1 ] + ", " + to_text( std::abs( dy ) - longest ) + " m short" );
                    return;
                }
                best = static_cast<int>( holes.holes.size() ) - 1;
                bestz = target.z;
            }
            if( std::abs( bestz - old.z ) > 0.15 ) {
                warn( "the " + w[ 0 ] + " had to move " + to_text( bestz - old.z ) + " m to reach the " + w[ 1 ] );
            }
            found->second.z = bestz;
            m_result.values[ w[ 0 ] + ".shift" ] = bestz - old.z;
            m_result.values[ w[ 3 ] + ".hole" ] = best;
            return;
        }

        if( op == "chain" ) {
            // łańcuch o długości skokowej (Length + jeden z otworów części Holes) od punktu Anchor do punktu odsuniętego
            // o Side od osi członu, w położeniu Station dobieranym tak, żeby odstęp Reference - Station leżał
            // w przedziale Min..Max możliwie blisko Nominal
            auto &found { line_named( w[ 0 ] ) };
            auto const &newstation { w[ 1 ] };
            auto const anchor { point( w[ 2 ] ) };
            auto const side { number( w[ 3 ] ) };
            auto const base { number( w[ 4 ] ) };
            auto const &holes { part( w[ 5 ] ) };
            auto const reference { station( w[ 0 ], w[ 6 ] ) };
            auto const gapmin { number( w[ 7 ] ) };
            auto const gap { number( w[ 8 ] ) };
            auto const gapmax { number( w[ 9 ] ) };
            if( holes.holes.empty() ) { fail( "part \"" + holes.name + "\" has no adjustment holes" ); }
            auto const pinbase { found.origin + found.normal * side - anchor };
            auto const along { dot( pinbase, found.direction ) };
            auto best { -1 };
            auto bestdistance { 0.0 };
            auto bestpenalty { 0.0 };
            auto bestfits { false };
            for( std::size_t hole = 0; hole < holes.holes.size(); ++hole ) {
                auto const reach { base + holes.holes[ hole ] };
                auto const discriminant { along * along - dot( pinbase, pinbase ) + reach * reach };
                if( discriminant < 0.0 ) { continue; }
                for( auto const sign : { -1.0, 1.0 } ) {
                    auto const distance { -along + sign * std::sqrt( discriminant ) };
                    auto const offset { reference - distance };
                    auto const fits { offset >= gapmin - 1e-9 && offset <= gapmax + 1e-9 };
                    auto const penalty { std::abs( offset - gap ) };
                    if( best < 0 || ( fits && false == bestfits ) || ( fits == bestfits && penalty < bestpenalty ) ) {
                        best = static_cast<int>( hole );
                        bestdistance = distance;
                        bestpenalty = penalty;
                        bestfits = fits;
                    }
                }
            }
            auto chosen { reference - gap };
            if( best < 0 ) {
                // łańcuch za krótki, żeby w ogóle sięgnąć członu: położenie zalecane, części ustawione w linii mimo luki
                warn( described( w[ 5 ] ) + " can't make the chain from the " + w[ 2 ] + " reach the " + w[ 0 ] );
                best = static_cast<int>( holes.holes.size() ) - 1;
            }
            else {
                chosen = bestdistance;
                if( false == bestfits ) {
                    warn( "no hole of \"" + holes.name + "\" puts the " + newstation + " " + to_text( gapmin, 2 ) + "-" + to_text( gapmax, 2 ) + " m from the " + w[ 6 ] + ", the nearest gives " + to_text( reference - chosen ) + " m" );
                }
            }
            found.stations[ newstation ] = chosen;
            m_result.values[ newstation + ".hole" ] = best;
            m_result.values[ newstation + ".gap" ] = reference - chosen;
            return;
        }

        if( op == "place" ) {
            auto role { w[ 0 ] };
            if( role[ 0 ] == '?' ) { role.erase( 0, 1 ); if( false == has_role( role ) ) { return; } }
            auto const &placedpart { part( role ) };
            put( place( role, placedpart.name, point( w[ 1 ] ), direction( w[ 2 ] ), rest_axis( w[ 3 ] ) ) );
            return;
        }

        if( op == "roll" ) {
            // ćwierć obrotu wokół własnej osi; z warunkiem: tylko gdy schemat ma podaną rolę
            if( w[ 1 ] != "-" && false == has_role( w[ 1 ] ) ) { return; }
            auto *target { placed( w[ 0 ] ) };
            if( target == nullptr ) { fail( "\"roll\" of \"" + w[ 0 ] + "\", which isn't placed yet" ); }
            roll_quarter( *target );
            return;
        }

        if( op == "flip" ) {
            // pół obrotu wokół własnej osi podłużnej (lokalnej Y): lokalne X i Z na przeciwną stronę (6010: ucho odciągu u góry, obejma wysięgu u dołu)
            auto *target { placed( w[ 0 ] ) };
            if( target == nullptr ) { fail( "\"flip\" of \"" + w[ 0 ] + "\", which isn't placed yet" ); }
            for( auto const index : { 0, 3, 6, 2, 5, 8 } ) { target->rotation[ index ] = -target->rotation[ index ]; }
            return;
        }

        if( op == "snap" ) {
            // położenie na członie przesunięte do najbliższego środka między sąsiednimi punktami <prefix>_1, _2, ... części roli
            // (punkty leżą wzdłuż -Y części, zero części na początku członu): uchwyt 6710 jak U-śruba przez dwa otwory 6010
            auto &found { line_named( w[ 0 ] ) };
            auto const station { found.stations.find( w[ 1 ] ) };
            if( station == found.stations.end() ) { fail( "line \"" + w[ 0 ] + "\" has no station \"" + w[ 1 ] + "\"" ); }
            auto const &holder { part( w[ 2 ] ) };
            std::vector<double> marks;
            for( int index = 1; ; ++index ) {
                auto const mark { holder.points.find( w[ 3 ] + "_" + std::to_string( index ) ) };
                if( mark == holder.points.end() ) { break; }
                marks.push_back( -mark->second.first );
            }
            if( marks.size() < 2 ) { fail( "part \"" + holder.name + "\" has fewer than two \"" + w[ 3 ] + "_N\" points" ); }
            std::sort( marks.begin(), marks.end() );
            auto best { 0.5 * ( marks[ 0 ] + marks[ 1 ] ) };
            auto pair { 1 };
            for( std::size_t index = 1; index + 1 < marks.size(); ++index ) {
                auto const middle { 0.5 * ( marks[ index ] + marks[ index + 1 ] ) };
                if( std::abs( middle - station->second ) < std::abs( best - station->second ) ) { best = middle; pair = static_cast<int>( index ) + 1; }
            }
            auto const shift { best - station->second };
            if( std::abs( shift ) > 0.03 ) {
                warn( "the " + w[ 1 ] + " on the " + w[ 0 ] + " is " + to_text( shift ) + " m from the nearest pair of " + w[ 3 ] + " points of \"" + holder.name + "\"" );
            }
            m_result.values[ w[ 1 ] + ".pair" ] = pair;
            m_result.values[ w[ 1 ] + ".snap" ] = shift;
            station->second = best;
            return;
        }

        if( op == "platform" ) {
            // ukośnik peronowy 6010 (rola strut) wsparty odciągiem bez izolatora: oko ukośnika na zaczepie StrutMount przesuwanym
            // w pionie, odciąg (stay + otwór stay_end) od zaczepu StayMount do stałego ucha "eye" ukośnika, kolumna od osi blachy
            // 6710 (plate) do liny Messenger odchyla się swobodnie (Jan). Dla każdej pary otworów pod U-śrubę (hole_N, rola
            // hanger_mount) i każdego otworu końcówki odciągu szukane jest takie obniżenie zaczepu ukośnika, przy którym kolumna
            // dokładnie sięga liny; wybierane jest najmniejsze obniżenie, przy równych - kolumna najbliżej pionu.
            auto const mount { point( w[ 0 ] ) };
            auto const staymount { point( w[ 1 ] ) };
            auto const messenger { point( w[ 2 ] ) };
            auto const &strutpart { part( "strut" ) };
            auto const &endpart { part( "stay_end" ) };
            auto const pointof = [&]( cantilever_part const &Part, std::string const &Name ) {
                auto const found { Part.points.find( Name ) };
                if( found == Part.points.end() ) { fail( "part \"" + Part.name + "\" has no point \"" + Name + "\"" ); }
                return vec2 { found->second.first, found->second.second };
            };
            auto const lug { pointof( strutpart, "eye" ) };                     // (Y, Z) w układzie rury: Y wzdłuż -rury
            auto const lugstation { -lug.y };
            auto const lugside { lug.z };                                        // ucho odciągu nad osią rury (modele 6010 od 2026-10-16)
            auto const pivotside { -pointof( part( "hanger_mount" ), "eye" ).z }; // oś blachy pod rurą
            auto const &plate { part( "plate" ) };
            auto const &hanger { part( "hanger" ) };
            auto const &clamp { part( "messenger_clamp" ) };
            auto const column { ( pointof( plate, "pivot" ).z - pointof( plate, "eye" ).z ) + part( "hanger_insulator" ).length - pointof( hanger, "eye" ).z + clamp.hook[ 1 ] };
            std::vector<double> pairs;
            {
                std::vector<double> marks;
                for( int index = 1; ; ++index ) {
                    auto const found { strutpart.points.find( "hole_" + std::to_string( index ) ) };
                    if( found == strutpart.points.end() ) { break; }
                    marks.push_back( -found->second.first );
                }
                std::sort( marks.begin(), marks.end() );
                for( std::size_t index = 0; index + 1 < marks.size(); ++index ) { pairs.push_back( 0.5 * ( marks[ index ] + marks[ index + 1 ] ) ); }
            }
            if( pairs.empty() ) { fail( "part \"" + strutpart.name + "\" has no pairs of \"hole_N\" points" ); }
            auto const lugradius { std::sqrt( lugstation * lugstation + lugside * lugside ) };
            auto const lugangle { std::atan2( lugside, lugstation ) };          // kąt ucha nad osią rury, widziany z oka
            struct pose { bool ok { false }; vec2 origin, direction, pivot; };
            auto const solve = [&]( double Drop, double Station, double Stay ) {
                pose result;
                vec2 const origin { mount.y, mount.z - Drop };
                auto const between { staymount - origin };
                auto const distance { length( between ) };
                if( distance > lugradius + Stay || distance < std::abs( lugradius - Stay ) ) { return result; }
                auto const along { ( lugradius * lugradius - Stay * Stay + distance * distance ) / ( 2.0 * distance ) };
                auto const across { std::sqrt( std::max( 0.0, lugradius * lugradius - along * along ) ) };
                auto const axis { unit( between ) };
                vec2 const normal { -axis.z, axis.y };
                // dwa punkty przecięcia; ucho jest po stronie toru (mniejsze Y)
                auto const first { origin + axis * along + normal * across };
                auto const second { origin + axis * along - normal * across };
                auto const lugpoint { ( first.y < second.y ? first : second ) };
                auto const toward { unit( lugpoint - origin ) };
                auto const angle { std::atan2( toward.z, toward.y ) + lugangle };
                // rura idzie w stronę toru (Y maleje); ucho leży nad nią
                vec2 const direction { std::cos( angle ), std::sin( angle ) };
                if( direction.y > -0.05 || direction.z < 0.0 ) { return result; }
                auto const up { upward_normal( direction ) };
                result.ok = true;
                result.origin = origin;
                result.direction = direction;
                result.pivot = origin + direction * Station - up * pivotside;
                return result;
            };
            auto bestscore { 1e9 };
            pose best;
            double bestdrop { 0.0 }, beststation { 0.0 };
            int besthole { -1 };
            for( auto const station : pairs ) {
                for( std::size_t hole = 0; hole < endpart.holes.size(); ++hole ) {
                    auto const stay { number( "stay.length" ) + endpart.holes[ hole ] };
                    // f(drop) = odległość osi blachy od liny - długość kolumny; szukane miejsce zerowe w -1,0 .. 2,5 m
                    auto f = [&]( double Drop ) { auto const p { solve( Drop, station, stay ) }; return p.ok ? length( p.pivot - messenger ) - column : 1e9; };
                    double previous { -1.0 }, fprevious { f( previous ) };
                    for( double drop = -1.0 + 0.02; drop <= 2.5; drop += 0.02 ) {
                        auto const fnow { f( drop ) };
                        if( fprevious < 1e8 && fnow < 1e8 && ( fprevious <= 0.0 ) != ( fnow <= 0.0 ) ) {
                            double low { previous }, high { drop }, flow { fprevious };
                            for( int step = 0; step < 50; ++step ) {
                                auto const middle { 0.5 * ( low + high ) };
                                auto const fmiddle { f( middle ) };
                                if( ( fmiddle <= 0.0 ) == ( flow <= 0.0 ) ) { low = middle; flow = fmiddle; } else { high = middle; }
                            }
                            auto const p { solve( low, station, stay ) };
                            auto const hanging { unit( messenger - p.pivot ) };
                            auto const tilt { std::abs( std::atan2( hanging.y, -hanging.z ) ) };
                            auto const score { std::abs( low ) + 0.2 * tilt };
                            if( p.ok && hanging.z < 0.0 && score < bestscore ) { bestscore = score; best = p; bestdrop = low; beststation = station; besthole = static_cast<int>( hole ); }
                        }
                        previous = drop; fprevious = fnow;
                    }
                }
            }
            if( besthole < 0 ) { fail( "no position of the platform strut \"" + strutpart.name + "\" lets the stay and the hanging column reach the messenger" ); }
            auto const hanging { unit( messenger - best.pivot ) };
            make_line( "strut", best.origin, best.direction );
            auto &strut { line_named( "strut" ) };
            strut.stations[ "top" ] = beststation;
            strut.stations[ "lug" ] = lugstation;
            m_points[ "strut_pin" ] = best.origin;
            m_points[ "stay_pin" ] = best.origin + best.direction * lugstation + upward_normal( best.direction ) * lugside;
            m_points[ "plate_pivot" ] = best.pivot;
            make_line( "column", best.pivot, hanging );
            m_result.values[ "strut.drop" ] = bestdrop;
            m_result.values[ "stay_end.hole" ] = besthole;
            m_result.values[ "column.tilt" ] = std::atan2( hanging.y, -hanging.z ) * 180.0 / 3.14159265358979323846;
            // obniżenie zaczepu nie jest ostrzeżeniem: rura peronowa jest z polimeru, mocowanie może zejść nawet metr nad ziemię (Jan)
            return;
        }

        if( op == "tee" ) {
            // wysięgnik z teownika (stary katalog Kolprojekt, 01xx, 09xx): izolator 7110 (rola strut_insulator) na zaczepie
            // StrutMount i ukośnik teowy 6101 (rola strut) to jedna sztywna bryła obracająca się wokół zaczepu; pręt górny
            // (rola stay, długość do punktu "eye") z izolatorem 7120 (rola stay_insulator) od zaczepu StayMount leży poziomo i łączy
            // się z punktem "eye" ukośnika; lina nośna leży w siodle (rola messenger_clamp, punkt "rope") na poziomym ramieniu
            // ukośnika, w zakresie slide_horizontal_min..max. Pozycja bryły: kąt z warunku poziomego pręta przy danym położeniu
            // zaczepu górnego w poziomie, wysokość zaczepu dolnego z wysokości liny (wartość strut.drop), wysokość zaczepu górnego
            // z wysokości ucha ukośnika (stay_mount.shift). Człony: strut_insulator, tee (oś poziomego ramienia, obrót bryły),
            // stay (pręt), leg (linia slide_min..slide_max skośnego ramienia, położenie "end" na jej końcu).
            auto const mount { point( w[ 0 ] ) };
            auto const upper { point( w[ 1 ] ) };
            auto const rope { point( w[ 2 ] ) };
            auto const pointof = [&]( cantilever_part const &Part, std::string const &Name ) {
                auto const found { Part.points.find( Name ) };
                if( found == Part.points.end() ) { fail( "part \"" + Part.name + "\" has no point \"" + Name + "\"" ); }
                return vec2 { found->second.first, found->second.second };
            };
            auto const &strutpart { part( "strut" ) };
            auto const slidemin { pointof( strutpart, "slide_min" ) };
            auto const slidemax { pointof( strutpart, "slide_max" ) };
            auto const flatmin { pointof( strutpart, "slide_horizontal_min" ) };
            auto const flatmax { pointof( strutpart, "slide_horizontal_max" ) };
            auto const eye { pointof( strutpart, "eye" ) };
            auto const ropez { pointof( part( "messenger_clamp" ), "rope" ).z };
            auto const legdir { unit( slidemax - slidemin ) };                 // skośne ramię w układzie części (w -Y i w górę)
            auto const insulator { part( "strut_insulator" ).length };
            auto const topinsulator { part( "stay_insulator" ).length };
            auto const base { legdir * insulator };                             // od zaczepu do zera ukośnika, bryła w spoczynku
            auto const body { base + eye };                                     // od zaczepu do ucha ukośnika
            // pręt poziomy łączy się z uchem ukośnika swoim punktem "eye" (przechylne, 4310) albo jednym z otworów hole_N
            // (nieprzechylne, 4320; otwór z najmniejszym obrotem bryły, bez preferencji - Jan 2026-10-18); punkt pręta leży
            // o -z pod jego osią. Obrót bryły wokół zaczepu dolnego tak, żeby ucho wypadło o długość pręta przed zaczepem górnym
            auto const &barpart { part( "stay" ) };
            std::vector<std::pair<std::string, vec2>> joints;
            if( barpart.points.count( "eye" ) ) { joints.push_back( { "eye", pointof( barpart, "eye" ) } ); }
            for( int index = 1; index < 20; ++index ) {
                auto const name { "hole_" + std::to_string( index ) };
                if( barpart.points.count( name ) == 0 ) { break; }
                joints.push_back( { name, pointof( barpart, name ) } );
            }
            if( joints.empty() ) { joints.push_back( { "length", vec2 { -barpart.length, 0.0 } } ); }
            auto const reach { length( body ) };
            auto const alpha { std::atan2( body.z, body.y ) };
            auto const normalized = []( double Angle ) {
                auto const pi { 3.14159265358979323846 };
                while( Angle > pi ) { Angle -= 2.0 * pi; }
                while( Angle < -pi ) { Angle += 2.0 * pi; }
                return Angle;
            };
            auto tilt { 0.0 };
            auto best { -1 };
            vec2 joint;
            for( std::size_t index = 0; index < joints.size(); ++index ) {
                auto const need { upper.y - ( topinsulator - joints[ index ].second.y ) - mount.y };
                if( std::abs( need ) > reach ) { continue; }
                auto const turn0 { std::acos( need / reach ) };
                auto const first { normalized( turn0 - alpha ) };
                auto const second { normalized( -turn0 - alpha ) };
                auto const candidate { std::abs( first ) < std::abs( second ) ? first : second };
                if( best < 0 || std::abs( candidate ) < std::abs( tilt ) ) { tilt = candidate; best = static_cast<int>( index ); joint = joints[ index ].second; }
            }
            if( best < 0 ) { fail( "the strut \"" + strutpart.name + "\" can't reach the top bar \"" + barpart.name + "\" from its mount" ); }
            m_result.values[ "stay.joint" ] = joints[ best ].first.compare( 0, 5, "hole_" ) == 0 ? std::stod( joints[ best ].first.substr( 5 ) ) : 0.0;
            if( std::abs( tilt ) > 5.0 * 3.14159265358979323846 / 180.0 ) {
                warn( "the strut \"" + strutpart.name + "\" is tilted by " + to_text( tilt * 180.0 / 3.14159265358979323846 ) + " degrees to meet the top bar" );
            }
            // lina w siodle: położenie siodła na ramieniu i wysokość zaczepu dolnego
            auto const along { rotate( toward_track, tilt ) };
            auto const offset { rotate( base + flatmin, tilt ) + rotate( vec2 { 0.0, ropez }, tilt ) };
            if( std::abs( along.y ) < 1e-6 ) { fail( "the strut \"" + strutpart.name + "\" stands vertical" ); }
            auto const slide { ( rope.y - mount.y - offset.y ) / along.y };
            vec2 const strutpin { mount.y, rope.z - offset.z - along.z * slide };
            auto const flatlength { length( flatmax - flatmin ) };
            if( slide < -1e-3 ) { warn( "the messenger is " + to_text( -slide ) + " m short of the saddle slide on the strut \"" + strutpart.name + "\"" ); }
            if( slide > flatlength + 1e-3 ) { warn( "the messenger is " + to_text( slide - flatlength ) + " m beyond the saddle slide on the strut \"" + strutpart.name + "\"" ); }
            auto const lug { strutpin + rotate( body, tilt ) };
            vec2 const stay { upper.y, lug.z - joint.z };
            auto const barline { unit( vec2 { lug.y, stay.z } - stay ) };
            m_points[ "strut_pin" ] = strutpin;
            m_points[ "strut_start" ] = strutpin + rotate( base, tilt );
            m_points[ "stay_pin" ] = stay;
            m_points[ "stay_start" ] = stay + barline * topinsulator;
            m_points[ "strut_eye" ] = lug;
            m_points[ "saddle" ] = strutpin + rotate( base + flatmin, tilt ) + along * slide;
            // wszystkie nazwane punkty ukośnika w układzie wysięgnika: strut.<punkt> (np. strut.hole_2 - uchwyt linki 4230)
            for( auto const &named : strutpart.points ) {
                m_points[ "strut." + named.first ] = strutpin + rotate( base + vec2 { named.second.first, named.second.second }, tilt );
            }
            make_line( "strut_insulator", strutpin, rotate( legdir, tilt ) );
            make_line( "tee", strutpin + rotate( base, tilt ), along );
            make_line( "stay", stay, barline );
            make_line( "leg", strutpin + rotate( base + slidemin, tilt ), rotate( legdir, tilt ) ).stations[ "end" ] = length( slidemax - slidemin );
            m_result.values[ "strut.drop" ] = mount.z - strutpin.z;
            m_result.values[ "stay_mount.shift" ] = stay.z - upper.z;
            m_result.values[ "strut.tilt" ] = tilt * 180.0 / 3.14159265358979323846;
            m_result.values[ "messenger_clamp.slide" ] = slide;
            return;
        }

        if( op == "tee_bar" ) {
            // teownik nieprzechylny z siodłem na pręcie górnym (0109 wyk. 1 i 5): pręt (rola stay) z izolatorem (stay_insulator)
            // poziomo od zaczepu StayMount, siodło (messenger_clamp, punkt "rope") na pręcie w zakresie slide_horizontal_min..max,
            // ukośnik (strut) z izolatorem (strut_insulator) jako sztywna bryła z zaczepu StrutMount, punkt ukośnika Point w punkcie
            // "eye" pręta. Wysokość pręta z liny (stay_mount.shift), obrót bryły z odległości w poziomie, wysokość zaczepu dolnego
            // z wysokości ucha pręta (strut.drop). Oś izolatora: slide_min..slide_max ukośnika albo prosta od zera do Point.
            auto const mount { point( w[ 0 ] ) };
            auto const upper { point( w[ 1 ] ) };
            auto const rope { point( w[ 2 ] ) };
            auto const pointof = [&]( cantilever_part const &Part, std::string const &Name ) {
                auto const found { Part.points.find( Name ) };
                if( found == Part.points.end() ) { fail( "part \"" + Part.name + "\" has no point \"" + Name + "\"" ); }
                return vec2 { found->second.first, found->second.second };
            };
            auto const &strutpart { part( "strut" ) };
            auto const &barpart { part( "stay" ) };
            auto const joint { pointof( strutpart, w[ 3 ] ) };
            auto const axis { ( strutpart.points.count( "slide_min" ) && strutpart.points.count( "slide_max" ) )
                ? unit( pointof( strutpart, "slide_max" ) - pointof( strutpart, "slide_min" ) ) : unit( joint ) };
            auto const bareye { pointof( barpart, "eye" ) };
            auto const flatmin { pointof( barpart, "slide_horizontal_min" ) };
            auto const flatmax { pointof( barpart, "slide_horizontal_max" ) };
            auto const ropez { pointof( part( "messenger_clamp" ), "rope" ).z };
            auto const insulator { part( "strut_insulator" ).length };
            auto const topinsulator { part( "stay_insulator" ).length };
            // pręt: oś na wysokości liny minus siodło
            auto const barz { rope.z - ropez - flatmin.z };
            vec2 const stay { upper.y, barz };
            vec2 const barzero { upper.y - topinsulator, barz };
            auto const eye { barzero + bareye };
            // ukośnik: obrót bryły tak, żeby punkt Point wypadł w poziomie ucha pręta
            auto const base { axis * insulator };
            auto const body { base + joint };
            auto const reach { length( body ) };
            auto const dy { eye.y - mount.y };
            if( std::abs( dy ) > reach ) { fail( "the strut \"" + strutpart.name + "\" can't reach the eye of the top bar \"" + barpart.name + "\"" ); }
            auto const angle { std::acos( dy / reach ) };                       // kierunek bryły nad poziomem, w górę
            auto const tilt { std::remainder( angle - std::atan2( body.z, body.y ), 2.0 * 3.14159265358979323846 ) };
            vec2 const strutpin { mount.y, eye.z - reach * std::sin( angle ) };
            auto const slide { barzero.y + flatmin.y - rope.y };
            auto const flatlength { length( flatmax - flatmin ) };
            if( slide < -1e-3 ) { warn( "the messenger is " + to_text( -slide ) + " m short of the saddle slide on the bar \"" + barpart.name + "\"" ); }
            if( slide > flatlength + 1e-3 ) { warn( "the messenger is " + to_text( slide - flatlength ) + " m beyond the saddle slide on the bar \"" + barpart.name + "\"" ); }
            m_points[ "strut_pin" ] = strutpin;
            m_points[ "strut_start" ] = strutpin + rotate( base, tilt );
            m_points[ "stay_pin" ] = stay;
            m_points[ "stay_start" ] = barzero;
            m_points[ "saddle" ] = vec2 { rope.y, barz + flatmin.z };
            for( auto const &named : strutpart.points ) {
                m_points[ "strut." + named.first ] = strutpin + rotate( base + vec2 { named.second.first, named.second.second }, tilt );
            }
            // nazwane punkty pręta w układzie wysięgnika: stay.<punkt> (np. stay.hole_1 - uchwyt linki 4230 na końcu pręta)
            for( auto const &named : barpart.points ) {
                m_points[ "stay." + named.first ] = barzero + vec2 { named.second.first, named.second.second };
            }
            make_line( "strut_insulator", strutpin, rotate( axis, tilt ) );
            make_line( "tee", strutpin + rotate( base, tilt ), rotate( toward_track, tilt ) );
            make_line( "stay", stay, toward_track );
            // zakres przesuwu na ukośniku (węzeł 4360 na 6103): człon "leg" od slide_min do slide_max, położenie "end"
            if( strutpart.points.count( "slide_min" ) && strutpart.points.count( "slide_max" ) ) {
                auto const slidemin { pointof( strutpart, "slide_min" ) };
                auto const slidemax { pointof( strutpart, "slide_max" ) };
                make_line( "leg", strutpin + rotate( base + slidemin, tilt ), rotate( unit( slidemax - slidemin ), tilt ) ).stations[ "end" ] = length( slidemax - slidemin );
            }
            m_result.values[ "strut.drop" ] = mount.z - strutpin.z;
            m_result.values[ "stay_mount.shift" ] = barz - upper.z;
            m_result.values[ "strut.tilt" ] = tilt * 180.0 / 3.14159265358979323846;
            m_result.values[ "messenger_clamp.slide" ] = slide;
            return;
        }

        if( op == "stretch" ) {
            // część rozciągnięta między dwoma punktami, skalowana wzdłuż swojej osi spoczynkowej
            auto role { w[ 0 ] };
            auto const optional { role[ 0 ] == '?' };
            if( optional ) { role.erase( 0, 1 ); }
            auto const from { point( w[ 1 ] ) };
            auto const to { point( w[ 2 ] ) };
            auto const span { length( to - from ) };
            m_result.values[ role + ".span" ] = span;
            if( optional && false == has_role( role ) ) { return; }
            auto const &stretched { part( role ) };
            auto placement { place( role, stretched.name, from, unit( to - from ), rest_axis( w[ 3 ] ) ) };
            auto const axis { ( w[ 3 ][ 1 ] == 'y' ? 1 : 2 ) };
            if( stretched.length > 1e-6 ) { placement.scale[ axis ] = span / stretched.length; }
            for( int across = 0; across < 3; ++across ) { if( across != axis ) { placement.scale[ across ] *= stretched.girth; } }
            put( placement );
            return;
        }

        if( op == "vee" ) {
            // dwie jednakowe części rozciągnięte od punktu From do dwóch punktów po obu stronach punktu To, rozsuniętych
            // wzdłuż toru (X) o +-Spread: linka w kształcie V. Druga część dostaje rolę <rola>_2
            auto role { w[ 0 ] };
            auto const optional { role[ 0 ] == '?' };
            if( optional ) { role.erase( 0, 1 ); }
            auto const from { point( w[ 1 ] ) };
            auto const to { point( w[ 2 ] ) };
            auto const spread { number( w[ 3 ] ) };
            auto const legs { std::sqrt( spread * spread + dot( to - from, to - from ) ) };
            m_result.values[ role + ".span" ] = legs;
            if( optional && false == has_role( role ) ) { return; }
            auto const &stretched { part( role ) };
            for( auto const side : { -1.0, 1.0 } ) {
                double const direction[ 3 ] { side * spread / legs, ( to.y - from.y ) / legs, ( to.z - from.z ) / legs };
                auto placement { place_3d( side < 0.0 ? role : role + "_2", stretched.name, from, direction, w[ 4 ] ) };
                auto const axis { ( w[ 4 ][ 1 ] == 'y' ? 1 : 2 ) };
                if( stretched.length > 1e-6 ) { placement.scale[ axis ] = legs / stretched.length; }
                for( int across = 0; across < 3; ++across ) { if( across != axis ) { placement.scale[ across ] *= stretched.girth; } }
                put( placement );
            }
            return;
        }

        if( op == "armropes" ) {
            // liny ramion (105x, 125x, 127x): od punktu From do ucha (holder) każdego postawionego ramienia (rola zaczynająca się od arms,
            // część z punktem holder) - przy parze dwie osobne liny. Linka obrócona o 90 stopni wokół własnej osi względem stretch
            // (płaska strona pionowo). Role: Rope, Rope.2, ...
            auto rope { w[ 0 ] };
            if( rope[ 0 ] == '?' ) { rope.erase( 0, 1 ); }
            if( false == has_role( rope ) ) { return; }
            auto const &cord { part( rope ) };
            auto const from2 { point( w[ 1 ] ) };
            double const from[ 3 ] { 0.0, from2.y, from2.z };
            std::vector<cantilever_placement> arms;
            for( auto const &placed : m_result.parts ) {
                if( placed.role.rfind( "arms", 0 ) != 0 ) { continue; }
                auto const *armpart { m_catalog.part( placed.part ) };
                if( armpart == nullptr || armpart->kind != "arms" || armpart->points.find( "holder" ) == armpart->points.end() ) { continue; }
                arms.push_back( placed );
            }
            for( std::size_t index = 0; index < arms.size(); ++index ) {
                auto const &arm { arms[ index ] };
                auto const *armpart { m_catalog.part( arm.part ) };
                auto const &hold { armpart->points.at( "holder" ) };
                auto const foundx { armpart->points_x.find( "holder" ) };
                auto const holdx { foundx != armpart->points_x.end() ? foundx->second : 0.0 };
                double to[ 3 ];
                for( int row = 0; row < 3; ++row ) {
                    to[ row ] = arm.position[ row ] + arm.rotation[ row * 3 + 0 ] * holdx + arm.rotation[ row * 3 + 1 ] * hold.first + arm.rotation[ row * 3 + 2 ] * hold.second;
                }
                double d[ 3 ] { to[ 0 ] - from[ 0 ], to[ 1 ] - from[ 1 ], to[ 2 ] - from[ 2 ] };
                auto const span { std::sqrt( d[ 0 ] * d[ 0 ] + d[ 1 ] * d[ 1 ] + d[ 2 ] * d[ 2 ] ) };
                if( span < 1e-6 ) { continue; }
                for( auto &value : d ) { value /= span; }
                double x[ 3 ] { 1.0 - d[ 0 ] * d[ 0 ], -d[ 0 ] * d[ 1 ], -d[ 0 ] * d[ 2 ] };
                auto const size { std::sqrt( x[ 0 ] * x[ 0 ] + x[ 1 ] * x[ 1 ] + x[ 2 ] * x[ 2 ] ) };
                for( auto &value : x ) { value /= size; }
                double const y[ 3 ] { d[ 1 ] * x[ 2 ] - d[ 2 ] * x[ 1 ], d[ 2 ] * x[ 0 ] - d[ 0 ] * x[ 2 ], d[ 0 ] * x[ 1 ] - d[ 1 ] * x[ 0 ] };
                cantilever_placement placement;
                placement.role = ( index == 0 ? rope : rope + "." + std::to_string( index + 1 ) );
                placement.part = cord.name;
                for( int row = 0; row < 3; ++row ) {
                    placement.position[ row ] = from[ row ];
                    // obrót o 90 stopni wokół osi linki: X części = dawne Y, Y części = -dawne X
                    placement.rotation[ row * 3 + 0 ] = y[ row ];
                    placement.rotation[ row * 3 + 1 ] = -x[ row ];
                    placement.rotation[ row * 3 + 2 ] = d[ row ];
                }
                if( cord.length > 1e-6 ) { placement.scale[ 2 ] = span / cord.length; }
                placement.scale[ 0 ] *= cord.girth; placement.scale[ 1 ] *= cord.girth;
                m_result.values[ placement.role + ".span" ] = span;
                put( placement );
            }
            return;
        }

        if( op == "armcords" ) {
            // linki ramion odciągowych do liny nośnej (106x, 1054): na każdym postawionym ramieniu (rola zaczynająca się od arms) z punktem
            // "cord_nom" obejma Clamp w tym punkcie (obrócona jak ramię, rura wzdłuż Y ramienia), z jej ucha linka Cord do punktu To
            // (lina nośna). Jedno ramię: linka w V do dwóch punktów rozsuniętych wzdłuż toru o +-Spread; kilka ramion: każde
            // pionowo do punktu To przesuniętego wzdłuż toru o +-Spread po stronie ramienia. Bez ramion z punktem cord_nom nic nie robi.
            auto cordrole { w[ 0 ] };
            if( cordrole[ 0 ] == '?' ) { cordrole.erase( 0, 1 ); }
            auto clamprole { w[ 1 ] };
            if( clamprole[ 0 ] == '?' ) { clamprole.erase( 0, 1 ); }
            if( false == has_role( cordrole ) || false == has_role( clamprole ) ) { return; }
            auto const &cord { part( cordrole ) };
            auto const &clamp { part( clamprole ) };
            auto const to { point( w[ 2 ] ) };
            auto const spread { number( w[ 3 ] ) };
            std::vector<cantilever_placement> arms;
            for( auto const &placed : m_result.parts ) {
                if( placed.role.rfind( "arms", 0 ) != 0 ) { continue; }
                auto const *armpart { m_catalog.part( placed.part ) };
                if( armpart == nullptr || armpart->points.find( "cord_nom" ) == armpart->points.end() ) { continue; }
                arms.push_back( placed );
            }
            if( arms.empty() ) { return; }
            auto const transform = []( cantilever_placement const &Placement, double const X, double const Y, double const Z, double ( &Out )[ 3 ] ) {
                for( int row = 0; row < 3; ++row ) {
                    Out[ row ] = Placement.position[ row ] + Placement.rotation[ row * 3 + 0 ] * X + Placement.rotation[ row * 3 + 1 ] * Y + Placement.rotation[ row * 3 + 2 ] * Z;
                }
            };
            auto const stretch = [ & ]( std::string const &Role, double const ( &From )[ 3 ], double const ( &To )[ 3 ] ) {
                double d[ 3 ] { To[ 0 ] - From[ 0 ], To[ 1 ] - From[ 1 ], To[ 2 ] - From[ 2 ] };
                auto const span { std::sqrt( d[ 0 ] * d[ 0 ] + d[ 1 ] * d[ 1 ] + d[ 2 ] * d[ 2 ] ) };
                if( span < 1e-6 ) { return; }
                for( auto &value : d ) { value /= span; }
                // Z części wzdłuż linki, X części: oś toru bez składowej wzdłuż linki
                double x[ 3 ] { 1.0 - d[ 0 ] * d[ 0 ], -d[ 0 ] * d[ 1 ], -d[ 0 ] * d[ 2 ] };
                auto const size { std::sqrt( x[ 0 ] * x[ 0 ] + x[ 1 ] * x[ 1 ] + x[ 2 ] * x[ 2 ] ) };
                for( auto &value : x ) { value /= size; }
                double const y[ 3 ] { d[ 1 ] * x[ 2 ] - d[ 2 ] * x[ 1 ], d[ 2 ] * x[ 0 ] - d[ 0 ] * x[ 2 ], d[ 0 ] * x[ 1 ] - d[ 1 ] * x[ 0 ] };
                cantilever_placement placement;
                placement.role = Role;
                placement.part = cord.name;
                for( int row = 0; row < 3; ++row ) {
                    placement.position[ row ] = From[ row ];
                    placement.rotation[ row * 3 + 0 ] = x[ row ];
                    placement.rotation[ row * 3 + 1 ] = y[ row ];
                    placement.rotation[ row * 3 + 2 ] = d[ row ];
                }
                if( cord.length > 1e-6 ) { placement.scale[ 2 ] = span / cord.length; }
                placement.scale[ 0 ] *= cord.girth; placement.scale[ 1 ] *= cord.girth;
                m_result.values[ Role + ".span" ] = span;
                put( placement );
            };
            for( std::size_t index = 0; index < arms.size(); ++index ) {
                auto const &arm { arms[ index ] };
                auto const *armpart { m_catalog.part( arm.part ) };
                auto const &nominal { armpart->points.at( "cord_nom" ) };
                auto const foundx { armpart->points_x.find( "cord_nom" ) };
                double at[ 3 ];
                transform( arm, ( foundx != armpart->points_x.end() ? foundx->second : 0.0 ), nominal.first, nominal.second, at );
                auto clampplacement { arm };
                clampplacement.role = clamprole + "." + std::to_string( index + 1 );
                clampplacement.part = clamp.name;
                clampplacement.model.clear();
                std::fill( std::begin( clampplacement.scale ), std::end( clampplacement.scale ), 1.0 );
                std::copy( std::begin( at ), std::end( at ), std::begin( clampplacement.position ) );
                put( clampplacement );
                double eye[ 3 ];
                transform( clampplacement, 0.0, 0.0, clamp.eye, eye );
                auto const suffix { "." + std::to_string( index + 1 ) };
                if( arms.size() == 1 ) {
                    for( auto const side : { -1.0, 1.0 } ) {
                        double const target[ 3 ] { side * spread, to.y, to.z };
                        stretch( cordrole + suffix + ( side < 0.0 ? "" : "_2" ), eye, target );
                    }
                }
                else {
                    double const target[ 3 ] { ( eye[ 0 ] < 0.0 ? -spread : spread ), to.y, to.z };
                    stretch( cordrole + suffix, eye, target );
                }
            }
            return;
        }

        if( op == "setarms" ) {
            // zestaw ramion bez wspornika na rurze (ramiona na linie, 105x, 125x): zestaw zerem w punkcie przewodu, Side jak w hangarms;
            // punkt zawieszenia (holder) zestawu trafia do punktu <rola>_holder. Bez roli zestawu nic nie robi
            auto setrole { w[ 0 ] };
            if( setrole[ 0 ] == '?' ) { setrole.erase( 0, 1 ); }
            if( false == has_role( setrole ) ) { return; }
            auto const &set { part( setrole ) };
            auto const flip { point( w[ 1 ] ).y < 0.0 };
            auto const found { set.points.find( "holder" ) };
            if( found == set.points.end() ) { fail( "part \"" + set.name + "\" has no \"holder\" point" ); }
            put_arm_set( setrole, set, flip );
            m_points[ setrole + "_holder" ] = m_points[ "wire" ] + vec2 { ( flip ? -1.0 : 1.0 ) * found->second.first, found->second.second };
            return;
        }

        if( op == "hangarms" ) {
            // zestaw ramion odciągowych: gotowy model z zerem w osi przewodu jezdnego i punktem "holder" (zawieszenie na wsporniku);
            // wspornik na rurze członu Line: zero na spodzie rury, wprost nad punktem zawieszenia. Wspornik z punktami
            // slide_min/slide_max (4240) ma zwis regulowany płynnie i głowicę (rola Head, punkt "eye" w głowicy); wspornik z punktem
            // "eye" (4250, 4260) jest stały. Zestaw po stronie toru przeciwnej słupowi (punkt Side z Y < 0) jest obrócony o 180 stopni.
            // Bez roli zestawu w schemacie klocek nic nie robi.
            auto setrole { w[ 0 ] };
            if( setrole[ 0 ] == '?' ) { setrole.erase( 0, 1 ); }
            if( false == has_role( setrole ) ) { return; }
            auto const &set { part( setrole ) };
            auto const &bracket { part( w[ 1 ] ) };
            auto headrole { w[ 2 ] };
            if( headrole[ 0 ] == '?' ) { headrole.erase( 0, 1 ); }
            auto const *head { has_role( headrole ) ? &part( headrole ) : nullptr };
            auto const &tube { line_named( w[ 3 ] ) };
            auto const flip { point( w[ 4 ] ).y < 0.0 };
            auto const side { flip ? -1.0 : 1.0 };
            auto const lookup = [ & ]( cantilever_part const &Part, char const *Name ) -> std::pair<double, double> const * {
                auto const found { Part.points.find( Name ) };
                return ( found != Part.points.end() ? &found->second : nullptr );
            };
            auto const *hold { lookup( set, "holder" ) };
            if( hold == nullptr ) { fail( "part \"" + set.name + "\" has no \"holder\" point" ); }
            // punkt zawieszenia względem przewodu, przy którym stoi zestaw (dwie sieci: przewód sieci B przestawiony na "wire")
            vec2 const hang { m_points[ "wire" ].y + side * hold->first, m_points[ "wire" ].z + hold->second };
            // zero wspornika: przesunięte od punktu zawieszenia o ucho głowicy (w głowicy ucho w -Y, po obróceniu w +Y)
            auto offset { 0.0 };
            if( head != nullptr ) {
                auto const *eye { lookup( *head, "eye" ) };
                if( eye != nullptr ) { offset = -side * eye->first; }
            }
            auto const bracket_y { hang.y + offset };
            // punkt na spodzie rury nad tym miejscem
            if( std::abs( tube.direction.y ) < 1e-6 ) { fail( "line \"" + w[ 3 ] + "\" is vertical, the arm holder can't be placed" ); }
            auto const radius { has_role( w[ 3 ] ) ? part( w[ 3 ] ).radius : 0.019 };
            // wspornik siedzi na rurze i przechyla się razem z nią (pochylony wysięg 6115 w 0194, Jan 2026-10-18): oś Y wspornika wzdłuż
            // rury w stronę słupa, Z prostopadle w górę; wspornik ze stałym uchem (bez zwisu i otworów) stawiany tak, żeby ucho wypadło
            // w poziomie punktu zawieszenia. Przy rurze poziomej wszystko jak dawniej
            vec2 const sup { tube.direction.y < 0.0 ? vec2 { -tube.direction.y, -tube.direction.z } : tube.direction };
            vec2 const up { -sup.z, sup.y };
            auto const *fixedeye { ( lookup( bracket, "slide_min" ) == nullptr && lookup( bracket, "hole_1" ) == nullptr ) ? lookup( bracket, "eye" ) : nullptr };
            auto along { ( bracket_y - tube.origin.y ) / tube.direction.y };
            if( fixedeye != nullptr && std::abs( up.y ) > 1e-9 ) {
                along = ( bracket_y - tube.origin.y + up.y * radius - sup.y * side * fixedeye->first - up.y * fixedeye->second ) / tube.direction.y;
            }
            auto const axis { tube.at( along ) };
            vec2 const zero { axis.y - up.y * radius, axis.z - up.z * radius };
            auto const turn = [ & ]( cantilever_placement &Placement ) {
                if( flip ) { for( int column : { 0, 1 } ) { for( int row = 0; row < 3; ++row ) { Placement.rotation[ row * 3 + column ] = -Placement.rotation[ row * 3 + column ]; } } }
            };
            put_arm_set( setrole, set, flip );
            auto const *slide_min { lookup( bracket, "slide_min" ) };
            auto const *slide_max { lookup( bracket, "slide_max" ) };
            auto drop { zero.z - hang.z };
            // ucho wspornika ze stałym uchem na pochylonej rurze: wysokość ucha liczona po przechyle
            if( fixedeye != nullptr ) { drop = zero.z + up.z * fixedeye->second - fixedeye->second - hang.z; }
            if( slide_min != nullptr && slide_max != nullptr ) {
                auto const low { -slide_min->second };
                auto const high { -slide_max->second };
                if( drop < low - 1e-3 || drop > high + 1e-3 ) {
                    warn( "arm holder \"" + bracket.name + "\" would need " + to_text( drop ) + " m between the tube and the arms, its range is " + to_text( low ) + "-" + to_text( high ) + " m" );
                }
                m_result.values[ setrole + ".slide" ] = drop;
                if( head != nullptr ) {
                    auto headplacement { place( headrole, head->name, vec2 { bracket_y, hang.z }, toward_support, toward_support ) };
                    turn( headplacement );
                    put( headplacement );
                }
            }
            else if( lookup( bracket, "hole_1" ) != nullptr ) {
                // wspornik z otworami (4290): głowica w otworze najbliższym punktowi zawieszenia
                std::pair<double, double> const *best { nullptr };
                std::string bestname;
                for( int hole = 1; hole < 20; ++hole ) {
                    auto const name { "hole_" + std::to_string( hole ) };
                    auto const *found { lookup( bracket, name.c_str() ) };
                    if( found == nullptr ) { break; }
                    if( best == nullptr || std::abs( drop + found->second ) < std::abs( drop + best->second ) ) { best = found; bestname = name; }
                }
                if( std::abs( drop + best->second ) > 0.01 ) {
                    warn( "no hole of arm holder \"" + bracket.name + "\" fits the arms, " + bestname + " is " + to_text( drop + best->second ) + " m off their hanging point" );
                }
                m_result.values[ setrole + ".hole" ] = std::stod( bestname.substr( 5 ) );
                if( head != nullptr ) {
                    auto headplacement { place( headrole, head->name, vec2 { bracket_y, zero.z + best->second }, toward_support, toward_support ) };
                    turn( headplacement );
                    put( headplacement );
                }
            }
            else {
                auto const *eye { lookup( bracket, "eye" ) };
                if( eye != nullptr && std::abs( drop + eye->second ) > 0.01 ) {
                    warn( "arm holder \"" + bracket.name + "\" holds the arms " + to_text( drop + eye->second ) + " m off their hanging point" );
                }
            }
            auto bracketplacement { place( w[ 1 ], bracket.name, zero, sup, toward_support ) };
            turn( bracketplacement );
            put( bracketplacement );
            // wspornik musi siedzieć na rurze
            auto const *tubepart { has_role( w[ 3 ] ) ? &part( w[ 3 ] ) : nullptr };
            if( tubepart != nullptr && ( along < 0.0 || along > tubepart->length ) ) {
                warn( "arm holder \"" + bracket.name + "\" falls " + to_text( along < 0.0 ? -along : along - tubepart->length ) + " m off the " + w[ 3 ] + " tube" );
            }
            return;
        }

        if( op == "registration" ) {
            // ramiona odciągowe: stałe położenie względem przewodów jezdnych
            if( m_registration == nullptr ) { fail( "unknown registration \"" + m_scheme.registration + "\"" ); }
            for( auto const &item : m_registration->items ) {
                cantilever_placement placement;
                placement.role = "registration";
                placement.part = item.part;
                std::copy( std::begin( item.position ), std::end( item.position ), std::begin( placement.position ) );
                std::copy( std::begin( item.rotation ), std::end( item.rotation ), std::begin( placement.rotation ) );
                m_result.parts.push_back( placement );
            }
            return;
        }

        fail( "unknown assembly step \"" + op + "\"" );
    }

    cantilever_catalog const &m_catalog;
    cantilever_scheme const &m_scheme;
    cantilever_input const &m_input;
    cantilever_registration const *m_registration;
    cantilever_result &m_result;
    std::string m_label;
    std::map<std::string, vec2> m_points;
    std::map<std::string, line> m_lines;

public:
    // kierunek członu, gdy istnieje (do wyników zgodnych z dawnym solverem)
    bool line_direction( std::string const &Name, vec2 &Direction ) const {
        auto const lookup { m_lines.find( Name ) };
        if( lookup == m_lines.end() ) { return false; }
        Direction = lookup->second.direction;
        return true;
    }
};

} // namespace

namespace {
bool solve_cantilever_once( cantilever_catalog const &Catalog, cantilever_scheme const &Scheme, cantilever_input const &Input, cantilever_result &Result ) {

    Result = {};
    auto const label { std::to_string( Scheme.card ) + "-" + std::to_string( Scheme.execution ) };
    auto const *assembly { Catalog.assembly( Scheme.assembly ) };
    if( assembly == nullptr ) {
        Result.warnings.push_back( "cantilever " + label + ": unknown assembly \"" + Scheme.assembly + "\"" );
        return false;
    }
    auto const *registration { Scheme.registration.empty() ? nullptr : Catalog.registration( Scheme.registration ) };
    if( false == Scheme.registration.empty() && registration == nullptr ) {
        Result.warnings.push_back( "cantilever " + label + ": unknown registration \"" + Scheme.registration + "\"" );
        return false;
    }
    assembler builder { Catalog, Scheme, Input, registration, Result };
    if( false == builder.run( assembly->steps ) ) { return false; }

    // wartości z nazwami używanymi przez wysięgniki przechylne
    vec2 strut;
    if( builder.line_direction( "strut", strut ) ) { Result.strut_angle = std::atan2( strut.z, -strut.y ); }
    auto const value = [ & ]( char const *Name, double const Default ) {
        auto const lookup { Result.values.find( Name ) };
        return ( lookup != Result.values.end() ? lookup->second : Default );
    };
    Result.strut_overhang = value( "strut.overhang", 0.0 );
    Result.steady_overhang = value( "steady.overhang", 0.0 );
    Result.stay_hole = static_cast<int>( value( "stay_clamp.hole", -1.0 ) );
    Result.stay_gap = value( "stay_clamp.gap", 0.0 );
    Result.cord_length = value( "cord.span", 0.0 );
    return true;
}

// podniesienia rury wysięgu do wypróbowania; zero, gdy schemat nie ma regulowanego uchwytu ramion
std::vector<double> tube_raises( cantilever_scheme const &Scheme ) {
    if( Scheme.tube_steps.empty() ) { return { 0.0 }; }
    auto raises { Scheme.tube_steps };
    // od najbliższego preferowanemu; przy równej odległości niższe
    std::stable_sort( raises.begin(), raises.end(), [ & ]( double const Left, double const Right ) {
        return std::abs( Left - Scheme.tube_prefer ) < std::abs( Right - Scheme.tube_prefer ) - 1e-9; } );
    return raises;
}

// wybór z kolejnych prób: najmniej ostrzeżeń, przy równej liczbie wcześniejsza (bliższa preferowanej) próba
template <typename Attempt>
bool solve_with_raises( cantilever_scheme const &Scheme, cantilever_result &Result, Attempt const &Solve ) {
    auto const raises { tube_raises( Scheme ) };
    // wypadnięcie wspornika ramion poza rurę nie steruje wysokością rury: zmiana wysokości go nie usuwa, a przesuwa resztę
    auto const scored = []( cantilever_result const &Tried ) {
        std::size_t count { 0 };
        for( auto const &warning : Tried.warnings ) { if( warning.find( ": arm holder" ) == std::string::npos || warning.find( " tube" ) == std::string::npos || warning.find( " falls " ) == std::string::npos ) { ++count; } }
        return count; };
    auto found { false };
    for( auto const raise : raises ) {
        cantilever_result attempt;
        auto const solved { Solve( raise, attempt ) };
        if( false == solved ) { if( false == found ) { Result = attempt; } continue; }
        attempt.values[ "tube.raise" ] = raise;
        if( false == found || scored( attempt ) < scored( Result ) ) { Result = attempt; found = true; }
        if( scored( Result ) == 0 ) { break; }
    }
    return found;
}
} // namespace

bool solve_cantilever( cantilever_catalog const &Catalog, cantilever_scheme const &Scheme, cantilever_input const &Input, cantilever_result &Result ) {

    if( Scheme.tube_steps.empty() ) { return solve_cantilever_once( Catalog, Scheme, Input, Result ); }
    auto const *registration { Catalog.registration( Scheme.registration ) };
    return solve_with_raises( Scheme, Result, [ & ]( double const Raise, cantilever_result &Attempt ) {
        auto input { Input };
        if( registration != nullptr ) { input.points[ "registration" ] = { registration->clamp, registration->tube + Raise }; }
        // zaczepy w schemacie odpowiadają podniesieniu preferowanemu
        input.strut_mount.z += Raise - Scheme.tube_prefer;
        return solve_cantilever_once( Catalog, Scheme, input, Attempt ); } );
}

cantilever_scene_part cantilever_to_scene( cantilever_placement const &Placement, double const Heading ) {

    cantilever_scene_part part;
    // układ wysięgnika (X wzdłuż toru, Y do konstrukcji, Z w górę) a lokalny układ scenerii: x = -X, y = Z, z = Y.
    // tak samo symulator przestawia osie wierzchołków modelu t3d, więc macierz obrotu części R przechodzi w C R C^T
    auto const &r { Placement.rotation };
    double const rotation[ 3 ][ 3 ] {
        {  r[ 0 ], -r[ 2 ], -r[ 1 ] },
        { -r[ 6 ],  r[ 8 ],  r[ 7 ] },
        { -r[ 3 ],  r[ 5 ],  r[ 4 ] } };
    // rozkład na obroty Y, X, Z w kolejności, w jakiej składa je symulator: R = Ry Rx Rz
    auto const pitch { -std::asin( std::clamp( rotation[ 1 ][ 2 ], -1.0, 1.0 ) ) };
    double yaw, roll;
    if( std::abs( std::cos( pitch ) ) > 1e-6 ) {
        yaw = std::atan2( rotation[ 0 ][ 2 ], rotation[ 2 ][ 2 ] );
        roll = std::atan2( rotation[ 1 ][ 0 ], rotation[ 1 ][ 1 ] );
    }
    else {
        // część ustawiona pionowo: obroty Y i Z się zlewają, cały idzie na Y
        yaw = std::atan2( -rotation[ 2 ][ 0 ], rotation[ 0 ][ 0 ] );
        roll = 0.0;
    }
    auto const degrees { 180.0 / 3.14159265358979323846 };
    part.angles[ 0 ] = pitch * degrees;
    part.angles[ 1 ] = ( yaw + Heading ) * degrees;
    part.angles[ 2 ] = roll * degrees;
    part.scale[ 0 ] = Placement.scale[ 0 ];
    part.scale[ 1 ] = Placement.scale[ 2 ];
    part.scale[ 2 ] = Placement.scale[ 1 ];
    // położenie: zamiana osi, potem obrót wokół Y o kierunek do konstrukcji
    auto const x { -Placement.position[ 0 ] };
    auto const z { Placement.position[ 1 ] };
    auto const cosine { std::cos( Heading ) };
    auto const sine { std::sin( Heading ) };
    part.offset[ 0 ] = x * cosine + z * sine;
    part.offset[ 1 ] = Placement.position[ 2 ];
    part.offset[ 2 ] = -x * sine + z * cosine;
    return part;
}

namespace {
bool solve_on_pole_once( cantilever_catalog const &Catalog, cantilever_scheme const &Scheme, cantilever_pole const &Pole,
    double const Distance, double const RailHead, cantilever_point const &Messenger, std::map<std::string, std::string> const &Overrides,
    std::map<std::string, cantilever_point> const &Points, cantilever_umbrella const &Umbrella, cantilever_result &Result );
}

bool solve_cantilever_on_pole( cantilever_catalog const &Catalog, cantilever_scheme const &Scheme, cantilever_pole const &Pole,
    double const Distance, double const RailHead, cantilever_point const &Messenger, std::map<std::string, std::string> const &Overrides, cantilever_result &Result ) {
    return solve_cantilever_on_pole( Catalog, Scheme, Pole, Distance, RailHead, Messenger, Overrides, cantilever_umbrella(), Result );
}

void umbrella_mounts( cantilever_catalog const &Catalog, cantilever_pole const &Pole, cantilever_result const &First, cantilever_result const &Second,
    cantilever_umbrella &FirstSide, cantilever_umbrella &SecondSide ) {

    FirstSide.shared.clear(); FirstSide.nuts.clear();
    SecondSide.shared.clear(); SecondSide.nuts.clear();
    std::map<std::string, std::string const *> const roles { { "stay", &Pole.stay }, { "strut", &Pole.strut }, { "steady", &Pole.steady } };
    auto const height = [ & ]( cantilever_result const &Result, std::string const &Role, double &Height ) {
        auto const *mount { Catalog.mount( *roles.at( Role ) ) };
        if( mount == nullptr || mount->twin.empty() ) { return false; }
        auto const found { Result.values.find( "mount." + Role ) };
        if( found == Result.values.end() ) { return false; }
        Height = found->second;
        return true;
    };
    struct candidate { double gap; std::string first; std::string second; int nutside; };   // nutside: 0 - First, 1 - Second
    std::vector<candidate> candidates;
    auto const consider = [ & ]( std::string const &A, std::string const &B, int const Nutside ) {
        double a, b;
        if( false == height( First, A, a ) || false == height( Second, B, b ) ) { return; }
        auto const gap { std::abs( a - b ) };
        if( gap > Catalog.rules.umbrella_spread + 1e-9 ) { return; }
        candidates.push_back( { gap, A, B, Nutside } );
    };
    for( auto const *role : { "stay", "strut", "steady" } ) { consider( role, role, 1 ); }
    consider( "strut", "stay", 1 );     // 4670: śruby po stronie ukośnika
    consider( "stay", "strut", 0 );
    std::stable_sort( candidates.begin(), candidates.end(), []( candidate const &L, candidate const &R ) { return L.gap < R.gap; } );
    for( auto const &pair : candidates ) {
        if( FirstSide.shared.count( pair.first ) || SecondSide.shared.count( pair.second ) ) { continue; }
        double a, b;
        height( First, pair.first, a );
        height( Second, pair.second, b );
        // średnia nie wychodzi ponad najwyższe położenie mocowania na słupie (odciąg ścięty do mount_max po jednej stronie)
        auto shared { 0.5 * ( a + b ) };
        if( Pole.mount_max != 0.0 ) { shared = std::min( shared, std::max( Pole.mount_max, std::min( a, b ) ) ); }
        FirstSide.shared[ pair.first ] = SecondSide.shared[ pair.second ] = shared;
        if( pair.nutside == 0 ) { FirstSide.nuts.insert( pair.first ); }
        else { SecondSide.nuts.insert( pair.second ); }
    }
}

bool solve_cantilever_on_pole( cantilever_catalog const &Catalog, cantilever_scheme const &Scheme, cantilever_pole const &Pole,
    double const Distance, double const RailHead, cantilever_point const &Messenger, std::map<std::string, std::string> const &Overrides,
    cantilever_umbrella const &Umbrella, cantilever_result &Result ) {

    auto const *registration { Catalog.registration( Scheme.registration ) };
    return solve_with_raises( Scheme, Result, [ & ]( double const Raise, cantilever_result &Attempt ) {
        // schemat bez regulacji, z dolnym zaczepem ukośnika podniesionym razem z rurą wysięgu
        auto scheme { Scheme };
        scheme.tube_steps.clear();
        scheme.tube_prefer = 0.0;
        scheme.mount_strut += Raise - Scheme.tube_prefer;
        std::map<std::string, cantilever_point> points;
        if( registration != nullptr && Raise != 0.0 ) { points[ "registration" ] = { registration->clamp, registration->tube + Raise }; }
        return solve_on_pole_once( Catalog, scheme, Pole, Distance, RailHead, Messenger, Overrides, points, Umbrella, Attempt ); } );
}

namespace {
bool solve_on_pole_once( cantilever_catalog const &Catalog, cantilever_scheme const &Scheme, cantilever_pole const &Pole,
    double const Distance, double const RailHead, cantilever_point const &Messenger, std::map<std::string, std::string> const &Overrides,
    std::map<std::string, cantilever_point> const &Points, cantilever_umbrella const &Umbrella, cantilever_result &Result ) {

    Result = cantilever_result();
    auto const label { std::to_string( Scheme.card ) + "-" + std::to_string( Scheme.execution ) + " on pole \"" + Pole.name + "\"" };
    auto const fail = [ & ]( std::string const &Reason ) { Result.warnings.push_back( "cantilever " + label + ": " + Reason ); return false; };
    auto const mountpart = [ & ]( std::string const &Name, char const *Use ) -> cantilever_mount const * {
        if( Name.empty() ) { fail( std::string( "the pole has no " ) + Use + " mount" ); return nullptr; }
        auto const *mount { Catalog.mount( Name ) };
        if( mount == nullptr ) { fail( std::string( Use ) + " mount \"" + Name + "\" isn't in the catalogue" ); }
        return mount;
    };
    auto const *strut { mountpart( Pole.strut, "strut" ) };
    auto const *stay { mountpart( Pole.stay, "stay" ) };
    auto const *steady { Scheme.steady_mounted ? mountpart( Pole.steady, "steady arm" ) : nullptr };
    if( strut == nullptr || stay == nullptr || ( Scheme.steady_mounted && steady == nullptr ) ) { return false; }

    std::vector<cantilever_placement> poleparts;
    auto const add = [ & ]( std::string const &Role, std::string const &Part, std::string const &Model, double const Y, double const Z ) {
        cantilever_placement placement;
        placement.role = Role;
        placement.part = Part;
        placement.model = Model;
        placement.position[ 1 ] = Y;
        placement.position[ 2 ] = Z;
        poleparts.push_back( placement );
    };
    std::vector<std::string> notes;
    // mocowanie z osią śrub na wysokości Bolts; zwraca punkt zaczepu izolatora
    std::map<std::string, double> heights;
    auto const fix = [ & ]( std::string const &Role, cantilever_mount const &Mount, double Bolts ) {
        // parasol: mocowanie dwustronne na wspólnej wysokości, bez tyłu; po stronie z nakrętkami jego połówka "twin"
        auto const shared { Umbrella.shared.find( Role ) };
        auto const twin { shared != Umbrella.shared.end() && false == Mount.twin.empty() };
        if( twin ) { Bolts = RailHead + shared->second; }
        auto const height { Bolts - RailHead };
        heights[ Role ] = height;
        if( height > Pole.top ) { notes.push_back( Role + " mount " + to_text( height - Pole.top ) + " m above the top of the pole" ); }
        auto const depth { Pole.depth( height ) };
        auto const face { Distance - 0.5 * depth };
        add( Role + "_mount", ( twin && Umbrella.nuts.count( Role ) ) ? Mount.twin : Mount.name, "", face, Bolts );
        if( false == twin && false == Mount.back.empty() ) { add( Role + "_back", Mount.back, "", face + depth, Bolts ); }
        if( false == Mount.hinge.empty() ) { add( Role + "_hinge", Mount.hinge, "", face + Mount.point[ 0 ], Bolts + Mount.point[ 1 ] ); }
        if( false == Mount.rod.empty() ) { add( Role + "_rod", Mount.rod, "", face + Mount.point[ 0 ], Bolts + Mount.point[ 1 ] ); }
        return cantilever_point { face + Mount.attach_y(), Bolts + Mount.attach_z() };
    };

    // słup bramki stawia bramka
    if( Umbrella.pole && false == Pole.gantry ) {
        add( "pole", Pole.name, Pole.model, Distance, RailHead );
        if( Pole.turn != 0.0 ) {
            // model obrócony wokół pionu (zwis na bramce: profil szerszym bokiem wzdłuż toru)
            auto const angle { Pole.turn * 3.14159265358979323846 / 180.0 };
            auto const c { std::cos( angle ) }, s { std::sin( angle ) };
            double const turned[ 9 ] { c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0 };
            std::copy( std::begin( turned ), std::end( turned ), poleparts.back().rotation );
        }
        // fundament ma to samo zero co słup (oś słupa na wysokości główki szyny)
        if( false == Pole.foundation.empty() ) { add( "foundation", Pole.name + "_foundation", Pole.foundation, Distance, RailHead ); }
    }
    auto const raise { Messenger.z - Scheme.height };
    auto staybolts { Scheme.mount_stay + raise - stay->attach_z() };
    if( Pole.mount_max != 0.0 && staybolts - RailHead > Pole.mount_max ) {
        notes.push_back( "the stay mount was lowered to the highest mount position of the pole (" + to_text( Pole.mount_max ) + " m above the rail head)" );
        staybolts = RailHead + Pole.mount_max;
    }
    // odciąg z góry (zaczep odciągu nad zaczepem ukośnika w schemacie), który po ścięciu do mount_max wypadłby na wysokości
    // dolnego zaczepu albo niżej: konstrukcja wyszłaby odwrócona (101x na słupie 1701 - Jan 2026-10-15), więc nie składamy
    {
        auto const strutbolts { Scheme.mount_strut + ( Scheme.steady_mounted ? raise : 0.0 ) - strut->attach_z() };
        auto const designgap { ( Scheme.mount_stay + raise - stay->attach_z() ) - strutbolts };
        if( designgap > 0.3 && staybolts - strutbolts < 0.5 * designgap ) {
            for( auto const &note : notes ) { Result.warnings.push_back( "cantilever " + label + ": " + note ); }
            return fail( "the pole is too short: the stay mount would have to be " + to_text( Scheme.mount_stay + raise - stay->attach_z() - RailHead ) + " m above the rail head, the pole allows "
                + to_text( Pole.mount_max ) + " m, which puts the stay at or below the strut; the cantilever isn't assembled" );
        }
    }
    cantilever_input input;
    input.messenger = Messenger;
    input.overrides = Overrides;
    input.points = Points;
    auto stayeye { fix( "stay", *stay, staybolts ) };
    input.strut_mount = fix( "strut", *strut, Scheme.mount_strut + ( Scheme.steady_mounted ? raise : 0.0 ) - strut->attach_z() );
    if( steady != nullptr ) { input.steady_mount = fix( "steady", *steady, Scheme.mount_steady - steady->attach_z() ); }
    // zwis na bramce: mocowania nie mogą zejść poniżej jego końca
    if( Pole.has_mount_min ) {
        for( auto const &height : heights ) {
            if( height.second < Pole.mount_min - 1e-6 ) { notes.push_back( height.first + " mount " + to_text( Pole.mount_min - height.second ) + " m below the lowest mount position of the pole" ); }
        }
    }

    // szekla odciągu wisi na uchu mocowania, a izolator na szekli: zaczep izolatora leży o długość szekli dalej wzdłuż odciągu,
    // którego kierunek zależy od złożenia, więc dwa przebiegi
    auto const shacklename { Overrides.count( "stay_shackle" ) ? Overrides.at( "stay_shackle" ) : ( Scheme.roles.count( "stay_shackle" ) ? Scheme.roles.at( "stay_shackle" ) : std::string() ) };
    auto const *shackle { shacklename.empty() ? nullptr : Catalog.part( shacklename ) };
    auto const shackle_length { shackle != nullptr ? shackle->length : 0.0 };
    input.stay_mount = { stayeye.y - shackle_length, stayeye.z };
    auto solved { solve_cantilever( Catalog, Scheme, input, Result ) };
    // zaczep odciągu przesunięty przez sposób składania (krok anchor): mocowanie na słupie idzie za nim
    auto const shifted { Result.values.find( "stay_mount.shift" ) };
    if( solved && shifted != Result.values.end() && std::abs( shifted->second ) > 1e-4 ) {
        auto const shift { shifted->second };
        poleparts.erase( std::remove_if( poleparts.begin(), poleparts.end(), []( cantilever_placement const &Placement ) { return Placement.role.compare( 0, 5, "stay_" ) == 0; } ), poleparts.end() );
        staybolts += shift;
        if( Pole.mount_max != 0.0 && staybolts - RailHead > Pole.mount_max + 1e-6 ) {
            notes.push_back( "the stay mount needs to be " + to_text( staybolts - RailHead - Pole.mount_max ) + " m above the highest mount position of the pole" );
        }
        stayeye = fix( "stay", *stay, staybolts );
        input.stay_mount = { stayeye.y - shackle_length, stayeye.z };
        solved = solve_cantilever( Catalog, Scheme, input, Result );
    }
    // zaczep ukośnika obniżony przez sposób składania (krok platform): mocowanie na słupie idzie za nim
    auto const dropped { Result.values.find( "strut.drop" ) };
    if( solved && dropped != Result.values.end() && std::abs( dropped->second ) > 1e-4 ) {
        auto const drop { dropped->second };
        poleparts.erase( std::remove_if( poleparts.begin(), poleparts.end(), []( cantilever_placement const &Placement ) { return Placement.role.compare( 0, 6, "strut_" ) == 0; } ), poleparts.end() );
        input.strut_mount = fix( "strut", *strut, Scheme.mount_strut + ( Scheme.steady_mounted ? raise : 0.0 ) - strut->attach_z() - drop );
        solved = solve_cantilever( Catalog, Scheme, input, Result );
    }
    if( solved && shackle_length > 0.0 ) {
        auto const *insulator { Result.find( "stay_insulator" ) };
        if( insulator == nullptr ) { insulator = Result.find( "stay" ); }   // odciąg bez izolatora (peronowe): cięgno wprost na szekli
        if( insulator != nullptr ) {
            // kierunek odciągu do toru to -Y części
            input.stay_mount = { stayeye.y - insulator->rotation[ 4 ] * shackle_length, stayeye.z - insulator->rotation[ 7 ] * shackle_length };
            solved = solve_cantilever( Catalog, Scheme, input, Result );
        }
    }
    for( auto const &note : notes ) { Result.warnings.push_back( "cantilever " + label + ": " + note ); }
    for( auto const &height : heights ) { Result.values[ "mount." + height.first ] = height.second; }
    if( false == solved ) { return false; }
    // wysięgnik pomocniczy bez ukośnika i odciągu (0167, 0169 - tylko zaczep na wysokości przewodu): bez ich mocowań na słupie
    if( Scheme.roles.count( "strut" ) == 0 && Scheme.roles.count( "stay" ) == 0 ) {
        poleparts.erase( std::remove_if( poleparts.begin(), poleparts.end(), []( cantilever_placement const &Placement ) {
            return Placement.role.compare( 0, 6, "strut_" ) == 0 || Placement.role.compare( 0, 5, "stay_" ) == 0; } ), poleparts.end() );
        Result.values.erase( "mount.stay" );
        Result.values.erase( "mount.strut" );
    }
    Result.parts.insert( Result.parts.end(), poleparts.begin(), poleparts.end() );
    return true;
}
} // namespace

} // namespace catenary

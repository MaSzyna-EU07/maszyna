/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/catenary_span.h"

#include <memory>

#include "world/Traction.h"
#include "world/catenary_network.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "rendering/renderer.h"
#include "utilities/utilities.h"

#include <fstream>
#include <locale>
#include <sstream>

catenary_settings CatenarySettings;

// stan przęsła rysowanego wg typu sieci
struct catenary_span {
    std::string symbol;                                         // typ sieci z dopisku "type"
    catenary::network_definition const *definition { nullptr }; // opis typu z katalogu; nullptr gdy brak albo przęsła nie da się policzyć
    std::vector<double> hangergaps;                             // odstępy wieszaków dobrane do rozpiętości; puste = odcinek bez wieszaków
    gfx::geometrybank_handle solidbank;                         // osobny bank na bryły tego przęsła
    catenary_solid solid;                                       // bryły do oglądania z bliska
    bool solidloaded { false };
    std::vector<catenary::solid_vertex> attached;               // bryła dołączona (lina kotwienia), układ scenerii
    std::shared_ptr<catenary::network_definition> own;          // kopia typu z naciągiem z dopisku "tension"
    bool rope2set[ 2 ] { false, false };                        // końce drugiej liny nośnej z dopisku "rope2"
    catenary::vec3 rope2[ 2 ];
};

namespace {


catenary::network_definition const *
definition( TTraction const &Traction ) {
    return ( Traction.m_catenaryspan ? Traction.m_catenaryspan->definition : nullptr );
}

catenary::span_points
points( TTraction const &Traction ) {
    catenary::span_points result {
        { Traction.pPoint1.x, Traction.pPoint1.y, Traction.pPoint1.z },
        { Traction.pPoint2.x, Traction.pPoint2.y, Traction.pPoint2.z },
        { Traction.pPoint3.x, Traction.pPoint3.y, Traction.pPoint3.z },
        { Traction.pPoint4.x, Traction.pPoint4.y, Traction.pPoint4.z } };
    if( Traction.m_catenaryspan ) {
        auto const &span { *Traction.m_catenaryspan };
        result.messenger2_start_set = span.rope2set[ 0 ];
        result.messenger2_end_set = span.rope2set[ 1 ];
        result.messenger2_start = span.rope2[ 0 ];
        result.messenger2_end = span.rope2[ 1 ];
    }
    return result;
}

// długość podwieszenia Y, jaką typ sieci przewiduje dla tego przęsła; 0 gdy brak
double
y_length( TTraction const &Traction ) {

    auto const *type { definition( Traction ) };
    if( type == nullptr ) { return 0.0; }
    if( Traction.m_catenaryspan->hangergaps.empty() ) { return 0.0; } // odcinek bez wieszaków
    if( type->type.y_tension <= 0.0 ) { return 0.0; }
    auto const length { std::hypot( Traction.pPoint2.x - Traction.pPoint1.x, Traction.pPoint2.z - Traction.pPoint1.z ) };
    auto const ylength { type->type.y_length( length ) };
    return ( ylength < length ? ylength : 0.0 );
}

// czy przy podwieszeniu od strony wskazanego końca (0: Point1, 1: Point2) jest linka Y wspólna z sąsiednim przęsłem.
// linka Y to jedno cięgno na dwa przęsła, więc jest tylko wtedy, gdy oba przęsła ją przewidują, w tej samej długości
// i z tych samych przewodów, a ich liny nośne schodzą się w jednym punkcie
bool
y_at( TTraction const &Traction, int const End ) {

    auto const ylength { y_length( Traction ) };
    if( ylength <= 0.0 ) { return false; }
    auto const *neighbour { Traction.hvNext[ End ] };
    if( neighbour == nullptr ) { return false; }
    auto const neighbourend { Traction.iNext[ End ] };
    if( neighbour->hvNext[ neighbourend ] != &Traction ) { return false; }
    if( std::abs( y_length( *neighbour ) - ylength ) > 0.01 ) { return false; }
    auto const &type { definition( Traction )->type };
    auto const &neighbourtype { definition( *neighbour )->type };
    if( type.y_tension != neighbourtype.y_tension
     || type.y_mass != neighbourtype.y_mass
     || type.messenger_count != neighbourtype.messenger_count ) {
        return false;
    }
    auto const &rope { End == 0 ? Traction.pPoint3 : Traction.pPoint4 };
    auto const &neighbourrope { neighbourend == 0 ? neighbour->pPoint3 : neighbour->pPoint4 };
    return ( glm::length( rope - neighbourrope ) < 0.05 );
}

// liczy przęsło osobno, z linkami Y tam, gdzie są wspólne z sąsiadami; false gdy się nie da (zgłasza błąd raz)
bool
solve( TTraction &Traction, catenary::span_solution &Solution ) {

    auto const *type { definition( Traction ) };
    if( type == nullptr ) { return false; }

    std::string error;
    if( false == catenary::solve_span_points( *type, Traction.m_catenaryspan->hangergaps, points( Traction ), y_at( Traction, 0 ), y_at( Traction, 1 ), Solution, error ) ) {
        ErrorLog(
            "Bad scenario: traction piece \"" + Traction.name() + "\" of type \"" + Traction.m_catenaryspan->symbol + "\" at location " + to_string( Traction.pPoint1 )
            + " can't be calculated, " + error + "; legacy geometry used instead" );
        Traction.m_catenaryspan->definition = nullptr;
        return false;
    }
    return true;
}

// połowa linki Y od strony wskazanego końca, do uzgodnienia z sąsiednim przęsłem; false gdy przęsła nie da się policzyć
bool
y_half( TTraction &Traction, int const End, catenary::y_half &Half ) {

    catenary::span_solution solution;
    if( false == solve( Traction, solution ) ) { return false; }
    Half = ( End == 0 ? solution.result.y_half_start : solution.result.y_half_end );
    return true;
}

// liczy przęsło i uzgadnia linki Y z sąsiednimi przęsłami, żeby połówki spotkały się w osi podwieszenia
bool
solve_joined( TTraction &Traction, catenary::span_solution &Solution ) {

    if( false == solve( Traction, Solution ) ) { return false; }
    for( int end = 0; end < 2; ++end ) {
        if( false == y_at( Traction, end ) ) { continue; }
        catenary::y_half neighbourhalf;
        if( y_half( *Traction.hvNext[ end ], Traction.iNext[ end ], neighbourhalf ) ) {
            catenary::join_y( *definition( Traction ), Solution, end == 1, neighbourhalf );
        }
        else {
            // sąsiad jednak nie da się policzyć i narysuje się po staremu, więc linki Y tu nie będzie
            if( false == solve( Traction, Solution ) ) { return false; }
            end = -1; // od nowa, już bez tego sąsiada
        }
    }
    return true;
}

// zbiera elementy przęsła: policzone, z linkami Y uzgodnionymi z sąsiadami także w planie
bool
build( TTraction &Traction, catenary::span_geometry &Geometry ) {

    catenary::span_solution solution;
    if( false == solve_joined( Traction, solution ) ) { return false; }
    auto const spanpoints { points( Traction ) };
    // przebieg linek Y w planie, wspólny z sąsiednim przęsłem: na łuku linka przechodzi obok załomu liny przy podwieszeniu
    catenary::y_plan plans[ 2 ];
    for( int end = 0; end < 2; ++end ) {
        if( false == y_at( Traction, end ) ) { continue; }
        auto &neighbour { *Traction.hvNext[ end ] };
        catenary::span_solution neighboursolution;
        if( false == solve_joined( neighbour, neighboursolution ) ) { continue; }
        plans[ end ] =
            catenary::solve_y_plan(
                *definition( Traction ),
                catenary::make_y_side( solution, spanpoints, end == 1 ),
                catenary::make_y_side( neighboursolution, points( neighbour ), Traction.iNext[ end ] == 1 ) );
    }
    catenary::build_geometry( *definition( Traction ), solution, spanpoints, Traction.WireOffset, Geometry, plans[ 0 ], plans[ 1 ] );
    return true;
}

gfx::basic_vertex
vertex( TTraction const &Traction, catenary::vec3 const &Point ) {
    gfx::basic_vertex result;
    result.position =
        glm::vec3(
            Point.x - Traction.m_origin.x,
            Point.y - Traction.m_origin.y,
            Point.z - Traction.m_origin.z );
    return result;
}

// zamienia bryły na wierzchołki do banku geometrii
gfx::vertex_array
vertices( TTraction const &Traction, std::vector<catenary::solid_vertex> const &Solid ) {
    gfx::vertex_array result;
    result.reserve( Solid.size() );
    for( auto const &point : Solid ) {
        auto item { vertex( Traction, point.position ) };
        item.normal = glm::vec3( point.normal.x, point.normal.y, point.normal.z );
        result.emplace_back( item );
    }
    return result;
}

// opróżnia fragment geometrii, zwalniając jego pamięć
void
release( gfx::geometry_handle const &Geometry ) {
    if( Geometry == null_handle ) { return; }
    gfx::vertex_array empty;
    gfx::userdata_array userdata;
    GfxRenderer->Replace( empty, userdata, Geometry, GL_TRIANGLES );
}

// wygląd metalu: [ metal: miedź, aluminium ][ przewód, osprzęt ][ stan: nowa, przejściowa, zaśniedziała ]
// osprzęt sieci miedzianej jest z brązu albo mosiądzu
catenary_metal Metals[ 2 ][ 2 ][ 3 ] {
    { { { { 0.80f, 0.50f, 0.27f }, 0.6f, 0.45f, 0.40f }, { { 0.50f, 0.30f, 0.16f }, 0.8f, 0.18f, 0.15f }, { { 0.30f, 0.52f, 0.46f }, 0.3f, 0.10f, 0.10f } },
      { { { 0.82f, 0.60f, 0.31f }, 0.6f, 0.40f, 0.35f }, { { 0.52f, 0.40f, 0.20f }, 0.8f, 0.15f, 0.15f }, { { 0.36f, 0.46f, 0.38f }, 0.4f, 0.10f, 0.10f } } },
    { { { { 0.80f, 0.81f, 0.83f }, 0.9f, 0.45f, 0.30f }, { { 0.55f, 0.55f, 0.56f }, 0.8f, 0.25f, 0.15f }, { { 0.30f, 0.30f, 0.31f }, 0.6f, 0.15f, 0.10f } },
      { { { 0.70f, 0.71f, 0.73f }, 0.9f, 0.35f, 0.30f }, { { 0.48f, 0.48f, 0.49f }, 0.8f, 0.20f, 0.15f }, { { 0.32f, 0.32f, 0.33f }, 0.6f, 0.15f, 0.10f } } } };
bool MetalsLoaded { false };

// czyta data/traction_materials.txt, o ile jest. wiersz: <copper|aluminium> <wire|fitting> <new|worn|tarnished> r g b metaliczność połysk odbicie
void
load_metals() {

    MetalsLoaded = true;
    std::ifstream file { "data/traction_materials.txt" };
    if( !file ) { return; }
    std::string line;
    int linenumber { 0 };
    while( std::getline( file, line ) ) {
        ++linenumber;
        auto const comment { line.find( "//" ) };
        if( comment != std::string::npos ) { line.erase( comment ); }
        std::istringstream stream { line };
        stream.imbue( std::locale::classic() );
        std::string metal, part, state;
        if( !( stream >> metal ) ) { continue; } // pusty wiersz
        catenary_metal value {};
        stream >> part >> state >> value.color[ 0 ] >> value.color[ 1 ] >> value.color[ 2 ] >> value.metalness >> value.gloss >> value.reflection;
        auto const metalindex { metal == "copper" ? 0 : metal == "aluminium" ? 1 : -1 };
        auto const partindex { part == "wire" ? 0 : part == "fitting" ? 1 : -1 };
        auto const stateindex { state == "new" ? 0 : state == "worn" ? 1 : state == "tarnished" ? 2 : -1 };
        if( stream.fail() || metalindex < 0 || partindex < 0 || stateindex < 0 ) {
            ErrorLog( "Bad file \"data/traction_materials.txt\" (line " + std::to_string( linenumber ) + "): expected <copper|aluminium> <wire|fitting> <new|worn|tarnished> and six numbers" );
            continue;
        }
        for( auto *number : { &value.color[ 0 ], &value.color[ 1 ], &value.color[ 2 ], &value.metalness, &value.gloss, &value.reflection } ) {
            *number = std::clamp( *number, 0.f, 1.f );
        }
        Metals[ metalindex ][ partindex ][ stateindex ] = value;
    }
}

} // namespace

// wygląd przewodów albo osprzętu przęsła o podanym materiale i stanie z wpisu scenerii
catenary_metal const &
catenary_span_metal( TTraction const &Traction, bool const Fitting ) {

    if( false == MetalsLoaded ) { load_metals(); }
    // stan z wpisu przęsła: bit 0 zaśniedziała, bit 1 przejściowa; zaśniedziała ma pierwszeństwo
    auto const state { ( Traction.DamageFlag & 1 ) != 0 ? 2 : ( Traction.DamageFlag & 2 ) != 0 ? 1 : 0 };
    return Metals[ Traction.Material == 2 ? 1 : 0 ][ Fitting ? 1 : 0 ][ state ];
}

// przypisuje przęsłu typ sieci z dopisku "type" i dobiera rozstaw wieszaków
void
catenary_span_assign( TTraction &Traction, std::string const &Symbol, double const Messenger, double const Contact, glm::dvec3 const *Rope2, bool const *Rope2set ) {

    Traction.m_catenaryspan.reset();
    if( Symbol.empty() ) { return; }

    auto span { std::make_shared<catenary_span>() };
    span->symbol = Symbol;
    for( int end = 0; end < 2 && Rope2 != nullptr && Rope2set != nullptr; ++end ) {
        span->rope2set[ end ] = Rope2set[ end ];
        span->rope2[ end ] = catenary::vec3 { Rope2[ end ].x, Rope2[ end ].y, Rope2[ end ].z };
    }
    Traction.m_catenaryspan = span;
    if( false == Global.CatenaryProcedural ) { return; }

    auto &registry { catenary::types() };
    if( !registry.on_error ) {
        registry.on_error = []( std::string const &Message ) { ErrorLog( Message ); };
    }
    span->definition = registry.find( Symbol );
    if( span->definition == nullptr ) { return; } // brak typu został już zgłoszony, przęsło rysuje się po staremu
    // naciąg z dopisku "tension" (daN): przęsło liczone z kopią typu
    if( Messenger > 0.0 || Contact > 0.0 ) {
        span->own = std::make_shared<catenary::network_definition>( *span->definition );
        if( Messenger > 0.0 ) { span->own->type.messenger_tension = 10.0 * Messenger; }
        if( Contact > 0.0 && span->own->type.contact_tension > 0.0 ) { span->own->type.contact_tension = 10.0 * Contact; }
        span->definition = span->own.get();
    }
    // odstęp wieszaków 0 w scenerii oznacza odcinek bez wieszaków (do kotwienia albo urządzenia naprężającego):
    // rysowane są same liny i przewody w liczbie wg typu, a rozstaw wieszaków zostaje pusty
    if( Traction.iNumSections == 0 ) { return; }
    span->hangergaps =
        registry.hanger_gaps(
            span->definition->hanger_table,
            std::hypot( Traction.pPoint2.x - Traction.pPoint1.x, Traction.pPoint2.z - Traction.pPoint1.z ) );
}

// wygląd liny stalowej (ocynk, lekko zmatowiały)
catenary_metal const &
catenary_rope_metal() {
    static catenary_metal const steel { { 0.56f, 0.57f, 0.58f }, 0.8f, 0.30f, 0.20f };
    return steel;
}

// symbol typu sieci przypisany przęsłu; pusty gdy brak
std::string
catenary_span_symbol( TTraction const &Traction ) {
    return ( Traction.m_catenaryspan ? Traction.m_catenaryspan->symbol : std::string() );
}

// odcinki przęsła policzone wg typu sieci; false gdy przęsło ma być narysowane po staremu
bool
catenary_span_lines( TTraction &Traction, gfx::vertex_array &Vertices ) {

    if( definition( Traction ) == nullptr ) { return false; }
    catenary::span_geometry geometry;
    if( false == build( Traction, geometry ) ) { return false; }
    Vertices.reserve( Vertices.size() + 2 * geometry.elements.size() );
    for( auto const &element : geometry.elements ) {
        Vertices.emplace_back( vertex( Traction, element.from ) );
        Vertices.emplace_back( vertex( Traction, element.to ) );
    }
    return true;
}

// bryły przęsła do oglądania z bliska; tworzone, gdy kamera jest blisko przęsła, i zwalniane, gdy się oddali
catenary_solid
catenary_span_solid( TTraction &Traction, glm::dvec3 const &Camera ) {

    if( definition( Traction ) == nullptr ) { return {}; }
    auto &span { *Traction.m_catenaryspan };
    // odległość kamery od odcinka łączącego końce przewodu jezdnego
    auto const axis { Traction.pPoint2 - Traction.pPoint1 };
    auto const lengthsquared { glm::dot( axis, axis ) };
    auto const t { lengthsquared > 0.0 ? std::clamp( glm::dot( Camera - Traction.pPoint1, axis ) / lengthsquared, 0.0, 1.0 ) : 0.0 };
    // przybliżenie kamery i rozdzielczość skracają odległość tak samo, jak przy doborze szczegółowości reszty scenerii
    auto const distance {
        glm::length( Camera - ( Traction.pPoint1 + axis * t ) )
        / ( std::max( 0.01, static_cast<double>( Global.ZoomFactor ) ) * std::sqrt( std::max( 0.01, static_cast<double>( Global.fDistanceFactor ) ) ) ) };
    // bryły powstają w zasięgu z ustawień i są zwalniane z zapasem, żeby nie migały na granicy
    auto const loaddistance { static_cast<double>( CatenarySettings.solid_range ) };
    auto const releasedistance { loaddistance * 1.25 + 10.0 };

    if( span.solidloaded ) {
        if( distance > releasedistance || loaddistance <= 0.0 ) {
            // kamera daleko: zwolnienie pamięci, linie wystarczą
            release( span.solid.wires );
            release( span.solid.fittings );
            release( span.solid.rope );
            span.solid = {};
            span.solidloaded = false;
            return {};
        }
        return span.solid;
    }
    if( loaddistance <= 0.0 || distance > loaddistance ) { return {}; }

    catenary::span_geometry geometry;
    if( false == build( Traction, geometry ) ) { return {}; }
    std::vector<catenary::solid_vertex> wires, fittings;
    catenary::build_solid( *definition( Traction ), geometry, wires, fittings );
    if( wires.empty() ) { return {}; }
    if( span.solidbank == null_handle ) {
        span.solidbank = GfxRenderer->Create_Bank();
    }
    // każde ponowne wczytanie tworzy nowe fragmenty w banku tego przęsła; poprzednie zostały opróżnione przy zwalnianiu
    gfx::userdata_array userdata;
    {
        auto data { vertices( Traction, wires ) };
        span.solid.wires = GfxRenderer->Insert( data, userdata, span.solidbank, GL_TRIANGLES );
    }
    if( false == fittings.empty() ) {
        auto data { vertices( Traction, fittings ) };
        span.solid.fittings = GfxRenderer->Insert( data, userdata, span.solidbank, GL_TRIANGLES );
    }
    if( false == span.attached.empty() ) {
        auto data { vertices( Traction, span.attached ) };
        span.solid.rope = GfxRenderer->Insert( data, userdata, span.solidbank, GL_TRIANGLES );
    }
    span.solidloaded = ( span.solid.wires != null_handle );
    return ( span.solidloaded ? span.solid : catenary_solid() );
}

namespace {

// bryły przęsła zwolnione; zrobią się od nowa, gdy kamera podejdzie
void
drop_solid( TTraction &Traction ) {
    if( false == static_cast<bool>( Traction.m_catenaryspan ) ) { return; }
    auto &span { *Traction.m_catenaryspan };
    if( false == span.solidloaded ) { return; }
    release( span.solid.wires );
    release( span.solid.fittings );
    release( span.solid.rope );
    span.solid = {};
    span.solidloaded = false;
}

} // namespace

// przęsło po zmianie sąsiadów (przęsło obok zrobione albo usunięte w edytorze): linie liczone od nowa, bryły do zrobienia
// od nowa. kształt przęsła zależy od sąsiadów (wspólne linki Y), a linie powstają raz, przy tworzeniu geometrii sekcji,
// i bryły też są trzymane, dopóki kamera jest blisko – bez tego zostawały policzone dla poprzednich sąsiadów
void
catenary_span_refresh( TTraction &Traction ) {
    drop_solid( Traction );
    if( Traction.m_geometry == null_handle ) { return; }
    gfx::vertex_array vertices;
    if( false == catenary_span_lines( Traction, vertices ) ) { return; }
    gfx::userdata_array userdata;
    GfxRenderer->Replace( vertices, userdata, Traction.m_geometry, GL_LINES );
}

// zwalnia linie i bryły usuwanego przęsła
void
catenary_span_release( TTraction &Traction ) {
    drop_solid( Traction );
    release( Traction.m_geometry );
}

// bryła spoza przęsła rysowana razem z nim (lina kotwienia); zrobi się przy następnym podejściu kamery
bool
catenary_span_attach( TTraction &Traction, std::vector<catenary::solid_vertex> const &Triangles ) {
    if( false == static_cast<bool>( Traction.m_catenaryspan ) ) { return false; }
    drop_solid( Traction );
    Traction.m_catenaryspan->attached = Triangles;
    return ( Traction.m_catenaryspan->definition != nullptr );
}

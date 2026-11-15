/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/TrackRails.h"
#include "rendering/renderer.h"

namespace track_rails {

namespace {

// in the colour of the top of the rail foot
glm::vec2 cap_texture( std::vector<gfx::basic_vertex> const &Ring, std::vector<glm::dvec2> const &Plan ) {

    auto bottom { 0.0 };
    auto top { 0.0 };
    for( auto const &point : Plan ) {
        bottom = std::min( bottom, point.y );
        top = std::max( top, point.y );
    }
    auto web { Ring[ 0 ].texture };
    auto nearest { std::numeric_limits<double>::max() };
    for( std::size_t j = 0; j < Ring.size() / 2; ++j ) {
        auto const distance { std::abs( Plan[ j ].y - 0.5 * ( bottom + top ) ) };
        if( distance < nearest ) {
            nearest = distance;
            web = Ring[ j ].texture;
        }
    }
    auto lowest { std::numeric_limits<double>::max() };
    for( std::size_t j = 0; j < Ring.size() / 2; ++j ) {
        if( Ring[ j ].normal.y > 0.5f && Plan[ j ].y < lowest ) {
            lowest = Plan[ j ].y;
            web = Ring[ j ].texture;
        }
    }
    return web;
}

// how the outline turns at B, coming from A and going on to C: above zero to the left
double corner_cross( glm::dvec2 const &A, glm::dvec2 const &B, glm::dvec2 const &C ) {

    return ( B.x - A.x ) * ( C.y - A.y ) - ( B.y - A.y ) * ( C.x - A.x );
}

// which way the outline winds: 1 anticlockwise, -1 clockwise
double winding_sign( std::vector<glm::dvec2> const &Polygon ) {

    double area { 0.0 };
    for( std::size_t j = 0; j < Polygon.size(); ++j ) {
        auto const &a { Polygon[ j ] };
        auto const &b { Polygon[ ( j + 1 ) % Polygon.size() ] };
        area += a.x * b.y - b.x * a.y;
    }
    return ( area < 0.0 ? -1.0 : 1.0 );
}

// the corner J of what's left of the outline is of no width: its points lie on a line
bool thin_corner( std::vector<glm::dvec2> const &Polygon, std::vector<std::size_t> const &Remaining, std::size_t const J ) {

    auto const &a { Polygon[ Remaining[ ( J + Remaining.size() - 1 ) % Remaining.size() ] ] };
    auto const &b { Polygon[ Remaining[ J ] ] };
    auto const &c { Polygon[ Remaining[ ( J + 1 ) % Remaining.size() ] ] };
    auto const longest { std::max( { glm::length( b - a ), glm::length( c - b ), glm::length( a - c ) } ) };
    return ( longest <= 0.0 || std::abs( corner_cross( a, b, c ) ) / longest < 1e-4 );
}

// the first corner of no width of what's left of the outline taken out of it; false: there's none
bool prune_thin( std::vector<glm::dvec2> const &Polygon, std::vector<std::size_t> &Remaining ) {

    auto const count { Remaining.size() };
    for( std::size_t j = 0; j < count; ++j ) {
        if( thin_corner( Polygon, Remaining, j ) ) {
            Remaining.erase( Remaining.begin() + j );
            return true;
        }
    }
    return false;
}

// a corner of what's left of the outline, but those at A, B and C, lies within the ear of them
bool corner_within( std::vector<glm::dvec2> const &Polygon, std::vector<std::size_t> const &Remaining, glm::dvec2 const &A, glm::dvec2 const &B, glm::dvec2 const &C, double const Orientation ) {

    return std::ranges::any_of( Remaining, [ &Polygon, &A, &B, &C, Orientation ]( std::size_t const K ) {
        auto const &p { Polygon[ K ] };
        return p != A && p != B && p != C
            && corner_cross( A, B, p ) * Orientation > 1e-12 && corner_cross( B, C, p ) * Orientation > 1e-12 && corner_cross( C, A, p ) * Orientation > 1e-12; } );
}

// how the side of the outline from P to Q winds around the point: 1 going up past it with it on the left, -1 going down past it with it on the right
int side_winding( glm::dvec2 const &P, glm::dvec2 const &Q, glm::dvec2 const &Point ) {

    if( P.y <= Point.y && Q.y > Point.y && corner_cross( P, Q, Point ) > 0.0 ) {
        return 1;
    }
    if( Q.y <= Point.y && P.y > Point.y && corner_cross( P, Q, Point ) < 0.0 ) {
        return -1;
    }
    return 0;
}

// how many times what's left of the outline winds around the point
int winding_around( std::vector<glm::dvec2> const &Polygon, std::vector<std::size_t> const &Remaining, glm::dvec2 const &Point ) {

    auto winding { 0 };
    for( std::size_t k = 0; k < Remaining.size(); ++k ) {
        winding += side_winding( Polygon[ Remaining[ k ] ], Polygon[ Remaining[ ( k + 1 ) % Remaining.size() ] ], Point );
    }
    return winding;
}

// the corner J of what's left of the outline is an ear to cut off: it turns the way the outline winds, or not at all, with nothing of the outline within it
bool is_ear( std::vector<glm::dvec2> const &Polygon, std::vector<std::size_t> const &Remaining, std::size_t const J, double const Orientation ) {

    auto const &a { Polygon[ Remaining[ ( J + Remaining.size() - 1 ) % Remaining.size() ] ] };
    auto const &b { Polygon[ Remaining[ J ] ] };
    auto const &c { Polygon[ Remaining[ ( J + 1 ) % Remaining.size() ] ] };
    auto const turn { corner_cross( a, b, c ) * Orientation };
    if( turn < -1e-12 ) {
        return false;
    }
    if( turn > 1e-12 ) {
        if( corner_within( Polygon, Remaining, a, b, c, Orientation ) ) {
            return false;
        }
        // an outline narrower than a millimetre in places lets an ear past the test above across a notch of it, with no corner inside;
        // the middle of the ear tells: out of the outline, the ear isn't one; the outline of a planed rail may go over itself, so it's what it winds around
        return ( winding_around( Polygon, Remaining, ( a + b + c ) / 3.0 ) != 0 );
    }
    return true;
}

// the first corner of what's left of the outline that is an ear; the count of the corners: none is
std::size_t find_ear( std::vector<glm::dvec2> const &Polygon, std::vector<std::size_t> const &Remaining, double const Orientation ) {

    for( std::size_t j = 0; j < Remaining.size(); ++j ) {
        if( is_ear( Polygon, Remaining, j, Orientation ) ) {
            return j;
        }
    }
    return Remaining.size();
}

std::vector<std::array<std::size_t, 3>> ear_clip( std::vector<glm::dvec2> const &Polygon ) {

    auto const orientation { winding_sign( Polygon ) };
    std::vector<std::size_t> remaining( Polygon.size() );
    for( std::size_t j = 0; j < Polygon.size(); ++j ) {
        remaining[ j ] = j;
    }
    // the parts of the outline of no width, as the web and the head of a rail planed away to nothing at the tip of a nose with its foot whole,
    // go out first: the ears cut across them would cover what lies beside them
    auto pruned { true };
    while( pruned && remaining.size() > 3 ) {
        pruned = prune_thin( Polygon, remaining );
    }
    std::vector<std::array<std::size_t, 3>> triangles;
    if( remaining.size() < 3 || ( remaining.size() == 3 && thin_corner( Polygon, remaining, 0 ) ) ) {
        return triangles;
    }
    while( remaining.size() > 3 ) {
        auto ear { find_ear( Polygon, remaining, orientation ) };
        if( ear == remaining.size() ) {
            ear = 0;
        }
        std::array const corners { remaining[ ( ear + remaining.size() - 1 ) % remaining.size() ], remaining[ ear ], remaining[ ( ear + 1 ) % remaining.size() ] };
        triangles.push_back( corners );
        remaining.erase( remaining.begin() + ear );
    }
    std::array const closing { remaining[ 0 ], remaining[ 1 ], remaining[ 2 ] };
    triangles.push_back( closing );
    return triangles;
}

// the face closing a lofted strip at an end of it, of the points of its cross-section there, facing Direction: both sides of each triangle of it
void cap_face( std::vector<gfx::basic_vertex> const &Ring, glm::vec3 const &Direction, gfx::vertex_array &Triangles ) {

    if( glm::length2( Direction ) == 0.f ) {
        return;
    }
    auto const outward { glm::normalize( Direction ) };
    auto across { glm::vec3{ -outward.z, 0.f, outward.x } };
    if( glm::length2( across ) == 0.f ) {
        return;
    }
    across = glm::normalize( across );
    std::vector<glm::dvec2> plan;
    for( auto const &vertex : Ring ) {
        auto const offset { vertex.position - Ring[ 0 ].position };
        plan.emplace_back( glm::dot( offset, across ), offset.y );
    }
    auto const web { cap_texture( Ring, plan ) };
    auto const triangles { ear_clip( plan ) };
    for( auto const &triangle : triangles ) {
        for( auto const &order : { std::array { 0, 1, 2 }, std::array { 0, 2, 1 } } ) {
            for( auto const k : order ) {
                auto const index { triangle[ k ] };
                Triangles.emplace_back( Ring[ index ].position, outward, web );
            }
        }
    }
}

} // namespace

void cap_loft( gfx::vertex_array const &Strip, std::size_t const Ring, bool const Start, bool const End, gfx::vertex_array &Triangles ) {

    if( Ring < 3 || Strip.size() < 2 * Ring ) {
        return;
    }
    std::vector<std::pair<std::vector<gfx::basic_vertex>, glm::vec3>> caps;
    if( Start ) {
        std::vector<gfx::basic_vertex> ring;
        for( std::size_t j = 0; j < Ring; ++j ) {
            ring.emplace_back( Strip[ 2 * j ] );
        }
        caps.emplace_back( ring, Strip[ 0 ].position - Strip[ 1 ].position );
    }
    if( End ) {
        auto const base { Strip.size() - 2 * Ring };
        std::vector<gfx::basic_vertex> ring;
        for( std::size_t j = 0; j < Ring; ++j ) {
            ring.emplace_back( Strip[ base + 2 * j + 1 ] );
        }
        caps.emplace_back( ring, Strip[ base + 1 ].position - Strip[ base ].position );
    }
    for( auto const &[ ring, direction ] : caps ) {
        cap_face( ring, direction, Triangles );
    }
}

gfx::userdata_array worn_userdata( std::size_t const Count, bool const Worn ) {

    return gfx::userdata_array( Count, gfx::vertex_userdata{ glm::vec4{ Worn ? 1.f : 0.f, 0.f, 0.f, 0.f } } );
}

namespace {

// the pool the chunks of the geometry laid now go into, none: new chunks
chunk_pool *&reused_pool() {

    static chunk_pool *pool { nullptr };
    return pool;
}

} // namespace

reused_chunks::reused_chunks( chunk_pool *Pool ) :
    m_previous { reused_pool() } {

    if( Pool != nullptr ) {
        for( auto &chunk : *Pool ) {
            chunk.used = false;
        }
    }
    reused_pool() = Pool;
}

reused_chunks::~reused_chunks() {

    reused_pool() = m_previous;
}

gfx::geometry_handle insert_chunk( gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Bank, int const Type ) {

    auto *const reused { reused_pool() };
    if( reused == nullptr ) {
        return GfxRenderer->Insert( Vertices, Userdata, Bank, Type );
    }
    for( auto &chunk : *reused ) {
        if( chunk.used || chunk.type != Type || chunk.handle.bank != Bank.bank ) {
            continue;
        }
        // the data are swapped into the chunk when of the same size, the caller keeps its own
        auto vertices { Vertices };
        auto userdata { Userdata };
        if( GfxRenderer->Replace( vertices, userdata, chunk.handle, Type ) ) {
            chunk.used = true;
            return chunk.handle;
        }
    }
    auto const handle { GfxRenderer->Insert( Vertices, Userdata, Bank, Type ) };
    if( handle != null_handle ) {
        reused->push_back( { Type, handle, true } );
    }
    return handle;
}

void cut_faces( gfx::vertex_array const &Strip, std::size_t const Ring, bool const Start, bool const End, gfx::geometrybank_handle const &Bank, TTrack::geometryhandle_sequence &Geometry, bool const Worn ) {

    gfx::vertex_array triangles;
    cap_loft( Strip, Ring, Start, End, triangles );
    if( false == triangles.empty() ) {
        auto userdata { worn_userdata( triangles.size(), Worn ) };
        Geometry.emplace_back( insert_chunk( triangles, userdata, Bank, GL_TRIANGLES ) );
    }
}

} // namespace track_rails

using namespace track_rails;

namespace {

gfx::vertex_array box_profile( float const Bottom, float const Top, float const Texture, float const Spread = 0.f ) {

    return {
        {{  1.f, Bottom, 0.f }, {  1.f, 0.f, 0.f }, { Texture + Spread, 0.f }},
        {{  1.f, Top, 0.f }, {  1.f, 0.f, 0.f }, { Texture + Spread, 0.f }},
        {{  1.f, Top, 0.f }, {  0.f, 1.f, 0.f }, { Texture + Spread, 0.f }},
        {{ -1.f, Top, 0.f }, {  0.f, 1.f, 0.f }, { Texture - Spread, 0.f }},
        {{ -1.f, Top, 0.f }, { -1.f, 0.f, 0.f }, { Texture - Spread, 0.f }},
        {{ -1.f, Bottom, 0.f }, { -1.f, 0.f, 0.f }, { Texture - Spread, 0.f }} };
}

// the highest point of the rail outline, on top of its head: how high it lies and the texture there
std::pair<float, float> highest_point( gfx::vertex_array const &Outline ) {

    auto texture { 0.f };
    auto highest { -1e9f };
    for( std::size_t i = 0; i < Outline.size() / 2; ++i ) {
        if( Outline[ i ].position.y > highest ) {
            highest = Outline[ i ].position.y;
            texture = Outline[ i ].texture.s;
        }
    }
    return { highest, texture };
}

// the texture of the running surface of a grooved rail outline: the mean of it over the points on top of the head, off the lip
float grooved_texture( gfx::vertex_array const &Outline, float const Highest ) {

    auto sum { 0.f };
    auto points { 0 };
    for( std::size_t i = 0; i < Outline.size() / 2; ++i ) {
        if( Outline[ i ].position.y > Highest - 0.0005f && Outline[ i ].position.x > -0.02f ) {
            sum += Outline[ i ].texture.s;
            ++points;
        }
    }
    return sum / static_cast<float>( std::max( points, 1 ) );
}

// the texture of the first point of the rail outline below the neck facing up, on top of the foot; Texture where there's none
float web_texture( gfx::vertex_array const &Outline, float const Neck, float const Texture ) {

    for( std::size_t i = 0; i < Outline.size() / 2; ++i ) {
        if( auto const &point { Outline[ i ] }; point.position.y < Neck && point.normal.y > 0.9f ) {
            return point.texture.s;
        }
    }
    return Texture;
}

// how fast the texture of the rail outline runs across the top of its head, by the width of it
float top_spread( gfx::vertex_array const &Outline, float const Highest ) {

    glm::vec2 across { 1e9f, -1e9f };
    glm::vec2 band { 1e9f, -1e9f };
    for( std::size_t i = 0; i < Outline.size() / 2; ++i ) {
        if( Outline[ i ].position.y > Highest - 0.0005f ) {
            across = { std::min( across.x, Outline[ i ].position.x ), std::max( across.y, Outline[ i ].position.x ) };
            band = { std::min( band.x, Outline[ i ].texture.s ), std::max( band.y, Outline[ i ].texture.s ) };
        }
    }
    return ( across.y > across.x ? ( band.y - band.x ) / ( across.y - across.x ) : 0.f );
}

} // namespace

TTrack::rail_drawing TTrack::make_rail_drawing( gfx::geometrybank_handle const &Bank ) const {

    auto const sides = [ this ]( gfx::vertex_array const &Shape, std::array<gfx::vertex_array, 2> &Sides ) {
        create_track_rail_profile( Sides[ 0 ], Sides[ 1 ], &Shape );
        for( auto &side : Sides ) {
            side.resize( Shape.size() );
        } };
    rail_drawing drawing;
    drawing.bank = Bank;
    drawing.size = rail_dimensions();
    drawing.halfwidth = 0.5f * std::abs( fTrackWidth );
    // check rail: a narrow head facing the running rail across the flangeway, on a thin web with a foot turned away from it, sized after the rail
    gfx::vertex_array checkshape {
        {{ -0.041f, -0.170f, 0.f }, {  1.000f,  0.000f, 0.f }, { 0.15f, 0.f }},
        {{ -0.041f, -0.010f, 0.f }, {  0.707f,  0.707f, 0.f }, { 0.40f, 0.f }},
        {{ -0.051f, -0.000f, 0.f }, {  0.000f,  1.000f, 0.f }, { 0.45f, 0.f }},
        {{ -0.071f, -0.000f, 0.f }, {  0.000f,  1.000f, 0.f }, { 0.55f, 0.f }},
        {{ -0.081f, -0.010f, 0.f }, { -0.707f,  0.707f, 0.f }, { 0.60f, 0.f }},
        {{ -0.081f, -0.045f, 0.f }, { -0.707f, -0.707f, 0.f }, { 0.65f, 0.f }},
        {{ -0.066f, -0.060f, 0.f }, { -0.707f, -0.707f, 0.f }, { 0.75f, 0.f }},
        {{ -0.066f, -0.150f, 0.f }, { -0.707f,  0.707f, 0.f }, { 0.85f, 0.f }},
        {{ -0.101f, -0.170f, 0.f }, { -1.000f,  0.000f, 0.f }, { 1.00f, 0.f }} };
    for( auto &point : checkshape ) {
        point.position.x = -0.041f + ( point.position.x + 0.041f ) * static_cast<float>( drawing.size.head / 0.0735 );
        point.position.y *= static_cast<float>( drawing.size.height / 0.172 );
    }
    sides( checkshape, drawing.checkshapes );
    if( false == m_profiles.empty() ) {
        auto const &outline { track_rail_profile( m_profile1.second ) };
        auto const [ highest, headtexture ] { highest_point( outline ) };
        auto const texture { drawing.size.grooved ? grooved_texture( outline, highest ) : headtexture };
        auto const top { static_cast<float>( ( drawing.size.grooved ? drawing.size.liptop : highest ) - 0.0005 ) };
        auto const bottom { static_cast<float>( -drawing.size.height ) };
        drawing.platetexture = texture;
        sides( box_profile( bottom, top, texture ), drawing.plates );
        auto const neck { static_cast<float>( drawing.size.neck ) };
        auto const webtexture { web_texture( outline, neck, texture ) };
        auto const spread { top_spread( outline, highest ) };
        sides( box_profile( bottom, neck, webtexture, spread ), drawing.fillers );
    }
    return drawing;
}

namespace {

struct piece_ends {
    double from { 0.0 };
    double to { 0.0 };
    loft_end fromend;
    loft_end toend;
};

struct rail_piece {
    material_handle material { null_handle };
    double from { 0.0 };
    double to { 0.0 };
    knots::line offsets;
    knots::line extents;
    knots::line inner;
    bool capfrom { false };
    bool capto { false };
    bool rolled { true };
    knots::line ramp;
    double fade { blend };
    bool walls { false };
    float lift { 0.f };
    float wallu { -1.f };
};

struct top_band {
    material_handle material { null_handle };
    knots::line extents;
    knots::line inner;
    bool rolled { true };
    knots::line ramp;
    double fade { blend };
};

struct grooved_profile {
    gfx::vertex_array body;
    std::vector<gfx::vertex_array> tops;
    std::vector<gfx::vertex_array> flats;
    float wallu { -1.f };
};

glm::vec3 const plain_mark { 0.f, 1.f, 1.f };

bool treads_marked() {

    static auto const marked { std::getenv( "EU07_EDITOR_TREADS" ) != nullptr };
    return marked;
}

void sort_cuts( std::vector<rail_cut> &Cuts ) {

    std::ranges::sort( Cuts, []( rail_cut const &A, rail_cut const &B ) { return A.at < B.at; } );
}

void add_meetings( knots::line const &A, knots::line const &B, double const Start, double const End, double const Margin, std::vector<double> &Splits ) {

    for( auto const distance : knots::meetings( A, B ) ) {
        if( distance > Start + Margin && distance < End - Margin ) {
            Splits.push_back( distance );
        }
    }
}

std::vector<double> cross_splits( rail_piece const &Whole, glm::dvec2 const &Reach, std::initializer_list<knots::line const *> const Edges, double const Margin ) {

    std::vector result { Whole.from, Whole.to };
    knots::line const lowline { { Whole.from, Reach.x }, { Whole.to, Reach.x } };
    knots::line const highline { { Whole.from, Reach.y }, { Whole.to, Reach.y } };
    for( auto const *edge : Edges ) {
        for( auto const *limit : { &lowline, &highline, &Whole.extents, &Whole.inner } ) {
            add_meetings( *edge, *limit, Whole.from, Whole.to, Margin, result );
        }
    }
    std::ranges::sort( result );
    return result;
}

// the face of a check or a guard rail toward the running rail, planed off at the ends by Fromdepth and Todepth down to nothing over Length; lines for the upper envelope of the reach
void planed( knots::line &Lines, double const From, double const To, double const Length, double const Face, double const Fromdepth, double const Todepth ) {

    if( Fromdepth > 0.0 ) {
        auto const slope { Fromdepth / Length };
        Lines.emplace_back( slope, Face - Fromdepth - slope * From );
    }
    if( Todepth > 0.0 ) {
        auto const slope { Todepth / Length };
        Lines.emplace_back( -slope, Face - Todepth + slope * To );
    }
}

// where the line I of the lines crosses the ones after it, between A and B
void add_crossings( knots::line const &Lines, std::size_t const I, double const A, double const B, std::vector<double> &Stations ) {

    for( std::size_t j = I + 1; j < Lines.size(); ++j ) {
        if( std::abs( Lines[ I ].x - Lines[ j ].x ) > 1e-12 ) {
            auto const d { ( Lines[ j ].y - Lines[ I ].y ) / ( Lines[ I ].x - Lines[ j ].x ) };
            if( d > A && d < B ) {
                Stations.push_back( d );
            }
        }
    }
}

knots::line envelope( knots::line const &Lines, bool const Upper, double const A, double const B ) {

    knots::line result;
    if( Lines.empty() ) {
        return result;
    }
    std::vector stations { A, B };
    for( std::size_t i = 0; i < Lines.size(); ++i ) {
        add_crossings( Lines, i, A, B, stations );
    }
    std::ranges::sort( stations );
    for( auto const d : stations ) {
        auto value { Upper ? 1e6 : -1e6 };
        for( auto const &line : Lines ) {
            value = Upper ? std::min( value, line.x * d + line.y ) : std::max( value, line.x * d + line.y );
        }
        result.emplace_back( d, value );
    }
    return result;
}

rail_cut band_cut( rail_band const &Band, glm::dvec2 const &Reach ) {

    rail_cut cut;
    cut.at = Band.at;
    auto const before { Band.into > 0.0 ? Band.from : Band.to };
    auto const after { Band.into > 0.0 ? Band.to : Band.from };
    auto const meets = [&]( double const Edge, double const Across ) {
        return glm::dvec2{ Band.at + ( Edge + Band.facing * Across ) / Band.into, Across }; };
    if( std::abs( Band.facing ) < 1e-3 ) {
        cut.end = meets( before, 0.0 ).x;
        cut.start = meets( after, 0.0 ).x;
        return cut;
    }
    knots::line endknots { meets( before, Reach.x ), meets( before, Reach.y ) };
    knots::line startknots { meets( after, Reach.x ), meets( after, Reach.y ) };
    knots::sort( endknots );
    knots::sort( startknots );
    cut.end = endknots.back().x;
    cut.start = startknots.front().x;
    cut.ending.cap = false;
    cut.starting.cap = false;
    auto const outwards { Band.facing / Band.into > 0.0 };
    ( outwards ? cut.ending.inner : cut.ending.extents ) = endknots;
    ( outwards ? cut.starting.extents : cut.starting.inner ) = startknots;
    return cut;
}

rail_cut seam( double const At, double const End, double const Start ) {

    rail_cut cut;
    cut.at = At;
    cut.end = End;
    cut.start = Start;
    cut.ending.cap = false;
    cut.starting.cap = false;
    return cut;
}

knots::line merged_blocks( knots::line Blocks, std::vector<rail_cut> const &Cuts ) {

    for( auto const &cut : Cuts ) {
        for( auto &block : Blocks ) {
            if( cut.at > block.x && cut.at < block.y ) {
                block = { std::min( block.x, cut.end ), std::max( block.y, cut.start ) };
            }
        }
    }
    knots::sort( Blocks );
    knots::line result;
    for( auto const &block : Blocks ) {
        if( false == result.empty() && block.x <= result.back().y ) {
            result.back().y = std::max( result.back().y, block.y );
        }
        else {
            result.push_back( block );
        }
    }
    return result;
}

rail_tread const *tread_at( std::vector<rail_tread> const &Treads, double const Middle ) {

    for( auto const &tread : Treads ) {
        if( Middle > tread.from && Middle < tread.to ) {
            return &tread;
        }
    }
    return nullptr;
}

material_handle tread_material( rail_tread const *Tread ) {

    return ( Tread != nullptr ? Tread->material : null_handle );
}

// a part of a guard rail between its ends or the cuts across it: where it starts and ends, the mitres there, its flared ends cut back across it
struct guard_part {
    double from { 0.0 };
    double to { 0.0 };
    glm::dvec2 frommiter { 0.0 };
    glm::dvec2 tomiter { 0.0 };
    bool bevelfrom { false };
    bool bevelto { false };
};

// a piece of a rail between two cuts, before it's split at the treads along it: the lines it's laid with, the reach of the wheels on it, its ends
struct cut_span {
    double from { 0.0 };
    double to { 0.0 };
    knots::line offsets;
    knots::line extents;
    knots::line inner;
    knots::line rolled;
    knots::line worn;
    material_handle wornby { null_handle };
    bool capfrom { false };
    bool capto { false };
    piece_ends ends;
};

// the points of the body of a grooved rail on the floor of the groove, and the texture of those of the profile there, summed up
struct groove_floor {
    std::vector<std::size_t> points;
    float texture { 0.f };
    int count { 0 };
};

// the polish of the wheels of a rail and of the rail beside it rolling on its head from the outside: how far each reaches, and where the two meet
struct shared_polish {
    knots::line reached;
    knots::line outside;
    knots::line meet;
};

// the treads along the piece of a rail from A to B: all, but those anchored at an end of the piece, which only where that end lies on them
std::vector<rail_tread> active_treads( std::vector<rail_tread> const &Treads, double const A, double const B ) {

    std::vector<rail_tread> active;
    for( auto const &tread : Treads ) {
        if( tread.anchor == 0
         || ( tread.anchor > 0 && B > tread.from + 0.01 && B < tread.to + 0.01 )
         || ( tread.anchor < 0 && A > tread.from - 0.01 && A < tread.to - 0.01 ) ) {
            active.push_back( tread );
        }
    }
    return active;
}

// where the piece of a rail from A to B is split: at its ends and at the edges of the treads within it, those closer together than 2 mm taken as one
std::vector<double> tread_splits( std::vector<rail_tread> const &Treads, double const A, double const B ) {

    std::vector<double> splits;
    for( auto const &tread : Treads ) {
        for( auto const edge : { tread.from, tread.to } ) {
            if( edge > A + 0.01 && edge < B - 0.01 ) {
                splits.push_back( edge );
            }
        }
    }
    std::ranges::sort( splits );
    splits.erase( std::ranges::unique( splits, []( double const Kept, double const Next ) { return Next - Kept < 0.002; } ).begin(), splits.end() );
    splits.insert( splits.begin(), A );
    splits.push_back( B );
    return splits;
}

// the cuts of a top of a grooved rail parted by the blocks along it: those within them go Inside, the rest Outside, each with seams along the blocks
void split_at_blocks( std::vector<rail_cut> const &Cuts, knots::line const &Blocks, std::vector<rail_cut> &Outside, std::vector<rail_cut> &Inside ) {

    for( auto const &cut : Cuts ) {
        auto const within { std::ranges::any_of( Blocks, [ &cut ]( glm::dvec2 const &Block ) { return cut.at > Block.x && cut.at < Block.y; } ) };
        ( within ? Inside : Outside ).push_back( cut );
    }
    Inside.push_back( seam( Blocks.front().x - 0.001, -1e9, Blocks.front().x ) );
    for( std::size_t m = 0; m < Blocks.size(); ++m ) {
        Outside.push_back( seam( 0.5 * ( Blocks[ m ].x + Blocks[ m ].y ), Blocks[ m ].x, Blocks[ m ].y ) );
        Inside.push_back( m + 1 < Blocks.size() ?
            seam( 0.5 * ( Blocks[ m ].y + Blocks[ m + 1 ].x ), Blocks[ m ].y, Blocks[ m + 1 ].x ) :
            seam( Blocks[ m ].y + 0.001, Blocks[ m ].y, 1e9 ) );
    }
}

// each point of a strip twice, as the points of a profile come in pairs
gfx::vertex_array doubled_strip( gfx::vertex_array const &Strip ) {

    gfx::vertex_array doubled { Strip.front(), Strip.front() };
    for( std::size_t k = 1; k + 1 < Strip.size(); ++k ) {
        doubled.insert( doubled.end(), 2, Strip[ k ] );
    }
    doubled.insert( doubled.end(), 2, Strip.back() );
    return doubled;
}

// the points of the body of a grooved rail on the floor of the groove facing up off it, in the mean of the texture there
void level_floor( gfx::vertex_array &Body, groove_floor const &Floor ) {

    if( Floor.points.size() <= 1 ) {
        return;
    }
    auto const across { Body[ Floor.points.back() ].position - Body[ Floor.points.front() ].position };
    auto up { glm::normalize( glm::vec3{ -across.y, across.x, 0.f } ) };
    if( up.y < 0.f ) {
        up = -up;
    }
    for( auto const index : Floor.points ) {
        Body[ index ].normal = up;
        if( Floor.count > 0 ) {
            Body[ index ].texture.x = Floor.texture / static_cast<float>( Floor.count );
        }
    }
}

// the profile of a grooved rail split by the floor of its groove: the body below it, the tops of the head and of the lip above it with
// their flats laid over blocks, by the outline of the rail on the side Side
class groove_splitter {

public:
// constructors
    groove_splitter( gfx::vertex_array const &Profile, track_rails::rail_size const &Size, float const Platetexture, int const Side ) :
        m_profile { Profile },
        m_size { Size },
        m_platetexture { Platetexture },
        m_side { Side }
    {}
// methods
    grooved_profile split_grooved( gfx::vertex_array const &Outline ) const;

private:
// methods
    void groove_ridge( gfx::vertex_array const &Outline, std::size_t const First, std::size_t const Last, grooved_profile &Result, std::vector<std::size_t> &Floors ) const;
    double groove_height( gfx::vertex_array const &Outline, std::size_t const Index ) const;
    bool above_groove( gfx::vertex_array const &Outline, std::size_t const Index ) const;
    gfx::basic_vertex floor_point( gfx::vertex_array const &Outline, std::size_t const A, std::size_t const B ) const;
    gfx::basic_vertex flattened( gfx::vertex_array const &Outline, std::size_t const Index ) const;
// members
    gfx::vertex_array const &m_profile;
    track_rails::rail_size const &m_size;
    float const m_platetexture;
    int const m_side;
};

grooved_profile groove_splitter::split_grooved( gfx::vertex_array const &Outline ) const {

    auto const count { m_profile.size() };
    grooved_profile result;
    groove_floor floors;
    std::size_t i { 0 };
    while( i < count ) {
        if( above_groove( Outline, i ) ) {
            auto const first { i };
            while( i + 1 < count && above_groove( Outline, i + 1 ) ) {
                ++i;
            }
            groove_ridge( Outline, first, i, result, floors.points );
        }
        else {
            if( groove_height( Outline, i ) > -1e-4 ) {
                floors.points.push_back( result.body.size() );
                floors.texture += m_profile[ i ].texture.x;
                ++floors.count;
            }
            result.body.push_back( m_profile[ i ] );
        }
        ++i;
    }
    level_floor( result.body, floors );
    result.wallu = ( floors.count > 0 ? floors.texture / static_cast<float>( floors.count ) : -1.f );
    return result;
}

// the points First to Last of the profile above the floor of the groove as a top of the head or the lip and its flat, ending on the floor where the profile goes below it
void groove_splitter::groove_ridge( gfx::vertex_array const &Outline, std::size_t const First, std::size_t const Last, grooved_profile &Result, std::vector<std::size_t> &Floors ) const {

    auto &body { Result.body };
    gfx::vertex_array top;
    gfx::vertex_array flat;
    if( First > 0 ) {
        if( groove_height( Outline, First - 1 ) > -1e-4 ) {
            top.push_back( m_profile[ First - 1 ] );
            flat.push_back( flattened( Outline, First - 1 ) );
        }
        else {
            top.push_back( floor_point( Outline, First - 1, First ) );
            flat.push_back( top.back() );
            body.push_back( top.back() );
            Floors.push_back( body.size() );
            body.push_back( top.back() );
        }
    }
    for( auto k { First }; k <= Last; ++k ) {
        top.push_back( m_profile[ k ] );
        flat.push_back( flattened( Outline, k ) );
    }
    if( Last + 1 < m_profile.size() ) {
        if( groove_height( Outline, Last + 1 ) > -1e-4 ) {
            top.push_back( m_profile[ Last + 1 ] );
            flat.push_back( flattened( Outline, Last + 1 ) );
        }
        else {
            top.push_back( floor_point( Outline, Last, Last + 1 ) );
            flat.push_back( top.back() );
            Floors.push_back( body.size() );
            body.push_back( top.back() );
            body.push_back( top.back() );
        }
    }
    Result.tops.push_back( doubled_strip( top ) );
    Result.flats.push_back( doubled_strip( flat ) );
}

// how far the point Index of the profile lies above the floor of the groove, by the outline of the rail
double groove_splitter::groove_height( gfx::vertex_array const &Outline, std::size_t const Index ) const {

    return static_cast<double>( Outline[ m_side > 0 ? Index : m_profile.size() - 1 - Index ].position.y ) - m_size.groove;
}

// the point Index of the profile lies above the floor of the groove
bool groove_splitter::above_groove( gfx::vertex_array const &Outline, std::size_t const Index ) const {

    return groove_height( Outline, Index ) > 1e-4;
}

// the point on the floor of the groove between the points A and B of the profile
gfx::basic_vertex groove_splitter::floor_point( gfx::vertex_array const &Outline, std::size_t const A, std::size_t const B ) const {

    auto const t { static_cast<float>( groove_height( Outline, A ) / ( groove_height( Outline, A ) - groove_height( Outline, B ) ) ) };
    auto point { m_profile[ above_groove( Outline, A ) ? A : B ] };
    point.position = glm::mix( m_profile[ A ].position, m_profile[ B ].position, t );
    point.texture = glm::mix( m_profile[ A ].texture, m_profile[ B ].texture, t );
    return point;
}

// the point Index of the profile as it's laid over a block: on top of the head or of the lip, facing up in the texture of the plates
gfx::basic_vertex groove_splitter::flattened( gfx::vertex_array const &Outline, std::size_t const Index ) const {

    auto point { m_profile[ Index ] };
    if( auto const &file { Outline[ m_side > 0 ? Index : m_profile.size() - 1 - Index ].position }; file.y > m_size.crest - 0.0005 || ( file.x < -0.02 && file.y > m_size.liptop - 0.0005 ) ) {
        point.normal = { 0.f, 1.f, 0.f };
        point.texture.x = m_platetexture;
    }
    return point;
}

// where the polish of the rail's own wheels, reaching to Rolled, meets that of the wheels beside, reaching to Worn: halfway, or past the overlap of the brighter
double polish_meeting( double const Rolled, double const Worn, bool const Same, bool const Theirsbrighter ) {

    if( Worn >= Rolled || Same ) {
        return 0.5 * ( Rolled + Worn );
    }
    return ( Theirsbrighter ? Worn : Rolled );
}

// the reaches of the polish of the rail's own wheels, Ownreach, and of the wheels beside, Theirreach, where they overlap: each fade cut off halfway between them
shared_polish shared_reaches( knots::line const &Ownreach, knots::line const &Theirreach, bool const Same, bool const Theirsbrighter ) {

    shared_polish result;
    auto const widest { knots::shifted( Ownreach, blend ) };
    auto all { knots::stations( Ownreach, Theirreach ) };
    for( auto const *edge : { &Ownreach, &widest } ) {
        auto const crossed { knots::meetings( Theirreach, *edge ) };
        all.insert( all.end(), crossed.begin(), crossed.end() );
    }
    std::ranges::sort( all );
    for( auto const position : all ) {
        if( false == result.outside.empty() && position - result.outside.back().x < 1e-9 ) {
            continue;
        }
        auto const rolled { knots::sample( Ownreach, position, 0.0 ) };
        auto const worn { knots::sample( Theirreach, position, 0.0 ) };
        auto const half { 0.5 * std::clamp( worn - rolled, 0.0, blend ) };
        result.reached.emplace_back( position, rolled - half + blend );
        result.outside.emplace_back( position, worn + half - blend );
        result.meet.emplace_back( position, polish_meeting( rolled, worn, Same, Theirsbrighter ) );
    }
    return result;
}

// the two triangles of the face of a loft from the pair of points at First to the next pair, with their userdata, but on a rolled face
void add_face( gfx::vertex_array const &Vertices, gfx::userdata_array const &Userdata, std::size_t const First, bool const Rolled, gfx::vertex_array &Triangles, gfx::userdata_array &Triangleuserdata ) {

    for( auto const index : { 0, 1, 2, 1, 3, 2 } ) {
        auto const vertex { First + index };
        Triangles.push_back( Vertices[ vertex ] );
        if( false == Rolled ) {
            Triangleuserdata.push_back( Userdata[ vertex ] );
        }
    }
}

} // namespace

class TTrack::rail_layer {

public:
// constructors
    rail_layer( TTrack &Track, rail_layout const &Layout, rail_drawing const &Drawing, gfx::vertex_array const &Profile, rail_course const &Course, geometryhandle_sequence &Geometry, geometryhandle_sequence &Cuts );
// methods
    bool lay();

private:
// methods
    template <typename Type_>
    Type_ const &ours( track_rails::path_sides<Type_> const &Sides ) const {
        return Sides[ m_path ][ m_side > 0 ]; }
    std::vector<rail_tread> const &treads() const {
        return ours( m_layout.treads ); }
    void lay_guards();
    void lay_guard( guard_rail const &Guard, double const Top, double const Back );
    guard_part guard_between( guard_rail const &Guard, std::vector<rail_cut> const &Cuts, std::size_t const I, double const Start, glm::dvec2 const &Startmiter, double const Top, double const Back ) const;
    void lay_guard_part( guard_rail const &Guard, knots::line const &Offsets, double const Top, guard_part const &Part );
    double mitered( glm::dvec2 const &Miter, double const Square, bool const Latest, double const Top ) const;
    void lay_checks();
    void lay_fills();
    void lay_treads( rail_piece const &Whole, std::vector<rail_tread> const &Treads, piece_ends const &Ends );
    void lay_cuts( gfx::vertex_array const &Shape, std::vector<rail_cut> Cuts, bool const Wheels, std::vector<rail_tread> const &Treads, float const Wallu = -1.f );
    cut_span between_cuts( double const Start, rail_cut_part const &From, rail_cut const &Next, bool const Own ) const;
    void lay_between( gfx::vertex_array const &Shape, cut_span const &Between, bool const Wheels, std::vector<rail_tread> const &Treads, float const Wallu );
    material_handle brighter( material_handle const A, material_handle const B ) const;
    void lay_grooved( gfx::vertex_array const &Outline );
    std::vector<rail_tread> block_treads( knots::line const &Blocks ) const;
    std::vector<rail_cut> top_cuts( gfx::vertex_array const &Top ) const;
    double top_edge( gfx::vertex_array const &Top, bool const Head ) const;
    void running( rail_piece const &Whole, knots::line const &Rolled, knots::line const &Worn, material_handle const Wornby, piece_ends const &Ends );
    void plain( rail_piece Whole, rail_tread const &Tread, piece_ends const &Ends );
    void treaded( rail_piece Whole, rail_tread const &Tread, piece_ends const &Ends );
    gfx::vertex_array lay_sides( rail_piece const &Whole, piece_ends const &Ends );
    template <typename Edges_>
    void lay_top( gfx::vertex_array const &Top, rail_piece const &Whole, glm::dvec2 const &Reach, std::vector<double> const &Splits, std::vector<top_band> const &Bands, Edges_ const &Edges, piece_ends const &Ends );
    // the faces between the points Leftfrom and Leftto of the shape are left out, laid apart
    void piece( gfx::vertex_array const &Shape, rail_piece const &Piece, piece_ends const &Ends = {}, std::size_t const Leftfrom = 0, std::size_t const Leftto = 0 );
    loft_span span( rail_piece const &Piece, piece_ends const &Ends ) const;
    double texture_length( material_handle const Material ) const;
    geometryhandle_sequence &sink( material_handle const Material, bool const Matte );
    material_handle paint( material_handle const Material, glm::vec3 const &Colour );
    glm::dvec2 reaches( gfx::vertex_array const &Shape ) const;
    gfx::vertex_array strip( std::size_t const First, std::size_t const Last ) const;
// members
    TTrack &m_track;
    rail_layout const &m_layout;
    rail_drawing const &m_drawing;
    int const m_path;
    gfx::vertex_array const &m_profile;
    int const m_side;
    float const m_texturelength;
    double const m_from;
    bool const m_openstart;
    bool const m_openend;
    geometryhandle_sequence &m_geometry;
    geometryhandle_sequence &m_cuts;
    TSegment const &m_segment;
    double const m_length;
    double const m_guardoffset;
    std::size_t m_topfirst;
    std::size_t m_toplast { 0 };
    std::vector<material_handle> m_tints;
};

TTrack::rail_layer::rail_layer( TTrack &Track, rail_layout const &Layout, rail_drawing const &Drawing, gfx::vertex_array const &Profile, rail_course const &Course, geometryhandle_sequence &Geometry, geometryhandle_sequence &Cuts ) :
    m_track { Track },
    m_layout { Layout },
    m_drawing { Drawing },
    m_path { Course.path },
    m_profile { Profile },
    m_side { Course.side },
    m_texturelength { Course.texturelength },
    m_from { Course.from },
    m_openstart { Course.openstart },
    m_openend { Course.openend },
    m_geometry { Geometry },
    m_cuts { Cuts },
    m_segment { track_rails::path_segment( Track, Course.path ) },
    m_length { m_segment.GetLength() },
    m_guardoffset { -( flangeway + Drawing.size.head ) },
    m_topfirst { Profile.size() } {
    // the running surface on top of the head is split along the edge of the wheel treads, the part of it out of their reach goes matte; the rest of the profile and the end faces are laid whole
    for( std::size_t j = 0; j < Profile.size(); ++j ) {
        if( Profile[ j ].position.y > static_cast<float>( -0.087 * Drawing.size.height ) ) {
            m_topfirst = std::min( m_topfirst, j );
            m_toplast = std::max( m_toplast, j );
        }
    }
}

bool TTrack::rail_layer::lay() {

    lay_guards();
    lay_checks();
    lay_fills();
    auto const &plan { ours( m_layout.plans ) };
    auto const &bands { ours( m_layout.bands ) };
    if( plan.cuts.empty() && bands.empty() && treads().empty() ) {
        return false;
    }
    auto const &outline { track_rail_profile( m_track.m_profile1.second ) };
    if( false == m_drawing.size.grooved || outline.size() != 2 * m_profile.size() ) {
        auto cuts { plan.cuts };
        auto const reach { reaches( m_profile ) };
        for( auto const &band : bands ) {
            cuts.push_back( band_cut( band, reach ) );
        }
        lay_cuts( m_profile, cuts, true, treads() );
        return true;
    }
    lay_grooved( outline );
    return true;
}

void TTrack::rail_layer::lay_guards() {

    auto const &size { m_drawing.size };
    auto const top { flangeway + size.head + size.low - 0.002 };
    // a flared end is cut slanted from the face of the guard rail toward the running rail: its back reaches out to the end, its face stops short of it
    auto const back { -( reaches( m_profile ).x + m_guardoffset ) };
    for( auto const &guard : ours( m_layout.guards ) ) {
        lay_guard( guard, top, back );
    }
}

// a guard rail in its parts between the cuts across it, its head reaching out to Top short of the running rail, its back Back behind its face
void TTrack::rail_layer::lay_guard( guard_rail const &Guard, double const Top, double const Back ) {

    knots::line const offsets { { Guard.from, m_guardoffset }, { Guard.to, m_guardoffset } };
    auto cuts { Guard.plan.cuts };
    sort_cuts( cuts );
    auto start { Guard.from };
    auto startmiter { Guard.frommiter };
    for( std::size_t i = 0; i <= cuts.size(); ++i ) {
        if( auto const part { guard_between( Guard, cuts, i, start, startmiter, Top, Back ) }; part.to - part.from > 0.01 ) {
            lay_guard_part( Guard, offsets, Top, part );
        }
        if( i < cuts.size() && cuts[ i ].start > start ) {
            start = cuts[ i ].start;
            startmiter = cuts[ i ].starting.miter;
        }
    }
}

// the part of a guard rail from Start, mitred by Startmiter, up to the cut I across it or past the last cut to its end, its ends mitred or cut back
guard_part TTrack::rail_layer::guard_between( guard_rail const &Guard, std::vector<rail_cut> const &Cuts, std::size_t const I, double const Start, glm::dvec2 const &Startmiter, double const Top, double const Back ) const {

    auto const end { I < Cuts.size() ? Cuts[ I ].end : Guard.to };
    auto const endmiter { I < Cuts.size() ? Cuts[ I ].ending.miter : Guard.tomiter };
    auto const bevelfrom { Start == Guard.from && Startmiter.y == 0.0 && Guard.flarefrom && Guard.from >= std::max( m_from, 0.0 ) };
    auto const bevelto { I == Cuts.size() && endmiter.y == 0.0 && Guard.flareto && Guard.to <= m_length };
    auto const a { bevelfrom ? Guard.from + guard_bevel * Back : std::max( { mitered( Startmiter, Start, false, Top ), m_from, 0.0 } ) };
    auto const b { bevelto ? Guard.to - guard_bevel * Back : std::min( { mitered( endmiter, end, true, Top ), m_length, mitered( Guard.tomiter, Guard.to, true, Top ) } ) };
    return { .from = a, .to = b, .frommiter = Startmiter, .tomiter = endmiter, .bevelfrom = bevelfrom, .bevelto = bevelto };
}

// a part of a guard rail, its face planed off toward its flared ends and its head cut along the mitres at its ends
void TTrack::rail_layer::lay_guard_part( guard_rail const &Guard, knots::line const &Offsets, double const Top, guard_part const &Part ) {

    knots::line uppers { { 0.0, Top } };
    knots::line lowers;
    planed( uppers, Guard.from, Guard.to, std::min( guard_flare, 0.25 * ( Guard.to - Guard.from ) ), m_drawing.size.head, Guard.flarefrom ? guard_planing : 0.0, Guard.flareto ? guard_planing : 0.0 );
    for( auto const &[ miter, ending ] : { std::pair{ Part.frommiter, false }, std::pair{ Part.tomiter, true } } ) {
        if( miter.y != 0.0 ) {
            ( ending == ( miter.y < 0.0 ) ? uppers : lowers ).emplace_back( 1.0 / miter.y, -miter.x / miter.y - m_guardoffset );
        }
    }
    lay_treads(
        { .from = Part.from, .to = Part.to, .offsets = Offsets, .extents = envelope( uppers, true, Part.from, Part.to ), .inner = envelope( lowers, false, Part.from, Part.to ), .capfrom = true, .capto = true, .rolled = false },
        Guard.treads,
        { Part.from, Part.to, { Part.bevelfrom ? guard_bevel : 0.0 }, { Part.bevelto ? -guard_bevel : 0.0 } } );
}

// where a part of a guard rail mitred against another rail ends: the first of the ends of its face and its back, or the Latest; Square where it's not mitred
double TTrack::rail_layer::mitered( glm::dvec2 const &Miter, double const Square, bool const Latest, double const Top ) const {

    if( Miter.y == 0.0 ) {
        return Square;
    }
    auto const inside { Miter.x + Miter.y * ( m_drawing.size.low + m_guardoffset ) };
    auto const outside { Miter.x + Miter.y * ( Top + m_guardoffset ) };
    return ( Latest ? std::max( inside, outside ) : std::min( inside, outside ) );
}

void TTrack::rail_layer::lay_checks() {

    auto const &shape { m_drawing.checkshapes[ m_side > 0 ? 0 : 1 ] };
    for( auto const &check : m_layout.checks[ m_path ][ m_side > 0 ] ) {
        auto const a { std::max( check.from, m_from ) };
        auto const b { std::min( check.to, m_length ) };
        if( replaced_by_wing( check, ours( m_layout.wingends ) ) || b - a <= 0.01 ) {
            continue;
        }
        // the flared ends are bent away from the running rail, the whole of the check rail
        auto const flare { check_flare( check ) };
        knots::line const offsets { { check.from, -check.flarefrom }, { check.from + flare, 0.0 }, { check.to - flare, 0.0 }, { check.to, -check.flareto } };
        piece(
            shape,
            { .from = a, .to = b, .offsets = offsets, .capfrom = a > 0.0 || check.from >= 0.0, .capto = check.to <= m_length, .rolled = false },
            { a, b, { a == check.from ? check.frombend : 0.0 }, { b == check.to ? check.tobend : 0.0 } } );
    }
}

void TTrack::rail_layer::lay_treads( rail_piece const &Whole, std::vector<rail_tread> const &Treads, piece_ends const &Ends ) {

    std::vector splits { Whole.from, Whole.to };
    for( auto const &tread : Treads ) {
        for( auto const edge : { tread.from, tread.to } ) {
            if( edge > Whole.from + 0.01 && edge < Whole.to - 0.01 ) {
                splits.push_back( edge );
            }
        }
    }
    std::ranges::sort( splits );
    auto const &ends { Ends };
    for( std::size_t k = 1; k < splits.size(); ++k ) {
        auto part { Whole };
        part.from = splits[ k - 1 ];
        part.to = splits[ k ];
        part.capfrom = ( k == 1 && Whole.capfrom );
        part.capto = ( k + 1 == splits.size() && Whole.capto );
        auto const *tread { tread_at( Treads, 0.5 * ( part.from + part.to ) ) };
        if( tread != nullptr && m_topfirst < m_toplast && tread->edge.empty() ) {
            plain( part, *tread, ends );
        }
        else if( tread != nullptr && m_topfirst < m_toplast ) {
            treaded( part, *tread, ends );
        }
        else {
            piece( m_profile, part, ends );
        }
    }
}

void TTrack::rail_layer::lay_fills() {

    for( auto const &fill : ours( m_layout.fills ) ) {
        auto const a { std::max( fill.from, m_from ) };
        auto const b { std::min( fill.to, m_length ) };
        if( b - a > 0.01 ) {
            auto const &shapes { fill.web ? m_drawing.fillers : m_drawing.plates };
            piece(
                shapes[ m_side > 0 ? 0 : 1 ],
                { .material = ( fill.web ? null_handle : tread_material( tread_at( treads(), 0.5 * ( a + b ) ) ) ), .from = a, .to = b, .extents = fill.outer, .inner = fill.inner, .capfrom = a == fill.from && fill.capfrom, .capto = b == fill.to && fill.capto, .walls = true, .lift = static_cast<float>( fill.lift ) } );
        }
    }
}

void TTrack::rail_layer::lay_cuts( gfx::vertex_array const &Shape, std::vector<rail_cut> Cuts, bool const Wheels, std::vector<rail_tread> const &Treads, float const Wallu ) {

    sort_cuts( Cuts );
    // what lies before the first cut and past the last one, as a cut far off: parts with no lines of their own
    rail_cut const beyond { .end = 1e9 };
    auto start { -1e9 };
    auto const *from { &beyond.starting };
    for( std::size_t i = 0; i <= Cuts.size(); ++i ) {
        // a piece carved between two cuts has the same lines at both
        auto const own { i < Cuts.size() && false == ( i > 0 && Cuts[ i - 1 ].carved && Cuts[ i ].carved ) };
        auto const &next { i < Cuts.size() ? Cuts[ i ] : beyond };
        if( auto const between { between_cuts( start, *from, next, own ) }; between.to - between.from > 0.01 ) {
            lay_between( Shape, between, Wheels, Treads, Wallu );
        }
        start = next.start;
        from = &next.starting;
    }
}

// the piece of the rail from Start, where the part From of the cut before starts, to the cut Next; Own: with the lines of the part of Next ending there
cut_span TTrack::rail_layer::between_cuts( double const Start, rail_cut_part const &From, rail_cut const &Next, bool const Own ) const {

    auto const end { Next.end };
    auto pieceoffsets { From.offsets };
    if( Own ) {
        pieceoffsets.insert( pieceoffsets.end(), Next.ending.offsets.begin(), Next.ending.offsets.end() );
    }
    knots::sort( pieceoffsets );
    // the limits of the head from both ends can overlap along a short piece, as the planed wing rail of one frog and the cut of the next: the tighter of them holds
    auto pieceextents { Own ? knots::bound( From.extents, Next.ending.extents, true ) : From.extents };
    auto pieceinner { Own ? knots::bound( From.inner, Next.ending.inner, false ) : From.inner };
    auto const a { std::max( Start, m_from ) };
    auto const b { std::min( end, m_length ) };
    auto rolled { Own ? knots::bound( From.rolled, Next.ending.rolled, true ) : From.rolled };
    auto worn { Own ? knots::bound( From.worn, Next.ending.worn, true ) : From.worn };
    auto const wornby { Own ? brighter( From.wornby, Next.ending.wornby ) : From.wornby };
    auto const capstart { Start > m_from ? From.cap : m_openstart };
    auto const capend { end < m_length ? Next.ending.cap : m_openend };
    piece_ends const ends {
        a, b,
        { Start > m_from ? From.bend : 0.0 },
        { end < m_length ? Next.ending.bend : 0.0 } };
    return {
        .from = a,
        .to = b,
        .offsets = std::move( pieceoffsets ),
        .extents = std::move( pieceextents ),
        .inner = std::move( pieceinner ),
        .rolled = std::move( rolled ),
        .worn = std::move( worn ),
        .wornby = wornby,
        .capfrom = capstart,
        .capto = capend,
        .ends = ends };
}

// the piece of a rail between two cuts, split at the edges of the treads along it
void TTrack::rail_layer::lay_between( gfx::vertex_array const &Shape, cut_span const &Between, bool const Wheels, std::vector<rail_tread> const &Treads, float const Wallu ) {

    auto const active { active_treads( Treads, Between.from, Between.to ) };
    auto const splits { tread_splits( active, Between.from, Between.to ) };
    for( std::size_t k = 1; k < splits.size(); ++k ) {
        auto const *tread { tread_at( active, 0.5 * ( splits[ k - 1 ] + splits[ k ] ) ) };
        rail_piece const whole {
            .material = ( tread != nullptr && treads_marked() ? paint( tread->material, plain_mark ) : tread_material( tread ) ),
            .from = splits[ k - 1 ],
            .to = splits[ k ],
            .offsets = Between.offsets,
            .extents = Between.extents,
            .inner = Between.inner,
            .capfrom = k == 1 && Between.capfrom,
            .capto = k + 1 == splits.size() && Between.capto,
            .walls = false == Wheels,
            .wallu = Wallu };
        if( Wheels && tread != nullptr && false == tread->edge.empty() && m_topfirst < m_toplast ) {
            treaded( whole, *tread, Between.ends );
        }
        else if( false == Wheels || Between.rolled.empty() || m_topfirst >= m_toplast ) {
            piece( Shape, whole, Between.ends );
        }
        else {
            running( whole, Between.rolled, Between.worn, Between.wornby, Between.ends );
        }
    }
}

// the brighter polish of the two, the one there is where there's only one
material_handle TTrack::rail_layer::brighter( material_handle const A, material_handle const B ) const {

    if( A == null_handle || B == null_handle ) {
        return ( A == null_handle ? B : A );
    }
    return ( tread_brightness( B, m_track.tread_band() ) > tread_brightness( A, m_track.tread_band() ) ? B : A );
}

void TTrack::rail_layer::lay_grooved( gfx::vertex_array const &Outline ) {

    auto const grooved { groove_splitter { m_profile, m_drawing.size, m_drawing.platetexture, m_side }.split_grooved( Outline ) };
    auto const &bands { ours( m_layout.bands ) };
    auto wholecuts { ours( m_layout.plans ).cuts };
    auto const bodyreach { reaches( grooved.body ) };
    for( auto const &band : bands ) {
        if( band.whole ) {
            wholecuts.push_back( band_cut( band, bodyreach ) );
        }
    }
    lay_cuts( grooved.body, wholecuts, false, {} );
    for( std::size_t k = 0; k < grooved.tops.size(); ++k ) {
        auto const cuts { top_cuts( grooved.tops[ k ] ) };
        auto const merged { merged_blocks( ours( m_layout.blocks ), cuts ) };
        if( merged.empty() ) {
            lay_cuts( grooved.tops[ k ], cuts, false, treads(), grooved.wallu );
            continue;
        }
        auto const blocktreads { block_treads( merged ) };
        std::vector<rail_cut> outside;
        std::vector<rail_cut> inside;
        split_at_blocks( cuts, merged, outside, inside );
        lay_cuts( grooved.tops[ k ], outside, false, treads(), grooved.wallu );
        lay_cuts( grooved.flats[ k ], inside, false, blocktreads, grooved.wallu );
    }
}

// the treads over the blocks of the head: over each block, the first tread reaching over it, along the whole block
std::vector<rail_tread> TTrack::rail_layer::block_treads( knots::line const &Blocks ) const {

    std::vector<rail_tread> blocktreads;
    for( auto const &block : Blocks ) {
        for( auto const &tread : treads() ) {
            if( tread.from < block.y && tread.to > block.x ) {
                blocktreads.emplace_back( block.x, block.y, tread.material );
                break;
            }
        }
    }
    return blocktreads;
}

std::vector<rail_cut> TTrack::rail_layer::top_cuts( gfx::vertex_array const &Top ) const {

    auto const reach { reaches( Top ) };
    auto const head { reach.x + reach.y > 0.0 };
    auto const edge { top_edge( Top, head ) };
    auto cuts { ours( m_layout.plans ).cuts };
    for( auto const &band : ours( m_layout.bands ) ) {
        auto cut { band_cut( band, reach ) };
        if( false == band.whole && std::abs( band.facing ) >= 1e-3 && std::abs( edge ) < 1.0 ) {
            auto const blunt { edge + ( head ? nose_tip : -nose_tip ) };
            if( head == ( band.facing / band.into > 0.0 ) ) {
                cut.start = std::max( cut.start, band.at + ( ( band.into > 0.0 ? band.to : band.from ) + band.facing * blunt ) / band.into );
                cut.starting.cap = true;
            }
            else {
                cut.end = std::min( cut.end, band.at + ( ( band.into > 0.0 ? band.from : band.to ) + band.facing * blunt ) / band.into );
                cut.ending.cap = true;
            }
        }
        cuts.push_back( cut );
    }
    return cuts;
}

// how far out from the gauge line the flat on top of the head of the strip reaches in, or the one on top of the lip reaches out; far off: none
double TTrack::rail_layer::top_edge( gfx::vertex_array const &Top, bool const Head ) const {

    auto const &size { m_drawing.size };
    auto edge { Head ? 1e9 : -1e9 };
    for( auto const &point : Top ) {
        if( point.position.y > ( Head ? size.crest : size.liptop ) - 0.0005 ) {
            auto const across { static_cast<double>( static_cast<float>( m_side ) * point.position.x - m_drawing.halfwidth ) };
            edge = ( Head ? std::min( edge, across ) : std::max( edge, across ) );
        }
    }
    return edge;
}

template <typename Edges_>
void TTrack::rail_layer::lay_top( gfx::vertex_array const &Top, rail_piece const &Whole, glm::dvec2 const &Reach, std::vector<double> const &Splits, std::vector<top_band> const &Bands, Edges_ const &Edges, piece_ends const &Ends ) {

    for( std::size_t i = 1; i < Splits.size(); ++i ) {
        auto const middle { 0.5 * ( Splits[ i - 1 ] + Splits[ i ] ) };
        auto const least { std::max( Reach.x, knots::sample( Whole.inner, middle, Reach.x ) ) };
        auto const most { std::min( Reach.y, knots::sample( Whole.extents, middle, Reach.y ) ) };
        auto edges { Edges( middle ) };
        edges.insert( edges.begin(), -1e9 );
        edges.push_back( 1e9 );
        for( std::size_t k = 0; k < Bands.size(); ++k ) {
            auto const &band { Bands[ k ] };
            if( edges[ k + 1 ] > edges[ k ] + 1e-6 && edges[ k + 1 ] > least && edges[ k ] < most ) {
                piece(
                    Top,
                    { .material = band.material, .from = Splits[ i - 1 ], .to = Splits[ i ], .offsets = Whole.offsets, .extents = band.extents, .inner = band.inner, .rolled = band.rolled, .ramp = band.ramp, .fade = band.fade },
                    Ends );
            }
        }
    }
}

void TTrack::rail_layer::running( rail_piece const &Whole, knots::line const &Rolled, knots::line const &Worn, material_handle const Wornby, piece_ends const &Ends ) {

    auto const top { lay_sides( Whole, Ends ) };
    auto const reach { reaches( top ) };
    auto const own { Whole.material == null_handle ? m_track.rail_material( m_path ) : Whole.material };
    // the wheels of the rail beside polish the head as they do their own; where the wheels of both roll on it, the brighter polish shows
    auto const theirs { ( Wornby == null_handle || Worn.empty() ) ? own : Wornby };
    auto const theirsbrighter { theirs != own && tread_brightness( theirs, m_track.tread_band() ) > tread_brightness( own, m_track.tread_band() ) };
    auto const theirmaterial { theirs == own ? Whole.material : theirs };
    // the wheels wander across, rolling a little further than their reach in the middle of the gauge
    auto const ownreach { knots::shifted( Rolled, wheel_play ) };
    auto const theirreach { Worn.empty() ? knots::line {} : knots::shifted( Worn, -wheel_play ) };
    // the polish fades across the edge of the reach of the wheels, as it does at the treads
    auto reached { knots::shifted( ownreach, 0.5 * blend ) };
    // where the head is planed flush against a rail beside it, the wheels of that rail roll on it again from the outside; where the reach of both
    // overlaps, the head is rolled across, each fade cut off halfway between them, or the duller polish where the brighter one reaches. the wheels
    // wander across the gap between the reaches narrower than a fade, so both fades close in on it, down to none where the reaches touch
    knots::line outside;
    knots::line meet;
    if( false == theirreach.empty() ) {
        auto shared { shared_reaches( ownreach, theirreach, theirs == own, theirsbrighter ) };
        reached = std::move( shared.reached );
        outside = std::move( shared.outside );
        meet = std::move( shared.meet );
    }
    auto const inner { outside.empty() ? knots::shifted( reached, -blend ) : knots::bound( knots::shifted( reached, -blend ), meet, true ) };
    auto const rolled { outside.empty() ? reached : knots::bound( reached, meet, true ) };
    auto const worn { outside.empty() ? knots::line {} : knots::bound( outside, meet, false ) };
    auto const outer { outside.empty() ? knots::line {} : knots::bound( knots::shifted( outside, blend ), meet, false ) };
    auto const splits { cross_splits( Whole, reach, { &rolled, &inner, &worn, &outer }, 0.0 ) };
    auto const marked { treads_marked() };
    std::vector<top_band> bands {
        { Whole.material, knots::bound( Whole.extents, inner, true ), Whole.inner },
        { marked ? paint( own, { 1.f, 1.f, 0.f } ) : Whole.material, knots::bound( Whole.extents, rolled, true ), knots::bound( inner, Whole.inner, false ), false, marked ? knots::line {} : reached },
        { marked ? paint( own, { 1.f, 0.f, 1.f } ) : Whole.material, worn.empty() ? Whole.extents : knots::bound( Whole.extents, worn, true ), knots::bound( rolled, Whole.inner, false ), false } };
    if( false == worn.empty() ) {
        bands.emplace_back( marked ? paint( own, { 0.f, 1.f, 1.f } ) : theirmaterial, knots::bound( Whole.extents, outer, true ), knots::bound( worn, Whole.inner, false ), false, marked ? knots::line {} : outside, -blend );
        bands.emplace_back( theirmaterial, Whole.extents, knots::bound( outer, Whole.inner, false ) );
    }
    lay_top(
        top, Whole, reach, splits, bands,
        [ &worn, &reached, &inner, &rolled, &outer ]( double const Middle ) {
            if( worn.empty() ) {
                auto const edge { knots::sample( reached, Middle, 0.0 ) };
                return std::vector<double> { edge - blend, edge };
            }
            return std::vector<double> { knots::sample( inner, Middle, 0.0 ), knots::sample( rolled, Middle, 0.0 ), knots::sample( worn, Middle, 0.0 ), knots::sample( outer, Middle, 0.0 ) }; },
        Ends );
}

void TTrack::rail_layer::plain( rail_piece Whole, rail_tread const &Tread, piece_ends const &Ends ) {

    Whole.material = null_handle;
    auto const top { lay_sides( Whole, Ends ) };
    Whole.material = ( treads_marked() ? paint( Tread.material, plain_mark ) : Tread.material );
    Whole.capfrom = false;
    Whole.capto = false;
    Whole.rolled = true;
    piece( top, Whole, Ends );
}

void TTrack::rail_layer::treaded( rail_piece Whole, rail_tread const &Tread, piece_ends const &Ends ) {

    auto const marked { treads_marked() };
    auto const own { m_track.rail_material( m_path ) };
    // where the wheels from beside polish the head as its own do, there's no tint between two polishes, only the polish of the rail itself
    auto const same { Tread.material == own && false == marked };
    if( same && Whole.rolled ) {
        plain( Whole, Tread, Ends );
        return;
    }
    Whole.material = null_handle;
    auto const top { lay_sides( Whole, Ends ) };
    auto const reach { reaches( top ) };
    // the lines of the tread run out from the gauge line of the rail, the piece is laid shifted off it
    knots::line back;
    for( auto const &knot : Whole.offsets ) {
        back.emplace_back( knot.x, -knot.y );
    }
    auto const edge { knots::sum( Tread.edge, back ) };
    auto const gauge { knots::sum( Tread.gauge, back ) };
    auto const &lower { Tread.below ? gauge : edge };
    auto const &upper { Tread.below ? edge : gauge };
    // the brighter polish of the tread fades out towards the edge of the reach of the wheels into the colour of the polish of the rail itself,
    // keeping the gloss of its steel
    auto const colour { glm::clamp( tread_colour( own, m_track.tread_band() ) / glm::max( tread_colour( Tread.material, m_track.tread_band() ), glm::vec3 { 1e-3f } ), 0.f, 4.f ) };
    auto const tint = [ this, marked, &colour ]( material_handle const Material, glm::vec3 const &Mark ) {
        return paint( Material, marked ? Mark : colour ); };
    auto const sharedtint { tint( Tread.material, { 0.f, 1.f, 0.f } ) };
    auto const shared { marked ? tint( Tread.material, { 0.f, 0.f, 1.f } ) : Tread.material };
    auto const ramp = [ marked ]( knots::line const &Knots ) {
        return marked ? knots::line {} : Knots; };
    auto const owned = [ marked, own, &Whole, &tint ]( knots::line const &Extents, knots::line const &Inner, double const Fade ) {
        if( false == marked ) {
            return top_band { null_handle, Extents, Inner, Whole.rolled };
        }
        auto const material { Whole.rolled ? tint( own, { 1.f, 0.f, 0.f } ) : null_handle };
        return top_band { material, Extents, Inner, false, {}, Fade }; };
    auto const fade { Tread.fade };
    auto const centre { knots::middle( lower, upper ) };
    auto const beneath { knots::shifted( lower, -fade ) };
    auto const beyond { knots::shifted( upper, fade ) };
    auto const lowfade { knots::bound( knots::shifted( lower, fade ), centre, true ) };
    auto const highfade { knots::bound( knots::shifted( upper, -fade ), centre, false ) };
    auto splits { cross_splits( Whole, reach, { &lower, &upper, &lowfade, &highfade, &beneath, &beyond }, 0.001 ) };
    add_meetings( knots::shifted( lower, 2.0 * fade ), upper, Whole.from, Whole.to, 0.001, splits );
    std::ranges::sort( splits );
    if( same ) {
        // the head no wheel of its own rolls on is polished across the tread alone, fading into the matte outside it
        lay_top(
            top, Whole, reach, splits,
            {
                { null_handle, knots::bound( Whole.extents, beneath, true ), Whole.inner, false },
                { null_handle, knots::bound( Whole.extents, lower, true ), knots::bound( Whole.inner, beneath, false ), false, beneath, -fade },
                { shared, knots::bound( Whole.extents, upper, true ), knots::bound( Whole.inner, lower, false ) },
                { null_handle, knots::bound( Whole.extents, beyond, true ), knots::bound( Whole.inner, upper, false ), false, beyond, fade },
                { null_handle, Whole.extents, knots::bound( Whole.inner, beyond, false ), false } },
            [ &lower, &upper, fade ]( double const Middle ) {
                auto const from { knots::sample( lower, Middle, 0.0 ) };
                auto const to { knots::sample( upper, Middle, 0.0 ) };
                return std::vector<double> { from - fade, from, to, to + fade }; },
            Ends );
        return;
    }
    lay_top(
        top, Whole, reach, splits,
        {
            { null_handle, knots::bound( Whole.extents, beneath, true ), Whole.inner, Whole.rolled },
            owned( knots::bound( Whole.extents, lower, true ), knots::bound( Whole.inner, beneath, false ), fade ),
            { sharedtint, knots::bound( Whole.extents, lowfade, true ), knots::bound( Whole.inner, lower, false ), false, ramp( lower ), -fade },
            { shared, knots::bound( Whole.extents, highfade, true ), knots::bound( Whole.inner, lowfade, false ), false == marked },
            { sharedtint, knots::bound( Whole.extents, upper, true ), knots::bound( Whole.inner, highfade, false ), false, ramp( upper ), fade },
            owned( knots::bound( Whole.extents, beyond, true ), knots::bound( Whole.inner, upper, false ), -fade ),
            { null_handle, Whole.extents, knots::bound( Whole.inner, beyond, false ), Whole.rolled } },
        [ &lower, &upper, &lowfade, &highfade, fade ]( double const Middle ) {
            auto const from { knots::sample( lower, Middle, 0.0 ) };
            auto const to { knots::sample( upper, Middle, 0.0 ) };
            return std::vector<double> { from - fade, from, knots::sample( lowfade, Middle, 0.0 ), knots::sample( highfade, Middle, 0.0 ), to, to + fade }; },
        Ends );
}

gfx::vertex_array TTrack::rail_layer::lay_sides( rail_piece const &Whole, piece_ends const &Ends ) {

    // the sides are lofted in the whole profile, the top left out: lofted apart, the points of a side of the head planed along the rail had none
    // of the top beside them to stop at, and went down past the wall of the cut, which went missing between the top and the underside of the head
    piece( m_profile, Whole, Ends, m_topfirst, m_toplast );
    return strip( m_topfirst, m_toplast );
}

void TTrack::rail_layer::piece( gfx::vertex_array const &Shape, rail_piece const &Piece, piece_ends const &Ends, std::size_t const Leftfrom, std::size_t const Leftto ) {

    gfx::vertex_array vertices;
    gfx::userdata_array userdata;
    auto loft { span( Piece, Ends ) };
    loft.userdata = ( Piece.rolled ? nullptr : &userdata );
    if( Shape.size() < 2 || false == m_segment.RenderLoftSpan( vertices, m_track.m_origin, Shape, loft ) ) {
        return;
    }
    // the cuts of the frogs worn, the ends of the track left open not: no wheel runs over them
    auto const openfrom { Piece.capfrom && m_openstart && Piece.from <= m_from + 1e-6 };
    auto const opento { Piece.capto && m_openend && Piece.to >= m_length - 1e-6 };
    cut_faces( vertices, Shape.size(), Piece.capfrom && false == openfrom, Piece.capto && false == opento, m_drawing.bank, sink( Piece.material, true ) );
    cut_faces( vertices, Shape.size(), openfrom, opento, m_drawing.bank, sink( Piece.material, true ), false );
    gfx::vertex_array triangles;
    gfx::userdata_array triangleuserdata;
    auto const ring { 2 * Shape.size() };
    for( std::size_t block = 0; block + ring <= vertices.size(); block += ring ) {
        for( std::size_t j = 0; j + 1 < Shape.size(); ++j ) {
            if( j >= Leftfrom && j < Leftto ) {
                continue;
            }
            add_face( vertices, userdata, block + 2 * j, Piece.rolled, triangles, triangleuserdata );
        }
    }
    if( triangles.empty() ) {
        return;
    }
    sink( Piece.material, false == Piece.rolled ).emplace_back( insert_chunk( triangles, triangleuserdata, m_drawing.bank, GL_TRIANGLES ) );
}

loft_span TTrack::rail_layer::span( rail_piece const &Piece, piece_ends const &Ends ) const {

    loft_span result;
    result.side = m_side;
    result.halfwidth = m_drawing.halfwidth;
    result.texturelength = texture_length( Piece.material );
    result.from = Piece.from;
    result.to = Piece.to;
    result.offsets = Piece.offsets;
    result.extents = Piece.extents;
    result.inner = Piece.inner;
    result.lift = Piece.lift;
    result.ramp = Piece.ramp;
    result.rampwidth = ( Piece.ramp.empty() ? 0.0 : Piece.fade );
    result.walls = Piece.walls;
    result.wallu = Piece.wallu;
    if( std::abs( Piece.from - Ends.from ) < 1e-6 ) {
        result.fromend = Ends.fromend;
    }
    if( std::abs( Piece.to - Ends.to ) < 1e-6 ) {
        result.toend = Ends.toend;
    }
    return result;
}

double TTrack::rail_layer::texture_length( material_handle const Material ) const {

    return ( Material == null_handle ? m_texturelength : m_track.texture_length( Material ) );
}

// parts no wheel rolls on go in the matte material of the cut faces
TTrack::geometryhandle_sequence &TTrack::rail_layer::sink( material_handle const Material, bool const Matte ) {

    if( Material == null_handle ) {
        return Matte ? m_cuts : m_geometry;
    }
    if( Material == m_track.m_material1 ) {
        return Matte ? m_track.GeometryCut1 : m_track.Geometry1;
    }
    if( m_track.eType == tt_Switch && Material == m_track.m_material2 ) {
        return Matte ? m_track.GeometryCut2 : m_track.Geometry2;
    }
    auto const tinted { std::ranges::find( m_tints, Material ) != m_tints.end() };
    auto &treads { m_track.GeometryTreads };
    auto found { std::ranges::find_if( treads, [ Material ]( tread_geometry const &Tread ) { return Tread.material == Material; } ) };
    if( found == treads.end() ) {
        found = treads.insert( treads.end(), { Material, tinted ? Material : cut_material( Material ), {}, {} } );
    }
    return Matte && false == tinted ? found->cuts : found->geometry;
}

material_handle TTrack::rail_layer::paint( material_handle const Material, glm::vec3 const &Colour ) {

    auto const tinted { tint_material( Material, Colour ) };
    if( tinted != Material ) {
        m_tints.push_back( tinted );
    }
    return tinted;
}

glm::dvec2 TTrack::rail_layer::reaches( gfx::vertex_array const &Shape ) const {

    glm::dvec2 result { 1e9, -1e9 };
    for( auto const &point : Shape ) {
        auto const reach { static_cast<double>( m_side * point.position.x - m_drawing.halfwidth ) };
        result = { std::min( result.x, reach ), std::max( result.y, reach ) };
    }
    return result;
}

gfx::vertex_array TTrack::rail_layer::strip( std::size_t const First, std::size_t const Last ) const {

    return gfx::vertex_array( m_profile.begin() + First, m_profile.begin() + Last + 1 );
}

bool TTrack::lay_rail( rail_layout const &Layout, rail_drawing const &Drawing, gfx::vertex_array const &Profile, rail_course const &Course, geometryhandle_sequence &Geometry, geometryhandle_sequence &Cuts ) {

    return rail_layer( *this, Layout, Drawing, Profile, Course, Geometry, Cuts ).lay();
}

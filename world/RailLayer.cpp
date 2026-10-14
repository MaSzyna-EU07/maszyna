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

// in the colour of the top of the rail foot
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
    for( auto const &cap : caps ) {
        auto const &ring { cap.first };
        if( glm::length2( cap.second ) == 0.f ) {
            continue;
        }
        auto const outward { glm::normalize( cap.second ) };
        auto across { glm::vec3{ -outward.z, 0.f, outward.x } };
        if( glm::length2( across ) == 0.f ) {
            continue;
        }
        across = glm::normalize( across );
        std::vector<glm::dvec2> plan;
        auto bottom { 0.0 }, top { 0.0 };
        for( auto const &vertex : ring ) {
            auto const offset { vertex.position - ring[ 0 ].position };
            plan.emplace_back( glm::dot( offset, across ), offset.y );
            bottom = std::min<double>( bottom, offset.y );
            top = std::max<double>( top, offset.y );
        }
        auto web { ring[ 0 ].texture };
        auto nearest { std::numeric_limits<double>::max() };
        for( std::size_t j = 0; j < Ring / 2; ++j ) {
            auto const distance { std::abs( plan[ j ].y - 0.5 * ( bottom + top ) ) };
            if( distance < nearest ) {
                nearest = distance;
                web = ring[ j ].texture;
            }
        }
        auto lowest { std::numeric_limits<double>::max() };
        for( std::size_t j = 0; j < Ring / 2; ++j ) {
            if( ring[ j ].normal.y > 0.5f && plan[ j ].y < lowest ) {
                lowest = plan[ j ].y;
                web = ring[ j ].texture;
            }
        }
        auto const cross = []( glm::dvec2 const &A, glm::dvec2 const &B, glm::dvec2 const &C ) {
            return ( B.x - A.x ) * ( C.y - A.y ) - ( B.y - A.y ) * ( C.x - A.x ); };
        double area { 0.0 };
        for( std::size_t j = 0; j < plan.size(); ++j ) {
            auto const &a { plan[ j ] };
            auto const &b { plan[ ( j + 1 ) % plan.size() ] };
            area += a.x * b.y - b.x * a.y;
        }
        auto const orientation { area < 0.0 ? -1.0 : 1.0 };
        std::vector<std::size_t> left( Ring );
        for( std::size_t j = 0; j < Ring; ++j ) {
            left[ j ] = j;
        }
        std::vector<std::array<std::size_t, 3>> triangles;
        while( left.size() > 3 ) {
            auto ear { left.size() };
            for( std::size_t j = 0; j < left.size() && ear == left.size(); ++j ) {
                auto const &a { plan[ left[ ( j + left.size() - 1 ) % left.size() ] ] };
                auto const &b { plan[ left[ j ] ] };
                auto const &c { plan[ left[ ( j + 1 ) % left.size() ] ] };
                auto const turn { cross( a, b, c ) * orientation };
                if( turn < -1e-12 ) {
                    continue;
                }
                auto inside { false };
                if( turn > 1e-12 ) {
                    for( auto const k : left ) {
                        auto const &p { plan[ k ] };
                        if( p == a || p == b || p == c ) {
                            continue;
                        }
                        if( cross( a, b, p ) * orientation > 1e-12 && cross( b, c, p ) * orientation > 1e-12 && cross( c, a, p ) * orientation > 1e-12 ) {
                            inside = true;
                            break;
                        }
                    }
                }
                if( false == inside ) {
                    ear = j;
                }
            }
            if( ear == left.size() ) {
                ear = 0;
            }
            triangles.push_back( { left[ ( ear + left.size() - 1 ) % left.size() ], left[ ear ], left[ ( ear + 1 ) % left.size() ] } );
            left.erase( left.begin() + ear );
        }
        triangles.push_back( { left[ 0 ], left[ 1 ], left[ 2 ] } );
        for( auto const &triangle : triangles ) {
            for( auto const &order : { std::array<int, 3>{ 0, 1, 2 }, std::array<int, 3>{ 0, 2, 1 } } ) {
                for( auto const k : order ) {
                    auto const index { triangle[ k ] };
                    Triangles.emplace_back( ring[ index ].position, outward, web );
                }
            }
        }
    }
}

gfx::userdata_array worn_userdata( std::size_t const Count ) {

    return gfx::userdata_array( Count, gfx::vertex_userdata{ glm::vec4{ 1.f, 0.f, 0.f, 0.f } } );
}

void cut_faces( gfx::vertex_array const &Strip, std::size_t const Ring, bool const Start, bool const End, gfx::geometrybank_handle const &Bank, TTrack::geometryhandle_sequence &Geometry ) {

    gfx::vertex_array triangles;
    cap_loft( Strip, Ring, Start, End, triangles );
    if( false == triangles.empty() ) {
        auto userdata { worn_userdata( triangles.size() ) };
        Geometry.emplace_back( GfxRenderer->Insert( triangles, userdata, Bank, GL_TRIANGLES ) );
    }
}

} // namespace track_rails

using namespace track_rails;

namespace {

gfx::vertex_array box_profile( float const Bottom, float const Top, float const Texture ) {

    return {
        {{  1.f, Bottom, 0.f }, {  1.f, 0.f, 0.f }, { Texture, 0.f }},
        {{  1.f, Top, 0.f }, {  1.f, 0.f, 0.f }, { Texture, 0.f }},
        {{  1.f, Top, 0.f }, {  0.f, 1.f, 0.f }, { Texture, 0.f }},
        {{ -1.f, Top, 0.f }, {  0.f, 1.f, 0.f }, { Texture, 0.f }},
        {{ -1.f, Top, 0.f }, { -1.f, 0.f, 0.f }, { Texture, 0.f }},
        {{ -1.f, Bottom, 0.f }, { -1.f, 0.f, 0.f }, { Texture, 0.f }} };
}

} // namespace

TTrack::rail_drawing TTrack::make_rail_drawing( gfx::geometrybank_handle const &Bank ) {

    auto const sides = [&]( gfx::vertex_array const &Shape, gfx::vertex_array ( &Sides )[ 2 ] ) {
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
        auto texture { 0.f };
        auto highest { -1e9f };
        for( std::size_t i = 0; i < outline.size() / 2; ++i ) {
            if( outline[ i ].position.y > highest ) {
                highest = outline[ i ].position.y;
                texture = outline[ i ].texture.s;
            }
        }
        if( drawing.size.grooved ) {
            auto sum { 0.f };
            auto points { 0 };
            for( std::size_t i = 0; i < outline.size() / 2; ++i ) {
                if( outline[ i ].position.y > highest - 0.0005f && outline[ i ].position.x > -0.02f ) {
                    sum += outline[ i ].texture.s;
                    ++points;
                }
            }
            texture = sum / std::max( points, 1 );
        }
        auto const top { static_cast<float>( ( drawing.size.grooved ? drawing.size.liptop : highest ) - 0.0005 ) };
        auto const bottom { static_cast<float>( -drawing.size.height ) };
        drawing.platetexture = texture;
        sides( box_profile( bottom, top, texture ), drawing.plates );
        auto const neck { static_cast<float>( drawing.size.neck ) };
        auto webtexture { texture };
        for( std::size_t i = 0; i < outline.size() / 2; ++i ) {
            auto const &point { outline[ i ] };
            if( point.position.y < neck && point.normal.y > 0.9f ) {
                webtexture = point.texture.s;
                break;
            }
        }
        sides( box_profile( bottom, neck, webtexture ), drawing.fillers );
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

bool treads_marked() {

    static auto const marked { std::getenv( "EU07_EDITOR_TREADS" ) != nullptr };
    return marked;
}

void sort_cuts( std::vector<rail_cut> &Cuts ) {

    std::sort( Cuts.begin(), Cuts.end(), []( rail_cut const &A, rail_cut const &B ) { return A.at < B.at; } );
}

void add_meetings( knots::line const &A, knots::line const &B, double const Start, double const End, double const Margin, std::vector<double> &Splits ) {

    for( auto const distance : knots::meetings( A, B ) ) {
        if( distance > Start + Margin && distance < End - Margin ) {
            Splits.push_back( distance );
        }
    }
}

std::vector<double> cross_splits( rail_piece const &Whole, glm::dvec2 const &Reach, std::initializer_list<knots::line const *> const Edges, double const Margin ) {

    std::vector<double> result { Whole.from, Whole.to };
    knots::line const lowline { { Whole.from, Reach.x }, { Whole.to, Reach.x } };
    knots::line const highline { { Whole.from, Reach.y }, { Whole.to, Reach.y } };
    for( auto const *edge : Edges ) {
        for( auto const *limit : { &lowline, &highline, &Whole.extents, &Whole.inner } ) {
            add_meetings( *edge, *limit, Whole.from, Whole.to, Margin, result );
        }
    }
    std::sort( result.begin(), result.end() );
    return result;
}

knots::line flared( double const From, double const To, double const Flare, double const Base, double const Fromdepth, double const Todepth ) {

    knots::line result;
    if( Fromdepth > 0.0 ) {
        result.insert( result.end(), { { From, Base - Fromdepth }, { From + Flare, Base } } );
    }
    else {
        result.push_back( { From, Base } );
    }
    if( Todepth > 0.0 ) {
        result.insert( result.end(), { { To - Flare, Base }, { To, Base - Todepth } } );
    }
    else {
        result.push_back( { To, Base } );
    }
    return result;
}

knots::line envelope( knots::line const &Lines, bool const Upper, double const A, double const B ) {

    knots::line result;
    if( Lines.empty() ) {
        return result;
    }
    std::vector<double> stations { A, B };
    for( std::size_t i = 0; i < Lines.size(); ++i ) {
        for( std::size_t j = i + 1; j < Lines.size(); ++j ) {
            if( std::abs( Lines[ i ].x - Lines[ j ].x ) > 1e-12 ) {
                auto const d { ( Lines[ j ].y - Lines[ i ].y ) / ( Lines[ i ].x - Lines[ j ].x ) };
                if( d > A && d < B ) {
                    stations.push_back( d );
                }
            }
        }
    }
    std::sort( stations.begin(), stations.end() );
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
    cut.endcap = false;
    cut.startcap = false;
    auto const outwards { Band.facing / Band.into > 0.0 };
    ( outwards ? cut.endinner : cut.endextents ) = endknots;
    ( outwards ? cut.startextents : cut.startinner ) = startknots;
    return cut;
}

rail_cut seam( double const At, double const End, double const Start ) {

    rail_cut cut;
    cut.at = At;
    cut.end = End;
    cut.start = Start;
    cut.endcap = false;
    cut.startcap = false;
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

} // namespace

class TTrack::rail_layer {

public:
// constructors
    rail_layer( TTrack &Track, rail_layout const &Layout, rail_drawing const &Drawing, int const Path, gfx::vertex_array const &Profile, int const Side, float const Texturelength, double const From, bool const Openstart, bool const Openend, geometryhandle_sequence &Geometry, geometryhandle_sequence &Cuts );
// methods
    bool lay();

private:
// methods
    template <typename Type_>
    Type_ const &ours( Type_ const ( &Sides )[ 2 ][ 2 ] ) const {
        return Sides[ m_path ][ m_side > 0 ]; }
    std::vector<rail_tread> const &treads() const {
        return ours( m_layout.treads ); }
    void lay_guards();
    void lay_checks();
    void lay_fills();
    void lay_cuts( gfx::vertex_array const &Shape, std::vector<rail_cut> Cuts, bool const Wheels, std::vector<rail_tread> const &Treads, float const Wallu = -1.f );
    void lay_grooved( gfx::vertex_array const &Outline );
    grooved_profile split_grooved( gfx::vertex_array const &Outline ) const;
    std::vector<rail_cut> top_cuts( gfx::vertex_array const &Top ) const;
    void running( rail_piece const &Whole, knots::line const &Rolled, piece_ends const &Ends );
    void treaded( rail_piece Whole, rail_tread const &Tread, piece_ends const &Ends );
    gfx::vertex_array lay_sides( rail_piece const &Whole, piece_ends const &Ends );
    void lay_top( gfx::vertex_array const &Top, rail_piece const &Whole, glm::dvec2 const &Reach, std::vector<double> const &Splits, std::vector<top_band> const &Bands, std::function<std::vector<double>( double )> const &Edges, piece_ends const &Ends );
    void piece( gfx::vertex_array const &Shape, rail_piece const &Piece, piece_ends const &Ends = {} );
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

TTrack::rail_layer::rail_layer( TTrack &Track, rail_layout const &Layout, rail_drawing const &Drawing, int const Path, gfx::vertex_array const &Profile, int const Side, float const Texturelength, double const From, bool const Openstart, bool const Openend, geometryhandle_sequence &Geometry, geometryhandle_sequence &Cuts ) :
    m_track { Track },
    m_layout { Layout },
    m_drawing { Drawing },
    m_path { Path },
    m_profile { Profile },
    m_side { Side },
    m_texturelength { Texturelength },
    m_from { From },
    m_openstart { Openstart },
    m_openend { Openend },
    m_geometry { Geometry },
    m_cuts { Cuts },
    m_segment { path_segment( Track, Path ) },
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
    auto const mitered = [&]( glm::dvec2 const &Miter, double const Square, bool const Latest ) {
        if( Miter.y == 0.0 ) {
            return Square;
        }
        auto const inside { Miter.x + Miter.y * ( size.low + m_guardoffset ) };
        auto const outside { Miter.x + Miter.y * ( top + m_guardoffset ) };
        return Latest ? std::max( inside, outside ) : std::min( inside, outside ); };
    for( auto const &guard : ours( m_layout.guards ) ) {
        auto const offsets { flared( guard.from, guard.to, std::min( 0.3, 0.25 * ( guard.to - guard.from ) ), m_guardoffset, guard.flarefrom ? 0.02 : 0.0, guard.flareto ? 0.02 : 0.0 ) };
        auto cuts { guard.plan.cuts };
        sort_cuts( cuts );
        auto start { guard.from };
        auto startmiter { guard.frommiter };
        rail_cut startcut;
        for( std::size_t i = 0; i <= cuts.size(); ++i ) {
            auto const end { i < cuts.size() ? cuts[ i ].end : guard.to };
            auto const endmiter { i < cuts.size() ? cuts[ i ].endmiter : guard.tomiter };
            auto const endcut { i < cuts.size() ? cuts[ i ] : rail_cut {} };
            auto const a { std::max( { mitered( startmiter, start, false ), m_from, 0.0 } ) };
            auto const b { std::min( { mitered( endmiter, end, true ), m_length, mitered( guard.tomiter, guard.to, true ) } ) };
            if( b - a > 0.01 ) {
                knots::line uppers { { 0.0, top } };
                knots::line lowers;
                for( auto const &[ miter, ending ] : { std::pair{ startmiter, false }, std::pair{ endmiter, true } } ) {
                    if( miter.y != 0.0 ) {
                        ( ending == ( miter.y < 0.0 ) ? uppers : lowers ).emplace_back( 1.0 / miter.y, -miter.x / miter.y - m_guardoffset );
                    }
                }
                auto const glued { a == start && false == startcut.startfit.empty() };
                auto const gluedend { b == end && false == endcut.endfit.empty() };
                piece(
                    m_profile,
                    { .from = a, .to = b, .offsets = offsets, .extents = envelope( uppers, true, a, b ), .inner = envelope( lowers, false, a, b ), .capfrom = false == glued, .capto = false == gluedend, .rolled = false },
                    { a, b, { glued ? startcut.startbend : 0.0, glued ? startcut.startfit : knots::line {} }, { gluedend ? endcut.endbend : 0.0, gluedend ? endcut.endfit : knots::line {} } } );
            }
            if( i < cuts.size() && cuts[ i ].start > start ) {
                start = cuts[ i ].start;
                startmiter = cuts[ i ].startmiter;
                startcut = cuts[ i ];
            }
        }
    }
}

void TTrack::rail_layer::lay_checks() {

    auto const &shape { m_drawing.checkshapes[ m_side > 0 ? 0 : 1 ] };
    auto const &wingends { ours( m_layout.wingends ) };
    for( auto const &check : m_layout.checks[ m_path ][ m_side > 0 ] ) {
        auto const replaced { std::any_of( wingends.begin(), wingends.end(), [&]( glm::dvec2 const &Wing ) {
            auto farend { 0.0 };
            return replaced_check( { check }, Wing.x, Wing.y, farend ); } ) };
        auto const a { std::max( check.from, m_from ) };
        auto const b { std::min( check.to, m_length ) };
        if( replaced || b - a <= 0.01 ) {
            continue;
        }
        piece(
            shape,
            { .from = a, .to = b, .offsets = flared( check.from, check.to, std::min( 0.6, 0.4 * ( check.to - check.from ) ), 0.0, check.flarefrom, check.flareto ), .capfrom = a > 0.0 || check.from >= 0.0, .capto = check.to <= m_length, .rolled = false },
            { a, b, { a == check.from ? check.frombend : 0.0 }, { b == check.to ? check.tobend : 0.0 } } );
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
    auto start { -1e9 };
    auto startcap { true };
    auto startbend { 0.0 };
    knots::line startfit;
    knots::line rolledfrom;
    knots::line offsets, extents, inner;
    for( std::size_t i = 0; i <= Cuts.size(); ++i ) {
        auto const end { i < Cuts.size() ? Cuts[ i ].end : 1e9 };
        auto pieceoffsets { offsets };
        auto pieceextents { extents };
        auto pieceinner { inner };
        if( i < Cuts.size() ) {
            pieceoffsets.insert( pieceoffsets.end(), Cuts[ i ].endoffsets.begin(), Cuts[ i ].endoffsets.end() );
            pieceextents.insert( pieceextents.end(), Cuts[ i ].endextents.begin(), Cuts[ i ].endextents.end() );
            pieceinner.insert( pieceinner.end(), Cuts[ i ].endinner.begin(), Cuts[ i ].endinner.end() );
        }
        knots::sort( pieceoffsets );
        knots::sort( pieceextents );
        knots::sort( pieceinner );
        auto const a { std::max( start, m_from ) };
        auto const b { std::min( end, m_length ) };
        auto const rolled { i < Cuts.size() ? knots::bound( rolledfrom, Cuts[ i ].endrolled, true ) : rolledfrom };
        auto const capstart { start > m_from ? startcap : m_openstart };
        auto const capend { end < m_length ? Cuts[ i ].endcap : m_openend };
        piece_ends const ends {
            a, b,
            { start > m_from ? startbend : 0.0, start > m_from ? startfit : knots::line {} },
            { end < m_length ? Cuts[ i ].endbend : 0.0, end < m_length ? Cuts[ i ].endfit : knots::line {} } };
        if( b - a > 0.01 ) {
            std::vector<rail_tread> active;
            for( auto const &tread : Treads ) {
                if( tread.anchor == 0
                 || ( tread.anchor > 0 && b > tread.from + 0.01 && b < tread.to + 0.01 )
                 || ( tread.anchor < 0 && a > tread.from - 0.01 && a < tread.to - 0.01 ) ) {
                    active.push_back( tread );
                }
            }
            std::vector<double> splits;
            for( auto const &tread : active ) {
                for( auto const edge : { tread.from, tread.to } ) {
                    if( edge > a + 0.01 && edge < b - 0.01 ) {
                        splits.push_back( edge );
                    }
                }
            }
            std::sort( splits.begin(), splits.end() );
            splits.erase( std::unique( splits.begin(), splits.end(), []( double const A, double const B ) { return B - A < 0.002; } ), splits.end() );
            splits.insert( splits.begin(), a );
            splits.push_back( b );
            for( std::size_t k = 1; k < splits.size(); ++k ) {
                auto const *tread { tread_at( active, 0.5 * ( splits[ k - 1 ] + splits[ k ] ) ) };
                rail_piece const whole {
                    .material = tread_material( tread ),
                    .from = splits[ k - 1 ],
                    .to = splits[ k ],
                    .offsets = pieceoffsets,
                    .extents = pieceextents,
                    .inner = pieceinner,
                    .capfrom = k == 1 && capstart,
                    .capto = k + 1 == splits.size() && capend,
                    .walls = false == Wheels,
                    .wallu = Wallu };
                if( Wheels && tread != nullptr && false == tread->edge.empty() && m_topfirst < m_toplast ) {
                    treaded( whole, *tread, ends );
                }
                else if( false == Wheels || rolled.empty() || m_topfirst >= m_toplast ) {
                    piece( Shape, whole, ends );
                }
                else {
                    running( whole, rolled, ends );
                }
            }
        }
        if( i < Cuts.size() ) {
            rolledfrom = Cuts[ i ].startrolled;
            start = Cuts[ i ].start;
            startcap = Cuts[ i ].startcap;
            startbend = Cuts[ i ].startbend;
            startfit = Cuts[ i ].startfit;
            offsets = Cuts[ i ].startoffsets;
            extents = Cuts[ i ].startextents;
            inner = Cuts[ i ].startinner;
        }
    }
}

void TTrack::rail_layer::lay_grooved( gfx::vertex_array const &Outline ) {

    auto const grooved { split_grooved( Outline ) };
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
        std::vector<rail_tread> blocktreads;
        for( auto const &span : merged ) {
            for( auto const &tread : treads() ) {
                if( tread.from < span.y && tread.to > span.x ) {
                    blocktreads.push_back( { span.x, span.y, tread.material } );
                    break;
                }
            }
        }
        std::vector<rail_cut> outside, inside;
        for( auto const &cut : cuts ) {
            auto const within { std::any_of( merged.begin(), merged.end(), [&]( glm::dvec2 const &Span ) { return cut.at > Span.x && cut.at < Span.y; } ) };
            ( within ? inside : outside ).push_back( cut );
        }
        inside.push_back( seam( merged.front().x - 0.001, -1e9, merged.front().x ) );
        for( std::size_t m = 0; m < merged.size(); ++m ) {
            outside.push_back( seam( 0.5 * ( merged[ m ].x + merged[ m ].y ), merged[ m ].x, merged[ m ].y ) );
            inside.push_back( m + 1 < merged.size() ?
                seam( 0.5 * ( merged[ m ].y + merged[ m + 1 ].x ), merged[ m ].y, merged[ m + 1 ].x ) :
                seam( merged[ m ].y + 0.001, merged[ m ].y, 1e9 ) );
        }
        lay_cuts( grooved.tops[ k ], outside, false, treads(), grooved.wallu );
        lay_cuts( grooved.flats[ k ], inside, false, blocktreads, grooved.wallu );
    }
}

grooved_profile TTrack::rail_layer::split_grooved( gfx::vertex_array const &Outline ) const {

    auto const &size { m_drawing.size };
    auto const count { m_profile.size() };
    auto const height = [&]( std::size_t const Index ) {
        return static_cast<double>( Outline[ m_side > 0 ? Index : count - 1 - Index ].position.y ) - size.groove; };
    auto const above = [&]( std::size_t const Index ) {
        return height( Index ) > 1e-4; };
    auto const floorpoint = [&]( std::size_t const A, std::size_t const B ) {
        auto const t { static_cast<float>( height( A ) / ( height( A ) - height( B ) ) ) };
        auto point { m_profile[ above( A ) ? A : B ] };
        point.position = glm::mix( m_profile[ A ].position, m_profile[ B ].position, t );
        point.texture = glm::mix( m_profile[ A ].texture, m_profile[ B ].texture, t );
        return point; };
    auto const flattened = [&]( std::size_t const Index ) {
        auto point { m_profile[ Index ] };
        auto const &file { Outline[ m_side > 0 ? Index : count - 1 - Index ].position };
        if( file.y > size.crest - 0.0005 || ( file.x < -0.02 && file.y > size.liptop - 0.0005 ) ) {
            point.normal = { 0.f, 1.f, 0.f };
            point.texture.x = m_drawing.platetexture;
        }
        return point; };
    grooved_profile result;
    auto &body { result.body };
    std::vector<std::size_t> floors;
    auto floortexture { 0.f };
    auto floortextures { 0 };
    for( std::size_t i = 0; i < count; ++i ) {
        if( false == above( i ) ) {
            if( height( i ) > -1e-4 ) {
                floors.push_back( body.size() );
                floortexture += m_profile[ i ].texture.x;
                ++floortextures;
            }
            body.push_back( m_profile[ i ] );
            continue;
        }
        auto const first { i };
        while( i + 1 < count && above( i + 1 ) ) {
            ++i;
        }
        gfx::vertex_array top, flat;
        if( first > 0 ) {
            if( height( first - 1 ) > -1e-4 ) {
                top.push_back( m_profile[ first - 1 ] );
                flat.push_back( flattened( first - 1 ) );
            }
            else {
                top.push_back( floorpoint( first - 1, first ) );
                flat.push_back( top.back() );
                body.push_back( top.back() );
                floors.push_back( body.size() );
                body.push_back( top.back() );
            }
        }
        for( auto k { first }; k <= i; ++k ) {
            top.push_back( m_profile[ k ] );
            flat.push_back( flattened( k ) );
        }
        if( i + 1 < count ) {
            if( height( i + 1 ) > -1e-4 ) {
                top.push_back( m_profile[ i + 1 ] );
                flat.push_back( flattened( i + 1 ) );
            }
            else {
                top.push_back( floorpoint( i, i + 1 ) );
                flat.push_back( top.back() );
                floors.push_back( body.size() );
                body.push_back( top.back() );
                body.push_back( top.back() );
            }
        }
        for( auto *strip : { &top, &flat } ) {
            gfx::vertex_array doubled { strip->front(), strip->front() };
            for( std::size_t k = 1; k + 1 < strip->size(); ++k ) {
                doubled.insert( doubled.end(), 2, ( *strip )[ k ] );
            }
            doubled.insert( doubled.end(), 2, strip->back() );
            *strip = doubled;
        }
        result.tops.push_back( top );
        result.flats.push_back( flat );
    }
    if( floors.size() > 1 ) {
        auto const across { body[ floors.back() ].position - body[ floors.front() ].position };
        auto up { glm::normalize( glm::vec3{ -across.y, across.x, 0.f } ) };
        if( up.y < 0.f ) {
            up = -up;
        }
        for( auto const index : floors ) {
            body[ index ].normal = up;
            if( floortextures > 0 ) {
                body[ index ].texture.x = floortexture / floortextures;
            }
        }
    }
    result.wallu = ( floortextures > 0 ? floortexture / floortextures : -1.f );
    return result;
}

std::vector<rail_cut> TTrack::rail_layer::top_cuts( gfx::vertex_array const &Top ) const {

    auto const &size { m_drawing.size };
    auto const reach { reaches( Top ) };
    auto const head { reach.x + reach.y > 0.0 };
    auto edge { head ? 1e9 : -1e9 };
    for( auto const &point : Top ) {
        if( point.position.y > ( head ? size.crest : size.liptop ) - 0.0005 ) {
            auto const across { static_cast<double>( m_side * point.position.x - m_drawing.halfwidth ) };
            edge = ( head ? std::min( edge, across ) : std::max( edge, across ) );
        }
    }
    auto cuts { ours( m_layout.plans ).cuts };
    for( auto const &band : ours( m_layout.bands ) ) {
        auto cut { band_cut( band, reach ) };
        if( false == band.whole && std::abs( band.facing ) >= 1e-3 && std::abs( edge ) < 1.0 ) {
            auto const blunt { edge + ( head ? nose_tip : -nose_tip ) };
            if( head == ( band.facing / band.into > 0.0 ) ) {
                cut.start = std::max( cut.start, band.at + ( ( band.into > 0.0 ? band.to : band.from ) + band.facing * blunt ) / band.into );
                cut.startcap = true;
            }
            else {
                cut.end = std::min( cut.end, band.at + ( ( band.into > 0.0 ? band.from : band.to ) + band.facing * blunt ) / band.into );
                cut.endcap = true;
            }
        }
        cuts.push_back( cut );
    }
    return cuts;
}

void TTrack::rail_layer::running( rail_piece const &Whole, knots::line const &Rolled, piece_ends const &Ends ) {

    auto const top { lay_sides( Whole, Ends ) };
    auto const reach { reaches( top ) };
    auto const inner { knots::shifted( Rolled, -blend ) };
    auto const splits { cross_splits( Whole, reach, { &Rolled, &inner }, 0.0 ) };
    auto const marked { treads_marked() };
    auto const own { Whole.material == null_handle ? m_track.rail_material( m_path ) : Whole.material };
    lay_top(
        top, Whole, reach, splits,
        {
            { Whole.material, knots::bound( Whole.extents, inner, true ), Whole.inner },
            { marked ? paint( own, { 1.f, 1.f, 0.f } ) : Whole.material, knots::bound( Whole.extents, Rolled, true ), knots::bound( inner, Whole.inner, false ), false, marked ? knots::line {} : Rolled },
            { marked ? paint( own, { 1.f, 0.f, 1.f } ) : Whole.material, Whole.extents, knots::bound( Rolled, Whole.inner, false ), false } },
        [&]( double const Middle ) {
            auto const rolled { knots::sample( Rolled, Middle, 0.0 ) };
            return std::vector<double> { rolled - blend, rolled }; },
        Ends );
}

void TTrack::rail_layer::treaded( rail_piece Whole, rail_tread const &Tread, piece_ends const &Ends ) {

    Whole.material = null_handle;
    auto const top { lay_sides( Whole, Ends ) };
    auto const reach { reaches( top ) };
    auto const &lower { Tread.below ? Tread.gauge : Tread.edge };
    auto const &upper { Tread.below ? Tread.edge : Tread.gauge };
    auto const marked { treads_marked() };
    auto const own { m_track.rail_material( m_path ) };
    auto const colour { 0.5f * ( tread_colour( Tread.material, m_track.tread_band() ) + tread_colour( own, m_track.tread_band() ) ) };
    auto const tint = [&]( material_handle const Material, glm::vec3 const &Mark ) {
        return paint( Material, marked ? Mark : colour ); };
    auto const owntint { tint( own, { 1.f, 0.f, 0.f } ) };
    auto const sharedtint { tint( Tread.material, { 0.f, 1.f, 0.f } ) };
    auto const shared { marked ? tint( Tread.material, { 0.f, 0.f, 1.f } ) : Tread.material };
    auto const ramp = [&]( knots::line const &Knots ) {
        return marked ? knots::line {} : Knots; };
    auto const centre { knots::middle( lower, upper ) };
    auto const beneath { knots::shifted( lower, -tread_fade ) };
    auto const beyond { knots::shifted( upper, tread_fade ) };
    auto const lowfade { knots::bound( knots::shifted( lower, tread_fade ), centre, true ) };
    auto const highfade { knots::bound( knots::shifted( upper, -tread_fade ), centre, false ) };
    auto splits { cross_splits( Whole, reach, { &lower, &upper, &lowfade, &highfade, &beneath, &beyond }, 0.001 ) };
    add_meetings( knots::shifted( lower, 2.0 * tread_fade ), upper, Whole.from, Whole.to, 0.001, splits );
    std::sort( splits.begin(), splits.end() );
    lay_top(
        top, Whole, reach, splits,
        {
            { null_handle, knots::bound( Whole.extents, beneath, true ), Whole.inner },
            { owntint, knots::bound( Whole.extents, lower, true ), knots::bound( Whole.inner, beneath, false ), false, ramp( lower ), tread_fade },
            { sharedtint, knots::bound( Whole.extents, lowfade, true ), knots::bound( Whole.inner, lower, false ), false, ramp( lower ), -tread_fade },
            { shared, knots::bound( Whole.extents, highfade, true ), knots::bound( Whole.inner, lowfade, false ), false == marked },
            { sharedtint, knots::bound( Whole.extents, upper, true ), knots::bound( Whole.inner, highfade, false ), false, ramp( upper ), tread_fade },
            { owntint, knots::bound( Whole.extents, beyond, true ), knots::bound( Whole.inner, upper, false ), false, ramp( upper ), -tread_fade },
            { null_handle, Whole.extents, knots::bound( Whole.inner, beyond, false ) } },
        [&]( double const Middle ) {
            auto const from { knots::sample( lower, Middle, 0.0 ) };
            auto const to { knots::sample( upper, Middle, 0.0 ) };
            return std::vector<double> { from - tread_fade, from, knots::sample( lowfade, Middle, 0.0 ), knots::sample( highfade, Middle, 0.0 ), to, to + tread_fade }; },
        Ends );
}

gfx::vertex_array TTrack::rail_layer::lay_sides( rail_piece const &Whole, piece_ends const &Ends ) {

    gfx::vertex_array vertices;
    if( m_segment.RenderLoftSpan( vertices, m_track.m_origin, m_profile, span( Whole, Ends ) ) ) {
        cut_faces( vertices, m_profile.size(), Whole.capfrom, Whole.capto, m_drawing.bank, sink( Whole.material, true ) );
    }
    auto side { Whole };
    side.capfrom = false;
    side.capto = false;
    piece( strip( 0, m_topfirst ), side, Ends );
    piece( strip( m_toplast, m_profile.size() - 1 ), side, Ends );
    return strip( m_topfirst, m_toplast );
}

void TTrack::rail_layer::lay_top( gfx::vertex_array const &Top, rail_piece const &Whole, glm::dvec2 const &Reach, std::vector<double> const &Splits, std::vector<top_band> const &Bands, std::function<std::vector<double>( double )> const &Edges, piece_ends const &Ends ) {

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

void TTrack::rail_layer::piece( gfx::vertex_array const &Shape, rail_piece const &Piece, piece_ends const &Ends ) {

    gfx::vertex_array vertices;
    gfx::userdata_array userdata;
    auto loft { span( Piece, Ends ) };
    loft.userdata = ( Piece.rolled ? nullptr : &userdata );
    if( Shape.size() < 2 || false == m_segment.RenderLoftSpan( vertices, m_track.m_origin, Shape, loft ) ) {
        return;
    }
    cut_faces( vertices, Shape.size(), Piece.capfrom, Piece.capto, m_drawing.bank, sink( Piece.material, true ) );
    gfx::vertex_array triangles;
    gfx::userdata_array triangleuserdata;
    auto const ring { 2 * Shape.size() };
    for( std::size_t block = 0; block + ring <= vertices.size(); block += ring ) {
        for( std::size_t j = 0; j + 1 < Shape.size(); ++j ) {
            for( auto const index : { 0, 1, 2, 1, 3, 2 } ) {
                auto const vertex { block + 2 * j + index };
                triangles.push_back( vertices[ vertex ] );
                if( false == Piece.rolled ) {
                    triangleuserdata.push_back( userdata[ vertex ] );
                }
            }
        }
    }
    sink( Piece.material, false == Piece.rolled ).emplace_back( GfxRenderer->Insert( triangles, triangleuserdata, m_drawing.bank, GL_TRIANGLES ) );
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
    auto const tinted { std::find( m_tints.begin(), m_tints.end(), Material ) != m_tints.end() };
    auto &treads { m_track.GeometryTreads };
    auto found { std::find_if( treads.begin(), treads.end(), [&]( tread_geometry const &Tread ) { return Tread.material == Material; } ) };
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

bool TTrack::lay_rail( rail_layout const &Layout, rail_drawing const &Drawing, int const Path, gfx::vertex_array const &Profile, int const Side, float const Texturelength, double const From, bool const Openstart, bool const Openend, geometryhandle_sequence &Geometry, geometryhandle_sequence &Cuts ) {

    return rail_layer( *this, Layout, Drawing, Path, Profile, Side, Texturelength, From, Openstart, Openend, Geometry, Cuts ).lay();
}

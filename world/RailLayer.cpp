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

TTrack::rail_drawing TTrack::make_rail_drawing( gfx::geometrybank_handle const &Bank ) {

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
    create_track_rail_profile( drawing.checkshapes[ 0 ], drawing.checkshapes[ 1 ], &checkshape );
    for( auto &shape : drawing.checkshapes ) {
        shape.resize( checkshape.size() );
    }
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
        gfx::vertex_array plate {
            {{  1.f, bottom, 0.f }, {  1.f, 0.f, 0.f }, { texture, 0.f }},
            {{  1.f, top, 0.f }, {  1.f, 0.f, 0.f }, { texture, 0.f }},
            {{  1.f, top, 0.f }, {  0.f, 1.f, 0.f }, { texture, 0.f }},
            {{ -1.f, top, 0.f }, {  0.f, 1.f, 0.f }, { texture, 0.f }},
            {{ -1.f, top, 0.f }, { -1.f, 0.f, 0.f }, { texture, 0.f }},
            {{ -1.f, bottom, 0.f }, { -1.f, 0.f, 0.f }, { texture, 0.f }} };
        drawing.platetexture = texture;
        create_track_rail_profile( drawing.plates[ 0 ], drawing.plates[ 1 ], &plate );
        for( auto &shape : drawing.plates ) {
            shape.resize( plate.size() );
        }
        auto const neck { static_cast<float>( drawing.size.neck ) };
        auto webtexture { texture };
        for( std::size_t i = 0; i < outline.size() / 2; ++i ) {
            auto const &point { outline[ i ] };
            if( point.position.y < neck && point.normal.y > 0.9f ) {
                webtexture = point.texture.s;
                break;
            }
        }
        gfx::vertex_array filler {
            {{  1.f, bottom, 0.f }, {  1.f, 0.f, 0.f }, { webtexture, 0.f }},
            {{  1.f, neck, 0.f }, {  1.f, 0.f, 0.f }, { webtexture, 0.f }},
            {{  1.f, neck, 0.f }, {  0.f, 1.f, 0.f }, { webtexture, 0.f }},
            {{ -1.f, neck, 0.f }, {  0.f, 1.f, 0.f }, { webtexture, 0.f }},
            {{ -1.f, neck, 0.f }, { -1.f, 0.f, 0.f }, { webtexture, 0.f }},
            {{ -1.f, bottom, 0.f }, { -1.f, 0.f, 0.f }, { webtexture, 0.f }} };
        create_track_rail_profile( drawing.fillers[ 0 ], drawing.fillers[ 1 ], &filler );
        for( auto &shape : drawing.fillers ) {
            shape.resize( filler.size() );
        }
    }
    return drawing;
}

bool TTrack::lay_rail( rail_layout const &Layout, rail_drawing const &Drawing, int const Path, gfx::vertex_array const &Profile, int const Side, float const Texturelength, double const From, bool const Openstart, bool const Openend, geometryhandle_sequence &Geometry, geometryhandle_sequence &Cuts ) {

    gfx::userdata_array empty_userdata;
    auto const &size { Drawing.size };
    auto const halfwidth { Drawing.halfwidth };
    auto const railhead { size.head };
    auto const guardoffset { -( flangeway + railhead ) };
    auto const &segment { path_segment( *this, Path ) };
    auto const &plan { Layout.plans[ Path ][ Side > 0 ] };
    auto const length { segment.GetLength() };
    auto const &treads { Layout.treads[ Path ][ Side > 0 ] };
    material_handle current { null_handle };
    std::vector<material_handle> tints;
    static auto const marked { std::getenv( "EU07_EDITOR_TREADS" ) != nullptr };
    auto const paint = [&]( material_handle const Material, glm::vec3 const &Colour ) {
        auto const tinted { tint_material( Material, Colour ) };
        if( tinted != Material ) {
            tints.push_back( tinted );
        }
        return tinted; };
    std::vector<glm::dvec2> const unramped;
    auto const sink = [&]( bool const Matte ) -> geometryhandle_sequence & {
        if( current == null_handle ) {
            return Matte ? Cuts : Geometry;
        }
        if( current == m_material1 ) {
            return Matte ? GeometryCut1 : Geometry1;
        }
        if( eType == tt_Switch && current == m_material2 ) {
            return Matte ? GeometryCut2 : Geometry2;
        }
        auto const tinted { std::find( tints.begin(), tints.end(), current ) != tints.end() };
        auto found { std::find_if( GeometryTreads.begin(), GeometryTreads.end(), [&]( tread_geometry const &Tread ) { return Tread.material == current; } ) };
        if( found == GeometryTreads.end() ) {
            found = GeometryTreads.insert( GeometryTreads.end(), { current, tinted ? current : cut_material( current ), {}, {} } );
        }
        return Matte && false == tinted ? found->cuts : found->geometry; };
    auto const texturelength = [&]() {
        return ( current == null_handle ? Texturelength : texture_length( current ) ); };
    auto const aim = [&]( std::vector<rail_tread> const &Spans, double const Middle ) -> rail_tread const * {
        current = null_handle;
        for( auto const &span : Spans ) {
            if( Middle > span.from && Middle < span.to ) {
                current = span.material;
                return &span;
            }
        }
        return nullptr;
    };
    // parts no wheel rolls on go in the matte material of the cut faces
    glm::dvec2 bendfrom { 0.0 };
    glm::dvec2 bendto { 0.0 };
    auto const bends = [&]( double const Start, double const End ) {
        return glm::dvec2 {
            std::abs( Start - bendfrom.x ) < 1e-6 ? bendfrom.y : 0.0,
            std::abs( End - bendto.x ) < 1e-6 ? bendto.y : 0.0 }; };
    std::pair<double, std::vector<glm::dvec2>> fitfrom, fitto;
    std::vector<glm::dvec2> const unfitted;
    auto const fitted = [&]( std::pair<double, std::vector<glm::dvec2>> const &Fit, double const Distance ) -> std::vector<glm::dvec2> const & {
        return ( std::abs( Distance - Fit.first ) < 1e-6 ? Fit.second : unfitted ); };
    auto const span = [&]( double const Start, double const End, knots::line const &Offsets, knots::line const &Extents, knots::line const &Inner ) {
        loft_span result;
        result.side = Side;
        result.halfwidth = halfwidth;
        result.texturelength = texturelength();
        result.from = Start;
        result.to = End;
        result.offsets = Offsets;
        result.extents = Extents;
        result.inner = Inner;
        auto const bent { bends( Start, End ) };
        result.fromend = { bent.x, fitted( fitfrom, Start ) };
        result.toend = { bent.y, fitted( fitto, End ) };
        return result; };
    auto const piece = [&]( double const Start, double const End, std::vector<glm::dvec2> const &Offsets, std::vector<glm::dvec2> const &Extents, bool const Capstart, bool const Capend, gfx::vertex_array const &Shape, bool const Rolled, std::vector<glm::dvec2> const &Inner, std::vector<glm::dvec2> const &Ramp = {}, bool const Walls = false, float const Lift = 0.f, float const Wallu = -1.f, double const Fade = blend ) {
        gfx::vertex_array vertices;
        gfx::userdata_array userdata;
        auto loft { span( Start, End, Offsets, Extents, Inner ) };
        loft.lift = Lift;
        loft.userdata = ( Rolled ? nullptr : &userdata );
        loft.ramp = Ramp;
        loft.rampwidth = ( Ramp.empty() ? 0.0 : Fade );
        loft.walls = Walls;
        loft.wallu = Wallu;
        if( Shape.size() > 1 && segment.RenderLoftSpan( vertices, m_origin, Shape, loft ) ) {
            cut_faces( vertices, Shape.size(), Capstart, Capend, Drawing.bank, sink( true ) );
            gfx::vertex_array triangles;
            gfx::userdata_array triangleuserdata;
            auto const ring { 2 * Shape.size() };
            for( std::size_t block = 0; block + ring <= vertices.size(); block += ring ) {
                for( std::size_t j = 0; j + 1 < Shape.size(); ++j ) {
                    for( auto const index : { 0, 1, 2, 1, 3, 2 } ) {
                        auto const vertex { block + 2 * j + index };
                        triangles.push_back( vertices[ vertex ] );
                        if( false == Rolled ) {
                            triangleuserdata.push_back( userdata[ vertex ] );
                        }
                    }
                }
            }
            sink( false == Rolled ).emplace_back( GfxRenderer->Insert( triangles, Rolled ? empty_userdata : triangleuserdata, Drawing.bank, GL_TRIANGLES ) );
        }
    };
    // the running surface on top of the head is split along the edge of the wheel treads, the part of it out of their reach goes matte; the rest of the profile and the end faces are laid whole
    std::size_t topfirst { Profile.size() };
    std::size_t toplast { 0 };
    for( std::size_t j = 0; j < Profile.size(); ++j ) {
        if( Profile[ j ].position.y > static_cast<float>( -0.087 * size.height ) ) {
            topfirst = std::min( topfirst, j );
            toplast = std::max( toplast, j );
        }
    }
    auto const strip = [&]( std::size_t const First, std::size_t const Last ) {
        return gfx::vertex_array( Profile.begin() + First, Profile.begin() + Last + 1 ); };
    auto const crossings = []( knots::line const &A, knots::line const &B, double const Start, double const End, double const Margin, std::vector<double> &Splits ) {
        for( auto const distance : knots::meetings( A, B ) ) {
            if( distance > Start + Margin && distance < End - Margin ) {
                Splits.push_back( distance );
            }
        }
    };
    auto const running = [&]( double const Start, double const End, std::vector<glm::dvec2> const &Offsets, std::vector<glm::dvec2> const &Extents, std::vector<glm::dvec2> const &Inner, std::vector<glm::dvec2> const &Rolled, bool const Capstart, bool const Capend ) {
        gfx::vertex_array vertices;
        if( segment.RenderLoftSpan( vertices, m_origin, Profile, span( Start, End, Offsets, Extents, Inner ) ) ) {
            cut_faces( vertices, Profile.size(), Capstart, Capend, Drawing.bank, sink( true ) );
        }
        piece( Start, End, Offsets, Extents, false, false, strip( 0, topfirst ), true, Inner );
        piece( Start, End, Offsets, Extents, false, false, strip( toplast, Profile.size() - 1 ), true, Inner );
        auto const top { strip( topfirst, toplast ) };
        auto low { 1e9 };
        auto high { -1e9 };
        for( auto const &point : top ) {
            low = std::min( low, static_cast<double>( Side * point.position.x - halfwidth ) );
            high = std::max( high, static_cast<double>( Side * point.position.x - halfwidth ) );
        }
        auto const inner { knots::shifted( Rolled, -blend ) };
        std::vector<double> splits { Start, End };
        std::vector<glm::dvec2> const lowline { { Start, low }, { End, low } };
        std::vector<glm::dvec2> const highline { { Start, high }, { End, high } };
        for( auto const *knots : { &Rolled, &inner } ) {
            for( auto const *limit : { &lowline, &highline, &Extents, &Inner } ) {
                crossings( *knots, *limit, Start, End, 0.0, splits );
            }
        }
        std::sort( splits.begin(), splits.end() );
        auto const shiny { knots::bound( Extents, inner, true ) };
        auto const blended { knots::bound( Extents, Rolled, true ) };
        for( std::size_t i = 1; i < splits.size(); ++i ) {
            auto const middle { 0.5 * ( splits[ i - 1 ] + splits[ i ] ) };
            auto const reach { knots::sample( Rolled, middle, 0.0 ) };
            auto const least { std::max( low, Inner.empty() ? low : knots::sample( Inner, middle, 0.0 ) ) };
            auto const most { std::min( high, Extents.empty() ? high : knots::sample( Extents, middle, 0.0 ) ) };
            auto const shown = [&]( double const Lowest, double const Highest ) {
                return Highest > Lowest + 1e-6 && Highest > least && Lowest < most; };
            if( shown( -1e9, reach - blend ) ) {
                piece( splits[ i - 1 ], splits[ i ], Offsets, shiny, false, false, top, true, Inner );
            }
            auto const plain { current };
            auto const own { current == null_handle ? rail_material( Path ) : current };
            if( shown( reach - blend, reach ) ) {
                current = ( marked ? paint( own, { 1.f, 1.f, 0.f } ) : plain );
                piece( splits[ i - 1 ], splits[ i ], Offsets, blended, false, false, top, false, knots::bound( inner, Inner, false ), marked ? unramped : Rolled );
            }
            if( shown( reach, 1e9 ) ) {
                current = ( marked ? paint( own, { 1.f, 0.f, 1.f } ) : plain );
                piece( splits[ i - 1 ], splits[ i ], Offsets, Extents, false, false, top, false, knots::bound( Rolled, Inner, false ) );
            }
            current = plain;
        }
    };
    auto const treaded = [&]( double const Start, double const End, std::vector<glm::dvec2> const &Offsets, std::vector<glm::dvec2> const &Extents, std::vector<glm::dvec2> const &Inner, bool const Capstart, bool const Capend, rail_tread const &Span ) {
        current = null_handle;
        gfx::vertex_array vertices;
        if( segment.RenderLoftSpan( vertices, m_origin, Profile, span( Start, End, Offsets, Extents, Inner ) ) ) {
            cut_faces( vertices, Profile.size(), Capstart, Capend, Drawing.bank, sink( true ) );
        }
        piece( Start, End, Offsets, Extents, false, false, strip( 0, topfirst ), true, Inner );
        piece( Start, End, Offsets, Extents, false, false, strip( toplast, Profile.size() - 1 ), true, Inner );
        auto const top { strip( topfirst, toplast ) };
        auto low { 1e9 };
        auto high { -1e9 };
        for( auto const &point : top ) {
            low = std::min( low, static_cast<double>( Side * point.position.x - halfwidth ) );
            high = std::max( high, static_cast<double>( Side * point.position.x - halfwidth ) );
        }
        auto const &lower { Span.below ? Span.gauge : Span.edge };
        auto const &upper { Span.below ? Span.edge : Span.gauge };
        auto const own { rail_material( Path ) };
        auto const colour { 0.5f * ( tread_colour( Span.material, tread_band() ) + tread_colour( own, tread_band() ) ) };
        auto const tint = [&]( material_handle const Material, glm::vec3 const &Mark ) {
            return paint( Material, marked ? Mark : colour ); };
        auto const owntint { tint( own, { 1.f, 0.f, 0.f } ) };
        auto const sharedtint { tint( Span.material, { 0.f, 1.f, 0.f } ) };
        auto const shared { marked ? tint( Span.material, { 0.f, 0.f, 1.f } ) : Span.material };
        auto const ramp = [&]( std::vector<glm::dvec2> const &Knots ) -> std::vector<glm::dvec2> const & {
            return marked ? unramped : Knots; };
        auto const centre { knots::middle( lower, upper ) };
        auto const beneath { knots::shifted( lower, -tread_fade ) };
        auto const beyond { knots::shifted( upper, tread_fade ) };
        auto const lowfade { knots::bound( knots::shifted( lower, tread_fade ), centre, true ) };
        auto const highfade { knots::bound( knots::shifted( upper, -tread_fade ), centre, false ) };
        std::vector<double> splits { Start, End };
        std::vector<glm::dvec2> const lowline { { Start, low }, { End, low } };
        std::vector<glm::dvec2> const highline { { Start, high }, { End, high } };
        for( auto const *knots : { &lower, &upper, &lowfade, &highfade, &beneath, &beyond } ) {
            for( auto const *limit : { &lowline, &highline, &Extents, &Inner } ) {
                crossings( *knots, *limit, Start, End, 0.001, splits );
            }
        }
        crossings( knots::shifted( lower, 2.0 * tread_fade ), upper, Start, End, 0.001, splits );
        std::sort( splits.begin(), splits.end() );
        for( std::size_t i = 1; i < splits.size(); ++i ) {
            auto const a { splits[ i - 1 ] };
            auto const b { splits[ i ] };
            auto const middle { 0.5 * ( a + b ) };
            auto const from { knots::sample( lower, middle, 0.0 ) };
            auto const to { knots::sample( upper, middle, 0.0 ) };
            auto const inner { knots::sample( lowfade, middle, 0.0 ) };
            auto const outer { knots::sample( highfade, middle, 0.0 ) };
            auto const least { std::max( low, Inner.empty() ? low : knots::sample( Inner, middle, 0.0 ) ) };
            auto const most { std::min( high, Extents.empty() ? high : knots::sample( Extents, middle, 0.0 ) ) };
            auto const shown = [&]( double const Lowest, double const Highest ) {
                return Highest > Lowest + 1e-6 && Highest > least && Lowest < most; };
            if( shown( -1e9, from - tread_fade ) ) {
                current = null_handle;
                piece( a, b, Offsets, knots::bound( Extents, beneath, true ), false, false, top, true, Inner );
            }
            if( shown( from - tread_fade, from ) ) {
                current = owntint;
                piece( a, b, Offsets, knots::bound( Extents, lower, true ), false, false, top, false, knots::bound( Inner, beneath, false ), ramp( lower ), false, 0.f, -1.f, tread_fade );
            }
            if( shown( from, inner ) ) {
                current = sharedtint;
                piece( a, b, Offsets, knots::bound( Extents, lowfade, true ), false, false, top, false, knots::bound( Inner, lower, false ), ramp( lower ), false, 0.f, -1.f, -tread_fade );
            }
            if( shown( inner, outer ) ) {
                current = shared;
                piece( a, b, Offsets, knots::bound( Extents, highfade, true ), false, false, top, false == marked, knots::bound( Inner, lowfade, false ) );
            }
            if( shown( outer, to ) ) {
                current = sharedtint;
                piece( a, b, Offsets, knots::bound( Extents, upper, true ), false, false, top, false, knots::bound( Inner, highfade, false ), ramp( upper ), false, 0.f, -1.f, tread_fade );
            }
            if( shown( to, to + tread_fade ) ) {
                current = owntint;
                piece( a, b, Offsets, knots::bound( Extents, beyond, true ), false, false, top, false, knots::bound( Inner, upper, false ), ramp( upper ), false, 0.f, -1.f, -tread_fade );
            }
            if( shown( to + tread_fade, 1e9 ) ) {
                current = null_handle;
                piece( a, b, Offsets, Extents, false, false, top, true, knots::bound( Inner, beyond, false ) );
            }
        }
        current = null_handle;
    };
    for( auto const &guard : Layout.guards[ Path ][ Side > 0 ] ) {
        auto const flare { std::min( 0.3, 0.25 * ( guard.to - guard.from ) ) };
        std::vector<glm::dvec2> offsets;
        if( guard.flarefrom ) {
            offsets.insert( offsets.end(), { { guard.from, guardoffset - 0.02 }, { guard.from + flare, guardoffset } } );
        }
        else {
            offsets.push_back( { guard.from, guardoffset } );
        }
        if( guard.flareto ) {
            offsets.insert( offsets.end(), { { guard.to - flare, guardoffset }, { guard.to, guardoffset - 0.02 } } );
        }
        else {
            offsets.push_back( { guard.to, guardoffset } );
        }
        auto const top { flangeway + railhead + size.low - 0.002 };
        auto const mitered = [&]( glm::dvec2 const &Miter, double const Square, bool const Latest ) {
            if( Miter.y == 0.0 ) {
                return Square;
            }
            auto const inside { Miter.x + Miter.y * ( size.low + guardoffset ) };
            auto const outside { Miter.x + Miter.y * ( top + guardoffset ) };
            return Latest ? std::max( inside, outside ) : std::min( inside, outside ); };
        auto const envelope = [&]( std::vector<glm::dvec2> const &Lines, bool const Upper, double const A, double const B ) {
            std::vector<glm::dvec2> knots;
            if( Lines.empty() ) {
                return knots;
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
                knots.emplace_back( d, value );
            }
            return knots; };
        auto cuts { guard.plan.cuts };
        std::sort( cuts.begin(), cuts.end(), []( rail_cut const &A, rail_cut const &B ) { return A.at < B.at; } );
        auto start { guard.from };
        auto startmiter { guard.frommiter };
        rail_cut startcut;
        for( std::size_t i = 0; i <= cuts.size(); ++i ) {
            auto const end { i < cuts.size() ? cuts[ i ].end : guard.to };
            auto const endmiter { i < cuts.size() ? cuts[ i ].endmiter : guard.tomiter };
            auto const endcut { i < cuts.size() ? cuts[ i ] : rail_cut {} };
            auto const a { std::max( { mitered( startmiter, start, false ), From, 0.0 } ) };
            auto const b { std::min( { mitered( endmiter, end, true ), length, mitered( guard.tomiter, guard.to, true ) } ) };
            if( b - a > 0.01 ) {
                std::vector<glm::dvec2> uppers { { 0.0, top } };
                std::vector<glm::dvec2> lowers;
                for( auto const &[ miter, ending ] : { std::pair{ startmiter, false }, std::pair{ endmiter, true } } ) {
                    if( miter.y != 0.0 ) {
                        ( ending == ( miter.y < 0.0 ) ? uppers : lowers ).emplace_back( 1.0 / miter.y, -miter.x / miter.y - guardoffset );
                    }
                }
                auto const glued { a == start && false == startcut.startfit.empty() };
                auto const gluedend { b == end && false == endcut.endfit.empty() };
                bendfrom = { a, glued ? startcut.startbend : 0.0 };
                bendto = { b, gluedend ? endcut.endbend : 0.0 };
                fitfrom = { a, glued ? startcut.startfit : unfitted };
                fitto = { b, gluedend ? endcut.endfit : unfitted };
                piece( a, b, offsets, envelope( uppers, true, a, b ), false == glued, false == gluedend, Profile, false, envelope( lowers, false, a, b ) );
                bendfrom = bendto = glm::dvec2 { 0.0 };
                fitfrom = fitto = {};
            }
            if( i < cuts.size() && cuts[ i ].start > start ) {
                start = cuts[ i ].start;
                startmiter = cuts[ i ].startmiter;
                startcut = cuts[ i ];
            }
        }
    }
    auto const &shape { Drawing.checkshapes[ Side > 0 ? 0 : 1 ] };
    for( auto const &check : Layout.checks[ Path ][ Side > 0 ] ) {
        auto const replaced { std::any_of( Layout.wingends[ Path ][ Side > 0 ].begin(), Layout.wingends[ Path ][ Side > 0 ].end(), [&]( glm::dvec2 const &Wing ) {
            auto farend { 0.0 };
            return replaced_check( { check }, Wing.x, Wing.y, farend ); } ) };
        auto const from { check.from };
        auto const to { check.to };
        auto const a { std::max( from, From ) };
        auto const b { std::min( to, length ) };
        if( replaced || b - a <= 0.01 ) {
            continue;
        }
        auto const flare { std::min( 0.6, 0.4 * ( to - from ) ) };
        std::vector<glm::dvec2> offsets;
        if( check.flarefrom > 0.0 ) {
            offsets.insert( offsets.end(), { { from, -check.flarefrom }, { from + flare, 0.0 } } );
        }
        else {
            offsets.push_back( { from, 0.0 } );
        }
        if( check.flareto > 0.0 ) {
            offsets.insert( offsets.end(), { { to - flare, 0.0 }, { to, -check.flareto } } );
        }
        else {
            offsets.push_back( { to, 0.0 } );
        }
        bendfrom = { a, a == from ? check.frombend : 0.0 };
        bendto = { b, b == to ? check.tobend : 0.0 };
        piece( a, b, offsets, {}, a > 0.0 || from >= 0.0, to <= length, shape, false, {} );
        bendfrom = bendto = glm::dvec2 { 0.0 };
    }
    for( auto const &fill : Layout.fills[ Path ][ Side > 0 ] ) {
        auto const a { std::max( fill.from, From ) };
        auto const b { std::min( fill.to, length ) };
        if( b - a > 0.01 ) {
            current = null_handle;
            if( false == fill.web ) {
                aim( treads, 0.5 * ( a + b ) );
            }
            auto const &shapes { fill.web ? Drawing.fillers : Drawing.plates };
            piece( a, b, {}, fill.outer, a == fill.from && fill.capfrom, b == fill.to && fill.capto, shapes[ Side > 0 ? 0 : 1 ], true, fill.inner, {}, true, static_cast<float>( fill.lift ) );
            current = null_handle;
        }
    }
    auto const &bands { Layout.bands[ Path ][ Side > 0 ] };
    if( plan.cuts.empty() && bands.empty() && treads.empty() ) {
        return false;
    }
    auto const byposition = []( glm::dvec2 const &A, glm::dvec2 const &B ) { return A.x < B.x; };
    auto const reaches = [&]( gfx::vertex_array const &Shape ) {
        glm::dvec2 result { 1e9, -1e9 };
        for( auto const &point : Shape ) {
            auto const reach { static_cast<double>( Side * point.position.x - halfwidth ) };
            result = { std::min( result.x, reach ), std::max( result.y, reach ) };
        }
        return result; };
    auto const band_cut = [&]( rail_band const &Band, glm::dvec2 const &Reach ) {
        rail_cut cut;
        cut.at = Band.at;
        auto const before { Band.into > 0.0 ? Band.from : Band.to };
        auto const after { Band.into > 0.0 ? Band.to : Band.from };
        auto const meets = [&]( double const Edge, double const Reach ) {
            return glm::dvec2{ Band.at + ( Edge + Band.facing * Reach ) / Band.into, Reach }; };
        if( std::abs( Band.facing ) < 1e-3 ) {
            cut.end = meets( before, 0.0 ).x;
            cut.start = meets( after, 0.0 ).x;
            return cut;
        }
        std::vector<glm::dvec2> endknots { meets( before, Reach.x ), meets( before, Reach.y ) };
        std::vector<glm::dvec2> startknots { meets( after, Reach.x ), meets( after, Reach.y ) };
        std::sort( endknots.begin(), endknots.end(), byposition );
        std::sort( startknots.begin(), startknots.end(), byposition );
        cut.end = endknots.back().x;
        cut.start = startknots.front().x;
        cut.endcap = false;
        cut.startcap = false;
        auto const outwards { Band.facing / Band.into > 0.0 };
        ( outwards ? cut.endinner : cut.endextents ) = endknots;
        ( outwards ? cut.startextents : cut.startinner ) = startknots;
        return cut; };
    auto const lay = [&]( gfx::vertex_array const &Shape, std::vector<rail_cut> cuts, bool const Wheels, std::vector<rail_tread> const &Spans, float const Wallu = -1.f ) {
        std::sort( cuts.begin(), cuts.end(), []( rail_cut const &A, rail_cut const &B ) { return A.at < B.at; } );
        auto start { -1e9 };
        auto startcap { true };
        auto startbend { 0.0 };
        std::vector<glm::dvec2> startfit;
        std::vector<glm::dvec2> rolledfrom;
        std::vector<glm::dvec2> offsets, extents, inner;
        for( std::size_t i = 0; i <= cuts.size(); ++i ) {
            auto const end { i < cuts.size() ? cuts[ i ].end : 1e9 };
            auto pieceoffsets { offsets };
            auto pieceextents { extents };
            auto pieceinner { inner };
            if( i < cuts.size() ) {
                pieceoffsets.insert( pieceoffsets.end(), cuts[ i ].endoffsets.begin(), cuts[ i ].endoffsets.end() );
                pieceextents.insert( pieceextents.end(), cuts[ i ].endextents.begin(), cuts[ i ].endextents.end() );
                pieceinner.insert( pieceinner.end(), cuts[ i ].endinner.begin(), cuts[ i ].endinner.end() );
            }
            std::sort( pieceoffsets.begin(), pieceoffsets.end(), byposition );
            std::sort( pieceextents.begin(), pieceextents.end(), byposition );
            std::sort( pieceinner.begin(), pieceinner.end(), byposition );
            auto const a { std::max( start, From ) };
            auto const b { std::min( end, length ) };
            auto const rolled { i < cuts.size() ? knots::bound( rolledfrom, cuts[ i ].endrolled, true ) : rolledfrom };
            auto const capstart { start > From ? startcap : Openstart };
            auto const capend { end < length ? cuts[ i ].endcap : Openend };
            bendfrom = { a, start > From ? startbend : 0.0 };
            bendto = { b, end < length ? cuts[ i ].endbend : 0.0 };
            fitfrom = { a, start > From ? startfit : unfitted };
            fitto = { b, end < length ? cuts[ i ].endfit : unfitted };
            if( b - a > 0.01 ) {
                std::vector<rail_tread> active;
                for( auto const &span : Spans ) {
                    if( span.anchor == 0
                     || ( span.anchor > 0 && b > span.from + 0.01 && b < span.to + 0.01 )
                     || ( span.anchor < 0 && a > span.from - 0.01 && a < span.to - 0.01 ) ) {
                        active.push_back( span );
                    }
                }
                std::vector<double> splits;
                for( auto const &span : active ) {
                    for( auto const edge : { span.from, span.to } ) {
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
                    auto const first { k == 1 };
                    auto const last { k + 1 == splits.size() };
                    auto const *span { aim( active, 0.5 * ( splits[ k - 1 ] + splits[ k ] ) ) };
                    if( Wheels && span != nullptr && false == span->edge.empty() && topfirst < toplast ) {
                        treaded( splits[ k - 1 ], splits[ k ], pieceoffsets, pieceextents, pieceinner, first && capstart, last && capend, *span );
                    }
                    else if( false == Wheels || rolled.empty() || topfirst >= toplast ) {
                        piece( splits[ k - 1 ], splits[ k ], pieceoffsets, pieceextents, first && capstart, last && capend, Shape, true, pieceinner, {}, false == Wheels, 0.f, Wallu );
                    }
                    else {
                        running( splits[ k - 1 ], splits[ k ], pieceoffsets, pieceextents, pieceinner, rolled, first && capstart, last && capend );
                    }
                }
                current = null_handle;
            }
            bendfrom = bendto = glm::dvec2 { 0.0 };
            fitfrom = fitto = {};
            if( i < cuts.size() ) {
                rolledfrom = cuts[ i ].startrolled;
                start = cuts[ i ].start;
                startcap = cuts[ i ].startcap;
                startbend = cuts[ i ].startbend;
                startfit = cuts[ i ].startfit;
                offsets = cuts[ i ].startoffsets;
                extents = cuts[ i ].startextents;
                inner = cuts[ i ].startinner;
            }
        }
    };
    auto const &outline { track_rail_profile( m_profile1.second ) };
    auto const count { Profile.size() };
    if( false == size.grooved || outline.size() != 2 * count ) {
        auto cuts { plan.cuts };
        auto const reach { reaches( Profile ) };
        for( auto const &band : bands ) {
            cuts.push_back( band_cut( band, reach ) );
        }
        lay( Profile, cuts, true, treads );
        return true;
    }
    auto const height = [&]( std::size_t const Index ) {
        return static_cast<double>( outline[ Side > 0 ? Index : count - 1 - Index ].position.y ) - size.groove; };
    auto const above = [&]( std::size_t const Index ) {
        return height( Index ) > 1e-4; };
    auto const floorpoint = [&]( std::size_t const A, std::size_t const B ) {
        auto const t { static_cast<float>( height( A ) / ( height( A ) - height( B ) ) ) };
        auto point { Profile[ above( A ) ? A : B ] };
        point.position = glm::mix( Profile[ A ].position, Profile[ B ].position, t );
        point.texture = glm::mix( Profile[ A ].texture, Profile[ B ].texture, t );
        return point; };
    auto const flattened = [&]( std::size_t const Index ) {
        auto point { Profile[ Index ] };
        auto const &file { outline[ Side > 0 ? Index : count - 1 - Index ].position };
        if( file.y > size.crest - 0.0005 || ( file.x < -0.02 && file.y > size.liptop - 0.0005 ) ) {
            point.normal = { 0.f, 1.f, 0.f };
            point.texture.x = Drawing.platetexture;
        }
        return point; };
    std::vector<gfx::vertex_array> tops, flats;
    gfx::vertex_array body;
    std::vector<std::size_t> floors;
    auto floortexture { 0.f };
    auto floortextures { 0 };
    for( std::size_t i = 0; i < count; ++i ) {
        if( false == above( i ) ) {
            if( height( i ) > -1e-4 ) {
                floors.push_back( body.size() );
                floortexture += Profile[ i ].texture.x;
                ++floortextures;
            }
            body.push_back( Profile[ i ] );
            continue;
        }
        auto const first { i };
        while( i + 1 < count && above( i + 1 ) ) {
            ++i;
        }
        gfx::vertex_array top, flat;
        if( first > 0 ) {
            if( height( first - 1 ) > -1e-4 ) {
                top.push_back( Profile[ first - 1 ] );
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
            top.push_back( Profile[ k ] );
            flat.push_back( flattened( k ) );
        }
        if( i + 1 < count ) {
            if( height( i + 1 ) > -1e-4 ) {
                top.push_back( Profile[ i + 1 ] );
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
        tops.push_back( top );
        flats.push_back( flat );
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
    std::vector<rail_cut> wholecuts { plan.cuts };
    auto const bodyreach { reaches( body ) };
    for( auto const &band : bands ) {
        if( band.whole ) {
            wholecuts.push_back( band_cut( band, bodyreach ) );
        }
    }
    lay( body, wholecuts, false, {} );
    auto const wallu { floortextures > 0 ? floortexture / floortextures : -1.f };
    for( std::size_t k = 0; k < tops.size(); ++k ) {
        auto const reach { reaches( tops[ k ] ) };
        auto const head { reach.x + reach.y > 0.0 };
        auto edge { head ? 1e9 : -1e9 };
        for( auto const &point : tops[ k ] ) {
            if( point.position.y > ( head ? size.crest : size.liptop ) - 0.0005 ) {
                auto const across { static_cast<double>( Side * point.position.x - halfwidth ) };
                edge = ( head ? std::min( edge, across ) : std::max( edge, across ) );
            }
        }
        std::vector<rail_cut> cuts { plan.cuts };
        for( auto const &band : bands ) {
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
        std::vector<glm::dvec2> spans { Layout.blocks[ Path ][ Side > 0 ] };
        for( auto const &cut : cuts ) {
            for( auto &span : spans ) {
                if( cut.at > span.x && cut.at < span.y ) {
                    span = { std::min( span.x, cut.end ), std::max( span.y, cut.start ) };
                }
            }
        }
        std::sort( spans.begin(), spans.end(), byposition );
        std::vector<glm::dvec2> merged;
        for( auto const &span : spans ) {
            if( false == merged.empty() && span.x <= merged.back().y ) {
                merged.back().y = std::max( merged.back().y, span.y );
            }
            else {
                merged.push_back( span );
            }
        }
        if( merged.empty() ) {
            lay( tops[ k ], cuts, false, treads, wallu );
            continue;
        }
        std::vector<rail_tread> blocktreads;
        for( auto const &span : merged ) {
            for( auto const &tread : treads ) {
                if( tread.from < span.y && tread.to > span.x ) {
                    blocktreads.push_back( { span.x, span.y, tread.material } );
                    break;
                }
            }
        }
        auto const seam = []( double const At, double const End, double const Start ) {
            rail_cut cut;
            cut.at = At;
            cut.end = End;
            cut.start = Start;
            cut.endcap = false;
            cut.startcap = false;
            return cut; };
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
        lay( tops[ k ], outside, false, treads, wallu );
        lay( flats[ k ], inside, false, blocktreads, wallu );
    }
    return true;
}


/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/TrackBlend.h"
#include "world/Track.h"
#include "world/TrackSleepers.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"

#include <algorithm>
#include <limits>

namespace track_blend {

namespace {

// how far aside of the line of an end its weight reaches, past the widest trackbeds
double constexpr lateral_reach { 6.0 };

blending const *current { nullptr };

int end_count( TTrack const &Track ) {

    switch( Track.eType ) {
        case tt_Normal: { return 2; }
        case tt_Switch: { return ( Track.SwitchExtension != nullptr ? 3 : 0 ); }
        default: { return 0; }
    }
}

// the segment an end of the track is the start (end 0) or the end of: p1, p2 of the first path, p4 of the second of a switch
TSegment const *end_segment( TTrack const &Track, int const End ) {

    if( Track.eType == tt_Switch ) {
        return ( End == 2 ? Track.SwitchExtension->Segments[ 1 ].get() : Track.SwitchExtension->Segments[ 0 ].get() );
    }
    return Track.Segment.get();
}

bool end_frame( TTrack const &Track, int const End, glm::dvec3 &Point, glm::dvec3 &Inward ) {

    auto const *segment { end_segment( Track, End ) };
    if( segment == nullptr ) {
        return false;
    }
    auto const length { segment->GetLength() };
    if( length < 0.01 ) {
        return false;
    }
    auto const step { std::clamp( 0.25 / length, 0.0001, 0.5 ) };
    auto const atstart { End == 0 };
    Point = ( atstart ? segment->FastGetPoint_0() : segment->FastGetPoint_1() );
    auto const way { segment->FastGetPoint( atstart ? step : 1.0 - step ) - Point };
    if( glm::length( way ) < 1e-6 ) {
        return false;
    }
    Inward = glm::normalize( way );
    return true;
}

TTrack *neighbour( TTrack const &Track, int const End ) {

    if( Track.eType == tt_Switch ) {
        switch( End ) {
            case 0: { return Track.SwitchExtension->pPrevs[ 0 ]; }
            case 1: { return Track.SwitchExtension->pNexts[ 0 ]; }
            case 2: { return Track.SwitchExtension->pNexts[ 1 ]; }
            default: { return nullptr; }
        }
    }
    return ( End == 0 ? Track.trPrev : ( End == 1 ? Track.trNext : nullptr ) );
}

// the end of the track the other one is joined at, the nearest to the point of those joined to it; -1 where it isn't joined
int end_toward( TTrack const &Track, TTrack const *Other, glm::dvec3 const &Point ) {

    auto result { -1 };
    auto nearest { std::numeric_limits<double>::max() };
    for( int end = 0; end < end_count( Track ); ++end ) {
        auto joined { neighbour( Track, end ) == Other };
        if( Track.eType == tt_Switch && end == 0 ) {
            // the second path of a switch starts at p3, its p1 as well
            joined = joined || Track.SwitchExtension->pPrevs[ 1 ] == Other;
        }
        glm::dvec3 point, inward;
        if( false == joined || false == end_frame( Track, end, point, inward ) ) {
            continue;
        }
        auto const distance { glm::length( point - Point ) };
        if( distance < nearest ) {
            nearest = distance;
            result = end;
        }
    }
    return result;
}

// the trackbed of a switch, taken from the plain tracks joined to it where it has none of its own, as create_switch_trackbed() does
material_handle switch_trackbed( TTrack const &Switch ) {

    auto material { Switch.SwitchExtension->m_material3 };
    if( material == null_handle && Switch.trPrev != nullptr && Switch.trPrev->eType == tt_Normal ) {
        material = Switch.trPrev->m_material2;
    }
    if( material == null_handle && Switch.trNext != nullptr && Switch.trNext->eType == tt_Normal ) {
        material = Switch.trNext->m_material2;
    }
    return material;
}

// the material the track draws its rails or its ballast with at the end, null where it has none there
material_handle material_of( TTrack const &Track, kind const Kind, int const End ) {

    if( false == Track.m_visible || Track.m_editorremoved || ( Track.iCategoryFlag & 15 ) != 1 ) {
        return null_handle;
    }
    if( Kind == kind::rails ) {
        switch( Track.eType ) {
            case tt_Normal: { return Track.m_material1; }
            case tt_Switch: { return ( End == 2 ? Track.m_material2 : Track.m_material1 ); }
            default: { return null_handle; }
        }
    }
    // no ballast is drawn under the 3D sleepers laid on the ground alone
    if( false == Global.Sleepers3DBed && track_sleepers::lays( Track ) ) {
        return null_handle;
    }
    switch( Track.eType ) {
        case tt_Normal: { return Track.m_material2; }
        case tt_Switch: { return ( Global.CreateSwitchTrackbeds && Track.SwitchExtension != nullptr ? switch_trackbed( Track ) : null_handle ); }
        default: { return null_handle; }
    }
}

// whether the track blends its rails or ballast at the end into those of the track joined there, and what with. the answer is
// the same for that track back, but for the lengths and the kinds of tracks which blend: switches don't blend their rails
bool blends( TTrack const &Track, kind const Kind, int const End, TTrack const **Other = nullptr, int *Otherend = nullptr, material_handle *Othermaterial = nullptr ) {

    if( End < 0 || End >= end_count( Track ) ) {
        return false;
    }
    if( Kind == kind::rails && Track.eType != tt_Normal ) {
        return false;
    }
    if( Track.m_blend[ End ] <= 0.f ) {
        return false;
    }
    auto const own { material_of( Track, Kind, End ) };
    if( own == null_handle ) {
        return false;
    }
    auto const *other { neighbour( Track, End ) };
    glm::dvec3 point, inward;
    if( other == nullptr || other == &Track || false == end_frame( Track, End, point, inward ) ) {
        return false;
    }
    auto const otherend { end_toward( *other, &Track, point ) };
    if( otherend < 0 ) {
        return false;
    }
    auto const theirs { material_of( *other, Kind, otherend ) };
    if( theirs == null_handle || theirs == own ) {
        return false;
    }
    if( false == GfxRenderer->Blendable_Material( own ) || false == GfxRenderer->Blendable_Material( theirs ) ) {
        return false;
    }
    if( Other != nullptr ) { *Other = other; }
    if( Otherend != nullptr ) { *Otherend = otherend; }
    if( Othermaterial != nullptr ) { *Othermaterial = theirs; }
    return true;
}

} // namespace

void blending::weigh( gfx::vertex_array const &Vertices, gfx::userdata_array &Userdata ) const {

    Userdata.resize( Vertices.size(), gfx::vertex_userdata { glm::vec4 { 0.f } } );
    for( std::size_t index = 0; index < Vertices.size(); ++index ) {
        glm::dvec3 const position { Vertices[ index ].position };
        auto &data { Userdata[ index ].data };
        for( int end = 0; end < 3; ++end ) {
            auto const &blend { ends[ end ] };
            auto weight { 0.f };
            if( blend.active ) {
                // measured along the line of the end, so both tracks give the vertices of their joint the same weight
                auto const offset { position - blend.point };
                auto const along { glm::dot( offset, blend.inward ) };
                auto const aside { glm::length( offset - blend.inward * along ) };
                if( along < blend.length && aside < lateral_reach ) {
                    weight = blend.peak * static_cast<float>( 1.0 - std::max( 0.0, along ) / blend.length );
                }
            }
            data[ 1 + end ] = weight;
        }
    }
}

blending setup( TTrack const &Track, kind const Kind ) {

    blending result;
    std::array<material_handle, 3> others { null_handle, null_handle, null_handle };
    auto found { false };
    for( int end = 0; end < end_count( Track ); ++end ) {
        TTrack const *other { nullptr };
        auto otherend { -1 };
        auto theirs { null_handle };
        if( false == blends( Track, Kind, end, &other, &otherend, &theirs ) ) {
            continue;
        }
        auto &blend { result.ends[ end ] };
        end_frame( Track, end, blend.point, blend.inward );
        blend.point -= Track.m_origin;
        blend.length = Track.m_blend[ end ];
        // where both tracks blend they meet half way at the joint, otherwise this one goes all the way to the other one there
        blend.peak = ( blends( *other, Kind, otherend ) ? 0.5f : 1.f );
        blend.active = true;
        others[ end ] = theirs;
        found = true;
    }
    if( false == found ) {
        return {};
    }
    auto const own { material_of( Track, Kind, 0 ) };
    auto const material { GfxRenderer->Blend_Material( own, others, Kind == kind::ballast ) };
    if( material == own ) {
        // the renderer can't blend them
        return {};
    }
    result.material = material;
    return result;
}

weighing::weighing( blending const *Blending ) :
    m_previous { current } {

    current = ( Blending != nullptr && Blending->any() ? Blending : nullptr );
}

weighing::~weighing() {

    current = m_previous;
}

blending const *weighed() {

    return current;
}

} // namespace track_blend

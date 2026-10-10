/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/frustum.h"

void
cFrustum::calculate() {

    auto const &projection = OpenGLMatrices.data( GL_PROJECTION );
    auto const &modelview = OpenGLMatrices.data( GL_MODELVIEW );

    calculate( projection, modelview );
}

void
cFrustum::calculate( glm::mat4 const &Projection, glm::mat4 const &Modelview ) {

    // multiply the matrices to retrieve clipping planes (column-major, same layout as the old float[16] form)
    auto const clipmatrix { Projection * Modelview };
    float clip[ 16 ];
    for( int column = 0; column < 4; ++column ) {
        for( int row = 0; row < 4; ++row ) {
            clip[ column * 4 + row ] = clipmatrix[ column ][ row ];
        }
    }

    // get the sides of the frustum.
    m_frustum[ side_RIGHT ][ plane_A ] = clip[ 3 ] - clip[ 0 ];
    m_frustum[ side_RIGHT ][ plane_B ] = clip[ 7 ] - clip[ 4 ];
    m_frustum[ side_RIGHT ][ plane_C ] = clip[ 11 ] - clip[ 8 ];
    m_frustum[ side_RIGHT ][ plane_D ] = clip[ 15 ] - clip[ 12 ];
    normalize_plane( side_RIGHT );

    m_frustum[ side_LEFT ][ plane_A ] = clip[ 3 ] + clip[ 0 ];
    m_frustum[ side_LEFT ][ plane_B ] = clip[ 7 ] + clip[ 4 ];
    m_frustum[ side_LEFT ][ plane_C ] = clip[ 11 ] + clip[ 8 ];
    m_frustum[ side_LEFT ][ plane_D ] = clip[ 15 ] + clip[ 12 ];
    normalize_plane( side_LEFT );

    m_frustum[ side_BOTTOM ][ plane_A ] = clip[ 3 ] + clip[ 1 ];
    m_frustum[ side_BOTTOM ][ plane_B ] = clip[ 7 ] + clip[ 5 ];
    m_frustum[ side_BOTTOM ][ plane_C ] = clip[ 11 ] + clip[ 9 ];
    m_frustum[ side_BOTTOM ][ plane_D ] = clip[ 15 ] + clip[ 13 ];
    normalize_plane( side_BOTTOM );

    m_frustum[ side_TOP ][ plane_A ] = clip[ 3 ] - clip[ 1 ];
    m_frustum[ side_TOP ][ plane_B ] = clip[ 7 ] - clip[ 5 ];
    m_frustum[ side_TOP ][ plane_C ] = clip[ 11 ] - clip[ 9 ];
    m_frustum[ side_TOP ][ plane_D ] = clip[ 15 ] - clip[ 13 ];
    normalize_plane( side_TOP );

    m_frustum[ side_BACK ][ plane_A ] = clip[ 3 ] - clip[ 2 ];
    m_frustum[ side_BACK ][ plane_B ] = clip[ 7 ] - clip[ 6 ];
    m_frustum[ side_BACK ][ plane_C ] = clip[ 11 ] - clip[ 10 ];
    m_frustum[ side_BACK ][ plane_D ] = clip[ 15 ] - clip[ 14 ];
    normalize_plane( side_BACK );

    m_frustum[ side_FRONT ][ plane_A ] = clip[ 3 ] + clip[ 2 ];
    m_frustum[ side_FRONT ][ plane_B ] = clip[ 7 ] + clip[ 6 ];
    m_frustum[ side_FRONT ][ plane_C ] = clip[ 11 ] + clip[ 10 ];
    m_frustum[ side_FRONT ][ plane_D ] = clip[ 15 ] + clip[ 14 ];
    normalize_plane( side_FRONT );
}

bool
cFrustum::point_inside( float const X, float const Y, float const Z ) const {

    // cycle through the sides of the frustum, checking if the point is behind them
    return std::ranges::all_of( m_frustum, [&]( auto const &frustumplane ) {
        return frustumplane[ plane_A ] * X
                + frustumplane[ plane_B ] * Y
                + frustumplane[ plane_C ] * Z
                + frustumplane[ plane_D ] > 0; } );
}

float
cFrustum::sphere_inside( float const X, float const Y, float const Z, float const Radius ) const {

    float distance;
    // go through all the sides of the frustum. bail out as soon as possible
    for (auto &frustumplane : m_frustum) {
        distance =
            frustumplane[ plane_A ] * X
            + frustumplane[ plane_B ] * Y
            + frustumplane[ plane_C ] * Z
            + frustumplane[ plane_D ];
        if( distance <= -Radius )
            return 0.0f;
    }
    return distance + Radius;
}

bool
cFrustum::cube_inside( float const X, float const Y, float const Z, float const Size ) const {

    // the cube is inside unless all of its corners lie behind one of the frustum planes
    return std::ranges::all_of( m_frustum, [&]( auto const &frustumplane ) {
        if( frustumplane[ plane_A ] * ( X - Size )
            + frustumplane[ plane_B ] * ( Y - Size )
            + frustumplane[ plane_C ] * ( Z - Size )
            + frustumplane[ plane_D ] > 0 )
            return true;
        if( frustumplane[ plane_A ] * ( X + Size )
            + frustumplane[ plane_B ] * ( Y - Size )
            + frustumplane[ plane_C ] * ( Z - Size )
            + frustumplane[ plane_D ] > 0 )
            return true;
        if( frustumplane[ plane_A ] * ( X - Size )
            + frustumplane[ plane_B ] * ( Y + Size )
            + frustumplane[ plane_C ] * ( Z - Size )
            + frustumplane[ plane_D ] > 0 )
            return true;
        if( frustumplane[ plane_A ] * ( X + Size )
            + frustumplane[ plane_B ] * ( Y + Size )
            + frustumplane[ plane_C ] * ( Z - Size )
            + frustumplane[ plane_D ] > 0 )
            return true;
        if( frustumplane[ plane_A ] * ( X - Size )
            + frustumplane[ plane_B ] * ( Y - Size )
            + frustumplane[ plane_C ] * ( Z + Size )
            + frustumplane[ plane_D ] > 0 )
            return true;
        if( frustumplane[ plane_A ] * ( X + Size )
            + frustumplane[ plane_B ] * ( Y - Size )
            + frustumplane[ plane_C ] * ( Z + Size )
            + frustumplane[ plane_D ] > 0 )
            return true;
        if( frustumplane[ plane_A ] * ( X - Size )
            + frustumplane[ plane_B ] * ( Y + Size )
            + frustumplane[ plane_C ] * ( Z + Size )
            + frustumplane[ plane_D ] > 0 )
            return true;
        if( frustumplane[ plane_A ] * ( X + Size )
            + frustumplane[ plane_B ] * ( Y + Size )
            + frustumplane[ plane_C ] * ( Z + Size )
            + frustumplane[ plane_D ] > 0 )
            return true;

        return false; } );
}

void cFrustum::normalize_plane( cFrustum::side const Side ) {

    float magnitude =
        std::sqrt(
            m_frustum[ Side ][ plane_A ] * m_frustum[ Side ][ plane_A ]
            + m_frustum[ Side ][ plane_B ] * m_frustum[ Side ][ plane_B ]
            + m_frustum[ Side ][ plane_C ] * m_frustum[ Side ][ plane_C ] );

    m_frustum[ Side ][ plane_A ] /= magnitude;
    m_frustum[ Side ][ plane_B ] /= magnitude;
    m_frustum[ Side ][ plane_C ] /= magnitude;
    m_frustum[ Side ][ plane_D ] /= magnitude;
}

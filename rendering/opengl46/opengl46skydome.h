/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "gl/vao.h"
#include "gl/shader.h"
#include "gl/buffer.h"

class opengl46_skydome {

public:
// constructors
    opengl46_skydome() = default;
// destructor
    ~opengl46_skydome() = default;
// methods
    // updates data stores on the opengl end. NOTE: unbinds buffers
    void update();
    // draws the skydome. with Atmosphere the colours come from the tables of the physical atmosphere (sky_atmosphere.frag),
    // which the caller binds along with the parameters of the shader
    void render( bool const Atmosphere = false );

private:
// members
	std::optional<gl::buffer> m_vertexbuffer;
	std::optional<gl::buffer> m_indexbuffer;
	std::optional<gl::buffer> m_coloursbuffer;
	std::optional<gl::program> m_shader;
	std::optional<gl::program> m_atmosphereshader;
	std::optional<gl::vao> m_vao;
};

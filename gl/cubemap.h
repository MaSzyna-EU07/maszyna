#pragma once

#include "object.h"

namespace gl
{
    // cubemap texture rendertarget
    // todo: integrate with texture system
    class cubemap : public object
    {
    public:
        cubemap();
        ~cubemap();

        void alloc(GLint format, int width, int height, GLenum components, GLenum type) const;
        // immutable storage of square faces with the given number of mipmap levels, filtered between the levels.
        // the levels are meant to be written by compute shaders (glBindImageTexture), so format has to be an image format
        void alloc_levels(GLenum format, int size, int levels) const;
        void bind(int unit) const;
        void generate_mipmaps() const;
    };
}

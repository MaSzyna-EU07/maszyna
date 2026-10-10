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
        void bind(int unit) const;
        void generate_mipmaps() const;
    };
}

#pragma once

#include <glad/glad.h>

namespace gl
{
    class object
    {
    private:
        GLuint id = 0;

    public:
        inline operator GLuint() const
        {
            return id;
        }

        inline operator GLuint*()
        {
            return &id;
        }

        inline operator const GLuint*() const
        {
            return &id;
        }

        object() = default;
        object(const object&) = delete;
        object& operator=(const object&) = delete;
    };
}

module;
#include "glad/glad.h"
#include <cstdint>
#include <cstddef>

module eu07.gl.ubo;

gl::ubo::ubo(size_t size, int idx, GLenum hint)
{
    allocate(buffer::UNIFORM_BUFFER, size, hint);
    index = idx;
    bind_uniform();
}

void gl::ubo::bind_uniform()
{
    bind_base(buffer::UNIFORM_BUFFER, index);
}

void gl::ubo::update(const uint8_t *data, int offset, GLsizeiptr size)
{
    upload(buffer::UNIFORM_BUFFER, data, offset, size);
}

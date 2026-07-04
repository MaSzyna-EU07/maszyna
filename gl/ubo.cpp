#include "stdafx.h"
#include "ubo.h"

gl::ubo::ubo(size_t size, int idx, GLenum hint)
{
    allocate(UNIFORM_BUFFER, size, hint);
    index = idx;
    bind_uniform();
}

void gl::ubo::bind_uniform()
{
    bind_base(UNIFORM_BUFFER, index);
}

void gl::ubo::update(const uint8_t *data, int offset, GLsizeiptr size)
{
    upload(UNIFORM_BUFFER, data, offset, size);
}

#include "stdafx.h"
#include "pbo.h"

void gl::pbo::request_read(const int x, const int y, const int lx, const int ly, const int pixsize, const GLenum format, const GLenum type)
{
	const int s = lx * ly * pixsize;
    if (s != size)
        allocate(PIXEL_PACK_BUFFER, s, GL_STREAM_DRAW);
    size = s;

    data_ready = false;
    sync.reset();

    bind(PIXEL_PACK_BUFFER);
    glReadPixels(x, y, lx, ly, format, type, 0);
    unbind(PIXEL_PACK_BUFFER);

    sync.emplace();
}

bool gl::pbo::read_data(const int lx, const int ly, void *data, const int pixsize)
{
    is_busy();

    if (!data_ready)
        return false;

	const int s = lx * ly * pixsize;
    if (s != size)
        return false;

    download(PIXEL_PACK_BUFFER, data, 0, s);
    unbind(PIXEL_PACK_BUFFER);
    data_ready = false;

    return true;
}

bool gl::pbo::is_busy()
{
    if (!sync)
        return false;

    if (sync->is_signalled())
    {
        data_ready = true;
        sync.reset();
        return false;
    }

    return true;
}

void* gl::pbo::map(const GLuint mode, const targets target)
{
	bind(target);
	return glMapBuffer(glenum_target(target), mode);
}

void gl::pbo::unmap(const targets target)
{
	bind(target);
	glUnmapBuffer(glenum_target(target));
	sync.emplace();
}

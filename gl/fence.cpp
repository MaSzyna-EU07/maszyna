#include "stdafx.h"
#include "fence.h"

gl::fence::fence()
{
    sync = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
}

gl::fence::~fence()
{
    glDeleteSync(sync);
}

bool gl::fence::is_signalled()
{
	// glClientWaitSync is faster than glGetSynciv. glGetSynciv probably tries to synchronize cpu and gpu
	// https://stackoverflow.com/questions/34601376/which-to-use-for-opengl-client-side-waiting-glgetsynciv-vs-glclientwaitsync
	GLenum r = glClientWaitSync(sync, GL_SYNC_FLUSH_COMMANDS_BIT, 1);
	return r == GL_ALREADY_SIGNALED || r == GL_CONDITION_SATISFIED;
}

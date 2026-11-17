// stand-ins for the parts of the engine which TModel3d (model/Model3d.cpp) links against, but which the tests of
// loading and saving models (test_e3d1_model.cpp) don't reach: the legacy opengl drawing and the occlusion queries

#include "stdafx.h"
#include "gl/query.h"
#include "utilities/Logs.h"

// the matrix of the legacy renderer is loaded straight from the sub-model, never in the tests
PFNGLLOADMATRIXFPROC glad_glLoadMatrixf{nullptr};

gl::query::~query() = default;

void Error(const std::string &asMessage, bool /*box*/)
{
	ErrorLog(asMessage);
}

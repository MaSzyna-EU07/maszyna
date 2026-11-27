// stand-ins for the parts of the engine which TModel3d (model/Model3d.cpp) links against, but which the tests of
// loading and saving models (test_e3d1_model.cpp) don't reach: the occlusion queries of the renderer and the error box

#include "stdafx.h"
#include "gl/query.h"
#include "utilities/Logs.h"

gl::query::~query() = default;

void Error(const std::string &asMessage, bool /*box*/)
{
	ErrorLog(asMessage);
}

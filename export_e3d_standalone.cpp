#include "stdafx.h"

#include "model/Model3d.h"
#include "utilities/Globals.h"
#include "rendering/renderer.h"

void export_e3d_standalone(std::string in, std::string out, int flags, bool dynamic)
{
    Global.iConvertModels = flags;
    Global.iWriteLogEnabled = 2;
    Global.ParserLogIncludes = true;
    GfxRenderer = gfx_renderer_factory::get_instance()->create("null");
    TModel3d model;
    model.LoadFromTextFile(in, dynamic);
    model.Init();
    model.SaveToBinFile(out);
}
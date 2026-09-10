#include "utilities/Globals_macros.h"
#include <string>
#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
import eu07.simcore;
import eu07.utilities.globals;
import eu07.rendering.renderer;


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
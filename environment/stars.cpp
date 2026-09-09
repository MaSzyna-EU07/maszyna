module;
#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include <ostream>
#include <string>
#include "utilities/Globals_macros.h"

module eu07.environment.stars;
import eu07.utilities.globals;
import eu07.model.mdlmngr;

//////////////////////////////////////////////////////////////////////////////////////////
// cStars -- simple starfield model, simulating appearance of starry sky

void
cStars::init() {

    m_stars = TModelsManager::GetModel( "skydome_stars.t3d", false );
}

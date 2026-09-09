/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;

// Everything glm reaches for from the standard library and the intrinsics headers
// is included here, in the global module fragment. Their include guards then turn
// glm's own #includes into no-ops, so only glm's declarations land in the purview
// below -- otherwise the export block would try to export system functions that
// have internal linkage, which is ill-formed.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cfloat>
#include <climits>
#include <limits>
#include <type_traits>
#include <iterator>
#include <algorithm>
#include <cassert>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <immintrin.h>

export module eu07.glm;

// glm is header-only. Including it inside the module purview attaches its
// declarations to this module, which is what lets them be exported. Every other
// module imports this one instead of including glm directly: GCC 16 rejects the
// merge when the same glm declarations arrive from several different modules
// ("conflicting imported declaration" on glm::detail::storage), so glm has to be
// compiled into exactly one module.
// Clang warns that including inside the purview attaches the declarations to
// this module. That is exactly the point here, so silence it for this block.
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Winclude-angled-in-module-purview"
#endif
export {
#include <glm/glm.hpp>
#include <glm/fwd.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/packing.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/matrix_transform_2d.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>
// gtx/string_cast is deliberately absent: it defines static label constants,
// and an export block may not export entities with internal linkage. The two
// call sites of glm::to_string include it in their own module fragment.
#include <glm/gtx/transform.hpp>
}
#ifdef __clang__
#pragma clang diagnostic pop
#endif

export {
// 12-byte vec3 that stays packed regardless of GLM_FORCE_DEFAULT_ALIGNED_GENTYPES.
// Use in structs that must match an exact binary layout (UBOs, vertex data,
// serialized formats).
using packed_vec3 = glm::vec<3, float, glm::packed_highp>;
}

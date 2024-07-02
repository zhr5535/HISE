#pragma once

#include <type_traits>
#include "_vec2_.h"
#include "_vec3_.h"

namespace deflow {
namespace internal {
template<typename _Ty, uint8_t _N, typename = std::enable_if_t<_N == 2 || _N == 3>>
using Vec_ = std::conditional_t<_N == 2, Vec2_<_Ty>, std::conditional_t<_N == 3, Vec3_<_Ty>, void>>;
}
}
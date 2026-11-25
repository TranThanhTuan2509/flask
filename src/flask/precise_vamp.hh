#pragma once

// Include before VAMP geometry/robot headers in each FLASK translation unit.
// Keep upstream source intact. Its x*rsqrt(x) implementation can underestimate
// the robot's radial extent, making a broad-phase break skip a true collision.
// This module-local specialization uses rounded AVX square root instead.
#include <vamp/vector.hh>

#if defined(__x86_64__)
namespace vamp
{
template <> inline auto SIMDVector<__m256>::sqrt<0>(__m256 value) noexcept -> __m256
{
    return _mm256_sqrt_ps(value);
}
} // namespace vamp
#endif

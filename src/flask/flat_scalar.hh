#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vamp/collision/environment.hh>

namespace flask::flat
{
struct ScalarMask
{
    bool value;
    bool any() const
    {
        return value;
    }
    bool all() const
    {
        return value;
    }
    friend ScalarMask operator|(ScalarMask a, ScalarMask b)
    {
        return {a.value || b.value};
    }
    friend ScalarMask operator&(ScalarMask a, ScalarMask b)
    {
        return {a.value && b.value};
    }
};

// One float per time sample. The implementation is instantiated in a separate
// translation unit compiled with both loop and SLP auto-vectorization disabled.
struct ScalarFloat
{
    static constexpr size_t num_scalars = 1;
    float value = 0.f;
    ScalarFloat() = default;
    ScalarFloat(float x) : value(x) {}
    ScalarFloat(const std::array<float, 1> &x) : value(x[0]) {}
    float element(size_t) const
    {
        return value;
    }
    ScalarFloat operator-() const
    {
        return {-value};
    }
    friend ScalarFloat operator+(ScalarFloat a, ScalarFloat b)
    {
        return {a.value + b.value};
    }
    friend ScalarFloat operator-(ScalarFloat a, ScalarFloat b)
    {
        return {a.value - b.value};
    }
    friend ScalarFloat operator*(ScalarFloat a, ScalarFloat b)
    {
        return {a.value * b.value};
    }
    friend ScalarFloat operator/(ScalarFloat a, ScalarFloat b)
    {
        return {a.value / b.value};
    }
    friend ScalarMask operator<(ScalarFloat a, ScalarFloat b)
    {
        return {a.value < b.value};
    }
    friend ScalarMask operator>(ScalarFloat a, ScalarFloat b)
    {
        return {a.value > b.value};
    }
    friend ScalarMask operator<=(ScalarFloat a, ScalarFloat b)
    {
        return {a.value <= b.value};
    }
    friend ScalarMask operator>=(ScalarFloat a, ScalarFloat b)
    {
        return {a.value >= b.value};
    }
    ScalarFloat sin() const
    {
        // Same range reduction, coefficients, and nearest-integer conversion
        // as the pinned VAMP AVX sine; do not compare a scalar libm shortcut
        // with a different vector polynomial in the SIMD attribution study.
        float x = std::abs(value), scaled = x * 1.27323954473516f;
        if (!std::isfinite(scaled) || scaled >= float(std::numeric_limits<int>::max()))
            return {std::numeric_limits<float>::quiet_NaN()};
        int quadrant = (int(std::nearbyint(scaled)) + 1) & ~1;
        float y = float(quadrant);
        x = x + y * (-.78515625f);
        x = x + y * (-2.4187564849853515625e-4f);
        x = x + y * (-3.77489497744594108e-8f);
        float z = x * x;
        float cosine = 2.443315711809948e-5f;
        cosine = cosine * z - 1.388731625493765e-3f;
        cosine = cosine * z + 4.166664568298827e-2f;
        cosine = cosine * z;
        cosine = cosine * z;
        cosine = cosine - z * .5f;
        cosine = cosine + 1.f;
        float sine = -1.9515295891e-4f;
        sine = sine * z + 8.3321608736e-3f;
        sine = sine * z - 1.6666654611e-1f;
        sine = sine * z;
        sine = sine * x;
        sine = sine + x;
        float result = (quadrant & 2) == 0 ? sine : cosine;
        if (std::signbit(value) != bool(quadrant & 4))
            result = -result;
        return {result};
    }
    ScalarFloat cos() const
    {
        constexpr float pi = 3.14159265359f;
        float shifted = value + pi / 2.f;
        if (shifted >= pi)
            shifted -= 2.f * pi;
        return ScalarFloat(shifted).sin();
    }
};

inline ScalarFloat precise_sqrt(const ScalarFloat &value)
{
    return {std::sqrt(value.value)};
}

// Geometry follows the same float sphere/capsule/OBB equations and sorted
// radial broad phase as the SIMD checker. Touching surfaces use its <0 policy.
inline bool scalar_environment_collision(const vamp::collision::Environment<float> &environment,
                                         float x, float y, float z, float radius)
{
    float extent = std::sqrt(x * x + y * y + z * z) + radius;
    for (const auto &s : environment.spheres)
    {
        if (s.min_distance >= extent)
            break;
        float dx = x - s.x, dy = y - s.y, dz = z - s.z, sum = radius + s.r;
        if (dx * dx + dy * dy + dz * dz - sum * sum < 0.f)
            return true;
    }
    auto capsules = [&](const auto &shapes)
    {
        for (const auto &shape : shapes)
        {
            if (shape.min_distance >= extent)
                break;
            float projection = ((x - shape.x1) * shape.xv + (y - shape.y1) * shape.yv +
                                (z - shape.z1) * shape.zv) *
                               shape.rdv;
            projection = std::clamp(projection, 0.f, 1.f);
            float dx = x - (shape.x1 + shape.xv * projection),
                  dy = y - (shape.y1 + shape.yv * projection),
                  dz = z - (shape.z1 + shape.zv * projection), sum = radius + shape.r;
            if (dx * dx + dy * dy + dz * dz - sum * sum < 0.f)
                return true;
        }
        return false;
    };
    if (capsules(environment.capsules) || capsules(environment.z_aligned_capsules))
        return true;
    auto cuboids = [&](const auto &shapes)
    {
        for (const auto &shape : shapes)
        {
            if (shape.min_distance >= extent)
                break;
            float dx = x - shape.x, dy = y - shape.y, dz = z - shape.z;
            float a = std::max(0.f, std::abs(shape.axis_1_x * dx + shape.axis_1_y * dy +
                                             shape.axis_1_z * dz) -
                                        shape.axis_1_r),
                  b = std::max(0.f, std::abs(shape.axis_2_x * dx + shape.axis_2_y * dy +
                                             shape.axis_2_z * dz) -
                                        shape.axis_2_r),
                  c = std::max(0.f, std::abs(shape.axis_3_x * dx + shape.axis_3_y * dy +
                                             shape.axis_3_z * dz) -
                                        shape.axis_3_r);
            if (a * a + b * b + c * c - radius * radius < 0.f)
                return true;
        }
        return false;
    };
    return cuboids(environment.cuboids) || cuboids(environment.z_aligned_cuboids);
}
} // namespace flask::flat

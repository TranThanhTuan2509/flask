#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace flask::polynomial
{
inline double evaluate(const std::vector<double> &coefficients, double x)
{
    double result = 0.;
    for (auto i = coefficients.rbegin(); i != coefficients.rend(); ++i)
        result = result * x + *i;
    return result;
}
inline double evaluation_scale(const std::vector<double> &coefficients, double x)
{
    double result = 0.;
    for (auto i = coefficients.rbegin(); i != coefficients.rend(); ++i)
        result = result * std::abs(x) + std::abs(*i);
    return result;
}

// Isolate real roots with derivative critical points. All stopping and root
// merging tolerances are relative to the roots, never to max(1, |root|).
// Tiny near-drift optima can coexist with O(1) stationary durations.
inline std::vector<double> roots_between(std::vector<double> p, double lower, double upper)
{
    double scale = 0.;
    for (double value : p)
    {
        if (!std::isfinite(value))
            return {};
        scale = std::max(scale, std::abs(value));
    }
    if (scale == 0. || !(lower < upper))
        return {};
    for (double &value : p)
        value /= scale;
    while (p.size() > 1 && p.back() == 0.)
        p.pop_back();
    if (p.size() < 2)
        return {};
    if (p.size() == 2)
    {
        double root = -p[0] / p[1];
        return root > lower && root < upper ? std::vector<double>{root} : std::vector<double>{};
    }
    std::vector<double> derivative(p.size() - 1);
    for (size_t i = 1; i < p.size(); ++i)
        derivative[i - 1] = i * p[i];
    auto critical = roots_between(derivative, lower, upper);
    std::vector<double> cuts{lower};
    cuts.insert(cuts.end(), critical.begin(), critical.end());
    cuts.push_back(upper);
    std::vector<double> roots;
    constexpr double tolerance = 64. * std::numeric_limits<double>::epsilon();
    for (double point : critical)
        if (std::abs(evaluate(p, point)) <= tolerance * evaluation_scale(p, point))
            roots.push_back(point);
    for (size_t interval = 1; interval < cuts.size(); ++interval)
    {
        double a = cuts[interval - 1], b = cuts[interval], fa = evaluate(p, a), fb = evaluate(p, b);
        if (fa == 0. || fb == 0. || (fa < 0.) == (fb < 0.))
            continue;
        double x = a + (b - a) * .5;
        for (int iteration = 0; iteration < 256; ++iteration)
        {
            double value = evaluate(p, x);
            if (value == 0.)
                break;
            if ((value < 0.) == (fa < 0.))
            {
                a = x;
                fa = value;
            }
            else
                b = x;
            if (std::nextafter(a, b) >= b ||
                b - a <= tolerance * std::max(std::abs(a), std::abs(b)))
            {
                x = a + (b - a) * .5;
                break;
            }
            double slope = evaluate(derivative, x);
            double next =
                slope != 0. ? x - value / slope : std::numeric_limits<double>::quiet_NaN();
            if (!std::isfinite(next) || !(next > a && next < b) || iteration % 8 == 7)
                next = a + (b - a) * .5;
            if (next == x)
                break;
            x = next;
        }
        roots.push_back(x);
    }
    std::sort(roots.begin(), roots.end());
    roots.erase(
        std::unique(roots.begin(), roots.end(), [&](double a, double b)
                    { return std::abs(a - b) <= tolerance * std::max(std::abs(a), std::abs(b)); }),
        roots.end());
    return roots;
}
} // namespace flask::polynomial

// This translation unit is compiled with automatic vectorization disabled.
#include "flask/flat_native.hh"

namespace flask::flat
{
bool scalar_valid_2_2(const Edge<2, 2> &edge, int robot, const World &world, const Settings &cfg,
                      Counters &counter)
{
    return validation_kernel<ScalarFloat, 2, 2>(edge, robot, world, cfg, counter);
}
bool scalar_valid_4_2(const Edge<4, 2> &edge, int robot, const World &world, const Settings &cfg,
                      Counters &counter)
{
    return validation_kernel<ScalarFloat, 4, 2>(edge, robot, world, cfg, counter);
}
bool scalar_valid_4_4(const Edge<4, 4> &edge, int robot, const World &world, const Settings &cfg,
                      Counters &counter)
{
    return validation_kernel<ScalarFloat, 4, 4>(edge, robot, world, cfg, counter);
}
bool scalar_valid_2_4(const Edge<2, 4> &edge, int robot, const World &world, const Settings &cfg,
                      Counters &counter)
{
    return validation_kernel<ScalarFloat, 2, 4>(edge, robot, world, cfg, counter);
}
} // namespace flask::flat

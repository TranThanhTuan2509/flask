"""Analytical scales for constant-control propagation in the flat metric.

These calculations diagnose reconstructed search settings. They do not infer
author parameters or certify that a propagated state satisfies robot limits.
"""
from dataclasses import dataclass
from math import factorial, isfinite, sqrt
from numbers import Integral


@dataclass(frozen=True)
class RestPropagationScale:
    maximum: float
    rms: float


def rest_propagation_scale(order, control_bounds, duration_min, duration_max,
                           derivative_metric_weight=.2):
    """Compute flat-metric displacement from a zero-derivative root (Eq. 29).

    Controls are independent uniform samples in [-bound_j, bound_j]; time is
    uniform on the supplied interval, independent of controls. Level zero has
    weight one; derivative level l has weight w / 3**(l-1), matching the native
    Euclidean metric. The maximum is over the entire closed proposal box. The
    RMS is unconditional, before any physical-limit or collision rejection.
    Position is arbitrary; every derivative stored in the initial state must
    be zero. Heading-aware and compound-norm metrics are outside this scope.
    """
    bounds=tuple(float(value) for value in control_bounds)
    if isinstance(order,bool) or not isinstance(order,Integral) or order<1:
        raise ValueError('order must be a positive integer')
    if not bounds or any(not isfinite(value) or value<=0 for value in bounds):
        raise ValueError('control_bounds must be a nonempty finite positive vector')
    if (not isfinite(duration_min) or not isfinite(duration_max)
            or not 0<=duration_min<=duration_max or duration_max<=0):
        raise ValueError('A finite positive ordered duration interval is required')
    if not isfinite(derivative_metric_weight) or derivative_metric_weight<=0:
        raise ValueError('derivative_metric_weight must be finite and positive')
    maximum_factor=mean_factor=0.
    for level in range(order):
        power=2*(order-level)
        weight=1. if level==0 else derivative_metric_weight/3**(level-1)
        denominator=factorial(order-level)**2
        # (hi**(p+1)-lo**(p+1))/((p+1)*(hi-lo)), without cancellation
        # for a narrow interval. Also valid when lo == hi.
        moment=sum(duration_max**(power-i)*duration_min**i
                   for i in range(power+1))/(power+1)
        maximum_factor+=weight*duration_max**power/denominator
        mean_factor+=weight*moment/denominator
    squared_bounds=sum(value*value for value in bounds)
    maximum=sqrt(squared_bounds*maximum_factor)
    rms=sqrt(squared_bounds*mean_factor/3.)
    if not isfinite(maximum) or not isfinite(rms):
        raise ValueError('Propagation scale exceeds the supported floating-point range')
    return RestPropagationScale(maximum,rms)

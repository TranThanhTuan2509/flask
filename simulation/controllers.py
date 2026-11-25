"""Pure mathematical controller components; no simulator or renderer import."""
import numpy as np


def quadrotor_tracking_torque(rotation,reference_rotation,omega,reference_omega,
                              reference_omega_dot,inertia,attitude_error,
                              attitude_gain=4.,rate_gain=1.4):
    """Transport reference rates and apply rigid-body inverse dynamics once.

    Angular quantities use body coordinates. Reference rates/accelerations are
    expressed in the reference body frame; their transport derivative includes
    the current-frame rotation term. Actuator saturation is handled separately.
    """
    transport=rotation.T@reference_rotation
    desired_rate=transport@reference_omega
    desired_rate_dot=transport@reference_omega_dot-np.cross(omega,desired_rate)
    return (inertia@desired_rate_dot+np.cross(omega,inertia@omega)
            -attitude_gain*attitude_error-rate_gain*(omega-desired_rate))

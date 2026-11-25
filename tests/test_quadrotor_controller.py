"""Rigid-body controller consistency without running MuJoCo or a showcase."""
import numpy as np
from scipy.spatial.transform import Rotation
from simulation.controllers import quadrotor_tracking_torque


def test_perfect_tracking_recovers_reference_torque_without_double_gyroscopic_term():
    rotation=Rotation.from_euler('xyz',[.2,-.3,.5]).as_matrix()
    inertia=np.diag([.1,.1,.2]);omega=np.array([1.,2.,3.]);derivative=np.array([.4,-.2,.1])
    reference=inertia@derivative+np.cross(omega,inertia@omega)
    result=quadrotor_tracking_torque(rotation,rotation,omega,omega,derivative,inertia,np.zeros(3))
    np.testing.assert_allclose(result,reference,atol=1e-14)


def test_reference_rate_transport_derivative_matches_rotation_finite_differences():
    current=Rotation.from_euler('xyz',[.2,-.1,.3]).as_matrix()
    reference=Rotation.from_euler('xyz',[-.3,.4,.1]).as_matrix()
    omega=np.array([.3,.1,-.2]);desired=np.array([.1,-.2,.05]);derivative=np.array([.03,.1,-.1])
    inertia=np.diag([.1,.1,.2]);dt=1e-5
    def rate(t):
        R=current@Rotation.from_rotvec(t*omega).as_matrix()
        Rd=reference@Rotation.from_rotvec(t*desired+.5*t*t*derivative).as_matrix()
        return R.T@Rd@(desired+t*derivative)
    numerical=(rate(dt)-rate(-dt))/(2*dt)
    result=quadrotor_tracking_torque(current,reference,omega,desired,derivative,inertia,np.zeros(3),0.,0.)
    np.testing.assert_allclose(result-np.cross(omega,inertia@omega),inertia@numerical,atol=1e-11)


def test_rate_feedback_dissipates_rotational_energy():
    inertia=np.array([[.1,.02,0.],[.02,.15,.01],[0.,.01,.2]])
    omega=np.array([.3,-.2,.1]);gain=1.4
    torque=quadrotor_tracking_torque(np.eye(3),np.eye(3),omega,np.zeros(3),np.zeros(3),inertia,np.zeros(3))
    derivative=np.linalg.solve(inertia,torque-np.cross(omega,inertia@omega))
    np.testing.assert_allclose(omega@inertia@derivative,-gain*(omega@omega),atol=1e-14)

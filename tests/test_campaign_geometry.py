"""Verify the independent batched measurement oracle against pointwise URDF FK."""
import numpy as np
import vamp

from benchmarks.inventory_mbm import SphereOracle, obstacle_hits
from benchmarks.manipulator_campaign import independent_sphere_centers, independent_environment_check
from flask_repro import mbm, robot


def test_batched_urdf_oracle_matches_pointwise_transforms_and_collision_witness():
    oracle = SphereOracle(mbm.VAMP_ROOT / 'resources/panda/panda_spherized.urdf')
    rb = robot.get_robot('panda');rng = np.random.default_rng(1008)
    dataset = mbm.load_dataset('panda')
    coordinates = rng.uniform(rb.q_lo, rb.q_hi, (17, rb.n))
    names = vamp.panda.joint_names()
    spheres = [oracle.forward(dict(zip(names, q))) for q in coordinates]
    expected = np.array([[sphere[1] for sphere in row] for row in spheres])
    np.testing.assert_allclose(independent_sphere_centers(oracle, coordinates), expected,
                               rtol=1e-12, atol=1e-12)
    for environment in mbm.ENVIRONMENTS:
        problem = dataset[environment][0]
        pointwise = [bool(obstacle_hits(row, problem)) for row in spheres]
        batched = independent_environment_check(oracle, coordinates, problem)
        assert batched['environment_collision'] == any(pointwise)
        if any(pointwise):
            assert batched['first_witness'][0]['sample'] == pointwise.index(True)
            assert batched['first_witness'][0]['hits']
    # The source-invalid 700th goal must remain a positive collision control.
    problem = dataset['table_pick'][40]
    assert independent_environment_check(oracle, np.array(problem['goals']), problem)['environment_collision']

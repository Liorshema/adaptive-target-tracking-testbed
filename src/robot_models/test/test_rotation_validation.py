"""SO(3) validation retains finite checks and the established tolerances."""

import numpy as np

import pytest

from robot_models.common.rotations import is_rotation_matrix

from scipy.spatial.transform import Rotation


@pytest.mark.parametrize('matrix,expected', [
    (np.eye(3), True), (np.diag([1., 1., -1.]), False),
    (np.zeros((3, 3)), False), (np.eye(4), False),
    (np.full((3, 3), np.nan), False), (np.full((3, 3), np.inf), False),
    (np.diag([1.01, 1., 1.]), False),
])
def test_rotation_validation_accepts_rotations_and_rejects_bad_geometry(matrix, expected):
    """Reflections, scale changes, invalid dimensions and nonfinite values fail."""
    with np.errstate(invalid='ignore'):
        assert is_rotation_matrix(matrix) is expected


@pytest.mark.parametrize('atol', [0., 1e-10, 1e-8, 1e-5])
def test_so3_validation_matches_original_tolerances(atol):
    """Compare actual near-boundary rotations with the pre-optimization contract."""
    generator = np.random.default_rng(17)
    matrices = list(Rotation.random(50, random_state=generator).as_matrix())
    for scale in [atol * .9, atol * 1.1, 5e-6, 1e-5, 2e-5]:
        matrices.extend(rotation + generator.normal(size=(3, 3)) * scale
                        for rotation in matrices[:10])
        matrices.extend(np.eye(3) + scale * basis for basis in np.eye(9).reshape(9, 3, 3))
    for rotation in matrices:
        expected = bool(np.allclose(rotation.T @ rotation, np.eye(3), atol=atol) and
                        np.isclose(np.linalg.det(rotation), 1., atol=atol))
        assert is_rotation_matrix(rotation, atol=atol) == expected

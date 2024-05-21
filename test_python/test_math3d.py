#!/usr/bin/env python

import numpy as np
import cffirmware
from cffirmware import *

def test_that_vec_is_converted_to_numpy_array():
    # Fixture
    v_cf = cffirmware.mkvec(1, 2, 3)

    # Test
    actual = np.array(v_cf)

    # Assert
    expected = np.array([1, 2, 3])
    assert np.allclose(expected, actual)


def test_normalize_radians():
    # Fixture
    angles = [-100, -5, 0, np.pi + 0.1, -np.pi - 0.1, 100]

    for angle in angles:
        # Test
        actual = cffirmware.normalize_radians(angle)
        # Assert
        expected = np.arctan2(np.sin(angle), np.cos(angle))
        assert np.allclose(expected, actual)


def test_shortest_signed_angle_radians():
    # Fixture
    angle_pairs = [(-np.pi/2, np.pi), (np.pi/2, np.pi), (np.pi, -np.pi/3)]

    for start, goal in angle_pairs:
        # Test
        actual = cffirmware.shortest_signed_angle_radians(start, goal)
        # Assert
        expected = np.arctan2(np.sin(goal - start), np.cos(goal - start))
        assert np.allclose(expected, actual)


def test_quat_log():
    rng = np.random.default_rng(0)
    for i in range(1000):
        # avoid singularity
        angle = rng.uniform(0, 3.14)
        axis = vnormalize(mkvec(*rng.normal(size=3)))
        q = qaxisangle(axis, angle)
        log = qlog(q)
        assert np.allclose(log, angle * axis, atol=1e-6, rtol=1e-4)

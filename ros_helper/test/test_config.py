#!/usr/bin/env python
import os
import unittest

import numpy
import rosunit

PKG = 'ros_helper'

# Methods to test
from ros_helper.config import parse_filename
from ros_helper.config import load_config
from ros_helper.config import quaternion_from_euler_deg


class TestConfig(unittest.TestCase):

    FIND_FILENAME = '$(find ros_helper)/test/resources/test_config.yaml'
    CONFIG = {'one': 1, 'two': 2, 'three': 3}

    def test_1(self):
        """Does parse_filename point to a file we know exists?"""
        self.assertTrue(os.path.exists(parse_filename(self.FIND_FILENAME)))

    def test_2(self):
        """Does load_config give the correct dictionary?"""
        self.assertEqual(load_config(self.FIND_FILENAME), self.CONFIG)

    def test_3(self):
        """Does quaternion_from_euler_deg give correct values."""

        tol = 1e-7
        test_results = []

        test1_eul = numpy.zeros(3)
        test1_quat = numpy.array([0, 0, 0, 1])
        test_results.append(
            numpy.linalg.norm(test1_quat - quaternion_from_euler_deg(test1_eul)) <= tol
        )

        test2_eul = numpy.array([ 57.29577951, -57.29577951,  28.64788976])
        test2_quat = numpy.array([ 0.51174747, -0.30356422,  0.41324185,  0.6893435])
        test_results.append(
            numpy.linalg.norm(test2_quat - quaternion_from_euler_deg(test2_eul)) <= tol
        )

        self.assertTrue(all(test_results))


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_config', TestConfig)

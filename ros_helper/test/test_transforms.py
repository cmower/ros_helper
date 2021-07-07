#!/usr/bin/env python
import os
import pickle
import unittest

import rosunit
import rospkg
import numpy

PKG = 'ros_helper'
PKG_PATH = rospkg.RosPack().get_path(PKG)

# Methods to test
from ros_helper.transforms import load_tf
from ros_helper.transforms import position_from_msg
from ros_helper.transforms import quaternion_from_msg
from ros_helper.transforms import euler_from_msg
from ros_helper.transforms import transform_from_msg
from ros_helper.transforms import rotation_from_msg


class TestTransforms(unittest.TestCase):

    TEST_FILENAME = os.path.join(PKG_PATH, 'test', 'resources', 'tf_test.npy')

    MATRIX_4x4 = numpy.eye(4)
    MATRIX_4x4[:3, -1] = 1

    MATRIX_3x3 = numpy.eye(3)

    MATRIX_3 = numpy.ones(3)

    MATRIX_4 = numpy.array([0, 0, 0, 1])

    MATRIX_7 = numpy.array([1, 1, 1, 0, 0, 0, 1])

    EULER = numpy.zeros(3)

    TOL = 1e-8

    with open(os.path.join(PKG_PATH, 'test', 'resources', 'tf_test.transform_stamped.pickle'), 'rb') as f:
        TRANSFORM_STAMPED = pickle.load(f)

    # load_tf

    def test_1_1(self):
        """does load_tf give a numpy.ndarray?"""
        self.assertIsInstance(load_tf(self.TEST_FILENAME), numpy.ndarray)

    def test_1_2(self):
        """does load_tf give correct matrix under default arguments?"""
        self.assertTrue(numpy.linalg.norm(load_tf(self.TEST_FILENAME) - self.MATRIX_4x4) <= self.TOL)

    def test_1_3(self):
        """does load_tf give correct matrix for fmt='3x3'?"""
        self.assertTrue(numpy.linalg.norm(load_tf(self.TEST_FILENAME, fmt='3x3') - self.MATRIX_3x3) <= self.TOL)

    def test_1_4(self):
        """does load_tf give correct matrix for fmt='3'?"""
        self.assertTrue(numpy.linalg.norm(load_tf(self.TEST_FILENAME, fmt='3') - self.MATRIX_3) <= self.TOL)

    def test_1_5(self):
        """does load_tf give correct matrix for fmt='4'?"""
        self.assertTrue(numpy.linalg.norm(load_tf(self.TEST_FILENAME, fmt='4') - self.MATRIX_4) <= self.TOL)

    def test_1_6(self):
        """does load_tf give correct matrix for fmt='7'?"""
        self.assertTrue(numpy.linalg.norm(load_tf(self.TEST_FILENAME, fmt='7') - self.MATRIX_7) <= self.TOL)

    # position_from_msg

    def test_2_1(self):
        """Position from message gives correct output under default arguments?"""
        self.assertTrue(numpy.linalg.norm(position_from_msg(self.TRANSFORM_STAMPED) - self.MATRIX_3) <= self.TOL)

    def test_2_2(self):
        """Position from message gives correct output for fmt='xy'?"""
        self.assertTrue(numpy.linalg.norm(position_from_msg(self.TRANSFORM_STAMPED, fmt='xy') - self.MATRIX_3[:2]) <= self.TOL)


    # quaterion_from_msg

    def test_3_1(self):
        """Quaterion from message gives correct output under default arguments?"""
        self.assertTrue(numpy.linalg.norm(quaternion_from_msg(self.TRANSFORM_STAMPED) - self.MATRIX_4) <= self.TOL)

    def test_3_2(self):
        """Quaterion from message gives correct output for fmt='xy'?"""
        self.assertTrue(numpy.linalg.norm(quaternion_from_msg(self.TRANSFORM_STAMPED, fmt='xyz') - self.MATRIX_4[:3]) <= self.TOL)

    # euler_from_msg

    def test_4(self):
        """Euler from message gives correct output under default arguments?"""
        self.assertTrue(numpy.linalg.norm(euler_from_msg(self.TRANSFORM_STAMPED) - self.EULER) <= self.TOL)

    # transform_from_msg

    def test_5(self):
        """Transform from message gives correct output under default arguments?"""
        self.assertTrue(numpy.linalg.norm(transform_from_msg(self.TRANSFORM_STAMPED) - self.MATRIX_4x4) <= self.TOL)

    # rotation_from_msg

    def test_6(self):
        """Rotation from message gives correct output under default arguments?"""
        self.assertTrue(numpy.linalg.norm(rotation_from_msg(self.TRANSFORM_STAMPED) - self.MATRIX_3x3) <= self.TOL)



if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_transforms', TestTransforms)


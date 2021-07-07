#!/usr/bin/env python
import os
import unittest

import rosunit

PKG = 'ros_helper'

# Methods to test
from ros_helper.config import parse_filename
from ros_helper.config import load_config


class TestConfig(unittest.TestCase):

    FIND_FILENAME = '$(find ros_helper)/test/resources/test_config.yaml'
    CONFIG = {'one': 1, 'two': 2, 'three': 3}

    def test_1(self):
        """Does parse_filename point to a file we know exists?"""
        self.assertTrue(os.path.exists(parse_filename(self.FIND_FILENAME)))

    def test_2(self):
        """Does load_config give the correct dictionary?"""
        self.assertEqual(load_config(self.FIND_FILENAME), self.CONFIG)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_config', TestConfig)

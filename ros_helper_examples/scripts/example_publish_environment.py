#!/usr/bin/env python3
"""
BSD 2-Clause License

Copyright (c) 2021, https://github.com/cmower/ros_helper by Christopher E. Mower
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import rospy
from math import cos, sin, pi
from ros_helper.node import RosNode

class Node(RosNode):

    def __init__(self):
        RosNode.__init__(self, 'example_publish_environment_node')
        self.create_timer('main_loop', 100, self.main_loop)

    def main_loop(self, event):
        t = rospy.Time.now().to_sec()
        p = [
            -0.1 + 0.8*sin(t*pi*0.2),
            0.5 + 0.8*sin(t*pi*0.5*0.2),
            cos(t*pi*0.2),
        ]
        self.set_tf('world', 'sphere', p)

if __name__ == '__main__':
    Node().spin()

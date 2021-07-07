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
from sensor_msgs.msg import Joy

from ros_helper.node import RosNode
from ros_helper.msg import MultiJoy

class Node(RosNode):

    def __init__(self):

        # Initialization
        RosNode.__init__(self, 'multijoy_node')

        # Get parameters
        self.collect_params([
            ('~joy_topics', ['joy']),  # a list of sensor_msgs/Joy topics
            ('~hz', 100),  # sampling frequency
        ])
        self.njoys = len(self.params['~joy_topics'])

        # Setup publisher
        self.create_publisher('joys_out', 'multijoy', MultiJoy)

        # Setup subscribers
        self.joys = [None]*self.njoys
        for j in range(self.njoys):
            topic = self.params['~joy_topics'][j]
            self.create_subscriber(f'joy{j}', topic, Joy, callback=self.callback, callback_args=j)

        # Setup output message
        self.joys_out = MultiJoy(njoys=self.njoys)

        # Start main timer
        self.create_timer('main_loop', self.params['~hz'], self.main_loop)

    def callback(self, joy, j):
        self.joys[j] = joy

    def main_loop(self, event):
        if any(joy is None for joy in self.joys):
            return
        self.joys_out.header.stamp = rospy.Time.now()
        self.joys_out.joys = self.joys
        self.pubs['joys_out'].publish(self.joys_out)


if __name__ == '__main__':
    Node().spin()

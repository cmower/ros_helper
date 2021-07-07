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
import sys
import rospy
import numpy

from ros_helper.node import RosNode
from ros_helper.transforms import transform_from_msg

class Node(RosNode):

    TIMEOUT = 100

    def __init__(self):

        # Initialization
        RosNode.__init__(self, 'save_tf_node', disable_signals=True)

        # Get frames, assumes they are given by command line arguments
        self.baseid = sys.argv[1]
        self.childid = sys.argv[2]
        n = 1
        if len(sys.argv) > 3:
            n = int(sys.argv[3])

        self.nfails = 0
        self.idx = 0
        self.transforms = [None]*n
        self.listen_to_tf('tf', self.baseid, self.childid, frequency=100, callback=self.callback)

    def did_timeout(self):
        self.nfails += 1
        return self.nfails == self.TIMEOUT

    def append_transform(self, tf):
        self.transforms[self.idx] = tf
        self.idx += 1
        rospy.loginfo(f'collected transform {self.idx}/{len(self.transforms)}')

    def is_finished(self):
        return self.idx == len(self.transforms)

    def callback(self, tf):
        if tf is None:
            if self.did_timeout():
                rospy.logerr('save_tf.py reached timeout')
                rospy.signal_shutdown('finished')
            return
        self.append_transform(tf)
        if self.is_finished():
            rospy.signal_shutdown('finished')

    def shutdown(self):
        self.kill()
        if self.is_finished():
            tf = numpy.array(list(map(transform_from_msg, self.transforms))).mean(axis=0)
            filename = f'tf_{self.baseid}_{self.childid}.npy'
            numpy.save(filename, tf)
            rospy.loginfo(f'saved {filename}')

if __name__ == '__main__':
    Node().spin()

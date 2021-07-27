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
from visualization_msgs.msg import Marker, MarkerArray

from ros_helper.node import RosNode
from ros_helper.config import load_config, quaternion_from_euler_deg

CONFIGS = '~configs'  # list of filenames
NS = 'env'
MARKERS_PUB = 'markers_publisher'
MARKERS_TIMER = 'markers_timer'
HZ = 40

class Node(RosNode):

    def __init__(self):

        # Initialization
        RosNode.__init__(self, 'publish_environment_node')

        # Setup marker array
        self.marker_array = MarkerArray()

        # Get parameters
        self.collect_params([
            (CONFIGS,),
        ])

        # Load configs
        #
        # Fields that configs must include:
        #   name          [str] - unique name
        #   type          [str] - object type
        #   rgba     [float[4]] - color
        #   scale    [float[3]] - scale of object
        #   base_frame_id [str] - name of the base frame
        #   posetype      [str] - 'tf' or 'static'
        #
        #   When posetype = 'tf', fields that config must include:
        #      child_frame_id
        #
        #   When posetype = 'static', fields that config should include (with defaults):
        #      position    [float[3]] - position of object (default: [0, 0, 0])
        #      orientation [float[3]] - orientation, euler angles, in degrees (default [0, 0, 0])
        #
        for idx, config in enumerate(self.params[CONFIGS]):

            # Load spec file
            spec = load_config(config)
            name = spec['name']

            # Setup marker (general)
            marker = Marker()
            marker.id = idx
            marker.type = getattr(Marker, spec['type'].upper())
            marker.ns = NS
            marker.action = Marker.ADD
            for d, dim in enumerate('rgba'):
                setattr(marker.color, dim, spec['rgba'][d])
            for d, dim in enumerate('xyz'):
                setattr(marker.scale, dim, spec['scale'][d])

            marker.header.frame_id = spec['base_frame_id']
            posetype = spec['posetype'].lower()
            if posetype == 'tf':

                def update_tf(tf, idx):
                    if tf is None: return
                    for dim in 'xyz':
                        setattr(self.marker_array.markers[idx].pose.position, dim, getattr(tf.transform.translation, dim))
                        setattr(self.marker_array.markers[idx].pose.orientation, dim, getattr(tf.transform.rotation, dim))
                    self.marker_array.markers[idx].pose.orientation.w = tf.transform.rotation.w

                self.listen_to_tf(f'{NS}/{name}/tf', spec['base_frame_id'], spec['child_frame_id'], frequency=HZ, callback=update_tf, callback_args=idx)

            elif posetype == 'static':
                # Assumes pose is static and given in config

                position = spec.get('position', [0, 0, 0])
                quaternion = quaternion_from_euler_deg(spec.get('orientation', [0, 0, 0]))
                for d, dim in enumerate('xyz'):
                    setattr(marker.pose.position, dim, position[d])
                    setattr(marker.pose.orientation, dim, quaternion[d])
                marker.pose.orientation.w = quaternion[3]

            # Marker type specific
            if marker.type == Marker.TEXT_VIEW_FACING:
                marker.text = spec['text']
            elif marker.type == Marker.MESH_RESOURCE:
                marker.mesh_resource = spec['mesh_resource']

            # Append marker
            self.marker_array.markers.append(marker)

            # Report progress
            rospy.loginfo(f'Loaded {name}')

        # Setup marker array publisher
        self.create_publisher(MARKERS_PUB, f'{NS}/markers', MarkerArray)

        # Start main timer
        self.create_timer(MARKERS_TIMER, HZ, self.main_loop)

    def main_loop(self, event):
        for m in self.marker_array.markers:
            m.header.stamp = rospy.Time.now()
        self.pubs[MARKERS_PUB].publish(self.marker_array)

if __name__ == '__main__':
    Node().spin()

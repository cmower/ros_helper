#!/usr/bin/env python3
import rospy
from ros_helper.msg import MultiJoy
from sensor_msgs.msg import Joy
from ros_helper.node import RosNode

class Node(RosNode):

    def __init__(self):

        # Initialization
        RosNode.__init__(self, rospy)
        self.initNode('multijoy_node')
        self.onShutdownUseBaseShutdownMethod()

        # Get parameters
        self.getParams([
            ('~joy_topics', ['joy']),  # a list of sensor_msgs/Joy topics
            ('~hz', 100),  # sampling frequency
        ])
        self.njoys = len(self.params['~joy_topics'])

        # Setup publisher
        self.setupPublisher('joys_out', 'multijoy', MultiJoy)

        # Setup subscribers
        self.joys = [None]*self.njoys
        for j in range(self.njoys):
            topic = self.params['~joy_topics'][j]
            self.subs[f'joy_sub_{j}'] = rospy.Subscriber(topic, Joy, self.callback, callback_args=j)

        # Setup output message
        self.joys_out = MultiJoy(njoys=self.njoys)

        # Start main timer
        self.startTime('main', self.params['~hz'], self.main)

    def callback(self, joy, j):
        self.joys[j] = joy

    def main(self, event):
        if any(joy is None for joy in self.joys):
            return
        self.joys_out.header.stamp = rospy.Time.now()
        self.joys_out.joys = self.joys
        self.pubs['joys_out'].publish(self.joys_out)


if __name__ == '__main__':
    Node().spin()

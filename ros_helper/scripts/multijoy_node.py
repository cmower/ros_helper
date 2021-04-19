#!/usr/bin/env python3
import rospy
import message_filters
from ros_helper.msg import MultiJoy
from sensor_msgs.msg import Joy
from ros_helper.node import RosNode

"""
TODO:
- investigate parameters for message_filters.ApproximateTimeSynchronizer method
- how to unregister subscribers?
"""

class Node(RosNode):

    def __init__(self):

        # Initialization
        super().__init__(rospy)
        self.initNode()

        # Get parameters
        self.getParams([
            ('~joy_topics', ['joy']), # a list of sensor_msgs/Joy topics
        ])
        self.n_joy = len(self.params['~joy_topics'])

        # Setup publisher and subscriber
        self.setupPublisher('multi_joy', 'multijoy', MultiJoy)

        # Setup subscribers
        for j in range(self.n_joy):
            topic = self.params['~joy_topics'][j]
            self.subs[topic] = message_filters.Subscriber(topic, Joy)

        self.time_sync=message_filters.ApproximateTimeSynchronizer(self.subs.values(), 10, self.njoys*100)
        self.time_sync.registerCallback(self.callback)

    def callback(self, *args):
        self.pubs['multi_joy'].publish(self.addTimeStampToMsg(MultiJoy(njoys=self.n_joys, joys=args)))

if __name__ == '__main__':
    Node().spin()

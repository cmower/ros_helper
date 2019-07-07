#!/usr/bin/env python
import rospy
import xml.etree.ElementTree as eltree
from ros_helper.msgs.visualization import *
from ros_helper.simple_pub_sub import SimpleConstPublisher, SimpleTfStaticPublisher

raise NotImplementedError("Do not use publish_environment_node, it is not fully implemented yet.")

class XMLLoad(object):

    def __init__(self):
        tree_root = eltree.parse(rospy.get_param("~filename")).getroot()

if __name__=='__main__':

    # Init
    rospy.init_node('publish_environment_node')
    env = XMLLoad()
    if rospy.has_param('~hz'):
        hz = rospy.get_param('~hz')
    else:
        hz = 20

    # Setup publishers and spin
    SimpleTfStaticPublisher(rospy, env.Transforms, hz)
    SimpleConstPublisher(rospy, env.Topic, hz, env.Markers)
    rospy.spin()

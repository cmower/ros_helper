#!/usr/bin/env python
import rospy
import tf
import numpy as np
import xml.etree.ElementTree as eltree
from ros_helper.msgs.visualization import *
from ros_helper.msgs.geometry import TransformStampedMsg
from ros_helper.simple_pub_sub import SimpleConstPublisher, SimpleTfStaticPublisher

raise NotImplementedError("Do not use publish_environment_node, it is not fully implemented yet.")

WORLD_FRAME = ''

class TransformFrame(object):

    def __init__(self, attr):

        # Unpack attr
        child_frame_id = attr['name']
        try:
            trans = np.fromstring(attr['trans'], sep=' ')
        except:
            trans = np.zeros(3)
        try:
            quat = np.fromstring(attr['quat'], sep=' ')
            if quat.shape[0] == 3: quat = np.transformations.quaternion_from_euler(quat[0], quat[1], quat[2])
        except:
            quat = np.array([0, 0, 0, 1])

            
            

        self.transform = TransformStampedMsg(trans, quat, child_frame_id='/'.join((namespace,child_frame_id)), base_frame_id=base_frame_id)
        self.hz = hz

class XMLLoad(object):

    def __init__(self):

        # Init
        transforms = []
        

        # Extract
        for child in eltree.parse(rospy.get_param("~filename")).getroot():
            if child.tag == 'TransformFrame':
                transform.append(

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

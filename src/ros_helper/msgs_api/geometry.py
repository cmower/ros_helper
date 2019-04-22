import tf
import numpy as np
from geometry_msgs.msg import Point, TransformStamped
from .utils import *

class PointMsg(Point):

    def __init__(self, pos):
        super(PointMsg, self).__init__()
        _msetter(self, XYZ, pos)

class TransformStampedMsg(TransformStamped):

    def __init__(self, t, trans, quat, child_frame_id, base_frame_id):
        super(TransformStampedMsg, self).__init__()
        self.header.stamp = t
        self.header.frame_id = base_frame_id
        self.child_frame_id = child_frame_id
        _msetter(self.transform.translation, XYZ, trans)
        _msetter(self.transform.rotation, XYZW, quat)

    @staticmethod
    def get_trans_quat(tr):
        return TransformStampedMsg.get_trans(tr), TransformStampedMsg.get_quat(tr)

    @staticmethod
    def get_trans(tr):
    	return np.array(_mgetter(tr.transform.translation, XYZ))

    @staticmethod
    def get_quat(tr):
    	return np.array(_mgetter(tr.transform.rotation, XYZW))

    @staticmethod
    def get_matrix(tr):
   	T = tf.transformations.quaternion_matrix(TransformStampedMsg.get_quat(tr))
   	T[:3,3] = TransformStampedMsg.get_trans(tr)
   	return T

    @staticmethod
    def get_euler(tr):
   	return tf.transformations.euler_from_quaternion(TransformStampedMsg.get_quat(tr))

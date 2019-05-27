import tf
import numpy as np
from geometry_msgs.msg import Point, TransformStamped
from .utils import *

class PointMsg(Point):

    def __init__(self, p):
        super(PointMsg, self).__init__()
        msetattr(self, XYZ, p)

    @staticmethod
    def as_np(pt):
        return np.array(mgetattr(pt, XYZ))        

    def to_np(self):
        return PointMsg.as_np(self)

class TransformStampedMsg(TransformStamped):

    def __init__(self, t, trans, quat, child_frame_id, base_frame_id):
        super(TransformStampedMsg, self).__init__()
        self.header.stamp = t
        self.header.frame_id = base_frame_id
        self.child_frame_id = child_frame_id
        msetattr(self.transform.translation, XYZ, trans)
        msetattr(self.transform.rotation, XYZW, quat)

    @staticmethod
    def as_trans_quat(tr):
        return TransformStampedMsg.as_trans(tr), TransformStampedMsg.as_quat(tr)

    @staticmethod
    def as_trans(tr):
    	return np.array(mgetattr(tr.transform.translation, XYZ))

    @staticmethod
    def as_quat(tr):
    	return np.array(mgetattr(tr.transform.rotation, XYZW))

    @staticmethod
    def as_matrix(tr):
   	T = tf.transformations.quaternion_matrix(TransformStampedMsg.as_quat(tr))
   	T[:3,3] = TransformStampedMsg.as_trans(tr)
   	return T

    @staticmethod
    def as_euler(tr):
   	return tf.transformations.euler_from_quaternion(TransformStampedMsg.as_quat(tr))

    def to_trans_quat(self):
        return TransformStampedMsg.as_trans(self), TransformStampedMsg.as_quat(self)

    def to_trans(self):
        return TransformStampedMsg.as_trans(self)

    def to_quat(self):
        return TransformStampedMsg.as_quat(self)

    def to_matrix(self):
        return TransformStampedMsg.as_matrix(self)

    def to_euler(self):
        return TransformStampedMsg.as_euler(self)    

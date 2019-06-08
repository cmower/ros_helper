import tf
import numpy as np
from geometry_msgs.msg import *
from .utils import *

XYZ = ['x', 'y', 'z']
XYZW = ['x', 'y', 'z', 'w']

class Vector3Msg(Vector3):

    def __init__(self, v):
        super(Vector3Msg, self).__init__()
        if type(v) in [Vector3, Vector3Msg]:
            for d in XYZ: setattr(self, d, getattr(v, d))
        else:
            msetattr(self, XYZ, v)

    def to_np(self):
        return np.array(mgetattr(self, XYZ))

class PointMsg(Point):

    def __init__(self, p):
        super(PointMsg, self).__init__()
        if type(p) in [Point, PointMsg]:
            for d in XYZ: setattr(self, d, getattr(p, d))
        else:
            msetattr(self, XYZ, p)
            
    def to_np(self):
        return np.array(mgetattr(self, XYZ))
    
class QuaternionMsg(Quaternion):

    def __init__(self, q):
        super(QuaternionMsg, self).__init__()
        if type(q) in [QuaternionMsg, Quaternion]:
            for d in XYZW: setattr(self, d, getattr(p, d))
        else:
            msetattr(self, XYZW, q)

    def to_np(self):
        return np.array(mgetter(self, XYZ))

    def to_matrix(self):
        return tf.transformations.quaternion_matrix(self.to_np())

    def to_euler(self):
        return tf.transformations.euler_from_quaternion(self.to_np())
    
class TransformStampedMsg(TransformStamped):

    def __init__(self, time, trans=None, quat=None, child_frame_id=None, base_frame_id=None):
        super(TransformStampedMsg, self).__init__()

        if type(time) in [TransformStampedMsg, TransformStamped]:
            tr = time
            self.header = tr.header
            self.child_frame_id = tr.child_frame_id
            self.transform.translation = Vector3Msg(tr.transform.translation)
            self.transform.rotation = QuaternionMsg(tr.transform.rotation)
        else:
            # assumes all input are not None
            self.header.stamp = time
            self.header.frame_id = base_frame_id
            self.child_frame_id = child_frame_id
            self.transform.translation = Vector3Msg(trans)
            self.transform.rotation = QuaternionMsg(quat)

    def to_trans_quat(self):
        return self.transform.translation.to_np(), self.transform.rotation.to_np()

    def to_trans(self):
        return self.transform.translation.to_np()

    def to_quat(self):
        return self.transform.rotation.to_np()

    def to_matrix(self):
        T = self.transform.rotation.to_matrix()
        T[:3,3] = self.transform.translation.to_np()
        return T

    def to_euler(self):
        return self.transform.rotation.to_euler()

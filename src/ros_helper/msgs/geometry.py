import tf
import numpy as np
from geometry_msgs.msg import *
from ..utils import *

#
# Geometry helper functions
#

def _set_trans(obj, sup_class, tr):
    msetattr(obj, XYZ, mgetattr(tr, XYZ) if sup_class in get_object_class_hierarchy(tr) else tr)

def _trans_to_np(obj):
    return np.asarray(mgetattr(obj, XYZ))

#
# Msg classes
#


class AccelMsg(Accel):

    def __init__(self):
        super(AccelMsg, self).__init__()
        raise NotImplementedError("AccelMsg is not yet implemented!")

class AccelStampedMsg(AccelStamped):

    def __init__(self):
        super(AccelStampedMsg, self).__init__()
        raise NotImplementedError("AccelStampedMsg is not yet implemented!")

class AccelWithCovarianceMsg(AccelWithCovariance):

    def __init__(self):
        super(AccelWithCovarianceMsg, self).__init__()
        raise NotImplementedError("AccelWithCovarianceMsg is not yet implemented!")

class AccelWithCovarianceStampedMsg(AccelWithCovarianceStamped):

    def __init__(self):
        super(AccelWithCovarianceStampedMsg, self).__init__()
        raise NotImplementedError("AccelWithCovarianceStampedMsg is not yet implemented!")

class InertiaMsg(Inertia):

    def __init__(self):
        super(InertiaMsg, self).__init__()
        raise NotImplementedError("InertiaMsg is not yet implemented!")

class InertiaStampedMsg(InertiaStamped):

    def __init__(self):
        super(InertiaStampedMsg, self).__init__()
        raise NotImplementedError("InertiaStampedMsg is not yet implemented!")

class PointMsg(Point):

    def __init__(self, trans=[0,0,0]):
        super(PointMsg, self).__init__()
        _set_trans(self, Point, trans)

    def to_np(self):
        return _trans_to_np(self)

class Point32Msg(Point32):

    def __init__(self, trans=[0,0,0]):
        super(Point32Msg, self).__init__()
        _set_trans(self, Point, trans)

    def to_np(self):
        return _trans_to_np(self)

class PointStampedMsg(PointStamped):

    def __init__(self, trans, time=None, frame_id=''):
        super(PointStampedMsg, self).__init__()
        if PointStamped in get_object_class_hierarchy(trans):
            self.point = trans.point
            self.header = trans.header
        else:
            # assume trans is a list, tuple, or np.array
            msetattr(self.point, XYZ, trans)
            if time is not None: self.time = time
            self.frame_id = frame_id

    @property
    def time(self):
        return self.header.stamp

    @time.setter
    def time(self, t):
        self.header.stamp = t

    def to_np(self):
        return _trans_to_np(self.point)

class PolygonMsg(Polygon):

    def __init__(self):
        super(PolygonMsg, self).__init__()
        raise NotImplementedError("PolygonMsg is not yet implemented!")

class PolygonStampedMsg(PolygonStamped):

    def __init__(self):
        super(PolygonStampedMsg, self).__init__()
        raise NotImplementedError("PolygonStampedMsg is not yet implemented!")

class PoseMsg(Pose):

    def __init__(self, trans=[0,0,0], quat=[0,0,0,1]):
        super(PoseMsg, self).__init__()
        if Pose in get_object_class_hierarchy(trans):
            self.position = PointMsg(trans.position)
            self.orientation = QuaternionMsg(trans.orientation)
        else:
            # assume trans/quat are a tuple, list, or np.array
            if len(trans) == 3:
                # assumes quat is defined
                trans_ = trans
                quat_ = quat
            else:
                # assumes trans is 6 or 7 in length, i.e. 3 trans and 3/4 ori
                trans_ = trans[:3]
                quat_ = trans[3:]
            self.position = PointMsg(trans_)
            self.orientation = QuaternionMsg(quat_)

    def to_np(self):
        T = tf.transformations.quaternion_matrix(self.orientation.to_np())
        T[:3,3] = self.position.to_np()
        return T

class Pose2DMsg(Pose2D):

    def __init__(self):
        super(Pose2DMsg, self).__init__()
        raise DeprecationWarning("ROS has deprecated Pose2D and so is not supported here, use PoseMsg.")

class PoseArrayMsg(PoseArray):

    def __init__(self):
        super(PoseArrayMsg, self).__init__()
        raise NotImplementedError("PoseArrayMsg is not yet implemented!")

class PoseStampedMsg(PoseStamped):

    def __init__(self):
        super(PoseStampedMsg, self).__init__()
        raise NotImplementedError("PoseStampedMsg is not yet implemented!")

class PoseWithCovarianceMsg(PoseWithCovariance):

    def __init__(self):
        super(PoseWithCovarianceMsg, self).__init__()
        raise NotImplementedError("PoseWithCovarianceMsg is not yet implemented!")

class PoseWithCovarianceStampedMsg(PoseWithCovarianceStamped):

    def __init__(self):
        super(PoseWithCovarianceStampedMsg, self).__init__()
        raise NotImplementedError("PoseWithCovarianceStampedMsg is not yet implemented!")

class QuaternionMsg(Quaternion):

    def __init__(self, quat=[0,0,0,1]):
        super(QuaternionMsg, self).__init__()

        # Extract quaternion as a vector
        if Quaternion in get_object_class_hierarchy(quat):
            quat_ = mgetattr(quat, XYZW)
        else:
            # Assume quat is list, tuple, or np.array
            n = len(quat)
            if n == 3:
                # assumes quat is rpy angles
                quat_ = tf.transformations.quaternion_from_euler(quat[0], quat[1], quat[2])
            elif n == 4:
                # assumes quat is given
                quat_ = quat
            else:
                raise ValueError("Expected input to be length 3 or 4, got %d" % n)

        # Set attr
        msetattr(self, XYZW, np.asarray(quat_) / np.linalg.norm(quat_)) # ensure quat_ is normalized

    def to_rpy(self):
        return tf.transformations.euler_from_quaternion(self.to_np())

    def to_matrix(self):
        return tf.transformations.quaternion_matrix(self.to_np())

    def to_np(self):
        return np.asarray(mgetattr(self, XYZW))

class QuaternionStampedMsg(QuaternionStamped):

    def __init__(self):
        super(QuaternionStampedMsg, self).__init__()
        raise NotImplementedError("QuaternionStampedMsg is not yet implemented!")

class TransformMsg(Transform):

    def __init__(self, trans=[0,0,0], quat=[0,0,0,1]):
        super(TransformMsg, self).__init__()
        if Transform in get_object_class_hierarchy(trans):
            self.translation = Vector3Msg(trans.translation)
            self.rotation = QuaternionMsg(trans.rotation)
        else:
            if len(trans) is 3:
                self.translation = Vector3Msg(trans)
                self.rotation = QuaternionMsg(quat)
            else:
                self.translation = Vector3Msg(trans[:3])
                self.rotation = QuaternionMsg(trans[3:])

    def to_np(self):
        T = self.rotation.to_matrix()
        T[:3,3] = self.translation.to_np()
        return T

class TransformStampedMsg(TransformStamped):

    def __init__(self, trans=[0, 0, 0], quat=[0, 0, 0, 1], time=None, child_frame_id='', base_frame_id=''):
        super(TransformStampedMsg, self).__init__()
        if TransformStamped in get_object_class_hierarchy(trans):
            self.header = trans.header
            self.child_frame_id = trans.child_frame_id
            self.transform = TransformMsg(trans.transform)
        else:
            self.transform = TransformMsg(trans, quat)
            if time is not None: self.time = time
            self.child_frame_id = child_frame_id
            self.base_frame_id = base_frame_id

    @property
    def base_frame_id(self):
        return self.header.frame_id

    @base_frame_id.setter
    def base_frame_id(self, fr_id):
        self.header.frame_id = fr_id

    @property
    def time(self):
        return self.header.stamp

    @time.setter
    def time(self, t):
        self.header.stamp = t

    def to_np(self):
        return self.transform.to_np()

class TwistMsg(Twist):

    def __init__(self):
        super(TwistMsg, self).__init__()
        raise NotImplementedError("TwistMsg is not yet implemented!")

class TwistStampedMsg(TwistStamped):

    def __init__(self):
        super(TwistStampedMsg, self).__init__()
        raise NotImplementedError("TwistStampedMsg is not yet implemented!")

class TwistWithCovarianceMsg(TwistWithCovariance):

    def __init__(self):
        super(TwistWithCovarianceMsg, self).__init__()
        raise NotImplementedError("TwistWithCovarianceMsg is not yet implemented!")

class TwistWithCovarianceStampedMsg(TwistWithCovarianceStamped):

    def __init__(self):
        super(TwistWithCovarianceStampedMsg, self).__init__()
        raise NotImplementedError("TwistWithCovarianceStampedMsg is not yet implemented!")

class Vector3Msg(Vector3):

    def __init__(self, trans=[0,0,0]):
        super(Vector3Msg, self).__init__()
        _set_trans(self, Vector3, trans)

    def to_np(self):
        return _trans_to_np(self)

class Vector3StampedMsg(Vector3Stamped):

    def __init__(self):
        super(Vector3StampedMsg, self).__init__()
        raise NotImplementedError("Vector3StampedMsg is not yet implemented!")

class WrenchMsg(Wrench):

    def __init__(self):
        super(WrenchMsg, self).__init__()
        raise NotImplementedError("WrenchMsg is not yet implemented!")

class WrenchStampedMsg(WrenchStamped):

    def __init__(self):
        super(WrenchStampedMsg, self).__init__()
        raise NotImplementedError("WrenchStampedMsg is not yet implemented!")

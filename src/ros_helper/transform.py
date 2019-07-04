import tf
import numpy as np
import tf2_ros
from .msgs.geometry import TransformStampedMsg

def _axis_intercept_point_in_plane(trans, quat, i_frame, i_base):
    """
    i_frame - denotes axis of frame expressed by trans, quat
    i_base - denotes axis of plane in base frame, i.e. i_base=2 means use xy plane

    i    Axis
    0 -> x axis
    1 -> y axis
    2 -> z axis
    """
    _trans = np.asarray(trans)
    _quat = np.asarray(quat)
    _axis = tf.transformations.quaternion_matrix(_quat)[:3,i_frame]
    return _trans - _axis * (_trans[i_base] / _axis[i_base])

def x_axis_intercept_point_in_xy_plane(trans, quat):
    """
    Given a transform frame defined wrt to a base frame. Compute the intercept position of the
    x-axis in the xy plane of the base axis.
    """
    return _axis_intercept_point_in_plane(trans, quat, 0, 2)

def y_axis_intercept_point_in_xy_plane(trans, quat):
    """
    Given a transform frame defined wrt to a base frame. Compute the intercept position of the
    y-axis in the xy plane of the base axis.
    """
    return _axis_intercept_point_in_plane(trans, quat, 1, 2)

def z_axis_intercept_point_in_xy_plane(trans, quat):
    """
    Given a transform frame defined wrt to a base frame. Compute the intercept position of the
    z-axis in the xy plane of the base axis.
    """
    return _axis_intercept_point_in_plane(trans, quat, 2, 2)

def nearest_point(x, P):
    """
    Returns the nearest point in P (npts-by-nvec np.array) to point x (nvec np.array) where npts is the number of points in P
    and nvec is the dimension of the vector x.

    Syntax
    ------

      pnear = rh.transform.nearest_point(x, P)

    Parameters
    ----------

      x (np.array or list) : vector with nvec elements.
      P (np.array or list) : list of points or npts-by-nvec array of points
      pnear (np.array)     : nearest point, always returned as a np.array

    """

    # Cast x, P as arrays
    x = np.asarray(x)
    P = np.asarray(P)

    # Find nearest point and return
    return P[np.argmin(np.linalg.norm(P - x, axis=1)),:]

class TfApi(object):

    """
    Api uses tf2, doesn't work for tf.
    """

    def __init__(self, rospy):
        self.br = tf2_ros.TransformBroadcaster()
        self.buff = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buff)
        self.rospy = rospy

    def get_tf(self, child_frame_id, base_frame_id):
        try:
            tr = TransformStampedMsg(self.buff.lookup_transform(base_frame_id, child_frame_id, self.rospy.Time()))
        except:
            tr = None
        return tr

    def set_tf(self, trans, quat=[0, 0, 0, 1], time=None, child_frame_id='', base_frame_id=''):
        self.br.sendTransform(TransformStampedMsg(trans, quat=quat, time=time, child_frame_id=child_frame_id, base_frame_id=base_frame_id))

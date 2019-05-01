import tf
import tf2_ros
from .msgs_api.geometry import TransformStampedMsg

__all__ = ['TfApi', 'x_axis_intercept_point_in_xy_plane', 'y_axis_intercept_point_in_xy_plane', 'z_axis_intercept_point_in_xy_plane']


def _axis_intercept_point_in_plane(trans, quat, i_frame, i_base):
    """
    i_frame - denotes axis of frame expressed by trans, quat
    i_base - denotes axis of plane in base frame, i.e. i_base=2 means use xy plane

    i    Axis
    0 -> x axis
    1 -> y axis
    2 -> z axis
    """
    _axis = tf.tf.transformations.quaternion_matrix(quat)[:3,i_frame]
    return np.array(trans) - _axis * (trans[i_base] / _axis[i_base])

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

class TfApi(object):
    
    def __init__(self, rospy):
        self.br = tf2_ros.TransformBroadcaster()
        self.buff = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buff)
        self.rospy = rospy
        
    def get_tf(self, child_frame_id, base_frame_id):
        did_recieve = False
        try:
            trans, quat = TransformStampedMsg.get_trans_quat(self.buff.lookup_transform(base_frame_id, child_frame_id, self.rospy.Time()))
            did_recieve = True
        except:
            trans = quat = None
        return trans, quat, did_recieve
    
    def set_tf(self, t, trans, quat, child_frame_id, base_frame_id):
        self.br.sendTransform(TransformStampedMsg(t, trans, quat, child_frame_id, base_frame_id))

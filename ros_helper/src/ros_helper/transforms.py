import numpy
import tf_conversions


def position_from_msg(tf_msg, fmt='xyz'):
    """Extract position from geomety_msg/TransformStamed message."""
    return numpy.array([getattr(tf_msg.transform.translation, d) for d in fmt])

def quaternion_from_msg(tf_msg, fmt='xyzw'):
    """Extract quaternion from geomety_msg/TransformStamed message."""
    return numpy.array([getattr(tf_msg.transform.rotation, d) for d in fmt])

def euler_from_msg(tf_msg):
    """Extract Euler angles from geomety_msg/TransformStamed message."""
    return tf_conversions.transformations.euler_from_quaternion(quaternion_from_msg(tf_msg))

def transformFromTf2Msg(tf_msg):
    """Extract transformation matrix from geomety_msg/TransformStamed message."""
    T = tf_conversions.transformations.quaternion_matrix(quaternion_from_msg(tf_msg))
    T[:3, -1] = position_from_msg(tf_msg)
    return T

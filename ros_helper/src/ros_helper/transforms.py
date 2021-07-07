import numpy
import tf_conversions

from .config import parse_filename


def load_tf(filename):
    data = numpy.load(parse_filename(filename))
    tf = numpy.eye(4)
    if data.shape == (4, 4):
        # Full transform
        tf = data.copy()
    elif data.shape == (3, 3):
        # Rotation matrix
        tf[:3, :3] = data.copy()
    elif data.shape == (3,):
        # Position
        tf[:3, -1] = data.copy()
    elif data.shape == (4,):
        # Quaterion
        tf = tf_conversions.transformations.quaternion_matrix(data.copy())
    elif data.shape == (7,):
        # Position+quaternion
        tf = tf_conversions.transformations.quaternion_matrix(data[3:].copy())
        tf[:3, -1] = data[:3].copy()
    return tf


def position_from_msg(tf_msg, fmt='xyz'):
    """Extract position from geomety_msg/TransformStamed message."""
    return numpy.array([getattr(tf_msg.transform.translation, d) for d in fmt])


def quaternion_from_msg(tf_msg, fmt='xyzw'):
    """Extract quaternion from geomety_msg/TransformStamed message."""
    return numpy.array([getattr(tf_msg.transform.rotation, d) for d in fmt])


def euler_from_msg(tf_msg):
    """Extract Euler angles from geomety_msg/TransformStamed message."""
    return tf_conversions.transformations.euler_from_quaternion(quaternion_from_msg(tf_msg))


def transform_from_msg(tf_msg):
    """Extract transformation matrix from geomety_msg/TransformStamed message."""
    T = tf_conversions.transformations.quaternion_matrix(quaternion_from_msg(tf_msg))
    T[:3, -1] = position_from_msg(tf_msg)
    return T

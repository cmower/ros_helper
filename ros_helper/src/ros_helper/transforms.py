import numpy
import tf_conversions

from .config import parse_filename


def load_tf(filename, fmt='4x4'):
    """Load transform from a file."""

    # Load data as 4x4 transform matrix
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
    else:
        raise ValueError("transform did not conform to any recognized shapes, options: (4, 4), (3, 3), (3,), (4,), (7,)")

    # Convert tf to user desired format
    if fmt == '4x4':
        return tf
    elif fmt == '3x3':
        return tf[:3, :3]

    try:
        fmt = int(fmt)
    except ValueError(f"bad format ({fmt})"):
        pass

    if fmt == 3:
        tf = tf[:3,-1].flatten()
    elif fmt == 4:
        tf = tf_conversions.transformations.quaternion_from_matrix(tf)
    elif fmt == 7:
        p =  tf[:3,-1].flatten()
        q = tf_conversions.transformations.quaternion_from_matrix(tf)
        tf = numpy.zeros(7)
        tf[:3] = p
        tf[3:] = q

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

def rotation_from_msg(tf_msg):
    """Extract rotation matrix from geomety_msg/TransformStamed message."""
    return tf_conversions.transformations.quaternion_matrix(quaternion_from_msg(tf_msg))[:3, :3]

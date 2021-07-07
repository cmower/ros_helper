from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import JointState

def transform_stamped(baseid, childid, t, p, q=[0, 0, 0, 1]):
    tf = TransformStamped(child_frame_id=childid)
    tf.header.frame_id = baseid
    tf.header.stamp = t
    tf.transform.translation = Vector3(x=p[0], y=p[1], z=p[2])
    tf.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return tf

def joint_state(t, n=[], p=[], v=[], e=[]):
    msg = JointState(name=n, position=p, velocity=v, effort=e)
    msg.header.stamp = t
    return msg


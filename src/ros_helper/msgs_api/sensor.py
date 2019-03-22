from sensor_msgs.msg import JointState

class JointStateMsg(JointState):

    def __init__(self, t, q, qd=None, eff=None, names=None):
        super(JointStateMsg, self).__init__()
        self.header.stamp = t
        self.position = q
        if names is not None: self.name = names
        if qd is not None: self.velocity = qd
        if eff is not None: self.effort = eff

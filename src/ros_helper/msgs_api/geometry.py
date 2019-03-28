from geometry_msgs.msg import Point, TransformStamped

XYZ = ['x', 'y', 'z']
XYZW = ['x', 'y', 'z', 'w']

class PointMsg(Point):

    def __init__(self, pos):
        super(PointMsg, self).__init__()
        for d, p in zip(XYZ, pos): setattr(self, d, p)

class TransformStampedMsg(TransformStamped):

    def __init__(self, t, trans, quat, child_frame_id, base_frame_id):
        super(TransformStampedMsg, self).__init__()
        self.header.stamp = t
        self.header.frame_id = base_frame_id
        self.child_frame_id = child_frame_id
        for d, p in zip(XYZ, trans): setattr(self.transform.translation, d, p)
        for d, q in zip(XYZW, quat): setattr(self.transform.rotation, d, q)

    @staticmethod
    def get_trans_quat(tr):
        return [getattr(tr.transform.translation, d) for d in XYZ], [getattr(tr.transform.rotation, d) for d in XYZW]

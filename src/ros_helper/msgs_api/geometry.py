from geometry_msgs.msg import Point

XYZ = ['x', 'y', 'z']

class PointMsg(Point):

    def __init__(self, pos):
        super(PointMsg, self).__init__()
        for d, p in zip(XYZ, pos): setattr(self, d, p)

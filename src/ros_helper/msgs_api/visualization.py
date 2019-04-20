from visualization_msgs.msg import Marker, MarkerArray

RGB = ['r', 'g', 'b']
RGBA = ['r', 'g', 'b', 'a']
XYZ = ['x', 'y', 'z']
XYZW = ['x', 'y', 'z', 'w']

class MarkerMsg(Marker):

    def __init__(self, marker_type, time, frame_id):
        super(MarkerMsg, self).__init__()
        self.header.stamp = time
        self.header.frame_id = frame_id
        self.id = 0
        self.type = marker_type
        self.action = Marker.ADD
        self.pose.orientation.w = 1

    def add_id(self, i):
        self.id = i

    def add_action(self, a):
        self.action = a

    def add_position(self, pos):
        for d, p in zip(XYZ, pos): setattr(self.pose.position, d, p)

    def add_orientation(self, ori):
        for d, o in zip(XYZW, ori): setattr(self.pose.orientation, d, o)
        
    def append_point(self, pt):
        self.points.append(pt)

    def add_scale(self, sc):
        for d, s in zip(XYZ, sc): setattr(self.scale, d, s)

    def add_mesh(self, filename, use_embedded_materials=False):
        self.mesh_use_embedded_materials = use_embedded_materials
        self.mesh_resource = filename

    def add_color(self, c):
        self.color = c

    def add_rgb(self, col):
        for d, c in zip(['r', 'g', 'b'], col): setattr(self.color, d, c)
        self.add_alpha(1.0)

    def add_rgba(self, col):
        for d, c in zip(['r', 'g', 'b', 'a'], col): setattr(self.color, d, c)

    def add_alpha(self, a):
        self.color.a = a

class MarkerArrayMsg(MarkerArray):

    def __init__(self, ms=None):
        super(MarkerArrayMsg, self).__init__()
        if type(ms) is list:
            self.markers = ms
            self.resolve_ids()
            
    def append(self, m):
        self.markers.append(m)

    def resolve_ids(self):
        for i, m in enumerate(self.markers): m.id = i

    @property
    def num_markers(self):
        return len(self.markers)

    def add_time(self, i, t):
        self.markers[i].header.stamp = t
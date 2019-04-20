from visualization_msgs.msg import Marker, MarkerArray
from .utils import *

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
    	_msetter(self.pose.position, XYZ, pos)

    def add_orientation(self, ori):
    	_msetter(self.pose.orientation, XYZW, ori)
        
    def append_point(self, pt):
        self.points.append(pt)

    def add_scale(self, sc):
    	_msetter(self.scale, XYZ, sc)

    def add_mesh(self, filename, use_embedded_materials=False):
        self.mesh_use_embedded_materials = use_embedded_materials
        self.mesh_resource = filename

    def add_color(self, c):
        self.color = c

    def add_rgb(self, col):
    	_msetter(self.color, RGB, col)
        self.add_alpha(1.0)

    def add_rgba(self, col):
    	_msetter(self.color, RGBA, col)

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
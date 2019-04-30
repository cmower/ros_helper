from visualization_msgs.msg import Marker, MarkerArray
from .geometry import PointMsg
import numpy as np
from .utils import *

__all__= ['MarkerMsg', 'StlMeshMsg', 'CubeMsg', 'SphereMsg', 'CylinderMsg', 'MarkerArrayMsg', 'LineStripMsg', 'SphereListMsg']

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

    def add_time(self, t):
        self.header.stamp = t

    def add_action(self, a):
        self.action = a

    def add_position(self, pos):
    	_msetter(self.pose.position, XYZ, pos)

    def add_orientation(self, ori):
    	_msetter(self.pose.orientation, XYZW, ori)
        
    def append_point(self, pt):
        self.points.append(pt)

    def add_points(self, pts):
        self.points = pts

    def add_scale(self, sc):
    	_msetter(self.scale, XYZ, sc)

    def add_mesh(self, filename, use_embedded_materials=False):
        self.mesh_use_embedded_materials = use_embedded_materials
        self.mesh_resource = filename

    def add_color(self, c):
        if type(c) is list or type(c) is tuple:
            if len(c) == 3:
                _msetter(self.color, RGB, c)
                self.add_alpha(1.0)
            elif len(c) == 4:
                _msetter(self.color, RGBA, c)
        else:
            # Assume given color is std_msgs.msg.ColorRGBA
            self.color = c

    def add_alpha(self, a):
        self.color.a = a

class StlMeshMsg(MarkerMsg):

    def __init__(self, filename, time, frame_id, scale, color):
        super(StlMeshMsg, self).__init__(Marker.MESH_RESOURCE, time, frame_id)
        self.add_mesh(filename, False)
        self.add_scale(scale)
        self.add_color(color)
        
class SphereMsg(MarkerMsg):
    
    def __init__(self, time, radius, frame_id, color, pos=None):
        super(SphereMsg, self).__init__(Marker.SPHERE, time, frame_id)
        self.add_scale([2.0 * radius]*3) 
        self.add_color(color)
        if pos is not None: self.add_position(pos)

class CubeMsg(MarkerMsg):

    def __init__(self, time, length, width, height, frame_id, color, pos=None, ori=None):
        super(CubeMsg, self).__init__(Marker.CUBE, time, frame_id)
        self.add_scale([length, width, height])
        self.add_color(color)
        if pos is not None: self.add_position(pos)
        if ori is not None: self.add_orientation(ori)

class CylinderMsg(MarkerMsg):

    def __init__(self, time, radius, height, frame_id, color, pos=None, ori=None):
        super(CylinderMsg, self).__init__(Marker.CYLINDER, time, frame_id)
        self.add_scale([2.0*radius, 2.0*radius, height])
        self.add_color(color)
        if pos is not None: self.add_position(pos)
        if ori is not None: self.add_orientation(ori)

class LineStripMsg(MarkerMsg):

    def __init__(self, time, frame_id, line_width, color, points=None):
        super(LineStripMsg, self).__init__(Marker.LINE_STRIP, time, frame_id)
        self.add_color(color)
        self.add_scale([line_width, 0, 0])
        if type(points) is list:
            # assume list of Point msgs
            self.add_points(points)
        elif type(points) is np.ndarray:
            # assume n-by-3
            for p in points: self.append_point(PointMsg(p))

class SphereListMsg(MarkerMsg):

    def __init__(self, time, frame_id, radius, color, points=None):
        super(SphereListMsg, self).__init__(Marker.SPHERE_LIST, time, frame_id)
        self.add_color(color)
        self.add_scale([2.0 * radius]*3)
        if type(points) is list:
            # assume list of Point msgs
            self.add_points(points)
        elif type(points) is np.ndarray:
            # assume n-by-3
            for p in points: self.append_point(PointMsg(p))
            
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

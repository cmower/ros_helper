from visualization_msgs.msg import Marker, MarkerArray
from .geometry import PointMsg
import numpy as np
from .utils import *

__all__= ['MarkerMsg', 'StlMeshMsg', 'CubeMsg', 'SphereMsg', 'CylinderMsg', 'MarkerArrayMsg', 'LineStripMsg', 'SphereListMsg']

class MarkerMsg(Marker):

    def __init__(self, marker_type=None, time=None, frame_id=None, color=None, position=None, orientation=None, scale=None):
        super(MarkerMsg, self).__init__()
        if marker_type is not None: self.add_marker_type(marker_type)
        if time is not None: self.add_time(time)
        if frame_id is not None: self.add_frame_id(frame_id)
        if color is not None: self.add_color(color)
        if position is not None: self.add_position(position)
        if orientation is not None:
            self.add_orientation(orientation)
        else:
            # Orientation orientation is now same as base 
            # frame, user can always update using add_orientation
            self.pose.orientation.w = 1

        # Add other params that are sometimes forgotten, user can always update later
        self.add_id(0)
        self.add_action(Marker.ADD)

    def add_marker_type(self, t):
        self.type = t

    def add_frame_id(self, frame_id):
        self.frame_id = frame_id

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
        typec = type(c)
        if typec is list or typec is tuple:
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

    def __init__(self, time=None, frame_id=None, color=None, position=None, orientation=None, scale=None, filename=None):
        super(StlMeshMsg, self).__init__(Marker.MESH_RESOURCE, time, frame_id, color, position, orientation, scale)
        if filename is not None: self.add_stl_mesh(filename)

    def add_stl_mesh(self, filename):
        self.add_mesh(filename, False)
        
class SphereMsg(MarkerMsg):
    
    def __init__(self, time=None, frame_id=None, color=None, position=None, scale = None, radius=None, diameter=None):
        super(SphereMsg, self).__init__(Marker.SPHERE, time, frame_id, color, position, None, scale)
        if radius is not None: self.add_radius(radius)
        if diameter is not None: self.add_diameter(diameter)

    def add_radius(self, r):
        self.add_diameter(2.0*r)

    def add_diameter(self, d):
        self.add_scale([d]*3)

class ArrowMsg(MarkerMsg):
    
    def __init__(self, time=None, frame_id=None, color=None, scale=None, start_pt=None, end_pt=None, shaft_radius=None, head_radius=None, shaft_diameter=None, head_diameter=None, head_length=None):
        super(ArrowMsg, self).__init__(Marker.ARROW, time, frame_id, color, None, None, scale)
        self.points = [None, None]
        if start_pt is not None: self.add_start_pt(start_pt)
        if end_pt is not None: self.add_end_pt(end_pt)
        if shaft_radius is not None: self.add_shaft_diameter(2.0 * shaft_radius)
        if head_radius is not None: self.add_head_diameter(2.0 * head_diameter)
        if shaft_diameter is not None: self.add_shaft_diameter(shaft_diameter)
        if head_diameter is not None: self.add_head_diameter(head_diameter)
        if head_length is not None: self.add_head_length(head_length)
    
    def add_start_pt(self, p):
        self.points[0] = p

    def add_end_pt(self, p):
        self.points[1] = p

    def add_start_and_end_points(self, ps):
        self.add_start_pt(ps[0])
        self.add_end_pt(ps[1])

    def add_shaft_diameter(self, d):
        self.scale.x = d

    def add_head_diameter(self, d):
        self.scale.y = d

    def add_head_length(self, l):
        self.scale.z = l
        
class CubeMsg(MarkerMsg):

    def __init__(self, time=None, frame_id=None, color=None, position=None, orientation=None, scale=None, length=None, width=None, height=None):
        super(CubeMsg, self).__init__(Marker.CUBE, time, frame_id, color, position, orientation, scale)
        if length is not None: self.add_length(length)
        if width is not None: self.add_width(width)
        if height is not None: self.add_height(height)

    def add_length(self, l):
        self.scale.x = l

    def add_width(self, w):
        self.scale.y = w

    def add_height(self, h):
        self.scale.z = h

class CylinderMsg(MarkerMsg):

    def __init__(self, time=None, frame_id=None, color=None, position=None, orientation=None, scale=None, radius=None, diameter=None, height=None)
        super(CylinderMsg, self).__init__(Marker.CYLINDER, time, frame_id, color, position, orientation, scale)
        if radius is not None: self.add_radius(radius)
        if diameter is not None: self.add_diameter(diameter)
        if height is not None: self.add_height(height)

    def add_radius(r):
        self.add_diameter(2.0*r)

    def add_diameter(d):
        self.scale.x = self.scale.y = d

    def add_height(h):
        self.scale.z = h

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

    def __init__(self, markers=None, nmarkers=None):
        super(MarkerArrayMsg, self).__init__()
        resolve_ids = False
        if markers is not None:
            self.markers = markers
            resolve_ids = True
        if nmarkers is not None:
            self.markers = [MarkerMsg()]*nmarkers
            resolve_ids = True
        if resolve_ids: self.resolve_ids()
            
    def __setitem__(self, i, m):
        self.markers[i] = m

    def __getitem__(self, i):
        return self.markers[i]
            
    def append(self, m):
        self.markers.append(m)

    def resolve_ids(self):
        for i, m in enumerate(self.markers): m.id = i

    @property
    def nmarkers(self):
        return len(self.markers)

    def add_time(self, i, t):
        self.markers[i].header.stamp = t        

from visualization_msgs.msg import Marker, MarkerArray
from .color import ColorMsg
from .geometry import *
import numpy as np
from .utils import *

__all__= ['MarkerMsg', 'StlMeshMsg', 'CubeMsg', 'SphereMsg', 'CylinderMsg', 'MarkerArrayMsg', 'LineStripMsg', 'SphereListMsg']

class MarkerMsg(Marker):

    def __init__(self, marker_type, **kwargs):
        super(MarkerMsg, self).__init__()
        
        if type(marker_type) in [MarkerMsg, Marker]:
            # Make self a copy of given marker
            m = marker_type
            self.header = m.header
            self.ns = m.ns
            self.id = m.id
            self.type = m.type
            self.action = m.action
            self.pose = m.pose
            self.scale = m.scale
            self.color = m.color
            self.lifetime = m.lifetime
            self.frame_locked = m.frame_locked
            self.points = m.points
            self.colors = m.colors
            self.text = m.text
            self.mesh_resource = m.mesh_resource
            self.mesh_use_embedded_materials = m.mesh_use_embedded_materials
        else:
            # assumes marker_type specifiesas a marker type as listed in Object Types section here: http://wiki.ros.org/rviz/DisplayTypes/Marker#Object_Types
            self.marker_type = marker_type
        self.parse_kwargs(kwargs)

    def parse_kwargs(self, kwargs):
        for key, value in kwargs.items():
            if key == 'scale':
                self.scale = Vector3Msg(value)
            elif key == 'color':
                self.color = ColorMsg(value)
            elif key == 'colors':
                self.colors = map(ColorMsg, value)
            elif key == 'points':
                self.points = map(PointMsg, value)
            else:
                setattr(self, key, value)

    #
    # Generic marker properties
    #

    @property
    def namespace(self):
        return self.ns

    @namespace.setter
    def namespace(self, n):
        self.ns = n

    @property
    def frame_id(self):
        return self.header.frame_id

    @frame_id.setter
    def frame_id(self, frame_id):
        self.header.frame_id = frame_id

    @property
    def marker_type(self):
        return self.type

    @marker_type.setter
    def marker_type(self, marker_type):
        self.type = marker_type

    @property
    def time(self):
        return self.header.stamp

    @time.setter
    def time(self, time):
        self.header.stamp = time

    @property
    def position(self):
        return PointMsg.as_np(self.pose.position)

    @position.setter
    def position(self, position):
        self.pose.position = PointMsg(position)

    @property
    def orientation(self):
        return QuaternionMsg.as_np(self.pose.orientation)

    @orientation.setter
    def orientation(self, o):
        self.pose.orientation = QuaternionMsg(o)

    @property
    def alpha(self):
        return self.color.a

    @alpha.setter
    def alpha(self, a):
        self.color.a = a

    #
    # Properties for a text_view_facing marker.
    #

    @property
    def fontsize(self):
        return self.scale.z

    @fontsize.setter
    def fontsize(self, fs):
        assert self.marker_type is Marker.TEXT_VIEW_FACING, "Marker needs to be a TEXT_VIEW_FACING to set a fontsize."
        self.scale.z = fs

    #
    # Properties for a mesh marker
    #

    @property
    def filename(self):
        return self.mesh_resource

    @filename.setter
    def filename(self, filename):
        assert self.marker_type is Marker.MESH_RESOURCE, "Marker needs to be a MESH_RESOURCE to set a filename."
        self.mesh_resource = filename

    #
    # Properties for a sphere/sphere_list/cylinder marker
    #

    @property
    def diameter(self):
        return self.scale.x
        
    @diameter.setter
    def diameter(self, d):
        if self.marker_type is Marker.SPHERE:
            self.scale = Vector3Msg([d]*3)
        elif self.marker_type is Marker.CYLINDER:
            self.scale.x = self.scale.y = d
        elif self.marker_type is Marker.SPHERE_LIST:
            self.scale = Vector3Msg([d]*3)
        else:
            raise TypeError("Marker needs to be a SPHERE, CYLINDER, or SPHERE_LIST to set a diameter.")

    @property
    def radius(self):
        return self.diameter/2.0

    @radius.setter
    def radius(self, r):
        self.diameter = 2.0*r

    #
    # Properties for an arrow marker
    #

    @property
    def start_point(self):
        return self.points[0]

    @start_point.setter
    def start_point(self, s):
        assert self.marker_type is Marker.ARROW, "Marker needs to be an ARROW to set start_point."
        self.points[0] = PointMsg(s)

    @property
    def end_point(self):
        return self.points[1]

    @end_point.setter
    def end_point(self, e):
        assert self.marker_type is Marker.ARROW, "Marker needs to be an ARROW to set end_point."
        self.points[1] = PointMsg(e)

    @property
    def shaft_diameter(self):
        return self.scale.x

    @shaft_diameter.setter
    def shaft_diameter(self, d):
        assert self.marker_type is Marker.ARROW, "Marker needs to be an ARROW to set shaft_diameter."
        self.scale.x = d

    @property
    def head_diameter(self):
        return self.scale.y

    @head_diameter.setter
    def head_diameter(self, d):
        assert self.marker_type is Marker.ARROW, "Marker needs to be an ARROW to set head_diameter."
        self.scale.y = d

    @property
    def head_length(self):
        return self.scale.z

    @head_length.setter
    def head_length(self, l):
        assert self.marker_type is Marker.ARROW, "Marker needs to be an ARROW to set head_length."
        self.scale.z = l

    @property
    def shaft_radius(self):
        return self.shaft_diameter/2.0

    @shaft_radius.setter
    def shaft_radius(self, r):
        self.shaft_diameter = 2.0 * r

    @property
    def head_radius(self):
        return self.head_diameter/2.0

    @head_radius.setter
    def head_radius(self, r):
        self.head_diameter = 2.0*r

    #
    # Properties for a cube/cube_list marker
    #

    @property
    def length(self):
        return self.scale.x

    @length.setter
    def length(self, l):
        assert self.marker_type in [Marker.CUBE, Marker.CUBE_LIST], "Marker needs to be either a CUBE or CUBE_LIST to set length."
        self.scale.x = l

    @property
    def width(self):
        return self.scale.y

    @width.setter
    def width(self, w):
        assert self.marker_type in [Marker.CUBE, Marker.CUBE_LIST], "Marker needs to be either a CUBE or CUBE_LIST to set width."
        self.scale.y = w

    @property
    def height(self):
        return self.scale.z

    @height.setter
    def height(self, h):
        assert self.marker_type in [Marker.CUBE, Marker.CUBE_LIST, Marker.CYLINDER], "Marker needs to be either a CUBE, CUBE_LIST or CYLINDER to set height."
        self.scale.z = h

    #
    # Other methods
    #

    def append_point(self, p):
        assert self.marker_type in [Marker.SPHERE_LIST, Marker.CUBE_LIST, Marker.POINTS, self.TRIANGLE_LIST], "Marker needs to be either a SPHERE_LIST, CUBE_LIST, POINTS, TRIANGLE_LIST to append points."
        self.points.append(PointMsg(p))

    def append_color(self, c):
        assert self.marker_type in [Marker.SPHERE_LIST, Marker.CUBE_LIST, Marker.POINTS, self.TRIANGLE_LIST], "Marker needs to be either a SPHERE_LIST, CUBE_LIST, POINTS, or TRIANGLE_LIST to append colors."
        self.colors.append(ColorMsg(c))

    def append_point_and_color(self, p, c):
        self.append_point(p)
        self.append_color(c)

class StlMeshMsg(MarkerMsg):

    def __init__(self, **kwargs):
        super(StlMeshMsg, self).__init__(Marker.MESH_RESOURCE)
        self.parse_kwargs(kwargs)
        
class SphereMsg(MarkerMsg):
    
    def __init__(self, **kwargs):
        super(SphereMsg, self).__init__(Marker.SPHERE)
        self.parse_kwargs(kwargs)

class ArrowMsg(MarkerMsg):
    
    def __init__(self, **kwargs):
        super(ArrowMsg, self).__init__(Marker.ARROW)
        self.points = [None, None]
        self.parse_kwargs(kwargs)
        
class CubeMsg(MarkerMsg):

    def __init__(self, **kwargs):
        super(CubeMsg, self).__init__(Marker.CUBE)
        self.parse_kwargs(kwargs)

class CylinderMsg(MarkerMsg):

    def __init__(self, **kwargs):
        super(CylinderMsg, self).__init__(Marker.CYLINDER)
        self.parse_kwargs(kwargs)

class LineStripMsg(MarkerMsg):

    def __init__(self, **kwargs):
        super(LineStripMsg, self).__init__(Marker.LINE_STRIP)
        self.parse_kwargs(kwargs)

class SphereListMsg(MarkerMsg):

    def __init__(self, **kwargs):
        super(SphereListMsg, self).__init__(Marker.SPHERE_LIST)
        self.parse_kwargs(kwargs)
            
class MarkerArrayMsg(MarkerArray):

    def __init__(self, markers):
        super(MarkerArrayMsg, self).__init__()
        if type(markers) in [MarkerArrayMsg, MarkerArray]:
            ms = markers.markers
        else:
            ms = markers
        self.markers = map(MarkerMsg, ms)
        self.resolve_ids()
        self._iter = 0

    def __iter__(self):
        return self

    def next(self):
        if self._iter < self.nmarkers:
            i = self._iter
            self._iter += 1
            return self[i]
        else:
            self._iter = 0
            raise StopIteration

    @property
    def nmarkers(self):
        return len(self)

    def __getitem__(self, i):
        return self.markers[i]

    def __setitem__(self, i, m):
        self.markers[i] = m

    def __len__(self):
        return len(self.markers)
        
    def append(self, m):
        self.markers.append(m)
        self.resolve_ids()

    def resolve_ids(self):
        for i, m in enumerate(self): m.id = i

    def add_time(self, t):
        for m in self: m.time = t

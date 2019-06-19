from  rospy.rostime import Time
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from .std import ColorMsg
from .geometry import Vector3Msg, QuaternionMsg, PointMsg, Point, Quaternion

class MarkerMsg(Marker):

    def __init__(self, marker_type, **kwargs):
        super(MarkerMsg, self).__init__()
        
        if type(marker_type) in [MarkerMsg,\
                                 Marker,\
                                 SphereListMsg,\
                                 LineStripMsg,\
                                 CylinderMsg,\
                                 CubeMsg,\
                                 ArrowMsg,\
                                 StlMeshMsg]:
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
    def namespace(self, ns):
        assert type(ns) is str, "Namespace must be a string!"
        self.ns = ns

    @property
    def frame_id(self):
        return self.header.frame_id

    @frame_id.setter
    def frame_id(self, frame_id):
        assert type(frame_id) is str, "Frame ID must be a string!"
        self.header.frame_id = frame_id

    @property
    def marker_type(self):
        return self.type

    @marker_type.setter
    def marker_type(self, marker_type):
        assert type(marker_type) is int, "Marker type must be an integer!"
        assert 0 <= marker_type <= 11, "Marker type must be in range [0, 11]. See http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html"
        self.type = marker_type

    @property
    def time(self):
        return self.header.stamp

    @time.setter
    def time(self, time):
        assert type(time) is Time, "Time must be a rospy.rostime.Time"
        self.header.stamp = time

    @property
    def position(self):
        return PointMsg.as_np(self.pose.position)

    @position.setter
    def position(self, position):
        assert type(position) in [np.ndarray, list, tuple, PointMsg, Point], "Position must either be a list, tuple, numpy array, a PointMsg, or Point."
        self.pose.position = PointMsg(position)

    @property
    def orientation(self):
        return QuaternionMsg.as_np(self.pose.orientation)

    @orientation.setter
    def orientation(self, orientation):
        assert type(orientation) in [np.ndarray, list, tuple, QuaternionMsg, Quaternion], "Orientation must either be a list, tuple, numpy array, a QuaternionMsg, or Quaternion."
        self.pose.orientation = QuaternionMsg(orientation)

    @property
    def alpha(self):
        return self.color.a

    @alpha.setter
    def alpha(self, a):
        assert type(a) is float, "Alpha must be a float."
        assert 0 <= a <= 1.0, "Alpha must be in range [0, 1]."
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
        assert type(fs) in [float, int], "Fontsize must be either a float or int."
        assert fs > 0.0, "Fontsize must be positive."
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
        assert type(filename) is str, "Filename must be a string."
        self.mesh_resource = filename

    #
    # Properties for a sphere/sphere_list/cylinder marker
    #

    @property
    def diameter(self):
        return self.scale.x
        
    @diameter.setter
    def diameter(self, d):
        assert type(d) is float, "Diameter must be a float."
        assert d > 0.0, "Diameter must be positive."
        if self.marker_type in [Marker.SPHERE, Marker.SPHERE_LIST]:
            self.scale.x = self.scale.y = self.scale.z = d
        elif self.marker_type is Marker.CYLINDER:
            self.scale.x = self.scale.y = d
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
        assert type(s) in [list, tuple, np.ndarray, PointMsg, Point], "Start point must be either a list, tuple, numpy array, PointMsg, or Point."
        self.points[0] = PointMsg(s)

    @property
    def end_point(self):
        return self.points[1]

    @end_point.setter
    def end_point(self, e):
        assert self.marker_type is Marker.ARROW, "Marker needs to be an ARROW to set end_point."
        assert type(e) in [list, tuple, np.ndarray, PointMsg, Point], "End point must be either a list, tuple, numpy array, PointMsg, or Point."
        self.points[1] = PointMsg(e)

    @property
    def shaft_diameter(self):
        return self.scale.x

    @shaft_diameter.setter
    def shaft_diameter(self, d):
        assert self.marker_type is Marker.ARROW, "Marker needs to be an ARROW to set shaft_diameter."
        assert type(d) is float, "Shaft diameter must be a float."
        assert d > 0.0, "Shaft diameter must be positive."
        self.scale.x = d

    @property
    def head_diameter(self):
        return self.scale.y

    @head_diameter.setter
    def head_diameter(self, d):
        assert self.marker_type is Marker.ARROW, "Marker needs to be an ARROW to set head_diameter."
        assert type(d) is float, "Head diameter must be a float."
        assert d > 0.0, "Head diameter must be positive."
        self.scale.y = d

    @property
    def head_length(self):
        return self.scale.z

    @head_length.setter
    def head_length(self, l):
        assert self.marker_type is Marker.ARROW, "Marker needs to be an ARROW to set head_length."
        assert type(l) is float, "Head length must be a float."
        assert l > 0.0, "Head length must be positive."        
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
    # Properties for a cube/cube_list marker, note that height is apart of cylinder too.
    #

    @property
    def length(self):
        return self.scale.x

    @length.setter
    def length(self, l):
        assert self.marker_type in [Marker.CUBE, Marker.CUBE_LIST], "Marker needs to be either a CUBE or CUBE_LIST to set length."
        assert type(l) is float, "Length must be a float."
        assert l > 0.0, "Length must be positive."
        self.scale.x = l

    @property
    def width(self):
        return self.scale.y

    @width.setter
    def width(self, w):
        assert self.marker_type in [Marker.CUBE, Marker.CUBE_LIST], "Marker needs to be either a CUBE or CUBE_LIST to set width."
        assert type(w) is float, "Width must be a float."
        assert w > 0.0, "Width must be positive."        
        self.scale.y = w

    @property
    def height(self):
        return self.scale.z

    @height.setter
    def height(self, h):
        assert self.marker_type in [Marker.CUBE, Marker.CUBE_LIST, Marker.CYLINDER], "Marker needs to be either a CUBE, CUBE_LIST or CYLINDER to set height."
        assert type(h) is float, "Height must be a float."
        assert h > 0.0, "Height must be positive."                
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

    def __init__(self, markers, time=None):
        super(MarkerArrayMsg, self).__init__()
        self.markers = map(MarkerMsg, markers if type(markers) not in [MarkerArrayMsg, MarkerArray] else markers.markers)
        if time is not None: self.time = time
        self.resolve_ids()

    def __iter__(self):
        return iter(self.markers)

    @property
    def time(self):
        assert self.nmarkers > 0, "At least one marker is required to get the time attribute."
        return self[0].time

    @time.setter
    def time(self, t):
        for m in self: m.time = t

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

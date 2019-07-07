from visualization_msgs.msg import *
from .std import ColorMsg
from .geometry import Vector3Msg, QuaternionMsg, PointMsg, Point, Quaternion
from ..utils import get_object_class_hierarchy

#
# Msg classes
#

class ImageMarkerMsg(ImageMarker):

    def __init__(self):
        super(ImageMarkerMsg, self).__init__()
        raise NotImplementedError("ImageMarkerMsg is not yet implemented")

class InteractiveMarkerMsg(InteractiveMarker):

    def __init__(self):
        super(InteractiveMarkerMsg, self).__init__()
        raise NotImplementedError("InteractiveMarkerMsg is not yet implemented")

class InteractiveMarkerControlMsg(InteractiveMarkerControl):

    def __init__(self):
        super(InteractiveMarkerControlMsg, self).__init__()
        raise NotImplementedError("InteractiveMarkerControlMsg is not yet implemented")

class InteractiveMarkerFeedbackMsg(InteractiveMarkerFeedback):

    def __init__(self):
        super(InteractiveMarkerFeedbackMsg, self).__init__()
        raise NotImplementedError("InteractiveMarkerFeedbackMsg is not yet implemented")

class InteractiveMarkerInitMsg(InteractiveMarkerInit):

    def __init__(self):
        super(InteractiveMarkerInitMsg, self).__init__()
        raise NotImplementedError("InteractiveMarkerInitMsg is not yet implemented")

class InteractiveMarkerPoseMsg(InteractiveMarkerPose):

    def __init__(self):
        super(InteractiveMarkerPoseMsg, self).__init__()
        raise NotImplementedError("InteractiveMarkerPoseMsg is not yet implemented")

class InteractiveMarkerUpdateMsg(InteractiveMarkerUpdate):

    def __init__(self):
        super(InteractiveMarkerUpdateMsg, self).__init__()
        raise NotImplementedError("InteractiveMarkerUpdateMsg is not yet implemented")

class MarkerMsg(Marker):

    def __init__(self, marker_type, **kwargs):
        super(MarkerMsg, self).__init__()

        if Marker in get_object_class_hierarchy(marker_type):
            # marker_type is a marker -> make self a copy of given marker
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
            # assumes marker_type specifies a marker type as listed in Object Types section here: http://wiki.ros.org/rviz/DisplayTypes/Marker#Object_Types
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

    def append_point(self, p):
        self.points.append(PointMsg(p))

    def append_color(self, c):
        self.colors.append(ColorMsg(c))

    #
    # Generic marker properties
    #

    @property
    def rgba(self):
        return self.color

    @rgba.setter
    def rgba(self, c):
        self.color = ColorMsg(c)

    @property
    def rgb(self):
        return self.color.rgb

    @rgb.setter
    def rgb(self, c):
        self.color = ColorMsg(c)

    @property
    def rgba(self):
        return self.color.rgba

    @rgba.setter
    def rgba(self, c):
        self.color = ColorMsg(c)

    @property
    def namespace(self):
        return self.ns

    @namespace.setter
    def namespace(self, ns):
        self.ns = ns

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
        return PointMsg(self.pose.position).to_np()

    @position.setter
    def position(self, position):
        self.pose.position = PointMsg(position)

    @property
    def orientation(self):
        return QuaternionMsg(self.pose.orientation).to_np()

    @orientation.setter
    def orientation(self, orientation):
        self.pose.orientation = QuaternionMsg(orientation)

    @property
    def alpha(self):
        return self.color.a

    @alpha.setter
    def alpha(self, a):
        self.color.a = float(a)

class MarkerArrayMsg(MarkerArray):

    def __init__(self, markers, time=None):
        super(MarkerArrayMsg, self).__init__()
        if MarkerArray in get_object_class_hierarchy(markers):
            # Given a marker array or MarkerArrayMsg -> make self a copy of markers
            markers_ = markers.markers
        else:
            # Assumes given a list of markers
            markers_ = markers
        self.markers = map(MarkerMsg, markers_)
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

class MenuEntryMsg(MenuEntry):

    def __init__(self):
        super(MenuEntryMsg, self).__init__()
        raise NotImplementedError("MenuEntryMsg is not yet implemented")

#
# Helper super classes for additional derived classes below
#

class _LineMsg(MarkerMsg):

    def __init__(self, marker_type):
        super(_LineMsg, self).__init__(marker_type)

    @property
    def line_width(self):
        return self.scale.x

    @line_width.setter
    def line_width(self, w):
        self.scale.x = float(w)

class _CubeMsg(MarkerMsg):

    def __init__(self, marker_type):
        super(_CubeMsg, self).__init__(marker_type)

    @property
    def length(self):
        return self.scale.x

    @length.setter
    def length(self, l):
        self.scale.x = float(l)

    @property
    def width(self):
        return self.scale.y

    @width.setter
    def width(self, w):
        self.scale.y = float(w)

    @property
    def height(self):
        return self.scale.z

    @height.setter
    def height(self, h):
        self.scale.z = float(h)

class _SphereMsg(MarkerMsg):

    def __init__(self, marker_type):
        super(_SphereMsg, self).__init__(marker_type)

    @property
    def diameter(self):
        return self.scale.x

    @diameter.setter
    def diameter(self, d):
        self.scale.x = self.scale.y = self.scale.z = float(d)

    @property
    def radius(self):
        return self.diameter/2.0

    @radius.setter
    def radius(self, r):
        self.diameter = 2.0*float(r)

# ARROW = 0

class ArrowMsg(MarkerMsg):

    def __init__(self, **kwargs):
        super(ArrowMsg, self).__init__(Marker.ARROW)
        self.points = [PointMsg(), PointMsg()]
        self.parse_kwargs(kwargs)

    @property
    def start_point(self):
        return self.points[0]

    @start_point.setter
    def start_point(self, s):
        self.points[0] = PointMsg(s)

    @property
    def end_point(self):
        return self.points[1]

    @end_point.setter
    def end_point(self, e):
        self.points[1] = PointMsg(e)

    @property
    def shaft_diameter(self):
        return self.scale.x

    @shaft_diameter.setter
    def shaft_diameter(self, d):
        self.scale.x = float(d)

    @property
    def head_diameter(self):
        return self.scale.y

    @head_diameter.setter
    def head_diameter(self, d):
        self.scale.y = float(d)

    @property
    def head_length(self):
        return self.scale.z

    @head_length.setter
    def head_length(self, l):
        self.scale.z = float(l)

    @property
    def shaft_radius(self):
        return self.shaft_diameter/2.0

    @shaft_radius.setter
    def shaft_radius(self, r):
        self.shaft_diameter = 2.0 * float(r)

    @property
    def head_radius(self):
        return self.head_diameter/2.0

    @head_radius.setter
    def head_radius(self, r):
        self.head_diameter = 2.0*float(r)

# CUBE = 1

class CubeMsg(_CubeMsg):

    def __init__(self, **kwargs):
        super(CubeMsg, self).__init__(Marker.CUBE)
        self.parse_kwargs(kwargs)

# SPHERE = 2

class SphereMsg(_SphereMsg):

    def __init__(self, **kwargs):
        super(SphereMsg, self).__init__(Marker.SPHERE)
        self.parse_kwargs(kwargs)

# CYLINDER = 3

class CylinderMsg(MarkerMsg):

    def __init__(self, **kwargs):
        super(CylinderMsg, self).__init__(Marker.CYLINDER)
        self.parse_kwargs(kwargs)

    @property
    def diameter(self):
        return self.scale.x

    @diameter.setter
    def diameter(self, d):
        self.scale.x = self.scale.y = float(d)

    @property
    def height(self):
        return self.scale.z

    @height.setter
    def height(self, h):
        self.scale.z = float(h)

# LINE_STRIP = 4

class LineStripMsg(_LineMsg):

    def __init__(self, **kwargs):
        super(LineStripMsg, self).__init__(Marker.LINE_STRIP)
        self.parse_kwargs(kwargs)

# LINE_LIST = 5

class LineListMsg(_LineMsg):

    def __init__(self, **kwargs):
        super(LineListMsg, self).__init__(Marker.LINE_LIST)
        self.parse_kwargs(kwargs)

# CUBE_LIST = 6

class CubeListMsg(_CubeMsg):

    def __init__(self, **kwargs):
        super(CubeListMsg, self).__init__(Marker.CUBE_LIST)
        self.parse_kwargs(kwargs)

# SPHERE_LIST = 7

class SphereListMsg(_SphereMsg):

    def __init__(self, **kwargs):
        super(SphereListMsg, self).__init__(Marker.SPHERE_LIST)
        self.parse_kwargs(kwargs)

# POINTS = 8

class PointsMsg(MarkerMsg):

    def __init__(self, **kwargs):
        super(PointsMsg, self).__init__(Marker.POINTS)
        self.parse_kwargs(kwargs)

    @property
    def width(self):
        return self.scale.x

    @width.setter
    def width(self, w):
        self.scale.x = float(w)

    @property
    def height(self):
        return self.scale.y

    @height.setter
    def height(self, h):
        self.scale.y = float(h)

# TEXT_VIEW_FACING = 9

class TextMsg(MarkerMsg):

    def __init__(self, **kwargs):
        super(TextMsg, self).__init__(Marker.TEXT_VIEW_FACING)
        self.parse_kwargs(kwargs)

    @property
    def fontsize(self):
        return self.scale.z

    @fontsize.setter
    def fontsize(self, fs):
        self.scale.z = float(fs)

# MESH_RESOURCE = 10
#  Note: currently, only STL messages are supported by ros_helper

class StlMeshMsg(MarkerMsg):

    def __init__(self, **kwargs):
        super(StlMeshMsg, self).__init__(Marker.MESH_RESOURCE)
        self.parse_kwargs(kwargs)
        self.mesh_use_embedded_materials = False

    @property
    def filename(self):
        return self.mesh_resource

    @filename.setter
    def filename(self, filename):
        if type(filename) in [list, tuple]: filename = "package://%s/%s" % (filename[0], filename[1]) # allow user to specify package_name, filename (where filename is path within package)
        self.mesh_resource = filename

# TRIANGLE_LIST = 11

class TriangleList(MarkerMsg):

    def __init__(self, **kwargs):
        super(TriangleList, self).__init__(Marker.TRIANGLE_LIST)
        self.parse_kwargs(kwargs)

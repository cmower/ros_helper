from sensor_msgs.msg import *
from ros_helper.msg import MultiJoy, Keyboard
from ..utils import *
import numpy as np

# Keyboard keys, ord : pygame key id
keyboard_keys = {\
                 48 : 'K_0',\
                 49 : 'K_1',\
                 50 : 'K_2',\
                 51 : 'K_3',\
                 52 : 'K_4',\
                 53 : 'K_5',\
                 54 : 'K_6',\
                 55 : 'K_7',\
                 56 : 'K_8',\
                 57 : 'K_9',\
                 38 : 'K_AMPERSAND',\
                 42 : 'K_ASTERISK',\
                 64 : 'K_AT',\
                 96 : 'K_BACKQUOTE',\
                 92 : 'K_BACKSLASH',\
                 8 : 'K_BACKSPACE',\
                 318 : 'K_BREAK',\
                 301 : 'K_CAPSLOCK',\
                 94 : 'K_CARET',\
                 12 : 'K_CLEAR',\
                 58 : 'K_COLON',\
                 44 : 'K_COMMA',\
                 127 : 'K_DELETE',\
                 36 : 'K_DOLLAR',\
                 274 : 'K_DOWN',\
                 279 : 'K_END',\
                 61 : 'K_EQUALS',\
                 27 : 'K_ESCAPE',\
                 321 : 'K_EURO',\
                 33 : 'K_EXCLAIM',\
                 282 : 'K_F1',\
                 291 : 'K_F10',\
                 292 : 'K_F11',\
                 293 : 'K_F12',\
                 294 : 'K_F13',\
                 295 : 'K_F14',\
                 296 : 'K_F15',\
                 283 : 'K_F2',\
                 284 : 'K_F3',\
                 285 : 'K_F4',\
                 286 : 'K_F5',\
                 287 : 'K_F6',\
                 288 : 'K_F7',\
                 289 : 'K_F8',\
                 290 : 'K_F9',\
                 0 : 'K_FIRST',\
                 62 : 'K_GREATER',\
                 35 : 'K_HASH',\
                 315 : 'K_HELP',\
                 278 : 'K_HOME',\
                 277 : 'K_INSERT',\
                 256 : 'K_KP0',\
                 257 : 'K_KP1',\
                 258 : 'K_KP2',\
                 259 : 'K_KP3',\
                 260 : 'K_KP4',\
                 261 : 'K_KP5',\
                 262 : 'K_KP6',\
                 263 : 'K_KP7',\
                 264 : 'K_KP8',\
                 265 : 'K_KP9',\
                 267 : 'K_KP_DIVIDE',\
                 271 : 'K_KP_ENTER',\
                 272 : 'K_KP_EQUALS',\
                 269 : 'K_KP_MINUS',\
                 268 : 'K_KP_MULTIPLY',\
                 266 : 'K_KP_PERIOD',\
                 270 : 'K_KP_PLUS',\
                 308 : 'K_LALT',\
                 323 : 'K_LAST',\
                 306 : 'K_LCTRL',\
                 276 : 'K_LEFT',\
                 91 : 'K_LEFTBRACKET',\
                 40 : 'K_LEFTPAREN',\
                 60 : 'K_LESS',\
                 310 : 'K_LMETA',\
                 304 : 'K_LSHIFT',\
                 311 : 'K_LSUPER',\
                 319 : 'K_MENU',\
                 45 : 'K_MINUS',\
                 313 : 'K_MODE',\
                 300 : 'K_NUMLOCK',\
                 281 : 'K_PAGEDOWN',\
                 280 : 'K_PAGEUP',\
                 19 : 'K_PAUSE',\
                 46 : 'K_PERIOD',\
                 43 : 'K_PLUS',\
                 320 : 'K_POWER',\
                 316 : 'K_PRINT',\
                 63 : 'K_QUESTION',\
                 39 : 'K_QUOTE',\
                 34 : 'K_QUOTEDBL',\
                 307 : 'K_RALT',\
                 305 : 'K_RCTRL',\
                 13 : 'K_RETURN',\
                 275 : 'K_RIGHT',\
                 93 : 'K_RIGHTBRACKET',\
                 41 : 'K_RIGHTPAREN',\
                 309 : 'K_RMETA',\
                 303 : 'K_RSHIFT',\
                 312 : 'K_RSUPER',\
                 302 : 'K_SCROLLOCK',\
                 59 : 'K_SEMICOLON',\
                 47 : 'K_SLASH',\
                 32 : 'K_SPACE',\
                 317 : 'K_SYSREQ',\
                 9 : 'K_TAB',\
                 95 : 'K_UNDERSCORE',\
                 0 : 'K_UNKNOWN',\
                 273 : 'K_UP',\
                 97 : 'K_a',\
                 98 : 'K_b',\
                 99 : 'K_c',\
                 100 : 'K_d',\
                 101 : 'K_e',\
                 102 : 'K_f',\
                 103 : 'K_g',\
                 104 : 'K_h',\
                 105 : 'K_i',\
                 106 : 'K_j',\
                 107 : 'K_k',\
                 108 : 'K_l',\
                 109 : 'K_m',\
                 110 : 'K_n',\
                 111 : 'K_o',\
                 112 : 'K_p',\
                 113 : 'K_q',\
                 114 : 'K_r',\
                 115 : 'K_s',\
                 116 : 'K_t',\
                 117 : 'K_u',\
                 118 : 'K_v',\
                 119 : 'K_w',\
                 120 : 'K_x',\
                 121 : 'K_y',\
                 122 : 'K_z'\
}

#
# Msg classes
#

class BatteryStateMsg(BatteryState):

    def __init__(self):
        super(BatteryStateMsg, self).__init__()
        raise NotImplementedError("BatteryStateMsg is not yet implemented")

class CameraInfoMsg(CameraInfo):

    def __init__(self):
        super(CameraInfoMsg, self).__init__()
        raise NotImplementedError("CameraInfoMsg is not yet implemented")

class ChannelFloat32Msg(ChannelFloat32):

    def __init__(self):
        super(ChannelFloat32Msg, self).__init__()
        raise NotImplementedError("ChannelFloat32Msg is not yet implemented")

class CompressedImageMsg(CompressedImage):

    def __init__(self):
        super(CompressedImageMsg, self).__init__()
        raise NotImplementedError("CompressedImageMsg is not yet implemented")

class FluidPressureMsg(FluidPressure):

    def __init__(self):
        super(FluidPressureMsg, self).__init__()
        raise NotImplementedError("FluidPressureMsg is not yet implemented")

class IlluminanceMsg(Illuminance):

    def __init__(self):
        super(IlluminanceMsg, self).__init__()
        raise NotImplementedError("IlluminanceMsg is not yet implemented")

class ImageMsg(Image):

    def __init__(self):
        super(ImageMsg, self).__init__()
        raise NotImplementedError("ImageMsg is not yet implemented")

class ImuMsg(Imu):

    def __init__(self):
        super(ImuMsg, self).__init__()
        raise NotImplementedError("ImuMsg is not yet implemented")

class JointStateMsg(JointState):

    def __init__(self, position, velocity=None, effort=None, name=None, time=None):
        super(JointStateMsg, self).__init__()
        if JointState in get_object_class_hierarchy(position):
            js = time
            self.header = js.header
            self.name = js.name
            self.position = js.position
            self.velocity = js.velocity
            self.effort = js.effort
        else:
            self.position = position
            if velocity is not None: self.velocity = velocity
            if effort is not None: self.effort = effort
            if name is not None: self.name = name
            if time is not None: self.time = time

    def __getitem__(self, i):
        return [self.name[i], self.position[i], self.velocity[i], self.effort[i]]

    def __setitem(self, i, state):
        self.name[i] = state[0]
        self.position[i] = state[1]
        self.velocity[i] = state[2]
        self.effort[i] = state[3]

    def __len__(self):
        return len(self.position)

    @property
    def time(self):
        return self.header.stamp

    @time.setter
    def time(self, t):
        self.header.stamp = t

    @property
    def njoints(self):
        return len(self)

class JoyMsg(Joy):

    def __init__(self, axes=[], buttons=[], time=None):
        super(JoyMsg, self).__init__()
        if Joy in get_object_class_hierarchy(axes):
            j = axes
            self.header = j.header
            self.axes = j.axes
            self.buttons = j.buttons
        else:
            self.axes = axes
            self.buttons = buttons
            if time is not None: self.time = time

    @property
    def Buttons(self):
        return np.asarray(self.buttons)

    @property
    def Axes(self):
        return np.asarray(self.axes)

    @property
    def time(self):
        return self.header.stamp

    @time.setter
    def time(self, t):
        self.header.stamp = t

    @property
    def nbuttons(self):
        return len(self.buttons)

    @property
    def naxes(self):
        return len(self.naxes)

class JoyFeedbackMsg(JoyFeedback):

    def __init__(self):
        super(JoyFeedbackMsg, self).__init__()
        raise NotImplementedError("JoyFeedbackMsg is not yet implemented")

class JoyFeedbackArrayMsg(JoyFeedbackArray):

    def __init__(self):
        super(JoyFeedbackArrayMsg, self).__init__()
        raise NotImplementedError("JoyFeedbackArrayMsg is not yet implemented")

class LaserEchoMsg(LaserEcho):

    def __init__(self):
        super(LaserEchoMsg, self).__init__()
        raise NotImplementedError("LaserEchoMsg is not yet implemented")

class LaserScanMsg(LaserScan):

    def __init__(self):
        super(LaserScanMsg, self).__init__()
        raise NotImplementedError("LaserScanMsg is not yet implemented")

class MagneticFieldMsg(MagneticField):

    def __init__(self):
        super(MagneticFieldMsg, self).__init__()
        raise NotImplementedError("MagneticFieldMsg is not yet implemented")

class MultiDOFJointStateMsg(MultiDOFJointState):

    def __init__(self):
        super(MultiDOFJointStateMsg, self).__init__()
        raise NotImplementedError("MultiDOFJointStateMsg is not yet implemented")

class MultiEchoLaserScanMsg(MultiEchoLaserScan):

    def __init__(self):
        super(MultiEchoLaserScanMsg, self).__init__()
        raise NotImplementedError("MultiEchoLaserScanMsg is not yet implemented")

class NavSatFixMsg(NavSatFix):

    def __init__(self):
        super(NavSatFixMsg, self).__init__()
        raise NotImplementedError("NavSatFixMsg is not yet implemented")

class NavSatStatusMsg(NavSatStatus):

    def __init__(self):
        super(NavSatStatusMsg, self).__init__()
        raise NotImplementedError("NavSatStatusMsg is not yet implemented")

class PointCloudMsg(PointCloud):

    def __init__(self):
        super(PointCloudMsg, self).__init__()
        raise NotImplementedError("PointCloudMsg is not yet implemented")

class PointCloud2Msg(PointCloud2):

    def __init__(self):
        super(PointCloud2Msg, self).__init__()
        raise NotImplementedError("PointCloud2Msg is not yet implemented")

class PointFieldMsg(PointField):

    def __init__(self):
        super(PointFieldMsg, self).__init__()
        raise NotImplementedError("PointFieldMsg is not yet implemented")

class RangeMsg(Range):

    def __init__(self):
        super(RangeMsg, self).__init__()
        raise NotImplementedError("RangeMsg is not yet implemented")

class RegionOfInterestMsg(RegionOfInterest):

    def __init__(self):
        super(RegionOfInterestMsg, self).__init__()
        raise NotImplementedError("RegionOfInterestMsg is not yet implemented")

class RelativeHumidityMsg(RelativeHumidity):

    def __init__(self):
        super(RelativeHumidityMsg, self).__init__()
        raise NotImplementedError("RelativeHumidityMsg is not yet implemented")

class TemperatureMsg(Temperature):

    def __init__(self):
        super(TemperatureMsg, self).__init__()
        raise NotImplementedError("TemperatureMsg is not yet implemented")

class TimeReferenceMsg(TimeReference):

    def __init__(self):
        super(TimeReferenceMsg, self).__init__()
        raise NotImplementedError("TimeReferenceMsg is not yet implemented")

#
# Additional message classes
#

class KeyboardMsg(Keyboard):

    def __init__(self, time=None, **kwargs):
        super(KeyboardMsg, self).__init__()
        if Keyboard in get_object_class_hierarchy(time):
            kb = time
            self.header = kb.header
            for k in [k for k in dir(kb) if k.startswith('K')]: setattr(self, k, getattr(kb, k))
        else:
            if time is not None: self.time = time
            for key, value in kwargs.items(): self[key] = value

    @property
    def time(self):
        return self.header.stamp

    @time.setter
    def time(self, time):
        self.header.stamp = time

    def __setitem__(self, idx, val):
        setattr(self, KeyboardMsg._idx_to_key(idx), val)

    def __getitem__(self, idx):
        return getattr(self, KeyboardMsg._idx_to_key(idx))

    @staticmethod
    def _idx_to_key(idx):
        if type(idx) is int:
            k = keyboard_keys[idx]
        elif type(idx) is str:
            k = idx
        else:
            raise "[ERROR] given idx can only be a int or str."
        return k

class MultiJoyMsg(MultiJoy):

    def __init__(self, time, joys=None):
        super(MultiJoyMsg, self).__init__()
        if type(time) in [MultiJoyMsg, MultiJoy]:
            mj = time
            self.header = mj.header
            self.njoys = mj.njoys
            self.joys = mj.joys
        else:
            # Assumes joys is not None
            self.header.stamp = time
            self.njoys = len(joys)
            self.joys = joys

class LogitechF710JoyMsg(JoyMsg):

    def __init__(self, axes=[], buttons=[], time=None, green_light_on=False):
        super(LogitechF710JoyMsg, self).__init__(axes, buttons, time)

        # Setup button mappings
        if not green_light_on:
            self.AXIS_L_HORI = 0
            self.AXIS_L_VERT = 1
            self.AXIS_R_HORI = 3
            self.AXIS_R_VERT = 4
            self.AXIS_LT = 2
            self.AXIS_RT = 5
            self.AXIS_D_PAD_HORI = 6
            self.AXIS_D_PAD_VERT = 7
        else:
            self.AXIS_L_HORI = 6
            self.AXIS_L_VERT = 7
            self.AXIS_R_HORI = 3
            self.AXIS_R_VERT = 4
            self.AXIS_LT = 2
            self.AXIS_RT = 5
            self.AXIS_D_PAD_HORI = 0
            self.AXIS_D_PAD_VERT = 1

        self.BUTTON_A = 0
        self.BUTTON_B = 1
        self.BUTTON_X = 2
        self.BUTTON_Y = 3
        self.BUTTON_LB = 4
        self.BUTTON_RB = 5
        self.BUTTON_BACK = 6
        self.BUTTON_START = 7
        self.BUTTON_LOGITECH = 8
        self.BUTTON_L3 = 9
        self.BUTTON_R3 = 10

    @property
    def axis_left_hori(self):
        return self.axes[self.AXIS_L_HORI]

    @property
    def axis_left_vert(self):
        return self.axes[self.AXIS_L_VERT]

    @property
    def axis_right_hori(self):
        return self.axes[self.AXIS_R_HORI]

    @property
    def axis_right_vert(self):
        return self.axes[self.AXIS_R_VERT]

    @property
    def axis_lt(self):
        return self.axes[self.AXIS_LT]

    @property
    def axis_rt(self):
        return self.axes[self.AXIS_RT]

    @property
    def axis_d_pad_hori(self):
        return self.axes[self.AXIS_D_PAD_HORI]

    @property
    def axis_d_pad_vert(self):
        return self.axes[self.AXIS_D_PAD_VERT]

    @property
    def button_a(self):
        return self.buttons[self.BUTTON_A]

    @property
    def button_b(self):
        return self.buttons[self.BUTTON_B]

    @property
    def button_x(self):
        return self.buttons[self.BUTTON_X]

    @property
    def button_y(self):
        return self.buttons[self.BUTTON_Y]

    @property
    def button_lb(self):
        return self.buttons[self.BUTTON_LB]

    @property
    def button_rb(self):
        return self.buttons[self.BUTTON_RB]

    @property
    def button_back(self):
        return self.buttons[self.BUTTON_BACK]

    @property
    def button_start(self):
        return self.buttons[self.BUTTON_START]

    @property
    def button_logitech(self):
        return self.buttons[self.BUTTON_LOGITECH]

    @property
    def button_l3(self):
        return self.buttons[self.BUTTON_L3]

    @property
    def button_r3(self):
        return self.buttons[self.BUTTON_R3]

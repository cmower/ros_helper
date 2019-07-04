import numpy as np
from std_msgs.msg import *
import colors
from ..rh_utils import msetattr, mgetattr, get_object_class_hierarchy

class BoolMsg(Bool):

    def __init__(self, d=False):
        super(BoolMsg, self).__init__()
        self.data = d.data if Bool in get_object_class_hierarchy(d) else bool(d)

class Float32Msg(Float32):

    def __init__(self, d=0.0):
        super(Float32Msg, self).__init__()
        self.data = d.data if Float32 in get_object_class_hierarchy(d) else np.float32(d)

class Float64Msg(Float64):

    def __init__(self, d=0.0):
        super(Float64Msg, self).__init__()
        self.data = d.data if Float64 in get_object_class_hierarchy(d) else np.float64(d)

class Int8Msg(Int8):

    def __init__(self, d=0):
        super(Int8Msg, self).__init__()
        self.data = d.data if Int8 in get_object_class_hierarchy(d) else np.int8(d)

class Int16Msg(Int16):

    def __init__(self, d=0):
        super(Int16sSg, self).__init__()
        self.data = d.data if Int16 in get_object_class_hierarchy(d) else np.int16(d)

class Int32Msg(Int32):

    def __init__(self, d=0):
        super(Int32Msg, self).__init__()
        self.data = d.data if Int32 in get_object_class_hierarchy(d) else np.int32(d)

class Int64Msg(Int64):

    def __init__(self, d=0):
        super(Int64Msg, self).__init__()
        self.data = d.data if Int64 in get_object_class_hierarchy(d) else np.int64(d)

class UInt8Msg(UInt8):

    def __init__(self, d=0):
        super(UInt8Msg, self).__init__()
        self.data = d.data if UInt8 in get_object_class_hierarchy(d) else np.uint8(d)

class UInt16Msg(UInt16):

    def __init__(self, d=0):
        super(UInt16sSg, self).__init__()
        self.data = d.data if Uint16 in get_object_class_hierarchy(d) else np.uint16(d)


class UInt32Msg(UInt32):

  def __init__(self, d=0):
      super(UInt32Msg, self).__init__()
      self.data = d.data if Uint32 in get_object_class_hierarchy(d) else np.uint32(d)

class UInt64Msg(UInt64):

  def __init__(self, d=0):
      super(UInt64Msg, self).__init__()
      self.data = d.data if UInt64 in get_object_class_hierarchy(d) else np.uint64(d)

class ColorMsg(ColorRGBA):

    __RGB = ['r', 'g', 'b']
    __RGBA = ['r', 'g', 'b', 'a']

    def __init__(self, c=[0, 0, 0, 1]):
        super(ColorMsg, self).__init__()
        if ColorRGBA in get_object_class_hierarchy(c):
            msetattr(self, self.__RGBA, mgetattr(c, self.__RGBA))
        else:
            # assumes input is numpy array or list or tuple
            n = len(c)
            if n == 3:
	        self.rgb = c
	        self.alpha = 1.0 # help user by ensuring they remember to set a=1, can always be changed later.
            elif n == 4:
	        self.rgba = c
            else:
	        raise TypeError("Given c is incorrect length.") # not sure if a better error type should be used

    @property
    def rgb(self):
        return mgetattr(self, self.__RGB)

    @rgb.setter
    def rgb(self, rgb):
        msetattr(self, self.__RGB, rgb)

    @property
    def rgba(self):
        return mgetattr(self, self.__RGBA)

    @rgba.setter
    def rgba(self, rgba):
        msetattr(self, self.__RGBA, rgba)

    @property
    def alpha(self):
        return self.a

    @alpha.setter
    def alpha(self, a):
        self.a = a

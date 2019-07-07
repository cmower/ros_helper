import numpy as np
from std_msgs.msg import *
import colors
from ..utils import msetattr, mgetattr, get_object_class_hierarchy

class BoolMsg(Bool):

    def __init__(self, d=False):
        super(BoolMsg, self).__init__()
        self.data = d.data if Bool in get_object_class_hierarchy(d) else bool(d)

class ByteMsg(Byte):

    def __init__(self):
        super(ByteMsg, self).__init__()
        raise NotImplementedError("ByteMsg is not yet implemented")

class ByteMultiArrayMsg(ByteMultiArray):

    def __init__(self):
        super(ByteMultiArrayMsg, self).__init__()
        raise NotImplementedError("ByteMultiArrayMsg is not yet implemented")

class CharMsg(Char):

    def __init__(self):
        super(CharMsg, self).__init__()
        raise NotImplementedError("CharMsg is not yet implemented")

class ColorRGBAMsg(ColorRGBA):

    def __init__(self):
        super(ColorRGBAMsg, self).__init__()
        raise NotImplementedError("ColorRGBAMsg is implemented as ColorMsg")

class DurationMsg(Duration):

    def __init__(self):
        super(DurationMsg, self).__init__()
        raise NotImplementedError("DurationMsg is not yet implemented")

class EmptyMsg(Empty):

    def __init__(self):
        super(EmptyMsg, self).__init__()
        raise NotImplementedError("EmptyMsg is not yet implemented")

class Float32Msg(Float32):

    def __init__(self, d=0.0):
        super(Float32Msg, self).__init__()
        self.data = d.data if Float32 in get_object_class_hierarchy(d) else np.float32(d)

class Float32MultiArrayMsg(Float32MultiArray):

    def __init__(self):
        super(Float32MultiArrayMsg, self).__init__()
        raise NotImplementedError("Float32MultiArrayMsg is not yet implemented")

class Float64Msg(Float64):

    def __init__(self, d=0.0):
        super(Float64Msg, self).__init__()
        self.data = d.data if Float64 in get_object_class_hierarchy(d) else np.float64(d)

class Float64MultiArrayMsg(Float64MultiArray):

    def __init__(self):
        super(Float64MultiArrayMsg, self).__init__()
        raise NotImplementedError("Float64MultiArrayMsg is not yet implemented")

class HeaderMsg(Header):

    def __init__(self):
        super(HeaderMsg, self).__init__()
        raise NotImplementedError("HeaderMsg is not yet implemented")


class Int16Msg(Int16):

    def __init__(self, d=0):
        super(Int16sSg, self).__init__()
        self.data = d.data if Int16 in get_object_class_hierarchy(d) else np.int16(d)

class Int16MultiArrayMsg(Int16MultiArray):

    def __init__(self):
        super(Int16MultiArrayMsg, self).__init__()
        raise NotImplementedError("Int16MultiArrayMsg is not yet implemented")

class Int32Msg(Int32):

    def __init__(self, d=0):
        super(Int32Msg, self).__init__()
        self.data = d.data if Int32 in get_object_class_hierarchy(d) else np.int32(d)

class Int32MultiArrayMsg(Int32MultiArray):

    def __init__(self):
        super(Int32MultiArrayMsg, self).__init__()
        raise NotImplementedError("Int32MultiArrayMsg is not yet implemented")

class Int64Msg(Int64):

    def __init__(self, d=0):
        super(Int64Msg, self).__init__()
        self.data = d.data if Int64 in get_object_class_hierarchy(d) else np.int64(d)

class Int64MultiArrayMsg(Int64MultiArray):

    def __init__(self):
        super(Int64MultiArrayMsg, self).__init__()
        raise NotImplementedError("Int64MultiArrayMsg is not yet implemented")

class Int8Msg(Int8):

    def __init__(self, d=0):
        super(Int8Msg, self).__init__()
        self.data = d.data if Int8 in get_object_class_hierarchy(d) else np.int8(d)

class Int8MultiArrayMsg(Int8MultiArray):

    def __init__(self):
        super(Int8MultiArrayMsg, self).__init__()
        raise NotImplementedError("Int8MultiArrayMsg is not yet implemented")

class MultiArrayDimensionMsg(MultiArrayDimension):

    def __init__(self):
        super(MultiArrayDimensionMsg, self).__init__()
        raise NotImplementedError("MultiArrayDimensionMsg is not yet implemented")

class MultiArrayLayoutMsg(MultiArrayLayout):

    def __init__(self):
        super(MultiArrayLayoutMsg, self).__init__()
        raise NotImplementedError("MultiArrayLayoutMsg is not yet implemented")

class StringMsg(String):

    def __init__(self):
        super(StringMsg, self).__init__()
        raise NotImplementedError("StringMsg is not yet implemented")

class TimeMsg(Time):

    def __init__(self):
        super(TimeMsg, self).__init__()
        raise NotImplementedError("TimeMsg is not yet implemented")

class UInt16Msg(UInt16):

    def __init__(self, d=0):
        super(UInt16sSg, self).__init__()
        self.data = d.data if Uint16 in get_object_class_hierarchy(d) else np.uint16(d)

class UInt16MultiArrayMsg(UInt16MultiArray):

    def __init__(self):
        super(UInt16MultiArrayMsg, self).__init__()
        raise NotImplementedError("UInt16MultiArrayMsg is not yet implemented")

class UInt32Msg(UInt32):

  def __init__(self, d=0):
      super(UInt32Msg, self).__init__()
      self.data = d.data if Uint32 in get_object_class_hierarchy(d) else np.uint32(d)

class UInt32MultiArrayMsg(UInt32MultiArray):

    def __init__(self):
        super(UInt32MultiArrayMsg, self).__init__()
        raise NotImplementedError("UInt32MultiArrayMsg is not yet implemented")

class UInt64Msg(UInt64):

  def __init__(self, d=0):
      super(UInt64Msg, self).__init__()
      self.data = d.data if UInt64 in get_object_class_hierarchy(d) else np.uint64(d)

class UInt64MultiArrayMsg(UInt64MultiArray):

    def __init__(self):
        super(UInt64MultiArrayMsg, self).__init__()
        raise NotImplementedError("UInt64MultiArrayMsg is not yet implemented")

class UInt8Msg(UInt8):

    def __init__(self, d=0):
        super(UInt8Msg, self).__init__()
        self.data = d.data if UInt8 in get_object_class_hierarchy(d) else np.uint8(d)

class UInt8MultiArrayMsg(UInt8MultiArray):

    def __init__(self):
        super(UInt8MultiArrayMsg, self).__init__()
        raise NotImplementedError("UInt8MultiArrayMsg is not yet implemented")

#
# Additional msg classes
#

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

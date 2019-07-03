import numpy as np
from std_msgs.msg import *
import colors
from ..rh_utils import msetattr, mgetattr, get_object_class_hierarchy

RGB = ['r', 'g', 'b']
RGBA = ['r', 'g', 'b', 'a']

class BoolMsg(Bool):

  def __init__(self, d):
    super(BoolMsg, self).__init__()
    self.data = d if type(d) not in [BoolMsg, Bool] else d.data

class Float32Msg(Float32):

  def __init__(self, d):
    super(Float32Msg, self).__init__()
    self.data = d if type(d) not in [Float32Msg, Float32] else d.data

class Float64Msg(Float64):

  def __init__(self, d):
    super(Float64Msg, self).__init__()
    self.data = d if type(d) not in [Float64Msg, Float64] else d.data

class Int8Msg(Int8):

  def __init__(self, d):
    super(Int8Msg, self).__init__()
    self.data = d if type(d) not in [Int8Msg, Int8] else d.data

class Int16Msg(Int16):

  def __init__(self, d):
    super(Int16sSg, self).__init__()
    self.data = d if type(d) not in [Int16Msg, Int16] else d.data

class Int32Msg(Int32):

  def __init__(self, d):
    super(Int32Msg, self).__init__()
    self.data = d if type(d) not in [Int32Msg, Int32] else d.data

class Int64Msg(Int64):

  def __init__(self, d):
    super(Int64Msg, self).__init__()
    self.data = d if type(d) not in [Int64Msg, Int64] else d.data


class UInt8Msg(UInt8):

  def __init__(self, d):
    super(UInt8Msg, self).__init__()
    self.data = d if type(d) not in [UInt8Msg, UInt8] else d.data

class UInt16Msg(UInt16):

  def __init__(self, d):
    super(UInt16sSg, self).__init__()
    self.data = d if type(d) not in [UInt16Msg, UInt16] else d.data

class UInt32Msg(UInt32):

  def __init__(self, d):
    super(UInt32Msg, self).__init__()
    self.data = d if type(d) not in [UInt32Msg, UInt32] else d.data

class UInt64Msg(UInt64):

  def __init__(self, d):
    super(UInt64Msg, self).__init__()
    self.data = d if type(d) not in [UInt64Msg, UInt64] else d.data    

class ColorMsg(ColorRGBA):

    def __init__(self, c):
      super(ColorMsg, self).__init__()
      if ColorRGBA in get_object_class_heirarchy(c):
        msetattr(self, RGBA, mgetattr(c, RGBA))
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
      return mgetattr(self, RGB)

    @rgb.setter
    def rgb(self, rgb):
      msetattr(self, RGB, rgb)

    @property
    def rgba(self):
      return mgetattr(self, RGBA)

    @rgba.setter
    def rgba(self, rgba):
      msetattr(self, RGBA, rgba)

    @property
    def alpha(self):
      return self.a

    @alpha.setter
    def alpha(self, a):
      self.a = a

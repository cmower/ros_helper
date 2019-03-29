from std_msgs.msg import *

class BoolMsg(Bool):

  def __init__(self, d):
	super(BoolMSg, self).__init__()
    self.data = d

class Float32Msg(Float32):

  def __init__(self, d):
    super(Float32MSg, self).__init__()
    self.data = d

class Float64Msg(Float64):

  def __init__(self, d):
    super(Float64MSg, self).__init__()
    self.data = d

class Int8Msg(Int8):

  def __init__(self, d):
    super(Int8MSg, self).__init__()
    self.data = d

class Int16Msg(Int16):

  def __init__(self, d):
    super(Int16MSg, self).__init__()
    self.data = d

class Int32Msg(Int32):

  def __init__(self, d):
    super(Int32MSg, self).__init__()
    self.data = d

class Int64Msg(Int64):

  def __init__(self, d):
    super(Int64MSg, self).__init__()
    self.data = d

class UInt8Msg(UInt8):

  def __init__(self, d):
    super(UInt8MSg, self).__init__()
    self.data = d

class UInt16Msg(UInt16):

  def __init__(self, d):
    super(UInt16MSg, self).__init__()
    self.data = d

class UInt32Msg(UInt32):

  def __init__(self, d):
    super(UInt32MSg, self).__init__()
    self.data = d

class UInt64Msg(UInt64):

  def __init__(self, d):
    super(UInt64MSg, self).__init__()
    self.data = d

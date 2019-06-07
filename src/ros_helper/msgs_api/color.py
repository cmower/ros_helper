from std_msgs.msg import ColorRGBA
from .utils import *
from .color_list import *

__all__ = ['ColorMsg']

class ColorMsg(ColorRGBA):

    def __init__(self, c):
        super(ColorMsg, self).__init__()
        if type(c) in [ColorRGBA, ColorMsg]:
            for d in RGBA: setattr(self, d, getattr(c, d))
        else:
            # assumes input is numpy array or list or tuple
            n = len(c)
            if n == 3:
                dims = RGB
                self.a = 1.0
            elif n == 4:
                dims = RGBA
            else:
                raise TypeError("Given c is incorrect length.") # not sure if a better error type should be used 
            msetattr(self, dims, c)

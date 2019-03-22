from std_msgs.msg import *

class BoolMsg(Bool):

    def __init__(self, b):
        super(BoolMsg, self).__init__()
        self.data = b

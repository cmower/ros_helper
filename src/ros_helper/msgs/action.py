from actionlib_msgs.msg import *
from ..utils import *

class GoalIDMsg(GoalID):

    def __init__(self):
        super(GoalIDMsg, self).__init__()
        raise NotImplementedError("GoalIDMsg is not yet implemented")

    @classmethod
    def open_doc(cls):
        open_msg_doc_in_browser(_msg_group, cls.__name__)

class GoalStatusMsg(GoalStatus):

    def __init__(self):
        super(GoalStatusMsg, self).__init__()
        raise NotImplementedError("GoalStatusMsg is not yet implemented")

    @classmethod
    def open_doc(cls):
        open_msg_doc_in_browser(_msg_group, cls.__name__)

class GoalStatusArrayMsg(GoalStatusArray):

    def __init__(self):
        super(GoalStatusArrayMsg, self).__init__()
        raise NotImplementedError("GoalStatusArrayMsg is not yet implemented")

    @classmethod
    def open_doc(cls):
        open_msg_doc_in_browser(_msg_group, cls.__name__)

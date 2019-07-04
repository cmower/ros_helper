from actionlib_msgs.msgs import *

class GoalIDMsg(GoalID):

    def __init__(self):
        super(GoalIDMsg, self).__init__()
        raise NotImplementedError("GoalIDMsg is not yet implemented")

class GoalStatusMsg(GoalStatus):

    def __init__(self):
        super(GoalStatusMsg, self).__init__()
        raise NotImplementedError("GoalStatusMsg is not yet implemented")

class GoalStatusArrayMsg(GoalStatusArray):

    def __init__(self):
        super(GoalStatusArrayMsg, self).__init__()
        raise NotImplementedError("GoalStatusArrayMsg is not yet implemented")

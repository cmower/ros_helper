from diagnostic_msgs.msg import *

class DiagnosticArrayMsg(DiagnosticArray):

    def __init__(self):
        super(DiagnosticArrayMsg, self).__init__()
        raise NotImplementedError("DiagnosticArrayMsg is not yet implemented")

class DiagnosticStatusMsg(DiagnosticStatus):

    def __init__(self):
        super(DiagnosticStatusMsg, self).__init__()
        raise NotImplementedError("DiagnosticStatusMsg is not yet implemented")

class KeyValueMsg(KeyValue):

    def __init__(self):
        super(KeyValueMsg, self).__init__()
        raise NotImplementedError("KeyValueMsg is not yet implemented")

from diagnostic_msgs.msg import *
from ..utils import *

class DiagnosticArrayMsg(DiagnosticArray):

    def __init__(self):
        super(DiagnosticArrayMsg, self).__init__()
        raise NotImplementedError("DiagnosticArrayMsg is not yet implemented")

    @classmethod
    def open_doc(cls):
        open_msg_doc_in_browser(_msg_group, cls.__name__)

class DiagnosticStatusMsg(DiagnosticStatus):

    def __init__(self):
        super(DiagnosticStatusMsg, self).__init__()
        raise NotImplementedError("DiagnosticStatusMsg is not yet implemented")

    @classmethod
    def open_doc(cls):
        open_msg_doc_in_browser(_msg_group, cls.__name__)

class KeyValueMsg(KeyValue):

    def __init__(self):
        super(KeyValueMsg, self).__init__()
        raise NotImplementedError("KeyValueMsg is not yet implemented")

    @classmethod
    def open_doc(cls):
        open_msg_doc_in_browser(_msg_group, cls.__name__)

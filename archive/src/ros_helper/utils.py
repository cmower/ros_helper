import inspect
import functools
import numpy as np
from math import pi
from subprocess import check_output
import webbrowser

XYZ = ['x', 'y', 'z']
XYZW = ['x', 'y', 'z', 'w']

ros_distro = check_output(['rosversion', '--distro']).splitlines()[0]

def ros_msg_doc_url(msg_group, msg_class):
    return "http://docs.ros.org/%s/api/%s/html/msg/%s.html" % (ros_distro, msg_group, msg_class)

def open_msg_doc_in_browser(msg_group, msg_class):
    webbrowser.open(ros_msg_doc_url(msg_group, msg_class.replace('Msg', '')))

def get_object_class_hierarchy(obj):
    return inspect.getmro(type(obj))

def mgetattr(obj, ids):
    """Multiple gettattr."""
    return [getattr(obj, i) for i in ids]

def msetattr(obj, ids, vals):
    """Multiple setattr."""
    for i, v in zip(ids, vals): setattr(obj, i, v)

def rsetattr(obj, attr, val):
    """Recursive setattr."""
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)

def rgetattr(obj, attr, *args):
    """Recursive getattr."""
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))

class TerminalUtils:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

#
# Evaluation from string utils
#

def eval_str_array(s, dtype=float):
    return np.array(eval(s), dtype=dtype)

def ones(n):
    return np.ones(n)

def ones3():
    return ones(3)

def zeros(n):
    return np.zeros(n)

def zeros3():
    return zeros(3)

def deg(r):
    return np.radians(r)

def array(a, dtype=float):
    return np.array(a, dtype=dtype)

import inspect

XYZ = ['x', 'y', 'z']
XYZW = ['x', 'y', 'z', 'w']

def get_object_class_hierarchy(obj):
    return inspect.getmro(type(obj))

def mgetattr(obj, ids):
    """Multiple gettattr."""
    return [getattr(obj, i) for i in ids]

def msetattr(obj, ids, vals):
    """Multiple setattr."""
    for i, v in zip(ids, vals): setattr(obj, i, v)

class TerminalUtils:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

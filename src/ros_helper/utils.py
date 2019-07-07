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

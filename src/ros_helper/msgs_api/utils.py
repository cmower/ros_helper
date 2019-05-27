
__all__ = ['RGB', 'RGBA', 'XYZ', 'XYZW', 'mgetattr', 'msetattr']

RGB = ['r', 'g', 'b']
RGBA = ['r', 'g', 'b', 'a']
XYZ = ['x', 'y', 'z']
XYZW = ['x', 'y', 'z', 'w']

def mgetattr(obj, ids):
    """Multiple gettattr."""
    return map(lambda i : getattr(obj, i), ids)

def msetattr(obj, ids, vals):
    """Multiple setattr."""
    map(lambda i, v : setattr(obj, i, v), zip(ids, vals))

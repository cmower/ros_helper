
__all__ = ['RGB', 'RGBA', 'XYZ', 'XYZW', '_mgetter', '_msetter']

RGB = ['r', 'g', 'b']
RGBA = ['r', 'g', 'b', 'a']
XYZ = ['x', 'y', 'z']
XYZW = ['x', 'y', 'z', 'w']

def _mgetter(_obj, _ids):
    """Multiple gettattr."""
    return [getattr(_obj, _id) for _id in _ids]

def _msetter(_obj, _ids, _vals):
    """Multiple setattr."""
    for _id, _val in zip(_ids, _vals): setattr(_obj, _id, _val)

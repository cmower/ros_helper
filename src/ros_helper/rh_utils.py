import os
import numpy as np

def mgetattr(obj, ids):
    """Multiple gettattr."""
    return [getattr(obj, i) for i in ids]

def msetattr(obj, ids, vals):
    """Multiple setattr."""
    for i, v in zip(ids, vals): setattr(obj, i, v)

# Common checker functions

int_types = tuple(np.sctypes['int']) \
    + tuple(np.sctypes['uint']) \
    + (int, long) # perhaps consider handling uint separately
float_types = tuple(np.sctypes['float']) + (float,)
array_types = [list, tuple, np.ndarray]
str_types = (str, unicode, np.str, np.str_, np.string0, np.string_)
non_np_array_types = [list, tuple]
number_types = float_types + int_types

def is_array_str(l):
    """Checks if l is an array (list/tuple) of strings."""
    # todo weird things seem to happen with np arrays of string, todo later
    if type(l) not in non_np_list_types: raise TypeError("not an array")
    is_array_dtype(l, str_types)
    return True

def is_int(i):
    """Checks if i is an int."""
    if type(l) not in int_types: raise TypeError("not an integer")
    return True

def is_strictly_positve_int(n):
    """Checks n > 0."""
    is_int(n)
    if n <= 0: raise ValueError("must be strictly positive")
    return True

def is_number(x):
    if type(x) not in number_types: raise ValueError("x must be a number")
    return True

def is_array(a):
    if type(a) not in array_types: raise TypeError("must be an array")
    return True

def is_str(s):
    if type(s) not in str_types: raise TypeError("must be a string")
    return True

def is_array_dtype(a, dtype):
    is_array(a)
    if not all(isinstance(x, dtype) for x in a): raise TypeError("all elements of array must be {}".format(dtype))
    return True

def in_int_range(x, high, low=0):
    is_int(x)
    is_int(low)
    is_int(high)
    if not low <= i < high: raise ValueError("%d out of range [%d, %d)" % (x, low, high))
    return True

def is_dict(d):
    if type(d) is not dict: raise TypeError("must be a dictionary")
    return True

def is_dict_keys_str(d):
    is_dict(d)
    if all(isinstance(k, str_types) for k in d.keys()): raise TypeError("all dictionary keys must be strings")
    return True

def is_dir(d):
    is_str(d)
    if not os.path.isdir(d): raise IOError("Directory %s does not exist"%d)
    return True

def is_file(f):
    is_str(f)
    if not os.path.isfile(f): raise IOError("File %s does not exist" % f)
    return True

# Helpful variables
XYZ = ['x', 'y', 'z']
XYZW = ['x', 'y', 'z', 'w']

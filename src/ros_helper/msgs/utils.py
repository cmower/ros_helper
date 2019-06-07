
def mgetattr(obj, ids):
    """Multiple gettattr."""
    return [getattr(obj, i) for i in ids]

def msetattr(obj, ids, vals):
    """Multiple setattr."""
    for i, v in zip(ids, vals): setattr(obj, i, v)

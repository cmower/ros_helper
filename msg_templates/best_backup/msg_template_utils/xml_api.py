import xml.etree.ElementTree as et

def load_root(filename):
    return et.parse(filename).getroot()

def get_dependancies(root):
    deps = []
    for ros_msg in root:
        for attr in ros_msg:
            if attr.tag == 'RosPropertyCollection':
                for grp in attr.attrib['groups'].split(','):
                    if grp not in deps: deps.append(grp)
            else:
                grp = attr.attrib['group']
                if grp not in deps: deps.append(grp)
    return deps

# Ros msg attribute

def ros_msg_attribute(elem):

    # Extract attributes attributes
    name = elem.attrib['name']
    list_length = elem.attrib['list_length']
    type = elem.attrib['type']
    group = elem.attrib['type']
    default = elem.attrib['default']

# Ros msg property
PROP_NOT_LIST = \
    """    @property
    def %s(self):
        return self.%s

    @%s.setter
    def %s(self, %sin):
        self.%s = %s(%sin)

    """

PROP_LIST = \
    """@property
    def %s(self):
        return self.%s

    @%s.setter
    def %s(self, %sin):
        self.%s = map(%s, %sin)

    """

def ros_msg_property(elem):

    # # Extract property attributes
    name = elem.attrib['name']
    is_list = elem.attrib['is_list']
    target = elem.attrib['target']
    type = elem.attrib['type']

    if is_list=='0':
        # not list
        out = PROP_NOT_LIST
    else:
        # list
        out = PROP_LIST

    return out % (name, taret, name, name, name, target, type, name)

# Ros property collection

PROP_COLLEC = \
    """    %s_TARGS = [%s]
        %s_TARG_TYPES = [%s]

    @property
    def %s(self):
        return [rgetattr(self, targ) for targ in self.%s_TARGS]

    @%s.setter
    def %s(self, %sin):
        for attr, targ, targ_type in zip(%sin, self.%s_TARGS, self.%s._TARGS_TYPES):
            rsetattr(self, targ, targ_type(attr))
    """

def ros_property_collection(elem):

    def parse(attr):
        return

    name = elem.attrib['name']
    NAME = name.upper()
    targets = ', '.join(["'%s'" % t for t in elem.attrib[attr].split(',')])
    target_types = ', '.join(["%s" % t for t in elem.attrib[attr].split(',')])

    return PROP_COLLEC % (NAME, targets, NAME, target_types, name, NAME, name, name, name, name, NAME, NAME)

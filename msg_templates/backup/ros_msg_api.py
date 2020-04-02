#!/usr/bin/env python
import re
import subprocess

def get_all_ros_msg_groups():
    group_names = []
    for line in subprocess.check_output(['rosmsg', 'list']).splitlines():
        group_name = line.split('/')[0]
        if group_name not in group_names:
            group_names.append(group_name)
            yield RosMsgGroup(group_name)

class RosMsgAttribute(object):

    def __init__(self, name, list_length, type, group):
        self.__name = name
        self.__list_length = list_length
        self.__type = type
        self.__group = group

    @property
    def name(self):
        return self.__name

    @property
    def list_length(self):
        return self.__list_length

    @property
    def type(self):
        return self.__type

    @property
    def group(self):
        return self.__group

class RosMsg(object):

    def __init__(self, grp, msg):
        """Initializes Msg class, note built-in type(...) cannot be used in this scope."""

        # Init
        self.__name = msg

        # Load msg lines (note, we split lines and remove trailing empty line)
        msg_lines = subprocess.check_output(['rosmsg','show','%s/%s' % (grp,msg)]).splitlines()[:-1]

        # Trim sub-msg attributes
        msg_lines = [line for line in msg_lines if not line.startswith('  ')]

        # Parse attributes
        ros_msg_attributes = []
        for line in msg_lines:

            # Is a property?
            if '=' in line:
                # skip all properties
                continue

            # Split msg type and name from line
            type, name = line.split(' ')

            # Is list?
            #     Defined here that matters for later:
            #       * is_list (bool) -> true if msg type is a list, false otherwise
            #       * list_length (str) -> indicates length of list (value is always >= -1)
            #             - -1  means msg is not a list,
            #             - 0 means msg is a list of variable length,
            #             - > 0 indicates msg is fixed length and value is the length of the list
            is_list = '[' in type and ']' in type
            list_length = '-1'
            if is_list:
                # list -> is a variable or fixed length?
                is_variable = '[]' in type
                if is_variable:
                    # variable length list -> remove []
                    list_length = '0'
                else:
                    # fixed length -> get list length
                    list_length = type[type.find("[")+1:type.find("]")]

            # Remove [] and enclosed number if exists
            type = re.sub(r'\[[^)]*\]', '', type)

            # Is basic msg type?
            #     Defined here that matters for later:
            #         * is_basic (bool) -> true if msg is a basic type, i.e. one of the following
            #               - int*, uint*,
            #               - float*,
            #               - string,
            #               - time
            is_basic = '/' not in type

            if is_basic:
                # is basic -> trim numbers from type (i.e. 32, or 64) and u for uints
                type = ''.join([c for c in type if not c.isdigit()]) # rm digits
                type = re.sub('uint', 'int', type) # rm u from uints
                group = ''
            else:
                group, type = type.split('/')

            # Append attribute
            ros_msg_attributes.append(RosMsgAttribute(name, list_length, type, group))

        self.__ros_msg_attributes = ros_msg_attributes

    @property
    def name(self):
        return self.__name

    @property
    def ros_msg_attributes(self):
        return self.__ros_msg_attributes

class RosMsgGroup(object):

    def __init__(self, name):

        # Init
        self.__name = name

        # Collect all ros msgs in group
        ros_msgs = []
        for msg_name in self.__ros_msg_names():
            ros_msgs.append(RosMsg(name, msg_name))
        self.__ros_msgs = ros_msgs

    def __ros_msg_names(self):
        for line in subprocess.check_output(['rosmsg', 'list']).splitlines():
            group, msg_name = line.split('/')
            if group == self.__name:
                yield msg_name

    @property
    def name(self):
        return self.__name

    @property
    def ros_msgs(self):
        return self.__ros_msgs

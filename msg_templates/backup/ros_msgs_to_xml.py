"""
This will cycle through all ROS messages on computer and generate a base xml.
"""
import os

from xml.etree import ElementTree
from xml.dom import minidom
from xml.etree.ElementTree import Element, SubElement, Comment, tostring

from msg_template_utils.ros_msg_api import get_all_ros_msg_groups

def prettify(elem):
    """Return a pretty-printed XML string for the Element."""
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

if __name__ == '__main__':

    print "Starting conversion ros msgs -> xml, ..."

    # Iterate over ros msg groups, and convert to xml
    for ros_msg_group in get_all_ros_msg_groups():

        # Init
        group_name = ros_msg_group.name
        filename = "%s/templates/%s_template.xml" % (os.getcwd(), group_name)

        if os.path.isfile(filename):
            user_input = raw_input("  File exists for group %s, overwrite [Y/n]: " % group_name).lower()
            if user_input == '' or user_input == 'y':
                pass
            elif user_input == 'n':
                continue
            else:
                print "Did not recognize input (%s), skipping %s" % (user_input, group_name)
                continue

        print "  [%s] started ..." % group_name,

        # Create root
        root = Element('RosMsgGroup', name=group_name)

        # Iterate over msgs in group
        for ros_msg in ros_msg_group.ros_msgs:

            # Create sub-element of root
            child = SubElement(root, 'RosMsg', name=ros_msg.name)

            # Iterate over ros msg attributes
            for ros_msg_attribute in ros_msg.ros_msg_attributes:
                SubElement(child, 'RosMsgAttribute',\
                           name=ros_msg_attribute.name,\
                           list_length=ros_msg_attribute.list_length,\
                           type=ros_msg_attribute.type,\
                           group=ros_msg_attribute.group)

        # Save xml
        with open(filename, 'w') as f:
            f.write(prettify(root))
        print "complete"

    print "Completed conversion ros msgs -> xml"

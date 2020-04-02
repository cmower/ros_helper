#!/usr/bin/env python
import os
import subprocess

from xml.etree import ElementTree
from xml.dom import minidom
from xml.etree.ElementTree import Element, SubElement, Comment, tostring

def save_xml(filename, root):
    with open(filename, 'w') as f: f.write(minidom.parseString(ElementTree.tostring(root, 'utf-8')).toprettyxml(indent="  "))

def packages():
    pkgs = subprocess.check_output(['rosmsg', 'packages']).splitlines()
    return pkgs, len(pkgs)

def messages(package):
    msgs = [m.split('/')[1] for m in subprocess.check_output(['rosmsg', 'package', package]).splitlines()]
    return msgs, len(msgs)

def attributes(package, message):
    if os.path.exists('temp'): os.remove('temp')
    os.system('rosmsg show %s/%s | grep -v "=" | grep "^[^ ]" >> temp' % (package, message))
    with open('temp', 'r') as f:
        atts = f.read().splitlines()
    os.remove('temp')
    return atts, len(atts)

def islist(attribute):
    return '[' in attribute and ']' in attribute

def isbasic(attribute):
    return '/' not in attribute

def attribute_name(attribute):
    return attribute[attribute.find(' ')+1:]

def attribute_list(attribute):
    if islist(attribute):
        nstr = attribute[attribute.find('[')+1:attribute.find(']')]
        return nstr if nstr.isdigit() else '0'
    else:
        return '-1'

def attribute_package(attribute):
    return attribute[:attribute.find('/')] if not isbasic(attribute) else 'basic'

def attribute_type(attribute):
    return attribute[:attribute.find(' ')].replace('[]', '') if isbasic(attribute) else attribute[attribute.find('/')+1:attribute.find(' ')].replace('[]', '')

if __name__ == '__main__':

    # Setup
    # ..

    # Iterate over packages/messages
    p, np = packages()
    for i, package in enumerate(p):

        print "[%d/%d] Package: %s" % (i+1, np, package)

        root = Element('Package', name=package)

        m, nm = messages(package)
        for j, message in enumerate(m):

            print "..[%d/%d] Message: %s" % (j+1, nm, message)

            msg_child = SubElement(root, 'Message', name=message)

            a, na = attributes(package, message)
            for k, attribute in enumerate(a):

                print "....[%d/%d] Attribute: %s" % (k+1, na, attribute_name(attribute))

                attr_child = SubElement(msg_child, 'Attribute',\
                                        name=attribute_name(attribute),\
                                        package=attribute_package(attribute),\
                                        list=attribute_list(attribute),\
                                        type=attribute_type(attribute))

        fn = os.getcwd() + "/templates/%s.xml" % package
        save_xml(fn, root)
        print "Saved %s template to %s" % (package, fn)

    print "Complete"

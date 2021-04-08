import xml.etree.cElementTree as et
import xml.dom.minidom

class MsgTemplate(object):

    def __init__(self, grp):
        self.root = et.parse('%s.xml'%grp).getroot()

    def msgs(self):
        for msg in self.root:
            yield msg

    def get_type_groups(self):
        type_groups = []
        for msg in self.msgs():
            for msg_attrib in msg.attrib:
                tg = msg_attrib.attrib['type_group']
                if tg not in type_groups and not tg is not 'basic':
                    type_groups.append(tg)
        return type_groups

class MsgsGenerator(object):

    def __init__(self, grp):
        self.msg_template = MsgTemplate(grp)

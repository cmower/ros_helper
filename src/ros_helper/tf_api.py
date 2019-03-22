import tf
import rospy

class Transform(object):

    def __init__(self):
        self.did_recieve = False
        self.trans = None
        self.quat = None
        
    def add_transform(self, trans_quat):
        self.trans = trans_quat[0]
        self.quat = trans_quat[1]
        self.did_recieve = True

class TfApi(object):
    
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.li=tf.TransformListener()
        self.br=tf.TransformBroadcaster()
        
    def get_tf(self, child_frame_id, base_frame_id):
        out_tf = Transform()
        try:
            trans, quat = self.li.lookupTransform(base_frame_id, child_frame_id, self.rospy.Time(0)) # t needs to be rospy.Time(0)
            out_tf.add_transform(trans, quat)
        except:
            print "[{}] I did not recieve anything".format(time.time())
        return out_tf
    
    def set_tf(self, trans, quat, child_frame_id, base_frame_id):
        self.br.sendTransform(trans, quat, self.rospy.Time.now(), child_frame_id, base_frame_id) # t needs to be rospy.Time.now()

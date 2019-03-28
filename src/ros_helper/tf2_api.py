import tf2_ros
from .msgs_api.geometry import TransformStampedMsg

class TfApi(object):
    
    def __init__(self, rospy):
        self.br = tf2_ros.TransformBroadcaster()
        self.buff = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buff)
        self.rospy = rospy
        
    def get_tf(self, child_frame_id, base_frame_id):
        did_recieve = False
        try:
            trans, quat = TransformStampedMsg.get_trans_quat(self.buff.lookup_transform(base_frame_id, child_frame_id, self.rospy.Time()))
            did_recieve = True
        except:
            trans = quat = None
        return trans, quat, did_recieve
    
    def set_tf(self, t, trans, quat, child_frame_id, base_frame_id):
        self.br.sendTransform(TransformStampedMsg(t, trans, quat, child_frame_id, base_frame_id))

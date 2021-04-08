import numpy
import tf2_ros
import tf_conversions
from std_msgs.msg import Int64
from geometry_msg.msg import TransformStamped

class RosNode:

    def __init__(self, rospy):
        """Initialization. Note, child-classes need to make a call to super().__init__(rospy)."""
        self.__rp = rospy
        self.__tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.__tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.__tf_buffer)
        self.subs = {} # Subscribers
        self.pubs = {} # Publishers
        self.srvs = {} # Services
        self.timers = {} # Timers
        self.msgs = {} # Messages

    def getTf(self, base_frame_id, child_frame_id):
        """Returns a transform from tf2 as a geometry_msgs/TransformStamped."""
        tf = None
        try:
            tf = self.__tf_buffer.lookup_transform(base_frame_id, child_frame_id, self.rp.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.__rp.logwarn(f'Did not recover frame {child_frame_id} in the frame {base_frame_id}!')
        return tf

    def positionFromTf2Msg(self, tf):
        return numpy.array([getattr(tf.transform.translation, d) for d in 'xyz'])

    def quaternionFromTf2Msg(self, tf):
        return numpy.array([getattr(tf.transform.rotation, d) for d in 'xyzw'])

    def eulerFromTf2Msg(self, tf):
        q = self.quaternionFromTf2Msg(tf)
        return tf_conversions.transformations.euler_from_quaternion(q)

    def transformFromTf2Msg(self, tf):
        T = tf_conversions.transformations.quaternion_matrix(self.quaternionFromTf2Msg(tf))
        T[:3,-1] = self.positionFromTf2Msg(tf)
        return T

    def setTf(self, position, quaternion, base_frame_id, child_frame_id):
        """Sets a transform using tf2."""
        tf = TransformStamped()
        tf.header.stamp = self.__rp.Time.now()
        tf.header.frame_id = base_frame_id
        for i, dim in enumerate('xyz'):
            setattr(tf.transform.translation, dim, position[i])
            setattr(tf.transform.rotation, dim, quaternion[i])
        tf.transform.rotation.w = quaternion[3]
        self.__tf_broadcaster.sendTransform(tf)

    def setupStateMarkerPublisher(self, name, topic, queue_size=10):
        """When running experiments, it is useful to make markers to determine when certain events occurred."""
        self.pubs[name] = self.__rp.Publisher(topic, Int64, queue_size=queue_size)

    def sendMark(self, name, flag=0):
        self.pubs[name].publish(Int64(data=flag))

    def startService(self, name, type, handle):
        self.srvs[name] = self.__rp.Service(name, type, handle)

    def startTimer(self, name, frequency, handle):
        dt = 1.0/float(frequency)
        duration = self.__rp.Duration(dt)
        self.timers[name] = self.__rp.Timer(duration, handle)

    def startSubscriber(self, name, topic, type, wait=False):
        if wait:
            msg = self.__rp.wait_for_message(topic, type)
            self.__callback(msg, name)
        self.subs[name] = self.__rp.Subscriber(topic, type, self.__callback, callback_args=name)

    def __callback(self, msg, name):
        self.msgs[name] = msg

    def msgsRecieved(self, name):
        return name in self.msgs.keys()

    def getMsg(self, name, default=None):
        try:
            msg = self.msgs[name]
        except KeyError:
            msg = default
        return msg

    def spin(self):
        self.__rp.spin()

    def base_shutdown(self):
        for timer in self.timers.values():
            timer.shutdown()
        for sub in self.subs.values():
            sub.unregister()

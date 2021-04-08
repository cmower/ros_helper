import tf2_ros
from geometry_msg.msg import TransformStamped

class RosNode:

    def __init__(self, rospy):
        """Child-classes need to make a call to super().__init__(rospy)"""
        self.__rp = rospy
        self.__tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.__tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.__tf_buffer)
        self.subs = {} # Subscribers
        self.pubs = {} # Publishers
        self.srvs = {} # Services
        self.srv_names = set()
        self.timers = {} # Timers
        self.timer_names = set()
        self.msgs = {} # Messages
        self.msg_names = set()

    def getTf(self, base_frame_id, child_frame_id):
        tf = None
        try:
            tf = self.__tf_buffer.lookup_transform(base_frame_id, child_frame_id, self.rp.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.__rp.logwarn(f'Did not recover frame {child_frame_id} in the frame {base_frame_id}!')
        return tf

    def setTf(self, position, quaternion, base_frame_id, child_frame_id):
        tf = TransformStamped()
        tf.header.stamp = self.__rp.Time.now()
        tf.header.frame_id = base_frame_id
        for i, dim in enumerate('xyz'):
            setattr(tf.transform.translation, dim, position[i])
            setattr(tf.transform.rotation, dim, quaternion[i])
        tf.transform.rotation.w = quaternion[3]
        self.__tf_broadcaster.sendTransform(tf)

    def startService(self, name, type, handle):
        assert name not in self.srv_names, f"Service name ({name}) must be unique!"
        self.srv_names.add(name)
        self.srvs[name] = self.__rp.Service(name, type, handle)

    def startTimer(self, name, frequency, handle):

        # Handle name
        assert name not in self.timer_names, f"Timer name ({name}) must be unique!"
        self.timer_names.add(name)

        # Setup/start timer
        dt = 1.0/float(frequency)
        duration = self.__rp.Duration(dt)
        self.timers[name] = self.__rp.Timer(duration, handle)

    def startSubscriber(self, name, topic, type, wait=False):

        # Handle name
        assert name not in self.msg_names, f"Subscriber name ({name}) must be unique!"
        self.msg_names.add(name)

        # Wait for the first message, if user wants to
        if wait:
            msg = self.__rp.wait_for_message(topic, type)
            self.__callback(msg, name)

        # Setup subscriber
        self.subs[name] = self.__rp.Subscriber(topic, type, self.__callback, callback_args=name)

    def __callback(self, msg, name):
        self.msgs[name] = msg

    def msgsRecieved(self, name):
        return name in self.msgs.keys()

    def spin(self):
        self.__rp.spin()

    def base_shutdown(self):
        for timer in self.timers.values():
            timer.shutdown()
        for sub in self.subs.values():
            sub.unregister()

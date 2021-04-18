import numpy
import tf2_ros
import tf_conversions
from std_msgs.msg import Int64
from geometry_msg.msg import TransformStamped
from sensor_msgs.msg import Joy

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
        self.params = {} # parameters

    def initNode(self, name):
        """Initializes a node, note that this assumes default options. Use rospy.init_node() when you need more flexibility."""
        self.__rp.init_node(name)

    def onShutdownUseBaseShutdownMethod(self):
        """Specify the shutdown method as baseShutdown."""
        self.__rp.on_shutdown(self.baseShutdown)

    def getParams(self, params):
        """Gets parameters, params must be a list of tuples. Each tuple must have length 1 or 2. The first element is required, it must be the parameter name. The second element is optional, if set it will be the default value."""
        for args in params:
            name = args[0]
            self.params[name] = self.__rp.get_param(*args)

    def getTf(self, base_frame_id, child_frame_id):
        """Returns a transform from tf2 as a geometry_msgs/TransformStamped."""
        tf = None
        try:
            tf = self.__tf_buffer.lookup_transform(base_frame_id, child_frame_id, self.rp.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.__rp.logwarn(f'Did not recover frame {child_frame_id} in the frame {base_frame_id}!')
        return tf

    def positionFromTf2Msg(self, tf):
        """Extract position from geomety_msg/TransformStamed message."""
        return numpy.array([getattr(tf.transform.translation, d) for d in 'xyz'])

    def quaternionFromTf2Msg(self, tf):
        """Extract quaternion from geomety_msg/TransformStamed message."""
        return numpy.array([getattr(tf.transform.rotation, d) for d in 'xyzw'])

    def eulerFromQuaternion(self, q):
        """Euler angles from quaternion."""
        return tf_conversions.transformations.euler_from_quaternion(q)

    def quaternionFromEuler(self, eul):
        """Quaternion from Euler angles."""
        return tf_conversions.transformations.quaternion_from_euler(eul[0], eul[1], eul[2])

    def eulerFromTf2Msg(self, tf):
        """Extract Euler angles from geomety_msg/TransformStamed message."""
        return self.eulerFromQuaternion(self.quaternionFromTf2Msg(tf))

    def transformFromTf2Msg(self, tf):
        """Extract transformation matrix from geomety_msg/TransformStamed message."""
        T = tf_conversions.transformations.quaternion_matrix(self.quaternionFromTf2Msg(tf))
        T[:3,-1] = self.positionFromTf2Msg(tf)
        return T

    def packTransformStampedMsg(self, base_frame_id, child_frame_id, position, quaternion=[0, 0, 0, 1]):
        tf = TransformStamped()
        tf.header.stamp = self.__rp.Time.now()
        tf.child_frame_id = child_frame_id
        tf.header.frame_id = base_frame_id
        for i, dim in enumerate('xyz'):
            setattr(tf.transform.translation, dim, position[i])
            setattr(tf.transform.rotation, dim, quaternion[i])
        tf.transform.rotation.w = quaternion[3]
        return tf

    def setTf(self, base_frame_id, child_frame_id, position, quaternion=[0, 0, 0, 1]):
        """Sets a transform using tf2."""
        self.__tf_broadcaster.sendTransform(
            self.packTransformStampedMsg(base_frame_id, child_frame_id, position, quaternion)
        )

    def setupExperimentMarkerPublisher(self, name, topic, queue_size=10):
        """When running experiments, it is useful to make markers to determine when certain events occurred."""
        self.pubs[name] = self.__rp.Publisher(topic, Int64, queue_size=queue_size)

    def sendExperimentMarker(self, name, flag=0):
        """Publish marker, user can make what they want of the flags."""
        self.pubs[name].publish(Int64(data=flag))

    def startService(self, name, type, handle):
        """Start a service."""
        self.srvs[name] = self.__rp.Service(name, type, handle)

    def startTimer(self, name, frequency, handle):
        """Start a timer."""
        self.timers[name] = self.__rp.Timer(self.__rp.Duration(1.0/float(frequency)), handle)

    def startSubscriber(self, name, topic, type, wait=False):
        """Start a subscriber, optionally pause and wait for the first message."""
        if wait:
            msg = self.__rp.wait_for_message(topic, type)
            self.__callback(msg, name)
        self.subs[name] = self.__rp.Subscriber(topic, type, self.__callback, callback_args=name)

    def __callback(self, msg, name):
        """Internal callback method."""
        self.msgs[name] = msg

    def startJoySubscriber(self, name, topic, joystick_cls, wait=False):
        """Starts a joystick subscriber that automatically parses sensor_msgs/Joy messages to joystick class from a class in joy.py script."""
        args = (name, joystick_cls)
        if wait:
            msg = self.__rp.wait_for_message(topic, Joy)
            self.__joy_callback(msg, args)
        self.subs[name] = self.__rp.Subscriber(topic, Joy, self.__joy_callback, callback_args=args)

    def __joy_callback(self, msg, args):
        """Converts joy message to given joystick class and logs to the msgs class attribute."""
        name, joystick_cls = args
        self.__callback(joystick_cls(msg), name)

    def msgsRecieved(self, name):
        """True when at least one named message has been received, False otherwise. """
        return name in self.msgs.keys()

    def getMsg(self, name, default=None):
        """Returns named message, optional default when no messages have been received yet."""
        try:
            msg = self.msgs[name]
        except KeyError:
            msg = default
        return msg

    def spin(self):
        """Simple wrapper for rospy.spin."""
        self.__rp.spin()

    def baseShutdown(self):
        """First kills all timers, then unregisters all subscribers."""
        for timer in self.timers.values():
            timer.shutdown()
        for sub in self.subs.values():
            sub.unregister()

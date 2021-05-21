import time
import numpy
# import re
import rospkg
import tf2_ros
import yaml
import tf_conversions
from std_msgs.msg import Int64, Float64MultiArray
from geometry_msgs.msg import TransformStamped, Point, PointStamped
from sensor_msgs.msg import Joy, JointState

class RosNode:

    # ----------------------------------------------------------------------------------
    #
    # Initialization
    # ----------------------------------------------------------------------------------

    def __init__(self, rospy):
        """Initialization. Note, child-classes need to make a call to RosNode.__init__(rospy)."""
        self.__rospy = rospy
        self.subs = {} # Subscribers
        self.pubs = {} # Publishers
        self.srvs = {} # Services
        self.timers = {} # Timers
        self.msgs = {} # Messages
        self.params = {} # parameters
        self.tfs = {} # Transforms

    def initNode(self, name):
        """Initializes a node, note that this assumes default options. Use rospy.init_node() when you need more flexibility."""
        self.__rospy.init_node(name)

    def initTf2(self):
        """Call if you want to interface with tf2 and use the getTf/setTf."""
        self.__tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.__tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.__tf_buffer)

    def onShutdownUseBaseShutdownMethod(self):
        """Specify the shutdown method as baseShutdown."""
        self.__rospy.on_shutdown(self.baseShutdown)

    def parseFilename(self, filename):
        """Parse the filename, i.e. replace $(find ..) with path to package."""
        # TODO: ideally, use re
        if ('$(find' in filename) and (')' in filename):
            # pattern = '$(find \(.*?\))'
            # matches = re.findall(pattern, filename)
            filename = filename.replace('$(find ', '') # assume filename starts with '$(find '
            idx_closing_bracket = filename.find(')')
            package = filename[:idx_closing_bracket]
            root = rospkg.RosPack().get_path(package)
            filename = filename.replace(f'{package})', root)
        return filename

    def loadConfig(self, filename):
        """Loads a yaml config file. You can use $(find package-name)."""
        with open(self.parseFilename(filename), 'r') as configfile:
            config = yaml.load(configfile, Loader=yaml.FullLoader)
        return config

    def __assertNameIsUnique(self, name, d_name):
        d = getattr(self, d_name) # retrieves either subs/pubs/srvs/timers/params/tfs
        assert name not in d.keys(), f"Given name ({name}) is not unique in {d_name}!"

    # ----------------------------------------------------------------------------------
    #
    # Helpful methods
    # ----------------------------------------------------------------------------------

    def null(self, *args, **kwargs):
        """Method takes any input, and does nothing."""
        pass

    def uniqueTag(self):
        """Returns a random string that can be used to uniquify labels."""
        time_now = time.time_ns()
        random_numbers = "".join([str(n) for n in numpy.random.randint(0, 10, size=(4,))])
        unique_stamp = f"{random_numbers}_{time_now}"
        return unique_stamp

    # ----------------------------------------------------------------------------------
    #
    # Handling parameters
    # ----------------------------------------------------------------------------------

    def getParams(self, params):
        """Gets parameters, params must be a list of tuples. Each tuple must have length 1 or 2. The first element is required, it must be the parameter name. The second element is optional, if set it will be the default value."""
        for args in params:
            name = args[0]
            self.__assertNameIsUnique(name, 'params')
            self.params[name] = self.__rospy.get_param(*args)

    # ----------------------------------------------------------------------------------
    #
    # Interface to tf2
    # ----------------------------------------------------------------------------------

    def getTf(self, base_frame_id, child_frame_id):
        """Returns a transform from tf2 as a geometry_msgs/TransformStamped."""
        tf = None
        try:
            tf = self.__tf_buffer.lookup_transform(base_frame_id, child_frame_id, self.__rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.__rospy.logwarn(f'Did not recover frame {child_frame_id} in the frame {base_frame_id}!')
        return tf

    def setTf(self, base_frame_id, child_frame_id, position, quaternion=[0, 0, 0, 1]):
        """Sets a transform using tf2."""
        self.__tf_broadcaster.sendTransform(
            self.packTransformStampedMsg(base_frame_id, child_frame_id, position, quaternion)
        )

    def listenToTf(self, name, base_frame_id, child_frame_id, only_once=False, frequency=50):
        """Keeps track of transforms using tf2. """

        self.__assertNameIsUnique(name, 'tfs')
        timer_name = f'listen_to_tf_{name}_{self.uniqueTag()}'

        def __retrieveTfHandle(event):
            tf = self.getTf(base_frame_id, child_frame_id)
            if tf is None: return
            self.tfs[name] = tf
            if only_once:
                self.timers[timer_name].shutdown()

        self.startTimer(timer_name, frequency, __retrieveTfHandle)

    def tfRetrieved(self, name):
        return name in self.tfs.keys()

    def retrieveTf(self, name):
        return self.tfs[name]

    # ----------------------------------------------------------------------------------
    #
    # Common transform conversion methods.
    # ----------------------------------------------------------------------------------

    def positionFromTf2Msg(self, tf, fmt='xyz'):
        """Extract position from geomety_msg/TransformStamed message."""
        return numpy.array([getattr(tf.transform.translation, d) for d in fmt])

    def quaternionFromTf2Msg(self, tf, fmt='xyzw'):
        """Extract quaternion from geomety_msg/TransformStamed message."""
        return numpy.array([getattr(tf.transform.rotation, d) for d in fmt])

    def eulerFromQuaternion(self, q):
        """Euler angles from quaternion."""
        return tf_conversions.transformations.euler_from_quaternion(q)

    def quaternionFromEuler(self, eul):
        """Quaternion from Euler angles."""
        return tf_conversions.transformations.quaternion_from_euler(*eul)

    def eulerFromTf2Msg(self, tf):
        """Extract Euler angles from geomety_msg/TransformStamed message."""
        return self.eulerFromQuaternion(self.quaternionFromTf2Msg(tf))

    def transformFromTf2Msg(self, tf):
        """Extract transformation matrix from geomety_msg/TransformStamed message."""
        T = tf_conversions.transformations.quaternion_matrix(self.quaternionFromTf2Msg(tf))
        T[:3,-1] = self.positionFromTf2Msg(tf)
        return T

    # ----------------------------------------------------------------------------------
    #
    # ROS message packing methods
    # ----------------------------------------------------------------------------------

    def addTimeStampToMsg(self, msg):
        """Add time stamp to a ROS message."""
        msg.header.stamp = self.__rospy.Time.now()
        return msg

    def packTransformStampedMsg(self, base_frame_id, child_frame_id, position, quaternion=[0, 0, 0, 1]):
        """Pack a transform stamped message"""
        tf = self.addTimeStampToMsg(TransformStamped())
        tf.child_frame_id = child_frame_id
        tf.header.frame_id = base_frame_id
        for i, dim in enumerate('xyz'):
            setattr(tf.transform.translation, dim, position[i])
            setattr(tf.transform.rotation, dim, quaternion[i])
        tf.transform.rotation.w = quaternion[3]
        return tf

    def packPointMsg(self, position):
        return Point(x=position[0], y=position[1], z=position[2])

    # ----------------------------------------------------------------------------------
    #
    # Setup publishers
    # ----------------------------------------------------------------------------------

    def setupPublisher(self, name, topic, msg_type, queue_size=10):
        """Setup a publisher."""
        self.pubs[name] = self.__rospy.Publisher(topic, msg_type, queue_size=queue_size)

    def publishMsg(self, name, msg):
        """Publish a message for a given publisher accessed by name."""
        self.pubs[name].publish(msg)

    def setupJointStatePublisher(self, name, topic, queue_size=10):
        """Setup joint state message publisher."""
        self.setupPublisher(name, topic, JointState, queue_size=queue_size)

    def setupPointPublisher(self, name, topic, queue_size=10):
        """Setup a point message publisher."""
        self.setupPublisher(name, topic, Point, queue_size=queue_size)

    def setupPointStampedPublisher(self, name, topic, queue_size=10):
        """Setup a point stamped message publisher."""
        self.setupPublisher(name, topic, PointStamped, queue_size=queue_size)

    def setupFloat64MultiArrayPublisher(self, name, topic, queue_size=10):
        """Setup a float64 multi array message publisher."""
        self.setupPublisher(name, topic, Float64MultiArray, queue_size=queue_size)

    def setupInt64Publisher(self, name, topic, queue_size=10):
        """Setup a int64 message publisher."""
        self.setupPublisher(name, topic, Int64, queue_size=queue_size)

    # ----------------------------------------------------------------------------------
    #
    # Easy-publish methods
    # ----------------------------------------------------------------------------------

    def publishJointState(self, name, joint_name=[], joint_position=[], joint_velocity=[], joint_effort=[]):
        """Publish a joint state message."""
        self.pubs[name].publish(self.addTimeStampToMsg(JointState(name=joint_name, position=joint_position, velocity=joint_velocity, effort=joint_effort)))

    def publishPoint(self, name, position):
        """Publish a point message."""
        self.pubs[name].publish(self.packPointMsg(position))


    def publishPointStamped(self, name, position):
        """Publish a point stamped message."""
        self.pubs[name].publish(self.addTimeStampToMsg(PointStamped(point=self.packPointMsg(position))))

    def publishFloat64MultiArray(self, name, data):
        """Publish a float64 multi array message."""
        self.pubs[name].publish(Float64MultiArray(data=data))


    def publishInt64(self, name, data):
        """Publish marker, user can make what they want of the flags."""
        self.pubs[name].publish(Int64(data=data))

    # ----------------------------------------------------------------------------------
    #
    # Setup services
    # ----------------------------------------------------------------------------------

    def startService(self, name, type, handle):
        """Start a service."""
        self.__assertNameIsUnique(name, 'srvs')
        self.srvs[name] = self.__rospy.Service(name, type, handle)

    # ----------------------------------------------------------------------------------
    #
    # Setup timers
    # ----------------------------------------------------------------------------------

    def startTimer(self, name, frequency, handle):
        """Start a timer."""
        self.__assertNameIsUnique(name, 'timers')
        self.timers[name] = self.__rospy.Timer(self.__rospy.Duration(1.0/float(frequency)), handle)

    # ----------------------------------------------------------------------------------
    #
    # Setup subscribers
    # ----------------------------------------------------------------------------------

    def startSubscriber(self, name, topic, topic_type, wait=False, timeout=None):
        """Start a subscriber, optionally pause and wait for the first message."""
        self.__assertNameIsUnique(name, 'subs')
        if wait:
            msg = self.__rospy.wait_for_message(topic, topic_type, timeout)
            self.__callback(msg, name)
        self.subs[name] = self.__rospy.Subscriber(topic, topic_type, self.__callback, callback_args=name)

    def startUserSubscriber(self, topic, type, callback, callback_args=None):
        name = f'sub/{self.uniqueTag()}'
        self.__assertNameIsUnique(name, 'subs')
        self.subs[name] = self.__rospy.Subscriber(topic, type, callback, callback_args=callback_args)

    def startJoySubscriber(self, name, topic, joystick_cls, wait=False):
        """Starts a joystick subscriber that automatically parses sensor_msgs/Joy messages to joystick class from a class in joy.py script."""
        self.__assertNameIsUnique(name, 'subs')
        args = (name, joystick_cls)
        if wait:
            msg = self.__rospy.wait_for_message(topic, Joy)
            self.__joy_callback(msg, args)
        self.subs[name] = self.__rospy.Subscriber(topic, Joy, self.__joy_callback, callback_args=args)

    # ----------------------------------------------------------------------------------
    #
    # Internal callback methods
    # ----------------------------------------------------------------------------------

    def __callback(self, msg, name):
        """Internal callback method."""
        self.msgs[name] = msg

    def __joy_callback(self, msg, args):
        """Converts joy message to given joystick class and logs to the msgs class attribute."""
        name, joystick_cls = args
        self.__callback(joystick_cls(msg), name)

    # ----------------------------------------------------------------------------------
    #
    # Retrieve messages
    # ----------------------------------------------------------------------------------

    def msgsRecieved(self, name):
        """True when at least one named message has been received, False otherwise."""
        return name in self.msgs.keys()

    def getMsg(self, name, default=None):
        """Returns named message, optional default when no messages have been received yet."""
        try:
            msg = self.msgs[name]
        except KeyError:
            msg = default
        return msg

    # ----------------------------------------------------------------------------------
    #
    # Spin and shutdown node
    # ----------------------------------------------------------------------------------

    def spin(self):
        """Simple wrapper for rospy.spin."""
        self.__rospy.spin()

    def baseShutdown(self):
        """Kills all timers, subscribers, services, and publishers."""
        for timer in self.timers.values():
            timer.shutdown()
        for sub in self.subs.values():
            sub.unregister()
        for srv in self.srvs.values():
            srv.shutdown()
        for pub in self.pubs.values():
            pub.unregister()

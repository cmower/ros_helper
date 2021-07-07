"""
BSD 2-Clause License

Copyright (c) 2021, https://github.com/cmower/ros_helper by Christopher E. Mower
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import time
import string
import random
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

    def __init__(self, rospy, use_shutdown=True):
        """Initialization. Note, child-classes need to make a call to RosNode.__init__(self, rospy)."""

        # Base setup
        self.__rospy = rospy
        self.__subs = {}    # Subscribers
        self.__pubs = {}    # Publishers
        self.__srvs = {}    # Services
        self.__timers = {}  # Timers
        self.__msgs = {}    # Messages
        self.__params = {}  # Parameters
        self.__tfs = {}     # Transforms

        # Get name of node
        self.node_name = rospy.get_name()

        # Init tf
        self.__tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.__tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.__tf_buffer)

        # Use shutdown
        if use_shutdown:
            self.__rospy.on_shutdown(self.shutdown)

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

    # ----------------------------------------------------------------------------------
    #
    # Helpful methods
    # ----------------------------------------------------------------------------------

    def null(self, *args, **kwargs):
        """Method takes any input, and does nothing."""
        pass

    def uniqueTag(self):
        """Returns a random string that can be used to uniquify labels."""
        random_letters = "".join(random.choice(string.ascii_letters) for _ in range(5))
        time_now = time.time_ns()
        random_numbers = "".join([str(n) for n in numpy.random.randint(0, 10, size=(4,))])
        unique_stamp = f"{random_letters}_{random_numbers}_{time_now}"
        return unique_stamp

    # ----------------------------------------------------------------------------------
    #
    # Handling parameters
    # ----------------------------------------------------------------------------------

    def setupParams(self, params):
        """Gets parameters, params must be a list of tuples. Each tuple must have length 1 or 2. The first element is required, it must be the parameter name. The second element is optional, if set it will be the default value."""
        # Iterate over params
        for args in params:

            # Extract name (NOTE: each item in list/tuple params MUST contain a
            # name argument that is the parameter to be retrieved from ROS)
            name = args[0]
            assert name not in self.__params.keys(), f"Given name ({name}) is not unique!"

            # Get param from ROS
            #
            # NOTE: for future dev, this will throw an error if default is not
            # given and the name is not found (this is useful to keep, i.e.
            # don't use self.getParam here).
            self.__params[name] = self.__rospy.get_param(*args)

    def getParam(self, name, default=None):
        """Retrieves parameter (if name doesn't exists in internal parameter dictionary then getParam attempts to retrieve parameter from ROS)."""
        if name in self.__params.keys():
            return self.__params[name]
        else:
            return self.__rospy.get_param(name, default)

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

    def listenToTf(self, name, base_frame_id, child_frame_id, only_one=False, attempt_frequency=50):
        """Keeps track of transforms using tf2. """

        # Check name for tf is unique
        assert name not in self.__tfs.keys(), f"Given name ({name}) is not unique!"

        # Make unique timer name
        timer_name = f'listen_to_tf_{name}_{self.uniqueTag()}'

        # Setup internal retrieval method
        def __retrieveTf(event):
            tf = self.getTf(base_frame_id, child_frame_id)
            if tf is None: return
            self.__tfs[name] = tf
            if only_one:
                self.__timers[timer_name].shutdown()

        # Start tf timer
        self.setupTimer(timer_name, attempt_frequency, __retrieveTf)

    def tfRetrieved(self, name):
        """True when at least one tf with given name has been received."""
        return name in self.__tfs.keys()

    def retrieveTf(self, name):
        """Get tf with given name - can throw an error when tf not yet received, check with tfRetrieved."""
        return self.__tfs[name]

    def loadTfFromFile(self, filename):
        """Loads a transform from a .npy file, assumes file has the same format as how the save_tf.py saves tfs"""
        tf_data = numpy.load(self.parseFilename(filename))
        p = tf_data[:3]
        q = tf_data[3:]
        return p, q

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
        self.__pubs[name] = self.__rospy.Publisher(topic, msg_type, queue_size=queue_size)

    def publishMsg(self, name, msg):
        """Publish a message for a given publisher accessed by name."""
        self.__pubs[name].publish(msg)

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
        self.__pubs[name].publish(self.addTimeStampToMsg(JointState(name=joint_name, position=joint_position, velocity=joint_velocity, effort=joint_effort)))

    def publishPoint(self, name, position):
        """Publish a point message."""
        self.__pubs[name].publish(self.packPointMsg(position))


    def publishPointStamped(self, name, position):
        """Publish a point stamped message."""
        self.__pubs[name].publish(self.addTimeStampToMsg(PointStamped(point=self.packPointMsg(position))))

    def publishFloat64MultiArray(self, name, data):
        """Publish a float64 multi array message."""
        self.__pubs[name].publish(Float64MultiArray(data=data))


    def publishInt64(self, name, data):
        """Publish marker, user can make what they want of the flags."""
        self.__pubs[name].publish(Int64(data=data))

    # ----------------------------------------------------------------------------------
    #
    # Setup services
    # ----------------------------------------------------------------------------------

    def setupService(self, name, srv_type, handle):
        """Start a service."""
        assert name not in self.__srvs.keys(), f"Given name ({name}) is not unique!"
        self.__srvs[name] = self.__rospy.Service(name, srv_type, handle)

    # ----------------------------------------------------------------------------------
    #
    # Setup timers
    # ----------------------------------------------------------------------------------

    def setupTimer(self, name, frequency, handle):
        """Start a timer."""
        assert name not in self.__timers.keys(), f"Given name ({name}) is not unique!"
        self.__timers[name] = self.__rospy.Timer(self.__rospy.Duration(1.0/float(frequency)), handle)

    # ----------------------------------------------------------------------------------
    #
    # Setup subscribers
    # ----------------------------------------------------------------------------------

    def setupSubscriber(self, name, topic, topic_type, wait=False, timeout=None):
        """Start a subscriber, optionally pause and wait for the first message."""
        assert name not in self.__subs.keys(), f"Given name ({name}) is not unique!"
        if wait:
            msg = self.__rospy.wait_for_message(topic, topic_type, timeout)
            self.__callback(msg, name)
        self.__subs[name] = self.__rospy.Subscriber(topic, topic_type, self.__callback, callback_args=name)

    def setupUserSubscriber(self, topic, type, callback, callback_args=None):
        name = f'sub/{self.uniqueTag()}'
        assert name not in self.__subs.keys(), f"Given name ({name}) is not unique!"
        self.__subs[name] = self.__rospy.Subscriber(topic, type, callback, callback_args=callback_args)

    def setupJoySubscriber(self, name, topic, joystick_cls, wait=False):
        """Starts a joystick subscriber that automatically parses sensor_msgs/Joy messages to joystick class from a class in joy.py script."""
        assert name not in self.__subs.keys(), f"Given name ({name}) is not unique!"
        args = (name, joystick_cls)
        if wait:
            msg = self.__rospy.wait_for_message(topic, Joy)
            self.__joy_callback(msg, args)
        self.__subs[name] = self.__rospy.Subscriber(topic, Joy, self.__joy_callback, callback_args=args)

    # ----------------------------------------------------------------------------------
    #
    # Internal callback methods
    # ----------------------------------------------------------------------------------

    def __callback(self, msg, name):
        """Internal callback method."""
        self.__msgs[name] = msg

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
        return name in self.__msgs.keys()

    def getMsg(self, name, default=None):
        """Returns named message, optional default when no messages have been received yet."""
        try:
            msg = self.__msgs[name]
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

    def shutdown(self):
        """Kills all timers, subscribers, services, and publishers."""
        for timer in self.__timers.values():
            timer.shutdown()
        for sub in self.__subs.values():
            sub.unregister()
        for srv in self.__srvs.values():
            srv.shutdown()
        for pub in self.__pubs.values():
            pub.unregister()

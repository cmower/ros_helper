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
import tf2_ros
import tf_conversions
from std_msgs.msg import Int64, Float64MultiArray
from geometry_msgs.msg import TransformStamped, Point, PointStamped
from sensor_msgs.msg import Joy, JointState

from .msgs import transform_stamped

# Constants
NUM_UNIQUE_LETTERS = 5
NUM_UNIQUE_RAND_INTS = 5

# Rospy
rospy = None

# Main
class RosNode:

    # ----------------------------------------------------------------------------------
    #
    # Initialization
    # ----------------------------------------------------------------------------------

    def __init__(self, rospy_, use_shutdown=True):
        """Initialization. Note, child-classes need to make a call to RosNode.__init__(self, rospy)."""

        global rospy
        rospy = rospy_

        # Base setup
        self.subs = {}    # Subscribers
        self.pubs = {}    # Publishers
        self.srvs = {}    # Services
        self.timers = {}  # Timers
        self.msgs = {}    # Messages
        self.params = {}  # Parameters
        self.tfs = {}     # Transforms

        # Get name of node
        self.node_name = rospy.get_name()

        # Check node is initialized
        if self.node_name.endswith('unnamed'):
            raise rospy.exceptions.ROSException('node not initialized, need to call rospy.init_node()!')

        # Init tf
        self.__tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.__tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.__tf_buffer)

        # Use shutdown
        if use_shutdown:
            rospy.on_shutdown(self.shutdown)

    # ----------------------------------------------------------------------------------
    #
    # Helpful misc methods
    # ----------------------------------------------------------------------------------

    @staticmethod
    def null(*args, **kwargs):
        """Method takes any input, and does nothing."""
        pass

    @staticmethod
    def unique_tag():
        """Returns a random string that can be used to uniquify labels."""
        random_letters = "".join(random.choice(string.ascii_letters) for _ in range(NUM_UNIQUE_LETTERS))
        random_numbers = "".join([str(n) for n in numpy.random.randint(0, 10, size=NUM_UNIQUE_RAND_INTS)])
        time_now = time.time_ns()
        return f"{random_letters}_{random_numbers}_{time_now}"

    # ----------------------------------------------------------------------------------
    #
    # Handling parameters
    # ----------------------------------------------------------------------------------

    def collect_params(self, params):
        """Gets parameters, params must be a list of tuples. Each tuple must have length 1 or 2. The first element is required, it must be the parameter name. The second element is optional, if set it will be the default value."""
        # Iterate over params
        for args in params:

            if not (1 <= len(args) <= 2):
                raise ValueError("incorrect number of arguments!")

            # Extract name (NOTE: each item in list/tuple params MUST contain a
            # name argument that is the parameter to be retrieved from ROS)
            name = args[0]
            assert name not in self.params.keys(), f"Given name ({name}) is not unique!"

            # Get param from ROS
            #
            # NOTE: for future dev, this will throw an error if default is not
            # given and the name is not found (this is useful to keep, i.e.
            # don't use self.getParam here).
            self.params[name] = rospy.get_param(*args)

    # ----------------------------------------------------------------------------------
    #
    # Interface to tf2
    # ----------------------------------------------------------------------------------

    def get_tf(self, baseid, childid):
        """Returns a transform from tf2 as a geometry_msgs/TransformStamped."""
        tf = None
        try:
            tf = self.__tf_buffer.lookup_transform(baseid, childid, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f'Did not recover frame {childid} in the frame {baseid}!')
        return tf

    def set_tf(self, baseid, childid, p, q=[0, 0, 0, 1]):
        """Sets a transform using tf2."""
        self.__tf_broadcaster.sendTransform(transform_stamped(baseid, childid, p, q))

    def listen_to_tf(self, name, baseid, childid, attempt_frequency=50):
        """Keeps track of transforms using tf2. """

        # Check name for tf is unique
        if name in self.tfs.keys():
            raise rospy.exceptions.ROSException(f"given name ({name}) for tf is not unique!")

        # Setup internal retrieval method
        def __get_tf(e):
            tf = self.get_tf(baseid, childid)
            if tf is None: return
            self.tfs[name] = tf

        # Start tf timer
        self.create_timer(f'listen_to_tf_{name}_{self.unique_tag()}', attempt_frequency, __get_tf)

    def tf_retrieved(self, name):
        """True when at least one tf with given name has been received."""
        return self.tfs.get(name, None) is not None

    # ----------------------------------------------------------------------------------
    #
    # Setup publishers
    # ----------------------------------------------------------------------------------

    def create_publisher(self, name, *args, **kwargs):
        if name in self.pubs.keys():
            raise rospy.exceptions.ROSException(f'publisher name ({name}) must be unique!')
        if kwargs.get('queue_size', None) is None:
            kwargs['queue_size'] = 10
        self.pubs[name] = rospy.Publisher(*args, **kwargs)

    def create_flag_publisher(self, name):
        self.create_publisher(name, f'flag/{name}', Int64)

    def flag(self, name, flag):
        self.pubs[name].publish(Int64(data=flag))

    # ----------------------------------------------------------------------------------
    #
    # Setup services
    # ----------------------------------------------------------------------------------

    def create_service(self, name, *args, **kwargs):
        if self.srvs.get(name, None) is not None:
            raise rospy.exceptions.ROSException(f'service name ({name}) must be unique!')
        self.srvs[name] = rospy.Service(name, *args, **kwargs)

    # ----------------------------------------------------------------------------------
    #
    # Setup timers
    # ----------------------------------------------------------------------------------

    def create_timer(self, name, hz, handle):
        """Start a timer."""
        if self.timers.get(name, None) is not None:
            raise rospy.exceptions.ROSException(f'timer name ({name}) must be unique!')
        self.timers[name] = rospy.Timer(rospy.Duration(1.0/float(hz)), handle)

    # ----------------------------------------------------------------------------------
    #
    # Setup subscribers
    # ----------------------------------------------------------------------------------

    def create_subscriber(self, name, topic, data_class, **kwargs):
        """Start a subscriber, optionally pause and wait for the first message."""
        if self.subs.get(name, None) is not None:
            raise rospy.exceptions.ROSException(f'subscriber name ({name}) must be unique!')

        def callback(msg, name):
            self.msgs[name] = msg

        if kwargs.get('wait', False):
            callback(rospy.wait_for_message(topic, data_class), name)

        self.subs[name] = rospy.Subscriber(topic, data_class, callback, callback_args=name, **kwargs)

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
        rospy.spin()

    def shutdown(self):
        """Kills all timers, subscribers, services, and publishers."""
        for timer in self.timers.values():
            timer.shutdown()
        for sub in self.subs.values():
            sub.unregister()
        for srv in self.srvs.values():
            srv.shutdown()
        for pub in self.pubs.values():
            pub.unregister()

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
import rospy
import tf2_ros
from std_msgs.msg import Int64

from .msgs import transform_stamped

# Constants
NUM_UNIQUE_LETTERS = 5
NUM_UNIQUE_RAND_INTS = 5

# Helper functions/classes
def _contains(d, n):
    """True when dictionary contains name in keys."""
    return d.get(n, None) is not None

class _callback:

    def __init__(self, callback=None, args=None):
        if callback is not None:
            self._callback = callback
        else:
            self._callback = self._null
        self._args = args
        if self._args is None:
            self._make_callback = self._make_call_noargs
        else:
            self._make_callback = self._make_call_args

    def _null(self, *args, **kwargs):
        pass

    def _make_call_noargs(self, x):
        self._callback(x)

    def _make_call_args(self, x):
        self._callback(x, self._args)

    def __call__(self, x):
        self._make_callback(x)

# Main
class RosNode:

    """Base-class for a ROS node containing several useful methods."""

    def __init__(self, name, **kwargs):
        """Initialization. Note, child-classes need to make a call to RosNode.__init__(self, name, **kwargs)."""

        # Initialize node
        rospy.init_node(name, **kwargs)

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

        # Init tf
        self.__tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.__tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.__tf_buffer)

        # Use shutdown
        rospy.on_shutdown(self.shutdown)


    def __is_name_unique(self, d, n, label):
        """Check if name is unique in the dictionary keys, ROSException is thrown if not."""
        if _contains(d, n):
            raise rospy.exceptions.ROSException(f"given name ({n}) is not unique in {label}!")


    @staticmethod
    def unique_tag():
        """Returns a random string that can be used to uniquify labels."""
        random_letters = "".join(random.choice(string.ascii_letters) for _ in range(NUM_UNIQUE_LETTERS))
        random_numbers = "".join([str(n) for n in numpy.random.randint(0, 10, size=NUM_UNIQUE_RAND_INTS)])
        time_now = time.time_ns()
        return f"{random_letters}_{random_numbers}_{time_now}"


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
            self.params[name] = rospy.get_param(*args)


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
        self.__tf_broadcaster.sendTransform(transform_stamped(baseid, childid, rospy.Time.now(), p, q))


    def listen_to_tf(self, name, baseid, childid, frequency=50, callback=None, callback_args=None):
        """Keeps track of transforms using tf2."""

        self.__is_name_unique(self.tfs, name, 'tfs')
        _user_callback = _callback(callback, callback_args)

        # Setup internal retrieval method
        def __get_tf(e):
            tf = self.get_tf(baseid, childid)
            _user_callback(tf)
            if tf is None: return
            self.tfs[name] = tf

        # Start tf timer
        self.create_timer(f'listen_to_tf_{name}_{self.unique_tag()}', frequency, __get_tf)


    def create_publisher(self, name, topic, data_class, **kwargs):
        """Creates a publisher."""
        self.__is_name_unique(self.pubs, name, 'publisher')
        if not _contains(kwargs, 'queue_size'):
            kwargs['queue_size'] = 10
        self.pubs[name] = rospy.Publisher(topic, data_class, **kwargs)


    def create_flag_publisher(self, name):
        """Creates a publisher for a flag, you can use this in experiments to stamp discrete events."""
        self.create_publisher(name, f'flag/{name}', Int64)


    def flag(self, name, flag=0):
        """Publish a flag."""
        self.pubs[name].publish(Int64(data=flag))


    def create_service(self, name, *args, **kwargs):
        """Create a service"""
        self.__is_name_unique(self.srvs, name, 'service')
        self.srvs[name] = rospy.Service(name, *args, **kwargs)


    def create_timer(self, name, frequency, handle):
        """Start a timer."""
        self.__is_name_unique(self.timers, name, 'timer')
        self.timers[name] = rospy.Timer(rospy.Duration(1.0/float(frequency)), handle)


    def create_subscriber(self, name, topic, data_class, **kwargs):
        """Start a subscriber, optionally pause and wait for the first message."""

        self.__is_name_unique(self.subs, name, 'subscriber')

        # User defined callback
        _user_callback = _callback(kwargs.get('callback', None), kwargs.get('callback_args', None))
        try:
            kwargs.pop('callback')
        except KeyError:
            pass
        try:
            kwargs.pop('callback_args')
        except KeyError:
            pass

        def _sub_callback(msg, name):
            self.msgs[name] = msg
            _user_callback(msg)

        if kwargs.get('wait', False):
            _sub_callback(rospy.wait_for_message(topic, data_class), name)

        self.subs[name] = rospy.Subscriber(topic, data_class, _sub_callback, callback_args=name, **kwargs)


    def spin(self):
        """Simple wrapper for rospy.spin."""
        rospy.spin()


    def shutdown_timers(self):
        """Shutdown all timers"""
        for timer in self.timers.values():
            timer.shutdown()


    def shutdown_services(self):
        """Shutdown all services"""
        for srv in self.srvs.values():
            srv.shutdown()


    def unregister_publishers(self):
        """Unregister all publishers"""
        for pub in self.pubs.values():
            pub.unregister()


    def unregister_subscribers(self):
        """Unregister all subscribers"""
        for sub in self.subs.values():
            sub.unregister()


    def kill(self):
        """Kills all timers, subscribers, services, and publishers."""
        self.shutdown_timers()
        self.unregister_subscribers()
        self.shutdown_services()
        self.unregister_publishers()


    def shutdown(self):
        """Shutdown method"""
        self.kill()

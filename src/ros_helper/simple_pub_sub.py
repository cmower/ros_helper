"""

Consider the following..

See the following on how to place constraints on user inheriting these classes.

https://www.youtube.com/watch?v=cKPlPJyQrt4&t=5426s

i.e. in SimplePublisher, we want to check before running that generate_message function is implemented.

"""

def Repeater(rospy, hz, fun):
    """
    Repeater

    Continuously call a function.

    Syntax
    ------

      Repeater(rospy, hz, fun)

    Parameters
    ----------

      rospy (module) : rospy module
      hz (int/float) : If hz >= 1.0 then hz is assumed to be the frequency in Hertz, otherwise if 0 < hz < 1.0 then it is assumed to be the difference in time (i.e. a dt).
      fun (function) : Function handle to continuously repeat calling. Note, it must contain one parameters; often fun(event).

    """

    # hz -> dt
    typ = type(hz)
    if typ is int:
        dt = 1.0 / float(hz)
    elif typ is float:
        dt = hz if hz < 1.0 else 1.0/hz
    else:
        raise ValueError("hz must be an int or float")
    dt = abs(dt) # ensure dt is positive

    # Setup ros timer
    rospy.Timer(rospy.Duration(dt), fun)

class SimplePublisher(object):

    def __init__(self, rospy, topic, msg_class, hz, queue_size=1, handle=None):
        self.init_pub(rospy, topic, msg_class, hz, queue_size, handle)

    def init_pub(self, rospy, topic, msg_class, hz, queue_size=1, handle=None):

        # Check input
        if handle is None:
            assert hasattr(self, 'generate_message'), "Derived class for SimplePublisher must have a generate_message method implemented."
        else:
            assert callable(handle), "[ERROR] Given handle must be callable"
            self.generate_message = handle


        # Setup counter
        self.__number_of_messages_published = 0

        # Setup ros publisher
        self.__pub = rospy.Publisher(topic, msg_class, queue_size=queue_size)

        # Setup ros timer with method self.update as handle
        Repeater(rospy, hz, self.update)

    @property
    def NumberOfMessagesPublished(self):
        return self.__number_of_messages_published

    def update(self, event):
        self.__pub.publish(self.generate_message())
        self.__number_of_messages_published += 1

class SimpleConstPublisher(SimplePublisher):

    def __init__(self, rospy, topic, msg_class, hz, msg, queue_size=1):

        # Init rospy and msg
        self.__rospy = rospy
        self.__msg = msg

        # Check if msg requires time to be added at every update iteration
        if hasattr(msg, 'header'):
            # true -> time needs to be updated
            self.__update_time = self.__add_time
        else:
            # false -> time does not need to be updated
            self.__update_time = self.__dont_add_time

        # Init pub
        self.init_pub(rospy, topic, msg_class, hz, queue_size)

    def __add_time(self):
        self.__msg.header.stamp = self.__rospy.Time.now()

    def __dont_add_time(self):
        pass

    def generate_message(self):
        self.__update_time()
        return self.__msg

class SimpleSubscriber(object):

    def __init__(self, rospy, topic, msg_class, callback_handle=None):
        self.init_sub(rospy, topic, msg_class, callback_handle)

    def init_sub(self, rospy, topic, msg_class, callback_handle=None):

        # Check input and set users callback
        if callback_handle is None:
            self.__user_callback = self.__pass_user_input
        else:
            assert callable(callback_handle), "[ERROR] Given callback_handle must be callable"
            self.__user_callback = callback_handle

        # Init private msg parameter and counter
        self.__msg = None
        self.__number_of_messages_recieved = 0

        # Init subscriber
        rospy.Subscriber(topic, msg_class, self.__callback)

    def __pass_user_input(self, msg):
        pass

    @property
    def Msg(self):
        return self.__msg

    @property
    def NumberOfMessagesRecieved(self):
        return self.__number_of_messages_recieved

    def __callback(self, msg):
        self.__msg = msg
        self.__number_of_messages_recieved += 1
        self.__user_callback(msg)

class SimpleMultiSubscriber(object):

    def __init__(self, rospy, topics, msg_classes, callback_handles = None):

        # Check input
        if not isinstance(msg_classes, (list, tuple)): msg_classes = [msg_classes]*len(topics)
        assert len(topics) == len(msg_classes), "[ERROR] topics must be same length as msg_classes"
        if callback_handles is None:
            callback_handles = [None]*len(msg_classes)
        else:
            assert len(callback_handles) == len(topics), "[ERROR] callback_handles must be same length as topics and msg_classes"

        # Init dictionary of subs
        self.__subs = {topic : SimpleSubscriber(rospy, topic, msg_class, callback) for topic, msg_class, callback in zip(topics, msg_classes, callback_handles)}

    def Msg(self, topic):
        return self.__subs[topic].Msg

    def NumberOfMessagesRecieved(self, topic):
        return self.__subs[topic].NumberOfMessagesRecieved

class SimpleSyncSubscriber(object):

    def __init__(self, rospy, topics, msg_classes, callback_handle=None):

        # Import and check input
        import message_filters as mf
        if not isinstance(msg_classes, (list, tuple)): msg_classes = [msg_classes]*len(topics)

        if callback_handle is None:
            self.__user_callback = self.__pass_user_callback
        else:
            assert callable(callback_handle), "[ERROR] callback_handle must be callable"
            self.__user_callback = callback_handle

        # Setup approximate time sync sub (maybe allow user to ammend these in later vers)
        mf.ApproximateTimeSynchronizer([mf.Subscriber(topic, msg_class) for topic, msg_class in zip(topics, msg_classes)],\
                                       10,\
                                       len(topics)*100).registerCallback(self.__callback)

        # Init vars
        self.__msgs = None
        self.__number_of_messages_recieved = 0
        self.__topics = topics
        self.__topics_maps = {topic : i for i, topic in enumerate(topics)}

    def __pass_user_callback(self, msgs):
        pass

    def __callback(self, *msgs):
        self.__msgs = msgs
        self.__number_of_messages_recieved += len(msgs)
        self.__user_callback(msgs)

    @property
    def Msgs(self):
        return self.__msgs

    @property
    def NumberOfMessagesRecieved(self):
        return self.__number_of_messages_recieved

    def Msg(self, topic):
        return self.__msgs[self.__topics_map[topic]]

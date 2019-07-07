"""

Consider the following..

See the following on how to place constraints on user inheriting these classes.

https://www.youtube.com/watch?v=cKPlPJyQrt4&t=5426s

i.e. in SimplePublisher, we want to check before running that generate_message function is implemented.

"""

class Repeater(object):

    def __init__(self, rospy, hz, func, pass_event=False):
        self.init_repeater(rospy, hz, func, pass_event)

    def init_repeater(self, rospy, hz, func, pass_event=False):

        # Setup update function
        self.__func = func

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
        self.__timer = rospy.Timer(rospy.Duration(dt), self.__update_pass_event if pass_event else self.__update_no_pass_event)

    def __update_pass_event(self, event):
        self.__func(event)

    def __update_no_pass_event(self, event):
        self.__func()

    def stop(self):
        self.__timer.shutdown()

    def get_timer(self):
        return self.__timer

class SimplePublisher(Repeater):

    def __init__(self, rospy, topic, msg_class, hz, gen_msg_handle, queue_size=1): 
        self.init_pub(rospy, topic, msg_class, hz, gen_msg_handle, queue_size)

    def init_pub(self, rospy, topic, msg_class, hz, gen_msg_handle=None, queue_size=1):

        # Set generate message, if using a derived class then must have attribute generate_message
        self.__generate_message = self.generate_message if gen_msg_handle is None else gen_msg_handle

        # Setup counter
        self.__number_of_messages_published = 0

        # Setup ros publisher and repeater
        self.__pub = rospy.Publisher(topic, msg_class, queue_size=queue_size)
        self.init_repeater(rospy, hz, self.__publish_message)


    def __publish_message(self):
        self.__pub.publish(self.__generate_message())
        self.__number_of_messages_published += 1

    @property
    def NumberOfMessagesPublished(self):
        return self.__number_of_messages_published

    @property
    def NumberOfConnections(self):
        return self.__pub.get_num_connections()

    def stop(self):
        self.get_timer().shutdown()
        self.__pub.unregister()

class SimpleTfStaticPublisher(Repeater):

    def __init__(self, rospy, transforms, hz):
        from .transform import TfApi
        self.tf_api = TfApi(rospy)
        self.__transforms = transforms
        self.__rospy = rospy
        self.init_repeater(rospy, hz, self.__send)

    def __send(self):
        for transform in self.__transforms:
            transform.time = self.__rospy.Time.now()
            self.tf_api.set_tf(transform)

class SimpleConstPublisher(SimplePublisher):

    def __init__(self, rospy, topic, hz, msg, queue_size=1):

        # Init rospy and msg
        self.__rospy = rospy
        self.__msg = msg

        # Init update time function and publisher
        self.__update_time = self.__add_time if hasattr(msg, 'header') else self.__dont_add_time
        self.init_pub(rospy, topic, type(msg), hz, queue_size=queue_size)

    def __add_time(self):
        self.__msg.header.stamp = self.__rospy.Time.now()

    def __dont_add_time(self):
        pass

    def generate_message(self):
        self.__update_time()
        return self.__msg

class SimpleSubscriber(object):

    def __init__(self, rospy, topic, msg_class, callback_handle=None, callback_args=None):
        self.init_sub(rospy, topic, msg_class, callback_handle, callback_args)

    def init_sub(self, rospy, topic, msg_class, callback_handle=None, callback_args=None):

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
        self.__sub = rospy.Subscriber(topic, msg_class, self.__callback, callback_args=callback_args)

    def __pass_user_input(self, msg):
        pass

    @property
    def Msg(self):
        return self.__msg

    @property
    def NumberOfMessagesRecieved(self):
        return self.__number_of_messages_recieved

    @property
    def NumberOfConnections(self):
        return self.__sub.get_num_connections()

    def __callback(self, msg):
        self.__msg = msg
        self.__number_of_messages_recieved += 1
        self.__user_callback(msg)

    def stop(self):
        self.__sub.unregister()

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
        self.topics = topics
        self.__subs = {topic : SimpleSubscriber(rospy, topic, msg_class, callback) for topic, msg_class, callback in zip(topics, msg_classes, callback_handles)}

    def Msg(self, topic):
        return self.__subs[topic].Msg

    def NumberOfMessagesRecieved(self, topic):
        return self.__subs[topic].NumberOfMessagesRecieved

    def NumberOfConnections(self, topic):
        return self.__subs[topic].NumberOfConnections

    def stop(self):
        for topic in self.topics: self.__subs[topic].stop()

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

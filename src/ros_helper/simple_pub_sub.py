"""

Consider the following..

See the following on how to place constraints on user inheriting these classes.

https://www.youtube.com/watch?v=cKPlPJyQrt4&t=5426s

i.e. in SimplePublisher, we want to check before running that generate_message function is implemented.

"""

class SimplePublisher(object):

    def __init__(self, rospy, topic_name, topic_type, hz, queue_size=1, generate_message_handle=None):
        self.init_pub(rospy, topic_name, topic_type, hz, queue_size, generate_message_handle)

    def init_pub(self, rospy, topic_name, topic_type, hz, queue_size=1, generate_message_handle=None):

        # Check input
        if generate_message_handle is None:
            assert hasattr(self, 'generate_message'), "Derived class for SimplePublisher must have a generate_message method implemented."
        else:
            assert callable(generate_message_handle), "[ERROR] Given generate_message_handle must be callable"
            self.generate_message = generate_message_handle

        typ = type(hz)
        if typ is int:
            dt = 1.0 / float(hz)
        elif typ is float:
            dt = hz if hz < 1.0 else 1.0/hz
        else:
            raise ValueError("hz must be an int or float")
        dt = abs(dt) # ensure dt is positive

        # Setup counter
        self.__number_of_messages_published = 0

        # Setup ros publisher
        self.__pub = rospy.Publisher(topic_name, topic_type, queue_size=queue_size)

        # Setup ros timer with method self.update as handle
        rospy.Timer(rospy.Duration(dt), self.update)

    @property
    def NumberOfMessagesPublished(self):
        return self.__number_of_messages_published

    def update(self, event):
        self.__pub.publish(self.generate_message())
        self.__number_of_messages_published += 1

class SimpleConstPublisher(SimplePublisher):

    def __init__(self, rospy, topic_name, topic_type, hz, msg, queue_size=1):

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
        self.init_pub(rospy, topic_name, topic_type, hz, queue_size)

    def __add_time(self):
        self.__msg.header.stamp = self.__rospy.Time.now()

    def __dont_add_time(self):
        pass

    def generate_message(self):
        self.__update_time()
        return self.__msg
    
class SimpleSubscriber(object):

    def __init__(self, rospy, topic_name, topic_type, callback_handle=None):
        self.init_sub(rospy, topic_name, topic_type, callback_handle)

    def init_sub(self, rospy, topic_name, topic_type, callback_handle=None):

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
        rospy.Subscriber(topic_name, topic_type, self.__callback)

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

    def __init__(self, rospy, topic_names, topic_types, callback_handles = None):

        # Check input
        assert len(topic_names) == len(topic_types), "[ERROR] topic_names must be same length as topic_types"
        if callback_handles is None:
            callback_handles = [None]*len(topic_types)
        else:
            assert len(callback_handles) == len(topic_types), "[ERROR] callback_handles must be same length as topic_names and topic_types"

        # Init dictionary of subs
        self.__subs = {}
        for topic_name, topic_type, callback_handle in zip(topic_names, topic_types, callback_handles): self.__subs[topic_name] = SimpleSubscriber(rospy, topic_name, topic_type, callback_handle)

        # Init vars
        self.__topic_names = topic_names

    def Msg(self, topic):
        return self.__subs[topic].Msg

    def NumberOfMessagesRecieved(self, topic):
        return self.__subs[topic].NumberOfMessagesRecieved 

class SimpleSyncSubscriber(object):

    def __init__(self, rospy, topic_names, topic_types, callback_handle=None):

        # Import and check input
        import message_filters as mf
        if len(topic_types) == 1:
            topic_types = [topic_types] * len(topic_names)
        else:
            assert len(topic_names) == len(topic_types), "[ERROR] topic_types must be length 1 or same length as topic_names"

        # Check input
        if callback_handle is None:
            self.__user_callback = self.__pass_user_callback
        else:
            assert callable(callback_handle), "[ERROR] callback_handle must be callable"
            self.__user_callback = callback_handle

        # Setup approximate time sync sub (maybe allow user to ammend these in later vers)
        mf.ApproximateTimeSynchronizer([mf.Subscriber(topic_name, topic_type) for topic_name, topic_type in zip(topic_names, topic_types)],\
                                       10,\
                                       len(topic_names)*100).registerCallback(self.__callback)

        # Init vars
        self.__msgs = None
        self.__number_of_messages_recieved = 0
        self.__topic_names = topic_names
        self.__topic_names_maps = {topic : i for i, topic in enumerate(topic_names)}

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
        return self.__msgs[self.__topic_names_map[topic]]
        
        
        
        

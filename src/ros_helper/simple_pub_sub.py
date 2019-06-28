
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
            assert callable(generate_message_handle), "Given generate_message_handle must be callable"
            self.generate_message = generate_message_handle

        # Setup ros publisher
        self.pub = rospy.Publisher(topic_name, topic_type, queue_size=queue_size)

        # Setup ros timer, self.update will be called at a frequency of hz 
        rospy.Timer(rospy.Duration(1.0/float(hz)), self.update)

    def update(self, event):
        self.pub.publish(self.generate_message())

class SimpleSubscriber(object):

    def init_sub(self, rospy, topic_name, topic_type):
        rospy.Subscriber(topic_name, topic_type, self.callback)
        self.received_msg = False
        self.msg = None

    def get_message(self):
        return self.msg

    def callback(self, msg):
        self.msg = msg
        self.received_msg = True

class SimplePublisherSubscriber(SimplePublisher, SimpleSubscriber):

    def init_pub_sub(self, rospy, pub_topic_name, pub_topic_type, pub_hz, sub_topic_name, sub_topic_type, pub_queue_size=1):
        self.init_pub(rospy, pub_topic_name, pub_topic_type, pub_hz, pub_queue_size)
        self.init_sub(rospy, sub_topic_name, sub_topic_type)

                                   

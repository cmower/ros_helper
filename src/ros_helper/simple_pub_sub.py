
"""
See the following on how to place constraints on user inheriting these classes.

https://www.youtube.com/watch?v=cKPlPJyQrt4&t=5426s

i.e. in SimplePublisher, we want to check before running that generate_message function is implemented.

Also, is it worth making a init_publisher function instead of assuming user will know to call super(UserDefPub, self).__init__(...)? Can we constrain user to this?

"""

class SimplePublisher(object):

    def __init__(self, rospy, topic_name, topic_type, queue_size=1):
        self.pub = rospy.Publisher(topic_name, topic_type, queue_size=queue_size)

    def update(self, event):
        self.pub.publish(self.generate_message())

class SimpleSubscriber(object):

    def __init__(self, rospy, topic_name, topic_type):
        self.sub = rospy.Subscriber(topic_name, topic_type, self.callback)
        self.received_msg = False
        self.msg = None

    def get_msg(self):
        return self.msg

    def callback(self, msg):
        self.msg = msg
        self.received_msg = True

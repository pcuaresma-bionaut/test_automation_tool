import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

# add logger
# error handling

QUEUE_SIZE = 10

class PublisherTopics(Enum):
    PUB_TOP_1 = '/pub_top_1'
    PUB_TOP_2 = '/pub_top_2'

class SubscriberTopics(Enum):
    SUB_TOP_1 = '/sub_top_1'
    SUB_TOP_2 = '/sub_top_2'


class AutoTestRunnerNode(Node):
    def __init__(self, node_name='AutoTestRunnerNode'):
        super().__init__(node_name)
        self.subscribers = []
        self.publishers = []
        self._init_subscribers()
        self._init_publishers()

    def _init_subscribers(self):
        for topics in SubscriberTopics:
            self.subscribers.append(self.create_subscription(String, topics.value, self._callback, QUEUE_SIZE))
       
    def _init_publishers(self):
        for topics in PublisherTopics:
            self.publishers.append(self.create_publisher(String, topics.value, QUEUE_SIZE))

    def _callback(self, topic_name, msg=None):
        received_msg = msg.data
        # consider logging reeived message
        self.get_logger().info('Received message topic, topic_name = %s' %topic_name)

        # for one of these handlers, if a mode is in waiting for confirm state then publish user confirm
        # log echoing which state procedure state is in
        match topic_name:
            case SubscriberTopics.SUB_TOP_1.value:
                self._handle_sub_top_1()
            case SubscriberTopics.SUB_TOP_2.value:
                self._handle_sub_top_2()
            case _:
                self._handle_default()



    




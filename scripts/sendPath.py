#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

class SendPath(Node):
    '''
    发布器节点类
    '''
    def __init__(self):
        super().__init__('target_pub')
        self.publisher_ = self.create_publisher(PoseStamped, '/setTarget/uavPose', 10)
        self.plannerState_sub_ = self.create_subscription(UInt8, '/plannerState', self.plannerStateCallback, 10)

    def plannerStateCallback(self, msg):
        if msg.data == 0:
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = 1.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 5.0
            self.publisher_.publish(pose)


def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)

    # 创建节点
    minimal_publisher = SendPath()

    # 运行节点
    rclpy.spin(minimal_publisher)

    # 销毁节点，退出ROS2
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

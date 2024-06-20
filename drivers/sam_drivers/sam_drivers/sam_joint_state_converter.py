#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sam_msgs.msg import ThrusterRPMs, ThrusterAngles,Topics
import math
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
class SamJointStateConverter(Node):

    def timer_callback(self):
        state = JointState()
        state.name = ["sam/thruster_joint_1", "sam/thruster_joint_2"]
        duration = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        state.position = [duration*vel for vel in self.velocities]
        self.joint_state_pub.publish(state)

    def thruster_callback(self, msg):
        state = JointState()
        state.name = ["sam/thruster_joint_1", "sam/thruster_joint_2"]
        self.velocities = [2.*math.pi/60.*float(msg.thruster_1_rpm), 2.*math.pi/60.*float(msg.thruster_2_rpm)]
        state.velocity = self.velocities
        self.joint_state_pub.publish(state)

    def vector_callback(self, msg):

        state = JointState()
        state.name = ["sam/shaft_joint1", "sam/shaft_joint2"]
        state.position = [msg.thruster_vertical_radians, msg.thruster_horizontal_radians]
        self.joint_state_pub.publish(state)

    def __init__(self):

        super().__init__("sam_joint_state_converter")
        self.start_time = self.get_clock().now()
        self.velocities = [0., 0.]

        self.thruster_sub = self.create_subscription(ThrusterRPMs,Topics.RPM_CMD_TOPIC, self.thruster_callback,callback_group=ReentrantCallbackGroup(), qos_profile=10)
        # self.vector_sub = self.create_subscription(ThrusterAngles,"~thrust_vector_cmd",  self.vector_callback)
        self.vector_sub = self.create_subscription(ThrusterAngles,Topics.THRUST_VECTOR_CMD_TOPIC,  self.vector_callback,callback_group=ReentrantCallbackGroup(),qos_profile=10)

        self.joint_state_pub = self.create_publisher(JointState,"command_states",  qos_profile=10)

        self.timer = self.create_timer(0.1, self.timer_callback)


def main(args=None):
    rclpy.init(args=args)
    sam_joint_state_converter = SamJointStateConverter()
    executor = MultiThreadedExecutor()
    rclpy.spin(sam_joint_state_converter,executor=executor)
    sam_joint_state_converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

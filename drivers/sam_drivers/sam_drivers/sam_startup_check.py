#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from sam_msgs.msg import PercentStamped, Leak, Topics
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header
# from sam_msgs.msg import  SystemsCheckFeedback, SystemsCheckResult
from sam_msgs.action import SystemsCheck
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup



class StartupCheckServer(Node):

    def test_leak(self):

        self.get_logger().info("Checking leak sensor...")

        # feedback_topic = "/sam/core/leak_fb"
        feedback_topic = Topics.LEAK_TOPIC

        try:
            leak = rclpy.wait_for_message(feedback_topic, Leak, 3.)
        except Exception:
            self.get_logger().info("Could not get leak message on %s, aborting...", feedback_topic)
            self._result.status = "Could not get leak message on %s, aborting..." % feedback_topic
            return False

        if leak.value:
            self.get_logger().info("Got leak sensor and detected leaks, aborting...")
            self._result.status = "Got leak sensor and detected leaks, aborting..."
            return False
        else:
            self.get_logger().info("Got leak sensor and no leaks, seems ok!")
            self._feedback.status = "Got leak sensor and no leaks, seems ok!"

        return True

    def test_pressure(self):

        self.get_logger().info("Checking pressure...")

        feedback_topic = Topics.DEPTH_TOPIC

        try:
            pressure = rclpy.wait_for_message(feedback_topic, FluidPressure, 3.)
        except Exception:
            self.get_logger().info("Could not get pressure on %s, aborting...", feedback_topic)
            self._result.status = "Could not get pressure on %s, aborting..." % feedback_topic
            return False

        self.get_logger().info("Got pressure message with value %f, seems ok!", pressure.fluid_pressure)
        self._feedback.status = "Got pressure message with value %f, seems ok!" % pressure.fluid_pressure

        return True

    def test_lcg(self, lcg_setpoint):

        self.get_logger().info("Checking lcg...")

        self.lcg_cmd.publish(PercentStamped(value=lcg_setpoint, header=self.header)) # publish setpoint 50
        self.get_logger().info("Published lcg setpoint at %f, waiting....", lcg_setpoint)
        rclpy.sleep(5.) # wait for 5s to reach 50
        # get lcg feedback, wait for 1s

        feedback_topic = Topics.LCG_FB_TOPIC

        try:
            lcg_feedback = rclpy.wait_for_message(feedback_topic, PercentStamped, 1.)
        except Exception:
            self.get_logger().info("Could not get feedback on %s, aborting...", feedback_topic)
            self._result.status = "Could not get feedback on %s, aborting..." % feedback_topic
            return False

        if abs(lcg_feedback.value - lcg_setpoint) > 2.:
            self.get_logger().info("Set point was %f and value was %f, aborting test...", lcg_setpoint, lcg_feedback.value)
            self._result.status = "Set point was %f and value was %f, aborting test..." % (lcg_setpoint, lcg_feedback.value)
            return False
        else:
            self.get_logger().info("Set point was %f and value was %f, seems ok!", lcg_setpoint, lcg_feedback.value)
            self._feedback.status = "Set point was %f and value was %f, seems ok!" % (lcg_setpoint, lcg_feedback.value)
            return True

    def test_vbs(self, vbs_setpoint):

        self.get_logger().info("Checking vbs...")

        self.vbs_cmd.publish(PercentStamped(value=vbs_setpoint, header=self.header)) # publish setpoint 50

        self.get_logger().info("Published vbs setpoint at %f, waiting....", vbs_setpoint)

        rclpy.sleep(20.) # wait for 5s to reach 50
        # get vbs feedback, wait for 1s
        feedback_topic = Topics.VBS_FB_TOPIC

        try:
            vbs_feedback = rclpy.wait_for_message(feedback_topic, PercentStamped, 1.)
        except Exception:
            self.get_logger().info("Could not get feedback on %s, aborting...", feedback_topic)
            self._result.status = "Could not get feedback on %s, aborting..." % feedback_topic
            return False

        if abs(vbs_feedback.value - vbs_setpoint) > 2.:
            self.get_logger().info("Set point was %f and value was %f, aborting test...", vbs_setpoint, vbs_feedback.value)
            self._result.status = "Set point was %f and value was %f, aborting test..." % (vbs_setpoint, vbs_feedback.value)
            return False
        else:
            self.get_logger().info("Set point was %f and value was %f, seems ok!", vbs_setpoint, vbs_feedback.value)
            self._feedback.status = "Set point was %f and value was %f, seems ok!" % (vbs_setpoint, vbs_feedback.value)
            return True

    def __init__(self, name):
        super().__init__('sam_startup_check')
        self._action_name = name
        self._as = ActionServer(self, SystemsCheck ,self._action_name, execute_callback=self.execute_cb,goal_callback=self.goal_cb)

        self.lcg_cmd = self.create_publisher(PercentStamped,Topics.LCG_CMD_TOPIC,  callback_group = ReentrantCallbackGroup() ,qos_profile=10)
        self.vbs_cmd = self.create_publisher(PercentStamped,Topics.VBS_CMD_TOPIC, callback_group = ReentrantCallbackGroup() ,qos_profile=10)
        self._feedback = SystemsCheck.Feedback()
        self._result = SystemsCheck.Result()
        # self._as.start()

    def goal_cb(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def execute_cb(self, goal_handle):

        rclpy.sleep(1.) # sleep for 1s to wait for stuff to start up

        self.header = Header()
        self.header.stamp = self.get_clock().now().to_msg()

        self.lcg_cmd.publish(PercentStamped(value=0., header=self.header)) # publish setpoint 0

        rclpy.sleep(1.)

        if not self.test_lcg(50.):
            self._as.set_aborted(self._result)
            return 


        if not self.test_lcg(50.):
            goal_handle.abort()
            return self._result

        goal_handle.publish_feedback(self._feedback)

        if not self.test_lcg(88.):
            goal_handle.abort()
            return self._result

        goal_handle.publish_feedback(self._feedback)

        if not self.test_vbs(0.):
            goal_handle.abort()
            return self._result

        goal_handle.publish_feedback(self._feedback)

        if not self.test_vbs(100.):
            goal_handle.abort()
            return self._result

        goal_handle.publish_feedback(self._feedback)

        if not self.test_pressure():
            goal_handle.abort()
            return self._result

        goal_handle.publish_feedback(self._feedback)

        if not self.test_leak():
            goal_handle.abort()
            return self._result

        goal_handle.publish_feedback(self._feedback)

        self.get_logger().info("All tests successful! Exiting...")
        self._result.status = "All tests successful! Exiting..."
        goal_handle.succeed()

        return self._result

def main(args=None):
    rclpy.init(args=args)

    check_server = StartupCheckServer("sam_startup_check")
    executor = MultiThreadedExecutor()
    rclpy.spin(check_server,executor=executor)

    check_server.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()


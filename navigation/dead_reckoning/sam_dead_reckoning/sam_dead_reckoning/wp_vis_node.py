#!/usr/bin/python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf2_ros
from geodesy import utm
import numpy as np
import tf2_ros
from tf2_ros.buffer import Buffer
from smarc_mission_msgs.msg import GotoWaypoint
import message_filters
from smarc_mission_msgs.msg import Topics

class PublishWPsVis(Node):

    def __init__(self, namespace=None):
        super().__init__('wp_vis_node', namespace=namespace)
        #self.wp_topic = rclpy.get_parameter('~wp_latest_topic', '/sam/smarc_bt/last_wp')
        self.wp_topic = f'/sam0/{Topics.LATEST_WP_TOPIC}'
        self.declare_parameter("utm_frame", "utm")
        self.utm_frame = self.get_parameter("utm_frame").value

        # GPS odom in UTM frame
        self.gps_sam_sub = self.create_subscription(GotoWaypoint,self.wp_topic, self.wp_vis_cb, 10)
        
        # Broadcast UTM to map frame
        self.tf_buffer = Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster(self)
        


    def wp_vis_cb(self, wp_msg):

        if wp_msg.name != '':
            
            try:

                transforms_stamped = self.tf_buffer.lookup_transform(target_frame=self.utm_frame,
                                                                source_frame=wp_msg.name,
                                                                 time=self.get_clock().now().to_msg())
                world_trans = transforms_stamped.transform.translation
                wolrd_rot = transforms_stamped.transform.rotation

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException):

                utm_wp = utm.fromLatLong(wp_msg.lat, wp_msg.lon)
                rot = [0., 0., 0., 1.]

                #rospy.loginfo("WP vis node: broadcasting transform %s to %s" % (self.utm_frame, wp_msg.name))
                self.get_logger().info("WP vis node: broadcasting transform %s to %s" % (self.utm_frame, wp_msg.name))            
                transformStamped = TransformStamped()
                transformStamped.transform.translation.x = utm_wp.easting
                transformStamped.transform.translation.y = utm_wp.northing
                transformStamped.transform.translation.z = 0.
                #transformStamped.transform.rotation = Quaternion(*rot)   
                q = Quaternion()
                q.x = rot[0]
                q.y = rot[1]
                q.z = rot[2]
                q.w = rot[3]    
                transformStamped.transform.rotation = q        
                transformStamped.header.frame_id = self.utm_frame
                transformStamped.child_frame_id = wp_msg.name
                transformStamped.header.stamp = self.get_clock().now().to_msg()
                self.static_tf_bc.sendTransform(transformStamped)

                return
   



def main(args=None, namespace=None):
    rclpy.init(args=args)
    dr_visual = PublishWPsVis(namespace=namespace)
    try:
        rclpy.spin(dr_visual)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main(namespace="sam0")
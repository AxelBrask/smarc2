#!/usr/bin/python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros
from geodesy import utm
import tf2_ros
from tf2_ros.buffer import Buffer
from smarc_mission_msgs.msg import GotoWaypoint
from smarc_mission_msgs.srv import UTMLatLon
from smarc_mission_msgs.msg import Topics
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geographic_msgs.msg import GeoPoint
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

class PublishWPsVis(Node):

    def __init__(self, namespace=None):
        super().__init__('wp_vis_node', namespace=namespace)
        #self.wp_topic = rclpy.get_parameter('~wp_latest_topic', '/sam/smarc_bt/last_wp')
        self.wp_topic = Topics.LATEST_WP_TOPIC
        self.declare_parameter("utm_frame", "utm")
        self.utm_frame = self.get_parameter("utm_frame").value
        call_back_group = ReentrantCallbackGroup()

        self.client = self.create_client(UTMLatLon, Topics.UTM_LATLON_CONVERSION_SERVICE,callback_group=call_back_group)
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('No UTM to lat/lon conversion service available')

        # GPS odom in UTM frame
        self.gps_sam_sub = self.create_subscription(GotoWaypoint,self.wp_topic, self.wp_vis_cb, 10,callback_group=call_back_group)
        
        # Broadcast UTM to map frame
        self.tf_buffer = Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster(self)
        


    def wp_vis_cb(self, wp_msg):
        
        self.get_logger().info("WP vis node callback")
        if wp_msg.name != '':
            
            try:

                transforms_stamped = self.tf_buffer.lookup_transform(target_frame=self.utm_frame,
                                                                source_frame=wp_msg.name,
                                                                 time=self.get_clock().now().to_msg())
                world_trans = transforms_stamped.transform.translation
                wolrd_rot = transforms_stamped.transform.rotation
                self.get_logger().info("WP vis node: broadcasting transform %s to %s" % (self.utm_frame, wp_msg.name))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
                latlon = GeoPoint()
                latlon.latitude = wp_msg.lat
                latlon.longitude = wp_msg.lon
                Latlong_to_utm = UTMLatLon.Request()
                Latlong_to_utm.lat_lon_points=[]
                Latlong_to_utm.lat_lon_points.append(latlon)
        
                future = self.client.call_async(Latlong_to_utm)
                print("Requesting conversion from lat/lon to UTM")
           
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                print("Conversion done")
                utm_wp = future.result().utm_points[0]
                # utm_wp = utm.fromLatLong(wp_msg.lat, wp_msg.lon)

                rot = [0., 0., 0., 1.]

                #rospy.loginfo("WP vis node: broadcasting transform %s to %s" % (self.utm_frame, wp_msg.name))
                           
                transformStamped = TransformStamped()
                transformStamped.transform.translation.x = utm_wp.point.x
                transformStamped.transform.translation.y = utm_wp.point.y
                # transformStamped.transform.translation.x = utm_wp.easting
                # transformStamped.transform.translation.y = utm_wp.northing
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
                self.get_logger().info("WP vis node: broadcasting transform %s to %s" % (self.utm_frame, wp_msg.name)) 
                self.static_tf_bc.sendTransform(transformStamped)

               
   



def main(args=None, namespace=None):
    rclpy.init(args=args)

    vis_node = PublishWPsVis(namespace=namespace)
    executor = SingleThreadedExecutor()
    executor.add_node(vis_node)
    try:
        executor.spin()
        
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main(namespace="sam0")
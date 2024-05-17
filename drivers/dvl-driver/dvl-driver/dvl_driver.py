#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# from waterlinked_a50_ros_driver.msg import DVL as waterlinkedDVLmsg
# from waterlinked_a50_ros_driver.msg import DVLBeam as waterlinkedDVLbeamMsg

from std_srvs.srv import SetBool


from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from smarc_msgs.msg import DVLBeam
from smarc_msgs.msg import DVL

import socket
import json
import time
import select
import traceback
import selectors
import threading
from time import monotonic as time
import os
#Topics
from sam_msgs.msg import Links as SamLinks
from smarc_msgs.msg import Topics as SmarcTopics
from sam_msgs.msg import Topics as SamTopics

__all__ = ["BaseServer", "TCPServer", "UDPServer",
           "ThreadingUDPServer", "ThreadingTCPServer",
           "BaseRequestHandler", "StreamRequestHandler",
           "DatagramRequestHandler", "ThreadingMixIn"]
if hasattr(os, "fork"):
    __all__.extend(["ForkingUDPServer","ForkingTCPServer", "ForkingMixIn"])
if hasattr(socket, "AF_UNIX"):
    __all__.extend(["UnixStreamServer","UnixDatagramServer",
                    "ThreadingUnixStreamServer",
                    "ThreadingUnixDatagramServer"])

# poll/select have the advantage of not requiring any extra file descriptor,
# contrarily to epoll/kqueue (also, they require a single syscall).
if hasattr(selectors, 'PollSelector'):
    _ServerSelector = selectors.PollSelector
else:
    _ServerSelector = selectors.SelectSelector


class DVLDriver(Node):
    

    def __init__(self,namespace = None):
        super().__init__("DVL driver", namespace=namespace)
        self.robot_name = self.get_parameter("robot_name").value
        # self.dvl_frame = rospy.get_param('~dvl_frame', 'sam/dvl_link')
        self.dvl_frame = f"{self.robot_name}_{SamLinks.DVL_LINK}"
        self.dvl_topic = SamTopics.DVL_TOPIC
        self.dvl_ctrl_srv = self.get_parameter('~dvl_on_off_srv', 'core/toggle_dvl')
        # self.dvl_raw_topic = rospy.get_param('~dvl_raw_topic', 'dvl_raw_output')
        self.dvl_raw_topic = SamTopics.DVL_RAW_TOPIC
        # self.relay_topic = rospy.get_param('~relay_topic', 'sam/core/dvl_relay')
        self.dvl_raw_topic = SamTopics.DVL_RELAY_TOPIC

        self.pub_raw = self.create_publisher( self.dvl_raw_topic, String, queue_size=10)
        self.pub_relay = self.create_publisher( self.relay_topic, Bool, queue_size=10)

        # Waterlinked parameters
        self.TCP_IP = self.get_parameter("~ip", "192.168.2.95").value
        self.TCP_PORT = self.get_parameter("~port", 16171)
        self.do_log_raw_data = self.get_parameter("~do_log_raw_data", False)

        # Waterlinked variables
        self.s = None
        self.dvl_on = False
        self.oldJson = ""

        # Topics for debugging
        self.dvl_en_pub = self.create_publisher('dvl_enable', Bool, queue_size=10)

        # Service to start/stop DVL and DVL data publisher
        self.switch_srv = self.create_service(self.dvl_ctrl_srv, SetBool, self.dvl_switch_cb)
        self.dvl_pub = self.create_publisher(self.dvl_topic, DVL, queue_size=10)
        # self.switch = False

        self.data_buffer = b""

        self.timeout = 1.
        self.connected = False
        try:
            while rclpy.ok():
                if self.connected:
                    rlist, _, _ = select.select([self.s], [], [], self.timeout)
                    if self.s in rlist:
                        dvl_msg = self.handle_socket_select()
                        if dvl_msg is not None:
                            self.receive_dvl(dvl_msg)
                else:
                    # rospy.loginfo_throttle(10, "DVL is off")
                    self.get_logger().info("DVL is off")

        except Exception:
            self.get_logger().error(traceback.format_exc())

        finally:
            self.send_relay_msg(relay=False)
            self.close()
            rclpy.sleep(1.)
            self.get_logger().info("DVL driver off")


    def handle_socket_select(self):
        """Handle one request, possibly blocking.

        Respects self.timeout.
        """

        # Support people who used socket.settimeout() to escape
        # handle_request before self.timeout was available.
        timeout = self.s.gettimeout()

        if timeout is None:
            timeout = self.timeout
        elif self.timeout is not None:
            timeout = min(timeout, self.timeout)
        if timeout is not None:
            deadline = time() + timeout

        return self.extract_data()

        # Wait until a request arrives or the timeout expires - the loop is
        # necessary to accommodate early wakeups due to EINTR.

        #with _ServerSelector() as selector:
        #       selector.register(self, selectors.EVENT_READ)

        #while True:
        #        ready = selector.select(timeout)
        #        if ready:
        #                return self.extract_data()
        #        else:
        #                if timeout is not None:
        #                        timeout = deadline - time()
        #                        if timeout < 0:
        #                                rospy.logwarn("Socket timeout")
        #                                return


    def extract_data(self):
        raw_data = b''

        while not b'\n' in raw_data and self.connected:
            try:
                rec = self.s.recv(1)
                if len(rec) == 0:
                    self.get_logger().error("rec == 0 received")
                    continue

                raw_data = raw_data + rec
            
            except IOError as err:
                self.get_logger().error("[DVL Driver] Damping last DVL msg: {}".format(err))
                return None

        if self.connected:

            raw_data = raw_data.decode("utf-8")
            raw_data = self.oldJson + raw_data
            self.oldJson = ''
            raw_data = raw_data.split('\n')
            self.oldJson = raw_data[1]
            raw_data = raw_data[0]
            return raw_data
        
        else:

            return None

        # data = self.s.recv(1024)
        # if data:  # If data is empty, the remote end has closed the socket
        #       self.data_buffer += data
        #       while b'\n' in self.data_buffer:
        #               line, self.data_buffer = self.data_buffer.split(b'\n', 1)
        #               self.receive_dvl(line.decode('utf-8'))



    # def handle_socket(self,sock):
    #       while True:
    #               try:
    #                       data = sock.recv(1024)
    #                       if not data:  # If data is empty, the remote end has closed the socket
    #                               # sock.close()
    #                               break
    #                       self.data_buffer += data
    #                       while b'\n' in self.data_buffer:
    #                               line, self.data_buffer = self.data_buffer.split(b'\n', 1)
    #                               self.receive_dvl(line.decode('utf-8'))
    #               except socket.timeout as err:
    #                       rospy.logerr("No data received? {}".format(err))
    #               except socket.error as err:
    #                       self.connect()
    #                       sock = self.s
    #                       rospy.sleep(2)



    def receive_dvl(self, raw_data):

        theDVL = DVL()
        beam0 = DVLBeam()
        beam1 = DVLBeam()
        beam2 = DVLBeam()
        beam3 = DVLBeam()
        data = json.loads(raw_data)

        # edit: the logic in the original version can't actually publish the raw data
        # we slightly change the if else statement so now
        # do_log_raw_data is true: publish the raw data to /dvl/json_data topic, fill in theDVL using velocity data and publish to dvl/data topic
        # do_log_raw_data is true: only fill in theDVL using velocity data and publish to dvl/data topic

        if self.do_log_raw_data:
            self.pub_raw.publish(raw_data)

        theDVL.header.stamp = self.get_clock().now().to_msg()
        theDVL.header.frame_id = self.dvl_frame

        theDVL.velocity.x = data["vx"]
        theDVL.velocity.y = data["vy"]
        theDVL.velocity.z = data["vz"]
        theDVL.velocity_covariance[0] = data["covariance"][0][0]
        theDVL.velocity_covariance[4] = data["covariance"][1][1]
        theDVL.velocity_covariance[8] = data["covariance"][2][2]
        theDVL.altitude = data["altitude"]

        # Todo : Add beam covariances (not available for waterlinked)

        beam0.range = data["transducers"][0]["distance"]
        beam0.velocity = data["transducers"][0]["velocity"]

        beam1.range = data["transducers"][1]["distance"]
        beam1.velocity = data["transducers"][1]["velocity"]

        beam2.range = data["transducers"][2]["distance"]
        beam2.velocity = data["transducers"][2]["velocity"]

        beam3.range = data["transducers"][3]["distance"]
        beam3.velocity = data["transducers"][3]["velocity"]

        theDVL.beams = [beam0, beam1, beam2, beam3]

        if data['velocity_valid']:
            self.dvl_pub.publish(theDVL)


    def dvl_switch_cb(self, request : SetBool,response: SetBool):

        # self.switch = True
        # rospy.loginfo(f"[DVL Driver] DVL request received : {switch_msg.data}")


        if request.data:
            self.send_relay_msg(relay=True)
            rclpy.sleep(30.)
            self.connected = self.connect()
        else:
            self.connected = False
            self.send_relay_msg(relay=False)
            self.close()

        response.success = self.connected
        return response


    def send_relay_msg(self, relay):

        if relay:
            self.get_logger().info(f"[DVL Driver] Powering on. This will take 30 sec")
        else:
            self.get_logger().info(f"[DVL Driver] Powering off")

        relay_msg = Bool()
        relay_msg.data = relay
        self.pub_relay.publish(relay_msg)


    def connect(self) -> bool:
        """
        Connect to the DVL
        """
        try:
            
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_address = self.s.getsockname()
            self.get_logger().info("[DVL Driver] socket created %s", self.server_address)
            self.s.connect((self.TCP_IP, self.TCP_PORT))
            connected = True
            self.get_logger().info("[DVL Driver] Successfully connected")

        except socket.error as err:
            self.get_logger().error("[DVL Driver] Could not connect, DVL might be booting? {}".format(err))
            connected = False

        return connected


    def close(self) -> bool:
        """
        Close the connection to the DVL
        """

        try:
            self.s.close()
            self.get_logger().info("[DVL Driver] Successfully disconnected")
            return True
        except Exception as e:
            self.get_logger().error("[DVL Driver] Could not disconnect, DVL might be booting? {}".format(e))
            return False



    def set_config(self,
            speed_of_sound = None,
            mounting_rotation_offset = None,
            acoustic_enabled = None,
            dark_mode_enabled = None,
            range_mode = None):

        """
        Set the configuration of the DVL

        Values not set will be left blank and will not be changed
        """

        # Validate input
        valid_input = True
        if speed_of_sound is not None and (speed_of_sound < 0 ):
            self.get_logger().error("Invalid speed of sound")
            valid_input = False
        if mounting_rotation_offset is not None and (mounting_rotation_offset < 0 or mounting_rotation_offset > 360):
            self.get_logger().error("Invalid mounting rotation offset")
            valid_input = False
        if acoustic_enabled is not None and (acoustic_enabled != "y" and acoustic_enabled != "n"):
            self.get_logger().error("Invalid acoustic enabled value")
            valid_input = False
        if dark_mode_enabled is not None and (dark_mode_enabled != "y" and dark_mode_enabled != "n"):
            self.get_logger().error("Invalid dark mode enabled value")
            valid_input = False
        if range_mode is not None and (range_mode != "auto"):
            self.get_logger().error("Invalid range mode")
            valid_input = False

        if not valid_input:
            return False

        paramaters = [speed_of_sound, mounting_rotation_offset, acoustic_enabled, dark_mode_enabled, range_mode]
        command_string = "wcs"
        for i, val in enumerate(paramaters):

            command_string +=","
            command_string+= str(val) if val is not None else ""

        command_string += "\n"

        self.get_logger().info("[DVL Driver] Sending command: {}".format(command_string))
        self.s.send(command_string.encode())

def main(args=None,namespace = None):
    rclpy.init(args=args)
    
    dvl_drive = DVLDriver(namespace)
    dvl_drive.get_logger().info  ('[DVL Driver] Starting DVL driver')
    try:
        DVLDriver()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node

import threading

import time

import math as m

from std_msgs.msg import Header, String

from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped

import cv2
from cv_bridge import CvBridge 

from .kinematic_helper import ARM_5DOF, ARM_INFO

import struct

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .user_input import UserInputHandler


class MainArmNode(Node):


    def __init__(self):    

        super().__init__('main_arm_node')

        self.delay_ms = 50
        self.delay_s = self.delay_ms / 1000.0
        self.timer = self.create_timer(self.delay_s, self.timerCallback) # start ros2 timer

        self.user_command = 1 # initialize user_command to a non-None value
        self.user_input_thread = threading.Thread(target=self.userInputThread, daemon=True) # start separate user input thread
        self.user_input_thread.start()

        joint_params = [200, 400, 400, 100, 100] # limb lengths in mm
        home_angles = [0.0, m.pi/2, 0.0, 0.0, 0.0] # home angles for each joint
        self.robot = ARM_5DOF(joint_params, home_angles) # init robot
        self.input_handler = UserInputHandler(self.robot) # init user input handler

        # ros2 pubs and subs
        self.joint_pub = self.create_publisher(PointCloud2, '/chessarm/out/joint_coordinates', 10)
        self.link_pub = self.create_publisher(Path, '/chessarm/out/link_coordinates', 10)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.publishStaticTF()

        # confirm node initialization after all steps complete
        print("Main control node initialized")

        # set user_command to None to start the user input loop
        self.user_command = None


    def timerCallback(self):

        if self.user_command:
            self.input_handler.parseUserInput(self.user_command)
            self.user_command = None

        self.updatePathMovement()
        self.robot.joint_coordinates = self.robot.solveFK(self.robot.joint_angles)
        self.robot.joint_coordinates_m = [[coord/1000.0 for coord in joint] for joint in self.robot.joint_coordinates]
        self.publishJointsToRviz()


    def userInputThread(self):
        time.sleep(0.1) # small delay to allow node to initialize
        while rclpy.ok():
            # wait until after command is parsed to ask for a new one
            if self.user_command is None:
                try:
                    user_input = input("Enter command: ")
                    self.user_command = user_input
                except EOFError:
                    break


    def publishJointsToRviz(self):
        
        link_msg = Path()
        joint_msg = PointCloud2()

        link_msg.header.frame_id = "map"
        link_msg.header.stamp = self.get_clock().now().to_msg()
        link_msg.poses = [self.toPoseStamped(joint) for joint in self.robot.joint_coordinates_m]

        joint_msg = self.toPC2(self.robot.joint_coordinates_m, self.robot.joint_colors)

        self.joint_pub.publish(joint_msg)
        self.link_pub.publish(link_msg)


    def updatePathMovement(self):

        if self.robot.movement_type != ARM_INFO.PATH_SPACE:
            return

        if not self.robot.sliced_path_points:
            self.robot.movement_type = ARM_INFO.JOINT_SPACE
            return
        
        self.robot.joint_angles = self.robot.sliced_path_angles[0]
        self.robot.sliced_path_angles.pop(0)
        self.robot.sliced_path_points.pop(0)


    def packRGB(self, r, g, b):
        return struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]


    def toPC2(self, points, colors):

        # restructure points to be x,y,z,rgb (packed)
        for i in range(len(points)):
            #print(colors[i]) # [color name, (r,g,b)]
            r, g, b = colors[i]
            points[i] = (points[i][0], points[i][1], points[i][2], self.packRGB(r, g, b))

        header = Header()
        header.stamp = self.get_clock().now().to_msg() 
        header.frame_id = 'map'

        cloud = pc2.create_cloud(
            header,
            fields=[
                PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ],
            points=points,
        )

        return cloud
    

    def toPoseStamped(self, tuple):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tuple[0]
        pose.pose.position.y = tuple[1]
        pose.pose.position.z = tuple[2]
        return pose
    

    def publishStaticTF(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)  
        self.get_logger().info('Published static transform: map -> map')


    def currentTimeMilliseconds(self):
        return int(round(time.time() * 1000))
    


def main(args=None):
    rclpy.init(args=args)
    node = MainArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

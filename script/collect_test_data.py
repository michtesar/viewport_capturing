#!/usr/bin/python
from math import radians, degrees, atan, sin, tan, acos, sqrt, cos, asin
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from rospy import init_node, sleep, loginfo, Subscriber, Rate
from capek_pycommander.capek_robot import CapekRobotCommander
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy

def euler_to_quaternion(x, y, z):
    """Enter Euler values in degrees X, Y, Z order 
    and get ROS compatible Quternion in X, Y, Z and 
    W order
    
    Arguments:
        x {float} -- Rotation angle (deg) in X axis
        y {float} -- Rotation angle (deg) in Y axis
        z {float} -- Rotation angle (deg) in Z axis
    
    Returns:
        list(float) -- ROS compatible quaternion X, Y, Z and W
    """
    euler = [radians(x), radians(y), radians(z)]
    return quaternion_from_euler(euler[0], euler[1], euler[2])

def construct_pose(position, quaternion, frame_id="/world"):
    """Construct a PoseStamped message based on given
    position and orientation (quaternion) for
    the reference frame. Default is frame '/world'
    
    Arguments:
        position {list(float)} -- Translation in X, Y and Z
        quaternion {list(float)} -- Orientation in X, Y, Z and W
        frame_id {str} -- Frame in which to plan movement, default '/world'
    
    Returns:
        Pose -- ROS geometry message
    """
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    return pose

def compute_camera_position(angles, planar, dist, d):
   """Computes a camera position for a proper
   viewport for robot.

   Arguments:
       angles {list(float)} -- Vertical camera angles (angles)
       planar {list(float)} -- Planar camera positoins (left, center, right)
       dist {float} -- Fixed distance from the scene up and down (in axis Y)
       d {float} -- Distance from baselink to scene center

   Returns:
       list (PoseStamped) -- List of PoseStamped poses for robot in '/world' frame
   """
   camera_poses = []
   x_angle = 0
   for i in angles:
       i = 90 - i
       x = dist * sin(radians(i))
       z = dist * cos(radians(i))
       for j in planar:
           y = j
           if j < 0:
               x2 = sqrt(x**2 + y**2)
               z_angle = -90+degrees(asin(x/x2))
           elif j > 0:
               x2 = sqrt(x**2 + y**2)
               z_angle = 90-degrees(asin(x/x2))
           else:
               z_angle = 0
           euler = [x_angle, 180-i, -z_angle]
           quaternion = euler_to_quaternion(x=euler[0], y=euler[1], z=euler[2])
           position = [d-x, y, z]
           camera_poses.append(construct_pose(position, quaternion))
   return camera_poses

if __name__ == "__main__":
    # Connect and initialize Capek robot    
    init_node("collect_test_data")
    crc = CapekRobotCommander("r1")
    crc.group.set_planner_id("RRTConnectkConfigDefault")
    crc.group.set_pose_reference_frame("r1_link_0")

    # Start the robot with L position
    crc.move_l_position()

    # Compute robot poses based on camera settings    
    camera_poses = compute_camera_position(
        angles=[40.0, 45.0, 50.0, 55.0],
        planar=[-0.3, 0.0, +0.3],
        dist=0.7,
        d=1.1
    )

    # Set smaller speed
    crc.set_speed(0.5)

    SCENE = 0

    bridge= CvBridge()
    for i in range(200):
        # Go over all poses and execute one by one
        for index, position in enumerate(camera_poses):
            loginfo("Pose number {}".format(index+1))

            # Set robot pose and execute
            crc.group.set_pose_target(position.pose)
            crc.group.go(wait=True)

            IMAGE = rospy.wait_for_message("/pylon_camera_node/image_raw", Image)
            IMAGE = bridge.imgmsg_to_cv2(IMAGE, "bgr8")
            cv2.imwrite("scn_{}_img_{}-{}.png".format(SCENE, index / 3, index), IMAGE)
        SCENE += 1

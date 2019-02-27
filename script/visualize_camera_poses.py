#!/usr/bin/python
import rospy
import numpy
import tf


camera_poses = numpy.load("camera_poses.npy")
if camera_poses.any():
	rospy.loginfo("Camera poses was loaded!")

tf_publisher = tf.TransformBroadcaster()


def parse_pose(pose):
	translation = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
	orientation = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
	return translation, orientation
	
def publish_transformation(pose, tf_name, base_tf):
	translation, orientation = parse_pose(pose)
	tf_publisher.sendTransform(
		translation,
		orientation,
		rospy.Time.now(),
		tf_name,
		base_tf
    )


def publish_static(translation, tf_name, base_tf):
	tf_publisher.sendTransform(
		translation,
		(0.0, 0.0, 0.0, 1.0),
		rospy.Time.now(),
		tf_name,
		base_tf
    )


if __name__ == '__main__':
    rospy.init_node("visualize_camera_poses_node")
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	publish_static((1.1, 0.0, 0.0), "scene_center", "iiwa_link_0")
        rospy.loginfo("Publishing TF")
        for i, p in enumerate(camera_poses):
            publish_transformation(p, str(i+1), "iiwa_link_0")
        rate.sleep()


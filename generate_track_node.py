import numpy as np
from PIL import Image
import os
import rospy
from tf_conversions import posemath
# from dslam_sp.msg import image_depth, PRrepresentor
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, PoseArray
from cv_bridge import CvBridge, CvBridgeError
import cv2


current_pose = Pose()
current_pose.orientation.w = 1
print(current_pose)
posearray = PoseArray()
posearray.header.frame_id = "map"
posearray_pub = None
trajectedge_array = []

class TrajectEdge (object):
    def __init__(self, fromid, toid, transform):
        self.fromid = fromid
        self.toid = toid
        self.transform = transform


def callback(data):
    global current_pose, posearray, posearray_pub, trajectedge_array
    tf_base_to_curr = PoseStamped()
    tf_base_to_curr.header = data.header
    tf_base_to_curr.pose.position = data.transform.translation
    tf_base_to_curr.pose.orientation = data.transform.rotation
    current_pose_tmp = ( posemath.fromMsg(current_pose) *  posemath.fromMsg(tf_base_to_curr.pose) )
    current_pose = posemath.toMsg(current_pose_tmp)

    trajectedge_array.append( TrajectEdge(data.child_frame_id, data.header.frame_id,  data.transform) )

    posearray.poses.append(current_pose)
    posearray_pub.publish(posearray)
    print("==================================")
    print(tf_base_to_curr.pose)
    print(current_pose)


def main():
    global posearray_pub
    rospy.init_node('generate_track_py', anonymous=True)
    rospy.Subscriber("/visual_odometry/transform_relative", TransformStamped, callback)
    posearray_pub = rospy.Publisher("posearray",PoseArray, queue_size=3)
    rospy.spin()


if __name__ == '__main__':
    main()
    # k1 = Pose()
    # k1.orientation.w = 1
    # k2 = Pose()
    # k2.orientation.w = 1

    # f = posemath.fromMsg(k1) * posemath.fromMsg(k2)
    # print f

    # [x, y, z, w] = f.M.GetQuaternion()
    # print x,y,z,w

    # pose = posemath.toMsg(f)
    # print pose







    
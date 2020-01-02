import tracknode
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, PoseArray
import time

import sys                                                              
import signal


def main():
    signal.signal(signal.SIGINT, tracknode.quit)                                
    signal.signal(signal.SIGTERM, tracknode.quit)

    rospy.init_node('publish_rel_node', anonymous=True)
    relpose_pub = rospy.Publisher("relpose",TransformStamped, queue_size=3)
    rel_pose_array = PoseArray()
    tracknode.RTtxt2Posearray("./testdata/scaled_09_rel.txt",rel_pose_array)
    from_id = 1
    to_id = 2
    for pose in rel_pose_array.poses:
        time.sleep(0.1)
        transformtmp = TransformStamped()
        transformtmp.header.frame_id = str(to_id)
        transformtmp.child_frame_id = str(from_id)
        from_id = to_id
        to_id += 1
        transformtmp.transform.rotation = pose.orientation
        transformtmp.transform.translation = pose.position
        relpose_pub.publish(transformtmp)
    




if __name__ == "__main__":
    main()
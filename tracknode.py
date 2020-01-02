import numpy as np
from PIL import Image
import os
import rospy
from tf_conversions import posemath
# from dslam_sp.msg import image_depth, PRrepresentor
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, PoseArray, Transform
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import sys                                                              
import signal
import copy
import gtsam
 
COVAR_STR = "1.000000 0.000000 0.000000 0.000000 0.000000 0.000000   1.000000 0.000000 0.000000 0.000000 0.000000   1.000000 0.000000 0.000000 0.000000   1.000000 0.000000 0.000000   1.000000 0.000000   1.000000"


def quit(signum, frame):
    print ''
    print 'stop fusion'
    sys.exit()

def isrot(R, tol):
    if not ( R.shape[0] ==3 and R.shape[1] ==3 ):
        print("R is not 3x3")
        return False
    else :
        RTR = R*R.transpose()
        resmat = RTR - np.eye(3)
        res = np.linalg.norm(resmat)
        if res > tol:
            print("R is not orthonormal, i.e. ||(R''R-I)|| > {}".format(tol) )
            return False
        elif (np.linalg.det(R) < 0 ):
            print("R determinant < 0")
            return False
        return True


def np2Transform(nparray):
    npmat = nparray.reshape((3,4))
    R = npmat[0:3,0:3]
    assert ( isrot(R,1) )
    # T = npmat[0:3,3]
    posetmp = posemath.fromMatrix(npmat)
    transformtmp = posemath.toMsg(posetmp)
    return transformtmp

def gstamPose2Transform(gstamPose):
    npmat = np.zeros((3,4))
    npmat[0:3,0:3] = gstamPose.rotation().matrix()
    npmat[0,3] = gstamPose.translation().x()
    npmat[1,3] = gstamPose.translation().y()
    npmat[2,3] = gstamPose.translation().z()
    posetmp = posemath.fromMatrix(npmat)
    transformtmp = posemath.toMsg(posetmp)
    return transformtmp


def RTtxt2Posearray(txtfname, pose_array):
    posearrays = np.loadtxt(txtfname,dtype=np.float32)
    assert(posearrays.shape[1] == 12)
    index = 1
    for lines in posearrays:
        print(index)
        pose_array.poses.append(np2Transform(lines) )
        index += 1
    return pose_array
    

def Posearray2G2O(g2ofname, pose_array):
    pointID = 123654
    index = 1
    prev_pose = Pose()
    prev_pose.orientation.w = 1
    rel_pose = Pose()

    with open(g2ofname, "w") as g2ofile:
        for pose in pose_array.poses:
            if index == 1:
                pass
            else :
                rel_pose = posemath.toMsg ( (posemath.fromMsg(prev_pose).Inverse() * posemath.fromMsg(pose)) )
                # tmp = posemath.toMsg( posemath.fromMsg(prev_pose)*posemath.fromMsg(rel_pose) )
                prev_pose = pose
                print >> g2ofile, "EDGE_SE3:QUAT {id1} {id2} {tx} {ty} {tz} {rx} {ry} {rz} {rw}  {COVAR_STR}".format(id1=pointID-1,id2=pointID, tx =rel_pose.position.x , ty =rel_pose.position.y , tz =rel_pose.position.z, rx =rel_pose.orientation.x, ry =rel_pose.orientation.y, rz =rel_pose.orientation.z, rw =rel_pose.orientation.w, COVAR_STR=COVAR_STR)
            
            print >> g2ofile, "VERTEX_SE3:QUAT {pointID} {tx} {ty} {tz} {rx} {ry} {rz} {rw}".format(pointID=pointID,tx=pose.position.x,ty=pose.position.y,tz=pose.position.z, 
                rx = pose.orientation.x, ry = pose.orientation.y, rz = pose.orientation.z, rw = pose.orientation.w)

            
            pointID += 1
            index += 1

def Globalarray2Relarray(global_array, rel_array):
    prev_pose = Pose()
    prev_pose.orientation.w = 1
    current_pose = Pose()
    current_pose.orientation.w = 1
    rel_pose = Pose()
    posearray = PoseArray()
    index = 1
    RT_array = []
    for pose in global_array.poses:
        if index == 1:
            pass
        else :
            rel_pose = posemath.toMsg ( (posemath.fromMsg(prev_pose).Inverse() * posemath.fromMsg(pose)) )
            rel_array.poses.append(rel_pose)
            prev_pose = pose

            current_pose_RT = ( posemath.fromMsg(current_pose) *  posemath.fromMsg(rel_pose) )
            current_pose = posemath.toMsg(current_pose_RT)

            posearray.poses.append(current_pose)
            RT_array.append(current_pose_RT)

        index+=1
    return rel_array
    




def Global2Rel(globalfname, pose_array):
    global_array = PoseArray()
    RTtxt2Posearray(globalfname,global_array)
    Globalarray2Relarray(global_array, pose_array)
    return pose_array

def Global2Relfile(globalfname, relfname):
    pose_array = PoseArray()
    Global2Rel(globalfname, pose_array)
    with open(relfname, "w") as relfile:
        for pose in pose_array.poses:
            outarray = posemath.toMatrix( posemath.fromMsg(pose) )
            outonedim = outarray[0:3,:].reshape((12))
            for index in range(11):
                relfile.write("{} ".format(outonedim[index]) )
            relfile.write("{}\n".format(outonedim[11]) )
        relfile.close()

def Relarray2Globalarray(rel_array, global_array):
    current_pose = Pose()
    current_pose.orientation.w = 1
    global_array.poses.append(current_pose)
    for pose in rel_array.poses:
        current_pose = posemath.toMsg( posemath.fromMsg(current_pose) *  posemath.fromMsg(pose) )
        global_array.poses.append(current_pose)
    return global_array


def Rel2Global(relfname, pose_array):
    rel_array = PoseArray()
    RTtxt2Posearray(relfname, rel_array)
    Relarray2Globalarray(rel_array, pose_array)


def Rel2Globalfile(relfname, globalfname ):
    pose_array = PoseArray()
    Rel2Global(relfname, pose_array)
    with open(globalfname, "w") as relfile:
        for pose in pose_array.poses:
            outarray = posemath.toMatrix( posemath.fromMsg(pose) )
            outonedim = outarray[0:3,:].reshape((12))
            for index in range(11):
                relfile.write("{} ".format(outonedim[index]) )
            relfile.write("{}\n".format(outonedim[11]) )
        relfile.close()


def Loop2G2O(g2ofname, transform_loops):
    with open(g2ofname, "a") as g2ofile:
        for transform_loop in transform_loops:
            print >> g2ofile, "EDGE_SE3:QUAT {id1} {id2} {tx} {ty} {tz} {rx} {ry} {rz} {rw}  {COVAR_STR}".format(id1=transform_loop.child_frame_id,id2=transform_loop.header.frame_id, tx =transform_loop.transform.translation.x , ty =transform_loop.transform.translation.y , tz =transform_loop.transform.translation.z, rx =transform_loop.transform.rotation.x, ry =transform_loop.transform.rotation.y, rz =transform_loop.transform.rotation.z, rw =transform_loop.transform.rotation.w, COVAR_STR=COVAR_STR)

def G2O2Posearray(g2ofname,pose_array):
    with open(g2ofname) as g2ofile:
        for line in g2ofile:
            spltline = line.split(' ')
            # print(spltline)
            if (spltline[0] == 'VERTEX_SE3:QUAT'):
                nparray = np.array(spltline[2:9]).astype(float)
                tmppose = Pose()
                tmppose.position.x = nparray[0]
                tmppose.position.y = nparray[1]
                tmppose.position.z = nparray[2]
                tmppose.orientation.x = nparray[3]
                tmppose.orientation.y = nparray[4]
                tmppose.orientation.z = nparray[5]
                tmppose.orientation.w = nparray[6]
            pose_array.poses.append(tmppose)
    return pose_array


def vector6(x, y, z, a, b, c):
    """Create 6d double numpy array."""
    return np.array([x, y, z, a, b, c], dtype=np.float)

def gtsamOpt(inputfname, outputfname):

    graph, initial = gtsam.readG2o(inputfname, True)
        # Add Prior on the first key
    priorModel = gtsam.noiseModel_Diagonal.Variances(vector6(1e-6, 1e-6, 1e-6,
                                                            1e-4, 1e-4, 1e-4))

    print("Adding prior to g2o file ")
    graphWithPrior = graph
    firstKey = initial.keys().at(0)
    graphWithPrior.add(gtsam.PriorFactorPose3(firstKey, gtsam.Pose3(), priorModel))

    params = gtsam.GaussNewtonParams()
    params.setVerbosity("Termination")  # this will show info about stopping conds
    optimizer = gtsam.GaussNewtonOptimizer(graphWithPrior, initial, params)
    result = optimizer.optimize()
    print("Optimization complete")

    print("initial error = ", graphWithPrior.error(initial))
    print("final error = ", graphWithPrior.error(result))

    print("Writing results to file: ", outputfname)
    graphNoKernel, _ = gtsam.readG2o(inputfname, True)
    gtsam.writeG2o(graphNoKernel, result, outputfname)
    print ("Done!")

def gtsamOpt2Posearray(inputfname, pose_array):
    pointID = 123654
    graph, initial = gtsam.readG2o(inputfname, True)
        # Add Prior on the first key
    priorModel = gtsam.noiseModel_Diagonal.Variances(vector6(1e-6, 1e-6, 1e-6,
                                                            1e-4, 1e-4, 1e-4))

    print("Adding prior to g2o file ")
    graphWithPrior = graph
    firstKey = initial.keys().at(0)
    graphWithPrior.add(gtsam.PriorFactorPose3(firstKey, gtsam.Pose3(), priorModel))

    params = gtsam.GaussNewtonParams()
    params.setVerbosity("Termination")  # this will show info about stopping conds
    optimizer = gtsam.GaussNewtonOptimizer(graphWithPrior, initial, params)
    result = optimizer.optimize()
    print("Optimization complete")

    print("initial error = ", graphWithPrior.error(initial))
    print("final error = ", graphWithPrior.error(result))


    resultPoses = gtsam.allPose3s(result)
    for i in range(pointID, pointID+resultPoses.size()):
        pose_array.poses.append(gstamPose2Transform(resultPoses.atPose3(i) ) )
    return pose_array


if __name__ == '__main__':
    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)

    Global2Relfile('testdata/scaled_09.txt', 'testdata/scaled_09_rel.txt')

    transformloop = []
    posearray = PoseArray()
    posearray.header.frame_id = "map"
    rospy.init_node('generate_track_py', anonymous=True)
    posearray_orig_pub = rospy.Publisher("posearray_orig",PoseArray, queue_size=3)
    posearray_optm_pub = rospy.Publisher("posearray_optm",PoseArray, queue_size=3)
    posearray_orig = copy.deepcopy(RTtxt2Posearray('testdata/scaled_09.txt',posearray) )
    posearray.poses = []
    pointID = 123654
    
    for index in range(1578,1579):
        tmpt = TransformStamped()
        tmpt.child_frame_id = pointID+index - 1578
        tmpt.header.frame_id = pointID+index
        tmpt.transform.rotation.w = 1
        transformloop.append(tmpt)


    Posearray2G2O("/tmp/tmplog.g2o",posearray_orig)
    Loop2G2O("/tmp/tmplog.g2o", transformloop )
    
    # gtsamOpt("./tmplog.g2o","./resultlog.g2o")


    posearray_optm = copy.deepcopy(gtsamOpt2Posearray("/tmp/tmplog.g2o",posearray) )


    while True:
        posearray_orig_pub.publish(posearray_orig)
        posearray_optm_pub.publish(posearray_optm)
        time.sleep(1)
        print("12345")
    
    # rospy.spin()







    
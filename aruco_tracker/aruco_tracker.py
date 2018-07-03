#!/usr/bin/env python
import rospy
import time
import rospkg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply
import numpy as np
import cv2
import cv2.aruco as aruco

# Load previously saved camera calibration
rospack = rospkg.RosPack()
with np.load(rospack.get_path('aruco_tracker')+'/aruco_tracker/calibration_files/wide.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

cap = cv2.VideoCapture(1)

def aruco_tracker():
    pubPose = rospy.Publisher('/aruco_tracker/pose', PoseStamped, queue_size=1 )
    pubID = rospy.Publisher('/aruco_tracker/droneID', Int16 , queue_size=1 )

    rospy.init_node('aruco_tracker', anonymous=True)
    rate = rospy.Rate(30) # 10hz

    #Font For OSD
    font = cv2.FONT_HERSHEY_SIMPLEX

    while not rospy.is_shutdown():
        try:
            #Read frame of webcamera
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #Define ArUco dictionary
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
            parameters = aruco.DetectorParameters_create()

            #Detect Markers
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids:
                #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
                rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners[0], 0.05, mtx, dist) 

                #RPY to Quaternion Conversion
                q = quaternion_from_euler(rvec[0][0][1],rvec[0][0][2],rvec[0][0][0])
		#Rotate Roll 180 Degrees
		q_rot = quaternion_from_euler(3.16, 0, 0)
		q_new = quaternion_multiply(q_rot, q)

		#Build poseStamped
                poseStamped=PoseStamped()
                poseStamped.header.frame_id = "/vislam"
                poseStamped.header.stamp = rospy.Time.now()
                poseStamped.pose.position.x = float(tvec[0][0][1]*-1)
                poseStamped.pose.position.y = float(tvec[0][0][0])
                poseStamped.pose.position.z = float(tvec[0][0][2])
                poseStamped.pose.orientation.x = q_new[0]
                poseStamped.pose.orientation.y = q_new[1]
                poseStamped.pose.orientation.z = q_new[2]
                poseStamped.pose.orientation.w = q_new[3]

                droneID = int(ids[0])

                #Publish Pose, ID and take a nap
                pubPose.publish(poseStamped)
                pubID.publish(droneID)

            rate.sleep()
        except Exception as e:
            #You mad bro?
            rospy.loginfo(e)

if __name__ == '__main__':
    try:
        aruco_tracker()
    except rospy.ROSInterruptException:
        pass

cap.release()

import numpy as np
import cv2
import cv2.aruco as aruco

# Load previously saved camera calibration 
with np.load('../calibration_files/MBP.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

cap = cv2.VideoCapture(1)

while (True):
    #Read Camera Input
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()

    #Detect Markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    #Font For OSD
    font = cv2.FONT_HERSHEY_SIMPLEX

    if np.all(ids != None):
        
        centered = 0
        #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
        rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners[0], 0.05, mtx, dist) 

        ### Draw Boarder Around ArUco
        aruco.drawDetectedMarkers(frame, corners) 

        ### Draw Axis
        aruco.drawAxis(frame, mtx, dist, rvec[0], tvec[0], 0.1) 

        ###### DRAW Drone Data #####
        cv2.putText(frame, "Drone" + str(ids[0][0]), (10,50), font, 1, (255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame, "Pitch: " + str(round(rvec[0][0][2],4)),(10,100),font,1,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame, "Roll: " + str(round(rvec[0][0][1],4)),(10,150),font,1,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame, "Yaw: " + str(round(rvec[0][0][0],4)),(10,200),font,1,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame, "X: " + str(round(tvec[0][0][1],4)*-1),(10,250),font,1,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame, "Y: " + str(round(tvec[0][0][0],4)),(10,300),font,1,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame, "Z: " + str(round(tvec[0][0][2],4)),(10,350),font,1,(255,255,255),2,cv2.LINE_AA)

        if tvec[0][0][0] >= 0:
            if tvec[0][0][0] == 0:
                cv2.putText(frame, "-",(350,350),font,1,(255,255,255),2,cv2.LINE_AA)
                centered = 1
            else:
                cv2.putText(frame, ">",(363,350),font,1,(255,255,255),2,cv2.LINE_AA)
        else:
            cv2.putText(frame, "<",(330,350),font,1,(255,255,255),2,cv2.LINE_AA)
        
        if tvec[0][0][1] >= 0:
            if tvec[0][0][0] == 0:
                if centered:
                    cv2.putText(frame, "+",(350,350),font,1,(255,255,255),2,cv2.LINE_AA)
                else: 
                    cv2.putText(frame, "|",(350,350),font,1,(255,255,255),2,cv2.LINE_AA)
            else:
                cv2.putText(frame, "^",(350,332),font,1,(255,255,255),2,cv2.LINE_AA)
        else:
            cv2.putText(frame, "v",(350,370),font,1,(255,255,255),2,cv2.LINE_AA)

        # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

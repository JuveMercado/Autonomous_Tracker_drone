#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
import ros_numpy
# Import OpenCV libraries and tools
import numpy as np
import cv2
#from cv_bridge import CvBridge, CvBridgeError

#red ball
colorLowerRed = [135, 54, 109]
colorUpperRed = [179, 255, 255]

lowerLimitRed = np.array(colorLowerRed)
upperLimitRed = np.array(colorUpperRed)

# Print "Hello!" to terminal
print ("Hello!")

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('Subscriber', anonymous=True)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

def empty(a):
    pass

cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 240)
cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)
cv2.createTrackbar("Sat Min", "TrackBars", 110, 255, empty)
cv2.createTrackbar("Val Min", "TrackBars", 153, 255, empty)
cv2.createTrackbar("Hue Max", "TrackBars", 19, 179, empty)
cv2.createTrackbar("Sat Max", "TrackBars", 240, 255, empty)
cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)

res = np.zeros((480, 856,3))
res0 = np.zeros((480, 856,3))
def vision_callback(data):
    '''
    vision callback for getting data from bebop image topic.
    '''
    global res, res0
    # create np array with correct dtype with ros_numpy package.
    image_arr = ros_numpy.numpify(data)
    res = cv2.cvtColor(image_arr, cv2.COLOR_BGR2RGB)
    imgBlur = cv2.GaussianBlur(res, (11, 11), 3)
    imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
    v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
    h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
    v_max = cv2.getTrackbarPos("Val Max", "TrackBars")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask0 = cv2.inRange(imgHSV, lower, upper)

    params = cv2.SimpleBlobDetector_Params()

    params.filterByColor = False
    params.filterByArea = False
    params.filterByInertia = False
    params.filterByConvexity = False
    params.filterByCircularity = False
    det = cv2.SimpleBlobDetector_create(params)  # detector

    #colorMaskRed = cv2.inRange(imgHSV, lowerLimitRed, upperLimitRed)
    
    res0 = cv2.bitwise_and(res, res, mask=mask0)
    keypoints0 = det.detect(mask0)
    if keypoints0:
        w = "R"
        cv2.putText(res, str(w), (20, 250), cv2.FONT_HERSHEY_PLAIN, 4, (0, 255, 0), 3)
        cv2.drawKeypoints(res, keypoints0, res, (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)



if __name__ == '__main__':
    try:
        # queue size 1 and large buffer size for images to avoid latency from ssd
        rospy.Subscriber('/q1/real/image_raw', Image, vision_callback, queue_size=1, buff_size=2**24)
        while(True):
            # Capture frame-by-frame
            cv2.imshow("Result", res0)
            cv2.imshow('Camara 1',res)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        pass

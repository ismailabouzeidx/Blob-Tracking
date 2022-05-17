#! usr/bin/python3.8
from cv_bridge import CvBridge
import cv2
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np



lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
bridge = CvBridge()
twist=Twist()



def callback(msg):
    img=bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    im_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    res = cv2.bitwise_and(img,img,mask = mask)
    imggray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    output = res.copy()
    circles = cv2.HoughCircles(im_gray,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
    # ensure at least some circles were found
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        xc=i[0]
        yc=i[1]
        cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
    cv2.putText(img,"Ball Center: x:{0} and y:{1} + moodz gay".format(xc,yc),(0,yc-200),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),1)

    cv2.putText(img,"ROI Center ({0},{1})".format(320,350),(240,290),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,150),3)
    cv2.rectangle(img,(270,300),(370,400),(255,0,0),3)
    cv2.imshow("output", np.hstack([img, output]))

    
    if (yc<300):
       twist.linear.x=0.1
    elif (yc>300):
        twist.linear.x=0
        movearm()
        print("inside")
    if(xc>380):
        twist.angular.z=-0.1
    elif (xc<260):
        twist.angular.z=0.1
    elif(270<xc<295):
        twist.angular.z=0
    pub.publish(twist) 
    cv2.waitKey(1)

rospy.init_node("blobber")

pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
rospy.Subscriber("/camera_1/color/image_raw",Image, callback) 

rate=rospy.Rate(10)

while not rospy.is_shutdown():
    rate.sleep()

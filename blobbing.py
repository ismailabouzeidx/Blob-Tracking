#! usr/bin/python3.8


#Vision Libraries:
from multiprocessing.connection import wait
from cv_bridge import CvBridge
import cv2
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np


#Manipulation Libraries:
import moveit_commander
import moveit_msgs.msg
import sys

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])



class blob_pusher:
    def __init__(self):
        self.xc = 0
        self.yc = 0
        self.circles = None
        self.camera_topic = "/camera_1/color/image_raw"
        self.vel_topic = "/cmd_vel"
        self.arm_topic = "/arm_state"

        rospy.Subscriber(self.camera_topic,Image,self.callback)
        self.pub = rospy.Publisher(self.vel_topic,Twist,queue_size=10)
        self.pub2 = rospy.Publisher(self.arm_topic,String,queue_size=10,latch=True)
        self.bridge = CvBridge()
        self.mobile_state = "No State"
        self.arm_state="Home"
        self.twist = Twist()
        self.current_img = None


    def callback(self,img_msg):
        self.current_img = img_msg

    def arm_mobile_state_chooser(self):
        self.mobile_state_machine()
        
        self.finding_circles()
        print(self.arm_state)
        


        
    def finding_circles(self):
        if self.current_img is not None:
            img=self.bridge.imgmsg_to_cv2(self.current_img,'bgr8')
            self.h,self.w,self.d=img.shape
            hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            im_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            res = cv2.bitwise_and(img,img,mask = mask)
            imggray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
            output = res.copy()
            self.circles = cv2.HoughCircles(im_gray,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
            if self.circles is not None: 
                self.circles = np.uint16(np.around(self.circles))
                for i in self.circles[0,:]:
                    # draw the outer circle
                    cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
                    # draw the center of the circle
                    self.xc=i[0]
                    self.yc=i[1]
                    cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
                cv2.putText(img,"Ball Center: x:{0} and y:{1}".format(self.xc,self.yc),(0,self.yc-200),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),1)

                cv2.putText(img,"ROI Center ({0},{1})".format(self.w/2 ,self.h/2 ),(240,290),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,150),3)
                cv2.rectangle(img,(350,450),(450,350),(0,255,0),3)
    
            cv2.imshow("output", img)
            cv2.waitKey(1)
            
            self.mobile_state_machine()


#Control Functions
    def locate(self):
        self.twist.angular.z=0.5
    def move_forward(self):
        self.twist.linear.x = 0.3
        self.twist.angular.z = 0
    def stop(self):
        self.twist.linear.x=0
        self.twist.angular.z=0
    def angular_adjustment(self):
        err = self.xc - self.w/2
        self.twist.angular.z = -float(err) / 700
#Controller
    def mobile_state_machine(self):
    
        if self.circles is None:
            self.mobile_state="Locate"
        else:
            if (self.yc)>340:
                self.mobile_state = "Forward"
                self.arm_state = "dont_push"
            elif self.yc<320:
                self.mobile_state = "In-Place"
                self.arm_state = "push"
                self.pub2.publish(self.arm_state)
                self.arm_state = "dont_push"
                self.pub2.publish(self.arm_state)
        self.mobile_change_states(self.mobile_state)




    def mobile_change_states(self,state):
        if state == "Forward":
            self.move_forward()
        elif state == "In-Place":
            self.stop()
        elif state == "Locate":
            self.locate()
        # print(state)
        if state!="Locate":
            self.angular_adjustment()
        self.pub.publish(self.twist)
                

if __name__=="__main__":
    rospy.init_node("blob_pusher")
    blobber = blob_pusher()
    rate = rospy.Rate(10)
    while True:
        try:
            blobber.arm_mobile_state_chooser()
            rate.sleep() #was important to have a smooth view
            #Take-Away: if not functioning in callback(at rate of callback) --> must specify a rate
            # blobber.state_machine()
        except KeyboardInterrupt:
            print("Interrupt")
            break

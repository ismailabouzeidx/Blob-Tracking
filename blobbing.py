#! usr/bin/python3.8


#Vision Libraries:
from multiprocessing.connection import wait
from cv_bridge import CvBridge
import cv2
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
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
        self.camera_topic = "/camera_1/color/image_raw"
        self.vel_topic = "/cmd_vel"

        moveit_commander.roscpp_initialize(sys.argv)
        robot= moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        arm_group = "arm_group"
        self.move_group = moveit_commander.MoveGroupCommander(arm_group)
        self.trajector_topic = "/move_group/display_planned_path"
        display_trajectory_publisher = rospy.Publisher(
            self.trajector_topic,
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
            )
        """" 
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        """

        rospy.Subscriber(self.camera_topic,Image,self.callback)
        self.pub = rospy.Publisher(self.vel_topic,Twist,queue_size=10)
        self.bridge = CvBridge()
        self.state = "No State"
        self.twist = Twist()
        self.current_img = None


    def callback(self,img_msg):
        self.current_img = img_msg
        
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
            else:
                self.locate()
            cv2.imshow("output", img)
            cv2.waitKey(1)
            self.state_machine()


#Control Functions
    def locate(self):
        self.twist.angular.z=0.5
    def move_forward(self):
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0
    def stop(self):
        self.twist.linear.x=0
        self.twist.angular.z=0
        
    def angular_adjustment(self):
        err = self.xc - self.w/2
        self.twist.angular.z = -float(err) / 700
        print("Turning")
    def push(self):
        self.move_group.set_named_target = "push"
        push_plan = self.move_group.go (wait=True)
        self.move_group.stop()
    def retract(self):
        self.move_group.set_named_target = "dont_push"
        retract_plan = self.move_group.go (wait=True)
        self.move_group.stop()

#Controller
    def state_machine(self):
    
        if (self.yc)<395:
            self.state = "Forward"
            self.move_forward()
            print("moving forward")
            # self.retract()
        else:
            self.state = "Stop"
            print("stopping")
            self.stop()

        
        if self.circles is None:
            # rospy.sleep(2)
            self.locate()
        else:
            self.angular_adjustment()
        self.push()
        self.pub.publish(self.twist)
        
            


if __name__=="__main__":
    rospy.init_node("blob_pusher")
    blobber = blob_pusher()
    rate = rospy.Rate(10)
    while True:
        try:
            blobber.finding_circles()
            rate.sleep() #was important to have a smooth view
            #Take-Away: if not functioning in callback(at rate of callback) --> must specify a rate
            # blobber.state_machine()
        except KeyboardInterrupt:
            print("Interrupt")
            break

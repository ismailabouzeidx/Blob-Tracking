#!usr/bin/python3.8


import rospy
from geometry_msgs.msg import Pose
import moveit_commander
import moveit_msgs.msg
import sys
import tf
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from tf2_geometry_msgs import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation

class move_arm:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.Subscriber("/arm_state",String,self.callback)
        #Instantiate a RobotCommander object. 
        # Provides information such as the robot’s kinematic model and the robot’s current joint states
        robot = moveit_commander.RobotCommander()
        #Instantiate a PlanningSceneInterface object. 
        # This provides a remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        #Instantiate a MoveGroupCommander object.
        #  This object is an interface to a planning group (group of joints). 
        group_name = "arm_group"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        #Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )


    def callback(self,state):
        self.state = state.data
        self.move_group.set_named_target(self.state)
        #Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

if __name__=="__main__":
    rospy.init_node("Arm_Node")
    arm = move_arm()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            arm.__init__()
            rate.sleep()
        except KeyboardInterrupt:
            rospy.logfatal("Program Shutoff")




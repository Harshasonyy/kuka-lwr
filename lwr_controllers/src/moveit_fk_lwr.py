#!/usr/bin/env python

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        # Connect to the full_lwr move group
        lwr_kuka4plus = moveit_commander.MoveGroupCommander('lwr_kuka4plus')
        
         # Get the name of the end-effector link
        end_effector_link =  lwr_kuka4plus.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))

        # Set a small tolerance on joint angles
        lwr_kuka4plus.set_goal_joint_tolerance(0.001)
        
 
        # Set target joint values for the arm: joints are in the order they appear in
        # the kinematic tree.
        joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.300266444513, 0.0]
 
        # Set the arm's goal configuration to the be the joint positions
        lwr_kuka4plus.set_joint_value_target(joint_positions)
                 
        # Plan and execute the motion
        lwr_kuka4plus.go()
        rospy.sleep(1)
         
        # Save this configuration for later
        lwr_kuka4plus.remember_joint_values('saved_config', joint_positions)               
       
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass

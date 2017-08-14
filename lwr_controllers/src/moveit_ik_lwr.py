#!/usr/bin/env python

"""   
    Use inverse kinemtatics to move the end effector to a specified pose
     
"""

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
                
        # Initialize the move group for the full_lwr
        lwr_kuka4plus = moveit_commander.MoveGroupCommander('lwr_kuka4plus')
                
        # Get the name of the end-effector link
        end_effector_link = lwr_kuka4plus.get_end_effector_link()

 	    # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))
                        
        # Set the reference frame for pose targets
        reference_frame = 'lwr_base_link'
        
        # Set the full_lwr reference frame accordingly
        lwr_kuka4plus.set_pose_reference_frame(reference_frame)
                
        # Allow replanning to increase the odds of a solution
        lwr_kuka4plus.allow_replanning(True)
        
        # Allow some leeway in position (meters) and orientation (radians)
        lwr_kuka4plus.set_goal_position_tolerance(0.01)
        lwr_kuka4plus.set_goal_orientation_tolerance(0.05)
        
        # Start the arm in the "home" pose stored in the SRDF file
        lwr_kuka4plus.set_named_target('home')
        lwr_kuka4plus.go()
        rospy.sleep(2)
               
        # Set the target pose.  
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.269297313764
        target_pose.pose.position.y = 0.629615625757
        target_pose.pose.position.z = 0.389413832365
        target_pose.pose.orientation.x = -0.53364
        target_pose.pose.orientation.y = -0.7964154
        target_pose.pose.orientation.z = -0.1037
        target_pose.pose.orientation.w = 0.2649042
        
        # Set the start state to the current state
        lwr_kuka4plus.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        lwr_kuka4plus.set_pose_target(target_pose, end_effector_link)
        
        # Plan the trajectory to the goal
        traj = lwr_kuka4plus.plan()
        
        # Execute the planned trajectory
        lwr_kuka4plus.execute(traj)
    
        # Pause for a second
        rospy.sleep(1)
         
        # Shift the end-effector to the right 
        lwr_kuka4plus.shift_pose_target(1, -0.1, end_effector_link)
        lwr_kuka4plus.go()
        rospy.sleep(1)
  
        #Rotate the end-effector some degrees
        lwr_kuka4plus.shift_pose_target(3, -1.57, end_effector_link)
        lwr_kuka4plus.go()
        rospy.sleep(1)
          
        # Store this pose as the new target_pose
        saved_target_pose = lwr_kuka4plus.get_current_pose(end_effector_link)
          
        # Move to the named pose "lookleft"
        lwr_kuka4plus.set_named_target('lwr_kukaforward')
        lwr_kuka4plus.go()
        rospy.sleep(1)
          
        # Go back to the stored target
        lwr_kuka4plus.set_pose_target(saved_target_pose, end_effector_link)
        lwr_kuka4plus.go()
        rospy.sleep(1)
           
        # Finish up in the home position  
        lwr_kuka4plus.set_named_target('home')
        lwr_kuka4plus.go()

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
        MoveItDemo()
   

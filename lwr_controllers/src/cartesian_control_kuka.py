#!/usr/bin/env python

"""
    moveit_cartesian_kuka.py 
    
    Plan and execute a Cartesian path for the end-effector through a number of waypoints
    
"""

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        cartesian = rospy.get_param('~cartesian', True)
                        
        # Connect to the lwr_kuka4plus move group
        lwr_kuka4plus = MoveGroupCommander('lwr_kuka4plus')
        
        # Allow replanning to increase the odds of a solution
        lwr_kuka4plus.allow_replanning(True)
        
        # Set the lwr_kuka4plus reference frame
        lwr_kuka4plus.set_pose_reference_frame('lwr_base_link')
                
        # Allow some leeway in position(meters) and orientation (radians)
        lwr_kuka4plus.set_goal_position_tolerance(0.01)
        lwr_kuka4plus.set_goal_orientation_tolerance(0.1)
        
        # Get the name of the end-effector link
        end_effector_link = lwr_kuka4plus.get_end_effector_link()
                                        
        # Start in the "lwr_kukaforward" configuration stored in the SRDF file
        lwr_kuka4plus.set_named_target('lwr_kukaforward')
        
        # Plan and execute a trajectory to the goal configuration
        lwr_kuka4plus.go()
        
        # Get the current pose so we can add it as a waypoint
        start_pose = lwr_kuka4plus.get_current_pose(end_effector_link).pose
                
        # Initialize the waypoints list
        waypoints = []
                
        # Set the first waypoint to be the starting pose
        if cartesian:
            # Append the pose to the waypoints list
            waypoints.append(start_pose)
            
        wpose = deepcopy(start_pose)
                
        # Set the next waypoint back 0.2 meters and right 0.2 meters
        wpose.position.x -= 0.2
        wpose.position.y -= 0.2

        if cartesian:
            # Append the pose to the waypoints list
            waypoints.append(deepcopy(wpose))
        else:
            lwr_kuka4plus.set_pose_target(wpose)
            lwr_kuka4plus.go()
            rospy.sleep(1)
         
        # Set the next waypoint to the right 0.15 meters
        wpose.position.x += 0.05
        wpose.position.y += 0.15
        wpose.position.z -= 0.15
          
        if cartesian:
            # Append the pose to the waypoints list
            waypoints.append(deepcopy(wpose))
        else:
            lwr_kuka4plus.set_pose_target(wpose)
            lwr_kuka4plus.go()
            rospy.sleep(1)
            
        if cartesian:
            # Append the pose to the waypoints list
            waypoints.append(deepcopy(start_pose))
        else:
            lwr_kuka4plus.set_pose_target(start_pose)
            lwr_kuka4plus.go()
            rospy.sleep(1)
            
        if cartesian:
            fraction = 0.0
            maxtries = 100
            attempts = 0
            
            # Set the internal state to the current state
            lwr_kuka4plus.set_start_state_to_current_state()
     
            # Plan the Cartesian path connecting the waypoints
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = lwr_kuka4plus.compute_cartesian_path (
                                        waypoints,   # waypoint poses
                                        0.01,        # eef_step
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions
                
                # Increment the number of attempts 
                attempts += 1
                
                # Print out a progress message
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
            # If we have a complete plan, execute the trajectory
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
    
                lwr_kuka4plus.execute(plan)
                            
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        # Move normally back to the 'home' position
        lwr_kuka4plus.set_named_target('home')
        lwr_kuka4plus.go()
        rospy.sleep(1)
        
        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass

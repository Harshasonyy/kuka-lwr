#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
        
        # Construct the initial scene object
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
        
        # Pause for the scene to get ready
        rospy.sleep(1)
                        
        # Initialize the move group for the lwr_kuka4plus
        lwr_kuka4plus = MoveGroupCommander('lwr_kuka4plus')
        
        # Get the name of the end-effector link
        end_effector_link = lwr_kuka4plus.get_end_effector_link()
        
        # Allow some leeway in position (meters) and orientation (radians)
        lwr_kuka4plus.set_goal_position_tolerance(0.01)
        lwr_kuka4plus.set_goal_orientation_tolerance(0.05)
       
        # Allow replanning to increase the odds of a solution
        lwr_kuka4plus.allow_replanning(True)
        
        # Set the reference frame for pose targets
        reference_frame = 'lwr_base_link'
        
        # Set the lwr_kuka4plus reference frame accordingly
        lwr_kuka4plus.set_pose_reference_frame(reference_frame)
        
        # Allow 5 seconds per planning attempt
        lwr_kuka4plus.set_planning_time(5)
        
        # Give each of the scene objects a unique name
        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'
        
        # Remove leftover objects from a previous run
        scene.remove_world_object(table_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)
        
        # Give the scene a chance to catch up
        rospy.sleep(1)
        
        # Start the arm in the "home" pose stored in the SRDF file
        lwr_kuka4plus.set_named_target('home')
        lwr_kuka4plus.go()
        
        rospy.sleep(2)
        
        # Set the height of the table off the ground
        table_ground = 0.75
        
        # Set the length, width and height of the table and boxes
        table_size = [0.2, 0.7, 0.01]
        box1_size = [0.1, 0.05, 0.05]
        box2_size = [0.05, 0.05, 0.15]
        
        # Add a table top and two boxes to the scene
        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = 0.46
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = reference_frame
        box1_pose.pose.position.x = 0.41
        box1_pose.pose.position.y = -0.1
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0   
        scene.add_box(box1_id, box1_pose, box1_size)
        
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = reference_frame
        box2_pose.pose.position.x = 0.39
        box2_pose.pose.position.y = 0.15
        box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
        box2_pose.pose.orientation.w = 1.0   
        scene.add_box(box2_id, box2_pose, box2_size)
        
        # Make the table red and the boxes orange
        self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_id, 0.8, 0.4, 0, 1.0)
        
        # Send the colors to the planning scene
        self.sendColors()    
        
        # Set the target pose in between the boxes and above the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x = 0.2
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = table_pose.pose.position.z + table_size[2] + 0.05
        target_pose.pose.orientation.w = 1.0
        
        # Set the target pose for the arm
        lwr_kuka4plus.set_pose_target(target_pose, end_effector_link)
        
        # Move the arm to the target pose (if possible)
        lwr_kuka4plus.go()
        
        # Pause for a moment...
        rospy.sleep(2)
        
        # Return the arm to the "home" pose stored in the SRDF file
        lwr_kuka4plus.set_named_target('home')
        lwr_kuka4plus.go()
        
        # Exit MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script        
        moveit_commander.os._exit(0)
        
    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except KeyboardInterrupt:
        raise

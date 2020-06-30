#!/usr/bin/env python

import rospy
import time
from std_srvs.srv import Empty
from gazebo_msgs.msg import ContactsState, ModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np


# start quadrotor
# os.system("timeout 1s rostopic pub /hummingbird/bridge/arm std_msgs/Bool 'True'")
# os.system("timeout 1s rostopic pub /hummingbird/autopilot/start std_msgs/Empty")


class DataGenerator:
    def __init__(self):
        rospy.init_node("data_generation_node")
        self.goal_pub = rospy.Publisher('goal_topic', Pose, queue_size=1)
        self.current_pose = None  # In world frame
        # In world frame, used to check if the robot has reached its intended position
        self.current_goal = None
        self.get_params()
        self.pause_physics_proxy = rospy.ServiceProxy(
            '/gazebo/pause_physics', Empty)
        self.unpause_physics_proxy = rospy.ServiceProxy(
            '/gazebo/unpause_physics', Empty)
        self.place_robot_proxy = rospy.ServiceProxy(
            '/gazebo/set_model_state', ModelState)
        rospy.Subscriber('contact_topic', ContactsState, self.contact_cb)
        rospy.Subscriber('odom_topic', Odometry, self.odom_cb)
        self.timeout_timer = rospy.Timer(rospy.Duration(
            self.goal_generation_radius * 5), self.timer_cb)

    def get_params(self):
        self.goal_generation_radius = rospy.get_param(
            'goal_generation_radius', 2)
        self.waypoint_radius = rospy.get_param('waypoint_radius', 0.1)
        self.robot_collision_frame = rospy.get_param(
            'robot_collision_frame', 'delta::delta/base_link::delta/base_link_fixed_joint_lump__delta_collision_collision')
        self.ground_collision_frame = rospy.get_param(
            'ground_collision_frame', 'ground_plane::link::collision')

    def generate_new_goal(self):
        # https://stackoverflow.com/questions/5837572/generate-a-random-point-within-a-circle-uniformly/5838055#5838055

        # Generate and return a pose in the sphere centered at the robot frame with radius as the goal_generation_radius
        goal = Pose()

        # Convert this goal into the world frame and set it as the current goal
        self.current_goal = goal

    def reset_sim(self):
        rospy.loginfo('Pausing physics')
        self.pause_physics_proxy(Empty())

        # Fill in the new position of the robot
        new_position = ModelState()
        new_position.model_name = 'delta'
        new_position.reference_frame = 'world'
        new_position.pose.position.x = 0
        new_position.pose.position.y = 0
        new_position.pose.position.z = 0
        new_position.pose.orientation.x = 0
        new_position.pose.orientation.y = 0
        new_position.pose.orientation.z = 0
        new_position.pose.orientation.w = 1
        new_position.twist.linear.x = 0
        new_position.twist.linear.y = 0
        new_position.twist.linear.z = 0
        new_position.twist.angular.x = 0
        new_position.twist.angular.y = 0
        new_position.twist.angular.z = 0
        rospy.loginfo('Placing robot')
        self.place_robot_proxy(new_position)

        self.reset_timer()

        rospy.loginfo('Unpausing physics')
        self.unpause_physics_proxy(Empty())

    def reset_timer(self):
        rospy.loginfo('Resetting the timeout timer')
        self.timeout_timer.shutdown()
        self.timeout_timer = rospy.Timer(rospy.Duration(
            self.goal_generation_radius * 5), self.timer_cb)

    def reset_model(self):
        # To be used later to reset a specific model. This should not be used right now. Huan is checking how to reset a single model.
        pass

    def timer_cb(self, event):
        self.goal_pub.publish(self.generate_new_goal())

    def get_pose_diff(self, p1, p2):
        # At the moment, only return the translation difference. Maybe we should be sending the yaw error also
        position_1 = np.array([p1.x, p1.y, p1.z])
        position_2 = np.array([p2.x, p2.y, p2.z])
        return np.linalg.norm(position_1 - position_2)

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose
        if self.get_pose_diff(self.current_pose, self.current_goal) < self.waypoint_radius:
            # If the robot has reached the given goal pose, send the next waypoint and reset the timer
            self.reset_timer()
            self.goal_pub.publish(self.generate_new_goal())

    def contact_cb(self, msg):
        # Check inside the models states for robot's contact state
        for i in range(len(msg.states)):
            if(msg.states[i].collision1_name == self.robot_collision_frame):
                rospy.loginfo('Contact found!')
                if(msg.states[i].collision2_name == self.ground_collision_frame):
                    rospy.loginfo('Robot colliding with the ground')
                else:
                    rospy.loginfo(
                        'Robot colliding with something else (not ground)')
                    self.reset_sim()
            else:
                rospy.loginfo('Contact not found yet ...')


if __name__ == '__main__':
    dg = DataGenerator()
    rospy.loginfo('Ready')
    rospy.spin()

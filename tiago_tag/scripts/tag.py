#!/usr/bin/env python

# Baran Onalan
# CPSC 5910 - Robotics

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt
import os

class Pose:
    """Data class to keep track of each tiago bot's position and orientation"""
    x = 0      # x-coordinate of the position
    y = 0      # y-coordinate of the position
    theta = 0  # angle of the orientation (in radians)

class Tag:
    """Controller class that controls the state transitions of tiago bots between the states of
    IT, IT-TO-NOT-IT, NOT-IT, NOT-IT-TO-IT"""

    def __init__(self):
        """Creates a controller node with publishers and subscribers to all three tiago bots."""
        rospy.init_node('controller', anonymous=True)

        # Publisher for tiago1
        self.velocity_publisher_1 = rospy.Publisher('/tiago1/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        # Publisher for tiago2
        self.velocity_publisher_2 = rospy.Publisher('/tiago2/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        # Publisher for tiago3
        self.velocity_publisher_3 = rospy.Publisher('/tiago3/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        # Subscriber for tiago1
        self.odom_subscriber_1 = rospy.Subscriber('/tiago1/mobile_base_controller/odom', Odometry, self.update_odom_tiago1)
        # Subscriber for tiago2
        self.odom_subscriber_2 = rospy.Subscriber('/tiago2/mobile_base_controller/odom', Odometry, self.update_odom_tiago2)
        # Subscriber for tiago3
        self.odom_subscriber_3 = rospy.Subscriber('/tiago3/mobile_base_controller/odom', Odometry, self.update_odom_tiago3)

        # These class variables keep track of positions and orientations of the three tiagos
        self.tiago1 = Pose()
        self.tiago2 = Pose()
        self.tiago3 = Pose()

        # At the beginning of the tag game, tiago1 is IT.
        self.it_bot = 1 
        # IT bot will have an unfolded arm. NOT-IT bots will have folded arms
        os.system('rosrun play_motion run_motion_python_node.py unfold_arm /play_motion:=/tiago1/play_motion')
        #client = SimpleActionClient('/tiago1/play_motion', PlayMotionAction)
        #rospy.loginfo("Waiting for Action Server...")
        #client.wait_for_server()
        #goal = PlayMotionGoal()
        #goal.motion_name = 'unfold_arm'
        #goal.skip_planning = True
        #goal.priority = 0  # Optional
        #client.send_goal(goal)
        # The bot currently getting chased by the IT bot is deemed as the target bot.
        self.target_bot = None

        self.rate = rospy.Rate(10)

    def update_odom_tiago1(self, data):
        """This function is called when a new message of type Odometry is received by the tiago1 subscriber."""
        self.tiago1.x = round(data.pose.pose.position.x, 4)
        self.tiago1.y = round(data.pose.pose.position.y, 4)
        rot_q = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.tiago1.theta = round(theta, 4)
        #print("Odom 1 " + str(self.tiago1.x) + " " + str(self.tiago1.y) + " " + str(self.tiago1.theta))

    def update_odom_tiago2(self, data):
        """This function is called when a new message of type Odometry is received by the tiago2 subscriber."""
        self.tiago2.x = round(data.pose.pose.position.x, 4)
        self.tiago2.y = round(data.pose.pose.position.y, 4)
        rot_q = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.tiago2.theta = round(theta, 4)
        #print("Odom 2 " + str(self.tiago2.x) + " " + str(self.tiago2.y) + " " + str(self.tiago2.theta))

    def update_odom_tiago3(self, data):
        """This function is called when a new message of type Odometry is received by the tiago3 subscriber."""
        self.tiago3.x = round(data.pose.pose.position.x, 4)
        self.tiago3.y = round(data.pose.pose.position.y, 4)
        rot_q = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.tiago3.theta = round(theta, 4)
        #print("Odom 3 " + str(self.tiago3.x) + " " + str(self.tiago3.y) + " " + str(self.tiago3.theta))

    def distance(self, current_pose, goal_pose):
        """Calculates and returns the distance between current and goal poses."""
        dist = sqrt(pow((goal_pose.x - current_pose.x), 2) +
                    pow((goal_pose.y - current_pose.y), 2))
        return dist

    def linear_vel(self, current_pose, goal_pose, constant=2.5):
        """Calculates and returns the linear velocity to reach the goal position from the current position.
        The constant multiplier allows the controller to have different bots moving at different speeds."""
        lin_vel = constant * self.distance(current_pose, goal_pose)
        return lin_vel

    def steering_angle(self, current_pose, goal_pose):
        """Calculates and returns the steering angle to reach the goal position from the current position."""
        str_ang = atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)
        return str_ang

    def angular_vel(self, current_pose, goal_pose, constant=0.6):
        """Calculates and returns the angular velocity to reach the goal position from the current position.
        The constant multiplier can be used to allow the controller to have different bots moving at different speeds.
        However increasing it too much has an undesired side effect: When a bot over-steers, it may get further away 
        from the goal before it can get closer."""
        ang_vel = constant * (self.steering_angle(current_pose, goal_pose) - current_pose.theta)
        return ang_vel

    def pose_tiago1(self, normalize = 1):
        """Normalized tiago1 position is calculated using its initial position of (-2, -2).
        Note initial positions of tiago bots on the tiago world coordinate system are set in the
        threetiago_tag.launch launch file.
        normalize: Pass value 1 to normalize, -1 to denormalize, 0 to return the original odom position."""
        norm_tiago1 = Pose()
        norm_tiago1.x = self.tiago1.x - 2 * normalize
        norm_tiago1.y = self.tiago1.y - 2 * normalize
        norm_tiago1.theta = self.tiago1.theta
        return norm_tiago1

    def pose_tiago2(self, normalize = 1):
        """Normalized tiago2 position is calculated using its initial position of (0, 2).
        Note initial positions of tiago bots on the tiago world coordinate system are set in the
        threetiago_tag.launch launch file.
        normalize: Pass value 1 to normalize, -1 to denormalize, 0 to return the original odom position."""
        norm_tiago2 = Pose()
        norm_tiago2.x = self.tiago2.x + 0 * normalize
        norm_tiago2.y = self.tiago2.y + 2 * normalize
        norm_tiago2.theta = self.tiago2.theta
        return norm_tiago2

    def pose_tiago3(self, normalize = 1):
        """Normalized tiago3 position is calculated using its initial position of (2, 0).
        Note initial positions of tiago bots on the tiago world coordinate system are set in the
        threetiago_tag.launch launch file.
        normalize: Pass value 1 to normalize, -1 to denormalize, 0 to return the original odom position."""
        norm_tiago3 = Pose()
        norm_tiago3.x = self.tiago3.x + 2 * normalize
        norm_tiago3.y = self.tiago3.y + 0 * normalize
        norm_tiago3.theta = self.tiago3.theta
        return norm_tiago3

    def it(self, norm_tiago1, norm_tiago2, norm_tiago3):
        """Return the normalized pose for IT tiago."""
        if self.it_bot == 1:
            it = norm_tiago1
        elif self.it_bot == 2:
            it = norm_tiago2
        else:
            it = norm_tiago3
        return it

    def target(self, norm_tiago1, norm_tiago2, norm_tiago3):
        """Return the normalized pose for target NOT-IT tiago."""
        if self.target_bot == 1:
            target = norm_tiago1
        elif self.target_bot == 2:
            target = norm_tiago2
        else:
            target = norm_tiago3
        return target

    def goal_tiago1(self, norm_tiago1, norm_tiago2, norm_tiago3, vel_msg):
        """Calculates and returns the velocity for tiago1 based on its state and goal:
        IT bot should chase the closest NOT-IT bot.
        Target NOT-IT bot that is getting chased by IT should run away from the IT bot.
        NOT-IT bot that is not the target should run away from IT if too close, create distraction by getting closer
        to the other two bots otherwise."""
        it = self.it(norm_tiago1, norm_tiago2, norm_tiago3)
        if self.it_bot == 1:
            # tiago1 is the IT bot.
            if self.distance(it, norm_tiago2) <= self.distance(it, norm_tiago3):
                # tiago1 is closer to tiago2, so it should chase tiago2.
                self.target_bot = 2
                vel_msg.linear.x = self.linear_vel(it, norm_tiago2, constant = 5)
                vel_msg.angular.z = self.angular_vel(it, norm_tiago2)
                print("tiago1 -> tiago2 dist: " + str(self.distance(it, norm_tiago2)) + " vel: " + str(vel_msg.linear.x))
            else:
                # tiago1 is closer to tiago3, so it should chase tiago3.
                self.target_bot = 3
                vel_msg.linear.x = self.linear_vel(it, norm_tiago3, constant = 5)
                vel_msg.angular.z = self.angular_vel(it, norm_tiago3)
                print("tiago1 -> tiago3 dist: " + str(self.distance(it, norm_tiago3)) + " vel: " + str(vel_msg.linear.x))
        elif self.target_bot == 1:
            # tiago1 is the target bot getting chased by IT.
            goal = Pose()
            goal.x = 2*norm_tiago1.x - it.x
            goal.y = 2*norm_tiago1.y - it.y
            vel_msg.linear.x = self.linear_vel(norm_tiago1, goal)
            vel_msg.angular.z = self.angular_vel(norm_tiago1, goal)+0.5
            print("tiago1 vel: " + str(vel_msg.linear.x) + " dist: " + str(self.distance(norm_tiago1, it)))
        else:
            # tiago1 is a NOT-IT bot that is not the target.
            if (self.distance(norm_tiago1, it)) > 3:
                # tiago1 is not close to IT - get closer to create distraction.
                goal = Pose()
                goal.x = (norm_tiago2.x + norm_tiago3.x) / 2
                goal.y = (norm_tiago2.y + norm_tiago3.y) / 2
                vel_msg.linear.x = self.linear_vel(norm_tiago1, goal, 10)
                vel_msg.angular.z = self.angular_vel(norm_tiago1, goal)
            else:
                # tiago1 is too close to IT - run away from IT.
                goal = Pose()
                goal.x = (norm_tiago2.x + norm_tiago3.x) / 2
                goal.y = (norm_tiago2.y + norm_tiago3.y) / 2
                goal.x = 2*norm_tiago1.x - goal.x
                goal.y = 2*norm_tiago1.y - goal.y
                vel_msg.linear.x = self.linear_vel(norm_tiago1, goal, 8)
                vel_msg.angular.z = self.angular_vel(norm_tiago1, goal)
        return vel_msg

    def goal_tiago2(self, norm_tiago1, norm_tiago2, norm_tiago3, vel_msg):
        """Calculates and returns the velocity for tiago2 based on its state and goal:
        IT bot should chase the closest NOT-IT bot.
        Target NOT-IT bot that is getting chased by IT should run away from the IT bot.
        NOT-IT bot that is not the target should run away from IT if too close, create distraction by getting closer
        to the other two bots otherwise."""
        it = self.it(norm_tiago1, norm_tiago2, norm_tiago3)
        if self.it_bot == 2:
            # tiago2 is the IT bot.
            if self.distance(it, norm_tiago1) <= self.distance(it, norm_tiago3):
                # tiago2 is closer to tiago1, so it should chase tiago1.
                self.target_bot = 1
                vel_msg.linear.x = self.linear_vel(it, norm_tiago1, constant = 5)
                vel_msg.angular.z = self.angular_vel(it, norm_tiago1)
                print("tiago2 -> tiago1 dist: " + str(self.distance(it, norm_tiago1)) + " vel: " + str(vel_msg.linear.x))
            else:
                # tiago2 is closer to tiago3, so it should chase tiago3.
                self.target_bot = 3
                vel_msg.linear.x = self.linear_vel(it, norm_tiago3, constant = 5)
                vel_msg.angular.z = self.angular_vel(it, norm_tiago3)+0.5
                print("tiago2 -> tiago3 dist: " + str(self.distance(it, norm_tiago3)) + " vel: " + str(vel_msg.linear.x))
        elif self.target_bot == 2:
            # tiago2 is the target bot getting chased by IT.
            goal = Pose()
            goal.x = 2*norm_tiago2.x - it.x
            goal.y = 2*norm_tiago2.y - it.y
            vel_msg.linear.x = self.linear_vel(norm_tiago2, goal)
            vel_msg.angular.z = self.angular_vel(norm_tiago2, goal)
            print("tiago2 vel: " + str(vel_msg.linear.x) + " dist: " + str(self.distance(norm_tiago2, it)))
        else:
            # tiago2 is a NOT-IT bot that is not the target.
            if (self.distance(norm_tiago2, it)) > 3:
                # tiago2 is not close to IT - get closer to create distraction.
                goal = Pose()
                goal.x = (norm_tiago1.x + norm_tiago3.x) / 2
                goal.y = (norm_tiago1.y + norm_tiago3.y) / 2
                vel_msg.linear.x = self.linear_vel(norm_tiago2, goal, 10)
                vel_msg.angular.z = self.angular_vel(norm_tiago2, goal)
            else:
                # tiago2 is too close to IT - run away from IT.
                goal = Pose()
                goal.x = (norm_tiago1.x + norm_tiago3.x) / 2
                goal.y = (norm_tiago1.y + norm_tiago3.y) / 2
                goal.x = 2*norm_tiago2.x - goal.x
                goal.y = 2*norm_tiago2.y - goal.y
                vel_msg.linear.x = self.linear_vel(norm_tiago2, goal, 8)
                vel_msg.angular.z = self.angular_vel(norm_tiago2, goal)
        return vel_msg

    def goal_tiago3(self, norm_tiago1, norm_tiago2, norm_tiago3, vel_msg):
        """Calculates and returns the velocity for tiago3 based on its state and goal:
        IT bot should chase the closest NOT-IT bot.
        Target NOT-IT bot that is getting chased by IT should run away from the IT bot.
        NOT-IT bot that is not the target should run away from IT if too close, create distraction by getting closer
        to the other two bots otherwise."""
        it = self.it(norm_tiago1, norm_tiago2, norm_tiago3)
        if self.it_bot == 3:
            # tiago3 is the IT bot.
            if self.distance(it, norm_tiago1) <= self.distance(it, norm_tiago2):
                # tiago3 is closer to tiago1, so it should chase tiago1.
                self.target_bot = 1
                vel_msg.linear.x = self.linear_vel(it, norm_tiago1, constant = 5)
                vel_msg.angular.z = self.angular_vel(it, norm_tiago1)
                print("tiago3 -> tiago1 dist: " + str(self.distance(it, norm_tiago1)) + " vel: " + str(vel_msg.linear.x))
            else:
                # tiago3 is closer to tiago2, so it should chase tiago2.
                self.target_bot = 2
                vel_msg.linear.x = self.linear_vel(it, norm_tiago2, constant = 5)
                vel_msg.angular.z = self.angular_vel(it, norm_tiago2)+0.5
                print("tiago3 -> tiago2 dist: " + str(self.distance(it, norm_tiago2)) + " vel: " + str(vel_msg.linear.x))
        elif self.target_bot == 3:
            # tiago3 is the target bot getting chased by IT.
            goal = Pose()
            goal.x = 2*norm_tiago3.x - it.x
            goal.y = 2*norm_tiago3.y - it.y
            vel_msg.linear.x = self.linear_vel(norm_tiago3, goal)
            vel_msg.angular.z = self.angular_vel(norm_tiago3, goal)
            print("tiago3 vel: " + str(vel_msg.linear.x) + " dist: " + str(self.distance(norm_tiago3, it)))
        else:
            # tiago3 is a NOT-IT bot that is not the target.
            if (self.distance(norm_tiago3, it)) > 3:
                # tiago3 is not close to IT - get closer to create distraction.
                goal = Pose()
                goal.x = (norm_tiago1.x + norm_tiago2.x) / 2
                goal.y = (norm_tiago1.y + norm_tiago2.y) / 2
                vel_msg.linear.x = self.linear_vel(norm_tiago3, goal, 10)
                vel_msg.angular.z = self.angular_vel(norm_tiago3, goal)
            else:
                # tiago3 is too close to IT - run away from IT.
                goal = Pose()
                goal.x = (norm_tiago1.x + norm_tiago2.x) / 2
                goal.y = (norm_tiago1.y + norm_tiago2.y) / 2
                goal.x = 2*norm_tiago3.x - goal.x
                goal.y = 2*norm_tiago3.y - goal.y
                vel_msg.linear.x = self.linear_vel(norm_tiago3, goal, 8)
                vel_msg.angular.z = self.angular_vel(norm_tiago3, goal)
        return vel_msg

    def chase(self):
        """This is the main function that controls who is chasing who, who is running away and who is creating distraction.
        To begin with, Tiago1 is the IT bot.
        IT will pick the closest tiago as the target.
        When IT gets close enough to the target NOT-IT, it will tag and a tag ritual will happen:
        - NOT-IT bot will raise arm,
        - IT bot will fold its arm (becoming a NOT-IT bot)
        - The new IT bot that has its arm raised will bring it to unfolded state (becoming IT bot)

        After tagging, both NOT-IT bots run away from the IT bot until they are both distant enough while the IT bot waits.
        
        NOT-IT bot that is the target (currently getting chased by IT) runs away from the IT bot.

        NOT-IT bot that is not the target (currently not getting chased by IT):
        - runs away from IT if it is close to it
        - moves closer to IT and target NOT-IT to create distraction otherwise.""" 

        # This is how close the bots need to get for "tagging"
        tag_dist = 1.7 

        # Controller will control the linear velocity towards the linear.x axis (forward) and 
        # angular velocity towards the angular.z axis (turning angle).
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        chase = True

        # Chase until tag (that is, IT bot is close enough to a NOT-IT bot).
        while chase:

            # Positions need to be normalized to account for initial displacement when we're taking relative positions between tiagos.
            # This is because odometer gives positions for each bot relative to their own starting positions.
            norm_tiago1 = self.pose_tiago1()
            norm_tiago2 = self.pose_tiago2()
            norm_tiago3 = self.pose_tiago3()

            # Identify IT tiago and its position.
            it = self.it(norm_tiago1, norm_tiago2, norm_tiago3)

            # Identiy target NOT-IT tiago and its position.
            target = self.target(norm_tiago1, norm_tiago2, norm_tiago3)

            # Calculate the distances from each tiago to IT.
            d1 = self.distance(norm_tiago1, it)
            d2 = self.distance(norm_tiago2, it)
            d3 = self.distance(norm_tiago3, it)
            # Stop chasing if IT is close enough to one of the NOT-ITs.
            if (d1 != 0 and d1 <= tag_dist) or (d2 != 0 and d2 <= tag_dist) or (d3 != 0 and d3 <= tag_dist):
                chase = False

            # vel_msg sets the linear velocity in the x-axis and the angular velocity in the z-axis.
            # Calculate and publish velocities for all three tiagos.
            vel_msg1 = self.goal_tiago1(norm_tiago1, norm_tiago2, norm_tiago3, vel_msg)
            self.velocity_publisher_1.publish(vel_msg1)

            vel_msg2 = self.goal_tiago2(norm_tiago1, norm_tiago2, norm_tiago3, vel_msg)
            self.velocity_publisher_2.publish(vel_msg2)

            vel_msg3 = self.goal_tiago3(norm_tiago1, norm_tiago2, norm_tiago3, vel_msg)
            self.velocity_publisher_3.publish(vel_msg3)

            print
            
            self.rate.sleep()

        # Stopping our robots for the tag ritual.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher_1.publish(vel_msg)
        self.velocity_publisher_2.publish(vel_msg)
        self.velocity_publisher_3.publish(vel_msg)
        print("Tag!")

        # Tagged NOT-IT bot raises the arm.
        os.system('rosrun play_motion run_motion_python_node.py reach_max /play_motion:=/tiago' + str(self.target_bot) + '/play_motion')
        # IT folds the arm, transitioning to NOT-IT.
        os.system('rosrun play_motion run_motion_python_node.py home /play_motion:=/tiago' + str(self.it_bot) + '/play_motion')
        self.it_bot = self.target_bot
        # Tagged tiago that just raised its arm brings the arm to unfolded position, transitioning to IT.
        os.system('rosrun play_motion run_motion_python_node.py unfold_arm /play_motion:=/tiago' + str(self.it_bot) + '/play_motion')

        # Prepare for the next round of chase with the new IT bot.
        # Before the chasing can start, both NOT-ITs should move sufficientlt away from UT.
        prep = True
        while prep:
            norm_tiago1 = self.pose_tiago1()
            norm_tiago2 = self.pose_tiago2()
            norm_tiago3 = self.pose_tiago3()

            # Identify IT tiago and its position.
            it = self.it(norm_tiago1, norm_tiago2, norm_tiago3)
            
            # Calculate the distances from each tiago to IT.
            d1 = self.distance(norm_tiago1, it)
            d2 = self.distance(norm_tiago2, it)
            d3 = self.distance(norm_tiago3, it)
            # Stop running away if both NOT-ITs are sufficiently far away from IT.
            if (d1 == 0 or d1 >= 3) and (d2 == 0 or d2 >= 3) and (d3 == 0 or d3 >= 3):
                prep = False

            if d1 > 0 and d2 < 4:
                self.target_bot = 1
                vel_msg1 = self.goal_tiago1(norm_tiago1, norm_tiago2, norm_tiago3, vel_msg)
                self.velocity_publisher_1.publish(vel_msg1)
            if d2 > 0 and d2 < 4:
                self.target_bot = 2
                vel_msg2 = self.goal_tiago2(norm_tiago1, norm_tiago2, norm_tiago3, vel_msg)
                self.velocity_publisher_2.publish(vel_msg2)
            if d3 > 0 and d3 < 4:
                self.target_bot = 3
                vel_msg3 = self.goal_tiago3(norm_tiago1, norm_tiago2, norm_tiago3, vel_msg)
                self.velocity_publisher_3.publish(vel_msg3)

            self.rate.sleep()

        self.target_bot = None

if __name__ == '__main__':
    try:
        x = Tag()
        # The cycle of chasing and prepping for the next round will continue until we press control+C.
        while True:
            x.chase()
    except rospy.ROSInterruptException:
        pass

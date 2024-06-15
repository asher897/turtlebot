#!/usr/bin/python

import rospy
from std_msgs.msg import String, Empty
from geometry_msg.msg import Twist, Vector3
from gazebo_msgs.srv import GetModelState
import math
import sympy as sym
from sympy import cos, sin, pi
from prm import Coordinate

class PIDController:
    def __init__(self):
        self.model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.move_forward_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rotation_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.kp = 1.15
        self.ki = 0.3
        self.kd = 0.3
        self.rate = rospy.Rate(10)
        self.T0_1 = sym.Matrix()

    def get_to_coordinate(self, coordinate_in_world_rf):
        acc_errors = []
        coordinate = self.get_coordinate_in_bot_reference_frame(coordinate_in_world_rf)
        current_pos = self.get_current_position()
        error = self.get_error(coordinate, current_pos)
        distance = self.distance_to_coordinate(coordinate, current_pos)

        while distance > 0.05:
            acc_errors.append(error)
            pid = self.calculate_pid(acc_errors)
            self.rotate_bot(current_pos, coordinate)
            self.movement(pid)
            self.rate.sleep()

            current_pos = self.get_current_position()
            error = self.get_error(coordinate, current_pos)
            distance = self.distance_to_coordinate(coordinate, current_pos)
        self.movement(Vector3(0, 0, 0))

    # Rotation
    def rotate_bot(self, current_pos, goal_pos):
        rotate_by = self.get_rotation(current_pos, goal_pos)
        if rotate_by == 0:
            return
        else:
            self.rotation(1)

        while abs(rotate_by) > 0:
            rotate_by = self.get_rotation(current_pos, goal_pos)

        self.rotation(0)

    def get_current_orientation(self):
        bot_state = self.model_state("turtlebot", "")
        orientation = bot_state.pose.orientation
        return orientation.yaw

    def get_rotation(self, current_pos, goal_pos):
        current_theta = self.get_current_orientation()
        desired_theta = math.atan2(goal_pos.y-current_pos.y, goal_pos.x-current_pos.x)

        rotate_by_radians = desired_theta-current_theta
        rotate_by = math.degrees(rotate_by_radians)
        return rotate_by

    def calculate_transformation_matrix(self, rotate_by):
        theta = sym.symbols("theta")
        current_pos = self.get_current_position()

        T0_1 = sym.Matrix(
            [
                [cos(theta), -sin(theta), 0, current_pos[0]],
                [sin(theta), cos(theta), 0, current_pos[1]],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
        )

        self.T0_1 = T0_1.replace(theta, rotate_by)

    def get_coordinate_in_bot_reference_frame(self, coordinate):
        p1 = sym.Matrix(
            [
                [coordinate.x, coordinate.y, 0, 1]
            ]
        ).transpose()

        p0 = self.T0_1*p1
        coordinate = Coordinate(p0[0], p0[1])
        return coordinate

    # PID calculation
    def get_current_position(self):
        bot_state = self.model_state("turtlebot", "")
        coordinates = bot_state.pose.position
        return coordinates

    def calculate_pid(self, acc_errors):
        previous_error = acc_errors[-2] if len(acc_errors) > 2 else [0,0]
        vx = self.kp*acc_errors[-1][0] + self.ki*sum([i][0] for i in acc_errors) + self.kd*(acc_errors[-1][0] - previous_error[0])
        vy = self.kp * acc_errors[-1][1] + self.ki * sum([i][1] for i in acc_errors) + self.kd * (acc_errors[-1][1] - previous_error[1])

        return Vector3(vx, vy, 0)

    # Publishers
    def movement(self, linear_movement):
        rospy.loginfo("Moving forward")
        linear = linear_movement
        angular = Vector3(0, 0, 0)
        self.move_forward_pub.publish(linear, angular)

    def rotation(self, z):
        rospy.loginfo("Rotating")
        linear = Vector3(0, 0, 0)
        angular = Vector3(0, 0, z)
        self.rotation_pub.publish(linear, angular)

    # Confirm subscribers
    def start_up(self):
        while self.move_forward_pub.get_num_connections() < 1 or self.rotation_pub.get_num_connections() < 1:
            pass

    @staticmethod
    def get_error(coordinate, current_pos):
        error = [coordinate.x - current_pos.x, coordinate.y - current_pos.y]
        return error

    @staticmethod
    def distance_to_coordinate(coordinate, current_pos):
        distance_to_goal = math.sqrt((coordinate.x - current_pos.x) ** 2 + (coordinate.y - current_pos.y) ** 2)
        return distance_to_goal

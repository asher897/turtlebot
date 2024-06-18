#!/usr/bin/python

import rospy
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.srv import GetModelState
import math
import sympy as sym
from sympy import cos, sin, pi
from prm import Coordinate
import tf.transformations

class PIDController:
    def __init__(self):
        self.model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.move_forward_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rotation_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.kp = 0.01
        self.ki = 0
        self.kd = 0
        self.rotate_kp = 0.08
        self.rotate_ki = 0
        self.rotate_kd = 0
        self.rate = rospy.Rate(1)
        self.movement_rate = rospy.Rate(10)
        self.T0_1 = sym.Matrix()

    def get_to_coordinate(self, coordinate_in_world_rf):
        acc_errors = []
        bot_state = self.model_state("mobile_base", "")
        orientation = bot_state.pose.orientation
        self.calculate_transformation_matrix(orientation.w)
        # coordinate = self.get_coordinate_in_bot_reference_frame(coordinate_in_world_rf)
        current_pos = self.get_current_position()
        error = self.get_error(coordinate_in_world_rf, current_pos)
        distance = self.distance_to_coordinate(coordinate_in_world_rf, current_pos)

        while distance > 0.05:
            print( "distance = ", distance)
            acc_errors.append(error)
            pid = self.calculate_pid(acc_errors)
            self.movement(pid)
            self.movement_rate.sleep()

            current_pos = self.get_current_position()
            error = self.get_error(coordinate_in_world_rf, current_pos)
            distance = self.distance_to_coordinate(coordinate_in_world_rf, current_pos)
        self.movement(Vector3(0, 0, 0))

    def rotate_bot(self, goal_pos):
        current_pos = self.get_current_position()
        acc_errors = []
        error = self.get_rotation(current_pos, goal_pos)

        while abs(error) > 0.05:
            print("error = ", error)
            acc_errors.append(error)
            pid = self.calculate_rotation_pid(acc_errors)
            print(pid)
            self.rotation(pid)
            self.rate.sleep()
            current_pos = self.get_current_position()
            error = self.get_rotation(current_pos, goal_pos)
        print("ending rotation...")
        self.rotation(Vector3(0, 0, 0))

    def get_current_orientation(self):
        bot_state = self.model_state("mobile_base", "")
        orientation = bot_state.pose.orientation
        return self.get_yaw_from_quaternion(orientation)

    @staticmethod
    def get_yaw_from_quaternion(orientation):
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]  # Yaw (theta)

    def get_rotation(self, current_pos, goal_pos):
        current_theta = self.get_current_orientation()
        print("Current theta = ", current_theta)
        desired_theta = math.atan2(goal_pos.y - current_pos.y, goal_pos.x - current_pos.x)
        print("desired Theta = ", desired_theta)
        rotate_by_radians = desired_theta - current_theta
        return rotate_by_radians

    def calculate_transformation_matrix(self, rotate_by):
        theta = sym.symbols("theta")
        current_pos = self.get_current_position()

        T0_1 = sym.Matrix(
            [
                [cos(theta), -sin(theta), 0, current_pos.x],
                [sin(theta), cos(theta), 0, current_pos.y],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
        )

        self.T0_1 = T0_1.subs(theta, rotate_by)

    def get_coordinate_in_bot_reference_frame(self, coordinate):
        p1 = sym.Matrix(
            [
                [coordinate.x, coordinate.y, 0, 1]
            ]
        ).transpose()

        p0 = self.T0_1 * p1
        coordinate = Coordinate(p0[0], p0[1])
        return coordinate

    def get_current_position(self):
        bot_state = self.model_state("mobile_base", "")
        coordinates = bot_state.pose.position
        return coordinates

    def calculate_pid(self, acc_errors):
        previous_error = acc_errors[-2] if len(acc_errors) > 2 else [0, 0]
        vx = self.kp * acc_errors[-1][0] + self.ki * sum(i[0] for i in acc_errors) + self.kd * (acc_errors[-1][0] - previous_error[0])
        return Vector3(vx, 0, 0)

    def calculate_rotation_pid(self, acc_errors):
        previous_error = acc_errors[-2] if len(acc_errors) > 2 else 0
        vz = self.rotate_kp * acc_errors[-1] + self.rotate_ki * sum(i for i in acc_errors) + self.rotate_kd * (acc_errors[-1] - previous_error)
        return Vector3(0, 0, vz)

    def movement(self, linear_movement):
        rospy.loginfo("Moving forward")
        linear = linear_movement
        angular = Vector3(0, 0, 0)
        self.move_forward_pub.publish(linear, angular)

    def rotation(self, angular_velocity):
        linear = Vector3(0, 0, 0)
        angular = angular_velocity
        self.rotation_pub.publish(linear, angular)

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

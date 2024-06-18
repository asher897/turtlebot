#!/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist, Vector3

from prm import PRM, Coordinate, init_from_center
from pid_control import PIDController


global OBSTACLE_COORDS


def map_callback(data):
    global OBSTACLE_COORDS
    OBSTACLE_COORDS = []
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution
    origin_x = data.info.origin.position.x
    origin_y = data.info.origin.position.y

    for y in range(height):
        for x in range(width):
            index = x + y * width
            if data.data[index] == 100:
                world_x = origin_x + (x + 0.5) * resolution
                world_y = origin_y + (y + 0.5) * resolution
                OBSTACLE_COORDS.append(Coordinate(world_x, world_y))
    print("OBSTACLES DONE")


def init_listener():
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    # rospy.spin()


def init_pid():
    # rospy.init_node('pid_controller', anonymous=True)
    pid_controller = PIDController()
    pid_controller.start_up()
    return pid_controller


def main():
    global OBSTACLE_COORDS

    init_listener()
    pid_controller = init_pid()
    goal = input()
    # coords = goal.split(",")
    goal_point = Coordinate(int(goal[0]), int(goal[1]))
    start_point = pid_controller.get_current_position()

    obs_coords = []
    for obs in OBSTACLE_COORDS:
        obs_coords.append(init_from_center(obs, resolution=0.05, turtlebot_radius=0.2))

    print("OBSTACLES DONE")

    # Establish coordinates
    x_range = (-13, 13)
    y_range = (-13, 13)

    # Create prm instance
    prm_instance = PRM(x_range, y_range, obs_coords)

    # Run algorithm

    print("Starting PRM algorithm...")

    path = prm_instance.generate_map(Coordinate(start_point.x, start_point.y), goal_point)
    
    print("Starting PID...")

    for coord in path[1:]:
        print("Coords: ",coord )
        pid_controller.rotate_bot(coord)
        pid_controller.get_to_coordinate(coord)



if __name__ == "__main__":
    main()

#!/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist, Vector3
import numpy as np

from prm import PRM, Coordinate, init_from_center
from pid_control import PIDController


global OBSTACLE_COORDS


def map_callback(data):
    global OBSTACLE_COORDS
    global map_sub
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
    # print("OBSTACLES DONE")
    map_sub.unregister()


def init_listener():
    global map_sub
    rospy.init_node('map_listener', anonymous=True)
    map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)
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

    obs_coords = []

    print(len(obs_coords))

    for obs in OBSTACLE_COORDS:
        obs_coords.append(init_from_center(obs, resolution=0.05, turtlebot_radius=0.2, padding=1.3))

    print("OBSTACLES DONE")

    # Establish coordinates
    x_range = (-10, 6)
    y_range = (-5, 12)

    # Create prm instance
    prm_instance = PRM(x_range, y_range, obs_coords)

    # Run algorithm

    print("Starting PRM algorithm...")

    done = 0

    while not done:

        start_point = pid_controller.get_current_position()

        path = np.array([])
        
        while not path.size:

            coords = raw_input("Where do you want me to go Master?\n")
            goal = coords.split(",")
            goal_point = Coordinate(float(goal[0]), float(goal[1]))

            if goal_point.x < x_range[0] or goal_point.x > x_range[1] or goal_point.y < y_range[0] or goal_point.y > y_range[1]:
                print("invalid coordinate. Please pick an x value between {x1} to {x2} and a y value between {y1} to {y2}".format(x1=x_range[0],x2=x_range[1],y1=y_range[0],y2=y_range[1]))
                continue

            print("Generating a path. Please be patient, I'm trying my best")

            path = prm_instance.generate_map(Coordinate(start_point.x, start_point.y), goal_point)

        # path = np.array([
        #     Coordinate(0.000789549996006,0.000484783623094),
        #     Coordinate(-4.0658,1.241),
        #     Coordinate(-4.1123,2.4114),
        #     Coordinate(0.148,3.7736),
        #     Coordinate(-0.5424,7.1672),
        #     Coordinate(-2.0,8.5),
        # ]
        # )

        # path = np.array([
        #     Coordinate(0.000741787065231,0.000499240364303),
        #     Coordinate(-0.1178,-2.017),
        #     Coordinate(4.249,-0.9126),
        #     Coordinate(4.193,5.1757),
        #     Coordinate(3.9249,6.0433),
        #     Coordinate(-2.0,8.5),

        # ])
        
        print("Starting PID...")

        for coord in path[1:]:
            print("Coords: ",str(coord) )
            pid_controller.rotate_bot(coord)
            pid_controller.get_to_coordinate(coord)

        is_done = raw_input("Are you finished Master? [y/n]")

        if is_done == 'y':
            done = True
            break



if __name__ == "__main__":
    main()

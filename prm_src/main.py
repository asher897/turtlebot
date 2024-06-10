#!/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist, Vector3

from prm import PRM, Coordinate, init_from_center


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


def init_listener():
    rospy.init_node('map_listener')
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()


def main():
    global OBSTACLE_COORDS

    init_listener()
    goal = input()
    coords = goal.split(";")
    goal_point = Coordinate(int(coords[1].split(",")[0]), int(coords[1].split(",")[1]))

    obs_coords = []
    for obs in OBSTACLE_COORDS:
        obs_coords.append(init_from_center(obs, resolution=0.05))
    # Establish coordinates
    x_range = (0, 100)
    y_range = (0, 100)

    # Create prm instance
    prm_instnace = PRM(x_range, y_range, obs_coords)

    # Run algorithm
    prm_instnace.generate_map(Coordinate(0,0), goal_point)

if __name__ == "__main__":
    main()

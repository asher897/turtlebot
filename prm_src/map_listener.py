#!/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist, Vector3


def map_callback(data):
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution
    origin_x = data.info.origin.position.x
    origin_y = data.info.origin.position.y

    print(data.data[0])

    for y in range(height):
        for x in range(width):
            index = x + y * width
            if data.data[index] == 100:
                world_x = origin_x + (x + 0.5) * resolution
                world_y = origin_y + (y + 0.5) * resolution
                print("Obstacle at ({}, {})".format(world_x, world_y))



def init_listner():
    rospy.init_node('map_listener')
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()

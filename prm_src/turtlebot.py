#!/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist, Vector3

# rospy.init_node('turtle', anonymous=True)

# linear = Vector3(1, 0, 0)
# angular = Vector3(0, 0, 0)


# turtle_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)

# rospy.loginfo("Making sure there are subscribers")
# while turtle_pub.get_num_connections() < 1:
#     pass

# rospy.loginfo("There are subscribers")


# for i in range(10000):

#     turtle_pub.publish(linear, angular)
def map_callback(data):

    obs_count = 0

    # Assuming each cell represents a square in the map
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution
    origin_x = data.info.origin.position.x
    origin_y = data.info.origin.position.y

    print(data.data[0])

    for y in range(height):
        for x in range(width):
            index = x + y * width
            if data.data[index] == 100:  # Assuming 100 represents an obstacle
                # Convert grid coordinates to world coordinates
                world_x = origin_x + (x + 0.5) * resolution
                world_y = origin_y + (y + 0.5) * resolution
                print("Obstacle at ({}, {})".format(world_x, world_y))
                obs_count += 1



def main():
    rospy.init_node('map_listener')
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()



if __name__ == '__main__':
    main()

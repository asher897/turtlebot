#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import random
import tf

class Coordinate(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return f"{self.x},{self.y}"


class Obstacle(object):
    def __init__(self, top_left: Coordinate, bottom_right: Coordinate):
        self.top_left = top_left
        self.bottom_right = bottom_right
        self.bottom_left = Coordinate(top_left.x, bottom_right.y)
        self.top_right = Coordinate(bottom_right.x, top_left.y)

        self.plot_points = [
            (self.bottom_left.x, self.bottom_left.y), (self.top_left.x, self.top_left.y), (self.top_right.x, self.top_right.y), (self.bottom_right.x, self.bottom_right.y)
        ]


class Edge(object):
    def __init__(self, current_node, neighbour):
        self.current_node: Node = current_node
        self.neighbour: Node = neighbour
        self.weight = np.linalg.norm(
            np.array([current_node.x, current_node.y]) - np.array([neighbour.x, neighbour.y])
        )


class Node(object):
    def __init__(self, coords: Coordinate):
        self.coords: Coordinate = coords
        self.x: int = coords.x
        self.y: int = coords.y
        self.edges = np.array([])
        self.g: float = 999999
        self.h: float = 0
        self.parent: Node = None
        self.searched = False

    def set_parent(self, parent):
        self.parent = parent

    def get_possible_g(self, possible_parent):
        weight = next(filter(lambda x: x if (x.neighbour.coords == possible_parent.coords) else 0, self.edges)).weight
        return possible_parent.g + weight

    def set_h(self, goal: Coordinate):
        if not self.h:
            self.h = np.linalg.norm(
                np.array([self.x, self.y]) - np.array([goal.x, goal.y])
            )

    def add_edge(self, edge):
        self.edges = np.append(self.edges, np.array([edge]))

    def reset(self):
        self.searched = False
        self.g = 99999
        self.h = 0
        self.parent = None

    @property
    def f(self):
        return self.g + self.h

    def __eq__(self, other):
        return self.coords == other.coords


class PRM(object):
    def __init__(self, x_range, y_range):
        self.x_range = x_range
        self.y_range = y_range
        self.nodes = np.array([])
        self.BATCH_SIZE = 10
        self.obstacles = []

    def laser_callback(self, msg):
        # Process laser scan data to update obstacles
        self.update_obstacles_from_laser(msg)

    def update_obstacles_from_laser(self, laser_scan):
        # Implement logic to convert laser scan data into obstacles
        pass

    def generate_map(self, start_point, goal):
        start_node = Node(start_point)
        goal_node = Node(goal)
        start_node.g = 0
        self.nodes = np.append(self.nodes, np.array([start_node, goal_node]))

        nodes_visited = np.array([])
        goal_threshold = 0.001
        last_g = goal_node.g
        current_g = 0
        while not nodes_visited.size or abs(current_g - last_g) > goal_threshold:
            last_g = current_g
            self.generate_batch(self.BATCH_SIZE)
            nodes_visited = self.a_star(start_node, goal_node)
            current_g = goal_node.g

        self.plot_path(nodes_visited)
        print("Path taken")
        for node in nodes_visited:
            print(node.coords)

    def a_star(self, start, goal):
        nodes_visited = np.array([])
        current_node = start
        while current_node and current_node != goal:
            for edge in current_node.edges:
                node_to_search = edge.neighbour
                if node_to_search not in nodes_visited:
                    possible_g = node_to_search.get_possible_g(current_node)
                    node_to_search.set_h(goal.coords)
                    node_to_search.searched = True
                    possible_f = possible_g + node_to_search.h
                    if possible_f < node_to_search.f:
                        node_to_search.set_parent(current_node)
                        node_to_search.g = possible_g
            nodes_visited = np.append(nodes_visited, np.array([current_node]))
            try:
                current_node = min(filter(lambda x: x.searched and x not in nodes_visited, self.nodes), key=lambda x: x.f)
            except ValueError as e:
                return np.array([])

        return self.get_path(start, goal)

    def get_path(self, start, goal):
        optimal_path = [goal]
        current_node = goal
        while current_node != start:
            optimal_path.insert(0, current_node.parent)
            current_node = current_node.parent

        return np.array(optimal_path)

    def reset(self):
        self.nodes = np.array(list(map(lambda x: x.reset(), self.nodes)))

    def generate_batch(self, batch_size):
        i = 0
        while i < batch_size:
            x = random.randint(self.x_range[0], self.x_range[1])
            y = random.randint(self.y_range[0], self.y_range[1])
            node = Node(Coordinate(x, y))

            collision = False
            for obstacle in self.obstacles:
                if self.is_in_obstacle(x, y, obstacle):
                    collision = True
                    break

            if not collision and not np.any(self.nodes == node):
                self.find_node_neighbours(node)
                self.nodes = np.append(self.nodes, np.array([node]))
                i += 1

    def find_node_neighbours(self, new_node):
        for node in self.nodes:
            if self.collision_free(node.coords, new_node.coords):
                node.add_edge(Edge(node, new_node))
                new_node.add_edge(Edge(new_node, node))

    def is_collision(self, node, new_node, m, c, obstacle):
        y = m * obstacle.top_left.x + c if m != float("inf") else float("inf")
        if self.is_in_collision(node, new_node, obstacle.top_left.x, y, obstacle):
            return True

        y = m * obstacle.bottom_right.x + c if m != float("inf") else float("inf")
        if self.is_in_collision(node, new_node, obstacle.bottom_right.x, y, obstacle):
            return True

        x = (obstacle.top_left.y - c) / m if m != 0 else float("inf")
        if self.is_in_collision(node, new_node, x, obstacle.top_left.y, obstacle):
            return True

        x = (obstacle.bottom_right.y - c) / m if m != 0 else float("inf")
        if self.is_in_collision(node, new_node, x, obstacle.bottom_right.y, obstacle):
            return True

        return False

    @staticmethod
    def is_in_collision(node, new_node, x, y, obstacle):
        if x == float("inf"):
            if obstacle.top_left.y >= y >= obstacle.bottom_right.y:
                nodes_sorted = sorted([node, new_node], key=lambda n: n.x)
                return nodes_sorted[0].x < obstacle.top_left.x and nodes_sorted[1].x > obstacle.bottom_right.x
        elif y == float("inf"):
            if obstacle.top_left.x <= x <= obstacle.bottom_right.x:
                nodes_sorted = sorted([node, new_node], key=lambda n: n.y)
                return nodes_sorted[1].y > obstacle.top_left.y and nodes_sorted[0].y < obstacle.bottom_right.y
        elif (obstacle.top_left.x <= x <= obstacle.bottom_right.x) and (obstacle.top_left.y >= y >= obstacle.bottom_right.y):
            x_nodes_sorted = sorted([node, new_node], key=lambda n: n.x)
            x_bounds = x_nodes_sorted[0].x <= x <= x_nodes_sorted[1].x
            y_nodes_sorted = sorted([node, new_node], key=lambda n: n.y)
            y_bounds = y_nodes_sorted[1].y >= y >= y_nodes_sorted[0].y
            return x_bounds and y_bounds

    def collision_free(self, node, new_node):
        for obstacle in self.obstacles:
            if self.intersects(node, new_node, obstacle):
                return False
        return True

    def intersects(self, node, new_node, obstacle):
        def ccw(A, B, C):
            return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)

        def intersect(A, B, C, D):
            return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

        corners = [
            Coordinate(obstacle.top_left.x, obstacle.top_left.y),
            Coordinate(obstacle.top_left.x, obstacle.bottom_right.y),
            Coordinate(obstacle.bottom_right.x, obstacle.top_left.y),
            Coordinate(obstacle.bottom_right.x, obstacle.bottom_right.y)
        ]

        for i in range(4):
            if intersect(node, new_node, corners[i], corners[(i + 1) % 4]):
                return True
        return False

    def set_batch_size(self, batch_size):
        self.BATCH_SIZE = batch_size

    @staticmethod
    def is_in_obstacle(x, y, obstacle):
        return (obstacle.top_left.x <= x <= obstacle.bottom_right.x) and (
                    obstacle.top_left.y >= y >= obstacle.bottom_right.y)

    def plot_path(self, nodes_visited):
        plt.clf()
        for obstacle in self.obstacles:
            polygon = plt.Polygon(obstacle.plot_points, closed=True, color='red', alpha=1)
            plt.gca().add_patch(polygon)

        x_points = list(map(lambda x: x.x, nodes_visited))
        all_x_points = list(map(lambda x: x.x, self.nodes))
        all_y_points = list(map(lambda x: x.y, self.nodes))
        y_points = list(map(lambda x: x.y, nodes_visited))

        plt.plot(x_points, y_points, '-o')
        plt.scatter(all_x_points, all_y_points, color='g')

        plt.xlim(0, 100)
        plt.ylim(0, 100)
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('2D Domain with Obstacles')
        plt.savefig("./path_taken.png")

def main():
    rospy.init_node('prm_planner')
    prm = PRM((0, 100), (0, 100))
    start_point = Coordinate(10, 10)
    goal_point = Coordinate(90, 90)
    
    rospy.Subscriber('/scan', LaserScan, prm.laser_callback)
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    path_pub = rospy.Publisher('/path', PoseStamped, queue_size=10)
    
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        prm.generate_map(start_point, goal_point)
        # Publish map and path here
        rate.sleep()

if __name__ == '__main__':
    main()
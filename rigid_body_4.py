import argparse
from math import pi, sqrt
from rigid_body import CarController, check_car
from create_scene import load_polygons
from rigid_body_1 import make_rigid_body
from rigid_body_3 import interpolate, reposition_car
from planar_arm import change_angles
import numpy as np
import matplotlib.pyplot as plt


class tree:
    def __init__(self, x, y, theta):
        self.x = x  # x coord
        self.y = y  # ycoord
        self.theta = theta  # angle of rigid body
        self.children = []  # all neighboring nodes
        self.parent = None


class rotate:
    def __init__(self, start, goal, iterations, stepSize):
        self.randomTree = tree(*start)
        self.goal = tree(*goal)
        self.nearestNode = None
        self.currentNode = None
        self.iterations = min(iterations, 1000)
        self.rho = stepSize
        self.path_distance = 0
        self.nearestDist = float('inf')
        self.numWaypoints = 0
        self.Waypoints = []

    def child_node(self, x, y, theta):
        newNode = self.goal if (x, y, theta) == (self.goal.x, self.goal.y, self.goal.theta) else tree(x, y, theta)
        self.nearestNode.children.append(newNode)
        newNode.parent = self.nearestNode
        if newNode is not self.goal:
            self.currentNode = newNode

    def point_sample(self, goal):
        if np.random.rand() < 0.05:
            return np.array(goal)
        else:
            return np.random.uniform([0.1, 0.1, -pi], [1.9, 1.9, pi])

    def search_point(self, start, end):
        offset = self.rho * (np.array(end) - np.array([start.x, start.y, start.theta])) / np.linalg.norm(
            np.array(end) - np.array([start.x, start.y, start.theta]))
        return np.array([start.x, start.y, start.theta]) + offset

    def obstacle(self, start, end, obstacles):
        points = interpolate((start.x, start.y, start.theta), tuple(end))
        return any(not check_car(make_rigid_body((point[0], point[1]), point[2]), obstacles) for point in points)

    def nearest(self, root, point):
        if root:
            dist = self.space(root, point)
            if dist < self.nearestDist:
                self.nearestNode, self.nearestDist = root, dist
            for child in root.children:
                self.nearest(child, point)

    def space(self, node1, point):
        linear_distance = sqrt((node1.x - point[0]) ** 2 + (node1.y - point[1]) ** 2)
        angular_distance = abs(change_angles(node1.theta) - change_angles(point[2]))
        return 0.7 * linear_distance + 0.3 * angular_distance

    def goal(self, point, obstacles):
        return self.space(self.goal, point) <= self.rho and not self.obstacle(self.currentNode,
                                                                              [self.goal.x, self.goal.y,
                                                                                      self.goal.theta], obstacles)

    def reset(self):
        self.nearestNode = None
        self.nearestDist = float('inf')

    def retrace(self, goal):
        if (goal.x, goal.y, goal.theta) != (self.randomTree.x, self.randomTree.y, self.randomTree.theta):
            self.numWaypoints += 1
            self.Waypoints.insert(0, [goal.x, goal.y, goal.theta])
            self.path_distance += self.rho
            self.retrace(goal.parent)


def calculate_tree(start, goal, obstacles):
    plt.close('all')
    ax = plt.figure("RTT Algorithm").gca()
    ax.set_xlim(0, 2)
    ax.set_ylim(0, 2)
    plt.plot(start[0], start[1], 'ro', alpha=0.1)

    rrt = rotate(start, goal, 1000, .3)
    for _ in range(1000):  # using for loop instead of while with manual increment
        rrt.reset()
        point = rrt.point_sample(goal)
        rrt.nearest(rrt.randomTree, point)
        new = rrt.search_point(rrt.nearestNode, point)
        if not rrt.obstacle(rrt.nearestNode, new, obstacles):
            rrt.child_node(*new)
            if rrt.goal(new, obstacles):
                rrt.child_node(*goal)
                print("Goal Found")
                break

    rrt.retrace(rrt.goal)
    rrt.Waypoints.insert(0, start)
    for waypoint_start, waypoint_end in zip(rrt.Waypoints, rrt.Waypoints[1:]):
        plt.plot([waypoint_start[0], waypoint_end[0]], [waypoint_start[1], waypoint_end[1]], 'ro', linestyle="--")

    plt.pause(2)
    plt.close()
    fig, ax = plt.subplots(dpi=100)
    rig_body = CarController(ax, car=make_rigid_body(start), obstacles=[])
    graph(start, rig_body, rrt.Waypoints, obstacles)


def graph(start, rig_body, waypoints, obstacles):
    for waypoint in waypoints[1:]:
        move_graph(rig_body, start, waypoint, obstacles)
        start = waypoint


def move_graph(rig_body, start, goal, obstacles):
    for pt in interpolate(start, goal, 0.01):
        print(pt)
        reposition_car(pt, rig_body)
        if not check_car(rig_body.car, obstacles):
            print("true")
        update_visuals(rig_body, obstacles)


def update_visuals(rig_body, obstacles):
    rig_body.ax.cla()
    rig_body.set_obstacles(obstacles)
    rig_body.ax.set_ylim([0, 2])
    rig_body.ax.set_xlim([0, 2])
    rig_body.ax.add_patch(rig_body.car)
    plt.draw()
    plt.pause(1e-5)
    rig_body.ax.figure.canvas.draw()
    print('\n')


def main():
    parser = argparse.ArgumentParser(
        description="finding the two configs")
    parser.add_argument('--start', type=float, nargs=3, required=True, help='start orientation')
    parser.add_argument('--map', required=True, help='file with map information')
    parser.add_argument('--goal', type=float, nargs=3, required=True, help='target orientation')
    args = parser.parse_args()

    start = np.array([*args.start[:2], change_angles(args.start[2])])
    goal = np.array([*args.goal[:2], change_angles(args.goal[2])])
    poly_map = load_polygons(args.map)

    calculate_tree(start, goal, poly_map)


if __name__ == '__main__':
    main()

# done
import argparse
from math import pi, radians, floor, sqrt
import random
from create_scene import load_polygons
from planar_arm import Arm_Controller, angle_mod
import numpy as np
import matplotlib.pyplot as plt

class treeNode:
    def __init__(self, x, y):
        self.x, self.y, self.children, self.parent = x, y, [], None

class RTT:
    def __init__(self, start, goal, iterations, stepSize):
        self.randomTree, self.goal = treeNode(*start), treeNode(*goal)
        self.iterations, self.rho = min(iterations, 1000), stepSize
        self.path_distance, self.nearestDist, self.numWaypoints = 0, float('inf'), 0
        self.nearestNode, self.Waypoints = None, []

    def addChild(self, x, y):
        newNode = self.goal if (x, y) == (self.goal.x, self.goal.y) else treeNode(x, y)
        self.nearestNode.children.append(newNode)
        newNode.parent = self.nearestNode

    def samplePoint(self, goal):
        return np.array(goal if random.random() < 0.05 else [random.uniform(-pi, pi) for _ in range(2)])

    def goToPoint(self, start, end):
        return np.array([start.x, start.y]) + self.rho * self.unitVector(start, end)

    def isInObstacle(self, start, end, arm):
        u_hat = self.unitVector(start, end)
        for i in range(floor(self.rho / radians(5))):
            test = [start.x + i * u_hat[0], start.y + i * u_hat[1]]
            arm.theta1, arm.theta2 = test
            arm.re_orient()
            if any(arm.check_arm_collisions()):
                return True
        return False

    def unitVector(self, start, end):
        v = np.array([end[0] - start.x, end[1] - start.y])
        return v / np.linalg.norm(v)

    def findNearest(self, root, point):
        if root:
            dist = self.distance(root, point)
            if dist < self.nearestDist:
                self.nearestNode, self.nearestDist = root, dist
            for child in root.children:
                self.findNearest(child, point)

    def distance(self, node, point):
        return sqrt((node.x - point[0]) ** 2 + (node.y - point[1]) ** 2)

    def goalFound(self, point):
        return self.distance(self.goal, point) <= self.rho

    def resetNearestValues(self):
        self.nearestNode, self.nearestDist = None, float('inf')

    def retracePath(self, goal):
        if (goal.x, goal.y) != (self.randomTree.x, self.randomTree.y):
            self.numWaypoints += 1
            self.Waypoints.insert(0, np.array([goal.x, goal.y]))
            self.path_distance += self.rho
            self.retracePath(goal.parent)


def rtt_tree(start, goal, arm):
    plt.close('all')
    fig, ax = plt.subplots()
    ax.plot(start[0], start[1], 'ro')
    ax.set_xlim(-pi, pi)
    ax.set_ylim(-pi, pi)
    rrt = RTT(start, goal, 1000, radians(5))

    for i in range(1000):
        rrt.resetNearestValues()
        point = rrt.samplePoint(goal)
        rrt.findNearest(rrt.randomTree, point)
        new_point = rrt.goToPoint(rrt.nearestNode, point)

        if not rrt.isInObstacle(rrt.nearestNode, new_point, arm):
            rrt.addChild(*new_point)
            ax.plot([rrt.nearestNode.x, new_point[0]], [rrt.nearestNode.y, new_point[1]], 'go', linestyle="--")
            plt.pause(0.01)

            if rrt.goalFound(new_point):
                rrt.addChild(*goal)
                print("Goal Found")
                break
        elif not i:
            print("Error: Start node in obstacle")
            break
    else:
        plt.pause(1)

    rrt.retracePath(rrt.goal)
    rrt.Waypoints.insert(0, start)
    waypoint_pairs = zip(rrt.Waypoints[:-1], rrt.Waypoints[1:])

    for (wp_start, wp_end) in waypoint_pairs:
        ax.plot([wp_start[0], wp_end[0]], [wp_start[1], wp_end[1]], 'ro', linestyle="--")

    plt.pause(1)
    plt.close()


def arm_graph(start, arm, waypoints):
    begin = start
    for waypoint in waypoints:
        move_arm(arm, begin, waypoint)
        begin = waypoint


def interpolate(start, goal, resolution):
    x1, y1 = start
    x2, y2 = goal
    slope = (y2 - y1) / (x2 - x1)
    num_points = abs(int((x2 - x1) / resolution)) + 1
    points = [(x1 + i * resolution, slope * (x1 + i * resolution - x1) + y1) for i in range(num_points)]
    return points + [goal] if points[-1] != goal else points


def move_arm(arm, start, goal):
    for pt in interpolate(start, goal, radians(5)):
        print(pt)
        arm.set_joint_angles(pt)
        arm.re_orient()
        arm.ax.cla()
        arm.draw_arm()
        arm.ax.figure.canvas.draw()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Find two configurations that are closest to the target.")
    parser.add_argument('--start', type=float, nargs=2, required=True, help='Start orientation')
    parser.add_argument('--map', required=True, help='Map file with polygons')
    parser.add_argument('--goal', type=float, nargs=2, required=True, help='Target orientation')
    args = parser.parse_args()

    # Directly use modified angles and load polygons for the arm controller
    planar_arm = Arm_Controller(*[angle_mod(arg) for arg in args.start])
    planar_arm.set_arm_obs(load_polygons(args.map))

    # Start the RTT tree process
    rtt_tree(np.array([angle_mod(arg) for arg in args.start]), np.array([angle_mod(arg) for arg in args.goal]),
             planar_arm)

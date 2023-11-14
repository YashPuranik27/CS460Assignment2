import numpy as np
import matplotlib.pyplot as plt
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
import random
from math import pi, radians, floor, sqrt
from planar_car import PlanarCar
import argparse

# Initial state [x, y, theta]
init_x = 1
init_y = 1
init_q = np.array([init_x, init_y, 0.0])
q = init_q

# Control input [v, omega]
init_v = 0.4
init_o = 0.5
u = np.array([init_v, init_o])

# Robot dimensions
length = 0.2
width = 0.1

# Time step
dt = 0.1



# Control limits
v_max = 0.5
v_min = -0.5
omega_max = np.pi / 4
omega_min = omega_max*-1

tireAngle=0

tireOffest=0.5


# node for RRT search tree
class treeNode:
    def __init__(self, x, y):
        self.x, self.y, self.children, self.parent = x, y, [], None



# RRT Claass ===============================================================
class RRT():

    def __init__(self, start, goal, numIterations, grid, stepSize=0.2):
        self.randomTree, self.goal = treeNode(*start), treeNode(*goal)
        self.iterations, self.rho = min(numIterations, 1000), stepSize
        self.path_distance, self.nearestDist, self.numWaypoints = 0, float('inf'), 0
        self.nearestNode, self.Waypoints = None, []

    #plan a path from start to end points
    def plan(self, start, end):
        pass

    #sample a random point (within the grid)
    def samplePoint(self, goal):
        return np.array(goal if random.random() < 0.05 else [random.uniform(-pi, pi) for _ in range(2)])

    #add to the nearest node
    def addChild(self, x, y):
        newNode = self.goal if (x, y) == (self.goal.x, self.goal.y) else treeNode(x, y) # if goal, add it, else , add the node
        self.nearestNode.children.append(newNode)
        newNode.parent = self.nearestNode

    def goToPoint(self, start, end):
        return np.array([start.x, start.y]) + self.rho * self.unitVector(start, end)

    #check if hit obstacle between the start and the end points
    def isInObstacle(self, start, end, car):
        u_hat = self.unitVector(start, end)
        for i in range(floor(self.rho / radians(5))):
            test = [start.x + i * u_hat[0], start.y + i * u_hat[1]]
            #car.theta1, car.theta2 = test
            #car.re_orient()
            #if any(car.check_arm_collisions()):
            #    return True
        return False

    def unitVector(self, start, end):
        v = np.array([end[0] - start.x, end[1] - start.y])
        return v / np.linalg.norm(v)

    #find the nearest node from the point
    def findNearest(self, root, point):
        if root:
            dist = self.distance(root, point)
            if dist < self.nearestDist:
                self.nearestNode, self.nearestDist = root, dist
            for child in root.children:
                self.findNearest(child, point)

    #calculate the distance between the node and the point
    def distance(self, node, point):
        return sqrt((node.x - point[0]) ** 2 + (node.y - point[1]) ** 2)

    #check if the goal is reached
    def goalFound(self, point):
        return self.distance(self.goal, point) <= self.rho

    def resetNearestValues(self):
        self.nearestNode, self.nearestDist = None, float('inf')

    #find the path between start and goal
    def retracePath(self, goal):
        if (goal.x, goal.y) != (self.randomTree.x, self.randomTree.y):
            self.numWaypoints += 1
            self.Waypoints.insert(0, np.array([goal.x, goal.y]))
            self.path_distance += self.rho
            self.retracePath(goal.parent)


# End of RRT Claass =========================================================

def rrt_tree(start, goal, car, map):

    global q

    print(start)
    print(goal)

    plt.close('all')
    fig, ax = plt.subplots()
    ax.plot(start[0], start[1], 'ro')
    ax.set_xlim(-pi, pi)
    ax.set_ylim(-pi, pi)
    #plt.xlim(0, 2)
    #plt.ylim(0, 2)

    [add_polygon_to_scene(poly, ax, True) for poly in poly_map]
    dq = car.differential_drive_model(q, u)
    q += dq
    car.draw_rotated_tire(ax, [q[0], q[1]], length, width, np.degrees(q[2] + tireAngle))
    car.draw_rotated_rectangle(ax, [q[0], q[1]], length, width, np.degrees(q[2]))


    rrt = RRT(start, goal, 1000, radians(5))

    for i in range(1000):
        rrt.resetNearestValues()
        point = rrt.samplePoint(goal)
        rrt.findNearest(rrt.randomTree, point)
        new_point = rrt.goToPoint(rrt.nearestNode, point)

        if not rrt.isInObstacle(rrt.nearestNode, new_point, car):
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
    #plt.close()
    plt.show()



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="RRT planning with ddynamics.")
    parser.add_argument('--start', type=float, nargs=3, required=True, help='Start orientation')
    parser.add_argument('--goal', type=float, nargs=3, required=True, help='Target orientation')
    parser.add_argument('--map', required=True, help='Map file with polygons')
    args = parser.parse_args()

    # Directly use modified angles and load polygons for the arm controller
    car = PlanarCar()
    #car.set_arm_obs(load_polygons(args.map))

    # Initialize plot
    #fig, ax = plt.subplots(figsize=(6, 6))
    poly_map = load_polygons(args.map)
    #plt.clf()
    #ax = plt.gca()
    #plt.xlim(0, 2)
    #plt.ylim(0, 2)

    #show_scene(ax)


    # Start the RTT tree process
    rrt_tree((args.start[0],args.start[1]), (args.goal[0],args.goal[1]), car, poly_map)

import numpy as np
import matplotlib.pyplot as plt
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
import random
from math import pi, radians, floor, sqrt
from planar_car import PlanarCar
import argparse
import math

from collision_checking import collides_SAT

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
x_max = 2
x_min = 0
v_max = 0.5
v_min = -0.5
omega_max = np.pi / 4
omega_min = omega_max*-1

tireAngle=0

tireOffest=0.5


# node for RRT search tree
class treeNode:
    def __init__(self, x, y, theta = 0, control = [0,0]):
        self.x, self.y, self.children, self.parent = x, y, [], None
        self.control = control
        self.theta = theta

    def distance(self, node):
        return math.sqrt(((self.x - node.x) ** 2) + ((self.y - node.y) ** 2))


# RRT Claass ===============================================================
class RRT():

    def __init__(self, start, goal, numIterations, grid, stepSize=0.2):
        self.randomTree, self.goal = treeNode(*start), treeNode(*goal)
        self.iterations, self.rho = min(numIterations, 1000), stepSize
        self.path_distance, self.nearestDist, self.numWaypoints = 0, float('inf'), 0
        self.nearestNode, self.Waypoints = None, []
        self.polyMap = []

    #plan a path from start to end points
    def plan(self, start, end):
        pass

    #sample a random point (within the grid)
    def samplePoint(self, goal):
        return np.array(goal if random.random() < 0.05 else [random.uniform(x_min, x_max) for _ in range(2)])

    # sample a set of random controls
    def sampleControl(self):
        controls = []
        for i in range(6):
            controls.append((np.random.randint(-5, 5+1), np.random.randint(-8, 8+1)))
        controls = np.array(controls)/10
        return  controls

    #add to the nearest node
    def addChild(self, child):
        newNode = child
        if (child.x, child.y) == (self.goal.x, self.goal.y):
            newNode = self.goal
        self.nearestNode.children.append(newNode)
        newNode.parent = self.nearestNode


    def addChild_old(self, x, y, angle=0):
        newNode = self.goal if (x, y) == (self.goal.x, self.goal.y) else treeNode(x, y) # if goal, add it, else , add the node
        self.nearestNode.children.append(newNode)
        newNode.parent = self.nearestNode

    def goToPoint(self, start, end):
        return np.array([start.x, start.y]) + self.rho * self.unitVector(start, end)


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

    # find the nearest node from the point
    def findNewNode(self, node, car, control):
        q= [node.x, node.y, node.theta]
        dq = car.differential_drive_model(q, control)
        print(dq)
        q+=dq
        print(q)
        return treeNode(q[0], q[1], q[2], control)


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

    # check if hit obstacle between the start and the end points
    def isInObstacle(self, start, end, poly_map):

        for poly in poly_map:
            for i in range(len(poly)):
                if (i == len(poly)-1):
                    p1=treeNode(*poly[i])
                    p2=treeNode(*poly[0])
                else:
                    p1=treeNode(*poly[i])
                    p2=treeNode(*poly[i+1])

                if self.isIntersect(p1, p2, start, end):
                    return True

        return False

    # Computes the direction of the three given points
    # Returns a positive value if they form a counter-clockwise orientation,
    # A negative value if they form a clockwise orientation,
    # And zero if they are collinear
    def direction(self, p, q, r):
        return (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)

    # Checks if two line segments are collinear and overlapping
    def areCollinearAndOverlapping(self, a1, b1, a2, b2):
        # Check if the line segments are collinear
        if self.direction(a1, b1, a2) == 0:
            # Check if the line segments overlap
            if a2.x <= max(a1.x, b1.x) and a2.x >= min(a1.x, b1.x) and a2.y <= max(a1.y, b1.y) and a2.y >= min(
                    a1.y,
                    b1.y):
                return True
        return False


    # Checks if two line segments intersect or not
    def isIntersect(self, a1, b1, a2, b2):
        # Compute the directions of the four line segments
        d1 = self.direction(a1, b1, a2)
        d2 = self.direction(a1, b1, b2)
        d3 = self.direction(a2, b2, a1)
        d4 = self.direction(a2, b2, b1)

        # Check if the two line segments intersect
        if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
            return True

        # Check if the line segments are collinear and overlapping
        if self.areCollinearAndOverlapping(a1, b1, a2, b2) or self.areCollinearAndOverlapping(a2, b2, a1, b1):
            return True

        return False


# End of RRT Claass =========================================================




def rrt_tree(ax, start, goal, car, poly_map):

    global q

    #print(start)
    #print(goal)


    rrt = RRT(start, goal, 1000, radians(5))
    rrt.polyMap = poly_map

    for i in range(1000):

    #for i in range(100):
        rrt.resetNearestValues()
        samplePoint = rrt.samplePoint(goal)
        print("============sample point")
        print(samplePoint)
        print("============sample control")
        controls = rrt.sampleControl()
        print(controls)

        rrt.findNearest(rrt.randomTree, samplePoint)
        print("============nearest")
        print(rrt.nearestNode.x, rrt.nearestNode.y, rrt.nearestNode.theta)


        closest = 999
        #loop through the control samples
        for control in controls:
            newNode = rrt.findNewNode(rrt.nearestNode, car, control)
            print("============newNode")
            distance = newNode.distance(rrt.nearestNode)
            if distance < closest:
                closest = distance
                closestNewNode = newNode
            print(control, newNode.x, newNode.y, newNode.theta, "-----", distance)

        print("closestNewNode", closestNewNode.x, closestNewNode.y)

        #new_point = rrt.goToPoint(rrt.nearestNode, point)

        new_point =  closestNewNode

        if not rrt.isInObstacle(rrt.nearestNode, new_point, poly_map):
            rrt.addChild(new_point)
            ax.plot([rrt.nearestNode.x, new_point.x], [rrt.nearestNode.y, new_point.y], 'go', linestyle="--")
            plt.pause(0.01)

            if rrt.goalFound((new_point.x, new_point.y)):
                rrt.addChild(treeNode(*goal))
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


    init_x = args.start[0]
    init_y = args.start[1]
    init_q = np.array([init_x, init_y, 0.0])
    q = init_q

    plt.close('all')
    fig, ax = plt.subplots()
    #ax.plot(args.start[0], args.start[1], 'ro')
    # ax.set_xlim(-pi, pi)
    # ax.set_ylim(-pi, pi)
    plt.xlim(x_min, x_max)
    plt.ylim(x_min, x_max)

    dq = car.differential_drive_model(q, u)
    q += dq
    car.draw_rotated_tire(ax, [q[0], q[1]], length, width, np.degrees(q[2] + tireAngle))
    car.draw_rotated_rectangle(ax, [q[0], q[1]], length, width, np.degrees(q[2]))

    circle = plt.Circle((args.start[0], args.start[1]), 0.05, color='orange')
    ax.add_artist(circle)
    circle = plt.Circle((args.goal[0], args.goal[1]), 0.05, color='purple')
    ax.add_artist(circle)

    #car.set_arm_obs(load_polygons(args.map))

    # Initialize plot
    #fig, ax = plt.subplots(figsize=(6, 6))
    poly_map = load_polygons(args.map)

    [add_polygon_to_scene(poly, ax, True) for poly in poly_map]

    #plt.clf()
    #ax = plt.gca()
    #plt.xlim(0, 2)
    #plt.ylim(0, 2)

    #show_scene(ax)


    # Start the RTT tree process
    car.dt = 1
    rrt_tree(ax, (args.start[0],args.start[1], args.start[2]), (args.goal[0],args.goal[1],args.goal[2]), car, poly_map)

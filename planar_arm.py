# Original code from assignment 1

import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from collision_checking import collides_SAT
from matplotlib.colors import *
from matplotlib.patches import *


class PlanarArm:
    def __init__(self, ax):
        # Initialize variables
        self.theta1 = self.theta2 = 0
        # radius, length between joints 1 and 2, length, width
        self.radius, self.L1, self.L2, self.W = 0.05, 0.4, 0.25, 0.1
        self.robot, self.start = [], True

        # Load and setup scene
        self.polygons = np.load('arm_polygons.npy', allow_pickle=True)
        self.ax, self.fig = ax, ax.figure

        # Set up ax parameters and plot initial arm
        ax.set(xlim=(0, 2), ylim=(0, 2), aspect='equal')
        self.plot_arm()

        # Connect event handler and display occupancy grid
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.visualize_configuration_space()

    def plot_arm(self):
        self.ax.clear()
        self.robot.clear()

        points = self.make_robot()

        for i in range(0, len(points), 2):
            self.make_joint(self.radius, points[i], points[i + 1])

        for i, final in enumerate([False, False, True, True, True]):
            self.make_link(np.array(self.robot[i]), final)

        self.plot_polygons()
        self.ax.axis([0, 2, 0, 2])
        self.fig.canvas.draw()

    def plot_polygons(self):
        if self.start:
            for part in self.robot:
                self.collision_on_spawn(part)
            self.start = False

        [self.ax.add_patch(Polygon(p, closed=True, ec='purple', facecolor='yellow', alpha=0.3)) for p in self.polygons]

        self.fig.canvas.draw()

    def make_robot(self):
        x0, y0 = 1, 1
        coords = [(x0, y0)]

        # Calculate vertices
        for (L, theta) in [(self.L1, self.theta1), (self.L2, self.theta2 + self.theta1)]:
            x, y = coords[-1]
            x_new = x + (L + .1) * np.cos(theta)
            y_new = y + (L + .1) * np.sin(theta)
            coords.append((x_new, y_new))

        x2, y2 = coords[-1]

        # Construct robot parts
        link_args = [
            (coords[0], self.radius, self.theta1, self.L1, self.W),
            (coords[1], self.radius, self.theta2 + self.theta1, self.L2, self.W),
            (coords[0], -0.05, self.theta1, self.radius * 2, self.radius * 2),
            (coords[1], -0.05, self.theta2, self.radius * 2, self.radius * 2),
            ((x2, y2), -0.05, self.theta2, self.radius * 2, self.radius * 2)
        ]

        self.robot.extend([np.array(self.get_link(*args)) for args in link_args])

        return [c for coord in coords for c in coord]  # Flatten coords to [x0, y0, x1, y1, x2, y2]

    def make_joint(self, radius, x, y):
        circle = Circle((x, y), radius, fill=True, ec='blue', color='blue')
        self.ax.add_patch(circle)

    def make_link(self, rectangle, bounding_box):
        if bounding_box:
            arm1 = Polygon(rectangle)
        else:
            arm1 = Polygon(rectangle, closed=True, ec='green', facecolor='green', alpha=0.5)
            self.ax.add_patch(arm1)

    def get_link(self, joint, radius, angle, length, width):
        # initialize rectangle points
        x0, y0 = joint[0] + radius, joint[1] - width / 2
        points = np.array([[x0, x0 + length, x0 + length, x0],
                           [y0, y0, y0 + width, y0 + width]])

        # get center point and shift points
        M = np.array(joint)
        points -= M.reshape(2, 1)

        # apply rotation matrix
        R = np.array([[np.cos(angle), -np.sin(angle)],
                      [np.sin(angle), np.cos(angle)]])
        points = np.dot(R, points)

        # shift points back and create rectangle
        points += M.reshape(2, 1)

        return points.T.tolist() + [points.T.tolist()[0]]

        # checks for collisions as the arm spawns in

    def collision_on_spawn(self, poly):
        to_delete = []
        for i, polygon in enumerate(self.polygons):
            if collides_SAT(poly, polygon):
                to_delete.append(i)
        self.polygons = np.delete(self.polygons, to_delete, 0)

    # polygon collision checking. Will cancel out if collision is detected
    def collides(self, polygon1, polygon2):
        self.plot_arm()

        if any(collides_SAT(part, polygon)
               for polygon in self.polygons for part in self.robot):
            self.theta1, self.theta2 = polygon1, polygon2
            self.plot_arm()

    def on_key_press(self, event):
        # hold previous joint angles
        prev_theta1 = self.theta1
        prev_theta2 = self.theta2

        step = np.pi / 30  # angle step
        self.event = event.key
        if event.key == 'up':
            self.theta1 += step
        elif event.key == 'down':
            self.theta1 -= step
        elif event.key == 'right':
            self.theta2 += step
        elif event.key == 'left':
            self.theta2 -= step

        # collision checking, will cancel out the movement if there is a collision
        self.collides(prev_theta1, prev_theta2)

    def visualize_configuration_space(self):
        data = np.zeros((100, 100), dtype=int)
        step = np.pi / 50

        for x, y in np.ndindex(data.shape):
            self.theta1, self.theta2 = x * step, y * step

            self.robot.clear()
            self.make_robot()

            if any(collides_SAT(part, polygon)
                   for polygon in self.polygons for part in self.robot):
                data[x, y] = 1

        plt.figure(figsize=(7, 7))
        plt.imshow(data.T, extent=(0, 100, 100, 0), cmap=ListedColormap(['purple', 'yellow']))
        plt.title('collision free c-space'), plt.xlabel('joint 1'), plt.ylabel('joint 2')
        plt.show()


fig, ax = plt.subplots()
robot = PlanarArm(ax)
plt.show()
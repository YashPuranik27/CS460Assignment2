from create_scene import load_polygons
from collision_checking import *
from matplotlib.colors import *
from matplotlib.patches import *
import matplotlib.patches as patches
from numpy import degrees
import matplotlib.pyplot as plt

import numpy as np


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
            if SAT_Collides(poly, polygon):
                to_delete.append(i)
        self.polygons = np.delete(self.polygons, to_delete, 0)

    # polygon collision checking. Will cancel out if collision is detected
    def collides(self, polygon1, polygon2):
        self.plot_arm()

        if any(SAT_Collides(part, polygon)
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

            if any(SAT_Collides(part, polygon)
                   for polygon in self.polygons for part in self.robot):
                data[x, y] = 1

        plt.figure(figsize=(7, 7))
        plt.imshow(data.T, extent=(0, 100, 100, 0), cmap=ListedColormap(['purple', 'yellow']))
        plt.title('collision free c-space'), plt.xlabel('joint 1'), plt.ylabel('joint 2')
        plt.show()


class Control_Arm:

    @staticmethod
    def calculate_rectangle(theta, center, rad, width):
        x, y = rad * np.cos(theta), rad * np.sin(theta)
        orth_x, orth_y = -y / rad, x / rad
        return center[0] + x + orth_x * width / 2, center[1] + y + orth_y * width / 2

    @staticmethod
    def calculate_circle(theta, center, rad, rlen):
        total_rad = rad * 2 + rlen
        return center[0] + total_rad * np.cos(theta), center[1] + total_rad * np.sin(theta)

    def __init__(self, theta1, theta2, ax=create_plot(), polygons=[]):
        if ax is None:
            ax = create_plot()
        if polygons is None:
            polygons = []
        self.ax = ax
        self.joint1 = (1, 1)
        self.rad = 0.05
        self.rwid = 0.1
        self.rlen1 = 0.3
        self.rlen2 = 0.15
        self.theta1 = theta1
        self.theta2 = theta2
        self.anchor1 = Control_Arm.calculate_rectangle(self.theta1, self.joint1, self.rad, self.rwid)
        self.joint2 = Control_Arm.calculate_circle(self.theta1, self.joint1, self.rad, self.rlen1)
        self.anchor2 = Control_Arm.calculate_rectangle(self.theta2, self.joint2, self.rad, self.rwid)
        self.joint3 = Control_Arm.calculate_circle(self.theta2, self.joint2, self.rad, self.rlen2)
        self.polygons = polygons

    def joint_angle(self, angles):
        self.theta1, self.theta2 = angles

    def print_arm(self, collisions=[False] * 5):
        self.ax.clear()
        self.set_plot()
        components = [
            patches.Circle(self.joint1, self.rad, fill=True, color='b' if not collisions[0] else 'r'),
            patches.Rectangle(self.anchor1, self.rwid, self.rlen1, angle=degrees(self.theta1 - np.pi / 2), fill=True,
                              color='g' if not collisions[3] else 'r'),
            patches.Circle(self.joint2, self.rad, fill=True, color='b' if not collisions[1] else 'r'),
            patches.Rectangle(self.anchor2, self.rwid, self.rlen2, angle=degrees(self.theta2 - np.pi / 2), fill=True,
                              color='g' if not collisions[4] else 'r'),
            patches.Circle(self.joint3, self.rad, fill=True, color='b' if not collisions[2] else 'r')
        ]

        for component in components:
            self.ax.add_patch(component)

        self.ax.set_xlim([0, 2])
        self.ax.set_ylim([0, 2])
        plt.draw()
        plt.pause(1e-5)

    # Draws the arm without adding it to the scene
    def add_more_arm(self, color='b', collisions=[False] * 5):
        comp = [patches.Circle(self.joint1, self.rad, fill=True, color=color),
                patches.Circle(self.joint2, self.rad, fill=True, color=color),
                patches.Circle(self.joint3, self.rad, fill=True, color=color),
                patches.Rectangle(self.anchor1, self.rwid, self.rlen1, angle=degrees(self.theta1 - np.pi / 2),
                                  fill=False, color=color),
                patches.Rectangle(self.anchor2, self.rwid, self.rlen2, angle=degrees(self.theta2 - np.pi / 2),
                                  fill=False, color=color)]

        for i, collision in enumerate(collisions):
            if collision:
                comp[i].set_color('r')

        for element in comp:
            self.ax.add_patch(element)

    def change_orientation(self):
        self.anchor1 = Control_Arm.calculate_rectangle(self.theta1, self.joint1, self.rad, self.rwid)
        self.joint2 = Control_Arm.calculate_circle(self.theta1, self.joint1, self.rad, self.rlen1)
        self.anchor2 = Control_Arm.calculate_rectangle(self.theta2, self.joint2, self.rad, self.rwid)
        self.joint3 = Control_Arm.calculate_circle(self.theta2, self.joint2, self.rad, self.rlen2)

    def initial_collision_avoider(self):
        self.theta1, self.theta2 = 0, 0
        colliding_polygons = self.check_collisions_from_arm(signal=True)
        self.polygons = np.array([poly for poly in self.polygons if poly not in colliding_polygons], dtype=object)

    @staticmethod
    def get_rectangle_angles(anchor, width, height, angle):
        # Create a rotation matrix and rectangle corners
        c, s = np.cos(angle), np.sin(angle)
        rotation_matrix = np.array([[c, -s], [s, c]])
        corners = np.array([[0, 0], [width, 0], [width, height], [0, height]])

        # Apply rotation and translation in one step
        return np.dot(corners, rotation_matrix.T) + anchor

    def set_arms(self, polygons):
        self.polygons = polygons

    def set_plot(self):
        for p in self.polygons:
            add_polygon_to_scene(p, self.ax, False)

    def check_collisions_from_arm(self, signal=False):
        circles = [self.joint1, self.joint2, self.joint3]
        rectangles = [Control_Arm.get_rectangle_angles(self.anchor1, self.rwid, self.rlen1, self.theta1 - np.pi / 2),
                      Control_Arm.get_rectangle_angles(self.anchor2, self.rwid, self.rlen2, self.theta2 - np.pi / 2)]
        circ_boxes = [bounding_for_circle(circle, self.rad) for circle in circles]
        rec_boxes = bounding(rectangles)
        poly_boxes = bounding(self.polygons)

        possible_circle_collisions = [(circles[i], self.polygons[j]) for i in range(len(circles))
                                      for j in range(len(self.polygons)) if
                                      box_col(circ_boxes[i], poly_boxes[j])]

        possible_rect_collisions = [(rectangles[i], self.polygons[j]) for i in range(len(rectangles))
                                    for j in range(len(self.polygons)) if
                                    box_col(rec_boxes[i], poly_boxes[j])]

        colliding_polygons = []
        joint_coll = [False] * 3
        for circle, polygon in possible_circle_collisions:
            if circle_poly_collides(circle, self.rad, polygon):
                colliding_polygons.append(polygon)
                joint_coll[circles.index(circle)] = True

        arm_coll = [False] * 2
        for rect, polygon in possible_rect_collisions:
            if SAT_Collides(rect, polygon):
                colliding_polygons.append(polygon)
                for i, rectangle in enumerate(rectangles):
                    if np.array_equal(rect, rectangle):
                        arm_coll[i] = True
                        break

        return colliding_polygons if signal else joint_coll + arm_coll


def change_angles(x, zero_2_2pi=False, degree=False):
    # Determine the data type of the input
    is_float = isinstance(x, float)

    # Convert the input to a flattened numpy array and convert to radians if necessary
    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    # Calculate the modulus of the angle
    mod_angle = x % (2 * np.pi) if zero_2_2pi else (x + np.pi) % (2 * np.pi) - np.pi

    # Convert back to degrees if necessary
    if degree:
        mod_angle = np.rad2deg(mod_angle)

    # Return a scalar if the input was a float, otherwise return the array
    return mod_angle.item() if is_float else mod_angle


def main():
    fig, ax = plt.subplots(dpi=100)
    arm = Control_Arm(0, 0, ax)
    obstacles = load_polygons('arm_polygons.npy')
    arm.set_arms(obstacles)
    arm.initial_collision_avoider()
    arm.set_plot()
    arm.print_arm()
    show_scene(arm.ax)


if __name__ == '__main__':
    main()


import matplotlib.patches as patches
import numpy as np
from create_scene import show_scene, create_plot, add_polygon_to_scene
from collision_checking import collides
import random
import math


class CarController:
    def __init__(self, ax, car, obstacles):
        self.car, self.ax, self.obstacles = car, ax, obstacles
        self.degrees, self.fig = car.get_angle, ax.figure
        ax.set_xlim(0, 2), ax.set_ylim(0, 2), ax.add_patch(car)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles
        for p in obstacles: add_polygon_to_scene(p, self.ax, True)

    def on_key_press(self, event):
        key, step = event.key, 0.05
        radians = math.radians(self.degrees())
        x_step = step * math.cos(radians)
        y_step = step * math.sin(radians)
        move = {'up': (x_step, y_step), 'down': (-x_step, -y_step),
                'left': (0, 0, 10), 'right': (0, 0, -10)}

        if key in ['up', 'down']:
            dx, dy = move[key]
            self.car.set_x(self.car.get_x() + dx)
            self.car.set_y(self.car.get_y() + dy)
        elif key in ['left', 'right']:
            _, _, angle_step = move[key]
            self.car.set_angle(self.degrees() + angle_step)

        if not (check_car(self.car, self.obstacles) and check_boundary(self.car)):
            if key in ['up', 'down']:
                self.car.set_x(self.car.get_x() - dx)
                self.car.set_y(self.car.get_y() - dy)
            elif key in ['left', 'right']:
                self.car.set_angle(self.degrees() - angle_step)

        self.fig.canvas.draw()


def check_boundary(car):
    return all(0 <= x <= 2 and 0 <= y <= 2 for x, y in get_coords(car))


def check_car(car, obstacles):
    return all(collides(polygon, get_coords(car)) for polygon in obstacles)



def get_coords(r1):
    return r1.get_corners()


def check_car_spawn(obstacles):
    generator = ((random.uniform(0, 1), random.uniform(0, 1)) for _ in iter(int, 1))
    for x, y in generator:
        car = patches.Rectangle((x, y), 0.2, 0.1, linewidth=1, edgecolor='r', angle=0, facecolor='blue')
        if check_car(car, obstacles):
            return car


if __name__ == '__main__':
    obstacles = np.load('arm_polygons.npy', allow_pickle=True)
    ax = create_plot()
    for polygon in obstacles:
        add_polygon_to_scene(polygon, ax, 'blue')
    car = check_car_spawn(obstacles)
    ax.add_patch(car)
    controller = CarController(ax, car, obstacles)
    show_scene(ax)

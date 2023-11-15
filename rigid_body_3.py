# done
import argparse
from math import sqrt, pi, degrees
from rigid_body import CarController
from create_scene import create_plot
from rigid_body_1 import make_rigid_body
import matplotlib.pyplot as plt


def get_slope_vector_concise(pt1, pt2):
    magnitude = sqrt(sum((x2 - x1) ** 2 for x1, x2 in zip(pt1, pt2)))
    return tuple((x2 - x1) / magnitude for x1, x2 in zip(pt1, pt2))


def change_car(config, car):
    car.car.set_x(config[0])
    car.car.set_y(config[1])
    car.car.set_angle(degrees(config[2]))


def interpolate(start, goal, res=.05):
    x1, y1, t1 = start
    x2, y2, t2 = goal
    delta_theta = (t2 - t1 + pi) % (2 * pi) - pi
    num_points = int(max(sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2), abs(delta_theta)) / res)

    points = [(x1 + (x2 - x1) * i / num_points,
               y1 + (y2 - y1) * i / num_points,
               t1 + delta_theta * i / num_points) for i in range(num_points + 1)]

    if points[-1] != goal:
        points.append(goal)

    return points


def main():
    parser = argparse.ArgumentParser(description='Animate interpolation in SE(2).')
    parser.add_argument('--start', nargs=3, type=float, required=True)
    parser.add_argument('--goal', nargs=3, type=float, required=True)
    args = parser.parse_args()

    points = interpolate(args.start, args.goal, 0.05)
    plt.close('all')
    ax = create_plot()
    rig_body = CarController(ax=ax, car=make_rigid_body(args.start[:2]), obstacles=[])
    rig_body.car.set_angle(degrees(args.start[2]))

    for pt in points:
        change_car(pt, rig_body)
        ax.cla()
        ax.set_ylim([0, 2])
        ax.set_xlim([0, 2])
        ax.add_patch(rig_body.car)
        plt.pause(.1)


if __name__ == '__main__':
    main()

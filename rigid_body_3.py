# done
import argparse
from math import sqrt, degrees, radians, atan
from rigid_body import CarController
from create_scene import create_plot
from rigid_body_1 import make_rigid_body
from planar_arm import angle_mod
import matplotlib.pyplot as plt


def reposition_car(config, car):
    x, y, theta = config
    car.car.set_x(x)
    car.car.set_y(y)
    car.car.set_angle(degrees(theta))


def interpolate(start, goal, resP=0.05, resA=radians(5)):
    (x1, y1, t1), (x2, y2, t2) = start, goal
    points = [(x1, y1, t1)]
    angle = atan((y2 - y1) / (x2 - x1)) if x2 != x1 else (
            3.141592653589793 / 2 * (y2 > y1) - 3.141592653589793 / 2 * (y2 < y1))
    for a in range(int((angle - t1) / resA)):
        points.append((x1, y1, angle_mod(t1 + (a + 1) * resA * (1 if angle > t1 else -1))))
    points += [((1 - t) * x1 + t * x2, (1 - t) * y1 + t * y2, angle) for t in
               (i / int(sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) / resP) for i in
                range(int(sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) / resP) + 1))]
    for a in range(int((t2 - angle) / resA)):
        points.append((x2, y2, angle_mod(angle + (a + 1) * resA * (1 if t2 > angle else -1))))
    points.append((x2, y2, t2))
    return points


def main():
    parser = argparse.ArgumentParser(description='Animate points interpolation in SE(2).')
    parser.add_argument('--start', nargs=3, type=float, required=True)
    parser.add_argument('--goal', nargs=3, type=float, required=True)
    args = parser.parse_args()

    plt.close('all')
    rig_body = CarController(ax=create_plot(), car=make_rigid_body(args.start[:2]), obstacles=[])
    rig_body.car.set_angle(degrees(args.start[2]))
    for pt in interpolate(args.start, args.goal):
        reposition_car(pt, rig_body)
        rig_body.ax.cla()
        rig_body.ax.set_ylim([0, 2])
        rig_body.ax.set_xlim([0, 2])
        rig_body.ax.add_patch(rig_body.car)
        plt.draw()
        plt.pause(0.1)
    print('done')


if __name__ == '__main__':
    main()
    print('done')

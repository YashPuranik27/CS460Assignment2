import argparse
from math import radians
from planar_arm import Control_Arm


def interpolate(start, goal, resolution):
    slope = (goal[1] - start[1]) / (goal[0] - start[0])
    num_points = abs(int((goal[0] - start[0]) / resolution)) + 1
    points = [(start[0] + i * resolution, slope * (start[0] + i * resolution - start[0]) + start[1]) for i in
              range(num_points)]
    return points + [goal] if points[-1] != goal else points


def arm_move(disc, args):
    planar_arm = Control_Arm(*args.start)
    for pt in disc:
        print(pt)
        planar_arm.joint_angle(pt)
        planar_arm.change_orientation()
        planar_arm.ax.cla()
        planar_arm.print_arm()
        planar_arm.ax.figure.canvas.draw()
    print('success')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', nargs=2, type=float, required=True, help='Start config radians.')
    parser.add_argument('--goal', nargs=2, type=float, required=True, help='Goal config radians.')
    args = parser.parse_args()

    discretized_pts = interpolate(args.start, args.goal, radians(5))
    arm_move(discretized_pts, args)


if __name__ == '__main__':
    main()

import argparse
from math import radians
from planar_arm import Arm_Controller


# Interpolate points on a line segment
def interpolate(start, goal, resolution):
    slope = (goal[1] - start[1]) / (goal[0] - start[0])
    num_points = abs(int((goal[0] - start[0]) / resolution)) + 1
    points = [(start[0] + i * resolution, slope * (start[0] + i * resolution - start[0]) + start[1]) for i in
              range(num_points)]
    return points + [goal] if points[-1] != goal else points


# Main function to parse arguments and visualize arm movement
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', nargs=2, type=float, required=True, help='Start config radians.')
    parser.add_argument('--goal', nargs=2, type=float, required=True, help='Goal config radians.')
    args = parser.parse_args()

    discretized_pts = interpolate(args.start, args.goal, radians(5))

    planar_arm = Arm_Controller(*args.start)
    for pt in discretized_pts:
        print(pt)
        planar_arm.set_joint_angles(pt)
        planar_arm.re_orient()
        planar_arm.ax.cla()
        planar_arm.draw_arm()
        planar_arm.ax.figure.canvas.draw()
    print('success')


if __name__ == '__main__':
    main()

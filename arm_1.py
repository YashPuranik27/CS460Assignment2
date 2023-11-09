import argparse
from math import pi
import random
from create_scene import create_plot, load_polygons, show_scene
from planar_arm import Arm_Controller


def get_sample(arm):
    while True:
        arm.theta1, arm.theta2 = (random.random() * 2 * pi for _ in range(2))
        arm.re_orient()
        if all(not collision for collision in arm.check_arm_collisions()):
            break


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    args = parser.parse_args()

    ax = create_plot()
    planar_arm = Arm_Controller(0, 0, ax, polygons=load_polygons(args.map))
    planar_arm.set_obs_plot()
    get_sample(planar_arm)
    planar_arm.add_arm()
    show_scene(ax)

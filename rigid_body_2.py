# done
import argparse
from math import sqrt

import matplotlib.pyplot as plt
import numpy as np

from create_scene import create_plot, show_scene
from planar_arm import change_angles
from rigid_body_1 import make_rigid_body


def find_smallest_distances(configs, goal, k):
    distances = [sqrt((c[0] - goal[0]) ** 2 + (c[1] - goal[1]) ** 2) + 0.3 * abs(change_angles(c[2]) - change_angles(c[2])) for
                 c in configs]
    return np.array(configs)[np.argsort(distances)[:k]]


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--configs', required=True)
    parser.add_argument('--k', type=int, required=True)
    parser.add_argument('--target', type=float, nargs=3, required=True)
    args = parser.parse_args()

    plt.close('all')
    ax, colors = create_plot(), ['r', 'g', 'b', 'y']
    configs, target, k = np.load(args.configs), args.target, args.k
    ax.add_patch(make_rigid_body(target[:2]))

    for count, rectangle in enumerate(find_smallest_distances(configs, target, k)):
        body = make_rigid_body((rectangle[0], rectangle[1]), rectangle[2], 0.5)
        body.set_facecolor(colors[min(count, len(colors) - 1)])
        ax.add_patch(body)

    show_scene(ax)

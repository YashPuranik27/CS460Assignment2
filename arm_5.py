from itertools import filterfalse

from planar_arm import Control_Arm
import argparse
from create_scene import create_plot, load_polygons
import numpy as np
import random
import matplotlib.pyplot as plt
from arm_2 import find_distance
from arm_3 import interpolate
from heapq import heappop, heappush
from typing import Callable


class Map:
    def __init__(self, config: tuple, edge: list, paths: dict):
        self.config = config
        self.edges = edge
        self.path = paths


def select():
    return tuple((random.random() * 2 - 1) * np.pi for _ in range(2))


def nearest_neighbors(config, other_configs, k, dist_fn):
    original = Control_Arm(*config)
    dists = [dist_fn(Control_Arm(*oc), original) for oc in other_configs]
    return np.array(other_configs)[np.argsort(dists)[:k]]


def check_collision(planar_arm: Control_Arm, config):
    planar_arm.joint_angle(config)
    planar_arm.change_orientation()
    return any(planar_arm.check_collisions_from_arm())


# need to change PRM
def probabilistic_roadmap(iters, neighbors, k, sampler, robot, collides, dist_fn, discretize, startConfig, endConfig, config_lims, res, animation_fn):
    ax = create_plot()
    ax.set_xlim(*config_lims)
    ax.set_ylim(*config_lims)

    Roadmap = {startConfig: Map(startConfig, edge=[], paths={})}
    for _ in range(iters):
        config = endConfig if random.random() < 0.05 else next(filterfalse(lambda c: collides(robot, c), iter(sampler, 1)))
        Roadmap.setdefault(config, Map(config, edge=[], paths={}))

        for q in neighbors(config, [x for x in Roadmap if x != config], k, dist_fn):
            q = tuple(q)
            path = discretize(config, q, res)
            if all(not collides(robot, pt) for pt in path):
                Roadmap[config].edges.append(q)
                Roadmap[config].path[q] = path
                Roadmap[q].edges.append(config)
                Roadmap[q].path[config] = path[::-1]
                animation_fn(config, Roadmap[config].edges, iters, ax)

    plt.show()
    return Roadmap


def roadmap_animation(config, edges, iters, ax):
    plt.scatter(*config, c='green')
    for edge in edges:
        plt.plot([config[0], edge[0]], [config[1], edge[1]], c='green')
    if iters % 50 == 0:
        ax.figure.canvas.draw()
        plt.pause(1e-6)


def A_star_algorithm(startConfig, goalConfig, Graph, dist_fn):
    def arm_dist(c1, c2):
        return dist_fn(Control_Arm(*c1), Control_Arm(*c2))

    def get_from_config(config):
        return arm_dist(config, goalConfig)

    def find_path(config):
        path = []
        while config:
            path.append(config)
            config = parents[config]
        return path[::-1]

    distances = {startConfig: 0}
    parents = {startConfig: None}
    fringe = [(get_from_config(startConfig), startConfig)]
    while fringe:
        _, curr = heappop(fringe)
        if curr == goalConfig:
            return True, find_path(curr)

        for child in Graph[curr].edges:
            tmpDist = distances[curr] + arm_dist(curr, child)
            if child not in distances or tmpDist < distances[child]:
                distances[child], parents[child] = tmpDist, curr
                heappush(fringe, (tmpDist + get_from_config(child), child))
    return False, []


def plot_points(planar_arm, all_points):
    for pt in all_points:
        planar_arm.joint_angle(pt)
        planar_arm.change_orientation()
        planar_arm.ax.cla()
        planar_arm.print_arm()
        planar_arm.ax.figure.canvas.draw()
        plt.pause(1e-3)
    print('finished')


def main():
    parser = argparse.ArgumentParser(description="Find closest configurations to the target.")
    parser.add_argument('--start', type=float, nargs=2, required=True)
    parser.add_argument('--map', required=True)
    parser.add_argument('--goal', type=float, nargs=2, required=True)
    args = parser.parse_args()

    start_config_tuple = tuple(args.start)  # Convert start to a tuple
    goal_config_tuple = tuple(args.goal)  # Convert goal to a tuple

    planar_arm = Control_Arm(*start_config_tuple, ax=create_plot(), polygons=load_polygons(args.map))
    graph = probabilistic_roadmap(100, nearest_neighbors, 3, select, planar_arm, check_collision, find_distance,
                                  interpolate,
                                  start_config_tuple, goal_config_tuple, (-np.pi, np.pi), np.radians(8),
                                  roadmap_animation)

    success, path = A_star_algorithm(start_config_tuple, goal_config_tuple, graph, find_distance)
    if success:
        all_points = [pt for i in range(len(path) - 1) for pt in graph[path[i]].path[path[i + 1]]]
        planar_arm = Control_Arm(*start_config_tuple, ax=create_plot(), polygons=load_polygons(args.map))
        planar_arm.set_plot()
        plot_points(planar_arm, all_points)


if __name__ == '__main__':
    main()

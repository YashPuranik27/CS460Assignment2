# done
import argparse
from math import pi, sqrt
import random
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from planar_arm import Arm_Controller
import numpy as np
from queue import PriorityQueue


class Node:
    def __init__(self, point,
                 split_dim): self.point, self.split_dim, self.left, self.right = point, split_dim, None, None


def kd_tree(points, depth=0):
    if not points:
        return None

    split_dim = depth % len(points[0])
    points.sort(key=lambda x: x[split_dim])
    median = len(points) // 2

    node = Node(points[median], split_dim)
    node.left = kd_tree(points[:median], depth + 1)
    node.right = kd_tree(points[median + 1:], depth + 1)
    return node


def k_nearest_neighbors(root, query_point, k):
    pq = PriorityQueue()

    def search(node):
        if node:
            dist = -np.linalg.norm(np.array(query_point) - np.array(node.point))
            if len(pq.queue) < k or dist > pq.queue[0][0]:
                pq.put((dist, node.point))
                if len(pq.queue) > k:
                    pq.get()
            search(getattr(node, 'left' if query_point[node.split_dim] < node.point[node.split_dim] else 'right'))
            if len(pq.queue) < k or abs(query_point[node.split_dim] - node.point[node.split_dim]) < -pq.queue[0][0]:
                search(getattr(node, 'right' if query_point[node.split_dim] < node.point[node.split_dim] else 'left'))

    search(root)
    return [point for _, point in sorted(pq.queue, reverse=True)]


def find_smallest_distances(pairs, ax, arm, k):
    return [pairs[i] for i in np.argsort(
        [find_distance(Arm_Controller(theta1, theta2, ax, polygons=[]), arm) for theta1, theta2 in pairs])[:k]]


def find_smallest_distances_kd(pairs, ax, arm, k):
    return k_nearest_neighbors(kd_tree(pairs.tolist()), (arm.theta1, arm.theta2), k)


def find_distance(arm1, arm2):
    return sqrt(sum((a - b) ** 2 for a, b in zip(arm1.joint2, arm2.joint2))) + \
        sqrt(sum((a - b) ** 2 for a, b in zip(arm1.joint3, arm2.joint3)))


if __name__ == '__main__':
    # Initialize parser and parse command line arguments
    parser = argparse.ArgumentParser(description="Find two closest configurations to the target")
    parser.add_argument('--configs', required=True, help='Path to the config file')
    parser.add_argument('-k', required=True, type=int)
    parser.add_argument('--target', nargs=2, required=True, type=float, help='target orientation')
    args = parser.parse_args()

    # Set up arm and plot
    ax = create_plot()
    planar_arm = Arm_Controller(*args.target, ax, polygons=[])
    planar_arm.set_obs_plot()
    planar_arm.re_orient()
    planar_arm.add_arm('b')  # Original Target Arm
    configs = np.load(args.configs)
    smallest_distances = find_smallest_distances(configs, ax, planar_arm, args.k)

    # Re-orient and display arms
    arm_colors = ['r', 'g', 'b', 'y', 'k']
    for count, (theta1, theta2) in enumerate(smallest_distances + [args.target]):
        planar_arm.theta1, planar_arm.theta2 = theta1, theta2
        planar_arm.re_orient()
        planar_arm.add_arm(arm_colors[count % len(arm_colors)])
    show_scene(ax)

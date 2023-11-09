import argparse
from math import sqrt
import numpy as np
from queue import PriorityQueue
from create_scene import create_plot, show_scene
from planar_arm import Arm_Controller


class Node:
    def __init__(self, point, split_dim):
        self.point, self.split_dim, self.left, self.right = point, split_dim, None, None


def kd_tree(points, depth=0):
    if not points:
        return None
    k = len(points[0])
    points.sort(key=lambda x: x[depth % k])
    median = len(points) // 2
    node = Node(points[median], depth % k)
    node.left = kd_tree(points[:median], depth + 1)
    node.right = kd_tree(points[median + 1:], depth + 1)
    return node


def k_nearest_neighbors(root, query_point, k):
    pq = PriorityQueue()

    def search(node):
        if not node: return
        dist = -np.linalg.norm(np.array(query_point) - np.array(node.point))
        if len(pq.queue) < k or dist > pq.queue[0][0]:
            if len(pq.queue) == k: pq.get()
            pq.put((dist, node.point))
        next_branches = [node.left, node.right] if query_point[node.split_dim] <= node.point[node.split_dim] else [
            node.right, node.left]
        search(next_branches[0])
        if len(pq.queue) < k or abs(query_point[node.split_dim] - node.point[node.split_dim]) < -pq.queue[0][0]:
            search(next_branches[1])

    search(root)
    return [point for _, point in sorted(pq.queue, reverse=True)]


def find_distance(arm1, arm2):
    return sum(
        sqrt((arm1.joints[i][0] - arm2.joints[i][0]) ** 2 + (arm1.joints[i][1] - arm2.joints[i][1]) ** 2) for i in
        [2, 3])


def find_smallest_distances(configs, ax, arm, k):
    tree = kd_tree(configs.tolist())
    return k_nearest_neighbors(tree, (arm.theta1, arm.theta2), k)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Find two closest arm configurations to the target.")
    parser.add_argument('--configs', required=True)
    parser.add_argument('-k', required=True, type=int)
    parser.add_argument('--target', nargs=2, required=True, type=float)
    args = parser.parse_args()

    ax, configs = create_plot(), np.load(args.configs)
    planar_arm = Arm_Controller(*args.target, ax, [])
    planar_arm.set_obs_plot()
    planar_arm.add_arm('b')  # Original Target Arm
    nearest_points = find_smallest_distances(configs, ax, planar_arm, args.k)
    for count, (theta1, theta2) in enumerate(nearest_points):
        planar_arm.theta1, planar_arm.theta2 = theta1, theta2
        planar_arm.re_orient()
        planar_arm.add_arm('rgby'[count % 4])
    planar_arm.theta1, planar_arm.theta2 = args.target
    planar_arm.re_orient()
    planar_arm.add_arm('k')
    show_scene(ax)

from arm_5 import probabilistic_roadmap
from math import pi, degrees, sqrt
from create_scene import create_plot, load_polygons
import random, heapq
from rigid_body_2 import find_smallest_distances
from rigid_body import CarController, check_car, check_boundary
from rigid_body_3 import reposition_car, interpolate
from rigid_body_1 import make_rigid_body
from planar_arm import change_angles
import argparse
import matplotlib.pyplot as plt


def find_distance(car1, car2):
    alpha = 0.7
    linear_part = (car1[0] - car2[0]) ** 2 + (car1[1] - car2[1]) ** 2
    angular_part = (change_angles(car1[2]) - change_angles(car2[2])) ** 2
    return alpha * sqrt(linear_part) + (1 - alpha) * sqrt(angular_part)


# samples random configuration (x,y,theta)
def sizes():
    return random.random() * 2, random.random() * 2, random.uniform(-pi, pi)


# Just reordering so we can reuse code
def kneighbors(currConfig, otherConfigs, k, dist_fn=None):
    return find_smallest_distances(otherConfigs, currConfig, k)


# Assumes we already added the obstacles as an instance of the CarController obj
def check_collides(rigid_body: 'CarController', config):
    reposition_car(config, rigid_body)
    return not check_car(rigid_body.car, rigid_body.obstacles) or not check_boundary(rigid_body.car)


def animate(config, edges, iters, ax):
    x1, y1, _ = config
    plt.scatter(x1, y1, c='g')
    for edge in edges:
        x2, y2, _ = edge
        plt.plot([x1, x2], [y1, y2], c='g')
    if iters % 50 == 0:
        ax.figure.canvas.draw()
        plt.pause(1e-6)


# Almost the same A star as we have in arm 5 with a small tweak for distance function
def Astar_algo(startConfig, goalConfig, Graph, dist_fn):
    def get_path(config):
        path = []
        while config:
            path.append(config)
            config = parents[config]
        return path[::-1]

    distances = {startConfig: 0}
    parents = {startConfig: None}
    fringe = [(dist_fn(startConfig, goalConfig), startConfig)]  # priority queue
    while fringe:
        _, curr_config = heapq.heappop(fringe)
        if curr_config == goalConfig:
            return True, get_path(curr_config)

        for child in Graph[curr_config].edges:
            tmpDist = distances[curr_config] + dist_fn(curr_config, child)
            if child not in distances or tmpDist < distances[child]:
                distances[child] = tmpDist
                parents[child] = curr_config
                heapq.heappush(fringe, (tmpDist + dist_fn(child, goalConfig), child))
    return False, []


def depthFirstSearch(graph, src, goal, visited=None, path=None):
    if visited is None:
        visited = set()
    if path is None:
        path = []
    path.append(src)
    visited.add(src)
    if src == goal:
        return path
    for neighbor in graph[src].edges:
        if neighbor not in visited:
            new_path = depthFirstSearch(graph, neighbor, goal, visited, path.copy())
            if new_path:
                return new_path
    return None


def main():
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(description="Finds collision-free path between two configs using PRM")
    parser.add_argument('--start', type=float, nargs=3, required=True, help='Start orientation')
    parser.add_argument('--map', required=True, help='Path to map file')
    parser.add_argument('--goal', type=float, nargs=3, required=True, help='Target orientation')
    args = parser.parse_args()

    # Load map and initialize car controllers
    poly_map = load_polygons(args.map)
    start_config, goal_config = tuple(args.start), tuple(args.goal)
    rig_body = CarController(ax=create_plot(), car=make_rigid_body(start_config[:2]), obstacles=poly_map)
    rig_body.car.set_angle(degrees(start_config[2]))

    # Generate PRM graph
    graph = probabilistic_roadmap(100, kneighbors, 3, sizes, rig_body, check_collides, find_distance, interpolate,
                                  start_config, goal_config, (0, 2), 0.05, animate)
    print('PRM finished')
    plt.close()

    # Search for a path using A*
    var, path = Astar_algo(start_config, goal_config, graph, find_distance)
    if var:
        # If a path is found, visualize it
        all_points = [point for i in range(len(path) - 1) for point in graph[path[i]].roads[path[i + 1]]]
        for pt in all_points:
            reposition_car(pt, rig_body)
            rig_body.ax.cla()
            rig_body.ax.set_ylim([0, 2])
            rig_body.ax.set_xlim([0, 2])
            rig_body.ax.add_patch(rig_body.car)
            rig_body.set_obstacles(rig_body.obstacles)
            plt.draw()
            plt.pause(1e-4)
            rig_body.ax.figure.canvas.draw()
    else:
        print('No path exists')

    print('Finished')


if __name__ == '__main__':
    main()

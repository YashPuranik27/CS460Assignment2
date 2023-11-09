import numpy as np
import matplotlib.pyplot as plt
import random
from scipy.spatial import ConvexHull


def make_polygons(p, n_min, n_max, r_min, r_max, xdim=2, ydim=2):
    polygons = []
    for _ in range(p):
        x, y = random.uniform(0, xdim), random.uniform(0, ydim)
        num_vertices = random.randint(n_min, n_max)
        vertices = [[x + radius * np.cos(angle), y + radius * np.sin(angle)]
                    for _ in range(num_vertices)
                    for radius in [random.uniform(r_min, r_max)]
                    for angle in [random.uniform(0, 2 * np.pi)]]
        hull = ConvexHull(np.array(vertices))
        polygons.append(vertices[hull.vertices])
    return np.array(polygons, dtype=object)


def add_polygon_to_scene(polygon, ax, fill):
    pol = plt.Polygon(polygon, closed=True, fill=fill, color='black', alpha=0.4)
    ax.add_patch(pol)


def create_plot():
    fig, ax = plt.subplots(dpi=100)
    return ax


def show_scene(ax):
    ax.set_xlim(0, 2)
    ax.set_ylim(0, 2)
    ax.set_aspect('equal')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.show()


def save_polygons(polygons, filename):
    np.save(filename, arr=polygons, allow_pickle=True)


def load_polygons(filename):
    return np.load(filename, allow_pickle=True)


if __name__ == '__main__':
    ax = create_plot()
    polygons = make_polygons(2, 25, 50, 0.3, 0.6)
    for p in polygons:
        add_polygon_to_scene(p, ax, 'b')
    save_polygons(polygons, 'ex4.npy')
    show_scene(ax)

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


"""
def construct_name_inital_polygons():

    #### letter Y
    name_polygons = []
    name_poly=[]
    name_poly.append([4.09/15, 14.3/15])
    name_poly.append([2.37/15, 13.14/15])
    name_poly.append([6.71/15, 6.78/15])
    name_poly.append([8.37/15, 8.46/15])
    name_polygons.append(name_poly)

    name_poly=[]
    name_poly.append([13.25/15, 13.78/15])
    name_poly.append([15.15/15, 12.26/15])
    name_poly.append([9.67/15, 6.62/15])
    name_poly.append([8.37/15, 8.46/15])
    name_polygons.append(name_poly)

    name_poly=[]
    name_poly.append([6.71/15, 6.78/15])
    name_poly.append([8.37/15, 8.46/15])
    name_poly.append([9.67/15, 6.62/15])
    name_poly.append([9.53/15, 0])
    name_poly.append([6.55/15, 0])
    name_polygons.append(name_poly)

    ### letter A
    name_poly = []
    name_poly.append([18.53/15, 6.96/15])
    name_poly.append([21.08/15, 7.02/15])
    name_poly.append([21.62/15, 5.04/15])
    name_poly.append([17.85/15, 5.09/15])
    name_polygons.append(name_poly)

    name_poly = []
    name_poly.append([18.55/15, 13.62/15])
    name_poly.append([19.91/15, 13.62/15])
    name_poly.append([20.0/15, 11.0/15])
    name_poly.append([16.0/15, 0.0])
    name_poly.append([13.73/15, 0.0])
    name_polygons.append(name_poly)

    name_poly = []
    name_poly.append([25.0/15, 0.0])
    name_poly.append([23.0/15, 0.0])
    name_poly.append([20.0/15, 11.0/15])
    name_poly.append([19.91/15, 13.62/15])
    name_poly.append([21.49/15, 13.68/15])

    name_polygons.append(name_poly)


    save_polygons_to_file(name_polygons, "name_initials.npy")
"""


def show_scene(ax):
    ax.set_xlim(0, 2)
    ax.set_ylim(0, 2)
    ax.set_aspect('equal')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.show()


def save_polygons_to_file(polygons, filename):
    # Convert list of polygons to a list of np arrays
    np_polygons = [np.array(polygon) for polygon in polygons]
    # Combine all polygons into a single numpy array and save as .npy
    np.save(filename, np.array(np_polygons, dtype=object))


def load_polygons(filename):
    return np.load(filename,allow_pickle=True)


def main():
    P = int(input("Enter the total number of polygons in the scene (P): "))
    N_min = int(input("Enter the minimum number of vertices (N_min): "))
    N_max = int(input("Enter the maximum number of vertices (N_max): "))
    r_min = float(input("Enter the minimum radius of the polygon (r_min): "))
    r_max = float(input("Enter the maximum radius of the polygon (r_max): "))
    filename = input(
        "Enter the filename to save/load polygons (without extension): ") + '.npy'  # Note the file extension change

    polygons = make_polygons(P, N_min, N_max, r_min, r_max)
    print("...... polygons", polygons)
    save_polygons_to_file(polygons, filename)

    loaded_polygons = load_polygons(filename)

    for polygon in loaded_polygons:
        # print("Generated Polygon:", polygon)
        print(repr(polygon), end=' ')
    print()

    # visualization of the name_initial letters
    # construct_name_inital_polygons()

    name_initial_polygons = load_polygons("name_initials.npy")
    # plot_name_initials(name_initial_polygons)

    plt.show()


if __name__ == '__main__':
    ax = create_plot()
    polygons = make_polygons(2, 25, 50, 0.3, 0.6)
    for p in polygons:
        add_polygon_to_scene(p, ax, 'b')
    save_polygons_to_file(polygons, 'ex4.npy')
    show_scene(ax)

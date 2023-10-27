# Original Code from assignment 1

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import timeit


# read polygon info from file
def load_polygons_from_file(filename):
    data = np.load(filename, allow_pickle=True)
    return [data[i] for i in range(len(data))]


def get_occupied_cells(polygon, grid_size):
    # Return cells occupied by the polygon
    min_coords = np.min(polygon, axis=0)
    max_coords = np.max(polygon, axis=0)

    min_cell = np.floor(min_coords / grid_size).astype(int)
    max_cell = np.floor(max_coords / grid_size).astype(int)

    occupied_cells = []
    for i in range(min_cell[0], max_cell[0] + 1):
        for j in range(min_cell[1], max_cell[1] + 1):
            occupied_cells.append((i, j))

    return occupied_cells


def collides_Spatial_Detection(polygons, polygon_state):
    # Grid configuration
    grid_size = 0.1  # Size of each grid cell

    # Map polygons to grid cells
    grid = {}
    for idx, polygon in enumerate(polygons):
        for cell in get_occupied_cells(polygon, grid_size):
            if cell not in grid:
                grid[cell] = []
            grid[cell].append(idx)

    # Check for collisions based on grid occupancy
    # colliding_pairs = set()
    for cell, occupants in grid.items():
        if len(occupants) > 1:
            for i in range(len(occupants)):
                for j in range(i + 1, len(occupants)):
                    # colliding_pairs.add((occupants[i], occupants[j]))
                    polygon_state[occupants[i]] = True
                    polygon_state[occupants[j]] = True

    # print("colliding pairs:", colliding_pairs)


def is_separating_axis(poly1, poly2, polya, polyb):
    """Checks if line perpendicular to the line segment (polya, polyb) is a separating axis"""
    perp = np.array([polya[1] - polyb[1], polyb[0] - polya[0]])

    # normalize
    # perp /= np.linalg.norm(perp)

    mina, maxa = float("inf"), float("-inf")
    minb, maxb = float("inf"), float("-inf")

    for vert in poly1:
        proj = np.dot(vert, perp)
        mina = min(mina, proj)
        maxa = max(maxa, proj)

    for vert in poly2:
        proj = np.dot(vert, perp)
        minb = min(minb, proj)
        maxb = max(maxb, proj)

    return maxa < minb or maxb < mina


# Checks if polygons OA and OB collide using the Separating Axis Theorem"
def collides_SAT(poly1, poly2):
    for i in range(len(poly1)):
        if is_separating_axis(poly1, poly2, poly1[i], poly1[(i + 1) % len(poly1)]):
            return False
    for i in range(len(poly2)):
        if is_separating_axis(poly1, poly2, poly2[i], poly2[(i + 1) % len(poly2)]):
            return False
    return True


def collides_bounding_box(polygon1, polygon2):
    # Calculate bounding boxes

    print(polygon1)
    min1, max1 = np.min(polygon1, axis=0), np.max(polygon1, axis=0)
    min2, max2 = np.min(polygon2, axis=0), np.max(polygon2, axis=0)

    # Check if bounding boxes collide
    collide_x = min1[0] < max2[0] and max1[0] > min2[0]
    collide_y = min1[1] < max2[1] and max1[1] > min2[1]
    result = collide_x and collide_y

    return result


def collides_Spatial_Detection_2Pass(polygons, polygon_state):
    # Grid configuration
    grid_size = 0.1  # Size of each grid cell

    # Map polygons to grid cells
    grid = {}
    for idx, polygon in enumerate(polygons):
        if polygon_state[idx] == True:
            polygon_state[idx] = False  # reset the state to False
            for cell in get_occupied_cells(polygon, grid_size):
                if cell not in grid:
                    grid[cell] = []
                grid[cell].append(idx)

    # Check for collisions based on grid occupancy
    # colliding_pairs = set()
    for cell, occupants in grid.items():
        if len(occupants) > 1:
            for i in range(len(occupants)):
                for j in range(i + 1, len(occupants)):
                    # colliding_pairs.add((occupants[i], occupants[j]))
                    polygon_state[occupants[i]] = True
                    polygon_state[occupants[j]] = True

    # print("colliding pairs:", colliding_pairs)


def plot(polygons, polygons_state):
    # Visualization
    fig, ax = plt.subplots(dpi=100)
    ax.set_aspect('equal')

    # Draw polygons
    i = 0
    for polygon in polygons:
        # print("Generated Polygon:", polygon)
        if (polygons_state[i] == True):
            # fill the polygon
            ax.fill(polygon[:, 0], polygon[:, 1], 'r', alpha=0.5)
        else:
            # only draw the border
            # ax.fill(polygon[:, 0], polygon[:, 1], 'b', alpha=0.5)
            polygon = np.concatenate((polygon, [polygon[0]]), axis=0)  # close the polygon
            xs, ys = zip(*polygon)
            plt.plot(xs, ys)
        i = i + 1

    plt.show()


def plot_old(polygon1, polygon2, check_result):
    # Visualization
    fig, ax = plt.subplots(dpi=100)
    ax.set_aspect('equal')

    # Draw polygons
    ax.fill(polygon1[:, 0], polygon1[:, 1], 'b', label='Polygon 1', alpha=0.5)
    ax.fill(polygon2[:, 0], polygon2[:, 1], 'r', label='Polygon 2', alpha=0.5)

    # Calculate bounding boxes
    min1, max1 = np.min(polygon1, axis=0), np.max(polygon1, axis=0)
    min2, max2 = np.min(polygon2, axis=0), np.max(polygon2, axis=0)

    # Draw bounding boxes
    rect1 = patches.Rectangle(min1, max1[0] - min1[0], max1[1] - min1[1], linewidth=1, edgecolor='b', facecolor='none')
    rect2 = patches.Rectangle(min2, max2[0] - min2[0], max2[1] - min2[1], linewidth=1, edgecolor='r', facecolor='none')

    ax.add_patch(rect1)
    ax.add_patch(rect2)

    ax.legend()

    if check_result:
        ax.set_title("Bounding boxes collide")
    else:
        ax.set_title("Bounding boxes don't collide")
    plt.show()


def main():
    polygons_state = []

    polygons = load_polygons_from_file(r'collision_checking_polygons.npy')
    for polygon in polygons:
        # print("Generated Polygon:", polygon)
        print(repr(polygon), end=' ')
        polygons_state.append(False)

    # print("num of poly "+str(polygon.ndim))

    # poly1 = polygons[0]
    # poly2 = polygons[1]

    start = timeit.default_timer()

    # fist pass bounding box checking
    for i in range(len(polygons)):
        poly1 = polygons[i]
        print("poly1 -- " + str(i))
        for j in range(i + 1, len(polygons)):
            poly2 = polygons[j]
            print("poly2 -- " + str(j))
            # check_result = collides_bounding_box(poly1, poly2)

            check_result = collides_SAT(poly1, poly2)
            # plot(poly1, poly2, check_result)

            if check_result == True:
                polygons_state[i] = True
                polygons_state[j] = True

    '''
    # second pass Spatial Partition

    polygons_pair=[]
    collides_Spatial_Detection_2Pass(polygons, polygons_state)'''

    # All the program statements
    stop = timeit.default_timer()
    execution_time = stop - start

    print("=========\n Program Executed in ===========\n" + str(execution_time))  # It returns time in seconds

    '''
    for i in range(len(polygons)):
        print("poly state  " + str(i) + str(polygons_state[i]))
    '''
    plot(polygons, polygons_state)


if __name__ == "__main__":
    main()

# Define two polygons
# polygon1 = np.array([[2, 3], [3, 8], [5, 6], [6, 4], [4, 2]])
# polygon2 = np.array([[5, 5], [7, 8], [9, 6], [8, 4]])


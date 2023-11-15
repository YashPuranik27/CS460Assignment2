# done
import argparse
from math import degrees
import random
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
import matplotlib.patches as patches
from rigid_body import check_car


def make_rigid_body(center, angle=0, opacity=1):
    return patches.Rectangle(
        (center[0] - .1, center[1] - .05), .2, .1, linewidth=1,
        angle=degrees(angle), rotation_point='center',
        edgecolor='r', facecolor='red', alpha=opacity
    )


def check_car_spawn(obstacles):
    while True:
        car = make_rigid_body((random.uniform(0, 1), random.uniform(0, 1)))
        print(car.get_xy())
        if check_car(car, obstacles):
            break
    return car


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    args = parser.parse_args()

    poly_map = load_polygons(args.map)
    ax = create_plot()
    [add_polygon_to_scene(poly, ax, True) for poly in poly_map]

    car = check_car_spawn(poly_map)
    ax.add_patch(car)
    show_scene(ax)


if __name__ == '__main__':
    main()

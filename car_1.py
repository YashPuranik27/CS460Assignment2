import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D
import sys, getopt
import argparse
from planar_car import PlanarCar

# Initial state [x, y, theta]
init_x = 1
init_y = 1
init_q = np.array([init_x, init_y, 0.0])
q = init_q

# Control input [v, omega]
init_v = 0.4
init_o = 0.5
u = np.array([init_v, init_o])

# Robot dimensions
length = 0.2
width = 0.1

# Time step
dt = 0.1

# Control limits
v_max = 0.5
v_min = -0.5
omega_max = np.pi / 4
omega_min = omega_max*-1

tireAngle=0

tireOffest=0.5



def main(argv):

    global q
    global u
    init_v = 0.4
    init_o = 0.5
    u = np.array([init_v, init_o])

    parser = argparse.ArgumentParser(description="Find robot moving in a free workspace.")
    parser.add_argument('--control', type=float, nargs=2, required=True, help='constant control -- v theta')
    args = parser.parse_args()
    print(args)

    init_v=args.control[0]
    init_o=args.control[1]

    print("initial x = " + str(init_x) + " y = " + str(init_y) + "\n")

    # Initialize plot
    fig, ax = plt.subplots(figsize=(6, 6))
    #fig.canvas.mpl_connect('key_press_event', on_key)

    omega_step = np.pi / 15
    v_step = 0.1
    u = np.array([init_v, init_o])
    tireAngle = np.pi / 30


    x_max = 2
    y_max = 2

    car = PlanarCar()

    while True:
        # Update state
        dq = car.differential_drive_model(q, u)
        q += dq

        # Visualization
        plt.clf()
        ax = plt.gca()
        plt.xlim(0, x_max)
        plt.ylim(0, y_max)

        if (q[0]<x_max and q[1]<y_max and q[0]>0 and q[1]>0 ):
            # Draw Front Tire
            car.draw_rotated_tire(ax, [q[0], q[1]], length, width, np.degrees(q[2] + tireAngle))

            # Draw robot body
            car.draw_rotated_rectangle(ax, [q[0], q[1]], length, width, np.degrees(q[2]))



            plt.pause(0.05)
        else:
            #move the car to initial state
            u = np.array([init_v, init_o])
            q = init_q


if __name__ == "__main__":
    main(sys.argv[1:])

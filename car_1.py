import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D
import sys, getopt

# Initial state [x, y, theta]
q = np.array([1.0, 1.0, 0.0])

# Control input [v, omega]
u = np.array([0.0, 0.0])

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

def differential_drive_model(q, u):
    dq = np.zeros_like(q)
    dq[0] = u[0] * np.cos(q[2]) * dt
    dq[1] = u[0] * np.sin(q[2]) * dt
    dq[2] = u[1] * dt
    return dq


def on_key(event):
    global u
    global tireAngle

    omega_step = np.pi/50


    if event.key == 'up':
        u[0] = np.clip(u[0] + 0.1, v_min, v_max)
        #tireAngle = 0
    elif event.key == 'down':
        u[0] = np.clip(u[0] - 0.1, v_min, v_max)
        #tireAngle = 0
    elif event.key == 'right':
        u[1] = np.clip(u[1] + omega_step, omega_min, omega_max)
        if u[1] > 0:
            tireAngle = np.pi / 30
        else:
            tireAngle = np.pi / 30 * -1
    elif event.key == 'left':
        u[1] = np.clip(u[1] - omega_step, omega_min, omega_max)
        if u[1] > 0:
            tireAngle = np.pi / 30
        else:
            tireAngle = np.pi / 30 * -1


def draw_rotated_tire(ax, center, width, height, angle_degrees, color='b'):

    x, y = center
    t = Affine2D().rotate_deg_around(x, y, angle_degrees) + ax.transData

    xF1 = x + (0.3 * width)
    yF1 = y + (0.3 * height)
    tireFront = patches.Rectangle((xF1 - 0.025, yF1 - 0.01), 0.05, 0.02, linewidth=1, edgecolor='red',
                                  facecolor='red')
    tireFront.set_transform(t)
    ax.add_patch(tireFront)

    xF2 = x + (0.3 * width)
    yF2 = y - (0.3 * height)
    tireFront = patches.Rectangle((xF2 - 0.025, yF2 - 0.01), 0.05, 0.02, linewidth=1, edgecolor='red',
                                  facecolor='red')
    tireFront.set_transform(t)
    ax.add_patch(tireFront)


def draw_rotated_rectangle(ax, center, width, height, angle_degrees, color='b'):

    global tireAngle


    x, y = center
    rect = patches.Rectangle((x - width / 2, y - height / 2), width, height, linewidth=1, edgecolor=color,
                             facecolor='none')
    t = Affine2D().rotate_deg_around(x, y, angle_degrees) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)

    # draw Rear Tire
    xR1 = x - (0.3 * width)
    yR1 = y + (0.3 * height)
    tireRear = patches.Rectangle((xR1 - 0.025, yR1 - 0.01), 0.05, 0.02, linewidth=1, edgecolor='red',
                                 facecolor='none')
    tireRear.set_transform(t)
    ax.add_patch(tireRear)

    xR2 = x - (0.3 * width)
    yR2 = y - (0.3 * height)
    tireRear = patches.Rectangle((xR2 - 0.025, yR2 - 0.01), 0.05, 0.02, linewidth=1, edgecolor='red',
                                 facecolor='none')
    tireRear.set_transform(t)
    ax.add_patch(tireRear)



#read input control


def main(argv):

    global q

    # Initialize plot
    fig, ax = plt.subplots(figsize=(6, 6))
    fig.canvas.mpl_connect('key_press_event', on_key)

    while True:
        # Update state
        dq = differential_drive_model(q, u)
        q += dq

        # Visualization
        plt.clf()
        ax = plt.gca()
        plt.xlim(0, 2)
        plt.ylim(0, 2)

        # Draw Front Tire
        draw_rotated_tire(ax, [q[0], q[1]], length, width, np.degrees(q[2] + tireAngle))

        # Draw robot body
        draw_rotated_rectangle(ax, [q[0], q[1]], length, width, np.degrees(q[2]))

        plt.pause(0.05)


if __name__ == "__main__":
    main(sys.argv[1:])




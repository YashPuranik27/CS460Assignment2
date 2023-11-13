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

tireOffest=0.5

def differential_drive_model(q, u):
    dq = np.zeros_like(q)

    # q[2] is theta(t)
    # u[0] is velocity
    # u[1] is steeringAngle

    dq[0] = u[0] * np.cos(q[2]) * dt # v * cos(theta(t)
    dq[1] = u[0] * np.sin(q[2]) * dt # v * sin(theta(t)
    dq[2] = u[0]/length * np.tan(u[1]) * dt  # v/L*tan(steeringAngle)

    return dq


def on_key(event):
    global u
    global tireAngle

    omega_step = np.pi/30
    v_step = 0.05


    if event.key == 'up':
        u[0] = np.clip(u[0] + v_step, v_min, v_max)
        #tireAngle = 0

    elif event.key == 'down':
        u[0] = np.clip(u[0] - v_step, v_min, v_max)
        #tireAngle = 0

    elif event.key == 'right':

        if u[1] < 0 : #car was steering left, turn it
            u[1] = 0
            tireAngle = np.pi / 30 * -1
        else:
            u[1] = np.clip(u[1] + omega_step, omega_min, omega_max)
            tireAngle = np.pi / 30

        #if u[0] == 0:
        #    u[0] = np.clip(u[0] + v_step, v_min, v_max) # if the car stopped, move it

    elif event.key == 'left':
        u[1] = np.clip(u[1] - omega_step, omega_min, omega_max)
        if u[1] > 0:
            u[1] = 0
            tireAngle = np.pi / 30
        else:
            u[1] = np.clip(u[1] - omega_step, omega_min, omega_max)
            tireAngle = np.pi / 30 * -1

        #if u[0] == 0:
        #    u[0] = np.clip(u[0] - v_step, v_min, v_max) # if the car stopped, move it


def draw_rotated_tire(ax, center, width, height, angle_degrees, color='b'):

    x, y = center
    t = Affine2D().rotate_deg_around(x, y, angle_degrees) + ax.transData

    xF1 = x + (tireOffest * width)
    yF1 = y + (tireOffest * height)
    tireFront = patches.Rectangle((xF1 - 0.025, yF1 - 0.01), 0.05, 0.02, linewidth=1, edgecolor='red',
                                  facecolor='red')
    tireFront.set_transform(t)
    ax.add_patch(tireFront)

    xF2 = x + (tireOffest * width)
    yF2 = y - (tireOffest * height)
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
    xR1 = x - (tireOffest * width)
    yR1 = y + (tireOffest * height)
    tireRear = patches.Rectangle((xR1 - 0.025, yR1 - 0.01), 0.05, 0.02, linewidth=1, edgecolor='red',
                                 facecolor='none')
    tireRear.set_transform(t)
    ax.add_patch(tireRear)

    xR2 = x - (tireOffest * width)
    yR2 = y - (tireOffest * height)
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

    #initial theta = 0
    #initial velocity = 0
    u[0] = 0
    u[1] = 0

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




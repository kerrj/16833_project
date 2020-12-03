import numpy as np
import math
import matplotlib.pyplot as plt
import time

fig = plt.figure()
ax = plt.axes()
logname = "../data/16833_log_pitt_house.txt"
plt.axis("equal")
final_sol = False
#this flag indicates to use the final_output file instead of output
#The difference is that final_output contains smoothed poses, where output
# contains pose values during the real-time execution


def visualize_line(r, rad_theta, xrange=(-15, 15), yrange=(-15, 15), **kwargs):
    x = np.linspace(xrange[0], xrange[1], 50)
    m = -np.cos(rad_theta) / np.sin(rad_theta)
    b = r / np.sin(rad_theta)
    y = m * x + b
    ax.plot(x[(y < yrange[1]) & (y > yrange[0])], y[(y < yrange[1]) & (y > yrange[0])], **kwargs)


def visualize_scan(pts, frame):
    R = np.array(
        [
            [np.cos(frame[2, 0]), -np.sin(frame[2, 0])],
            [np.sin(frame[2, 0]), np.cos(frame[2, 0])],
        ]
    )
    T = frame[0:2, :]
    transpts = R.dot(pts.T) + T
    ax.plot(transpts[0, :], transpts[1, :], ".", color='k', markersize=0.75)


logf = open(logname)
if final_sol:
    resf = open("final_output.txt")
else:
    resf = open("output.txt")
iteration = 0
while True:
    iteration += 1
    print(f"_____ITERATION_({iteration})_____")

    # plot pose
    pose = resf.readline()[5:-2]
    pose = np.array([[float(s)] for s in pose.split(",")])
    plt.arrow(pose[0, 0], pose[1, 0], 0.4*math.cos(pose[2, 0]), 0.4*math.sin(pose[2, 0]), width=0.05, ec='r')

    # plot landmarks
    nlandmarks = resf.readline()
    nlandmarks = int(nlandmarks[nlandmarks.find(":") + 1 :])
    for i in range(nlandmarks):
        line = resf.readline()
        line = line[line.find("r,th=(") + 6 : -2]
        line = [float(s) for s in line.split(",")]  # r,th
        visualize_line(line[0], line[1], linewidth=1)
    print("# landmarks:", nlandmarks)

    # plot detected lines
    if not final_sol:
        nlines = resf.readline()
        nlines = int(nlines[nlines.find(":") + 1 :])
        for i in range(nlines):
            line = resf.readline()
            line = line[line.find("r,th=(") + 6 : -2]
            line = [float(s) for s in line.split(",")]  # r,th
            visualize_line(line[0], line[1], linewidth=1, alpha=0.4)
        print("# detected lines:", nlines)

    # plot scan
    while True:
        logline = logf.readline().strip().split(" ")
        if logline[1] == "S":
            break
    logline = logline[3:]
    # pts is an Nx2 matrix of (x,y) points in the robot frame
    pts = np.array(
        [
            [float(logline[2 * i]), float(logline[2 * i + 1])]
            for i in range(len(logline) // 2)
        ]
    )
    visualize_scan(pts, pose)

    plt.draw()
    ax.set_xlim((-10, 10))
    ax.set_ylim((-10, 10))
    plt.pause(0.001)
    # if plt.waitforbuttonpress(0): exit()
    plt.cla()

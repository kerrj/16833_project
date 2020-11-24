import numpy as np
import math
import matplotlib.pyplot as plt
import time

fig = plt.figure()
ax = plt.axes()
logname = "data/16833_log_upstairs.txt"
plt.axis("equal")


def visualize_line(r, rad_theta, xrange=(-10, 10), yrange=(-10, 10)):
    x = np.linspace(xrange[0], xrange[1], 50)
    m = -np.cos(rad_theta) / np.sin(rad_theta)
    b = r / np.sin(rad_theta)
    y = m * x + b
    ax.plot(x[(y < yrange[1]) & (y > yrange[0])], y[(y < yrange[1]) & (y > yrange[0])])


def visualize_scan(pts, frame):
    R = np.array(
        [
            [np.cos(frame[2, 0]), -np.sin(frame[2, 0])],
            [np.sin(frame[2, 0]), np.cos(frame[2, 0])],
        ]
    )
    T = frame[0:2, :]
    transpts = R.dot(pts.T) + T
    ax.plot(transpts[0, :], transpts[1, :], ".")


logf = open(logname)
resf = open("output.txt")
while True:
    pose = resf.readline()[5:-2]
    pose = np.array([[float(s)] for s in pose.split(",")])
    nlandmarks = resf.readline()
    nlandmarks = int(nlandmarks[nlandmarks.find(":") + 1 :])
    for i in range(nlandmarks):
        line = resf.readline()
        line = line[line.find("r,th=(") + 6 : -2]
        line = [float(s) for s in line.split(",")]  # r,th
        visualize_line(line[0], line[1])
    while True:
        logline = logf.readline().strip().split(" ")
        if logline[1] == "S":
            break
    logline = logline[3:]
    pts = np.array(
        [
            [float(logline[2 * i]), float(logline[2 * i + 1])]
            for i in range(len(logline) // 2)
        ]
    )
    # pts is an Nx2 matrix of (x,y) points in the robot frame
    visualize_scan(pts, pose)
    plt.draw()
    plt.pause(0.01)
    # plt.waitforbuttonpress(0)
    plt.cla()

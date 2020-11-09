import numpy as np
import math
import matplotlib.pyplot as plt

fig = plt.figure()
ax = plt.axes()
x = np.linspace(-1, 3)

def visualize_lines():
  f = open("lines.txt", "r")

  for l in f:
    r, deg_theta = l.split()
    rad_theta = float(deg_theta) * math.pi/180.0

    r = float(r)
    r /= 100.0

    m = -np.cos(rad_theta) / np.sin(rad_theta)
    b = r / np.sin(rad_theta)
    ax.plot(x, m*x + b)

def visScan(i):
  logfile=open('16833_log_pitt_house.txt')
  cnt=0
  while True:
    line=logfile.readline()
    line=line.split(" ")
    if line[1]=='S':
      cnt+=1
      if cnt==i:break
  num_pts=int(line[2])
  line=line[3:]
  xs=[]
  ys=[]
  for i in range(num_pts):
    xs.append(float(line[2*i]))
    ys.append(float(line[2*i+1]))
  ax.plot(xs,ys,'.')

visScan(1)
visualize_lines()
plt.show()

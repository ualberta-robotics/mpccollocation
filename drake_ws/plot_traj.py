import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib
import matplotlib.pyplot as plt
import re

try:
    # Python 2
    from future_builtins import filter
except ImportError:
    # Python 3
    pass


tf = []
with open('commanded position.txt', 'rb') as f:
    for line in f:
        t = re.findall("\d+\.\d+", line)
        b = np.array(t)
        b = np.transpose(b)
        tf = np.concatenate((tf, b))

rows = np.size(tf)/3
tf = np.reshape(tf, (rows,3))

x_comm = np.zeros(rows)
y_comm = np.zeros(rows)

for i in range(rows):
    x_comm[i] = tf[i][0]
    y_comm[i] = tf[i][1]


tf = []
with open('current position.txt', 'rb') as f:
    for line in f:
        t = re.findall("\d+\.\d+", line)
        b = np.array(t)
        b = np.transpose(b)
        tf = np.concatenate((tf, b))
rows = np.size(tf)/3
tf = np.reshape(tf, (rows,3))

x_curr = np.zeros(rows)
y_curr = np.zeros(rows)

for i in range(rows):
    x_curr[i] = tf[i][0]
    y_curr[i] = tf[i][1]

# PLOTTING TRAJECTORY AND STREAMLINES
aa = 0.2
bb = 0.6
cc = 0.5 
zz1 = 0.3
zz2 = 1.0
w = 0.6
Y, X = np.mgrid[-cc:w:100j, aa:bb:100j]

U = 0.0094 - 0.0468*X + 0.0667*Y - 0.0493 - 0.0226*X*X + 0.0417*X*Y + 0.3267*X + 0.0692*Y*Y - 0.0087*Y - 0.0079
V = 0.0839 - 0.0471*X - 0.1227*Y - 0.1645 - 0.0552*X*X + 0.0122*X*Y + 0.0350*X - 0.2174*Y*Y + 0.3898*Y - 0.0405

fig = plt.figure(figsize=(150, 150))
gs = gridspec.GridSpec(nrows=3, ncols=2, height_ratios=[6, 6, 6])

# Varying density along a streamline
ax0 = fig.add_subplot(gs[0, 0])

# ax0.streamplot(X, Y, U, V, density=[2, 2], color = 'orange')
# ax0.set_title('Streamlines', color = 'white')

plt.plot(x_comm,y_comm, color = 'green', linewidth=2.0)
plt.plot(x_curr,y_curr, color = 'red', linewidth=2.0)

ax0.streamplot(X, Y, U, V, density=[2, 2], color = 'orange')
ax0.set(xlabel='X Pos', ylabel='Y Pos', title='Red: Current Pos      Green: Commanded Pos')

plt.show()


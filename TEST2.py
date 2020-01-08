import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

fig = plt.figure()
ax = fig.gca(projection='3d')
p0 = np.array([2.0593152046203613, 0.820719301700592, 6.297834873199463])
p1 = np.array([1.1117547750473022, 1.0336993932724, 5.61042594909668])
t = 0.4684555
q0 = p1[0] - p0[0]
q1 = p1[1] - p0[1]
q2 = p1[2] - p0[2]
vx = q0 / t
vy = (q1 + 4.9 * (t ** 2)) / t
vz = q2 / t
# vy = 9.8 * t
print(vx, vy,vz)
x = []
y = []
z = []
for t1 in np.arange(0, 15, 0.01):
    x.append(p0[0] + vx * t1)
    y.append(p0[1] + vy * t1 - 4.9 * (t1 * t1))
    # y.append(a0[1] - 5 * (t1 * t1))
    z.append(p0[2] + vz * t1)
qqq = [x, y, z]
# print(qqq)
zzz = np.array(qqq)
# print(zzz[:,1])
# res = np.zeros((3,150))
# for i in range(0,150):
#     res[:,i]=np.dot(trans1,zzz[:,i])
# # res = np.dot(trans1,qqq)
for i in range(150):
    if -0.48 > zzz[1][i]:
        print(zzz[0][i], zzz[1][i], zzz[2][i], i)
        break
print(zzz[0], zzz[1], zzz[2])
ax.view_init(135, -90)
ax.grid(True)
ax.plot(zzz[0], zzz[1], zzz[2])
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-10, 10)
plt.ylabel("y")
plt.xlabel("x")
plt.show()

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpy as np

fig = plt.figure()
ax = fig.gca(projection='3d')
p0 = np.array([1.1107524633407593,1.0999507904052734,5.605597496032715])
p1 = np.array([0.8236871957778931,0.9067187905311584,5.173789978027344])
k = (p1[0] - p0[0]) / (p1[2] - p0[2])
cos = ((1 / (1 + k ** 2)) ** 0.5)
sin = k * cos
print(sin ** 2 + cos ** 2)
# p0 = np.array([23, -65, 647])
# p1 = np.array([18, -81, 600])
# p0 = np.array([200, 200, 190])
# p1 = np.array([20, 11, 19])
t = 0.46729
q0 = p1[0] - p0[0]
q1 = p1[1] - p0[1]
q2 = p1[2] - p0[2]
print(k)
# print(q0 / ((q0 ** 2 + q2 ** 2) ** 0.5))
# trans1 = np.array([[q0 / ((q0 ** 2 + q2 ** 2) ** 0.5), 0, -1 * q2 / ((q0 ** 2 + q2 ** 2) ** 0.5)],
#                    [0, 1, 0],
#                    [q2 / ((q0 ** 2 + q2 ** 2) ** 0.5), 0, q0 / ((q0 ** 2 + q2 ** 2) ** 0.5)]])
# trans0 = np.array([[q0 / ((q0 ** 2 + q2 ** 2) ** 0.5), 0, q2 / ((q0 ** 2 + q2 ** 2) ** 0.5)],
#                    [0, 1, 0],
#                    [-1 * q2 / ((q0 ** 2 + q2 ** 2) ** 0.5), 0, q0 / ((q0 ** 2 + q2 ** 2) ** 0.5)]])
trans1 = np.array([[sin, 0, -cos],
                   [0, 1, 0],
                   [cos, 0, sin]])
trans0 = np.array([[sin, 0, cos],
                   [0, 1, 0],
                   [-cos, 0, sin]])
a0 = np.dot(trans0, p0)
a1 = np.dot(trans0, p1)
print(a0, a1)
# tran0,tran1=tran(a0,a1)
# a0 = np.dot(tran0, a0)
# a1 = np.dot(tran0, a1)
# print(a0, a1)
b0 = a1[0] - a0[0]
b1 = a1[1] - a0[1]
vx = b0 / t
vy = (b1 + 4.9 * (t ** 2)) / t
# vy = 9.8 * t
print(vx, vy)
x = []
y = []
z = []
for t1 in np.arange(0, 15, 0.01):
    x.append(a0[0] + vx * t1)
    y.append(a0[1] + vy * t1 - 4.9 * (t1 * t1))
    # y.append(a0[1] - 5 * (t1 * t1))
    z.append(a0[2])
qqq = [x, y, z]
# print(qqq)
zzz = np.array(qqq)
res = np.zeros((3, 150))
# print(zzz[:,1])
# res = np.zeros((3,150))
# for i in range(0,150):
#     res[:,i]=np.dot(trans1,zzz[:,i])
# # res = np.dot(trans1,qqq)
res = np.dot(trans1, zzz)
for i in range(150):
    if -0.43 > res[1][i]:
        print(res[0][i], res[1][i], res[2][i], i)
        break
print(res[0], res[1], res[2])
ax.view_init(135, -90)
ax.grid(True)
ax.plot(res[0], res[1], res[2])
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-10, 10)
plt.ylabel("y")
plt.xlabel("x")
plt.show()

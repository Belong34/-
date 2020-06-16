import numpy as np
import math

num = 1
tran = 2
x0 = 1
x = 1.56
q = np.zeros(shape=[4, 1])
src = np.zeros(shape=[4, 1])
q[num] = tran
q[3] = 1
print(q)
rot = np.array([[math.cos(x), 0, math.sin(x)],
                [0, 1, 0],
                [-1 * math.sin(x), 0, math.cos(x)],
                [0, 0, 0]])
p = np.c_[rot, q]
src[num] = x0
src[3] = 1
result = np.dot(p, src)
print(p)
print(result)

# import pandas as pd
import numpy as np
# # #    x  y  z
# # v = np.array([1, 2, 3])
# # print(v)
#
# v = np.zeros(3)
# v[0] = 1
# v[1] = 2
# v[2] = 3
#
#
# #     x1 x2 x3  y1  y2 y3  z1  z2 z3
# x = [[1, 2, 4], [3, 9, 5], [5, 6, 7]]
# x = np.array(x)
#
# testtest = x.transpose()
# difference = testtest-v
#
# test = np.linalg.norm(difference, axis=0)
# print(test)
#
# pottest = [1, 2, 3]
#
# addition = pottest + test
# print(addition)
#
# vector_test = [0, 4, 5]
# print(np.linalg.norm(vector_test))
z = np.array([0.5, 5, 6])
z = np.ma.array(z, mask=(z==0.5))
d = 1/(z-0.5)
print(d)
e = 5+d[0]
print(e)
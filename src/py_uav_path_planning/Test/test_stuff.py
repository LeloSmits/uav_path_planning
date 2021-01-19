import pandas as pd
import numpy as np

hello = np.zeros(shape=(2,2))
print(hello)

bye = np.array([[0.85, 0.98]])
print(type(bye))



you = np.add(hello, bye)
print(you)

golf = you.astype(np.float32).tolist()
print(golf)



import pandas as pd
import numpy as np

flag_array = (np.genfromtxt('_2024-07-25 17:29:02.csv', delimiter=','))[:, 0]
print("success rate:", np.sum(flag_array)/len(flag_array))

import math
import numpy as np

goal_distance = round(math.hypot(0.6 - (-0.9), 0 - 0, 2), 2)
teste = round(np.linalg.norm(np.array([0.6, 0]) - np.array([-0.9, 0])),2)
print(goal_distance)
print(teste)
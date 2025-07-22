import numpy as np
import math
l = 0.3
d = 0.2
h = 0.1

I_b = 1/12 * np.array([[h**2 + d**2, 0, 0],[0, l**2 + h**2, 0] , [0, 0, l**2 + d**2]])

print(I_b)

acceleration = 4476 * math.pi/180
print(acceleration)
acceleration_vector = np.array([0, acceleration, 0])

torque = np.dot(I_b, acceleration_vector)
print(torque)
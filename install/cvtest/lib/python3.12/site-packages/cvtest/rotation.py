import numpy as np

Q = np.array([[3.0, 2.0, 1.0]])
rotation_about_X = 45 
translate_X_u = 1
translate_Y_u = 2
translate_Z_u = 3

rotation_matrix = np.array([
    [1, 0, 0],
    [0, np.cos(np.radians(rotation_about_X)), -np.sin(np.radians(rotation_about_X))],
    [0, np.sin(np.radians(rotation_about_X)), np.cos(np.radians(rotation_about_X))]
])
Q = Q @ rotation_matrix.T + np.array([[translate_X_u, translate_Y_u, translate_Z_u]])

print(Q)
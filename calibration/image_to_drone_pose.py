import numpy as np
from scipy.spatial.transform import Rotation

# Data: https://docs.google.com/document/d/1k8WualHEj0pRNi6kvquVwHG8lyX27G990YWq4Eg_IbQ/edit#heading=h.96o2q9vim5xm

# With respect to drone's coordinate frame
T_drone_to_camera = [0.03, 0.03, -0.12]
R_drone_to_camera = np.array([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1]
]).astype(float)

# With respect to camera's coordinate frame
T_camera_to_image = [0, 0, 0]
R_camera_to_image = Rotation.from_euler("XYZ", [-45, 0, 0], degrees=True).as_matrix()
np.savez("image_to_drone",
         T_drone_to_camera=T_drone_to_camera,
         R_drone_to_camera=R_drone_to_camera,
         T_camera_to_image=T_camera_to_image,
         R_camera_to_image=R_camera_to_image)

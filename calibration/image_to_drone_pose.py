import numpy as np
from scipy.spatial.transform import Rotation
import cv2


def store_transforms():
    # Data: https://docs.google.com/document/d/1k8WualHEj0pRNi6kvquVwHG8lyX27G990YWq4Eg_IbQ/edit#heading=h.96o2q9vim5xm
    # With respect to drone's coordinate frame
    T_drone_to_camera = np.array([0.03, 0.03, -0.12])
    R_drone_to_camera = np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]
    ]).astype(float)

    # With respect to camera's coordinate frame
    T_camera_to_image = np.array([0, 0, 0])
    R_camera_to_image = Rotation.from_euler("XYZ", [45, 0, 0], degrees=True).as_matrix()

    img = cv2.imread("check_image_to_drone.jpg")
    height, width, _ = img.shape

    np.savez("image_to_drone",
             T_drone_to_camera=T_drone_to_camera,
             R_drone_to_camera=R_drone_to_camera,
             T_camera_to_image=T_camera_to_image,
             R_camera_to_image=R_camera_to_image,
             height=height,
             width=width)


def test():
    extrinsic_data = np.load("../calibration/image_to_drone.npz", allow_pickle=True)
    T_drone_to_camera = extrinsic_data["T_drone_to_camera"]
    R_drone_to_camera = extrinsic_data["R_drone_to_camera"]
    T_camera_to_image = extrinsic_data["T_camera_to_image"]
    R_camera_to_image = extrinsic_data["R_camera_to_image"]
    height = extrinsic_data["height"]
    width = extrinsic_data["width"]
    cv_file = cv2.FileStorage("../calibration/camera_calibration.yaml", cv2.FILE_STORAGE_READ)
    K = cv_file.getNode("K").mat()
    K[0, -1] = width / 2
    K[1, -1] = height / 2

    T_world_to_camera = np.array([0.0, 0.0, -0.065])  # random guess

    # Get pixel location on image: https://yangcha.github.io/ImageViewer/
    # centerX, centerY = 1024, 34
    centerX, centerY = width / 2, 196  # 30mm
    pixel = np.array([centerX, centerY, 1])
    true_x = 0
    true_y = 100 * 1e-3  # mm, with respect to camera!
    true_z = 0  # TODO NOT SURE WHAT HEIGHT ARE DRONE's Legs
    # img = cv2.circle(img, (int(centerX), int(centerY)), 3, (255, 0, 0), 2)
    # cv2.imshow("image", img)
    # cv2.waitKey(-1)
    pixel_camera = R_camera_to_image.T @ np.linalg.inv(K) @ pixel

    # width, height, _ = img.shape
    # K[0, -1] = width
    # K[1, -1] = height

    # assume world has same orientation as drone, but different transform
    R_camera_to_drone = np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]
    ])
    pixel_world = R_camera_to_drone @ pixel_camera + T_world_to_camera
    import ipdb
    ipdb.set_trace()

    vec = pixel_world - T_world_to_camera
    vec /= np.linalg.norm(vec)

    # project ray onto ground plane
    ground_normal, ground_point = np.array([0, 0, 1]), np.array([0, 0, 0])
    t = (-ground_normal @ (T_world_to_camera - ground_point)) / (ground_normal @ vec)
    pred_3d_pos_raw = T_world_to_camera + t * vec
    print(pixel_world, pred_3d_pos_raw)

    # center: np.array([1325, 1254, 1])


store_transforms()
test()

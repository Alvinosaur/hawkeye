import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import scipy.linalg as lin
from scipy.spatial.transform import Rotation


def points_on_plane(x, y, p):
    normal = p[:3]
    d = p[3]
    # Solve for all z values given some arbitrary x, y
    # ax + by + cz + d = 0  ->  z = (-d - ax - by) / c
    z = (-d - normal[0] * x - normal[1] * y) / normal[2]
    return z


def visualize_plane(planes, center, img_plane_points, ground_plane_points):
    # Create mesh of plane
    xx, yy, = np.meshgrid(np.linspace(-5, 5, num=11),
                          np.linspace(-5, 5, num=11))

    plt3d = plt.figure().gca(projection='3d')
    for p in planes:
        z = points_on_plane(xx, yy, p)
        plt3d.plot_surface(xx, yy, z, alpha=0.2, label=np.array2string(p, precision=1))

    # Ensure that the next plot doesn't overwrite the first plot
    ax = plt.gca()
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(-5, 5)
    # plt.legend()

    ax.scatter(center[0], center[1], center[2], color='green')

    for point in img_plane_points:
        ax.scatter(point[0], point[1], point[2], color='red')

    for point in ground_plane_points:
        ax.scatter(point[0], point[1], point[2], color='blue')

    plt.show()


def affine_from_transform(R, T):
    aff_R = np.eye(4)
    aff_R[:3, :3] = R
    aff_T = np.eye(4)
    aff_T[:3, -1] = T
    aff_T[3, 3] = 1
    return aff_R @ aff_T


# original image plane
normal = np.array([0, 0, -1])
d = -2
p = np.append(normal, d)  # [a, b, c, d] of a plane
c = np.array([0, 0, 0])

# transform image by some rotation and translation into world frame
R = Rotation.from_euler('zyx', [0, 45, 0], degrees=True).as_matrix()
T = np.array([0, 0, 0])
affine = affine_from_transform(R=R, T=T)
p1 = np.linalg.inv(affine).T @ p
print(p)
print(p1)
# p1 = affine @ p
c1 = affine @ np.append(c, 1)
c1 = c1[:3] / c1[3]
img_point = np.array([0, 0, points_on_plane(x=0, y=0, p=p1)])

# camera P matrix
K = [[282.363047, 0., 0],
     [0., 280.10715905, 0],
     [0., 0., 1.]]
K = np.array(K)
P = K.dot(np.hstack((R, np.expand_dims(T, axis=1))))
invP = lin.pinv(P)  # pseudo inverse

# Ground plane and a point on the plane
ground_normal = np.array([0, 0, 1])
ground_d = 2
ground_p = np.append(ground_normal, ground_d)
ground_point = np.array([0, 0, points_on_plane(x=0, y=0, p=ground_p)])

img_plane_points = []
ground_plane_points = []
for x in range(0, 301, 100):
    for y in range(0, 301, 100):
        if x == 0 and y == 0: continue
        if x == 100: continue
        pixel = [x, y, 1.]  # relative to center, append z = 1
        pixel_world = np.dot(invP, pixel)
        if not np.isclose(pixel_world[3], 0):
            pixel_world = pixel_world[:3] / pixel_world[3]
        else:
            pixel_world = pixel_world[:3]
        vec = pixel_world - c1
        vec /= np.linalg.norm(vec)

        # intersect with image plane
        # t = (-p1[:3] @ (c1 - img_point)) / (p1[:3] @ vec)
        # img_plane_point = c1 + t * vec
        # img_plane_points.append(img_plane_point)

        # intersect with ground plane
        t = (-ground_normal @ (c1 - ground_point)) / (ground_normal @ vec)
        ground_plane_point = c1 + t * vec
        ground_plane_points.append(ground_plane_point)

visualize_plane(planes=[p], center=c, img_plane_points=[],
                ground_plane_points=[])

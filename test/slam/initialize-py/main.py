import sys

import cv2
import numpy as np
from numpy.linalg import svd
from numpy.linalg import inv
import matplotlib.pyplot as plt

from fundamental import get_RANSAC_fundamental_mat, get_fundamental_mat
from homography import get_RANSAC_homography_mat
from decomposition import get_essential_mat, decomposite_H, decomposite_E
# ref for reference
# cur for current

def orb_match(img_ref, img_cur, img_name):
    """match orb characteristic points"""
    MATCH_NUM = 200

    orb = cv2.ORB_create()
    keypoint1, descriptor1 = orb.detectAndCompute(img_ref, None)
    keypoint2, descriptor2 = orb.detectAndCompute(img_cur, None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(descriptor1, descriptor2)
    matches = sorted(matches, key=lambda x: x.distance)
    matches = matches[:MATCH_NUM]

    img_match = cv2.drawMatches(img_ref, keypoint1, img_cur, keypoint2, matches, None, flags=2)
    cv2.imwrite('./matches/'+img_name +'-match-'+str(MATCH_NUM)+'.jpg', img_match)

    ref_points = []
    cur_points = []
    for m in matches:
        ref_index = m.queryIdx
        cur_index = m.trainIdx
        ref_p = keypoint1[ref_index].pt
        ref_points.append(np.array(ref_p))
        cur_p = keypoint2[cur_index].pt
        cur_points.append(np.array(cur_p))
    ref_points = np.array(ref_points)
    cur_points = np.array(cur_points)
    
    return ref_points, cur_points, matches


def get_3d_points(R, t, K, ref_points, cur_points):
    # 3D points
    num_points = ref_points.shape[0]
    P_3ds = []
    colors = []
    for i in range(num_points):
        ref_p = np.concatenate([ref_points[i], np.array([1])]) # Point camera ref
        cur_p = np.concatenate([cur_points[i], np.array([1])]) # Point camera cur

        # calculate cur depth z_cur
        # $z_2 (K^{-1}P_{c1}) × (K^{-1}P_{c2}) = - (K^{-1}P_{c1}) × t$
        tmp_1 = np.cross(np.matmul(inv(K), ref_p), np.matmul(inv(K), cur_p))
        tmp_2 = - np.cross(np.matmul(inv(K), ref_p), t)
        z_cur = np.dot(tmp_1, tmp_2) / np.dot(tmp_1, tmp_1) # MSE solution

        # calculate ref depth z_ref
        # $z_1 (K^{-1}P_{c1}) = z_2 RK^{-1}P_{c2} + t$
        tmp_3 = np.matmul(inv(K), ref_p)
        tmp_4 = z_cur * np.matmul(R, np.matmul(inv(K), cur_p)) + t
        z_ref = np.dot(tmp_3, tmp_4) / np.dot(tmp_3, tmp_3) # MSE solution

        P_3d = np.matmul(inv(K), ref_p) * z_ref
        P_3ds.append(P_3d)
        camera_coordinate = ref_points[i].astype(np.int32) # [x, y]
        color = img_ref[camera_coordinate[1], camera_coordinate[0]]
        colors.append(color)
    P_3ds = np.array(P_3ds)
    colors = np.array(colors).astype(np.float64) / 255.0
    return P_3ds, colors

def show_3d_points(P_3ds, colors):
    ax = plt.axes(projection='3d')
    ax.scatter3D(P_3ds[:, 0], P_3ds[:, 1], P_3ds[:, 2], c=colors)
    SCALE = np.max(abs(P_3ds[:, :2]))
    STEP = SCALE / 5
    x_major_locator = plt.MultipleLocator(STEP)
    y_major_locator = plt.MultipleLocator(STEP)
    ax.xaxis.set_major_locator(x_major_locator)
    ax.yaxis.set_major_locator(y_major_locator)
    plt.xlim(-SCALE, SCALE)
    plt.ylim(-SCALE, SCALE)
    ax.set(title='Image', xlabel='x', ylabel='y', zlabel='z')
    plt.show()


if __name__ == "__main__":
    img_name = sys.argv[1]
    img_ref = cv2.imread('imgs/' + img_name + '_left.jpg') # [y, x, 3]
    img_cur = cv2.imread('imgs/' + img_name + '_right.jpg') # [y, x, 3]

    # orb point match
    ref_points, cur_points, matches = orb_match(img_ref, img_cur, img_name) # [x, y]
    
    # fundamental matrix
    F, score_F, inner_index_F = get_RANSAC_fundamental_mat(ref_points, cur_points)
    # homography matrix
    H, score_H, inner_index_H = get_RANSAC_homography_mat(ref_points, cur_points)
    print("Inner point number: fundamental {0}, homography {1}".format(
        len(inner_index_F), len(inner_index_H)))
    print("Score: fundamental {0}, homography {1}".format(score_F, score_H))
    # intrinsics parameters
    K = np.array(
        [[3.08226610e+03, 3.51400282e+00, 1.50807238e+03],
        [0.00000000e+00, 3.07619378e+03, 1.48854398e+03],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00],]
    )

    # choose a model
    R_H = score_H / (score_H + score_F)
    if R_H > 0.45:
        # select H
        print("Choose homography.")
        R, t = decomposite_H(H, K, ref_points, cur_points)
    else:
        # select F
        print("Choose fundamental.")
        E = get_essential_mat(F, K)
        R, t = decomposite_E(E, K, ref_points, cur_points)
    print("R = ", R)
    print("t = ", t)

    # 3d points
    P_3ds, colors = get_3d_points(R, t, K, ref_points, cur_points)
    show_3d_points(P_3ds, colors)
    
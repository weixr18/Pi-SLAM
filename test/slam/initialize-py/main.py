import cv2
import numpy as np
from numpy.linalg import lstsq
from numpy.linalg import svd

from numpy.linalg import inv
import matplotlib.pyplot as plt


def orb_match(img_left, img_right):
    """match orb characteristic points"""
    MATCH_NUM = 100

    orb = cv2.ORB_create()
    keypoint1, descriptor1 = orb.detectAndCompute(img_left, None)
    keypoint2, descriptor2 = orb.detectAndCompute(img_right, None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(descriptor1, descriptor2)
    matches = sorted(matches, key=lambda x: x.distance)
    matches = matches[:MATCH_NUM]

    img_match = cv2.drawMatches(img_left, keypoint1, img_right, keypoint2, matches, None, flags=2)
    cv2.imwrite('match-'+str(MATCH_NUM)+'.jpg', img_match)

    left_points = []
    right_points = []
    for m in matches:
        left_index = m.queryIdx
        right_index = m.trainIdx
        left_p = keypoint1[left_index].pt
        left_points.append(np.array(left_p))
        right_p = keypoint2[right_index].pt
        right_points.append(np.array(right_p))
    left_points = np.array(left_points)
    right_points = np.array(right_points)
    
    return left_points, right_points, matches

def get_fundamental_mat(left_points, right_points):
    """get fundamental matrix F"""
    num_points = left_points.shape[0]
    A = np.zeros([num_points, 8])
    b = - np.ones([num_points])
    A[:, 0] = left_points[:, 0] * right_points[:, 0]
    A[:, 1] = left_points[:, 0] * right_points[:, 1]
    A[:, 2] = left_points[:, 0]
    A[:, 3] = left_points[:, 1] * right_points[:, 0]
    A[:, 4] = left_points[:, 1] * right_points[:, 1]
    A[:, 5] = left_points[:, 1]
    A[:, 6] = right_points[:, 0]
    A[:, 7] = right_points[:, 1]

    x = lstsq(A, b, rcond=None)[0]
    F = np.zeros([3,3])
    F[0, :] = x[:3]
    F[1, :] = x[3:6]
    F[2, :2] = x[6:8]
    F[2, 2] = 1

    return F

def get_essential_mat(F, K):
    """get essential matrix E"""
    E = np.matmul(K.T, np.matmul(F, K))
    U, Sigma, VT = svd(E)
    sigma = (Sigma[0] + Sigma[1]) / 2
    Sigma_new = np.diag([sigma, sigma, 0])
    E_new = np.matmul(U, np.matmul(Sigma_new, VT))
    return E_new

def get_rotation_mat(E, K, left_points, right_points):
    """get rotation matrix R and t"""

    # SVD
    U, Sigma, VT = svd(E)
    MSigma = np.diag(Sigma)
    R_z_pos = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) # Matrix rotated 90 degrees around z-axis
    R_z_neg = inv(R_z_pos)

    # 4 possible solutions
    t_cross_0 = np.matmul(U, np.matmul(R_z_pos, np.matmul(MSigma, U.T)))
    R_0 = np.matmul(U, np.matmul(R_z_pos.T, VT))
    t_cross_1 = np.matmul(U, np.matmul(R_z_neg, np.matmul(MSigma, U.T)))
    R_1 = np.matmul(U, np.matmul(R_z_neg.T, VT))
    t_cross_2 = -t_cross_0
    R_2 = -R_0
    t_cross_3 = -t_cross_1
    R_3 = -R_1
    t_crosses = [t_cross_0, t_cross_1, t_cross_2, t_cross_3]
    Rs = [R_0, R_1, R_2, R_3]

    # select the one solution
    num_points = left_points.shape[0]
    negative_num = 999999
    R_select, t_select = 0, 0
    for i in range(4):
        R = Rs[i]
        t_cross = t_crosses[i]
        t = np.array([t_cross[2,1], t_cross[0,2], t_cross[1,0]])

        z_rights = []
        z_lefts = []
        for j in range(num_points):
            lp = np.concatenate([left_points[j], np.array([1])])
            rp = np.concatenate([right_points[j], np.array([1])])

            # calculate right depth z_right
            # $z_2 (K^{-1}P_{c1}) × (K^{-1}P_{c2}) = - (K^{-1}P_{c1}) × t$
            tmp_1 = np.cross(np.matmul(inv(K), lp), np.matmul(inv(K), rp))
            tmp_2 = - np.cross(np.matmul(inv(K), lp), t)
            z_right = np.dot(tmp_1, tmp_2) / np.dot(tmp_1, tmp_1) # MSE solution
            # z_right = tmp_2 / tmp_1
            z_rights.append(z_right)

            # calculate left depth z_left
            # $z_1 (K^{-1}P_{c1}) = z_2 RK^{-1}P_{c2} + t$
            tmp_3 = np.matmul(inv(K), lp)
            tmp_4 = z_right * np.matmul(R, np.matmul(inv(K), rp)) + t
            # tmp_4 = z_right.max() * np.matmul(R, np.matmul(inv(K), rp)) + t
            z_left = np.dot(tmp_3, tmp_4) / np.dot(tmp_3, tmp_3) # MSE solution
            # z_left = tmp_3 / tmp_4
            z_lefts.append(z_left)
        z_rights = np.array(z_rights)
        z_lefts = np.array(z_lefts)

        z_right_negative_num =  np.sum((z_rights < 0).astype(int))
        z_left_negative_num = np.sum((z_lefts < 0).astype(int))
        print("Round =", i, z_right_negative_num, z_left_negative_num)
        if (z_right_negative_num + z_left_negative_num < negative_num):
            negative_num = z_left_negative_num + z_right_negative_num
            R_select = R
            t_select = t
        pass
    
    return R_select, t_select

def get_3d_points(R, t, K, left_points, right_points):
    # 3D points
    num_points = left_points.shape[0]
    P_3ds = []
    colors = []
    for i in range(num_points):
        pcl = np.concatenate([left_points[i], np.array([1])]) # Point camera left
        pcr = np.concatenate([right_points[i], np.array([1])]) # Point camera right

        # calculate right depth z_right
        # $z_2 (K^{-1}P_{c1}) × (K^{-1}P_{c2}) = - (K^{-1}P_{c1}) × t$
        tmp_1 = np.cross(np.matmul(inv(K), pcl), np.matmul(inv(K), pcr))
        tmp_2 = - np.cross(np.matmul(inv(K), pcl), t)
        z_right = np.dot(tmp_1, tmp_2) / np.dot(tmp_1, tmp_1) # MSE solution

        # calculate left depth z_left
        # $z_1 (K^{-1}P_{c1}) = z_2 RK^{-1}P_{c2} + t$
        tmp_3 = np.matmul(inv(K), pcl)
        tmp_4 = z_right * np.matmul(R, np.matmul(inv(K), pcr)) + t
        z_left = np.dot(tmp_3, tmp_4) / np.dot(tmp_3, tmp_3) # MSE solution

        P_3d = np.matmul(inv(K), pcl) * z_left
        P_3ds.append(P_3d)
        camera_coordinate = left_points[i].astype(np.int32) # [x, y]
        color = img_left[camera_coordinate[1], camera_coordinate[0]]
        colors.append(color)
    P_3ds = np.array(P_3ds)
    colors = np.array(colors).astype(np.float64) / 255.0
    return P_3ds, colors

def show_3d_points(P_3ds, colors):
    ax = plt.axes(projection='3d')
    ax.scatter3D(P_3ds[:, 0], P_3ds[:, 1], P_3ds[:, 2], c=colors)
    SCALE = 5
    STEP = 1
    x_major_locator = plt.MultipleLocator(STEP)
    y_major_locator = plt.MultipleLocator(STEP)
    ax.xaxis.set_major_locator(x_major_locator)
    ax.yaxis.set_major_locator(y_major_locator)
    plt.xlim(-SCALE, SCALE)
    plt.ylim(-SCALE, SCALE)
    ax.set(title='Image', xlabel='x', ylabel='y', zlabel='z')
    plt.show()


if __name__ == "__main__":
    img_left = cv2.imread('imgs/6_left.jpg') # [y, x, 3]
    img_right = cv2.imread('imgs/6_right.jpg') # [y, x, 3]

    # orb point match
    left_points, right_points, matches = orb_match(img_left, img_right) # [x, y]
    
    # fundamental matrix
    F = get_fundamental_mat(left_points, right_points)
    # intrinsics parameters
    K = np.array(
        [[3.08226610e+03, 3.51400282e+00, 1.50807238e+03],
        [0.00000000e+00, 3.07619378e+03, 1.48854398e+03],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00],]
    )
    # essential matrix
    E = get_essential_mat(F, K)
    print("E = ", E)
    # gestures
    R, t = get_rotation_mat(E, K, left_points, right_points)
    print("R = ", R)
    print("t = ", t)

    # 3d points
    P_3ds, colors = get_3d_points(R, t, K, left_points, right_points)
    show_3d_points(P_3ds, colors)
    
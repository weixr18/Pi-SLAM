import numpy as np
from numpy.linalg import lstsq
from numpy.linalg import inv

from utils import THRESHOLD_H

def point_score_H(error):
    return np.max([0, THRESHOLD_H - error])

def get_homography_mat(ref_points, cur_points):
    """get fundamental matrix F"""

    # Direct Linear Transform
    num_points = ref_points.shape[0]
    A = np.zeros([num_points*2, 8])
    bb = np.zeros([num_points*2])
    A[:num_points, 0] = ref_points[:, 0] 
    A[:num_points, 1] = ref_points[:, 1]
    A[:num_points, 2] = 1
    A[:num_points, 6] = - ref_points[:, 0] * cur_points[:, 0]
    A[:num_points, 7] = - ref_points[:, 1] * cur_points[:, 0]
    bb[:num_points] = cur_points[:, 0]

    A[num_points:, 3] = ref_points[:, 0] 
    A[num_points:, 4] = ref_points[:, 1]
    A[num_points:, 5] = 1
    A[num_points:, 6] = - ref_points[:, 0] * cur_points[:, 1]
    A[num_points:, 7] = - ref_points[:, 1] * cur_points[:, 1]
    bb[num_points:] = cur_points[:, 1]

    if num_points > 4:
        x = lstsq(A, bb, rcond=None)[0]
    elif num_points == 4:
        x = np.matmul(inv(A), bb)
    else:
        print("Error: too few points for cumputing H.")
        return None
    
    H = np.zeros([3,3])
    H[0, :] = x[:3]
    H[1, :] = x[3:6]
    H[2, :2] = x[6:8]
    H[2, 2] = 1
    return H


def get_inner_points_and_score_H(H, ref_points, cur_points):
    num_points = ref_points.shape[0]
    inner_index = []
    score = 0
    for i in range(num_points):
        ref_p = np.concatenate([ref_points[i], np.array([1])]) # Point camera ref
        cur_p = np.concatenate([cur_points[i], np.array([1])]) # Point camera cur

        # reprojectrion error
        # import pdb;pdb.set_trace()
        ref_to_cur_p = np.matmul(H, ref_p)
        ref_to_cur_p = ref_to_cur_p / ref_to_cur_p[2]
        square_err_cur = np.sum((cur_p[:2] - ref_to_cur_p[:2])**2)
        rpj_err_cur = square_err_cur

        cur_to_ref_p = np.matmul(inv(H), cur_p)
        cur_to_ref_p = cur_to_ref_p / cur_to_ref_p[2]
        square_err_ref = np.sum((ref_p[:2] - cur_to_ref_p[:2])**2)
        rpj_err_ref = square_err_ref

        # inner judge
        if rpj_err_ref < THRESHOLD_H and rpj_err_cur < THRESHOLD_H:
            inner_index.append(i)
        # score
        score += point_score_H(rpj_err_ref)
        score += point_score_H(rpj_err_cur)
    
    return inner_index, score


def get_RANSAC_homography_mat(ref_points, cur_points):

    num_points = ref_points.shape[0]
    MAX_ITER = 200
    k = MAX_ITER
    i = 0
    num_inner_points = 0
    H = 0
    score = 0    
    while i < k:
        i += 1
        rand_4_index = np.random.choice(num_points, size=4, replace=False)
        sample_ref_points = ref_points[rand_4_index]
        sample_cur_points = cur_points[rand_4_index]

        # $cur_p = k H ref_p$
        cur_H = get_homography_mat(sample_ref_points, sample_cur_points)
        inner_index, cur_score = get_inner_points_and_score_H(cur_H, ref_points, cur_points)
        if len(inner_index) / num_points > 0.95:
            H = cur_H
            score = cur_score
            break
        if len(inner_index) > num_inner_points:
            num_inner_points = len(inner_index)
            H = cur_H
            score = cur_score
        if i > MAX_ITER:
            break
        omega = num_inner_points / num_points
        if num_inner_points > 0:
            k = np.log10(1 - 0.95) / np.log10(1 - np.power(omega, 4))
    return H, score, inner_index


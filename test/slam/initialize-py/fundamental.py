
import numpy as np
from numpy.linalg import inv, lstsq

from utils import THRESHOLD_F, THRESHOLD_H, EPSILON

def get_fundamental_mat(ref_points, cur_points):
    """get fundamental matrix F"""

    # epipolar geometry
    num_points = ref_points.shape[0]
    A = np.zeros([num_points, 8])
    bb = - np.ones([num_points])
    A[:, 0] = ref_points[:, 0] * cur_points[:, 0]
    A[:, 1] = ref_points[:, 0] * cur_points[:, 1]
    A[:, 2] = ref_points[:, 0]
    A[:, 3] = ref_points[:, 1] * cur_points[:, 0]
    A[:, 4] = ref_points[:, 1] * cur_points[:, 1]
    A[:, 5] = ref_points[:, 1]
    A[:, 6] = cur_points[:, 0]
    A[:, 7] = cur_points[:, 1]

    if num_points > 8:
        x = lstsq(A, bb, rcond=None)[0]
    elif num_points == 8:
        x = np.matmul(inv(A), bb)
    else:
        print("Error: too few points for cumputing F.")
        return None

    F = np.zeros([3,3])
    F[0, :] = x[:3]
    F[1, :] = x[3:6]
    F[2, :2] = x[6:8]
    F[2, 2] = 1
    return F

def point_score_F(error):
    if error > THRESHOLD_F:
        return 0
    else:
        return THRESHOLD_H - error


def get_inner_points_and_score_F(F, ref_points, cur_points):
    num_points = ref_points.shape[0]
    inner_index = []
    score = 0
    for i in range(num_points):
        ref_p = np.concatenate([ref_points[i], np.array([1])]) # Point camera ref
        cur_p = np.concatenate([cur_points[i], np.array([1])]) # Point camera cur

        # reprojectrion error
        cur_to_ref_p = np.matmul(F, cur_p)
        square_err_ref = (np.dot(cur_to_ref_p, ref_p))**2 / (np.dot(cur_to_ref_p[:2], cur_to_ref_p[:2]) + EPSILON) # ORB-SLAM
        rpj_err_ref = square_err_ref

        ref_to_cur_p = np.matmul(F.T, ref_p)
        square_err_cur = (np.dot(ref_to_cur_p, cur_p))**2 / (np.dot(ref_to_cur_p[:2], ref_to_cur_p[:2]) + EPSILON) # ORB-SLAM
        rpj_err_cur = square_err_cur

        # inner judge
        if rpj_err_ref < THRESHOLD_F and rpj_err_cur < THRESHOLD_F:
            inner_index.append(i)
        # score
        score += point_score_F(rpj_err_ref)
        score += point_score_F(rpj_err_cur)
    
    return inner_index, score

def test_F(cur_F, num_points, ref_points,cur_points, rand_8_index):
    sum_select = 0
    sum_unselected = 0
    for j in range(num_points):
        ref_p = np.concatenate([ref_points[j], np.array([1])]) # Point camera ref
        cur_p = np.concatenate([cur_points[j], np.array([1])]) # Point camera cur
        err = np.matmul(ref_p.T, np.matmul(cur_F, cur_p))
        if j in rand_8_index:
            sum_select += abs(err)
        else:
            sum_unselected += abs(err)
    print(sum_select, sum_unselected)

def get_RANSAC_fundamental_mat(ref_points, cur_points):

    # import pdb;pdb.set_trace()
    num_points = ref_points.shape[0]
    MAX_ITER = 200
    k = MAX_ITER
    i = 0
    num_inner_points = 0
    F = 0
    score = 0    
    while i < k:
        i += 1
        rand_8_index = np.random.choice(num_points, size=50, replace=False)
        sample_ref_points = ref_points[rand_8_index]
        sample_cur_points = cur_points[rand_8_index]
        cur_F = get_fundamental_mat(sample_ref_points, sample_cur_points)
        # test_F(cur_F, num_points, ref_points,cur_points, rand_8_index)
        inner_index, cur_score = get_inner_points_and_score_F(cur_F, ref_points, cur_points)
        # print("Round:", i,", inner point number:", len(inner_index), ", score:", cur_score, score)
        if len(inner_index) / num_points > 0.95:
            F = cur_F
            score = cur_score
            break
        if len(inner_index) > num_inner_points:
            num_inner_points = len(inner_index)
            F = cur_F
            score = cur_score
        if i > MAX_ITER:
            break
        omega = num_inner_points / num_points
        if num_inner_points > 0:
            k = np.log10(1 - 0.95) / np.log10(1 - np.power(omega, 8))
    return F, score, inner_index

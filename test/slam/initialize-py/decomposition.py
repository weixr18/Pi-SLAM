from copy import deepcopy
import numpy as np
from numpy.linalg import svd, inv, det


def get_essential_mat(F, K):
    """get essential matrix E"""
    E = np.matmul(K.T, np.matmul(F, K))
    U, Sigma, VT = svd(E)
    sigma = (Sigma[0] + Sigma[1]) / 2
    Sigma_new = np.diag([sigma, sigma, 0])
    E_new = np.matmul(U, np.matmul(Sigma_new, VT))
    return E_new

def get_possible_decomposition_E(E):

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
    ts = []
    for t_c in t_crosses:
        ts.append(np.array([t_c[2,1], t_c[0,2], t_c[1,0]]))
    return Rs, ts

def get_possible_decomposition_H(H, K):

    A = np.matmul(inv(K), np.matmul(H, K))
    U, Sigma, VT = svd(A)
    d1, d2, d3 = Sigma
    assert d1/d2 > 1.00001
    assert d2/d3 > 1.00001

    sin_theta = np.sqrt((d1**2-d2**2)*(d2**2-d3**2)) / (d2*(d1+d3))
    cos_theta = (d1*d3+d2**2) / (d2*(d1+d3))
    x1 = np.sqrt((d1**2-d2**2)/(d1**2-d3**2))
    x3 = np.sqrt((d2**2-d3**2)/(d1**2-d3**2))
    n_ = np.array([x1, 0, x3])
    R_ = np.array([[cos_theta, 0, sin_theta], [0, 1, 0], [-sin_theta, 0, cos_theta]])
    t_ = (d1 - d3) * np.array([x1, 0, -x3])

    n_s = [n_, np.array([x1, 0, -x3]), np.array([-x1, 0, -x3]), np.array([-x1, 0, -x3])]
    R_s= [
        R_, 
        np.array([[cos_theta, 0, -sin_theta], [0, 1, 0], [sin_theta, 0, cos_theta]]),
        np.array([[cos_theta, 0, -sin_theta], [0, 1, 0], [sin_theta, 0, cos_theta]]),
        deepcopy(R_),
    ]
    t_s = [
        t_,
        (d1 - d3) * np.array([x1, 0, x3]),
        (d1 - d3) * np.array([-x1, 0, x3]),
        (d1 - d3) * np.array([-x1, 0, x3])
    ]

    Rs = []
    ts = []
    for i in range(4):
        s = np.sign(det(U) * det(VT))
        R = s * np.matmul(U, np.matmul(R_s[i], VT))
        t = np.matmul(U, t_s[i])
        n = np.matmul(VT.T, n_s[i])
        A_reconstruct = R + np.matmul(t, n.T) / d2
        H_reconstruct = np.matmul(K, np.matmul(A_reconstruct, inv(K)))
        Rs.append(R)
        ts.append(t)
    return Rs, ts

def select_right_decomposition(Rs, ts, K, ref_points, cur_points):

    # select the one solution
    num_points = ref_points.shape[0]
    negative_num = 999999
    R_select, t_select = 0, 0
    for i in range(4):
        R = Rs[i]
        t = ts[i]

        # triangle measurement
        z_curs = []
        z_refs = []
        for j in range(num_points):
            ref_p = np.concatenate([ref_points[j], np.array([1])])
            cur_p = np.concatenate([cur_points[j], np.array([1])])

            # calculate cur depth z_cur
            # $z_2 (K^{-1}P_{c1}) × (K^{-1}P_{c2}) = - (K^{-1}P_{c1}) × t$
            tmp_1 = np.cross(np.matmul(inv(K), ref_p), np.matmul(inv(K), cur_p))
            tmp_2 = - np.cross(np.matmul(inv(K), ref_p), t)
            z_cur = np.dot(tmp_1, tmp_2) / np.dot(tmp_1, tmp_1) # MSE solution
            z_curs.append(z_cur)

            # calculate ref depth z_ref
            # $z_1 (K^{-1}P_{c1}) = z_2 RK^{-1}P_{c2} + t$
            tmp_3 = np.matmul(inv(K), ref_p)
            tmp_4 = z_cur * np.matmul(R, np.matmul(inv(K), cur_p)) + t
            z_ref = np.dot(tmp_3, tmp_4) / np.dot(tmp_3, tmp_3) # MSE solution
            z_refs.append(z_ref)
        z_curs = np.array(z_curs)
        z_refs = np.array(z_refs)

        z_cur_negative_num =  np.sum((z_curs < 0).astype(int))
        z_ref_negative_num = np.sum((z_refs < 0).astype(int))
        print("Decomposite: Round =", i, z_cur_negative_num, z_ref_negative_num)
        if (z_cur_negative_num + z_ref_negative_num < negative_num):
            negative_num = z_ref_negative_num + z_cur_negative_num
            R_select = R
            t_select = t
        pass
    return R_select, t_select


def decomposite_E(E, K, ref_points, cur_points):
    """get R and t from E"""
    Rs, ts = get_possible_decomposition_E(E)
    R, t = select_right_decomposition(Rs, ts, K, ref_points, cur_points)
    return R, t


def decomposite_H(H, K, ref_points, cur_points):
    """get R and t from H"""
    Rs, ts = get_possible_decomposition_H(H, K)
    R, t = select_right_decomposition(Rs, ts, K, ref_points, cur_points)
    return R, t

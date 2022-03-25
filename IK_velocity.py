import numpy as np
from lib.calcJacobian import calcJacobian


def IK_velocity(q_in, v_in, omega_in):
    """
    :param q: 0 x 7 vector corresponding to the robot's current configuration.
    :param v: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 0 x 7 vector corresponding to the joint velocities. If v and omega
         are infeasible, then dq should minimize the least squares error. If v
         and omega have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    # STUDENT CODE GOES HERE
    # check nan case
    dq = np.zeros(7)
    jaco = calcJacobian(q_in)
    v_total = np.hstack((v_in, omega_in))
    #print(v_total)
    nan_index = []
    for a in range(0, 6):
        if v_total[a] != v_total[a]:
            nan_index.append(a)

    if len(nan_index) == 6:
        return dq

    v_total = np.delete(v_total, nan_index)
    jaco = np.delete(jaco, nan_index, 0)
    #print(jaco)

    # check solutions
    solution_exist = 0    # when this index is 0, it has no solution
    v_total_trans = v_total.reshape(-1, 1)
    #print(v_total_trans)
    jaco_aug = np.hstack((jaco, v_total_trans))
    rank_jaco_aug = np.linalg.matrix_rank(jaco_aug)
    rank_jaco = np.linalg.matrix_rank(jaco)

    if rank_jaco == rank_jaco_aug:
        solution_exist = 1
        jaco_plus = np.transpose(jaco) @ np.linalg.inv(jaco @ np.transpose(jaco))

    if solution_exist == 1:
        dq = jaco_plus @ v_total_trans
    else:
        dq = np.linalg.lstsq(jaco, v_total_trans)

    dq = np.transpose(dq)
    dq = dq[0]
    return dq


if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    v1 = [np.nan, np.nan, np.nan]
    v2 = [np.nan, np.nan, np.nan]
    print(np.round(IK_velocity(q, v1, v2),3))

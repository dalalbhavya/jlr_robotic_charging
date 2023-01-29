#! /usr/bin/env python3

from time import time
import osqp
import numpy as np
from scipy import sparse
import matplotlib.pyplot as plt

def matrix_P(dimension):
    row = list()
    col = list()
    val = list()
    for i in range(2*dimension, 3*dimension - 1):
        #All diagonal elements
        row.append(i)
        col.append(i)
        val.append(2)
        
        #All bottom diagonal elements
        row.append(i)
        col.append(i+1)
        val.append(-1)

        #All uppper diagonal elements
        row.append(i+1)
        col.append(i)
        val.append(-1)

    row.append(3*dimension - 1)
    col.append(3*dimension - 1)
    val.append(2)


    P = sparse.csc_matrix((val, (row, col)), shape=(3*dimension, 3*dimension))
    return P

def constraints_matrix(discrete_steps, time_step, ang_max, vel_max, acc_max, jerk_max, initial_ang, goal_ang):

    row = list()
    col = list()
    val = list()

    lower_bound = list()
    upper_bound = list()


    #q_min <= qi <= q_max      - (H+1) constraints
    for i in range((discrete_steps + 1)):
        row.append(i)
        col.append(i)
        val.append(1)
        lower_bound.append(-ang_max)
        upper_bound.append(ang_max)

    #v_min <= vi <= v_max      - (H+1) constraints
    for i in range((discrete_steps + 1), 2*(discrete_steps + 1)):
        row.append(i)
        col.append(i)
        val.append(1)
        lower_bound.append(-vel_max)
        upper_bound.append(vel_max)

    #a_min <= ai <= a_max      - (H+1) constraints
    for i in range(2*(discrete_steps + 1), 3*(discrete_steps + 1)):
        row.append(i)
        col.append(i)
        val.append(1)
        lower_bound.append(-acc_max)
        upper_bound.append(acc_max)

    #j_min <= ji <= j_max
    for i in range(discrete_steps):
        #a_(i+1)
        row.append(len(lower_bound))
        col.append(2*(discrete_steps + 1)  + 1 + i)
        val.append(1/time_step)

        #a_(i)
        row.append(len(lower_bound))
        col.append(2*(discrete_steps + 1) + i)
        val.append(-1/time_step)
        
        lower_bound.append(-jerk_max)
        upper_bound.append(jerk_max)

    #0 <= v_(i+1) - v_(i) - a_(i)*t_step <= 0      - (H) constraints
    for i in range(discrete_steps):
        #v_i
        row.append(len(lower_bound))
        col.append(discrete_steps + 1 +i)
        val.append(-1)

        #v_(i+1)
        row.append(len(lower_bound))
        col.append(discrete_steps + 2 +i)
        val.append(1)

        #a_i*t_step
        row.append(len(lower_bound))
        col.append(2*(discrete_steps + 1) + i)
        val.append(-1*time_step)

        lower_bound.append(0)
        upper_bound.append(0)

    #0 <= q_(i+1) - q_i - v_i*t_step <= 0
    for i in range(discrete_steps):
        #q_i
        row.append(len(lower_bound))
        col.append(i)
        val.append(-1)

        #q_(i+1)
        row.append(len(lower_bound))
        col.append(i + 1)
        val.append(1)

        #v_i*t_step
        row.append(len(lower_bound))
        col.append(discrete_steps + 1 + i)
        val.append(-1*time_step)

        lower_bound.append(0)
        upper_bound.append(0)

    #q_o <= qo <= q_o          - 1 constraint
    row.append(len(lower_bound))
    col.append(0)
    val.append(1)
    lower_bound.append(initial_ang)
    upper_bound.append(initial_ang)

    #q_h <= qh <= q_h          - 1 constraint
    row.append(len(lower_bound))
    col.append(discrete_steps)
    val.append(1)
    lower_bound.append(goal_ang)
    upper_bound.append(goal_ang)

    #0 <= vo <= 0              - 1 constraint
    row.append(len(lower_bound))
    col.append(discrete_steps + 1)
    val.append(1)
    lower_bound.append(0)
    upper_bound.append(0)


    #0 <= vh <= 0              - 1 constraint
    row.append(len(lower_bound))
    col.append(2*discrete_steps + 1)
    val.append(1)
    lower_bound.append(0)
    upper_bound.append(0)

    #0 <= ao <= 0              - 1 constraint
    row.append(len(lower_bound))
    col.append(2*(discrete_steps + 1))
    val.append(1)
    lower_bound.append(0)
    upper_bound.append(0)


    #0 <= ah <= 0              - 1 constraint
    row.append(len(lower_bound))
    col.append(2*(discrete_steps + 1) + discrete_steps)
    val.append(1)
    lower_bound.append(0)
    upper_bound.append(0)

    A = sparse.csc_matrix((val, (row, col)), shape=(len(lower_bound), 3*(discrete_steps+1)))

    lower_bound = np.array(lower_bound)
    upper_bound = np.array(upper_bound)

    return A, lower_bound, upper_bound

def main():
    start_time = time()
    N_DOF = 6
    H = 27
    T_STEP = 0.1
    PI = 3.14159265
    VEL_MAX = PI
    ACC_MAX = PI
    ANG_MAX = PI
    JERK_MAX = 6

    initial_pose_joint_space = [-0.498977, 0.580166, -1.69259, -2.10077, 2.85878, -0.924968]
    goal_pose_joint_space = [1.01352, 0.277848, -1.90593, -1.52476, 1.34359, 2.25875]

    #Initializing P
    P = matrix_P(H+1)

    #Initializing p
    small_p = np.zeros(3*(H+1))

    joint_wise_solution = list()

    for i in range(len(initial_pose_joint_space)):
        #Setting up QP for particular joint
        #Initializing A
        A, l, u = constraints_matrix(H, T_STEP, ANG_MAX, VEL_MAX, ACC_MAX, JERK_MAX, initial_pose_joint_space[i], goal_pose_joint_space[i])

        print(P.shape, A.shape, small_p.shape, l.shape, u.shape)


        prob = osqp.OSQP()
        # Setup workspace and change alpha parameter
        prob.setup(P, small_p, A, l, u, alpha=1.0)
        res = prob.solve()
        joint_wise_solution.append(list(res.x))

    time_array = []
    for i in range(H+1):
        time_array.append(i*T_STEP)

    print("printing trajectory Start")
    #Printing trajectory in recorded form
    for i in range(H+1):
        #printing first 6 joint values
        for j in range(N_DOF):
            print(joint_wise_solution[j][i].round(4))
        for j in range(N_DOF):
            print(joint_wise_solution[j][H+1 + i].round(4))
        for j in range(N_DOF):
            print(joint_wise_solution[j][2*(H+1) + i].round(4))
        print(int(time_array[i]))
        print(int((time_array[i] - int(time_array[i])) * 1000000000))

    print("Printing Trajectory End")

    for i in range(len(joint_wise_solution)):
        for j in range(len(joint_wise_solution[i])):
            joint_wise_solution[i][j] = joint_wise_solution[i][j].round(4)

    print("Execution Time: ", time() -  start_time)

if __name__ == "__main__":
    main()
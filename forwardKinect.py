import numpy as np


# np sin in radians 2pi = 360
def get_rot_mat(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                     [np.sin(theta),  np.cos(theta), 0, 0],
                     [0,               0,            1, 0],
                     [0,               0,            0, 1]])
def get_trans_mat(link_len):
    return np.array([[1, 0, 0, link_len],
                     [0, 1, 0,        0],
                     [0, 0, 1,        0],
                     [0, 0, 0,        1]])

def get_A(theta, link_len):
    return np.dot(get_rot_mat(theta), get_trans_mat(link_len))



# if __name__ == '__main__':
#     # 1 - base; 2 - elbow; 3 - wrist -radius
#     theta1 = 0.1
#     theta2 = 0.2
#     theta3 = 0.3

#     # joint lenghs -mm
#     a1 = 99
#     a2 = 99
#     a3 = 110

#     A1 = get_A(theta1, a1)
#     A2 = get_A(theta2-theta1, a2)
#     A3 = get_A(theta3-theta2, a3)  

#     A = np.dot(np.dot(A1, A2), A3)
#     print(A)

#     # tranform [0,0,0,1] to
#     origin = np.array([[0],[0],[0],[1]])
#     end_loc = np.dot(A, origin)
#     print('pin loc: \n', end_loc)


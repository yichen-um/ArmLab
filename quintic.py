import numpy as np
from math import pow

def quinticInter(q0, qf, t0, tf, v0, vf, ac0, acf, num_pt):
    '''
    perform quintic interpolation
    '''
	# perform quintic interpolation
    M = np.array([
	    [1, t0, pow(t0,2),   pow(t0,3),   pow(t0,4),   pow(t0,5)],
	    [0, 1,  2*t0, 3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4)],
	    [0, 0,  2,    6*t0,  12*pow(t0,2), 20*pow(t0,3)],
	    [1, tf, pow(tf,2),   pow(tf,3),   pow(tf,4),   pow(tf,5)],
	    [0, 1,  2*tf, 3*pow(tf,2), 4*pow(tf,3), 5*pow(tf,4)],
	    [0, 0,  2,    6*tf,  12*pow(tf,2), 20*pow(tf,3)],
	    ])


    b = np.array([q0, v0, ac0, qf, vf, acf])
    b.shape = (6,1)

    a = np.dot(np.linalg.inv(M), b)

    t = np.linspace(t0, tf, num_pt)

    c = np.ones(t.shape)


    qd = a[0][0]*c + a[1][0]*t + a[2][0]*np.power(t,2)+ a[3][0]*np.power(t,3) +\
         a[4][0]*np.power(t,4)+a[5][0]*np.power(t,5)

    qv = a[1][0]*c + 2*a[2][0]*t+ 3*a[3][0]*np.power(t,2) +\
         4 * a[4][0]*np.power(t,3)+ 5*a[5][0]*np.power(t,4)

    return (qd, qv)

# def getVelocity(rex, dt):
#     all_poses = np.asarray(rex.plan)
#     velocity = np.zeros(all_poses.shape)
#     pose_pre = all_poses[0:-2,:]
#     pose_post = all_poses[2:,:]

#     diff = (pose_post-pose_pre)/ 2 / dt
#     velocity[1:-1,:] = diff
#     return velocity

def getDerivative(mat, dt):
    '''
    Calculate derivative using Euler difference
    '''
    all_poses = np.asarray(mat)
    derive = np.zeros(all_poses.shape)
    pre = all_poses[0:-2,:]
    post = all_poses[2:,:]

    diff = (post-pre)/ 2 / dt
    derive[1:-1,:] = diff
    return derive

def calAffine(src, dst):
    '''
    Calculate robust affine transformation using least square
    '''
    
    # TODO: add asssertion to see of dim of src and dst are the same

    dim = len(src)   # # of points used for calclting affine matrix

    A = np.zeros((dim * 2, 6))
    y = np.zeros((dim*2, 1))

    for ii in range(0, dim):
        A[2*ii,0] = src[ii,0]
        A[2*ii,1] = src[ii,1]
        A[2*ii,2] = 1.0
        y[2*ii,0] = dst[ii,0]
        
        A[2*ii + 1,3] = src[ii,0]
        A[2*ii + 1,4] = src[ii,1]
        A[2*ii + 1,5] = 1.0
        y[2*ii + 1,0] = dst[ii,1]       
            

    beta = np.matmul(np.linalg.pinv(A), y)
    affine = beta.reshape((2,3))

    # AA = np.matmul(A.T, A)
    # w, v = np.linalg.eig(AA)
    
   
    # result = np.where(w == np.min(w))
    # result = np.asarray(result)
    # print(result.shape)
    # print('=' *10)
    # print(result[0,1])
    # print('$' *10)
    # print(v[result[0,0]])
    return affine


def calAffine3D(src, dst):
    '''
    Calculate robust affine transformation using least square
    '''
    
    # TODO: add asssertion to see of dim of src and dst are the same

    dim = len(src)   # # of points used for calclting affine matrix

    A = np.zeros((dim * 3, 12))
    y = np.zeros((dim*3, 1))

    for ii in range(0, dim):
        A[3*ii,0] = src[ii,0]
        A[3*ii,1] = src[ii,1]
        A[3*ii,2] = src[ii,2]
        A[3*ii,3] = 1.0
        y[3*ii,0] = dst[ii,0]

        A[3*ii + 1,4] = src[ii,0]
        A[3*ii + 1,5] = src[ii,1]
        A[3*ii + 1,6] = src[ii,2]
        A[3*ii + 1,7] = 1.0
        y[3*ii + 1,0] = dst[ii,1]

        A[3*ii + 2,8] = src[ii,0]
        A[3*ii + 2,9] = src[ii,1]
        A[3*ii + 2,10] = src[ii,2]
        A[3*ii + 2,11] = 1.0
        y[3*ii + 2,0] = dst[ii,2]      
            

    beta = np.matmul(np.linalg.pinv(A), y)
    affine3D = beta.reshape((3,4))

    return affine3D







if __name__ == '__main__':
    # helper - calibrate affine from rgb to depth
    rgb = np.array([[427.0, 150.0],[309.0, 136.0], [201.0, 264.0], \
                    [199.0, 383.0],[314.0, 389.0]])

    depth = np.array([[455.0, 125.0],[327.0, 109.0],[204.0, 257.0],\
                    [204.0, 382.0],[334.0, 389.0]])

    # rgb = np.array([[129,93],[121,435], [190,139], \
    #                 [387,153], [371,222], [473,99]])
    # depth = np.array([[131,70],[129,441],[194,115],\
    #                 [412,128],[401,202],[503,70]])

    rgb2depth = calAffine(rgb, depth)
    # depth2rgb = calAffine(depth, rgb)
    print('$' *10)
    print(rgb2depth)
    print('=' *10)
    # print(w)
    print('$' *10)
    # print(v)


    # print(getDepthfromD(722))
    # print(getDepthfromD(710))

import numpy as np
# Goal: given a set of (x, y, z, phi) -> theta 0 to 3
# all angles are in radians
#     o ------ z
#     |
#     |
#     |
#     x
# class NotInRangeError(Exception):
#     pass
D2R = 3.141592/180.0
R2D = 180.0/3.141592
def getJointAnglesFromIK(pose, flag_elbow):
    # arm configs
    a1 = 99 # link length, shoulder to elbow, mm
    a2 = 99 # link length, elbow to wrist
    # a3 = 140 # link length wrist to pin tip
    a3 = 135 # link length wrist to gripper bottom with 10 mm ofset
    H = 130 # base hight
    #  TODO: double check with Li
    # #  original
    # x, y, z, phi = pose
    #  now
    z, x, y, phi = pose
    # define working space
    dist2Origin = np.sqrt(x*x + z*z + (y-H)*(y-H))
    if a1 + a2 + a3 < dist2Origin:
        # print('Can not reach the given location!')
        return None
    # base rotation - 0
    theta0 = np.arctan2(z, x)
    while theta0 >= np.pi:
        theta0 -= 2*np.pi
    while theta0 < -np.pi:
        theta0 += 2*np.pi
    # albow rotation - 2
    x_3 = np.sqrt(z*z + x*x) - a3*np.cos(phi)
    # x_3 = x/np.cos(theta0) - a3*np.cos(phi)
    y_3 = y - a3*np.sin(phi) - H
    cos_angle3 = (x_3*x_3 + y_3*y_3 - a1*a1 - a2*a2)/2/a1/a2
    if cos_angle3 > 1 or cos_angle3 < -1:
        # print('cos angle greater than 1')
        return None
    theta2 = np.arccos(cos_angle3)
    while theta2 >= np.pi:
        theta2 -= 2*np.pi
    while theta2 < -np.pi:
        theta2 += 2*np.pi

    # shoulder rotation - 1
    beta = np.arctan2(y_3, x_3)
    cos_angle2 = (x_3*x_3 + y_3*y_3 + a1*a1 - a2*a2)/2/np.sqrt(x_3*x_3+y_3*y_3)/a1
    if cos_angle2 > 1 or cos_angle2 < -1:
        # print('cos angle greater than 1')
        return None
    tsai = np.arccos(cos_angle2) 
    
    # elbow down (0) or elbow up (1)
    if flag_elbow == 0: 
        theta1 = np.pi / 2 - beta - tsai
    else: # cfg == 1, elbow up
        theta1 = np.pi / 2 - beta + tsai
        theta2 = -theta2
    while theta1 >= np.pi:
        theta1 -= 2*np.pi
    while theta1 < -np.pi:
        theta1 += 2*np.pi
    # wrist rotation - 3
    theta3 = -phi + np.pi/2 - theta1 - theta2
    while theta3 >= np.pi:
        theta3 -= 2*np.pi
    while theta3 < -np.pi:
        theta3 += 2*np.pi
    return [theta0, theta1, theta2, theta3]

def inverse_kinematic(position, distance_threshold, lift, phi_near = 90, phi_far = 10):
    '''
  
    '''
    x_wld, y_wld, z_wld = position
    distance = np.sqrt(x_wld * x_wld + y_wld * y_wld)
    h_offset = 19

    # Convert z_wld to the center of the cuber
    z_wld -= h_offset
 
    # for close points use phi = -90
    if distance <= distance_threshold:
        # print('Distance is less than: ' + str(distance_threshold))
        # print('Approaching from top')
        phi = -phi_near * D2R
        
        # Min height of placement
        min_height = 35

        if z_wld <= min_height:
            z_wld = min_height

        pose = (x_wld, y_wld, z_wld, phi)
        
        # prefer using elbow up
        flag_elbow = 0
        angles_elbowup_top = getJointAnglesFromIK(pose, flag_elbow)
        
        if angles_elbowup_top is not None:
            # print('Using elbowup aproaching from top')
            IK_angles = angles_elbowup_top
            return IK_angles
        else :
            # print('Can not reach the given location using elbow up configurtion approaching from top!')
            # print('try approaching using elbow down')
            # prefer using elbow up
            flag_elbow = 1
            angles_elbowdown_top = getJointAnglesFromIK(pose, flag_elbow)
            if angles_elbowdown_top is not None:
                # print('Using elbowdown aproaching from top')
                IK_angles = angles_elbowdown_top
                return IK_angles
            else: 
                # print('Can not reach the given location using elbow down configurtion approaching from top either!')
                # print('No solution found approaching from top!')
                pass
    # for far away points use phi = 0
    else:
        # print('Distance is greater than: ' + str(distance_threshold))
        # print('Approaching from side')
        phi = -phi_far * D2R

        min_height = np.abs(45 * np.sin(phi)) 

        if z_wld <= min_height:
            z_wld = min_height

        pose = (x_wld, y_wld, z_wld, phi)
        # prefer using elbow down
        flag_elbow = 0
        angles_elbowdown_side = getJointAnglesFromIK(pose, flag_elbow)
        
        if angles_elbowdown_side is not None:
            # print('Using elbowdown aproaching from side')
            IK_angles = angles_elbowdown_side
            return IK_angles
        else :
            # print('Can not reach the given location using elbow down configurtion approaching from side!')
            # print('try approaching using elbow up')
            # prefer using elbow up
            flag_elbow = 1
            angles_elbowup_side = getJointAnglesFromIK(pose, flag_elbow)
            if angles_elbowup_side is not None:
                # print('Using elbowup aproaching from side')
                IK_angles = angles_elbowup_side
                return IK_angles
            else: 
                # print('Can not reach the given location using elbow up configurtion approaching from side either!')
                pass
    print('No solution found for I.K.!')
    return None
if __name__ == '__main__':
    print('before call')
    configs = inverse_kinematic((0, 200, 50),120, 40)
    print(configs)
    print('after call')
    configs2 = inverse_kinematic((-200, 100, 50), 120, 40)
    print(configs2)
  

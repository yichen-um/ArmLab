from inverse_kinematics import *



D2R = 3.141592/180.0
R2D = 180.0/3.141592

def generate_wpts (initPos, targetPos, eventMode, distance_threshold = 190, lift = 30, offset_height = 70, offset_angle = 10, phi_near = 90, phi_far = 15):
	'''
	distance_threshold: determine whether it's close or far
	lift: lift of the gripper tip from the ground
	offset_angle: used to generate way points right above the target, typically only changing the shoulder joint 
	'''
	# print('initial position: ' + str(initPos))
	way_point_pick = []
	way_point_place = []
	neutural_angle = []

	x_init, y_init, z_init = initPos
	x_target, y_target, z_target = targetPos

	angles_initPos = inverse_kinematic(initPos, distance_threshold, lift, phi_near, phi_far)
	angles_targetPos = inverse_kinematic(targetPos,distance_threshold, lift, phi_near, phi_far)
	# angles_before_initPos = inverse_kinematic(initPos, distance_threshold, lift, phi_near, phi_far)

	# if angles_initPos[1] < 0 :
	# 	angles_before_initPos[1] += offset_angle*D2R
	# 	else : 
	# 	angles_before_initPos[1] -= offset_angle*D2R
	# 		print(angles_before_initPos[1])

	# Calculate a angle above place of targetPos
	# if angles_targetPos[1] < 0 :
	# 	angles_before_targetPos[1] += offset_angle * D2R
	# 	if angles_before_targetPos[1] >= 0 :
	# 		angles_before_targetPos[1] = 0
	# else : 
	# 	angles_before_targetPos[1] -= offset_angle * D2R
	# 	if angles_before_targetPos[1] <= 0 :
	# 		angles_before_targetPos = 0

	position_above_targetPos = x_target, y_target, z_target + offset_height
	angles_above_targetPos = inverse_kinematic(position_above_targetPos,distance_threshold, lift, phi_near, phi_far)

	if eventMode == 5:
		# neutural_angle.append(angles_above_targetPos)
		neutural_angle = angles_above_targetPos
	else:
		neutural_angle.append(angles_above_targetPos)

	way_point_pick.append(angles_initPos)
	way_point_place.append(angles_targetPos)

	return way_point_pick, neutural_angle, way_point_place

if __name__ == '__main__':
    way_point_pick, neutural, way_point_place = generate_wpts((181.27319446061529, 1.0899161927865864, 39.751825699056944), 
    	(-180, 0, 38))

    # way_point_pick, neutural, way_point_place = generate_wpts((181.27319446061529, 1.0899161927865864, 39.751825699056944), 
    # 	(-181.26507721884641, -0.62044927238999115, 39.768184327216368))

    print(np.asarray(way_point_pick)*R2D)
    print(np.asarray(way_point_place)*R2D)

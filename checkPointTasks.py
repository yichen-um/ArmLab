import time
import numpy as np


R2D = 180.0/3.141592
D2R = 3.141592/180.0

class checkPointTasks():

	"""
	Retired code for teach and repeat
	TODO: To place rex as rexarm
	"""
	
	def teach(self):
        self.rex.torque_multiplier = 0
        self.rex.plan = []
        self.rex.plan_status = 0
        self.rex.wpt_number = 0
        self.rex.wpt_total = 0
        self.rex.cmd_publish()
        print(self.rex.wpt_number, self.rex.plan)

    def record(self):
        """Record the waypoints"""
        angles = [self.rex.joint_angles_fb[0], self.rex.joint_angles_fb[1], \
                    self.rex.joint_angles_fb[2], self.rex.joint_angles_fb[3], \
                    self.rex.joint_angles_fb[4]]
        self.rex.plan.append(angles)
        self.rex.wpt_total += 1
        
        for i in range(len(angles)):
            print(angles[i] * R2D)
        print("---------") 
        
        np.save('waypoints', self.rex.plan)

        self.ui.rdoutStatus.setText("Recording: point %d" \
            %(self.rex.wpt_total))

	def playback(self, maxMove):
        """let arm touch each point recorded in advance"""
        print('start playing back')

        # when playback is executed, the recording is done
        self.rex.plan = np.load('waypoints.npy')

        print(self.rex.plan)
        #repeat the motion of the arm through the recorded waypoints
        self.rex.torque_multiplier = 0.3
        self.rex.speed_multiplier = 0.4
        self.rex.plan_status = 1

        all_motor_p = self.planMotion(interp_num = 5, interp_time = 1)

        # start in idle state
        curState = self.statemachine.idle()
         
        for i in range(len(all_motor_p[0])):

            targetP = all_motor_p[:,i]
            self.rex.wpt_number += 1

            # command to move to the target position
            cmdState = 1
            # counter is used to check the number of moves to target position
            counter = 0
            # if it's not in emergency stop state
            if (curState != -1):
                print('======')
                self.runStateMachine(targetP, cmdState)
                curState = self.statemachine.getCurrentState()
                print(curState)
                inPos, idxInPose = self.statemachine.checkPose(self.rex, targetP, epsilon=0.05)

                if not inPose and counter <= maxMove:
                    self.runStateMachine(targetP, cmdState)
                    counter += 1
                else:
                    curState = self.statemachine.estop(self.rex)
                    print('ERROR: Robot stopped before reaching to the target position')
                    print('Command repeated '+ str(counter) + ' times')
                    print('Motors in position' + str(idxInPose))
            
            else:
                break
                # self.statemachine.estop(self.rex)

	"""
	Retired code for simple pick and place
	TODO: To place rex as rexarm
	"""
    def simple_picknplace(self):
        self.flag_pickAndPlace = 1
        print('Pick and place flag = ' + str(self.flag_pickAndPlace))
    
    def executePickAndPlace(self, posList):
        # Do all the motion
        print('The reqiured end effector position is: ')
        print(posList)
        wldx,wldy,wldz = posList
        phi = - 45 * D2R
        target = (wldx,wldy,wldz,phi)
        print('The converted joint angles are:')
        self.reach_up(target)
        print('Motion finished')
        # self.flag_pickAndPlace = 0
        # print('Now, Pick and place flat = ' + str(self.flag_pickAndPlace))
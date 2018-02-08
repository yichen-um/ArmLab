import time
import numpy as np
from generate_wp import generate_wpts
from inverse_kinematics import inverse_kinematic, getJointAnglesFromIK


R2D = 180.0/3.141592
D2R = 3.141592/180.0
pOpen = 90*D2R
pClose = pOpen - 90*D2R
pRotate = 15*D2R
pRotateBack = pRotate - 90*D2R
# correct_order = ['black', 'red','orange', 'yellow', 'green', 'blue', 'violet', 'pink']
correct_order = ['black',  'green',  'violet',  'green',  'green',  'violet',  'violet',  'violet']

class Statemachine():
    curState = 1 # idle = 0; estop = -1; move = 1
    
    def runStateMachine(self, video, rexarm, eventMode):
        """
        Run state machine
        """
        print('Play event ' + str(eventMode))
        nMaxTry = 1
        eventFinished = False # True means assigned event is finished, otherwise False
        # Determine which event will be played
        for i in range(nMaxTry):
            # Detect all the blocks on the board
            colorList, coordList = video.blockDetector()
            for idx in range(len(coordList)):
                print(str(idx)+' Current coord' + str(coordList[idx]))
                print(str(idx)+' Identified color: ' + str(colorList[idx]))
                print('\n')  
            # print('!!!', colorList)
            # print('Current block location list: ' + str(coordList))

            # Based on the event mode, evaluate the board, decide to whether or not finish the event, return coords
            if eventMode != 6:
                eventFinished, orignal_locs, target_locs = self.boardEvaluation(colorList, coordList, eventMode)            
            else:
                orignal_locs = [np.asarray(video.wld_coord[0])]
                target_locs = [np.asarray(video.wld_coord[1])]
            # Move block accordingly

            for i in range(len(orignal_locs)):
                # Check constrain for each positions, generate way points
                wayPtsPick, wayPtsN, wayPtsPlace = generate_wpts(orignal_locs[i], target_locs[i], eventMode) # 0 for pick, 1 for place
                # if (wayPtsPick is None or wayPtsN is None or wayPtsPlace is None):
                    # print('No solution found, way points are following: ')
                print('Way points are following: ')
                print(np.asarray(wayPtsPick)*R2D)
                print(np.asarray(wayPtsN) * R2D)
                print(np.asarray(wayPtsPlace) * R2D)
                print('1', np.asarray(wayPtsN[0]))
                print('2', wayPtsN[0])

                if eventMode == 1:
                    self.pickAndPlace_E1_E2(rexarm, wayPtsPick, wayPtsN, wayPtsPlace)
                elif eventMode == 2:
                    self.pickAndPlace_E1_E2(rexarm, wayPtsPick, wayPtsN, wayPtsPlace)
                elif eventMode == 3:
                    self.pickAndPlace_E1_E2(rexarm, wayPtsPick, wayPtsN, wayPtsPlace)
                elif eventMode == 4:
                    pass
                elif eventMode == 5:
                    self.pickAndPlace_E5(rexarm, wayPtsPick, wayPtsN, wayPtsPlace)
                    pass
                elif eventMode == 6:
                    self.pickAndPlace_E1_E2(rexarm, wayPtsPick, wayPtsN, wayPtsPlace)
                # print('Way points for pick' + str(wayPtsPick))
                # print('The following block has been moved' + str(i) + ' ' + str(success))

        # End events, reset
        eventFinished = True
        print('Event is finished, end of updating Gui')

        # return True

    def findPicks(self, colorList, coordList, eventMode):
        # Identify the coordinates of the blocks need to be picked
        coordPicks = list()
        xMin, yMin = -310, -310
        xMax, yMax = 310, 310

        if eventMode == 1 or eventMode == 2:
            for coord in coordList:
                if coord[0] > 10:
                    coordPicks.append(coord)
        elif eventMode == 3:
            coordPicks = self.sortCoordsByColor(colorList, coordList)
        elif eventMode == 5:
            for coord in coordList:
                if coord[1] > 10 and coord[1] < yMax and coord[0] > xMin and coord[0] <xMax:
                    coordPicks.append(coord)
         # Output message and return
        return coordPicks

    def sortCoordsByColor(self, colorList, coordList):
        # given a color list, sort the coordlist based on the global color
        sorted_coord_list = []
        for color in correct_order:
            sorted_coord_list.append(coordList[colorList.index(color)])
        return sorted_coord_list

    def findPlaces(self, coordPicks, eventMode):
        # Identify the coordinates of the selected blocks that need to be placed
        coordPlaces = list()
        # Parameters for Event 2
        coordE2 = [-220, 0, 38]
        hStack = 50
        # Parameters for Event 3
        coordE3 = [-200, 200, 38]
        gap = 70
        # Parameters for Event 5
        level = 3
        dx = 50
        dz = 60
        coordE5 = [[-50, -150, 38], [-50 + dx, -150, 38], [-50 + dx/2, -150, 38+dz]]

        if eventMode == 1:
            for coord in coordPicks:
                coordPlaces.append([-coord[0], coord[1], coord[2]])
        elif eventMode == 2:
            for idx in range(len(coordPicks)):
                coordPlaces.append([coordE2[0], coordE2[1]-idx*5, coordE2[2] + hStack*idx])
        elif eventMode == 3:
            for i in range(len(coordPicks)):
                coordPlaces.append([coordE3[0], coordE3[1] - gap*i, coordE3[2]])
        elif eventMode == 5:
            coordPlaces = coordE5
        # Output message and return
        return coordPlaces

    def boardEvaluation(self, colorList, coordList, eventMode):
        coordPicks = list()
        coordPlaces = list()
        eventFinished = False
        # Find picks
        coordPicks = self.findPicks(colorList, coordList, eventMode)
        
        # Determine if events finished, if not, find coordinate for places
        if not coordPicks:
            # list is empty
            eventFinished = True
        else:
            coordPlaces = self.findPlaces(coordPicks, eventMode)
            eventFinished = False
        # print('To be moved, color list: ' + str(colorList) + '\n')
        print('To be moved, coord list: ' + str(coordPicks) + '\n')
        print('target locs list: ' + str(coordPlaces) + '\n')
        return eventFinished, coordPicks, coordPlaces


    def pickAndPlace_E5(self, rexarm, wayPtsPick, wayPtsN, wayPtsPlace, epsilon = 0.17, nMaxMove = 300):
        armInPose = False
        gripOpened = False
        gripClosed = False
        wayPtsReset = [0.0,0.0,0.0,0.0]
        nMaxMoveOpen = 500
        epsilonGrip = 0.2
        
        # dThetaSafe = 30*D2R
        # wayPtsSafe = wayPtsPlace[0]
        # Overall motion sequence:
        # V1: Open->Pick->Close->Lift-> Neural->Place->Rotate->Open->Lift->Neural->Next block

        # Open gripper
        gripOpened = self.openGrip(rexarm, pOpen, epsilon, nMaxMoveOpen)

        # To the pick location
        for pickMove in wayPtsPick:
            armInPose = self.moveArmToPick(rexarm, pickMove, epsilon, nMaxMove)


        # Close the gripper
        gripClosed = self.closeGrip(rexarm, pClose, epsilon, nMaxMove)

        # Lift to safe, neutral ready
        liftSafe = self.liftArmToSafe(rexarm, epsilon, nMaxMove)

        # To next place ready positoin
        self.reset(rexarm)
        # armInPose = self.moveArmToPlace(rexarm, wayPtsN, epsilon, nMaxMove)

        # To rotate the gripper
        print('Now, rotate gripper')
        gripOpened = self.rotateGrip(rexarm, pRotate, epsilonGrip, nMaxMoveOpen)
        if not gripOpened:
            self.resetOpen(rexarm)
            print('Actually not rotated')
            return False
        else:
            print('Actually rotated')

        # To the place location
        for placeMove in wayPtsPlace: # gripper at targted location
            armInPose = self.moveArmToPlace(rexarm, placeMove, epsilon, nMaxMove)
            # if not armInPose:
            #     self.resetOpen(rexarm)
            #     return False

        # Open gripper to drop
        print('Now, open gripper to drop it')
        gripOpened = self.openGrip(rexarm, pOpen, epsilonGrip, nMaxMoveOpen)
        if not gripOpened:
            self.resetOpen(rexarm)
            print('Actually not opened')
            return False
        else:
            print('Actually opened')

        # Lift to safe, neutral ready
        liftSafe = self.moveArmSameTime(rexarm, wayPtsN, epsilonGrip, nMaxMoveOpen)

        # To neutral positoin
        self.resetOpen(rexarm)

    def pickAndPlace_E1_E2(self, rexarm, wayPtsPick, wayPtsN, wayPtsPlace, epsilon = 0.17, nMaxMove = 300):
        armInPose = False
        gripOpened = False
        gripClosed = False
        wayPtsReset = [0.0,0.0,0.0,0.0]
        nMaxMoveOpen = 300
        epsilonGrip = 0.2

        # Overall motion sequence:
        # V1: Open->Pick->Close->Lift-> Neural ->Place->Open->Lift-> Neural ->Next block
        # V2: Open->Pick->Close->Lift-> Place above ->Place->Open->Lift->Next block

        # Open gripper
        gripOpened = self.openGrip(rexarm, pOpen, epsilon, nMaxMoveOpen)
        # if not gripOpened:
        #     self.resetOpen(rexarm)
        #     return False

        # To the pick location
        for pickMove in wayPtsPick:
            armInPose = self.moveArmToPick(rexarm, pickMove, epsilon, nMaxMove)
            # if not armInPose:
            #     self.resetOpen(rexarm)
            #     return False

        # Close the gripper
        gripClosed = self.closeGrip(rexarm, pClose, epsilon, nMaxMove)
        # if not gripClosed:
        #     self.resetOpen(rexarm)
        #     return False

        # Lift to safe, neutral ready
        liftSafe = self.liftArmToSafe(rexarm, epsilon, nMaxMove)
        # if not liftSafe:
        #     self.resetOpen(rexarm)
        #     return False

        # To next place ready positoin
        self.reset(rexarm)
        # armInPose = self.moveArmToPlace(rexarm, wayPtsN, epsilon, nMaxMove)

        # To the place location
        for placeMove in wayPtsPlace: # gripper at targted location
            armInPose = self.moveArmToPlace(rexarm, placeMove, epsilon, nMaxMove)
            # if not armInPose:
            #     self.resetOpen(rexarm)
            #     return False

        # Open gripper
        print('Now, open gripper to drop it')
        gripOpened = self.openGrip(rexarm, pOpen, epsilonGrip, nMaxMoveOpen)
        if not gripOpened:
            self.resetOpen(rexarm)
            print('Actually not opened')
            return False
        else:
            print('Actually opened')

        # Lift to safe, neutral ready
        liftSafe = self.liftArmToSafe(rexarm, epsilon, nMaxMove)
        # if not liftSafe:
        #     self.resetOpen(rexarm)
        #     return False

        # To neutral positoin
        self.resetOpen(rexarm)

        # armInPose = self.moveArmToPlace(rexarm, wayPtsReset, epsilon, nMaxMove)
        # if not armInPose:
        #     self.reset2Upright(rexarm)
        #     return False


    def moveArmToPick(self, rexarm, targetP, epsilon, nMaxMove):
        # Move arm to target location, if failed, report status
        # B -> E -> S -> W
        # 0 -> 2 -> 1 -> 3

        allInPose = [False] * 4
        motionOrder = [0, 2, 3, 1]

        for i in range(len(motionOrder)):
            idxJoint = motionOrder[i]

            for idxMove in range(nMaxMove):
                inPose = self.checkArmPose(rexarm, idxJoint, targetP[idxJoint], epsilon)
                if inPose:
                    allInPose[idxJoint] = True
                    break
                else:
                    # print(targetP[i])
                    # print(rexarm.joint_angles_fb[i])
                    # print('=====')
                    # print('## not in pose')
                    rexarm.joint_angles[idxJoint] = targetP[idxJoint]
                    rexarm.cmd_publish()
        print('Move arm to pick failed')
        return np.all(np.asarray(allInPose))

    def moveArmToPlace(self, rexarm, targetP, epsilon, nMaxMove):
        # Move arm to target location, if failed, report status
        # B -> S -> E -> W
        # 0 -> 1 -> 2 -> 3
        allInPose = [False] * 4
        # motionOrder = [0, 1, 2, 3]
        motionOrder = [0, 1, 3, 2]

        for i in range(len(motionOrder)):
            idxJoint = motionOrder[i]

            for idxMove in range(nMaxMove):
                inPose = self.checkArmPose(rexarm, idxJoint, targetP[idxJoint], epsilon)
                if inPose:
                    allInPose[idxJoint] = True
                    break
                else:
                    # print(targetP[i])
                    # print(rexarm.joint_angles_fb[i])
                    # print('=====')
                    # print('## not in pose')
                    rexarm.joint_angles[idxJoint] = targetP[idxJoint]
                    rexarm.cmd_publish()
        print('Move arm to place failed')
        return np.all(np.asarray(allInPose))

    def moveArmSameTime(self, rexarm, targetP, epsilon, nMaxMove):
        # Move arm to target location, if failed, report status
        allInPose = [True] * 4
        for idxMove in range(nMaxMove):
            inPose = self.checkArmPose(rexarm, 3, targetP[3], epsilon)
            if inPose:
                allInPose = [True] * 4
            else:
                rexarm.joint_angles[0] ,rexarm.joint_angles[1], rexarm.joint_angles[2], rexarm.joint_angles[3] = targetP
                rexarm.cmd_publish()
        print('Move arm to place failed')
        return np.all(np.asarray(allInPose))

    def liftArmToSafe(self, rexarm, epsilon, nMaxMove, dTheta = 50):
        dTheta = dTheta*D2R
        idxJoint = 1

        # Define safe position
        rexarm.get_feedback()
        joint_shoulder_fb = rexarm.joint_angles_fb[idxJoint]
        if joint_shoulder_fb < 0:
            joint_shoulder_target =  joint_shoulder_fb + dTheta
            if joint_shoulder_target >= 0:
                joint_shoulder_target = -5 * D2R
        else:
            joint_shoulder_target =  joint_shoulder_fb - dTheta
            if joint_shoulder_target <= 0:
                joint_shoulder_target = 5 * D2R
        # Publish
        for idxMove in range(nMaxMove):
            inPose = self.checkArmPose(rexarm, idxJoint, joint_shoulder_target, epsilon)
            if inPose:
                return True
            else:
                rexarm.joint_angles[idxJoint] = joint_shoulder_target
                rexarm.cmd_publish()
        print('Arm failed to lift to safe position, motor resets')
        return False

    def rotateGrip(self, rexarm, targetP, epsilon, nMaxMove):
        # Rotate gripper to target location, if failed, report status
        for idxMove in range(nMaxMove):
            inPose = self.checkGripRotation(rexarm, targetP, epsilon)
            if not inPose:
                rexarm.joint_angles[4] = targetP
                rexarm.cmd_publish()
            else:
                return inPose
        
        inPose = self.checkGripRotation(rexarm, targetP, epsilon)
        if (not inPose):
            print ('Gripper is not rotated')
        return inPose

    def openGrip(self, rexarm, targetP, epsilon, nMaxMove):
        # Open gripper to target location, if failed, report status
        for idxMove in range(nMaxMove):
            inPose = self.checkGripPose(rexarm, targetP, epsilon)
            if not inPose:
                rexarm.joint_angles[5] = targetP
                rexarm.cmd_publish()
            else:
                return inPose
        
        inPose = self.checkGripPose(rexarm, targetP, epsilon)
        if (not inPose):
            print ('Gripper is not opened')
        return inPose

    def closeGrip(self, rexarm, targetP, epsilon, nMaxMove):
        # Close gripper to target location, if failed, report status
        for idxMove in range(nMaxMove):
            inPose = self.checkGripPose(rexarm, targetP, epsilon)
            if not inPose:
                rexarm.joint_angles[5] = targetP
                rexarm.cmd_publish()
            else:
                return inPose
        
        inPose = self.checkGripPose(rexarm, targetP, epsilon)
        if (not inPose):
            print ('Gripper is not closed')
        return inPose

    def checkArmPose(self, rexarm, motorID, target, epsilon):
        """ check the pose for a single motor (at ID)"""
        inPose = False
        rexarm.get_feedback()
        if abs(rexarm.joint_angles_fb[motorID] - target) < epsilon:
            inPose = True
        return inPose

    # def checkArmPose(self, rexarm, targetP, epsilon):
    #     inPose = False
    #     idxInPose = [False] * 5
    #     rexarm.get_feedback()
    #     for i in range(len(idxInPose)):
    #         if abs(rexarm.joint_angles_fb[i]-targetP[i]) < epsilon:
    #             idxInPose[i] = True
    #     if (idxInPose[0] and idxInPose[1] and idxInPose[2] and idxInPose[3] and idxInPose[4]):
    #         inPose = True
    #     return inPose, idxInPose
            
    def checkGripPose(self, rexarm, targetP, epsilon):
        rexarm.get_feedback()
        if abs(rexarm.joint_angles_fb[5]-targetP) < epsilon:
            return True
        else:
            return False

    def checkGripRotation(self, rexarm, targetP, epsilon):
        rexarm.get_feedback()
        if abs(rexarm.joint_angles_fb[4]-targetP) < epsilon:
            return True
        else:
            return False

    def reset(self, rexarm):
        # reset the arm to upright
        rexarm.max_torque = [1, 1, 1, 1, 1, 1]
        rexarm.joint_angles[0] = 0
        rexarm.joint_angles[1] = 0
        rexarm.joint_angles[2] = 0

        rexarm.joint_angles[3] = 0
        rexarm.joint_angles[4] = pRotateBack
        rexarm.joint_angles[5] = pClose
        rexarm.cmd_publish()
        print('Robot locaiton is reset')
        return True

    def resetOpen(self, rexarm):
        # reset the arm to upright
        rexarm.max_torque = [1, 1, 1, 1, 1, 1]
        rexarm.joint_angles[0] = 0
        rexarm.joint_angles[1] = 0
        rexarm.joint_angles[2] = 0

        rexarm.joint_angles[3] = 0
        rexarm.joint_angles[4] = pRotateBack
        rexarm.joint_angles[5] = pOpen
        rexarm.cmd_publish()
        print('Robot locaiton is reset with gripper open')
        return True

    def idle(self):
        #state that runs by default
        self.curState = 0
        return self.curState

    def estop(self, rexarm):
        #sets the torque of all motors to 0, in case of emergency
        rexarm.max_torque = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rexarm.cmd_publish()
        print('Robot is in emergency stop status')
        return True

    def planMotion(self, interp_num = 5, interp_time = 2):
        all_motor_p = []
        all_motor_v = []
        p_mat = self.rex.plan
        v_mat = getDerivative(p_mat, interp_time)
        a_mat = getDerivative(v_mat,interp_time)

        # q0, qf, t0, tf, v0, vf, ac0, acf

        for i in range(len(self.rex.plan[0])): # for each motor
            current_p_list = []
            current_v_list = []
            for j in range(len(self.rex.plan)-1): # for each position
                currentP = p_mat[j][i]
                currentV = v_mat[j][i]
                currentA = a_mat[j][i]
                nextP = p_mat[j+1][i]
                nextV = v_mat[j+1][i]
                nextA = a_mat[j+1][i]

                qd, qv = quinticInter(currentP,nextP,0,interp_time, \
                    currentV, nextV, currentA, nextA, interp_num)

                current_p_list.append(qd)
                current_v_list.append(qv)

            current_p_list = [item for sublist in \
                                current_p_list for item in sublist]
            current_v_list = [item for sublist in \
                                current_v_list for item in sublist]
            all_motor_p.append(current_p_list)
            all_motor_v.append(current_v_list)

        all_motor_p = np.asarray(all_motor_p)
        all_motor_v = np.asarray(all_motor_v)

        return all_motor_p

    def testArmMove(self, ui, rexarm):
        targetP = [None] * 6
        targetP[0] = ui.sldrBase.value()*D2R
        targetP[1] = ui.sldrShoulder.value()*D2R
        targetP[2] = ui.sldrElbow.value()*D2R
        targetP[3] = ui.sldrWrist.value()*D2R
        targetP[4] = -75*D2R
        targetP[5] = 0*D2R

        resetP = [0] * 6
        resetP[4] = -75*D2R
        resetP[5] = 0*D2R

        # print('Target pose = ' + str(targetP))
        # print('Reset pose = ' + str(resetP))

        self.moveArmB2T(rexarm, targetP, 0.1, 300)
        self.moveArmT2B(rexarm, resetP, 0.1, 300)
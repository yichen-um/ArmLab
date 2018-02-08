#!/usr/bin/env python
import sys
import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt
from ui import Ui_MainWindow
from rexarm import Rexarm
from video import Video
from statemachine import Statemachine
import time
from quintic import quinticInter, getDerivative, calAffine, calAffine3D
from cube_detection import *
from threading import Thread
from transformCoord import *

""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592

""" Pyxel Positions of image in GUI """
MIN_X = 310
MAX_X = 950

MIN_Y = 30
MAX_Y = 510

""" Affine transformation to align RGB camera and depth sensor"""
# Calibration, 2018-02-01
rgb2depthAff = np.array([[1.09741502e+00, 1.57203325e-02, -1.46117793e+01], \
                [-5.17735445e-03, 1.09015604e+00, -3.14459092e+01]])

""" Camera Intrinsic Matrix"""
intrinsicMat = np.array([[520.76800036,  0.,     316.38263636],
                        [   0.,     519.22609794, 252.45681909],
                        [   0.,         0.           ,1.      ]])

invIntrinsicMat = np.linalg.inv(intrinsicMat)


extrinsicMat = [[1.00348518e+00, -4.63333524e-03, 1.34996182e-02, 7.87388251e+00], \
                [-1.37645066e-03, 1.00849377e+00, -4.07334822e-02, 2.58645282e+01], \
                [1.57344000e-02, -9.93546667e-03, -1.03098860e+00, 9.34091883e+02], \
                [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]

invExtrinsicMat = np.linalg.pinv(extrinsicMat)

class Gui(QtGui.QMainWindow):
    """ 
    Main GUI Class
    contains the main function and interfaces between 
    the GUI and functions
    """
    def __init__(self,parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        """ Main Variables Using Other Classes"""
        self.rex = Rexarm()
        self.video = Video()
        self.statemachine = Statemachine()

        """ Other Variables """
        self.last_click = np.float32([0,0])

        """ Customized events """
        self.flag_pickAndPlace = 0
        self.n_pickLocation = 2
        self.flag_event = 0

        """ Set GUI to track mouse """
        QtGui.QWidget.setMouseTracking(self,True)

        """ 
        GUI timer 
        Creates a timer and calls update_gui() function 
        according to the given time delay (27ms) 
        """
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.update_gui)
        self._timer.start(27)
        
        """
        Event timer
        Creates a timer and calls updateEvent() function 
        according to the given time delay (???ms) 
        """
        # Thread(target=self.event_one).start()
        # self._timer3 = QtCore.QTimer(self)
        # self._timer3.timeout.connect(self.event_one)
        # self._timer3.start(50)
        

        """ 
        LCM Arm Feedback timer
        Creates a timer to call LCM handler for Rexarm feedback
        """  
        self._timer2 = QtCore.QTimer(self)
        self._timer2.timeout.connect(self.rex.get_feedback)
        self._timer2.start()

        """ 
        Connect Sliders to Function
        TODO: CONNECT THE OTHER 5 SLIDERS IMPLEMENTED IN THE GUI 
        """ 
        self.ui.sldrBase.valueChanged.connect(self.sliderChange)
        self.ui.sldrShoulder.valueChanged.connect(self.sliderChange)
        self.ui.sldrElbow.valueChanged.connect(self.sliderChange)
        self.ui.sldrWrist.valueChanged.connect(self.sliderChange)
        self.ui.sldrGrip1.valueChanged.connect(self.sliderChange)
        self.ui.sldrGrip2.valueChanged.connect(self.sliderChange)

        """ Initial command of the arm to home position"""
        self.sliderChange() 
        
        """ Connect Buttons to Functions 
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        self.ui.btnUser1.setText("Reset Location")
        self.ui.btnUser1.clicked.connect(self.reset)

        self.ui.btnUser2.setText("Play Event 1")
        self.ui.btnUser2.clicked.connect(self.event_one)
        self.ui.btnUser3.setText("Play Event 2")
        self.ui.btnUser3.clicked.connect(self.event_two)
        self.ui.btnUser4.setText("Play Event 3")
        self.ui.btnUser4.clicked.connect(self.event_three)
        self.ui.btnUser5.setText("Play Event 4")
        self.ui.btnUser5.clicked.connect(self.event_four)
        self.ui.btnUser6.setText("Play Event 5")
        self.ui.btnUser6.clicked.connect(self.event_five)
        # self.ui.btnUser6.setText("Test Arm Movement")
        # self.ui.btnUser6.clicked.connect(self.testArm)
        self.ui.btnUser7.setText("Click to grab and place")
        self.ui.btnUser7.clicked.connect(self.GrabAndPlace)
        # self.ui.btnUser8.setText("Teach and Repeat")
        # self.ui.btnUser8.clicked.connect(self.teach)
        # self.ui.btnUser9.setText("Record Arm Location")
        # self.ui.btnUser9.clicked.connect(self.record)
        # self.ui.btnUser10.setText("Play Arm Locations")
        # self.ui.btnUser10.clicked.connect(self.playback)
        self.ui.btnUser10.setText("Calibrate Camera")
        self.ui.btnUser10.clicked.connect(self.simple_cal)
        self.ui.btnUser11.setText("Configure Servos")
        self.ui.btnUser12.clicked.connect(self.rex.cfg_publish_default)
        self.ui.btnUser12.setText("Emergency Stop")
        self.ui.btnUser12.clicked.connect(self.estop)
        # self.ui.btnUser11.setText("Reach DW")
        # self.ui.btnUser11.clicked.connect(self.reach_down)

    def reset(self):
        self.statemachine.reset(self.rex)

    def event_one(self):
        eventMode = 1
        print('Start playing event 1:')
        self.statemachine.runStateMachine(self.video, self.rex, eventMode)

    def event_two(self):
        eventMode = 2
        print('Start playing event 2:')
        self.statemachine.runStateMachine(self.video, self.rex, eventMode)

    def event_three(self):
        eventMode = 3
        print('Start playing event 3:')
        self.statemachine.runStateMachine(self.video, self.rex, eventMode)


    def event_four(self):
        eventMode = 4
        print('Start playing event 4:')
        self.statemachine.runStateMachine(self.video, self.rex, eventMode)

    def event_five(self):
        eventMode = 5
        print('Start playing event 5:')
        self.statemachine.runStateMachine(self.video, self.rex, eventMode)

    def GrabAndPlace(self):
        eventMode = 6
        self.video.click2grab = 1
        self.video.mouse_click_id = 0
        # self.video.wld_coord = np.zeros((2,3))
        if self.video.click2grab == 2:
            print('Pick and place the following' + self.video.wld_coord)
            # self.statemachine.runStateMachine(self.video, self.rex, eventMode)        
  
    def testArm(self):
        self.statemachine.testArmMove(self.ui, self.rex)

    def estop(self):
        self.statemachine.estop(self.rex)

    def update_gui(self):
        """ 
        update_gui function
        Continuously called by timer1 
        """

        """ Renders the video frames
            Displays a dummy image if no video available
            HINT: you can use the radio buttons to select different
            video sources like is done below
        """
        
        if(self.video.kinectConnected == 1):
            try:
                self.video.captureVideoFrame()
                self.video.captureDepthFrame()

            except TypeError:
                self.video.kinectConnected = 0
                self.video.loadVideoFrame()
                self.video.loadDepthFrame()

        if(self.ui.radioVideo.isChecked()):
            # self.ui.videoFrame
            self.ui.videoFrame.setPixmap(self.video.convertFrame())


        if(self.ui.radioDepth.isChecked()):
            self.ui.videoFrame.setPixmap(self.video.convertDepthFrame())

        """ 
        Update GUI Joint Coordinates Labels
        """
        self.ui.rdoutBaseJC.setText(str("%.2f" % (self.rex.joint_angles_fb[0]*R2D)))
        self.ui.rdoutShoulderJC.setText(str("%.2f" % (self.rex.joint_angles_fb[1]*R2D)))
        self.ui.rdoutElbowJC.setText(str("%.2f" % (self.rex.joint_angles_fb[2]*R2D)))
        self.ui.rdoutWristJC.setText(str("%.2f" % (self.rex.joint_angles_fb[3]*R2D)))
        self.ui.rdoutGrip1.setText(str("%.2f" % (self.rex.joint_angles_fb[4]*R2D)))
        self.ui.rdoutGrip2.setText(str("%.2f" % (self.rex.joint_angles_fb[5]*R2D)))

        """ 
        Mouse position presentation in GUI
        TODO: after implementing workspace calibration 
        display the world coordinates the mouse points to 
        in the RGB video image.
        """    
        x = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).x()
        y = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).y()
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)):
            self.ui.rdoutMousePixels.setText("(-,-,-)")
            self.ui.rdoutMouseWorld.setText("(-,-,-)")
        else:        
            x = x - MIN_X
            y = y - MIN_Y
            d = 0
            """ RGB, depth sensor registration"""
            if (self.ui.radioVideo.isChecked()):
                x_depth, y_depth = np.dot(rgb2depthAff,np.array([x,y,1]).T)
                if y_depth >= 480 or x_depth >= 600 or y_depth <= 0 or x_depth <= 0:
                    self.ui.rdoutMousePixels.setText("(-,-,-)")
                else:
                    d = self.video.currentDepthFrame[int(y_depth)][int(x_depth)]
            else:
                if y >= 480 or x >= 600 or y <= 0 or x <= 0:
                    self.ui.rdoutMousePixels.setText("(-,-,-)")
                else:
                    d = self.video.currentDepthFrame[int(y)][int(x)]
            """ Convert image coord  into world coord"""
            x_wld, y_wld, z_wld = 0, 0, 0
            x_wld, y_wld, z_wld = img2wld(x, y, d)
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" % (x, y, d))
            self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,%.0f)" % (x_wld, y_wld, z_wld))
        """ 
        Updates status label when rexarm playback is been executed.
        This can be extended to include other appropriate messages
        """ 
        # if(self.rex.plan_status == 1):
        #     self.playback(maxMove)
        #     self.ui.rdoutStatus.setText("Playing Back - Waypoint %d"
        #                             %(self.rex.wpt_number))
            
    

    def playEvent(self):
        """
        Run state machine
        """
        if self.flag_event != 0:
            eventStatus = False # True means assigned event is finished, otherwise False
            eventStatus = self.statemachine.runStateMachine(self.video, self.rex, self.flag_event)
        if eventStatus:
            self.flag_event = 0
            print('Event is finished, end of updating Gui')
        else:
            print('Event is not finished, end of updating Gui')

    def sliderChange(self):
        """ 
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position
        """
        self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
        self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
        self.ui.rdoutShoulder.setText(str(self.ui.sldrShoulder.value()))
        self.ui.rdoutElbow.setText(str(self.ui.sldrElbow.value()))
        self.ui.rdoutWrist.setText(str(self.ui.sldrWrist.value()))
        self.ui.rdoutGrip1.setText(str(self.ui.sldrGrip1.value()))
        self.ui.rdoutGrip2.setText(str(self.ui.sldrGrip2.value()))

        self.rex.torque_multiplier = self.ui.sldrMaxTorque.value()/100.0
        self.rex.speed_multiplier = self.ui.sldrSpeed.value()/100.0

        self.rex.joint_angles[0] = self.ui.sldrBase.value()*D2R
        self.rex.joint_angles[1] = self.ui.sldrShoulder.value()*D2R
        self.rex.joint_angles[2] = self.ui.sldrElbow.value()*D2R
        self.rex.joint_angles[3] = self.ui.sldrWrist.value()*D2R
        self.rex.joint_angles[4] = self.ui.sldrGrip1.value()*D2R
        self.rex.joint_angles[5] = self.ui.sldrGrip2.value()*D2R
        print('Joint angles from UI:')
        print(self.rex.joint_angles)
        self.rex.cmd_publish()
        print('Rexarm angle command published')
        # self.forward_kinematic()

    def mousePressEvent(self, QMouseEvent):
        """ 
        Function used to record mouse click positions for calibration 
        """
 
        """ Get mouse posiiton """
        x = QMouseEvent.x()
        y = QMouseEvent.y()

        """ If mouse position is not over the camera image ignore """
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)): return

        """ Change coordinates to image axis """
        self.last_click[0] = x - MIN_X
        self.last_click[1] = y - MIN_Y

        "Transform coordinates if cam calibration is not called"
        if (self.video.cal_flag == 0): 
            if (self.ui.radioVideo.isChecked()):
                # Obtain x, y, z in RGB and depth sensor
                x = x - MIN_X
                y = y - MIN_Y
                x_depth, y_depth = np.dot(rgb2depthAff,np.array([x,y,1]).T)
                if y_depth >= 480 or x_depth >= 600 or y_depth <= 0 or x_depth <= 0:
                    return
                z = self.video.currentDepthFrame[int(y_depth)][int(x_depth)]
                # Convert coord into world frame
                imgHom2Cam = cvtD2Z(z)* invIntrinsicMat
                camCoord = np.dot(imgHom2Cam, np.float32([x,y,1]))
                camHomCoord = np.append(camCoord,[1])
                rw_v2 = np.dot(invExtrinsicMat,camHomCoord)
                x_wld, y_wld, z_wld = rw_v2[0]/rw_v2[3], rw_v2[1]/rw_v2[3], rw_v2[2]/rw_v2[3]
                print('The mouse clicked world frame is: ')
                print(x_wld, y_wld, z_wld)


        """ get the RGB value at the cursor location"""
        # rgb_frame = cv2.cvtColor(self.video.currentVideoFrame, cv2.COLOR_RGB2BGR)
        # print(rgb_frame[y-MIN_Y][x-MIN_X])
        print('rgb: ')
        print(self.video.currentVideoFrame[y-MIN_Y][x-MIN_X]) # print RGB value at the click location
        print(self.video.currentDepthFrame[y-MIN_Y][x-MIN_X]) # print depth d at the click location

        """ Return clicked coords """
        # self.video.mouse_click_id = 0
        if (self.video.click2grab == 1):
            self.video.wld_coord[self.video.mouse_click_id] = x_wld, y_wld, z_wld
            self.video.mouse_click_id += 1

            if (self.video.mouse_click_id == 2):
                print('flag' + str(self.video.click2grab))
                self.video.click2grab = 2
                print('flag changed to ' + str(self.video.click2grab))
                self.statemachine.runStateMachine(self.video, self.rex, 6)   


        """ If calibration is called to perform"""
        if (self.video.cal_flag == 1): # 0 - not performing calibration, 1 - performing calibraiton
            
            """ Save last mouse coordinate """
            self.video.mouse_coord[self.video.mouse_click_id] = [(x-MIN_X),(y-MIN_Y)]

            """ Update the number of used poitns for calibration """
            self.video.mouse_click_id += 1

            """ Update status label text """
            self.ui.rdoutStatus.setText("Affine Calibration: Click Point %d" 
                                      %(self.video.mouse_click_id + 1))

            """ 
            If the number of click is equal to the expected number of points
            computes the affine calibration.
            
            TODO: LAB TASK: Change this code to use your workspace calibration routine
            and NOT the simple calibration function as is done now.
            """
            x = x - MIN_X
            y = y - MIN_Y

            if (self.ui.radioVideo.isChecked()):
                x, y = np.dot(rgb2depthAff,np.array([x,y,1]).T)
                
            d = self.video.currentDepthFrame[int(y)][int(x)]
            
            depth = cvtD2Z(d)
            image_coord = np.append(self.video.mouse_coord[self.video.mouse_click_id-1],[1])
            # projection_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
            
            camera_c =  depth* np.dot(invIntrinsicMat, image_coord)

            print('Current cammera cord;')
            print(camera_c)
            self.video.camera_coord[self.video.mouse_click_id - 1] = camera_c
            
            print ('Current #' + str(self.video.mouse_click_id))

            if(self.video.mouse_click_id == self.video.cal_points):
                """ 
                Update status of calibration flag and number of mouse
                clicks
                """
                self.video.mouse_click_id = 0
                
                wld2CamAffine = calAffine3D(self.video.real_coord, self.video.camera_coord)
                print(wld2CamAffine)

                wld2CamPadding = np.zeros((1,4), dtype=float)
                wld2CamPadding[0,3] = 1.
                
                self.video.extrinsicMat = np.concatenate((wld2CamAffine,wld2CamPadding),axis = 0)
                print('Before thresholed extrinsic matrix:')
                print(self.video.extrinsicMat)
                threshold = 1e-7
                self.video.invExtrinsicMat = np.linalg.pinv(self.video.extrinsicMat)

                img2CamPadding = np.zeros((1,3), dtype=float)
                img2CamPadding[0,2] = 1.
                self.video.imgHom2CamHom = np.concatenate((depth*invIntrinsicMat,img2CamPadding),axis = 0)
                self.video.CamHom2WldHom = np.linalg.pinv(self.video.extrinsicMat)
                self.video.aff_matrix = np.dot(self.video.CamHom2WldHom,self.video.imgHom2CamHom)
                
                print('Image coord is ')
               
                # self.video.aff_matrix = np.dot(extrinsicMat, np.dot(projection_mat.T,intrinsicMat))

                # self.video.aff_matrix = cv2.getAffineTransform(
                #                         self.video.mouse_coord,
                #                         self.video.real_coord)
            
                """ Updates Status Label to inform calibration is done """ 
                self.ui.rdoutStatus.setText("Waiting for input")
                print('**'*10)
                # print(self.video.aff_matrix)
                print('Extrinsic matrix:')
                print(self.video.extrinsicMat)
                print('Inverse of Extrinsic matrix:')
                print(np.linalg.pinv(self.video.extrinsicMat))
                print('Intrinsic matrix:')
                print(intrinsicMat)
                print('Inverse Intrinsic matrix:')
                print(invIntrinsicMat)
                print('camera_c:')
                print(self.video.camera_coord)
                self.viedo.cal_flag = 0

        if (self.flag_pickAndPlace == 1):
            # if(self.video.mouse_click_id == self.video.cal_points):
            pos = np.array([x_wld, y_wld, z_wld])           
            self.executePickAndPlace(pos)

    def simple_cal(self):
        """ 
        Function called when affine calibration button is called.
        Note it only chnages the flag to record the next mouse clicks
        and updates the status text label 
        """
        self.video.cal_flag = 1
        self.ui.rdoutStatus.setText("Affine Calibration: Click Point %d" 
                                    %(self.video.mouse_click_id + 1))

    """ 
    Retired code, for testing kinematic 
    """
    # def inverse_kinematic_elbowup(self, target):
        # using the x,y,z,phi from self.getCurLoc()
        # to calculate theta0-3 and then go to the
        # targeted location
        # target = self.forward_kinematic()

        # angles = self.rex.rexarm_IK(target, 1) # 1-elbow up

        # if angles is None:
        #     print('ERROR: Can not reach the given location!')
        # else:
        #     print(np.asarray(angles) * R2D)
        #     return angles

    # def inverse_kinematic_elbowdown(self, target):
    #     # target = self.forward_kinematic()
    #     angles = self.rex.rexarm_IK(target, 0) # 0-elbow down

    #     if angles is None:
    #         print('ERROR: Can not reach the given location!')
    #     else:
    #         print(np.asarray(angles) * R2D)
    #         return angles
        

    # def reach_up(self, target):
    #     angles = self.inverse_kinematic_elbowup(target)
    #     print(angles)
        # self.rex.joint_angles[0], self.rex.joint_angles[1], \
        # self.rex.joint_angles[2], self.rex.joint_angles[3] = angles
        # # self.rex.cmd_publish()

    # def reach_down(self, target):
    #     angles = self.inverse_kinematic_elbowdown(target)
    #     self.rex.joint_angles[0], self.rex.joint_angles[1], \
    #     self.rex.joint_angles[2], self.rex.joint_angles[3] = angles
    #     self.rex.cmd_publish()


    # def forward_kinematic(self):
        # self.rex.torque_multiplier = 0
        # self.rex.cmd_publish()

        # all angles are in radius
        # theta1 = self.rex.joint_angles_fb[1] # squihoulder
        # theta2 = self.rex.joint_angles_fb[2] # elbow
        # theta3 = self.rex.joint_angles_fb[3] # wrist
        # # TODO: check link with i
        # a1 = 99 # link length
        # a2 = 99
        # a3 = 140

        # baseTheta = self.rex.joint_angles_fb[0] # base angle

        # dh_table = (theta1,theta2,theta3, a1, a2, a3, baseTheta)

        # x, y, z, phi = self.rex.rexarm_FK(dh_table, None)
        # self.ui.rdoutX.setText(str("%.2f mm" %x))
        # self.ui.rdoutY.setText(str("%.2f mm" %y))
        # self.ui.rdoutZ.setText(str("%.2f mm" %z))
        # self.ui.rdoutT.setText(str("%.2f deg" % (phi*R2D)))

        # return (x,y,z,phi)
    ""
    """
    Retired code, for teach and repeat, and pick and place
    """
    #  def teach(self):
    #     self.rex.torque_multiplier = 0
    #     self.rex.plan = []
    #     self.rex.plan_status = 0
    #     self.rex.wpt_number = 0
    #     self.rex.wpt_total = 0
    #     self.rex.cmd_publish()
    #     print(self.rex.wpt_number, self.rex.plan)

    # def record(self):
    #     """Record the waypoints"""
    #     angles = [self.rex.joint_angles_fb[0], self.rex.joint_angles_fb[1], \
    #                 self.rex.joint_angles_fb[2], self.rex.joint_angles_fb[3], \
    #                 self.rex.joint_angles_fb[4]]
    #     self.rex.plan.append(angles)
    #     self.rex.wpt_total += 1
        
    #     for i in range(len(angles)):
    #         print(angles[i] * R2D)
    #     print("---------") 
        
    #     np.save('waypoints', self.rex.plan)

    #     self.ui.rdoutStatus.setText("Recording: point %d" \
    #         %(self.rex.wpt_total))


    # def playback(self, maxMove):
    #     """let arm touch each point recorded in advance"""
    #     print('start playing back')

    #     # when playback is executed, the recording is done
    #     self.rex.plan = np.load('waypoints.npy')

    #     print(self.rex.plan)
    #     #repeat the motion of the arm through the recorded waypoints
    #     self.rex.torque_multiplier = 0.3
    #     self.rex.speed_multiplier = 0.4
    #     self.rex.plan_status = 1

    #     all_motor_p = self.planMotion(interp_num = 5, interp_time = 1)

    #     # start in idle state
    #     curState = self.statemachine.idle()
         
    #     for i in range(len(all_motor_p[0])):

    #         targetP = all_motor_p[:,i]
    #         self.rex.wpt_number += 1

    #         # command to move to the target position
    #         cmdState = 1
    #         # counter is used to check the number of moves to target position
    #         counter = 0
    #         # if it's not in emergency stop state
    #         if (curState != -1):
    #             print('======')
    #             self.runStateMachine(targetP, cmdState)
    #             curState = self.statemachine.getCurrentState()
    #             print(curState)
    #             inPos, idxInPose = self.statemachine.checkPose(self.rex, targetP, epsilon=0.05)

    #             if not inPose and counter <= maxMove:
    #                 self.runStateMachine(targetP, cmdState)
    #                 counter += 1
    #             else:
    #                 curState = self.statemachine.estop(self.rex)
    #                 print('ERROR: Robot stopped before reaching to the target position')
    #                 print('Command repeated '+ str(counter) + ' times')
    #                 print('Motors in position' + str(idxInPose))
            
    #         else:
    #             break
    #             # self.statemachine.estop(self.rex)
                
    # def simple_picknplace(self):
    #     self.flag_pickAndPlace = 1
    #     print('Pick and place flag = ' + str(self.flag_pickAndPlace))
    
    # def executePickAndPlace(self, posList):
    #     # Do all the motion
    #     print('The reqiured end effector position is: ')
    #     print(posList)
    #     wldx,wldy,wldz = posList
    #     phi = - 45 * D2R
    #     target = (wldx,wldy,wldz,phi)
    #     print('The converted joint angles are:')
    #     self.reach_up(target)
    #     print('Motion finished')
    #     # self.flag_pickAndPlace = 0
    #     # print('Now, Pick and place flat = ' + str(self.flag_pickAndPlace))



    # def planMotion(self, interp_num = 5, interp_time = 2):
    #     all_motor_p = []
    #     all_motor_v = []


    #     p_mat = self.rex.plan
    #     v_mat = getDerivative(p_mat, interp_time)
    #     a_mat = getDerivative(v_mat,interp_time)

    #     # q0, qf, t0, tf, v0, vf, ac0, acf

    #     for i in range(len(self.rex.plan[0])): # for each motor
    #         current_p_list = []
    #         current_v_list = []
    #         for j in range(len(self.rex.plan)-1): # for each position
    #             currentP = p_mat[j][i]
    #             currentV = v_mat[j][i]
    #             currentA = a_mat[j][i]
    #             nextP = p_mat[j+1][i]
    #             nextV = v_mat[j+1][i]
    #             nextA = a_mat[j+1][i]

    #             qd, qv = quinticInter(currentP,nextP,0,interp_time, \
    #                 currentV, nextV, currentA, nextA, interp_num)

    #             current_p_list.append(qd)
    #             current_v_list.append(qv)

    #         current_p_list = [item for sublist in \
    #                             current_p_list for item in sublist]
    #         current_v_list = [item for sublist in \
    #                             current_v_list for item in sublist]
    #         all_motor_p.append(current_p_list)
    #         all_motor_v.append(current_v_list)

    #     all_motor_p = np.asarray(all_motor_p)
    #     all_motor_v = np.asarray(all_motor_v)

    #     return all_motor_p
"""main function"""
def main():
    app = QtGui.QApplication(sys.argv)
    app_window = Gui()
    app_window.show()
    sys.exit(app.exec_())
 
if __name__ == '__main__':
    main()

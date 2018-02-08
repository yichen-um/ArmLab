import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt
import freenect
from cube_detection import *
from transformCoord import *
from color_threshold import *
class Video():
    def __init__(self):
        self.currentVideoFrame=np.array([])
        self.currentDepthFrame=np.array([])
        self.kinectConnected = 1 #expect kinect to be available

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        
        """ 
        Calibration Variables 
        
        Currently this takes three points to perform a simple calibration.
        The simple calibration only works for points on the board and uses
        an affine transform to perform the calibration.

        To use: 
            1. Click on simple calibration button
            2. Click the center of the arm
            3. Click the upper right corner of the board
            4. Click the upper left corner of the board

        Note that OpenCV requires float32 arrays

        TODO: Modify these to enable a robust 3D calibration

        """
        # self.cal_points = 4 # number of points for the simple calibration
        # self.real_coord = np.float32([[305,-305,0],[0,-305,38],[-305,-305,38*2], \
        #                              [-305,0,38*3],[-305,305,0],[0, 305,  38], \
        #                              [305,305,38*2],[305,0,38*3],[0,  0, 130]])

        # self.mouse_coord = np.float32([[0.0, 0.0],[0.0, 0.0],[0.0,0.0],\
        #                                [0.0, 0.0],[0.0, 0.0],[0.0,0.0],\
        #                                [0.0, 0.0],[0.0, 0.0],[0.0,0.0]])

        # self.camera_coord = np.float32([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],\
        #                                 [0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],\
        #                                 [0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])

        self.cal_points = 16 # number of points for the simple calibration
        # self.real_coord = np.zeros((self.cal_points, 3))
        self.real_coord = np.array([
        [200, -200, 0],
        [0, -200, 38],
        [-200, -200, 76],
        [-200, 0, 114],
        [-200, 200, 0],
        [0, 200, 38],
        [200, 200, 76],
        [200, 0, 114],
        [100, -100, 152],
        [0, -100, 38],
        [-100, -100, 76],
        [-100, 0, 114],
        [-100, 100, 152],
        [0, 100, 38],
        [100, 100, 76],
        [100, 0, 114]])

        self.mouse_coord = np.zeros((self.cal_points,2))
        self.camera_coord = np.zeros((self.cal_points, 3))
        self.wld_coord = np.zeros((2,3))

        self.click2grab = 0
        self.playGrabPlace = 0
        
        self.mouse_click_id = 0
        self.cal_flag = 0 # 0 - Not performing calibration, 1 - performing calibration
        self.aff_matrix = np.empty((2,3)) # affine matrix for simple calibration
        self.imgHom2CamHom = np.empty((4,3))
        self.CamHom2WldHom = np.empty((4,4))
        self.extrinsicMat = np.empty((4,4))
        self.invExtrinsicMat = np.empty((4,4))
    
    def blockDetector(self):
        """ detect the coords (center) of blocks from depth img
        and then find out the color """
        tolerance = 40

        # Capture current frame
        self.captureDepthFrame() # hook the depth frame first
        self.captureVideoFrame()
        depth_img = self.currentDepthFrame
        video_img = self.currentVideoFrame

        # Detect block
        color_lst = []
        coords_lst = []
        """ filter the depth image to get blocks"""
        upper_gray = np.array([710])
        lower_gray = np.array([600])
        mask = cv2.inRange(depth_img, lower_gray, upper_gray)
        res = cv2.bitwise_and(depth_img, depth_img, mask=mask)
        # # """smooth"""
        # kernel = np.ones((5,5),np.uint8)
        # smoothed = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
        # print(np.sum(mask))


        """ find contours """
        _, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if 300 < cv2.contourArea(cnt) < 1500:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                
                for i in range(len(box)):
                    cord = np.append(box[i],[1])
                    box[i] = np.matmul(depth2camera, cord.T)[:-1]
                M = cv2.moments(box)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                color = getColor(video_img, (cx, cy))
                # cv2.putText(video_img, color, (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))
                # cv2.circle(video_img,(cx,cy), 2, (0,0,255), -1)
                # cv2.drawContours(video_img,[box],0,(255,255,0),2)
                cx_depth, cy_depth = np.dot(rgb2depthAff,np.array([cx,cy,1]).T)
                cd = self.currentDepthFrame[int(cy_depth)][int(cx_depth)]
                cz = cvtD2Z(cd)
                cx_wld, cy_wld, cz_wld = img2wld(cx, cy, cd)

                color_lst.append(color)
                coords_lst.append([cx_wld, cy_wld, cz_wld])
                # print('For current block , the depth coords and world coords are: ')
                # print(cx, cy, cd)
                # print(cx_wld, cy_wld, cz_wld)
                # print('\n')
        return color_lst, coords_lst
        # return np.asarray(color_lst), np.asarray(coords_lst)

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        self.currentVideoFrame = freenect.sync_get_video()[0]
        # self.currentVideoFrame = self.blockDetector()


    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        self.currentDepthFrame = freenect.sync_get_depth()[0]
    
    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for QtGui  """
        try:
            img=QtGui.QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QtGui.QImage.Format_RGB888
                             )

            return QtGui.QPixmap.fromImage(img)
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for QtGui  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)

            img=QtGui.QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QtGui.QImage.Format_RGB888
                             )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None


    def loadCalibration(self):
        """
        TODO (OPTIONAL):
        Load camera distortion calibration from file and applies to the image:
        Look at camera_cal.py final lines to see how that is done
        This is optional, you do not need to implement distortion correction
        """
        pass

def color_mask (frame, chanel_boundary) :
    ''' 
    channle bpundary is a list of boundaries for each color channel
    channel boundaries sequence h, s, v, l,, b, g, r
    '''
    # print(chanel_boundary)
    h = chanel_boundary[0]
    s = chanel_boundary[1]
    v = chanel_boundary[2]
    l = chanel_boundary[3]
    r = chanel_boundary[4]
    g = chanel_boundary[5]
    b = chanel_boundary[6]

    h_binary = h_threshold(frame, h)
    s_binary = s_threshold(frame, s)
    v_binary = v_threshold(frame, v)
    l_binary = l_threshold(frame, l)
    r_binary = r_threshold(frame, r)
    g_binary = g_threshold(frame, g)
    b_binary = b_threshold(frame, b)
       
    mask = np.zeros_like(h_binary)
    # print(mask.shape)
    mask[(h_binary == 1) & (s_binary == 1) & (v_binary == 1) & (l_binary == 1) & (r_binary == 1) & (g_binary == 1) & (b_binary == 1)] = 1

    blurred_mask = gaussian_blur(mask, 5)
    return mask

def getColor(frame, coords):
    """ get the color at each coord, use majority vote"""
    cx, cy = coords

    color = ['yellow', 'orange', 'blue', 'black', 'green', 'red', 'pink', 'violet']
    # color boundaries for different channel to realize roboust detection
    # channle sequence h, s, v, l, r, g, b
    color_boundaries = [
        ([(20, 30),(0, 255),(0, 255),(0, 255),(0, 255),(150, 255),(0, 255)]), # yellow
        ([(10, 40),(0, 255),(0, 255),(0, 255),(150, 255),(60, 140),(0, 50)]), # orange
        ([(80, 130),(0, 255),(0, 255),(0, 255),(10, 100),(0, 255),(100, 255)]), # blue
        ([(0, 255),(0, 80),(0, 90),(0, 100),(0, 255),(0, 255),(0, 255)]), # black
        ([(40, 100),(0, 255),(0, 255),(0, 255),(0, 150),(50, 255),(0, 255)]), # green
        ([(20, 30),(0, 255),(0, 255),(0, 255),(0, 255),(150, 255),(0, 255)]), # red
        ([(120, 255),(0, 255),(0, 255),(120, 255),(180, 255),(0, 150),(0, 255)]), # pink
        ([(100, 140),(0, 255),(0, 255),(0, 255),(100, 255),(0, 255),(100, 255)])  # purple
    ]
 
    color_sum = np.zeros(8)
    
    for color_idx in range(len(color_boundaries)):
        # print(color_idx)
        current_color = color[color_idx]
        boundary = color_boundaries[color_idx]
        # print(boundary)
        mask = color_mask(frame, boundary)
        # pixels around centroid
        offset = 20
        patch_sum = np.sum(mask[cy-1-offset:cy-1+offset, cx-1-offset:cx-1+offset])
        # print(patch_sum)
        color_sum[color_idx] = patch_sum
        
    Total_points = 20 * 20
    color_prob = color_sum / Total_points
    # print(color_prob)
    idx, = np.where( color_prob == np.max(color_prob))
    # print(idx)
    # print(color[idx[0]])
    return color[idx[0]]
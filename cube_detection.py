# use HSV for segmentiong images based on the color of objects

import cv2
import numpy as np
from transformCoord import *

# depth2camera = np.array([[9.120e-01, -1.857e-02, 1.367e+01],[1.685e-02, 9.189e-01,2.793e+01],[0, 0, 1]])
depth2camera = np.array([[9.10970078e-01,-1.31403920e-02,1.29690344e+01],\
                        [4.32338504e-03, 9.17066718e-01, 2.89525597e+01],[0, 0, 1]])

def getColor(frame, coords, tolerance=40):
    """ get the color at each coord, use majority vote"""
    cx, cy = coords

    # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # use rgb img
    # frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # use rgb img
    
    # h_channel = frame_hsv[;,:,0]
    # s_channel = frame_hsv[;,:,1]
    # r_channel = frame_hsv[;,:,0]
    # g_channel = frame_hsv[;,:,1]
    # b_channel = frame_hsv[;,:,2]


    # yellow_mask = np.zeros_like(h_channel)
    # orange_mask = np.zeros_like(h_channel)
    # black_mask = np.zeros_like(h_channel)
    # green_mask = np.zeros_like(h_channel)
    # blue_mask = np.zeros_like(h_channel)
    # pink_mask = np.zeros_like(h_channel)
    # red_mask = np.zeros_like(h_channel)
    # purple_mask = np.zeros_like(h_channel)

    # yellow = np.array([30, ])
    # orange = np.array([20, ])
    # black  = np.array([51,60,72])
    # green  = np.array([90])
    # blue   = np.array([160])
    # pink   = np.array([])
    # red    = np.array([])
    # purple = np.array([])
    

    # yellow_tol = 5
    # orange_tol = 5
    # green_tol  =30
    # blue _tol = 30

    # y_mask_h = cv2.inRange(frame, lower, upper)

    #     color_masks = [
    #     ('yellow', yellow_mask),
    #     ('orange', yellow_mask),
    #     ('pink', yellow_mask),
    #     ('black',yellow_mask),
    #     ('red',yellow_mask),
    #     ('purple',yellow_mask),
    #     ('green',yellow_mask),
    #     ('blue',yellow_mask)
    # ]
   
    # for (color, mask) in color_masks:
    #     output = cv2.bitwise_and(frame, frame, mask = mask)
    #     if np.all(output[cy][cx]):
    #         return color

    # HSV
    # yellow = np.array([247,198,8])
    # orange = np.array([199,90,17])
    # pink   = np.array([204,56,93])
    # black  = np.array([33,26,27])
    # red    = np.array([148,34,41])
    # purple = np.array([132,82,115])
    # green  = np.array([101,125,99])
    # blue   = np.array([80,96,149])
    

    # RGB
    yellow = np.array([213,217,220])
    orange = np.array([199,90,17])
    pink   = np.array([218,86,129])
    black  = np.array([51,60,72])
    red    = np.array([205,198,195])
    purple = np.array([113,78,134])
    green  = np.array([138,170,180])
    blue   = np.array([85,94,160])  

    color_boundaries = [
        ('yellow', yellow-tolerance,yellow+tolerance),
        ('orange', orange-tolerance,orange+tolerance),
        ('pink', pink-tolerance, pink+tolerance),
        ('black',black-tolerance, black+tolerance),
        ('red',red-tolerance, red+tolerance),
        ('purple',purple-tolerance, purple+tolerance),
        ('green',green-tolerance, green+tolerance),
        ('blue',blue-tolerance, blue+tolerance)
    ]

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # use rgb img
    # frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # use HSV img
   

    for (color, lower, upper) in color_boundaries:
        mask = cv2.inRange(frame, lower, upper)
        output = cv2.bitwise_and(frame, frame, mask = mask)
        if np.all(output[cy][cx]):
            return color


def detectBlocks(depth_img, video_img, tolerance):
    """ detect the coords (center) of blocks from depth img
    and then find out the color """

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
            print(cx, cy)
            color = getColor(video_img, (cx, cy), tolerance)
            # cv2.putText(video_img, color, (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))
            # cv2.circle(video_img,(cx,cy), 2, (0,0,255), -1)
            # cv2.drawContours(video_img,[box],0,(255,255,0),2)
            color_lst.append(color)
            coords_lst.append([cx, cy])

    return color_lst, coords_lst
    # return video_img

    # cv2.namedWindow("depth img ex0",cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('depth img ex0', smoothed)

    # cv2.imshow('corrected img', result)

    # while True:
    #     ch = 0xff & cv2.waitKey(10)
    #     if ch == 0x1B:
    #         break
    # cv2.destroyAllWindows()


def detect_block(self):
    color_lst, coord_lst = self.video.blockDetector()
    for i in range(len(color_lst)):
        print('#%d color: %s, coords: %s' %(i, color_lst[i], coord_lst[i]))
        cx, cy =  coord_lst[i] 
        imgHomCoord = np.array([cx,cy,1]).T
        cx_depth, cy_depth = np.dot(rgb2depthAff,imgHomCoord)
        cz_depth = self.video.currentDepthFrame[int(cy_depth)][int(cx_depth)]
        imgHom2Cam = cvtD2Z(cz_depth)* invIntrinsicMat
        camCoord = np.dot(imgHom2Cam, imgHomCoord)
        camHomCoord = np.append(camCoord,[1])


        self.video.extrinsicMat = extrinsicMat

        
        wldHomCoord = np.dot(np.linalg.pinv(self.video.extrinsicMat),camHomCoord)
        
        wldx, wldy, wldz =  wldHomCoord[0]/ wldHomCoord[3], wldHomCoord[1]/ wldHomCoord[3], wldHomCoord[2]/ wldHomCoord[3]

        phi = -90*D2R
 

        target = (wldx,wldy,wldz,phi)
        print(target)

        # TODO:
        # Check x,y,z correspondance with IK
        # self.reach_up(target)
        
if __name__ == '__main__':
    # depth_img = cv2.imread('./example/ex0_depth8.png')
    # video_img = cv2.imread('./example/ex0_bgr.png')
    depth_img = cv2.imread('./example/ex0_depth8.png')
    video_img = cv2.imread('./example/ex0_bgr.png')
    tolerance = 40
    color, cx, cy = detectBlocks(depth_img, video_img, tolerance)
    print(color, cx, cy)



import cv2
import numpy as np


depth2camera = np.array([[9.10970078e-01,-1.31403920e-02,1.29690344e+01],[4.32338504e-03,9.17066718e-01,2.89525597e+01],[0, 0, 1]])
''' 
helper functions to detect color
'''
def h_threshold(image, h_thresh=(0, 255)):
    # 1) Convert to HLS and and separate the h channel
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    h_channel = hls[:,:,0]
    
    h_binary = np.zeros_like(h_channel)
    h_binary[(h_channel >= h_thresh[0]) & (h_channel <= h_thresh[1])] = 1
    return h_binary

def l_threshold(image, l_thresh=(0, 255)):
    # 1) Convert to HLS and and separate the l channel
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    l_channel = hls[:,:,1]
    
    l_binary = np.zeros_like(l_channel)
    l_binary[(l_channel >= l_thresh[0]) & (l_channel <= l_thresh[1])] = 1
    return l_binary

def s_threshold(image, s_thresh=(0, 255)):
    # 1) Convert to HLS and and separate the S channel
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    s_channel = hls[:,:,2]
    
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
    return s_binary

def v_threshold(image, v_thresh=(0, 255)):
    # 1) Convert to HSV and and separate the S channel
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    v_channel = hsv[:,:,2]
    
    v_binary = np.zeros_like(v_channel)
    v_binary[(v_channel >= v_thresh[0]) & (v_channel <= v_thresh[1])] = 1
    return v_binary

def b_threshold(image, b_thresh=(0, 255)):
    # 1) Input image is in BGR
    
    b_channel = image[:,:,0]
    
    b_binary = np.zeros_like(b_channel)
    b_binary[(b_channel >= b_thresh[0]) & (b_channel <= b_thresh[1])] = 1
    return b_binary

def g_threshold(image, g_thresh=(0, 255)):
    # 1) Input image is in BGR
    
    g_channel = image[:,:,1]
    
    g_binary = np.zeros_like(g_channel)
    g_binary[(g_channel >= g_thresh[0]) & (g_channel <= g_thresh[1])] = 1
    return g_binary

def r_threshold(image, r_thresh=(0, 255)):
    # 1) Input image is in BGR
    
    r_channel = image[:,:,2]
    
    r_binary = np.zeros_like(r_channel)
    r_binary[(r_channel >= r_thresh[0]) & (r_channel <= r_thresh[1])] = 1
    return r_binary

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
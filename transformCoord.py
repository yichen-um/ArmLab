# Transform coordinate
import numpy as np


# class transCoord():

""" Pyxel Positions of image in GUI """
MIN_X = 310
MAX_X = 950

MIN_Y = 30
MAX_Y = 510

""" Affine transformation to convert RGB xy into depth xy """
rgb2depthAff = np.array([[1.09741502e+00, 1.57203325e-02, -1.46117793e+01], \
                [-5.17735445e-03, 1.09015604e+00, -3.14459092e+01]])

depth2rgbAff = np.array([[9.10970078e-01, -1.31403920e-02, 1.29690344e+01], \
                [4.32338504e-03, 9.17066718e-01, 2.89525597e+01]])

""" Camera Intrinsic Matrix"""
matIn = np.array([[520.76800036,  0.,     316.38263636],
                  [0.,     519.22609794, 252.45681909],
                  [0.,         0.           ,1.      ]])

matEx = [[1.00348518e+00, -4.63333524e-03, 1.34996182e-02, 7.87388251e+00], \
         [-1.37645066e-03, 1.00849377e+00, -4.07334822e-02, 2.58645282e+01], \
         [1.57344000e-02, -9.93546667e-03, -1.03098860e+00, 9.34091883e+02], \
         [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]


matInInv = np.linalg.inv(matIn)
matExInv = np.linalg.pinv(matEx)

def cvtD2Z(d):
    z = 0.0044 *d*d - 3.898 *d + 1452.5
    return z

def img2wld(x, y, d):
    # Check if pixel is out of boundary
    x_wld, y_wld, z_wld = 0, 0, 0
    # Convert image coord into world coord
    imgHom2Cam = cvtD2Z(d)* matInInv
    coordCam = np.dot(imgHom2Cam, np.float32([x, y, 1]))
    coordCamHom = np.append(coordCam,[1])
    coordWldHom = np.dot(matExInv,coordCamHom)
    x_wld, y_wld, z_wld = coordWldHom[0]/coordWldHom[3],coordWldHom[1]/coordWldHom[3],coordWldHom[2]/coordWldHom[3]
    return x_wld, y_wld, z_wld


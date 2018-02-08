import numpy as np
from quintic import calAffine
nData = 12
coordRGB = np.zeros((nData, 2))
coordDep = np.zeros((nData, 2))

# Old measurement
rgb = np.array([[427.0, 150.0],[309.0, 136.0], [201.0, 264.0], \
                [199.0, 383.0],[314.0, 389.0]])
depth = np.array([[455.0, 125.0],[327.0, 109.0],[204.0, 257.0],\
                [204.0, 382.0],[334.0, 389.0]])

# new measurement, 2018-02-01
coordRGB[0, :] = [144, 93]
coordRGB[1, :] = [140, 448]
coordRGB[2, :] = [204, 211]
coordRGB[3, :] = [203, 270]
coordRGB[4, :] = [201, 325]
coordRGB[5, :] = [321, 155]
coordRGB[6, :] = [318, 387]
coordRGB[7, :] = [435, 211]
coordRGB[8, :] = [434, 268]
coordRGB[9, :] = [436, 330]
coordRGB[10, :] = [498, 93]
coordRGB[11, :] = [499, 450]

coordDep[0, :] = [147, 70]
coordDep[1, :] = [147, 456]
coordDep[2, :] = [211, 196]
coordDep[3, :] = [210, 260]
coordDep[4, :] = [210, 326]
coordDep[5, :] = [340, 133]
coordDep[6, :] = [340, 389]
coordDep[7, :] = [468, 197]
coordDep[8, :] = [470, 259]
coordDep[9, :] = [471, 325]
coordDep[10, :] = [530, 69]     
coordDep[11, :] = [538, 456]

rgb2depth = calAffine(coordRGB,coordDep)
depth2rgb = calAffine(coordDep,coordRGB)

print(depth2rgb)

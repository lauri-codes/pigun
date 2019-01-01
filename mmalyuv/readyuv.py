import numpy as np
import matplotlib.pyplot as plt
# import cv2

width = 640
height = 320

stream = open('test.yuv', 'rb')

# Seek to the fourth frame in the file
stream.seek(int(4 * width * height * 1.5))

# Calculate the actual image size in the stream (accounting for rounding
# of the resolution)
fwidth = (width + 31) // 32 * 32
fheight = (height + 15) // 16 * 16

# Load the Y (luminance) data from the stream
Y = np.fromfile(stream, dtype=np.uint8, count=fwidth*fheight).\
reshape((fheight, fwidth))

# Load the UV (chrominance) data from the stream, and double its size
U = np.fromfile(stream, dtype=np.uint8, count=(fwidth//2)*(fheight//2)).\
reshape((fheight//2, fwidth//2)).\
repeat(2, axis=0).repeat(2, axis=1)
V = np.fromfile(stream, dtype=np.uint8, count=(fwidth//2)*(fheight//2)).\
reshape((fheight//2, fwidth//2)).\
repeat(2, axis=0).repeat(2, axis=1)
# Stack the YUV channels together, crop the actual resolution, convert to
# floating point for later calculations, and apply the standard biases
YUV = np.dstack((Y, U, V))[:height, :width, :].astype(np.float)
YUV[:, :, 0]  = YUV[:, :, 0]  - 16   # Offset Y by 16
YUV[:, :, 1:] = YUV[:, :, 1:] - 128  # Offset UV by 128
# YUV conversion matrix from ITU-R BT.601 version (SDTV)
# Note the swapped R and B planes!
#              Y       U       V
M = np.array([[1.164,  2.017,  0.000],    # B
                    [1.164, -0.392, -0.813],    # G
                                [1.164,  0.000,  1.596]])   # R
# Take the dot product with the matrix to produce BGR output, clamp the
# results to byte range and convert to bytes
BGR = YUV.dot(M.T).clip(0, 255).astype(np.uint8)
# Display the image with OpenCV
plt.imshow(Y, cmap='gray')
plt.show()
# cv2.imshow('image', BGR)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

from PIL import Image
import numpy as np
import matplotlib.pyplot as plt


im_frame = Image.open('img2.png')
im_frame = np.array(im_frame.convert("YCbCr"))
y = im_frame[:, :, 0].astype(np.uint8)

# Test data
# y = np.zeros((2, 7), dtype=np.uint8)
# y[:, 0] = 5
# y[0:2, 0:2] = 5
# y = y[0:10, 0:10]
# print(y)
# print(y.dtype)
print(y.shape)

# y = y[60:80, 140:160]
plt.plot(y[249, :])
plt.imshow(y, cmap="gray")
plt.show()

# with open('ybinary_1280x720.bin', 'wb') as f:
    # y.tofile(f)

# with open('ybinary_1280x720.bin', 'rb') as f:
# a = np.fromfile('ybinary_1280x720.bin', dtype=np.uint8)
# print(a.dtype)
# print(a.shape)

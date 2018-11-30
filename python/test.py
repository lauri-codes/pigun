
from io import BytesIO
from time import sleep

from picamera import PiCamera
import numpy
import scipy
from scipy.ndimage import gaussian_filter
from PIL import Image
import time


camera = PiCamera()
#camera.start_preview()
#sleep(1)
#camera.color_effects = (128, 128)

camera.resolution = (640, 480)
camera.framerate = 60

output = numpy.zeros((480 * 640 * 3,), dtype=numpy.uint8)
luma = None

sleep(1)
print("starting...")

while True:
    
    start = time.time()
    
    camera.capture(output, 'yuv', use_video_port=True)
    luma = numpy.reshape(output[0 : 640*480], (480,640))
    # do more processing?
    luma = gaussian_filter(luma, 10)
    
    end = time.time()
    print(end-start)
    # save snapshot
    numpy.save("snapshot.npy",luma)
    
#img = Image.open(stream).convert('L')
#mat = numpy.asarray(img)#.astype(numpy.float32)

#print(mat)



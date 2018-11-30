
from io import BytesIO
from time import sleep

from picamera import PiCamera
import numpy
import scipy
from scipy.ndimage import gaussian_filter
from scipy import ndimage

from PIL import Image
import time


camera = PiCamera()
#camera.start_preview()
#sleep(1)
#camera.color_effects = (128, 128)

camera.resolution = (320, 240)
camera.framerate = 60

output = numpy.zeros((320 * 240 * 3,), dtype=numpy.uint8)
luma = None

sleep(1)
print("starting...")
times = numpy.zeros((200))

start = time.time()

for i in range(100):
    
    camera.capture(output, 'yuv', use_video_port=True)
    luma = numpy.reshape(output[0 : 320 * 240], (240,320))
    # do more processing?
    #luma = gaussian_filter(luma, 1)
    mask = numpy.where(luma>0.75, 1, 0)
    
    label_im, nb_labels = ndimage.label(mask)
    
    #print(end-start)
    #times[i] = end-start
    # save snapshot
    numpy.save("snapshot.npy",luma)
    print("done")


end = time.time()
print((end-start)/100.0)

#print(numpy.mean(times))

#img = Image.open(stream).convert('L')
#mat = numpy.asarray(img)#.astype(numpy.float32)

#print(mat)



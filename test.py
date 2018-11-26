
from io import BytesIO
from time import sleep

from picamera import PiCamera
import numpy
import scipy
from PIL import Image


camera = PiCamera()
#camera.start_preview()
#sleep(1)
camera.color_effects = (128, 128)

for res in [1024]:

    imgID = 0
    camera.resolution = (800, 600)

    for repo in range(5):
        stream = BytesIO()
        camera.capture(stream, 'png')
        stream.seek(0)

        #with open("./testimgs/test.{}.{}.png".format(res,imgID), "wb") as fout:
        #    fout.write(stream.read())
        imgID += 1
        #stream.flush()
        print("ready...")
        #sleep(2)
        print("go!")

#img = Image.open(stream).convert('L')
#mat = numpy.asarray(img)#.astype(numpy.float32)

#print(mat)



from picamera import PiCamera
import time
import datetime

ts = time.time()
st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')

camera = PiCamera()
camera.resolution = (1280, 720)
camera.capture("../testimgs/resolutions/test{}.png".format(st))

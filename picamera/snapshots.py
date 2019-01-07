from picamera import PiCamera
import time
import datetime

ts = time.time()
st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')

camera = PiCamera()
camera.capture("../testimgs/resolutions/test{}.png".format(st))

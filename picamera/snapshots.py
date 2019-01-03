from picamera import PiCamera

camera = PiCamera()
resolutions = [(1920, 1440), (1280, 960), (1024, 768), (800, 600), (640, 480)]

for res in resolutions:
    camera.resolution = res
    camera.capture("../testimgs/resolutions/test_{}x{}.png".format(res[0], res[1]))

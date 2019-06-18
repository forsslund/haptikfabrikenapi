from H3DInterface import *
import time

fps = 0
t = 0



def traverseSG():
  global fps
  global t
  if time.time() > t +0.5:
    t = time.time()
    print "fps: ",fps/0.5
    fps = 0

    # Get haptic rate
    di = getActiveDeviceInfo()
    if di:
      hd = di.device.getValue()[0]
      print "Haptic rate: ", hd.hapticsRate.getValue()
  fps = fps + 1



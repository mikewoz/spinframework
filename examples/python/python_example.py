import time
import sys
import os

from spinFramework import *
#import spinFramework.*



class spinClient(pySpinContext):
	def sceneCallback(self, d):
		print "in python sceneCallback"
		print d

# old way before singleton:
#spin = spinClient(pySpinContext.mode.LISTENER_MODE)

spin = spinClient()


spin.start()

spin.sendSceneMessage(["createNode", "foo", "GroupNode"])
spin.sendSceneMessage(["debug"])

spin.sendNodeMessage(["foo", "debug"])

time.sleep(2)

spin.sendSceneMessage(["clear"])
spin.sendSceneMessage(["debug"])

time.sleep(5)



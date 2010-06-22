import spin
import libSPINPyWrap
import math

class Script( spin.ScriptBase ):

    c = 1
    angle = 0
    radius = 5
    def __init__(self, id):
        print "orbit init!"
        self.c = 1000
        self._nodeID = id

    def run(self): 
        if self.angle % 10 == 0:
            print "running: _nodeID = ", self._nodeID, "angle = " , self.angle
        self.angle += 0.05
        x = self.radius * math.cos(self.angle);
        y = self.radius * math.sin(self.angle);
        method = "setTranslation " + `x` + " " + `y` + " 0.0"
        #print "the method is: ", method ... cascadeEvents = 1: setTranslation can trigger more event scripts! like hotspot.py
        libSPINPyWrap.callback(self._nodeID, "sfff", method, 1)

print "orbit module loaded."

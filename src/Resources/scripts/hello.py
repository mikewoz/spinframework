import spin
import libSPINPyWrap
import math

class Script( spin.ScriptBase ):

    c = 1
    angle = 0
    radius = 5
    def __init__(self, id):
        print "why hello there!"
        self.c = 1000
        self._nodeID = id
        print "script for node '", self._nodeID, "' initialized"

    def run(self): 
        if self.angle % 10 == 0:
            print "running: _nodeID = ", self._nodeID, "angle = " , self.angle
        self.angle += 0.001
        x = self.radius * math.cos(self.angle);
        y = self.radius * math.sin(self.angle);
        method = "setTranslation " + `x` + " " + `y` + " 0.0"
        #print "the method is: ", method
        libSPINPyWrap.callback(self._nodeID, "sfff", method, 0)
        
        method = "setDamping 0.234"
        libSPINPyWrap.callback(self._nodeID, "sf", method)

        ret = libSPINPyWrap.callback(self._nodeID, "s", "getTranslation", 0)
        print "ret?? = ", ret.getVector()


print "module loaded."

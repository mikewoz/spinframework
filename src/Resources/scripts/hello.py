import spin
import spinframework
import math

__spin_behavior_class__ = "Script"

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
        spinframework.callback(self._nodeID, "setTranslation", [x, y, 0.0], 0)
        
        ret = spinframework.callback(self._nodeID, "getTranslation", [], 0)
        print "ret?? = ", ret.getVector()


print "module loaded."

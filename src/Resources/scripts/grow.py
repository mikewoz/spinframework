import spin
import libSPINPyWrap
import math

class Script( spin.ScriptBase ):

    scale = 1
    def __init__(self, id):
        print "grow init!"
        self.scale = 1.0
        self._nodeID = id

    def run(self, eventMethod, eventTypes, eventArgs): 
        print "grow.run( ", eventMethod, ", [", eventTypes , "], [", eventArgs , "]"
        self.scale += 0.1
        method = "setScale " + `self.scale` + " " + `self.scale` + " " + `self.scale`
        libSPINPyWrap.callback(self._nodeID, "sfff", method, 0)

print "grow module loaded."

import spin
import libSPINPyWrap
import math

__spin_behavior_class__ = "Script"

class Script( spin.ScriptBase ):

    scale = 1
    def __init__(self, id):
        print "grow init!"
        self.scale = 1.0
        self._nodeID = id

    def run(self, eventMethod, eventArgs): 
        print "grow.run( ", eventMethod, eventArgs, " )"
        self.scale += 0.1
        libSPINPyWrap.callback(self._nodeID, "setScale", [self.scale, self.scale, self.scale], 0)

print "grow module loaded."

import spin
import libSPINPyWrap
import math

def isInside(x, y, z, box):
    if x > box[0] and x < box[1] and y > box[2] and y < box[3] and z > box[4] and z < box[5]:
        return True
    else:
        return False



class Script( spin.ScriptBase ):
    
    box = [0, 0, 0, 0, 0, 0] # bottom top left right front back
    inbox = False
    normalScale = [1,1,1]
    scale = 2.0;

    def __init__(self, id, scl):
        print "hotspot init"
        self.box = [-1, 1, 4, 6,-1, 1]
        self._nodeID = id
        self.normalScale = [1,1,1]
        self.scale = scl

    

    def run(self, eventMethod, eventTypes, eventArgs): 
        print "hotspot.run( ", eventMethod, ", [", eventTypes, "], [", eventArgs , "]"
        
        self.applyEvent(eventMethod, eventTypes, eventArgs, 0) # cascadeEvents = 0... actually do the translation, don't redo hotspot.run

        #print "am I in the box?", self.box
        ret = libSPINPyWrap.callback(self._nodeID, "s", "getTranslation", 0)
        x,y,z = ret.getVector()

        if isInside(x, y, z, self.box):
            if not self.inbox:
                print "IN the box!"
                method = "setScale " + `self.scale` + " " + `self.scale` + " " + `self.scale`;
                libSPINPyWrap.callback(self._nodeID, "sfff", method, 0)
            self.inbox = True
        else:
            if self.inbox:
                print "OUT of the box!"
                method = "setScale " + `self.normalScale[0]` + " "  + `self.normalScale[1]` + " "  + `self.normalScale[2]`
                libSPINPyWrap.callback(self._nodeID, "sfff", method, 0)
            self.inbox = False

print "hotspot module loaded."

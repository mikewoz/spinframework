import libSPINPyWrap
import math

def isInside(x, y, z, box):
    if x > box[0] and x < box[1] and y > box[2] and y < box[3] and z > box[4] and z < box[5]:
        return True
    else:
        return False



class Script:
    nodeID = ""
    box = [0, 0, 0, 0, 0, 0] # bottom top left right front back
    inbox = False
    normalScale = [1,1,1]
    scale = 2.0;

    def __init__(self, id, scl):
        self.box = [-1, 1, 4, 6,-1, 1]
        self.nodeID = id
        self.normalScale = [1,1,1]
        self.scale = scl

    def run(self): 

        #print "am I in the box?", self.box
        ret = libSPINPyWrap.callback(self.nodeID, "s", "getTranslation")
        x,y,z = ret.getVector()

        if isInside(x, y, z, self.box):
            if not self.inbox:
                print "IN the box!"
                method = "setScale " + `self.scale` + " " + `self.scale` + " " + `self.scale`;
                libSPINPyWrap.callback(self.nodeID, "sfff", method)
            self.inbox = True
        else:
            if self.inbox:
                print "OUT of the box!"
                method = "setScale " + `self.normalScale[0]` + " "  + `self.normalScale[1]` + " "  + `self.normalScale[2]`
                libSPINPyWrap.callback(self.nodeID, "sfff", method)
            self.inbox = False

print "hotspot module loaded."

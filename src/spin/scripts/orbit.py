import libSPINPyWrap
import math

class Script:
    nodeID = ""
    c = 1
    angle = 0
    radius = 5
    def __init__(self, id):
        print "orbit init!"
        self.c = 1000
        self.nodeID = id

    def run(self): 
        if self.angle % 10 == 0:
            print "running: nodeID = ", self.nodeID, "angle = " , self.angle
        self.angle += 0.001
        x = self.radius * math.cos(self.angle);
        y = self.radius * math.sin(self.angle);
        method = "setTranslation " + `x` + " " + `y` + " 0.0"
        #print "the method is: ", method
        libSPINPyWrap.callback(self.nodeID, "sfff", method, 4)

print "orbit module loaded."

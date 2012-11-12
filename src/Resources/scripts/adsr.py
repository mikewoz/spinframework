import spin
import spinframework
import math

__spin_behavior_class__ = "ADSR"

def interpolate( start, end, dur, t ):
    return (end-start) * (t / dur) + start

class ADSR( spin.ScriptBase ):

    def __init__( self, id, a, d, s, r, scale, duration ):
        print "adsr init!"
        self._nodeID = id
        self.attack = a
        self.decay = d
        self.sustain = s
        self.release = r
        self.scale = scale
        self.duration = duration
        self.start_time = spinframework.time_s()

    def run( self ): 
        
        t = spinframework.time_s()
        val = 0

        if (t > self.start_time + self.duration ):
            self.start_time = t;

        # start < NOW < start + attack        
        if ( self.start_time <= t and
             t < self.start_time + self.attack ):
            #print "ATTACK"
            val = interpolate( 0, self.scale, self.attack, t - self.start_time)

        # start + attack < NOW < start + attack + decay
        elif ( self.start_time + self.attack <= t and
               t < self.start_time + self.attack + self.decay ):
            #print "DECAY"
            val = interpolate( self.scale, self.sustain*self.scale, self.decay, t - self.start_time - self.attack )

        # start + attack + decay < NOW < start + soundclip duration - decay
        elif ( self.start_time + self.attack + self.decay <= t and
               t < self.start_time + self.duration - self.release ):
            #print "SUSTAIN"
            val = self.sustain*self.scale
        
        # start + soundclip duration - release < NOW < start + soundclip duration
        elif ( self.start_time + self.duration - self.release <= t and
               t <self.start_time + self.duration ):
            #print "RELEASE"
            val = interpolate( self.sustain*self.scale, 0, self.release , t - (self.start_time + self.duration - self.release) )

        else:
            print "DONE"

        spinframework.callback( self._nodeID, "setTranslation", [0, 0, val], 1 )

print "adsr module loaded."

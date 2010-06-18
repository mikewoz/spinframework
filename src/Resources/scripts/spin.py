import hashlib
import os.path
import imp
import traceback
import sys
import libSPINPyWrap

##########################################################################################

class ScriptBase:
    _nodeID = ""
    
    def applyEvent(self, eventMethod, eventTypes, eventArgs, cascade):
        method = eventMethod + " " + " ".join(map(str,eventArgs))
        print( method )
        libSPINPyWrap.callback( self._nodeID, eventTypes, method, cascade)


##########################################################################################

def load_module(code_path):
    try:
        try:
            code_dir = os.path.dirname(code_path)
            code_file = os.path.basename(code_path)

            fin = open(code_path, 'rb')

            return  imp.load_source(hashlib.md5(code_path).hexdigest(), code_path, fin)
        finally:
            try: fin.close()
            except: pass
    except ImportError, x:
        traceback.print_exc(file = sys.stderr)
        raise
    except:
        traceback.print_exc(file = sys.stderr)
        raise


def test():
    print "testing..."


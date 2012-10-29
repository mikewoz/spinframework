import hashlib
import os.path
import imp
import traceback
import sys
import spinframework

##########################################################################################

class ScriptBase:
    _nodeID = ""

    def __init__(self, id):
        print "ScriptBase init"
        self._nodeID = id

    def __del__(self):
        print "Node ", self._nodeID, ": script destroyed"

    def applyEvent(self, eventMethod, eventArgs, cascade):
        spinframework.callback( self._nodeID, eventMethod, eventArgs, cascade)

  

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


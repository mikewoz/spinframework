import time
import sys
import os

import spinFramework

spinServer = spinFramework.spinContext(spinFramework.spinContext.mode.SERVER_MODE)

#spinServer.sendSceneMessage("s", "clear", 0xdeadbeef, 0xf00baa)


spinServer.sendSceneMessage(["createNode", "foo", "GroupNode"]
spinServer.sendSceneMessage(["debug"])

time.sleep(5)

spinServer.sendSceneMessage(["clear"])
spinServer.sendSceneMessage(["debug"])







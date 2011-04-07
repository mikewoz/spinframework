#!/usr/bin/env python
"""
Please document this file.

This example is in the public domain.
"""
import time
import sys
import os
import spinFramework

class spinClient(spinFramework.pySpinContext):
    """
    Please document this class.
    """
    def sceneCallback(self, d):
        """
        Please document this method.
        """
        print "in python sceneCallback"
        print d


if __name__ == "__main__":
    # old way before singleton:
    #client = spinClient(pySpinContext.mode.LISTENER_MODE)

    client = spinClient()
    client.start()

    client.sendSceneMessage(["createNode", "foo", "GroupNode"])
    client.sendSceneMessage(["debug"])
    client.sendNodeMessage(["foo", "debug"])
    time.sleep(2)

    client.sendSceneMessage(["clear"])
    client.sendSceneMessage(["debug"])
    time.sleep(5)


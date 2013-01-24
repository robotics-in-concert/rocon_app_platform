"""
    redirects print message to callbacks
"""
import sys

class Logger(object):
    def __init__(self,default_out=None):
        self.terminal = default_out

        self.callbacks = []

    def addCallback(self,callback):
        self.callbacks.append(callback) 

    # Not implemented
    #def removeCallback

    def write(self,message):
        if self.terminal:
            self.terminal.write(message)

        for callback in self.callbacks:
            callback(message)

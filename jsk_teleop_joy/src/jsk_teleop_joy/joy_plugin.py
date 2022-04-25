# joy_plugins.py

import rospy

class JSKJoyPlugin():
    def __init__(self, name, args):
        self.name = name
        self.args = args
        self.manager = None
    def registerManager(self, manager):
        self.manager = manager;
    def joyCB(self, status):
        # a callback function
        rospy.logerr("%s: no joyCB is overriden" % (self.name))
    def enable(self):
        pass
    def disable(self):
        pass
    def getArg(self, key, default=None):
        if key in self.args:
            return self.args[key]
        else:
            return default

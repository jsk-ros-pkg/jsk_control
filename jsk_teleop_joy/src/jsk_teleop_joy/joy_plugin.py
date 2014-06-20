# joy_plugins.py

import rospy

class JSKJoyPlugin():
    def __init__(self, name, args):
        self.name = name
        self.args = args
    def joyCB(self, status):
        # a callback function
        rospy.logerr("%s: no joyCB is overriden" % (self.name))
    def enable(self):
        pass
    def disable(self):
        pass
    def getArg(self, key, default=None):
        if self.args.has_key(key):
            return self.args[key]
        else:
            return default

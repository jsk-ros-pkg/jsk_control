from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
from jsk_rviz_plugins.msg import OverlayMenu
from std_srvs.srv import Empty

import rospy

class EnvironmentPlaneModelingClient(JSKJoyPlugin):
    def __init__(self, name, args):
        JSKJoyPlugin.__init__(self, name, args)
        self.menu_pub = rospy.Publisher("/overlay_menu", OverlayMenu)
        self.menus = [
            ["Append Pre-defined Polygon", self.appendPreDefinedPolygon],
            ["Register Current GridMap", self.registerCurrentGridMap],
            ["Clear Environment", self.clearEnvironment],
            ["Exit", self.exit]]
        self.clear_maps_srv = rospy.ServiceProxy(
            self.getArg("clear_maps", "/env_server/clear_maps"), Empty)
        self.register_current_map_srv = rospy.ServiceProxy(
            self.getArg("register_current_map", "/env_server/register_to_hisotry"), Empty)
        self.register_completed_map_srv = rospy.ServiceProxy(
            self.getArg("register_completed_map", "/env_server/register_completion_to_hisotry"), Empty)
        self.current_index = 0
    def joyCB(self, status, history):
        # self.pub.publish(status.orig_msg)
        if history.new(status, 'up'):
            self.upCursor()
        elif history.new(status, 'down'):
            self.downCursor()
        #publish menu anyway
        if history.new(status, 'cross'):
            self.publishMenu(close=True)
            rospy.sleep(0.2)
            self.manager.forceToPluginMenu(publish_menu=False)
        elif history.new(status, 'circle'):
            # call the function
            func = self.menus[self.current_index][1]
            func()
        else:
            self.publishMenu() 
    def downCursor(self):
        if self.current_index == len(self.menus) - 1:
            self.current_index = 0
        else:
            self.current_index = self.current_index + 1
    def upCursor(self):
        if self.current_index == 0:
            self.current_index = len(self.menus) - 1
        else:
            self.current_index = self.current_index - 1
    def exit(self):                       #same to cross button
        self.publishMenu(close=True)
        rospy.sleep(0.2)
        self.manager.forceToPluginMenu(publish_menu=False)
    def registerCurrentGridMap(self):
        try:
            self.register_current_map_srv()
        except rospy.ServiceException as e:
            rospy.logfatal("failed to register current map: %s" % (e))
    def appendPreDefinedPolygon(self):
        try:
            self.register_completed_map_srv()
        except rospy.ServiceException as e:
            rospy.logfatal("failed to register completed current map: %s" % (e))
    def clearEnvironment(self):
        try:
            self.clear_maps_srv()
        except rospy.ServiceException as e:
            rospy.logfatal("failed to clear_map: %s" % (e))
    def publishMenu(self, close=False):
        menu = OverlayMenu()
        menu.menus = [m[0] for m in self.menus]
        menu.current_index = self.current_index
        menu.title = self.name
        if close:
            menu.action = OverlayMenu.ACTION_CLOSE
        self.menu_pub.publish(menu)


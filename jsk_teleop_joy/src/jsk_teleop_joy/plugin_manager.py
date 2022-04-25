import os
import sys, traceback
import rospy

from rospkg import RosPack
from rospkg.common import MANIFEST_FILE, PACKAGE_FILE
from subprocess import Popen, PIPE

from xml.etree import ElementTree
import __builtin__

class PluginManager():
  def __init__(self, package_name):
    self.package_name = package_name
  def loadPluginInstances(self, plugins, manager):
    """
    plugins := {name: spec, name: spec, ...}
    spec := {class: class, args: args}
    """
    self.plugins = []

    #for (instance_name, instance_spec) in plugins.items():
    plugin_keys = plugins.keys()
    plugin_keys.sort()
    for instance_name in plugin_keys:
      instance_spec = plugins[instance_name]
      instance_class = instance_spec["class"]
      instance_args = instance_spec["args"]
      if instance_class not in self.plugin_defs:
        rospy.logerr('cannot find %s in plugins for %s' % (instance_class,
                                                           self.package_name))
      else:
        try:
          module_path = self.plugin_defs[instance_class]
          module_name, class_from_class_type = module_path.rsplit('.', 1)
          module = __builtin__.__import__(module_name, 
                                          fromlist=[class_from_class_type],
                                          level=0)
          class_ref = getattr(module, class_from_class_type, None)
          if class_ref is None:
            rospy.logfatal('cannot find %s' % (class_from_class_type))
          else:
            self.plugins.append(class_ref(instance_name, instance_args))
            self.plugins[-1].registerManager(manager)
        except:
          rospy.logerr('failed to load %s' % (instance_class))
          traceback.print_exc(file=sys.stdout)
    return self.plugins
  def loadPlugins(self):
    """
    load plugins of jsk_teleop_joy defined in the packages.
    """
    self.plugin_defs = {}
    p = Popen(["rospack", 'plugins', '--attrib', 'plugin', self.package_name],
              stdin=PIPE,
              stdout=PIPE,
              stderr=PIPE)
    p.wait()
    output = p.stdout.readlines()
    for output_line in output:
      package_name = output_line.split(' ')[0]
      xml_path = output_line.split(' ')[1].strip()
      if os.path.isfile(xml_path):
        try:
          root = ElementTree.parse(xml_path)
        except:
          rospy.logerr("failed to open %s" % (xml_path))
        for library_elem in root.getiterator('library'):
          for class_elem in library_elem.getiterator('class'):
            items = class_elem.attrib
            if 'name' not in items:
              rospy.logerr('class tag of %s does not have name attribute' 
                           % (xml_path))
            else:
              name = items['name']
              if 'type' not in items:
                rospy.logerr('%s does not have type attribute' % (name))
              else:
                plugin_name = items['type']
                self.plugin_defs[name] = plugin_name

    

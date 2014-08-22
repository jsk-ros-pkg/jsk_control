#!/usr/bin/env python

# a script to disable/enable mouse device
import subprocess
import re
def getList():
    # array of array such as:
    # [['Virtual core pointer', 2], ['Power Button', 7], ...]
    name_output = subprocess.check_output(["xinput", "list", "--name-only"]).split("\n")
    id_output = subprocess.check_output(["xinput", "list", "--id-only"]).split("\n")
    return [[name, int(id)] 
            for name, id in zip(name_output, id_output)
            if name and id]

def setDeviceEnable(id, enable):
    subprocess.check_call(["xinput", "set-prop",
                           str(id), "Device Enabled", str(enable)])

def disableDeviceById(id):
    setDeviceEnable(id, 0)

def enableDeviceById(id):
    setDeviceEnable(id, 1)

def disableDeviceByName(name):
    name_id_list = getList()
    for device_name, id in name_id_list:
        if name == device_name:
            disableDeviceByName(id)
            
def enableDeviceByName(name):
    name_id_list = getList()
    for device_name, id in name_id_list:
        if name == device_name:
            enableDeviceByName(id)

def disableDeviceByRegexp(regexp):
    name_id_list = getList()
    for device_name, id in name_id_list:
        if re.match(regexp, device_name) is not None:
            disableDeviceById(id)

def enableDeviceByRegexp(regexp):
    name_id_list = getList()
    for device_name, id in name_id_list:
        if re.match(regexp, device_name) is not None:
            enableDeviceById(id)

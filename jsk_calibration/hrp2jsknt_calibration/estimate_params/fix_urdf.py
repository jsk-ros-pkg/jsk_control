#!/usr/bin/env python


import sys
from xml.dom.minidom import parse
import re

def main(from_file, to_file):
    root = parse(from_file)
    # remove illegal link/join
    # link .*_cb_link
    # joint .*_cb
    links = root.getElementsByTagName("link")
    joints = root.getElementsByTagName("joint")
    for link in links:
        name = link.getAttribute("name")
        if re.match(".*_cb_link$", name):
            link.parentNode.removeChild(link)
    for joint in joints:
        name = joint.getAttribute("name")
        if re.match(".*_cb$", name):
            joint.parentNode.removeChild(joint)
    with open(to_file, "w") as f:
        f.write(root.toxml())


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])

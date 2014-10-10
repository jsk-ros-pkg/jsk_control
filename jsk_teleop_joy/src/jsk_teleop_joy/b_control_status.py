#!/usr/bin/env python

import rospy
import roslib


class BControl2Status():
    def __init__(self, msg):
        if msg.axes[8] == 1.0:
            self.buttonU1 = True
        else:
            self.buttonU1 = False
        if msg.axes[9] == 1.0:
            self.buttonU2 = True
        else:
            self.buttonU2 = False
        if msg.axes[10] == 1.0:
            self.buttonU3 = True
        else:
            self.buttonU3 = False
        if msg.axes[11] == 1:
            self.buttonU4 = True
        else:
            self.buttonU4 = False
        if msg.axes[12] == 1.0:
            self.buttonU5 = True
        else:
            self.buttonU5 = False
        if msg.axes[13] == 1.0:
            self.buttonU6 = True
        else:
            self.buttonU6 = False
        if msg.axes[14] == 1.0:
            self.buttonU7 = True
        else:
            self.buttonU7 = False
        if msg.axes[15] == 1.0:
            self.buttonU8 = True
        else:
            self.buttonU8 = False

        if msg.axes[16] == 1.0:
            self.buttonL1 = True
        else:
            self.buttonL1 = False
        if msg.axes[17] == 1.0:
            self.buttonL2 = True
        else:
            self.buttonL2 = False
        if msg.axes[18] == 1.0:
            self.buttonL3 = True
        else:
            self.buttonL3 = False
        if msg.axes[19] == 1.0:
            self.buttonL4 = True
        else:
            self.buttonL4 = False
        if msg.axes[20] == 1.0:
            self.buttonL5 = True
        else:
            self.buttonL5 = False
        if msg.axes[21] == 1.0:
            self.buttonL6 = True
        else:
            self.buttonL6 = False
        if msg.axes[22] == 1.0:
            self.buttonL7 = True
        else:
            self.buttonL7 = False
        if msg.axes[23] == 1.0:
            self.buttonL8 = True
        else:
            self.buttonL8 = False

        self.slide1 = msg.axes[24]
        self.slide2 = msg.axes[25]
        self.slide3 = msg.axes[26]
        self.slide4 = msg.axes[27]
        self.slide5 = msg.axes[28]
        self.slide6 = msg.axes[29]
        self.slide7 = msg.axes[30]
        self.slide8 = msg.axes[31]

        self.rotate1_1 = msg.axes[0]
        self.rotate1_2 = msg.axes[1]
        self.rotate1_3 = msg.axes[2]
        self.rotate1_4 = msg.axes[3]
        self.rotate1_5 = msg.axes[4]
        self.rotate1_6 = msg.axes[5]
        self.rotate1_7 = msg.axes[6]
        self.rotate1_8 = msg.axes[7]

        self.rotate2_1 = msg.axes[32]
        self.rotate2_2 = msg.axes[33]
        self.rotate2_3 = msg.axes[34]
        self.rotate2_4 = msg.axes[35]
        self.rotate2_5 = msg.axes[36]
        self.rotate2_6 = msg.axes[37]
        self.rotate2_7 = msg.axes[38]
        self.rotate2_8 = msg.axes[39]

        self.rotate3_1 = msg.axes[40]
        self.rotate3_2 = msg.axes[41]
        self.rotate3_3 = msg.axes[42]
        self.rotate3_4 = msg.axes[43]
        self.rotate3_5 = msg.axes[44]
        self.rotate3_6 = msg.axes[45]
        self.rotate3_7 = msg.axes[46]
        self.rotate3_8 = msg.axes[47]

        self.rotate4_1 = msg.axes[48]
        self.rotate4_2 = msg.axes[49]
        self.rotate4_3 = msg.axes[50]
        self.rotate4_4 = msg.axes[51]
        self.rotate4_5 = msg.axes[52]
        self.rotate4_6 = msg.axes[53]
        self.rotate4_7 = msg.axes[54]
        self.rotate4_8 = msg.axes[55]


import pprint

def joyCB(msg):
    status = BControl2Status(msg)
    pprint.PrettyPrinter().pprint(status.__dict__)


def main():
    import roslib
    import rospy
    from sensor_msgs.msg import Joy

    rospy.sleep(1)
    rospy.init_node('b_control_controller')
    s = rospy.Subscriber('/b_control/joy', Joy, joyCB)

    rospy.spin()

if __name__ == '__main__':
    main()

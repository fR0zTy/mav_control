#!/usr/bin/env python

import pygame
from pygame.locals import *
import rospy
from geometry_msgs.msg import Twist

from keyboard_base_alt import BaseKeyboardController


class KeyboardTwistController(BaseKeyboardController):
    def rospy_init(self, rate=10):
        BaseKeyboardController.rospy_init(self, rate)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.msg = Twist()
 


    def draw(self):
        BaseKeyboardController.draw(self)
        width = float(self.background.get_rect().width)

        self._draw_x = width / 2
        self._draw_text("Keyboard Twist controller", 40)

        old_draw_y = self._draw_y
        self._draw_x = width / 4
        self._draw_text("Linear vel", 30)
        self._draw_text("X (q/e) : %.3f" % self.msg.linear.x)
        self._draw_text("Y (a/d) : %.3f" % self.msg.linear.y)
        self._draw_text("Z (w/s) : %.3f" % self.msg.linear.z)

        self._draw_y = old_draw_y # On the side
        self._draw_x = width * 3 / 4
        self._draw_text("Angular vel", 30)
        self._draw_text("X (u/o): %.5f" % self.msg.angular.x)
        self._draw_text("Y (j/l): %.5f" % self.msg.angular.y)
        self._draw_text("Z (i/k) : %.5f" % self.msg.angular.z)


if __name__ == '__main__':
    try:
        keyboard = KeyboardTwistController()
        keyboard.run()
    except rospy.ROSInterruptException:
        print("Closed the program due to ROS Interrupt exception")
    except KeyboardInterrupt:
        print("Closed the program due to Keyboard Interrupt exception")


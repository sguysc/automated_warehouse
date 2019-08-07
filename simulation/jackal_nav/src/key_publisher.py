#!/usr/bin/env python
# BEGIN ALL
import sys, select, tty, termios
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
  key_pub = rospy.Publisher('keys', String, queue_size=1)
  rospy.init_node("keyboard_driver")
  rate = rospy.Rate(100)
  # BEGIN TERMIOS
  old_attr = termios.tcgetattr(sys.stdin)
  tty.setcbreak(sys.stdin.fileno())
  # END TERMIOS
  print "Publishing keystrokes. Press Ctrl-C to exit..."
  char1 = ""
  while not (rospy.is_shutdown()):
    # BEGIN SELECT
    if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
      char1 = sys.stdin.read(1)
      key_pub.publish(char1)
    rate.sleep()
    # END SELECT
  # BEGIN TERMIOS_END
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
  # END TERMIOS_END
# END ALL

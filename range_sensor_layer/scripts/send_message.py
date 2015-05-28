#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Range

def talker():
  pub = rospy.Publisher('/sensors/range', Range)
  rospy.init_node('mock_sonar')
  rate = rospy.Rate(10)  # 10 Hz

  # Msg creation
  r = Range()
  r.header.frame_id = '/right_sonar_link'
  r.field_of_view = 25*3.14/180
  r.max_range = 100
  r.range = 1

  while not rospy.is_shutdown():
    r.header.stamp = rospy.Time.now()
    rospy.loginfo(r)
    pub.publish(r)
    rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass

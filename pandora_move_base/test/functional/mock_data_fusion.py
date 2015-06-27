#!/usr/binenv python

import rospy
from pandora_data_fusion_msgs import ObstacleInfo
import mock_map_patcher_params as params

def talker():
  pub = rospy.Publisher(params.obstacleTopic, ObstacleInfo, queue_size = 1)
  rospy.init_node('mock_data_fusion', anonymous=True)
  rate = rospy.Rate(1) # 1Hz
  while not rospy.is_shutdown():
      # Do message stuff
      pub.publish(obs)
      rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass

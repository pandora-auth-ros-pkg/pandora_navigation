def setCostBoundingBox(self, minX, maxX, minY, maxY, cost):
  
    for i in range(0, self.mock_map_.info.width):
      if i >= minX and i <= maxX:
        for j in range(0, self.mock_map_.info.height):
          if j >= minY and j <= maxY:
            self.mock_map_.data[i + self.mock_map_.info.width * j] = cost
def talker():
  
  
    # 10 Hz
  # se rads
  quat = quaternion_from_euler(0, 0, 0) # (roll, pitch, yaw)
  rospy.loginfo(quat)
  # Msg creation
  # TODO Assure that slam doesnt clear the patch
  map_update_msg = OccupancyGrid()
  #
  map_update_msg.header.frame_id = '/map'
  map_update_msg.info.resolution = 0.02
  
  map_update_msg.info.origin.position.x = -7.0
  map_update_msg.info.origin.position.y = -7.0
  map_update_msg.info.origin.position.z = 0.0

  # map_update_msg.info.origin.orientation.x = quat[0]
  # map_update_msg.info.origin.orientation.y = quat[1]
  # map_update_msg.info.origin.orientation.z = quat[2]
  # map_update_msg.info.origin.orientation.w = quat[3]

  map_update_msg.info.width = 100
  map_update_msg.info.height = 100

  temp_array = [51] * map_update_msg.info.width * map_update_msg.info.height

  #for i in range(0,map_update_msg.info.width * map_update_msg.info.height/2):
  #  temp_array[i] = 0;
  map_update_msg.data = temp_array
  setCostBoundingBox(map_update_msg.data, 0, 100, 0, 100, map_update_msg.info.width, map_update_msg.info.height, 90)


  

if __name__ == '__main__':
  # Initialize the node 
  rospy.init_node('mock_soft_patcher')
  try:
    mocknode = MockMapPatcher()
  except rospy.ROSInterruptException:
    pass
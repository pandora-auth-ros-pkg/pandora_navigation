# The topic of the map we take from SLAM
slamMapTopic = '/slam/map'

# The topic that sends the obstacle messages
obstacleTopic = '/data_fusion/obstacle_info'

# The topic where we post the new map with obstacle on it
patchTopic = '/map_patch/obstacles'

# Unknown cost param
unknownCost = 51
lethalCost = 90
freeCost = 0
softObstacleType = 1
hardObstacleType = 2

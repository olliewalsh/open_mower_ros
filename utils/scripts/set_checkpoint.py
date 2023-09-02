>>> import rosbag
>>> from mower_logic.msg import CheckPoint
>>> cp = CheckPoint()
>>> cp.currentMowingPath = 1
>>> cp.currentMowingPathIndex = 23000
>>> cp.currentMowingPlanDigest = "FE0C67D48F9DD638FE0EEC6FE7D4209D4EE70A046B8B607F2E85D80F50A86054"
>>> bag = rosbag.Bag('checkpoint.bag', 'w')
>>> bag.write('checkpoint', cp)
>>> bag.close()
>>> 

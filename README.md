# ros-rcll_btr_msgs

This is the communication program between ROS and RobView using UDP.

from the view of ROS
  Services: setting data for RobView
    'rvw2/setVelocity', SetVelocity
    'rvw2/positionDriver', SetPosition
    'rvw2/setOdometry', SetOdometry
  Publish: getting data from RobView
    'robotino/odometry', Odometry
    'robotino/checkFlag', Bool
    'robotino/getVelocity', Float32MultiArray

# FrontierList.msg
#
# Published by each drone's frontier detector.
# Contains the centroids (centers) of detected frontier clusters.
# The coordinator uses this to decide where to send each drone.
#
# A frontier cluster is a group of adjacent cells on the occupancy grid
# that sit on the boundary between FREE and UNKNOWN space.
# Larger clusters = more unexplored area behind them = higher information gain.

# Standard header: timestamp + coordinate frame (usually "map_d1" or "map_d2")
std_msgs/Header header

# Array of 3D points (we only use x,y; z is the flight altitude).
# Each point is the centroid of one frontier cluster.
geometry_msgs/Point[] centroids

# How many cells are in each frontier cluster.
# Same length as centroids[]. Bigger = more to explore.
# The goal selector uses this as an "information gain" proxy.
float32[] sizes

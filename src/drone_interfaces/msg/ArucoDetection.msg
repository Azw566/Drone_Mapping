# ArucoDetection.msg
#
# Published each time a drone's camera detects an ArUco tag.
# The detector node fills in the tag_id, the TF pipeline resolves world_pose.
#
# Usage in Python:
#   msg = ArucoDetection()
#   msg.tag_id = 7
#   msg.world_pose.header.frame_id = "map_d1"
#   msg.world_pose.pose.position.x = 3.5
#   msg.confidence = 0.92
#   msg.detected_by = "d1"

# Which ArUco tag was detected (matches the dictionary index)
uint32 tag_id

# The tag's 6DoF pose transformed into the drone's map frame.
# PoseStamped includes a Header (timestamp + frame_id) and a Pose (position + orientation).
# This lets you know WHERE the tag is in the world AND WHEN it was seen.
geometry_msgs/PoseStamped world_pose

# Detection quality score (0.0 = garbage, 1.0 = perfect).
# Computed from: marker size in pixels, reprojection error, viewing angle.
# The POI manager uses this to pick the best estimate when the same tag
# is seen multiple times from different angles/drones.
float32 confidence

# Which drone made this detection ("d1" or "d2").
# The POI manager uses this for multi-drone deduplication.
string detected_by

# EKF2: Use Vision for Position and Heading
param set EKF2_EV_CTRL 15
param set EKF2_HGT_MODE 3

# Safety: Allow arming without GPS indoors
param set COM_ARM_WO_GPS 1
param set COM_RC_IN_MODE 4

# Initialize EKF Origin (Required to turn xy_valid to True)
commander set_ekf_origin 47.397742 8.545594 488.0

#!/bin/bash
# PX4 startup script with vision parameters for GPS-denied operation

cd /root/PX4-Autopilot/build/px4_sitl_default/rootfs
rm -f dataman parameters*.bson

# Create parameter override file
cat > /tmp/px4_params_override << 'PARAMS'
# Disable GPS
param set SYS_HAS_GPS 0
param set EKF2_GPS_CTRL 0

# Enable external vision velocity (bit 2 = 4)
param set EKF2_EV_CTRL 4
param set EKF2_EVV_NOISE 0.15
param set EKF2_EV_DELAY 20

# Disable GPS checks
param set COM_ARM_WO_GPS 1

# Use baro for height
param set EKF2_HGT_REF 1
PARAMS

# Start PX4 with SIH quadcopter (has internal simulation)
PX4_SYS_AUTOSTART=10040 ../bin/px4 -d

#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roslaunch dt_street_light_controller dt_street_light_controller.launch veh:=$VEHICLE_NAME

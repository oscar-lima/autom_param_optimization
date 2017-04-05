#!/bin/bash

# check if command line arguments are available
if [ $# == 0 ]; then
    TIMES_TO_RUN=1
else
    TIMES_TO_RUN=$1
fi

# yb params
# MAP_UPDATE_INTERVAL=5.0
# LINEARUPDATE=1.0
# ANGULAR_UPDATE=0.5
# RESAMPLE_THRESHOLD=0.5
# PARTICLES=30

# algorithm params
MAP_UPDATE_INTERVAL=1.0
LINEARUPDATE=0.8
ANGULAR_UPDATE=0.3
RESAMPLE_THRESHOLD=0.5
PARTICLES=30

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done

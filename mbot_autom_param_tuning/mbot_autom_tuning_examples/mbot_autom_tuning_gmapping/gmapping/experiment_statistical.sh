#!/bin/bash

TIMES_TO_RUN=10

#map_update_interval {1.0, 3.0, 5.0} [5.0]
#linearUpdate {0.8, 1.0, 1.2} [1.0]
#angularUpdate {0.3, 0.4, 0.5} [0.5]
#resampleThreshold {0.3, 0.5, 0.7} [0.5]
#particles {30, 50, 200 } [30]

# 1
MAP_UPDATE_INTERVAL=5.0
LINEARUPDATE=1.0
ANGULAR_UPDATE=0.5
RESAMPLE_THRESHOLD=0.5
PARTICLES=30

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 1'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done

# 2
MAP_UPDATE_INTERVAL=1.0
LINEARUPDATE=1.0
ANGULAR_UPDATE=0.5
RESAMPLE_THRESHOLD=0.5
PARTICLES=30

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 2'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done
    
# 3
MAP_UPDATE_INTERVAL=3.0
LINEARUPDATE=1.0
ANGULAR_UPDATE=0.5
RESAMPLE_THRESHOLD=0.5
PARTICLES=30

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 3'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done
    
# 4
MAP_UPDATE_INTERVAL=5.0
LINEARUPDATE=0.8
ANGULAR_UPDATE=0.5
RESAMPLE_THRESHOLD=0.5
PARTICLES=30

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 4'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done
    
# 5
MAP_UPDATE_INTERVAL=5.0
LINEARUPDATE=0.3
ANGULAR_UPDATE=0.5
RESAMPLE_THRESHOLD=0.5
PARTICLES=30

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 5'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done
    
    
# 6
MAP_UPDATE_INTERVAL=5.0
LINEARUPDATE=1.0
ANGULAR_UPDATE=0.5
RESAMPLE_THRESHOLD=0.5
PARTICLES=200

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 6'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done
    
# 7
MAP_UPDATE_INTERVAL=5.0
LINEARUPDATE=0.3
ANGULAR_UPDATE=0.3
RESAMPLE_THRESHOLD=0.5
PARTICLES=200

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 7'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done
    
# 8
MAP_UPDATE_INTERVAL=1.0
LINEARUPDATE=0.2
ANGULAR_UPDATE=0.2
RESAMPLE_THRESHOLD=0.5
PARTICLES=200

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 8'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done
    
# 9
MAP_UPDATE_INTERVAL=2.0
LINEARUPDATE=0.5
ANGULAR_UPDATE=0.8
RESAMPLE_THRESHOLD=0.5
PARTICLES=50

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 9'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done
    
# 10
MAP_UPDATE_INTERVAL=5.0
LINEARUPDATE=1.0
ANGULAR_UPDATE=0.5
RESAMPLE_THRESHOLD=0.5
PARTICLES=30

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 10'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done
    
#11
MAP_UPDATE_INTERVAL=5.0
LINEARUPDATE=1.0
ANGULAR_UPDATE=0.5
RESAMPLE_THRESHOLD=0.5
PARTICLES=50

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 11'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done

# 12
MAP_UPDATE_INTERVAL=5.0
LINEARUPDATE=1.0
ANGULAR_UPDATE=0.5
RESAMPLE_THRESHOLD=0.5
PARTICLES=200

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 12'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done
    
# 13
MAP_UPDATE_INTERVAL=5.0
LINEARUPDATE=1.0
ANGULAR_UPDATE=0.5
RESAMPLE_THRESHOLD=0.5
PARTICLES=500

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 13'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done
    
# 14
MAP_UPDATE_INTERVAL=1.0
LINEARUPDATE=1.0
ANGULAR_UPDATE=0.5
RESAMPLE_THRESHOLD=0.3
PARTICLES=300

for j in `seq 1 $TIMES_TO_RUN`;
    do
        echo '[run_gmpping_instance_example.sh] running gmapping experiment 14'
        rosrun mbot_automatic_param_tuning coordinator_trigger_node -map_update_interval $MAP_UPDATE_INTERVAL -linearUpdate $LINEARUPDATE -angularUpdate $ANGULAR_UPDATE -resampleThreshold $RESAMPLE_THRESHOLD -particles $PARTICLES
        sleep 3
    done

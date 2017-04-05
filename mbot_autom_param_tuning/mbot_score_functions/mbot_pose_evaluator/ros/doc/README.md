mbot_pose_evaluator
===================

Calculate the error between two poses, linear and angular.

The following commands can be used to test this component:

```
rostopic pub -r 10 /move_base_simple/goal2 geometry_msgs/PoseStamped '{header:  {seq: 0.1, stamp: {secs: 0, nsecs: 0}, frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

rostopic pub -r 10 /move_base_simple/goal2 geometry_msgs/PoseStamped '{header:  {seq: 0.1, stamp: {secs: 0, nsecs: 0}, frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.347014294721, w: 0.93785983988}}}'

rostopic pub -r 10 /move_base_simple/goal2 geometry_msgs/PoseStamped '{header:  {seq: 0.1, stamp: {secs: 0, nsecs: 0}, frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.627936913751, w: 0.778264243267}}}'
```

The other pose can be published using rviz 2D Nav Goal

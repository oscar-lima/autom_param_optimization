amcl automatic parameter tuning
===============================

This document is work under progress... (is incomplete for now)

# example of how to run one amcl instance
rosrun mbot_autom_param_tuning_tools smac_wrapper_node amcl no_instance 0 1.7976931348623157E308 2147483647 -1 -odom_alpha1 '0.005' -odom_alpha2 '0.005' -odom_alpha3 '0.01' -odom_alpha4 '0.005' -odom_alpha5 '0.003'

# example of running smac
rosrun mbot_autom_param_tuning_tools smac --use-instances false --numberOfRunsLimit 100 --pcs-file example_scenarios/amcl/params.pcs --algo "rosrun mbot_autom_param_tuning_tools smac_wrapper_node amcl" --run-objective QUALITY

# snippet on how to transform the mocap pose stamped message orientation into yaw

```
#!/usr/bin/env python
import tf

x = -0.00534526305273
y = -0.00332543952391
z = -0.417838066816
w = 0.908499658108

roll, pitch , yaw = tf.transformations.euler_from_quaternion([x, y, z, w])
print yaw
```
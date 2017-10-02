# autom_param_optimization
ROS wrapper for [SMAC](http://www.cs.ubc.ca/labs/beta/Projects/SMAC/), a versatile tool for optimizing algorithm parameters.

Is recommended that you consult the IEEE publication with title: A Case Study on Automatic Parameter Optimization
of a Mobile Robot Localization Algorithm, published on [ICARSC 2017](http://icarsc2017.isr.uc.pt/index.php/en/) conference in Coimbra, Portugal.

Abstract of the publication:

Algorithms in robotics typically tend to expose
several parameters for the user to configure. This allows both re-
usability and fine tuning of the algorithm to a particular setup,
but at the expense of significant effort in tuning, since this task
is typically done manually. However, recent results in parameter
optimization have shown to be quite successful, namely in
automatic configuration of boolean satisfiability problem solvers,
contributing to a significant increase in scalability. In this paper
we address the applicability of these methods to the area of mobile
robot localization. In particular, we applied a sequential model-
based optimization method to the automatic parameter tuning
of the well-known Adaptive Monte Carlo Localization algorithm.
Our results show a statistically significant improvement over
the default algorithm values. We also contribute with an open
source experimental setup, based on the popular Robot Operating
System ROS, which can be easily adapted to other algorithms.

You can consult the full article [here](https://github.com/oscar-lima/autom_param_optimization/blob/master/autom_param_optimization.pdf)

A nice tutorial on how to make this code work can be found [here](https://github.com/socrob/autonomous_systems/tree/master/lab2)

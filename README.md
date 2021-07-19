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

# Tested On
- Ubuntu 18.04 
- ROS1 melodic,kinetic

# Usage

## For testing the working with their data and ROS1

1. git clone https://github.com/oscar-lima/autom_param_optimization.git
2. git clone https://github.com/socrob/autonomous_systems.git
3. Copy the folder /mcr_manipulation_msgs from autonomous_systems/resources to your workspace. For example, if you cloned autonomous systems to the home folder and your workspace is named catkin_ws you can do:
      
           cp -avr ~/autonomous_systems/resources/mcr_manipulation_msgs/ ~/catkin_ws/src/

4. Build your catkin workspace:

       roscd
       catkin build
       source ~/.bashrc

5. Install dependencies:

       cp -r $HOME/autonomous_systems/resources/mbot_world_model/ $ROS_WORKSPACE
       cp -r $HOME/autonomous_systems/resources/mcr_states/ $ROS_WORKSPACE
       cp -r $HOME/autonomous_systems/resources/mcr_manipulation_msgs/ $ROS_WORKSPACE
       cp -r $HOME/autonomous_systems/resources/mcr_common_msgs/ $ROS_WORKSPACE
       sudo apt-get install ros-kinetic-map-server ros-kinetic-moveit-msgs ros-kinetic-amcl

6. Build the dependencies:

       roscd
       catkin build
       source ~/.bashrc

NOTE: Please notice for new pkgs to be recognized by ROS you need to 1) build it 2) source the .bashrc !

7. Run the server:

       roscore
       roslaunch mbot_autom_tuning_amcl amcl_instance_server.launch

8. Run the client:

         rosrun mbot_autom_tuning_amcl run_amcl_instance_client_node

9. Configure rviz to visualize the localization:

See the following youtube video [here](https://www.youtube.com/watch?v=8Tb2poqgDqM) that explains how to configure rviz

For more details and basic sources refer [here](https://github.com/socrob/autonomous_systems.git)

## To test/train with your own data and ROS1 

**1. First generate your bag file-**
 
- Get AMCL working with your world model,and map ([reference](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/))
- Generate 3 additional topics needed-

    1. Remap the scan topic to /scan_front
    2. Publish the ground truth using the topic gazebo/model_states in ros1.Reference script - (Attach)
    3. Publish the tf between the above published ground_truth and map.Reference script - (Attach) ([reference](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29))

 - Record all these topics as a bag file in addition to the ones published by the amcl setup or you can even specifically record only the required topics as below.
 
         rosbag record -O bag_file_name /cmd_vel /odom /rosout /rosout_agg /tf /scan_front /vrpn_client_node/base_link_gt/pose /tf_static /amcl_pose
- NOTE: 
  - For recording own bag file - make sure to record a larger size of file at least 15-20 mb and also a good duration minimum of 3-5 min.
  - amcl_pose is included because it is needed later to compare between amcl_pose(using these default parameters) and the amcl_pose computed by SMAC tuning process

**2. Filter the recorded bag file-**
- The bag file as input to the SMAC algorithm should not contain map-to-odom transform and /amcl_pose as recorded above,because these will be  published by the SMAC algorithm itself.So these topics need to be filtered out of the bag file using the command-

      rosbag filter your_bag_file_as_recorded_above without_tf_map_to_odom.bag 'topic == "/cmd_vel" or topic == "/odom" or topic == "/rosout" or topic == "/rosout_agg" or topic == "/scan_front" or topic == "/scan_rear" or topic == "/vrpn_client_node/base_link_gt/pose" or topic == "/tf_static" or topic == "/tf"  and m.transforms[0].child_frame_id != "odom"'
 - NOTE:
   - If needed you can include additional topics in the above command,but these are the minimum required topics.
   - In the above command,make sure that "m.transforms[0].child_frame_id != "odom"" comes after "/tf" and not "tf_static".([reference](https://answers.ros.org/question/56935/how-to-remove-a-tf-from-a-ros-bag/))

**3. Visualize the tf-tree of the filtered bag file.Tf-tree for reference- (Attach)**

**4. Following changes are needed-**
- Change the map path in amcl_dependencies.launch, Change the map path in amcl_instance_server.launch,Change the bag_file path in amcl_run_bag.launch.Check the other launch files,if any other paths are different for your setup.In the amcl_run_bag.launch ,you can change the start and duration time if neeeded.
- Change the initial position in default_amcl_params.yaml according to your bot's initial position in the simulation.
- In case if your bag file is large change the timeout in these scripts- run_amcl_instance_client.py, run_algorithm_instance_server.py,smac_wrapper.py

**5. To get the quality score-**
- Once the above steps are completed for your custom data,run the amcl server and client commands and visualize the localization in rviz.(as done for their data previously,).You can also get a score value(lower the score,better is your localization) for your data using the command- 
       
       rosrun mbot_autom_param_tuning_tools smac_wrapper_node amcl no_instance 
 
**6. This step is optional-**
- You can also change the friction parameters for your turtlebot(if using) by changing the mu1,mu2 parameters in .xacro file of the particular turtlebot3 model ([example file](https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro)).For more details refer [here](http://gazebosim.org/tutorials?tut=friction)

**7. This step is optional-**
- Compute the error between the SMAC estimated amcl_pose and ground_truth.Reference scripts - (Attach 3)
- Usage and description of the scripts-
  -  Description-

      1. sync_amcl.py- To subscribe to the SMAC estimated amcl_pose.
      2. sync_odo.py- To subscribe to the ground_truth pose.
      (The above two scripts are used so that the text files can be used to generate the graph of (x vs y) using matplotlib to compare the two poses graphically.Refernce script to generate graph- (Attach))
      3. sync_both.py - To sync the SMAC estimated amcl_pose and ground_truth pose so that the values stored in the text files(after executing these scripts) can be read and the MSE(Mean_squared_error) can be calculated.

    - Usage- 
      
      1. Run these scripts(script 1,2,3) in parallel(in different terminals) to the SMAC algorithm,so that these scripts can subscribe to the topics as published by the SMAC.Once the text files are generated ,use these to visualize the graph and compute the MSE.

      2. To compare this SMAC MSE to the amcl_pose recorded in the bag file (topic /amcl_pose),play the recorded bag file (not the filtered one) and run the same scripts(1,2 and 3) again to get the text files and compute the MSE.(in other words repeat the same steps as done for computing the MSE for SMAC and for graph visualization of SMAC.)

      3. Now the MSE of /amcl_pose(the pose with default amcl parameters) and the MSE of SMAC estimated pose can be compared,to check how much the error has reduced after tuning.

**8. To train or tune the SMAC algorithm to your custom data-**
(This step is needed if either the MSE is too high or localization needs further improvement)
- Example of the Command used to tune-

      rosrun mbot_autom_param_tuning_tools smac --use-instances false --numberOfRunsLimit 100 --pcs-file example_scenarios/amcl/params.pcs --algo "rosrun mbot_autom_param_tuning_tools smac_wrapper_node amcl" --run-objective QUALITY
     - The --numberOfRunsLimit can be configured according to your data,as to how many iterations of the algorithm are needed to run.
     - --pcs-file is the path of the file - amcl_smac_params.pcs (provided in the repo). This file includes the parameters ranges and their default values which will be used for the SMAC algorithm.The ranges of the parameters can be changed according to your data,so that the algorithm searches for the best parameters only in the required range)
     - The above command will log all the details and best configuration in smac_output folder with self-explanatory files.

- While the above command is running,you can visualize the localization in rviz,(as show in the [youtube video](https://www.youtube.com/watch?v=8Tb2poqgDqM).You can change the default_amcl_parameters to start the training with ,or the duration of the bag file or the number of iterations,according to the custom data.
- Once you get the best parameters out after training(and validation),use these to check error(MSE) as done previously.Once finalized the best parameters, you can use these as the default parameters in your amcl ros localization package.


## FOR ROS2
- You can use rosbridge,and directly record a bag file in ros1 or setup the world and map file in ros1.([reference](https://github.com/ros2/ros1_bridge/blob/foxy/README.md))
- Once the bag file is obtained,the same steps(above) followed for ros1 can be repeated,because the only difference is the new bag file.


## Common ERRORS-
- can transform error.. look into future...- if it publishes constantly- then make sure your tf tree and transforms are correct. It might be due to some timesource issue.The algo will still seem to work ,but you will not know which are the best parameters.

- process has died amcl- mainly due to short bag file.







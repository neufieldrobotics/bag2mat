# bagconvert

* Takes a rosbag file and converts it to a matlab mat file based on settings specified in a yaml config file.  See config/bag2mat_config.yaml for an example.  
Basically specify a list of -[topic name, message type, name for variable in matfile]
* Currently needs the following packages:
  - Matlab
  - std_msgs
  - sensor_msgs
  - nav_msgs
  - rosbag
  - roslib
  
* Put this rep in the src folder of ros workspace and run 
  ```
  catkin_make
  source devel/setup.bash
  rosrun bag2mat bag2mat <rosbag> [<config_file>]
  ```
  
## Acknowledgments
* This package was inspired and forked from the bagconvert utility which is a part of the [kalibr_allan](https://github.com/rpng/kalibr_allan) package

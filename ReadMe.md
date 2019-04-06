# bagconvert

* Takes a rosbag file and converts it to a matlab mat file based on settings specified in a yaml config file.  See config/bag2mat_config.yaml for example.  
Basically specify a list of `-[topic name, message type, name for variable in matfile]`
* Currently needs the following packages:
  - rosbag
  - roslib
  - scipy

* Put this repo in the src folder of ros workspace and run 
  ```
  catkin_make
  source devel/setup.bash
  rosrun bag2mat bag2mat <rosbag> [<config_file>]
  ```

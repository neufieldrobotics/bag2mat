# bagconvert

* Takes a rosbag file and converts it to a matlab mat file based on settings specified in a Dictionary and a yaml config file.
  - Dictonary: The dicionary file specified which variables from each message type should be moved to the matlab array.
    eg `sensor_msgs/NavSatFix: t.to_sec(), m.latitude, m.longitude, m.altitude`
    See config/bag2mat_dic.yaml for full example.
  - Configuration: This file specified which topic names should be transferred to matlab and under which variable name.
    A list of `-[topic name, message type, name for variable in matfile]`
    See config/bag2mat_config.yaml for full example.
* Currently needs the following packages:
  - rosbag
  - roslib
  - scipy

* Put this repo in the src folder of ros workspace and run 
  ```
  catkin_make
  source devel/setup.bash
  usage: bag2matpy [-h] [-c CONFIG_FILE] [-d DICTIONARY] input_bagfile
  ```

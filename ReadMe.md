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
  
 * **If you add any of the standard ROS message to this code, please submit a pull request.**

## adding your custom messages to it
 * Modify bag2mat.cpp:
   * Add include line similar to `#include <nav_msgs/Odometry.h>`
   * Create a function to handle your custom message similar to:
   ```
    bool handle_odom (string msgTopic, rosbag::View& view, MATFile *pmat, string matlab_label ) {
    vector<double> data = vector<double>();
    geometry_msgs::Pose pos;
    geometry_msgs::Twist twst;

    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {
        nav_msgs::Odometry::ConstPtr s1 = m.instantiate<nav_msgs::Odometry>();

        if (s1 != NULL && m.getTopic() == msgTopic) {
            data.push_back(m.getTime().toSec());
            
            pos = s1->pose.pose;
            twst = s1->twist.twist;
            data.push_back(pos.position.x);
            data.push_back(pos.position.y);
            data.push_back(pos.position.z);
            data.push_back(twst.linear.x);
            data.push_back(twst.linear.y);
            data.push_back(twst.linear.z);
            data.push_back(pos.orientation.x);
            data.push_back(pos.orientation.y);
            data.push_back(pos.orientation.z);
            data.push_back(pos.orientation.w);
            data.push_back(twst.angular.x);
            data.push_back(twst.angular.y);
            data.push_back(twst.angular.z);
        }

    }

    size_t no_of_cols = 14;       // update this line to count number of entries being added above 
    
    ROS_ASSERT_MSG(data.size()%no_of_cols == 0, "The no_of_cols set in bag2mat.cpp doen't match number of entries");
        
    mxArray *pa1 = mxCreateDoubleMatrix(data.size()/no_of_cols,no_of_cols,mxREAL);
    if (pa1 == NULL) {
        printf("%s : Out of memory on line %d\n", __FILE__, __LINE__);
        printf("Unable to create mxArray.\n");
        return(EXIT_FAILURE);
    }
    // Correctly copy data over (column-wise)
    double* pt1 = mxGetPr(pa1);
    for(size_t i=0; i<data.size(); i+=no_of_cols) {

        for(size_t col_iter = 0; col_iter<no_of_cols; col_iter+=1) {
        pt1[(i + col_iter * data.size())/no_of_cols] = data.at(i+col_iter);
        }
    }
    
    // Add it to the matlab mat file
    int status = matPutVariable(pmat, matlab_label.c_str(), pa1);
    if(status != 0) {
        printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
        return(EXIT_FAILURE);
    }
    
    mxDestroyArray(pa1);
    ROS_INFO_STREAM("Finished writing topic: "<<msgTopic<<" to variable: "<<matlab_label);
    return 0;
   }
   ```

   * Add an elif line in main to handle the custom message similar to: `else if (topics[i][1]=="Odometry") handle_odom (topics[i][0], view, pmat, topics[i][2] ) ;`
   * Add the custom message to the yaml config file
   
## Acknowledgments
* This package was inspired and forked from the bagconvert utility which is a part of the [kalibr_allan](https://github.com/rpng/kalibr_allan) package

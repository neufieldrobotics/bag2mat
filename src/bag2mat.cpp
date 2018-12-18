#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/topic.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Duration.h>
#include <nav_msgs/Odometry.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <apriltags2_ros/AprilTagDetection.h>
#include <rosfly_core/currentgoal.h>



#include <boost/filesystem.hpp>
#include <ros/package.h>

// Matlab header, this is needed to save mat files
// Note that we use the FindMatlab.cmake to get this
#include "mat.h"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <thread>


using namespace std;


bool handle_imu (string imuTopic, rosbag::View& view, MATFile *pmat, string matlab_label ) {
    vector<double> data = vector<double>();

    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {

        sensor_msgs::Imu::ConstPtr s1 = m.instantiate<sensor_msgs::Imu>();

        if (s1 != NULL && m.getTopic() == imuTopic) {
            data.push_back(m.getTime().toSec());
            data.push_back(s1->linear_acceleration.x);
            data.push_back(s1->linear_acceleration.y);
            data.push_back(s1->linear_acceleration.z);
            data.push_back(s1->angular_velocity.x);
            data.push_back(s1->angular_velocity.y);
            data.push_back(s1->angular_velocity.z);
            data.push_back(s1->orientation.x);
            data.push_back(s1->orientation.y);
            data.push_back(s1->orientation.z);
            data.push_back(s1->orientation.w);

        }

    }
    size_t no_of_cols = 11;       // update this line to count number of entries being added above 
    
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
    ROS_INFO_STREAM("Finished writing topic: "<<imuTopic<<" to variable: "<<matlab_label);
    return 0;
}

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

bool handle_posestamped (string msgTopic, rosbag::View& view, MATFile *pmat, string matlab_label ) {
    vector<double> data = vector<double>();
    geometry_msgs::Pose pos;
    geometry_msgs::Twist twst;

    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {
        geometry_msgs::PoseStamped::ConstPtr s1 = m.instantiate<geometry_msgs::PoseStamped>();

        if (s1 != NULL && m.getTopic() == msgTopic) {
            data.push_back(m.getTime().toSec());
            
            pos = s1->pose;
            data.push_back(pos.position.x);
            data.push_back(pos.position.y);
            data.push_back(pos.position.z);
            data.push_back(pos.orientation.x);
            data.push_back(pos.orientation.y);
            data.push_back(pos.orientation.z);
            data.push_back(pos.orientation.w);
            
        }

    }

    size_t no_of_cols = 8;       // update this line to count number of entries being added above 
    
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
bool handle_currentgoal (string msgTopic, rosbag::View& view, MATFile *pmat, string matlab_label ) {
    vector<double> data = vector<double>();
    geometry_msgs::Pose pos;
    geometry_msgs::Point poi;
    rosfly_core::currentgoal goals;
    bool flag;

    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {
        rosfly_core::currentgoal::ConstPtr s1 = m.instantiate<rosfly_core::currentgoal>();

        if (s1 != NULL && m.getTopic() == msgTopic) {
            data.push_back(m.getTime().toSec());
            flag = s1->reached;
            poi = s1->gnss_origin;
            pos = s1->pose;

            data.push_back(flag);
            data.push_back(poi.x);
            data.push_back(poi.y);
            data.push_back(poi.z);
            data.push_back(pos.position.x);
            data.push_back(pos.position.y);
            data.push_back(pos.position.z);
            data.push_back(pos.orientation.x);
            data.push_back(pos.orientation.y);
            data.push_back(pos.orientation.z);
            data.push_back(pos.orientation.w);     
        }

    }

    size_t no_of_cols = 12;       // update this line to count number of entries being added above 
    
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

bool handle_apriltag (string msgTopic, rosbag::View& view, MATFile *pmat, string matlab_label ) {
    vector<double> data = vector<double>();
    geometry_msgs::Pose pos;
    geometry_msgs::Twist twst;
    apriltags2_ros::AprilTagDetectionArray tagArray;
    vector<apriltags2_ros::AprilTagDetection> tags;

    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {
        apriltags2_ros::AprilTagDetectionArray::ConstPtr s1 = m.instantiate<apriltags2_ros::AprilTagDetectionArray>();

        if (s1 != NULL && m.getTopic() == msgTopic) {
            
            
            tags = s1 -> detections;
            //ROS_INFO_STREAM("The size of detections is  "<< tags.size() << "raw is "<< (s1->detections).size());
            int a = tags.size();
            if(a >=1){
                for(int z=0; z<=a-1;z++) {
                    apriltags2_ros::AprilTagDetection currenttag;
                    currenttag = tags[z];
                    data.push_back(m.getTime().toSec());
                    data.push_back(currenttag.id[0]);
                    data.push_back(currenttag.size[0]);
                    data.push_back(currenttag.pose.pose.pose.position.x);
                    data.push_back(currenttag.pose.pose.pose.position.y);
                    data.push_back(currenttag.pose.pose.pose.position.z);
                    data.push_back(currenttag.pose.pose.pose.orientation.x);
                    data.push_back(currenttag.pose.pose.pose.orientation.y);
                    data.push_back(currenttag.pose.pose.pose.orientation.z);
                    data.push_back(currenttag.pose.pose.pose.orientation.w);
                }
            }
        }
    }

    size_t no_of_cols = 10;       // update this line to count number of entries being added above 
    
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

bool handle_navsat (string msgTopic, rosbag::View& view, MATFile *pmat, string matlab_label ) {
    vector<double> data = vector<double>();
    
    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {
        sensor_msgs::NavSatFix::ConstPtr s1 = m.instantiate<sensor_msgs::NavSatFix>();

        if (s1 != NULL && m.getTopic() == msgTopic) {
            data.push_back(m.getTime().toSec());
            //data.push_back(s1->status);
            data.push_back(s1->latitude);
            data.push_back(s1->longitude);
            data.push_back(s1->altitude);
        }

    }

    size_t no_of_cols = 4;       // update this line to count number of entries being added above 
    
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

bool handle_vector3 (string msgTopic, rosbag::View& view, MATFile *pmat, string matlab_label ) {
    vector<double> data = vector<double>();
    
    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {
        geometry_msgs::Vector3::ConstPtr s1 = m.instantiate<geometry_msgs::Vector3>();

        if (s1 != NULL && m.getTopic() == msgTopic) {
            data.push_back(m.getTime().toSec());
            data.push_back(s1->x);
            data.push_back(s1->y);
            data.push_back(s1->z);
        }

    }

    size_t no_of_cols = 4;       // update this line to count number of entries being added above 
    
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
    
bool handle_float64 (string msgTopic, rosbag::View& view, MATFile *pmat, string matlab_label ) {
    vector<double> data = vector<double>();

    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {
        // Handle IMU message
        std_msgs::Float64::ConstPtr s1 = m.instantiate<std_msgs::Float64>();
        if (s1 != NULL && m.getTopic() == msgTopic) {
            data.push_back(m.getTime().toSec());
            data.push_back(s1->data);
        }
    }

    mxArray *pa1 = mxCreateDoubleMatrix(data.size()/2,2,mxREAL);
    if (pa1 == NULL) {
        printf("%s : Out of memory on line %d\n", __FILE__, __LINE__);
        printf("Unable to create mxArray.\n");
        return(EXIT_FAILURE);
    }
    // Correctly copy data over (column-wise)
    double* pt1 = mxGetPr(pa1);
    for(size_t i=0; i<data.size(); i+=2) {
        pt1[i/2] = data.at(i);
        pt1[(i + data.size())/2] = data.at(i+1);
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

bool handle_int8 (string msgTopic, rosbag::View& view, MATFile *pmat, string matlab_label ) {
    vector<double> data = vector<double>();

    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {
        // Handle IMU message
        std_msgs::Int8::ConstPtr s1 = m.instantiate<std_msgs::Int8>();
        if (s1 != NULL && m.getTopic() == msgTopic) {
            data.push_back(m.getTime().toSec());
            data.push_back(s1->data);
        }
    }

    mxArray *pa1 = mxCreateDoubleMatrix(data.size()/2,2,mxREAL);
    if (pa1 == NULL) {
        printf("%s : Out of memory on line %d\n", __FILE__, __LINE__);
        printf("Unable to create mxArray.\n");
        return(EXIT_FAILURE);
    }
    // Correctly copy data over (column-wise)
    double* pt1 = mxGetPr(pa1);
    for(size_t i=0; i<data.size(); i+=2) {
        pt1[i/2] = data.at(i);
        pt1[(i + data.size())/2] = data.at(i+1);
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

bool handle_duration (string msgTopic, rosbag::View& view, MATFile *pmat, string matlab_label ) {
    vector<double> data = vector<double>();

    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {
        // Handle IMU message
        std_msgs::Duration::ConstPtr s1 = m.instantiate<std_msgs::Duration>();
        if (s1 != NULL && m.getTopic() == msgTopic) {
            data.push_back(m.getTime().toSec());
            data.push_back(s1->data.toSec());
        }
    }

    mxArray *pa1 = mxCreateDoubleMatrix(data.size()/2,2,mxREAL);
    if (pa1 == NULL) {
        printf("%s : Out of memory on line %d\n", __FILE__, __LINE__);
        printf("Unable to create mxArray.\n");
        return(EXIT_FAILURE);
    }
    // Correctly copy data over (column-wise)
    double* pt1 = mxGetPr(pa1);
    for(size_t i=0; i<data.size(); i+=2) {
        pt1[i/2] = data.at(i);
        pt1[(i + data.size())/2] = data.at(i+1);
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

int main(int argc, char **argv) {


    // Debug message
    ROS_INFO("Starting up");

    // Check if there is a path to a dataset
    

    string configFile;
    
    if(argc < 2) {
        ROS_ERROR("Error please specify atleast rosbag file");
        ROS_ERROR("Command Example: rosrun bag2mat bag2mat <rosbag> [<config_file>]");
        return EXIT_FAILURE;
    }

    else if(argc < 3) {
        ROS_INFO("config not specified using default config");
        configFile = ros::package::getPath("bag2mat")+"/config/bag2mat_config.yaml";
    }
    
    else if(argc == 3) {
       // string imuTopic = argv[2];
        configFile = argv[2];        
    }
    else {
        ROS_ERROR("Too many arguments");
        ROS_ERROR("Command Example: rosrun bag2mat bag2mat <rosbag> [<config_file>]");
        return EXIT_FAILURE;
    }
    
    string imuTopic = "/seabed/imu/data";

    // Startup this node
    ros::init(argc, argv, "bag2mat");
    
    // Parse the input

    string pathBag = argv[1];

    ROS_INFO_STREAM("Using config file: "<<configFile);
    YAML::Node node = YAML::LoadFile(configFile);
    assert(node.Type() == YAML::NodeType::Sequence);
    assert(node.IsSequence());  // a shortcut!
    
    std::vector<std::vector<std::string>> topics;
    
    for (YAML::const_iterator it=node.begin();it!=node.end();++it) {
        topics.push_back(it->as<std::vector<std::string>>());
    }    

    // Get path
    boost::filesystem::path p(pathBag);
    string pathParent = p.parent_path().string();
    string pathMat;
    if(!pathParent.empty()) {
        pathMat = pathParent+"/"+p.stem().string()+".mat";
    } else {
        pathMat = p.stem().string()+".mat";
    }


    // Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(pathBag, rosbag::bagmode::Read);

    rosbag::View view(bag);
        


    // We should load the bag as a view
    // Here we go from beginning of the bag to the end of the bag

    // Debug
    ROS_INFO("BAG Path is: %s", pathBag.c_str());
    ROS_INFO("MAT Path is: %s", pathMat.c_str());
    ROS_INFO("Reading in rosbag file...");


    // Create the matlab mat file
    MATFile *pmat = matOpen(pathMat.c_str(), "w");
    if (pmat == NULL) {
        ROS_ERROR("Error could not create the mat file");
        return(EXIT_FAILURE);
    }
    
    //bool handle_imu (string, rosbag::Bag*, MATFile*);         

    for ( std::vector<std::vector<string>>::size_type i = 0; i < topics.size(); i++ )
    {
     //  for ( std::vector<string>::size_type j = 0; j < topics[i].size(); j++ )
     //   {
     //     std::cout << topics[i][j] << ' ';
     //  }
     //  std::cout << std::endl;
     
        if (topics[i][1]=="Imu") handle_imu (topics[i][0], view, pmat, topics[i][2] ) ;
        else if (topics[i][1]=="Float64") handle_float64 (topics[i][0], view, pmat, topics[i][2] ) ;
        else if (topics[i][1]=="Int8") handle_int8 (topics[i][0], view, pmat, topics[i][2] ) ;
        else if (topics[i][1]=="Odometry") handle_odom (topics[i][0], view, pmat, topics[i][2] ) ;
        else if (topics[i][1]=="Duration") handle_duration (topics[i][0], view, pmat, topics[i][2] ) ;
        else if (topics[i][1]=="NavSatFix") handle_navsat (topics[i][0], view, pmat, topics[i][2] ) ;
        else if (topics[i][1]=="Vector3") handle_vector3 (topics[i][0], view, pmat, topics[i][2] ) ;
        else if (topics[i][1]=="PoseStamped") handle_posestamped (topics[i][0], view, pmat, topics[i][2] ) ;
        else if (topics[i][1]=="AprilTagDetectionArray") handle_apriltag (topics[i][0], view, pmat, topics[i][2] ) ;
        else if (topics[i][1]=="currentgoal") handle_currentgoal (topics[i][0], view, pmat, topics[i][2] ) ;

    }


//    handle_imu ("/seabed/imu/data_raw", view, pmat, "imu_data_raw" ) ;


    ROS_INFO("Done processing bag");

    // Close the mat file
    
    
    if (matClose(pmat) != 0) {
        ROS_ERROR("Error closing the mat file");
        return(EXIT_FAILURE);
    }
    
//    ROS_INFO("Press ctrl+c to exit...");

//    while (true) {std::this_thread::sleep_for(std::chrono::milliseconds(100));}
    
    return EXIT_SUCCESS;
}

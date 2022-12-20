#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZI PointT;

pcl::PointCloud<PointT>::Ptr cloud_pcl (new pcl::PointCloud<PointT>);
sensor_msgs::PointCloud2 cloud_ros;

pcl::PointCloud<PointT>::Ptr floor_remove(pcl::PointCloud<PointT>::Ptr &cloud_pcl);

void cloud_cb(sensor_msgs::PointCloud2 raw_ros)
{
    pcl::fromROSMsg(raw_ros, *cloud_pcl);

    cloud_pcl = floor_remove(cloud_pcl);

    pcl::toROSMsg(*cloud_pcl, cloud_ros);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_filter");
    
    ros::NodeHandle nh("~");
    
    ros::Subscriber sub = nh.subscribe("/mid/points", 1, cloud_cb);
    
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/floor_removed", 1);

    ros::Rate rate(20);
    
    while (ros::ok())
    {
        
        pub.publish(cloud_ros);

        ros::spinOnce();

        rate.sleep();
        
    }
    return 0;
}

pcl::PointCloud<PointT>::Ptr floor_remove(pcl::PointCloud<PointT>::Ptr &cloud_pcl)
{
    //Construct a pcl::PointCloud<PointT>::Ptr object in which to save the 
    //new/processed PointCloud 
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    //Construct a pcl::PointIndices::Ptr variable to save the floor_indices
    pcl::PointIndices::Ptr floor_indices(new pcl::PointIndices());
    //Declare a pcl::ModelCoefficients::Ptr variable to save the floor_coeficients
    pcl::ModelCoefficients::Ptr floor_coefficients;
    //Initialize the previous variable with a constructor 
    //? THOUGHT, maybe the declaring and initialization could be done in one line
    //and save space
    floor_coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());

    //Create a pcl::SACSegmentation<pcl::PointXYZI> variable
    pcl::SACSegmentation<pcl::PointXYZI> floor_finder;
    //True will enable a coefficient refinement
    floor_finder.setOptimizeCoefficients(true);
    //Define the type of model to be used
    floor_finder.setModelType(pcl::SACMODEL_PLANE);
    //Define the method to use
    floor_finder.setMethodType(pcl::SAC_RANSAC);
    //Define max iterations
    floor_finder.setMaxIterations(300);
    floor_finder.setAxis(Eigen::Vector3f(0, 0, 1));  
    floor_finder.setDistanceThreshold(0.10);  
    floor_finder.setEpsAngle(0.174); 

    floor_finder.setInputCloud(cloud_pcl);
    
    floor_finder.segment(*floor_indices, *floor_coefficients);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_pcl);
    //Then we feed it our floor_indices that we obtained from the segmentation
    extract.setIndices(floor_indices);
    //Set whether the regular conditions for points filtering should apply, 
    //or the inverted conditions. 
    //false = normal filter behavior (default), true = inverted behavior
    extract.setNegative(true);
    //Apply the filter and save the data in cloud_filtered variable 
    extract.filter(*cloud_filtered);

    //Return the just filtered set of data points
    return cloud_filtered;
}

//Deprecated, not longer needed
/*#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>

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

        //With spin it stops showing data. It should use spinOnce
        //PCL does not publish continuosly. It is triggered and till it is closed
        //it works with the same data
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
    //Declare a pcl::ModelCoefficients::Ptr variable to save the floor_coeficients
    //The coefficients are the model coefficients
    //Meaning that if a 'x' model is found to fit the inliers, the equation
    //coefficients (those corresponding to the line, plane, etc that fitted 
    //the inliers) are saved here
    //The number of coefficients will depend on the SACMODEL choosen
    //PLANE returns 4 coefficients: normal_x, normal_y, normal_z, and d
    //Which is used in equations related with planes
    pcl::ModelCoefficients::Ptr floor_coefficients (new pcl::ModelCoefficients());
    //Construct a pcl::PointIndices::Ptr variable to save the inliers
    //Inliers are going to be the point cloud array indices that are going to be kept
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    //Create a pcl::SACSegmentation<pcl::PointXYZI> variable
    //Select the points to be considered inliers
    pcl::SACSegmentation<pcl::PointXYZI> floor_finder;
    //True will enable a coefficient refinement
    floor_finder.setOptimizeCoefficients(true);
    //Define the type of model to be used
    floor_finder.setModelType(pcl::SACMODEL_PLANE); //Ok
    //Define the method to use
    floor_finder.setMethodType(pcl::SAC_RANSAC); //Ok
    //Define max iterations. The more iterations, the more precise the model is
    floor_finder.setMaxIterations(300);
    //floor_finder.setAxis(Eigen::Vector3f(0, 0, 1));  
    floor_finder.setDistanceThreshold(0.01); //10cm  
    floor_finder.setEpsAngle(0.174); 

    floor_finder.setInputCloud(cloud_pcl);
    floor_finder.segment(*inliers, *floor_coefficients);

    //Create the filtering object
    //Apply the filter using the above selected points
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_pcl);
    //Then we feed it our inliers that we obtained from the segmentation
    extract.setIndices(inliers);
    //If set to true, you will get the original cloud minus the inliers
    //This means, you will remove the planes you obtained in the previous segmentation
    extract.setNegative(true);
    //Apply the filter and save the data in cloud_filtered variable 
    extract.filter(*cloud_filtered);

    //Return the just filtered set of data points
    return cloud_filtered;
}
*/
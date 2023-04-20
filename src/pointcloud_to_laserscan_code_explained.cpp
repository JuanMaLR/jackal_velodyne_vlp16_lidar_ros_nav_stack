//Declare libraries to be used in the code
#include <limits>
#include <pluginlib/class_list_macros.h>
#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

//To tell that all the code belongs to pointcloud_to_laserscan scope 
//If there are two variables with the same name, the namespace prevents a conflict. It is a logical separation at code level.
namespace pointcloud_to_laserscan
{
PointCloudToLaserScanNodelet::PointCloudToLaserScanNodelet()
{
}

//Function to be executed at the beginning on the program, doesn't return anything
void PointCloudToLaserScanNodelet::onInit()
{
  //Mutex helps prevent two threads from accessing the same resource at the same time
  //scoped_lock locks the mutex until the scoped is exited
  boost::mutex::scoped_lock lock(connect_mutex_);
  //As a general rule (there are exceptions), you should access parameters using getPrivateNodeHandle(), and topics via getNodeHandle().
  //That is because parameters are (generally) private to your nodelet, while topics will be shared with other nodes and nodelets.
  private_nh_ = getPrivateNodeHandle();

  //Declare the node (class) parameters. If no information is provided when the node is called from the launch file,
  //the default values here (3rd parameter) are used to initialize the parameter. 
  //Second parameter is the variable name used in the program that has either the default or launch file value for the parameter
  private_nh_.param<std::string>("target_frame", target_frame_, "");
  private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
  private_nh_.param<double>("min_height", min_height_, std::numeric_limits<double>::min());
  private_nh_.param<double>("max_height", max_height_, std::numeric_limits<double>::max());

  private_nh_.param<double>("angle_min", angle_min_, -M_PI);
  private_nh_.param<double>("angle_max", angle_max_, M_PI);
  private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
  private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
  private_nh_.param<double>("range_min", range_min_, 0.0);
  private_nh_.param<double>("range_max", range_max_, std::numeric_limits<double>::max());
  private_nh_.param<double>("inf_epsilon", inf_epsilon_, 1.0);

  int concurrency_level;
  private_nh_.param<int>("concurrency_level", concurrency_level, 1);
  private_nh_.param<bool>("use_inf", use_inf_, true);

  // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
  if (concurrency_level == 1)
  {
    nh_ = getNodeHandle();
  }
  else
  {
    nh_ = getMTNodeHandle();
  }

  // Only queue one pointcloud per running thread
  if (concurrency_level > 0)
  {
    input_queue_size_ = concurrency_level;
  }
  else
  {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }

  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty())
  {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
    message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
    message_filter_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
    message_filter_->registerFailureCallback(boost::bind(&PointCloudToLaserScanNodelet::failureCb, this, _1, _2));
  }
  else  // otherwise setup direct subscription
  {
    sub_.registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
  }

  //Declare publisher that will post LaserScan messages to the scan topic with 10 messages in queue
  pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
                                               boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
}

//Functions to be added to the publisher
//First one is for connecting when the code is executed
//It manages threads to prevent deadlocks and launches the subscriber 
void PointCloudToLaserScanNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
    sub_.subscribe(nh_, "cloud_in", input_queue_size_);
  }
}

//Second one disconnects (eliminates) the threads once the program is finished 
//It shutdown subscriber connections
void PointCloudToLaserScanNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
    sub_.unsubscribe();
  }
}

//To display errors if necessary 
void PointCloudToLaserScanNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                             tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
                                                                             << message_filter_->getTargetFramesString()
                                                                             << " at time " << cloud_msg->header.stamp
                                                                             << ", reason: " << reason);
}

//Callback to execute once a message is received in the subscriber to the PointCloud data
void PointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // build laserscan output
  //Declare from the sensor_msgs class (package) a LaserScan datatype and name it output
  sensor_msgs::LaserScan output;
  //Set the LaserScan header to the same header from the incoming PointCloud
  output.header = cloud_msg->header;
  //If the target frame wasn't initialized in the launch file, the default value is "", but if it was initialized, then 
  //assign it to the LaserScan frame_id parameter
  if (!target_frame_.empty())
  {
    output.header.frame_id = target_frame_;
  }

  //Assign the parameters obtained from the launch file or the default values to the LaserScan parameters 
  output.angle_min = angle_min_;
  output.angle_max = angle_max_;
  output.angle_increment = angle_increment_;
  output.time_increment = 0.0;
  output.scan_time = scan_time_;
  output.range_min = range_min_;
  output.range_max = range_max_;

  //So far from the LaserScan message parameters: 
	std_msgs/Header header
	float32 angle_min
	float32 angle_max
	float32 angle_increment
	float32 time_increment
	float32 scan_time
	float32 range_min
	float32 range_max
	float32[] ranges
	float32[] intensities

  //Almost all, but the last two parameters have been already assigned. Meaning that only ranges and intensities arrays are missing

  // determine amount of rays to create
  //This varies from LiDAR to LiDAR depending on its physical characteristics. The formula below tell the ranges and intensities arrays size
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  // Depending if in the launch file or default values it was specified that inf datatype is used or no, include it or configure what data to 
  // use when no obstacle data is obtained 
  if (use_inf_)
  {
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  }
  else
  {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }

  //Declare the pointers for working with the PointCloud data
  //First one will be the output PointCloud or the PointCloud that will be passed to LaserScan once data is converted
  sensor_msgs::PointCloud2ConstPtr cloud_out;
  //Second one is the original PointCloud as received from LiDAR
  sensor_msgs::PointCloud2Ptr cloud;

  // Transform cloud if necessary
  //If the frame id from the LaserScan and the PointCloud are not the same, some transformations must be executed to make sure 
  //the data is seen from the same reference frame so both readings match (data coming from them comes from the same perspective/view in space). 
  if (!(output.header.frame_id == cloud_msg->header.frame_id))
  {
    try
    {
      cloud.reset(new sensor_msgs::PointCloud2);
      tf2_->transform(*cloud_msg, *cloud, target_frame_, ros::Duration(tolerance_));
      cloud_out = cloud;
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
      return;
    }
  }
  else
  {
	 //If they have the same frame id, then the pointers are equaled 
    cloud_out = cloud_msg;
  }

  // Iterate through pointcloud (a pointcloud at one instant of time [the data obtained by the subscriber])
  //Declare three iterators, one per each axis (x, y and z) that a pointcloud has (because it is in 3D space) 
  //The condition is a full loop through the elements in the pointcloud. Meaning that when all the points in the PointCloud have been 
  //traversed the for loop will exit. Increment x, y and z iterators each time the loops finishes. 
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
       iter_z(*cloud_out, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
	//Check to see if any of the axis has a not a number value. If any axis has a not a number value, then a log is triggered, and the continue expression terminates the current iteration of the loop, increments x, y and z values, compares the condition and starts the iteration from the beginning.
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

	 //If the value of the z axis is greater than the maximum height specified in the launch file or less than the minimum height 
	 //tigger a log and the continue expression terminates the current iteration of the loop, increments x, y and z values, compares the condition and starts the iteration from the beginning
    if (*iter_z > max_height_ || *iter_z < min_height_)
    {
      NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
      continue;
    }

	 //Assumes x and y points are the legs (catetos) of a right-angled triangle it returns the hypotenuse 
    double range = hypot(*iter_x, *iter_y);
	 //Check to see if the current point of the PointCloud is within the min and max ranges specified in the launch file
	 //If they aren't (meaning there is a point closer than the min range or a point farther away than the max range) then a log is triggered
	 //and the continue expression terminates the current iteration of the loop, increments x, y and z values, compares the condition and starts the iteration from the beginning
    if (range < range_min_)
    {
      NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_)
    {
      NODELET_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }

	 // Obtain the inverse tangent (in radians) between x and y points to determine if the point is within the specified angle min and angle 
	 //max values of the LiDAR. This means that if it was specified in the launch file that only 180° in front of the LiDAR (90° left and 90°
	 // right) should be used for the conversion, then all the points that are at the back of the LiDAR (other 180°) will be eliminated
	 //A log is triggered and the continue expression terminates the current iteration of the loop, increments x, y and z values, compares the condition and starts the iteration from the beginning
    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max)
    {
      NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
	 //This will first obtain the index of the array of the current point and then compare to see if the pointcloud range value (the distance  
	 //at which the point in the pointcloud is located in space) is smaller than the range (distance) of the LaserScan value already saved.
	 //If the current point in the pointcloud is closer to the LiDAR (or has a smaller range) then the previous value saved in the LaserScan
	 //is overwritten. This will ensure that at different heights, the LaserScan equivalent always keeps the obstacles value that are closer
	 //to the LiDAR. 
    int index = (angle - output.angle_min) / output.angle_increment;
    if (range < output.ranges[index])
    {
      output.ranges[index] = range;
    }
  }

  //Finally, publish the output LaserScan variable with the converted PointCloud data
  pub_.publish(output);
}
}  // namespace pointcloud_to_laserscan

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet)
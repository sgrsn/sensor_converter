#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class RealsenseToVelodyne
{
public:
  RealsenseToVelodyne(
    ros::NodeHandle node, ros::NodeHandle private_nh, 
    std::string const & node_name = ros::this_node::getName())
  {
    output_ = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
    velodyne_scan_ = node.subscribe("/camera/depth_registered/points", 10, &RealsenseToVelodyne::convert, this);
    
    cloud.fields.clear();
    cloud.fields.reserve(fields);
    int offset = 0;
    offset = addPointField(cloud, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(cloud, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(cloud, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(cloud, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(cloud, "ring", 1, sensor_msgs::PointField::UINT16, offset);
    cloud.point_step = offset;
    cloud.row_step = 0 * cloud.point_step;
  }

  void convert(const sensor_msgs::PointCloud2::ConstPtr& realsense_msg)
  {
    sensor_msgs::PointCloud2 realsense_cloud = *realsense_msg;
    cloud.header.stamp = realsense_msg->header.stamp;
    cloud.data.resize(realsense_cloud.width * realsense_cloud.height * cloud.point_step );
    cloud.width = realsense_cloud.width;
    cloud.height = realsense_cloud.height;
    cloud.is_dense = static_cast<uint8_t>(true);

    sensor_msgs::PointCloud2Iterator<float> iter_x = sensor_msgs::PointCloud2Iterator<float>(realsense_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y = sensor_msgs::PointCloud2Iterator<float>(realsense_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z = sensor_msgs::PointCloud2Iterator<float>(realsense_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> new_iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> new_iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> new_iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> new_iter_i = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> new_iter_r = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "ring");
    for (int j = 0, k = 0; j < realsense_cloud.data.size() / 36; j++)
    {
      if(j % 1000 == 0)
      ROS_INFO_STREAM("j: " << j << ", x: " << *iter_x << ", y: " << *iter_y << ", z: " << *iter_z);
      if(!(std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z) ))
      {
        *new_iter_x = *iter_x;
        *new_iter_y = *iter_y;
        *new_iter_z = *iter_z;
        *new_iter_i = 100 / ((*iter_x)*(*iter_x) + (*iter_y)*(*iter_y) + (*iter_z)*(*iter_z));
        *new_iter_r = 1;
        ++new_iter_x;
        ++new_iter_y;
        ++new_iter_z;
        ++new_iter_i;
        ++new_iter_r;
      }
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    cloud.header.frame_id = "velodyne";
    output_.publish(cloud);
  }

  ros::Subscriber velodyne_scan_;
  ros::Publisher output_;
  sensor_msgs::PointCloud2 cloud;
  unsigned int fields = 5;
};


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense2velodyne_node");

  // create conversion class, which subscribes to raw data
  RealsenseToVelodyne r2v_converter(ros::NodeHandle(),
                                  ros::NodeHandle("~"));

  // handle callbacks until shut down
  ros::spin();

  return 0;
}

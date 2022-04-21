#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class RfansToVelodyne
{
public:
  RfansToVelodyne(
    ros::NodeHandle node, ros::NodeHandle private_nh, 
    std::string const & node_name = ros::this_node::getName())
  {
    output_ = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
    velodyne_scan_ = node.subscribe("/rfans_driver/rfans_points", 10, &RfansToVelodyne::convert, this);
    
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

  void convert(const sensor_msgs::PointCloud2::ConstPtr& rfans_msg)
  {
    sensor_msgs::PointCloud2 rfans_cloud = *rfans_msg;
    cloud.header.stamp = rfans_msg->header.stamp;
    cloud.data.resize(rfans_cloud.width * rfans_cloud.height * cloud.point_step );
    cloud.width = rfans_cloud.width;
    cloud.height = rfans_cloud.height;
    cloud.is_dense = static_cast<uint8_t>(true);

    sensor_msgs::PointCloud2Iterator<float> iter_x = sensor_msgs::PointCloud2Iterator<float>(rfans_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y = sensor_msgs::PointCloud2Iterator<float>(rfans_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z = sensor_msgs::PointCloud2Iterator<float>(rfans_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i = sensor_msgs::PointCloud2Iterator<float>(rfans_cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> iter_l = sensor_msgs::PointCloud2Iterator<float>(rfans_cloud, "laserid");
    sensor_msgs::PointCloud2Iterator<float> new_iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> new_iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> new_iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> new_iter_i = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> new_iter_r = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "ring");
    for (int j = 0; j < rfans_cloud.height*rfans_cloud.width; j++)
    {
      //if(j % 1000 == 0)
        //ROS_INFO_STREAM("j: " << j << ", x: " << *iter_x << ", y: " << *iter_y << ", z: " << *iter_z << ", size: ");
      if(!(std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z) ))
      {
        *new_iter_x = *iter_x;
        *new_iter_y = *iter_y;
        *new_iter_z = *iter_z;
        *new_iter_i = *iter_i;
        *new_iter_r = *iter_l;
        ++new_iter_x;
        ++new_iter_y;
        ++new_iter_z;
        ++new_iter_i;
        ++new_iter_r;
      }
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
      ++iter_l;
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
  ros::init(argc, argv, "rfans2velodyne_node");

  // create conversion class, which subscribes to raw data
  RfansToVelodyne r2v_converter(ros::NodeHandle(),
                                  ros::NodeHandle("~"));

  // handle callbacks until shut down
  ros::spin();

  return 0;
}

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class pointcloud2_transform_pcl

private:
    ros::Subscriber sub_cloud2;
    ros::Publisher pub_pcl;

    void Callback(const sensor_msgs::PointCloud2ConstPtr& input) 
    {
      pcl::PointCloud<pcl::PointXYZ> p_cloud;
      pcl::fromROSMsg (*input, p_cloud);
    }

public:
    pointcloud2_transform_pcl()
    {
      //PointCloud2をsubscribeするノードハンドル
      ros::NodeHandle cloud2_nh("~");

      //PCLをpublishするノードハンドル
      ros::NodeHandle pcl_nh;
      pub_pcl = pch_nh.advertise<


      pub_pcl = pcl_nh.subscribe("pcl",100,
                                 &pointcloud2_transform_pcl::Callback,this);


  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 10, Callback);
}

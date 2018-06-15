#include <ros/ros.h>                    
#include <sensor_msgs/PointCloud2.h>     //PointCloud

#include <pcl/point_types.h>  //PCL
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

/*** PointCloud2型 を受け取った際、PCLに変換する ***/

class pointcloud2_transform_pcl
{
private:
    ros::Subscriber sub_point_cloud2; //PointCloud2型のsubscriber

    ros::Publisher pub_pcl; //PCLのpublisher



    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;

    /*** URGのデータを受け取った際に以下を行う ***/
    //①　座標変換待ち
    //②　URGデータを受け取った際、PointCloud2型に変換する
    //③　PointCloud型に返還後,publish

    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
      //①　
      if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))) //1.0停止
         { 
           return;
         }
       //②　
       sensor_msgs::PointCloud2 cloud2;
       projector_.transformLaserScanToPointCloud("/base_link",*scan_in,
          cloud2,listener_);
       
       //③　
       pub_cloud2.publish(cloud2);

    }

};

int main(int argc, char *argv[])
{
    //ノードの初期化
    ros::init(argc, argv, "urg_transform_pointcloud2");

    //ypspur_run_testクラスの　robot_testを作成
    urg_transform_pointcloud2 urg_transform;
    
    ros::spin();
}


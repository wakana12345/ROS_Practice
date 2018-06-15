#include <ros/ros.h>                    
#include <geometry_msgs/Twist.h>        //msg型
#include <tf/transform_listener.h>      //TF

#include <sensor_msgs/LaserScan.h>      //Laser
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>     //PointCloud

/*** URG(Laser scan data) を受け取った後　PointCloud型に変換する ***/

class urg_transform_pointcloud2
{
private:
    ros::Subscriber sub_scan;  //URGデータのsubscriber
    ros::Publisher pub_cloud2; //pointCloud2のpublisher 

    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;

    /*** URGのデータを受け取った際に以下を行う ***/
    //①　座標変換待ち
    //②　URGデータを受け取った際、PointCloud2型に変換する
    //③　PointCloud型に返還後,publish

    void scan_transform(const sensor_msgs::LaserScan::ConstPtr &scan_in)
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


public:
   urg_transform_pointcloud2()
   {
     //URGデータをsubscribeするノードハンドル
     ros::NodeHandle urg_node_handle("~");

     //PointCloud2をpublishするノードハンドルとPointCloud2のトピック
     ros::NodeHandle pointcloud2_nh;
     pub_cloud2 = pointcloud2_nh.advertise<sensor_msgs::PointCloud2>("cloud_2",100);

    //URGデータを読み込んだを受け取った際 cb_scan関数を呼び出す
    sub_scan = urg_node_handle.subscribe("/scan",5,
                                         &urg_transform_pointcloud2::scan_transform,this);

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


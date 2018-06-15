#include <ros/ros.h>                    
#include <nav_msgs/Odometry.h>          //Robot のodometry 
#include <geometry_msgs/Twist.h>        //msg型


#include <tf/transform_datatypes.h>     //TF
#include <tf/transform_listener.h>      //TF

#include <sensor_msgs/LaserScan.h>      //Laser
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>     //PointCloud

#include <message_filters/subscriber.h> //message_filters
#include <tf/message_filter.h>
#include <filters/filter_chain.h>

class ypspur_run_test_node
{
private:

    ros::Publisher pub_cloud;

    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;

    /*** URGのデータを受け取る ***/
    //①　URGデータを受け取った際、PointCloud型(rosmsg) に変換する
    //②　PointCloud型(rosmsg) に返還後、x,y,zの座標値を取り出す
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
      //base_linkとURGの座標のどちらも利用可能(座標変換するために)になるまで待つ
      if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){  //1.0停止
     return;
     }
       //①　
       sensor_msgs::PointCloud2 cloud;
       projector_.transformLaserScanToPointCloud("/base_link",*scan_in,
          cloud,listener_);
       
       pub_cloud.publish(cloud);

       //② 

  // Do something with cloud.

    }



};

int main(int argc, char *argv[])
{
    //ノードの初期化
    ros::init(argc, argv, "ypspur_run_test");
    //ypspur_run_testクラスの　robot_testを作成
    ypspur_run_test_node robot_test;
    ros::spin();
}


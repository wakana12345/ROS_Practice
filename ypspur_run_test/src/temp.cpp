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
    ros::Subscriber sub_odom;
    ros::Publisher pub_twist;
    ros::Subscriber sub_scan;

    ros::Publisher pub_cloud;

    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;

    //ロボットの速度を受け取る
    void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
    {
      ROS_INFO("vel %f",
      msg->twist.twist.linear.x);
    }

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



public:
    ypspur_run_test_node()
    {
        //cmd_vel を送る publishするノードハンドル
        ros::NodeHandle public_node_handle;

        //ocomを受け取るsubscribeするノードハンドル
        ros::NodeHandle private_node_handle("~"); 

        //Laserデータを受け取る　subscribeするノードハンドル
        ros::NodeHandle urg_node_handle("~"); 


        //速度指令を送る
        pub_twist = public_node_handle.advertise<geometry_msgs::Twist>(
                    "cmd_vel", 100);
 
        //速度を読み取る
        sub_odom = private_node_handle.subscribe("odom", 1,
                                &ypspur_run_test_node::cb_odom, this);
       
        //URGデータを読み取る
        sub_scan = urg_node_handle.subscribe("/scan",5,
                                &ypspur_run_test_node::cb_scan, this);


    }
    void mainloop()
    {
        ROS_INFO("Hello ROS World!");

        ros::Rate rate(40.0);
        while(ros::ok())
        {
            ros::spinOnce();
            // ここに速度指令の出力コード
            geometry_msgs::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = 0.05;
            cmd_vel_msg.angular.z = 0.0;
            
            //速度を書き込む
            pub_twist.publish(cmd_vel_msg);

            rate.sleep();
        }
        // ここに終了処理のコード
    }
};

int main(int argc, char *argv[])
{
    //ノードの初期化
    ros::init(argc, argv, "ypspur_run_test");
    //ypspur_run_testクラスの　robot_testを作成
    ypspur_run_test_node robot_test;
    //mainループの実行
    robot_test.mainloop();
}


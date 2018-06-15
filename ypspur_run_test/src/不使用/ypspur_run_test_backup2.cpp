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

    //ロボットの速度を受け取る
    void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
    {
      ROS_INFO("vel %f",
      msg->twist.twist.linear.x);
    }

    /*** URGのデータを受け取る ***/
    //①　URGデータを受け取った際、PointCloud型(rosmsg) に変換する
    //②　PointCloud型(rosmsg) に返還後、x,y,zの座標値を取り出す
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
      ros::NodeHandle node;
      laser_geometry::LaserProjection projector;
      tf::TransformListener listener;

      message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
      tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;
      ros::Publisher scan_pub;

      LaserScanToPointCloud(ros::NodeHandle n) :
      node(n),
      laser_sub(node, "base_scan", 10),
      laser_notifier(laser_sub,listener, "base_link", 10)

  {
      printf("setting up callback\r\n");
      laser_notifier.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
      laser_notifier.setTolerance(ros::Duration(0.01));
      scan_pub = node.advertise<sensor_msgs::PointCloud>("my_cloud",1);
      printf("set up callback\r\n");
  }

      //PointCloud型の定義
      sensor_msg::PointCloud localcloud;
      ROS_INFO("%ds:%lfm\r\n",msg->header.stamp.sec,msg->ranges[128]);

      //ここは実行　エラーが出れば catch文へ移動
      try
      {
        //LaserScanデータからPointcloud型に変換
        //引数は親フレーム, レーザスキャンデータ, pointcloud型, listener
        projector.transformLaserScanToPointCloud("base_link",*msg, localcloud,listener);
      }
      
      //例外時の対応
      catch(tf::TransformException& e)
      {
        std::cout << e.what();
        return;
      }

      //表示
      ROS_INFO("(%f,%f,%f)\r\n",localcloud.points.data()->x,localcloud.points.data()->y,localcloud.points.data()->z);
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


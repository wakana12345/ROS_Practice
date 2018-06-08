#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>

#include <sensor_msgs/LaserScan.h>
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

    //URGのデータを受け取る
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
      int i = msg->ranges.size() / 2;
      if(msg->ranges[i] < msg->range_min ||  //最小値以上に小さい場合
         msg->ranges[i] > msg->range_max ||  //最大値以上に大きい場合
         std::isnan(msg->ranges[i] ))        //無限遠の場合
         {
           ROS_INFO("front-range : measurement erro");  //エラー表示
         }

      else
      {
        ROS_INFO("front-range :%0.3f",
          msg->ranges[msg->ranges.size() / 2] );  //ちょうど真ん中　つまり真正面の距離を表示
      }
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
        ROS_INFO("abbaHello ROS World!");

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
    ros::init(argc, argv, "my_pcl_tutorial");
    //ypspur_run_testクラスの　robot_testを作成
    ypspur_run_test_node robot_test;
    //mainループの実行
    robot_test.mainloop();
}


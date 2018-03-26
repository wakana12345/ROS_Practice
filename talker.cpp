#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  //ノードの作成
  ros::init(argc, argv,"vel_publisher");

  //ノードハンドルの作成
  ros::NodeHandle n;

  //文字型格納変数の定義
  char ss;

  //publisherの作成
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("chatter",1000);

  std::cout << "f 前 b後・・・\n";
  
  while(ros::ok())
  {
    //Twist型の変数の定義
    geometry_msgs::Twist vel;
    
    //文字変数にキーボード入力値を代入(キーボード入力)
    std::cin >> ss;

    //fであれば　Twsit型に0.5格納
    if(ss == 'f')
      vel.linear.x = 0.5;
    
    //b
    if(ss == 'b')
      vel.linear.x = -0.5;

    //l
    if(ss == 'l')
      vel.linear.z = 1.0;

    //r
    if(ss == 'f')
      vel.linear.z = -1.0;

    //q
    if(ss=='q')
      break;
    
    //Twist型変数をpublish
    chatter_pub.publish(vel);
  }

return 0;

}

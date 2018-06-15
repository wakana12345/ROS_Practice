#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){

//初期化とsenseor_tf_testというノード作成
  ros::init(argc, argv, "robot_tf_publisher");

//ノードハンドルの作成
  ros::NodeHandle n;


//100Hz止まるよう設定値を定義
  ros::Rate r(100);


//TransformBroadcaster という型(各座標の情報を流す)を作成
  tf::TransformBroadcaster broadcaster;

//urgとロボットの距離の差分を行列に格納
  while(n.ok()){

//クオータニオンの座標の型を作成(ROSはクータにオン系で扱っているため)
    tf::Quaternion quaternion;

//URGの取り付けがひっくり返っていないため0を使用　回転定義
    quaternion.setRPY(0, 0, 0);    //if your URG is not reversed, use this program
    //quaternion.setRPY(M_PI, 0, 0); //if your URG is reversed, use this program 

//ノードハンドルnに対して返り値okの場合は以下を繰り返す
    broadcaster.sendTransform(

//TransformBroadcasterを使って、座標変換したものを>    送る際は5つの引数をとる
//1:URGが回転定義のquaternion 2:URGとロボットの座標差分,3:現在のタイムスタンプ　4:親ノードの名前 5:子ノ>    ードの名前
     
      tf::StampedTransform(
        tf::Transform(quaternion, tf::Vector3(0.06, 0.0, 0.1)),
        ros::Time::now(),"base_link", "laser"));

//sleepにて100Hzとまる
    r.sleep();
  }
}

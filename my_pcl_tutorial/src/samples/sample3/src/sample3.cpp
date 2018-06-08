/*

　山彦セミナー(点群処理：Point Cloud Library)用　サンプルプログラム
　2013.4.3 MTM 作成

　---
　KinectまたはXtionから３次元点群を取り込んで表示するプログラムです。
　実行するときはKinectかXtionを用意しましょう。(旧人から借りるといいかも)

　※grabberまわりのもっとスマートな書き方知っている方教えてください

*/


//--------------------------------------------
//include
//--------------------------------------------
#include<cstdio> //stdio.hのc++版
#include<csignal> //signal.hのc++版
//PCL
#include<pcl/point_types.h> //pcl::PointXYZクラスを使う
#include<pcl/point_cloud.h> //pcl::PointCloudクラスを使う
#include<pcl/visualization/cloud_viewer.h> //cloudViewerクラスを使う
#include<pcl/io/openni_grabber.h> //OpenNIGrabberクラスを使う

//--------------------------------------------
//ctrl-cで停止させるための記述
//--------------------------------------------
bool gIsShuttingDown = false;

void ctrl_c( int aStatus )
{
    gIsShuttingDown = true;
    signal(SIGINT, NULL);
}

//--------------------------------------------
//OpenNIGrabberを使うクラス
//--------------------------------------------
class myGrabber
{
    public:

        myGrabber(): //コンストラクタでフラグを初期化
            run(false),
            flag(false)
        {}

        virtual ~myGrabber()
        {
            stop();
        }

        void start() //myGrabberの初期処理
        {
            if( run )return; //すでにGrabberが起動していたらやめる

            interface = new pcl::OpenNIGrabber( "" , pcl::OpenNIGrabber::OpenNI_QVGA_30Hz ); //OpenNIGrabberの初期化　第２引数を変えると取得する距離画像の大きさなどを変えることが出来る。
            boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&myGrabber::interfaceCallback, this, _1);
            interface->registerCallback(f); //コールバック関数の登録
            interface->start(); //Grabberを開始
            run = true;

            printf("myGrabber: Started\n");
        }

        void stop() //myGrabberの終了処理
        {
            if( !run )return; //すでにGrabberが終了していたらやめる

            interface->stop(); //Grabberを終了
            run = false;
            flag = false;

            printf("myGrabber: Stopped\n");
        }

        bool update( bool& signal ) //クラウドを更新する
        {
            if( !run )return false; //interface初期化フラグが立っていなかったらやめる

            flag = true; //更新要求をする
            while( flag && !signal )usleep(10000); //更新要求フラグがfalseになるまで待つ

            if( signal )return false;
            else return true;
        }

        pcl::PointCloud< pcl::PointXYZRGBA > cloud; //実際に更新されるクラウド

    private:

        void interfaceCallback( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & input_cloud ) //OpenNIGrabberがクラウドを取得するたびに呼び出される関数
        {
            if( !flag )return; //更新要求が来ていなかったらやめる

            cloud = *input_cloud; //更新要求が来ていればcloudにコピー
            flag = false; //更新要求フラグをおる
        }

        bool run; //OpenNIGrabberの初期化完了フラグ
        bool flag; //クラウドの更新要求フラグ
        pcl::Grabber* interface; //クラウドを取得するためのクラス
};

//--------------------------------------------
//本体
//--------------------------------------------
int main( int argc , char **argv )
{
     signal(SIGINT, ctrl_c);				//ctrl-c停止の設定

    myGrabber grabber; //自作Grabberインスタンスの作成
    grabber.start(); //grabberの初期処理

    pcl::visualization::CloudViewer viewer("PCL Kinect Viewer");	//クラウド表示のためのクラスを定義

    while( !gIsShuttingDown && !viewer.wasStopped() ) //ctrl-cを押されるか、ビューワが止められるまで繰り返す
    {
            grabber.update( gIsShuttingDown ); //grabberをアップデート
            viewer.showCloud( grabber.cloud.makeShared() ); //grabberのクラウドを表示
            usleep(1000);
    }

    grabber.stop(); //grabberの終了処理

    return 0;
}

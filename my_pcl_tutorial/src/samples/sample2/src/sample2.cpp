/*

　山彦セミナー(点群処理：Point Cloud Library)用　サンプルプログラム
　2013.4.3 MTM 作成

　---
　ポイントクラウドクラスの基本操作のサンプルです

*/

//--------------------------------------------
//include
//--------------------------------------------
#include<iostream> //c++の入出力
//PCL
#include<pcl/point_types.h> //pcl::PointXYZクラスを使う
#include<pcl/point_cloud.h> //pcl::PointCloudクラスを使う
#include<pcl/visualization/cloud_viewer.h> //cloudViewerクラスを使う
#include<pcl/io/pcd_io.h> //PCDファイルの入出力に使う

//--------------------------------------------
//クラウドのデータを端末に表示する自作関数
//--------------------------------------------
void dispCloud( pcl::PointCloud< pcl::PointXYZ >& input )	//自作表示用関数
{
    std::cout 	<< "==================\n"
                << input  //入力クラウドの基本データの表示
                << "-----\n";
    for( unsigned int i=0; i < input.size(); i++ )
    {
        std::cout << "points[" << i << "]: " << input.points[i] << "\n"; //入力クラウドのそれぞれの点の座標を表示
    }
    std::cout 	<< "==================\n";
}

//--------------------------------------------
//メイン
//--------------------------------------------
int main( int argc , char **argv )
{
    //+++++
    //0.クラウドの初期化
    //+++++
    pcl::PointCloud< pcl::PointXYZ > cloud; //ポイントクラウド(宣言しただけでは空のクラウド)
    dispCloud( cloud );	//クラウドを表示

    //+++++
    //1.クラウドの操作1
    // resize()点の数を変更、x,y,zへ座標を直接代入
    //+++++
    std::cout << "Operation 1" << std::endl;
    cloud.points.resize( 3 ); //ポイントクラウドの点数を3個にする
    cloud.points[0].x = 0.0; cloud.points[0].y = 0.0; cloud.points[0].z = 0.0; //直接アクセスして指定する
    cloud.points[1].x = 0.5; cloud.points[1].y = 0.5; cloud.points[1].z = 0.5;
    cloud.points[2].x = 1.0; cloud.points[2].y = 1.0; cloud.points[2].z = 1.0;
    dispCloud( cloud );	//クラウドを表示

    //+++++
    //2.クラウドの操作2
    // clear()で全消去、push_back()で１点追加
    //+++++
    std::cout << "Operation 2" << std::endl;
    cloud.clear(); //ポイントクラウドの点を全消去
    for(unsigned int i=0; i<5; i++)
    {
            pcl::PointXYZ p; //1つの点を定義
            p.x = (float)i;			//点のX,Y,Z座標を適当に定義
            p.y = (float)i - 1;
            p.z = (float)i - 2;

            cloud.points.push_back( p ); //クラウドに点Pを追加
    }
    dispCloud( cloud );	//クラウドを表示

    //+++++
    //3.クラウドのコピー
    // イコール代入などでコピーも出来る
    //+++++
    pcl::PointCloud< pcl::PointXYZ > cloud2; //あたらしいクラウドの作成
    cloud2 = cloud; //コピーも出来る
    std::cout << "Operation 3" << std::endl;
    dispCloud( cloud );	//クラウドを表示

    //+++++
    //4.クラウドのパラメータの調整
    // cloudのwidthやheightなどのパラメータの調整
    //+++++
    //これらのパラメータは設定していなくても問題ないが、PCLの関数によっては「cloud.size()とcloud.width * cloud.heightが違う」などのエラーを返すものもあるので注意！
    std::cout << "Operation 4" << std::endl;
    cloud.height = 1; //クラウドのデータ配列の縦の個数を設定
    cloud.width = 3; //クラウドのデータ配列の横の個数を設定
    cloud.is_dense = true; //クラウドが密かどうか(nanの点を含むかどうか)の設定
    dispCloud( cloud );	//クラウドを表示

    //+++++
    //5.クラウドの読み込み
    // 予め用意したサンプルのPCDファイルを読み込む
    //+++++
    if( pcl::io::loadPCDFile( "../data/loadSample.pcd" , cloud ) != -1 ) //PCDファイル(クラウドを保存するためのファイル形式)の読み込み関数 　戻り値-1が失敗
    {
            std::cout << "Operation 5( Loaded \"loadSample.pcd\" )" << std::endl;
            dispCloud( cloud );	//クラウドを表示
    }
    else
    {
            std::cout << "[ERROR] Loading \"loadSample.pcd\" Failed!!" << std::endl;
    }

    //+++++
    //6.クラウドの書き込み
    // 1000点が入ったクラウドを適当に作って、PCDファイルに書きこむ
    //+++++
    std::cout << "Operation 6" << std::endl;
    cloud.clear(); //ポイントクラウドの点を全消去
    for(unsigned int i=0; i<10; i++)
    for(unsigned int j=0; j<10; j++)
    for(unsigned int k=0; k<10; k++)
    {
        pcl::PointXYZ p; //1つの点を定義
        p.x = (float)i;			//点のX,Y,Z座標を適当に定義
        p.y = (float)j;
        p.z = (float)k;

        cloud.points.push_back( p ); //クラウドに点Pを追加
    }
    cloud.width = 1000;
    cloud.height = 1;

    if( pcl::io::savePCDFile( "../data/saveSample.pcd" , cloud ) != -1 ) //PCDファイルの書き込み関数　戻り値-1が失敗
    {
        std::cout << "\"saveSample.pcd\" Saved Successful\n" << std::endl;
    }
    else
    {
        std::cout << "[ERROR] Saving \"saveSample.pcd\" Failed!!" << std::endl;
    }

    return 0;
}

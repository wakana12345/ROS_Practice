/*

　山彦セミナー(点群処理：Point Cloud Library)用　サンプルプログラム
　2013.4.3 MTM 作成

　---
　2つのPCDファイルを読み込んで、ICP(Iterative Closest Point)と呼ばれる手法で位置合わせするプログラムです。
　
*/


//--------------------------------------------
//include
//--------------------------------------------
#include<cstdio> //stdio.hのc++版
//PCL
#include<pcl/point_types.h> //pcl::PointXYZクラスを使う
#include<pcl/point_cloud.h> //pcl::PointCloudクラスを使う
#include<pcl/registration/icp.h> //ICPをするためのインクルードファイル
#include<pcl/io/pcd_io.h> //PCDファイルの入出力に使う

//--------------------------------------------
//本体
//--------------------------------------------
int main( int argc , char **argv )
{
    //++++
    // 引数の解析
    //++++
    if( argc < 3 ) //引き数が少ない場合はエラーを返して終了
    {
            printf("[ERROR]Please Input Template And Target PCD File Name!\n");
            printf("ex) %s ../data/templateA.pcd ../data/targetA.pcd\n" , argv[0] );
            return 0;
    }

    //++++
    // PCDの読み取り
    //++++
    pcl::PointCloud< pcl::PointXYZRGBA > temp , targ;	//位置合わせのテンプレート・ターゲットクラウド
    if( pcl::io::loadPCDFile( argv[1] , temp ) == -1 )
    {
            printf("[ERROR]Loading Template PCD File \"%s\" Failed!!\n" , argv[1]);
            return 0;
    }
    printf("Template PCD File \"%s\" Loaded Successful.\n" , argv[1]);
    if( pcl::io::loadPCDFile( argv[2] , targ ) == -1 )
    {
            printf("[ERROR]Loading Target PCD File \"%s\" Failed!!\n" , argv[2]);
            return 0;
    }
    printf("Target PCD File \"%s\" Loaded Successful.\n" , argv[2]);

    //++++
    // ICPの設定
    //++++
    pcl::IterativeClosestPoint< pcl::PointXYZRGBA, pcl::PointXYZRGBA > icp; //ICPを行うクラス
    icp.setMaximumIterations( 30 ); //ICPの最大繰り返し回数の設定
    icp.setInputSource( temp.makeShared() ); //テンプレートのクラウドをセット
    icp.setInputTarget( targ.makeShared() ); //ターゲットのクラウドをセット

    //++++
    // ICPの実行
    //++++
    printf("Calculating...\n"); //ICPの結果表示
    pcl::PointCloud< pcl::PointXYZRGBA > result; //結果出力用のクラウドをセット
    icp.align( result ); //ICPを実行して結果をresultにセット
    Eigen::Matrix4f trans = icp.getFinalTransformation(); //最終変換行列を取得

    //++++
    // ICPの結果を表示・保存
    //++++
    printf("ICP Result------------\n"); //ICPの結果表示
    if( icp.hasConverged() )printf("Converged: TRUE\n"); //ICPが収束したか？
    else printf("Converged: FALSE\n");
    printf("Fitness Score: %lf\n" , icp.getFitnessScore() ); //ICPの収束の精度は？
    printf("Final Transformation Matrix:\n"); //最終変換行列の表示
    printf("%4.1lf,%4.1lf,%4.1lf,%4.1lf\n", trans(0,0) , trans(0,1) , trans(0,2) , trans(0,3) );
    printf("%4.1lf,%4.1lf,%4.1lf,%4.1lf\n", trans(1,0) , trans(1,1) , trans(1,2) , trans(1,3) );
    printf("%4.1lf,%4.1lf,%4.1lf,%4.1lf\n", trans(2,0) , trans(2,1) , trans(2,2) , trans(2,3) );
    printf("%4.1lf,%4.1lf,%4.1lf,%4.1lf\n", trans(3,0) , trans(3,1) , trans(3,2) , trans(3,3) );
    printf("----------------------\n");

    pcl::io::savePCDFileASCII( "../data/output.pcd" , result ); //結果の保存
    printf("Result PCD File \"output.pcd\" Saved Successful.\n");

    return 0;
}

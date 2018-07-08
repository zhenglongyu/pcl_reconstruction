#include <iostream>

//pcl libraries
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<pcl/features/normal_3d.h>

#include<cv.h>
#include<highgui.h>
#include<opencv2/opencv.hpp>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//相机内参
double camera_factor = 1000;
double camera_cx = 325.5;
double camera_cy = 253.5;
double camera_fx = 518.0;
double camera_fy = 519.0;

int main(int argc, char** argv)
{
    cv::Mat depth_map = cv::imread(argv[1]);
    //load pcd
    PointCloud::Ptr cloud(new PointCloud);
    cloud->width = depth_map.cols;
    cloud->height = depth_map.rows;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width*cloud->height);

//    int point_id = 0;
    for(int i = 0;i<depth_map.rows;i++)
    {
        for(int j = 0;j<depth_map.cols;j++)
        {
            // 获取深度图中(i,j)处的值
            ushort d = depth_map.ptr<ushort>(i)[j];
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera_factor;
            p.x = (j- camera_cx) * p.z / camera_fx;
            p.y = (i - camera_cy) * p.z / camera_fy;

            // 把p加入到点云中
            cloud->at(j,i) = p;
        }
    }


    cout<<"the width is "<<cloud->width<<", the height is "<<cloud->height<<std::endl;
//    //remove outliers
//    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//    outrem.setInputCloud(cloud);
//    outrem.setRadiusSearch(0.01);
//    outrem.setMinNeighborsInRadius(10);
//    outrem.filter(*cloud);

    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<PointT, pcl::Normal>(cloud, normals);
//        viewer.addPointCloud(cloud);

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    return 0;
}

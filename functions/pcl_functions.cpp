#include <Eigen/Geometry>
#include "pcl/point_cloud.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <iostream>
#include <chrono>

#include "pcl_functions.hpp"

using namespace std;

extern const double fx =672.8525634486431;
extern const double fy = 672.758607019733;
extern const double cx = 399.48117481367757;
extern const double cy = 300.6428102963154;

extern const int HEIGHT = 592;
extern const int WIDTH = 800;

//extern const double fx;
//extern const double fy;
//extern const double cx;
//extern const double cy;

//extern const int HEIGHT;
//extern const int WIDTH;



pcl::PointXYZRGB addcol(pcl::PointXYZ p, uint8_t r = 255, uint8_t g = 255, uint8_t b = 255){
    pcl::PointXYZRGB rgb;
    rgb.rgb = rgb2float(r,g,b);
    rgb.x = p.x;
    rgb.y = p.y;
    rgb.z = p.z;
    return rgb;
}

float rgb2float(uint8_t r, uint8_t g, uint8_t b){
    uint32_t color = r << 16 | g << 8 | b;
    return *reinterpret_cast<float*>(&color);
}

void colorless_to_rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud){
    pcl::PointXYZ pt;
    pcl::PointXYZRGB rgb;
    rgb_cloud->clear();
    for(int i = 0; i < cloud->points.size(); i++){
        pt = cloud->points[i];
        rgb.x = pt.x;
        rgb.y = pt.y;
        rgb.z = pt.z;
        rgb.rgb = rgb2float(255,255,255);
        rgb_cloud->points.push_back(rgb);
    }
    cloud->height = cloud->points.size();
    cloud->width = 1;
}

pcl::PointXYZRGB Eigen_to_pcl(Eigen::Vector3f a, uint8_t r = 255, uint8_t g = 255, uint8_t bl = 255){
    pcl::PointXYZRGB b;
    b.x = a.x(); b.y = a.y(); b.z = a.z(); b.rgb = rgb2float(r,g,bl);
    return b;
}

void rgb_to_colorless(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PointXYZ pt;
    pcl::PointXYZRGB rgb;
    cloud->clear();
    for(int i = 0; i < rgb_cloud->points.size(); i++){
        rgb = rgb_cloud->points[i];
        pt.x = rgb.x;
        pt.y = rgb.y;
        pt.z = rgb.z;
        cloud->points.push_back(pt);
    }
    cloud->height = cloud->points.size();
    cloud->width = 1;
}



pcl::PointXYZRGB cv_to_pcl(cv::Point3i pt, uint32_t rgb){
    pcl::PointXYZRGB p;
    p.x = ((double)pt.x - cx)/fx * (double)pt.z / 1000;
    p.y = ((double)pt.y - cy)/fy * (double)pt.z / 1000;
    p.z = (double)pt.z / 1000;
    p.rgb = *reinterpret_cast<float*>(&rgb);
    return p;
}

void remove_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZ>::Ptr outliers){
    pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(indices);
      extract.setNegative(true);
      extract.filter(*outliers);
      outliers->height = outliers->points.size();
      outliers->width = 1;
}

void remove_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers){
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(indices);
      extract.setNegative(true);
      extract.filter(*outliers);
      outliers->height = outliers->points.size();
      outliers->width = 1;
}

pcl::ModelCoefficients::Ptr find_plane_ransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double threshold, 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers = nullptr){

  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  rgb_to_colorless(cloud, basic_cloud);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (threshold);
  seg.setInputCloud(basic_cloud);
  seg.segment(*inliers, *coefficients);

  for(int i = 0; i < 4; i++){
      cout << "coef " << i << " : " << coefficients->values[i] << endl;
  }

  uint8_t r = 255,b=0,g=0;
  uint32_t color = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

  for(int i = 0; i < inliers->indices.size(); i++){
      cloud->points[inliers->indices[i]].rgb = *reinterpret_cast<float*>(&color);
  }
  if(outliers != nullptr){
      remove_cloud(cloud, inliers,outliers);
  }
  return coefficients;
}

void convert_to_cloud(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int limit = 100000){
    cv::Mat image;
    cv::FileStorage fs;
    fs.open(name, cv::FileStorage::READ);
    fs["matName"] >> image;
    imshow("name",image);
    //waitKey();
    pcl::PointXYZRGB pt;
    cv::Point3i tmp;
    uint8_t r = 255,b=255,g=255;
    uint32_t rgb =(static_cast<std::uint32_t>(r) << 16 |
              static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));;
    for(int i = 0; i < HEIGHT; i++){
        for(int j = 0; j < WIDTH; j++){
            tmp.x = j;
            tmp.y = i;
            tmp.z = image.at<uint16_t>(i,j);
            if(tmp.z != 65535 && tmp.z < limit){
                pt = cv_to_pcl(tmp, rgb);
                cloud->points.push_back(pt);
            }
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr make_plane_cloud(pcl::ModelCoefficients::Ptr coefficients){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB p;
    for(float i = -20; i < 20; i +=0.5){
        for(float j = -20; j < 120; j += 0.5){
            //p.x = i; p.y = j;
            //p.z = (-coefficients->values[3] - coefficients->values[0] * i - coefficients->values[1] * j)/coefficients->values[2];
            p.x = i; p.z = j;
            p.y = (-coefficients->values[3] - coefficients->values[0] * i - coefficients->values[2] * j)/coefficients->values[1];
            p.rgb = rgb2float(0, 255,0);
            cloud->push_back(p);
        }
    }
    return cloud;
}

My_line plane_intersection(Eigen::Vector4f p1, Eigen::Vector4f p2){
    Eigen::Vector3f v1(p1[0],p1[1],p1[2]);
    Eigen::Vector3f v2(p2[0],p2[1],p2[2]);

    Eigen::Vector3f lin_v = v1.cross(v2);
    Eigen::Vector3f lin_p;
    lin_p[0] = 0;
    lin_p[2] = ((p2[1]/p1[1])*p1[3] - p2[3])/(p2[2] - p1[2]*p2[1]/p1[1]);
    lin_p[1] = (-p1[2]*lin_p[2] - p1[3])/ p1[1];
    My_line my_line;
    my_line.line = lin_v;
    my_line.point = lin_p;
    return my_line;
}

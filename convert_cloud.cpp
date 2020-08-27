#include <Eigen/Geometry>
#include <pcl/pcl_config.h>
#include "pcl/point_cloud.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <iostream>
#include <chrono>

#include "functions/pcl_functions.hpp"

extern const double fx;
extern const double fy;
extern const double cx;
extern const double cy;

extern const int HEIGHT;
extern const int WIDTH;

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

std::vector<float> find_pillar_planes_ransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double threshold);
vector<float> find_pillar_planes_ransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double threshold){
//finds 4 planes which represent pillar. First find two parallel planes, then 2 perpendicular
//TODO try finding perpendicular planes by projection -> more robust ? 
  vector<float> plane_coeff;

  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZ>);

  rgb_to_colorless(cloud, basic_cloud);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (threshold);
  seg.setInputCloud(basic_cloud);
  seg.segment(*inliers, *coefficients);
  for(int i = 0; i < 4; i++){plane_coeff.push_back(coefficients->values[i]);}
  
  cloud->clear();
  //for(int i = 0; i < inliers->indices.size(); i++){
      //cloud->points[inliers->indices[i]].rgb = rgb2float(255,0,0);
  //}
  pcl::PointXYZ pt;
  pcl::PointXYZRGB rgb_pt;
  for(int i = 0; i < inliers->indices.size(); i++){
      pt = basic_cloud->points[inliers->indices[i]];
      cloud->points.push_back(addcol(pt,255,0,0));
  }

  remove_cloud(basic_cloud, inliers, outliers);
  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  seg.setEpsAngle(0.1);
  seg.setAxis(Eigen::Vector3f(coefficients->values[0],coefficients->values[1],coefficients->values[2]));
  seg.setInputCloud(outliers);
  seg.segment(*inliers,*coefficients);
  for(int i = 0; i < 4; i++){plane_coeff.push_back(coefficients->values[i]);}

  for(int i = 0; i < inliers->indices.size(); i++){
      pt = outliers->points[inliers->indices[i]];
      cloud->points.push_back(addcol(pt,0,255,0));
  }

  remove_cloud(outliers, inliers, outliers);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis(Eigen::Vector3f(coefficients->values[0],coefficients->values[1],coefficients->values[2]));
  seg.setInputCloud(outliers);
  seg.segment(*inliers,*coefficients);
  for(int i = 0; i < 4; i++){plane_coeff.push_back(coefficients->values[i]);}

  for(int i = 0; i < inliers->indices.size(); i++){
      pt = outliers->points[inliers->indices[i]];
      cloud->points.push_back(addcol(pt,0,0,255));
  }

  remove_cloud(outliers, inliers, outliers);
  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  seg.setAxis(Eigen::Vector3f(coefficients->values[0],coefficients->values[1],coefficients->values[2]));
  seg.setInputCloud(outliers);
  seg.segment(*inliers,*coefficients);
  for(int i = 0; i < 4; i++){plane_coeff.push_back(coefficients->values[i]);}

  for(int i = 0; i < inliers->indices.size(); i++){
      pt = outliers->points[inliers->indices[i]];
      cloud->points.push_back(addcol(pt,255,255,0));
  }
  return plane_coeff;
}

void test_parelel_ransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::visualization::PCLVisualizer::Ptr viewer){

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  double threshold = 0.15;
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZRGB>);
      


  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  rgb_to_colorless(cloud, basic_cloud);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  //seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setDistanceThreshold (threshold);
  seg.setInputCloud(basic_cloud);
  seg.setAxis(Eigen::Vector3f(0,1,0));
  seg.setDistanceThreshold(0.2);
  seg.setEpsAngle(0.01);
  seg.segment(*inliers, *coefficients);
  cout << "size of found plane" << inliers->indices.size() << endl;
  cout << "test plane ransac" << endl;
  for(int i = 0; i < 4; i++){
  cout << coefficients->values[i] << endl;
  }

  for(int i = 0; i < inliers->indices.size(); i++){
      pt = basic_cloud->points[inliers->indices[i]];
      outliers->points.push_back(addcol(pt,0,255,0));
  }
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_plane_viz(outliers);
    viewer->addPointCloud<pcl::PointXYZRGB> (outliers, rgb_plane_viz, "plane_viz");



}

bool plane_direction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr c){
    int count = 0;
    pcl::PointXYZ pt;
    for(int i = 0; i < cloud->points.size(); i++){
        pt = cloud->points[i];
        if( ( pt.x * c->values[0] + pt.y * c->values[1] + pt.z * c->values[2] ) > -c->values[3]){
            count++;}
        else{count--;}
    }
    return (count > 0) ? true : false;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr corners_based_on_base(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coeff, double threshold, 
    pcl::visualization::PCLVisualizer::Ptr viewer ){

  vector<float>plane_coeff;
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointXYZ pt;

  rgb_to_colorless(cloud, basic_cloud);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (threshold);
  seg.setInputCloud(basic_cloud);
  seg.segment(*inliers, *coefficients);
  
  for(int i = 0; i < 4; i++){plane_coeff.push_back(coefficients->values[i]);
  cout << "coeff value " << i << " " << coefficients -> values[i] << endl;}
  for(int i = 0; i < inliers->indices.size();i++){
      pt = basic_cloud->points[inliers->indices[i]];
      display_cloud->points.push_back(addcol(pt,0,255,0));
  }

  Eigen::Vector4f p1(coefficients->values[0], coefficients->values[1],coefficients->values[2], coefficients->values[3]);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_viz = make_plane_cloud(coefficients);

  remove_cloud(basic_cloud, inliers, basic_cloud);
  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  seg.setAxis(Eigen::Vector3f(coefficients->values[0],coefficients->values[1],coefficients->values[2]));
  seg.setEpsAngle(0.2);
  seg.setOptimizeCoefficients (true);
  seg.setInputCloud(basic_cloud);
  seg.segment(*inliers, *coefficients2);
  cout << " paralel plane size " << inliers->indices.size() << endl;
  for(int i = 0; i < 4; i++){
      plane_coeff.push_back(coefficients2->values[i]);
      cout << "coeff value " << i << " " << coefficients2 -> values[i] << endl;
  }
  for(int i = 0; i < inliers->indices.size();i++){
      pt = basic_cloud->points[inliers->indices[i]];
      display_cloud->points.push_back(addcol(pt,255,0,0));
  }
  double sum = 0;
  for(int i = 0; i < 3; i++){
     sum += pow(coefficients->values[i], 2); 
  }
  cout << sqrt(sum) <<  " this was coef norm " << endl;


  Eigen::Vector4f p2(coefficients2->values[0], coefficients2->values[1],coefficients2->values[2], coefficients2->values[3]);
  Eigen::Vector4f gnd(coeff->values[0], coeff->values[1], coeff->values[2], coeff->values[3]);

  My_line line1 = plane_intersection(gnd, p1);
  My_line line2 = plane_intersection(gnd, p2);
  


  for(int i = 0; i < 8; i++){
        if( i % 4 == 0){cout << "plane number: " << i / 4 << endl;}
        std::cout << plane_coeff[i] << std::endl;   
  }
  remove_cloud(basic_cloud, inliers, basic_cloud);

  bool orientation = plane_direction(basic_cloud, coefficients);
  coefficients -> values[3] = coefficients->values[3] - ((orientation) ? 1.75 : -1.75); 
  orientation = plane_direction(basic_cloud, coefficients2);
  coefficients2 -> values[3] = coefficients2->values[3] - ((orientation) ? 1.75 : -1.75); 

  Eigen::Vector4f p3(coefficients->values[0], coefficients->values[1],coefficients->values[2], coefficients->values[3]);
  Eigen::Vector4f p4(coefficients2->values[0], coefficients2->values[1],coefficients2->values[2], coefficients2->values[3]);

  My_line line3 = plane_intersection(gnd, p3);
  My_line line4 = plane_intersection(gnd, p4);

  vector<Eigen::Vector3f> corners_eigen;
  corners_eigen.push_back(line1.intersection(line2));
  corners_eigen.push_back(line1.intersection(line4));
  corners_eigen.push_back(line3.intersection(line2));
  corners_eigen.push_back(line3.intersection(line4));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr corners (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB conver;
  for(int i = 0; i < 4; i++){
      conver.x = corners_eigen[i].x();
      conver.y = corners_eigen[i].y();
      conver.z = corners_eigen[i].z();
      conver.rgb = rgb2float(0,0, 255);
      corners->push_back(conver);
  }



  //TODO delete? just to know where are points
  //pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::ProjectInliers<pcl::PointXYZ> projection;
  //projection.setModelType(pcl::SACMODEL_PLANE);
  //projection.setInputCloud(basic_cloud);
  //projection.setModelCoefficients(coeff);
  //projection.filter(*projected_cloud);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> display_cloud_rgb(display_cloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> corners_rgb(corners);
  viewer->addPointCloud<pcl::PointXYZRGB> (display_cloud, display_cloud_rgb, "basic");
  viewer->addPointCloud<pcl::PointXYZRGB> (corners, corners_rgb, "corners");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "corners");


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_viz_2 = make_plane_cloud(coefficients2);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_plane_viz(plane_viz);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_plane_viz2(plane_viz_2);
    //viewer->addPointCloud<pcl::PointXYZRGB> (plane_viz, rgb_plane_viz, "plane_viz");
    //viewer->addPointCloud<pcl::PointXYZRGB> (plane_viz_2, rgb_plane_viz2, "plane_viz_2");

  return nullptr;
}
    




pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_pillar_base_corners(pcl::ModelCoefficients::Ptr plane, vector<My_line> lines){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    double t;
    Eigen::Vector3f norm(plane->values[0],plane->values[1],plane->values[2]);
    double d = plane->values[3];
    Eigen::Vector3f intersection;
    pcl::PointXYZRGB pt;

    for(int i =0; i < 4; i++){
        t = -(d+ lines[i].point.dot(norm))/(lines[i].line.dot(norm));
        intersection = lines[i].point + t*lines[i].line;
        cloud->push_back(Eigen_to_pcl(intersection, 0, 0, 255));
    }
    return cloud;
}

std::vector<My_line> pillar_planes_intesection(std::vector<float> planes){

    Eigen::Vector4f p1,p2;
    vector<My_line> lines;
    //conter clokwise order of verticies
    const int c[4] = {0,4,4,0};
    const int d[4] = {12,12,8,8};

    for(int n = 0; n < 4; n++){
    for(int i = 0; i < 4 ; i++){
        p1[i] = planes[i+c[n]];
        p2[i] = planes[i+d[n]];
    }
    My_line my_line = plane_intersection(p1,p2);
    lines.push_back(my_line);
    }
    return lines;
}

            
    
int main(int argc, char **argv){
    cout << PCL_VERSION << endl;
    string name = argv[1];
    name = name.erase(name.rfind('.'));
    //define point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pillar_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pillar_corners ;
    if(argc > 1){
        convert_to_cloud(argv[1], point_cloud_ptr, 20000);
    }
    else{
        cout << "wrong number of arguments" << endl;
        return -1;
    }
    //Approximate with vixel grid
    rgb_to_colorless(point_cloud_ptr, point_cloud);
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud(point_cloud);
    grid.setLeafSize(0.1f,0.1f,0.1f);
    grid.filter(*voxel_cloud);
    colorless_to_rgb(voxel_cloud, point_cloud_ptr);

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer);

    //find plane by RANSAC
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    coefficients = find_plane_ransac(point_cloud_ptr,0.3, pillar_cloud);

    corners_based_on_base(pillar_cloud,coefficients, 0.15, viewer);

    //test_parelel_ransac(point_cloud_ptr, viewer);
    //vector<float> planes = find_pillar_planes_ransac(pillar_cloud, 0.15);


    //vector<My_line> lines = pillar_planes_intesection(planes);
    //pillar_corners = get_pillar_base_corners(coefficients,lines);

    //Make plane to visualize RANSAC result
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_viz = make_plane_cloud(coefficients);

    //Visualization

    viewer->setBackgroundColor (0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_plane(plane_viz);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_pillar(pillar_cloud);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_pillar_corners(pillar_corners);
    //viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "cloud");
    //viewer->addPointCloud<pcl::PointXYZRGB> (plane_viz, rgb_plane, "plane");
    //viewer->addPointCloud<pcl::PointXYZRGB> (pillar_corners, rgb_pillar_corners, "pillar_corners");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pillar_corners");

    //viewer->addPointCloud<pcl::PointXYZRGB> (pillar_cloud, rgb_pillar, "pillar");

    //viewer->addPointCloud<pcl::PointXYZ> (voxel_cloud, " voxel cloud" );
    viewer->addCoordinateSystem(3);
    //cout << "After filtering" << voxel_cloud->points.size() << "     begore filtering " << point_cloud->points.size() << endl;

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
    return 0;
}

        


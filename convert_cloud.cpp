#include <Eigen/Geometry>
#include <Eigen/Core>
#include <pcl/pcl_config.h>
#include "pcl/common/io.h"
#include "pcl/impl/point_types.hpp"
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

  return corners;
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




//TODO filter cloud function 
//pcl::copyPointCloud<pcl::PointXYZ>::Ptr (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
void remove_non_pillar_pts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients){
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    

    //pcl::copyPointCloud(*cloud, *tmp_cloud);
    for(int i = 0; i < cloud->points.size(); i++){
        tmp_cloud->points.push_back(cloud->points[i]);
    }
    cout << "convert cloud OK" << endl;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(tmp_cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*projected_cloud);

    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(1.0f,1.0f,1.0f);
    grid.filter(*voxel_cloud);

    int* density = new int[voxel_cloud->points.size()]; 
    for(int i =0; i < voxel_cloud->points.size(); i++){density[i] = 0;}

    for(int i = 0; i < voxel_cloud->points.size(); i++){
       for(int j = 0; j < projected_cloud->points.size(); j++){
         //TODO set distance constant based on experiments
         if(pcl::euclideanDistance(voxel_cloud->points[i], projected_cloud->points[j]) < 3){
                density[i]++;
            }
       }
    }

    int max = 0, index;
    for(int i = 0; i < voxel_cloud->points.size(); i++){
        if(density[i] > max){
            max = density[i];
            index = i;
        }
    }
    pcl::PointXYZ pt = voxel_cloud->points[index];
    cloud->clear();
    for(int i = 0; i < tmp_cloud->points.size(); i++){
        //2.47 is max distance in projected pillar
        if(pcl::euclideanDistance(pt, projected_cloud->points[i]) < 2.6){
            cloud->points.push_back(tmp_cloud->points[i]);
        }
    }
    cout << cloud->points.size() << endl;
}
        


Eigen::Matrix3f getR(Eigen::Vector3f n, Eigen::Vector3f t)
{
    Eigen::Vector3f u = n.cross(t)/((n.cross(t).norm()));
    float alp = atan2((n.cross(t).norm()), n.dot(t));
    float s = sin(alp);
    float c = cos(alp);
    float x = u(0);
    float y = u(1);
    float z = u(2);
    float mc = 1 - c;
    Eigen::Matrix3f R;
        R <<  c + x * x * mc,      x * y * mc - z * s,   x * z * mc + y * s, 
            x * y * mc + z * s,  c + y * y * mc,       y * z * mc - x * s, 
            x * z * mc - y * s,  y * z * mc + x * s,   c + z * z * mc ;
    return R;
}

double euclideanDistance(Eigen::Vector2f a, Eigen::Vector2f b){
    return sqrt( pow(a.x() - b.x(),2) + pow(a.y() - b.y(),2));
}

double corners_error(Eigen::Matrix<float,2,4> Points, Eigen::Matrix<float,2,4> Corners){
    double error = 0, max = INFINITY;
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            if(euclideanDistance(Points.col(i), Corners.col(j)) < max){
                max = euclideanDistance(Points.col(i), Corners.col(j));
            }
        }        
        error += max;
        max = INFINITY;
    }
    return error;
}
    

class Circle {
    public:
        Eigen::Vector2f c;
        float r;
        Circle( Eigen::Vector2f v, float radius){
            r = radius;
            c = v;
        }
        pair<Eigen::Vector2f, Eigen::Vector2f> intersection(Circle circ){
            float d = (circ.c - c).norm();
            if(d > circ.r + r || d - abs(circ.r - r)){cout << "Error in circ intersection" << endl;}
            double a = (pow(r,2) - pow(circ.r,2) + pow(d,2))/ (2*d);
            double h = sqrt(pow(r,2) - pow(a,2));
            Eigen::Vector2f  Middle= c + a*(circ.c - c)/d;
            Eigen::Vector2f sol1, sol2;
            sol1.x() = Middle.x() + h*(circ.c.y() - c.y())/d;
            sol2.x() = Middle.x() - h*(circ.c.y() - c.y())/d;
            sol1.y() = Middle.y() - h*(circ.c.x() - c.x())/d;
            sol2.y() = Middle.y() + h*(circ.c.x() - c.x())/d;

            return make_pair(sol1, sol2);
        }
};


void make_dot(Mat& img, Vec3b color, Point2i c){
    Point2i t;
    for(int i = c.y -2; i < c.y + 2; i++){
        for(int j = c.x -2; j < c.x + 2; j++){
           t.x = j;
           t.y = i;
           img.at<Vec3b>(t.y,t.x) = color;
        }
    }

}
            
    
int main(int argc, char **argv){
    //return 0;
    cout << PCL_VERSION << endl;
    cout << argc << endl;
    //define point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pillar_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_voxel_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pillar_corners ;

    if(argc > 1){
        convert_to_cloud(argv[1], point_cloud_ptr, 20000);
    }
    else{
        convert_to_cloud("/home/kuntik/diam_fix7.xml", point_cloud_ptr, 20000);
        cout << " point cloud size " << point_cloud_ptr->points.size() << endl;
        //cout << "wrong number of arguments" << endl;
        //return -1;
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

    rgb_to_colorless(pillar_cloud, voxel_cloud);
    remove_non_pillar_pts(voxel_cloud, coefficients);
    cout << " after function " << voxel_cloud->points.size() << endl;
    colorless_to_rgb(voxel_cloud, pillar_cloud);

    pillar_corners = corners_based_on_base(pillar_cloud,coefficients, 0.15, viewer);

    pcl::PointCloud<pcl::PointXYZ>::Ptr zero_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    zero_cloud->points.push_back(pcl::PointXYZ(0,0,0));

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(zero_cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*zero_cloud);

    Eigen::Matrix3f Rot = getR(Eigen::Vector3f(coefficients->values[ 0 ], coefficients->values[1], coefficients->values[2]), Eigen::Vector3f(0,0,1)); 
    Eigen::Matrix<float, 3,4> Points_3d;
    for(int i = 0; i < 4; i++){
        Eigen::Vector3f pt(pillar_corners->points[i].x,pillar_corners->points[i].y,pillar_corners->points[i].z);
        Points_3d.col(i) << pt;
    }
    Points_3d = Rot * Points_3d;
    Eigen::Vector3f origin(0, 0, Points_3d(2,0));
    //TODO fix eigen ( slicing not aviable ) install eigen from github
    Eigen::Matrix<float, 2, 4> Points;
    Points.row(0) << Points_3d.row(0);
    Points.row(1) << Points_3d.row(1);

    double min = INFINITY, second_min = INFINITY, distance;
    int min_idx =-1, sMin_idx;
    for(int i = 0; i < 4; i++){
        distance = Points.col(i).norm();
        if(min > distance){
            second_min = min;
            min = distance;
            sMin_idx = min_idx;
            min_idx = i;
            }
        else if( second_min > distance){
            second_min = distance;
            sMin_idx = i;
        }
    }
    cout << Points.col(min_idx).norm() << "    " << Points.col(sMin_idx).norm() << endl;
    Eigen::Vector2f inters1,inters2;
    if( pillar_corners->points[min_idx].x < pillar_corners->points[sMin_idx].x ){
        Circle a(Eigen::Vector2f(-0.875, -0.875), min), b(Eigen::Vector2f(0.875, -0.875), second_min);
        tie(inters1, inters2 ) = a.intersection(b);
        cout << "intersection: " <<  inters1 << "   " << inters2 << endl;
    }
    else{ 
        Circle a(Eigen::Vector2f(0.875, -0.875), min), b(Eigen::Vector2f(-0.875, -0.875), second_min);
        tie(inters1, inters2 ) = a.intersection(b);
        cout << "intersection: " <<  inters1 << "   " << inters2 << endl;
    }
    cv::Vec3b darkorange(255,50,60);
    cv::Vec3b lightsalmon(255,100,40);
    cv::Vec3b brown(26,96,175);

    cv::Mat disp(cv::Size(1000, 800), CV_8UC3, Scalar(0,0,0));
    float xCenter = 500, yCenter = 400, scale = 20;
    make_dot(disp, brown, cv::Point2i((int)(xCenter - 875/scale), (int)(yCenter - 875/scale)));
    make_dot(disp, brown, cv::Point2i((int)(xCenter + 875/scale), (int)(yCenter - 875/scale)));
    make_dot(disp, brown, cv::Point2i((int)(xCenter + 875/scale), (int)(yCenter + 875/scale)));
    make_dot(disp, brown, cv::Point2i((int)(xCenter - 875/scale), (int)(yCenter + 875/scale)));
    make_dot(disp, lightsalmon, cv::Point2i((int)(xCenter + inters1.x()*1000/scale), (int)(yCenter + inters1.y()*1000/scale)));
    make_dot(disp, lightsalmon, cv::Point2i((int)(xCenter + inters2.x()*1000/scale), (int)(yCenter + inters2.y()*1000/scale)));

    cout << "test souradnic, bod1 : " << (int)(xCenter + inters1.x()*1000/scale) << " , " << (int)(yCenter + inters1.y()*1000/scale) << endl;
    cout << "test souradnic, bod1 : " << (int)(xCenter + inters2.x()*1000/scale) << " , " << (int)(yCenter + inters2.y()*1000/scale) << endl;
    
    //TODO some kind of error "window called name" appearing
    cvDestroyWindow("name");
    cv::imshow("pillar", disp);
    cv::waitKey();


            



    //cout << " rotovane body " << Points << origin << endl;

    //Eigen::Vector2f pts_mean = Points.rowwise().sum()/4;







    //test_parelel_ransac(point_cloud_ptr, viewer);
    //vector<float> planes = find_pillar_planes_ransac(pillar_cloud, 0.15);



    //vector<My_line> lines = pillar_planes_intesection(planes);
    //pillar_corners = get_pillar_base_corners(coefficients,lines);

    //Make plane to visualize RANSAC result
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_viz = make_plane_cloud(coefficients);

    //Visualization

    viewer->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_pillar(pillar_cloud);
    //viewer->addPointCloud<pcl::PointXYZRGB> (pillar_cloud, rgb_pillar, "pillar");
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_plane(plane_viz);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> projected_cloud_rgb(projected_cloud);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> projected_voxel_cloud_rgb(projected_voxel_cloud);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_pillar_corners(pillar_corners);
    //viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "cloud");
    //viewer->addPointCloud<pcl::PointXYZRGB> (plane_viz, rgb_plane, "plane");
    //viewer->addPointCloud<pcl::PointXYZRGB> (pillar_corners, rgb_pillar_corners, "pillar_corners");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pillar_corners");

    //viewer->addPointCloud<pcl::PointXYZRGB> (projected_cloud, projected_cloud_rgb, "projected_cloud_rgb");
    //viewer->addPointCloud<pcl::PointXYZRGB> (projected_voxel_cloud, projected_voxel_cloud_rgb, "projected_cloud_rgb");

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

        


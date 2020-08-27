#pragma once


class My_line{
    public:
        Eigen::Vector3f line;
        Eigen::Vector3f point;
    Eigen::Vector3f intersection(My_line l){

        Eigen::Vector3f g = l.point - point;
        double k = (l.line.cross(line)).norm();
        double h = (l.line.cross(g)).norm();
        if(k == 0 || h == 0 ) { 
            std::cout << "ERROR: Lines not intersecting " << std::endl;
            return Eigen::Vector3f(0,0,0);
        }
        int sign = ((l.line.cross(line)).dot(l.line.cross(g)) > 0) ? 1 : -1;
        return (point + sign * h/k * line);
        }
};

pcl::PointXYZRGB addcol(pcl::PointXYZ p, std::uint8_t r, std::uint8_t g, std::uint8_t b);
//convert PointXYZ to PointXYZRGB
float rgb2float(std::uint8_t r, std::uint8_t g, std::uint8_t b);
//justo make float to use for pcl::PointXYZ
void remove_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZ>::Ptr outliers);
void remove_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers);
//Remove part of the cloud (succesive approximation)
void rgb_to_colorless(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void colorless_to_rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud);
//convert cloud to color(less)
pcl::PointXYZRGB cv_to_pcl(cv::Point3i pt, std::uint32_t rgb);
pcl::PointXYZRGB Eigen_to_pcl(Eigen::Vector3f a, std::uint8_t r, std::uint8_t g, std::uint8_t bl);

void convert_to_cloud(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int limit);
//make point cloud given only name of file ( depth map )
pcl::PointCloud<pcl::PointXYZRGB>::Ptr make_plane_cloud(pcl::ModelCoefficients::Ptr coefficients);
//just for visualisation

pcl::ModelCoefficients::Ptr find_plane_ransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double threshold, 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers);
//find plane using RANSAC method
//
My_line plane_intersection(Eigen::Vector4f p1, Eigen::Vector4f p2);

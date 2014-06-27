#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "position_model.h"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>

class Registration
{
  public:

    Registration();
    ~Registration();

    void
    readDataFromOBJFiles (std::string source_points_path, std::string target_points_path);

    void
    readOBJFile (std::string file_path, std::vector < Eigen::Vector3d >& points_vector);

    void
    readDataFromOBJFileAndPCDScan(std::string source_points_path, std::string target_points_path);

    void
    getDataFromModel (std::string database_path, std::string output_path, Eigen::MatrixX3d rotation, Eigen::Vector3d translation);

    void
    calculateRigidTransformation (int number_of_iterations);

    void
    applyRigidTransformation ();

    void
    writeDataToPCD (std::string file_path);

    void
    setKdTree (pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr,pcl::PointCloud<pcl::Normal>::Ptr target_normal_cloud_ptr);


  private:


    Eigen::Matrix4d homogeneus_matrix_;

    std::vector < Eigen::Vector3d > source_points_;
    std::vector < Eigen::Vector3d > transformed_points_;

    std::vector < Eigen::Vector4d > rigid_transformed_points_;

    pcl::search::KdTree<pcl::PointNormal> kdtree_;


    pcl::PointCloud<pcl::PointNormal>::Ptr target_point_normal_cloud_ptr_;



    PositionModel* position_model_;

};

#endif // REGISTRATION_H

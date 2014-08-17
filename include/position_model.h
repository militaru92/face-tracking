#ifndef POSITIONMODEL_HPP
#define POSITIONMODEL_HPP

#include <cstdlib>

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

class PositionModel
{

  public:


    void
    readDataFromFolders (std::string, int number_samples, int number_vertices, Eigen::Matrix3d transformation_matrix, Eigen::Vector3d translation);



    Eigen::VectorXd
    calculateMeanFace (bool write = false);

    void
    calculateEigenValuesAndVectors ();





    std::vector <  pcl::Vertices >
    getMeshes (bool write = false);

    Eigen::VectorXd
    getEigenValues (bool write = false);

    Eigen::MatrixXd
    getEigenVectors (bool write = false);


  private:
    int number_points_;
    int number_faces_;

    std::vector <  pcl::Vertices > meshes_;
    std::vector < Eigen::VectorXd > faces_position_cordiantes_;

    Eigen::VectorXd mean_face_positions_;

    std::vector <double> eigenvalues_vector_;
    std::vector < Eigen::VectorXd > eigenvectors_vector_;


    Eigen::VectorXd eigenvalues_;
    Eigen::MatrixXd eigenvectors_;



};

#endif // POSITIONMODEL_HPP

#ifndef POSITIONMODEL_HPP
#define POSITIONMODEL_HPP

#include <cstdlib>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

class PositionModel
{
  private:
    int number_points_, number_faces_;
    std::vector <  pcl::Vertices > meshes_;
    std::vector < Eigen::VectorXd > faces_position_cordiantes_; /// This vector stores the S vectores as described in the paper, meaning that we store X_1,Y_1,Z_1, X_2,Y_2,Z_2, ...

    Eigen::VectorXd mean_face_positions_;
    Eigen::VectorXd face_position_model_;
    //Eigen::JacobiSVD<MatrixXd> m_SVD;

    std::vector <double> eigenvalues_vector_;
    std::vector < Eigen::VectorXd > eigenvectors_vector_;

    std::vector <double> weights_vector_;
    //std::vector <Eigen::Vector3d

  public:
    void
    readDataFromFolders (std::string, int number_samples, int number_vertices);

    void
    readDataFromFiles (std::string );

    void
    readWeights (std::string file_path);

    void
    calculateMeanFace ();

    void
    calculateEigenVectors ();

    void
    calculateModel ();

    void
    printEigenValues ();

    void
    calculateRandomWeights (int number_samples, std::string name);

    void
    viewModel (int index);

    void
    writeModel (int index,std::string path);

    //debuging methods

    void
    writeMeanFaceAndRotatedMeanFace(Eigen::MatrixX3d rotation, Eigen::Vector3d translation, std::string mean_path, std::string transformed_path, std::vector < Eigen::Vector3d>& source_points, std::vector < Eigen::Vector3d>& target_points);


};

#endif // POSITIONMODEL_HPP

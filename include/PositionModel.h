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

  public:
    void
    readDataFromFolders (std::string,int,int);

    void
    readDataFromFiles (std::string);

    void
    readWeights (std::string);

    void
    calculateMeanFace ();

    void
    calculateEigenVectors ();

    void
    calculateModel ();

    void
    printEigenValues ();

    void
    calculateRandomWeights (int, std::string);

    void
    viewModel (int);

    void
    writeModel (int,std::string);


};

#endif // POSITIONMODEL_HPP

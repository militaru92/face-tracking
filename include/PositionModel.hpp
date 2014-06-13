#ifndef POSITIONMODEL_HPP
#define POSITIONMODEL_HPP

#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "Point.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

class PositionModel
{

private:

    int m_NumberPoints, m_NumberFaces;
    std::vector < std::vector < Point > > m_vFaceVertices;
    std::vector <  pcl::Vertices > m_vMeshes;
    std::vector < Eigen::VectorXd > m_vFace_S; /// This vector stores the S vectores as described in the paper, meaning that we store X_1,Y_1,Z_1, X_2,Y_2,Z_2, ...


    Eigen::VectorXd m_MeanFace_S;
    Eigen::VectorXd m_FaceModel_S;
    //Eigen::JacobiSVD<MatrixXd> m_SVD;

    std::vector <double> m_vEigenValues_S;
    std::vector < Eigen::VectorXd > m_vEigenVectors_S;

    std::vector <double> m_vWeights;


public:
    void readDataFromFolders(std::string,int,int);
    void readDataFromFiles(std::string);
    void readWeights(std::string);
    void calculateMeanFace();
    //void calculateCovariance();
    void calculateEigenVectors();
    void calculateModel_S();

    void debug();
    void printEigenValues();

    void viewModel_S(int);


    static std::string intToString(int);
    static u_int32_t stringToUInt(std::string);
};

#endif // POSITIONMODEL_HPP

#ifndef POSITIONMODEL_HPP
#define POSITIONMODEL_HPP

#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include "Point.hpp"

class PositionModel
{

private:

    int m_NumberPoints, m_NumberFaces;
    std::vector < std::vector < Point > > m_vFaceVertices;
    std::vector < Eigen::VectorXd > m_vFace_S;
    std::vector < Eigen::VectorXd > m_vEigenVectors_S;

    Eigen::VectorXd m_MeanFace_S;
    Eigen::MatrixXd m_Covariance_S;


public:
    void readDataFromFolders(std::string,int);
    void readDataFromFiles(std::string);
    void calculateMeanFace();
    void calculateCovariance();
    void calculateEigenVectors();


    static std::string intToString(int);
};

#endif // POSITIONMODEL_HPP

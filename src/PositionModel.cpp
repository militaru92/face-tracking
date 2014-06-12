#include "PositionModel.hpp"

void PositionModel::readDataFromFolders(std::string Path, int NumberSamples)
{
    std::string Tester("Tester_"),FullPath,LastPart("/Blendshape/shape_0.obj");
    int i,j;

    this->m_NumberFaces = NumberSamples;

    for(i = 1; i <= NumberSamples; ++i)
    {
        FullPath = Path + Tester + intToString(i) + LastPart;
        std::ifstream ins(FullPath.c_str());

        this->m_NumberPoints = 11510;

        std::vector <Point> Face;
        Point Vertex;


        Eigen::VectorXd S(this->m_NumberPoints * 3);


        if(ins.fail())
        {
            std::cout<<"Could not open file\n" << FullPath <<std::endl;
            exit(1);
        }

        j = 0;

        while(1)
        {
            std::string line;
            std::getline(ins,line);

            std::stringstream ss(line);
            std::string s;
            double d;

            ss >> s;

            if(s.compare("v") != 0)
                break;

            ss >> d;
            S[j++] = d;
            Vertex.m_Positons[0] = d;

            ss >> d;
            S[j++] = d;
            Vertex.m_Positons[1] = d;

            ss >> d;
            S[j++] = d;
            Vertex.m_Positons[2] = d;

            Face.push_back(Vertex);


        }

        this->m_vFace_S.push_back(S);
        this->m_vFaceVertices.push_back(Face);

    }



}

void PositionModel::calculateMeanFace()
{
    int i;

    this->m_MeanFace_S = Eigen::VectorXd::Zero(3 * this->m_NumberPoints);


    for(i = 0; i < this->m_NumberFaces; ++i)
    {

        this->m_MeanFace_S += this->m_vFace_S[i];
    }

    this->m_MeanFace_S /= (0.0 + this->m_NumberFaces);


}

void PositionModel::calculateCovariance()
{



    int i,j,k;
    double d;
    this->m_Covariance_S = Eigen::MatrixXd::Zero(this->m_NumberPoints * 3, this->m_NumberPoints * 3);

    std::ifstream ins("covariance.txt");

    if(!ins.fail())
    {
        for(i = 0; i < this->m_Covariance_S.rows(); ++i)
            for(j = 0; j < this->m_Covariance_S.cols(); ++j)
            {
                ins >> d;
                this->m_Covariance_S(i,j) = d;

            }

        return;

    }

    Eigen::MatrixXd v;


    for(i = 0; i < this->m_NumberFaces; ++i)
    {

        v = this->m_vFace_S[i] - this->m_MeanFace_S;

        for(j = 0; j < this->m_Covariance_S.rows(); ++j)
        {
            for(k = 0; k < this->m_Covariance_S.cols(); ++k)
            {
                this->m_Covariance_S(j,k) = this->m_Covariance_S(j,k) + v(j) * v(k);
                std::cout << i << " " << j << "--" << k << " " << this->m_NumberFaces << " " << this->m_Covariance_S.rows() <<" "<< this->m_Covariance_S.cols() <<std::endl;
            }


        }


    }

    this->m_Covariance_S /= (0.0 + this->m_NumberFaces);

    std::ofstream ofs("covariance.txt");

    for(i = 0; i < this->m_Covariance_S.rows(); ++i)
    {
        for(j = 0; j < this->m_Covariance_S.cols(); ++j)
        {
            ofs << m_Covariance_S(i,j) << " ";
        }
        ofs << std::endl;
    }





}

void PositionModel::calculateEigenVectors()
{

    std::ifstream ifs("eigenvectors.txt");
    int i,j,k;
    double d;

    if(ifs.fail())
    {
        this->m_Solver.compute(this->m_Covariance_S);

        if(this->m_Solver.eigenvalues().rows() <= 1)
        {
            std::cout<<"Solver row is 1" << std::endl;
            exit(1);
        }

        this->m_vEigenValues_S = this->m_Solver.eigenvalues().real();

        for(i = 0; i < this->m_Solver.eigenvectors().cols(); ++i)
            this->m_vEigenVectors_S.push_back(m_Solver.eigenvectors().col(i).real());

        std::ofstream ofs("eigenvectors.txt");

        for(i = 0; i < this->m_vEigenVectors_S.size(); ++i)
        {
            for(j = 0; j < this->m_vEigenVectors_S[i].rows(); ++j)
            {
                ofs << this->m_vEigenVectors_S[i](j) << " ";
            }

            ofs << std::endl;
        }

        ofs.close();

        ofs.open("eigenvalues.txt");

        for(k = 0; k < this->m_vEigenValues_S.rows(); ++k)
            ofs << this->m_vEigenValues_S(k) << std::endl;

        ofs.close();

        return;
    }

    Eigen::VectorXd v(this->m_Covariance_S.cols());

    for(i = 0; i < this->m_Covariance_S.rows(); ++i)
    {
        for(j = 0; j < this->m_Covariance_S.cols(); ++j)
        {
            ifs >> d;
            v(j) = d;
        }

        this->m_vEigenVectors_S.push_back(v);
    }

    ifs.close();

    ifs.open("eigenvalues.txt");

    for(k = 0; k < this->m_Covariance_S.cols(); ++k)
    {
        ifs >> d;
        v(k) = d;
    }

    this->m_vEigenValues_S = v;

    ifs.close();

}

void PositionModel::readWeights(std::string FilePath)
{
    std::ifstream ifs(FilePath.c_str());
    double d;

    if(ifs.fail())
    {
        std::cout << "No file for weights" << std::endl;
        exit(1);
    }

    while(!ifs.eof())
    {
        ifs >> d;
        this->m_vWeights.push_back(d);
    }



}

void PositionModel::calculateModel_S()
{
    int i;
    this->m_FaceModel_S = this->m_MeanFace_S;

    for(i = 0; i < this->m_vWeights.size(); ++i)
        this->m_FaceModel_S += this->m_vWeights[i] * this->m_vEigenVectors_S[i];
}

std::string PositionModel::intToString(int Number)
{
    std::stringstream ss;
    std::string s;
    ss<<Number;
    ss>>s;
    return s;
}

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
    std::cout<<"in func\n";

    this->m_MeanFace_S = Eigen::VectorXd::Zero(3 * this->m_NumberPoints);
    std::cout<<"iafd\n";


    for(i = 0; i < this->m_NumberFaces; ++i)
    {

        this->m_MeanFace_S += this->m_vFace_S[i];
    }

    this->m_MeanFace_S /= (0.0 + this->m_NumberFaces);


}

void PositionModel::calculateCovariance()
{
    this->m_Covariance_S = Eigen::MatrixXd::Zero(this->m_NumberPoints * 3, this->m_NumberPoints * 3);

    Eigen::MatrixXd v;

    for(int i = 0; i < this->m_NumberFaces; ++i)
    {

        std::cout<<"Before v\n";
        v = this->m_vFace_S[i] - this->m_MeanFace_S;
        std::cout<<"After v\n";
        this->m_Covariance_S += v * v.transpose(); //Bad alloc

    }

    this->m_Covariance_S /= (0.0 + this->m_NumberFaces);


}

std::string PositionModel::intToString(int Number)
{
    std::stringstream ss;
    std::string s;
    ss<<Number;
    ss>>s;
    return s;
}

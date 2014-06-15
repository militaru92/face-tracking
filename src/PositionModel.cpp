#include "PositionModel.hpp"

/**
 * @brief PositionModel::readDataFromFolders
 * @param Path
 * @param NumberSamples
 *
 * This method will read the vertex positions of neutral faces when the files are stored in the original folders
 */

void PositionModel::readDataFromFolders(std::string Path, int NumberSamples, int NumberVertices)
{
    std::string Tester("Tester_"),FullPath,LastPart("/Blendshape/shape_0.obj"),line,value;
    size_t index;
    int i,j,k;

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

        if(i == 1)
        {
            while(line[0] != 'f')
            {
                std::getline(ins,line);
            }

            while(line[0] == 'f' && !ins.eof())
            {
                pcl::Vertices mesh;
                for(k = 0; k < NumberVertices; ++k)
                {
                    index = line.find(' ');
                    line = line.substr(index+1);
                    index = line.find('/');
                    value = line.substr(0,index);
                    mesh.vertices.push_back(this->stringToUInt(value));



                }
                this->m_vMeshes.push_back(mesh);

                std::getline(ins,line);
            }
        }

        this->m_vFace_S.push_back(S);
        this->m_vFaceVertices.push_back(Face);

    }



}
/**
 * @brief PositionModel::calculateMeanFace
 * This method will calculate the average S
 */

void PositionModel::calculateMeanFace()
{
    int i;

    this->m_MeanFace_S = Eigen::VectorXd::Zero(3 * this->m_NumberPoints);


    for(i = 0; i < this->m_NumberFaces; ++i)
    {

        this->m_MeanFace_S += this->m_vFace_S[i];
    }

    this->m_MeanFace_S /= static_cast <double> (this->m_NumberFaces);

    std::cout << "Done with average face\n";


}


void PositionModel::calculateEigenVectors()
{
    Eigen::MatrixXd T(3 * this->m_NumberPoints, this->m_NumberFaces);
    int i,j;
    Eigen::VectorXd v;

    for(i = 0; i < T.cols(); ++i)
    {
        T.col(i) = this->m_vFace_S[i] - this->m_MeanFace_S;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> SVD(T.transpose() * T, Eigen::ComputeFullU | Eigen::ComputeFullV);

    for(i = 0; i < SVD.singularValues().rows(); ++i)
    {
        this->m_vEigenValues_S.push_back(SVD.singularValues()(i));
    }

    for(i = 0; i < SVD.matrixU().cols(); ++i)
    {
        v = T * SVD.matrixU().col(i);
        v.normalize();
        this->m_vEigenVectors_S.push_back(v);

        //std::cout<< i <<" "<< v.rows()<<std::endl;
    }



    //std::cout<<SVD.singularValues().cols() <<" " <<SVD.singularValues().rows()<<std::endl;
    std::cout << "Done with Eigenvectors\n";



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

void PositionModel::debug()
{
    std::ofstream ofs1("meshes.txt");
    int i,j;

    for(i = 0; i < this->m_vMeshes.size(); ++i)
    {
        for(j = 0; j < this->m_vMeshes[i].vertices.size(); ++j)
        {
            ofs1 << this->m_vMeshes[i].vertices[j] << ' ';
        }

        ofs1 << std::endl;
    }
}

void PositionModel::printEigenValues()
{

    std::ofstream ofs("eigenvalues.txt");
    int i;

    for(i = 0; i < this->m_vEigenValues_S.size(); ++i)
    {
        ofs << this->m_vEigenValues_S[i] <<std::endl;
    }
}

void PositionModel::calculateRandomWeights(int NumberSamples, std::string Name)
{
    int i;
    double w,d;
    std::ofstream ofs_1((std::string("random_") + Name + std::string(".txt")).c_str()), ofs_2((std::string("weights_") + Name + std::string(".txt")).c_str());

    srand (time(NULL));

    for(i = 0; i < NumberSamples; ++i)
    {

        d = (double)rand() / RAND_MAX;
        w = d * 6.0 - 3.0;

        ofs_1 << w <<std::endl;

        this->m_vWeights.push_back(w * this->m_vEigenValues_S[i]);

        ofs_2 << this->m_vWeights[i] << std::endl;
    }


}

void PositionModel::viewModel_S(int index)
{
    int i;
    Eigen::VectorXd positions;

    if(index == -1)
        positions = this->m_FaceModel_S;
    else
        positions = this->m_vFace_S[index];

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    pcl::PointXYZ basic_point;
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    for(i = 0; i < 3 * this->m_NumberPoints;)
    {
        basic_point.x = static_cast<float>(positions(i++));
        basic_point.y = static_cast<float>(positions(i++));
        basic_point.z = static_cast<float>(positions(i++));

        basic_cloud_ptr->points.push_back(basic_point);
    }

    std::string polygon("polygon");

    viewer->addPolygonMesh <pcl::PointXYZ> (basic_cloud_ptr,this->m_vMeshes,polygon,0);



    viewer->addCoordinateSystem (1.0, 0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}

void PositionModel::writeModel_S(int index,std::string Path)
{
    Eigen::VectorXd *positions;

    if(index == -1)
        positions = &(this->m_FaceModel_S);
    else
        positions = &(this->m_vFace_S[index]);

    std::ofstream ofs((Path + ".obj").c_str());

    int i,j;

    for(i = 0; i < positions->rows(); ++i)
    {
        if(i % 3 == 0)
            ofs << "v ";

        ofs << (*positions)(i);

        if(i % 3 == 2)
            ofs << std::endl;
        else
            ofs << ' ';

    }


    for(i = 0; i < this->m_vMeshes.size(); ++i)
    {
        ofs << 'f';

        for(j = 0; j < this->m_vMeshes[i].vertices.size(); ++j)
        {
            ofs<< ' ' << this->m_vMeshes[i].vertices[j];
        }

        ofs << std::endl;
    }

    ofs.close();
}

std::string PositionModel::intToString(int Number)
{
    std::stringstream ss;
    std::string s;
    ss<<Number;
    ss>>s;
    return s;
}

u_int32_t PositionModel::stringToUInt(std::string String)
{
    std::stringstream ss(String);
    u_int32_t n;
    ss>>n;
    return n;
}

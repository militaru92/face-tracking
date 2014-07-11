#include <registration.h>
#include <pcl/console/parse.h>

int main(int argc, char** argv)
{

  if(argc < 3)
  {
    PCL_ERROR("You need to pass the path of the data base and the path where you want to store the model\n");
    exit(1);
  }

  std::string database_path,obj_path,source_path,target_path;

  database_path = argv[1];
  obj_path = argv[2];
/*
  PositionModel model;


  //model.readDataFromFolders("/media/ace/New Volume/Google Summer/Part1/",150,4);

  model.readDataFromFolders(database_path,150,4);
  model.calculateMeanFace();



  model.calculateEigenVectors();
  model.printEigenValues();
  model.calculateRandomWeights(50,obj_path);
  model.calculateModel();
  model.writeModel(obj_path);
  */

  double pi =  4 * atan(1);

  double distance_limit = 0.001, angle_limit = pi * 0.25;

  Registration registrator;

  if(pcl::console::find_argument (argc, argv, "-o") >= 0)
  {
    source_path = argv[3];
    target_path = argv[4];
    registrator.readDataFromOBJFiles(source_path,target_path);
  }

  if(pcl::console::find_argument (argc, argv, "-m") >= 0)
  {

    target_path = argv[4];

    Eigen::Matrix3d transform_matrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    transform_matrix(0,0) = 0.125;
    transform_matrix(1,1) = 0.125;
    transform_matrix(2,2) = 0.125;

    transform_matrix = Eigen::AngleAxisd(pi,Eigen::Vector3d::UnitX()) * transform_matrix;


    translation[0] = 1.5;
    translation[1] = 1.5;
    translation[2] = 0.125;

    if(argc > 6)
    {
      angle_limit = boost::lexical_cast<double>(argv[5]) * pi;
      distance_limit = boost::lexical_cast<double>(argv[6]);
    }

    registrator.getDataFromModel(database_path, target_path, transform_matrix, translation);
  }

  if(pcl::console::find_argument (argc, argv, "-p") >= 0)
  {
    source_path = argv[3];
    target_path = argv[4];

    Eigen::Matrix3d transform_matrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    transform_matrix(0,0) = 0.15;
    transform_matrix(1,1) = 0.15;
    transform_matrix(2,2) = 0.15;

    transform_matrix = Eigen::AngleAxisd(pi,Eigen::Vector3d::UnitX()) * transform_matrix;




    translation[0] = 1.5;
    translation[1] = 1.5;
    translation[2] = 0.125;

    if(argc > 6)
    {
      angle_limit = boost::lexical_cast<double>(argv[5]) * pi;
      distance_limit = boost::lexical_cast<double>(argv[6]);
    }




    registrator.readDataFromOBJFileAndPCDScan(source_path,target_path,transform_matrix,translation);

  }

  registrator.calculateAlternativeTransformations(50,0.001,4,5,angle_limit,distance_limit);

  registrator.writeDataToPCD(obj_path);



  //model.writeMeanFaceAndRotatedMeanFace(rotation,translation,"Average.obj","Transformed.obj");

  return (0);
}

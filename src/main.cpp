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

  Registration registrator;

  if(pcl::console::find_argument (argc, argv, "-o") >= 0)
  {
    source_path = argv[3];
    target_path = argv[4];
    registrator.readDataFromOBJFiles(source_path,target_path);
  }

  if(pcl::console::find_argument (argc, argv, "-m") >= 0)
  {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero(),axis;
    Eigen::Matrix3d rotation,cross;

    axis << 1, 1, 1;
    translation << 0, 0, 0;

    cross << 0, -1, 1,
             1, 0, -1,
            -1, 0, 1;

    rotation = cos(pi/3.0) * Eigen::Matrix3d::Identity() + sin(pi/3.0) * cross + (1 - cos(pi/3.0)) * axis * axis.transpose();

    std::cout << "Rotation\n" << rotation << std::endl << std::endl;

    registrator.getDataFromModel(database_path, obj_path, rotation, translation);
  }

  if(pcl::console::find_argument (argc, argv, "-p") >= 0)
  {
    source_path = argv[3];
    target_path = argv[4];

    Eigen::Matrix3d transform_matrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    transform_matrix(0,0) = 0.1;
    transform_matrix(1,1) = 0.1;
    transform_matrix(2,2) = 0.1;

    transform_matrix = Eigen::AngleAxisd(pi,Eigen::Vector3d::UnitX()) * transform_matrix;




    translation[0] = 1.5;
    translation[1] = 1.5;
    translation[2] = 0.125;




    registrator.readDataFromOBJFileAndPCDScan(source_path,target_path,transform_matrix,translation);

  }

  registrator.calculateRigidTransformation(10);

  registrator.applyRigidTransformation();

  registrator.writeDataToPCD(obj_path);



  //model.writeMeanFaceAndRotatedMeanFace(rotation,translation,"Average.obj","Transformed.obj");

  return (0);
}

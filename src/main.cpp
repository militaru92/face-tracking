#include <registration.h>
#include <camera_grabber.h>
#include <pcl/console/parse.h>

int main(int argc, char** argv)
{

/*
  if(pcl::console::find_argument (argc, argv, "-g") >= 0)
  {
    std::string xml_file(argv[2]);
    CameraGrabber grabber;
    grabber.runCamera(CV_CAP_OPENNI_ASUS,xml_file);
    return (0);
  }
*/
  std::string database_path,obj_path,source_path,target_path;

  database_path = argv[1];
  obj_path = argv[2];

  double pi =  4 * atan(1);

  double distance_limit = 0.001, angle_limit = pi * 0.25;

  Registration registrator;

  if(pcl::console::find_argument (argc, argv, "-o") >= 0)
  {
    source_path = argv[3];
    target_path = argv[4];
    registrator.readDataFromOBJFiles(source_path,target_path);
  }

  if(pcl::console::find_argument (argc, argv, "-c") >= 0)
  {


    Eigen::Matrix3d transform_matrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    transform_matrix(0,0) = 0.125;
    transform_matrix(1,1) = 0.125;
    transform_matrix(2,2) = 0.125;

    transform_matrix = Eigen::AngleAxisd(pi,Eigen::Vector3d::UnitX()) * transform_matrix;




    if(argc > 5)
    {
      angle_limit = boost::lexical_cast<double>(argv[4]) * pi;
      distance_limit = boost::lexical_cast<double>(argv[5]);
    }

    std::string xml_file(argv[3]);

    registrator.getDataFromModel(database_path, transform_matrix, translation);
    registrator.getTargetPointCloudFromCamera(CV_CAP_OPENNI_ASUS,xml_file);
  }


  if(pcl::console::find_argument (argc, argv, "-m") >= 0)
  {

    target_path = argv[4];

    Eigen::Matrix3d transform_matrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    transform_matrix(0,0) = 0.09;
    transform_matrix(1,1) = 0.09;
    transform_matrix(2,2) = 0.09;

    transform_matrix = Eigen::AngleAxisd(pi,Eigen::Vector3d::UnitX()) * transform_matrix;


    //translation sub1

    /*
    translation[0] = 1.5;
    translation[1] = 1.5;
    translation[2] = 0.125;
    */

    //translation sub2


    translation[0] = 1.5;
    translation[1] = 1.70;
    translation[2] = 0.35;





    if(argc > 6)
    {
      angle_limit = boost::lexical_cast<double>(argv[5]) * pi;
      distance_limit = boost::lexical_cast<double>(argv[6]);
    }

    registrator.getDataFromModel(database_path, transform_matrix, translation);
    registrator.readTargetPointCloud(target_path);
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

  registrator.calculateAlternativeTransformations(50,0.001,10,15,angle_limit,distance_limit,true);

  registrator.writeDataToPCD(obj_path);



  //model.writeMeanFaceAndRotatedMeanFace(rotation,translation,"Average.obj","Transformed.obj");

  return (0);
}

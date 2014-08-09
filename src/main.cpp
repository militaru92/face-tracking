#include <registration.h>
#include <camera_grabber.h>
#include <pcl/console/parse.h>

int main(int argc, char** argv)
{

  std::string database_path,result_path;

  database_path = "/media/ace/New Volume/Google Summer/Part1/";
  result_path = "result";

  double pi =  4 * atan(1);

  double distance_limit = 0.001, angle_limit = pi * 0.25;

  pcl::console::parse_argument (argc, argv, "-database", database_path);
  pcl::console::parse_argument (argc, argv, "-result", result_path);
  pcl::console::parse_argument (argc, argv, "-distance", distance_limit);
  pcl::console::parse_argument (argc, argv, "-angle", angle_limit);

  Registration registrator;



  if(pcl::console::find_switch (argc, argv, "--camera"))
  {


    Eigen::Matrix3d transform_matrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    transform_matrix(0,0) = 0.12;
    transform_matrix(1,1) = 0.12;
    transform_matrix(2,2) = 0.12;

    transform_matrix = Eigen::AngleAxisd(pi,Eigen::Vector3d::UnitX()) * transform_matrix;

    std::string xml_file("haarcascade_frontalface_alt.xml");

    pcl::console::parse_argument (argc, argv, "-xml_file", xml_file);



    registrator.getTargetPointCloudFromCamera(CV_CAP_OPENNI_ASUS,xml_file);
    registrator.getDataFromModel(database_path, transform_matrix, translation);
    registrator.alignModel();

  }

  if(pcl::console::find_switch (argc, argv, "--scan"))
  {


    Eigen::Matrix3d transform_matrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    transform_matrix(0,0) = 0.12;
    transform_matrix(1,1) = 0.12;
    transform_matrix(2,2) = 0.12;

    transform_matrix = Eigen::AngleAxisd(pi,Eigen::Vector3d::UnitX()) * transform_matrix;



    std::string pcd_file("target.pcd");

    pcl::console::parse_argument (argc, argv, "-target", pcd_file);

    registrator.getTargetPointCloudFromFile(pcd_file);
    registrator.getDataFromModel(database_path, transform_matrix, translation);
    registrator.alignModel();

  }

  registrator.calculateAlternativeTransformations(50,0.001,10,15,angle_limit,distance_limit,false);

  registrator.writeDataToPCD(result_path);



  return (0);
}

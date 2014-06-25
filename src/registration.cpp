#include <registration.h>

Registration::Registration()
{
  homogeneus_matrix_ = Eigen::Matrix4d::Identity();
  position_model_ = NULL;
}

Registration::~Registration()
{
  if( position_model_ )
  {
    PCL_INFO ("Model deleted\n");
    delete position_model_;
  }

}

void
Registration::readDataFromOBJFiles(std::string source_points_path, std::string target_points_path)
{
  // to be later implemented if needed

  pcl::PointXYZ pcl_point;


  int i;

  readOBJFile(source_points_path,source_points_);

  readOBJFile(target_points_path, target_points_);


  target_point_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  target_point_cloud_ptr_->width = target_points_.size();
  target_point_cloud_ptr_->height = 1;

  for( i = 0; i < target_points_.size(); ++i)
  {
    pcl_point.x = target_points_[i][0];
    pcl_point.y = target_points_[i][1];
    pcl_point.z = target_points_[i][2];

    target_point_cloud_ptr_->points.push_back(pcl_point);
  }

  kdtree_.setInputCloud(target_point_cloud_ptr_);


}

void
Registration::readOBJFile(std::string file_path, std::vector < Eigen::Vector3d >& points_vector)
{

  std::ifstream instream(file_path.c_str());
  std::string line;

  Eigen::Vector3d point;

  if(instream.fail())
  {
    PCL_ERROR("Could not open file %s\n", file_path.c_str());
    exit(1);
  }



  while(1)
  {
    std::getline(instream,line);

    std::stringstream ss(line);
    std::string s;

    ss >> s;
    if(s.compare("v") != 0)
      break;

    ss >> point[0];
    ss >> point[1];
    ss >> point[2];

    points_vector.push_back(point);


  }

  instream.close();
}

void
Registration::getDataFromModel(std::string database_path, std::string output_path, Eigen::MatrixX3d rotation, Eigen::Vector3d translation)
{
  int i;

  pcl::PointXYZ point;

  if(!position_model_)
  {
    PCL_INFO ("Created Model\n");
    position_model_ = new PositionModel;
  }

  position_model_->readDataFromFolders(database_path,150,4);
  position_model_->calculateMeanFace();

  position_model_->calculateEigenVectors();
  position_model_->printEigenValues();
  position_model_->calculateRandomWeights(50,output_path);
  position_model_->calculateModel();
  position_model_->writeModel(output_path);

  position_model_->writeMeanFaceAndRotatedMeanFace(rotation, translation, output_path + "_source.obj", output_path +"_transformed.obj",source_points_,target_points_);


  target_point_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  target_point_cloud_ptr_->width = target_points_.size();
  target_point_cloud_ptr_->height = 1;

  for( i = 0; i < target_points_.size(); ++i)
  {
    point.x = target_points_[i][0];
    point.y = target_points_[i][1];
    point.z = target_points_[i][2];

    target_point_cloud_ptr_->points.push_back(point);
  }

  kdtree_.setInputCloud(target_point_cloud_ptr_);


}

void
Registration::calculateRigidTransformation(int number_of_iterations)
{

  PCL_INFO("In calculate method\n");
  PCL_INFO("Size of sources_ and targets_ %d %d \n",source_points_.size(),target_points_.size());
  int i,j,k;

  pcl::PointXYZ search_point;

  Eigen::MatrixXd J(source_points_.size(),4), JJ,J_transpose;
  Eigen::VectorXd y[3];
  Eigen::Vector4d solutions[3];

  Eigen::Matrix3d current_iteration_rotation;
  Eigen::Vector3d current_iteration_translation;

  std::vector < Eigen::Vector3d > current_iteration_source_points = source_points_;
  std::vector < Eigen::Vector3d > current_iteration_target_points;

  std::vector < int > point_index(1);
  std::vector < float > point_distance(1);

  for(i = 0; i < 3; ++i)
  {
    y[i].resize(target_points_.size());

  }

  for(i = 0; i < current_iteration_source_points.size(); ++i)
  {
    J(i,3) = 1.0;
  }



  for(k = 0; k < number_of_iterations; ++k)
  {

    PCL_INFO("Iteration %d\n",k);
    for(i = 0; i < current_iteration_source_points.size(); ++i)
    {
      J(i,0) = current_iteration_source_points[i][0];
      J(i,1) = current_iteration_source_points[i][1];
      J(i,2) = current_iteration_source_points[i][2];


      search_point.x = current_iteration_source_points[i][0];
      search_point.y = current_iteration_source_points[i][1];
      search_point.z = current_iteration_source_points[i][2];

      kdtree_.nearestKSearch(search_point,1,point_index,point_distance);
      y[0][i] = target_point_cloud_ptr_->points[point_index[0]].x;
      y[1][i] = target_point_cloud_ptr_->points[point_index[0]].y;
      y[2][i] = target_point_cloud_ptr_->points[point_index[0]].z;

    }

    J_transpose = J.transpose();
    JJ = J_transpose * J;



    for(i = 0; i < 3; ++i)
    {
      solutions[i] = JJ.colPivHouseholderQr().solve(J_transpose * y[i]);
    }

    PCL_INFO("Done with linear Solvers\n");

    for(i = 0; i < 3; ++i)
    {
      current_iteration_translation(i) = solutions[i][3];
    }

    for(i = 0; i < 3; ++i)
    {
      for(j = 0; j < 3; ++j)
      {
        current_iteration_rotation(i,j) = solutions[i][j];
      }
    }

    for( i = 0; i < current_iteration_source_points.size(); ++i)
    {
      current_iteration_source_points[i] = current_iteration_rotation * current_iteration_source_points[i] + current_iteration_translation;
    }

    Eigen::Matrix4d current_homogeneus_matrix;
    //std::cout << rotation_ <<std::endl<<std::endl;

    //std::cout << translation_ << std::endl<<std::endl;

    current_homogeneus_matrix.block(0, 0, 3, 3) = current_iteration_rotation;
    current_homogeneus_matrix.block(0, 3, 3, 1) = current_iteration_translation;
    current_homogeneus_matrix.row(3) << 0, 0, 0, 1;

    //std::cout << homogeneus_matrix_<<std::endl<<std::endl;

    homogeneus_matrix_ = current_homogeneus_matrix * homogeneus_matrix_;

    //std::cout << homogeneus_matrix_<<std::endl<<std::endl;
  }



}

void
Registration::applyRigidTransformation()
{
  int i;
  Eigen::Vector4d transformed_point,original_point;

  transformed_point(3) = 1.0;
  original_point(3) = 1.0;

  for( i = 0; i < source_points_.size(); ++i)
  {
    original_point.block(0, 0, 3, 1) = source_points_[i];

    transformed_point = homogeneus_matrix_ * original_point;

    rigid_transformed_points_.push_back(transformed_point);
  }

  std::cout<< "Apply\n" << original_point <<std::endl << std::endl << transformed_point<<std::endl<<std::endl;
}

void
Registration::writeDataToPCD(std::string file_path)
{
  pcl::PointCloud < pcl::PointXYZRGB > cloud;
  pcl::PointXYZRGB point;
  int i;
  uint32_t rgb;
  uint8_t value(255);

  cloud.width = 2 * rigid_transformed_points_.size();
  cloud.height = 1;

  for( i = 0; i < rigid_transformed_points_.size(); ++i)
  {
    /*
    point.x = source_points_[i][0];
    point.y = source_points_[i][1];
    point.z = source_points_[i][2];

    rgb = ((uint32_t)value) << 16;
    point.rgb = *reinterpret_cast<float*>(&rgb);

    cloud.points.push_back(point);
    */


    point.x = target_points_[i][0];
    point.y = target_points_[i][1];
    point.z = target_points_[i][2];

    rgb = ((uint32_t)value) << 8;
    point.rgb = *reinterpret_cast<float*>(&rgb);

    cloud.points.push_back(point);



    point.x = rigid_transformed_points_[i][0];
    point.y = rigid_transformed_points_[i][1];
    point.z = rigid_transformed_points_[i][2];

    rgb = ((uint32_t)value);
    point.rgb = *reinterpret_cast<float*>(&rgb);

    cloud.points.push_back(point);




  }

  pcl::io::savePCDFileASCII (file_path + ".pcd", cloud);


}

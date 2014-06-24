#include <registration.h>

Registration::Registration()
{
  homogeneus_matrix_ = Eigen::Matrix4d::Identity();
  model_ = new PositionModel;
}

Registration::~Registration()
{
  delete model_;
}

void
Registration::readData(std::string source_points_path, std::string target_points_path)
{
  // to be later implemented if needed



}

void
Registration::getDataFromModel(std::string database_path,Eigen::MatrixX3d rotation, Eigen::Vector3d translation)
{
  model_->readDataFromFolders(database_path,150,4);
  model_->calculateMeanFace();
  model_->writeMeanFaceAndRotatedMeanFace(rotation,translation,"Average.obj","Transformed.obj",source_points_,target_points_);
}

void
Registration::calculateRigidTransformation(int number_of_iterations)
{

  PCL_INFO("In calculate method\n");
  PCL_INFO("Size of sources_ and targets_ %d %d \n",source_points_.size(),target_points_.size());
  int i,j,k;
  Eigen::MatrixXd J(source_points_.size(),4), JJ,J_transpose;
  Eigen::VectorXd y[3];
  Eigen::Vector4d solutions[3];

  std::vector <Eigen::VectorXd> dimension_targets;

  for(i = 0; i < source_points_.size(); ++i)
  {
    J(i,0) = source_points_[i][0];
    J(i,1) = source_points_[i][1];
    J(i,2) = source_points_[i][2];
    J(i,3) = 1.0;
  }

  J_transpose = J.transpose();
  JJ = J_transpose * J;

  for(i = 0; i < 3; ++i)
  {
    y[i].resize(target_points_.size());
    for(j = 0; j < target_points_.size(); ++j)
    {
      y[i][j] = target_points_[j][i];
    }
  }

  for(i = 0; i < 3; ++i)
  {
    solutions[i] = JJ.colPivHouseholderQr().solve(J_transpose * y[i]);
  }

  PCL_INFO("Done with linear Solvers\n");

  for(i = 0; i < 3; ++i)
  {
    translation_(i) = solutions[i][3];
  }

  for(i = 0; i < 3; ++i)
  {
    for(j = 0; j < 3; ++j)
    {
      rotation_(i,j) = solutions[i][j];
    }
  }

  Eigen::Matrix4d current_homogeneus_matrix;

  current_homogeneus_matrix.block(0, 0, 3, 3) = rotation_;
  current_homogeneus_matrix.block(0, 3, 3, 1) = translation_;

  homogeneus_matrix_ = current_homogeneus_matrix * homogeneus_matrix_;



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
    point.x = source_points_[i][0];
    point.y = source_points_[i][1];
    point.z = source_points_[i][2];

    rgb = ((uint32_t)value) << 16;
    point.rgb = *reinterpret_cast<float*>(&rgb);

    cloud.points.push_back(point);

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

  pcl::io::savePCDFileASCII (file_path, cloud);


}

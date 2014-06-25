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

  pcl::PointXYZ pcl_point;


  int i;

  std::vector < Eigen::Vector3d > target_points;

  readOBJFile(source_points_path,source_points_);

  readOBJFile(target_points_path, target_points);


  target_point_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  target_point_cloud_ptr_->width = target_points.size();
  target_point_cloud_ptr_->height = 1;

  for( i = 0; i < target_points.size(); ++i)
  {
    pcl_point.x = target_points[i][0];
    pcl_point.y = target_points[i][1];
    pcl_point.z = target_points[i][2];

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
Registration::readDataFromOBJFileAndPCDScan(std::string source_points_path, std::string target_points_path)
{

  int i;
  Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation;

  transform(0,0) = 0.1;
  transform(1,1) = 0.1;
  transform(2,2) = 0.1;

  translation[0] = 1.5;
  translation[1] = 1.0;
  translation[2] = 0.0;


  readOBJFile(source_points_path,source_points_);

  for( i = 0; i < source_points_.size(); ++i)
      source_points_[i] = transform * source_points_[i] + translation;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  target_point_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_points_path, *scan_cloud) == -1) //* load the file
  {
    PCL_ERROR("Could not open file %s\n", target_points_path.c_str());
    exit(1);
  }

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outliers_filter;
  outliers_filter.setInputCloud(scan_cloud);
  outliers_filter.setMeanK(50);
  outliers_filter.setStddevMulThresh(10);
  outliers_filter.filter(*target_point_cloud_ptr_);

  pcl::io::savePCDFileASCII("filtered_cloud.pcd", *target_point_cloud_ptr_);

  kdtree_.setInputCloud(target_point_cloud_ptr_);


}

void
Registration::getDataFromModel(std::string database_path, std::string output_path, Eigen::MatrixX3d rotation, Eigen::Vector3d translation)
{
  int i;

  pcl::PointXYZ point;

  std::vector < Eigen::Vector3d > target_points;

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

  position_model_->writeMeanFaceAndRotatedMeanFace(rotation, translation, output_path + "_source.obj", output_path +"_transformed.obj",source_points_,target_points);


  target_point_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  target_point_cloud_ptr_->width = target_points.size();
  target_point_cloud_ptr_->height = 1;

  for( i = 0; i < target_points.size(); ++i)
  {
    point.x = target_points[i][0];
    point.y = target_points[i][1];
    point.z = target_points[i][2];

    target_point_cloud_ptr_->points.push_back(point);
  }

  kdtree_.setInputCloud(target_point_cloud_ptr_);


}

void
Registration::calculateRigidTransformation(int number_of_iterations)
{

  PCL_INFO("In calculate method\n");
  PCL_INFO("Size of sources_ and targets_ %d\n",source_points_.size());
  int i,j,k;

  pcl::PointXYZ search_point;

  Eigen::MatrixXd J(source_points_.size(),4), JJ,J_transpose;
  Eigen::VectorXd y[3];
  Eigen::Vector4d solutions[3];

  Eigen::Matrix3d current_iteration_rotation;
  Eigen::Vector3d current_iteration_translation;

  std::vector < Eigen::Vector3d > current_iteration_source_points = source_points_;

  std::vector < int > point_index(1);
  std::vector < float > point_distance(1);

  for(i = 0; i < 3; ++i)
  {
    y[i].resize(source_points_.size());

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

    Eigen::Matrix4d current_homogeneus_matrix,resulting_homogeneus_matrix;

    current_homogeneus_matrix.block(0, 0, 3, 3) = current_iteration_rotation;
    current_homogeneus_matrix.block(0, 3, 3, 1) = current_iteration_translation;
    current_homogeneus_matrix.row(3) << 0, 0, 0, 1;


    resulting_homogeneus_matrix = current_homogeneus_matrix * homogeneus_matrix_;

    double difference = ( resulting_homogeneus_matrix - homogeneus_matrix_ ).norm();
/*
    if(difference < 0.1)
        break;
*/
    homogeneus_matrix_ = resulting_homogeneus_matrix;

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

  //std::cout<< "Transformation matrix\n"<<homogeneus_matrix_<<std::endl<<std::endl;

}

void
Registration::writeDataToPCD(std::string file_path)
{
  pcl::PointCloud < pcl::PointXYZRGB > rigid_cloud, output_cloud, target_cloud;
  pcl::PointXYZRGB point;
  int i;
  uint32_t rgb;
  uint8_t value(255);

  rigid_cloud.width = rigid_transformed_points_.size();
  rigid_cloud.height = 1;

  for( i = 0; i < rigid_transformed_points_.size(); ++i)
  {
    /*
    point.x = source_points_[i][0];
    point.y = source_points_[i][1];
    point.z = source_points_[i][2];

    rgb = ((uint32_t)value) << 16;
    point.rgb = *reinterpret_cast<float*>(&rgb);

    rigid_cloud.points.push_back(point);
    */


    point.x = rigid_transformed_points_[i][0];
    point.y = rigid_transformed_points_[i][1];
    point.z = rigid_transformed_points_[i][2];

    rgb = ((uint32_t)value);
    point.rgb = *reinterpret_cast<float*>(&rgb);

    rigid_cloud.points.push_back(point);

  }

  pcl::copyPointCloud(*target_point_cloud_ptr_,target_cloud);


  rgb = ((uint32_t)value) << 8;

  for( i = 0; i < target_cloud.points.size(); ++i)
  {
      target_cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }

  output_cloud = rigid_cloud + target_cloud;





  pcl::io::savePCDFileASCII (file_path + ".pcd", output_cloud);


}

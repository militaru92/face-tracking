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

/*
  translation[0] = 1.5;
  translation[1] = 1.0;
  translation[2] = 0.0;

*/

  translation[0] = 0.0;
  translation[1] = 0.0;
  translation[2] = 0.0;



  readOBJFile(source_points_path,source_points_);

  for( i = 0; i < source_points_.size(); ++i)
      source_points_[i] = transform * source_points_[i] + translation;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  target_point_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  target_normal_cloud_ptr_.reset(new pcl::PointCloud<pcl::Normal>);


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

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(target_point_cloud_ptr_);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimator.setRadiusSearch(0.1);
  normal_estimator.compute(*target_normal_cloud_ptr_);



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

  Eigen::MatrixXd JJ, J_transpose, J;
  Eigen::VectorXd y( 3 * source_points_.size());
  Eigen::VectorXd solutions;

  Eigen::Matrix3d current_iteration_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d current_iteration_translation, dot_product, cross_product, normal,eigen_point;

  std::vector < Eigen::Vector3d > current_iteration_source_points = source_points_;

  std::vector < int > point_index(1);
  std::vector < float > point_distance(1);

  y.resize( 3 * source_points_.size());

  J = Eigen::MatrixXd::Zero(3 * source_points_.size(), 6);





  for(k = 0; k < number_of_iterations; ++k)
  {

    PCL_INFO("Iteration %d\n",k);

    j = 0;

    for(i = 0; i < current_iteration_source_points.size(); ++i)
    {


      search_point.x = current_iteration_source_points[i][0];
      search_point.y = current_iteration_source_points[i][1];
      search_point.z = current_iteration_source_points[i][2];

      kdtree_.nearestKSearch(search_point,1,point_index,point_distance);

      normal[0] = target_normal_cloud_ptr_->points[point_index[0]].normal_x;
      normal[1] = target_normal_cloud_ptr_->points[point_index[0]].normal_y;
      normal[2] = target_normal_cloud_ptr_->points[point_index[0]].normal_z;

      cross_product = current_iteration_source_points[i].cross(normal);

      y[j] = (target_point_cloud_ptr_->points[point_index[0]].x - search_point.x) * normal[0];
      y[j+1] = (target_point_cloud_ptr_->points[point_index[0]].y - search_point.y) * normal[1];
      y[j+2] = (target_point_cloud_ptr_->points[point_index[0]].z - search_point.z) * normal[2];

      J(j,0) = normal[0];
      J(j,1) = cross_product[0];

      J(j+1,2) = normal[1];
      J(j+1,3) = cross_product[1];

      J(j+2,4) = normal[2];
      J(j+2,5) = cross_product[2];

      j = j+3;

    }

    J_transpose = J.transpose();
    JJ = J_transpose * J;



    solutions = JJ.colPivHouseholderQr().solve(J_transpose * y);

    PCL_INFO("Done with linear Solvers\n");

    j = 0;

    for(i = 0; i < 3; ++i)
    {
      current_iteration_translation(i) = solutions(j);
      j = j+2;
    }

    current_iteration_rotation(2,1) = solutions[1];
    current_iteration_rotation(1,2) = -solutions[1];

    current_iteration_rotation(0,2) = solutions[3];
    current_iteration_rotation(2,0) = -solutions[3];

    current_iteration_rotation(1,0) = solutions[5];
    current_iteration_rotation(0,1) = -solutions[5];


    for( i = 0; i < current_iteration_source_points.size(); ++i)
    {
      current_iteration_source_points[i] = current_iteration_rotation * current_iteration_source_points[i] + current_iteration_translation;
    }

    Eigen::Matrix4d current_homogeneus_matrix,resulting_homogeneus_matrix;

    current_homogeneus_matrix.block(0, 0, 3, 3) = current_iteration_rotation;
    current_homogeneus_matrix.block(0, 3, 3, 1) = current_iteration_translation;
    current_homogeneus_matrix.row(3) << 0, 0, 0, 1;


    resulting_homogeneus_matrix = current_homogeneus_matrix * homogeneus_matrix_;

    /*
    double difference = ( resulting_homogeneus_matrix - homogeneus_matrix_ ).norm();

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

  rigid_cloud.width = 2 * rigid_transformed_points_.size();
  rigid_cloud.height = 1;

  for( i = 0; i < rigid_transformed_points_.size(); ++i)
  {

    point.x = source_points_[i][0];
    point.y = source_points_[i][1];
    point.z = source_points_[i][2];

    rgb = ((uint32_t)value) << 16;
    point.rgb = *reinterpret_cast<float*>(&rgb);

    rigid_cloud.points.push_back(point);



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

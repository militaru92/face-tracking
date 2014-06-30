#include <registration.h>

Registration::Registration()
{
  homogeneus_matrix_ = Eigen::Matrix4d::Identity();
  position_model_ = NULL;

  //target_point_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
  //target_normal_cloud_ptr.reset(new pcl::PointCloud<pcl::Normal>);
  target_point_normal_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointNormal>);

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);


  int i;

  std::vector < Eigen::Vector3d > target_points;

  readOBJFile(source_points_path,source_points_, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  readOBJFile(target_points_path, target_points, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());



  target_point_cloud_ptr->width = target_points.size();
  target_point_cloud_ptr->height = 1;

  for( i = 0; i < target_points.size(); ++i)
  {
    pcl_point.x = target_points[i][0];
    pcl_point.y = target_points[i][1];
    pcl_point.z = target_points[i][2];

    target_point_cloud_ptr->points.push_back(pcl_point);
  }

    setKdTree(target_point_cloud_ptr);



}

void
Registration::readOBJFile(std::string file_path, std::vector < Eigen::Vector3d >& points_vector, Eigen::Matrix3d transform_matrix, Eigen::Vector3d translation)
{

  std::ifstream instream(file_path.c_str());
  std::string line;

  Eigen::Vector3d point, result_point;

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

    result_point = transform_matrix * point + translation;

    points_vector.push_back(result_point);


  }

  instream.close();
}

void
Registration::readDataFromOBJFileAndPCDScan(std::string source_points_path, std::string target_points_path, Eigen::Matrix3d transform_matrix, Eigen::Vector3d translation)
{

  int i;

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  readOBJFile(source_points_path,source_points_, transform_matrix, translation);


  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_points_path, *target_point_cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR("Could not open file %s\n", target_points_path.c_str());
    exit(1);
  }
/*
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outliers_filter;
  outliers_filter.setInputCloud(scan_cloud);
  outliers_filter.setMeanK(50);
  outliers_filter.setStddevMulThresh(10);
  outliers_filter.filter(*target_point_cloud_ptr);
*/
  //pcl::io::savePCDFileASCII("filtered_cloud.pcd", *target_point_cloud_ptr);

  setKdTree(target_point_cloud_ptr);



}

void
Registration::getDataFromModel(std::string database_path, std::string output_path, Eigen::MatrixX3d rotation, Eigen::Vector3d translation)
{
  int i;

  pcl::PointXYZ point;

  std::vector < Eigen::Vector3d > target_points;


  pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);




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


  target_point_cloud_ptr->width = target_points.size();
  target_point_cloud_ptr->height = 1;

  for( i = 0; i < target_points.size(); ++i)
  {
    point.x = target_points[i][0];
    point.y = target_points[i][1];
    point.z = target_points[i][2];

    target_point_cloud_ptr->points.push_back(point);
  }

  setKdTree(target_point_cloud_ptr);





}

void
Registration::calculateRigidTransformation(int number_of_iterations)
{

  PCL_INFO("In calculate method\n");
  PCL_INFO("Size of sources_ and targets_ %d\n",source_points_.size());
  int i,j,k;



  Eigen::MatrixXd JJ, J_transpose, J(source_points_.size(), 6);
  Eigen::VectorXd y(source_points_.size());
  Eigen::VectorXd solutions;

  Eigen::Matrix3d current_iteration_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d current_iteration_translation;

  std::vector < Eigen::Vector3d > current_iteration_source_points = source_points_;


  for(k = 0; k < number_of_iterations; ++k)
  {

    PCL_INFO("Iteration %d\n",k);


    for(i = 0; i < current_iteration_source_points.size(); ++i)
    {

      pcl::PointNormal search_point;


      search_point.x = current_iteration_source_points[i][0];
      search_point.y = current_iteration_source_points[i][1];
      search_point.z = current_iteration_source_points[i][2];

      std::vector < int > point_index(1);
      std::vector < float > point_distance(1);

      kdtree_.nearestKSearch(search_point,1,point_index,point_distance);

      Eigen::Vector3d cross_product, normal,eigen_point;

      normal[0] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_x;
      normal[1] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_y;
      normal[2] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_z;

      cross_product = current_iteration_source_points[i].cross(normal);

      eigen_point[0] = target_point_normal_cloud_ptr_->points[point_index[0]].x - search_point.x;
      eigen_point[1] = target_point_normal_cloud_ptr_->points[point_index[0]].y - search_point.y;
      eigen_point[2] = target_point_normal_cloud_ptr_->points[point_index[0]].z - search_point.z;

      y[i] = eigen_point.dot(normal);

      J(i,0) = cross_product[0];
      J(i,1) = cross_product[1];
      J(i,2) = cross_product[2];
      J(i,3) = normal[0];
      J(i,4) = normal[1];
      J(i,5) = normal[2];


    }

    J_transpose = J.transpose();
    JJ = J_transpose * J;

    Eigen::MatrixXd right_side = J_transpose * y;



    solutions = JJ.colPivHouseholderQr().solve(right_side);

    PCL_INFO("Done with linear Solvers\n");


    current_iteration_rotation = Eigen::AngleAxisd(solutions[0],Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(solutions[1],Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(solutions[2],Eigen::Vector3d::UnitZ()) ;

    /*
    Eigen::Matrix3d R_x,R_y,R_z;

    R_x = Eigen::Matrix3d::Identity();
    R_y = Eigen::Matrix3d::Identity();
    R_z = Eigen::Matrix3d::Identity();



    R_x(1,1) = cos(solutions[0]);
    R_x(2,2) = cos(solutions[0]);

    R_x(1,2) = -sin(solutions[0]);
    R_x(2,1) = sin(solutions[0]);

    R_y(0,0) = cos(solutions[1]);
    R_y(2,2) = cos(solutions[1]);

    R_y(0,2) = sin(solutions[1]);
    R_y(2,0) = -sin(solutions[1]);

    R_z(0,0) = cos(solutions[2]);
    R_z(1,1) = cos(solutions[2]);

    R_z(1,0) = sin(solutions[2]);
    R_z(0,1) = -sin(solutions[2]);

    current_iteration_rotation = R_x * R_y * R_z;
    */


    for(i = 0; i < 3; ++i)
    {
      current_iteration_translation(i) = solutions(i+3);
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

/*
    double difference = ( resulting_homogeneus_matrix - homogeneus_matrix_ ).norm();

    if(difference < 0.1)
        break;
*/
    homogeneus_matrix_ = resulting_homogeneus_matrix;

  }

  transformed_points_ = current_iteration_source_points;





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

  pcl::copyPointCloud(*target_point_normal_cloud_ptr_,target_cloud);


  rgb = ((uint32_t)value) << 8;

  for( i = 0; i < target_cloud.points.size(); ++i)
  {
      target_cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }

  output_cloud = rigid_cloud + target_cloud;

  pcl::PCDWriter pcd_writer;

  pcd_writer.writeBinary < pcl::PointXYZRGB > (file_path + ".pcd", output_cloud);


}

void
Registration::setKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr)
{

  pcl::PointCloud<pcl::Normal>::Ptr target_normal_cloud_ptr(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);


  normal_estimator.setInputCloud(target_point_cloud_ptr);
  tree->setInputCloud(target_point_cloud_ptr);
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setKSearch(10);
  //normal_estimator.setRadiusSearch(0.10);
  normal_estimator.compute(*target_normal_cloud_ptr);

  pcl::concatenateFields (*target_point_cloud_ptr, *target_normal_cloud_ptr, *target_point_normal_cloud_ptr_);

  kdtree_.setInputCloud(target_point_normal_cloud_ptr_);

}

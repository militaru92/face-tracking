#include <registration.h>

Registration::Registration()
{
  homogeneus_matrix_ = Eigen::Matrix4d::Identity();
  position_model_ = NULL;

  vis_source_point_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  vis_scan_point_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  vis_model_point_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  source_point_normal_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  target_point_normal_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  rigid_transformed_points_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

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

  readOBJFile(source_points_path,source_point_normal_cloud_ptr_, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  readOBJFile(target_points_path, target_point_normal_cloud_ptr_, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());


  kdtree_.setInputCloud(target_point_normal_cloud_ptr_);



}

void
Registration::readOBJFile(std::string file_path, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, Eigen::Matrix3d transform_matrix, Eigen::Vector3d translation, bool advance, int number_vertices)
{

  std::ifstream instream(file_path.c_str());
  std::string line,value;
  int i,j;
  size_t index;

  pcl::PointXYZRGBNormal point;

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

    ss >> point.x;
    ss >> point.y;
    ss >> point.z;


    cloud->push_back(point);


  }


  if (advance)
  {

    std::vector < std::vector < Eigen::Vector3d > > normal_vector(cloud->points.size());
    std::vector < std::vector < double > > surface_vector(cloud->points.size());

    while(line[0] != 'f')
    {
      std::getline(instream,line);
    }



    while(line[0] == 'f' && !instream.eof())
    {
      int mesh[number_vertices];
      Eigen::Vector3d normal;

      for(j = 0; j < number_vertices; ++j)
      {
        index = line.find(' ');
        line = line.substr(index+1);
        index = line.find(' ');
        value = line.substr(0,index);
        mesh[j] = (boost::lexical_cast<int>(value) -1);
      }

      Eigen::Vector3d vector_lines[number_vertices-1];


      for( i = 0; i < number_vertices - 1; ++i)
      {
        vector_lines[i] = cloud->points[mesh[i+1]].getVector3fMap().cast<double>() - cloud->points[mesh[0]].getVector3fMap().cast<double>();
      }



      double area = 0.0;

      for( i = 0; i < number_vertices - 2; ++i)
      {
        normal = vector_lines[i].cross(vector_lines[i+1]);
        area = area + (0.5 * normal.norm());
      }



      normal = vector_lines[0].cross(vector_lines[1]);
      normal.normalize();



      for(i = 0; i < number_vertices; ++i)
      {
        normal_vector[mesh[i]].push_back(normal);
        surface_vector[mesh[i]].push_back(area);
      }

      std::getline(instream,line);

    }


    for( i = 0; i < cloud->points.size(); ++i)
    {
      double ratio = 0.0;
      Eigen::Vector3d normal_result = Eigen::Vector3d::Zero();

      for( j = 0; j < normal_vector[i].size(); ++j)
      {
        ratio += surface_vector[i][j];
      }

      for( j = 0; j < normal_vector[i].size(); ++j)
      {
        normal_result += ( ( surface_vector[i][j] * normal_vector[i][j] ) );
      }


      normal_result /= ratio;
      normal_result.normalize();



      cloud->points[i].normal_x = normal_result[0];
      cloud->points[i].normal_y = normal_result[1];
      cloud->points[i].normal_z = normal_result[2];


    }

  }

  Eigen::Matrix4d homogeneus_transform;

  homogeneus_transform.block(0, 0, 3, 3) = transform_matrix;
  homogeneus_transform.block(0, 3, 3, 1) = translation;
  homogeneus_transform.row(3) << 0, 0, 0, 1;

  pcl::PointCloud<pcl::PointXYZRGBNormal> result_point_cloud;


  pcl::transformPointCloudWithNormals(*cloud,result_point_cloud,homogeneus_transform);

  *cloud = result_point_cloud;





  instream.close();


}


void
Registration::readDataFromOBJFileAndPCDScan(std::string source_points_path, std::string target_points_path, Eigen::Matrix3d transform_matrix, Eigen::Vector3d translation)
{


  pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);


  readOBJFile(source_points_path,source_point_normal_cloud_ptr_, transform_matrix, translation);


  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_points_path, *target_point_cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR("Could not open file %s\n", target_points_path.c_str());
    exit(1);
  }


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

  position_model_->writeMeanFaceAndRotatedMeanFace(rotation, translation, output_path + "_source.obj", output_path +"_transformed.obj",source_point_normal_cloud_ptr_,target_point_normal_cloud_ptr_);


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
  int i,j,k;


  pcl::visualization::PCLVisualizer visualizer("3D Viewer");
  visualizer.setBackgroundColor(0, 0, 0);
  visualizer.initCameraParameters();

  //visualizer.registerMouseCallback <Registration>(&Registration::mouseEventOccurred, *this);


  uint32_t rgb;
  uint8_t value(255);

  rgb = ((uint32_t)value) << 16;


  for( i = 0; i < source_point_normal_cloud_ptr_->points.size(); ++i)
  {
      source_point_normal_cloud_ptr_->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }



  rgb = ((uint32_t)value) << 8;


  for( i = 0; i < target_point_normal_cloud_ptr_->points.size(); ++i)
  {
      target_point_normal_cloud_ptr_->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }


  Eigen::MatrixXd J(source_point_normal_cloud_ptr_->points.size(), 6);
  Eigen::VectorXd y(source_point_normal_cloud_ptr_->points.size());


  Eigen::Matrix3d current_iteration_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d current_iteration_translation;

  pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr current_iteration_source_points_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  pcl::copyPointCloud(*source_point_normal_cloud_ptr_, *current_iteration_source_points_ptr);



  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_source(source_point_normal_cloud_ptr_);


  visualizer.addPointCloud <pcl::PointXYZRGBNormal> (source_point_normal_cloud_ptr_,rgb_source, "source");
  visualizer.addPointCloudNormals <pcl::PointXYZRGBNormal> (source_point_normal_cloud_ptr_, 20, 0.1, "normals");

  visualizer.spin();

  visualizer.removeAllPointClouds();



  for(j = 0; j < number_of_iterations; ++j)
  {

    PCL_INFO("Iteration %d\n",j);

    pcl::Correspondences iteration_correspondences;

    k = 0;

    for(i = 0; i < current_iteration_source_points_ptr->size(); ++i)
    {

      pcl::PointXYZRGBNormal search_point;


      search_point = current_iteration_source_points_ptr->at(i);

      std::vector < int > point_index(1);
      std::vector < float > point_distance(1);

      kdtree_.nearestKSearch(search_point,1,point_index,point_distance);



      Eigen::Vector3d cross_product, normal,eigen_point;
      Eigen::Vector3d source_normal;

      double dot_product;

      source_normal = search_point.getNormalVector3fMap().cast<double>();


      normal[0] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_x;
      normal[1] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_y;
      normal[2] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_z;


      normal.normalize();

      source_normal.normalize();

      dot_product = source_normal.dot(normal);


      if( point_distance[0] < 0.01 )
      {

        if( dot_product > 1.0 / sqrt(2.0) )
        {
          Eigen::Vector3d aux_vector;

          aux_vector = (current_iteration_source_points_ptr->at(i).getVector3fMap()).cast<double>();

          cross_product = aux_vector.cross(normal);

          eigen_point = (target_point_normal_cloud_ptr_->at(point_index[0]).getVector3fMap().cast<double>()) - (search_point.getVector3fMap().cast<double>());

          pcl::Correspondence correspondence(i,point_index[0],point_distance[0]);

          iteration_correspondences.push_back(correspondence);

          y[k] = eigen_point.dot(normal);

          J(k,0) = cross_product[0];
          J(k,1) = cross_product[1];
          J(k,2) = cross_product[2];
          J(k,3) = normal[0];
          J(k,4) = normal[1];
          J(k,5) = normal[2];

          ++k;
        }
      }


    }


    PCL_INFO ("Remaining points %d \n",k);


    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_cloud_target(target_point_normal_cloud_ptr_);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_cloud_current_source(current_iteration_source_points_ptr);


    visualizer.addPointCloud < pcl::PointXYZRGBNormal > (current_iteration_source_points_ptr, rgb_cloud_current_source, "source");
    visualizer.addPointCloud < pcl::PointXYZRGBNormal > (target_point_normal_cloud_ptr_, rgb_cloud_target, "scan");
    visualizer.addCorrespondences <pcl::PointXYZRGBNormal> (current_iteration_source_points_ptr, target_point_normal_cloud_ptr_, iteration_correspondences);


    visualizer.spin();


    visualizer.removeAllShapes();
    visualizer.removeAllPointClouds();
    visualizer.removeCorrespondences();


    Eigen::MatrixXd J_transpose,JJ, J_filtered(k,6);

    Eigen::VectorXd solutions, y_filtered;

    J_filtered = J.block(0,0,k,6);

    y_filtered = y.block(0,0,k,1);



    J_transpose = J_filtered.transpose();
    JJ = J_transpose * J_filtered;

    Eigen::MatrixXd right_side = J_transpose * y_filtered;



    solutions = JJ.colPivHouseholderQr().solve(right_side);

    PCL_INFO("Done with linear Solvers\n");


    current_iteration_rotation = Eigen::AngleAxisd(solutions[0],Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(solutions[1],Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(solutions[2],Eigen::Vector3d::UnitZ()) ;


    for(i = 0; i < 3; ++i)
    {
      current_iteration_translation(i) = solutions(i+3);
    }



    Eigen::Matrix4d current_homogeneus_matrix,resulting_homogeneus_matrix;

    current_homogeneus_matrix.block(0, 0, 3, 3) = current_iteration_rotation;
    current_homogeneus_matrix.block(0, 3, 3, 1) = current_iteration_translation;
    current_homogeneus_matrix.row(3) << 0, 0, 0, 1;

    pcl::PointCloud<pcl::PointXYZRGBNormal> result;

    pcl::transformPointCloudWithNormals(*current_iteration_source_points_ptr,result,current_homogeneus_matrix);

    *current_iteration_source_points_ptr = result;


    resulting_homogeneus_matrix = current_homogeneus_matrix * homogeneus_matrix_;

/*
    double difference = ( resulting_homogeneus_matrix - homogeneus_matrix_ ).norm();

    if(difference < 0.1)
        break;
*/

    homogeneus_matrix_ = resulting_homogeneus_matrix;

    //homogeneus_matrices_vector_.push_back(current_homogeneus_matrix);

    //iteration_correspondences_vector_.push_back(iteration_correspondences);




  }

}



void
Registration::applyRigidTransformation()
{



  pcl::transformPointCloudWithNormals (*source_point_normal_cloud_ptr_,*rigid_transformed_points_ptr_,homogeneus_matrix_);


  uint32_t rgb;
  uint8_t value(255);

  rgb = ((uint32_t)value);

  for( int i = 0; i < rigid_transformed_points_ptr_->points.size(); ++i)
  {
      rigid_transformed_points_ptr_->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }



}

void
Registration::writeDataToPCD(std::string file_path)
{
  pcl::PointCloud < pcl::PointXYZRGBNormal > initial_cloud, rigid_cloud, output_cloud, target_cloud;
  pcl::PointXYZRGB point;
  int i;
  uint32_t rgb;
  uint8_t value(255);

  output_cloud = *source_point_normal_cloud_ptr_ + (*rigid_transformed_points_ptr_ + *target_point_normal_cloud_ptr_);

  pcl::PCDWriter pcd_writer;

  pcd_writer.writeBinary < pcl::PointXYZRGBNormal > (file_path + ".pcd", output_cloud);



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

void
Registration::mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

  if (event.getButton () == pcl::visualization::MouseEvent::RightButton)
  {
    std::cout << "X: " << event.getX() << " Y: " << event.getY() << std::endl;
  }
}




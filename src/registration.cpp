#include <registration.h>

Registration::Registration()
{
  homogeneus_matrix_ = Eigen::Matrix4d::Identity();

  original_source_point_normal_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  target_point_normal_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  rigid_transformed_points_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  iteration_source_point_normal_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  visualizer_ptr_.reset(new pcl::visualization::PCLVisualizer("3D Visualizer"));
  visualizer_ptr_->setBackgroundColor(1, 1, 1);
  visualizer_ptr_->initCameraParameters();

}


void
Registration::readDataFromOBJFiles(std::string source_points_path, std::string target_points_path)
{

  readOBJFile(source_points_path, original_source_point_normal_cloud_ptr_, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

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
        mesh[j] = (boost::lexical_cast<int>(value) - 1);
      }

      Eigen::Vector3d eigen_vector_1,eigen_vector_2;

      int maximum_index = 0;
      double angle, maximum_angle = 0.0;




      for( i = number_vertices; i < number_vertices * 2; ++i)
      {
        eigen_vector_1 = cloud->points[mesh[ (i - 1) % number_vertices ]].getVector3fMap().cast<double>() - cloud->points[mesh[ i % number_vertices ]].getVector3fMap().cast<double>();
        eigen_vector_2 = cloud->points[mesh[ (i + 1) % number_vertices ]].getVector3fMap().cast<double>() - cloud->points[mesh[ i % number_vertices ]].getVector3fMap().cast<double>();

        eigen_vector_1.normalize();
        eigen_vector_2.normalize();

        angle = eigen_vector_1.dot(eigen_vector_2);

        angle = acos(angle);

        if(angle > maximum_angle)
        {
          maximum_angle = angle;
          maximum_index = i % number_vertices;
        }
      }

      Eigen::Vector3d edge,normal_1,normal_2;


      edge = cloud->points[mesh[ (maximum_index + number_vertices - 2) % number_vertices ]].getVector3fMap().cast<double>() - cloud->points[mesh[ maximum_index ]].getVector3fMap().cast<double>();
      eigen_vector_1 = cloud->points[mesh[ (maximum_index + number_vertices - 1) % number_vertices ]].getVector3fMap().cast<double>() - cloud->points[mesh[ maximum_index ]].getVector3fMap().cast<double>();
      eigen_vector_2 = cloud->points[mesh[ (maximum_index + number_vertices + 1) % number_vertices ]].getVector3fMap().cast<double>() - cloud->points[mesh[ maximum_index ]].getVector3fMap().cast<double>();

      normal_1 = edge.cross(eigen_vector_1);
      normal_2 = eigen_vector_2.cross(edge);

      double area_1,area_2;

      area_1 = normal_1.norm() * 0.5;
      area_2 = normal_2.norm() * 0.5;

      normal_1.normalize();
      normal_2.normalize();


      normal_vector [ mesh [maximum_index] ].push_back(normal_1);
      normal_vector [ mesh [maximum_index] ].push_back(normal_2);
      surface_vector[ mesh [maximum_index] ].push_back(area_1);
      surface_vector[ mesh [maximum_index] ].push_back(area_2);


      normal_vector [ mesh [(maximum_index + number_vertices - 2) % number_vertices] ].push_back(normal_1);
      normal_vector [ mesh [(maximum_index + number_vertices - 2) % number_vertices] ].push_back(normal_2);
      surface_vector[ mesh [(maximum_index + number_vertices - 2) % number_vertices] ].push_back(area_1);
      surface_vector[ mesh [(maximum_index + number_vertices - 2) % number_vertices] ].push_back(area_2);


      normal_vector [ mesh [(maximum_index + number_vertices - 1) % number_vertices] ].push_back(normal_1);
      surface_vector[ mesh [(maximum_index + number_vertices - 1) % number_vertices] ].push_back(area_1);

      normal_vector [ mesh [(maximum_index + number_vertices + 1) % number_vertices] ].push_back(normal_2);
      surface_vector[ mesh [(maximum_index + number_vertices + 1) % number_vertices] ].push_back(area_2);



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

  pcl::copyPointCloud(result_point_cloud,*cloud);

  PCL_INFO("Warning %lf\n", cloud->at(0).getNormalVector3fMap().cast<double>().norm());




  instream.close();


}


void
Registration::readDataFromOBJFileAndPCDScan(std::string source_points_path, std::string target_points_path, Eigen::Matrix3d transform_matrix, Eigen::Vector3d translation)
{


  pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);


  readOBJFile(source_points_path, original_source_point_normal_cloud_ptr_, transform_matrix, translation);


  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_points_path, *target_point_cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR("Could not open file %s\n", target_points_path.c_str());
    exit(1);
  }


  setKdTree(target_point_cloud_ptr);



}

void
Registration::getDataFromModel(std::string database_path, Eigen::MatrixX3d transformation_matrix, Eigen::Vector3d translation)
{
  int i,j;

  PositionModel position_model;

  position_model.readDataFromFolders(database_path,150,4,transformation_matrix,translation);

  eigen_source_points_ = position_model.calculateMeanFace();

  position_model.calculateEigenValuesAndVectors();

  model_mesh_ = position_model.getMeshes();

  debug_model_mesh_ = model_mesh_;

  for(i = 0; i < debug_model_mesh_.size(); ++i)
  {
    for(j = 0; j < debug_model_mesh_[i].vertices.size(); ++j)
    {
      debug_model_mesh_[i].vertices[j] = debug_model_mesh_[i].vertices[j] - 1;
    }
  }
  eigenvalues_vector_ = position_model.getEigenValues();
  eigenvectors_matrix_ = position_model.getEigenVectors();
  PCL_INFO("Done with eigenvectors\n");

  convertEigenToPointCLoud();

  pcl::copyPointCloud(*iteration_source_point_normal_cloud_ptr_,*original_source_point_normal_cloud_ptr_);


  uint32_t rgb;
  uint8_t value(255);

  rgb = ((uint32_t)value) <<16;

  center_point_ = Eigen::Vector3d::Zero();

  for( i = 0; i < original_source_point_normal_cloud_ptr_->points.size(); ++i)
  {
    original_source_point_normal_cloud_ptr_->points[i].rgb = *reinterpret_cast<float*>(&rgb);
    center_point_ += original_source_point_normal_cloud_ptr_->points[i].getVector3fMap().cast<double>();
  }

  center_point_ /= static_cast<double> (original_source_point_normal_cloud_ptr_->points.size());

  PCL_INFO("Done with reading from model\n");


}

void
Registration::readTargetPointCloud(std::string target_points_path)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_points_path, *target_point_cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR("Could not open file %s\n", target_points_path.c_str());
    exit(1);
  }

  setKdTree(target_point_cloud_ptr);

  PCL_INFO("Done with reading from Target\n");

}

void
Registration::getTargetPointCloudFromCamera (int device, std::string file_classifier)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);


  CameraGrabber camera;

  camera.setCamera(device,file_classifier);

  pcl::copyPointCloud(*(camera.getPointCloud(center_coordinates_)),*target_point_cloud_ptr);

  setKdTree(target_point_cloud_ptr);



  PCL_INFO("Done with camera scanning\n");

}

void
Registration::getTargetPointCloudFromFile(std::string pcd_file)
{

  center_coordinates_.first = 335;
  center_coordinates_.second = 191;

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *target_point_cloud_ptr) == -1)
  {
    PCL_ERROR("Could not open file %s\n", pcd_file.c_str());
    exit(1);
  }


  setKdTree(target_point_cloud_ptr);
}


void
Registration::alignModel ()
{

  Eigen::Vector3d translation = target_point_normal_cloud_ptr_->at(center_coordinates_.first,center_coordinates_.second).getVector3fMap().cast<double>() - center_point_;

  pcl::transformPointCloudWithNormals(*original_source_point_normal_cloud_ptr_,*iteration_source_point_normal_cloud_ptr_,translation,Eigen::Quaternion<double>::Identity());

  pcl::PCDWriter pcd_writer;
  pcd_writer.writeBinary < pcl::PointXYZRGBNormal > ("model.pcd", *iteration_source_point_normal_cloud_ptr_);

  //pcl::copyPointCloud(*iteration_source_point_normal_cloud_ptr_, *source_point_normal_cloud_ptr_);
}

void
Registration::convertPointCloudToEigen()
{
  int i,j=0;



  for(i = 0; i < iteration_source_point_normal_cloud_ptr_->size(); ++i)
  {
    eigen_source_points_[j++] = iteration_source_point_normal_cloud_ptr_->at(i).x;
    eigen_source_points_[j++] = iteration_source_point_normal_cloud_ptr_->at(i).y;
    eigen_source_points_[j++] = iteration_source_point_normal_cloud_ptr_->at(i).z;
  }

}


void
Registration::convertEigenToPointCLoud()
{
   pcl::PointXYZRGBNormal pcl_point;
   int i,j,k;

   iteration_source_point_normal_cloud_ptr_->clear();

   for(i = 0; i < eigen_source_points_.rows(); i = i+3)
   {
     pcl_point.x = eigen_source_points_[i];
     pcl_point.y = eigen_source_points_[i+1];
     pcl_point.z = eigen_source_points_[i+2];

     iteration_source_point_normal_cloud_ptr_->push_back(pcl_point);
   }

   std::vector < std::vector < Eigen::Vector3d > > normal_vector(iteration_source_point_normal_cloud_ptr_->points.size());
   std::vector < std::vector < double > > surface_vector(iteration_source_point_normal_cloud_ptr_->points.size());

   for( k = 0; k < model_mesh_.size(); ++k)
   {

     Eigen::Vector3d eigen_vector_1,eigen_vector_2;

     int maximum_index = 0;
     int number_vertices = model_mesh_[k].vertices.size();
     double angle, maximum_angle = 0.0;




     for( i = model_mesh_[k].vertices.size(); i < model_mesh_[k].vertices.size() * 2; ++i)
     {
       eigen_vector_1 = iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ (i - 1) % number_vertices ] - 1].getVector3fMap().cast<double>() - iteration_source_point_normal_cloud_ptr_->points[ model_mesh_[k].vertices[ i % number_vertices ] - 1].getVector3fMap().cast<double>();
       eigen_vector_2 = iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ (i + 1) % number_vertices ] - 1].getVector3fMap().cast<double>() - iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ i % number_vertices ] - 1].getVector3fMap().cast<double>();

       eigen_vector_1.normalize();
       eigen_vector_2.normalize();

       angle = eigen_vector_1.dot(eigen_vector_2);

       angle = acos(angle);

       if(angle > maximum_angle)
       {
         maximum_angle = angle;
         maximum_index = i % number_vertices;
       }
     }

     Eigen::Vector3d edge,normal_1,normal_2;


     edge = iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ (maximum_index + number_vertices - 2) % number_vertices ] - 1].getVector3fMap().cast<double>() - iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ maximum_index ] - 1].getVector3fMap().cast<double>();
     eigen_vector_1 = iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ (maximum_index + number_vertices - 1) % number_vertices ] - 1].getVector3fMap().cast<double>() - iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ maximum_index ] - 1].getVector3fMap().cast<double>();
     eigen_vector_2 = iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ (maximum_index + number_vertices + 1) % number_vertices ] - 1].getVector3fMap().cast<double>() - iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ maximum_index ] - 1].getVector3fMap().cast<double>();

     normal_1 = edge.cross(eigen_vector_1);
     normal_2 = eigen_vector_2.cross(edge);

     double area_1,area_2;

     area_1 = normal_1.norm() * 0.5;
     area_2 = normal_2.norm() * 0.5;

     normal_1.normalize();
     normal_2.normalize();


     normal_vector [ model_mesh_[k].vertices [maximum_index] - 1 ].push_back(normal_1);
     normal_vector [ model_mesh_[k].vertices [maximum_index] - 1 ].push_back(normal_2);
     surface_vector[ model_mesh_[k].vertices [maximum_index] - 1 ].push_back(area_1);
     surface_vector[ model_mesh_[k].vertices [maximum_index] - 1 ].push_back(area_2);


     normal_vector [ model_mesh_[k].vertices [(maximum_index + number_vertices - 2) % number_vertices] - 1 ].push_back(normal_1);
     normal_vector [ model_mesh_[k].vertices [(maximum_index + number_vertices - 2) % number_vertices] - 1 ].push_back(normal_2);
     surface_vector[ model_mesh_[k].vertices [(maximum_index + number_vertices - 2) % number_vertices] - 1 ].push_back(area_1);
     surface_vector[ model_mesh_[k].vertices [(maximum_index + number_vertices - 2) % number_vertices] - 1 ].push_back(area_2);


     normal_vector [ model_mesh_[k].vertices [(maximum_index + number_vertices - 1) % number_vertices] - 1 ].push_back(normal_1);
     surface_vector[ model_mesh_[k].vertices [(maximum_index + number_vertices - 1) % number_vertices] - 1 ].push_back(area_1);

     normal_vector [ model_mesh_[k].vertices [(maximum_index + number_vertices + 1) % number_vertices] - 1 ].push_back(normal_2);
     surface_vector[ model_mesh_[k].vertices [(maximum_index + number_vertices + 1) % number_vertices] - 1 ].push_back(area_2);

   }


   for( i = 0; i < iteration_source_point_normal_cloud_ptr_->points.size(); ++i)
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



     iteration_source_point_normal_cloud_ptr_->points[i].normal_x = normal_result[0];
     iteration_source_point_normal_cloud_ptr_->points[i].normal_y = normal_result[1];
     iteration_source_point_normal_cloud_ptr_->points[i].normal_z = normal_result[2];


   }


   uint32_t rgb;
   uint8_t value(255);

   rgb = ((uint32_t)value);

   for( i = 0; i < iteration_source_point_normal_cloud_ptr_->points.size(); ++i)
   {
     iteration_source_point_normal_cloud_ptr_->points[i].rgb = *reinterpret_cast<float*>(&rgb);
   }

}

void
Registration::calculateRigidTransformation(int number_of_iterations, double angle_limit, double distance_limit, bool visualize)
{

  PCL_INFO("In calculate method\n");
  int i,j,k;


  uint32_t rgb;
  uint8_t value(255);

  rgb = ((uint32_t)value) << 16;

/*
  for( i = 0; i < original_source_point_normal_cloud_ptr_->points.size(); ++i)
  {
    original_source_point_normal_cloud_ptr_->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }
*/


  rgb = ((uint32_t)value) << 8;


  for( i = 0; i < target_point_normal_cloud_ptr_->points.size(); ++i)
  {
      target_point_normal_cloud_ptr_->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }


  Eigen::MatrixXd J(iteration_source_point_normal_cloud_ptr_->points.size(), 6);
  Eigen::VectorXd y(iteration_source_point_normal_cloud_ptr_->points.size());


  Eigen::Matrix3d current_iteration_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d current_iteration_translation;

  pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr current_iteration_source_points_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  pcl::copyPointCloud(*iteration_source_point_normal_cloud_ptr_, *current_iteration_source_points_ptr);

  if(visualize)
  {

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_source(iteration_source_point_normal_cloud_ptr_);

    visualizer_ptr_->addPointCloud <pcl::PointXYZRGBNormal> (iteration_source_point_normal_cloud_ptr_,rgb_source, "source");
    visualizer_ptr_->addPointCloudNormals <pcl::PointXYZRGBNormal> (iteration_source_point_normal_cloud_ptr_, 20, 0.1, "normals");

    visualizer_ptr_->spin();

    visualizer_ptr_->removeAllPointClouds();


  }


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


      if( point_distance[0] < distance_limit && std::acos(dot_product) < angle_limit)
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


    PCL_INFO ("Remaining points %d \n",k);
/*
    if(k == 0)
    {
      PCL_ERROR("No correspondences found for Rigid Transformation; The initial alignment must be wrong\n");
      exit(1);
    }
*/
    if(visualize)
    {
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_cloud_target(target_point_normal_cloud_ptr_);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_cloud_current_source(current_iteration_source_points_ptr);


      //visualizer_ptr_->addPointCloud < pcl::PointXYZRGBNormal > (current_iteration_source_points_ptr, rgb_cloud_current_source, "source");
      visualizer_ptr_->addPolygonMesh < pcl::PointXYZRGBNormal > (current_iteration_source_points_ptr,debug_model_mesh_);
      visualizer_ptr_->addPointCloud < pcl::PointXYZRGBNormal > (target_point_normal_cloud_ptr_, rgb_cloud_target, "scan");
      visualizer_ptr_->addCorrespondences <pcl::PointXYZRGBNormal> (current_iteration_source_points_ptr, target_point_normal_cloud_ptr_, iteration_correspondences);

      visualizer_ptr_->spin();

      visualizer_ptr_->removeAllShapes();
      visualizer_ptr_->removeAllPointClouds();
      visualizer_ptr_->removeCorrespondences();

    }



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

    pcl::copyPointCloud(result,*current_iteration_source_points_ptr);


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


  pcl::copyPointCloud(*current_iteration_source_points_ptr,*iteration_source_point_normal_cloud_ptr_);
  convertPointCloudToEigen();

}

void
Registration::calculateNonRigidTransformation(int number_eigenvectors, double reg_weight, double angle_limit, double distance_limit, bool visualize)
{

  int i;

  pcl::Correspondences correspondences;

  correspondences = filterNonRigidCorrespondences(angle_limit,distance_limit);


  Eigen::MatrixXd J_transpose,JJ_total, J_point_to_point(correspondences.size() * 3,number_eigenvectors), Reg_diagonal_matrix;
  Eigen::VectorXd d,Jy,y(correspondences.size() * 3);

  Reg_diagonal_matrix = Eigen::MatrixXd::Identity(number_eigenvectors,number_eigenvectors);

  for(i = 0; i < Reg_diagonal_matrix.rows(); ++i)
  {
    Reg_diagonal_matrix(i,i) = 1.0 / eigenvalues_vector_[i];
  }



  for( i = 0; i < correspondences.size(); ++i)
  {

    J_point_to_point.row(i * 3) = eigenvectors_matrix_.block(correspondences[i].index_query * 3,0,1,number_eigenvectors);
    J_point_to_point.row(i * 3 + 1) = eigenvectors_matrix_.block(correspondences[i].index_query * 3 + 1,0,1,number_eigenvectors);
    J_point_to_point.row(i * 3 + 2) = eigenvectors_matrix_.block(correspondences[i].index_query * 3 + 2,0,1,number_eigenvectors);


    y(i * 3) = target_point_normal_cloud_ptr_->at(correspondences[i].index_match).x - eigen_source_points_(correspondences[i].index_query * 3);
    y(i * 3 + 1) = target_point_normal_cloud_ptr_->at(correspondences[i].index_match).y - eigen_source_points_(correspondences[i].index_query * 3 + 1);
    y(i * 3 + 2) = target_point_normal_cloud_ptr_->at(correspondences[i].index_match).z - eigen_source_points_(correspondences[i].index_query * 3 + 2);


  }



  J_transpose = J_point_to_point.transpose();

  JJ_total = (J_transpose * J_point_to_point) + (Reg_diagonal_matrix * reg_weight);


  Jy = J_transpose * y;



  d = JJ_total.colPivHouseholderQr().solve(Jy);

  eigen_source_points_ += eigenvectors_matrix_.block(0,0,eigenvectors_matrix_.rows(),number_eigenvectors) * d;

  convertEigenToPointCLoud();

  if(visualize)
  {
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_cloud_target(target_point_normal_cloud_ptr_);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_cloud_current_source(iteration_source_point_normal_cloud_ptr_);


    visualizer_ptr_->addPointCloud < pcl::PointXYZRGBNormal > (iteration_source_point_normal_cloud_ptr_, rgb_cloud_current_source, "source");
    //visualizer_ptr_->addPointCloud < pcl::PointXYZRGBNormal > (target_point_normal_cloud_ptr_, rgb_cloud_target, "scan");
    //visualizer_ptr_->addCorrespondences <pcl::PointXYZRGBNormal> (iteration_source_point_normal_cloud_ptr_, target_point_normal_cloud_ptr_, correspondences);


    visualizer_ptr_->spin();


    visualizer_ptr_->removeAllShapes();
    visualizer_ptr_->removeAllPointClouds();
    visualizer_ptr_->removeCorrespondences();

  }


}


void
Registration::calculateAlternativeTransformations(int number_eigenvectors, double reg_weight, int number_of_total_iterations, int number_of_rigid_iterations, double angle_limit, double distance_limit, bool visualize)
{
  int i;

  for( i = 0; i < number_of_total_iterations; ++i)
  {

    calculateRigidTransformation(number_of_rigid_iterations,angle_limit,distance_limit,visualize);

    calculateNonRigidTransformation(number_eigenvectors,reg_weight,angle_limit,distance_limit,visualize);

  }
}




void
Registration::writeDataToPCD(std::string file_path)
{
    /*
  pcl::PointCloud < pcl::PointXYZRGBNormal > output_cloud;

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

  transform.block(0, 0, 3, 3) = (Eigen::AngleAxisd(4 * atan(1), Eigen::Vector3d::UnitX()) *  Eigen::Matrix3d::Identity());

  pcl::transformPointCloudWithNormals(*iteration_source_point_normal_cloud_ptr_, output_cloud, transform);
*/
  pcl::PCDWriter pcd_writer;

  pcd_writer.writeBinary < pcl::PointXYZRGBNormal > (file_path + ".pcd", *iteration_source_point_normal_cloud_ptr_);
  pcd_writer.writeBinary < pcl::PointXYZRGBNormal > (file_path + "_target.pcd", *target_point_normal_cloud_ptr_);




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

  int i,j=0;

  eigen_target_points_.resize(3 * target_point_normal_cloud_ptr_->size());

  for(i = 0; i < target_point_normal_cloud_ptr_->size(); ++i)
  {
    eigen_target_points_[j++] = target_point_normal_cloud_ptr_->at(i).x;
    eigen_target_points_[j++] = target_point_normal_cloud_ptr_->at(i).y;
    eigen_target_points_[j++] = target_point_normal_cloud_ptr_->at(i).z;
  }



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


pcl::Correspondences
Registration::filterNonRigidCorrespondences(double angle_limit, double distance_limit)
{
  int i;

  pcl::Correspondences correspondences_vector;



  for(i = 0; i < iteration_source_point_normal_cloud_ptr_->size(); ++i)
  {

    pcl::PointXYZRGBNormal search_point;


    search_point = iteration_source_point_normal_cloud_ptr_->at(i);

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


    if( point_distance[0] < distance_limit )
    {

      if( std::acos(dot_product) < angle_limit )
      {
        Eigen::Vector3d aux_vector;

        aux_vector = (iteration_source_point_normal_cloud_ptr_->at(i).getVector3fMap()).cast<double>();

        cross_product = aux_vector.cross(normal);

        eigen_point = (target_point_normal_cloud_ptr_->at(point_index[0]).getVector3fMap().cast<double>()) - (search_point.getVector3fMap().cast<double>());

        pcl::Correspondence correspondence(i,point_index[0],point_distance[0]);

        correspondences_vector.push_back(correspondence);


      }
    }


  }

  return correspondences_vector;
}



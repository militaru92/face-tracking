#include <registration.h>

Registration::Registration ()
{
  target_point_normal_cloud_ptr_.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  iteration_source_point_normal_cloud_ptr_.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  index_ = 0;
  continue_tracking_ = true;
  first_face_found_ = false;
  debug_mode_on_ = false;
  calculate_ = false;

}

void
Registration::setDebugMode (bool debug_mode)
{
  debug_mode_on_ = debug_mode;
}

void
Registration::getDataFromModel (std::string database_path, Eigen::MatrixX3d transformation_matrix, Eigen::Vector3d translation)
{
  int i,j;

  boost::filesystem::path data_path (database_path);

  if ( boost::filesystem::exists (data_path) )
  {
    if ( boost::filesystem::is_directory (data_path) )
    {
      PositionModel position_model;

      position_model.readDataFromFolders (database_path,150,4,transformation_matrix,translation);

      eigen_source_points_ = position_model.calculateMeanFace (true);

      position_model.calculateEigenValuesAndVectors ();

      model_mesh_ = position_model.getMeshes (true);

      eigenvalues_vector_ = position_model.getEigenValues (true);
      eigenvectors_matrix_ = position_model.getEigenVectors (true);
      PCL_INFO ("Done with eigenvectors\n");
    }

    else if ( boost::filesystem::is_regular_file (data_path))
    {

      std::ifstream ins (database_path.c_str ());

      int i,j,rows,cols;

      uint32_t vertice;

      std::string line;

      ins >> rows;

      eigen_source_points_.resize (rows);

      double aux;


      for (i = 0; i < rows; ++i)
      {
        ins >> aux;
        eigen_source_points_ (i) = aux;
      }

      std::getline (ins,line);
      std::getline (ins,line);


      while (true)
      {
        std::getline (ins,line);

        if (line == "")
        {
          break;
        }

        std::istringstream iss (line);

        pcl::Vertices vertice_vector;

        while (iss >> vertice)
        {
          vertice_vector.vertices.push_back (vertice);
        }

        model_mesh_.push_back (vertice_vector);

      }


      ins >> rows;

      eigenvalues_vector_.resize (rows);


      for (i = 0; i < rows; ++i)
      {
        ins >> eigenvalues_vector_ (i);
      }

      std::getline (ins,line);


      ins >> rows >> cols;

      eigenvectors_matrix_.resize (rows,cols);



      for (j = 0; j < cols; ++j)
      {
        for (i = 0; i < rows; ++i)
        {
          ins >> eigenvectors_matrix_ (i,j);
        }
      }

    }

    else
    {
      PCL_ERROR ("Unknown file type\n");
      exit (1);
    }
  }

  else
  {
    PCL_ERROR ("Could not find database\n");
    exit (1);
  }


  debug_model_mesh_ = model_mesh_;

  for (i = 0; i < debug_model_mesh_.size (); ++i)
  {
    for (j = 0; j < debug_model_mesh_[i].vertices.size (); ++j)
    {
      debug_model_mesh_[i].vertices[j] = debug_model_mesh_[i].vertices[j] - 1;
    }
  }

  convertEigenToPointCLoud ();



  uint32_t rgb;
  uint8_t value (255);

  rgb = ( (uint32_t)value);

  calculateModelCenterPoint ();


  PCL_INFO ("Done with reading the statistical model\n");

}

void
Registration::calculateModelCenterPoint ()
{

  model_center_point_ = Eigen::Vector3d::Zero ();
  int i;

  for ( i = 0; i < iteration_source_point_normal_cloud_ptr_->points.size (); ++i)
  {
    model_center_point_ += iteration_source_point_normal_cloud_ptr_->points[i].getVector3fMap ().cast<double> ();
  }

  model_center_point_ /= static_cast<double> (iteration_source_point_normal_cloud_ptr_->points.size ());

}


void
Registration::getTargetPointCloudFromCamera (int device, std::string file_classifier)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  std::pair<int,int> center_coordinates;


  CameraGrabber camera;

  camera.setCamera (device,file_classifier);

  pcl::copyPointCloud (* (camera.getPointCloud (center_coordinates)),*target_point_cloud_ptr);

  setKdTree (target_point_cloud_ptr);

  face_center_point_.x = target_point_normal_cloud_ptr_->at (center_coordinates.first,center_coordinates.second).x;
  face_center_point_.y = target_point_normal_cloud_ptr_->at (center_coordinates.first,center_coordinates.second).y;
  face_center_point_.z = target_point_normal_cloud_ptr_->at (center_coordinates.first,center_coordinates.second).z;



  PCL_INFO ("Done with camera scanning\n");

}

void
Registration::getTargetPointCloudFromFile (std::string pcd_file)
{

  std::pair<int,int> center_coordinates;

  center_coordinates.first = 335;
  center_coordinates.second = 191;

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *target_point_cloud_ptr) == -1)
  {
    PCL_ERROR ("Could not open file %s\n", pcd_file.c_str ());
    exit (1);
  }


  face_center_point_.x = target_point_normal_cloud_ptr_->at (center_coordinates.first,center_coordinates.second).x;
  face_center_point_.y = target_point_normal_cloud_ptr_->at (center_coordinates.first,center_coordinates.second).y;
  face_center_point_.z = target_point_normal_cloud_ptr_->at (center_coordinates.first,center_coordinates.second).z;


  setKdTree (target_point_cloud_ptr);
}


void
Registration::alignModel ()
{

  Eigen::Vector3d translation = face_center_point_.getVector3fMap ().cast<double> () - model_center_point_;

  pcl::transformPointCloudWithNormals (*iteration_source_point_normal_cloud_ptr_,*iteration_source_point_normal_cloud_ptr_,translation,Eigen::Quaternion<double>::Identity ());

}

void
Registration::convertPointCloudToEigen ()
{
  int i,j=0;



  for (i = 0; i < iteration_source_point_normal_cloud_ptr_->size (); ++i)
  {
    eigen_source_points_[j++] = iteration_source_point_normal_cloud_ptr_->at (i).x;
    eigen_source_points_[j++] = iteration_source_point_normal_cloud_ptr_->at (i).y;
    eigen_source_points_[j++] = iteration_source_point_normal_cloud_ptr_->at (i).z;
  }

}


void
Registration::convertEigenToPointCLoud ()
{
   pcl::PointXYZRGBNormal pcl_point;
   int i,j,k;

   iteration_source_point_normal_cloud_ptr_->clear ();

   for (i = 0; i < eigen_source_points_.rows (); i = i+3)
   {
     pcl_point.x = eigen_source_points_[i];
     pcl_point.y = eigen_source_points_[i+1];
     pcl_point.z = eigen_source_points_[i+2];

     iteration_source_point_normal_cloud_ptr_->push_back (pcl_point);
   }


   std::vector < std::vector < Eigen::Vector3d > > normal_vector (iteration_source_point_normal_cloud_ptr_->points.size ());
   std::vector < std::vector < double > > surface_vector (iteration_source_point_normal_cloud_ptr_->points.size ());

   for ( k = 0; k < model_mesh_.size (); ++k)
   {

     Eigen::Vector3d eigen_vector_1,eigen_vector_2;

     int maximum_index = 0;
     int number_vertices = model_mesh_[k].vertices.size ();
     double angle, maximum_angle = 0.0;




     for ( i = model_mesh_[k].vertices.size (); i < model_mesh_[k].vertices.size () * 2; ++i)
     {
       eigen_vector_1 = iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ (i - 1) % number_vertices ] - 1].getVector3fMap ().cast<double> () - iteration_source_point_normal_cloud_ptr_->points[ model_mesh_[k].vertices[ i % number_vertices ] - 1].getVector3fMap ().cast<double> ();
       eigen_vector_2 = iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ (i + 1) % number_vertices ] - 1].getVector3fMap ().cast<double> () - iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ i % number_vertices ] - 1].getVector3fMap ().cast<double> ();

       eigen_vector_1.normalize ();
       eigen_vector_2.normalize ();

       angle = eigen_vector_1.dot (eigen_vector_2);

       angle = acos (angle);

       if (angle > maximum_angle)
       {
         maximum_angle = angle;
         maximum_index = i % number_vertices;
       }
     }

     Eigen::Vector3d edge,normal_1,normal_2;


     edge = iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ (maximum_index + number_vertices - 2) % number_vertices ] - 1].getVector3fMap ().cast<double> () - iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ maximum_index ] - 1].getVector3fMap ().cast<double> ();
     eigen_vector_1 = iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ (maximum_index + number_vertices - 1) % number_vertices ] - 1].getVector3fMap ().cast<double> () - iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ maximum_index ] - 1].getVector3fMap ().cast<double> ();
     eigen_vector_2 = iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ (maximum_index + number_vertices + 1) % number_vertices ] - 1].getVector3fMap ().cast<double> () - iteration_source_point_normal_cloud_ptr_->points[model_mesh_[k].vertices[ maximum_index ] - 1].getVector3fMap ().cast<double> ();

     normal_1 = edge.cross (eigen_vector_1);
     normal_2 = eigen_vector_2.cross (edge);

     double area_1,area_2;

     area_1 = normal_1.norm () * 0.5;
     area_2 = normal_2.norm () * 0.5;

     normal_1.normalize ();
     normal_2.normalize ();


     normal_vector [ model_mesh_[k].vertices [maximum_index] - 1 ].push_back (normal_1);
     normal_vector [ model_mesh_[k].vertices [maximum_index] - 1 ].push_back (normal_2);
     surface_vector[ model_mesh_[k].vertices [maximum_index] - 1 ].push_back (area_1);
     surface_vector[ model_mesh_[k].vertices [maximum_index] - 1 ].push_back (area_2);


     normal_vector [ model_mesh_[k].vertices [ (maximum_index + number_vertices - 2) % number_vertices] - 1 ].push_back (normal_1);
     normal_vector [ model_mesh_[k].vertices [ (maximum_index + number_vertices - 2) % number_vertices] - 1 ].push_back (normal_2);
     surface_vector[ model_mesh_[k].vertices [ (maximum_index + number_vertices - 2) % number_vertices] - 1 ].push_back (area_1);
     surface_vector[ model_mesh_[k].vertices [ (maximum_index + number_vertices - 2) % number_vertices] - 1 ].push_back (area_2);


     normal_vector [ model_mesh_[k].vertices [ (maximum_index + number_vertices - 1) % number_vertices] - 1 ].push_back (normal_1);
     surface_vector[ model_mesh_[k].vertices [ (maximum_index + number_vertices - 1) % number_vertices] - 1 ].push_back (area_1);

     normal_vector [ model_mesh_[k].vertices [ (maximum_index + number_vertices + 1) % number_vertices] - 1 ].push_back (normal_2);
     surface_vector[ model_mesh_[k].vertices [ (maximum_index + number_vertices + 1) % number_vertices] - 1 ].push_back (area_2);

   }


   for ( i = 0; i < iteration_source_point_normal_cloud_ptr_->points.size (); ++i)
   {
     double ratio = 0.0;
     Eigen::Vector3d normal_result = Eigen::Vector3d::Zero ();

     for ( j = 0; j < normal_vector[i].size (); ++j)
     {
       ratio += surface_vector[i][j];
     }

     for ( j = 0; j < normal_vector[i].size (); ++j)
     {
       normal_result += ( ( surface_vector[i][j] * normal_vector[i][j] ) );
     }


     normal_result /= ratio;
     normal_result.normalize ();



     iteration_source_point_normal_cloud_ptr_->points[i].normal_x = normal_result[0];
     iteration_source_point_normal_cloud_ptr_->points[i].normal_y = normal_result[1];
     iteration_source_point_normal_cloud_ptr_->points[i].normal_z = normal_result[2];


   }


   uint32_t rgb;
   uint8_t value (255);

   rgb = ( (uint32_t)value);

   for ( i = 0; i < iteration_source_point_normal_cloud_ptr_->points.size (); ++i)
   {
     iteration_source_point_normal_cloud_ptr_->points[i].rgb = *reinterpret_cast<float*> (&rgb);
   }

}

void
Registration::calculateRigidRegistration (int number_of_iterations, double angle_limit, double distance_limit, bool visualize)
{

  int i,j,k;


  Eigen::MatrixXd J (iteration_source_point_normal_cloud_ptr_->points.size (), 6);
  Eigen::VectorXd y (iteration_source_point_normal_cloud_ptr_->points.size ());


  Eigen::Matrix3d current_iteration_rotation = Eigen::Matrix3d::Identity ();
  Eigen::Vector3d current_iteration_translation;

  pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr current_iteration_source_points_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  pcl::copyPointCloud (*iteration_source_point_normal_cloud_ptr_, *current_iteration_source_points_ptr);

  if (visualize)
  {

    if (debug_mode_on_)
    {
      visualizer_ptr_->spin ();
    }

    else
    {
      visualizer_ptr_->spinOnce (500);
    }


    visualizer_ptr_->removeAllShapes ();
    visualizer_ptr_->removeAllPointClouds ();
    visualizer_ptr_->removeCorrespondences ();


  }

  Eigen::Matrix4f current_homogeneus_matrix;
  pcl::Correspondences iteration_correspondences;

  pcl::registration::DefaultConvergenceCriteria < float > convergence(j, current_homogeneus_matrix,iteration_correspondences);


  for (j = 0; j < number_of_iterations; ++j)
  {

    iteration_correspondences.clear ();

    k = 0;

    for (i = 0; i < current_iteration_source_points_ptr->size (); ++i)
    {

      pcl::PointXYZRGBNormal search_point;


      search_point = current_iteration_source_points_ptr->at (i);

      std::vector < int > point_index (1);
      std::vector < float > point_distance (1);


      kdtree_.nearestKSearch (search_point,1,point_index,point_distance);


      Eigen::Vector3d cross_product, normal,eigen_point;
      Eigen::Vector3d source_normal;

      double dot_product;

      source_normal = search_point.getNormalVector3fMap ().cast<double> ();


      normal[0] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_x;
      normal[1] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_y;
      normal[2] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_z;


      normal.normalize ();

      source_normal.normalize ();

      dot_product = source_normal.dot (normal);


      if ( point_distance[0] < distance_limit && std::acos (dot_product) < angle_limit)
      {

        Eigen::Vector3d aux_vector;

        aux_vector = (current_iteration_source_points_ptr->at (i).getVector3fMap ()).cast<double> ();

        cross_product = aux_vector.cross (normal);

        eigen_point = (target_point_normal_cloud_ptr_->at (point_index[0]).getVector3fMap ().cast<double> ()) - (search_point.getVector3fMap ().cast<double> ());

        pcl::Correspondence correspondence (i,point_index[0],point_distance[0]);

        iteration_correspondences.push_back (correspondence);

        y[k] = eigen_point.dot (normal);

        J (k,0) = cross_product[0];
        J (k,1) = cross_product[1];
        J (k,2) = cross_product[2];
        J (k,3) = normal[0];
        J (k,4) = normal[1];
        J (k,5) = normal[2];

        ++k;
      }


    }


    if (visualize)
    {
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_cloud_target (target_point_normal_cloud_ptr_);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_cloud_current_source (current_iteration_source_points_ptr);


      visualizer_ptr_->addPointCloud < pcl::PointXYZRGBNormal > (current_iteration_source_points_ptr, rgb_cloud_current_source, "source");
      //visualizer_ptr_->addPolygonMesh < pcl::PointXYZRGBNormal > (current_iteration_source_points_ptr,debug_model_mesh_);
      visualizer_ptr_->addPointCloud < pcl::PointXYZRGBNormal > (target_point_normal_cloud_ptr_, rgb_cloud_target, "scan");
      visualizer_ptr_->addCorrespondences <pcl::PointXYZRGBNormal> (current_iteration_source_points_ptr, target_point_normal_cloud_ptr_, iteration_correspondences);

      if(debug_mode_on_)
      {
        visualizer_ptr_->spin ();
      }

      else
      {
        visualizer_ptr_->spinOnce (500);
      }

      visualizer_ptr_->removeAllShapes ();
      visualizer_ptr_->removeAllPointClouds ();
      visualizer_ptr_->removeCorrespondences ();

    }



    Eigen::MatrixXd J_transpose,JJ, J_filtered (k,6);

    Eigen::VectorXd solutions, y_filtered;

    J_filtered = J.block (0,0,k,6);

    y_filtered = y.block (0,0,k,1);



    J_transpose = J_filtered.transpose ();
    JJ = J_transpose * J_filtered;

    Eigen::MatrixXd right_side = J_transpose * y_filtered;



    solutions = JJ.colPivHouseholderQr ().solve (right_side);


    current_iteration_rotation = Eigen::AngleAxisd (solutions[0],Eigen::Vector3d::UnitX ()) * Eigen::AngleAxisd (solutions[1],Eigen::Vector3d::UnitY ()) * Eigen::AngleAxisd (solutions[2],Eigen::Vector3d::UnitZ ()) ;


    for (i = 0; i < 3; ++i)
    {
      current_iteration_translation (i) = solutions (i+3);
    }





    current_homogeneus_matrix.block (0, 0, 3, 3) = current_iteration_rotation.cast < float > ();
    current_homogeneus_matrix.block (0, 3, 3, 1) = current_iteration_translation.cast < float > ();
    current_homogeneus_matrix.row (3) << 0, 0, 0, 1;

    pcl::PointCloud<pcl::PointXYZRGBNormal> result;

    pcl::transformPointCloudWithNormals (*current_iteration_source_points_ptr,result,current_homogeneus_matrix);

    pcl::copyPointCloud (result,*current_iteration_source_points_ptr);

    if( convergence.hasConverged () )
    {
      break;
    }




  }

  pcl::copyPointCloud (*current_iteration_source_points_ptr,*iteration_source_point_normal_cloud_ptr_);
  convertPointCloudToEigen ();

}

void
Registration::calculateNonRigidRegistration (int number_eigenvectors, double reg_weight, double angle_limit, double distance_limit, bool visualize)
{

  int i;

  pcl::Correspondences correspondences;

  correspondences = filterNonRigidCorrespondences (angle_limit,distance_limit);


  Eigen::MatrixXd J_transpose,JJ_total, J_point_to_point (correspondences.size () * 3,number_eigenvectors), Reg_diagonal_matrix;
  Eigen::VectorXd d,Jy,y (correspondences.size () * 3);

  Reg_diagonal_matrix = Eigen::MatrixXd::Identity (number_eigenvectors,number_eigenvectors);

  for (i = 0; i < Reg_diagonal_matrix.rows (); ++i)
  {
    Reg_diagonal_matrix (i,i) = 1.0 / eigenvalues_vector_[i];
  }



  for ( i = 0; i < correspondences.size (); ++i)
  {

    J_point_to_point.row (i * 3) = eigenvectors_matrix_.block (correspondences[i].index_query * 3,0,1,number_eigenvectors);
    J_point_to_point.row (i * 3 + 1) = eigenvectors_matrix_.block (correspondences[i].index_query * 3 + 1,0,1,number_eigenvectors);
    J_point_to_point.row (i * 3 + 2) = eigenvectors_matrix_.block (correspondences[i].index_query * 3 + 2,0,1,number_eigenvectors);


    y (i * 3) = target_point_normal_cloud_ptr_->at (correspondences[i].index_match).x - eigen_source_points_ (correspondences[i].index_query * 3);
    y (i * 3 + 1) = target_point_normal_cloud_ptr_->at (correspondences[i].index_match).y - eigen_source_points_ (correspondences[i].index_query * 3 + 1);
    y (i * 3 + 2) = target_point_normal_cloud_ptr_->at (correspondences[i].index_match).z - eigen_source_points_ (correspondences[i].index_query * 3 + 2);


  }



  J_transpose = J_point_to_point.transpose ();

  JJ_total = (J_transpose * J_point_to_point) + (Reg_diagonal_matrix * reg_weight);


  Jy = J_transpose * y;



  d = JJ_total.colPivHouseholderQr ().solve (Jy);

  eigen_source_points_ += eigenvectors_matrix_.block (0,0,eigenvectors_matrix_.rows (),number_eigenvectors) * d;

  convertEigenToPointCLoud ();

  if (visualize)
  {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_cloud_current_source (iteration_source_point_normal_cloud_ptr_);


    visualizer_ptr_->addPointCloud < pcl::PointXYZRGBNormal > (iteration_source_point_normal_cloud_ptr_, rgb_cloud_current_source, "model");


    if (debug_mode_on_)
    {
      visualizer_ptr_->spin ();
    }

    else
    {
      visualizer_ptr_->spinOnce (500);
    }


  }


}


void
Registration::calculateAlternativeRegistrations (int number_eigenvectors, double reg_weight, int number_of_total_iterations, int number_of_rigid_iterations, double angle_limit, double distance_limit, bool visualize)
{
  int i;

  if (visualize)
  {
    visualizer_ptr_.reset (new pcl::visualization::PCLVisualizer ("3D Visualizer"));
    visualizer_ptr_->setBackgroundColor (1, 1, 1);
    visualizer_ptr_->initCameraParameters ();
  }

  for ( i = 0; i < number_of_total_iterations; ++i)
  {

    calculateRigidRegistration (number_of_rigid_iterations,angle_limit,distance_limit,visualize);

    calculateNonRigidRegistration (number_eigenvectors,reg_weight,angle_limit,distance_limit,visualize);

  }
}

void
Registration::calculateKinfuTrackerRegistrations (int device, int number_eigenvectors, double reg_weight, int number_of_rigid_iterations, double angle_limit, double distance_limit)
{

  visualizer_ptr_.reset (new pcl::visualization::PCLVisualizer ("3D Visualizer"));
  visualizer_ptr_->setBackgroundColor (1, 1, 1);
  visualizer_ptr_->initCameraParameters ();
  visualizer_ptr_->registerKeyboardCallback <Registration> (&Registration::keyboardCallback, *this,(void*) this);

  tracker_ptr_.reset (new Tracker (device));
  tracker_ptr_->startUp ();

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr;
  target_point_cloud_ptr = tracker_ptr_->getKinfuCloud ();

  PCL_INFO("Press \' p \' to scan the first part of the cloud\n");

  while ( continue_tracking_ )
  {

    visualizer_ptr_->spinOnce (100);
    if ( tracker_ptr_->execute () )
    {

      if ( !first_face_found_ && !tracker_ptr_->isFaceFound () )
      {
        PCL_ERROR ("No faces were found\n");
        exit(1);
      }

      if( !first_face_found_ )
      {


        face_center_point_ = tracker_ptr_->getFaceCenter ();
        calculateModelCenterPoint ();
        alignModel ();

        first_face_found_ = true;
      }

      setKdTree (target_point_cloud_ptr);

      visualizer_ptr_->removeAllPointClouds();
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_target (target_point_normal_cloud_ptr_);
      visualizer_ptr_->addPointCloud < pcl::PointXYZRGBNormal > (target_point_normal_cloud_ptr_, rgb_target, "target");

      PCL_INFO("Press either \' p \' to scan the next part, \' t \' to start the transormation' or \' x \' to exit program\n");

    }

    if(calculate_)
    {
      calculate_ = false;

      calculateRigidRegistration (number_of_rigid_iterations,angle_limit,distance_limit,true);

      calculateNonRigidRegistration (number_eigenvectors,reg_weight,angle_limit,distance_limit,true);

      PCL_INFO("Press either \' p \' to scan the next part, \' t \' to start the transormation' or \' x \' to exit program\n");

    }
  }

  tracker_ptr_->close ();


}




void
Registration::writeDataToPCD (std::string file_path)
{

  pcl::PCDWriter pcd_writer;

  pcd_writer.writeBinary < pcl::PointXYZRGBNormal > (file_path + ".pcd", *iteration_source_point_normal_cloud_ptr_);
  pcd_writer.writeBinary < pcl::PointXYZRGBNormal > (file_path + "_target.pcd", *target_point_normal_cloud_ptr_);

}

void
Registration::setKdTree (pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr)
{

  pcl::PointCloud<pcl::Normal>::Ptr target_normal_cloud_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);


  normal_estimator.setInputCloud (target_point_cloud_ptr);
  tree->setInputCloud (target_point_cloud_ptr);
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setKSearch (10);
  //normal_estimator.setRadiusSearch (0.10);
  normal_estimator.compute (*target_normal_cloud_ptr);

  pcl::concatenateFields (*target_point_cloud_ptr, *target_normal_cloud_ptr, *target_point_normal_cloud_ptr_);

  kdtree_.setInputCloud (target_point_normal_cloud_ptr_);

  uint32_t rgb;
  uint8_t value (255);


  rgb = ( (uint32_t)value) << 8;


  for ( int i = 0; i < target_point_normal_cloud_ptr_->points.size (); ++i)
  {
      target_point_normal_cloud_ptr_->points[i].rgb = *reinterpret_cast<float*> (&rgb);
  }



}


pcl::Correspondences
Registration::filterNonRigidCorrespondences (double angle_limit, double distance_limit)
{
  int i;

  pcl::Correspondences correspondences_vector;



  for (i = 0; i < iteration_source_point_normal_cloud_ptr_->size (); ++i)
  {

    pcl::PointXYZRGBNormal search_point;


    search_point = iteration_source_point_normal_cloud_ptr_->at (i);

    std::vector < int > point_index (1);
    std::vector < float > point_distance (1);

    kdtree_.nearestKSearch (search_point,1,point_index,point_distance);



    Eigen::Vector3d cross_product, normal,eigen_point;
    Eigen::Vector3d source_normal;

    double dot_product;

    source_normal = search_point.getNormalVector3fMap ().cast<double> ();


    normal[0] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_x;
    normal[1] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_y;
    normal[2] = target_point_normal_cloud_ptr_->points[point_index[0]].normal_z;


    normal.normalize ();

    source_normal.normalize ();

    dot_product = source_normal.dot (normal);


    if ( point_distance[0] < distance_limit )
    {

      if ( std::acos (dot_product) < angle_limit )
      {
        Eigen::Vector3d aux_vector;

        aux_vector = (iteration_source_point_normal_cloud_ptr_->at (i).getVector3fMap ()).cast<double> ();

        cross_product = aux_vector.cross (normal);

        eigen_point = (target_point_normal_cloud_ptr_->at (point_index[0]).getVector3fMap ().cast<double> ()) - (search_point.getVector3fMap ().cast<double> ());

        pcl::Correspondence correspondence (i,point_index[0],point_distance[0]);

        correspondences_vector.push_back (correspondence);


      }
    }


  }

  return (correspondences_vector);
}


void
Registration::keyboardCallback (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{

  char c = event.getKeyCode ();

  if (c == 'p')
  {
    tracker_ptr_->setScan(true);
  }

  if (c == 't')
  {
    calculate_ = true;
  }

  if (c == 's' && debug_mode_on_)
  {
    visualizer_ptr_->saveScreenshot("ScreenShot" + boost::lexical_cast<std::string> ( index_ ) + ".png");
    ++index_;
  }

  if (c == 'x')
  {
    continue_tracking_ = false;
  }

  if (c == '1' && debug_mode_on_)
  {
    pcl::io::savePCDFile ("target_cloud_bin_" + boost::lexical_cast<std::string> (index_) + ".pcd", *target_point_normal_cloud_ptr_, true);
    ++index_;
  }

  if (c == '2' && debug_mode_on_)
  {
    pcl::io::savePCDFile ("model_cloud_bin_" + boost::lexical_cast<std::string> (index_) + ".pcd", *iteration_source_point_normal_cloud_ptr_, true);
    ++index_;
  }




}


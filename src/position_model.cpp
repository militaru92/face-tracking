#include <position_model.h>


void
PositionModel::readDataFromFolders (std::string path, int number_samples, int number_vertices, Eigen::Matrix3d transformation_matrix, Eigen::Vector3d translation)
{
  /* When I implemented this program I used the database in its original form
   * The folder of the database contained the meshes in paths of the form Tester_< number>/Blendshape/shape_0.obj
   *
   */
  std::string tester ("Tester_"),full_path,last_part ("/Blendshape/shape_0.obj"),line,value;
  size_t index;
  int i,j,k;
  number_faces_ = number_samples;

  for (i = 1; i <= number_samples; ++i)
  {
    full_path = path + tester + boost::lexical_cast<std::string> (i) + last_part;
    std::ifstream ins (full_path.c_str ());

    number_points_ = 11510;

    Eigen::VectorXd S (number_points_ * 3);

    if (ins.fail ())
    {
      PCL_ERROR ("Could not open file %s\n", full_path.c_str ());
      exit (1);
    }

    j = 0;

    while (1)
    {
      std::getline (ins,line);

      std::stringstream ss (line);
      std::string s;


      Eigen::Vector3d eigen_point;

      ss >> s;
      if (s.compare ("v") != 0)
        break;

      ss >> eigen_point[0];

      ss >> eigen_point[1];

      ss >> eigen_point[2];

      eigen_point = transformation_matrix * eigen_point;
      eigen_point = eigen_point + translation;

      S[j++] = eigen_point[0];
      S[j++] = eigen_point[1];
      S[j++] = eigen_point[2];


    }

    if (i == 1)
    {
      while (line[0] != 'f')
      {
        std::getline (ins,line);
      }

      while (line[0] == 'f' && !ins.eof ())
      {
        pcl::Vertices mesh;

        for (k = 0; k < number_vertices; ++k)
        {
          index = line.find (' ');
          line = line.substr (index+1);
          index = line.find ('/');
          value = line.substr (0,index);
          mesh.vertices.push_back (boost::lexical_cast<u_int32_t> (value));
        }

        meshes_.push_back (mesh);
        std::getline (ins,line);
      }
    }

    faces_position_cordiantes_.push_back (S);

  }



}


Eigen::VectorXd
PositionModel::calculateMeanFace (bool write)
{
  int i;

  mean_face_positions_ = Eigen::VectorXd::Zero (3 * number_points_);

  for (i = 0; i < number_faces_; ++i)
  {
    mean_face_positions_ += faces_position_cordiantes_[i];
  }

  mean_face_positions_ /= static_cast <double> (number_faces_);



  PCL_INFO ("Done with average face\n");

  if (!write)
    return mean_face_positions_;

  std::ofstream ofs;
  ofs.open ("PCA.txt", std::ofstream::out | std::ofstream::app);

  ofs << mean_face_positions_.rows () << std::endl;


  for (int i = 0; i < mean_face_positions_.rows (); ++i)
    ofs << mean_face_positions_ (i) << ' ';

  ofs << std::endl << std::endl;

  ofs.close ();


  return (mean_face_positions_);


}


void
PositionModel::calculateEigenValuesAndVectors ()
{
  Eigen::MatrixXd T (3 * number_points_, number_faces_), T_tT;
  int i,j;
  Eigen::VectorXd v;

  /* The idea is to calculate the eigenvectors in an optimal way
   * The covariance matrix is calculatd by C = T * T.transpose() and it has a size of 34530 x 34530.
   * To find the eigenvectors with fewer computations we can apply the fact that if we calculate an eigenvector of T.transpose() * T
   * and then we calculate T * eigenvector (T.transpose() * T), the eigenvector of C is obtined
   *
   *
   */

  for (i = 0; i < T.cols (); ++i)
  {
    T.col (i) = faces_position_cordiantes_[i] - mean_face_positions_;
  }

  T_tT = (T.transpose () * T) / static_cast<double> (number_faces_) ;

  Eigen::EigenSolver <Eigen::MatrixXd> solver (T_tT);

  Eigen::MatrixXd eigenvectors_matrix (T.rows (),solver.eigenvectors ().cols ());

  for (i = 0; i < solver.eigenvalues ().rows (); ++i)
  {
    eigenvalues_vector_.push_back ( (solver.eigenvalues ()[i]).real ());
  }

  for (i = 0; i < solver.eigenvectors ().cols (); ++i)
  {
    v = T * ( (solver.eigenvectors ().col (i)).real ());
    v.normalize ();
    eigenvectors_matrix.col (i) = v;
    eigenvectors_vector_.push_back (v);
  }

  eigenvectors_ = eigenvectors_matrix;
  eigenvalues_ = solver.eigenvalues ().real ().cast<double> ();


}


Eigen::VectorXd
PositionModel::getEigenValues (bool write)
{
  if (!write)
    return eigenvalues_;

  std::ofstream ofs;
  ofs.open ("PCA.txt", std::ofstream::out | std::ofstream::app);

  ofs << eigenvalues_.rows () << std::endl;


  for (int i = 0; i < eigenvalues_.rows (); ++i)
    ofs << eigenvalues_ (i) << ' ';

  ofs << std::endl << std::endl;

  ofs.close ();

  return (eigenvalues_);
}

Eigen::MatrixXd
PositionModel::getEigenVectors (bool write)
{
  if (!write)
    return eigenvectors_;

  std::ofstream ofs;
  ofs.open ("PCA.txt", std::ofstream::out | std::ofstream::app);

  ofs << eigenvectors_.rows () << ' ' << eigenvectors_.cols () << std::endl;

  for (int j = 0; j < eigenvectors_.cols (); ++j)
  {
    for (int i = 0; i < eigenvectors_.rows (); ++i)
    {
      ofs << eigenvectors_ (i,j) << ' ';
    }

    ofs << std::endl;
  }

  ofs << std::endl;

  ofs.close ();

  return (eigenvectors_);

}

std::vector <  pcl::Vertices >
PositionModel::getMeshes (bool write)
{
  if (!write)
    return meshes_;


  std::ofstream ofs;
  ofs.open ("PCA.txt", std::ofstream::out | std::ofstream::app);

  for (int i = 0; i < meshes_.size (); ++i)
  {
    for (int j = 0; j < meshes_[i].vertices.size (); ++j)
    {
      ofs << meshes_[i].vertices[j] << ' ';
    }

    ofs << std::endl;
  }

  ofs << std::endl;

  ofs.close ();

  return (meshes_);

}


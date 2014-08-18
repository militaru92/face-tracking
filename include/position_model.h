#ifndef POSITIONMODEL_HPP
#define POSITIONMODEL_HPP

#include <cstdlib>

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

class PositionModel
{

  public:

    /**
     * @brief Method to read the FaceWarehouse database stored in the default folders
     * @param [in] Number of faces in the database
     * @param [in] Number of points in a sample mesh
     * @param A 3*3 matrix which represents the initial scaling and rotation that have to applied on the samples
     * @param Initial translation to be applied on the samples
     */


    void
    readDataFromFolders (std::string, int number_samples, int number_vertices, Eigen::Matrix3d transformation_matrix, Eigen::Vector3d translation);


    /**
     * @brief Method to calculate the average face from the database
     * @param [in] Boolean value according to which the method writes the calculated data to a file or not
     * @return The mean face of the database
     */

    Eigen::VectorXd
    calculateMeanFace (bool write = false);

    /**
     * @brief Method to calculate the eigenvalues and eigenvectors of the database
     */

    void
    calculateEigenValuesAndVectors ();

    /**
     * @brief Method to get the vertex-indices of a mesh from the database
     * @param [in] Boolean value according to which the method writes the calculated data to a file or not
     * @return The indices of the mesh
     */


    std::vector <  pcl::Vertices >
    getMeshes (bool write = false);

    /**
     * @brief Method to get the eigenvalues from the database
     * @param [in] Boolean value according to which the method writes the calculated data to a file or not
     * @return An Eigen::Vector of eigenvalues
     */

    Eigen::VectorXd
    getEigenValues (bool write = false);

    /**
     * @brief Method to get the eigenvectors calculated in calculateEigenValuesAndVectors()
     * @param [in] Boolean value according to which the method writes the calculated data to a file or not
     * @return The matrix of the eigenvectors
     */

    Eigen::MatrixXd
    getEigenVectors (bool write = false);


  private:

    /**
     * @brief Number of vertices in a mesh
     */
    int number_points_;

    /**
     * @brief Number of faces in the database
     */
    int number_faces_;


    /**
     * @brief The indices of a sample mesh
     */


    std::vector <  pcl::Vertices > meshes_;

    /**
     * @brief A vector of column vectors in which the coordinates of face are stored
     */
    std::vector < Eigen::VectorXd > faces_position_cordiantes_;

    /**
     * @brief Column vector containing the coordinates of the average face
     */

    Eigen::VectorXd mean_face_positions_;

    /**
     * @brief eigenvalues_vector_
     */

    /**
     * @brief std::vector containing the eigenvalues
     */

    std::vector <double> eigenvalues_vector_;

    /**
     * @brief std::vector containing the eigenvectors
     */
    std::vector < Eigen::VectorXd > eigenvectors_vector_;

    /**
     * @brief Eigen container for the eigenvalues
     */

    Eigen::VectorXd eigenvalues_;

    /**
     * @brief Eigen container for the eigenvectors
     */
    Eigen::MatrixXd eigenvectors_;



};

#endif // POSITIONMODEL_HPP

#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "position_model.h"
#include "camera_grabber.h"
#include "tracker.h"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/default_convergence_criteria.h>


/**
 * @brief This class returns the shaped, statistical model
 */
class Registration
{
  public:

    Registration ();
    /**
     * @brief Deprecated method used for debugging in the first stages
     * @param source_points_path
     * @param target_points_path
     */

    void
    setDebugMode (bool debug_mode);

    /**
     * @brief Method for calculating the statistical model or for reading it from a file depending on where the database_path points to
     * @param [in] database_path Path to either the file that contains information about the model or to the directory that contains the Facewarehouse Database
     * @param [in] transformation_matrix Matrix used for bringing the statistical model to PCL scale
     * @param [in] translation Translation vector to be applied to the model.
     */

    void
    getDataFromModel (std::string database_path, Eigen::MatrixX3d transformation_matrix, Eigen::Vector3d translation);

    /**
     * @brief Method for a simple scanning of a face and for determining the coordinates of the face
     * @param [in] device OpenCV code for the type of device used for scanning. FOr example: CV_CAP_OPENNI_ASUS
     * @param [in] file_classifier Path to the XML file used by the OpenCV face detection function
     */

    void
    getTargetPointCloudFromCamera (int device, std::string file_classifier);

    /**
     * @brief Deprecated method to get the target point-cloud from a .pcd file
     * @param [in] pcd_file Path to the PCD file
     */

    void
    getTargetPointCloudFromFile (std::string pcd_file);

    /**
     * @brief Method to apply the Rigid Registration on the statistical model by operating on iteration_source_point_normal_cloud_ptr_
     * @param [in] Number of maximum iterations to apply
     * @param [in] The maximum allowed difference between the normals of two points to be considered correspondences
     * @param [in] The maximum distance between two points to be considered correspondences
     * @param [in] Boolean to set the PCLVisualizer to debug mode
     */

    void
    calculateRigidRegistration (int number_of_iterations, double angle_limit, double distance_limit, bool visualize);

    /**
     * @brief Method to apply the Non Rigid Registration on the model by operating on eigen_source_points_
     * @param [in] The number of the first eigenvectors to be used for this step
     * @param [in] The weight to which the Regulating Energy is multiplied by
     * @param [in] The maximum allowed difference between the normals of two points to be considered correspondences
     * @param [in] The maximum distance between two points to be considered correspondences
     * @param [in] Boolean to set the PCLVisualizer to debug mode
     */

    void
    calculateNonRigidRegistration (int number_eigenvectors, double reg_weight, double angle_limit, double distance_limit, bool visualize);

    /**
     * @brief Method to successively apply the two types of Registrations. It should be used if the target cloud is obtained by using getTargetPointCloudFromFile() or getTargetPointCloudFromCamera()
     * @param [in] Number of Eigenvectors to be used in the Non-Rigid Registration step
     * @param [in] Regularizing to be used in the Non-Rigid Registration step
     * @param [in] Number of times to apply the two registrations
     * @param [in] Number of iterations to apply the Rigid Registration
     * @param [in] The angle limit used for both Registration types
     * @param [in] The maximum distance to be used for both the registration steps
     * @param [in] Boolean to set the PCLVisualizer to debug mode
     */

    void
    calculateAlternativeRegistrations (int number_eigenvectors, double reg_weight, int number_of_total_iterations, int number_of_rigid_iterations, double angle_limit, double distance_limit, bool visualize = false);


    /**
     * @brief Method to scan the target point-cloud using the Kinfu Algorithm
     * @param [in] Number of Eigenvectors to be used in the Non-Rigid Registration step
     * @param [in] Regularizing to be used in the Non-Rigid Registration step
     * @param [in] Number of iterations to apply the Rigid Registration
     * @param [in] The angle limit used for both Registration types
     * @param [in] The maximum distance to be used for both the registration steps
     */
    void
    calculateKinfuTrackerRegistrations (int device, int number_eigenvectors, double reg_weight, int number_of_rigid_iterations, double angle_limit, double distance_limit);

    /**
     * @brief Method to bring the model close to the center of the face in the scan
     */

    void
    alignModel ();

    /**
     * @brief Method to write the output to .pcd
     * @param Path to the result file
     */

    void
    writeDataToPCD (std::string file_path);

    pcl::Correspondences
    filterNonRigidCorrespondences (double angle_limit, double distance_limit);

    /**
     * @brief Method to calculate an inside point of the model to be used as reference against the center of the face
     */

    void
    calculateModelCenterPoint ();



  private:

    /**
     * @brief Callback method for the visualizer
     */

    void
    keyboardCallback (const pcl::visualization::KeyboardEvent &event, void* viewer_void);


    /**
     * @brief Method to store the points from iteration_source_point_normal_cloud_ptr_ in eigen_source_points_
     */

    void
    convertPointCloudToEigen ();

    /**
     * @brief Method to store the points from eigen_source_points_ in iteration_source_point_normal_cloud_ptr_
     */

    void
    convertEigenToPointCLoud ();

    /**
     * @brief Method to create the kdtree of the scanned pointcloud and to calculate its normals
     * @param [in] The scaned pointcloud
     */

    void
    setKdTree (pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr);

    /**
     * @brief The kdtree for target_point_normal_cloud_ptr_ to be used for establishing the correspondences
     */

    pcl::search::KdTree<pcl::PointXYZRGBNormal> kdtree_;


    /**
     * @brief The scanned point cloud is stored in this data structure
     */


    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target_point_normal_cloud_ptr_;

    /**
     * @brief The statistical model is stored in this data structure for an optimal application of the Rigid Registration
     */
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr iteration_source_point_normal_cloud_ptr_;

    pcl::visualization::PCLVisualizer::Ptr visualizer_ptr_;

    /**
     * @brief The vertices of the mesh stored in obj format
     */

    std::vector <  pcl::Vertices > model_mesh_;

    /**
     * @brief The vertices of the mesh stored in pcl format
     */
    std::vector <  pcl::Vertices > debug_model_mesh_;


    /**
     * @brief The statistical model is stored in this data structure for an optimal application of the Non Rigid Registration
     */

    Eigen::VectorXd eigen_source_points_;

    /**
     * @brief The eigenvalues of the model are stored in this data structure
     */
    Eigen::VectorXd eigenvalues_vector_;

    /**
     * @brief The eigenvectors of the model are stored in this data structure
     */
    Eigen::MatrixXd eigenvectors_matrix_;

    /**
     * @brief The average point of the model is stored in this data structure
     */

    Eigen::Vector3d model_center_point_;

    /**
     * @brief The center of the face, detected with OpenCV, is stored in this structure
     */

    pcl::PointXYZ face_center_point_;

    /**
     * @brief Pointer to the Tracker object
     */

    boost::shared_ptr < Tracker > tracker_ptr_;

    /**
     * @brief Index used for debugging intermediate steps
     */

    int index_;

    /**
     * @brief Boolean value used in the KinfuTracker approach to determine when the program should finish
     */

    bool continue_tracking_;

    /**
     * @brief Boolean value used in the KinfuTracker approach to determine if a face was detected
     */


    bool first_face_found_;

    /**
     * @brief Boolean value to determine if the program should show the intermediate results or jst the final one
     */

    bool debug_mode_on_;

    bool calculate_;



};

#endif // REGISTRATION_H

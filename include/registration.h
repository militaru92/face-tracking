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
/*
    void
    readDataFromOBJFiles (std::string source_points_path, std::string target_points_path);


    void
    readOBJFile (std::string file_path, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, Eigen::Matrix3d transform_matrix, Eigen::Vector3d translation, bool advance = true, int number_vertices = 4);


    void
    readDataFromOBJFileAndPCDScan (std::string source_points_path, std::string target_points_path, Eigen::Matrix3d transform_matrix, Eigen::Vector3d translation);
*/
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
     * @brief Method to apply the Rigid Registration on the statistical model
     * @param [in] Number of maximum iterations to apply
     * @param [in] The maximum allowed difference between the normals of two points to be considered correspondences
     * @param [in] The maximum distance between two points to be considered correspondences
     * @param [in] Boolean to set the PCLVisualizer to debug mode
     */

    void
    calculateRigidRegistration (int number_of_iterations, double angle_limit, double distance_limit, bool visualize);

    /**
     * @brief Method to apply the Non Rigid Registration on the model
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


    void
    keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);


    void
    convertPointCloudToEigen ();

    void
    convertEigenToPointCLoud ();


    void
    setKdTree (pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_ptr);


    std::pair<int,int> center_coordinates_;


    pcl::search::KdTree<pcl::PointXYZRGBNormal> kdtree_;


    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr original_source_point_normal_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target_point_normal_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rigid_transformed_points_ptr_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr iteration_source_point_normal_cloud_ptr_;

    pcl::visualization::PCLVisualizer::Ptr visualizer_ptr_;

    std::vector <  pcl::Vertices > model_mesh_;
    std::vector <  pcl::Vertices > debug_model_mesh_;



    Eigen::VectorXd eigen_source_points_;
    Eigen::VectorXd eigenvalues_vector_;
    Eigen::MatrixXd eigenvectors_matrix_;

    Eigen::Vector3d model_center_point_;

    pcl::PointXYZ face_center_point_;

    boost::shared_ptr < Tracker > tracker_ptr_;

    int index_;

    bool continue_tracking_;
    bool first_face_found_;
    bool debug_mode_on_;



};

#endif // REGISTRATION_H

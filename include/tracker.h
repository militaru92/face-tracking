#ifndef TRACKER_H
#define TRACKER_H

#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



class Tracker : public pcl::OpenNIGrabber
{

  public:

    typedef boost::shared_ptr<openni_wrapper::DepthImage> DepthImagePtr;

    /**
     * @brief Tracker
     * @param [in] OpenCV code for the type of sensor to be used
     */

    Tracker (int device);


    ~Tracker () throw ();

    /**
     * @brief Method to return the accumulated point cloud
     * @return Pointer to the cloud in question
     */

    pcl::PointCloud <pcl::PointXYZ>::Ptr
    getKinfuCloud();

    /**
     * @brief Method to get the pcl::PointXYZ of the center of the face
     * @return
     */

    pcl::PointXYZ
    getFaceCenter ();

    /**
     * @brief Method to check if a face was found
     * @return True if a face was found, false otherwise
     */

    bool
    isFaceFound ();

    /**
     * @brief Set method for the scan_ attribute
     * @param scan
     */

    void
    setScan (bool scan);

    /**
     * @brief Method for starting the thread of the recording
     */

    void
    startUp ();

    /**
     * @brief Method to close the recording thread and for clean-up
     */

    void
    close ();

    /**
     * @brief Method to be called periodically to scan the point cloud at the right moment
     * @return Returns true if a point cloud has been just scanned
     */

    bool
    execute ();




  private:

    /**
     * @brief CallBack method to be run in the OpenNIGrabber and stored an ordered cloud component in cloud_depth_ptr_
     * @param [in]
     *
     */

    void
    source_cb1_device (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper);

    /**
     * @brief Method that determines the position of the face and stores the tracked point cloud in cloud_kinfu_ptr_
     */

    void
    takeKinfuCloud ();

    /**
     * @brief connection_
     */

    boost::signals2::connection connection_;

    /**
     * @brief OpenCV VideoCapture
     */

    cv::VideoCapture video_grabber_;

    /**
     * @brief OpenCV CascadeClassifier
     */

    cv::CascadeClassifier face_classifier_;

    /**
     * @brief KinfuTracker
     */
    pcl::gpu::KinfuTracker kinfu_;

    /**
     * @brief DepthMap to be updated in execute()
     */
    pcl::gpu::KinfuTracker::DepthMap depth_device_;

    /**
     * @brief Pointer to the cloud where the accumulated pointcloud is stored
     */

    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_kinfu_ptr_;

    /**
     * @brief Pointer to an ordered cloud which represents one phase of the scanning
     */
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_depth_ptr_;

    /**
     * @brief PCL point which represents the center of the face detected in the last scanning
     */

    pcl::PointXYZ face_center_;

    /**
     * @brief DeviceArray to get data from GPU
     */
    pcl::gpu::DeviceArray<pcl::PointXYZ> cloud_buffer_device_;


    /**
     * @brief Data structure used in the callback function
     */


    pcl::gpu::PtrStepSz<const unsigned short> depth_;

    boost::mutex data_ready_mutex_;
    boost::condition_variable data_ready_cond_;

    /**
     * @brief Data structure used in the callback function
     */


    std::vector<unsigned short> source_depth_data_;

    /**
     * @brief Translation vector used for the calibration of the KinfuTracker
     */

    Eigen::Vector3f translation_;

    boost::shared_ptr < boost::unique_lock < boost::mutex > > lock_ptr_;

    /**
     * @brief Boolean value that determines when a snapshot of the cloud should be taken
     */

    bool scan_;

    /**
     * @brief Boolean value to determine weather a face was detected
     */
    bool face_found_;



};

#endif // TRACKER_H

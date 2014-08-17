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
     * @brief CallBack method to be run in the OpenNIGrabber
     *
     */

    void
    source_cb1_device (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper);

    void
    takeKinfuCloud ();

    boost::signals2::connection connection_;

    cv::VideoCapture video_grabber_;
    cv::CascadeClassifier face_classifier_;

    pcl::gpu::KinfuTracker kinfu_;
    pcl::gpu::KinfuTracker::DepthMap depth_device_;

    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_kinfu_ptr_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_depth_ptr_;

    pcl::PointXYZ face_center_;
    pcl::gpu::DeviceArray<pcl::PointXYZ> cloud_buffer_device_;


    pcl::gpu::PtrStepSz<const unsigned short> depth_;

    boost::mutex data_ready_mutex_;
    boost::condition_variable data_ready_cond_;

    std::vector<unsigned short> source_depth_data_;

    Eigen::Vector3f translation_;

    boost::shared_ptr < boost::unique_lock < boost::mutex > > lock_ptr_;

    bool scan_;
    bool face_found_;



};

#endif // TRACKER_H

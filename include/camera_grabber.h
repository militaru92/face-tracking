#ifndef CAMERA_GRABBER_H
#define CAMERA_GRABBER_H

#include <pcl/common/common_headers.h>
#include <pcl/common/geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <cstdlib>

#include <opencv2/opencv.hpp>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @brief This class returns a simple point cloud and the coordinates of a face in the point cloud
 */
class CameraGrabber
{
  public:

    CameraGrabber ();

    /**
     * @brief Method for the
     * @param [in] OpenCV code for the type of sensor to be used
     * @param [in] path to the XML file containing the haarcascade file.
     */

    void
    setCamera (int device, std::string file_classifier);

    /**
     * @brief Method to return the point cloud and the coordinates in the point cloud for the center of the face
     * @param Reference for the std::pair in which the face coordinates are stored
     * @return Pointer to the cloud obtained from the camera
     */

    pcl::PointCloud <pcl::PointXYZRGB >::Ptr
    getPointCloud (std::pair < int, int >& center_coordinates);


  private:

    /**
     * @brief OpenCV VideoCapture
     */

    cv::VideoCapture video_grabber_;

    /**
     * @brief OpenCV CascadeClassifier
     */
    cv::CascadeClassifier face_classifier_;

    /**
     * @brief DataStrucutre in which the scaned pointcloud is stored
     */

    pcl::PointCloud <pcl::PointXYZRGB >::Ptr point_cloud_ptr_;

    /**
     * @brief Callback method for the OpenNIGrabber
     *
     */

    void
    getCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
};

#endif // CAMERA_GRABBER_H

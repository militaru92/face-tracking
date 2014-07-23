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

class CameraGrabber
{
  public:

    CameraGrabber();

    void
    setCamera(int device, std::string file_classifier);

    pcl::PointCloud <pcl::PointXYZRGB >::Ptr
    getPointCloud(std::pair < int, int >& center_coordinates);


  private:

    cv::VideoCapture video_grabber_;
    cv::CascadeClassifier face_classifier_;

    pcl::PointCloud <pcl::PointXYZRGB >::Ptr point_cloud_ptr_;

    pcl::visualization::PCLVisualizer::Ptr visualizer_ptr_;

    void
    getCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
};

#endif // CAMERA_GRABBER_H

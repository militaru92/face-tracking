#ifndef CAMERA_GRABBER_H
#define CAMERA_GRABBER_H

#include <pcl/common/common_headers.h>
#include <cstdlib>

#include "opencv2/opencv.hpp"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class CameraGrabber
{
  public:

    void
    runCamera(int device, std::string file_classifier, bool display = true);


  private:

    cv::VideoCapture video_grabber;
    cv::CascadeClassifier face_classifier;

    void
    display_frame(cv::Mat frame);
};

#endif // CAMERA_GRABBER_H

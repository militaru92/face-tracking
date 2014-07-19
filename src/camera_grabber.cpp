#include "camera_grabber.h"

CameraGrabber::CameraGrabber()
{
  point_cloud_ptr_.reset(new pcl::PointCloud <pcl::PointXYZRGB >);
  visualizer_ptr_.reset(new pcl::visualization::PCLVisualizer);
  visualizer_ptr_->setBackgroundColor(0, 0, 0);
  visualizer_ptr_->initCameraParameters();
}

void
CameraGrabber::runCamera(int device, std::string file_classifier,bool display)
{

  cv::Mat frame;
  int i;


  if( !video_grabber_.open(device) )
  {
    PCL_ERROR("Can't detect the camera\n");
    exit(-1);
  }

  if( !face_classifier_.load( file_classifier) )
  {
    PCL_ERROR("Did not find the XML file\n");
    exit(-1);
  }

  for(i = 0; i < 10; ++i)
  {
    video_grabber_.grab();

    video_grabber_.retrieve( frame, CV_CAP_OPENNI_GRAY_IMAGE );

  }

  writePCLPointCloud(frame);



}

void
CameraGrabber::getCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_ptr)
{
  int i;
  pcl::copyPointCloud(*cloud_ptr,*point_cloud_ptr_);

  uint32_t rgb;
  uint8_t value(255);

  rgb = ((uint32_t)value) <<16;

  for( i = 0; i < cloud_ptr->points.size(); ++i)
  {
    point_cloud_ptr_->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }


}

void
CameraGrabber::writePCLPointCloud(cv::Mat frame)
{
  std::vector<cv::Rect> faces;
  cv::Mat frame_gray;


  int i;

  pcl::OpenNIGrabber::Ptr openni_grabber(new pcl::OpenNIGrabber);

  boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> function_grabber =
    boost::bind (&CameraGrabber::getCloud, this, _1);

  openni_grabber->registerCallback (function_grabber);


  openni_grabber->start();

  for(i = 0; i < 3; ++i)
  {
    boost::this_thread::sleep (boost::posix_time::seconds (1));
  }

  openni_grabber->stop();




  visualizer_ptr_->addPointCloud <pcl::PointXYZRGB> (point_cloud_ptr_, "scan");

  cv::equalizeHist( frame, frame_gray);

  face_classifier_.detectMultiScale(frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

  for(i = 0; i < faces.size(); ++i)
  {
    pcl::PointXYZ center(static_cast<float>(faces[i].x + faces[i].width*0.5), static_cast<float>(faces[i].y + faces[i].height*0.5),0.0);
    visualizer_ptr_->addSphere(center,static_cast<float>(faces[i].height*0.5),0.0,0.5,0.5,"sphere"+boost::lexical_cast<std::string>(i));
    visualizer_ptr_->spin();
  }

}

#include <camera_grabber.h>

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

  writeMatToPointCloud(frame);



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
CameraGrabber::writeMatToPointCloud(cv::Mat frame)
{
  std::vector<cv::Rect> faces;
  cv::Mat frame_gray;

  int i,j,k;

  uint32_t rgb;
  uint8_t value(255);


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



  cv::equalizeHist( frame, frame_gray);

  face_classifier_.detectMultiScale(frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

  for(k = 0; k < faces.size(); ++k)
  {
    pcl::PointXYZRGB center;

    center = point_cloud_ptr_->at(faces[k].x + faces[k].width/2, faces[k].y + faces[k].height/2);


    rgb = ((uint32_t)value);

    for(i = 0; i < point_cloud_ptr_->width; ++i)
    {
      for(j = 0; j < point_cloud_ptr_->height; ++j)
      {
        point_cloud_ptr_->at(i,j).rgb = *reinterpret_cast<float*>(&rgb);
      }
    }

    rgb = ((uint32_t)value) << 16;

    for( i = faces[k].x; i < faces[k].x + faces[k].width; ++i)
    {
      for( j = faces[k].y; j < faces[k].y + faces[k].height; ++j)
      {
        point_cloud_ptr_->at(i,j).rgb = *reinterpret_cast<float*>(&rgb);
      }
    }


    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_field(point_cloud_ptr_);

    visualizer_ptr_->addPointCloud <pcl::PointXYZRGB> (point_cloud_ptr_, rgb_field,"scan"+boost::lexical_cast<std::string>(k));
    visualizer_ptr_->spin();
  }

}

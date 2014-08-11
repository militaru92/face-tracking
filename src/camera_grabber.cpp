#include <camera_grabber.h>

CameraGrabber::CameraGrabber ()
{
  point_cloud_ptr_.reset (new pcl::PointCloud <pcl::PointXYZRGB >);
}

void
CameraGrabber::setCamera (int device, std::string file_classifier)
{

  if ( !video_grabber_.open (device) )
  {
    PCL_ERROR ("The camera is disconnected\n");
    exit (-1);
  }

  if ( !face_classifier_.load ( file_classifier ) )
  {
    PCL_ERROR ("Did not find the XML file\n");
    exit (-1);
  }


}

void
CameraGrabber::getCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_ptr)
{
  int i;
  pcl::copyPointCloud (*cloud_ptr,*point_cloud_ptr_);

  uint32_t rgb;
  uint8_t value (255);

  rgb = ( (uint32_t ) value) <<16;

  for ( i = 0; i < cloud_ptr->points.size (); ++i)
  {
    point_cloud_ptr_->points[i].rgb = *reinterpret_cast<float*> (&rgb);
  }


}

pcl::PointCloud <pcl::PointXYZRGB >::Ptr
CameraGrabber::getPointCloud (std::pair < int, int >& center_coordinates)
{
  std::vector<cv::Rect> faces;
  cv::Mat frame, frame_gray;

  int i,j;

  uint32_t rgb;
  uint8_t value (255);

  for (i = 0; i < 10; ++i)
  {
    video_grabber_.grab ();

    video_grabber_.retrieve ( frame, CV_CAP_OPENNI_GRAY_IMAGE );

  }


  pcl::OpenNIGrabber::Ptr openni_grabber (new pcl::OpenNIGrabber);

  boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> function_grabber =
    boost::bind (&CameraGrabber::getCloud, this, _1);

  openni_grabber->registerCallback (function_grabber);


  openni_grabber->start ();

  for (i = 0; i < 3; ++i)
  {
    boost::this_thread::sleep (boost::posix_time::seconds (1));
  }

  openni_grabber->stop ();



  cv::equalizeHist ( frame, frame_gray);

  face_classifier_.detectMultiScale (frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size (30, 30));

  if (faces.size () != 1)
  {
    PCL_ERROR ("No faces detected\n");
    exit (1);
  }

  pcl::PointXYZRGB center;

  center = point_cloud_ptr_->at (faces[0].x + faces[0].width/2, faces[0].y + faces[0].height/2);


  rgb = ( (uint32_t)value) << 8;

  for (i = 0; i < point_cloud_ptr_->width; ++i)
  {
    for (j = 0; j < point_cloud_ptr_->height; ++j)
    {
      point_cloud_ptr_->at (i,j).rgb = *reinterpret_cast<float*> (&rgb);
    }
  }

  rgb = ( (uint32_t)value) << 16;

  for ( i = faces[0].x; i < faces[0].x + faces[0].width; ++i)
  {
    for ( j = faces[0].y; j < faces[0].y + faces[0].height; ++j)
    {
      point_cloud_ptr_->at (i,j).rgb = *reinterpret_cast<float*> (&rgb);
    }
  }

  center_coordinates.first = faces[0].x + faces[0].width/2;
  center_coordinates.second = faces[0].y + faces[0].height/2;

  std::cout<<"Face is:" <<center_coordinates.first << " " << center_coordinates.second << std::endl;

  return (point_cloud_ptr_);

}

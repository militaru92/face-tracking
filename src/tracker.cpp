
#include <tracker.h>

Tracker::Tracker (int device)
{

  scan_ = false;
  face_found_ = false;

  cloud_kinfu_ptr_.reset ( new pcl::PointCloud<pcl::PointXYZ>);


  Eigen::Vector3f volume_size = Eigen::Vector3f::Constant  (1.2f);
  kinfu_.volume ().setSize  (volume_size);

  Eigen::Matrix3f R = Eigen::Matrix3f::Identity  ();
  translation_ = volume_size * 0.5f - Eigen::Vector3f  (0, 0, volume_size  (2) / 2 * 1.2f);


  Eigen::Affine3f pose = Eigen::Translation3f  (translation_) * Eigen::AngleAxisf  (R);

  kinfu_.setInitalCameraPose  (pose);
  kinfu_.setCameraMovementThreshold (0.001f);


  if ( !video_grabber_.open (device) )
  {
    PCL_ERROR ("The camera is disconnected\n");
    exit (-1);
  }

  if ( !face_classifier_.load ( "haarcascade_frontalface_alt.xml" ) )
  {
    PCL_ERROR ("Did not find the XML file\n");
    exit (-1);
  }

  boost::function<void  (const DepthImagePtr&)> func2_dev = boost::bind  (&Tracker::source_cb1_device, this, _1);
  connection_ = registerCallback  (func2_dev);

}

Tracker::~Tracker () throw  ()
{

}

pcl::PointCloud <pcl::PointXYZ>::Ptr
Tracker::getKinfuCloud ()
{
  return  (cloud_kinfu_ptr_);
}

pcl::PointXYZ
Tracker::getFaceCenter ()
{
  return  (face_center_);
}

bool
Tracker::isFaceFound ()
{
  return  (face_found_);
}

void
Tracker::setScan (bool scan)
{
  scan_ = scan;
}



void
Tracker::takeKinfuCloud ()
{

  /* OpenCV code to detect the center of the face */

  face_found_ = false;

  cv::Mat frame,frame_gray;
  std::vector<cv::Rect> faces;

  video_grabber_.grab ();

  video_grabber_.retrieve ( frame, CV_CAP_OPENNI_GRAY_IMAGE );

  cv::equalizeHist ( frame, frame_gray);

  face_classifier_.detectMultiScale (frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size (30, 30));


  std::pair < int, int > center_coordinates;

  if (faces.size () > 0)
  {
    face_found_ = true;
    center_coordinates.first = faces[0].x + faces[0].width/2;
    center_coordinates.second = faces[0].y + faces[0].height/2;

    face_center_.x = cloud_depth_ptr_->at (center_coordinates.first,center_coordinates.second).x + translation_[0];
    face_center_.y = cloud_depth_ptr_->at (center_coordinates.first,center_coordinates.second).y + translation_[1];
    face_center_.z = cloud_depth_ptr_->at (center_coordinates.first,center_coordinates.second).z + translation_[2];

  }

  /* This part accumulates the point cloud in cloud_kinfu_ptr_ */


  pcl::gpu::DeviceArray<pcl::PointXYZ> extracted = kinfu_.volume ().fetchCloud  (cloud_buffer_device_);

  extracted.download  (cloud_kinfu_ptr_->points);

  cloud_kinfu_ptr_->width =  (int) cloud_kinfu_ptr_->points.size  ();
  cloud_kinfu_ptr_->height = 1;


}


void
Tracker::source_cb1_device (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper)
{

  boost::mutex::scoped_try_lock lock (data_ready_mutex_);

  if (!lock)
    return;


  depth_.cols = depth_wrapper->getWidth ();
  depth_.rows = depth_wrapper->getHeight ();
  depth_.step = depth_.cols * depth_.elemSize ();


  source_depth_data_.resize (depth_.cols * depth_.rows);
  depth_wrapper->fillDepthImageRaw (depth_.cols, depth_.rows, &source_depth_data_[0]);
  depth_.data = &source_depth_data_[0];

  cloud_depth_ptr_ = convertToXYZPointCloud (depth_wrapper);

  data_ready_cond_.notify_one ();
}

bool
Tracker::execute ()
{
  bool has_data = data_ready_cond_.timed_wait  (*lock_ptr_, boost::posix_time::millisec (100));

  if  (has_data)
  {
    depth_device_.upload (depth_.data, depth_.step, depth_.rows, depth_.cols);
    kinfu_ (depth_device_);

    if (scan_)
    {
      scan_ = false;
      takeKinfuCloud ();
      return (true);
    }

  }

  return (false);

}

void
Tracker::startUp ()
{
  lock_ptr_.reset ( new boost::unique_lock<boost::mutex> (data_ready_mutex_));
  start ();

}

void
Tracker::close ()
{
  stop ();
  connection_.disconnect ();
}

#include "camera_grabber.h"

void
CameraGrabber::runCamera(int device, std::string file_classifier,bool display)
{

  cv::Mat frame;
  bool flag = true;


  if( !video_grabber.open(device) )
  {
    std::cout<<"Can't detect the camera\n";
    exit(-1);
  }

  if( !face_classifier.load( file_classifier) )
  {
    std::cout<<"Did not find the XML file\n";
    exit(-1);
  }

  while(flag)
  {
    video_grabber.grab();

    video_grabber.retrieve( frame, CV_CAP_OPENNI_GRAY_IMAGE );

    display_frame(frame);

    int c = cv::waitKey(10);

    if((char)c == 'c')
    {
      break;
    }
  }

  if( !flag )
  {
    std::cout<<"The camera was disconnected\n";
    exit(-1);
  }



}

void
CameraGrabber::display_frame(cv::Mat frame)
{
  std::vector<cv::Rect> faces;
  cv::Mat frame_gray;

  int i;

  //cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
  cv::equalizeHist( frame, frame_gray);

    //-- Detect faces
  face_classifier.detectMultiScale(frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

  for(i = 0; i < faces.size(); ++i)
  {
    cv::Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);
    cv::ellipse(frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0);
  }

  cv::imshow("Face", frame);

}



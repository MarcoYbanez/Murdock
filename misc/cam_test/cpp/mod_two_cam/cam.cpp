#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>
using namespace cv;
using namespace std;


int main(int, char**)
{
  Mat frame1;
  Mat frame2;
  //--- INITIALIZE VIDEOCAPTURE
  VideoCapture cap1;
  VideoCapture cap2;
  // open the default camera using default API
  // cap.open(0);
  // OR advance usage: select any API backend
  int deviceID1 = 0;             // 0 = open default camera
  int deviceID2 = 1;             // 0 = open default camera
  int apiID = cv::CAP_ANY;      // 0 = autodetect default API
  // open selected camera using selected API
  cap1.open(deviceID1, apiID);
  cap2.open(deviceID2, apiID);

  // check if we succeeded
  if (!cap1.isOpened()) {
    cerr << "ERROR! Unable to open camera\n";
    return -1;
  }
  if (!cap2.isOpened()) {
    cerr << "ERROR! Unable to open camera\n";
    return -1;
  }

  cap1.set(CAP_PROP_CONVERT_RGB , false);
  cap2.set(CAP_PROP_CONVERT_RGB , false);


  std::cout <<"\t[PARAM_FRAME_WIDTH] ";
  if(!cap1.set(CAP_PROP_FRAME_WIDTH,320)){
    std::cout <<"SUCCESS\n";
  }else{
    std::cout <<"FAIL\n";
  }

  std::cout <<"\t[PARAM_FRAME_HEIGHT] ";
  if(!cap1.set(CAP_PROP_FRAME_HEIGHT,240)){
    std::cout <<"SUCCESS\n";
  }else{
    std::cout <<"FAIL\n";
  }
  std::cout <<"\t[PARAM_FPS] ";
  if(!cap1.set(CAP_PROP_FPS,30)){
    std::cout <<"SUCCESS\n";
  }else{
    std::cout <<"FAIL\n";
  }

  int nFPS = cap1.get(CAP_PROP_FPS);
  std::cout << "Loaded FPS : " << nFPS << "\n";

  std::cout <<"\t[PARAM_FRAME_WIDTH] ";
  if(!cap2.set(CAP_PROP_FRAME_WIDTH,320)){
    std::cout <<"SUCCESS\n";
  }else{
    std::cout <<"FAIL\n";
  }

  std::cout <<"\t[PARAM_FRAME_HEIGHT] ";
  if(!cap2.set(CAP_PROP_FRAME_HEIGHT,240)){
    std::cout <<"SUCCESS\n";
  }else{
    std::cout <<"FAIL\n";
  }
  std::cout <<"\t[PARAM_FPS] ";
  if(!cap2.set(CAP_PROP_FPS,30)){
    std::cout <<"SUCCESS\n";
  }else{
    std::cout <<"FAIL\n";
  }

  nFPS = cap2.get(CAP_PROP_FPS);
  std::cout << "Loaded FPS : " << nFPS << "\n";

  //--- GRAB AND WRITE LOOP
  cout << "Start grabbing" << endl<< "Press any key to terminate" << endl;

  for (;;)
  {
    // wait for a new frame from camera and store it into 'frame'
    cap1.read(frame1);
    cap2.read(frame2);
    // check if we succeeded
    if (frame1.empty()) {
      cerr << "ERROR! blank frame grabbed\n";
      break;
    }
    if (frame2.empty()) {
      cerr << "ERROR! blank frame grabbed\n";
      break;
    }
    // show live and wait for a key with timeout long enough to show images
    imshow("Camera 1", frame1);
    imshow("Camera 2", frame2);

    if (waitKey(5) >= 0)
      break;
  }

  return 0;
}


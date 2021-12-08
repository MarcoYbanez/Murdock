#include <stdio.h>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <pthread.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"

#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/core/cuda.hpp>

using namespace std;
using namespace cv;


#define MINE 1
#define LIVE_FEED 1
#define DISPARITY 0
#define OBJECT_DETECTION 1
#define OB_DEPTH 1
#define MONITOR_AND_EXPORT 1
#define PI 3.14159265 


Ptr<cuda::CascadeClassifier> body_cascade;

//Ptr<cuda::HOG> body_cascade;
cv::Mat LoadDatafromymlfile(std::string ymlfilename, std::string varriablestring);
void detectAndDisplay( Mat frame );

//std::vector<int> violations(24,0);
std::vector<int> violations(60,0);

int hour = 0;
int minute = 0;
bool report = true;
int violation_count = 0;
pthread_mutex_t export_l = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t violation_l = PTHREAD_MUTEX_INITIALIZER;

void export_violations();
void *monitor_and_export(void *);
void *violation_per_sec(void* );
std::string return_time(string str);

int main(){

  //hour = stoi(return_time("H"));
  //monitor_and_export(NULL);
  cv::Mat distortCoeff1 = LoadDatafromymlfile("parameters_v3/parameters.yaml", "distortionCoefficients1");
  cv::Mat distortCoeff2 = LoadDatafromymlfile("parameters_v3/parameters.yaml", "distortionCoefficients2");
  cv::Mat intrinsicMat1= LoadDatafromymlfile("parameters_v3/parameters.yaml", "intrinsicMatrix1");
  cv::Mat intrinsicMat2= LoadDatafromymlfile("parameters_v3/parameters.yaml", "intrinsicMatrix2");
  cv::Mat rot_cam2 = LoadDatafromymlfile("parameters_v3/parameters.yaml", "rotationOfCamera2");
  cv::Mat translation_cam2 = LoadDatafromymlfile("parameters_v3/parameters.yaml", "translationOfCamera2");
  export_violations();
#if MINE
  
#if OBJECT_DETECTION

  cuda::GpuMat left_frame_gpu, left_gray_gpu, left_facesBuf_gpu;
  cuda::GpuMat right_frame_gpu, right_gray_gpu, right_facesBuf_gpu;
  body_cascade = cuda::CascadeClassifier::create("./haarcascade_lowerbody.xml");
  //body_cascade = cuda::CascadeClassifier::create("./haarcascade_fullbody.xml");
  //body_cascade = cuda::CascadeClassifier::create("./haarcascade_frontalface_alt.xml");
  if(body_cascade->empty()){
    cerr << "DID NOT LOAD\n";
  }
  else{
    cerr << "LOADED CASCADE\n";
  }
  std::vector<Rect> right_faces, left_faces;

#endif

  cv::Mat frameR = cv::imread("./images/R/img10.jpg");
  cv::Mat frameL = cv::imread("./images/L/img10.jpg");
  
  cv::Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;
  cv::Matx31d trans(translation_cam2.at<float>(0,0),translation_cam2.at<float>(0,1), translation_cam2.at<float>(0,2));

  cv::Mat in1, in2, dist1, dist2, rot, tr;
  intrinsicMat1.convertTo(in1, CV_64F);
  distortCoeff1.convertTo(dist1, CV_64F);
  intrinsicMat2.convertTo(in2, CV_64F);
  distortCoeff2.convertTo(dist2, CV_64F);
  rot_cam2.convertTo(rot, CV_64F);
  translation_cam2.convertTo(tr, CV_64F);

  cv::stereoRectify(in1,
                    dist1,
                    in2,
                    dist2,
                    frameR.size(),
                    rot,
                    trans,
                    rect_l,
                    rect_r,
                    proj_mat_l,
                    proj_mat_r,
                    Q,
                    0
      );
   //alpha could be 0 or 1 read docs


  cv::Mat Left_Stereo_Map1, Left_Stereo_Map2, Right_Stereo_Map1, Right_Stereo_Map2;

  cv::initUndistortRectifyMap(in1,
                              dist1,
                              rect_l,
                              proj_mat_l,
                              frameR.size(),
                              CV_16SC2,
                              Left_Stereo_Map1,
                              Left_Stereo_Map2);

  cv::initUndistortRectifyMap(in2,
                              dist2,
                              rect_r,
                              proj_mat_r,
                              frameR.size(),
                              CV_16SC2,
                              Right_Stereo_Map1,
                              Right_Stereo_Map2);

  cv::Mat Left_nice, Right_nice;
  cv::remap(frameL,
            Left_nice,
            Left_Stereo_Map1,
            Left_Stereo_Map2,
            cv::INTER_LANCZOS4,
            cv::BORDER_CONSTANT,
            0);

  cv::remap(frameR,
            Right_nice,
            Right_Stereo_Map1,
            Right_Stereo_Map2,
            cv::INTER_LANCZOS4,
            cv::BORDER_CONSTANT,
            0);



  //cv::imshow("Left image after rectification",Left_nice);
  //cv::imshow("Right image after rectification",Right_nice);

  cerr << proj_mat_l << endl;
  cerr << proj_mat_r << endl;
  cerr << intrinsicMat1 << endl;
#if LIVE_FEED

  //cv::Mat Left_nice, Right_nice;
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

#if MONITOR_AND_EXPORT
  
  pthread_t mon_time;
  pthread_t record_violation;
  minute = stoi(return_time("S"));
  pthread_create(&mon_time, NULL, monitor_and_export, NULL );

#endif
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

    //cvtColor(frame1, g1, CV_BGR2GRAY);
    //cvtColor(frame2, g2, CV_BGR2GRAY);

    cv::remap(frame1,
              Left_nice,
              Left_Stereo_Map1,
              Left_Stereo_Map2,
              cv::INTER_LANCZOS4,
              cv::BORDER_CONSTANT,
              0);

    cv::remap(frame2,
              Right_nice,
              Right_Stereo_Map1,
              Right_Stereo_Map2,
              cv::INTER_LANCZOS4,
              cv::BORDER_CONSTANT,
              0);

#if OBJECT_DETECTION

  //detectAndDisplay(Left_nice);
  //detectAndDisplay(Right_nice);
    Q.convertTo(Q, CV_32F);
    
    left_frame_gpu.upload(Left_nice);
    right_frame_gpu.upload(Right_nice);

    cv::cuda::cvtColor(left_frame_gpu, left_gray_gpu, COLOR_BGR2GRAY);
    cv::cuda::cvtColor(right_frame_gpu, right_gray_gpu, COLOR_BGR2GRAY);

    body_cascade->setFindLargestObject(false);
    body_cascade->setScaleFactor(1.2);
    body_cascade->setMinNeighbors(4);
    body_cascade->detectMultiScale(left_gray_gpu, left_facesBuf_gpu);
    body_cascade->detectMultiScale(right_gray_gpu, right_facesBuf_gpu);
    body_cascade->convert(left_facesBuf_gpu, left_faces);
    body_cascade->convert(right_facesBuf_gpu, right_faces);
    //cout << left_faces << endl;
  
    double focal_length = proj_mat_l.at<double>(0,0) * (double)3.58 / 320;
    focal_length = focal_length/10;
    double B = 0.254; // in to m
    //double B = abs(proj_mat_r.at<double>(0,3));
    double disp2 = ((proj_mat_l.at<double>(0,2)) + proj_mat_r.at<double>(0,2));
 
    //parallel vectors of information per detected object
    vector<double> distances; 
    vector<double> horizontal_dist; 



    for ( size_t i = 0; i < left_faces.size(); i++ ) // process every detection found in left frame
    {
      Point zero( left_faces[i].x, left_faces[i].y );
      Point max( left_faces[i].x + left_faces[i].width, left_faces[i].y + left_faces[i].height );

      rectangle(Left_nice, zero, max, Scalar( 255, 128, 0 ) ,2);

      if(right_faces.size() != 0){
        
        Point maxR( right_faces[i].x +right_faces[i].width, right_faces[i].y + right_faces[i].height );
        Point zeroR( right_faces[i].x, right_faces[i].y );
        rectangle(Right_nice, zeroR, maxR, Scalar( 255, 128, 0 ) ,2);

        double disp = (left_faces[i].x+left_faces[i].width/2)-(right_faces[i].x+ right_faces[i].width/2);
        // Issue is the further back the disparity is too small - if(disp == 0) {disp =;} //
        double distance = focal_length * (sqrt((320*320)+(240*240)))* B / (disp);
        string distance_str = "Distance: " + to_string(fabs(distance));
        distances.push_back(distance); // could make data type a tuple of other needed values


        cerr << "\t\tDISTANCE: " << fabs(distance) <<endl;
        /*cerr << disp << endl;
        cerr << "\t\tx(pixel): " << left_faces[i].x+left_faces[i].width/2 << endl;
        cerr << "\t\tfocal len: " << focal_length << "   m: " << (sqrt((320*320)+(240*240))) << "      " << focal_length * (sqrt((320*320)+(240*240))) << endl;
        cerr << "x CORD: " << ((distance * (left_faces[i].x+left_faces[i].width/2)) / (focal_length * (sqrt((320*320)+(240*240))))) << endl;
        */
        double xx = left_faces[i].x+left_faces[i].width/2;
        double cx = 160.843452453613;
        //double cx = 165.52037;
        //double cx = 143.367923736572;
        double z = fabs(distance);
        double f_length = focal_length * (sqrt((320*320)+(240*240)));
        //double f_length = focal_length ;
        double x_REAL = z * (cx-xx) /f_length;
        

        //cerr << x_REAL/10 << endl;
        horizontal_dist.push_back(x_REAL/10);
      }
    }
  
    int violation_count = 0;

    for(int i = 0; i < (int)distances.size(); ++i){
      double theta_base;
      double base_hypo = distances[i];
      double base_x = horizontal_dist[i];



      //double c_squared = pow(distances[i], 2);
      //double a_squared = pow(horizontal_dist[i], 2);
      //double y_base = sqrt(c_squared - a_squared);
      
      try{
        theta_base = acos(base_x/base_hypo) * 180.0 / PI;
      }
      catch(...){
        theta_base = 361;
      }

      for(int j = i+1; j < (int)left_faces.size(); ++j){
        //let i left_faces[i] = box 1 to all other boxes - let j = others
        //calc distance between each objects on screen

        double theta_other;
        double other_hypo = distances[j];
        double other_x = horizontal_dist[j];
        try{
            theta_other= acos(other_x/other_hypo) * 180.0 / PI;
        }
        catch(...){
          theta_base = 361;
        }

        if(theta_base != 361 || theta_other != 361){
          double inner_theta = 180 - theta_base - theta_other;
          double distance_between = sqrt(pow(base_hypo, 2) + pow(other_hypo, 2) - (2*base_hypo*other_hypo*cos(inner_theta)));

          cerr << "BETWEEN:  "<<distance_between << endl;
          //if(distance_between < ){
            //++violation_count;
          //}


        }

        //could draw line between boxes too
      }
    }
    violation_count = 1;

    if(report){ 
      report = false;
      pthread_create(&record_violation, NULL, violation_per_sec, (void*)violation_count);
    }


#endif

    imshow("Left Camera",Left_nice);
    imshow("Right Camera", Right_nice);
    if (waitKey(5) >= 0){
      break;
    }
  }

#endif
#endif
  return 0;
}

cv::Mat LoadDatafromymlfile(std::string ymlfilename, std::string varriablestring)
{
    cv::Mat temp;
    cv::FileStorage fs(ymlfilename, cv::FileStorage::READ);
    fs[varriablestring] >> temp;
    fs.release();
    return temp;
}


void export_violations(){

  std::ofstream file("violations.csv", std::ios::trunc);
  
  //for(auto i = new_set_violations.begin(); i != new_set_violations.end(); ++i){
  for(int i = 0; i < (int)violations.size(); ++i){
    int begin = i;
    int end = i+1;
    if(end == 60){
      end = 0;
    }
    file << begin << ", " << end << ", " << violations[i] << std::endl;

  }

  file.close();
}

void *monitor_and_export(void* empt){
  
  string str;
  size_t pos;
  time_t rawtime;

  while(1){

    int cur = stoi(return_time("S"));
    int prev = minute; 
    
    if(cur != prev){
      minute = cur;
    }
    
    //cout << return_time("S") << endl;
    if(minute == 59){//update per minute

      pthread_mutex_lock(&export_l);
      cerr << "EXPORT\n";
      export_violations();
      pthread_mutex_unlock(&export_l);
      sleep(1);
    }
    if(stoi(return_time("H")) != hour){ //reset on hour
      pthread_mutex_lock(&violation_l);
      violations.clear();
      violations.resize(60, 0);
      pthread_mutex_unlock(&violation_l);
    }

  }



}
// https://stackoverflow.com/questions/24686846/get-current-time-in-milliseconds-or-hhmmssmmm-format

std::string return_time(string hr)
{
    using namespace std::chrono;

    // get current time
    auto now = system_clock::now();

    // get number of milliseconds for the current second
    // (remainder after division into seconds)
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    // convert to std::time_t in order to convert to std::tm (broken time)
    auto timer = system_clock::to_time_t(now);

    // convert to broken time
    std::tm bt = *std::localtime(&timer);

    std::ostringstream oss;

    if(hr == "H"){
      oss << std::put_time(&bt, "%H"); // HH:MM:SS

    }else if(hr == "M"){
      oss << std::put_time(&bt, "%M"); // HH:MM:SS
    }else if(hr == "S"){
      oss << std::put_time(&bt, "%S"); // HH:MM:SS

    }else{

      oss << std::put_time(&bt, "%H:%M::%S"); // HH:MM:SS
    }

    return oss.str();
}

void *violation_per_sec(void* violation_count){
    
  pthread_mutex_lock(&export_l);

  if(violations[minute] != 0){
    violations[minute] = 0;  
  }
  violations[minute] += 0;  

  pthread_mutex_unlock(&export_l);
  sleep(1);

  report = true;
}




/** @function detectAndDisplay */


/*
void detectAndDisplay( Mat frame )
{
    Mat frame_gray;
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
    std::vector<Rect> right_faces;
    std::vector<Rect> left_faces;
    //face_cascade.detectMultiScale( frame_gray, faces );

    for ( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4 );

        Mat faceROI = frame_gray( faces[i] );

        //-- In each face, detect eyes
        std::vector<Rect> eyes;
        eyes_cascade.detectMultiScale( faceROI, eyes );

        for ( size_t j = 0; j < eyes.size(); j++ )
        {
            Point eye_center( faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2 );
            int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
            circle( frame, eye_center, radius, Scalar( 255, 0, 0 ), 4 );
        }
    }

    //-- Show what you got
    imshow( "Capture - Face detection", frame );
}*/


#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "opencv2/imgcodecs.hpp"



cv::Mat LoadDatafromymlfile(std::string ymlfilename, std::string varriablestring);

#define MINE 1
#define LIVE_FEED 1
#define DISPARITY 1


#define LIVE_FEED_OTHER 0
#define DISPARITY_OTHER 0


using namespace std;
using namespace cv;

#if DISPARITY
// Defining callback functions for the trackbars to update parameter values

//https://learnopencv.com/depth-perception-using-stereo-camera-python-c/

  //cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create();
  cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
  int numDisparities = 1;
  int blockSize = 4;
  int preFilterType = 1;
  int preFilterSize = 1;
  int preFilterCap = 31;
  int minDisparity = 1;
  int textureThreshold = 10;
  int uniquenessRatio = 30;
  int speckleRange = 0;
  int speckleWindowSize = 0;
  int disp12MaxDiff = 1;
  int dispType = CV_16S;
  
  int P1 = 0;
  int P2 = 0;
static void on_trackbar1( int, void* )
{
  stereo->setNumDisparities(numDisparities*16);
  numDisparities = numDisparities*16;
  if(numDisparities == 0){
    numDisparities = 1;
  }
}

static void on_trackbar2( int, void* )
{
  stereo->setBlockSize(blockSize*2+5);
  blockSize = blockSize*2+5;
}

/*
static void on_trackbarP2( int, void* )
{
  stereo->setP2(P2);
}

static void on_trackbarP1( int, void* )
{
  stereo->setP1(P1);
}
*/

static void on_trackbar3( int, void* )
{
  stereo->setPreFilterType(preFilterType);
}

static void on_trackbar4( int, void* )
{
  stereo->setPreFilterSize(preFilterSize*2+5);
  preFilterSize = preFilterSize*2+5;
}

static void on_trackbar5( int, void* )
{
  stereo->setPreFilterCap(preFilterCap);
}

static void on_trackbar6( int, void* )
{
  stereo->setTextureThreshold(textureThreshold);
}

static void on_trackbar7( int, void* )
{
  stereo->setUniquenessRatio(uniquenessRatio);
}

static void on_trackbar8( int, void* )
{
  stereo->setSpeckleRange(speckleRange);
}

static void on_trackbar9( int, void* )
{
  stereo->setSpeckleWindowSize(speckleWindowSize*2);
  speckleWindowSize = speckleWindowSize*2;
}

static void on_trackbar10( int, void* )
{
  stereo->setDisp12MaxDiff(disp12MaxDiff);
}

static void on_trackbar11( int, void* )
{
  stereo->setMinDisparity(minDisparity);
}

#endif

int main(){

  cv::Mat distortCoeff1 = LoadDatafromymlfile("parameters_v2/distortionCoefficients1.yaml", "distortionCoefficients1");
  cv::Mat distortCoeff2 = LoadDatafromymlfile("parameters_v2/distortionCoefficients2.yaml", "distortionCoefficients2");
  cv::Mat intrinsicMat1= LoadDatafromymlfile("parameters_v2/intrinsicMatrix1.yaml", "intrinsicMatrix1");
  cv::Mat intrinsicMat2= LoadDatafromymlfile("parameters_v2/intrinsicMatrix2.yaml", "intrinsicMatrix2");
  cv::Mat rot_cam2 = LoadDatafromymlfile("parameters_v2/rotationOfCamera2.yaml", "rotationOfCamera2");
  cv::Mat translation_cam2 = LoadDatafromymlfile("parameters_v2/translationOfCamera2.yaml", "translationOfCamera2");
#if MINE
  
  /*
  std::cout << distortCoeff1 << std::endl;
  std::cout << distortCoeff2<< std::endl;
  std::cout << intrinsicMat1 << std::endl;
  std::cout << intrinsicMat2 << std::endl;
  std::cout << rot_cam2 << std::endl;
  std::cout << translation_cam2 << std::endl;

  */
  cv::Mat frameR = cv::imread("./images/R/img10.jpg");
  cv::Mat frameL = cv::imread("./images/L/img10.jpg");
  
  cv::Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;
  cv::Matx31d trans(translation_cam2.at<float>(0,0),translation_cam2.at<float>(0,1), translation_cam2.at<float>(0,2));
  std::cerr << trans << std::endl;

  //cv::Mat trans(Trns.cols, Trns.rows, Trns.type());
  //for(auto i = 0; i < 3; ++i){
    //translation_cam2.col(i).copyTo(Trns.row(i));
  //}
  //std::cerr << frameR.size(); 

    std::cerr << "StereoRectify\n";
    std::cerr << intrinsicMat1.type() << std::endl
                    <<"Dist 1:\n"
                    << distortCoeff1.type() << std::endl
                    <<"in 1:\n" 
                    << intrinsicMat2.type() << std::endl
                    <<"Dist 2:\n" 
                    << distortCoeff2.type() << std::endl
                    << frameR.size() << std::endl
                    <<"rot:\n" 
                    << rot_cam2.type() << std::endl
                    <<"trans:\n" 
                    << trans<< std::endl<<std::endl;
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
  std::cerr << "asdfasdfasfdasdfasdfasfd\n";
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
  cv::waitKey(0);

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
#if DISPARITY
    // initialize values for StereoSGBM parameters

  // Creating an object of StereoSGBM algorithm

  cv::Mat imgL;
  cv::Mat imgR;
  cv::Mat imgL_gray;
  cv::Mat imgR_gray;
// Creating a named window to be linked to the trackbars
  cv::namedWindow("disparity",cv::WINDOW_NORMAL);
  cv::resizeWindow("disparity",600,600);
  
  //stereo->setMinDisparity(minDisparity);
  //stereo->setTextureThreshold(textureThreshold);

  // Creating trackbars to dynamically update the StereoBM parameters
  cv::createTrackbar("numDisparities", "disparity", &numDisparities, 18, on_trackbar1);
  cv::createTrackbar("blockSize", "disparity", &blockSize, 50, on_trackbar2);
  cv::createTrackbar("preFilterType", "disparity", &preFilterType, 1, on_trackbar3);
  cv::createTrackbar("preFilterSize", "disparity", &preFilterSize, 25, on_trackbar4);
  cv::createTrackbar("preFilterCap", "disparity", &preFilterCap, 62, on_trackbar5);
  cv::createTrackbar("textureThreshold", "disparity", &textureThreshold, 100, on_trackbar6);
  cv::createTrackbar("uniquenessRatio", "disparity", &uniquenessRatio, 100, on_trackbar7);
  cv::createTrackbar("speckleRange", "disparity", &speckleRange, 100, on_trackbar8);
  cv::createTrackbar("speckleWindowSize", "disparity", &speckleWindowSize, 25, on_trackbar9);
  cv::createTrackbar("disp12MaxDiff", "disparity", &disp12MaxDiff, 25, on_trackbar10);
  cv::createTrackbar("minDisparity", "disparity", &minDisparity, 25, on_trackbar11);
  //cv::createTrackbar("P1", "disparity", &P1, 1000, on_trackbar3);
  //cv::createTrackbar("P2", "disparity", &P2, 1000, on_trackbar3);

  cv::Mat disp, disparity, g1, g2;

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

    cvtColor(frame1, g1, CV_BGR2GRAY);
    cvtColor(frame2, g2, CV_BGR2GRAY);

    cv::remap(g1,
              Left_nice,
              Left_Stereo_Map1,
              Left_Stereo_Map2,
              cv::INTER_LANCZOS4,
              cv::BORDER_CONSTANT,
              0);

    cv::remap(g2,
              Right_nice,
              Right_Stereo_Map1,
              Right_Stereo_Map2,
              cv::INTER_LANCZOS4,
              cv::BORDER_CONSTANT,
              0);
    imshow("Camera 1",  Left_nice);
    imshow("Camera 2",Right_nice);

#if DISPARITY

    stereo->compute(Left_nice, Right_nice, disp);
    disp.convertTo(disparity, CV_32F, 1.0);
    disparity = (disparity/16.0f-(float)minDisparity/(float)numDisparities);
  

  imshow("Disparity", disparity);

#endif

    //imshow("Camera 1",Left_nice);
    //imshow("Camera 2", Right_nice);
    if (waitKey(5) >= 0)
      break;
  }



#endif
#else
  // Defining the dimensions of checkerboard
  int CHECKERBOARD[2]{7,9}; 

  // Creating vector to store vectors of 3D points for each checkerboard image
  std::vector<std::vector<cv::Point3f> > objpoints;

  // Creating vector to store vectors of 2D points for each checkerboard image
  std::vector<std::vector<cv::Point2f> > imgpointsL, imgpointsR;

  // Defining the world coordinates for 3D points
  std::vector<cv::Point3f> objp;
  for(int i{0}; i<CHECKERBOARD[1]; i++)
  {
    for(int j{0}; j<CHECKERBOARD[0]; j++)
      objp.push_back(cv::Point3f(j,i,0));
  }

  // Extracting path of individual image stored in a given directory
  std::vector<cv::String> imagesL, imagesR;
  // Path of the folder containing checkerboard images
  cerr << "CRASH\n";
  //std::string pathL = "./images/snap_shot/L/*.jpg";
  //std::string pathR = "./images/snap_shot/R/*.jpg";
  //std::string pathL = "./images/L/*.jpg";
  //std::string pathR = "./images/R/*.jpg";
  cerr << "CRASH\n";

  cv::glob(pathL, imagesL);
  cv::glob(pathR, imagesR);

  cv::Mat frameL, frameR, grayL, grayR;
  // vector to store the pixel coordinates of detected checker board corners 
  std::vector<cv::Point2f> corner_ptsL, corner_ptsR;
  bool successL, successR;

  std::cout << "ASDFASD: " <<  imagesL.size() << std::endl;
  // Looping over all the images in the directory
  for(int i{0}; i<imagesL.size(); i++)
  {
    frameL = cv::imread(imagesL[i]);
    cv::cvtColor(frameL,grayL,cv::COLOR_BGR2GRAY);

    frameR = cv::imread(imagesR[i]);
    cv::cvtColor(frameR,grayR,cv::COLOR_BGR2GRAY);

    // Finding checker board corners
    // If desired number of corners are found in the image then success = true  
    successL = cv::findChessboardCorners(
      grayL,
      cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]),
      corner_ptsL);
      // cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

    successR = cv::findChessboardCorners(
      grayR,
      cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]),
      corner_ptsR);
      // cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
      /*
      * If desired number of corner are detected,
      * we refine the pixel coordinates and display 
      * them on the images of checker board*/
    if((successL) && (successR))
    {
      cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

      // refining pixel coordinates for given 2d points.
      cv::cornerSubPix(grayL,corner_ptsL,cv::Size(11,11), cv::Size(-1,-1),criteria);
      cv::cornerSubPix(grayR,corner_ptsR,cv::Size(11,11), cv::Size(-1,-1),criteria);

      // Displaying the detected corner points on the checker board
      cv::drawChessboardCorners(frameL, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_ptsL,successL);
      cv::drawChessboardCorners(frameR, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_ptsR,successR);

      objpoints.push_back(objp);
      imgpointsL.push_back(corner_ptsL);
      imgpointsR.push_back(corner_ptsR);
    }

    cv::imshow("ImageL",frameL);
    cv::imshow("ImageR",frameR);
    cv::waitKey(0);
  }

  //frameL = cv::imread("~/sample.jpeg");
  //frameR = cv::imread("~/sample1.jpeg");
  frameL = cv::imread("snap_shot/L/img1.jpg");
  frameR = cv::imread("snap_shot/R/img1.jpg");


  cv::destroyAllWindows();

  cv::Mat mtxL,distL,R_L,T_L;
  cv::Mat mtxR,distR,R_R,T_R;
  cv::Mat Rot, Trns, Emat, Fmat;
  cv::Mat new_mtxL, new_mtxR;

  // Calibrating left camera
  cv::calibrateCamera(objpoints,
                      imgpointsL,
                      grayL.size(),
                      mtxL,
                      distL,
                      R_L,
                      T_L);

  new_mtxL = cv::getOptimalNewCameraMatrix(mtxL,
                                distL,
                                grayL.size(),
                                1,
                                grayL.size(),
                                0);

  // Calibrating right camera
  cv::calibrateCamera(objpoints,
                      imgpointsR,
                      grayR.size(),
                      mtxR,
                      distR,
                      R_R,
                      T_R);

  new_mtxR = cv::getOptimalNewCameraMatrix(mtxR,
                                distR,
                                grayR.size(),
                                1,
                                grayR.size(),
                                0);
   
  int flag = 0;
  flag |= cv::CALIB_FIX_INTRINSIC;

  // This step is performed to transformation between the two cameras and calculate Essential and
  // Fundamenatl matrix
  cv::stereoCalibrate(objpoints,
                      imgpointsL,
                      imgpointsR,
                      new_mtxL,
                      distL,
                      new_mtxR,
                      distR,
                      grayR.size(),
                      Rot,
                      Trns,
                      Emat,
                      Fmat,
                      flag,
                      cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));
    cv::Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;


  // Once we know the transformation between the two cameras we can perform
  // stereo rectification
  

    std::cerr << "stereoRectify--\n";


  std::cerr << new_mtxL.type() << std::endl 
                    << "dust 1\n"
                    << distL.type() << std::endl
                    << "in 2\n"
                    << new_mtxR.type() << std::endl
                    << "dust 2\n"
                    << distR << std::endl
                    << grayR.size() << std::endl
                    << "Rot\n"
                    << Rot.type() << std::endl
                    << "trans\n"
                    << Trns.type() << std::endl << std::endl;
  cv::stereoRectify(new_mtxL,
                    distL,
                    new_mtxR,
                    distR,
                    grayR.size(),
                    Rot,
                    Trns,
                    rect_l,
                    rect_r,
                    proj_mat_l,
                    proj_mat_r,
                    Q,
                    1);
  cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
  cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;

  cv::initUndistortRectifyMap(new_mtxL,
                              distL,
                              rect_l,
                              proj_mat_l,
                              grayR.size(),
                              CV_16SC2,
                              Left_Stereo_Map1,
                              Left_Stereo_Map2);

  cv::initUndistortRectifyMap(new_mtxR,
                              distR,
                              rect_r,
                              proj_mat_r,
                              grayR.size(),
                              CV_16SC2,
                              Right_Stereo_Map1,
                              Right_Stereo_Map2);

/*
std::cerr << "intrinsicMat1\n";
std::cerr << intrinsicMat1<< std::endl ;
std::cerr << new_mtxL << std::endl << std::endl;


std::cerr << "intrinsicMat2\n";
std::cerr << intrinsicMat2<< std::endl ;
std::cerr << new_mtxR<< std::endl << std::endl;

std::cerr << "dist L\n";
std::cerr << distL << std::endl;
std::cerr << distortCoeff1<< std::endl << std::endl;

std::cerr << "dist R\n";
std::cerr << distR << std::endl;
std::cerr << distortCoeff2<< std::endl << std::endl;

std::cerr << "Rotation\n";
std::cerr <<  Rot << std::endl; 
std::cerr << rot_cam2 << std::endl << std::endl;


std::cerr << "Translation\n";
std::cerr <<  Trns << std::endl; 
std::cerr << translation_cam2<< std::endl << std::endl;

//cv::Mat trans(Trns.cols, Trns.rows, Trns.type());

//for(auto i = 0; i < 3; ++i){
  //translation_cam2.col(0).copyTo(trans.row(0));
//}

std::cerr << "Translation\n";
std::cerr <<  Trns << std::endl; 
std::cerr << translation_cam2<< std::endl << std::endl;
std::cerr << "\n\n";
std::cerr << Trns.rows <<std::endl;
std::cerr << Trns.cols<<std::endl;
std::cerr << translation_cam2.rows <<std::endl;
std::cerr << translation_cam2.cols<<std::endl;
std::cerr << Trns.channels() <<std::endl;
std::cerr << translation_cam2.channels()<<std::endl;
*/




  cv::FileStorage cv_file = cv::FileStorage("improved_params2_cpp.xml", cv::FileStorage::WRITE);
  cv_file.write("Left_Stereo_Map_x",Left_Stereo_Map1);
  cv_file.write("Left_Stereo_Map_y",Left_Stereo_Map2);
  cv_file.write("Right_Stereo_Map_x",Right_Stereo_Map1);
  cv_file.write("Right_Stereo_Map_y",Right_Stereo_Map2);

  cv::imshow("Left image before rectification",frameL);
  cv::imshow("Right image before rectification",frameR);

  cv::Mat Left_nice, Right_nice;

  // Apply the calculated maps for rectification and undistortion 
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

  //cv::waitKey(0);

  cv::Mat Left_nice_split[3], Right_nice_split[3];

  std::vector<cv::Mat> Anaglyph_channels;

  cv::split(Left_nice, Left_nice_split);
  cv::split(Right_nice, Right_nice_split);

  Anaglyph_channels.push_back(Right_nice_split[0]);
  Anaglyph_channels.push_back(Right_nice_split[1]);
  Anaglyph_channels.push_back(Left_nice_split[2]);

  cv::Mat Anaglyph_img;

  cv::merge(Anaglyph_channels, Anaglyph_img);

  //cv::imshow("Anaglyph image", Anaglyph_img);
  //cv::waitKey(0);

#endif

#if LIVE_FEED_OTHER

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
#if DISPARITY_OTHER
    // initialize values for StereoSGBM parameters

  // Creating an object of StereoSGBM algorithm

  cv::Mat imgL;
  cv::Mat imgR;
  cv::Mat imgL_gray;
  cv::Mat imgR_gray;
// Creating a named window to be linked to the trackbars
  cv::namedWindow("disparity",cv::WINDOW_NORMAL);
  cv::resizeWindow("disparity",600,600);
  
  //stereo->setMinDisparity(minDisparity);
  //stereo->setTextureThreshold(textureThreshold);

  // Creating trackbars to dynamically update the StereoBM parameters
  cv::createTrackbar("numDisparities", "disparity", &numDisparities, 18, on_trackbar1);
  cv::createTrackbar("blockSize", "disparity", &blockSize, 50, on_trackbar2);
  cv::createTrackbar("preFilterType", "disparity", &preFilterType, 1, on_trackbar3);
  cv::createTrackbar("preFilterSize", "disparity", &preFilterSize, 25, on_trackbar4);
  cv::createTrackbar("preFilterCap", "disparity", &preFilterCap, 62, on_trackbar5);
  cv::createTrackbar("textureThreshold", "disparity", &textureThreshold, 100, on_trackbar6);
  cv::createTrackbar("uniquenessRatio", "disparity", &uniquenessRatio, 100, on_trackbar7);
  cv::createTrackbar("speckleRange", "disparity", &speckleRange, 100, on_trackbar8);
  cv::createTrackbar("speckleWindowSize", "disparity", &speckleWindowSize, 25, on_trackbar9);
  cv::createTrackbar("disp12MaxDiff", "disparity", &disp12MaxDiff, 25, on_trackbar10);
  cv::createTrackbar("minDisparity", "disparity", &minDisparity, 25, on_trackbar11);

  cv::Mat disp, disparity, g1, g2;

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

    cvtColor(frame1, g1, CV_BGR2GRAY);
    cvtColor(frame2, g2, CV_BGR2GRAY);

    cv::remap(g1,
              Left_nice,
              Left_Stereo_Map1,
              Left_Stereo_Map2,
              cv::INTER_LANCZOS4,
              cv::BORDER_CONSTANT,
              0);

    cv::remap(g2,
              Right_nice,
              Right_Stereo_Map1,
              Right_Stereo_Map2,
              cv::INTER_LANCZOS4,
              cv::BORDER_CONSTANT,
              0);
    imshow("Camera 1",  Left_nice);
    imshow("Camera 2",Right_nice);

#if DISPARITY_OTHER

    stereo->compute(Left_nice, Right_nice, disp);
    disp.convertTo(disparity, CV_32F, 1.0);
    disparity = (disparity/16.0f-(float)minDisparity/(float)numDisparities);
  

  imshow("Disparity", disparity);

#endif




    //imshow("Camera 1",Left_nice);
    //imshow("Camera 2", Right_nice);
    if (waitKey(5) >= 0)
      break;
  }



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

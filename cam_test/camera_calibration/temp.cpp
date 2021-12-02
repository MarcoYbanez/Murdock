// Reading the left and right images.
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>


using namespace std;
using namespace cv;

int main(){
cv::Mat imgL,imgR;
imgL = cv::imread("../im0.jpg"); // path to left image is "../im0.png"
imgR = cv::imread("../im1.jpg"); // path to left image is "../im1.png"

// Setting parameters for StereoSGBM algorithm
int minDisparity = 0;
int numDisparities = 64;
int blockSize = 8;
int disp12MaxDiff = 1;
int uniquenessRatio = 10;
int speckleWindowSize = 10;
int speckleRange = 8;

if(imgL.empty() || imgR.empty()){
  std::cout << "EMPTY\n";
}

// Creating an object of StereoSGBM algorithm
cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(minDisparity,numDisparities,blockSize,
disp12MaxDiff,uniquenessRatio,speckleWindowSize,speckleRange);

// Calculating disparith using the StereoSGBM algorithm
cv::Mat disp;
stereo->compute(imgL,imgR,disp);

// Normalizing the disparity map for better visualisation 
cv::normalize(disp, disp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

// Displaying the disparity map
cv::imshow("disparity",disp);
cv::waitKey(0);


  return 0;
}

#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
/** @function main */
int main(int argc, char** argv)
{
Mat img_1 = imread("lena.jpg");
Mat img_2 = imread("lena.jpg");
//-- Step 1: Detect the keypoints using a FAST Detector
Ptr<FastFeatureDetector> detector = FastFeatureDetector::create();
std::vector<KeyPoint> keypoints_1, keypoints_2;
detector->detect(img_1, keypoints_1);
detector->detect(img_2, keypoints_2);
//-- Draw keypoints
Mat img_keypoints_1; Mat img_keypoints_2;
drawKeypoints(img_1, keypoints_1, img_keypoints_1, Scalar::all(-1),
DrawMatchesFlags::DEFAULT);
drawKeypoints(img_2, keypoints_2, img_keypoints_2, Scalar::all(-1),
DrawMatchesFlags::DEFAULT);
//-- Show detected (drawn) keypoints
imshow("Keypoints 1", img_keypoints_1);
imshow("Keypoints 2", img_keypoints_2);
waitKey();
return 0;
 }

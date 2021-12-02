#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6,9}; 

int main()
{
  // Creating vector to store vectors of 3D points for each checkerboard image
  std::vector<std::vector<cv::Point3f> > objpoints;

  // Creating vector to store vectors of 2D points for each checkerboard image
  std::vector<std::vector<cv::Point2f> > imgpoints;

  // Defining the world coordinates for 3D points
  std::vector<cv::Point3f> objp;
  for(int i{0}; i<CHECKERBOARD[1]; i++)
  {
    for(int j{0}; j<CHECKERBOARD[0]; j++)
      objp.push_back(cv::Point3f(j,i,0));
  }


  // Extracting path of individual image stored in a given directory
  std::vector<cv::String> images;

  // Path of the folder containing checkerboard images
  std::string path = "./images/R/*.jpg";

  try{
    cv::glob(path, images);

  }
  catch(...){
    std::cout << "Error: Caught error\n";
    return 1;
  }

  cv::Mat frame, gray;
  // vector to store the pixel coordinates of detected checker board corners 
  std::vector<cv::Point2f> corner_pts;
  bool success;

  // Looping over all the images in the directory
  for(int i{0}; i<images.size(); i++)
  {
    frame = cv::imread(images[i]);
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

    // Finding checker board corners
    // If desired number of corners are found in the image then success = true  
    success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    
    /* 
     * If desired number of corner are detected,
     * we refine the pixel coordinates and display 
     * them on the images of checker board
    */
    if(success)
    {
      cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
      
      // refining pixel coordinates for given 2d points.
      cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);
      
      // Displaying the detected corner points on the checker board
      cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
      
      objpoints.push_back(objp);
      imgpoints.push_back(corner_pts);
    }
    else{
      std::cout << "NO SUCCESS" << i << "\n" ;
    }

    cv::imshow("Image",frame);
    cv::waitKey(0);
  }

  cv::destroyAllWindows();

  cv::Mat cameraMatrix,distCoeffs,R,T;

  /*
   * Performing camera calibration by 
   * passing the value of known 3D points (objpoints)
   * and corresponding pixel coordinates of the 
   * detected corners (imgpoints)
  */
  cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);

  std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
  std::cout << "distCoeffs : " << distCoeffs << std::endl;
  std::cout << "Rotation vector : " << R << std::endl;
  std::cout << "Translation vector : " << T << std::endl;


// apply transformation

  frame = cv::imread("R/img10.jpg");


  cv::Mat dst, map1, map2,new_camera_matrix;
  cv::Size imageSize(cv::Size(frame.cols,frame.rows));

// Refining the camera matrix using parameters obtained by calibration
  new_camera_matrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);

// Method 1 to undistort the image
  cv::undistort( frame, dst, new_camera_matrix, distCoeffs, new_camera_matrix );

// Method 2 to undistort the image
  //cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),imageSize, CV_16SC2, map1, map2);

  cv::remap(frame, dst, map1, map2, cv::INTER_LINEAR);

//Displaying the undistorted image
  cv::imshow("undistorted image",dst);
  cv::waitKey(0);;

  return 0;
}

/*
cv::Mat dst, map1, map2,new_camera_matrix;
cv::Size imageSize(cv::Size(image.cols,image.rows));

// Refining the camera matrix using parameters obtained by calibration
new_camera_matrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);

// Method 1 to undistort the image
cv::undistort( frame, dst, new_camera_matrix, distCoeffs, new_camera_matrix );

// Method 2 to undistort the image
cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),imageSize, CV_16SC2, map1, map2);

cv::remap(frame, dst, map1, map2, cv::INTER_LINEAR);

//Displaying the undistorted image
cv::imshow("undistorted image",dst);
cv::waitKey(0);;

 *
 * */



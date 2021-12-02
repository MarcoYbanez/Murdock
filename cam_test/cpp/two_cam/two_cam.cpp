#include <iostream>
#include <stdio.h>



#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;
int main(){

	Mat frame;
	VideoCapture cap;

	cap.open(0);
	int deviceID = 0;
	int apiID = 0;


	cap.open(deviceID, apiID);

	if(!cap.isOpened()){
	
		std::cout << "Error: unable to open camera\n";
		return 1;
	
	}
	
	std::cout << "Start grabbing\n";


	while(1)
	{
		cap.read(frame);
		if(frame.empty()){
			std::cout << "Error reading frame\n";
			break;
		}
		imshow("Live", frame);
		if(waitKey(5) >= 0)
			break;

	}


	return 0;
}



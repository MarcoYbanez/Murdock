FLAGS=`pkg-config --libs opencv4`

all:
	g++ cam.cpp -o main -I /usr/local/include/opencv4 -L /usr/local/lib $(FLAGS) -lopencv_core -lopencv_highgui

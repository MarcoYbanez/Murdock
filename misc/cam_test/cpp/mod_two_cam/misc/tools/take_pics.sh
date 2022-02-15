#!/bin/bash

ffmpeg -f video4linux2 -s 320x240 -i /dev/video0 -frames 1 ../RIGHT_CAM/out.jpg
ffmpeg -f video4linux2 -s 320x240 -i /dev/video1 -frames 1 ../LEFT_CAM/out.jpg


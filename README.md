# Murdock

Murdock is a social distance high traffic monitoring system. It monitors a given area and records the number of violation per (sec/hour), and displays the busiest times. It's purpose was intended for businesses or areas of high activity so individuals can make the decision to go an alternate route or wait for a better time.


## Hardware 
  - 2x C270 Logitech Cameras
  - Nvidia Jetson Nano 2gb

## Software
  - OpenCV 4.5.0
  - CUDA 10.0
  - Jetpack 4.2.1

## How
  The main issue is determining the distance of a target from a camera. Depth cannot be accurately determined with a 2D picture frame. However, if another camera is introduced and the same target is identified in the second frame we could determine depth by triangulation and epipolar geometry just like how the human eyes determine distances of objects from ourselves.\
  Once the distance from the cameras to N number of objects are found, distances between each of the objects was determined using the law of cosine.
  

## Use

  cd Final\
  ./run.sh

  Notes:
  Here is a photo of my roommate and I running a test. This configuration read violations per sec, and reported by the minute for demonstration purposes.
  Mimicing human eyes, the left camera is the primary/dominant camera that displays important information while the right camera is used for triangulation 
  assistantance 
  
  ![alt text](https://github.com/MarcoYbanez/Murdock/blob/main/Photos/Murdock_demo.png?raw=true)
  
  <br>

  Camera setup\ 
  ![alt text](https://github.com/MarcoYbanez/Murdock/blob/main/Photos/system.jpg?raw=true)


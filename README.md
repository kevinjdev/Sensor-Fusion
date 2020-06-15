# Udacity-Sensor-Fusion
Completed Projects from Udacity's Sensor Fusion NanoDegree

## Getting Started
Clone the repository to your local machine by running the following command in a terminal window or command prompt from the location you wish to copy the folder

`git clone https://github.com/kevinjdev/Udacity-Sensor-Fusion.git`

## Lidar Obstacle Detection
In this project, I learned to process Lidar Point Cloud data to perform vehicle tracking with the following steps:
1. Segmenting the point cloud into two parts: Road and Obstacles using the plane segmentation algorithm Ransac.
2. Cluster the obstacle cloud based on the proximity of neighboring points using K-D trees and Euclidean Clustering.
3. Place Bounding Boxes around the clusters
4. Repeat process for each frame.

PCL has built in functions for Ransac and Euclidean Clustering, but I implemented the functions for these as part of this project.

### Prerequisites
* cmake, make, and gcc/g++ compiler installed.
* Point Cloud Library(pcl) >=1.6 installed. `https://github.com/PointCloudLibrary/pcl`
* pcl can be installed on MacOS using Homebrew `brew install pcl`. I used version 1.9.1

### How To Run
Create a build directory inside the project folder, compile with cmake and make, run the executable.  
```
mkdir build
cd build
cmake ..
make
./environment
```
## 3D Object Tracking
In this project, I learned about various keypoint detectors, descriptors, and matching techniques. Also, I was introduced to the YOLO deep-learning framework for object detection and learned how to associate regions in a camera image with 3D Lidar points. I implemented the following functionality:
1. Match 3D objects over time by using keypoint correspondences. 
2. Compute the TTC(time to collision) based on Lidar measurements. 
3. Compute the TTC using camera measurements by associating keypoint matches to regions of interest.

### Prerequisites 
* cmake, make, and gcc/g++ compiler installed.
* OpenCV >= 4.1 installed.
* MacOS Installation instructions: `https://docs.opencv.org/master/d0/db2/tutorial_macos_install.html`
* Note: Must be compiled from source. Ensure **opencv_contrib** is also downloaded. I used version 4.3.
* This is the command I used for cmake when compiling OpenCV: 
`cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.3.0/modules BUILD_DOCS=ON -DOPENCV_ENABLE_NONFREE=ON ../opencv-4.3.0`
* Once OpenCV is installed, the yolo weights file used for image classification needs to be downloaded for this project and placed in the **dat/yolo** folder by running the following command: `!wget "https://pjreddie.com/media/files/yolov3.weights"`
* Replace the line below in the **CmakeLists.txt** file with the path to your build folder for openCV
`set(OPENCV_DIR, /Users/kevinjaeger/Developer/build_opencv)`

### How to Run
Once the prerequisites are handled, you should be ready to build and run the project.
Create a build directory inside the project folder, compile with cmake and make, run the executable.
```
mkdir build
cd build
cmake ..
make
./3D_object_tracking
```
## Radar Target Generation and Detection
In this project, I wrote code to model a Frequency Modulated Continuous Wave Radar and perform signal processing for it using the following process:
1. Radar Configuration
2. Moving Target Generation
3. Signal Propagation
4. Processing Received Reflected Signal
5. Range/Doppler Fast Fourier Transform (FFT)
6. CFAR Detection

### Prerequisites
Matlab. I used version 2019b.

### How to Run
Open the script **Radar_Target_Generation_and_Detection.m** and run

## Unscented Kalman Filter
In this project, I implemented the code for an Unscented Kalman Filter in ukf.cpp. The UKF processing a 2 step process: 
1. Prediction Step
In the prediction step I performed the following tasks:
  a. Generate Sigma Points
  b. Predict Sigma Points
  c. Predict Mean and Covariance
2. Update Step
In the update step I performed the following tasks:
  a. Predict the sensor measurement
  b. Update the state

For this project the motion model used was the CTRV (constant turn rate and velocity magnitude model) leading to a 5 dimensional state vector (x_position,y_position,velocity,yaw_angle,yaw_rate).

Radar and Lidar measurements are processed through the Unscented Kalman Filter.

### Prerequisites
* cmake, make, and gcc/g++ compiler installed.
* Point Cloud Library (pcl) >= 1.2 installed. `https://github.com/PointCloudLibrary/pcl`
* pcl can be installed on MacOS using Homebrew `brew install pcl`. I used version 1.9.1.

### How to Run
Create a build directory inside the project folder, compile with cmake and make, run the executable.  
```
mkdir build
cd build
cmake ..
make
./ukf_highway
```

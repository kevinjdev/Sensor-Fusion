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
* Point Cloud Library installed. `https://github.com/PointCloudLibrary/pcl`
* pcl >= 1.6
* pcl can be installed on MacOS using Homebrew `brew install pcl`. Locally I am running version 1.9.1

### How To Run
create a build directory inside the project folder, call cmake, call make, run the executable.  
```
mkdir build
cd build
cmake ..
make
./environment
```


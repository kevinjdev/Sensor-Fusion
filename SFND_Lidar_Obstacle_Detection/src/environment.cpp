/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
// #include "./quiz/cluster/kdtree.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor->FilterCloud(inputCloud, 0.6, Eigen::Vector4f (-10, -6, -10, 1), Eigen::Vector4f ( 25, 6, 10, 1));
    pcl::PointCloud<pcl::PointXYZI>::Ptr vehicleRoof = pointProcessor->FilterRoof(inputCloud);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->MySegmentPlane(filterCloud, 100, .1);
    
    // renderPointCloud(viewer, segmentCloud.first, "ObstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(0,1,0));
    
    Box roof_box = pointProcessor->BoundingBox(vehicleRoof);
    renderBox(viewer, roof_box, INT_MAX, Color(0,1,1) );
    
    // Instantiate a kd tree with points to prepare for clustering
    mytree::KdTree* tree = new mytree::KdTree;

    // Convert the Inputcloud to vector<float> representing each point for KDtree insertion and cluster processing
    std::vector<std::vector<float>> points;
    for (int i = 0; i < segmentCloud.first->points.size(); i++)
	{
	 	std::vector<float> point;
	 	point.push_back(segmentCloud.first->points[i].x);
	 	point.push_back(segmentCloud.first->points[i].y);
	 	point.push_back(segmentCloud.first->points[i].z);
	 	point.push_back(segmentCloud.first->points[i].intensity);

        tree->insert(point, i);

        points.push_back(point);
    }
    // std::cout << "pushed back " << points.size() << " points to the kd tree" << std::endl;

    // Perform clustering on cloud
    std::vector<std::vector<int>> cloudClusterPoints = pointProcessor->euclideanCluster(points, tree, 0.8, 6, 200);
    // std::cout << "received back " << cloudClusterPoints.size() << " clusters from euclideanCuster function" << std::endl; 

    // Convert the 2d vector of cloud indexes to a vector of Cloud clusters
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    
    for (auto it = cloudClusterPoints.begin (); it != cloudClusterPoints.end (); ++it)
  	{
    	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    	for (auto pit = it->begin(); pit != it->end (); ++pit)
        {
      		cloud_cluster->points.push_back (segmentCloud.first->points[*pit]);
        }
    	cloud_cluster->width = cloud_cluster->points.size ();
    	cloud_cluster->height = 1;
    	cloud_cluster->is_dense = true;
      
      	clusters.push_back(cloud_cluster);
    }

    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, .8, 20, 300);

    // Display the cloud clusters
	int clusterId = 0;
	std::vector<Color> colors = {Color(0,0,0), Color(.5,0,.5), Color(1,1,0)};

	for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
	{
    	std::cout << "cluster size ";
    	pointProcessor->numPoints(cluster);
        Box box = pointProcessor->BoundingBox(cluster);
    	renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        renderBox(viewer, box, clusterId);
    	++clusterId;
	}

    delete tree;
    tree = NULL;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0);
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud, "InputCloud");

    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, .2);
    // renderPointCloud(viewer, segmentCloud.first, "ObstacleCloud", Color(1,0,0));
    // renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(0,1,0));

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

	for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
	{
    	std::cout << "cluster size ";
    	pointProcessor->numPoints(cluster);
        Box box = pointProcessor->BoundingBox(cluster);
    	renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        renderBox(viewer, box, clusterId);
    	++clusterId;
	}
  	
  	delete pointProcessor;
  	pointProcessor = NULL;
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
  	// cityBlock(viewer);
	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	std::vector<boost::filesystem::path> stream = 	pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
	auto streamIterator = stream.begin();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
  	
  	while (!viewer->wasStopped ())
	{

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
          streamIterator = stream.begin();

        viewer->spinOnce ();
	}
}
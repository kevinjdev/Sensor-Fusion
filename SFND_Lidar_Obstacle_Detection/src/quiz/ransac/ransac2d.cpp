/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	for (int iteration = 0; iteration < maxIterations; ++iteration)
	{
		// Randomly sample subset and fit line

		// Sample 2 random points from cloud
		int random_index1 = rand() % cloud->points.size();
		int random_index2 = rand() % cloud->points.size();

		pcl::PointXYZ point1 =  cloud->points[random_index1];
		pcl::PointXYZ point2 = cloud->points[random_index2];

		//Create equation of line Ax + By + C = 0
		float A = point1.y - point2.y;
		float B = point2.x - point1.x;
		float C = point1.x * point2.y - point2.x * point1.y;
		
		// Declare this to inliers for each sample
		std::unordered_set<int> inliersSample;
		
		// Measure distance between every point and fitted line
		for (int i = 0; i < cloud->points.size(); ++i)
		{
			float distance = (abs(A*cloud->points[i].x + B*cloud->points[i].y + C)) / sqrt(A*A + B*B);
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol)
			{
				inliersSample.insert(i);
			}
		}
		if (inliersSample.size() > inliersResult.size())
		{
			inliersResult = inliersSample;
		}
	}
	return inliersResult;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	for (int iteration = 0; iteration < maxIterations; ++iteration)
	{
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
		{
			inliers.insert(rand() % cloud->points.size());
		}
		// int random_index1 = rand() % cloud->points.size();
		// int random_index2 = rand() % cloud->points.size();
		// int random_index3 = rand() % cloud->points.size();
		// Randomly sample subset and fit plane
		std::vector<pcl::PointXYZ> points;
		auto it = inliers.begin(); 
		pcl::PointXYZ point1 = cloud->points[*it];
		it++;
		pcl::PointXYZ point2 = cloud->points[*it];
		it++;
		pcl::PointXYZ point3 = cloud->points[*it];
		
		// pcl::PointXYZ point1 = cloud->points[random_index1];
		// pcl::PointXYZ point2 = cloud->points[random_index2];
		// pcl::PointXYZ point3 = cloud->points[random_index3];
		// std::cout << "Iteration" << iteration << std::endl;
		// std::cout << "point1: " << point1.x << point1.y << point1.z << std::endl;
		// std::cout << "point2: " << point2.x << point2.y << point2.z << std::endl;
		// std::cout << "point3: " << point3.x << point3.y << point3.z << std::endl;

		// Create  equation of a plane from 3 points selected
		
		// Used to represent vector indices as coordinates
		enum {x, y, z};
		enum {i, j, k};

		// vector v1 (point1 to point 2)
		std::vector<float> v1(3);
		v1[x] = point2.x - point1.x;
		v1[y] = point2.y - point1.y;
		v1[z] = point2.z - point1.z;

		// vector v2 (point1 to point3)
		std::vector<float> v2(3);
		v2[x] = point3.x - point1.x;
		v2[y] = point3.y - point1.y;
		v2[z] = point3.z - point1.z;

		// Cross product of v1 and v2 using i,j,k notation
		std::vector<float> v3(3);
		v3[i] = v1[y] * v2[z] - v1[z] * v2[y];
		v3[j] = v1[z] * v2[x] - v1[x] * v2[z];
		v3[k] = v1[x] * v2[y] - v1[y] * v2[x];
		
		// Equation of a plane is Ax + By + Cz + D = 0
		// A = i, B = j, C = k, D = -(ix1 + jy1 + kz1)
		float A = v3[i];
		float B = v3[j];
		float C = v3[k];
		float D = -(v3[i] * point1.x + v3[j] * point1.y + v3[k] * point1.z);

		// Declare this to inliers for each sample
		// std::unordered_set<int> inliersSample;
		
		// Measure distance between every point and plane
		for (int i = 0; i < cloud->points.size(); ++i)
		{	
			// if this point is already an inlier then don't process
			if (inliers.count(i) > 0)
			{
				continue;
			}
			// Distance from a point to a plane
			// |A*x + B*y + C*z + D| / sqrt(A^2 + B^2 + C^2)
			float distance = (abs(A*cloud->points[i].x + B*cloud->points[i].y + C*cloud->points[i].z + D)) / sqrt(A*A + B*B + C*C);
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol)
			{
				// inliersSample.insert(i);
				inliers.insert(i);
			}
		}
		// if (inliersSample.size() > inliersResult.size())
		if (inliers.size() > inliersResult.size())
		{
			// inliersResult = inliersSample;
			inliersResult = inliers;
		}
	}
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, .5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data", Color (0,0,1));
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
}

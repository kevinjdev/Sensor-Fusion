// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include <random>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

  	// Voxel Filtering
  	typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
  	pcl::VoxelGrid<PointT> voxel_grid;
  	voxel_grid.setInputCloud (cloud);
  	voxel_grid.setLeafSize (filterRes, filterRes, filterRes);
  	voxel_grid.filter (*filtered_cloud);
	
  // Region Based Filtering
  	typename pcl::PointCloud<PointT>::Ptr region_cloud (new pcl::PointCloud<PointT>);
  	pcl::CropBox<PointT> cropBox(true);
  	cropBox.setMin(minPoint);
  	cropBox.setMax(maxPoint);
  	cropBox.setInputCloud(filtered_cloud);
  	cropBox.filter(*region_cloud);
	
	typename pcl::PointCloud<PointT>::Ptr roof_filtered_cloud = FilterRoof(region_cloud);
	// Remove the vehicle roof from the point cloud data
  	std::vector<int> indices;
	pcl::CropBox<PointT> roof(true);
  	roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
  	roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
  	roof.setInputCloud(region_cloud);
  	roof.filter(indices);
	
  	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  	for (int point: indices)
    {
    	inliers->indices.push_back(point);
    }
  	pcl::ExtractIndices<PointT> extract;
  	extract.setInputCloud(region_cloud);
  	extract.setIndices(inliers);
  	extract.setNegative(true);
  	extract.filter (*region_cloud);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return region_cloud;
	// return roof_filtered_cloud;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    // Generate plane cloud by adding inliers
    for (auto index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Subract inliers from point cloud to generate Obstacle Cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);
    

    // Store the plane cloud and obstacle cloud in a pair object and return
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // This code below is based on the pcl tutorial on planar segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setMaxIterations (maxIterations);
    seg.setInputCloud (cloud);

    // Perform segmentation
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setClusterTolerance (clusterTolerance); // 1 meter
  	ec.setMinClusterSize (minSize);
  	ec.setMaxClusterSize (maxSize);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud);
  	ec.extract (cluster_indices);
  
  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
    	typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
      		cloud_cluster->points.push_back (cloud->points[*pit]);
        }
    	cloud_cluster->width = cloud_cluster->points.size ();
    	cloud_cluster->height = 1;
    	cloud_cluster->is_dense = true;
      
      	clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterRoof(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        // Remove the vehicle roof from the point cloud data
        std::vector<int> roof_indices;
        pcl::CropBox<PointT> roof(true);
        roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
        roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
        roof.setInputCloud(cloud);
        roof.filter(roof_indices);
        
        typename pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);
        
        pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  	    
        for (int index = 0; index < roof_indices.size(); index++)
        {
            inliers->indices.push_back(roof_indices[index]);
        }

        for (auto index : inliers->indices)
        {
            filtered_cloud->points.push_back(cloud->points[index]);
        }

        return filtered_cloud;
    }

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int index, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& is_processed, mytree::KdTree* tree, float distanceTol)
{
	// Proximity(point,cluster):
	//     mark point as processed
	//     add point to cluster
	//     nearby points = tree(point)
	//     Iterate through each nearby point
	//         If point has not been processed
	// 
	is_processed[index] = true;
  	cluster.push_back(index);
  	// std::cout << "index " << index << "is processed" << is_processed[index] << std::endl;

  	std::vector<int> neighbors = tree->search(points[index], distanceTol);

	  for (int i: neighbors)
    {	
		// std::cout << "Neighbor" << neighbors[i] << "of " << points[index][0] << " " << points[index][1] << "is " << points[i][0] << ", " << points[i][1] << std::endl;
    	if (!is_processed[i])
        {
          	proximity(i, points, cluster, is_processed, tree, distanceTol);
        }
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, mytree::KdTree* tree, float distanceTol, int minSize, int maxSize)
{
    // 	EuclideanCluster():
    //     list of clusters 
    //     Iterate through each point
    //         If point has not been processed
    //             Create cluster
    //             Proximity(point, cluster)
    //             cluster add clusters
	std::vector<std::vector<int>> clusters;
  
  	std::vector<bool> is_processed(points.size(), false);

  	for (int i = 0; i < points.size(); i++)
    {
    	if (is_processed[i])
        {
        	continue;
        }
      
      	std::vector<int> cluster;
      	proximity(i, points, cluster, is_processed, tree, distanceTol);
		
		if (cluster.size() >= minSize && cluster.size() <= maxSize) // only add clusters that meet min and max size requirement
		{
      		clusters.push_back(cluster);
		}
    }
	return clusters;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	for (int iteration = 0; iteration < maxIterations; ++iteration)
	{
		std::unordered_set<int> inliers;

		// Generate 3 random points from cloud
		while (inliers.size() < 3)
		{
			inliers.insert(rand() % cloud->points.size());
		}

		auto it = inliers.begin(); 
		PointT point1 = cloud->points[*it];
		it++;
		PointT point2 = cloud->points[*it];
		it++;
		PointT point3 = cloud->points[*it];
		
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
		
		// Measure distance between every point and plane
		for (int i = 0; i < cloud->points.size(); ++i)
		{	
			// if this point is already an inlier (random point selected) then don't process
			if (inliers.count(i) > 0)
			{
				continue;
			}
			// Calculate Distance from the point to the plane
			// |A*x + B*y + C*z + D| / sqrt(A^2 + B^2 + C^2)
			float distance = (fabs(A*cloud->points[i].x + B*cloud->points[i].y + C*cloud->points[i].z + D)) / sqrt(A*A + B*B + C*C);
			
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceThreshold)
			{
				inliers.insert(i);
			}
		}

		// Check to see if this is the best fitting plane
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // convert inliersResult unordered set to PointIndices object for SeparateCloud function
    pcl::PointIndices::Ptr inliersPtIndices(new pcl::PointIndices);
    inliersPtIndices->indices.reserve(inliersResult.size());
    inliersPtIndices->indices.insert(inliersPtIndices->indices.end(), inliersResult.begin(), inliersResult.end());

    std::cout << "inliers size" << inliersPtIndices->indices.size() << std::endl;
    std::cout << "cloud size" << cloud->points.size() << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersPtIndices, cloud);

    return segResult;
}
// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() 
{
  m_filterRes = 0.25 ;
  m_filterMinPoint << -10, -5, -2, 1;
  m_filtermaxPoint << 25, 7, 2, 1;
  m_segMaxIterations = 50;
  m_segDistanceThreshold = 0.2;
  m_clusterTolerance = 2.0;
  m_clusterMinSize = 3;
  m_clusterMaxSize = 30;
}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
void ProcessPointClouds<PointT>::setPcdCfg(struct pcdCfg &pcdConfig)
{
  m_filterRes            = pcdConfig.filterRes;
  m_filterMinPoint      << pcdConfig.filterMinX, pcdConfig.filterMinY, pcdConfig.filterMinZ, 1;
  m_filtermaxPoint      << pcdConfig.filterMaxX, pcdConfig.filterMaxY, pcdConfig.filterMaxZ, 1;
  m_segMaxIterations     = pcdConfig.segMaxIterations;
  m_segDistanceThreshold = pcdConfig.segDistanceThreshold;
  m_clusterTolerance     = pcdConfig.clusterTolerance;
  m_clusterMinSize       = pcdConfig.clusterMinSize;
  m_clusterMaxSize       = pcdConfig.clusterMaxSize;

}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::
FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    pcl::VoxelGrid<PointT> vgrid;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    vgrid.setInputCloud(cloud);
    vgrid.setLeafSize(m_filterRes, m_filterRes, m_filterRes);
    vgrid.filter(*cloud_filtered);

    //create cloud region to crop area
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(m_filterMinPoint);
    region.setMax(m_filtermaxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point: indices) 
    {
      inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::
SeparateClouds(pcl::PointIndices::Ptr pInliers, typename pcl::PointCloud<PointT>::Ptr pCloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

  //create  two  newpoint  cloud  pointers,  one  for  obstacles,  and  one  for  road)
  typename pcl::PointCloud<PointT>::Ptr  pObstCloud (new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr  pPlanCloud (new pcl::PointCloud<PointT>(*pCloud, pInliers->indices));

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  // Extract the inliers and set cfg
  extract.setInputCloud (pCloud);
  extract.setIndices (pInliers);
  extract.setNegative (true);
  extract.filter (*pObstCloud);
  std::cerr << "PointCloud representing the planar component: " 
            << pPlanCloud->width * pPlanCloud->height << " data points." << std::endl;

  //return the output
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
  segResult(pObstCloud, pPlanCloud);

  return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::
SegmentPlane(typename pcl::PointCloud<PointT>::Ptr pCloud)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr pCoefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr pInliers (new pcl::PointIndices ());

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;

    // Optional segmentation cfg
    seg.setOptimizeCoefficients (true);

    // Mandatory segmentation cfg
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (m_segMaxIterations);
    seg.setDistanceThreshold (m_segDistanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (pCloud);
    seg.segment (*pInliers, *pCoefficients);
    if (pInliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    //separate the inliers from outliers
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult 
    = SeparateClouds(pInliers,pCloud);

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::
Clustering(typename pcl::PointCloud<PointT>::Ptr cloud)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (m_clusterTolerance); // 2cm
    ec.setMinClusterSize (m_clusterMinSize);
    ec.setMaxClusterSize (m_clusterMaxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back(cloud_cluster);
  }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() 
    << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::
Ransac(typename pcl::PointCloud<PointT>::Ptr cloud)
{
	
  typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr  cloudOutliers(new pcl::PointCloud<PointT>());
  int maxIterations = m_segMaxIterations;
  float distanceThreshold = m_segDistanceThreshold;

  std::unordered_set<int> inliersResult;
	srand(time(NULL));
	auto startTime = std::chrono::steady_clock::now();
	
	// TODO: Fill in this function

	//1) For max iterations
	while (maxIterations--)
	{
	    //2) Randomly sample subset by selecting three points and store them in the set
		std::unordered_set<int>inliers;
		while(inliers.size() < 3)
		{
			inliers.insert(rand()%(cloud->points.size()));
		}
		//create temp variables to hold points
		float x1,y1,z1,x2,y2,z2,x3,y3,z3;
		auto iter = inliers.begin();

		x1 = cloud->points[*iter].x;
		y1 = cloud->points[*iter].y;
		z1 = cloud->points[*iter].z;
		iter++;
		x2 = cloud->points[*iter].x;
		y2 = cloud->points[*iter].y;
		z2 = cloud->points[*iter].z;
		iter++;
		x3 = cloud->points[*iter].x;
		y3 = cloud->points[*iter].y;
		z3 = cloud->points[*iter].z;

		//create temp variables to hlod line coff
		float a,b,c,d;
		a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		d = - (a*x1 + b*y1 + c*z1);

	    //3) Measure distance between every point and fitted line
	    // If distance is smaller than threshold count it as inlier
		//filter out the already slected points
		for(int index = 0; index < cloud->points.size(); index++)
		{
			//check if the current index not equal to (x1,y1) or (x2,y2),(x3,y3) index in cloud points.
			if(inliers.count(index)>0)
			{
				continue;
			}

			//create temp variables to hold points of current index.
			float x,y,z,distance;
			PointT point = cloud->points[index];
			x = point.x;
			y = point.y;
			z = point.z;

			//calculate the distance between line and point.
			distance = fabs(a*x + b*y + c*z + d)/sqrt(a*a+b*b+c*c);

			//if the calculated distance less than the treshold , save the point.
			if(distance < distanceThreshold)
			{
				inliers.insert(index);
			}
		}

		//Save all points in the overall set of inliers.
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}		
	}

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	
	auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
   std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
  segResult(cloudOutliers,cloudInliers);

  return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::euclideanClusterHelper(std::vector<int> &cluster,
                                                        std::vector<bool> &porcessedPoints,
                                                        const std::vector<std::vector<float>>& points, 
                                                        KdTree* tree, float distanceTol,int id,int maxSize)
{
  if(!porcessedPoints[id] && cluster.size()< maxSize)
  {
    porcessedPoints[id] = true;
    cluster.push_back(id);

    std::vector<int> nearest = tree->search(points[id],distanceTol);

    for(int id : nearest)
    {
      if(!porcessedPoints[id])
      {
        euclideanClusterHelper(cluster, porcessedPoints, points, tree, distanceTol, id, maxSize);
      }
    } 
  }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>  ProcessPointClouds<PointT>::
euclideanCluster(typename pcl::PointCloud<PointT>::Ptr outliers)
{
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterClouds;
	KdTree* tree = new KdTree;
	std::vector<std::vector<float>> tree_points;
  float clusterTolerance = m_clusterTolerance;
  int minSize = m_clusterMinSize;
  int maxSize = m_clusterMaxSize;

	for (int i=0; i<outliers->points.size(); i++) 
	{
		PointT point = outliers->points[i];
		std::vector<float> tree_point = {point.x, point.y, point.z};
		tree->insert(tree_point,i);
		tree_points.push_back(tree_point);
	}

	//list of clusters 
	std::vector<std::vector<int>> clusters;
	std::vector<bool> porcessedPoints(outliers->points.size(),false);
	int counter =0;
	while(counter < outliers->points.size())
	{
		if(porcessedPoints[counter])
		{
			counter++;
			continue;
		}
		std::vector<int> cluster;
		euclideanClusterHelper(cluster, porcessedPoints, tree_points, tree, clusterTolerance, counter, maxSize);
		clusters.push_back(cluster);
	}

  for(std::vector<int> cluster : clusters)
  {
    typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
    for(int indice: cluster)
    {
      PointT point;
      point.x = tree_points[indice][0];
      point.y = tree_points[indice][1];
      point.z = tree_points[indice][2];
      clusterCloud->points.push_back(point);
    }
  
    if((cluster.size()>=minSize)&&cluster.size()<=maxSize)
    {
      clusterClouds.push_back(clusterCloud);
    }
    
  }

	return clusterClouds;

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

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
    boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
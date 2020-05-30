/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"


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
			pcl::PointXYZ point = cloud->points[index];
			x = point.x;
			y = point.y;
			z = point.z;

			//calculate the distance between line and point.
			distance = fabs(a*x + b*y + c*z + d)/sqrt(a*a+b*b+c*c);

			//if the calculated distance less than the treshold , save the point.
			if(distance < distanceTol)
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
	
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;
}

void euclideanClusterHelper(std::vector<int> &cluster,
                             std::vector<bool> &porcessedPoints,
                             const std::vector<std::vector<float>>& points, 
                             KdTree* tree, float distanceTol,int id)
{
	porcessedPoints[id] = true;
	cluster.push_back(id);

	std::vector<int> nearest = tree->search(points[id],distanceTol);

	for(int id : nearest)
	{
		if(!porcessedPoints[id])
		{
			euclideanClusterHelper(cluster, porcessedPoints, points, tree, distanceTol, id);
		}
	}
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr outliers,
                                                         float clusterTolerance)
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterClouds;
	KdTree* tree = new KdTree;
	std::vector<std::vector<float>> tree_points;

	for (int i=0; i<outliers->points.size(); i++) 
	{
		pcl::PointXYZ point = outliers->points[i];
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
		euclideanClusterHelper(cluster, porcessedPoints, tree_points, tree, clusterTolerance, counter);
		clusters.push_back(cluster);
	}

  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(tree_points[indice][0],
			                                             tree_points[indice][1],
														 tree_points[indice][2]));
		
  		clusterClouds.push_back(clusterCloud);
  	}

	return clusterClouds;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.3);

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

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters 
    = cluster(cloudOutliers, 2.0);



	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
		//Render the output of the cluster
		int clusterId = 0;
		std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
		for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
		{
			std::cout << "cluster size ";
			renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
			++clusterId;
		}
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}

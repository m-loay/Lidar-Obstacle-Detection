/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "_generated/LidarConfig.h"


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


#if defined(USE_PCL_CLUSTER) || defined(USE_SEG) || defined(USE_POINT_CLOUD) || defined(USE_RAY)
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
#ifdef USE_RAY
    bool renderScene = true;
#else
    bool renderScene = false;
#endif

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor , take a scan , render the  ray data.
    Lidar *p_lidar = new Lidar(cars,0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloud = p_lidar->scan();

#ifdef USE_RAY
    renderRays(viewer, p_lidar->position, pPointCloud);
#endif

#ifdef USE_POINT_CLOUD
    renderPointCloud(viewer, pPointCloud, "point cloud");
#endif

#if defined(USE_PCL_CLUSTER) || defined(USE_SEG)
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    //segment the plan
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud 
    = pointProcessor.SegmentPlane(pPointCloud);

    //render the ouput and view plan cloud points and obstacle cloud points
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
#endif

#ifdef USE_PCL_CLUSTER
    //cluster the obstacles point cloud 
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters 
    = pointProcessor.Clustering(segmentCloud.first);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    //Render the output of the cluster
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box boundingBoxI = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, boundingBoxI, clusterId);
        ++clusterId;
    }
#endif

}
#endif

#ifdef USE_PCL_FILTER
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud 
    = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    inputCloud = pointProcessorI->FilterCloud(inputCloud);

    renderPointCloud(viewer,inputCloud,"inputCloud");
}
#endif


#if defined(USE_FINAL_PROJECT) || defined(USE_FINAL_PROJECT_SELF)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    //filter the plan
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud =
    pointProcessorI->FilterCloud(inputCloud);

#ifdef USE_FINAL_PROJECT
    //segment the plan
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
    pointProcessorI->SegmentPlane(filterCloud);

    //cluster the obstacles point cloud 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
    pointProcessorI->Clustering(segmentCloud.first);

#else
    //segment the plan
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
    pointProcessorI->Ransac(filterCloud);

    //cluster the obstacles point cloud 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
    pointProcessorI->euclideanCluster(segmentCloud.first);
#endif

    //render the ouput and view plan cloud points and obstacle cloud points
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    //Render the output of the cluster
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        Box boundingBoxI = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, boundingBoxI, clusterId);
        ++clusterId;
    }
}
#endif


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


#if defined(USE_FINAL_PROJECT) || defined(USE_FINAL_PROJECT_SELF)
std::pair<std::string, struct pcdCfg>
 split(const std::string& s, char delimiter)
{
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   auto iter = tokens.begin();

   //get the path
   std::string path = *iter;
   iter++;

   //get pcd configuration
   struct pcdCfg pcdConfig;

   //1)Filter Resoultion 
   pcdConfig.filterRes = std::stod(*iter);
   iter++;

   //2)Filter Min Point
   pcdConfig.filterMinX = std::stod(*iter);
   iter++;
   pcdConfig.filterMinY = std::stod(*iter);
   iter++;
   pcdConfig.filterMinZ = std::stod(*iter);
   iter++;

   //3)Filter MAx Point
   pcdConfig.filterMaxX = std::stod(*iter);
   iter++;
   pcdConfig.filterMaxY = std::stod(*iter);
   iter++;
   pcdConfig.filterMaxZ = std::stod(*iter);
   iter++;

   //4)Segmentation parameters 
   pcdConfig.segDistanceThreshold = std::stod(*iter);
   iter++;
   pcdConfig.segMaxIterations = std::stoi(*iter);
   iter++;

   //5)Clustring parameters 
   pcdConfig.clusterTolerance = std::stod(*iter);
   iter++;
   pcdConfig.clusterMinSize = std::stoi(*iter);
   iter++;
   pcdConfig.clusterMaxSize = std::stoi(*iter);

   //return the output
   std::pair<std::string, struct pcdCfg> res(path,pcdConfig);
   return res;
}
#endif


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

#if defined(USE_FINAL_PROJECT) || defined(USE_FINAL_PROJECT_SELF)
    // std::string input = argv[1];
    std::string input = "../src/sensors/data/pcd/data_1,0.15,-20.0,-6.0,-2.0,30.0,7.0,5.0,0.2,50,0.3,10,800";
    std::cout<<"Input"<<input<<std::endl;
    std::pair<std::string, struct pcdCfg> res = split(input, ',');
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(res.first);
    pointProcessorI->setPcdCfg(res.second);
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
#else
    #if defined(USE_PCL_CLUSTER) || defined(USE_SEG) || defined(USE_POINT_CLOUD) || defined(USE_RAY)
        simpleHighway(viewer);
    #endif

    #ifdef USE_PCL_FILTER
        cityBlock(viewer);
    #endif
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce ();
        }
#endif 
}
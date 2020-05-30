// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "quiz/cluster/kdtree.h"
#include "render/box.h"

// Structure to represent node of kd tree
struct pcdCfg
{
    float filterRes ;
    float filterMinX;
    float filterMinY;
    float filterMinZ;
    float filterMaxX;
    float filterMaxY;
    float filterMaxZ;
    float segDistanceThreshold;
    float clusterTolerance;
    int   segMaxIterations;
    int   clusterMinSize;
    int   clusterMaxSize;
};

template<typename PointT>
class ProcessPointClouds 
{
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    void setPcdCfg(struct pcdCfg & pcdCgf);

    typename pcl::PointCloud<PointT>::Ptr 
    FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> 
    Clustering(typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    Ransac(typename pcl::PointCloud<PointT>::Ptr cloud);

    void euclideanClusterHelper(std::vector<int> &cluster,
                                std::vector<bool> &porcessedPoints,
                                const std::vector<std::vector<float>>& points, 
                                KdTree* tree, float distanceTol,int id,int maxSize);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> 
    euclideanCluster(typename pcl::PointCloud<PointT>::Ptr outliers);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
private:

    float m_filterRes ;
    Eigen::Vector4f m_filterMinPoint;
    Eigen::Vector4f m_filtermaxPoint;
    int m_segMaxIterations;
    float m_segDistanceThreshold;
    float m_clusterTolerance;
    int m_clusterMinSize;
    int m_clusterMaxSize;
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */
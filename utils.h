//
// Created by Lifan on 1/25/17.
//

#ifndef POINTS_GROUPING_UTILS_H
#define POINTS_GROUPING_UTILS_H


//#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>


// Eigens
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>



#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/search/kdtree.h>

#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <pcl-1.8/pcl/common/eigen.h>
#include <pcl-1.8/pcl/registration/transformation_validation.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

#include <dirent.h>


namespace utils{

    typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPointer;
    using namespace std;



    /**
     * Check if the width follows the rule of write before write to file.
     * @param dirPath
     * @param fileName
     * @param cloud
     */
    void pclPointCloudWriter(string &dirPath, string &fileName, CloudPointer cloud);


    /**
     * Calc point to plane distance. Given the coefficients of the plane
     * @param plane
     * @param point
     * @return
     */
    double calcPointPlaneDist(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ point);

    double calcPointPlaneDist(pcl::ModelCoefficients plane, pcl::PointXYZ point);


    /**
     * visualize cloud
     * @param first
     * @param msg
     */
    void visualizeCloud(CloudPointer first, const std::string msg = "");

    /**
     *
     * @param first
     * @param msg
     */
    void visualizeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr first, const std::string msg = "");

    /**
     * [important prior parameters]
     * @param curPoint
     * @return
     */
    bool checkWithPriorInfo(pcl::PointXYZ& curPoint);


    /**
     *
     * @param coefficients
     * @return
     */
    bool checkIfGroundPlaneCoeffi(pcl::ModelCoefficients::Ptr coefficients);


    /**
     * visualize point clouds pairwisely. Can be replaced with multiVisualizeCloud
     * @param first
     * @param second
     * @param msg
     */
    void pairwiseVisualizeCloud( CloudPointer first,  CloudPointer second, const std::string msg = "");



    /**
     *
     * @param vPath
     * @return
     */
    CloudPointer loadCloudFromPath(std::string vPath);


    /**
     *
     * @param vPath
     * @param cloudNum
     * @param prefix
     * @param suffix
     * @return
     */
    std::vector<CloudPointer> loadCloudsFromDirectory(std::string vPath, int cloudNum, const std::string
                        prefix = "cloud_cluster_", const std::string suffix = ".pcd");

    /*!
     * visualize all the point clouds within a vector
     * @param clouds
     * @param msg
     */
    void multiVisualizeCloud( std::vector<CloudPointer> clouds, const std::string msg = "");

    /*!
     * given the transformation matrix, calculate the distance (translation distance).
     * @param mat
     * @return
     */

    double calcDistanceByTranslation(Eigen::Matrix<float, 4, 4> mat);
    /*!
     * get the fitness score of two matches
     * @param cloud_a
     * @param cloud_b
     * @return
     */
    double calcFitnessScore(pcl::PointCloud<pcl::PointXYZ> cloud_a, pcl::PointCloud<pcl::PointXYZ> cloud_b);

    /*!
     * get the minimum distance among two clouds.
     * @param cloud_a
     * @param cloud_b
     * @return
     */
    double calcDistanceBetweenClouds(pcl::PointCloud<pcl::PointXYZ> cloud_a, pcl::PointCloud<pcl::PointXYZ> cloud_b);

    /*!
     * get the total count of files in a folder (excludes . and ..)
     * @param vPath
     * @return
     */
    int getFileCountInFolder(string vPath);


    /*!
     * perform a certain transformation matrix on a point cloud
     * @param transform
     * @param cloud
     * @return
     */
    CloudPointer transformCloudFromMatrix(Eigen::Matrix4f transform, CloudPointer cloud);

    /*!
     *
     * @param point
     * @param coefficients
     * @param dist2center
     * @return
     */
    double getPointDistance(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr coefficients, double dist2center = -1);


    /**
     *
     * @param cloud
     * @param coefficients
     * @param threshold
     * @param inliers
     */
    void getInliersByWeightedDistance(CloudPointer cloud, pcl::ModelCoefficients::Ptr coefficients, double threshold,
                                      pcl::PointIndices::Ptr inliers);

    /**
     *
     * @param cloud
     * @param coefficients
     * @param inliers
     * @return
     */
    double getTotalCost(CloudPointer cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers );


    void extractSubarea(CloudPointer input, CloudPointer target, double minx = -INT_MAX, double maxx = INT_MAX,
                        double minz = -INT_MAX, double maxz = INT_MAX, double miny = -INT_MAX, double maxy = INT_MAX);

    void fitGroundPlane(CloudPointer cloud, pcl::ModelCoefficients::Ptr resultCoeffi, pcl::PointIndices::Ptr resultInliers);

    std::vector<std::vector<int>> voxelizePointCloud(CloudPointer input);

    double inner(vector<double> v, vector<double> u);

    double norm(vector<double> v);

    double calcAngleBetweenTwoVector(vector<double> v, vector<double> u);

    pcl::PointXYZ pointAssociateToMap(pcl::PointXYZ pi, const std::vector<float> transformTobeMapped);

    pcl::PointXYZ pointAssociateTobeMapped(pcl::PointXYZ pi, const std::vector<float> transformTobeMapped);

    bool checkWithEffectiveRegion(pcl::PointXYZ pori, const std::vector<float> transform);

    void cloudAlignment(const std::vector<float> transformTobeMapped, CloudPointer src);


    std::vector<float> loadTransformFromTXT(const std::string vPath);

    void loadTransformFromTXT(const std::string vPath, cv::Mat &P, cv::Mat &translation);

    void loadKFromTXT(const std::string vPath, cv::Mat &K);




}




#endif //POINTS_GROUPING_UTILS_H

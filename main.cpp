#include <iostream>
#include <opencv2/core/core.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "utils.h"

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

#include <set>


// for convenience
using namespace std;





int main(int argc, char** argv) {
    cv::Mat rotation(3, 3, CV_64F);
    cv::Mat translation(3, 1, CV_64F);
    cv::Mat K(3, 3, CV_64F);
    cv::Mat H(3, 4, CV_64F);

    string transformPath = "../calib_cam_to_range_00.txt";
    string KPath = "../calib_cam_to_cam.txt";

    utils::loadKFromTXT(KPath, K);

    cv::Mat P(4, 4, CV_64F);
    utils::loadTransformFromTXT(transformPath, P, translation);
    P.at<double>(3, 0) = 0;
    P.at<double>(3, 1) = 0;
    P.at<double>(3, 2) = 0;
    P.at<double>(3, 3) = 1;
    cout << P <<endl;
    cout << K <<endl;
    P = P.inv();
    cout << "inv P" << P <<endl;

    string dirPath = "/home/lifan/LIDAR/workspace/image-lidar/";
    DIR *dir;
    struct dirent *ent;
    dir = opendir(dirPath.c_str());
    set<string> loadedFiles;
    while((ent = readdir(dir))!= NULL){

        if(string(ent->d_name) == "." || string(ent->d_name) == "..") // exclude '.' and '..'
            continue;
        string curfile = string(ent->d_name).substr(0, 13);
        if(loadedFiles.find(curfile)!=loadedFiles.end())     // exclude already readed files
            continue;

        cout << curfile << endl;
        cout << dirPath << endl;

        ofstream outputFile2D("./data/" + curfile + "-2d-points.txt");
        ofstream outputFile3D("./data/" + curfile + "-3d-points.txt");
        ofstream outpuFile3D_World("./data/" + curfile + "-3d-points-world.txt");


        // load in image file
        string imagePath = dirPath + curfile + ".jpg";
        cv::Mat image = cv::imread(imagePath, CV_LOAD_IMAGE_COLOR);


        // load in point cloud file
        string pointCloudPath = dirPath + curfile + "FullRes.pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = utils::loadCloudFromPath(pointCloudPath);

        // load in the transformation matrix
        string transformPath = dirPath + curfile + ".txt";
        vector<float> transform = utils::loadTransformFromTXT(transformPath);





        vector<cv::Point3f> input;
        vector<cv::Point2f> output;
        for (int i = 0; i < cloud->points.size(); i++) {
            pcl::PointXYZ curPoint = cloud->points[i];
            if ((curPoint.x * curPoint.x) + (curPoint.y * curPoint.y) + (curPoint.z * curPoint.z) <= 25 &&
                curPoint.z > -1)
                continue;
            cv::Point3f newPoint(curPoint.x, curPoint.y, curPoint.z);
            input.push_back(newPoint);
        }
//    vector<float> distort = {0, 0,0, 0};
//    cv::Mat rvec;
//    cv::Rodrigues(P(cv::Range(0, 3), cv::Range(0, 3)), rvec);
//    cv::projectPoints(input,  rvec, P(cv::Range(0, 3), cv::Range(3, 4)), K,  distort, output);
        vector<cv::Point3f> output_3d;
        cv::Mat Projection = P(cv::Range(0, 3), cv::Range(0, 4));
        vector<double> distance;
        for (int i = 0; i < input.size(); i++) {
            cv::Mat cur(4, 1, CV_64F);
            cur.at<double>(0, 0) = input[i].x;
            cur.at<double>(1, 0) = input[i].y;
            cur.at<double>(2, 0) = input[i].z;
            cur.at<double>(3, 0) = 1;
            cv::Mat pointHomo(3, 1, CV_64F);
            pointHomo = Projection * cur;
            if (pointHomo.at<double>(2, 0) < 0.1)
                continue;
            pointHomo = K * pointHomo;
            cv::Point2f pointDehomo(pointHomo.at<double>(0, 0) / pointHomo.at<double>(2, 0),
                                    pointHomo.at<double>(1, 0) / pointHomo.at<double>(2, 0));
            output.push_back(pointDehomo);
            output_3d.push_back(input[i]);
            distance.push_back((input[i].x * input[i].x) + (input[i].y * input[i].y) + (input[i].z * input[i].z));

        }

        double maxDistance = *max_element(distance.begin(), distance.end());
        for (int i = 0; i < output.size(); i++) {
            outputFile2D << output[i].x << " " <<output[i].y<<endl;
            outputFile3D << output_3d[i].x << " " <<output_3d[i].y << " " << output_3d[i].z <<endl;

             if (output[i].x > 0 && output[i].x < image.cols && output[i].y > 0 && output[i].y < image.rows) {
                //cv::circle(image, output[i], 1, cv::Scalar(0, 0, 255 * min((distance[i] / 200), 1.0)), 1);
                cv::circle(image, output[i], 1, cv::Scalar(0, 255 * min((distance[i] / 350), 1.0), 255 * min((distance[i] / 350), 1.0)), 1);
            }
        }


        cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
        cv::imshow("Display window", image);                   // Show our image inside it.
        cv::waitKey(0);
        outputFile3D.close();
        outputFile2D.close();
    }

    cout << "Done!\n";
}
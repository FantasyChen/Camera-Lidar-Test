//
// Created by lifan on 2/11/17.
//

#include "utils.h"
/**
 * Check if the width follows the rule of write before write to file.
 * @param dirPath
 * @param fileName
 * @param cloud
 */
namespace utils {
    void pclPointCloudWriter(string &dirPath, string &fileName, CloudPointer cloud) {
        pcl::PCDWriter writer;
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        string path = dirPath + fileName + ".pcd";
        cout << "Writing point cloud to " << path << endl;
        writer.write<pcl::PointXYZ>(path, *cloud, false);
    }


    double calcPointPlaneDist(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ point) {
        double a = plane->values[0];
        double b = plane->values[1];
        double c = plane->values[2];
        double d = plane->values[3];
        double x = point.x;
        double y = point.y;
        double z = point.z;
        double dist = abs(a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c);
        return dist;
    }

    double calcPointPlaneDist(pcl::ModelCoefficients plane, pcl::PointXYZ point) {
        double a = plane.values[0];
        double b = plane.values[1];
        double c = plane.values[2];
        double d = plane.values[3];
        double x = point.x;
        double y = point.y;
        double z = point.z;
        double dist = abs(a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c);
        return dist;
    }


    void visualizeCloud(CloudPointer first, const std::string msg) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(first, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZ>(first, tgt_h, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        if (msg != "") {
            viewer->addText(msg, 5, 5, 10, 255, 0, 0, "text");
        }
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

    }


    void visualizeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr first, const std::string msg) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> tgt_h(first);
        viewer->addPointCloud<pcl::PointXYZRGB>(first, tgt_h, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        if (msg != "") {
            viewer->addText(msg, 5, 5, 10, 255, 0, 0, "text");
        }
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

    }


    bool checkWithPriorInfo(pcl::PointXYZ &curPoint) {
        // First Prior
//        if(curPoint.y < -1.7)  // ground plane
//            return false;
//
//        if(curPoint.y > 2)  // sky
//            return false;
//
//        if(curPoint.x > 20.0 || curPoint.x < -5) // Inside the road
//            return false;
//        if(fabs(curPoint.z) > 30.0) // to far away
//            return false;



        // Second Prior
        if (curPoint.y < -2)  // ground plane
            return false;

        if (curPoint.y > 1.5)  // sky
            return false;

        if (curPoint.x > 30.0 || curPoint.x < -30) // Inside the road
            return false;
//        if(fabs(curPoint.z) > 45) // to far away
//            return false;
        return true;

    }

    bool checkIfGroundPlaneCoeffi(pcl::ModelCoefficients::Ptr coefficients) {
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];
        double d = coefficients->values[3];
        double yFraction = fabs(b) / (a * a + b * b + c * c);
        cout << "yFraction is " << yFraction << "d is " << d << endl;

        if (yFraction > 0.6)  // 0.9
        {
            if (d < -1.4) // -1.8
            {
                for (int k = 0; k < 4; ++k) {
                    std::cout << coefficients->values[k] << " ";
                }
                cout << ", " << yFraction << " " << d << endl;

                return true;
            }
        }
        return false;
    }


/*!
 * visualize point clouds pairwise
 * @param first
 * @param second
 * @param msg
 */
    void pairwiseVisualizeCloud(CloudPointer first, CloudPointer second, const std::string msg) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(first, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(second, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(first, tgt_h, "sample cloud");
        viewer->addPointCloud<pcl::PointXYZ>(second, src_h, "sample cloud 2");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud 2");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        if (msg != "") {
            viewer->addText(msg, 5, 5, 10, 255, 0, 0, "text");
        }
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

    }

    CloudPointer loadCloudFromPath(std::string vPath) {
        CloudPointer curCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2 cloud_blob;
        pcl::io::loadPCDFile(vPath.c_str(), cloud_blob);
        pcl::fromPCLPointCloud2(cloud_blob, *curCloud);
        return curCloud;
    }

/*!
 * load point clouds from a directoy
 * @param vPath: the path of the directory
 * @param cloudNum: the number of total clouds in the directory
 * @param prefix: the prefix of a cloud file name. Typically "cloud_cluster_"
 * @param suffix: the suffix of a cloud file name. Typically ".pcd"
 * @return
 */
    std::vector<CloudPointer> loadCloudsFromDirectory(std::string vPath, int cloudNum, const std::string
    prefix, const std::string suffix) {
        std::vector<CloudPointer> clouds(cloudNum);
        for (int i = 0; i < cloudNum; i++) {
            std::string curPath = vPath + prefix + std::to_string(i) + suffix;
            CloudPointer curCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PCLPointCloud2 cloud_blob;
            pcl::io::loadPCDFile(curPath.c_str(), cloud_blob);
            pcl::fromPCLPointCloud2(cloud_blob, *curCloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
            std::cout << "Loading..." << i << "  Size:" << curCloud->points.size() << endl;
            clouds[i] = curCloud->makeShared();  // make copy into vector
        }
        return clouds;
    }

/*!
 * visualize all the point clouds within a vector
 * @param clouds
 * @param msg
 */
    void multiVisualizeCloud(std::vector<CloudPointer> clouds, const std::string msg) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

        viewer->setBackgroundColor(0, 0, 0);
        int _size = clouds.size();
        for (int i = 0; i < _size; i++) {
            pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> src_h(clouds[i]);
            viewer->addPointCloud<pcl::PointXYZ>(clouds[i], src_h, "sample cloud" + std::to_string(i));
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                                     "sample cloud" + std::to_string(i));

        }
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        if (msg != "") {
            viewer->addText(msg, 5, 5, 10, 255, 0, 0, "text");
        }
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

    }

/*!
 * given the transformation matrix, calculate the distance (translation distance).
 * @param mat
 * @return
 */

    double calcDistanceByTranslation(Eigen::Matrix<float, 4, 4> mat) {
        // pass
        return sqrt(mat(0, 3) * mat(0, 3) + mat(1, 3) * mat(1, 3) + mat(2, 3) * mat(2, 3));
    }


/*!
 * get the fitness score of two matches
 * @param cloud_a
 * @param cloud_b
 * @return
 */
    double calcFitnessScore(pcl::PointCloud<pcl::PointXYZ> cloud_a, pcl::PointCloud<pcl::PointXYZ> cloud_b) {
        pcl::search::KdTree<pcl::PointXYZ> tree_b;
        tree_b.setInputCloud(cloud_b.makeShared());
        double sum_dist_a = 0;
        for (size_t i = 0; i < cloud_a.points.size(); ++i) {
            std::vector<int> indices(1);
            std::vector<float> sqr_distances(1);

            tree_b.nearestKSearch(cloud_a.points[i], 1, indices, sqr_distances);
            sum_dist_a += sqrt(sqr_distances[0]);
        }

        // compare B to A
        pcl::search::KdTree<pcl::PointXYZ> tree_a;
        tree_a.setInputCloud(cloud_a.makeShared());
        double sum_dist_b = 0;
        for (size_t i = 0; i < cloud_b.points.size(); ++i) {
            std::vector<int> indices(1);
            std::vector<float> sqr_distances(1);

            tree_a.nearestKSearch(cloud_b.points[i], 1, indices, sqr_distances);
            sum_dist_b += sqrt(sqr_distances[0]);
        }


        double dist = (sum_dist_a + sum_dist_b) / 2 / (cloud_b.points.size() + cloud_a.points.size());
        return 1 / dist;

    }

/*!
 * get the minimum distance among two clouds.
 * @param cloud_a
 * @param cloud_b
 * @return
 */
    double calcDistanceBetweenClouds(pcl::PointCloud<pcl::PointXYZ> cloud_a, pcl::PointCloud<pcl::PointXYZ> cloud_b) {
        pcl::search::KdTree<pcl::PointXYZ> tree_b;
        tree_b.setInputCloud(cloud_b.makeShared());
        double sum_dist_a = 0;
        for (size_t i = 0; i < cloud_a.points.size(); ++i) {
            std::vector<int> indices(1);
            std::vector<float> sqr_distances(1);

            tree_b.nearestKSearch(cloud_a.points[i], 1, indices, sqr_distances);
            sum_dist_a += sqrt(sqr_distances[0]);
        }

        // compare B to A
        pcl::search::KdTree<pcl::PointXYZ> tree_a;
        tree_a.setInputCloud(cloud_a.makeShared());
        double sum_dist_b = 0;
        for (size_t i = 0; i < cloud_b.points.size(); ++i) {
            std::vector<int> indices(1);
            std::vector<float> sqr_distances(1);

            tree_a.nearestKSearch(cloud_b.points[i], 1, indices, sqr_distances);
            sum_dist_b += sqrt(sqr_distances[0]);
        }


        double dist = (sum_dist_a + sum_dist_b) / 2 / (cloud_b.points.size() + cloud_a.points.size());
        return dist;
    }

/*!
 * get the total count of files in a folder (excludes . and ..)
 * @param vPath
 * @return
 */
    int getFileCountInFolder(string vPath) {
        DIR *dir;
        struct dirent *ent;
        int _count = 0;
        if ((dir = opendir(vPath.c_str())) != NULL) {
            /* print all the files and directories within directory */
            while ((ent = readdir(dir)) != NULL) {
                // cout << ent->d_name << endl;
                _count++;
            }
            closedir(dir);
            return _count - 2;  // -2 for . and ..
        } else {
            /* could not open directory */
            perror("");
            return EXIT_FAILURE;
        }
    }


/*!
 * perform a certain transformation matrix on a point cloud
 * @param transform
 * @param cloud
 * @return
 */
    CloudPointer transformCloudFromMatrix(Eigen::Matrix4f transform, CloudPointer cloud) {
        // Executing the transformation
        CloudPointer transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        // You can either apply transform_1 or transform_2; they are the same
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
        return transformed_cloud;
    }

    double getPointDistance(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr coefficients, double dist2center) {
        double dist = calcPointPlaneDist(coefficients, point);
        return dist - dist2center * 0.01;
    }

    void getInliersByWeightedDistance(CloudPointer cloud, pcl::ModelCoefficients::Ptr coefficients, double threshold,
                                      pcl::PointIndices::Ptr inliers) {
        inliers->indices.clear();
        for (int i = 0; i < cloud->points.size(); i++) {
            pcl::PointXYZ curPoint = cloud->points[i];
            double dist2center = fabs(curPoint.x);
            if (utils::getPointDistance(curPoint, coefficients, dist2center) < threshold) {
                inliers->indices.push_back(i);
            }

        }
    }

    double getTotalCost(CloudPointer cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers) {
        if (inliers->indices.size() == 0) {
            return 0;
        }
        double totalCost = 0;
        for (int i = 0; i < inliers->indices.size(); i++) {
            totalCost += calcPointPlaneDist(coefficients, cloud->points[inliers->indices[i]]);
        }
        return totalCost / inliers->indices.size();
    }


    void extractSubarea(CloudPointer input, CloudPointer target, double minx, double maxx,
                        double minz, double maxz, double miny, double maxy) {
        for (int i = 0; i < input->points.size(); i++) {
            pcl::PointXYZ curPoint = input->points[i];
            if (curPoint.x <= maxx && curPoint.x >= minx &&
                curPoint.y <= maxy && curPoint.y >= miny &&
                curPoint.z <= maxz && curPoint.z >= minz)
                target->points.push_back(curPoint);

        }
    }

    void
    fitGroundPlane(CloudPointer cloud, pcl::ModelCoefficients::Ptr resultCoeffi, pcl::PointIndices::Ptr resultInliers) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        coefficients->values.resize(4);
        coefficients->values[0] = 0;    //  initial guess of the plane
        coefficients->values[1] = 1;
        coefficients->values[2] = 0;
        coefficients->values[3] = -1;
        double distThreshold = 1;  // TOCHECK
        double sizeThreshold = cloud->points.size() / 1 * 2;
        double k = 1.0;
        double costThreshold = 0.0001;
        double preCost = 0;
        int iterNum = 0;
        int maxIterNum = 100;
        int bestInliersCount = -INT_MAX;
        double prob = 0.99;
        double log_prob = log(1 - prob);
        int RANSACPointNum = 8;

        int cloudSize = cloud->points.size();
        Eigen::Vector4f planeParamters;
        float curvature;

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::PointIndices::Ptr currentInliers(new pcl::PointIndices);
        std::vector<int> selection;
        while (iterNum < maxIterNum) {
            // Generate random samples for this cycle
            currentInliers->indices.clear();
            std::default_random_engine engine;
            std::uniform_int_distribution<> distribution(0, cloudSize - 1);
            for (int i = 0; i < RANSACPointNum; i++) {
                int randomIndice = distribution(engine);
                if (std::find(selection.begin(), selection.end(), randomIndice) != selection.end())
                    selection.push_back(randomIndice);
            }
            pcl::computePointNormal(*cloud, selection, planeParamters, curvature);
            coefficients->values[0] = planeParamters[0];    //  initial guess of the plane
            coefficients->values[1] = planeParamters[1];
            coefficients->values[2] = planeParamters[2];
            coefficients->values[3] = planeParamters[3];
            for (int i = 0; i < cloud->points.size(); i++) {
                pcl::PointXYZ curPoint = cloud->points[i];
                double dist2center = fabs(curPoint.x);
                if (getPointDistance(curPoint, coefficients, dist2center) < distThreshold) {
                    inliers->indices.push_back(i);
                }
            }
            if (inliers->indices.size() < 8) {
                distThreshold -= 0.01;
                cout << "Can not estimate a plane from given inliers by given threshold" << endl;
            } else {
                distThreshold = min(distThreshold + 0.01, -0.1);
            }


            ++iterNum;
        }

        *resultInliers = *inliers;
        *resultCoeffi = *coefficients;


    }

    std::vector<std::vector<int>> voxelizePointCloud(CloudPointer input) {
        // Downsample the cloud first. Find key points in the middle of each voxel.

        std::vector<std::vector<int>> indices;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        float voxelLength = 1.0;
        CloudPointer downSample(new pcl::PointCloud<pcl::PointXYZ>);
        sor.setInputCloud(input);
        sor.setLeafSize(voxelLength, voxelLength, voxelLength);
        sor.filter(*downSample);
//        std::set<int> allIndices;
//        for(int i = 0; i < input->points.size(); i++){
//            allIndices.insert(i);
//        }


        // Create octree for voxel search
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(1.5);
        octree.setInputCloud(input);
        octree.addPointsFromInputCloud();
        for (int i = 0; i < downSample->points.size(); i++) {
            pcl::PointXYZ queryPoint = downSample->points[i];
            if (fabs(queryPoint.x) > 30)
                continue;
            std::vector<int> curIndices;
            if (octree.voxelSearch(queryPoint, curIndices)) {
                indices.push_back(curIndices);
            }
        }
        // test
        for (int i = 0; i < indices.size(); i++) {
            CloudPointer curCloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (int j = 0; j < indices[i].size(); j++) {
                curCloud->points.push_back(input->points[indices[i][j]]);
            }
            // utils::visualizeCloud(curCloud);
        }
        return indices;

    }


    double inner(vector<double> v, vector<double> u) {
        double sum = 0;
        for (int i = 0; i < v.size(); i++) {
            sum += v[i] * u[i];
        }
        return sum;
    }

    double norm(vector<double> v) {
        double sum = 0;
        for (int i = 0; i < v.size(); i++) {
            sum += v[i] * v[i];
        }
        return sqrt(sum);
    }

    double calcAngleBetweenTwoVector(vector<double> v, vector<double> u) {
        double cosTheta = utils::inner(v, u) /
                          (utils::norm(v) * utils::norm(u));
        if (cosTheta < 0) {
            cosTheta = -cosTheta;
        }
        double PI = 3.1415926;
        double angle = acos(cosTheta) * 180.0 / PI;
        return angle;
    }


    pcl::PointXYZ pointAssociateToMap(pcl::PointXYZ pi, const std::vector<float> transformTobeMapped) {
        float x1 = cos(transformTobeMapped[2]) * pi.x
                   - sin(transformTobeMapped[2]) * pi.y;
        float y1 = sin(transformTobeMapped[2]) * pi.x
                   + cos(transformTobeMapped[2]) * pi.y;
        float z1 = pi.z;

        float x2 = x1;
        float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
        float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

        pcl::PointXYZ po;
        po.x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
               + transformTobeMapped[3];
        po.y = y2 + transformTobeMapped[4];
        po.z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
               + transformTobeMapped[5];
        return po;
    }


    pcl::PointXYZ pointAssociateTobeMapped(pcl::PointXYZ pi, const std::vector<float> transformTobeMapped) {
        float x1 = cos(transformTobeMapped[1]) * (pi.x - transformTobeMapped[3])
                   - sin(transformTobeMapped[1]) * (pi.z - transformTobeMapped[5]);
        float y1 = pi.y - transformTobeMapped[4];
        float z1 = sin(transformTobeMapped[1]) * (pi.x - transformTobeMapped[3])
                   + cos(transformTobeMapped[1]) * (pi.z - transformTobeMapped[5]);

        float x2 = x1;
        float y2 = cos(transformTobeMapped[0]) * y1 + sin(transformTobeMapped[0]) * z1;
        float z2 = -sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;
        pcl::PointXYZ po;
        po.x = cos(transformTobeMapped[2]) * x2
               + sin(transformTobeMapped[2]) * y2;
        po.y = -sin(transformTobeMapped[2]) * x2
               + cos(transformTobeMapped[2]) * y2;
        po.z = z2;
        return po;
    }

    bool checkWithEffectiveRegion(pcl::PointXYZ pori, const std::vector<float> transform) {
        pcl::PointXYZ pi = pointAssociateTobeMapped(pori, transform);
        double maxX = 30, minX = -30, maxZ = 30, minZ = -30, maxY = 1.5, minY = -2;
        if (pi.x < maxX && pi.x > minX && pi.y < maxY && pi.y > minY && pi.z > minZ && pi.z < maxZ) {
            return true;
        }
        return false;
    }


    void cloudAlignment(const std::vector<float> transformTobeMapped, CloudPointer src) {
        visualizeCloud(src);
        for (int i = 0; i < src->points.size(); i++) {
            pointAssociateToMap(src->points[i], transformTobeMapped);
        }
        visualizeCloud(src);
    }

    std::vector<float> loadTransformFromTXT(const std::string vPath) {
        std::ifstream textfile(vPath);
        std::string line;
        std::vector<float> transform;
        while (std::getline(textfile, line)) {
            std::stringstream linestream(line);
            std::string data;
            while (std::getline(linestream, data, ' ')) {
                transform.push_back(std::atof(data.c_str()));
            }
        }
        if (transform.size() == 0) {
            cout << "Unable to open the transform file, or it is empty!" << endl;
        }
        return transform;
    }

    void loadTransformFromTXT(const std::string vPath, cv::Mat &P, cv::Mat &translation) {
        std::ifstream textfile(vPath);
        std::string line;
        while (std::getline(textfile, line)) {
            std::stringstream linestream(line);
            std::string data;
            std::getline(linestream, data, ' ');
            if (data == "R:") {
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        std::getline(linestream, data, ' ');
                        P.at<double>(i, j) = atof(data.c_str());
                        cout << " Got rotation " << P.at<double>(i, j) << endl;
                    }
                }
            }
            if (data == "T:") {
                for (int j = 0; j < 3; j++) {
                    std::getline(linestream, data, ' ');
                    P.at<double>(j, 3) = atof(data.c_str());
                    cout << " Got translation " << P.at<double>(j, 3) << endl;
                }
            }
        }

    }

    void loadKFromTXT(const std::string vPath, cv::Mat &K) {
        std::ifstream textfile(vPath);
        std::string line;
        while (std::getline(textfile, line)) {
            std::stringstream linestream(line);
            std::string data;
            std::getline(linestream, data, ' ');
            if (data == "K_00:") {
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        std::getline(linestream, data, ' ');
                        K.at<double>(i, j) = atof(data.c_str());
                        cout << " Got K " << K.at<double>(i, j) << endl;
                    }
                }
            }
        }


    }
}
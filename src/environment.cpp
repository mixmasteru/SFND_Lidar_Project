/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


const std::vector<Color> colors = {Color(1, 0, 0),
                                   Color(1, 1, 0),
                                   Color(0, 0, 1)};

const Box roof = Box(-1.5, -1.7, -1, 2.6, 1.7, -0.4);
const Color roofColor = Color(1, 0.75, 0.75);

const Eigen::Vector4f minPoint = Eigen::Vector4f(-30, -6.0, -3, 1);
const Eigen::Vector4f maxPoint = Eigen::Vector4f(30, 7.0, 10, 1);


void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud,
                                                                                    0.2f,
                                                                                    minPoint,
                                                                                    maxPoint);

    renderBox(viewer, roof, 100, roofColor, 0.5);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud;
    segmentCloud = pointProcessorI->SegmentPlaneRansac(filterCloud, 10, 0.3);

    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
    cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 1.5, 10, 1000);


    int clusterId = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[1]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, colors[0], 0.5);
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle) {
        case XY :
            viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
            break;
        case TopDown :
            viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
            break;
        case Side :
            viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
            break;
        case FPS :
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}


int main(int argc, char **argv) {
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped()) {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}

// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    // Render the cars using constructors
    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    // Render all objects in the PCL viewers
    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

// Using simulated point cloud data.
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS (Note: Obst + Plane = pointCloud!)
    bool render_scene = false;
    bool render_lidarRays = false;
    bool render_pointCloud = false;
    bool render_obst = true;
    bool render_plane = true;
    bool render_clusters = true;
    bool render_box = true;

    std::vector<Car> cars = initHighway(render_scene, viewer);

    // TODO:: Create lidar sensor - 0 ground plane slope
    Lidar *lidar = new Lidar(cars, 0);

    // TODO:: Create point processor, and render results in viewer
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    if (render_lidarRays)
        renderRays(viewer, lidar->position, inputCloud);
    if (render_pointCloud)
        renderPointCloud(viewer, inputCloud, "Simple Point Cloud");

    // Use Point Processor object to render segmented point cloud (obstacles from environment)
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlaneOwnImplementation(inputCloud, 50, 0.1);
    // std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    if (render_obst)
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    if (render_plane)
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    // Use Point Processor object to identify and render point clusters
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.ClusteringOwnImplementation(segmentCloud.first, 1.0, 3, 30);
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if (render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        }

        if (render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
}

// Using real point cloud data. Point cloud is passed in as reference to prevent copying (memory-efficient).
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // RENDER OPTIONS (Note: Obst + Plane = pointCloud!)
    bool render_pointCloud = false;
    bool render_pointCloud_filtered = false;
    bool render_obst = false;
    bool render_plane = true;
    bool render_clusters = true;
    bool render_box = true;

    // HYPER PARAMETERS
    Eigen::Vector4f boxFilterMin = {-10.0f, -6.0f, -5.0f, 1};
    Eigen::Vector4f boxFilterMax = {35.0f, 7.0f, 5.0f, 1};
    int segmentationMaxIterations = 20;
    float segmentationDistThreshold = 0.2;
    float voxelFilterResolution = 0.2;
    float clusterTolerance = 0.4;
    int clusterMinSize = 10;
    int clusterMaxSize = 1000;

    if (render_pointCloud)
        renderPointCloud(viewer, inputCloud, "inputCloud"); //overloaded renderPointCloud function for point<XYZI> type.

    // Voxel grid filtering + box filtering to reduce number of points
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor->FilterCloud(inputCloud, voxelFilterResolution, boxFilterMin, boxFilterMax);
    if (render_pointCloud_filtered)
        renderPointCloud(viewer, filteredCloud, "filterCloud");

    // Plane segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlaneOwnImplementation(filteredCloud, segmentationMaxIterations, segmentationDistThreshold);
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filteredCloud, 100, 0.2);
    if (render_obst)
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    if (render_plane)
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    // Use Point Processor object to identify and render point clusters
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->ClusteringOwnImplementation(segmentCloud.first, clusterTolerance, clusterMinSize, clusterMaxSize);
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if (render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        }

        if (render_box)
        {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();             // init in stack or heap is ok.
    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1"); // alternative: data 2
    auto streamIterator = stream.begin();                                                                      // iterator pointing at pcd data stream.
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    // Update loop - keep looping until viewer is closed
    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloud = pointProcessor->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessor, inputCloud);

        // Updates streamIterator. Loops from beginning if reach end.
        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}

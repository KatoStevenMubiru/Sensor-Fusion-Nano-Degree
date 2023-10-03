/*Kato Steven Mubiru */
// Functions and structs used to render the enviroment

#include "render.h"

void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    const double roadLength = 50.0;
    const double roadWidth = 12.0;
    const double roadHeight = 0.2;
    
    const double roadHalfLength = roadLength / 2;
    const double roadHalfWidth = roadWidth / 2;

    viewer->addCube(-roadHalfLength, roadHalfLength, -roadHalfWidth, roadHalfWidth, -roadHeight, 0, 0.2, 0.2, 0.2, "highwayPavement");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "highwayPavement");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.2, 0.2, "highwayPavement");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "highwayPavement");
    
    const double lineZ = 0.01;
    viewer->addLine(pcl::PointXYZ(-roadHalfLength, -roadWidth/6, lineZ), pcl::PointXYZ(roadHalfLength, -roadWidth/6, lineZ), 1, 1, 0, "line1");
    viewer->addLine(pcl::PointXYZ(-roadHalfLength, roadWidth/6, lineZ), pcl::PointXYZ(roadHalfLength, roadWidth/6, lineZ), 1, 1, 0, "line2");
}

void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    int countRays = 0;
    for (const pcl::PointXYZ& point : cloud->points)
    {
        const std::string rayName = "ray" + std::to_string(countRays);
        viewer->addLine(pcl::PointXYZ(origin.x, origin.y, origin.z), point, 1, 0, 0, rayName);
        countRays++;
    }
}

void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    while (countRays)
    {
        countRays--;
        const std::string rayName = "ray" + std::to_string(countRays);
        viewer->removeShape(rayName);
    }
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& name, const Color& color)
{
    viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& name, const Color& color)
{
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>::Ptr colorHandler;
    
    if (color.r == -1)
    {
        // Select color based on cloud intensity
        colorHandler = pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>(cloud, "intensity");
    }
    else
    {
        // Select color based on input value
        colorHandler = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(cloud, color.r, color.g, color.b);
    }
    
    viewer->addPointCloud<pcl::PointXYZI>(cloud, colorHandler, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, const Box& box, int id, const Color& color, float opacity)
{
    if (opacity > 1.0)
        opacity = 1.0;
    if (opacity < 0.0)
        opacity = 0.0;

    const std::string cubeName = "box" + std::to_string(id);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cubeName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cubeName);

    const std::string cubeFillName = "boxFill" + std::to_string(id);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFillName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFillName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFillName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cubeFillName);
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, const BoxQ& box, int id, const Color& color, float opacity)
{
    if (opacity > 1.0)
        opacity = 1.0;
    if (opacity < 0.0)
        opacity = 0.0;

    const std::string cubeName = "box" + std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cubeName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cubeName);

    const std::string cubeFillName = "boxFill" + std::to_string(id);
    viewer->

addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFillName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFillName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFillName);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cubeFillName);
}
```


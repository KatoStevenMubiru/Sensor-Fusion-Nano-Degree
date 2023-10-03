#ifndef RENDER_H
#define RENDER_H

#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include <iostream>
#include <vector>
#include <string>

// Define a struct for colors
struct Color
{
    float r, g, b;

    Color(float setR, float setG, float setB)
        : r(setR), g(setG), b(setB)
    {}
};

// Define a struct for 3D vectors
struct Vect3
{
    double x, y, z;

    Vect3(double setX, double setY, double setZ)
        : x(setX), y(setY), z(setZ)
    {}

    Vect3 operator+(const Vect3& vec)
    {
        Vect3 result(x + vec.x, y + vec.y, z + vec.z);
        return result;
    }
};

// Define an enum for camera angles
enum CameraAngle
{
    XY, TopDown, Side, FPS
};

// Define a class for rendering cars
class Car
{
public:
    // Constructor
    Car(const Vect3& setPosition, const Vect3& setDimensions, const Color& setColor, const std::string& setName)
        : position(setPosition), dimensions(setDimensions), color(setColor), name(setName)
    {}

    // Render the car
    void render(pcl::visualization::PCLVisualizer::Ptr& viewer)
    {
        // Render the bottom of the car
        renderCube(viewer, position.z, position.z + dimensions.z * 2.0 / 3.0, name, color);

        // Render the top of the car
        renderCube(viewer, position.z + dimensions.z * 2.0 / 3.0, position.z + dimensions.z, name + "Top", color);
    }

    // Check collision with a point
    bool checkCollision(const Vect3& point)
    {
        return (inBetween(point.x, position.x, dimensions.x / 2) &&
                inBetween(point.y, position.y, dimensions.y / 2) &&
                inBetween(point.z, position.z + dimensions.z / 3, dimensions.z / 3)) ||
               (inBetween(point.x, position.x, dimensions.x / 4) &&
                inBetween(point.y, position.y, dimensions.y / 2) &&
                inBetween(point.z, position.z + dimensions.z * 5.0 / 6, dimensions.z / 6));
    }

private:
    Vect3 position;
    Vect3 dimensions;
    std::string name;
    Color color;

    // Helper function to render a cube
    void renderCube(pcl::visualization::PCLVisualizer::Ptr& viewer, double zMin, double zMax, const std::string& cubeName, const Color& cubeColor)
    {
        viewer->addCube(position.x - dimensions.x / 2, position.x + dimensions.x / 2, position.y - dimensions.y / 2,
                         position.y + dimensions.y / 2, zMin, zMax, cubeColor.r, cubeColor.g, cubeColor.b, cubeName);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeName);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cubeColor.r, cubeColor.g, cubeColor.b, cubeName);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, cubeName);
    }

    // Helper function to check if a point is within a range
    bool inBetween(double point, double center, double range)
    {
        return (center - range <= point) && (center + range >= point);
    }
};

// Other rendering functions
void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& name, const Color& color = Color(1, 1, 1));
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& name, const Color& color = Color(-1, -1, -1));
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, const Box& box, int id, const Color& color = Color(1, 0, 0), float opacity = 1);
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, const BoxQ& box, int id, const Color& color = Color(1, 0, 0), float opacity = 1);

#endif

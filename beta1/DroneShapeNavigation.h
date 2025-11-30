#ifndef DRONE_SHAPE_NAVIGATION_H
#define DRONE_SHAPE_NAVIGATION_H

#include <vector>
#include <string>

class DroneShapeNavigation {
public:
    // Constructor
    DroneShapeNavigation();

    // Core functionality
    bool detectShape(const std::string& shapeType);
    bool navigateToShape(const std::string& shapeType);
    bool landOnPad();
    bool executeCourse(const std::vector<std::string>& shapes);

    // Getters
    std::string getCurrentShape() const;
    bool isLanded() const;

private:
    std::string currentShape;
    bool landed;
    std::vector<std::string> courseShapes;
};

#endif // DRONE_SHAPE_NAVIGATION_H


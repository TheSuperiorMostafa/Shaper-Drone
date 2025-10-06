#include "DroneShapeNavigation.h"
#include <iostream>

int main() {
    DroneShapeNavigation drone;
    
    // Example usage
    std::vector<std::string> shapes = {"circle", "square", "triangle"};
    
    drone.detectShape("circle");
    drone.navigateToShape("circle");
    drone.landOnPad();
    drone.executeCourse(shapes);
    
    return 0;
}


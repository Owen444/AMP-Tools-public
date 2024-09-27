// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
#include "HW4Functions.h"
// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    // Problem 1 Code
    Obstacle2D obstacle = HW4::getEx1TriangleObstacle();
     std::vector<Eigen::Vector2d> verticiesR = obstacle.verticesCCW();
    // Make verticiesR = -verticiesR
    for(int i = 0; i < verticiesR.size(); i++) {
        verticiesR[i] = -verticiesR[i];
    }
    Obstacle2D obstacle_R(verticiesR);
    Obstacle2D obstacle_C = HW4Functions::getMinkowskiSum(obstacle, obstacle);
    Visualizer::makeFigure({obstacle,obstacle_R});
    Visualizer::makeFigure({obstacle_C});
    std::cout<<"obstacle C vertices: "<<std::endl;
    for(auto v:obstacle_C.verticesCCW()){
        std::cout<<v.transpose()<<std::endl;
    }
    std::vector<Obstacle2D> rot_obstacles;
    std::vector<Obstacle2D> rot_obstacles_C;
    std::vector<double> angles;
    Obstacle2D obstacle_rot;
    Obstacle2D obstacle_rot_C;
    for(int i = 0; i < 12; i++) {
        angles.push_back(i*M_PI/6);
        obstacle_rot =  HW4Functions::rotatePolygon(obstacle, angles[i]);
        obstacle_rot_C = HW4Functions::getMinkowskiSum(obstacle, obstacle_rot);
        rot_obstacles_C.push_back(obstacle_rot_C);
        rot_obstacles.push_back(obstacle_rot);
    }
    Visualizer::makeFigure(rot_obstacles);
    Visualizer::makeFigure(rot_obstacles_C,angles);

    // Problem 2 Code
    // Forward Kinematics
    std::vector<double> link_lengths = {0.5, 1.0, 0.5};
    std::vector<double> link_lengths_b = {1.0, 0.5,1.0};
    //std::vector<double> link_lengths_b = {1.0, 0.5};
    Eigen::Vector2d joint_location;
    MyManipulator2D manipulator(link_lengths);
    MyManipulator2D manipulator_inverse(link_lengths_b);
    // You can visualize your manipulator given an angle state like so:
    amp::ManipulatorState test_state(3);
    test_state << M_PI/6 , M_PI/3, 7*M_PI/4;
    // // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    Visualizer::makeFigure(manipulator, test_state); 
    // Inverse Kinematics
    Eigen::Vector2d end_effector_location(2.0, 0.0);
    amp::ManipulatorState ik_state = manipulator_inverse.getConfigurationFromIK(end_effector_location);
    // // You can visualize your manipulator given an end effector location like so:
    Visualizer::makeFigure(manipulator_inverse, ik_state);

    // Problem 3 Code
    // Create the collision space constructor
    std::size_t n_cells = 500;
    std::vector<double> link_lengths_3 = {1.0, 1.0};
    MyManipulatorCSConstructor cspace_constructor(n_cells);
    MyManipulator2D manipulator_3(link_lengths_3);
    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace1 = cspace_constructor.construct(manipulator_3, HW4::getEx3Workspace1());
    std::unique_ptr<amp::GridCSpace2D> cspace2 = cspace_constructor.construct(manipulator_3, HW4::getEx3Workspace2());
    std::unique_ptr<amp::GridCSpace2D> cspace3 = cspace_constructor.construct(manipulator_3, HW4::getEx3Workspace3());
    // You can visualize your cspace 
    Visualizer::makeFigure(HW4::getEx3Workspace1());
    Visualizer::makeFigure(HW4::getEx3Workspace2());
    Visualizer::makeFigure(HW4::getEx3Workspace3());
    Visualizer::makeFigure(*cspace1);
    Visualizer::makeFigure(*cspace2);
    Visualizer::makeFigure(*cspace3);
    Visualizer::showFigures();

    // // Grade method
   // amp::HW4::grade<MyManipulator2D>(cspace_constructor, "owen.craig@colorado.edu", argc, argv);
    return 0;
}
// Create a function that takes the minkowski sum of two polygons this function should take in two objects of class Obstacle2D and return a new object of class Obstacle2D. The function should be called getMinkowskiSum. The function should be a static member of the HW4 class. 

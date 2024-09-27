#include "CSpaceSkeleton.h"
#include "HW4Functions.h"
using namespace amp;
// Override this method for returning whether or not a point is in collision

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    // Note that the first cell at the minimum x0,x1 should be (0,0)
    const std::pair<double, double> x0_bounds= x0Bounds();
    const std::pair<double, double> x1_bounds= x1Bounds();
    std::pair<std::size_t, std::size_t> cells = DenseArray2D::size();
    const double grid_min_x0 = x0_bounds.first; // Minimum x value of the grid
    const double grid_max_x0 = x0_bounds.second; // Minimum x value of the grid
    const double grid_min_x1 = x1_bounds.first; // Minimum y value of the grid
    const double grid_max_x1 = x1_bounds.second; // Minimum y value of the grid
    const double cell_size_x0 = (grid_max_x0-grid_min_x0)/cells.first;     // Size of each cell in the x direction
    const double cell_size_x1 = (grid_max_x1-grid_min_x1)/cells.second;     // Size of each cell in the y direction
    // Calculate the cell indices
    int cell_x = static_cast<int>((x0 - grid_min_x0) / cell_size_x0);
    int cell_y = static_cast<int>((x1 - grid_min_x1) / cell_size_x1);
    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0,2*M_PI, 0, 2*M_PI);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    step_size = 2*M_PI/m_cells_per_dim;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    GenreateCSpace(cspace, manipulator, env);

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

void MyManipulatorCSConstructor::GenreateCSpace(MyGridCSpace2D& cspace, const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) { 
    amp::ManipulatorState manipulator_state(2);
    double theta1, theta2;
    for (std::size_t i = 0; i < m_cells_per_dim; i ++) {
        for (std::size_t j = 0; j < m_cells_per_dim; j ++) {
            theta1 =i*step_size;
            theta2 =j*step_size;
            manipulator_state << theta1, theta2;
            bool intersection_detected = false;
            for (int k = 0; k < manipulator.nLinks(); k++) {
                Eigen::Vector2d joint_location1 = manipulator.getJointLocation(manipulator_state, k);
                Eigen::Vector2d joint_location2 = manipulator.getJointLocation(manipulator_state, k + 1);
                bool intersection = HW4Functions::lineSegmentIntersection(env, joint_location1, joint_location2);
                if (intersection) {
                    intersection_detected = true;
                    break;
                }
            }
            if (intersection_detected) {
                cspace(i, j) = true;
            }
        }
    }
}

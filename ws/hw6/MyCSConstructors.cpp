#include "MyCSConstructors.h"

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


// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    double step_size_x = (env.x_max - env.x_min) / m_cells_per_dim;
    double step_size_y = (env.y_max - env.y_min) / m_cells_per_dim;
    // Do point in polygon check for each cell
    for (std::size_t i = 0; i < m_cells_per_dim; i ++) {
        for (std::size_t j = 0; j < m_cells_per_dim; j ++) {
            double x = env.x_min + i * step_size_x;
            double y = env.y_min + j * step_size_y;
            // find center of the cell 
            Eigen::Vector2d point(x + step_size_x/2, y + step_size_y/2);
            for (const auto& obstacle : env.obstacles) {
                if(point_in_polygon(point, obstacle)) {
                    cspace(i, j) = true;
                    break;
                }else{
                    cspace(i, j) = false;
                }
            }
        }
    }
    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}


bool MyPointAgentCSConstructor::point_in_polygon(Eigen::Vector2d point, const amp::Obstacle2D& obstacle){
    std::vector<Eigen::Vector2d> polygon = obstacle.verticesCW();
    int num_vertices = polygon.size();
    double x = point.x(), y = point.y();
    bool inside = false;
 
    // Store the first point in the polygon and initialize
    // the second point
    Eigen::Vector2d p1 = polygon[0], p2;
 
    // Loop through each edge in the polygon
    for (int i = 1; i <= num_vertices; i++) {
        // Get the next point in the polygon
        p2 = polygon[i % num_vertices];
 
        // Check if the point is above the minimum y
        // coordinate of the edge
        if (y > std::min(p1.y(), p2.y())) {
            // Check if the point is below the maximum y
            // coordinate of the edge
            if (y <= std::max(p1.y(), p2.y())) {
                // Check if the point is to the left of the
                // maximum x coordinate of the edge
                if (x <= std::max(p1.x(), p2.x())) {
                    // Calculate the x-intersection of the
                    // line connecting the point to the edge
                    double x_intersection
                        = (y - p1.y()) * (p2.x() - p1.x())
                              / (p2.y() - p1.y())
                          + p1.x();
 
                    // Check if the point is on the same
                    // line as the edge or to the left of
                    // the x-intersection
                    if (p1.x() == p2.x()
                        || x <= x_intersection) {
                        // Flip the inside flag
                        inside = !inside;
                    }
                }
            }
        }
 
        // Store the current point as the first point for
        // the next iteration
        p1 = p2;
    }
 
    // Return the value of the inside flag
    return inside;
}
 
 

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    // Given the initial and goal configurations, and the cspace, return a path
    amp::Path2D path;
    int count = 2; 
    path.waypoints.push_back(q_init);
    // Implement wavefront algorithm here
    // Define neighbors of cell using Moore neighborhood
    // For testing purposes print the neighbors of each cell
    std::vector<Eigen::Vector2d> neighbors = getNeighbors(Eigen::Vector2d(499, 499), grid_cspace);
    for (const auto& neighbor : neighbors) {
        std::cout << "Neighbor: " << neighbor << std::endl;
    }
    // For each cell in the grid, check its 8 neighbors (including diagonals)
    // If a neighbor is not in collision, add it to the wavefront
    // Continue until the goal is reached 
    // Use Breadth-First Search (BFS) to find the shortest path
    path.waypoints.push_back(q_goal);
    return path;
}

std::vector<Eigen::Vector2d> MyWaveFrontAlgorithm::getNeighbors(const Eigen::Vector2d& cell, const amp::GridCSpace2D& grid_cspace) {
    std::vector<Eigen::Vector2d> neighbors;
    Eigen::Vector2d neighbor;
    int row = cell[0];
    int col = cell[1];
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            if (i == 0 && j == 0) continue;
            
            int neighbor_row = (row + i + m_cells_per_dim) % m_cells_per_dim;
            int neighbor_col = (col + j + m_cells_per_dim) % m_cells_per_dim;
            
            neighbors.push_back(Eigen::Vector2d(neighbor_col, neighbor_row));
        }
    }
    return neighbors;
}       

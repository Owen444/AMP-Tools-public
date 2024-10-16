#include "MyCSConstructors.h"
#include <fstream>
#include <iomanip>
#include <iostream>
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
 
 

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Given the initial and goal configurations, and the cspace, return a path
    amp::Path2D path;
    Eigen::Vector2d q_init_manipulator;
    Eigen::Vector2d q_goal_manipulator;
    std::pair<std::size_t, std::size_t> q_goal_cell;
    std::pair<std::size_t, std::size_t> q_init_cell;
    int count = 0;
    // define a 2D array to store the cost to goal for each cell
    std::pair<std::size_t, std::size_t> grid_size= grid_cspace.size();
    int distance[grid_size.first][grid_size.second];
    // If manipulator Need to unwrap the angles to be within 0 and 2*pi
    if(isManipulator){
        q_init_manipulator.x() = wrapAngle(q_init.x());
        q_init_manipulator.y() = wrapAngle(q_init.y());
        q_goal_manipulator.x() = wrapAngle(q_goal.x());
        q_goal_manipulator.y() = wrapAngle(q_goal.y());
    }
    //saveGridCSpaceToCSV(grid_cspace, "/Users/owencraig/Desktop/grid_cspace.csv");
    // initialize the distance to goal for each cell
    for (std::size_t i = 0; i < grid_size.first; i ++) {
        for (std::size_t j = 0; j < grid_size.second; j ++) {
            if (grid_cspace(i, j)) {
                distance[i][j] = 1;
                // add custion around the obstacle to be 1
                std::vector<std::pair<int, int>> neighbors = getNeighborsManipulator(Eigen::Vector2d(i, j), grid_cspace);
                for (const auto& neighbor : neighbors) {
                    distance[neighbor.first][neighbor.second] = 1;
                }
            }else if(grid_cspace(i, j) == false || distance[i][j] != 1) {
                distance[i][j] = 0;
            }
        }
    }
    //saveDistanceGridToCSV(&distance[0][0], m_cells_per_dim, m_cells_per_dim, "/Users/owencraig/Desktop/distance_grid.csv");
    // Set goal to 2 
    if(isManipulator){
        q_goal_cell = grid_cspace.getCellFromPoint(q_goal_manipulator.x(), q_goal_manipulator.y());
        q_init_cell = grid_cspace.getCellFromPoint(wrapAngle(q_init_manipulator.x()), wrapAngle(q_init_manipulator.y()));
        distance[q_goal_cell.first][q_goal_cell.second] = 2;
    }else{
        q_goal_cell = grid_cspace.getCellFromPoint(q_goal.x(), q_goal.y());
        q_init_cell = grid_cspace.getCellFromPoint(q_init.x(), q_init.y());
        distance[q_goal_cell.first][q_goal_cell.second] = 2;
    }

   // Loop until the wavefront reaches the initial configuration
   if(isManipulator){
    count = 0;
        std::queue<std::pair<int, int>> wavefront;
        wavefront.push(q_goal_cell);
        while (!wavefront.empty()) {
            std::pair<int, int> cell = wavefront.front();
            wavefront.pop();
            std::vector<std::pair<int, int>> neighbors = getNeighborsManipulator(Eigen::Vector2d(cell.first, cell.second), grid_cspace);
            for (const auto& neighbor : neighbors) {
                if (distance[neighbor.first][neighbor.second] == 0) {
                    distance[neighbor.first][neighbor.second] = distance[cell.first][cell.second] + 1;
                    wavefront.push(neighbor);
                }
            }
        }
    }else{   
        count = 0;
        std::queue<std::pair<int, int>> wavefront;
        wavefront.push(q_goal_cell);
        while (!wavefront.empty()) {
            std::pair<int, int> cell = wavefront.front();
            wavefront.pop();
            std::vector<std::pair<int, int>> neighbors = getNeighborsPointAgent(Eigen::Vector2d(cell.first, cell.second), grid_cspace);
            for (const auto& neighbor : neighbors) {
                if (distance[neighbor.first][neighbor.second] == 0) {
                    distance[neighbor.first][neighbor.second] = distance[cell.first][cell.second] + 1;
                    wavefront.push(neighbor);
                }
            }
        }
   }
saveDistanceGridToCSV(&distance[0][0], m_cells_per_dim, m_cells_per_dim, "/Users/owencraig/Desktop/distance_grid.csv");
    // Follow lowest distance path 
    if(isManipulator){
        count = 0;
        path.waypoints.push_back(q_init_manipulator);
        std::pair<int, int> q_current = q_init_cell;
        //std::cout<<"q_init_cell: "<<q_init_cell.first<<","<<q_init_cell.second<<std::endl;
        while (q_current != q_goal_cell) {
            std::vector<std::pair<int, int>> neighbors = getNeighborsManipulator(Eigen::Vector2d(q_current.first, q_current.second), grid_cspace);
            for (const auto& neighbor : neighbors) {
                if (distance[neighbor.first][neighbor.second] < distance[q_current.first][q_current.second] && distance[neighbor.first][neighbor.second] != 1) {
                    q_current = neighbor;
                    Eigen::Vector2d q_current_point = getPointFromCell(Eigen::Vector2d(q_current.first, q_current.second), grid_cspace); 
                    // Wrap the angle to be within 0 and 2*pi
                    q_current_point.x() = wrapAngle(q_current_point.x());
                    q_current_point.y() = wrapAngle(q_current_point.y());
                    path.waypoints.push_back(q_current_point);
                    break;
                }
            }
            //create break condition for while loop
            if (count > 1000) {
                std::cout << "Wavefront algorithm failed to find a path: Manipulator" << std::endl;
                break;
            }
            count++;
        }
        path.waypoints.push_back(q_goal_manipulator);
        //std::cout<<"Q_goal_manipulator: "<<q_goal_manipulator.x()<<","<<q_goal_manipulator.y()<<std::endl;
    }else{
        count = 0;
        path.waypoints.push_back(q_init);
        std::pair<int, int> q_current = q_init_cell ;
        while (q_current != q_goal_cell) {
            int min_distance = distance[q_current.first][q_current.second];
            std::vector<std::pair<int, int>> neighbors = getNeighborsPointAgent(Eigen::Vector2d(q_current.first, q_current.second), grid_cspace);
            for (const auto& neighbor : neighbors) {
                //Move to the neighbor with the lowest distance
                if (distance[neighbor.first][neighbor.second] < min_distance && distance[neighbor.first][neighbor.second] != 1) {
                    q_current = neighbor;
                    Eigen::Vector2d q_current_point = getPointFromCell(Eigen::Vector2d(q_current.first, q_current.second), grid_cspace); 
                    path.waypoints.push_back(q_current_point);
                    break;
                }
            }
            //create break condition for while loop
            if (count > 1000) {
                std::cout << "Wavefront algorithm failed to find a path: Point Agent" << std::endl;
                break;
            }
            count++;
        }
        path.waypoints.push_back(q_goal);
    }

    //For testing purposes print the neighbors of each cell
    // std::vector<Eigen::Vector2d> neighbors = getNeighborsPointAgent(Eigen::Vector2d(499, 499), grid_cspace);
    // for (const auto& neighbor : neighbors) {
    //     std::cout << "Neighbor: " << neighbor << std::endl;
    // }

    // Unwrap the waypoints if it is a manipulator
    // Comment out for plotting
    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}

std::vector<std::pair<int, int>> MyWaveFrontAlgorithm::getNeighborsManipulator(const Eigen::Vector2d& cell, const amp::GridCSpace2D& grid_cspace) {
    std::vector<std::pair<int, int>> neighbors;
    int row = static_cast<int>(cell[0]);
    int col = static_cast<int>(cell[1]);

    // Define the four directions: up, down, left, right
    const std::vector<std::pair<int, int>> directions = {
        {-1, 0},  // up
        {1, 0},   // down
        {0, -1},  // left
        {0, 1}    // right
    };

    for (const auto& [dx, dy] : directions) {
        int neighbor_row = (row + dx + m_cells_per_dim) % m_cells_per_dim;
        int neighbor_col = (col + dy + m_cells_per_dim) % m_cells_per_dim;
        
        neighbors.emplace_back(neighbor_row, neighbor_col);
    }

    return neighbors;
}

std::vector<std::pair<int, int>> MyWaveFrontAlgorithm::getNeighborsPointAgent(const Eigen::Vector2d& cell, const amp::GridCSpace2D& grid_cspace) {
    std::vector<std::pair<int, int>> neighbors;
    int row = static_cast<int>(cell[0]);
    int col = static_cast<int>(cell[1]);

    // Get the dimensions of the grid
    std::pair<std::size_t, std::size_t> grid_size = grid_cspace.size();
    int rows = static_cast<int>(grid_size.first);
    int cols = static_cast<int>(grid_size.second);

    // Define the four directions: up, down, left, right
    const std::vector<std::pair<int, int>> directions = {
        {-1, 0},  // up
        {1, 0},   // down
        {0, -1},  // left
        {0, 1}    // right
    };

    for (const auto& [dx, dy] : directions) {
        int neighbor_row = row + dx;
        int neighbor_col = col + dy;  
        // Check if the neighbor is within the grid boundaries
        if (neighbor_row >= 0 && neighbor_row < rows && neighbor_col >= 0 && neighbor_col < cols) {
            neighbors.emplace_back(neighbor_row, neighbor_col);
        }
    }

    return neighbors;
}

Eigen::Vector2d MyWaveFrontAlgorithm::getPointFromCell(const Eigen::Vector2d& cell, const amp::GridCSpace2D& grid_cspace) {
    std::pair<double, double> x0_bounds = grid_cspace.x0Bounds();
    std::pair<double, double> x1_bounds = grid_cspace.x1Bounds();
    std::pair<std::size_t, std::size_t> grid_size = grid_cspace.size();
    double x_min = x0_bounds.first;
    double x_max = x0_bounds.second;
    double y_min = x1_bounds.first;
    double y_max = x1_bounds.second;
    double x_step = (x_max - x_min) / grid_size.first;
    double y_step = (y_max - y_min) / grid_size.second;
    double x = x_min + (cell[0]+0.5) * x_step;
    double y = y_min + (cell[1]+0.5) * y_step;
    return Eigen::Vector2d(x, y);
}



void MyWaveFrontAlgorithm::saveDistanceGridToCSV(const int* distance, std::size_t rows, std::size_t cols, const std::string& filename) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            file << distance[i * cols + j];
            if (j < cols - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
    std::cout << "Distance grid saved to " << filename << std::endl;
}


void MyWaveFrontAlgorithm::saveGridCSpaceToCSV(const amp::GridCSpace2D& grid_cspace, const std::string& filename) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    std::pair<std::size_t, std::size_t> size = grid_cspace.size();
    std::size_t rows = size.first;
    std::size_t cols = size.second;

    for (std::size_t i = 0; i < rows; ++i) {
        for (std::size_t j = 0; j < cols; ++j) {
            file << (grid_cspace(i, j) ? "T" : "F");
            if (j < cols - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
    std::cout << "Grid CSpace saved to " << filename << std::endl;
}

double MyWaveFrontAlgorithm::wrapAngle(double angle) {
        angle = std::fmod(angle, 2 * M_PI);
        if (angle < 0) {
            angle += 2 * M_PI;
        }
    return angle;
}
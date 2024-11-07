#include "MyKinoRRT.h"
using namespace amp;
void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    // Create path, node map and graph
    amp::KinoPath path;
    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::map<amp::Node, amp::Node> parent_map;
    std::map<amp::Node, Eigen::VectorXd> control_map;
    // Initialize the first node
    nodes[0] = problem.q_init;
    // Define constants
    int n = m_numSamples;
    double radius = m_connectionRadius;
    double p_goal = 0.1;
    double dt = 0.25;
    int iterations = 0;
    int max_iterations = 1000;
    // Initialize Random Number Generator to sample (x,y) coordinates
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x(problem.x_min, problem.x_max);
    std::uniform_real_distribution<double> y(problem.y_min, problem.y_max);
    Eigen::Vector2d q_rand;
    // While solution is not found and number of samples is less than n
    while (true){
        // Sample a random configuration
        std::uniform_real_distribution<double> random_number(0,1);
        if(random_number(gen)<p_goal){
            q_rand << (problem.q_goal[0].first+problem.q_goal[0].second)/2, (problem.q_goal[1].first+problem.q_goal[1].second)/2;
        }
        else{
            q_rand << x(gen), y(gen);
        }
        // Find nearest node to q_rand
        double distance_min = std::numeric_limits<double>::max();
        amp::Node nearest_node;
        for (const auto& [node, q] : nodes) {
            double dist2node = (q - q_rand).norm();
            if (dist2node < distance_min) {
                distance_min = dist2node;
                nearest_node = node;
            }
        }
        Eigen::VectorXd q_near = nodes[nearest_node];
        auto [q_new, control] = GenerateLocalTrajectory(problem, agent, q_near, q_rand,dt);
        std::pair<bool, Eigen::VectorXd> valid = IsSubTrajectoryValid(problem, agent, q_near, control, dt);
        q_new << valid.second;
        if(valid.first){
            nodes[nodes.size()] = q_new;
            parent_map[nodes.size()-1] = nearest_node;
            control_map[nodes.size()-1] = control;
        }
        // Check if the new node is in the goal region
        bool in_goal_region = true;
        for (int i = 0; i < problem.q_goal.size(); i++) {
            if (q_new(i) < problem.q_goal[i].first || q_new(i) > problem.q_goal[i].second) {
                in_goal_region = false;
                break;
            }
        }
        if(in_goal_region){
            std::cout<<"Goal reached"<<std::endl;
            path.valid = true;
            break;
        }
        // Check if the iterations is greater than max_iterations
        if(iterations>max_iterations){
            std::cout<<"Max iterations reached"<<std::endl;
            break;
        }
        iterations++;
        //std::cout<<"Iteration: "<<iterations<<std::endl;
    }
    // Create the path
    amp::Node current_node = nodes.size()-1;
    Eigen::VectorXd current_state = nodes[current_node];
    Eigen::VectorXd current_control = control_map[current_node];
    while(current_node != 0){
        path.waypoints.push_back(current_state);
        path.controls.push_back(current_control);
        path.durations.push_back(dt);
        current_node = parent_map[current_node];
        current_state = nodes[current_node];
        current_control = control_map[current_node];
    }
    path.waypoints.push_back(problem.q_init);
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    return path;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> MyKinoRRT::GenerateLocalTrajectory(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, Eigen::VectorXd q_near,Eigen::VectorXd q_rand,double dt) {
    double min_distance = std::numeric_limits<double>::max();
    Eigen::VectorXd q_new, best_control;
    for (int i = 0; i < m_numSamples; i++) {
        Eigen::VectorXd control = Eigen::VectorXd::Random(problem.q_init.size());
        for (int j = 0; j < control.size(); j++) {
            control(j) =  ((problem.u_bounds[j].second - problem.u_bounds[j].first)/2)*control(j)  + ((problem.u_bounds[j].second + problem.u_bounds[j].first)/2);
            //std::cout<<"Control "<<j<<": "<<control(j)<<std::endl;
        }
        Eigen::VectorXd state = q_near;
        agent.propagate(state, control, dt);
        double distance = (q_rand - state).norm();
        if(distance < min_distance){
            min_distance = distance;
            q_new = state;
            best_control = control;
        }
    }
    return {q_new, best_control};
}

std::pair<bool, Eigen::VectorXd> MyKinoRRT::IsSubTrajectoryValid(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, Eigen::VectorXd q_near, Eigen::VectorXd control, double dt) {
    Eigen::VectorXd q_step = q_near;
    int steps = 1000;
    for (int i = 0; i < steps; i++) {
        agent.propagate(q_step, control, dt/steps);
        bool inside_obstacle = false;
        for (const auto& obstacle : problem.obstacles) {
            inside_obstacle = point_in_polygon(q_step, obstacle);
            Eigen::Vector2d min_point = MinimumPoint(q_step, obstacle);
            if (inside_obstacle || (min_point-q_step).norm() < 0.25) {
                return {false, q_step};
            }
        }
        bool line_collision = HW4Functions::lineSegmentIntersection(problem, q_near, q_step);
        if(line_collision) {
            return {false, q_step};
        }
    }
    return {true, q_step};
}

bool point_in_polygon(Eigen::Vector2d point, const amp::Obstacle2D& obstacle){
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

Eigen::Vector2d MinimumPoint(Eigen::Vector2d q, const amp::Obstacle2D& obstacle){
    double min_distance_to_obstacle = std::numeric_limits<double>::max();
    Eigen::Vector2d min_point;
    Eigen::Vector2d vertex1;
    Eigen::Vector2d vertex2;
    Eigen::Vector2d AB;
    Eigen::Vector2d AQ;
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCW();
    for (int i = 0; i < vertices.size(); i++) {
        vertex1 = vertices[i];
        vertex2 = vertices[(i + 1) % vertices.size()];
        AB = vertex2 - vertex1;
        AQ = q - vertex1;
        double projection = (AQ.dot(AB) / AB.dot(AB));
        projection = std::max(0.0, std::min(1.0, projection));
        Eigen::Vector2d point = vertex1 + projection * AB;
        double distance = (q - point).norm();
        if (distance < min_distance_to_obstacle) {
            min_distance_to_obstacle = distance;
            min_point = point;
        }
    }
    return min_point;
}
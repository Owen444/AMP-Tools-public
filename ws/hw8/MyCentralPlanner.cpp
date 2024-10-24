#include "MyMultiAgentPlanners.h"
#include <algorithm>
amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
    }
    //path = MultiAgentRRT(problem);
    return path;
}

amp::MultiAgentProblem2D MyCentralPlanner::expand_obstacles(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentProblem2D expanded_problem = problem;
    for (amp::Obstacle2D& obstacle : expanded_problem.obstacles) {
        double radius = problem.agent_properties[0].radius; 
        std::vector<Eigen::Vector2d> obstacle_verticies = obstacle.verticesCCW();
        // Calculate the centroid of the obstacle  
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (const Eigen::Vector2d& vertex : obstacle_verticies) {
            centroid += vertex;
        }
        centroid /= obstacle_verticies.size();
        // Expand the obstacle by the radius
        for (Eigen::Vector2d& vertex : obstacle_verticies) {
            vertex = (vertex - centroid)+(radius * centroid.normalized());
        }
        obstacle = amp::Obstacle2D(obstacle_verticies);
    }
    return expanded_problem;
}

amp::MultiAgentPath2D MyCentralPlanner::MultiAgentRRT(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentProblem2D expanded_problem = expand_obstacles(problem);
    amp::MultiAgentPath2D path;
    // Define the graph and nodes
    graphPtr = std::make_shared<amp::Graph<double>>();
    MyAStarAlgo::GraphSearchResult astar_result;
    node_maps = std::vector<std::map<amp::Node, Eigen::Vector2d>>(problem.numAgents());
    std::vector<std::map<amp::Node, amp::Node>> parent_maps = std::vector<std::map<amp::Node, amp::Node>>(problem.numAgents());
    std::vector<Eigen::Vector2d> nearest_node_locations = std::vector<Eigen::Vector2d>(problem.numAgents());
    std::vector<amp::Node> nearest_nodes = std::vector<amp::Node>(problem.numAgents());
    std::vector<Eigen::Vector2d> q_step = std::vector<Eigen::Vector2d>(problem.numAgents());
    for (int i = 0; i < problem.numAgents(); i++) {
        node_maps[i][0] = expanded_problem.agent_properties[i].q_init;
        node_maps[i][1] = expanded_problem.agent_properties[i].q_goal;
        nearest_node_locations[i] = expanded_problem.agent_properties[i].q_init;
        nearest_nodes[i] = 0;
    }  
    // Define the parameters
    double step_size = 1;
    double episilon_goal = 0.25;
    double p_goal = 0.05;
    std::random_device rd;
    std::mt19937 gen(rd());
    Eigen::Vector2d random_node;
    std::vector<Eigen::Vector2d> random_nodes;
    std::vector<Eigen::Vector2d> near_nodes;
    int max_iter = 10000;
    int iteration = 0;
    // Run the RRT algorithm
    while(true){
        for (int i = 0; i < problem.numAgents(); i++) {
            // Generate a random node in free space
            // generate a random number between 0 and 1
            std::uniform_real_distribution<double> random_number(0,1);
            if(random_number(gen) < p_goal){
                random_node = expanded_problem.agent_properties[i].q_goal;
            }else{
                std::uniform_real_distribution<double> x(expanded_problem.x_min,expanded_problem.x_max);
                std::uniform_real_distribution<double> y(expanded_problem.y_min,expanded_problem.y_max);
                random_node = Eigen::Vector2d(x(gen),y(gen));
            }
            random_nodes[i] = random_node;
            // Find the nearest node
            double nearest_node_distance = std::numeric_limits<double>::max();
            amp::Node nearest_node = 0;
            Eigen::Vector2d nearest_node_location = node_maps[i][0];
            // Take a step from the nearest node towards the random node defined by the step size
            for (const auto& [node, point] : node_maps[i]) {
            if ((point - random_node).norm() <= nearest_node_distance && node != 1) {
                    nearest_node = node;
                    nearest_node_location = point;
                    nearest_node_distance = (point - random_node).norm();
                }
            }
            nearest_nodes[i] = nearest_node;
            nearest_node_locations[i] = nearest_node_location;
            Eigen::Vector2d step = (random_node - nearest_node_location);
            q_step[i] = nearest_node_location + step_size * step.normalized();
        }
        int goal_count = 0;
        if(isSubpathCollisionFree(nearest_node_locations, q_step, expanded_problem)){
            for (int i = 0; i < problem.numAgents(); i++){
                if(node_maps[i][node_maps[i].size()-1] == expanded_problem.agent_properties[i].q_goal&& iteration !=0){
                    std::cout << "Goal Found for Agent " << i << std::endl;
                    goal_count++;
                    continue;
                }
                if((q_step[i]-expanded_problem.agent_properties[i].q_goal).norm() < episilon_goal){
                    node_maps[i][node_maps[i].size()] = expanded_problem.agent_properties[i].q_goal;
                    parent_maps[i][node_maps[i].size()] = nearest_nodes[i];
                }else{
                    node_maps[i][node_maps[i].size()] = q_step[i];
                    parent_maps[i][node_maps[i].size()] = nearest_nodes[i];
                }
            }
        }
        if(goal_count == expanded_problem.numAgents()){
            std::cout << "All Goals Found" << std::endl;
            break;
        }
        iteration++;
        if(iteration > max_iter){
            std::cout << "Max Iterations Reached" << std::endl;
            break;
        }
    }
    // Use parent map to create the path
    for(int i = 0; i< expanded_problem.numAgents();i++){
        amp::Path2D agent_path;
        amp::Node current_node = 1;
        while(current_node != 0){
            agent_path.waypoints.push_back(node_maps[i][current_node]);
            current_node = parent_maps[i][current_node];
        }
        agent_path.waypoints.push_back(expanded_problem.agent_properties[i].q_init);
        std::reverse(agent_path.waypoints.begin(), agent_path.waypoints.end());
        path.agent_paths.push_back(agent_path);
    }
    // output size of the path
    //std::cout << "Path Size: " << path.numAgents() << std::endl;
    return path;
}

bool MyCentralPlanner::isSubpathCollisionFree(std::vector<Eigen::Vector2d> nearest_node_locations, std::vector<Eigen::Vector2d> q_steps, const amp::MultiAgentProblem2D& problem){
    for (int i = 0; i < nearest_node_locations.size(); i++){
        bool Obstacle_Intersection =  HW4Functions::lineSegmentIntersection(problem, nearest_node_locations[i], q_steps[i]);
        for (int j = 0; j < nearest_node_locations.size(); j++){    
            if(i != j){
                bool Agent_Intersection = AgentIntersection(nearest_node_locations[i], q_steps[i], nearest_node_locations[j], q_steps[j]);
                if(Obstacle_Intersection || Agent_Intersection){
                    return false;
                }
            }
        }
    }
    return true;
}
bool MyCentralPlanner::AgentIntersection(Eigen::Vector2d Q1, Eigen::Vector2d Q2, Eigen::Vector2d P1, Eigen::Vector2d P2){
    if(Intersect(P1,P2,Q1,Q2)){
        return true;
    }
    return false;
}
// Helper function to check the orientation of the triplet (p, q, r)
int  MyCentralPlanner::orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
   // Calculate the determinant of the matrix formed by the vectors pq and qr
   double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
  
   if (val == 0) return 0; // collinear
   return (val > 0) ? 1 : 2; // clock or counterclock wise
}
// Check if point q lies on segment pr
bool  MyCentralPlanner::onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
   return q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
          q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y());
}
// Main function to check if line segments (p1, q1) and (p2, q2) intersect
bool  MyCentralPlanner::Intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2) {
   // Find the four orientations needed for the general and special cases
   int o1 = orientation(p1, q1, p2);
   int o2 = orientation(p1, q1, q2);
   int o3 = orientation(p2, q2, p1);
   int o4 = orientation(p2, q2, q1);
  
   // General case: different orientations indicate intersection
   if (o1 != o2 && o3 != o4)
       return true;
  
   // Special Cases: when points are collinear and lie on the segment
   if (o1 == 0 && onSegment(p1, p2, q1)) return true;
   if (o2 == 0 && onSegment(p1, q2, q1)) return true;
   if (o3 == 0 && onSegment(p2, p1, q2)) return true;
   if (o4 == 0 && onSegment(p2, q1, q2)) return true;
  
   return false; // No intersection
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
    }
    return path;
}

// point in polygon
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
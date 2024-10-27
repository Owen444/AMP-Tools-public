#include "MyMultiAgentPlanners.h"
#include <algorithm>
amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    path = MultiAgentRRT(problem);
    return path;
}

amp::MultiAgentProblem2D MyCentralPlanner::expand_obstacles(const amp::MultiAgentProblem2D& problem) {
    double max_radius = 0;
    for(const auto& agent : problem.agent_properties){
       //find the max radius
       if(agent.radius > max_radius){
           max_radius = agent.radius;
       }
    }
   
    double radius = 1.2*max_radius;
    amp::MultiAgentProblem2D expanded_problem = problem;
    std::vector<amp::Obstacle2D> new_obstacles;
    std::vector<amp::Obstacle2D> obstacles = problem.obstacles;
    for (const auto& obstacle : obstacles){
        std::vector<Eigen::Vector2d> new_vertices;
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (const auto& vertex : obstacle.verticesCW()){
            centroid += vertex;
        }
        centroid /= obstacle.verticesCW().size();
        for (const auto& vertex : obstacle.verticesCW()){
            Eigen::Vector2d direction;
            for(int k = 0;k<2;k++){
                if(vertex[k]>centroid[k]){
                    direction[k] = 1;
                }else{
                    direction[k] = -1;
                }
            }
            Eigen::Vector2d new_vertex = vertex + radius*direction;
            new_vertices.push_back(new_vertex);
        }
        amp::Obstacle2D new_obstacle(new_vertices);
        new_obstacles.push_back(new_obstacle);
    }
    expanded_problem.obstacles = new_obstacles;
    return expanded_problem;
}

amp::MultiAgentPath2D MyCentralPlanner::MultiAgentRRT(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    tree_size = 0;
    path_success = false;
    // Define the parameters
    double step_size = 0.5;
    double episilon_goal = 0.25;
    double p_goal = 0.05;
    std::random_device rd;
    std::mt19937 gen(rd());
    int max_iter = 7500;
    int iteration = 0;
    
    // Define the meta agent node map 
    std::map<amp::Node, Eigen::VectorXd> meta_node_map;
    std::map<amp::Node, amp::Node> parent_map;
    amp::Node meta_node = 0;
    Eigen::VectorXd initial_meta_agent(problem.numAgents()*2);
    Eigen::VectorXd goal_meta_agent(problem.numAgents()*2);
    // Define the random, step, and nearest meta agent
    Eigen::VectorXd random_meta_agent(problem.numAgents()*2);
    Eigen::VectorXd nearest_meta_agent(problem.numAgents()*2);
    Eigen::VectorXd step_meta_agent(problem.numAgents()*2);
    std::vector<bool> goal_found(problem.numAgents(), false);
    // Define the initial and goal meta agent
    int i = 0;
    for (int l = 0; l < problem.numAgents(); l++){
        initial_meta_agent[i] = problem.agent_properties[i/2].q_init.x();
        initial_meta_agent[i+1] = problem.agent_properties[i/2].q_init.y();
        goal_meta_agent[i] = problem.agent_properties[i/2].q_goal.x();
        goal_meta_agent[i+1] = problem.agent_properties[i/2].q_goal.y();
        i+=2;
    }
    meta_node_map[meta_node] = initial_meta_agent;

    // Run the RRT algorithm
    std::uniform_real_distribution<double> random_number(0,1);
    std::uniform_real_distribution<double> x(problem.x_min,problem.x_max);
    std::uniform_real_distribution<double> y(problem.y_min,problem.y_max);
    while(true){
        i = 0;
        for(int l = 0; l < problem.numAgents(); l++) {
            Eigen::Vector2d agent_location(meta_node_map[meta_node_map.size()-1][i], meta_node_map[meta_node_map.size()-1][i+1]);
            Eigen::Vector2d agent_goal(goal_meta_agent[i], goal_meta_agent[i+1]);
            if((agent_location - agent_goal).norm() <= episilon_goal){
                goal_found[l] = true;
                random_meta_agent[i] = goal_meta_agent[i];
                random_meta_agent[i+1] = goal_meta_agent[i+1];
                //std::cout << "Goal Found for Agent: " << l << "On Iteration: " << iteration << std::endl;
            }else{
                // Generate a random node in free space
                // generate a random number between 0 and 1
                if(random_number(gen) < p_goal){
                    random_meta_agent[i] = goal_meta_agent[i];
                    random_meta_agent[i+1] = goal_meta_agent[i+1];
                }else{
                    random_meta_agent[i] = x(gen);
                    random_meta_agent[i+1] = y(gen);
                }
            }
            i+=2;
        }
        //std::cout << "Random Meta Agent: " << random_meta_agent << std::endl;
        // Find the Nearest Node for the entire meta agent 
        double min_distance = std::numeric_limits<double>::max();
        amp::Node nearest_node = 0;
        Eigen::VectorXd nearest_node_location(problem.numAgents()*2);
        for (const auto& [node, point] : meta_node_map){
            if((point - random_meta_agent).norm() <= min_distance){
                nearest_node = node;   
                nearest_node_location = point;
                min_distance = (point - random_meta_agent).norm();
            }
        }
        // Take a step towards the random node
        Eigen::VectorXd step_meta_agent(problem.numAgents()*2);
        i = 0;
        for (int l = 0; l < problem.numAgents(); l++){
            if(goal_found[l]){
                step_meta_agent[i] = goal_meta_agent[i];
                step_meta_agent[i+1] = goal_meta_agent[i+1];
            }else{
                Eigen::Vector2d agent_q_random(random_meta_agent[i], random_meta_agent[i+1]);
                Eigen::Vector2d agent_q_nearest(nearest_node_location[i], nearest_node_location[i+1]);
                Eigen::Vector2d step_vec = agent_q_nearest + step_size * (agent_q_random - agent_q_nearest).normalized();
                step_meta_agent[i] = step_vec.x();
                step_meta_agent[i+1] = step_vec.y();
            }
            i+=2;
        }
        // std::cout << "Nearest Node Location: " << nearest_node_location << std::endl;
        // std::cout << "Step Meta Agent: " << step_meta_agent << std::endl;

        if(isSubpathCollisionFree(nearest_node_location, step_meta_agent, problem)){
            amp::Node new_node = meta_node_map.size();
            if((step_meta_agent - goal_meta_agent).norm() <= episilon_goal){
                std::cout << "Goal Found for Meta Agent" << std::endl;
                path_success = true;
                meta_node_map[new_node] = goal_meta_agent;
                parent_map[new_node] = nearest_node;
                break;
            }else{
                meta_node_map[new_node] = step_meta_agent;
                parent_map[new_node] = nearest_node;
                //std::cout << "New Node: " << step_meta_agent << std::endl;
            }
        }else{
            //std::cout << "Collision Detected" << std::endl;
        }
        //std::cout << "Meta Agent Goal Distance: " << (meta_node_map[meta_node_map.size()-1] - goal_meta_agent).norm() << std::endl;
        
        if(iteration >= max_iter){
            std::cout << "Max Iterations Reached" << std::endl;
            amp::MultiAgentPath2D path;
            for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
                amp::Path2D agent_path;
                agent_path.waypoints = {agent.q_init, agent.q_goal};
                path.agent_paths.push_back(agent_path);
            }
            return path;
        }
        //std::cout << "Iteration: " << iteration <<std::endl;
        iteration++;
    }

    // Use parent map to create the path
    i = 0;
    for(int l = 0; l < problem.numAgents();l++){
        amp::Path2D agent_path;
        amp::Node current_node = meta_node_map.size()-1;
        Eigen::VectorXd current_meta_agent(meta_node_map[current_node]);
        Eigen::Vector2d agent_q_current(current_meta_agent[i], current_meta_agent[i+1]);
        //agent_path.waypoints.push_back(expanded_problem.agent_properties[i/2].q_goal);
        while(current_node != 0){
            agent_path.waypoints.push_back(agent_q_current);
            current_node = parent_map[current_node];
            current_meta_agent = meta_node_map[current_node];
            agent_q_current << current_meta_agent[i], current_meta_agent[i+1];
        }
        agent_path.waypoints.push_back(problem.agent_properties[i/2].q_init);
        std::reverse(agent_path.waypoints.begin(), agent_path.waypoints.end());
        //std::cout << "Agent Path Size: " << agent_path.waypoints.size() << std::endl;
        path.agent_paths.push_back(agent_path);
        i+=2;
    }
    tree_size = meta_node_map.size();
    return path;
}

bool MyCentralPlanner::isSubpathCollisionFree(Eigen::VectorXd nearest_node_locations, Eigen::VectorXd step_meta_agent, const amp::MultiAgentProblem2D& problem){
    // take k number of steps between nearest node locations and q_steps
    int k = 100;
    for(int i = 0; i < k; i++){
        Eigen::VectorXd meta_agent_k = nearest_node_locations + i*(step_meta_agent - nearest_node_locations)/k;
        if(!isSystemValid(problem, nearest_node_locations, meta_agent_k)){
            return false;
        }
    }
    return true;
}   

bool MyCentralPlanner::isSystemValid(amp::MultiAgentProblem2D problem, Eigen::VectorXd nearest_node_locations, Eigen::VectorXd meta_agent_k){
    int i =0;
    for(const auto& agent : problem.agent_properties){
        Eigen::Vector2d agent_q_k(meta_agent_k[i], meta_agent_k[i+1]);
        Eigen::Vector2d agent_q_nearest(nearest_node_locations[i], nearest_node_locations[i+1]);
        for(const auto& obstacle : problem.obstacles){
            Eigen::Vector2d min_point = MinimumPoint(agent_q_k, obstacle);
            if((agent_q_k - min_point).norm() < 1.5*agent.radius){
                return false;
            }
        }
        int j = 0;
        for(const auto& other_agent : problem.agent_properties){
            if(i != j){
                Eigen::Vector2d other_agent_q_k(meta_agent_k[j], meta_agent_k[j+1]);
                bool Agent_Intersection = AgentIntersection(problem, agent_q_k, other_agent_q_k);
                if(Agent_Intersection){
                    //std::cout << "Agent Intersection: " << Agent_Intersection << std::endl;
                    // std::cout << "Agent q_1: " << agent_q_k << std::endl;
                    // std::cout << "Agent q_2: " << other_agent_q_k << std::endl;
                    return false;
                }
            }
            j+=2;
        }
        i+=2;
    }
    return true;
}

bool MyCentralPlanner::AgentIntersection(amp::MultiAgentProblem2D problem, Eigen::Vector2d P1, Eigen::Vector2d P2){
    double max_radius = 0;
    for(const auto& agent : problem.agent_properties){
       //find the max radius
       if(agent.radius > max_radius){
           max_radius = agent.radius;
       }
    }
   
    double radius = max_radius;
    double distance = (P1-P2).norm();
    if(distance <= 2*radius){
        return true;
    }
    return false;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    std::cout << "Number of Agents: " << problem.numAgents() << std::endl;
    int iteration = 0;
    int max_iter = 10000;
    bool collision;
    amp::Path2D agent_path = MultiAgentRRT(problem, problem.agent_properties[0].q_init, problem.agent_properties[0].q_goal,0);
    path.agent_paths.push_back(agent_path);
    while(true){
        if(path.agent_paths.size() == problem.numAgents()){
            std::cout << "All Agents Planned" << std::endl;
            return path;
        }
        collision = false;
        path_success = false;
        int count = 0;
        while(!path_success){
            agent_path = MultiAgentRRT(problem,problem.agent_properties[path.agent_paths.size()].q_init, problem.agent_properties[path.agent_paths.size()].q_goal,path.agent_paths.size());
            count++;
            if(count>20){
                agent_path.waypoints = {problem.agent_properties[path.agent_paths.size()].q_init, problem.agent_properties[path.agent_paths.size()].q_goal};
                std::cout << "Path Failed" << std::endl;
                break;
            }
        }
        for(int i = 0; i<path.agent_paths.size();i++){
            int max_waypoints = std::max(agent_path.waypoints.size(), path.agent_paths[i].waypoints.size());
            for(int j = 0; j < max_waypoints; j++){
                Eigen::Vector2d agent1;
                Eigen::Vector2d agent2;
                if(j < agent_path.waypoints.size()){
                   agent1 = agent_path.waypoints[j];
                }else{
                    agent1 = agent_path.waypoints[agent_path.waypoints.size()-1];
                }
                if(j < path.agent_paths[i].waypoints.size()){
                    agent2 = path.agent_paths[i].waypoints[j];
                }else{
                    agent2 = path.agent_paths[i].waypoints[path.agent_paths[i].waypoints.size()-1];
                }
                double distance = (agent2 - agent1).norm();
                double collision_distance = (problem.agent_properties[i].radius+problem.agent_properties[i+1].radius);
                //std::cout<<"Agent 1: "<<agent1<<" Agent 2: "<<agent2<<std::endl;
                //std::cout << "Distance: " << distance << " Collision Distance: " << collision_distance << std::endl;
                if(distance < collision_distance){
                    collision = true;
                    break;
                }
            }
            if(collision){
                break;
            }
        }
        if(!collision){
            path.agent_paths.push_back(agent_path);
            //std::cout<<"Collision Free Path Found"<<std::endl;
        }else{
            //std::cout<<"Collision Detected"<<std::endl;
        }
        if(iteration >= max_iter){
           amp::MultiAgentPath2D path_fix;
           std::cout << "Max Iterations Reached Decentral" << std::endl;
           for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
               amp::Path2D agent_path;
               agent_path.waypoints = {agent.q_init, agent.q_goal};
               path_fix.agent_paths.push_back(agent_path);
           }
           return path_fix;
        }
        iteration++;
    }
    //std::cout << "Decentralized Iteration: " << iteration << std::endl;
    return path;
}

amp::MultiAgentProblem2D MyDecentralPlanner::expand_obstacles(const amp::MultiAgentProblem2D& problem) {
    double max_radius = 0;
    for(const auto& agent : problem.agent_properties){
       //find the max radius
       if(agent.radius > max_radius){
           max_radius = agent.radius;
       }
    }
   
    double radius = 1.2*max_radius;
    amp::MultiAgentProblem2D expanded_problem = problem;
    std::vector<amp::Obstacle2D> new_obstacles;
    std::vector<amp::Obstacle2D> obstacles = problem.obstacles;
    for (const auto& obstacle : obstacles){
        std::vector<Eigen::Vector2d> new_vertices;
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (const auto& vertex : obstacle.verticesCW()){
            centroid += vertex;
        }
        centroid /= obstacle.verticesCW().size();
        for (const auto& vertex : obstacle.verticesCW()){
            Eigen::Vector2d direction;
            for(int k = 0;k<2;k++){
                if(vertex[k]>centroid[k]){
                    direction[k] = 1;
                }else{
                    direction[k] = -1;
                }
            }
            Eigen::Vector2d new_vertex = vertex + radius*direction;
            new_vertices.push_back(new_vertex);
        }
        amp::Obstacle2D new_obstacle(new_vertices);
        new_obstacles.push_back(new_obstacle);
    }
    expanded_problem.obstacles = new_obstacles;
    return expanded_problem;
}

amp::Path2D MyDecentralPlanner::MultiAgentRRT(amp::MultiAgentProblem2D problem, Eigen::Vector2d q_init, Eigen::Vector2d q_goal,int agent_index){
    amp::MultiAgentProblem2D expanded_problem = expand_obstacles(problem);
    amp::Path2D path;
    // Create maps for nodes
    std::map<amp::Node, Eigen::Vector2d> nodes;
    std::map<amp::Node, amp::Node> parent_map;
    // Define Constants and Initial node
    nodes[0] = q_init;
    Eigen::Vector2d nearest_node_location = nodes[0];
    double step_size = 0.5;
    double episilon_goal = 0.25;
    double p_goal = 0.05;
    int max_iter = 5000;
    int iteration = 0;
    // Define the random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    Eigen::Vector2d random_node;
    path_success = false;
    while(true){
        while(true){
            // Generate a random node in free space
            // generate a random number between 0 and 1
            std::uniform_real_distribution<double> random_number(0,1);
            if(random_number(gen) < p_goal){
                random_node = q_goal;
            }else{
                std::uniform_real_distribution<double> x(problem.x_min,problem.x_max);
                std::uniform_real_distribution<double> y(problem.y_min,problem.y_max);
                random_node = Eigen::Vector2d(x(gen),y(gen));
            }
            // Check if the node is in free space
            bool in_free_space = true;
            for (const auto& obstacle : problem.obstacles) {
                Eigen::Vector2d min_point = MinimumPoint(random_node, obstacle);
                if ((random_node - min_point).norm() < 2.0*problem.agent_properties[agent_index].radius) {
                    in_free_space = false;
                    break;
                }
            }
            if(in_free_space){
                //std::cout<<"Random Node in Free Space"<<std::endl;
                break;
            }
        }
        // Initialize the nearest node
        double nearest_node_distance = std::numeric_limits<double>::max();
        amp::Node nearest_node = 0;
        Eigen::Vector2d nearest_node_location = nodes[0];
        // Take a step from the nearest node towards the random node defined by the step size
        for (const auto& [node, point] : nodes) {
            if ((point - random_node).norm() <= nearest_node_distance) {
                nearest_node = node;
                nearest_node_location = point;
                nearest_node_distance = (point - random_node).norm();
            }
        }
        Eigen::Vector2d step = (random_node - nearest_node_location).normalized();
        Eigen::Vector2d new_node_location = nearest_node_location + step_size * step;
        amp::Node new_node_index;
        // Check if the new node is in free space
        if(isStepValid(problem, nearest_node_location, new_node_location, agent_index)){
            new_node_index = nodes.size();
            nodes[new_node_index] = new_node_location;
            parent_map[new_node_index] = nearest_node;
            //std::cout<<"Goal Distance: "<<(new_node_location - q_goal).norm()<<std::endl;
            if((new_node_location - q_goal).norm() < episilon_goal){
                amp::Node goal_node = nodes.size();
                nodes[goal_node] = q_goal;
                parent_map[goal_node] = goal_node-1;
                path_success = true;
                //std::cout << "Goal reached" << std::endl;
                break;
            }
        }
        
        if(iteration >= max_iter){
            std::cout << "RRT Max iterations reached" << std::endl;
            path_success = false;
            break;
        }
        iteration++;
    }
    // Create the path
    amp::Node current_node = nodes.size()-1;
    Eigen::Vector2d current_location = nodes[current_node];
    while(current_node != 0){
        path.waypoints.push_back(current_location);
        current_node = parent_map[current_node];
        current_location = nodes[current_node];
    }
    path.waypoints.push_back(q_init);
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    //std::cout<<"RRT Iteration: "<<iteration<<std::endl;
    return path;
}

bool MyDecentralPlanner::isStepValid(amp::MultiAgentProblem2D problem, Eigen::Vector2d nearest_node_locations, Eigen::Vector2d agent_k, int agent_index){ 
    for(const auto& obstacle : problem.obstacles){
        Eigen::Vector2d min_point = MinimumPoint(agent_k, obstacle);
        if((agent_k - min_point).norm() < 1.5*problem.agent_properties[agent_index].radius){
            return false;
        }
    }
    amp::MultiAgentProblem2D expanded_problem = expand_obstacles(problem);
    bool line_collision = HW4Functions::lineSegmentIntersection(expanded_problem, nearest_node_locations, agent_k);
    if(line_collision){
        return false;
    }
    return true;
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
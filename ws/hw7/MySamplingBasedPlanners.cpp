# include "MySamplingBasedPlanners.h"
using namespace amp;

amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    // Create a graph and a map for nodes
    graphPtr = std::make_shared<amp::Graph<double>>();
    nodes = std::map<amp::Node, Eigen::Vector2d>();
    // Add initial and goal nodes
    nodes[0] = problem.q_init;
    nodes[1] = problem.q_goal;
    //generate n number of random points in the workspace and add them to the graph if not in obstacle
    int n = m_numSamples;
    double radius = m_connectionRadius;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x(problem.x_min,problem.x_max);
    std::uniform_real_distribution<double> y(problem.y_min,problem.y_max);
    // std::uniform_real_distribution<double> x(-1,11);
    // std::uniform_real_distribution<double> y(-3,3);
    Eigen::Vector2d random_node;

    for (int i = 0; i < n; i++) {

        random_node = Eigen::Vector2d(x(gen),y(gen));
        // check if the point is in free space
        bool in_free_space = true;
        for (const auto& obstacle : problem.obstacles) {
            if (point_in_polygon(random_node, obstacle)) {
                in_free_space = false;
                break;
            }
            }
            if(in_free_space) {
                // if it is, add it to the map
                Node node_index = nodes.size();
                nodes[node_index] = random_node;
            }
    }

    // Connect nodes and create heuristic
     amp::LookupSearchHeuristic heuristic;
    for (const auto& [node1, point1] : nodes) {
         heuristic.heuristic_values[node1] = (point1 - problem.q_goal).norm();
        for (const auto& [node2, point2] : nodes) {
            double distance = (point1 - point2).norm();
            bool intersect = HW4Functions::lineSegmentIntersection(problem, point1, point2);
            if (node1 != node2 && !intersect && distance <= radius) {
                graphPtr->connect(node1, node2, distance);
            }
        }

    }
    // Set the heuristic value for the initial node to 0
    heuristic.heuristic_values[0] = 0;
    // Find path using A*
    amp::ShortestPathProblem graph_problem;
    graph_problem.init_node = 0;
    graph_problem.goal_node = 1;
    graph_problem.graph = graphPtr;

    MyAStarAlgo astar;
    MyAStarAlgo::GraphSearchResult astar_result = astar.search(graph_problem, heuristic);
    m_success = astar_result.success;
    // Convert node path to waypoints
    for (auto node : astar_result.node_path) {
        path.waypoints.push_back(nodes[node]);
    }
    path = PathSmoothing(path, problem);
    return path;
}

 
amp::Path2D MyPRM::PathSmoothing(const amp::Path2D& path, const amp::Problem2D& problem){
    amp::Path2D smoothed_path;
    Eigen::Vector2d point1 = path.waypoints[0]; 
    smoothed_path.waypoints.push_back(point1);
    // Check if points can be connected without intersection
    if(!m_success){
        return path;
    }
    while(true){    
        for(int j = path.waypoints.size()-1; j>0; j--){
            if(path.waypoints[j] !=point1){
                Eigen::Vector2d point2 = path.waypoints[j];
                bool intersects = HW4Functions::lineSegmentIntersection(problem, point1, point2);
                if(!intersects){
                    smoothed_path.waypoints.push_back(point2);
                    point1 = point2;
                    break;
                }
            }
        }
        if(point1 == problem.q_goal){
            break;
        }
    }
    return smoothed_path;
}
// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    // Create a graph and a map for nodes
    graphPtr = std::make_shared<amp::Graph<double>>();
    nodes = std::map<amp::Node, Eigen::Vector2d>();
    MyAStarAlgo::GraphSearchResult astar_result;
    nodes[0] = problem.q_init;
    nodes[1] = problem.q_goal;
    Eigen::Vector2d nearest_node_location = nodes[0];
    Node nearest_node = 0;
    double step_size = m_stepSize;
    double episilon_goal = 0.25;
    double p_goal = m_goalBias;
    std::random_device rd;
    std::mt19937 gen(rd());
    Eigen::Vector2d random_node;
    int max_iter = m_numSamples;
    while(true){
        while(true){
            // Generate a random node in free space
            // generate a random number between 0 and 1
            std::uniform_real_distribution<double> random_number(0,1);
            if(random_number(gen) < p_goal){
                random_node = problem.q_goal;
            }else{
                std::uniform_real_distribution<double> x(problem.x_min,problem.x_max);
                std::uniform_real_distribution<double> y(problem.y_min,problem.y_max);
                random_node = Eigen::Vector2d(x(gen),y(gen));
            }
            // Check if the node is in free space
            bool in_free_space = true;
            for (const auto& obstacle : problem.obstacles) {
                if (point_in_polygon(random_node, obstacle)) {
                    in_free_space = false;
                    break;
                }
            }
            if(in_free_space){
            break;
                }
        }
        // Initialize the nearest node
        double nearest_node_distance = std::numeric_limits<double>::max();
        Node nearest_node = 0;
        Eigen::Vector2d nearest_node_location = nodes[0];
        // Take a step from the nearest node towards the random node defined by the step size
        for (const auto& [node, point] : nodes) {
            if ((point - random_node).norm() <= nearest_node_distance && node != 1) {
                nearest_node = node;
                nearest_node_location = point;
                nearest_node_distance = (point - random_node).norm();
            }
        }
        Eigen::Vector2d step = (random_node - nearest_node_location);
        Eigen::Vector2d new_node_location = nearest_node_location + step_size * step.normalized();
        // Check if the new node is in free space
        bool in_free_space = true;
        for (const auto& obstacle : problem.obstacles) {
            bool intersects = HW4Functions::lineSegmentIntersection(problem, nearest_node_location, new_node_location);
            if (intersects) {
                in_free_space = false;
                break;
            }else{
                Node new_node_index = nodes.size();
                nodes[new_node_index] = new_node_location;
                // connect the node to the nearest node
                graphPtr->connect(nearest_node, new_node_index, (new_node_location - nearest_node_location).norm());
            }
        }
        if((new_node_location - problem.q_goal).norm() < episilon_goal){
            graphPtr->connect(nearest_node, 1, (new_node_location - nearest_node_location).norm());
            std::cout << "Goal reached" << std::endl;
            break;
        }
        if(max_iter == 0){
            std::cout << "Max iterations reached" << std::endl;
            break;
        }
        max_iter--;
    }
    amp::ShortestPathProblem graph_problem;
    graph_problem.init_node = 0;
    graph_problem.goal_node = 1;
    graph_problem.graph = graphPtr;
    MyAStarAlgo astar;
    amp::LookupSearchHeuristic heuristic;
    for (const auto& [node, point] : nodes) {
        heuristic.heuristic_values[node] = (point - problem.q_goal).norm();
    }
    astar_result = astar.search(graph_problem, heuristic);
    for (auto node : astar_result.node_path) {
        path.waypoints.push_back(nodes[node]);
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
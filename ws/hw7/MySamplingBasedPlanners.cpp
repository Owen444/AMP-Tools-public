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
    int n = 1500;
    std::random_device rd;
    std::mt19937 gen(rd());
    Eigen::Vector2d random_node;
    for (int i = 0; i < n; i++) {
        std::uniform_real_distribution<double> x(problem.x_min,problem.x_max);
        std::uniform_real_distribution<double> y(problem.y_min,problem.y_max);
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

    // Connect nodes
    double radius = 1.5;
    for (const auto& [node1, point1] : nodes) {
        for (const auto& [node2, point2] : nodes) {
            double distance = (point1 - point2).norm();
            bool intersect = HW4Functions::lineSegmentIntersection(problem, point1, point2);
            if (node1 != node2 && !intersect && distance <= radius) {
                graphPtr->connect(node1, node2, distance);
            }
        }
    }

    // Find path using A*
    amp::ShortestPathProblem graph_problem;
    graph_problem.init_node = 0;
    graph_problem.goal_node = 1;
    graph_problem.graph = graphPtr;
    
    MyAStarAlgo astar;
    amp::LookupSearchHeuristic heuristic;
    for (const auto& [node, point] : nodes) {
        heuristic.heuristic_values[node] = (point - problem.q_goal).norm();
    }
    MyAStarAlgo::GraphSearchResult astar_result = astar.search(graph_problem, heuristic);

    // Convert node path to waypoints
    for (auto node : astar_result.node_path) {
        path.waypoints.push_back(nodes[node]);
    }

    return path;
}


// point in polygon
bool MyPRM::point_in_polygon(Eigen::Vector2d point, const amp::Obstacle2D& obstacle){
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
 

// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}
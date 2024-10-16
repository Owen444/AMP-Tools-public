#include "MyAStar.h"
using namespace amp;
// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object

    std::priority_queue<std::pair<double, Node>, std::vector<std::pair<double, Node>>, std::greater<std::pair<double, Node>>> Queue; // min heap
    // Add the start node to the open list
    Queue.push(std::make_pair(0.0, problem.init_node));
    // Create a map to store the cost of the path from the start node to any node
    std::unordered_map<Node, double> path_cost;
    // Create a map to store the node's parent
    std::unordered_map<Node, Node> parent;
    // Create a set to store the processed nodes
    std::unordered_set<Node> processed_nodes;
    // Initialize the cost of the path from the start node to the start node as 0
    path_cost[problem.init_node] = 0.0;
    double total_cost;
    int count = 0;
    while (!Queue.empty()) {
        Node current = Queue.top().second;
        Queue.pop();
        //std::cout << "Current Node: " << current << std::endl;
        if(current == problem.goal_node) {
            result.success = true;
            break; // Goal node found
        }
        std::vector<double> edges = problem.graph->outgoingEdges(current);
        std::vector<Node> children = problem.graph->children(current);
        for(int i = 0; i < children.size(); i++) {
            Node child = children[i];
            double edge_cost = edges[i];
            if(path_cost.find(child) == path_cost.end() || path_cost[current] + edge_cost < path_cost[child]) {
                // Push all connected nodes to the queue
                //std::cout << "Child: " << child << " Parent: " << current << std::endl;
                parent[child] = current;
                path_cost[child] = path_cost[current] + edge_cost;
                total_cost = path_cost[child]; //+ heuristic(child);
                Queue.push(std::make_pair(total_cost, child));
            }
        }
        count++;
    }   
    Node current_node = problem.goal_node;
    while(current_node != problem.init_node){
        result.node_path.push_front(current_node);
        current_node = parent[current_node];
    }
    std::cout << "Count: " << count << std::endl;
    result.node_path.push_front(problem.init_node);
    result.path_cost = path_cost[problem.goal_node];
    result.print();
    return result;
}

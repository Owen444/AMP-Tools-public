// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <chrono>
using namespace amp;
using namespace std::chrono;
int main(int argc, char** argv) {
    // HW7::hint(); // Consider implementing an N-dimensional planner 

    // Example of creating a graph and adding nodes for visualization
    // std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    // std::map<amp::Node, Eigen::Vector2d> nodes;
    
    // std::vector<Eigen::Vector2d> points = {{3, 3}, {4, 5}, {5, 3}, {6, 5}, {5, 7}, {7, 3}}; // Points to add to the graph
    // for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map
    // std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {{0, 4, 1}, {0, 5, 1}, {4, 5, 1}, {1, 2, 1}, {1, 3, 1}, {2, 3, 1}}; // Edges to connect
    // for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); // Connect the edges in the graph
    // graphPtr->print();

    // Test PRM on Workspace1 of HW2
    Problem2D problem = HW5::getWorkspace1();
    Path2D path;
    const std::vector<std::pair<int, double>> configurations = {
        {200, 0.5}, {200, 1.0}, {200, 1.5}, {200, 2.0},
        {500, 0.5}, {500, 1.0}, {500, 1.5}, {500, 2.0}
    };

    std::list<std::vector<double>> allTimes;
    std::list<std::vector<double>> all_pathLengths;
    std::list<std::vector<double>> all_success;
    std::vector<std::string> labels;

    for (const auto& config : configurations) {
        int numSamples = config.first;
        double connectionRadius = config.second;
        std::vector<double> times;
        std::vector<double> pathLengths;
        int num_success = 0;
        std::vector<double> success;
        for (int i = 0; i < 100; i++) {
            auto start = high_resolution_clock::now();
            MyPRM prm(numSamples, connectionRadius);
            path = prm.plan(problem);  // Assuming your PRM class accepts these parameters
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            times.push_back(duration.count() / 1000000.0);
            pathLengths.push_back(path.length());
            if(prm.getSuccess()){
               num_success++;
            }
        }
        success.push_back(num_success);
        allTimes.push_back(times);
        all_pathLengths.push_back(pathLengths);
        all_success.push_back(success);
        labels.push_back("n=" + std::to_string(numSamples) + ", r=" + std::to_string(connectionRadius));
    }

    Visualizer::makeBoxPlot(allTimes, labels, "PRM Performance", "Configuration", "Time (s)");
    Visualizer::makeBoxPlot(all_pathLengths, labels, "PRM Path Length", "Configuration", "Path Length");
    Visualizer::makeBoxPlot(all_success, labels, "PRM Success", "Configuration", "Success");
    //Visualizer::makeFigure(problem, path, *prm.getGraphPtr(), prm.getNodes());

    // Generate a random problem and test RRT
    // MyRRT rrt;
    // Path2D rrt_path;
    // //HW7::generateAndCheck(rrt, path, problem);
    // rrt_path = rrt.plan(problem);
    // std::cout<<"Path Length:"<<rrt_path.length()<<std::endl;
    // Visualizer::makeFigure(problem, rrt_path, *rrt.getGraphPtr(), rrt.getNodes());
    Visualizer::showFigures();

    // Grade method
   //HW7::grade<MyPRM, MyRRT>("owen.craig@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}
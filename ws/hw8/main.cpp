// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"
#include <chrono>
using namespace std::chrono;
using namespace amp;

int main(int argc, char** argv) {
   // Initialize Workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());
    MultiAgentProblem2D problem = HW8::getWorkspace1(3);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;
    
    // // Solve using a centralized approach
    // MyCentralPlanner central_planner;
    // MultiAgentPath2D path = central_planner.plan(problem);
    // bool isValid1 = HW8::check(path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);
    // bool isValid2 = HW8::generateAndCheck(central_planner, path, problem, collision_states);
    // amp::MultiAgentProblem2D expanded_problem = central_planner.expand_obstacles(problem);
    // Visualizer::makeFigure(expanded_problem, path, collision_states);
    // Benchmarking Centralized
    // std::list<std::vector<double>> allTimes;
    // std::list<std::vector<double>> all_graph_sizes;
    // std::vector<double> success;
    // std::vector<std::string> labels;
    // std::vector<double> times;
    // std::vector<double> graph_sizes;
    // int num_success = 0;
    // labels.push_back("6 Agents");
    // for (int i = 0; i < 50; i++){
    //     auto start = high_resolution_clock::now();
    //     MyCentralPlanner central_planner;
    //     MultiAgentPath2D path = central_planner.plan(problem);
    //     auto stop = high_resolution_clock::now();
    //     auto duration = duration_cast<microseconds>(stop - start);
    //     times.push_back(duration.count()/1000.0);
    //     if(central_planner.getPathSuccess()){
    //         num_success++;
    //         graph_sizes.push_back(static_cast<double>(central_planner.getTreeSize())); 
    //     } 
    //     std::cout<<"Iteration: "<<i<<std::endl;
    // }
    // success.push_back(num_success);
    // allTimes.push_back(times);
    // all_graph_sizes.push_back(graph_sizes);
    // Visualizer::makeBoxPlot(allTimes, labels, "Centralized Planner Performance", "Number of Agents", "Time (ms)");
    // Visualizer::makeBoxPlot(all_graph_sizes, labels, "Centralized Planner Graph Size", "Number of Agents", "Graph Size");
    // Visualizer::makeBarGraph(success, labels, "Centralized Planner Success", "Number of Agents", "Success");
    // std::cout<<"Average Time: "<<std::accumulate(times.begin(), times.end(), 0.0) / times.size()<<std::endl;
    // std::cout<<"Average Graph Size: "<<std::accumulate(graph_sizes.begin(), graph_sizes.end(), 0.0) / graph_sizes.size()<<std::endl;


    // Solve using a decentralized approach
    MyDecentralPlanner decentral_planner;
    MultiAgentPath2D path = decentral_planner.plan(problem);
    bool isValid1 = HW8::check(path, problem, collision_states);
    Visualizer::makeFigure(problem, path, collision_states);


    // bool isValid2 = HW8::generateAndCheck(decentral_planner, path, problem, collision_states);
    // amp::MultiAgentProblem2D expanded_problem = decentral_planner.expand_obstacles(problem);
    // Visualizer::makeFigure(problem, path, collision_states);
    // //Visualizer::makeFigure(expanded_problem, path, collision_states);



    // Benchmarking Decentralized
    // std::list<std::vector<double>> allTimes;
    // std::vector<double> success;
    // std::vector<std::string> labels;
    // std::vector<double> times;
    // int num_success = 0;
    // labels.push_back("2 Agents");
    // for (int i = 0; i < 50; i++){
    //     auto start = high_resolution_clock::now();
    //     MyDecentralPlanner decentral_planner;
    //     MultiAgentPath2D path = decentral_planner.plan(problem);
    //     auto stop = high_resolution_clock::now();
    //     auto duration = duration_cast<microseconds>(stop - start);
    //     times.push_back(duration.count()/1000.0);
    //     if(decentral_planner.get_success()){
    //         num_success++;
    //     } 
    //     std::cout<<"Iteration: "<<i<<std::endl;
    // }
    // success.push_back(num_success);
    // allTimes.push_back(times);
    // Visualizer::makeBoxPlot(allTimes, labels, "Centralized Planner Performance", "Number of Agents", "Time (ms)");
    // Visualizer::makeBarGraph(success, labels, "Centralized Planner Success", "Number of Agents", "Success");
    // std::cout<<"Average Time: "<<std::accumulate(times.begin(), times.end(), 0.0) / times.size()<<std::endl;
  






    // //Visualize and grade methods
    Visualizer::showFigures();
    //HW8::grade<MyCentralPlanner, MyDecentralPlanner>("owen.craig@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}

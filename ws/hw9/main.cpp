// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"
#include <chrono>
using namespace amp;

// Load problems and map agent for quick testing
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};

int main(int argc, char** argv) {
    //Select problem, plan, check, and visualize
    int select = 7;
    KinodynamicProblem2D prob = problems[select];
//     MyKinoRRT kino_planner;
//     kino_planner.setNumControlSamples(10);
//     if(prob.agent_type == AgentType::FirstOrderUnicycle){
//         prob.u_bounds[0] = {-2, 4};
//         prob.u_bounds[1] = {-1.5, 1.5};
//     }
//     if(prob.agent_type == AgentType::SecondOrderUnicycle){
//         prob.u_bounds[0] = {-1, 1.5};
//         prob.u_bounds[1] = {-0.75, 0.75};
//     }
//     if(prob.agent_type == AgentType::SimpleCar){
//         prob.u_bounds[0] = {-1.5, 2};
//         prob.u_bounds[1] = {-0.3, 0.3};
//     }
  
//     KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
//    KinoPath simulated_path = kino_planner.simulatePathRK4(prob, path, *agentFactory[prob.agent_type]());
//     HW9::check(path, prob);
//     if (path.valid){
//         Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation
//         //Visualizer::makeFigure(prob, simulated_path, false); // Set to 'true' to render animation
//         Visualizer::showFigures();
//     }


//Benchmark
// Benchmarking parameters
    // const std::vector<std::pair<int, int>> configurations = {
    //     {100000, 1}, {100000, 5}, {100000, 10}, {100000, 15}
    // };
    
    // std::list<std::vector<double>> allTimes;
    // std::list<std::vector<double>> all_pathLengths;
    // std::vector<double> success;
    // std::vector<std::string> labels;

    // // Run benchmarks for each configuration
    // for (const auto& config : configurations) {
    //     int numNodes = config.first;
    //     int numControlSamples = config.second;
        
    //     std::vector<double> times;
    //     std::vector<double> pathLengths;
    //     int num_success = 0;
        
    //     // Run multiple trials for this configuration
    //     for (int i = 0; i < 100; i++) {
    //         MyKinoRRT kino_planner;
    //         auto start = std::chrono::high_resolution_clock::now();
    //         // Set the parameters for this run
    //         kino_planner.setMaxNodes(numNodes);
    //         kino_planner.setNumControlSamples(numControlSamples);
    //         KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
            
    //         auto stop = std::chrono::high_resolution_clock::now();
    //         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            
    //         times.push_back(duration.count()/1000.0); // Convert to seconds
            
    //         if (path.valid) {
    //             num_success++;
    //             // Calculate the path length
    //             double length = 0.0;
    //             for (int i = 1; i < path.waypoints.size(); i++) {
    //                 length += (path.waypoints[i] - path.waypoints[i - 1]).norm();
    //             }
    //             pathLengths.push_back(length);
    //         }
            
    //         std::cout << "Configuration n=" << numNodes << ", u=" << numControlSamples 
    //                   << " Trial " << i + 1 << "/50\r" << std::flush;
    //     }
    //     std::cout << std::endl;
        
    //     // Store results for this configuration
    //     success.push_back(num_success);
    //     allTimes.push_back(times);
    //     all_pathLengths.push_back(pathLengths);
    //     labels.push_back("n=" + std::to_string(numNodes) + "\nu=" + std::to_string(numControlSamples));
    // }

    // // Visualize results
    // Visualizer::makeBoxPlot(allTimes, labels, "Kinodynamic RRT Performance", "Configuration", "Time (s)");
    // Visualizer::makeBoxPlot(all_pathLengths, labels, "Path Length Distribution", "Configuration", "Path Length");
    // Visualizer::makeBarGraph(success, labels, "Success Rate (out of 100)", "Configuration", "Successes");
    
    // Visualizer::showFigures();


   //HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("owen.craig@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;
}
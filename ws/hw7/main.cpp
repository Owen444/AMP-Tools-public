// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <chrono>
using namespace amp;
using namespace std::chrono;
int main(int argc, char** argv) {
    //Problem 1 
    //Part I
    // Problem2D problem = HW5::getWorkspace1();
    // MyPRM prm(500,2);
    // Path2D path = prm.plan(problem);
    // Visualizer::makeFigure(problem, path, *prm.getGraphPtr(), prm.getNodes());
    // std::cout<<"Path Length:"<<path.length()<<std::endl;

    //Part II
    //Test PRM on Workspace1 of HW2
    Problem2D problem = HW2::getWorkspace2();
    Path2D path;
    //Part a configurations
    // const std::vector<std::pair<int, double>> configurations = {
    //     {200, 0.5}, {200, 1.0}, {200, 1.5}, {200, 2.0},
    //     {500, 0.5}, {500, 1.0}, {500, 1.5}, {500, 2.0}
    // };
    //Part b configurations
    const std::vector<std::pair<int, double>> configurations = {
        {200, 1.0}, {200, 2.0}, {500, 1.0}, {500, 2.0},
        {1000, 1.0}, {1000, 2.0}
    };
    std::list<std::vector<double>> allTimes;
    std::list<std::vector<double>> all_pathLengths;
    std::vector<double> success;
    std::vector<std::string> labels;


    for (const auto& config : configurations) {
        int numSamples = config.first;
        double connectionRadius = config.second;
        std::vector<double> times;
        std::vector<double> pathLengths;
        int num_success = 0;
        for (int i = 0; i < 100; i++) {
            auto start = high_resolution_clock::now();
            MyPRM prm(numSamples, connectionRadius);
            path = prm.plan(problem);  // Assuming your PRM class accepts these parameters
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            times.push_back(duration.count() / 1000000.0);
            if(prm.getSuccess()){
               num_success++;
               pathLengths.push_back(path.length());
            }
        }
        success.push_back(num_success);
        allTimes.push_back(times);
        all_pathLengths.push_back(pathLengths);
        labels.push_back("n=" + std::to_string(numSamples) + ", r=" + std::to_string(connectionRadius));
    }

    Visualizer::makeBoxPlot(allTimes, labels, "PRM Performance", "Configuration", "Time (s)");
    Visualizer::makeBoxPlot(all_pathLengths, labels, "PRM Path Length", "Configuration", "Path Length");
    Visualizer::makeBarGraph(success, labels, "PRM Success", "Configuration", "Success");


    // Generate a random problem and test RRT
    // MyRRT rrt(5000, 0.05, 0.5);
    // Path2D rrt_path;
    // //HW7::generateAndCheck(rrt, path, problem);
    // rrt_path = rrt.plan(problem);
    // std::cout<<"Path Length:"<<rrt_path.length()<<std::endl;
    // Visualizer::makeFigure(problem, rrt_path, *rrt.getGraphPtr(), rrt.getNodes());


    // //Benchmarking RRT
    // Path2D rrt_path;
    // const std::vector<std::vector<double>> rrt_configurations = {
    //     {5000, 0.05, 0.5}, {5000, 0.05, 2.0},
    //     {5000, 0.1, 0.5}, {5000, 0.1, 2.0},
    //     {5000, 0.2, 0.5}, {5000, 0.2, 2.0}
    // };
    // std::list<std::vector<double>> all_pathLengths;
    // std::list<std::vector<double>> allTimes;
    // std::vector<double> success;
    // std::vector<std::string> labels;
    // for(const auto& config : rrt_configurations){
    //     int numSamples = config[0];
    //     double goalBias = config[1];
    //     double stepSize = config[2];
    //     std::vector<double> rrt_path_lengths;
    //     std::vector<double> rrt_times;
    //     int num_success = 0;
    //     for(int i = 0; i < 100; i++){
    //         auto start = high_resolution_clock::now();
    //         MyRRT rrt(numSamples, goalBias, stepSize);
    //         rrt_path = rrt.plan(problem);
    //         auto stop = high_resolution_clock::now();
    //         auto duration = duration_cast<microseconds>(stop - start);
    //         rrt_times.push_back(duration.count() / 1000000.0);
    //         if(rrt.getSuccess()){
    //             num_success++;
    //             rrt_path_lengths.push_back(rrt_path.length());
    //         }
    //     }
    //     success.push_back(num_success);
    //     all_pathLengths.push_back(rrt_path_lengths);
    //     allTimes.push_back(rrt_times);
    //     labels.push_back("p=" + std::to_string(goalBias) + ", step=" + std::to_string(stepSize));
    // }
    // Visualizer::makeBoxPlot(allTimes, labels, "RRT Performance", "Configuration", "Time (s)");
    // Visualizer::makeBoxPlot(all_pathLengths, labels, "RRT Path Length", "Configuration", "Path Length");
    // Visualizer::makeBarGraph(success, labels, "RRT Success", "Configuration", "Success");

    Visualizer::showFigures();

    // Grade method
   //HW7::grade<MyPRM, MyRRT>("owen.craig@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}
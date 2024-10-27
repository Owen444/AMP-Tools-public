// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"

using namespace amp;

int main(int argc, char** argv) {
   // Initialize Workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());
    MultiAgentProblem2D problem = HW8::getWorkspace1(2);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;
    
    // Solve using a centralized approach
    MyCentralPlanner central_planner;
    MultiAgentPath2D path = central_planner.plan(problem);
    //path.agent_paths[0].print();
    bool isValid1 = HW8::check(path, problem, collision_states);
    Visualizer::makeFigure(problem, path, collision_states);
    //bool isValid2 = HW8::generateAndCheck(central_planner, path, problem, collision_states);
    //amp::MultiAgentProblem2D expanded_problem = central_planner.expand_obstacles(problem);
    //Visualizer::makeFigure(expanded_problem, path, collision_states);

    // Solve using a decentralized approach
    // MyDecentralPlanner decentral_planner;
    // MultiAgentPath2D path = decentral_planner.plan(problem);
    // bool isValid1 = HW8::check(path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);
    // bool isValid2 = HW8::generateAndCheck(decentral_planner, path, problem, collision_states);
    // amp::MultiAgentProblem2D expanded_problem = decentral_planner.expand_obstacles(problem);
    // Visualizer::makeFigure(problem, path, collision_states);
    // //Visualizer::makeFigure(expanded_problem, path, collision_states);
    // //Visualize and grade methods
    Visualizer::showFigures();
    //HW8::grade<MyCentralPlanner, MyDecentralPlanner>("owen.craig@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}
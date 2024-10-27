#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
// Include the correct homework headers
#include "hw/HW8.h"
#include "HW4Functions.h"
#include "MyAStar.h"
// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 

bool point_in_polygon(Eigen::Vector2d point, const amp::Obstacle2D& obstacle);
Eigen::Vector2d MinimumPoint(Eigen::Vector2d q, const amp::Obstacle2D& obstacle);
class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override; 
        amp::MultiAgentProblem2D expand_obstacles(const amp::MultiAgentProblem2D& problem);
        amp::MultiAgentPath2D MultiAgentRRT(const amp::MultiAgentProblem2D& problem);
        bool AgentIntersection(amp::MultiAgentProblem2D problem, Eigen::Vector2d P1, Eigen::Vector2d P2);
        bool isSubpathCollisionFree(Eigen::VectorXd nearest_node_locations, Eigen::VectorXd step_meta_agent, const amp::MultiAgentProblem2D& problem);
        bool isSystemValid(amp::MultiAgentProblem2D problem, Eigen::VectorXd nearest_node_locations, Eigen::VectorXd meta_agent_k);
    private:
        std::shared_ptr<amp::Graph<double>> graphPtr;
        std::vector<std::map<amp::Node, Eigen::Vector2d>> node_maps;
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
        amp::Path2D MultiAgentRRT(amp::MultiAgentProblem2D problem, Eigen::Vector2d q_init, Eigen::Vector2d q_goal,int agent_index);
        amp::MultiAgentProblem2D expand_obstacles(const amp::MultiAgentProblem2D& problem);
        bool isStepValid(amp::MultiAgentProblem2D problem, Eigen::Vector2d nearest_node_locations, Eigen::Vector2d agent_k, int agent_index);
        private:
        bool path_success;
};  
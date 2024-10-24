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
class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override; 
        amp::MultiAgentProblem2D expand_obstacles(const amp::MultiAgentProblem2D& problem);
        amp::MultiAgentPath2D MultiAgentRRT(const amp::MultiAgentProblem2D& problem);
        int orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
        bool onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
        bool Intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2);
        bool AgentIntersection(Eigen::Vector2d Q1, Eigen::Vector2d Q2, Eigen::Vector2d P1, Eigen::Vector2d P2);
        bool isSubpathCollisionFree(std::vector<Eigen::Vector2d>nearest_node_location, std::vector<Eigen::Vector2d> q_step, const amp::MultiAgentProblem2D& problem);
    private:
        std::shared_ptr<amp::Graph<double>> graphPtr;
        std::vector<std::map<amp::Node, Eigen::Vector2d>> node_maps;
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
};
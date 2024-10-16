#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "HW4Functions.h"
#include "MyAStar.h"
// Include the correct homework headers
#include "hw/HW7.h"

class MyPRM : public amp::PRM2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        bool point_in_polygon(Eigen::Vector2d point, const amp::Obstacle2D& obstacle);
        std::shared_ptr<amp::Graph<double>> getGraphPtr() const {
            return graphPtr;
        }
        const std::map<amp::Node, Eigen::Vector2d>& getNodes() const {
            return nodes;
        }
    private:
        std::map<amp::Node, Eigen::Vector2d> nodes;
        std::shared_ptr<amp::Graph<double>> graphPtr;
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
};

struct LookupSearchHeuristic : public amp::SearchHeuristic {
	/// @brief Get the heuristic value stored in `heuristic_values`. 
	/// @param node Node to get the heuristic value h(node) for. 
	/// @return Heuristic value
	virtual double operator()(amp::Node node) const override {return heuristic_values.at(node);}

    /// @brief Store the heursitic values for each node in a map
    std::map<amp::Node, double> heuristic_values; 
};
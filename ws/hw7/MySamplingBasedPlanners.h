#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "HW4Functions.h"
#include "MyAStar.h"
// Include the correct homework headers
#include "hw/HW7.h"
bool point_in_polygon(Eigen::Vector2d point, const amp::Obstacle2D& obstacle);
class MyPRM : public amp::PRM2D {
    public:
        MyPRM(int numSamples = 200, double connectionRadius = 1.5) 
            : m_numSamples(numSamples), m_connectionRadius(connectionRadius) {}
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        amp::Path2D PathSmoothing(const amp::Path2D& path, const amp::Problem2D& problem);
        std::shared_ptr<amp::Graph<double>> getGraphPtr() const {return graphPtr;}
        const std::map<amp::Node, Eigen::Vector2d>& getNodes() const {return nodes;}
        bool getSuccess() const {return m_success;}
    private:
        std::map<amp::Node, Eigen::Vector2d> nodes;
        std::shared_ptr<amp::Graph<double>> graphPtr;
        int m_numSamples;
        double m_connectionRadius;
        bool m_success;
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        MyRRT(int numSamples = 200, double goalBias = 0.05, double stepSize = 0.5) 
            : m_numSamples(numSamples), m_goalBias(goalBias), m_stepSize(stepSize) {}
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        std::shared_ptr<amp::Graph<double>> getGraphPtr() const {
            return graphPtr;
        }
        const std::map<amp::Node, Eigen::Vector2d>& getNodes() const {
            return nodes;
        }
        bool getSuccess() const {
            return m_success;
        }
    private:
        std::map<amp::Node, Eigen::Vector2d> nodes;
        std::shared_ptr<amp::Graph<double>> graphPtr;
        bool m_success;
        int m_numSamples;
        double m_goalBias;
        double m_stepSize;
};

struct LookupSearchHeuristic : public amp::SearchHeuristic {
	/// @brief Get the heuristic value stored in `heuristic_values`. 
	/// @param node Node to get the heuristic value h(node) for. 
	/// @return Heuristic value
	virtual double operator()(amp::Node node) const override {return heuristic_values.at(node);}

    /// @brief Store the heursitic values for each node in a map
    std::map<amp::Node, double> heuristic_values; 
};
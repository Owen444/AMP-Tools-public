#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"
#include "HW4Functions.h"

bool point_in_polygon(Eigen::Vector2d point, const amp::Obstacle2D& obstacle);
Eigen::Vector2d MinimumPoint(Eigen::Vector2d q, const amp::Obstacle2D& obstacle);
void propagate_car(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt,double L);
class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;
        std::pair<Eigen::VectorXd, Eigen::VectorXd> GenerateLocalTrajectory(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, Eigen::VectorXd q_near, Eigen::VectorXd q_rand,double dt);
        std::pair<bool, Eigen::VectorXd> IsSubTrajectoryValid(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, Eigen::VectorXd q_near, Eigen::VectorXd control, double dt);
        Eigen::VectorXd get_random_state(const amp::KinodynamicProblem2D& problem, double p_goal);
        void simulate_car(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt,double L);
        bool PolygonPolygonIntersection(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent,Eigen::VectorXd state);
        amp::KinoPath simulatePathRK4(const amp::KinodynamicProblem2D& problem, const amp::KinoPath& plannedPath, amp::DynamicAgent& agent);
        void setMaxNodes(int n) { RRT_samples = n; }
        void setNumControlSamples(int n) { m_numSamples = n; }
    private:
        int m_numSamples = 50;
        double RRT_samples = 100000;
};  

class MySingleIntegrator : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

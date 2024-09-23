#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        amp::Path2D Bug1(const amp::Problem2D& problem);
        amp::Path2D Bug2(const amp::Problem2D& problem);
        bool lineSegmentIntersection(const amp::Problem2D& problem, Eigen::Vector2d position, Eigen::Vector2d POI,int& collision_vertex_index, amp::Obstacle2D& collision_obstacle);
        bool Intersect(Eigen::Vector2d position, Eigen::Vector2d heading, Eigen::Vector2d vertex1,Eigen::Vector2d vertex);
        int orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
        bool onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
        //std::vector<Eigen::Vector2d> rotate_heading(std::vector<Eigen::Vector2d> state);
        std::vector<Eigen::Vector2d> obstaclefollowing(const amp::Problem2D& problem, amp::Path2D& path, std::vector<Eigen::Vector2d> state, int& collision_vertex_index, amp::Obstacle2D& collision_obstacle, std::vector<Eigen::Vector2d>& collision_path, Eigen::Vector2d& leave_point, int& leave_index,int& mode, double& vertex_distance);
        std::vector<Eigen::Vector2d> obstaclefollowing_Bug2(const amp::Problem2D& problem, amp::Path2D& path, std::vector<Eigen::Vector2d> state, int& collision_vertex_index, amp::Obstacle2D& collision_obstacle, std::vector<Eigen::Vector2d>& collision_path, Eigen::Vector2d& leave_point, int& leave_index,int& mode, double& vertex_distance, double& distance_to_goal);
        std::vector<Eigen::Vector2d> align_to_obstacle(const amp::Problem2D& problem,std::vector<Eigen::Vector2d> state, int& collision_vertex_index, amp::Obstacle2D& collision_obstacle,double& vertex_distance);
        std::vector<Eigen::Vector2d> return_to_leave_point(const amp::Problem2D& problem,amp::Path2D& path,std::vector<Eigen::Vector2d> state,std::vector<Eigen::Vector2d> collision_path ,Eigen::Vector2d leave_point,int leave_index, int& mode);
        std::vector<Eigen::Vector2d> go_to_goal(const amp::Problem2D& problem,amp::Path2D& path,std::vector<Eigen::Vector2d> state, int& collision_vertex_index, amp::Obstacle2D& collision_obstacle,int& mode);
        bool insidePolygon(const amp::Problem2D& problem, amp::Path2D& path,const Eigen::Vector2d& point);
        bool lineSegmentIntersection_mline(const amp::Problem2D& problem, Eigen::Vector2d position, Eigen::Vector2d POI,double& distance_to_goal);
        //Wite a function called get primitives that returns a vector of primitives for all the objects in the environment
    private:
        // Add any member variables here...
};
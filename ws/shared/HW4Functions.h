#include "AMPCore.h"
#include "hw/HW4.h"
#include "Eigen/Geometry"
class HW4Functions {
    public:
        static amp::Obstacle2D getMinkowskiSum(const amp::Obstacle2D& obstacle1, const amp::Obstacle2D& obstacle2);
        static double angle(Eigen::Vector2d Vertice1, Eigen::Vector2d Vertice2);
        static std::vector<Eigen::Vector2d> orderVertices(std::vector<Eigen::Vector2d> vertices);
        static amp::Obstacle2D rotatePolygon(const amp::Obstacle2D& obstacle, double angle);
        static bool lineSegmentIntersection(const amp::Environment2D& env, Eigen::Vector2d P1, Eigen::Vector2d P2);
        static int  orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
        static bool  onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
        static bool  Intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2);
};
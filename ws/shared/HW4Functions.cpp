#include "HW4Functions.h"

amp::Obstacle2D HW4Functions::getMinkowskiSum(const amp::Obstacle2D& obstacle1, const amp::Obstacle2D& obstacle2) {
    std::vector<Eigen::Vector2d> verticiesO = obstacle1.verticesCCW();
    std::vector<Eigen::Vector2d> verticiesR = obstacle2.verticesCCW();
    std::vector<Eigen::Vector2d> verticiesC;

    // Make verticiesR = -verticiesR
    for(int i = 0; i < verticiesR.size(); i++) {
        verticiesR[i] = -verticiesR[i];
    }
    verticiesO = HW4Functions::orderVertices(verticiesO);
    verticiesR = HW4Functions::orderVertices(verticiesR);
    int i = 0;
    int j = 0;
    while(i != verticiesO.size()+1 && j != verticiesR.size()+1) {
        // std::cout << "i: " << i << " j: " << j << std::endl;
        verticiesC.push_back(verticiesO[i%verticiesO.size()] + verticiesR[j%verticiesR.size()]);
        double angle1 = HW4Functions::angle(verticiesO[i%verticiesO.size()], verticiesO[(i+1) % verticiesO.size()]);
        double angle2 = HW4Functions::angle(verticiesR[j%verticiesR.size()], verticiesR[(j+1) % verticiesR.size()]);
        if(angle1 < angle2) {
            i++;
        } else if(angle1 > angle2) {
            j++;
        } else {
            i++;
            j++;
        }
    }

    amp::Obstacle2D minkowski_sum(verticiesC);
    return minkowski_sum;
}

double HW4Functions::angle(Eigen::Vector2d Vertice1, Eigen::Vector2d Vertice2) {
    double angle = atan2(Vertice2.y() - Vertice1.y(), Vertice2.x() - Vertice1.x());
    if(angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}

std::vector<Eigen::Vector2d> HW4Functions::orderVertices(std::vector<Eigen::Vector2d> vertices) {
    // Sort the vertices in CCW order where the first verticies so the first one is the one with the smallest y value
   int index = 0;
    std::vector<Eigen::Vector2d> ordered_vertices;
    
   for(int i = 0; i < vertices.size(); i++) {
        if(vertices[i].y() < vertices[index].y()) {
            index = i;
        }
    }
    for(int i = 0; i < vertices.size(); i++) {
            ordered_vertices.push_back(vertices[(index+i)%vertices.size()]);
    }
    return ordered_vertices;
}

amp::Obstacle2D HW4Functions::rotatePolygon(const amp::Obstacle2D& obstacle, double angle) {
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
    std::vector<Eigen::Vector2d> rotatedVertices;
    Eigen::Rotation2Dd rot(angle);
    for (const auto& vertex : vertices) {
        rotatedVertices.push_back(rot * vertex);
    }
    amp::Obstacle2D rotatedObstacle(rotatedVertices);
    return rotatedObstacle;
}

bool HW4Functions::lineSegmentIntersection(const amp::Environment2D& env, Eigen::Vector2d P1, Eigen::Vector2d P2){
   for (const auto& obstacle : env.obstacles){
       std::vector<Eigen::Vector2d> verticies = obstacle.verticesCW();
        for (int i = 0; i < verticies.size(); i++){
           Eigen::Vector2d vertex1 = verticies[i];
           Eigen::Vector2d vertex2 = verticies[(i+1)%verticies.size()];
           if(Intersect(P1,P2,vertex1,vertex2)){
               return true;
           }
        }
   }
   return false;
}
// Helper function to check the orientation of the triplet (p, q, r)
int  HW4Functions::orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
   // Calculate the determinant of the matrix formed by the vectors pq and qr
   double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
  
   if (val == 0) return 0; // collinear
   return (val > 0) ? 1 : 2; // clock or counterclock wise
}
// Check if point q lies on segment pr
bool  HW4Functions::onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
   return q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
          q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y());
}
// Main function to check if line segments (p1, q1) and (p2, q2) intersect
bool  HW4Functions::Intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2) {
   // Find the four orientations needed for the general and special cases
   int o1 = orientation(p1, q1, p2);
   int o2 = orientation(p1, q1, q2);
   int o3 = orientation(p2, q2, p1);
   int o4 = orientation(p2, q2, q1);
  
   // General case: different orientations indicate intersection
   if (o1 != o2 && o3 != o4)
       return true;
  
   // Special Cases: when points are collinear and lie on the segment
   if (o1 == 0 && onSegment(p1, p2, q1)) return true;
   if (o2 == 0 && onSegment(p1, q2, q1)) return true;
   if (o3 == 0 && onSegment(p2, p1, q2)) return true;
   if (o4 == 0 && onSegment(p2, q1, q2)) return true;
  
   return false; // No intersection
}

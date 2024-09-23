#include "MyBugAlgorithm.h"
// NEXT STEP FIND A WAY TO KEEP THE HEADING ALIGNED WITH THE OBSTACLE
// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    //path = Bug1(problem);
   path = Bug2(problem);
   return path;
}


amp::Path2D MyBugAlgorithm::Bug1(const amp::Problem2D& problem){
    amp::Path2D path;
   // Define useful constants
   int mode = 0;   // Mode 0 go to goal, Mode 1 follow obstacle, Mode 2 return to leave point, Mode 3 Goal Reached
   int step =1;
   double step_size = 0.1;
   double tolerance = 0.1;
   int leave_index;
   double vertex_distance;
   path.waypoints.push_back(problem.q_init);
   int collision_vertex_index;
   amp::Obstacle2D collision_obstacle;
   std::vector<Eigen::Vector2d> collision_path;
   Eigen::Vector2d leave_point;
   // Define state as a vector of 3 vectors, where the first vector is the current position, the second vector is the heading, and the third vector is the right hand heading
   std::vector<Eigen::Vector2d> state(4);
   state[0] = path.waypoints.back();
   state[1] = (problem.q_goal - problem.q_init).normalized() * step_size;
   state[2] = Eigen::Vector2d(state[1].y(), -state[1].x());
   while(1){
       //std::cout<<mode<<std::endl;
       if(mode == 0){
           // Go to goal
           state = go_to_goal(problem, path, state,collision_vertex_index, collision_obstacle ,mode);
       }else if(mode == 1){
           // Follow obstacle
           state  = obstaclefollowing(problem, path, state, collision_vertex_index, collision_obstacle, collision_path, leave_point,leave_index,mode,vertex_distance);
       } else if (mode == 2){
           // Return to leave point
              state = return_to_leave_point(problem, path, state, collision_path , leave_point,leave_index, mode);
       }
       else if (mode == 3){
           break;
       }
       if (step >100000){
           std::cout << "Loop Broken main" << std::endl;
           break;
       }
       step++;


   }
   path.waypoints.push_back(problem.q_goal);


   return path;
}

amp::Path2D MyBugAlgorithm::Bug2(const amp::Problem2D& problem){
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    // Define useful constants/Variables
    int step =1;
    double step_size = 0.01;
    int mode = 0;
    std::vector<Eigen::Vector2d> state(4);
    state[0] = path.waypoints.back();
    state[1] = (problem.q_goal - problem.q_init).normalized() * step_size;
    state[2] = Eigen::Vector2d(state[1].y(), -state[1].x());
    int collision_vertex_index;
    int leave_index;
    double vertex_distance;
    double distance_to_goal;
    std::vector<Eigen::Vector2d> collision_path;
    Eigen::Vector2d leave_point;
    amp::Obstacle2D collision_obstacle;
    while(1){
        if(mode == 0){
            state = go_to_goal(problem, path, state,collision_vertex_index, collision_obstacle ,mode);
        }else if(mode==1){
            state  = obstaclefollowing_Bug2(problem, path, state, collision_vertex_index, collision_obstacle, collision_path, leave_point,leave_index,mode,vertex_distance,distance_to_goal);
        }else if(mode == 3){
           std::cout<<"Exit with Failure"<<std::endl;
           break;
        }
        if (step >10000){
            std::cout << "Main Loop Broken" << std::endl;
            break;
        }
        step++;
    }
    path.waypoints.push_back(problem.q_goal);
   return path;
}

std::vector<Eigen::Vector2d> MyBugAlgorithm::go_to_goal(const amp::Problem2D& problem,amp::Path2D& path,std::vector<Eigen::Vector2d> state, int& collision_vertex_index, amp::Obstacle2D& collision_obstacle,int& mode){
   std::cout << "Mode 0: Go to Goal" << std::endl;
   Eigen::Matrix2d rotationMatrix;
   rotationMatrix << 0, 1,
                    -1, 0;
   double tolerance = 0.01;
   double step_size = 0.01;
   state[1] = (problem.q_goal - state[0]).normalized() * step_size;
   state[2] = Eigen::Vector2d(state[1].y(), -state[1].x());
   if(!lineSegmentIntersection(problem, state[0], (state[1])+state[0], collision_vertex_index, collision_obstacle)){
       state[0] += state[1];
       path.waypoints.push_back(state[0]);
       // Break Conditions
       double distance_to_goal = (state[0] - problem.q_goal).norm();
       if(distance_to_goal < tolerance){
           path.waypoints.push_back(problem.q_goal);
           std::cout << "GOOOOOOOOOOAAAAAAAAALLLLLLL" << std::endl;
           mode = 3;
       }
   }else{
       mode = 1;
   }


   return state;
}

std::vector<Eigen::Vector2d> MyBugAlgorithm::obstaclefollowing(const amp::Problem2D& problem,amp::Path2D& path,std::vector<Eigen::Vector2d> state,int& collision_vertex_index, amp::Obstacle2D& collision_obstacle,std::vector<Eigen::Vector2d>& collision_path,Eigen::Vector2d& leave_point,int& leave_index,int& mode, double& vertex_distance){
    std::cout << "Mode 1: Obstacle Encountered" << std::endl;
    int step =1;
    double step_size = 0.1;
    double tolerance = 0.1;
    double min_goal_distance = (problem.q_goal - state[0]).norm();
    Eigen::Vector2d hit_point = state[0];
    state  = align_to_obstacle(problem, state, collision_vertex_index, collision_obstacle,vertex_distance);
    while(1){
        if(!lineSegmentIntersection(problem,state[0],(state[1]*1.05)+state[0],collision_vertex_index,collision_obstacle)){
            // std::cout<<vertex_distance<<std::endl;
            // std::cout<<(collision_obstacle.verticesCW()[(collision_vertex_index)]-state[0]).norm()<<std::endl;
            // break;
          if((collision_obstacle.verticesCW()[collision_vertex_index]-state[0]).norm() >vertex_distance+.05){
            collision_vertex_index = (collision_vertex_index + 1) % collision_obstacle.verticesCW().size();
            state = align_to_obstacle(problem, state, collision_vertex_index, collision_obstacle,vertex_distance);
          }else{
            state[0] += state[1];
            path.waypoints.push_back(state[0]);
          }
        }else{
            // Navigate around the obstacle
            state = align_to_obstacle(problem, state, collision_vertex_index, collision_obstacle,vertex_distance);
        }
        collision_path.push_back(state[0]);
        if (min_goal_distance > (problem.q_goal - state[0]).norm()){
            min_goal_distance = (problem.q_goal - state[0]).norm();
            leave_point = state[0];
            leave_index = collision_path.size()-1;
        }
        double distance_to_hitpoint = (state[0] - hit_point).norm();
        // Check if the distance is within the tolerance
        if (distance_to_hitpoint < tolerance) {
            std::cout<<hit_point<<std::endl;
            std::cout << "Hit Point Reached" << std::endl;
            mode = 2;
            break;
        }
        if(step >100000){
            std::cout << "Loop Broken" << std::endl;
            break;
        }
        step++;
    }
    return state;
 }

std::vector<Eigen::Vector2d> MyBugAlgorithm::obstaclefollowing_Bug2(const amp::Problem2D& problem,amp::Path2D& path,std::vector<Eigen::Vector2d> state,int& collision_vertex_index, amp::Obstacle2D& collision_obstacle,std::vector<Eigen::Vector2d>& collision_path,Eigen::Vector2d& leave_point,int& leave_index,int& mode, double& vertex_distance, double& distance_to_goal){
    std::cout << "Mode 1: Obstacle Encountered" << std::endl;
    int step =1;
    double step_size = 0.01;
    double tolerance = 0.009;
    double min_goal_distance = (problem.q_goal - state[0]).norm();
    Eigen::Vector2d hit_point = state[0];
    state  = align_to_obstacle(problem, state, collision_vertex_index, collision_obstacle,vertex_distance);
    while(1){
        if(!lineSegmentIntersection(problem,state[0],(state[1]*1.05)+state[0],collision_vertex_index,collision_obstacle)){
          if((collision_obstacle.verticesCW()[collision_vertex_index]-state[0]).norm() >vertex_distance+.05){
            collision_vertex_index = (collision_vertex_index + 1) % collision_obstacle.verticesCW().size();
            state = align_to_obstacle(problem, state, collision_vertex_index, collision_obstacle,vertex_distance);
          }else{
            state[0] += state[1];
            path.waypoints.push_back(state[0]);
          }
        }else{
            // Navigate around the obstacle
            state = align_to_obstacle(problem, state, collision_vertex_index, collision_obstacle,vertex_distance);
            //break;
            if(!lineSegmentIntersection(problem,state[0],(state[1])+state[0],collision_vertex_index,collision_obstacle)){
                state[0] += state[1];
                path.waypoints.push_back(state[0]);
                mode = 0;
            }else{
                state = align_to_obstacle(problem, state, collision_vertex_index, collision_obstacle,vertex_distance);
            }
        }
        collision_path.push_back(state[0]);
        if (min_goal_distance > (problem.q_goal - state[0]).norm()){
            min_goal_distance = (problem.q_goal - state[0]).norm();
            leave_point = state[0];
            leave_index = collision_path.size()-1;
        }
        if(lineSegmentIntersection_mline(problem, state[0], state[1]+state[0], distance_to_goal)){
            if(distance_to_goal < (problem.q_goal-hit_point).norm()){
                // state[0] += state[1];
                // path.waypoints.push_back(state[0]);
                mode = 0;
                break;
            }
        }
        double distance_to_hitpoint = (state[0] - hit_point).norm();
        // Check if the distance is within the tolerance
        if (distance_to_hitpoint < tolerance) {
            std::cout<<hit_point<<std::endl;
            std::cout << "Hit Point Reached" << std::endl;
            mode = 3;
            break;
        }
        if(step >10000){
            std::cout << "Loop Broken" << std::endl;
            break;
        }
        step++;
    }
    return state;
 }

bool MyBugAlgorithm::lineSegmentIntersection(const amp::Problem2D& problem, Eigen::Vector2d position, Eigen::Vector2d POI,int& collision_vertex_index, amp::Obstacle2D& collision_obstacle){
   for (const auto& obstacle : problem.obstacles){
       std::vector<Eigen::Vector2d> verticies = obstacle.verticesCW();
        for (int i = 0; i < verticies.size(); i++){
           Eigen::Vector2d vertex1 = verticies[i];
           Eigen::Vector2d vertex2 = verticies[(i+1)%verticies.size()];
           if(Intersect(position,POI,vertex1,vertex2)){
               collision_vertex_index = i;
               collision_obstacle = obstacle;
               return true;
           }
        }
   }
   return false;
}
// Helper function to check the orientation of the triplet (p, q, r)
int MyBugAlgorithm::orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
   // Calculate the determinant of the matrix formed by the vectors pq and qr
   double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
  
   if (val == 0) return 0; // collinear
   return (val > 0) ? 1 : 2; // clock or counterclock wise
}
// Check if point q lies on segment pr
bool MyBugAlgorithm::onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
   return q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
          q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y());
}
// Main function to check if line segments (p1, q1) and (p2, q2) intersect
bool MyBugAlgorithm::Intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2) {
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

std::vector<Eigen::Vector2d> MyBugAlgorithm::align_to_obstacle(const amp::Problem2D& problem,std::vector<Eigen::Vector2d> state, int& collision_vertex_index, amp::Obstacle2D& collision_obstacle,double& vertex_distance){
   double step_size = 0.01;
   Eigen::Vector2d new_heading = state[1];
   int i = 0;
   // collision_obstacle.verticesCW();
   Eigen::Vector2d vertex1 = collision_obstacle.verticesCW()[collision_vertex_index];
   Eigen::Vector2d vertex2 = collision_obstacle.verticesCW()[(collision_vertex_index+1)%collision_obstacle.verticesCW().size()];
   vertex_distance = (vertex2 - vertex1).norm();
   // Calculate the heading as the direction between vertex1 and vertex2
   state[1] = (vertex2 - vertex1).normalized()*step_size;
   // Update state[2] as the right-hand direction (perpendicular to the heading direction)
   state[2] = Eigen::Vector2d(state[1].y(), -state[1].x());  // Rotate the direction vector for the right-hand side
   // Debug output
   // std::cout << "State 1: " << state[1].transpose() << std::endl;
   // std::cout << "State 2: " << state[2].transpose() << std::endl;


   return state;
}

std::vector<Eigen::Vector2d> MyBugAlgorithm::return_to_leave_point(const amp::Problem2D& problem, amp::Path2D& path,std::vector<Eigen::Vector2d> state,std::vector<Eigen::Vector2d> collision_path ,Eigen::Vector2d leave_point,int leave_index, int& mode){
// find the shortest path through collision path to leave point
   std::cout << "Mode 2: Hit Point Reached" << std::endl;
   double step_size = 0.01;
   // std::cout<<leave_index<<std::endl;
   // std::cout<<collision_path.size()<<std::endl;
   double half = collision_path.size()/2;
   if (leave_index < half) {
       // Move forward through the collision_path starting from leave_index
       for (int i = 0; i < leave_index; ++i) {
           path.waypoints.push_back(collision_path[i]);
       }
   } else {
       // Move backward through the collision_path starting from leave_index
       for (int i = collision_path.size()-1; i >= leave_index; --i) {  // Use >= 0 to include index 0
           path.waypoints.push_back(collision_path[i]);
       }
   }
   std::cout << "Mode 0: Leave Point: Reached" << std::endl;
   // std::cout <<leave_point<<std::endl;
   state[0] = path.waypoints.back();
   // std::cout<<state[0]<<std::endl;
   // state[1] = ((problem.q_goal - state[0]).normalized() * step_size);
   // state[2] = Eigen::Vector2d(state[1].y(), -state[1].x());
   mode = 0;
   std::cout << mode << std::endl;
   return state;


}

bool MyBugAlgorithm::lineSegmentIntersection_mline(const amp::Problem2D& problem, Eigen::Vector2d position, Eigen::Vector2d POI,double& distance_to_goal){
    distance_to_goal = (problem.q_goal - position).norm();
    if(Intersect(position,POI,problem.q_init,problem.q_goal)){
        return true;
    }
   return false;
}



// Archive

// problem.obstacles[].vertices.cw;

// bool MyBugAlgorithm::insidePolygon(const amp::Problem2D& problem, amp::Path2D& path,const Eigen::Vector2d& point){
//    bool Inside = false;
//    double x1;
//    double y1;
//    double x2;
//    double y2;
//    double x = point[0];
//    double y = point[1];
//    for (int i = 0; i < problem.obstacles.size(); i++){
//         for (int j = 0; j < problem.obstacles[i].verticesCW().size(); j++){
//            if (j == problem.obstacles[i].verticesCW().size() - 1){
//                x1 = problem.obstacles[i].verticesCW()[j][0];
//                y1 = problem.obstacles[i].verticesCW()[j][1];
//                x2 = problem.obstacles[i].verticesCW()[0][0];
//                y2 = problem.obstacles[i].verticesCW()[0][1];
//                if (((y1 > y) != (y2 > y) && (x < x1 + ((y - y1) * (x2 - x1) / (y2 - y1))))){
//                    Inside = !Inside;
//                }
//            }
//            else{
//                x1 = problem.obstacles[i].verticesCW()[j][0];
//                y1 = problem.obstacles[i].verticesCW()[j][1];
//                x2 = problem.obstacles[i].verticesCW()[j+1][0];
//                y2 = problem.obstacles[i].verticesCW()[j+1][1];
//                if (((y1 > y) != (y2 > y) && (x < x1 + ((y - y1) * (x2 - x1) / (y2 - y1))))){
//                    Inside = !Inside;
//                }
//            }
//         }
//    }
//    return Inside;
// }

// std::vector<Eigen::Vector2d> MyBugAlgorithm::obstaclefollowing(const amp::Problem2D& problem,amp::Path2D& path,std::vector<Eigen::Vector2d> state,int& collision_vertex_index, amp::Obstacle2D& collision_obstacle,std::vector<Eigen::Vector2d>& collision_path,Eigen::Vector2d& leave_point,int& leave_index,int& mode){
//    std::cout << "Mode 1: Obstacle Encountered" << std::endl;
//    int count = 1;
//    int step =1;
//    double step_size = 0.1;
//    double tolerance = 0.1;
//    double min_goal_distance = (problem.q_goal - state[0]).norm();
//    Eigen::Vector2d hit_point = state[0];
//    state  = align_to_obstacle(problem, state, collision_vertex_index, collision_obstacle);
//    while(1){
//        if(!lineSegmentIntersection(problem,state[0],(state[1])+state[0],collision_vertex_index,collision_obstacle)){
//            if(lineSegmentIntersection(problem,state[0],(state[2])+state[0],collision_vertex_index,collision_obstacle)){
//                state[0] += state[1];
//                path.waypoints.push_back(state[0]);
//            }
//            else{
//                // Increment collision vertex index
//                collision_vertex_index = (collision_vertex_index + 1) % collision_obstacle.verticesCW().size();
//                // Navigate around the obstacle
//                // path.waypoints.pop_back();
//                // state[0] = path.waypoints.back();
//                state = align_to_obstacle(problem, state, collision_vertex_index, collision_obstacle);
//                 if(!lineSegmentIntersection(problem,state[0],(state[1])+state[0],collision_vertex_index,collision_obstacle)){
//                    state[0] += state[1];
//                    path.waypoints.push_back(state[0]);
//                 }else{
//                    state = align_to_obstacle(problem, state, collision_vertex_index, collision_obstacle);
//                 }
//            }
//        }else{
//        
//            // Navigate around the obstacle
//            // path.waypoints.pop_back();
//            // state[0] = path.waypoints.back();
//            count++;
//            state = align_to_obstacle(problem, state, collision_vertex_index, collision_obstacle);
//            state[0] += state[1];
//            path.waypoints.push_back(state[0]);
//        }
//        collision_path.push_back(state[0]);
//        // // Break Conditions
//        // double distance_to_goal = (state[0] - problem.q_goal).norm();
//        // if(distance_to_goal < tolerance){
//        //     path.waypoints.push_back(problem.q_goal);
//        //     std::cout << "GOOOOOOOOOOAAAAAAAAALLLLLLL" << std::endl;
//        //     break;  
//        // }
//        if (min_goal_distance > (problem.q_goal - state[0]).norm()){
//            min_goal_distance = (problem.q_goal - state[0]).norm();
//            leave_point = state[0];
//            leave_index = collision_path.size()-1;
//        }
//        double distance_to_hitpoint = (state[0] - hit_point).norm();
//        // Check if the distance is within the tolerance
//        if (distance_to_hitpoint < tolerance) {
//            std::cout<<hit_point<<std::endl;
//            std::cout << "Hit Point Reached" << std::endl;
//            mode = 2;
//            break;
//        }
//        if(step >10000){
//            std::cout << "Loop Broken" << std::endl;
//            break;
//        }
//        step++;
//    }
//    return state;
// }
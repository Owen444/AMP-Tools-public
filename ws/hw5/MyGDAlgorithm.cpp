#include "MyGDAlgorithm.h"
// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    Eigen::Vector2d new_q;
    path.waypoints.push_back(problem.q_init);
    MyPotentialFunction potential(*this, problem);
    double learning_rate = 0.4;
    double momentum = 0.1;
    int main_count = 0;
    while ((problem.q_goal - path.waypoints.back()).norm() > 0.1) {
        Eigen::Vector2d q = path.waypoints.back();
        Eigen::Vector2d grad = potential.Gradient(q);
        bool intersect = true;
        int count = 0;
        if (grad.norm() > 20) {
            // std::cout<<"BIG BOI"<<std::endl;
            grad = grad.normalized();
            q = q - 0.01 * grad;
            path.waypoints.push_back(q);
        } else if (grad.norm() < 0.1) {
            q = q - 0.01 * grad;
            path.waypoints.push_back(q);
            count = 0;
            // Take a step in a random direction
            // std::cout << "Taking Random Step" << std::endl;
            while(1){
                Eigen::Vector2d random_direction = Eigen::Vector2d::Random();
                q = q - 2*random_direction;
                // Check if the new point intersects with any obstacles
                intersect = HW4Functions::lineSegmentIntersection(problem, path.waypoints.back(), q);
                if (!intersect) {
                    path.waypoints.push_back(q);
                    break;
                }else{
                    count++;
                }
                if(count > 100){
                    std::cout << "No Valid Step Found" << std::endl;
                    break;
                }
            }
        } else {
            q = q-0.01*grad;
            path.waypoints.push_back(q);
        }
        main_count++;
        // std::cout << "Current Location:" << path.waypoints.back() << std::endl;
        // std::cout << "Distance to Goal:" << (problem.q_goal - path.waypoints.back()).norm() << std::endl;
        // std::cout << "Main Count:"<< main_count<< std::endl;
        if(main_count > 100000) {
            // std::cout << "Max Steps Reached" << std::endl;
            // std::cout<<"Current Location"<<path.waypoints.back()<<std::endl;
            break;
        }

    }
    if ((problem.q_goal - path.waypoints.back()).norm() <0.25) {
        std::cout << "Goal Reached" << std::endl;
        path.waypoints.push_back(problem.q_goal);
    }
    return path;
}

double MyPotentialFunction::Attractor(Eigen::Vector2d q) const {
    double eta = algorithm.getEta();
    double zetta = algorithm.getZetta();
    double d_star = algorithm.getDStar();
    double Q_star = algorithm.getQStar();
    Eigen::Vector2d q_goal = problem.q_goal;
    double distance_to_goal = (q_goal - q).norm();
    double U__att = 0;
    
    if (distance_to_goal <= d_star){
        U__att = 0.5*zetta * distance_to_goal*distance_to_goal;
    } else {
        U__att =d_star*zetta*distance_to_goal - 0.5*zetta*d_star*d_star;
    }
    return U__att;
}

double MyPotentialFunction::Repulsor(Eigen::Vector2d q) const {
    double eta = algorithm.getEta();
    double zetta = algorithm.getZetta();
    double d_star = algorithm.getDStar();
    double Q_star = algorithm.getQStar();
    double distance_to_obstacle;
    Eigen::Vector2d q_goal = problem.q_goal;
    double U_rep = 0;
    // Calculate the repulsive potential by looping though all obstacles
    for (const auto& obstacle : problem.obstacles){
       Eigen::Vector2d min_point = MinimumDistance(q, obstacle);
        distance_to_obstacle = (q - min_point).norm();
        if (distance_to_obstacle <= Q_star){
            U_rep += 0.5*eta*((1/distance_to_obstacle - 1/Q_star)*(1/distance_to_obstacle - 1/Q_star));
        } else {
            U_rep += 0;
        }
    }
    return U_rep;
   
}

Eigen::Vector2d MyPotentialFunction::MinimumDistance(Eigen::Vector2d q, const amp::Obstacle2D& obstacle) const {
    double min_distance_to_obstacle = std::numeric_limits<double>::max();
    Eigen::Vector2d min_point;
    Eigen::Vector2d vertex1;
    Eigen::Vector2d vertex2;
    Eigen::Vector2d AB;
    Eigen::Vector2d AQ;
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCW();

    for (int i = 0; i < vertices.size(); i++) {
        vertex1 = vertices[i];
        vertex2 = vertices[(i + 1) % vertices.size()];
        AB = vertex2 - vertex1;
        AQ = q - vertex1;
        double projection = (AQ.dot(AB) / AB.dot(AB));
        projection = std::max(0.0, std::min(1.0, projection));
        Eigen::Vector2d point = vertex1 + projection * AB;
        double distance = (q - point).norm();
        if (distance < min_distance_to_obstacle) {
            min_distance_to_obstacle = distance;
            min_point = point;
        }
    }
    return min_point;
}

Eigen::Vector2d MyPotentialFunction::Gradient(Eigen::Vector2d q) const{
    Eigen::Vector2d gradient;
    Eigen::Vector2d q_goal = problem.q_goal;
    Eigen::Vector2d q_init = problem.q_init;
    Eigen::Vector2d min_point;
    double eta = algorithm.getEta();
    double zetta = algorithm.getZetta();
    double d_star = algorithm.getDStar();
    double Q_star = algorithm.getQStar();
    double U_rep = 0;
    Eigen::Vector2d U_att_grad = Eigen::Vector2d::Zero();
    Eigen::Vector2d U_rep_grad = Eigen::Vector2d::Zero();
    Eigen::Vector2d grad;
    double distance_to_goal = (q-q_goal).norm();
    double distance_to_obstacle;
    // Calculate the attractive potential gradient
    if (distance_to_goal <= d_star){
        U_att_grad = zetta*(q-q_goal);
    } else {
        U_att_grad = d_star*zetta*(q-q_goal)/distance_to_goal;
    }
    // Calculate the repulsive potential gradient
    for (const auto& obstacle : problem.obstacles){
        min_point = MinimumDistance(q, obstacle);
        distance_to_obstacle = (q - min_point).norm();
        // std::cout << "Distance::" <<distance_to_obstacle << std::endl;
        // std::cout<< "Closest Point"<<min_point<<std::endl;
        if (distance_to_obstacle <= Q_star){
            U_rep_grad += eta*(1/Q_star-1/distance_to_obstacle)*(1/(distance_to_obstacle*distance_to_obstacle))*((q - min_point)/distance_to_obstacle);
        } else{
            U_rep = 0;
            U_rep_grad += U_rep*(q - min_point)/distance_to_obstacle;
        }
    }
    // std::cout << "U_att_grad::" << U_att_grad << std::endl;
    // std::cout << "U_rep_grad::" << U_rep_grad << std::endl;
    grad = U_att_grad + U_rep_grad;
    // std::cout << "Gradient::" << grad << std::endl;
    return grad;
}

Eigen::Vector2d MyPotentialFunction::NesterovUpdate(Eigen::Vector2d& q, double learning_rate, double momentum) {
    // Lookahead step
    Eigen::Vector2d lookahead_q = q - momentum * velocity;
    // Calculate gradient at the lookahead position
    Eigen::Vector2d grad = Gradient(lookahead_q);
    // Update velocity
    velocity = momentum * velocity - learning_rate * grad;
    // Update position
    q = q + velocity;
    return q;
}
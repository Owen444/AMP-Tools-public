#include "ManipulatorSkeleton.h"
#include <cmath>
#include <vector>
#include <Eigen/Dense>

MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)
    std::vector<Eigen::Vector2d> joint_positions;
    double rotation_angle =0;
    double x = 0.0;
    double y = 0.0;
    std::vector a  = getLinkLengths();
    joint_positions.push_back(Eigen::Vector2d(0.0, 0.0));
    for (int i=0; i<joint_index; i++){
        rotation_angle += state[i];
        x += a[i]*cos(rotation_angle);
        y += a[i]*sin(rotation_angle);
        joint_positions.push_back(Eigen::Vector2d(x, y));
    }
    if(joint_index == a.size()){
        //std::cout << "End Effector Positions: " << joint_positions[joint_index] << std::endl;
    }
    //std::cout << "joint_positions: " << joint_positions[joint_index] << std::endl;
    return joint_positions[joint_index];
}

// // Override this method for implementing inverse kinematics
// amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
//     // Implement inverse kinematics here
//     std::vector a  = getLinkLengths();
//      amp::ManipulatorState joint_angles(nLinks());
//      joint_angles.setZero();
//     // If you have different implementations for 2/3/n link manipulators, you can separate them here
//     if (nLinks() == 2) {
//         double alpha = acos((-end_effector_location.x()*end_effector_location.x() - end_effector_location.y()*end_effector_location.y() + a[0]*a[0] + a[1]*a[1])/(2*a[0]*a[1]));
//         double beta =  acos((a[0]*a[0] - a[1]*a[1] + end_effector_location.x()*end_effector_location.x() + end_effector_location.y()*end_effector_location.y())/(2*sqrt(end_effector_location.x()*end_effector_location.x() + end_effector_location.y()*end_effector_location.y())*a[0]));
//         joint_angles[0] = atan2(end_effector_location.y(),end_effector_location.x()) - beta;
//         joint_angles[1] = M_PI - alpha;
//         if(joint_angles[0]==NAN || joint_angles[1]==NAN){
//             joint_angles.setZero();
//         }
//         return joint_angles;
//     } else if (nLinks() == 3) {
//         double gamma = atan2(end_effector_location.y(), end_effector_location.x());
//         double P3_x = end_effector_location.x() - a[2]*cos(gamma); 
//         double P3_y = end_effector_location.y() - a[2]*sin(gamma);
//         double alpha = acos((-P3_x*P3_x - P3_y*P3_y + a[0]*a[0] + a[1]*a[1])/(2*a[0]*a[1]));
//         double beta =  acos((a[0]*a[0] - a[1]*a[1] + P3_x*P3_x + P3_y*P3_y)/(2*sqrt(P3_x*P3_x + P3_y*P3_y)*a[0]));
//         joint_angles[0] = atan2(P3_y, P3_x) - beta;
//         joint_angles[1] = M_PI - alpha;
//         joint_angles[2] = gamma - joint_angles[0] - joint_angles[1];
//         if(joint_angles[0]==NAN || joint_angles[1]==NAN || joint_angles[2]==NAN){
//             joint_angles.setZero();
//         }
//         return joint_angles;
//     } else {
//         return joint_angles;
//     }

//     return joint_angles;
// }

amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here
    std::vector<double> a = getLinkLengths();
    amp::ManipulatorState joint_angles(nLinks());
    joint_angles.setZero();
    double gamma;
    std::cout<<a.size()<<std::endl;
    // std::cout << "End effector location: " << end_effector_location.transpose() << std::endl;
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
        double d = end_effector_location.norm();
        if (d > a[0] + a[1] || d < std::abs(a[0] - a[1])) {
            // The target is unreachable
            return joint_angles;
        }

        double alpha = acos((a[0]*a[0] + a[1]*a[1] - d*d) / (2 * a[0] * a[1]));
        double beta = acos((d*d + a[0]*a[0] - a[1]*a[1]) / (2 * d * a[0]));
        joint_angles[0] = atan2(end_effector_location.y(), end_effector_location.x()) - beta;
        joint_angles[1] = M_PI - alpha;

        if (std::isnan(joint_angles[0]) || std::isnan(joint_angles[1])) {
            joint_angles.setZero();
        }
        return joint_angles;
    }else if (nLinks() == 3) {   
        for (int i = 0; i < 360; i++) {
            gamma = 2*M_PI/360*i;
            // std::cout << "gamma: " << gamma << std::endl;
            double P3_x = end_effector_location.x() - a[2] * cos(gamma);
            double P3_y = end_effector_location.y() - a[2] * sin(gamma);
            double d = sqrt(P3_x * P3_x + P3_y * P3_y);
            // Ensure the argument to acos is within the valid range [-1, 1]
            double cos_alpha = (a[0]*a[0] + a[1]*a[1] - d*d) / (2 * a[0] * a[1]);
            double cos_beta = (d*d + a[0]*a[0] - a[1]*a[1]) / (2 * d * a[0]);
            double alpha = acos(cos_alpha);
            double beta = acos(cos_beta);
            joint_angles[0] = atan2(P3_y, P3_x) - beta;
            joint_angles[1] = M_PI - alpha;
            joint_angles[2] = gamma - joint_angles[0] - joint_angles[1];
            if (std::isnan(joint_angles[0]) || std::isnan(joint_angles[1]) || std::isnan(joint_angles[2])) {
                //std::cerr << "NaN detected in joint angles: " << joint_angles.transpose() << std::endl;
                joint_angles.setZero();
            }
                Eigen::Vector2d test_end_location = getJointLocation(joint_angles, 3);
                
                if ((test_end_location - end_effector_location).norm() < 0.01) {
                    // std::cout << "Solution Found " << std::endl;
                    return joint_angles;
                }
        }
    }else{
        std::cerr << "Invalid number of links" << std::endl;
        return joint_angles;
    }
    return joint_angles;
}
#include "MyKinoRRT.h"
using namespace amp;
void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    int numSteps = 200;
    double h = dt / numSteps;
        
    for (int step = 0; step < numSteps; step++) {
        // k1 = f(t_n, y_n)
        Eigen::VectorXd k1 = control;
            
        // k2 = f(t_n + h/2, y_n + (h/2)k1)
        Eigen::VectorXd state_k2 = state + (h/2) * k1;
        Eigen::VectorXd k2 = control;
            
        // k3 = f(t_n + h/2, y_n + (h/2)k2)
        Eigen::VectorXd state_k3 = state + (h/2) * k2;
        Eigen::VectorXd k3 = control;
            
            // k4 = f(t_n + h, y_n + hk3)
        Eigen::VectorXd state_k4 = state + h * k3;
        Eigen::VectorXd k4 = control;

        // Update state using RK4 formula
        state = state + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
    }
};

Eigen::VectorXd MyKinoRRT::get_random_state(const amp::KinodynamicProblem2D& problem, double p_goal){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x(problem.x_min, problem.x_max);
    std::uniform_real_distribution<double> y(problem.y_min, problem.y_max);
    std::uniform_real_distribution<double> theta(-M_PI, M_PI);
    std::uniform_real_distribution<double> sigma(-3, 4);
    std::uniform_real_distribution<double> w(-1.5, 1.5);
    std::uniform_real_distribution<double> v(-3, 6);
    std::uniform_real_distribution<double> phi(-M_PI/3, M_PI/3);
    // Sample a random configuration
    std::uniform_real_distribution<double> random_number(0,1);
    Eigen::VectorXd q_rand;
    if(problem.q_goal.size()== 2){
        q_rand = Eigen::VectorXd(2);
        if(random_number(gen)<p_goal){
        q_rand << (problem.q_goal[0].first+problem.q_goal[0].second)/2, (problem.q_goal[1].first+problem.q_goal[1].second)/2;
    }
    else{
        q_rand << x(gen), y(gen);
        }
    }
    if(problem.q_goal.size()== 3){
        q_rand = Eigen::VectorXd(3);
        if(random_number(gen)<p_goal){
            q_rand << (problem.q_goal[0].first+problem.q_goal[0].second)/2, (problem.q_goal[1].first+problem.q_goal[1].second)/2, (problem.q_goal[2].first+problem.q_goal[2].second)/2;
        }
        else{
            q_rand << x(gen), y(gen), theta(gen);
        }
    }
    if(problem.agent_type == AgentType::SecondOrderUnicycle){
        q_rand = Eigen::VectorXd(5);
        if(random_number(gen)<p_goal){
           q_rand << (problem.q_goal[0].first+problem.q_goal[0].second)/2, (problem.q_goal[1].first+problem.q_goal[1].second)/2, (problem.q_goal[2].first+problem.q_goal[2].second)/2, (problem.q_goal[3].first+problem.q_goal[3].second)/2, (problem.q_goal[4].first+problem.q_goal[4].second)/2;
        }
        else{
            q_rand << x(gen), y(gen), theta(gen), sigma(gen), w(gen);
        }
    }
    if(problem.agent_type == AgentType::SimpleCar){
        q_rand = Eigen::VectorXd(5);
        if(random_number(gen)<p_goal){
            q_rand << (problem.q_goal[0].first+problem.q_goal[0].second)/2, (problem.q_goal[1].first+problem.q_goal[1].second)/2, (problem.q_goal[2].first+problem.q_goal[2].second)/2, (problem.q_goal[3].first+problem.q_goal[3].second)/2, (problem.q_goal[4].first+problem.q_goal[4].second)/2;
        }
        else{
            q_rand << x(gen), y(gen), theta(gen), v(gen), phi(gen);
        }
    }
    return q_rand;
}

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    double r = 0.25;
    state(0) += control(0)*r*cos(state(2))*dt;
    state(1) += control(0)*r*sin(state(2))*dt;
    state(2) += control(1)*dt;
    
}

void MySecondOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt){
    double r = 0.25;
    state(0) += state(3)*r*cos(state(2))*dt;
    state(1) += state(3)*r*sin(state(2))*dt;
    state(2) += state(4)*dt;
    state(3) += control(0)*dt;
    state(4) += control(1)*dt;
}

void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    double L = agent_dim.length;
    state(0) += state(3)*cos(state(2))*dt;
    state(1) += state(3)*sin(state(2))*dt;
    state(2) += (state(3)/L)*tan(state(4))*dt;
    state(3) += control(0)*dt;
    state(4) += control(1)*dt;

}

void propagate_car(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt,double L) {
    state(0) += state(3)*cos(state(2))*dt;
    state(1) += state(3)*sin(state(2))*dt;
    state(2) += (state(3)/L)*tan(state(4))*dt;
    state(3) += control(0)*dt;
    state(4) += control(1)*dt;
}

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem_org, amp::DynamicAgent& agent) {
    //Recreate probelm with correct bounds
    amp::KinodynamicProblem2D problem = problem_org;
    if(problem.agent_type == AgentType::FirstOrderUnicycle){
        problem.u_bounds[0] = {-2, 4};
        problem.u_bounds[1] = {-1.5, 1.5};
    }
    if(problem.agent_type == AgentType::SecondOrderUnicycle){
        problem.u_bounds[0] = {-1, 1.5};
        problem.u_bounds[1] = {-0.75, 0.75};
    }
    if(problem.agent_type == AgentType::SimpleCar){
        problem.u_bounds[0] = {-1.5, 2};
        problem.u_bounds[1] = {-0.3, 0.3};
    }
    // Create path, node map and graph
    amp::KinoPath path;
    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::map<amp::Node, amp::Node> parent_map;
    std::map<amp::Node, Eigen::VectorXd> control_map;
    agent.agent_dim = problem.agent_dim;
    // Initialize the first node
    nodes[0] = problem.q_init;
    control_map[0] = Eigen::VectorXd::Zero(problem.q_init.size());
    // Define constants
    int n = m_numSamples;
    double radius = m_connectionRadius;
    double p_goal = 0.05;
    double dt = 0.25;
    int iterations = 0;
    int max_iterations = 150000;
    // While solution is not found and number of samples is less than n
    while (true){
        Eigen::VectorXd q_rand = get_random_state(problem, p_goal);
        // Find nearest node to q_rand
        double distance_min = std::numeric_limits<double>::max();
        amp::Node nearest_node;
        for (const auto& [node, q] : nodes) {
            double dist2node = (q - q_rand).norm();
            if (dist2node < distance_min) {
                distance_min = dist2node;
                nearest_node = node;
            }
        }
        Eigen::VectorXd q_near = nodes[nearest_node];
        auto [q_new, control] = GenerateLocalTrajectory(problem, agent, q_near, q_rand,dt);
        std::pair<bool, Eigen::VectorXd> valid = IsSubTrajectoryValid(problem, agent, q_near, control, dt);
        q_new << valid.second;
        if(valid.first){
            nodes[nodes.size()] = q_new;
            parent_map[nodes.size()-1] = nearest_node;
            control_map[nodes.size()-1] = control;
        }
        // Check if the new node is in the goal region
        bool in_goal_region = true;
        for (int i = 0; i < problem.q_goal.size(); i++) {
            if (q_new(i) < problem.q_goal[i].first || q_new(i) > problem.q_goal[i].second) {
                in_goal_region = false;
                break;
            }
        }
        if(in_goal_region){
            std::cout<<"Goal reached"<<std::endl;
            path.valid = true;
            break;
        }
        // Check if the iterations is greater than max_iterations
        if(iterations>max_iterations){
            std::cout<<"Max iterations reached"<<std::endl;
            break;
        }
        iterations++;
    }
    std::cout<<"Iteration: "<<iterations<<std::endl;
    // Create the path
    amp::Node current_node = nodes.size()-1;
    Eigen::VectorXd current_state = nodes[current_node];
    Eigen::VectorXd current_control = control_map[current_node];
    while(current_node != 0){
        path.waypoints.push_back(current_state);
        path.controls.push_back(current_control);
        path.durations.push_back(dt);
        current_node = parent_map[current_node];
        current_state = nodes[current_node];
        current_control = control_map[current_node];
    }
    path.waypoints.push_back(problem.q_init);
    path.controls.push_back(Eigen::VectorXd::Zero(2));
    path.durations.push_back(0);
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    std::reverse(path.controls.begin(), path.controls.end());
    std::reverse(path.durations.begin(), path.durations.end());
    // Simulate the path
    //amp::KinoPath simulated_path = simulatePathRK4(path, agent);
    return path;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> MyKinoRRT::GenerateLocalTrajectory(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, Eigen::VectorXd q_near,Eigen::VectorXd q_rand,double dt) {
    double min_distance = std::numeric_limits<double>::max();
    Eigen::VectorXd q_new, best_control;
    for (int i = 0; i < m_numSamples; i++) {
        Eigen::VectorXd control = Eigen::VectorXd::Random(2);
        for (int j = 0; j < control.size(); j++) {
            control(j) = ((problem.u_bounds[j].second - problem.u_bounds[j].first)/2)*control(j)  + ((problem.u_bounds[j].second + problem.u_bounds[j].first)/2);
        }
        Eigen::VectorXd state = q_near;
        //output agent legnth and width
        if(problem.agent_type == AgentType::SimpleCar){
            //std::cout<<"CAR"<<std::endl;
            //std::cout<<"Initial state: "<<state.transpose()<<std::endl;
            propagate_car(state, control, dt,agent.agent_dim.length);
            //std::cout<<"Final state: "<<state.transpose()<<std::endl;
        }else{
            agent.propagate(state, control, dt);
        }
        double distance = (q_rand - state).norm();
        if(distance < min_distance){
            min_distance = distance;
            q_new = state;
            best_control = control;
        }
    }
    // std::cout<<"Q_near: "<<q_near.transpose()<<std::endl;
    //std::cout<<"Q_new: "<<q_new.transpose()<<std::endl;
    return {q_new, best_control};
}

std::pair<bool, Eigen::VectorXd> MyKinoRRT::IsSubTrajectoryValid(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, Eigen::VectorXd q_near, Eigen::VectorXd control, double dt) {
    Eigen::VectorXd q_step = q_near;
    int steps = 300;
    if(problem.agent_type == AgentType::SimpleCar){
        steps = 200;
        for (int i = 0; i < steps; i++) {
            simulate_car(q_step, control, dt/steps,agent.agent_dim.length);
            bool polygon_collision = PolygonPolygonIntersection(problem, agent, q_step);
            if(polygon_collision){
                return {false, q_step};
            }
        }
    }else{
        for (int i = 0; i < steps; i++) {
            agent.propagate(q_step, control, dt/steps);
            bool inside_obstacle = false;
            for (const auto& obstacle : problem.obstacles) {
            inside_obstacle = point_in_polygon(q_step, obstacle);
            Eigen::Vector2d min_point = MinimumPoint(q_step, obstacle);
            if (inside_obstacle || (min_point-q_step).norm() < 0.3) {
                return {false, q_step};
            }
        }
        bool line_collision = HW4Functions::lineSegmentIntersection(problem, q_near, q_step);
        if(line_collision) {
            return {false, q_step};
        }
        if(q_step(0)<problem.x_min || q_step(0)>problem.x_max || q_step(1)<problem.y_min || q_step(1)>problem.y_max){
                return {false, q_step};
            }
        }
    }
    return {true, q_step};
}

bool point_in_polygon(Eigen::Vector2d point, const amp::Obstacle2D& obstacle){
    std::vector<Eigen::Vector2d> polygon = obstacle.verticesCW();
    int num_vertices = polygon.size();
    double x = point.x(), y = point.y();
    bool inside = false;
 
    // Store the first point in the polygon and initialize
    // the second point
    Eigen::Vector2d p1 = polygon[0], p2;
 
    // Loop through each edge in the polygon
    for (int i = 1; i <= num_vertices; i++) {
        // Get the next point in the polygon
        p2 = polygon[i % num_vertices];
 
        // Check if the point is above the minimum y
        // coordinate of the edge
        if (y > std::min(p1.y(), p2.y())) {
            // Check if the point is below the maximum y
            // coordinate of the edge
            if (y <= std::max(p1.y(), p2.y())) {
                // Check if the point is to the left of the
                // maximum x coordinate of the edge
                if (x <= std::max(p1.x(), p2.x())) {
                    // Calculate the x-intersection of the
                    // line connecting the point to the edge
                    double x_intersection
                        = (y - p1.y()) * (p2.x() - p1.x())
                              / (p2.y() - p1.y())
                          + p1.x();
 
                    // Check if the point is on the same
                    // line as the edge or to the left of
                    // the x-intersection
                    if (p1.x() == p2.x()
                        || x <= x_intersection) {
                        // Flip the inside flag
                        inside = !inside;
                    }
                }
            }
        }
 
        // Store the current point as the first point for
        // the next iteration
        p1 = p2;
    }
 
    // Return the value of the inside flag
    return inside;
}

Eigen::Vector2d MinimumPoint(Eigen::Vector2d q, const amp::Obstacle2D& obstacle){
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

void MyKinoRRT::simulate_car(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt, double L) {
    // Calculate k1
    Eigen::VectorXd k1(5);
    k1 << state(3)*cos(state(2)), 
          state(3)*sin(state(2)), 
          (state(3)/L)*tan(state(4)), 
          control(0), 
          control(1);
    
    // Calculate k2
    Eigen::VectorXd state_k2 = state + (dt/2) * k1;
    Eigen::VectorXd k2(5);
    k2 << state_k2(3)*cos(state_k2(2)),
          state_k2(3)*sin(state_k2(2)),
          (state_k2(3)/L)*tan(state_k2(4)),
          control(0),
          control(1);
    
    // Calculate k3
    Eigen::VectorXd state_k3 = state + (dt/2) * k2;
    Eigen::VectorXd k3(5);
    k3 << state_k3(3)*cos(state_k3(2)),
          state_k3(3)*sin(state_k3(2)),
          (state_k3(3)/L)*tan(state_k3(4)),
          control(0),
          control(1);
    
    // Calculate k4
    Eigen::VectorXd state_k4 = state + dt * k3;
    Eigen::VectorXd k4(5);
    k4 << state_k4(3)*cos(state_k4(2)),
          state_k4(3)*sin(state_k4(2)),
          (state_k4(3)/L)*tan(state_k4(4)),
          control(0),
          control(1);
    
    // Update state using RK4 formula
    state = state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
}

bool MyKinoRRT::PolygonPolygonIntersection(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent,Eigen::VectorXd state){
    std::vector<Eigen::Vector2d> robot_vertices;
    // The reference point is the center of the rear axle
    double x_ref = state(0);
    double y_ref = state(1);
    double theta = state(2);
    double L = agent.agent_dim.length;
    double W = agent.agent_dim.width;
    // Unrotated vertices relative to the center of the rear axle
    Eigen::Vector2d front_left = Eigen::Vector2d(L, W/2.0);
    Eigen::Vector2d front_right = Eigen::Vector2d(L, -W/2.0);
    Eigen::Vector2d rear_left = Eigen::Vector2d(0, W/2.0);
    Eigen::Vector2d rear_right = Eigen::Vector2d(0, -W/2.0);
    // Deine rotation matrix
    Eigen::Matrix2d rotation;
    rotation << cos(theta), -sin(theta), sin(theta), cos(theta);
    // Rotate the vertices and translate to the global frame
    front_left = (rotation * front_left) + Eigen::Vector2d(x_ref, y_ref);
    front_right = (rotation * front_right) + Eigen::Vector2d(x_ref, y_ref);
    rear_left = (rotation * rear_left) + Eigen::Vector2d(x_ref, y_ref);
    rear_right = (rotation * rear_right) + Eigen::Vector2d(x_ref, y_ref);
    robot_vertices.push_back(front_left);
    robot_vertices.push_back(front_right);
    robot_vertices.push_back(rear_right);
    robot_vertices.push_back(rear_left);

    //output the problem bounds
    //std::cout<<"Problem bounds: "<<problem.x_min<<", "<<problem.x_max<<", "<<problem.y_min<<", "<<problem.y_max<<std::endl;
    // Check for intersection between robot and obstacles
    for (int i = 0; i < robot_vertices.size(); i++) {
        bool line_collision = HW4Functions::lineSegmentIntersection(problem, robot_vertices[i], robot_vertices[(i+1)%robot_vertices.size()]);
        if(line_collision){
            return true;
        }
    }

    // Check that all vertices are within the bounds of the environment
    for (int i = 0; i < robot_vertices.size(); i++) {
        if(robot_vertices[i](0) < problem.x_min + 0.2 || 
           robot_vertices[i](0) > problem.x_max - 0.2 || 
           robot_vertices[i](1) < problem.y_min + 0.2 || 
           robot_vertices[i](1) > problem.y_max - 0.2) {
            return true;
        }
    }

    // Also check the center point
    if(x_ref < problem.x_min || 
       x_ref > problem.x_max || 
       y_ref < problem.y_min || 
       y_ref > problem.y_max) {
        return true;
    }

    return false;
}
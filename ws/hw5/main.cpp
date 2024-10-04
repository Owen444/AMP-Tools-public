// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"
// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test your gradient descent algorithm on a random problem.
    //double d_star, double zetta, double Q_star, double eta
    MyGDAlgorithm algo(2, 1.5, 1, .9);


    // amp::Problem2D prob = HW5::getWorkspace1();
    // amp::Path2D path = algo.plan(prob);
    // amp::Visualizer::makeFigure(prob, path);
    // amp::Visualizer::makeFigure(MyPotentialFunction{algo,prob}, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 500);
    // bool success = HW5::generateAndCheck(algo, path, prob);

    amp::Problem2D prob = HW2::getWorkspace1();
    amp::Path2D path = algo.plan(prob);
    amp::Visualizer::makeFigure(prob, path);
    amp::Visualizer::makeFigure(MyPotentialFunction{algo,prob}, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 500);
    //bool success = HW5::generateAndCheck(algo, path, prob);

    // amp::Path2D path1;
    // amp::Problem2D prob1;
    // bool success = HW5::generateAndCheck(algo, path1, prob1);
    // Visualizer::makeFigure(prob1, path1);

    // Visualize your potential function
    Visualizer::showFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    //HW5::grade<MyGDAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, 1.0, 1.0, 1.0, 1.0);
    return 0;
}
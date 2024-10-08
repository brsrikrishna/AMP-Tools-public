// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    //amp::RNG::seed(amp::RNG::randiUnbounded());
    Problem2D problem = HW5::getWorkspace1();
    problem.obstacles.at(0).print();
    // Test your gradient descent algorithm on a random problem.
<<<<<<< HEAD
    MyGDAlgorithm algo(1.0, 1.0, 1.5, 0.25);
    amp::Path2D path;
    amp::Problem2D prob;
=======
    MyGDAlgorithm algo(1.0, 1.0, 1.0, 1.0);
    Path2D path;
    Problem2D prob;
>>>>>>> bf8c7cec16243ee2b85b056a6eff23b6b246326d
    bool success = HW5::generateAndCheck(algo, path, prob);
    Visualizer::makeFigure(prob, path);
    Visualizer::showFigures();
    // Visualize your potential function
<<<<<<< HEAD
    amp::Visualizer::makeFigure(MyPotentialFunction{}, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 20);
    //Visualizer::showFigures();
=======
    Visualizer::makeFigure(MyPotentialFunction{}, prob, 30);
    Visualizer::showFigures();
>>>>>>> bf8c7cec16243ee2b85b056a6eff23b6b246326d
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("srikrishna.bangaloreraghu@colorado.edu", argc, argv, 1.0, 1.0, 1.5, 0.25);
    return 0;
}
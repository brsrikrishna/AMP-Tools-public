// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include <cmath>
#include "math.h"
// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    
    amp::Obstacle2D obstacle = HW4::getEx1TriangleObstacle();
    std::vector<Eigen::Vector2d> vertices_ccw_robot;
    std::vector<Eigen::Vector2d> vertices_ccw_obstacle;
    vertices_ccw_obstacle = obstacle.verticesCCW(); 
    
    ///Flipping the Robot

    for(int i=0; i<3; i++)
    {
        vertices_ccw_robot.push_back(-1*vertices_ccw_obstacle.at(i));
    }
    
    ///Ensuring the bottom left corner is the first in the polygon with CCW direction

    Eigen::Vector2d temp = vertices_ccw_robot.at(0);
    vertices_ccw_robot.at(0) = vertices_ccw_robot.at(1);
    vertices_ccw_robot.at(1) = temp;

    temp = vertices_ccw_robot.at(1);
    vertices_ccw_robot.at(1) = vertices_ccw_robot.at(2);
    vertices_ccw_robot.at(2) = temp;
    amp::Obstacle2D robot(vertices_ccw_robot);

    //Obtaining the angles made by the polygon sides wrt x axis

    std::vector<double> obstacle_angles{atan2((vertices_ccw_obstacle[1][1]-vertices_ccw_obstacle[0][1]),(vertices_ccw_obstacle[1][0]-vertices_ccw_obstacle[0][0])),atan2((vertices_ccw_obstacle[2][1]-vertices_ccw_obstacle[1][1]),(vertices_ccw_obstacle[2][0]-vertices_ccw_obstacle[1][0])),atan2((vertices_ccw_obstacle[0][1]-vertices_ccw_obstacle[2][1]),(vertices_ccw_obstacle[0][0]-vertices_ccw_obstacle[2][0]))};
    std::vector<double> robot_angles{atan2((vertices_ccw_robot[1][1]-vertices_ccw_robot[0][1]),(vertices_ccw_robot[1][0]-vertices_ccw_robot[0][0])),atan2((vertices_ccw_robot[2][1]-vertices_ccw_robot[1][1]),(vertices_ccw_robot[2][0]-vertices_ccw_robot[1][0])),atan2((vertices_ccw_robot[0][1]-vertices_ccw_robot[2][1]),(vertices_ccw_robot[0][0]-vertices_ccw_robot[2][0]))};

    //Convert all angles to 0 to 2*pi

    for(int i=0; i<3; i++)
    {
        if(obstacle_angles.at(i) < 0)
        {
            obstacle_angles.at(i) = 2*3.14159 + obstacle_angles.at(i);
        }
        if(robot_angles.at(i) < 0)
        {
            robot_angles.at(i) = 2*3.14159 + robot_angles.at(i);
        }
        std::cout<<"OBSTACLE"<<obstacle_angles.at(i)*180/3.14159<<std::endl;
        std::cout<<"ROBOT"<<robot_angles.at(i)*180/3.14159<<std::endl;
    }

    std::vector<Eigen::Vector2d> final_c_space_vertices_CCW{vertices_ccw_robot[0]+vertices_ccw_obstacle[0]};
    int current_robot_index = 0;
    int current_obstacle_index = 0;
    while(obstacle_angles.size()!=0 && robot_angles.size()!=0)
    {
        if(obstacle_angles[0]>robot_angles[0])
        {
            std::cout<<"Obstacle Angle"<<obstacle_angles[0]*180/3.14159<<"is greater than"<<robot_angles[0]*180/3.14159<<std::endl;
            current_robot_index+=1;
            if (current_robot_index == 3)
            {
                current_robot_index = 0;
            }
            std::cout<<"NEWLY ADDED VECTOR IS OBSTACLE"<<current_obstacle_index<<"AND ROBOT"<<current_robot_index<<std::endl;
            final_c_space_vertices_CCW.push_back(vertices_ccw_robot[current_robot_index]+vertices_ccw_obstacle[current_obstacle_index]);
            robot_angles.erase((robot_angles.begin()));
        }
        else
        {
            std::cout<<"Obstacle Angle"<<obstacle_angles[0]*180/3.14159<<"is lesser than"<<robot_angles[0]*180/3.14159<<std::endl;
            current_obstacle_index+=1;
            if (current_obstacle_index == 3)
            {
                current_obstacle_index = 0;
            }
            std::cout<<"NEWLY ADDED VECTOR IS OBSTACLE"<<current_obstacle_index<<"AND ROBOT"<<current_robot_index<<std::endl;
            final_c_space_vertices_CCW.push_back(vertices_ccw_robot[current_robot_index]+vertices_ccw_obstacle[current_obstacle_index]);
            obstacle_angles.erase((obstacle_angles.begin()));
        }
    }

    amp::Obstacle2D c_space_obj(final_c_space_vertices_CCW);

    //obstacle.print();
    //robot.print();
    //c_space_obj.print();
    //Visualizer::makeFigure(polygons);
    const std::vector<amp::Polygon> vector_polygons{obstacle, robot, c_space_obj};
    //vector_polygons.push_back(obstacle);
    //vector_polygons.push_back(robot); 
    Visualizer::makeFigure(vector_polygons);

    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());
    std::cout<<"REACHED HEREEEEEEEEEEEEEEEEEEEe"<<std::endl;
    MyManipulator2D manipulator;
    Eigen::Vector3d state(0.5235,1.0471,5.4977);
    Eigen::Vector2d position = manipulator.getJointLocation(state,3);
    std::cout<<"FINISHED!"<<std::endl;
    std::cout<<"POSITION"<<position[0]<<position[1]<<std::endl;
    // You can visualize your manipulator given an angle state like so:
    //amp::ManipulatorState test_state;
    //test_state.setZero();
    // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    Visualizer::makeFigure(manipulator, state); 

    Visualizer::showFigures();


    // Create the collision space constructor
    std::size_t n_cells = 5;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace1());

    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace);

    Visualizer::showFigures();

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "srba2850@colorado.edu", argc, argv);
    return 0;
}
#include "MyBugAlgorithm.h"
#include <cmath> 

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    amp::Path2D path;
    /*// Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    obs = problem.obstacles;
    //getPrimitives()
    
    path.waypoints.push_back(problem.q_init);
    Eigen::Vector2d state = problem.q_init;
    while(true)
    {
        if(!inCollision(state))
        {
            state(0)+=0.1*cos(std::atan((problem.q_goal(1)-state(1))/(problem.q_goal(0)-state(0))));
            state(1)+=0.1*sin(std::atan((problem.q_goal(1)-state(1))/(problem.q_goal(0)-state(0))));
            path.waypoints.push_back(state);
            if(state(0)-problem.q_goal(0) <0.5 && state(1)-problem.q_goal(1)<0.5) 
            {
                break;
            }
        }
    }
    
    //path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
    //path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    path.waypoints.push_back(problem.q_goal);

    return path;*/
    return path;
}

bool MyBugAlgorithm::inCollision(Eigen::Vector2d position)
{
    /*for(Obstacle2D obstacle : obs)
    {
        std::vector<Eigen::Vector2d> current_vertices = problem.obstacles[0].verticesCCW();
        for(int i = 0; i<current_vertices.size()-1,i++)
        {
            slope = (current_vertices[i+1][1]-current_vertices[i][1])/(current_vertices[i+1][0]-current_vertices[i][0])
            intercept = current_vertices[i+1][1] - slope*current_vertices[i+1][0]
            if(position(1)-((slope)*position(0))-(intercept) < 0)
            {
                return true;
            }
        }
    }*/
    return false;
}
#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    std::vector<amp::Obstacle2D> obs = problem.obstacles;
    path.waypoints.push_back(problem.q_init);
    Eigen::Vector2d current_state = problem.q_init;
    double distance_to_goal = distance(current_state,problem.q_goal);
    Eigen::Vector2d del_U_attractive, del_U_repulsive(0,0);
    
    while(distance_to_goal > 1)
    {
        if (distance_to_goal > d_star)
        {
            double shortest_distance = 100000;
            Eigen::Vector2d nearest_vertex(0,0);
            del_U_attractive = zetta*(current_state-problem.q_goal);
            for(int i=0; i < obs.size();i++)
            {   
                std::vector<Eigen::Vector2d> current_vertices = obs[i].verticesCCW();
                for(int i = 0; i<current_vertices.size();i++)
                {
                    double current_distance = distance(current_state,current_vertices[i]);
                    if(current_distance < shortest_distance)
                    {
                        shortest_distance = current_distance;
                        nearest_vertex = current_vertices[i];
                    }    
                
                }
                
            }
            if(shortest_distance < Q_star)
            {
                std::cout<<"CLOSE TO OBSTACLE!!!"<<nearest_vertex<<std::endl;
                del_U_repulsive = del_U_repulsive + (eta*((1/Q_star)-(1/shortest_distance))*(1/shortest_distance*shortest_distance)*(current_state-nearest_vertex)); 
                std::cout<<"??????????????????"<<(1/Q_star)<<"AND"<<(1/shortest_distance)<<"AND"<<(1/(shortest_distance*shortest_distance))<<"FINALLY"<<del_U_repulsive<<std::endl;
            }
        }
        else
        {
            del_U_attractive = (d_star/distance(current_state,problem.q_goal))*zetta*(current_state-problem.q_goal);
            for(int i=0; i < obs.size();i++)
            {
                double shortest_distance = 100000;
                Eigen::Vector2d nearest_vertex(0,0);
                std::vector<Eigen::Vector2d> current_vertices = obs[i].verticesCCW();
                for(int i = 0; i<current_vertices.size();i++)
                {
                    double current_distance = distance(current_state,current_vertices[i]);
                    if(current_distance < shortest_distance)
                    {
                        shortest_distance = current_distance;
                        nearest_vertex = current_vertices[i];
                    }    
                
                }
                if(shortest_distance < Q_star)
                {
                    std::cout<<"CLOSE TO OBSTACLE!!!"<<std::endl;
                    del_U_repulsive = del_U_repulsive + (eta*((1/Q_star)-(1/shortest_distance))*(1/shortest_distance*shortest_distance)*(current_state-nearest_vertex)); 
                    std::cout<<"??????????????????"<<del_U_repulsive<<std::endl;
                }
            }
        }
        std::cout<<"THE ATTRATIVE AND REPULSIVE ARE:"<< del_U_attractive<< "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"<<del_U_repulsive<<std::endl;
        current_state = current_state-(0.1*(del_U_attractive+1*del_U_repulsive));
        distance_to_goal = distance(current_state,problem.q_goal);
        path.waypoints.push_back(current_state);
    }
    path.waypoints.push_back(problem.q_goal);
    return path;
}

double MyGDAlgorithm::distance(Eigen::Vector2d position1,Eigen::Vector2d position2) {
    double distance = (position1 - position2).norm();
    return distance;
}

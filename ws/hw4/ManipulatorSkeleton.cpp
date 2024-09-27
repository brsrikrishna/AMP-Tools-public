#include "ManipulatorSkeleton.h"
#include <cmath>

MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    
    int no_of_links = state.size();

    std::vector<double> link_lengths{0,0.5,1,0.5};
    std::vector<Eigen::Vector2d> joint_positions;
    std::cout<<"STATE IS"<<state<<std::endl;
    std::vector<Eigen::Matrix3d> tx_matrices;
    Eigen::VectorXd final_states = state;
    final_states.conservativeResize(final_states.size() + 1);
    final_states(final_states.size() - 1) = 0;
    for(int i=0; i<=no_of_links; i++)
    {
        Eigen::Matrix3d current_matrix;
        current_matrix  <<  cos(final_states[i]), -sin(final_states[i]), link_lengths[i],
                            sin(final_states[i]), cos(final_states[i]), 0,
                            0,             0,             1;
        tx_matrices.push_back(current_matrix);
    }
    
    std::vector<Eigen::MatrixXd> final_results;
    Eigen::Vector3d origin(0,0,1);
    for(int i=0; i<tx_matrices.size();i++)
    {
        final_results.push_back(Eigen::Matrix3d::Identity());
    }
    for(int i=0; i<tx_matrices.size();i++)
    {
        int current_index = i;
        final_results[i] = final_results[i]*origin;
        while(current_index>=0)
        {   
            final_results[i] = tx_matrices[current_index]*final_results[i] ;
            current_index-=1;
        }
    }
    
    for(int i=0; i<final_results.size();i++)
    {
        Eigen::Vector2d joint_position = final_results[i].topRows(1).leftCols(2);
        joint_positions.push_back(joint_position);
        
    }
    std::cout<<"THE FINAL POS IS"<<joint_positions[joint_index]<<std::endl;
    return joint_positions[joint_index];
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here

    amp::ManipulatorState joint_angles;
    joint_angles.setZero();
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {

        return joint_angles;
    } else if (nLinks() == 3) {

        return joint_angles;
    } else {

        return joint_angles;
    }

    return joint_angles;
}
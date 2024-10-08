#include "MyAStar.h"

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object
    result.node_path.push_back(problem.init_node);
    std::list<amp::Node> priority_queue;
    priority_queue.push_back(problem.init_node);
    int count = 0;
    while(count < 100)
    {
        count++;
        priority_queue = SortPriorityQueue(priority_queue);
        amp::Node best_node = priority_queue.pop_front();

    }

    result.node_path.push_back(problem.goal_node);
    result.path_cost += 1.0;

    result.print();
    return result;
}

std:list<amp::Node> MyAStarAlgo::SortPriorityQueue(std:list<amp::Node> priority_queue)
{
    
    return priority_queue;
}
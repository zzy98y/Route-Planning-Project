#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // find the closest node given the x and y coordinate 
    start_node = &m_Model.FindClosestNode(start_x,start_y); 
    end_node = &m_Model.FindClosestNode(end_x,end_y);

}


// we can use the distance from the node to end node as the h_value 
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);

}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    
    for (auto neighbor : current_node->neighbors) {
        /* go through each neighbor of the current node and find the attribute of the node 
        and add it to the open_list */
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor);

        open_list.push_back(neighbor); 
        neighbor->visited = true;

    }
    

}

// custom function Compare that will help sort in descending order 
bool Compare(const RouteModel::Node *first, const RouteModel::Node *second) {
    auto first_sum = first->h_value + first->g_value; 
    auto second_sum = second->h_value + second->g_value; 
    return first_sum > second_sum; 
}

RouteModel::Node *RoutePlanner::NextNode() {
    // sort the open_list in descending order and remove/return the last one (lowest_node)
    std::sort(open_list.begin(),open_list.end(),Compare); 
    auto lowest_node = open_list.back();
    open_list.pop_back();
    lowest_node->visited = true;
    return lowest_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr) {
        // add nodes to path, update parent, update distance 
        path_found.push_back(*current_node);
        const RouteModel::Node parent = *(current_node->parent);
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    // reverse the path found 
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    open_list = {};
    open_list.push_back(start_node);

    while(open_list.size() >0) {
        current_node = this->NextNode();
        if(current_node == end_node) {
            break;
        }
        this->AddNeighbors(current_node);
    }
    m_Model.path = ConstructFinalPath(end_node);

}
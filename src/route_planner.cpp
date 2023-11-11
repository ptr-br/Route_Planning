#include "route_planner.h"
#include <algorithm>

#include <iostream> // remove later

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h_value = node->distance(*this->end_node);
    return h_value;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors){
        if (neighbor->visited != true){
        // calc & set h-Value
        neighbor->h_value = this->CalculateHValue(neighbor);
        // calc & set g-Value
        // g-value is the g-value of the prev node + the dist to the prev node
        neighbor->g_value = (current_node->g_value) + neighbor->distance(*current_node);
        // set visited -> True
        neighbor->visited = true;
        this->open_list.push_back(neighbor);
        // set parent
        neighbor->parent = current_node;
        } 
    }
}


bool RoutePlanner::CellSort( RouteModel::Node *n1,  RouteModel::Node *n2) {
  auto f1 = n1->h_value + n1->g_value;
  auto f2 = n2->h_value + n2->g_value;
  return f1 < f2;
}

RouteModel::Node *RoutePlanner::NextNode() {

    std::sort(open_list.begin(), open_list.end(), [this]( RouteModel::Node *n1,  RouteModel::Node *n2) {
        return this->CellSort(n1, n2);
    });

    RouteModel::Node *ptr = this->open_list.front();
    this->open_list.erase(this->open_list.begin());
    return ptr;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
  
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // while parent node exits keep on looping 
    while (current_node != start_node){
        
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        // set current_node to parent to travel back to the start node
        current_node = current_node->parent;
    }
    // add starting node as well
    path_found.push_back(*current_node);

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // begin with start node
    current_node = this->start_node;
    current_node->visited = true;
    this->open_list.emplace_back(start_node);

    // loop over open_list until final node is found
    while (current_node != this->end_node){
        // add all the neighbors
        this->AddNeighbors(current_node);
        current_node = this->NextNode();
    }
    
    m_Model.path = this->ConstructFinalPath(current_node);

}
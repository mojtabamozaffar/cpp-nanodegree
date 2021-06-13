#include "route_planner.h"
#include <algorithm>
#include <iostream>

using std::cout;

bool Compare(RouteModel::Node const *node_1, RouteModel::Node const *node_2) {
  float f1 = node_1->g_value + node_1->h_value;
  float f2 = node_2->g_value + node_2->h_value;
  return f1 > f2; 
}

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node =  &(m_Model.FindClosestNode(start_x, start_y));
    this->end_node = &(m_Model.FindClosestNode(end_x, end_y));

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*(this->end_node));
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node* neighbor : current_node->neighbors){
        if (neighbor->visited == false){
            neighbor->parent = current_node;
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
            neighbor->h_value = this->CalculateHValue(neighbor);
            neighbor->visited = true;
            open_list.push_back(neighbor);
        }
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(), Compare);
    RouteModel::Node *next = this->open_list.back();
    this->open_list.pop_back();
    return next;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.push_back(*(current_node));

    while(current_node->parent != nullptr){
        distance += current_node->distance(*(current_node->parent));
        path_found.push_back(*(current_node->parent));
        current_node = current_node->parent;
    }
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;
    this->start_node->visited = true;
    open_list.push_back(current_node);

    while (this->open_list.size() > 0){
        auto current_node = NextNode();
        if (current_node == this->end_node){
            auto path = ConstructFinalPath(current_node);
            this->m_Model.path = path;
            return;
        }
        AddNeighbors(current_node);
    }
}

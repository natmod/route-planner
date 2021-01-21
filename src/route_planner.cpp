#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();
    for (RouteModel::Node *node : current_node->neighbors)
    {
        node->parent = current_node;
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->h_value = this->CalculateHValue(node);
        this->open_list.push_back(node);
        node->visited = true;
    }
}

bool Compare(const RouteModel::Node *a, const RouteModel::Node *b)
{
    float a_sum = a->g_value + a->h_value;
    float b_sum = b->g_value + b->h_value;
    return a_sum > b_sum;
}

RouteModel::Node *RoutePlanner::NextNode()
{
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node *lowest = open_list.back();
    open_list.pop_back();
    return lowest;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node != start_node)
    {
        RouteModel::Node *parent_node = current_node->parent;
        distance += current_node->distance(*parent_node);
        path_found.push_back(*current_node);
        current_node = parent_node;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = nullptr;

    this->open_list.push_back(this->start_node);
    this->start_node->visited = true;

    while (open_list.size() > 0)
    {
        current_node = this->NextNode();

        if (current_node->distance(*end_node) == 0)
        {
            m_Model.path = this->ConstructFinalPath(current_node);
            break;
        }

        this->AddNeighbors(current_node);
    }
}
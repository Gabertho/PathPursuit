

#ifndef ASTAR_H
#define ASTAR_H

#include "occupancy_map.h"

#include <vector>
#include <queue>
#include <memory>
#include <limits>
#include <cmath>
#include <tf2/LinearMath/Vector3.h>
#include <boost/numeric/ublas/matrix.hpp>

namespace astar {

#define NULL_STATE State(-1, -1)
#define DISTANCE_THRESHOLD 15  // Adjust as needed
#define PROXIMITY_THRESHOLD 1  // Adjust as needed
#define COST_FACTOR 100 // Adjust as needed

using namespace boost::numeric::ublas;

typedef std::shared_ptr<occupancy_map::OccupancyMap> MapPtr;

enum Status{kNotInOpenList, kInOpenList};

struct State{
    int row = -1;
    int col = -1;
    State(int row_, int col_) : row(row_), col(col_) {}
};

State operator+(const State &lhs, const State &rhs){
    return {lhs.row + rhs.row, lhs.col + rhs.col};
}

State operator-(const State &lhs, const State &rhs){
    return {lhs.row - rhs.row, lhs.col - rhs.col};
}

bool operator==(const State &lhs, const State &rhs){
    return (lhs.row == rhs.row) && (lhs.col == rhs.col);
}

bool operator!=(const State &lhs, const State &rhs){
    return !(lhs == rhs);
}

bool operator<(const State &lhs, const State &rhs){
    return (lhs.row != rhs.row) ? (lhs.row < rhs.row) : (lhs.col < rhs.col);
}

bool operator>(const State &lhs, const State &rhs){
    return !(lhs < rhs);
}


struct Edge{
    State action = {0, 0};
    float weight = 0.0f;
};


struct Node{
    State come_from = NULL_STATE;
    Status status = kNotInOpenList;
    float g_cost = std::numeric_limits<float>::infinity();
    float f_cost = std::numeric_limits<float>::infinity();
};


class Astar{
public:
    Astar(){};
    Astar(MapPtr &map_ptr);
    void InitializeGraph();
    void Plan(const tf2::Vector3 &source_point_w, const tf2::Vector3 &target_point_w, std::vector<tf2::Vector3> &path_w);

private:
    matrix<Node> graph_;
    MapPtr map_ptr_ = nullptr;
    std::vector<Edge> kEdges = {
        { {1,  0},  1},
        { {1,  1},  std::sqrt(2)},
        { {0,  1},  1},
        { {-1, 1},  std::sqrt(2)},
        { {-1, 0},  1},
        { {-1,-1},  std::sqrt(2)},
        { {0, -1},  1},
        { {1, -1},  std::sqrt(2)},
    };
    State WorldPointToState_(const tf2::Vector3 &point_w);
    bool IsStateValid_(const State &state);
    float Heuristic_(const State &lhs, const State &rhs);
    void ReconstructPath_(const matrix<Node> &graph, const State &target_state, std::vector<tf2::Vector3> &path_w);
    bool IsTooCloseToWall(const State &state, float threshold);
    float CalculateWallProximityCost(const State &state);
};

Astar::Astar(MapPtr &map_ptr){
    map_ptr_ = map_ptr;
};

void Astar::InitializeGraph(){
    // If map size has not changed, then clear; else reconstruct
    if(graph_.size1() == map_ptr_->GetNumRows() && graph_.size2() == map_ptr_->GetNumCols() ) {
        graph_.clear();
    }
    else{
        graph_ = matrix<Node>(map_ptr_->GetNumRows(), map_ptr_->GetNumCols());
    }
}

void Astar::Plan(const tf2::Vector3 &source_point_w, const tf2::Vector3 &target_point_w, std::vector<tf2::Vector3> &path_w){
    bool found = false;

    // Clear the storage
    path_w.clear();

    // Get states
    State source_state = WorldPointToState_(source_point_w);
    State target_state = WorldPointToState_(target_point_w);

    // Check query's vality
    if(IsStateValid_(source_state) == false || IsStateValid_(target_state)  == false){
        // ROS_INFO("There is no valid solution for given query.");
        return;
    }

    // Initilize data structure
    InitializeGraph();

    // Compare function
    auto compare = [&](const State &lhs, const State &rhs){
        return graph_(lhs.row, lhs.col).f_cost > graph_(rhs.row, rhs.col).f_cost; 
    };

    // Priority queue
    std::priority_queue<State, std::vector<State>, decltype(compare)> open_list(compare);

    // Calculate cost values for source state
    graph_(source_state.row, source_state.col).g_cost = 0;
    graph_(source_state.row, source_state.col).f_cost = Heuristic_(source_state, target_state);

    // Push source state to open list
    open_list.push(source_state);

    // Update the source state`s flag
    graph_(source_state.row, source_state.col).status = kInOpenList;

    while(!open_list.empty()){
        // Get top state
        auto curr_state = open_list.top();
        
        // Pop the top state
        open_list.pop();

        // Update the flag
        graph_(curr_state.row, curr_state.col).status = kNotInOpenList;

        // Check whether target Node is found
        if(curr_state == target_state){
            found = true;
            break;
        }

        // Iterate over neighbors
        for(auto &edge : kEdges){
            State neighbor_state  = curr_state + edge.action;

            // If new state is not valid, pass
            if(!IsStateValid_(neighbor_state)){ continue; }

            // Possible g score
            float tentative_g_score = graph_(curr_state.row, curr_state.col).g_cost + edge.weight;

            // Add cost for proximity to walls
            tentative_g_score += CalculateWallProximityCost(neighbor_state);

            // Check whether node should be updated
            if(tentative_g_score < graph_(neighbor_state.row, neighbor_state.col).g_cost){
                // Update cost values and come from
                graph_(neighbor_state.row, neighbor_state.col).come_from = curr_state;
                graph_(neighbor_state.row, neighbor_state.col).g_cost = tentative_g_score;
                graph_(neighbor_state.row, neighbor_state.col).f_cost = graph_(neighbor_state.row, neighbor_state.col).g_cost 
                    + Heuristic_(neighbor_state, target_state);

                // Push neighbor to open list if it is not in yet
                if(graph_(neighbor_state.row, neighbor_state.col).status == kNotInOpenList){
                    open_list.push(neighbor_state);
                    graph_(neighbor_state.row, neighbor_state.col).status == kInOpenList;
                }
            }
        }
    }

    // Reconstruct path
    if(found == true){
        ReconstructPath_(graph_, target_state, path_w);
    }
}

inline State Astar::WorldPointToState_(const tf2::Vector3 &point_w){
    auto indices = map_ptr_->WorldToMapIndices(point_w);
    return {indices.first, indices.second};
}

inline bool Astar::IsStateValid_(const State &state){
    if (!map_ptr_->IsMapIndicesValid(state.row, state.col)) {
        return false;
    }
    if (IsTooCloseToWall(state, DISTANCE_THRESHOLD)) {
        return false;
    }
    return true;
}

inline float Astar::Heuristic_(const State &lhs, const State &rhs){
    // Manhattan
    return std::abs(lhs.row - rhs.row) + std::abs(lhs.col - rhs.col);
}

void Astar::ReconstructPath_(const matrix<Node> &graph, const State &target_state, std::vector<tf2::Vector3> &path_w){
    auto curr_state = target_state;

    // Back track from target state
    while(curr_state != NULL_STATE){
        // Transform state to point in world frame
        tf2::Vector3 point_w = map_ptr_->MapIndicesToWorld(curr_state.row, curr_state.col);

        // Push current point to path
        path_w.push_back(point_w);

        // Update current state
        curr_state = graph(curr_state.row, curr_state.col).come_from;
    }
    std::reverse(path_w.begin(), path_w.end());
}

bool Astar::IsTooCloseToWall(const State &state, float threshold) {
    // Verifique em volta do estado atual para detectar paredes próximas
    for (int i = -threshold; i <= threshold; ++i) {
        for (int j = -threshold; j <= threshold; ++j) {
            if (map_ptr_->IsWall(state.row + i, state.col + j)) {
                return true;
            }
        }
    }
    return false;
}

float Astar::CalculateWallProximityCost(const State &state) {
    // Custo baseado na proximidade da parede
    if (IsTooCloseToWall(state, PROXIMITY_THRESHOLD)) {
        return COST_FACTOR;
    }
    return 0;
}

}  // namespace astar

#endif  // ASTAR_H

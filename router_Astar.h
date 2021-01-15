#ifndef ROUTER_ASTAR
#define ROUTER_ASTAR

#include <iostream>
#include <stack>
#include <queue>
#include <vector>
#include <map>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include "gridmap.h"
#include "input.h"
#include "block_pin.h"
using namespace std; 
struct Grid_node {
    pair<int, int> grid_index;
    int Layer;
    int manhattan;
    bool via_use=false;
};
struct Map_grid {
    pair<int, int> grid_index;
    int Layer;
    bool operator < (const Map_grid m) const {
        if (Layer != m.Layer) return Layer < m.Layer;
        if (grid_index.first != m.grid_index.first) return grid_index.first < m.grid_index.first;
        return grid_index.second < m.grid_index.second;
    }
};

struct cmp
{
    bool operator() (const Grid_node& lhs, const Grid_node& rhs) {return lhs.manhattan  > rhs.manhattan;}
};

struct via_type{
    int via_number;
    int priority;
};

struct compare
{
    bool operator() (const via_type &lhs , const via_type &rhs){
        return lhs.priority > rhs.priority;
    }
};
struct triangle_turn {
    int idx; // from 1
    bool tri_type; // 0 = up
    bool last_dir; // 0 = pos
};
struct route {
    RECT route_shape;
    int Layer;
};
struct PINBITS_route {
    string name;
    vector<route> routes;
};
struct BUSES_route{
    string name;
    uint32_t half_parameter;
    vector<uint32_t> LowerWidth;
    vector<PINBITS_route> bit_routes;
    vector<vector<Grid_node> > via;
};

struct All_route {
    vector<BUSES_route> path;
    vector<vector<bool> > bus_result;
    vector<vector<int*> > types;
};
struct Turn_queue {
    bool up_down;
    priority_queue<Grid_node, vector<Grid_node>, cmp > Turn_point;
    vector<int*> used_grid_node;
    vector<vector<Grid_node> > pre_turnpoints;
    map<Map_grid, bool> path_check;
    int line_cost;
    int cost;
    int spv_cnt;
    pair<int, int> guide_coord;
    bool last_dir;
    vector<triangle_turn> triangle_turns;
};
struct Seg_tmp{
    vector<pair<int, int> > seg_end;
    vector<pair<int, int> > seg_start;
};
struct cmp_Astar
{
    bool operator() (const Turn_queue& lhs, const Turn_queue& rhs) {
        if (lhs.cost != rhs.cost)
            return lhs.cost  > rhs.cost;
        return lhs.line_cost < rhs.line_cost;
    }
};
struct route_workspace {
    priority_queue<Turn_queue, vector<Turn_queue>, cmp_Astar > Astar_queue;
};
bool operator < (Grid_node a, Grid_node b);
void print_coordinate (Grid_node node, GridMap &G, MainInput &M);
pair<int, int> get_coordinate (Grid_node node, GridMap &G, MainInput &M);
bool change_layer (Grid_node origin, Grid_node& target, int t_layer, GridMap &G, MainInput &M);
int cal_manhattan (Grid_node a, Grid_node b, GridMap &G, MainInput &M);
int cal_pair_manhattan (pair<int, int> a, pair<int, int> b);
int cal_real_manhattan (Grid_node a, int b , int c);
bool one_bus_router (All_route &all, GridMap &G, BUSES &B, MainInput &M, BUSES_block &B_block, Preserve_space &P, int bus_id );
bool bus_router_func (All_route &all, GridMap &G, MainInput &M);
bool n_one_bus_router (All_route &all, GridMap &G, BUSES &B, MainInput &M, BUSES_block &B_block, int bus_id , int bit_id , int bit_id2, BUSES_route &BR);
bool n_bus_router_func (All_route &all, GridMap &G, MainInput &M);
bool f_one_bus_router (All_route &all, GridMap &G, BUSES &B, MainInput &M, BUSES_block &B_block, int bus_id);
bool f_bus_router_func (All_route &all, GridMap &G, MainInput &M);
#endif

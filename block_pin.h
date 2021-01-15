#ifndef BLOCK_PIN
#define BLOCK_PIN

#include <iostream>
#include <stack>
#include <queue>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include "gridmap.h"
#include "input.h"
using namespace std;
struct Bus_grid {
    int Layer;
    pair<int, int> grid_index;
    bool operator < (const Bus_grid a) const {
        return grid_index.first < a.grid_index.first;
    }
    pair<int, int> coord (GridMap &G, MainInput &M) {
        pair<int, int> coord;
        if (M.LayerSet[Layer].Orientation == 1) {
            coord.first = G.GridMapSet[Layer].track[grid_index.first].coord;
            coord.second = G.GridMapSet[Layer].track[grid_index.first].gridpoint[grid_index.second].coord;
        }
        else {
            coord.first =  G.GridMapSet[Layer].track[grid_index.first].gridpoint[grid_index.second].coord;
            coord.second = G.GridMapSet[Layer].track[grid_index.first].coord;
        }
        return coord;
    }
};
int busgrids_manhattan (Bus_grid a, Bus_grid b, GridMap &G, MainInput &M);
struct BUSES_preserve {
    vector<vector<int*> > pin_preserve;
};
struct Preserve_space {
    vector<BUSES_preserve> bus_preserve;
};
struct BITS_block {
    vector<vector<int*> > block_grids;
};
struct BUSES_block {
    vector<BITS_block> bits_block;
};
// block pins
void init_pin_blockage (BUSES_block &B_block, GridMap &G, MainInput &M);
void block_pin_deactivate (BUSES_block &B_block, int bus_id, int net_id);
void block_pin_activate (BUSES_block &B_block, int bus_id, int net_id);
void BUSES_block_deactivate (BUSES_block &B_block, int bus_id);
// block preserve space
void init_preserve (Preserve_space &P, GridMap &G, MainInput &M);
void preserve_deactivate (Preserve_space &P, int bus_id, int net_id);
void preserve_activate (Preserve_space &P, int bus_id, int net_id);
void BUSES_preserve_deactivate (Preserve_space &P, int bus_id);
#endif

#include "block_pin.h"
// block pins
int busgrids_manhattan (Bus_grid a, Bus_grid b, GridMap &G, MainInput &M) {
    int sum = 0;
    sum += abs(a.coord(G, M).first-b.coord(G, M).first);
    sum += abs(a.coord(G, M).second-b.coord(G, M).second);
    return sum;
}
void block_pin_activate (BUSES_block &B_block, int bus_id, int net_id) {
    for (int i = 0; i < B_block.bits_block[bus_id].block_grids[net_id].size(); i++) {
        *(B_block.bits_block[bus_id].block_grids[net_id][i]) = 3;
    }
}
void block_pin_deactivate (BUSES_block &B_block, int bus_id, int net_id) {
    for (int i = 0; i < B_block.bits_block[bus_id].block_grids[net_id].size(); i++) {
        *(B_block.bits_block[bus_id].block_grids[net_id][i]) = 0;
    }
}
void BUSES_block_deactivate (BUSES_block &B_block, int bus_id) {
    for (int i = 0; i < B_block.bits_block[bus_id].block_grids.size(); i++)
        block_pin_deactivate(B_block, bus_id, i);
}
void init_pin_blockage (BUSES_block &B_block, GridMap &G, MainInput &M) {
    B_block.bits_block.resize(M.BusSet.size());
    for (int i = 0; i < M.BusSet.size(); i++)
        B_block.bits_block[i].block_grids.resize(M.BusSet[i].PinBitSet[0].PinShapeSet.size());
    for (int i = 0; i < M.BusSet.size(); i++) {
        for (int j = 0; j < M.BusSet[i].PinBitSet[0].PinShapeSet.size(); j++) {
            int current_layer = M.BusSet[i].PinBitSet[0].PinShapeSet[j].LayerNum;
            bool Layer_violation = 0;
            if (M.LayerSet[current_layer].Orientation == 1) {
                if (M.BusSet[i].PinBitSet[0].PinShapeSet[j].Pin.LB.x == M.BusSet[i].PinBitSet[1].PinShapeSet[j].Pin.LB.x)
                    Layer_violation = 1;
            }
            else {
                if (M.BusSet[i].PinBitSet[0].PinShapeSet[j].Pin.LB.y == M.BusSet[i].PinBitSet[1].PinShapeSet[j].Pin.LB.y)
                    Layer_violation = 1;
            }
            if (Layer_violation == 1) {
                int next_layer = current_layer;
                for (int i = 1; i < M.LayerSet.size(); i++)  // go lower layer
                    if (current_layer-i >= 0 && M.LayerSet[current_layer-i].Orientation != M.LayerSet[current_layer].Orientation) {
                        next_layer = current_layer - i;
                        break;
                    }
                if (next_layer == current_layer) {
                    for (int i = 1; i < M.LayerSet.size(); i++)  // go upper layer
                        if (current_layer+i < M.LayerSet.size() && M.LayerSet[current_layer+i].Orientation != M.LayerSet[current_layer].Orientation) {
                            next_layer = current_layer + i;
                            break;
                        }
                }
                current_layer = next_layer;
            }
            BUSES B = M.BusSet[i];
            if (current_layer-1 >= 0) {
                int change_layer = current_layer-1;
                for (int k = 0; k < B.PinBitSet.size(); k++) {
                    PINSHAPES pintmp = B.PinBitSet[k].PinShapeSet[j];
                    int orientation = M.LayerSet[change_layer].Orientation;
                    for (int l = 0; l < G.GridMapSet[change_layer].track.size(); l++) {
                        if (orientation == 1) {
                            if (G.GridMapSet[change_layer].track[l].coord >= pintmp.Pin.LB.x && G.GridMapSet[change_layer].track[l].coord <= pintmp.Pin.RT.x) {
                                for (int ii = 0; ii < G.GridMapSet[change_layer].track[l].gridpoint.size(); ii++) {
                                    if (G.GridMapSet[change_layer].track[l].gridpoint[ii].coord >= pintmp.Pin.LB.y && G.GridMapSet[change_layer].track[l].gridpoint[ii].coord <= pintmp.Pin.RT.y) {
                                        if (G.GridMapSet[change_layer].track[l].gridpoint[ii].type == 0) {
                                        B_block.bits_block[i].block_grids[j].push_back(&(G.GridMapSet[change_layer].track[l].gridpoint[ii].type));
                                        }
                                    }
                                }
                            }
                        }
                        else {
                            if (G.GridMapSet[change_layer].track[l].coord >= pintmp.Pin.LB.y && G.GridMapSet[change_layer].track[l].coord <= pintmp.Pin.RT.y) {
                                for (int ii = 0; ii < G.GridMapSet[change_layer].track[l].gridpoint.size(); ii++) {
                                    if (G.GridMapSet[change_layer].track[l].gridpoint[ii].coord >= pintmp.Pin.LB.x && G.GridMapSet[change_layer].track[l].gridpoint[ii].coord <= pintmp.Pin.RT.x) {
                                       if (G.GridMapSet[change_layer].track[l].gridpoint[ii].type == 0) {
                                        B_block.bits_block[i].block_grids[j].push_back(&(G.GridMapSet[change_layer].track[l].gridpoint[ii].type));
                                       }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (current_layer+1 < M.LayerSet.size()) {
                int change_layer = current_layer+1;
                for (int k = 0; k < B.PinBitSet.size(); k++) {
                    PINSHAPES pintmp = B.PinBitSet[k].PinShapeSet[j];
                    int orientation = M.LayerSet[change_layer].Orientation;
                    for (int l = 0; l < G.GridMapSet[change_layer].track.size(); l++) {
                        if (orientation == 1) {
                            if (G.GridMapSet[change_layer].track[l].coord >= pintmp.Pin.LB.x && G.GridMapSet[change_layer].track[l].coord <= pintmp.Pin.RT.x) {
                                for (int ii = 0; ii < G.GridMapSet[change_layer].track[l].gridpoint.size(); ii++) {
                                    if (G.GridMapSet[change_layer].track[l].gridpoint[ii].coord >= pintmp.Pin.LB.y && G.GridMapSet[change_layer].track[l].gridpoint[ii].coord <= pintmp.Pin.RT.y) {
                                       if (G.GridMapSet[change_layer].track[l].gridpoint[ii].type == 0) {
                                        B_block.bits_block[i].block_grids[j].push_back(&(G.GridMapSet[change_layer].track[l].gridpoint[ii].type));
                                       }
                                    }
                                }
                            }
                        }
                        else {
                            if (G.GridMapSet[change_layer].track[l].coord >= pintmp.Pin.LB.y && G.GridMapSet[change_layer].track[l].coord <= pintmp.Pin.RT.y) {
                                for (int ii = 0; ii < G.GridMapSet[change_layer].track[l].gridpoint.size(); ii++) {
                                    if (G.GridMapSet[change_layer].track[l].gridpoint[ii].coord >= pintmp.Pin.LB.x && G.GridMapSet[change_layer].track[l].gridpoint[ii].coord <= pintmp.Pin.RT.x) {
                                       if (G.GridMapSet[change_layer].track[l].gridpoint[ii].type == 0) {
                                        B_block.bits_block[i].block_grids[j].push_back(&(G.GridMapSet[change_layer].track[l].gridpoint[ii].type));
                                       }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
// block preserve space
void init_preserve (Preserve_space &P, GridMap &G, MainInput &M) {
    P.bus_preserve.resize(M.BusSet.size());
    for (int i = 0; i < M.BusSet.size(); i++)
        P.bus_preserve[i].pin_preserve.resize(M.BusSet[i].PinBitSet[0].PinShapeSet.size());
    for (int i = 0; i < M.BusSet.size(); i++) {
        for (int j = 0; j < M.BusSet[i].PinBitSet[0].PinShapeSet.size(); j++) {
            int current_layer = M.BusSet[i].PinBitSet[0].PinShapeSet[j].LayerNum;
            bool Layer_violation = 0;
            if (M.LayerSet[current_layer].Orientation == 1) {
                if (M.BusSet[i].PinBitSet[0].PinShapeSet[j].Pin.LB.x == M.BusSet[i].PinBitSet[1].PinShapeSet[j].Pin.LB.x)
                    Layer_violation = 1;
            }
            else {
                if (M.BusSet[i].PinBitSet[0].PinShapeSet[j].Pin.LB.y == M.BusSet[i].PinBitSet[1].PinShapeSet[j].Pin.LB.y)
                    Layer_violation = 1;
            }
            if (Layer_violation == 1) {
                int next_layer = current_layer;
                for (int i = 1; i < M.LayerSet.size(); i++)  // go lower layer
                    if (current_layer-i >= 0 && M.LayerSet[current_layer-i].Orientation != M.LayerSet[current_layer].Orientation) {
                        next_layer = current_layer - i;
                        break;
                    }
                if (next_layer == current_layer) {
                    for (int i = 1; i < M.LayerSet.size(); i++)  // go upper layer
                        if (current_layer+i < M.LayerSet.size() && M.LayerSet[current_layer+i].Orientation != M.LayerSet[current_layer].Orientation) {
                            next_layer = current_layer + i;
                            break;
                        }
                }
                current_layer = next_layer;
            }
            BUSES B = M.BusSet[i];
            int spacing = 0;
            if (current_layer+1 < M.LayerSet.size()) {
                int change_layer = current_layer+1;
                if (M.LayerSet[change_layer].Orientation != M.LayerSet[current_layer].Orientation) {
                    int tmp_spacing = (B.Width[change_layer]/2)+M.LayerSet[change_layer].Width;
                    if (tmp_spacing > spacing) spacing = tmp_spacing;
                }
            }
            if (current_layer-1 >= 0) {
                int change_layer = current_layer-1;
                if (M.LayerSet[change_layer].Orientation != M.LayerSet[current_layer].Orientation) {
                    int tmp_spacing = (B.Width[change_layer]/2)+M.LayerSet[change_layer].Width;
                    if (tmp_spacing > spacing) spacing = tmp_spacing;
                }
            }
            //cout << "BUS" << i << "-" << j << " Spacing : " << spacing << endl;
            vector<Bus_grid> GN_origin(B.PinBitSet.size());
            // find pins location
            for (int j0 = 0; j0 < B.PinBitSet.size(); j0++) {
                PINSHAPES pintmp = B.PinBitSet[j0].PinShapeSet[j];
                bool find_pin = 0;
                int track_tmp, gridpoint_tmp;
                for (int j1 = 0; j1 < G.GridMapSet[current_layer].track.size(); j1++) {
                    if (M.LayerSet[current_layer].Orientation == 1) {
                        if (G.GridMapSet[current_layer].track[j1].coord >= pintmp.Pin.LB.x && G.GridMapSet[current_layer].track[j1].coord <= pintmp.Pin.RT.x) {
                            track_tmp = j1;
                            for (int j2 = 0; j2 < G.GridMapSet[current_layer].track[j1].gridpoint.size(); j2++) {
                                if (G.GridMapSet[current_layer].track[j1].gridpoint[j2].coord >= pintmp.Pin.LB.y && G.GridMapSet[current_layer].track[j1].gridpoint[j2].coord <= pintmp.Pin.RT.y) {
                                    gridpoint_tmp = j2;
                                    GN_origin[j0].Layer = current_layer;
                                    GN_origin[j0].grid_index.first = track_tmp;
                                    GN_origin[j0].grid_index.second = gridpoint_tmp;
                                    find_pin = 1;
                                    break;
                                }
                            }
                        }
                        if (find_pin) break;
                    }
                    else {
                        if (G.GridMapSet[current_layer].track[j1].coord >= pintmp.Pin.LB.y && G.GridMapSet[current_layer].track[j1].coord <= pintmp.Pin.RT.y) {
                            track_tmp = j1;
                            for (int j2 = 0; j2 < G.GridMapSet[current_layer].track[j1].gridpoint.size(); j2++) {
                                if (G.GridMapSet[current_layer].track[j1].gridpoint[j2].coord >= pintmp.Pin.LB.x && G.GridMapSet[current_layer].track[j1].gridpoint[j2].coord <= pintmp.Pin.RT.x) {
                                    gridpoint_tmp = j2;
                                    GN_origin[j0].Layer = current_layer;
                                    GN_origin[j0].grid_index.first = track_tmp;
                                    GN_origin[j0].grid_index.second = gridpoint_tmp;
                                    find_pin = 1;
                                    break;
                                }
                            }
                        }
                        if (find_pin) break;
                    }
                }
            }
            sort(GN_origin.begin(), GN_origin.end());
            // go_up
            PINSHAPES pintmp = B.PinBitSet[0].PinShapeSet[j];
            int find_cnt = 0;
            int last_point;
            if (M.LayerSet[current_layer].Orientation == 1) last_point = pintmp.Pin.RT.y;
            else last_point = pintmp.Pin.RT.x;
            vector<Bus_grid> GN = GN_origin;
            while (find_cnt <= B.PinBitSet.size() && GN[0].grid_index.second < G.GridMapSet[current_layer].track[GN[0].grid_index.first].gridpoint.size()) {
                // check obstacle
                bool check_obs = 0;
                for (int k = 0; k < GN.size(); k++) {
                    if (G.GridMapSet[current_layer].track[GN[k].grid_index.first].gridpoint[GN[k].grid_index.second].type != 0) {
                        check_obs = 1;
                        break;
                    }
                }
                if (check_obs == 1) break;
                if (M.LayerSet[current_layer].Orientation == 1) {
                    if (GN[0].coord(G, M).second <= pintmp.Pin.RT.y) {
                        for (int k = 0; k < GN.size(); k++) GN[k].grid_index.second++;
                        continue;
                    }
                }
                else {
                    if (GN[0].coord(G, M).first <= pintmp.Pin.RT.x) {
                        for (int k = 0; k < GN.size(); k++) GN[k].grid_index.second++;
                        /*cout << "here\n";
                        cout << "GN[0] = ( " << GN[0].coord(G, M).first << ", " << GN[0].coord(G, M).second << " )\n";*/
                        continue;
                    }
                }
                // BLOCK ALL NODE WITH SPACING
                // CHECK THE DISTANCE
                bool check_dis = 0;
                if (M.LayerSet[current_layer].Orientation == 1) {
                    if (GN[0].coord(G, M).second - last_point < spacing) check_dis = 1;
                }
                else {
                    if (GN[0].coord(G, M).first - last_point < spacing) check_dis = 1;
                }
                if (check_dis) {
                    for (int k = 0; k < GN.size()-1; k++) {
                        for (int kk = GN[k].grid_index.first; kk >= 0; kk--) {
                            if (G.GridMapSet[current_layer].track[GN[k].grid_index.first].coord-G.GridMapSet[current_layer].track[kk].coord < (B.Width[current_layer]/2)+M.LayerSet[current_layer].Width) {
                                if (G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type == 0) {
                                    G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type = 3;
                                    P.bus_preserve[i].pin_preserve[j].push_back(&(G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                        for (int kk = GN[k].grid_index.first; kk < G.GridMapSet[current_layer].track.size(); kk++) {
                            if (G.GridMapSet[current_layer].track[kk].coord-G.GridMapSet[current_layer].track[GN[k].grid_index.first].coord < (B.Width[current_layer]/2)+M.LayerSet[current_layer].Width) {
                                if (G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type == 0) {
                                    G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type = 3;
                                    P.bus_preserve[i].pin_preserve[j].push_back(&(G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                    }
                    for (int k = 0; k < GN.size(); k++) GN[k].grid_index.second++;
                    continue;
                }
                // change last_point, accumulate find_cnt;
                find_cnt++;
                if (find_cnt == B.PinBitSet.size()+1) break;
                if (M.LayerSet[current_layer].Orientation == 1) last_point = GN[0].coord(G, M).second;
                else last_point = GN[0].coord(G, M).first;
                // GO NEXT ITERATION
                for (int k = GN[0].grid_index.first; k <= GN[GN.size()-1].grid_index.first; k++) {
                    if (G.GridMapSet[current_layer].track[k].gridpoint[GN[0].grid_index.second].type == 0) {
                        G.GridMapSet[current_layer].track[k].gridpoint[GN[0].grid_index.second].type = 3;
                        P.bus_preserve[i].pin_preserve[j].push_back(&(G.GridMapSet[current_layer].track[k].gridpoint[GN[0].grid_index.second].type));
                    }
                }
                for (int k = 0; k < GN.size(); k++) GN[k].grid_index.second++;
            }
            // go_down
            find_cnt = 0;
            if (M.LayerSet[current_layer].Orientation == 1) last_point = pintmp.Pin.LB.y;
            else last_point = pintmp.Pin.LB.x;
            GN = GN_origin;
            while (find_cnt <= B.PinBitSet.size() && GN[0].grid_index.second >= 0) {
                // check obstacle
                bool check_obs = 0;
                for (int k = 0; k < GN.size(); k++) {
                    if (G.GridMapSet[current_layer].track[GN[k].grid_index.first].gridpoint[GN[k].grid_index.second].type != 0) {
                        check_obs = 1;
                        break;
                    }
                }
                if (check_obs == 1) break;
                if (M.LayerSet[current_layer].Orientation == 1) {
                    if (GN[0].coord(G, M).second >= pintmp.Pin.LB.y) {
                        for (int k = 0; k < GN.size(); k++) GN[k].grid_index.second--;
                        continue;
                    }
                }
                else {
                    if (GN[0].coord(G, M).first >= pintmp.Pin.LB.x) {
                        for (int k = 0; k < GN.size(); k++) GN[k].grid_index.second--;
                        /*cout << "here\n";
                         cout << "GN[0] = ( " << GN[0].coord(G, M).first << ", " << GN[0].coord(G, M).second << " )\n";*/
                        continue;
                    }
                }
                // BLOCK ALL NODE WITH SPACING
                // CHECK THE DISTANCE
                bool check_dis = 0;
                if (M.LayerSet[current_layer].Orientation == 1) {
                    if (last_point - GN[0].coord(G, M).second < spacing) check_dis = 1;
                }
                else {
                    if (last_point - GN[0].coord(G, M).first < spacing) check_dis = 1;
                }
                if (check_dis) {
                    for (int k = 0; k < GN.size()-1; k++) {
                        for (int kk = GN[k].grid_index.first; kk >= 0; kk--) {
                            if (G.GridMapSet[current_layer].track[GN[k].grid_index.first].coord-G.GridMapSet[current_layer].track[kk].coord < (B.Width[current_layer]/2)+M.LayerSet[current_layer].Width) {
                                if (G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type == 0) {
                                    G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type = 3;
                                    P.bus_preserve[i].pin_preserve[j].push_back(&(G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                        for (int kk = GN[k].grid_index.first; kk < G.GridMapSet[current_layer].track.size(); kk++) {
                            if (G.GridMapSet[current_layer].track[kk].coord-G.GridMapSet[current_layer].track[GN[k].grid_index.first].coord < (B.Width[current_layer]/2)+M.LayerSet[current_layer].Width) {
                                if (G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type == 0) {
                                    G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type = 3;
                                    P.bus_preserve[i].pin_preserve[j].push_back(&(G.GridMapSet[current_layer].track[kk].gridpoint[GN[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                    }
                    for (int k = 0; k < GN.size(); k++) GN[k].grid_index.second--;
                    continue;
                }
                // change last_point, accumulate find_cnt;
                find_cnt++;
                if (find_cnt == B.PinBitSet.size()+1) break;
                if (M.LayerSet[current_layer].Orientation == 1) last_point = GN[0].coord(G, M).second;
                else last_point = GN[0].coord(G, M).first;
                // GO NEXT ITERATION
                for (int k = GN[0].grid_index.first; k <= GN[GN.size()-1].grid_index.first; k++) {
                    if (G.GridMapSet[current_layer].track[k].gridpoint[GN[0].grid_index.second].type == 0) {
                        G.GridMapSet[current_layer].track[k].gridpoint[GN[0].grid_index.second].type = 3;
                        P.bus_preserve[i].pin_preserve[j].push_back(&(G.GridMapSet[current_layer].track[k].gridpoint[GN[0].grid_index.second].type));
                    }
                }
                for (int k = 0; k < GN.size(); k++) GN[k].grid_index.second--;
            }
        }
    }
}
void preserve_deactivate (Preserve_space &P, int bus_id, int net_id) {
    for (int i = 0; i < P.bus_preserve[bus_id].pin_preserve[net_id].size(); i++) {
        *(P.bus_preserve[bus_id].pin_preserve[net_id][i]) = 0;
    }
}
void preserve_activate (Preserve_space &P, int bus_id, int net_id) {
    for (int i = 0; i < P.bus_preserve[bus_id].pin_preserve[net_id].size(); i++) {
        *(P.bus_preserve[bus_id].pin_preserve[net_id][i]) = 3;
    }
}
void BUSES_preserve_deactivate (Preserve_space &P, int bus_id) {
    for (int i = 0; i < P.bus_preserve[bus_id].pin_preserve.size(); i++)
        preserve_deactivate(P, bus_id, i);
}

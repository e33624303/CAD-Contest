#ifndef _OUTPUT_GDS_P
#define _OUTPUT_GDS_P
#include "input.h"
#include "router_Astar.h"
#include <iostream>
#include <fstream>

using namespace std;

void input_to_gds_parser (MainInput &M, All_route &A) {
    fstream fout;
    fout.open("input_to_gds_parser.txt", ios::out);
    fout << M.LayerSet.size() << endl;
    for (int i = 0; i < M.LayerSet.size(); i++) {
        fout << M.LayerSet[i].TrackSet.size() << endl;
        for (int j = 0; j < M.LayerSet[i].TrackSet.size(); j++) {
            fout << M.LayerSet[i].TrackSet[j].Track.LB.x << " " <<  M.LayerSet[i].TrackSet[j].Track.LB.y << " " <<  M.LayerSet[i].TrackSet[j].Track.RT.x << " " <<  M.LayerSet[i].TrackSet[j].Track.RT.y << endl;
        }
        fout << M.LayerSet[i].ObstacleSet.size() << endl;
        for (int j = 0; j < M.LayerSet[i].ObstacleSet.size(); j++) {
            fout << M.LayerSet[i].ObstacleSet[j].LB.x << " " << M.LayerSet[i].ObstacleSet[j].LB.y << " " << M.LayerSet[i].ObstacleSet[j].RT.x << " " << M.LayerSet[i].ObstacleSet[j].RT.y << endl;
        }
        int bus_cnt = 0;
        for (int j = 0; j < M.BusSet.size(); j++)
            for (int k = 0; k < M.BusSet[j].PinBitSet.size(); k++)
                for (int l = 0; l < M.BusSet[j].PinBitSet[k].PinShapeSet.size(); l++) {
                    if (M.BusSet[j].PinBitSet[k].PinShapeSet[l].LayerNum == M.MapLayerName[M.LayerSet[i].name]) {
                        bus_cnt++;
                    }
                }
        fout << bus_cnt << endl;
        for (int j = 0; j < M.BusSet.size(); j++)
            for (int k = 0; k < M.BusSet[j].PinBitSet.size(); k++)
                for (int l = 0; l < M.BusSet[j].PinBitSet[k].PinShapeSet.size(); l++) {
                    if (M.BusSet[j].PinBitSet[k].PinShapeSet[l].LayerNum == M.MapLayerName[M.LayerSet[i].name]) {
                        fout << M.BusSet[j].PinBitSet[k].PinShapeSet[l].Pin.LB.x << " " << M.BusSet[j].PinBitSet[k].PinShapeSet[l].Pin.LB.y << " " << M.BusSet[j].PinBitSet[k].PinShapeSet[l].Pin.RT.x << " " << M.BusSet[j].PinBitSet[k].PinShapeSet[l].Pin.RT.y << endl;
                    }
                }
        int route_cnt = 0;
        for (int j = 0; j < A.path.size(); j++) {
            for (int k = 0; k < A.path[j].bit_routes.size(); k++) {
                for (int l = 0; l < A.path[j].bit_routes[k].routes.size(); l++) {
                    if (A.path[j].bit_routes[k].routes[l].Layer == i) route_cnt++;
                }
            }
        }
        fout << route_cnt << endl;
        for (int j = 0; j < A.path.size(); j++) {
            for (int k = 0; k < A.path[j].bit_routes.size(); k++) {
                for (int l = 0; l < A.path[j].bit_routes[k].routes.size(); l++) {
                    if (A.path[j].bit_routes[k].routes[l].Layer == i) {
                        fout << A.path[j].bit_routes[k].routes[l].route_shape.LB.x << " " << A.path[j].bit_routes[k].routes[l].route_shape.LB.y << " " << A.path[j].bit_routes[k].routes[l].route_shape.RT.x << " " << A.path[j].bit_routes[k].routes[l].route_shape.RT.y << endl;
                    }
                }
            }
        }
    }
}

#endif

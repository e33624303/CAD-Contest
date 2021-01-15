#ifndef gridmap_h
#define gridmap_h

#include <iostream>
#include <vector>
#include <cstring>
#include <utility>
#include <algorithm>
#include "input.h"

using namespace std;

struct Coord{
	pair<int , int> limit;
	int coord;
	uint32_t width;
	bool extra=false;
	bool change_type=false;
};

struct Coordinate{
	int Orientation;
	//int spacing;
	vector<Coord> information;
};

struct PointType
{
	bool exist=false;
	pair<int , int> layer_track;
};

struct PointInformation{
	PointType up;
	PointType down;
	int type=0;	//0:unused  1:used  2:not exist 3:obstacle
	int coord;
};

struct Track{
	uint32_t coord;
	uint32_t width;
	bool neighbor=false;
	vector<int> neighbor_layer;
	vector<PointInformation> gridpoint;
};

struct MapInformation{
	vector<int> obstacle;
	vector<Track> track; 
};


class GridMap
{
public:
	GridMap( MainInput &MainInput );
	void OutputMap( vector<MapInformation> &GridMapSet );
	vector<MapInformation> GridMapSet;
	vector<Coordinate> MapSet;
	//void ConstructMapSet(vector<Coordinate> &MapSet , );
};

#endif


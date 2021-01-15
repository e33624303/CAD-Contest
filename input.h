#ifndef __input_H__
#define __input_H__

#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <string>
#include <map>

#define horizontal 0
#define vertical 1 

using namespace std ;

struct POINT{
	int x ;
	int y ;
	
	friend ostream& operator<<(ostream& os, const POINT& POINT){
		os << "(" << POINT.x << " " << POINT.y << ")";
		return os; 
	} 
};

struct RECT{
	POINT LB ;
	POINT RT ;
	
	friend ostream& operator<<(ostream& os, const RECT& oRECT){
		os << "(" << oRECT.LB.x << " " << oRECT.LB.y << ") (" << oRECT.RT.x << " " << oRECT.RT.y << ")" ;  
		return os; 
	} 
};

struct PINSHAPES{
	int LayerNum ;
	RECT Pin ;
};

struct PINBITS{
	vector<PINSHAPES> PinShapeSet ;
	//int BitNum ;
	string name ;
};

struct BUSES{
	string name ;
	vector<int> Width ;
	vector<PINBITS> PinBitSet ;
};

struct TRACKS{
	RECT Track ;
	int Width ;
};

struct LAYERS{
	string name ;
	int Orientation ;
	int Width ;
	vector<TRACKS> TrackSet ;
	vector<RECT> ObstacleSet ;
};

struct MainInput{
	int RUNTIME ;
	int ALPHA ;
	int BETA ;
	int GAMMA ;
	int DELTA ;
	int EPSILON ; 
	int bus_run_time;
	RECT DESIGN_BOUNDARY ;
	vector<LAYERS> LayerSet ;
	vector<BUSES> BusSet ;
	map<string,int> MapLayerName ;
	
	MainInput(string InputFileName);	
	void F_BasicPara( ifstream &tmpfile );
	void F_BOUNDARY( ifstream &tmpfile );
	void F_LAYERS( ifstream &tmpfile );
	void F_TRACKS( ifstream &tmpfile );
	void F_BUSES( ifstream &tmpfile );
	void F_OBSTACLE( ifstream &tmpfile );
	POINT F_POINT( ifstream &tmpfile );
	RECT F_RECT( ifstream &tmpfile );
};
//overeide out 



#endif 

//int
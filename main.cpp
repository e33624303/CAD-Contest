#include <iostream>

#include "gridmap.h"

#include "output_gds_parser.h"

#include "router_Astar.h"

#include "output.h"

#define OUTPUT_TO_GDS false

//void outputtest( MainInput &MainInput );

int main(int argc, char* argv[]){
	
	cout << "start" << endl ;
	string InputFileName(argv[1]);
	string OutputFileName(argv[2]);
	MainInput MainInput( InputFileName );
	// Start runtime calculation
    int bus_cnt = 0;
    for (int i = 0; i < MainInput.BusSet.size(); i++)
    	for (int j = 0; j < MainInput.BusSet[i].PinBitSet[0].PinShapeSet.size(); j++)
    		if (j > 0) bus_cnt++;
    float time_limit = (MainInput.RUNTIME * 0.4)/(float)(bus_cnt);
    if (time_limit > 5) time_limit = 5;
    int time_limit_sec = time_limit*60;
    cout << "BUS_COUNT = " << bus_cnt << endl;
    cout << "TIME_LIMIT(min) = " << time_limit << endl;
    cout << "TIME_LIMIT(sec) = " << time_limit_sec << endl;
    MainInput.bus_run_time = time_limit_sec;
    // END runtime calculation
	GridMap GridMap( MainInput );
    All_route routing_result;
    bus_router_func (routing_result, GridMap, MainInput);
    n_bus_router_func (routing_result , GridMap , MainInput);
    f_bus_router_func (routing_result , GridMap , MainInput);
    if (OUTPUT_TO_GDS) {
    	 cout << "Transfer to GDS_PARSER input\n";
    	 input_to_gds_parser(MainInput, routing_result);
    }
	//cout << "penalty_cost: " << penalty_cost( routing_result , MainInput) << endl ;
	cout << "====output====\n" ;
	OutputResult( routing_result , MainInput ,OutputFileName );
}


/*
void outputtest( MainInput &MainInput){
	cout << endl ;
	cout << MainInput.RUNTIME << MainInput.ALPHA << MainInput.BETA << MainInput.GAMMA << MainInput.DELTA << MainInput.EPSILON << endl ;
	cout << "DESIGN_BOUNDARY: " << MainInput.DESIGN_BOUNDARY << endl << endl ;
	
	//LAYERS
	cout << "LAYERNUM: " << MainInput.LayerSet.size() << endl << endl ;
	for ( int i = 0 ; i < MainInput.LayerSet.size() ; i++ ){
		cout <<  MainInput.LayerSet[i].name << " " << MainInput.LayerSet[i].Orientation << " " << MainInput.LayerSet[i].Width << endl ;
		cout << "TRACK" << endl ;
		for ( int j = 0 ; j < MainInput.LayerSet[i].TrackSet.size() ; j++ ){
			cout << MainInput.LayerSet[i].TrackSet[j].Track << " " << MainInput.LayerSet[i].TrackSet[j].Width << endl ;
		}
		cout << "OBS" << endl ;
		for ( int j = 0 ; j < MainInput.LayerSet[i].ObstacleSet.size() ; j++ ){
			cout << MainInput.LayerSet[i].ObstacleSet[j] << endl ;
		}
		cout << endl << endl ;
	}
	
	//BUSS
	cout << "BUSSIZE: " << MainInput.BusSet.size() << endl << endl ;
	for ( int i = 0 ; i < MainInput.BusSet.size() ; i++ ){
		cout << MainInput.BusSet[i].name << endl << "Width: " << MainInput.BusSet[i].Width.size()-1 << " : " ;
		for ( int j = 1 ; j < MainInput.BusSet[i].Width.size() ; j++ ) cout << MainInput.BusSet[i].Width[j] << " ";
		cout << endl << "BITSIZE: " << MainInput.BusSet[i].PinBitSet.size() << endl ;
		for ( int j = 0 ; j < MainInput.BusSet[i].PinBitSet.size() ; j++ ){
			cout << "BITNAME: " <<  MainInput.BusSet[i].PinBitSet[j].name << endl ;
			for ( int k = 0 ; k < MainInput.BusSet[i].PinBitSet[j].PinShapeSet.size() ; k++ ){
				cout << "L" <<  MainInput.BusSet[i].PinBitSet[j].PinShapeSet[k].LayerNum << 
				" " <<  MainInput.BusSet[i].PinBitSet[j].PinShapeSet[k].Pin << endl ;
			}
			
		}
		
		cout << endl << endl ;
	}
	
}*/

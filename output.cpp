#include "output.h"
#define no_or_some 0

bool cmp_h ( route i , route j ){ 
	return i.route_shape.LB.x < j.route_shape.LB.x ;
}

bool cmp_v ( route i , route j ){ 
	return i.route_shape.LB.y < j.route_shape.LB.y ;
}

void OutputResult(All_route& A , MainInput &M , string OutputFileName )
{
	fstream outfile ;
	outfile.open( OutputFileName , ios::out | ios::trunc );
	for( int i = 0 ; i < A.path.size() ; i++ ){
		bool route_success = true ;
		for ( int j = 0 ; j < A.bus_result[i].size() ; j++ ){
			if ( A.bus_result[i][j] == 0 ){ route_success = false ; break ;}
		}
		if ( route_success == false ){
			if ( no_or_some == 1 ){
				outfile << "BUS " << A.path[i].name << endl ;
				for ( int j = 0 ; j < A.path[i].bit_routes.size() ; j++ ){
					outfile << "BIT " << A.path[i].bit_routes[j].name << endl ;
					outfile << "PATH 0" << endl ;
					outfile << "ENDPATH" << endl ;
					outfile << "ENDBIT" << endl ;
				}
				outfile << "ENDBUS" << endl ;
			}
			continue ;
		}
		//cout << "BUS" << A.path[i].name << endl ;
		outfile << "BUS " << A.path[i].name << endl ;
		for ( int j = 0 ; j < A.path[i].bit_routes.size() ; j++ ){
			vector<route> tmp_route_to_via ;
			//cout << "BIT " << A.path[i].bit_routes[j].name << endl ;
			outfile << "BIT " << A.path[i].bit_routes[j].name << endl ;
			//outfile << "PATH " << A.path[i].bit_routes[j].routes.size() + A.path[i].via.size() << endl ;
			//outfile << "VIA " << A.path[i].via.size() << endl ;
			//outfile << "OLD " << A.path[i].bit_routes[j].routes.size() << endl ;
			/*via*/
			
			/*path*/
			/*connect path*/
			map<int,int> map_of_layer ;
			map<int,int>::iterator it;
			vector<vector<route>> layer_route ;
			vector<route> final_route ;
			int count_layer = 0 ;
			//cout << "A.path[i].bit_routes[j].routes.size()= " << A.path[i].bit_routes[j].routes.size() << endl ;
			for ( int k = 0 ; k < A.path[i].bit_routes[j].routes.size() ; k++ ){
				if (A.path[i].bit_routes[j].routes[k].route_shape.LB.x == A.path[i].bit_routes[j].routes[k].route_shape.RT.x && 
					A.path[i].bit_routes[j].routes[k].route_shape.LB.y == A.path[i].bit_routes[j].routes[k].route_shape.RT.y &&
					A.path[i].bit_routes[j].routes[k].Layer != M.LayerSet.size()-1
					){
					tmp_route_to_via.push_back(A.path[i].bit_routes[j].routes[k]);			
				}else{
					if ( map_of_layer.find( A.path[i].bit_routes[j].routes[k].Layer ) ==  map_of_layer.end() ){
						map_of_layer[A.path[i].bit_routes[j].routes[k].Layer] = count_layer ;
						vector<route> tmp_vector_route ;
						tmp_vector_route.push_back( A.path[i].bit_routes[j].routes[k] );
						layer_route.push_back(tmp_vector_route);
						count_layer++ ;
					}else{
						layer_route[ map_of_layer[A.path[i].bit_routes[j].routes[k].Layer] ].push_back( A.path[i].bit_routes[j].routes[k]);
					}
					/*outfile << M.LayerSet[A.path[i].bit_routes[j].routes[k].Layer].name << " " ;
					outfile << A.path[i].bit_routes[j].routes[k].route_shape << endl ;*/
				}	
			}
			
			for ( int k = 0 ; k < layer_route.size() ; k++ ){
				int ori =  M.LayerSet[layer_route[k][0].Layer].Orientation ;
				if( ori == 0 ){
					
					sort( layer_route[k].begin() , layer_route[k].end() , cmp_h ) ;
					sort( layer_route[k].begin() , layer_route[k].end() , cmp_v ) ;	
					int now_y = layer_route[k][0].route_shape.LB.y ;
					int now_L_x = layer_route[k][0].route_shape.LB.x ;
					int now_R_x = layer_route[k][0].route_shape.RT.x ;
					
					for ( int z = 0 ; z < layer_route[k].size() ; z++ ){
						if ( now_y == layer_route[k][z].route_shape.LB.y ){
							if ( layer_route[k][z].route_shape.LB.x > now_R_x ){
								route tmp_route ;
								tmp_route.Layer = layer_route[k][0].Layer ;
								tmp_route.route_shape.LB.x = now_L_x ;
								tmp_route.route_shape.RT.x = now_R_x ;
								tmp_route.route_shape.LB.y = now_y ;
								tmp_route.route_shape.RT.y = now_y ;
								final_route.push_back(tmp_route);
								
								now_L_x = layer_route[k][z].route_shape.LB.x ;
								now_R_x = layer_route[k][z].route_shape.RT.x ;
							}else {
								if ( now_R_x < layer_route[k][z].route_shape.RT.x ) now_R_x = layer_route[k][z].route_shape.RT.x ;
							}
						}else {
							route tmp_route ;
							tmp_route.Layer = layer_route[k][0].Layer ;
							tmp_route.route_shape.LB.x = now_L_x ;
							tmp_route.route_shape.RT.x = now_R_x ;
							tmp_route.route_shape.LB.y = now_y ;
							tmp_route.route_shape.RT.y = now_y ;
							final_route.push_back(tmp_route);
							
							now_L_x = layer_route[k][z].route_shape.LB.x ;
							now_R_x = layer_route[k][z].route_shape.RT.x ;
							now_y = layer_route[k][z].route_shape.LB.y ;
						}
						
						if ( z == layer_route[k].size()-1 ){
							route tmp_route ;
							tmp_route.Layer = layer_route[k][0].Layer ;
							tmp_route.route_shape.LB.x = now_L_x ;
							tmp_route.route_shape.RT.x = now_R_x ;
							tmp_route.route_shape.LB.y = now_y ;
							tmp_route.route_shape.RT.y = now_y ;
							final_route.push_back(tmp_route);
						}
						
					}					
				}else{ 
					sort( layer_route[k].begin() , layer_route[k].end() , cmp_v ) ;
					sort( layer_route[k].begin() , layer_route[k].end() , cmp_h ) ;
					int now_x = layer_route[k][0].route_shape.LB.x ;
					int now_B_y = layer_route[k][0].route_shape.LB.y ;
					int now_T_y = layer_route[k][0].route_shape.RT.y ;
					
					for ( int z = 0 ; z < layer_route[k].size() ; z++ ){
						if ( now_x == layer_route[k][z].route_shape.LB.x ){
							if ( layer_route[k][z].route_shape.LB.y > now_T_y ){
								route tmp_route ;
								tmp_route.Layer = layer_route[k][0].Layer ;
								tmp_route.route_shape.LB.y = now_B_y ;
								tmp_route.route_shape.RT.y = now_T_y ;
								tmp_route.route_shape.LB.x = now_x ;
								tmp_route.route_shape.RT.x = now_x ;
								final_route.push_back(tmp_route);
								
								now_B_y = layer_route[k][z].route_shape.LB.y ;
								now_T_y = layer_route[k][z].route_shape.RT.y ;
							}else {
								if ( now_T_y < layer_route[k][z].route_shape.RT.y ) now_T_y = layer_route[k][z].route_shape.RT.y ;
							}
						}else {
							route tmp_route ;
							tmp_route.Layer = layer_route[k][0].Layer ;
							tmp_route.route_shape.LB.y = now_B_y ;
							tmp_route.route_shape.RT.y = now_T_y ;
							tmp_route.route_shape.LB.x = now_x ;
							tmp_route.route_shape.RT.x = now_x ;
							final_route.push_back(tmp_route);
							
							now_B_y = layer_route[k][z].route_shape.LB.y ;
							now_T_y = layer_route[k][z].route_shape.RT.y ;
							now_x = layer_route[k][z].route_shape.LB.x ;
						}
						
						if ( z == layer_route[k].size()-1 ){
							route tmp_route ;
							tmp_route.Layer = layer_route[k][0].Layer ;
							tmp_route.route_shape.LB.y = now_B_y ;
							tmp_route.route_shape.RT.y = now_T_y ;
							tmp_route.route_shape.LB.x = now_x ;
							tmp_route.route_shape.RT.x = now_x ;
							final_route.push_back(tmp_route);
						}
						
					}	
					
				}
			}
			
			// via minus
			/*
			vector<Grid_node> via_final ;
			via_final.clear();

			for ( int k = 0 ; k < A.path[i].via.size() ; k++ ){
				bool same = false ;
				for ( int z = 0 ; z < via_final.size() ; z++ ){
					if ( A.path[i].via[k][j].grid_index.first == via_final[z].grid_index.first && A.path[i].via[k][j].grid_index.second == via_final[z].grid_index.second ){
						via_final[z].Layer = min( A.path[i].via[k][0].Layer , via_final[z].Layer ) ;
						same = true ;
						break ;
					}
				}
				
				if ( same == false ) via_final.push_back( A.path[i].via[k][j] );
			}
				*/
			// real output
			
			outfile << "PATH " << tmp_route_to_via.size() + A.path[i].via.size() + final_route.size() << endl ;
			//cout << "PATH " << tmp_route_to_via.size() + A.path[i].via.size() + final_route.size() << endl ;

			for ( int k = 0 ; k < tmp_route_to_via.size() ; k++ ){
				outfile << M.LayerSet[tmp_route_to_via[k].Layer].name << " " ;
				outfile << tmp_route_to_via[k].route_shape.LB << endl ;
				//cout << M.LayerSet[tmp_route_to_via[k].Layer].name << " " ;
				//cout << tmp_route_to_via[k].route_shape.LB << endl ;
			}
			//cout << endl ;
			//outfile << endl ;
			for ( int k = 0 ; k < A.path[i].via.size() ; k++ ){
				outfile << M.LayerSet[A.path[i].via[k][j].Layer].name << " " ;
				outfile << "(" << A.path[i].via[k][j].grid_index.first << " " << A.path[i].via[k][j].grid_index.second << ")" << endl ;
				//cout << M.LayerSet[A.path[i].via[k][j].Layer].name << " " ;
				//cout << "(" << A.path[i].via[k][j].grid_index.first << " " << A.path[i].via[k][j].grid_index.second << ")" << endl ;
			}
			
			for ( int k = 0 ; k < final_route.size() ; k++ ){
				outfile << M.LayerSet[final_route[k].Layer].name << " " ;
				outfile << final_route[k].route_shape << endl ;
				//cout << M.LayerSet[final_route[k].Layer].name << " " ;
				//cout << final_route[k].route_shape << endl ;
			}
			//outfile << "NEW " << final_route.size() << endl ;
			outfile << "ENDPATH" << endl << "ENDBIT" << endl ;
			//cout << "ENDPATH" << endl << "ENDBIT" << endl ;
		}
		outfile << "ENDBUS" << endl ;
		//cout << "ENDBUS" << endl ;
	}
	
}


/*
struct route{
	RECT route_shape ;
	int Layer ;
	vector<Grid_node> grid_points;
};


struct PINBITS_route{
	string name ;
	vector<route> routes ;
};

struct BUSES_route{
	string name ;
	uint32_t half_parameter ;
	vector<uint32_t> LowerWidth ;
	vector<PINBITS_route> bit_routes;
};
struct ALL_route{
	vector<BUSES_route> path ;
};


*/
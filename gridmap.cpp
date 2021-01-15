#include "gridmap.h"

bool compare(const PointInformation &a , const PointInformation &b){
	return a.coord < b.coord;
};

bool compare_coord(const Coord &a , const Coord &b){
	return a.coord < b.coord;
}


GridMap::GridMap(MainInput &MainInput){

	cout<<"start\n";
	for ( int i = 0 ; i < MainInput.LayerSet.size() ; i++ ){
		Coordinate coordinate;
		if(MainInput.LayerSet[i].Orientation == 0){		//horizontal
			Coord tmp;
			//tmp.coord=0;
			//tmp.limit=make_pair(MainInput.DESIGN_BOUNDARY.LB.x , MainInput.DESIGN_BOUNDARY.RT.x);
			//coordinate.information.push_back(tmp);
			for( int j = 0 ; j < MainInput.LayerSet[i].TrackSet.size() ; j++ ){
				tmp.coord=MainInput.LayerSet[i].TrackSet[j].Track.LB.y;
				tmp.width=MainInput.LayerSet[i].TrackSet[j].Width;
				//cout<<"coord push back "<<MainInput.LayerSet[i].TrackSet[j].Track.LB.y<<endl;
				int limit_first=(MainInput.DESIGN_BOUNDARY.LB.x < MainInput.LayerSet[i].TrackSet[j].Track.LB.x)?\
					MainInput.LayerSet[i].TrackSet[j].Track.LB.x : MainInput.DESIGN_BOUNDARY.LB.x ;
				int limit_second=(MainInput.DESIGN_BOUNDARY.RT.x < MainInput.LayerSet[i].TrackSet[j].Track.RT.x)?\
					MainInput.DESIGN_BOUNDARY.RT.x : MainInput.LayerSet[i].TrackSet[j].Track.RT.x;
				tmp.limit=make_pair(limit_first , limit_second);
				coordinate.information.push_back(tmp);
				//cout<<"coord limit push back "<<limit_first<<"  "<<limit_second<<endl;
			}
			for(int i2=0 ; i2<MainInput.BusSet.size() ; i2++){
				//for(int j=0 ; j<MainInput.BusSet[i2].PinBitSet.size() ; j++){
					for(int k=0 ; k<MainInput.BusSet[i2].PinBitSet[0].PinShapeSet.size() ; k++){
						int layer=MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].LayerNum;
						if(MainInput.LayerSet[layer].Orientation == 1 && abs(layer-i)==1 ){		//careful here
							if(MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.LB.x != MainInput.BusSet[i2].PinBitSet[1].PinShapeSet[k].Pin.LB.x){
								tmp.coord = (MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.LB.y+MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.RT.y)/2;
								tmp.limit = make_pair(MainInput.DESIGN_BOUNDARY.LB.x , MainInput.DESIGN_BOUNDARY.RT.x);
								tmp.extra=true;
								tmp.width=0;	//careful here
								bool find=false;
								for(int j=0 ; j<coordinate.information.size() ; j++){
									if(coordinate.information[j].coord == tmp.coord){
										find=true;
										break;
									}
								}
								if(!find)
									coordinate.information.push_back(tmp);
							}
							//cout<<"now bus "<<i2<<" bit0"<<" shape"<<k<<endl;
							//cout<<"coord : "<<MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.LB.x<<" , "<<MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.LB.y<<endl;
							//cout<<"now is layer "<<i<<endl;
							//cout<<"now another layere is "<<layer<<endl;
							//cout<<"coord is "<<tmp.coord<<endl;
						}
					}
				//}
			}
			//tmp.coord=MainInput.DESIGN_BOUNDARY.RT.y;
			//tmp.limit=make_pair(MainInput.DESIGN_BOUNDARY.LB.x , MainInput.DESIGN_BOUNDARY.RT.x);
			//coordinate.information.push_back(tmp);
			coordinate.Orientation=0;
		}		
		else{											//vertical
			Coord tmp;
			//tmp.coord=0;
			//tmp.limit=make_pair(MainInput.DESIGN_BOUNDARY.LB.y , MainInput.DESIGN_BOUNDARY.RT.y);
			//coordinate.information.push_back(tmp);
			for( int j = 0 ; j < MainInput.LayerSet[i].TrackSet.size() ; j++ ){
				tmp.coord=MainInput.LayerSet[i].TrackSet[j].Track.LB.x;
				tmp.width=MainInput.LayerSet[i].TrackSet[j].Width;
				int limit_first=(MainInput.DESIGN_BOUNDARY.LB.y < MainInput.LayerSet[i].TrackSet[j].Track.LB.y)?\
					MainInput.LayerSet[i].TrackSet[j].Track.LB.y : MainInput.DESIGN_BOUNDARY.LB.y ;
				int limit_second=(MainInput.DESIGN_BOUNDARY.RT.y < MainInput.LayerSet[i].TrackSet[j].Track.RT.y)?\
					 MainInput.DESIGN_BOUNDARY.RT.y : MainInput.LayerSet[i].TrackSet[j].Track.RT.y ;
				tmp.limit=make_pair(limit_first , limit_second);
				coordinate.information.push_back(tmp);
			}
			for(int i2=0 ; i2<MainInput.BusSet.size() ; i2++){
				//for(int j=0 ; j<MainInput.BusSet[i2].PinBitSet.size() ; j++){
					for(int k=0 ; k<MainInput.BusSet[i2].PinBitSet[0].PinShapeSet.size() ; k++){
						int layer=MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].LayerNum;
						//cout<<"now bus "<<i2<<" bit0"<<" shape"<<k<<endl;
						//cout<<"coord : "<<MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.LB.x<<" , "<<MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.LB.y<<endl;
						//cout<<"now i="<<i<<". layer="<<layer<<endl;
						if(MainInput.LayerSet[layer].Orientation == 0 && abs(layer-i)==1 ){		//careful here
							if(MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.LB.y != MainInput.BusSet[i2].PinBitSet[1].PinShapeSet[k].Pin.LB.y){
								tmp.coord = (MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.LB.x+MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.RT.x)/2;
								tmp.limit = make_pair(MainInput.DESIGN_BOUNDARY.LB.y , MainInput.DESIGN_BOUNDARY.RT.y);
								tmp.extra=true;
								tmp.width=0;	//careful here
								bool find=false;
								for(int j=0 ; j<coordinate.information.size() ; j++){
									if(coordinate.information[j].coord == tmp.coord){
										find=true;
										break;
									}
								}
								if(!find)
									coordinate.information.push_back(tmp);
							}
							//cout<<"now bus "<<i2<<" bit0"<<" shape"<<k<<endl;
							//cout<<"coord : "<<MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.LB.x<<" , "<<MainInput.BusSet[i2].PinBitSet[0].PinShapeSet[k].Pin.LB.y<<endl;
							//cout<<"now is layer "<<i<<endl;
							//cout<<"now another layere is "<<layer<<endl;
							//cout<<"coord is "<<tmp.coord<<endl;
						}
					}
				//}
			}

			//tmp.coord=MainInput.DESIGN_BOUNDARY.RT.x;
			//tmp.limit=make_pair(MainInput.DESIGN_BOUNDARY.LB.y , MainInput.DESIGN_BOUNDARY.RT.y);
			//coordinate.information.push_back(tmp);
			coordinate.Orientation=1;
		}
		sort(coordinate.information.begin() , coordinate.information.end() , compare_coord);
		MapSet.push_back(coordinate);
	}
	/*for(int i=0 ; i<MainInput.LayerSet.size() ; i++){
		for(int j=0 ; j<MapSet[i].information.size() ; j++){
			if(MapSet[i].information[j].extra == true){
				bool change=false;
				if(i!=0){
					if(MapSet[i-1].Orientation != MapSet[i].Orientation){
						if(MapSet[i-1].information[0].coord > MapSet[i].information[j].)
					}
				}
			}
		}
	}*/
	//cout<<"part 1 finish\n";
	//start construct gridmap

	for( int i = 0 ; i < MainInput.LayerSet.size() ; i++ ){
		int top=i;
		int bottom=i;
		while(top!=MainInput.LayerSet.size()-1){
			top++;
			if(MainInput.LayerSet[top].Orientation != MainInput.LayerSet[i].Orientation)
				break;
		}
		while(bottom!=0){
			bottom--;
			if(MainInput.LayerSet[bottom].Orientation != MainInput.LayerSet[i].Orientation)
				break;
		}
		cout<<"Layer"<<i<<"  top reference : "<<top<<"  bottom reference : "<<bottom<<endl;


		MapInformation map_tmp;
		vector<Track> track_tmp,track_tmp2;
		vector<PointInformation > point_tmp;
		bool initial=false;

		for(int j = top ; j >= i ; j--){
			if(j == i)
				continue;
			if(j == top){		//i 當前層   j 參考層   k 當前層座標   l 參考層座標   m obstacle處理
				//cout<<"now layer"<<i<<" reference top layer "<<j<<endl;
				initial=true;
				for(int k = 0 ; k < MapSet[i].information.size() ; k++){
					Track track;
					track.coord=MapSet[i].information[k].coord;
					track.width=MapSet[i].information[k].width;
					for(int l = 0 ; l < MapSet[j].information.size() ; l++){
						if(l!=MapSet[j].information.size()-1 && MapSet[j].information[l].coord == MapSet[j].information[l+1].coord){
							//cout<<"now is j = "<<j<<"  coord is "<<MapSet[j].information[l].coord<<endl;
						}
						PointInformation point;
						point.up.exist=true;
						point.coord=MapSet[j].information[l].coord;
						point.up.layer_track=make_pair(j,l);
						/*if(MapSet[j].information[l].limit.first > track.coord || MapSet[j].information[l].limit.second < track.coord)
							point.type=2;
						else */
						if(MapSet[i].information[k].extra==true)
							point.type=2;
						else if(MapSet[j].information[l].coord < MapSet[i].information[k].limit.first || MapSet[j].information[l].coord > MapSet[i].information[k].limit.second)
							point.type=2;
						else
							point.type=0;
						point_tmp.push_back(point);
					}
					sort(point_tmp.begin() , point_tmp.end() , compare);
					track.gridpoint=point_tmp;
					track_tmp.push_back(track);
					point_tmp.clear();
				}
			}
			else{
				for(int k = 0 ; k < MapSet[i].information.size() ; k++){
					for(int l = 0 ; l < MapSet[j].information.size() ; l++){
						if( MapSet[i].information[k].coord == MapSet[j].information[l].coord ){
							track_tmp[k].neighbor=true;
							track_tmp[k].neighbor_layer.push_back(j);
							for(int m = 0 ; m < track_tmp[k].gridpoint.size() ; m++){
								track_tmp[k].gridpoint[m].up.layer_track=make_pair(j,l);
							}
						}
					}
				}
			}
		}
		//cerr<<"here\n";
		for(int j = bottom ; j <= i ; j++){
			if( j == i){
				int back=0;
				for(int k=0 ; k<track_tmp.size() ; k++){
					back=k;
					if(k!=track_tmp.size()-1){
						for(int l=k+1 ; l<track_tmp.size() ; l++){
							if(track_tmp[k].coord != track_tmp[l].coord){
								break;
							}
							else
								back=l;
						}
					}
					if(back!=k){
						for(int l=k ; l<=back ; l++){
							for(int m=0 ; m<track_tmp[l].gridpoint.size() ; m++){
								if(track_tmp[l].gridpoint[m].type == 0){
									track_tmp[k].gridpoint[m].type=0;
								}
							}
						}
						track_tmp2.push_back(track_tmp[k]);
						k=back;
					}
					else{
						track_tmp2.push_back(track_tmp[k]);
					}
				}
				track_tmp.clear();
				track_tmp=track_tmp2;
				continue;
			}
			if(j == bottom && initial){		//i 當前層   j 參考層   k 當前層座標   l 參考層座標
				//cout<<"now layer"<<i<<" reference bottom layer "<<j<<endl;
				for(int k = 0 ; k < MapSet[i].information.size() ; k++){
					for(int l = 0 ; l < MapSet[j].information.size() ; l++){
						bool exist=false;
						int position=0;
						/*for(int m = 0 ; m < track_tmp[k].gridpoint.size() ; m++){
							if(MapSet[j].information[l].coord == track_tmp[k].gridpoint[m].coord){
								exist=true;
								position=m;
								break;
							}
						}*/
						if(!exist){
							PointInformation point;
							point.down.exist=true;
							point.coord=MapSet[j].information[l].coord;
							point.down.layer_track=make_pair(j , l);
							/*if(MapSet[j].information[l].limit.first > MapSet[i].information[k].coord || MapSet[j].information[l].limit.second < MapSet[i].information[k].coord)
								point.type=2;
							else */
							if(MapSet[i].information[k].extra==true)
								point.type=2;
							else if(MapSet[j].information[l].coord < MapSet[i].information[k].limit.first || MapSet[j].information[l].coord > MapSet[i].information[k].limit.second)
								point.type=2;
							else
								point.type=0;
							track_tmp[k].gridpoint.push_back(point);
						}
						else{
							track_tmp[k].gridpoint[position].down.exist=true;
							track_tmp[k].gridpoint[position].down.layer_track=make_pair(j,l);
							if(track_tmp[k].gridpoint[position].type==2){
								/*if(MapSet[j].information[l].limit.first > MapSet[i].information[k].coord || MapSet[j].information[l].limit.second < MapSet[i].information[k].coord)
									track_tmp[k].gridpoint[position].type=2;*/
								if(MapSet[i].information[k].extra==true)
									track_tmp[k].gridpoint[position].type=2;
								else if(MapSet[j].information[l].coord < MapSet[i].information[k].limit.first || MapSet[j].information[l].coord > MapSet[i].information[k].limit.second)
									track_tmp[k].gridpoint[position].type=2;
								else
									track_tmp[k].gridpoint[position].type=0;
							}
						}
					}
					//cout<<"before size : "<<track_tmp[k].gridpoint.size()<<endl;
					sort(track_tmp[k].gridpoint.begin() , track_tmp[k].gridpoint.end() , compare );
					vector<PointInformation> new_gridpoint;
					bool push=true;
					bool push2=true;
					bool push3=true;
					bool push4=true;
					int tmp_number; 
					for(int l = 0 ; l < track_tmp[k].gridpoint.size() ; l++){
						tmp_number=l;
						if(l!=track_tmp[k].gridpoint.size()-1){
							for(int m = l+1 ; m < track_tmp[k].gridpoint.size() ; m++){
								if(track_tmp[k].gridpoint[l].coord != track_tmp[k].gridpoint[m].coord)
									break;
								else
									tmp_number=m;
							}
						}
						if(tmp_number!=l){
							for(int m = l ; m <= tmp_number ; m++){
								if(track_tmp[k].gridpoint[m].up.exist == true)
									track_tmp[k].gridpoint[l].up.exist=true;
								if(track_tmp[k].gridpoint[m].down.exist == true)
									track_tmp[k].gridpoint[l].down.exist=true;
							}
							new_gridpoint.push_back(track_tmp[k].gridpoint[l]);
							l=tmp_number;
						}
						else{
							new_gridpoint.push_back(track_tmp[k].gridpoint[l]);
						}
					}
					track_tmp[k].gridpoint.clear();
					track_tmp[k].gridpoint=new_gridpoint;
					//cout<<"after size : "<<track_tmp[k].gridpoint.size()<<endl;
				}
				int back=0;
				for(int k=0 ; k<track_tmp.size() ; k++){
					back=k;
					if(k!=track_tmp.size()-1){
						for(int l=k+1 ; l<track_tmp.size() ; l++){
							if(track_tmp[k].coord != track_tmp[l].coord){
								break;
							}
							else
								back=l;
						}
					}
					if(back!=k){
						for(int l=k ; l<=back ; l++){
							for(int m=0 ; m<track_tmp[l].gridpoint.size() ; m++){
								if(track_tmp[l].gridpoint[m].type == 0){
									track_tmp[k].gridpoint[m].type=0;
								}
							}
						}
						track_tmp2.push_back(track_tmp[k]);
						k=back;
					}
					else{
						track_tmp2.push_back(track_tmp[k]);
					}
				}
				track_tmp.clear();
				track_tmp=track_tmp2;
			}
			else if(j == bottom){
				//cout<<"now layer"<<i<<" reference bottom layer "<<j<<endl;
				for(int k = 0 ; k < MapSet[i].information.size() ; k++){
					Track track;
					track.coord=MapSet[i].information[k].coord;
					track.width=MapSet[i].information[k].width;
					for(int l = 0 ; l < MapSet[j].information.size() ; l++){
						PointInformation point;
						point.down.exist=true;
						point.coord=MapSet[j].information[l].coord;
						point.down.layer_track=make_pair(j , l);
						/*if(MapSet[j].information[l].limit.first > track.coord || MapSet[j].information[l].limit.second < track.coord)
							point.type=2;
						else */
						if(MapSet[i].information[k].extra==true)
							point.type=2;
						else if(MapSet[j].information[l].coord < MapSet[i].information[k].limit.first || MapSet[j].information[l].coord > MapSet[i].information[k].limit.second)
							point.type=2;
						else
							point.type=0;
						point_tmp.push_back(point);
					}
					sort(point_tmp.begin() , point_tmp.end() , compare);
					track.gridpoint=point_tmp;
					track_tmp.push_back(track);
					point_tmp.clear();
				}
				int back=0;
				for(int k=0 ; k<track_tmp.size() ; k++){
					back=k;
					if(k!=track_tmp.size()-1){
						for(int l=k+1 ; l<track_tmp.size() ; l++){
							if(track_tmp[k].coord != track_tmp[l].coord){
								break;
							}
							else
								back=l;
						}
					}
					if(back!=k){
						for(int l=k ; l<=back ; l++){
							for(int m=0 ; m<track_tmp[l].gridpoint.size() ; m++){
								if(track_tmp[l].gridpoint[m].type == 0){
									track_tmp[k].gridpoint[m].type=0;
								}
							}
						}
						track_tmp2.push_back(track_tmp[k]);
						k=back;
					}
					else{
						track_tmp2.push_back(track_tmp[k]);
					}
				}
				track_tmp.clear();
				track_tmp=track_tmp2;
			}
			else{
				for(int k = 0 ; k < MapSet[i].information.size() ; k++){
					for(int l = 0 ; l < MapSet[j].information.size() ; l++){
						if( MapSet[i].information[k].coord == MapSet[j].information[l].coord ){
							track_tmp[k].neighbor=true;
							track_tmp[k].neighbor_layer.push_back(j);
							for(int m = 0 ; m < track_tmp[k].gridpoint.size() ; m++){
								track_tmp[k].gridpoint[m].down.layer_track=make_pair(j,l);
							}
						}
					}
					if( track_tmp[k].neighbor_layer.size() > 0 )
						sort(track_tmp[k].neighbor_layer.begin() , track_tmp[k].neighbor_layer.end());
				}
			}
		}
		map_tmp.track=track_tmp;
		GridMapSet.push_back(map_tmp);
	}

	//finish gridmap part

	//start obstacle part
/*for(int m = 0 ; m < MainInput.LayerSet[j].ObstacleSet.size() ; m++){
							if(MainInput.LayerSet[j].Orientation == 0){			//horizontal
								if( (MainInput.LayerSet[j].ObstacleSet[m].RT.x < MapSet[j].coord[l] || \
									MainInput.LayerSet[j].ObstacleSet[m].LB.x > MapSet[j].coord[l] ) && \
									(MainInput.LayerSet[j].ObstacleSet[m].RT.y > MapSet[i].coord[k] && \
									MainInput.LayerSet[j].ObstacleSet[m].LB.y < MapSet[i].coord[k])
								  )
									point.type=3;
							}	
							else{												//vertical
								if( (MainInput.LayerSet[j].ObstacleSet[m].RT.y < MapSet[j].coord[l] || \
									MainInput.LayerSet[j].ObstacleSet[m].LB.y > MapSet[j].coord[l] ) && \
									(MainInput.LayerSet[j].ObstacleSet[m].RT.x > MapSet[i].coord[k] && \
									MainInput.LayerSet[j].ObstacleSet[m].LB.x < MapSet[i].coord[k])
								  )
									point.type=3;
							}
						}*/



	for(int i=0 ; i<GridMapSet.size() ; i++){		//i:每層
		for(int j=0;j<MainInput.LayerSet[i].ObstacleSet.size();j++){	//j:每個障礙物

			bool use=false;

			for(int k=0;k<GridMapSet[i].track.size();k++){		//k:每個track
				if(MainInput.LayerSet[i].Orientation == 0){

					if(GridMapSet[i].track[k].coord <= MainInput.LayerSet[i].ObstacleSet[j].LB.y-(MainInput.LayerSet[i].Width+0.5*GridMapSet[i].track[k].width)\
						|| GridMapSet[i].track[k].coord >= MainInput.LayerSet[i].ObstacleSet[j].RT.y+MainInput.LayerSet[i].Width+0.5*GridMapSet[i].track[k].width)
						continue;

					for(int l=0;l<GridMapSet[i].track[k].gridpoint.size();l++){		//l:每個track之point
						if(GridMapSet[i].track[k].gridpoint[l].coord > MainInput.LayerSet[i].ObstacleSet[j].LB.x-(MainInput.LayerSet[i].Width+0.5*GridMapSet[i].track[k].width)\
							&& GridMapSet[i].track[k].gridpoint[l].coord < MainInput.LayerSet[i].ObstacleSet[j].RT.x+MainInput.LayerSet[i].Width+0.5*GridMapSet[i].track[k].width){
							GridMapSet[i].track[k].gridpoint[l].type=3;
							use=true;
						}
					}
				}
				else{
					if(GridMapSet[i].track[k].coord <= MainInput.LayerSet[i].ObstacleSet[j].LB.x-(MainInput.LayerSet[i].Width+0.5*GridMapSet[i].track[k].width)\
						|| GridMapSet[i].track[k].coord >= MainInput.LayerSet[i].ObstacleSet[j].RT.x+MainInput.LayerSet[i].Width+0.5*GridMapSet[i].track[k].width)
						continue;
				
					for(int l=0;l<GridMapSet[i].track[k].gridpoint.size();l++){		//l:每個track之point
						if(GridMapSet[i].track[k].gridpoint[l].coord > MainInput.LayerSet[i].ObstacleSet[j].LB.y-(MainInput.LayerSet[i].Width+0.5*GridMapSet[i].track[k].width)\
							&& GridMapSet[i].track[k].gridpoint[l].coord < MainInput.LayerSet[i].ObstacleSet[j].RT.y+MainInput.LayerSet[i].Width+0.5*GridMapSet[i].track[k].width){
							GridMapSet[i].track[k].gridpoint[l].type=3;
							use=true;
						}
					}
				}
			}

			if(use == false){		//this obstacle is not on any gridpoint
				for(int k=0 ; k<GridMapSet[i].track.size();k++){
					if(MainInput.LayerSet[i].Orientation == 0){
						if(GridMapSet[i].track[k].coord >= MainInput.LayerSet[i].ObstacleSet[j].LB.y\
							&& GridMapSet[i].track[k].coord <= MainInput.LayerSet[i].ObstacleSet[j].RT.y){
							PointInformation point;
							point.type=3;
							point.coord=(MainInput.LayerSet[i].ObstacleSet[j].LB.x + MainInput.LayerSet[i].ObstacleSet[j].RT.x)/2;
							GridMapSet[i].track[k].gridpoint.push_back(point);
							sort(GridMapSet[i].track[k].gridpoint.begin() , GridMapSet[i].track[k].gridpoint.end() , compare);
						}
					}
					else{
						if(GridMapSet[i].track[k].coord >= MainInput.LayerSet[i].ObstacleSet[j].LB.x\
							&& GridMapSet[i].track[k].coord <= MainInput.LayerSet[i].ObstacleSet[j].RT.x){
							PointInformation point;
							point.type=3;
							point.coord=(MainInput.LayerSet[i].ObstacleSet[j].LB.y + MainInput.LayerSet[i].ObstacleSet[j].RT.y)/2;
							GridMapSet[i].track[k].gridpoint.push_back(point);
							sort(GridMapSet[i].track[k].gridpoint.begin() , GridMapSet[i].track[k].gridpoint.end() , compare);
						}
					}
				}
			}
		}
	} 

	/*for(int i=0 ; i<MainInput.BusSet.size() ; i++){
		for(int j=0 ; j<MainInput.BusSet[i].PinBitSet.size() ; j++){
			for(int k=0 ; k<MainInput.BusSet[i].PinBitSet[j].PinShapeSet.size() ; k++){
				int layer=MainInput.BusSet[i].PinBitSet[j].PinShapeSet[k].LayerNum;
				if(MainInput.LayerSet[layer].Orientation == 0){
					for(int l=0 ; l<GridMapSet[layer].track.size() ; l++){
						if(GridMapSet[layer].track[l].coord >= MainInput.BusSet[i].PinBitSet[j].PinShapeSet[k].Pin.LB.y\
							&& GridMapSet[layer].track[l].coord <= MainInput.BusSet[i].PinBitSet[j].PinShapeSet[k].Pin.RT.y){
							for(int m=0 ; m<GridMapSet[layer].track[l].gridpoint.size() ; m++){
								if(GridMapSet[layer].track[l].gridpoint[m].coord >= MainInput.BusSet[i].PinBitSet[j].PinShapeSet[k].Pin.LB.x\
									&& GridMapSet[layer].track[l].gridpoint[m].coord <= MainInput.BusSet[i].PinBitSet[j].PinShapeSet[k].Pin.RT.x){
									GridMapSet[layer].track[l].gridpoint[m].type=0;
									break;
								}
							}
							break;
						}
					}
				}
				else{
					for(int l=0 ; l<GridMapSet[layer].track.size() ; l++){
						if(GridMapSet[layer].track[l].coord >= MainInput.BusSet[i].PinBitSet[j].PinShapeSet[k].Pin.LB.x\
							&& GridMapSet[layer].track[l].coord <= MainInput.BusSet[i].PinBitSet[j].PinShapeSet[k].Pin.RT.x){
							for(int m=0 ; m<GridMapSet[layer].track[l].gridpoint.size() ; m++){
								if(GridMapSet[layer].track[l].gridpoint[m].coord >= MainInput.BusSet[i].PinBitSet[j].PinShapeSet[k].Pin.LB.y\
									&& GridMapSet[layer].track[l].gridpoint[m].coord <= MainInput.BusSet[i].PinBitSet[j].PinShapeSet[k].Pin.RT.y){
									GridMapSet[layer].track[l].gridpoint[m].type=0;
									break;
								}
							}
							break;
						}
					}
				}
			}
		}
	}*/

	/*for(int i=0 ; i<GridMapSet.size() ; i++){
		cout<<"now layer"<<i<<" erase track coord : "<<GridMapSet[i].track.begin()->coord<<endl;
		GridMapSet[i].track.erase(GridMapSet[i].track.begin());
		cout<<"now layer"<<i<<" erase track coord : "<<(GridMapSet[i].track.end()-1)->coord<<endl;
		GridMapSet[i].track.erase(GridMapSet[i].track.end()-1);
	}*/
	cout<<"finish Gridmap part\n";

	//utputMap(GridMapSet);
}

void GridMap::OutputMap( vector<MapInformation> &GridMapSet ){
	for(int i = 0 ; i < GridMapSet.size() ; i++){
		for(int j = 0 ; j < GridMapSet[i].track.size() ; j++){
			cout<<"now track"<<j<<" on layer"<<i<<" and it's coord is "<<GridMapSet[i].track[j].coord<<endl;
			cout<<"this track have "<<GridMapSet[i].track[j].gridpoint.size()<<" gridpoint\n";
			for(int k = 0 ; k < GridMapSet[i].track[j].gridpoint.size() ; k++){
				if(GridMapSet[i].track[j].gridpoint[k].up.exist){
					cout<<"gridpoint top"<<k<<" coord is "<<GridMapSet[i].track[j].gridpoint[k].coord;
					cout<<" and type is "<<GridMapSet[i].track[j].gridpoint[k].type<<endl;
					cout<<"it's layer "<<GridMapSet[i].track[j].gridpoint[k].up.layer_track.first<<"  it's track "<<GridMapSet[i].track[j].gridpoint[k].up.layer_track.second<<endl;
				}
				if(GridMapSet[i].track[j].gridpoint[k].down.exist){
					cout<<"gridpoint down"<<k<<" coord is "<<GridMapSet[i].track[j].gridpoint[k].coord;
					cout<<" and type is "<<GridMapSet[i].track[j].gridpoint[k].type<<endl;
					cout<<"it's layer "<<GridMapSet[i].track[j].gridpoint[k].down.layer_track.first<<"  it's track "<<GridMapSet[i].track[j].gridpoint[k].down.layer_track.second<<endl;
				}
			}
		}
	}
}

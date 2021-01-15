#include "input.h"
#include <cmath>

MainInput::MainInput(string InputFileName )
{
	ifstream infile( InputFileName );
	
	if ( !infile.is_open() ) cerr << "open file fail!" << endl ;
	else {
		
		cout << "open file success!" << endl ;
		F_BasicPara(infile);
		cout << "F_BasicPara success!" << endl ;
		F_BOUNDARY(infile);
		cout << "F_BOUNDARY success!" << endl ;
		F_LAYERS(infile);
		cout << "F_LAYERS success!" << endl ;
		F_TRACKS(infile);
		cout << "F_TRACKS success!" << endl ;
		F_BUSES(infile);
		cout << "F_BUSES success!" << endl ;
		F_OBSTACLE(infile);
		cout << "F_OBSTACLE success!" << endl ;
		cout << "read file success!" << endl ;
	}
	
	infile.close();
}


void MainInput::F_BasicPara( ifstream &tmpfile ){
	string Trash ;
	tmpfile >> Trash >> RUNTIME ;
	tmpfile >> Trash >> ALPHA ;
	tmpfile >> Trash >> BETA ;
	tmpfile >> Trash >> GAMMA ;
	tmpfile >> Trash >> DELTA ;
	tmpfile >> Trash >> EPSILON ;
}

void MainInput::F_BOUNDARY( ifstream &tmpfile ){
	string Trash ;
	tmpfile >> Trash ;
	DESIGN_BOUNDARY = F_RECT( tmpfile ) ;
}


void MainInput::F_LAYERS( ifstream &tmpfile ){
	//LAYERS layer0 ;
	//LayerSet.push_back(layer0);
	
	string Trash ;
	string orient ;
	int Layersize ;
	
	tmpfile >> Trash >> Layersize ;
	for ( int i = 0 ; i < Layersize ; i++ ){
		LAYERS tmpLayer ;
		string tmpS ;
		tmpfile >> tmpLayer.name >> tmpS >> tmpLayer.Width ;
		MapLayerName[tmpLayer.name] = i ;
		if ( tmpS == "horizontal" ) tmpLayer.Orientation = 0 ;
		else if ( tmpS == "vertical" ) tmpLayer.Orientation = 1 ;
		LayerSet.push_back(tmpLayer); 
	}
	tmpfile >> Trash ;
	
}

void MainInput::F_TRACKS( ifstream &tmpfile ){
	
	string Trash ;
	//char tmpChar ;
	int Layernum ;
	int Tracksize ;
	
	tmpfile >> Trash >> Tracksize ;
	while( Tracksize-- ){
		TRACKS tmpTRACKS ;
		tmpfile >> Trash ;
		Layernum = MapLayerName[ Trash ];
		//cout << tmpChar << " " << Layernum << " " ;
		tmpTRACKS.Track = F_RECT(tmpfile);
		tmpfile >> tmpTRACKS.Width ;
		//cout << tmpTRACKS.Width << endl ;
		LayerSet[Layernum].TrackSet.push_back(tmpTRACKS);
	}
	tmpfile >> Trash ;
}

void MainInput::F_BUSES( ifstream &tmpfile ){
	//BUSES bus0 ;
	//BusSet.push_back(bus0);
	
	string Trash ;
	//char tmpChar ;
	int Bussize ;
	
	tmpfile >> Trash >> Bussize ;
	while( Bussize-- ){
		BUSES tmpBUSES ;
		int BusPinBitSize , BusPinShapeSize , WidthSize ;
		tmpfile >> Trash >> tmpBUSES.name >> BusPinBitSize >> BusPinShapeSize >> Trash >> WidthSize ;

		for ( int i = 0 ; i < WidthSize ; i++ ){
			int tmpUINT ;
			tmpfile >> tmpUINT ;
			tmpBUSES.Width.push_back(tmpUINT);
		}
				
		tmpfile >> Trash ; //ENDWIDTH  
		for ( int i = 0 ; i < BusPinBitSize ; i++ ){
			PINBITS tmpPINBITS ;
			tmpfile >> Trash >> tmpPINBITS.name ;//BIT name
			tmpPINBITS.PinShapeSet.resize(BusPinShapeSize);
			for ( int j = 0 ; j < BusPinShapeSize ; j++ ){
				PINSHAPES tmpPINSHAPES ;
				tmpfile >> Trash ;
				tmpPINSHAPES.LayerNum = MapLayerName[ Trash ]; //L1 
				tmpPINSHAPES.Pin = F_RECT(tmpfile);
				int nearest = -1 ;
				int nearestnum ;
				//cout << "BusPinShapeSize" << j << endl ;
				if ( i != 0 ){
					for ( int k = 0 ; k < BusPinShapeSize ; k++ ){
						if ( tmpBUSES.PinBitSet[i-1].PinShapeSet[k].LayerNum == tmpPINSHAPES.LayerNum ){
							int distance = 0 ;
							distance = max(abs( tmpBUSES.PinBitSet[i-1].PinShapeSet[k].Pin.LB.x + tmpBUSES.PinBitSet[i-1].PinShapeSet[k].Pin.RT.x - tmpPINSHAPES.Pin.LB.x - tmpPINSHAPES.Pin.RT.x ),abs( tmpBUSES.PinBitSet[i-1].PinShapeSet[k].Pin.LB.y + tmpBUSES.PinBitSet[i-1].PinShapeSet[k].Pin.RT.y - tmpPINSHAPES.Pin.LB.y - tmpPINSHAPES.Pin.RT.y )) ;

							if ( nearest < 0 || nearest > distance ){
								nearest = distance ;
								nearestnum = k ;
							}
							//cout << k << " distance == " << distance << endl ;
						}
					}
					//cout << " nearestnum == " << nearestnum << endl ;
					tmpPINBITS.PinShapeSet[nearestnum] = tmpPINSHAPES;
					//tmpPINBITS.PinShapeSet[nearestnum] = tmpPINSHAPES ;
				}else {
					tmpPINBITS.PinShapeSet[j] = tmpPINSHAPES;

					//tmpPINBITS.PinShapeSet[j] = tmpPINSHAPES;
				}
				
			}
		
			tmpBUSES.PinBitSet.push_back(tmpPINBITS);
			tmpfile >> Trash ;//ENDBITS
		}
		tmpfile >> Trash ; //ENDNUS
		
		

		BusSet.push_back(tmpBUSES);
	}
	tmpfile >> Trash ; //ENDBUSES
}

void MainInput::F_OBSTACLE( ifstream &tmpfile ){
	
	string Trash ;
	//char tmpChar ;
	int Layernum ;
	int ObsracleSize ;
	
	tmpfile >> Trash >> ObsracleSize ;
	while( ObsracleSize-- ){	
		tmpfile >> Trash ;
		Layernum = MapLayerName[ Trash ];
		//cout << tmpChar << " " << Layernum << " " << endl ;
		LayerSet[Layernum].ObstacleSet.push_back(F_RECT(tmpfile));
	}
	tmpfile >> Trash ;
}

//Bacis data

POINT MainInput::F_POINT( ifstream &tmpfile ){
	string Trash ;
	POINT tmpPoint ;
	char cTrash ;
	tmpfile >> cTrash >> tmpPoint.x ;	
	tmpfile >> tmpPoint.y >> cTrash ;	
	return tmpPoint ;
}

RECT MainInput::F_RECT( ifstream &tmpfile ){
	RECT tmpRECT ;
	tmpRECT.LB = F_POINT(tmpfile);
	tmpRECT.RT = F_POINT(tmpfile);
	return tmpRECT ;
}



#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <time.h>
#include <fstream>
#include <deque>
#include <fstream>
#include <math.h>
#include "GDT-4.0.4/gdt2gds.h"
#include "RTree.h"
#include "Segment.cpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/foreach.hpp>
#include <boost/polygon/polygon.hpp>
#include <boost/polygon/polygon_set_data.hpp>
#include "clipper/clipper.hpp"

using namespace boost::polygon::operators;
using namespace std;
using namespace ClipperLib;
using boost::geometry::get;

namespace geom = boost::geometry;
namespace gtl = boost::polygon;

typedef geom::model::d2::point_xy<int> point_type;
typedef geom::model::polygon<point_type> polygon;
typedef boost::geometry::model::d2::point_xy<int> boost_point;

class via{
public :
	int x, y;
	int layer;
};

class trp2gdt{

	ifstream trp;
	FILE *gdt;
public:
	trp2gdt(string fname_in, string fname_out){
		trp.open(fname_in.c_str(), ios_base::in);
		gdt = fopen(fname_out.c_str(), "w");
	}
	void start(){
		string line;
		fprintf(gdt, "gds2{600\nm = 2014 - 02 - 19 23:14 : 30 a = 2014 - 02 - 19 23 : 14 : 30\nlib 'Flat' 1 1e-09\n");
		fprintf(gdt, "cell{ c = 2014 - 02 - 19 23:14 : 30 m = 2014 - 02 - 19 23 : 14 : 30 'trp'\n");
		while (!trp.eof()){
			getline(trp, line);
			fprintf(gdt, "b{1 xy(");
			if ((int)line.at(line.size() - 1) > 57 || (int)line.at(line.size() - 1) <48)
				line.at(line.size() - 1) = '\0';
			fprintf(gdt, line.c_str());
			fprintf(gdt, ")}\n");
		}
		fprintf(gdt, "}\n}");
		fclose(gdt);
	}
	void layout2gdt(vector< vector<int> > layout){
		string line;
		fprintf(gdt, "gds2{600\nm = 2014 - 02 - 19 23:14 : 30 a = 2014 - 02 - 19 23 : 14 : 30\nlib 'Flat' 1 1e-09\n");
		fprintf(gdt, "cell{ c = 2014 - 02 - 19 23:14 : 30 m = 2014 - 02 - 19 23 : 14 : 30 'trp'\n");
		for (int i = 0; i < layout.size();i++){
			fprintf(gdt, "b{1 xy(");
			for (int j = 0; j < layout[i].size(); j++){
				fprintf(gdt, "%d",layout[i][j]);
				if (j != layout[i].size() - 1){
					fprintf(gdt, " ");
				}
			}
			fprintf(gdt, ")}\n");
		}
		fprintf(gdt, "}\n}");
		fclose(gdt);
	}
	void layout2gdt(vector< polygon > layout1, vector< polygon > layout2, int layer){
		string line;
		fprintf(gdt, "gds2{600\nm = 2014 - 02 - 19 23:14 : 30 a = 2014 - 02 - 19 23 : 14 : 30\nlib 'Flat' 1 1e-09\n");
		fprintf(gdt, "cell{ c = 2014 - 02 - 19 23:14 : 30 m = 2014 - 02 - 19 23 : 14 : 30 'trp'\n");
		for (int i = 0; i < layout1.size(); i++){
			fprintf(gdt, "b{%d xy(",layer-1);
			vector<boost_point> & points = layout1[i].outer();
			const int min[] = { static_cast<int>(get<0>(points[0])), static_cast<int>(get<1>(points[0])) };
			const int max[] = { static_cast<int>(get<0>(points[2])), static_cast<int>(get<1>(points[2])) };
			for (int j = 0; j < points.size() -1; j++){
				fprintf(gdt, "%d %d", static_cast<int>(get<0>(points[j])), static_cast<int>(get<1>(points[j])));
				if (j != points.size() - 1){
					fprintf(gdt, " ");
				}
			}
			fprintf(gdt, ")}\n");
		}
		for (int i = 0; i < layout2.size(); i++){
			fprintf(gdt, "b{%d xy(", layer);
			vector<boost_point> & points = layout2[i].outer();
			const int min[] = { static_cast<int>(get<0>(points[0])), static_cast<int>(get<1>(points[0])) };
			const int max[] = { static_cast<int>(get<0>(points[2])), static_cast<int>(get<1>(points[2])) };
			for (int j = 0; j < points.size() - 1; j++){
				fprintf(gdt, "%d %d", static_cast<int>(get<0>(points[j])), static_cast<int>(get<1>(points[j])));
				if (j != points.size() - 1){
					fprintf(gdt, " ");
				}
			}
			fprintf(gdt, ")}\n");
		}
		fprintf(gdt, "}\n}");
		fclose(gdt);
	}
	void layout2gdt(vector<vector< polygon > > layout1, vector< polygon > layout2, int layer){
		string line;
		fprintf(gdt, "gds2{600\nm = 2014 - 02 - 19 23:14 : 30 a = 2014 - 02 - 19 23 : 14 : 30\nlib 'Flat' 1 1e-09\n");
		fprintf(gdt, "cell{ c = 2014 - 02 - 19 23:14 : 30 m = 2014 - 02 - 19 23 : 14 : 30 'trp'\n");
		for (int i = 0; i < layout1.size(); i++){
			for (int k = 0; k < layout1[i].size(); k++){
				fprintf(gdt, "b{%d xy(", layer - 1);
				vector<boost_point> & points = layout1[i][k].outer();
				const int min[] = { static_cast<int>(get<0>(points[0])), static_cast<int>(get<1>(points[0])) };
				const int max[] = { static_cast<int>(get<0>(points[2])), static_cast<int>(get<1>(points[2])) };
				for (int j = 0; j < points.size() - 1; j++){
					fprintf(gdt, "%d %d", static_cast<int>(get<0>(points[j])), static_cast<int>(get<1>(points[j])));
					if (j != points.size() - 1){
						fprintf(gdt, " ");
					}
				}
			}
			fprintf(gdt, ")}\n");
		}
		for (int i = 0; i < layout2.size(); i++){
			fprintf(gdt, "b{%d xy(", layer);
			vector<boost_point> & points = layout2[i].outer();
			const int min[] = { static_cast<int>(get<0>(points[0])), static_cast<int>(get<1>(points[0])) };
			const int max[] = { static_cast<int>(get<0>(points[2])), static_cast<int>(get<1>(points[2])) };
			for (int j = 0; j < points.size() - 1; j++){
				fprintf(gdt, "%d %d", static_cast<int>(get<0>(points[j])), static_cast<int>(get<1>(points[j])));
				if (j != points.size() - 1){
					fprintf(gdt, " ");
				}
			}
			fprintf(gdt, ")}\n");
		}
		fprintf(gdt, "}\n}");
		fclose(gdt);
	}
	void centerlineDebug(vector< polygon> layout1){
		string line;
		fprintf(gdt, "gds2{600\nm = 2014 - 02 - 19 23:14 : 30 a = 2014 - 02 - 19 23 : 14 : 30\nlib 'Flat' 1 1e-09\n");
		fprintf(gdt, "cell{ c = 2014 - 02 - 19 23:14 : 30 m = 2014 - 02 - 19 23 : 14 : 30 'trp'\n");
		for (int i = 0; i < layout1.size(); i++){
			fprintf(gdt, "b{%d xy(", 1);
			vector<boost_point> & points = layout1[i].outer();
			const int min[] = { static_cast<int>(get<0>(points[0])), static_cast<int>(get<1>(points[0])) };
			const int max[] = { static_cast<int>(get<0>(points[2])), static_cast<int>(get<1>(points[2])) };
			for (int j = 0; j < points.size() - 1; j++){
				fprintf(gdt, "%d %d", static_cast<int>(get<0>(points[j])), static_cast<int>(get<1>(points[j])));
				if (j != points.size() - 1){
					fprintf(gdt, " ");
				}
			}
			fprintf(gdt, ")}\n");
		}
		fprintf(gdt, "}\n}");
		fclose(gdt);
	}
	void layout2gdt(vector<vector< polygon > > layout1){
		string line;
		fprintf(gdt, "gds2{600\nm = 2014 - 02 - 19 23:14 : 30 a = 2014 - 02 - 19 23 : 14 : 30\nlib 'Flat' 1 1e-09\n");
		fprintf(gdt, "cell{ c = 2014 - 02 - 19 23:14 : 30 m = 2014 - 02 - 19 23 : 14 : 30 'trp'\n");
		for (int i = 0; i < layout1.size(); i++){
			for (int k = 0; k < layout1[i].size(); k++){
				fprintf(gdt, "b{%d xy(", i + 1);
				vector<boost_point> & points = layout1[i][k].outer();
				const int min[] = { static_cast<int>(get<0>(points[0])), static_cast<int>(get<1>(points[0])) };
				const int max[] = { static_cast<int>(get<0>(points[2])), static_cast<int>(get<1>(points[2])) };
				for (int j = 0; j < points.size() - 1; j++){
					fprintf(gdt, "%d %d", static_cast<int>(get<0>(points[j])), static_cast<int>(get<1>(points[j])));
					if (j != points.size() - 1){
						fprintf(gdt, " ");
					}
				}
				fprintf(gdt, ")}\n");
			}
		}
		fprintf(gdt, "}\n}");
		fclose(gdt);
	}
};

void WriteFile(vector <polygon>inflate_layout, vector< polygon > intersection,string fileName){
	// write file
	trp2gdt t2g("", "new.gdt");
	t2g.layout2gdt(inflate_layout, intersection, 2);
	gdt2gds("new.gdt", fileName);
}

void WriteFile(vector<vector< polygon > >inflate_layout, string fileName){
	// write file
	trp2gdt t2g("", "new.gdt");
	t2g.layout2gdt(inflate_layout);
	gdt2gds("new.gdt", fileName);
}

void Clipper_addPoint(int x, int y, Path *path)
{
	IntPoint ip;
	ip.X = x;
	ip.Y = y;
	path->push_back(ip);
}

bool IsCounterclockwise(polygon poly){
	vector<boost_point> & points = poly.outer();
	int xmin = 99999999; int ymin = 99999999; int index = -1;
	for (int j = 0; j < points.size() - 1; j++){
		int x = static_cast<int>(get<0>(points[j])); int y = static_cast<int>(get<1>(points[j]));
		if (x <= xmin && y <= ymin){
			xmin = x; ymin = y; index = j;
		}
	}
	int x = static_cast<int>(get<0>(points[index + 1])); int y = static_cast<int>(get<1>(points[index + 1]));
	if (y > ymin){  // up
		// Clockwise
		return false;
	}
	else if (x > xmin){ // right
		// Counterclockwise
		return true;
	}
}

void Clipper_Union(vector<polygon> inflate_layout,vector<polygon> &union_output){

	Paths layout;
	union_output.resize(0);
	for (int i = 0; i < inflate_layout.size() ; i++){
		Path rectangle;
		vector<boost_point> & points = inflate_layout[i].outer();
		const int min[] = { static_cast<int>(get<0>(points[0])), static_cast<int>(get<1>(points[0])) };
		const int max[] = { static_cast<int>(get<0>(points[2])), static_cast<int>(get<1>(points[2])) };
		Clipper_addPoint(min[0], min[1], &rectangle);
		Clipper_addPoint(max[0], min[1], &rectangle);
		Clipper_addPoint(max[0], max[1], &rectangle);
		Clipper_addPoint(min[0], max[1], &rectangle);
		layout.push_back(rectangle);
	}

	Clipper c;
	Paths solution;
	c.AddPaths(layout, ptSubject,true);
	c.Execute(ctUnion, solution, pftNonZero, pftNonZero);

	for (unsigned i = 0; i< solution.size(); i++)
	{
		Path rec = solution.at(i);
		polygon tp;
		for (unsigned j = 0; j<rec.size(); j++)
		{
			IntPoint ip = rec.at(j);
			geom::append(geom::exterior_ring(tp), geom::make<point_type>(ip.X, ip.Y));
			//printf("%d = %lld, %lld\n", j, ip.X, ip.Y);
		}
		geom::append(geom::exterior_ring(tp), geom::make<point_type>(rec[0].X, rec[0].Y));
		if (IsCounterclockwise(tp))
			union_output.push_back(tp);
	}

}

int CAD_contest_fileInput(istream &file, vector<vector<polygon> > &RoutedShape, vector<vector<polygon> > &Obstacle, vector<via> &ViaList){
	string line;
	if (!file.eof()){
		getline(file, line);
		if (line.size() > 0 && line.at(0) != '#'){
			if (line.find("RoutedShape M") != string::npos){ // contain "RoutedShape M"
				int M_pos = line.find('M');
				string layer_num_s;
				layer_num_s.clear();
				int index_start_coor;
				for (index_start_coor = M_pos + 1; line.at(index_start_coor) != ' '; index_start_coor++){
					layer_num_s.push_back(line.at(index_start_coor));
				}
				int layer = atoi(layer_num_s.c_str());
				vector <int> coor;
				string number;
				for (int i = index_start_coor ; i < line.size(); i++){
					char c = line.at(i);
					if (c == '(' || c == ' ') continue;
					else if (c == ','){
						coor.push_back(atoi(number.c_str()));
						number.clear();
					}
					else if (c == ')'){
						coor.push_back(atoi(number.c_str()));
						number.clear();
					}
					else {
						number.push_back(c);
					}
				}
				int p[4][2] = { 0 };
				p[0][0] = coor.at(0);
				p[0][1] = coor.at(1);
				p[2][0] = coor.at(2);
				p[2][1] = coor.at(3);
				p[1][0] = coor.at(0);
				p[1][1] = coor.at(3);
				p[3][0] = coor.at(2);
				p[3][1] = coor.at(1);
				polygon tp;
				for (int j = 0; j < 4; j++){
					geom::append(geom::exterior_ring(tp), geom::make<point_type>(p[j][0], p[j][1]));
				}
				geom::append(geom::exterior_ring(tp), geom::make<point_type>(p[0][0], p[0][1]));
				RoutedShape[layer-1].push_back(tp);

				return 1;
			}
			else if (line.find("Obstacle M") != string::npos){ // contain "Obstacle M"
				int M_pos = line.find('M');
				string layer_num_s;
				layer_num_s.clear();
				int index_start_coor;
				for (index_start_coor = M_pos + 1; line.at(index_start_coor) != ' '; index_start_coor++){
					layer_num_s.push_back(line.at(index_start_coor));
				}
				int layer = atoi(layer_num_s.c_str());
				vector <int> coor;
				string number;
				for (int i = index_start_coor ; i < line.size(); i++){
					char c = line.at(i);
					if (c == '(' || c == ' ') continue;
					else if (c == ','){
						coor.push_back(atoi(number.c_str()));
						number.clear();
					}
					else if (c == ')'){
						coor.push_back(atoi(number.c_str()));
						number.clear();
					}
					else {
						number.push_back(c);
					}
				}
				int p[4][2] = { 0 };
				p[0][0] = coor.at(0);
				p[0][1] = coor.at(1);
				p[2][0] = coor.at(2);
				p[2][1] = coor.at(3);
				p[1][0] = coor.at(0);
				p[1][1] = coor.at(3);
				p[3][0] = coor.at(2);
				p[3][1] = coor.at(1);
				polygon tp;
				for (int j = 0; j < 4; j++){
					geom::append(geom::exterior_ring(tp), geom::make<point_type>(p[j][0], p[j][1]));
				}
				geom::append(geom::exterior_ring(tp), geom::make<point_type>(p[0][0], p[0][1]));
				Obstacle[layer-1].push_back(tp);
				return 2;
			}
			else if (line.find("RoutedVia V") != string::npos){ // contain "Obstacle M"
				int M_pos = line.find_last_of('V');
				string layer_num_s;
				layer_num_s.clear();
				int index_start_coor;
				for (index_start_coor = M_pos + 1; line.at(index_start_coor) != ' '; index_start_coor++){
					layer_num_s.push_back(line.at(index_start_coor));
				}
				int layer = atoi(layer_num_s.c_str());
				vector <int> coor;
				string number;
				for (int i = index_start_coor; i < line.size(); i++){
					char c = line.at(i);
					if (c == '(' || c == ' ') continue;
					else if (c == ','){
						coor.push_back(atoi(number.c_str()));
						number.clear();
					}
					else if (c == ')'){
						coor.push_back(atoi(number.c_str()));
						number.clear();
						break;
					}
					else {
						number.push_back(c);
					}
				}
				via via_temp;
				via_temp.x = coor[0];
				via_temp.y = coor[1];
				via_temp.layer = layer;
				cout << "via : "<< via_temp.x << "," << via_temp.y << endl;
				ViaList.push_back(via_temp);
				return 2;
			}
			else if (line.find("ViaCost") != string::npos){ // contain b{

				return 3;
			}
			else if (line.find("Spacing") != string::npos){ // stack pop
				return 4;
			}
			else if (line.find("Boundary") != string::npos){ // stack pop
				return 4;
			}
		}
		else { // #
			if (line.find("MetalLayers") != string::npos){ // stack pop
				int pos = line.find('=');
				string layer_num_s;
				layer_num_s.clear();
				for (int i = pos+1; i < line.size(); i++){
					if (line.at(i)!=' ')
						layer_num_s.push_back(line.at(i));
				}
				int layer_num = atoi(layer_num_s.c_str());
				RoutedShape.resize(layer_num);
				Obstacle.resize(layer_num);
				return 4;
			}
			return 0;
		}

	}
	else{  // EOF
		return -1;
	}

	return 0;
}

int main(){
	vector<vector<polygon> > RoutedShape;
	vector<vector<polygon> > Obstacle;
	vector<via> ViaList;
	//ObstacleRtree obstacle_tree;

	vector<vector<polygon> > gdslayout;

	vector<vector<polygon> > RoutedPolygon;
	vector<vector<polygon> > ObstaclePolygon;
	gdslayout.resize(0);
	RoutedPolygon.resize(0);
	ObstaclePolygon.resize(0);
	ifstream readfile;
	string fileName;
	cin >> fileName;
	readfile.open(fileName.c_str(), ios_base::in);
	int spacing = 0;
	int width = 0;
	int height = 0;
	// Input file of CAD Contest Loading
	while (CAD_contest_fileInput(readfile, RoutedShape, Obstacle, ViaList) != -1);
	int layer_num = RoutedShape.size();
	FILE *fp = fopen("layer1", "w");
	fprintf(fp, ".tech\n%d %d 0 0 0 0 0 0 %d %d\n.end\n", layer_num, spacing, width, height);
	fprintf(fp, ".net\n");
	int output_layer = 0;
	int poly_num = 0;
	cout << "Write" << endl;
	while (output_layer < layer_num){
		vector<polygon> Shape_union_output;
		vector<polygon> Obstacle_union_output;
		Clipper_Union(RoutedShape[output_layer], Shape_union_output);
		poly_num += Shape_union_output.size();
		//obstacle_tree.ObstacleRtreeConstruction(Obstacle[output_layer]);
		Clipper_Union(Obstacle[output_layer], Obstacle_union_output);
		/*for (int i = 0; i < Shape_union_output.size(); i++){
			vector<Segment> Segment_list;
			Nemo_segment_construct(Shape_union_output[i], Segment_list);
		}*/
		if (output_layer == 1){
			//WriteFile(Shape_union_output, RoutedShape[output_layer], "CAD.gds");
		}
		gdslayout.push_back(Shape_union_output);
		RoutedPolygon.push_back(Shape_union_output);
		gdslayout.push_back(Obstacle_union_output);
		ObstaclePolygon.push_back(Obstacle_union_output);
		output_layer++;
	}
	// .net file output
	for (int i = 0; i < poly_num; i++){
		if (i == 0){
			fprintf(fp, "0 net0 0 ");
		}
		else if (i == poly_num - 1){
			fprintf(fp, "%d\n",i);
		}
		else{
			fprintf(fp, "%d\n%d net0 %d ", i,i,i);
		}
	}
	fprintf(fp, ".end\n");
	fprintf(fp, ".poly\n");
	cout << "Write" << endl;
	int id = 0;
	for (int i = 0; i < RoutedPolygon.size(); i++){
		for (int j = 0; j < RoutedPolygon[i].size(); j++){
			//.id x y layer net nBorders
 			vector<Segment> Segment_list;
			//Nemo_segment_construct(RoutedPolygon[i][j], Segment_list);
			fprintf(fp, ".%d %d %d %d 0 %d\n", id, Segment_list[0].line_end[0].x, Segment_list[0].line_end[0].y, i, Segment_list.size());
			for (int list = 0; list < Segment_list.size(); list++){
				fprintf(fp, "%d %d %d %d %d\n", Segment_list[list].line_end[0].x, Segment_list[list].line_end[0].y, Segment_list[list].line_end[1].x, Segment_list[list].line_end[1].y, Segment_list[list].dir);
			}
			id++;
		}
	}
	for (int i = 0; i < ObstaclePolygon.size(); i++){
		for (int j = 0; j < ObstaclePolygon[i].size(); j++){
			//.id x y layer net nBorders
			vector<Segment> Segment_list;
			//Nemo_segment_construct(ObstaclePolygon[i][j], Segment_list);
			fprintf(fp, ".%d %d %d %d 1 %d\n", id, Segment_list[0].line_end[0].x, Segment_list[0].line_end[0].y, i, Segment_list.size());
			for (int list = 0; list < Segment_list.size(); list++){
				fprintf(fp, "%d %d %d %d %d\n", Segment_list[list].line_end[0].x, Segment_list[list].line_end[0].y, Segment_list[list].line_end[1].x, Segment_list[list].line_end[1].y, Segment_list[list].dir);
			}
			id++;
		}
	}
	cout << "Write" << endl;
	fprintf(fp, ".end\n");
	fprintf(fp, ".via\n");
	for (int i = 0; i < ViaList.size(); i++){
		fprintf(fp, "%d %d %d %d %d\n", i, ViaList[i].x, ViaList[i].y, ViaList[i].layer - 1, ViaList[i].layer,0);
	}
	fprintf(fp, ".end\n");
	WriteFile(gdslayout, "CAD_all_layer.gds");
}



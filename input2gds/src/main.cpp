#define GDS_NOT_UNION
//#define GDS_UNION

#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <time.h>
#include <fstream>
#include <deque>
#include <fstream>
#include <utility>
#include <math.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/foreach.hpp>
#include <boost/polygon/polygon.hpp>
#include <boost/polygon/polygon_set_data.hpp>
#include "clipper/clipper.hpp"
#include "../GDT-4.0.4/gdt2gds.h"
#include "Segment.cpp"

using namespace boost::polygon::operators;
using namespace std;
using namespace ClipperLib;
using boost::geometry::get;

namespace geom = boost::geometry;
namespace gtl = boost::polygon;

typedef geom::model::d2::point_xy<unsigned int> point_type;
typedef geom::model::polygon<point_type> polygon;
typedef boost::geometry::model::d2::point_xy<unsigned int> boost_point;

class via{
public :
	unsigned int x, y;
	unsigned int layer;
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
	void layout2gdt(vector< vector<unsigned int> > layout){
		string line;
		fprintf(gdt, "gds2{600\nm = 2014 - 02 - 19 23:14 : 30 a = 2014 - 02 - 19 23 : 14 : 30\nlib 'Flat' 1 1e-09\n");
		fprintf(gdt, "cell{ c = 2014 - 02 - 19 23:14 : 30 m = 2014 - 02 - 19 23 : 14 : 30 'trp'\n");
		for (unsigned int i = 0; i < layout.size();i++){
			fprintf(gdt, "b{1 xy(");
			for (unsigned int j = 0; j < layout[i].size(); j++){
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
		for (unsigned int i = 0; i < layout1.size(); i++){
			fprintf(gdt, "b{%d xy(",layer-1);
			vector<boost_point> & points = layout1[i].outer();
			const unsigned int min[] = { static_cast<unsigned int>(get<0>(points[0])), static_cast<unsigned int>(get<1>(points[0])) };
			const unsigned int max[] = { static_cast<unsigned int>(get<0>(points[2])), static_cast<unsigned int>(get<1>(points[2])) };
			for (unsigned int j = 0; j < points.size() -1; j++){
				fprintf(gdt, "%d %d", static_cast<unsigned int>(get<0>(points[j])), static_cast<unsigned int>(get<1>(points[j])));
				if (j != points.size() - 1){
					fprintf(gdt, " ");
				}
			}
			fprintf(gdt, ")}\n");
		}
		for (unsigned int i = 0; i < layout2.size(); i++){
			fprintf(gdt, "b{%d xy(", layer);
			vector<boost_point> & points = layout2[i].outer();
			const unsigned int min[] = { static_cast<unsigned int>(get<0>(points[0])), static_cast<unsigned int>(get<1>(points[0])) };
			const unsigned int max[] = { static_cast<unsigned int>(get<0>(points[2])), static_cast<unsigned int>(get<1>(points[2])) };
			for (unsigned int j = 0; j < points.size() - 1; j++){
				fprintf(gdt, "%d %d", static_cast<unsigned int>(get<0>(points[j])), static_cast<unsigned int>(get<1>(points[j])));
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
		for (unsigned int i = 0; i < layout1.size(); i++){
			for (unsigned int k = 0; k < layout1[i].size(); k++){
				fprintf(gdt, "b{%d xy(", layer - 1);
				vector<boost_point> & points = layout1[i][k].outer();
				const unsigned int min[] = { static_cast<unsigned int>(get<0>(points[0])), static_cast<unsigned int>(get<1>(points[0])) };
				const unsigned int max[] = { static_cast<unsigned int>(get<0>(points[2])), static_cast<unsigned int>(get<1>(points[2])) };
				for (unsigned int j = 0; j < points.size() - 1; j++){
					fprintf(gdt, "%d %d", static_cast<unsigned int>(get<0>(points[j])), static_cast<unsigned int>(get<1>(points[j])));
					if (j != points.size() - 1){
						fprintf(gdt, " ");
					}
				}
			}
			fprintf(gdt, ")}\n");
		}
		for (unsigned int i = 0; i < layout2.size(); i++){
			fprintf(gdt, "b{%d xy(", layer);
			vector<boost_point> & points = layout2[i].outer();
			const unsigned int min[] = { static_cast<unsigned int>(get<0>(points[0])), static_cast<unsigned int>(get<1>(points[0])) };
			const unsigned int max[] = { static_cast<unsigned int>(get<0>(points[2])), static_cast<unsigned int>(get<1>(points[2])) };
			for (unsigned int j = 0; j < points.size() - 1; j++){
				fprintf(gdt, "%d %d", static_cast<unsigned int>(get<0>(points[j])), static_cast<unsigned int>(get<1>(points[j])));
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
		for (unsigned int i = 0; i < layout1.size(); i++){
			fprintf(gdt, "b{%d xy(", 1);
			vector<boost_point> & points = layout1[i].outer();
			const unsigned int min[] = { static_cast<unsigned int>(get<0>(points[0])), static_cast<unsigned int>(get<1>(points[0])) };
			const unsigned int max[] = { static_cast<unsigned int>(get<0>(points[2])), static_cast<unsigned int>(get<1>(points[2])) };
			for (unsigned int j = 0; j < points.size() - 1; j++){
				fprintf(gdt, "%d %d", static_cast<unsigned int>(get<0>(points[j])), static_cast<unsigned int>(get<1>(points[j])));
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
		for (unsigned int i = 0; i < layout1.size(); i++){
			for (unsigned int k = 0; k < layout1[i].size(); k++){
				fprintf(gdt, "b{%d xy(", i + 1);
				vector<boost_point> & points = layout1[i][k].outer();
				const unsigned int min[] = { static_cast<unsigned int>(get<0>(points[0])), static_cast<unsigned int>(get<1>(points[0])) };
				const unsigned int max[] = { static_cast<unsigned int>(get<0>(points[2])), static_cast<unsigned int>(get<1>(points[2])) };
				for (unsigned int j = 0; j < points.size() - 1; j++){
					fprintf(gdt, "%d %d", static_cast<unsigned int>(get<0>(points[j])), static_cast<unsigned int>(get<1>(points[j])));
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

void Clipper_addPoint(unsigned int x, unsigned int y, Path *path)
{
	IntPoint ip;
	ip.X = x;
	ip.Y = y;
	path->push_back(ip);
}

bool IsCounterclockwise(polygon poly){
	vector<boost_point> & points = poly.outer();
	unsigned int xmin = UINT_MAX; unsigned int ymin = UINT_MAX; int index = -1;
	for (int j = 0; j < points.size() - 1; j++){
		unsigned int x = static_cast<unsigned int>(get<0>(points[j])); unsigned int y = static_cast<unsigned int>(get<1>(points[j]));
		if (x <= xmin && y <= ymin){
			xmin = x; ymin = y; index = j;
		}
	}
	unsigned int x = static_cast<unsigned int>(get<0>(points[index + 1])); unsigned int y = static_cast<unsigned int>(get<1>(points[index + 1]));
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
	for (unsigned int i = 0; i < inflate_layout.size() ; i++){
		Path rectangle;
		vector<boost_point> & points = inflate_layout[i].outer();
		const unsigned int min[] = { static_cast<unsigned int>(get<0>(points[0])), static_cast<unsigned int>(get<1>(points[0])) };
		const unsigned int max[] = { static_cast<unsigned int>(get<0>(points[2])), static_cast<unsigned int>(get<1>(points[2])) };
		Clipper_addPoint(min[0], min[1], &rectangle);
		Clipper_addPoint(max[0], min[1], &rectangle);
		Clipper_addPoint(max[0], max[1], &rectangle);
		Clipper_addPoint(min[0], max[1], &rectangle);
		layout.push_back(rectangle);
	}
	
#ifdef GDS_NOT_UNION
	for (unsigned i = 0; i< layout.size(); i++)
	{
		Path rec = layout.at(i);
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

#endif
	
	
#ifdef GDS_UNION

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
#endif
}

int ISPD2018_guide_fileInput(istream &file, vector<vector<polygon>> &RoutedShape, vector<pair<int, int>> &count_height, vector<pair<int, int>> &count_width, char *inputstr)
{
	int current_layer = 0, net_num = 10;

	string line;
	if (!file.eof()){
		getline(file, line);
		
		if (line.size() > 0 && line.compare(inputstr) == 0)
		{
			//cout << line << endl;
			
			string line_guide;
			current_layer++;
			getline(file, line_guide);
			while (line_guide.size() > 0)
			{
				getline(file, line_guide);//56608
				if (line_guide.size() > 0 && line_guide.at(0) == ')')
				{
					return -1;
				}

				if (line_guide.find(" ") != string::npos)
				{
					int M_pos = line_guide.find(' ');
					
					int index_start_coor;
					unsigned int _x[2], _y[2], layer_number ;

					string layer_num_s;
					layer_num_s.clear();
					for ( int ind = 0 ; ind < 4 ; ind++)
					{
						for (index_start_coor = 0; line_guide.at(index_start_coor) != ' ';)
						{
							layer_num_s.push_back(line_guide.at(index_start_coor));
							line_guide.erase(line_guide.begin());
						}

						line_guide.erase(line_guide.begin());

						if(ind == 0)
							_x[0] = atoi(layer_num_s.c_str());
						else if (ind == 1)
							_x[1] = atoi(layer_num_s.c_str());
						else if (ind == 2)
							_y[0] = atoi(layer_num_s.c_str());
						else if (ind == 3)
							_y[1] = atoi(layer_num_s.c_str());

						layer_num_s.clear();
					} // for

					layer_num_s.push_back(line_guide.at(line_guide.size()-1));
					layer_number = atoi(layer_num_s.c_str());

					int c_height, c_width;
					c_width = _y[0] - _x[0];
					c_height = _y[1] - _x[1];
					
					if (!count_height.empty())
					{
						bool found_same = false;
						for(int index3 = 0 ; index3 < count_height.size() ; index3++)
						{
							if(c_height % count_height.at(index3).first == 0)
							{
								count_height.at(index3).second++;
								found_same = true;
								break;
							}
						}
						if (!found_same)
						{
							count_height.push_back(make_pair(c_height, 1));
						}
					}
					else
					{
						count_height.push_back(make_pair(c_height, 1));
					}

					if (!count_width.empty())
					{
						bool found_same = false;
						for (int index3 = 0; index3 < count_width.size(); index3++)
						{
							if (c_width % count_width.at(index3).first == 0)
							{
								count_width.at(index3).second++;
								found_same = true;
								break;
							}
						}
						if (!found_same)
						{
							count_width.push_back(make_pair(c_width, 1));
						}
					}
					else
					{
						count_width.push_back(make_pair(c_width, 1));
					}

					cout << ">> " << _x[0] << " " << _x[1] << " " << _y[0] << " " << _y[1] << " " << layer_number << endl;

					unsigned int p[4][2] = {0};
					p[0][0] = _x[0];
					p[0][1] = _x[1];
					p[2][0] = _y[0];
					p[2][1] = _y[1];
					p[1][0] = _x[0];
					p[1][1] = _y[1];
					p[3][0] = _y[0];
					p[3][1] = _x[1];
					polygon tp;
					for (int j = 0; j < 4; j++)
					{
						geom::append(geom::exterior_ring(tp), geom::make<point_type>(p[j][0], p[j][1]));
					}
					geom::append(geom::exterior_ring(tp), geom::make<point_type>(p[0][0], p[0][1]));

					RoutedShape[layer_number-1].push_back(tp);
				}

			} // while

			return -1;
		} // if
		else { // #
		/*
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
				ViaList.resize(layer_num);
				return 4;
			}
			*/
			return 0;
		}

	}
	else{  // EOF
		return -1;
	}

	return 0;
}
polygon transfer_to_polygon (int x0, int x1, int y0, int y1) {
    unsigned int p[4][2] = {0};
    p[0][0] = x0;
    p[0][1] = x1;
    p[2][0] = y0;
    p[2][1] = y1;
    p[1][0] = x0;
    p[1][1] = y1;
    p[3][0] = y0;
    p[3][1] = x1;
    polygon tp;
    for (int j = 0; j < 4; j++)
    {
        geom::append(geom::exterior_ring(tp), geom::make<point_type>(p[j][0], p[j][1]));
    }
    geom::append(geom::exterior_ring(tp), geom::make<point_type>(p[0][0], p[0][1]));
    return tp;
}
int main(int argc, char *argv[]){

	cout << "start\n";
	if(argc < 2)
	{
		cout << "usage [gds_parser_input]\n";
		return 0;
	}

	vector<pair<int, int> > count_height, count_width;

	vector<vector<polygon> > Tracks;
	vector<vector<polygon> > Obstacle;
	vector<vector<polygon> > Pins;
    vector<vector<polygon> > RoutedShapes;
	vector<vector<polygon> > gdslayout;
	//vector<vector<polygon> > RoutedPolygon;
	//vector<vector<polygon> > ObstaclePolygon;
	gdslayout.resize(0);
	//RoutedPolygon.resize(0);
	//ObstaclePolygon.resize(0);

	ifstream readfile;
	string fileName(argv[1]);
	readfile.open(fileName.c_str(), ios_base::in);
	int spacing = 0;
	int width = 0;

	int height = 0;

	/*char inputstr[256];
	cin >> inputstr;
	// Input file of CAD Contest Loading
	RoutedShape.resize(10);
	while (ISPD2018_guide_fileInput(readfile, RoutedShape, count_height, count_width, inputstr) != -1)
		;
    */
	/*cout << "Kind of height count : " << count_height.size() << "\n";
	for (int index3 = 0; index3 < count_height.size(); index3++)
	{
		cout << "(" << count_height.at(index3).first << ") : " << count_height.at(index3).second << endl;
	}
	cout << "Kind of width count : " << count_width.size() << "\n";
	for (int index3 = 0; index3 < count_width.size(); index3++)
	{
		cout << "(" << count_width.at(index3).first << ") : " << count_width.at(index3).second << endl;
	}*/
    int layercnt = 0, trackcnt = 0, obscnt = 0, pincnt = 0, routecnt = 0;
    readfile >> layercnt;
    Tracks.resize(layercnt);
    Obstacle.resize(layercnt);
    Pins.resize(layercnt);
    RoutedShapes.resize(layercnt);
	int layer_num = layercnt;
	FILE *fp = fopen("layer1", "w");
	fprintf(fp, ".tech\n%d %d 0 0 0 0 0 0 %d %d\n.end\n", layer_num, spacing, width, height);
	fprintf(fp, ".net\n");
	int output_layer = 0;
	int poly_num = 0;
	cout << "Write" << endl;
	/*while (output_layer < layer_num){
		vector<polygon> Shape_union_output;
		//vector<polygon> Obstacle_union_output;
		//vector<polygon> Via_union_output;
		//Clipper_Union(RoutedShape[output_layer], Shape_union_output);
		poly_num += Shape_union_output.size();

		//Clipper_Union(Obstacle[output_layer], Obstacle_union_output);
		//Clipper_Union(ViaList[output_layer], Via_union_output);

		gdslayout.push_back(RoutedShape[output_layer]);
		RoutedPolygon.push_back(RoutedShape[output_layer]);
		//gdslayout.push_back(Obstacle_union_output);
		//ObstaclePolygon.push_back(Obstacle_union_output);
		//gdslayout.push_back(Via_union_output);
		//ObstaclePolygon.push_back(Via_union_output);
		output_layer++;
	}*/
    for (int i = 0; i < layercnt; i++) {
        readfile >> trackcnt;
        for (int j = 0; j < trackcnt; j++) {
            int x0, x1, y0, y1;
            readfile >> x0 >> x1 >> y0 >> y1;
            Tracks[i].push_back(transfer_to_polygon(x0, x1, y0, y1));
        }
        gdslayout.push_back(Tracks[i]);
        readfile >> obscnt;
        for (int j = 0; j < obscnt; j++) {
            int x0, x1, y0, y1;
            readfile >> x0 >> x1 >> y0 >> y1;
            Obstacle[i].push_back(transfer_to_polygon(x0, x1, y0, y1));
        }
        gdslayout.push_back(Obstacle[i]);
        readfile >> pincnt;
        for (int j = 0; j < pincnt; j++) {
            int x0, x1, y0, y1;
            readfile >> x0 >> x1 >> y0 >> y1;
            Pins[i].push_back(transfer_to_polygon(x0, x1, y0, y1));
        }
        gdslayout.push_back(Pins[i]);
        readfile >> routecnt;
        for (int j = 0; j < routecnt; j++) {
            int x0, x1, y0, y1;
            readfile >> x0 >> x1 >> y0 >> y1;
            RoutedShapes[i].push_back(transfer_to_polygon(x0, x1, y0, y1));
        }
        gdslayout.push_back(RoutedShapes[i]);
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
	
	cout << "Write" << endl;
	fprintf(fp, ".end\n");
	fprintf(fp, ".via\n");
	fprintf(fp, ".end\n");
	WriteFile(gdslayout, "CAD_all_layer.gds");
	
}



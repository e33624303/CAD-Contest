#define GDS_NOT_UNION
//#define GDS_UNION
#include "GuideParser.hpp"
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



int main(int argc, char *argv[]){

	cout << "start\n";
	if(argc < 4)
	{
		cout << "usage [lef] [def] [guide]\n";
		return 0;
	}

	vector<vector<polygon> > RoutedShape;

	vector<vector<polygon> > gdslayout;
	vector<vector<polygon> > RoutedPolygon;
	vector<vector<polygon> > ObstaclePolygon;
	gdslayout.resize(0);
	RoutedPolygon.resize(0);
	ObstaclePolygon.resize(0);

	ifstream readfile;
	string fileName(argv[3]);
	readfile.open(fileName.c_str(), ios_base::in);
	int spacing = 0;
	int width = 0;
	int height = 0;
	// Input file of CAD Contest Loading
	GuideParser GP;
	RoutedShape.resize(10);
	GP.ISPD2018_guide_fileInput(readfile);
	vector<GuideParser::rectangle> net_guide4 = GP.net_guide.at(0);
	vector<polygon> tempP;
	for(int a = 0 ; a < net_guide4.size() ; a++)
	{
		int p[4][2] = {0};
		p[0][0] = net_guide4.at(a).lbx;
		p[0][1] = net_guide4.at(a).lby;
		p[2][0] = net_guide4.at(a).rtx;
		p[2][1] = net_guide4.at(a).rty;
		p[1][0] = net_guide4.at(a).lbx;
		p[1][1] = net_guide4.at(a).rty;
		p[3][0] = net_guide4.at(a).rtx;
		p[3][1] = net_guide4.at(a).lby;

		polygon tp;
		tp.push_back(p);
		
	}

	// do something
	int layer_num = RoutedShape.size();
	FILE *fp = fopen("layer1", "w");
	fprintf(fp, ".tech\n%d %d 0 0 0 0 0 0 %d %d\n.end\n", layer_num, spacing, width, height);
	fprintf(fp, ".net\n");
	int output_layer = 0;
	int poly_num = 0;
	cout << "Write" << endl;
	while (output_layer < layer_num){
		vector<polygon> Shape_union_output;
		poly_num += Shape_union_output.size();

		gdslayout.push_back(RoutedShape[output_layer]);
		RoutedPolygon.push_back(RoutedShape[output_layer]);
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
	
	cout << "Write" << endl;
	fprintf(fp, ".end\n");
	fprintf(fp, ".via\n");
	fprintf(fp, ".end\n");
	WriteFile(gdslayout, "CAD_all_layer.gds");
	
	
}



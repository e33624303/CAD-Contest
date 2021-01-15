struct coor {
    int x ;
    int y ;
} ;

class Segment{
public:
	coor line_end[2];
	int dir;  // 0:U , 1:D , 2:L , 3:R
	Segment(){
		;
	}
	Segment(int x1, int y1, int x2, int y2){
		line_end[0].x = x1;
		line_end[0].y = y1;
		line_end[1].x = x2;
		line_end[1].y = y2;
	}
	void coor_set(int x1, int y1, int x2, int y2){
		line_end[0].x = x1;
		line_end[0].y = y1;
		line_end[1].x = x2;
		line_end[1].y = y2;
	}
};

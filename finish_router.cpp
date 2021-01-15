#include "router_Astar.h"
#include <time.h>
#define DIS_PARAMETER 1.5
#define GDS_WIDTH 0
#define RUN_PARA 1.3
bool f_one_bus_router (All_route &all, GridMap &G, BUSES &B, MainInput &M, BUSES_block &B_block, int bus_id ) {
    BUSES_route bus_tmp,bus_tmp2,bus_tmp3;
    bus_tmp.name = B.name;
    bus_tmp.bit_routes.resize(B.PinBitSet.size());
    bus_tmp.via.clear();
	bus_tmp2.via.clear();
    for (int i = 0; i < bus_tmp.bit_routes.size(); i++)
        bus_tmp.bit_routes[i].name = B.PinBitSet[i].name;

    int turn=0;
    priority_queue<via_type , vector<via_type> , compare> via_queue;
    vector<bool> result;
    for(int k=0 ; k<B.PinBitSet[0].PinShapeSet.size()-1 ; k++){
        bool start_orientation;
        bool end_orientation;
        if (k == 0) {
            if (B.PinBitSet[0].PinShapeSet[0].Pin.LB.x == B.PinBitSet[B.PinBitSet.size()-1].PinShapeSet[0].Pin.LB.x) start_orientation = 1;
            else start_orientation = 0;
        }
        if (B.PinBitSet[0].PinShapeSet[k+1].Pin.LB.x == B.PinBitSet[B.PinBitSet.size()-1].PinShapeSet[k+1].Pin.LB.x) end_orientation = 1;
        else end_orientation = 0;
        int time_1, time_2;
        bool time_out = 0;
        time_1 = time(NULL);
        route_workspace R;
        stack<vector<Grid_node> > solution_turn;
        vector<Grid_node> start_node;
        vector<Grid_node> via_node;
        vector<Grid_node> end_node;
        int via_number=0;
        if(k >= 1 && bus_tmp.via.size() == 0){
            //cout<<"empty via \n";
			bus_tmp3.bit_routes.clear();
			bus_tmp3.via.clear();
			all.path[bus_id] = bus_tmp;
            return 0;
        }
        for(int i=0 ; i<B.PinBitSet.size() ; i++){
            if(k == 0){
                for(int l=0 ; l<2 ; l++){
                    if(l == 0){
                        PINSHAPES pintmp = B.PinBitSet[i].PinShapeSet[0];
                        pair<int, int> start_pin;
                        start_pin.first = (pintmp.Pin.LB.x + pintmp.Pin.RT.x)/2;
                        start_pin.second = (pintmp.Pin.LB.y + pintmp.Pin.RT.y)/2;
                        int layer = pintmp.LayerNum;
                        int orientation = M.LayerSet[layer].Orientation;
                        Grid_node node_tmp;
                        node_tmp.Layer = layer;
                        vector<Grid_node> possi_point;
                        for (int j = 0; j < G.GridMapSet[layer].track.size(); j++) {
                            if (orientation == 1) {
                                if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.x) {
                                    node_tmp.grid_index.first = j;
                                    //break;
                                    for (int j2 = 0; j2 < G.GridMapSet[layer].track[j].gridpoint.size(); j2++) {
                                        if (G.GridMapSet[layer].track[j].gridpoint[j2].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].gridpoint[j2].coord <= pintmp.Pin.RT.y && G.GridMapSet[layer].track[j].gridpoint[j2].type == 0) {
                                            node_tmp.grid_index.second = j2;
                                            possi_point.push_back(node_tmp);
                                        }
                                    }
                                }
                                
                            }
                            else {
                                if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.y) {
                                    node_tmp.grid_index.first = j;
                                    //break;
                                    for (int j2 = 0; j2 < G.GridMapSet[layer].track[j].gridpoint.size(); j2++) {
                                        if (G.GridMapSet[layer].track[j].gridpoint[j2].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].gridpoint[j2].coord <= pintmp.Pin.RT.x && G.GridMapSet[layer].track[j].gridpoint[j2].type == 0) {
                                            node_tmp.grid_index.second = j2;
                                            possi_point.push_back(node_tmp);
                                        }
                                    }
                                }
                            }
                        }
                        if (possi_point.size() == 0)  {
                            cout << "invalid pins!\n";
                            return 0;
                        }
                        int min_start = cal_pair_manhattan(get_coordinate(possi_point[0], G, M), start_pin);
                        Grid_node final_start = possi_point[0];
                        for (int j = 1; j < possi_point.size(); j++) {
                            int tmp_min = cal_pair_manhattan(get_coordinate(possi_point[j], G, M), start_pin);
                            if (tmp_min < min_start) {
                                final_start = possi_point[j];
                                min_start = tmp_min;
                            }
                        }
                        start_node.push_back(final_start);
                    }
                    else if(l == 1){
                        PINSHAPES pintmp = B.PinBitSet[i].PinShapeSet[1];
                        int layer = pintmp.LayerNum;
                        int orientation = M.LayerSet[layer].Orientation;
                        pair<int, int> end_pin;
                        end_pin.first = (pintmp.Pin.LB.x + pintmp.Pin.RT.x)/2;
                        end_pin.second = (pintmp.Pin.LB.y + pintmp.Pin.RT.y)/2;
                        vector<Grid_node> possi_point;
                        Grid_node node_tmp;
                        node_tmp.Layer = layer;
                        for (int j = 0; j < G.GridMapSet[layer].track.size(); j++) {
                            if (orientation == 1) {
                                if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.x) {
                                    node_tmp.grid_index.first = j;
                                    //break;
                                    for (int j2 = 0; j2 < G.GridMapSet[layer].track[j].gridpoint.size(); j2++) {
                                        if (G.GridMapSet[layer].track[j].gridpoint[j2].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].gridpoint[j2].coord <= pintmp.Pin.RT.y && G.GridMapSet[layer].track[j].gridpoint[j2].type == 0) {
                                            node_tmp.grid_index.second = j2;
                                            possi_point.push_back(node_tmp);
                                        }
                                    }
                                }
                            }
                            else {
                                if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.y) {
                                    node_tmp.grid_index.first = j;
                                    //break;
                                    for (int j2 = 0; j2 < G.GridMapSet[layer].track[j].gridpoint.size(); j2++) {
                                        if (G.GridMapSet[layer].track[j].gridpoint[j2].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].gridpoint[j2].coord <= pintmp.Pin.RT.x && G.GridMapSet[layer].track[j].gridpoint[j2].type == 0) {
                                            node_tmp.grid_index.second = j2;
                                            possi_point.push_back(node_tmp);
                                        }
                                    }
                                }
                            }
                        }
                        if (possi_point.size() == 0)  {
                            cout << "invalid pins!\n";
                            return 0;
                        }
                        int min_end = cal_pair_manhattan(get_coordinate(possi_point[0], G, M), end_pin);
                        Grid_node final_end = possi_point[0];
                        for (int j = 1; j < possi_point.size(); j++) {
                            int tmp_min = cal_pair_manhattan(get_coordinate(possi_point[j], G, M), end_pin);
                            if (tmp_min < min_end) {
                                final_end = possi_point[j];
                                min_end = tmp_min;
                            }
                        }
                        end_node.push_back(final_end);
                    }
                }
            }
            else{
                for(int l=0 ; l<2 ; l++){
                    if(l == 0){
                        if(i==0)
                            bus_tmp2 = bus_tmp;
                        //PINSHAPES pintmp = B.PinBitSet[i].PinShapeSet[0];
                        //bus_tmp.via[via_number];
                        int tmp;
                        //tmp=cal_real_manhattan(bus_tmp.via[0][0] , B.PinBitSet[i].PinShapeSet[k+1].Pin.LB.x , B.PinBitSet[i].PinShapeSet[k+1].Pin.LB.y);
                        if(i==0 && turn==0){
                            via_queue=priority_queue<via_type , vector<via_type> , compare>();
                            for(int j=0 ; j<bus_tmp.via.size() ; j++){
                                if(bus_tmp.via[j][0].via_use == false){
                                    int cal=cal_real_manhattan(bus_tmp.via[j][0] , B.PinBitSet[i].PinShapeSet[k+1].Pin.LB.x , B.PinBitSet[i].PinShapeSet[k+1].Pin.LB.y);
                                    via_type via_tmp;
                                    via_tmp.via_number=j;
                                    via_tmp.priority=cal;
                                    via_queue.push(via_tmp);                                   
                                }
                            }
                        }
                        via_number=via_queue.top().via_number;
                        /*for(int j = 0 ; j < bus_tmp.via[via_number].size() ; j++){
                            bus_tmp.via[via_number][j].via_use=true;
                        }*/
                        int layer = bus_tmp.via[via_number][0].Layer;
                        int orientation = M.LayerSet[layer].Orientation;
                        
                        if(orientation == 0 && i==0){
                            int first;
                            int second;
                            bool left;
                            for(int j=0 ; j<G.GridMapSet[layer].track.size() ; j++){
                                if(G.GridMapSet[layer].track[j].coord >= bus_tmp.via[via_number][0].grid_index.second && G.GridMapSet[layer].track[j].coord <= bus_tmp.via[via_number][0].grid_index.second){
                                    first = j;
                                    break;
                                }
                            }
                            for(int j=0 ; j<G.GridMapSet[layer].track[first].gridpoint.size(); j++){
                                if (G.GridMapSet[layer].track[first].gridpoint[j].coord >= bus_tmp.via[via_number][0].grid_index.first && G.GridMapSet[layer].track[first].gridpoint[j].coord <= bus_tmp.via[via_number][0].grid_index.first) {
                                    second = j;
                                    break;
                                }
                            }
                            
                            int neighbor_via=0;
                            bool find=false;
                            /*if(via_number != bus_tmp.via.size()-1){
                                if(bus_tmp.via[via_number+1][0].grid_index.second == bus_tmp.via[via_number][0].grid_index.second){
                                    cout<<"into 1\n";
                                    neighbor_via=via_number+1;
                                }
                            }
                            if(via_number != 0){
                                if(bus_tmp.via[via_number-1][0].grid_index.second == bus_tmp.via[via_number][0].grid_index.second){
                                    cout<<"into 2\n";
                                    neighbor_via=via_number-1;
                                }
                            }*/
                            for(int j=0 ; j<bus_tmp.via.size() ; j++){
                                if(j == via_number)
                                    continue;
                                if(bus_tmp.via[j][0].grid_index.second == bus_tmp.via[via_number][0].grid_index.second){
                                    find=true;
                                    cout<<"find\n";
                                    neighbor_via=j;
                                    break;
                                }
                            }
                            if(!find){
                                for(int j=0 ; j<=k ;j++){
                                    for(int l=0 ; l<B.PinBitSet.size() ; l++){
                                        if(bus_tmp.via[j][0].grid_index.second <= B.PinBitSet[l].PinShapeSet[j].Pin.RT.y && bus_tmp.via[j][0].grid_index.first >= B.PinBitSet[l].PinShapeSet[j].Pin.LB.y){
                                            cout<<"find by pin\n";
                                            if(bus_tmp.via[j][0].grid_index.first < B.PinBitSet[l].PinShapeSet[j].Pin.LB.x)
                                                left=true;
                                            else
                                                left=false;
                                            break;
                                        }
                                    }
                                }
                            }
                            else{
                                if(bus_tmp.via[via_number][0].grid_index.first < bus_tmp.via[neighbor_via][0].grid_index.first)
                                    left=true;
                                else
                                    left=false;
                            }
                            cout<<bus_tmp.via[via_number][0].grid_index.first<<" , "<<bus_tmp.via[via_number][0].grid_index.second<<endl;
                            cout<<bus_tmp.via[neighbor_via][0].grid_index.first<<" , "<<bus_tmp.via[neighbor_via][0].grid_index.second<<endl;
                            /*cout<<G.GridMapSet[layer].track[first].gridpoint[second].type<<endl;
                            cout<<G.GridMapSet[layer].track[first].gridpoint[second-2].coord<<" , "<<G.GridMapSet[layer].track[first].coord<<endl;
                            cout<<G.GridMapSet[layer].track[first].gridpoint[second-2].type<<endl;
                            cout<<G.GridMapSet[layer].track[first].gridpoint[second+2].coord<<" , "<<G.GridMapSet[layer].track[first].coord<<endl;
                            cout<<G.GridMapSet[layer].track[first].gridpoint[second+2].type<<endl;*/
                            /*if(G.GridMapSet[layer].track[first].gridpoint[second+1].type!=0)
                                left=true;
                            else
                                left=false;*/
                            /*if(G.GridMapSet[layer].track[first].gridpoint[second-1].type!=0)
                                left=false;
                            else
                                left=true;
                            if(turn!=0)
                                left=true;*/
                            
                            if(left){
                                int judge_coord;
                                if(bus_tmp.via[via_number][0].grid_index.first > bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.first){
                                    judge_coord=bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.first;
                                    //cout<<"now left and coord is "<<bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.first<<endl;
                                }
                                else{
                                    judge_coord=bus_tmp.via[via_number][0].grid_index.first;
                                    //cout<<"now left and coord is "<<bus_tmp.via[via_number][0].grid_index.first<<endl;
                                }

                                for(int j=0 ; j<bus_tmp2.via[via_number].size() ; j++){
                                    bus_tmp2.via[via_number][j].grid_index.first=judge_coord;
                                }
                            }
                            else{
                                int judge_coord;
                                if(bus_tmp.via[via_number][0].grid_index.first < bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.first){
                                    judge_coord=bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.first;
                                    //cout<<"now right and coord is "<<bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.first<<endl;
                                }
                                else{
                                    judge_coord=bus_tmp.via[via_number][0].grid_index.first;
                                    //cout<<"now right and coord is "<<bus_tmp.via[via_number][0].grid_index.first<<endl;
                                }
                                for(int j=0 ; j<bus_tmp2.via[via_number].size() ; j++){
                                    bus_tmp2.via[via_number][j].grid_index.first=judge_coord;
                                }
                            }
                        }
                        else if(orientation==1 && i==0){
                            int first;
                            int second;
                            bool down;
                            for(int j=0 ; j<G.GridMapSet[layer].track.size() ; j++){
                                if(G.GridMapSet[layer].track[j].coord >= bus_tmp.via[via_number][0].grid_index.first && G.GridMapSet[layer].track[j].coord <= bus_tmp.via[via_number][0].grid_index.first){
                                    first = j;
                                    break;
                                }
                            }
                            for(int j=0 ; j<G.GridMapSet[layer].track[first].gridpoint.size(); j++){
                                if (G.GridMapSet[layer].track[first].gridpoint[j].coord >= bus_tmp.via[via_number][0].grid_index.second && G.GridMapSet[layer].track[first].gridpoint[j].coord <= bus_tmp.via[via_number][0].grid_index.second) {
                                    second = j;
                                    break;
                                }
                            }
                            int neighbor_via=0;
                            bool find=false;
                            /*if(via_number != bus_tmp.via.size()-1){
                                if(bus_tmp.via[via_number+1][0].grid_index.first == bus_tmp.via[via_number][0].grid_index.first){
                                    cout<<"into 1\n";
                                    neighbor_via=via_number+1;
                                }
                            }
                            if(via_number != 0){
                                if(bus_tmp.via[via_number-1][0].grid_index.first == bus_tmp.via[via_number][0].grid_index.first){
                                    cout<<"into 2\n";
                                    neighbor_via=via_number-1;
                                }
                            }*/
                            for(int j=0 ; j<bus_tmp.via.size() ; j++){
                                if(j == via_number)
                                    continue;
                                if(bus_tmp.via[j][0].grid_index.first == bus_tmp.via[via_number][0].grid_index.first){
                                    //cout<<"find\n";
                                    neighbor_via=j;
                                    find=true;
                                    break;
                                }
                            }
                            if(!find){
                                for(int j=0 ; j<=k ;j++){
                                    for(int l=0 ; l<B.PinBitSet.size() ; l++){
                                        if(bus_tmp.via[j][0].grid_index.first <= B.PinBitSet[l].PinShapeSet[j].Pin.RT.x && bus_tmp.via[j][0].grid_index.first >= B.PinBitSet[l].PinShapeSet[j].Pin.LB.x){
                                            //cout<<"find by pin\n";
                                            if(bus_tmp.via[j][0].grid_index.second < B.PinBitSet[l].PinShapeSet[j].Pin.LB.y)
                                                down=true;
                                            else
                                                down=false;
                                            break;
                                        }
                                    }
                                }
                            }
                            else{
                                if(bus_tmp.via[via_number][0].grid_index.second < bus_tmp.via[neighbor_via][0].grid_index.second)
                                    down=true;
                                else
                                    down=false;
                            }
                            cout<<bus_tmp.via[via_number][0].grid_index.first<<" , "<<bus_tmp.via[via_number][0].grid_index.second<<endl;
                            cout<<bus_tmp.via[neighbor_via][0].grid_index.first<<" , "<<bus_tmp.via[neighbor_via][0].grid_index.second<<endl;
                            /*cout<<G.GridMapSet[layer].track[first].gridpoint[second].type<<endl;
                            cout<<G.GridMapSet[layer].track[first].coord<<" , "<<G.GridMapSet[layer].track[first].gridpoint[second-1].coord<<endl;
                            cout<<G.GridMapSet[layer].track[first].gridpoint[second-1].type<<endl;
                            cout<<G.GridMapSet[layer].track[first].coord<<" , "<<G.GridMapSet[layer].track[first].gridpoint[second+1].coord<<endl;
                            cout<<G.GridMapSet[layer].track[first].gridpoint[second+1].type<<endl;*/
                            /*if(G.GridMapSet[layer].track[first].gridpoint[second+1].type!=0)
                                down=true;
                            else
                                down=false;*/
                            if(down){
                                int judge_coord;
                                if(bus_tmp.via[via_number][0].grid_index.second < bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.second){
                                    judge_coord=bus_tmp.via[via_number][0].grid_index.second;
                                    //cout<<"now down and coord is "<<bus_tmp.via[via_number][0].grid_index.second<<endl;
                                }
                                else{
                                    judge_coord=bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.second;
                                    //cout<<"now down and coord is "<<bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.second<<endl;
                                }
                                for(int j=0 ; j<bus_tmp2.via[via_number].size() ; j++)
                                    bus_tmp2.via[via_number][j].grid_index.second=judge_coord;
                            }
                            else{
                                int judge_coord;
                                if(bus_tmp.via[via_number][0].grid_index.second > bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.second){
                                    judge_coord=bus_tmp.via[via_number][0].grid_index.second;
                                    //cout<<"now top and coord is "<<bus_tmp.via[via_number][0].grid_index.second<<endl;
                                }
                                else{
                                    judge_coord=bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.second;
                                    //cout<<"now top and coord is "<<bus_tmp.via[via_number][bus_tmp.via[via_number].size()-1].grid_index.second<<endl;
                                }
                                for(int j=0 ; j<bus_tmp2.via[via_number].size() ; j++)
                                    bus_tmp2.via[via_number][j].grid_index.second=judge_coord; 
                            }
                        }
                        if(i==0){
                            /*cout<<"bus_tmp\n";
                                for(int p=0 ; p<bus_tmp.via[via_number].size() ; p++){
                                    cout<<"via["<<via_number<<"] ";
                                    cout<<p<<" bit : "<<bus_tmp.via[via_number][p].grid_index.first<<" , "<<bus_tmp.via[via_number][p].grid_index.second<<endl;
                                }
                            cout<<"\nbus_tmp2\n";
                                for(int p=0 ; p<bus_tmp2.via[via_number].size() ; p++){
                                    cout<<"via["<<via_number<<"] ";
                                    cout<<p<<" bit : "<<bus_tmp2.via[via_number][p].grid_index.first<<" , "<<bus_tmp2.via[via_number][p].grid_index.second<<endl;
                                }*/
                        }

                        Grid_node node_tmp,node_tmp2;
                        node_tmp.Layer=layer;
                        node_tmp2.Layer=layer;
                        for(int j=0 ; j<G.GridMapSet[layer].track.size() ; j++){
                            if(orientation == 1){
                                if(G.GridMapSet[layer].track[j].coord >= bus_tmp2.via[via_number][i].grid_index.first && G.GridMapSet[layer].track[j].coord <= bus_tmp2.via[via_number][i].grid_index.first){
                                    node_tmp.grid_index.first = j;
                                    break;
                                }
                            }
                            else{
                                if(G.GridMapSet[layer].track[j].coord >= bus_tmp2.via[via_number][i].grid_index.second && G.GridMapSet[layer].track[j].coord <= bus_tmp2.via[via_number][i].grid_index.second){
                                    node_tmp.grid_index.first = j;
                                    break;
                                }
                            }
                        }
                        for(int j=0 ; j<G.GridMapSet[layer].track.size() ; j++){
                            if(orientation == 1){
                                if(G.GridMapSet[layer].track[j].coord >= bus_tmp.via[via_number][i].grid_index.first && G.GridMapSet[layer].track[j].coord <= bus_tmp.via[via_number][i].grid_index.first){
                                    node_tmp2.grid_index.first = j;
                                    break;
                                }
                            }
                            else{
                                if(G.GridMapSet[layer].track[j].coord >= bus_tmp.via[via_number][i].grid_index.second && G.GridMapSet[layer].track[j].coord <= bus_tmp.via[via_number][i].grid_index.second){
                                    node_tmp2.grid_index.first = j;
                                    break;
                                }
                            }
                        }
                        for(int j=0 ; j<G.GridMapSet[layer].track[node_tmp.grid_index.first].gridpoint.size(); j++){
                            if(orientation == 1 ){
                                if (G.GridMapSet[layer].track[node_tmp.grid_index.first].gridpoint[j].coord >= bus_tmp2.via[via_number][i].grid_index.second && G.GridMapSet[layer].track[node_tmp.grid_index.first].gridpoint[j].coord <= bus_tmp2.via[via_number][i].grid_index.second) {
                                    node_tmp.grid_index.second = j;
                                    break;
                                }
                            }
                            else{
                                if (G.GridMapSet[layer].track[node_tmp.grid_index.first].gridpoint[j].coord >= bus_tmp2.via[via_number][i].grid_index.first && G.GridMapSet[layer].track[node_tmp.grid_index.first].gridpoint[j].coord <= bus_tmp2.via[via_number][i].grid_index.first) {
                                    node_tmp.grid_index.second = j;
                                    break;
                                }
                            }
                        }
                        for(int j=0 ; j<G.GridMapSet[layer].track[node_tmp2.grid_index.first].gridpoint.size(); j++){
                            if(orientation == 1 ){
                                if (G.GridMapSet[layer].track[node_tmp2.grid_index.first].gridpoint[j].coord >= bus_tmp.via[via_number][i].grid_index.second && G.GridMapSet[layer].track[node_tmp2.grid_index.first].gridpoint[j].coord <= bus_tmp.via[via_number][i].grid_index.second) {
                                    node_tmp2.grid_index.second = j;
                                    break;
                                }
                            }
                            else{
                                if (G.GridMapSet[layer].track[node_tmp2.grid_index.first].gridpoint[j].coord >= bus_tmp.via[via_number][i].grid_index.first && G.GridMapSet[layer].track[node_tmp2.grid_index.first].gridpoint[j].coord <= bus_tmp.via[via_number][i].grid_index.first) {
                                    node_tmp2.grid_index.second = j;
                                    break;
                                }
                            }
                        }
                        G.GridMapSet[layer].track[node_tmp.grid_index.first].gridpoint[node_tmp.grid_index.second].type=0;
                        G.GridMapSet[layer].track[node_tmp2.grid_index.first].gridpoint[node_tmp2.grid_index.second].type=0;
                        /*node_tmp.Layer = layer;
                        for (int j = 0; j < G.GridMapSet[layer].track.size(); j++) {
                            if (orientation == 1) {
                                if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.x) {
                                    node_tmp.grid_index.first = j;
                                    break;
                                }
                            }
                            else {
                                if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.y) {
                                    node_tmp.grid_index.first = j;
                                    break;
                                }
                            }
                        }
                        for (int j = 0; j < G.GridMapSet[layer].track[node_tmp.grid_index.first].gridpoint.size(); j++) {
                            if (orientation == 1) {
                                if (G.GridMapSet[layer].track[node_tmp.grid_index.first].gridpoint[j].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[node_tmp.grid_index.first].gridpoint[j].coord <= pintmp.Pin.RT.y) {
                                    node_tmp.grid_index.second = j;
                                    break;
                                }
                                
                            }
                            else {
                                if (G.GridMapSet[layer].track[node_tmp.grid_index.first].gridpoint[j].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[node_tmp.grid_index.first].gridpoint[j].coord <= pintmp.Pin.RT.x) {
                                    node_tmp.grid_index.second = j;
                                    break;
                                }
                            }
                        }*/
                        /*cout<<"now node_tmp\n";
                        cout<<node_tmp.grid_index.first<<"  "<<node_tmp.grid_index.second<<endl;
                        cout<<"now node_tmp2\n";
                        cout<<node_tmp2.grid_index.first<<"  "<<node_tmp2.grid_index.second<<endl;*/
                        via_node.push_back(node_tmp2);
                        start_node.push_back(node_tmp);
                        //cerr<<bus_tmp.via.size()<<endl;
                        //start_node=bus_tmp.via[via_number];
                    }
                    else if(l == 1){
                        PINSHAPES pintmp = B.PinBitSet[i].PinShapeSet[k+1];
                        int layer = pintmp.LayerNum;
                        int orientation = M.LayerSet[layer].Orientation;
                        pair<int, int> end_pin;
                        end_pin.first = (pintmp.Pin.LB.x + pintmp.Pin.RT.x)/2;
                        end_pin.second = (pintmp.Pin.LB.y + pintmp.Pin.RT.y)/2;
                        vector<Grid_node> possi_point;
                        Grid_node node_tmp;
                        node_tmp.Layer = layer;
                        for (int j = 0; j < G.GridMapSet[layer].track.size(); j++) {
                            if (orientation == 1) {
                                if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.x) {
                                    node_tmp.grid_index.first = j;
                                    //break;
                                    for (int j2 = 0; j2 < G.GridMapSet[layer].track[j].gridpoint.size(); j2++) {
                                        if (G.GridMapSet[layer].track[j].gridpoint[j2].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].gridpoint[j2].coord <= pintmp.Pin.RT.y && G.GridMapSet[layer].track[j].gridpoint[j2].type == 0) {
                                            node_tmp.grid_index.second = j2;
                                            possi_point.push_back(node_tmp);
                                        }
                                    }
                                }
                            }
                            else {
                                if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.y) {
                                    node_tmp.grid_index.first = j;
                                    //break;
                                    for (int j2 = 0; j2 < G.GridMapSet[layer].track[j].gridpoint.size(); j2++) {
                                        if (G.GridMapSet[layer].track[j].gridpoint[j2].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].gridpoint[j2].coord <= pintmp.Pin.RT.x && G.GridMapSet[layer].track[j].gridpoint[j2].type == 0) {
                                            node_tmp.grid_index.second = j2;
                                            possi_point.push_back(node_tmp);
                                        }
                                    }
                                }
                            }
                        }
                        if (possi_point.size() == 0)  {
                            cout << "invalid pins(end)!\n";
                            return 0;
                        }
                        int min_end = cal_pair_manhattan(get_coordinate(possi_point[0], G, M), end_pin);
                        Grid_node final_end = possi_point[0];
                        for (int j = 1; j < possi_point.size(); j++) {
                            int tmp_min = cal_pair_manhattan(get_coordinate(possi_point[j], G, M), end_pin);
                            if (tmp_min < min_end) {
                                final_end = possi_point[j];
                                min_end = tmp_min;
                            }
                        }
                        end_node.push_back(final_end);
                    }
                }
            }
        }
        //change invalid start
        if (k == 0) {
            for (int i1 = 0; i1 < 2; i1++) {
                bool invalid_start = 0;
                if (start_orientation == 1) {
                    int tmp_first = get_coordinate(start_node[0], G, M).first;
                    for (int i = 0; i < start_node.size(); i++)
                        if (tmp_first != get_coordinate(start_node[i], G, M).first) {
                            invalid_start = 1;
                            break;
                        }
                }
                else {
                    int tmp_second = get_coordinate(start_node[0], G, M).second;
                    for (int i = 0; i < start_node.size(); i++)
                        if (tmp_second != get_coordinate(start_node[i], G, M).second) {
                            invalid_start = 1;
                            break;
                        }
                }
                if (invalid_start) {
                    for(int i=0 ; i<B.PinBitSet.size() ; i++){
                        PINSHAPES pintmp = B.PinBitSet[i].PinShapeSet[0];
                        pair<int, int> start_pin;
                        if (i1 == 0) {
                            if (start_orientation == 1) {
                                start_pin.first = pintmp.Pin.LB.x;
                                start_pin.second = (pintmp.Pin.LB.y + pintmp.Pin.RT.y)/2;
                            }
                            else {
                                start_pin.first = (pintmp.Pin.LB.x + pintmp.Pin.RT.x)/2;
                                start_pin.second = pintmp.Pin.LB.y;
                            }
                        }
                        else {
                            if (start_orientation == 1) {
                                start_pin.first = pintmp.Pin.RT.x;
                                start_pin.second = (pintmp.Pin.LB.y + pintmp.Pin.RT.y)/2;
                            }
                            else {
                                start_pin.first = (pintmp.Pin.LB.x + pintmp.Pin.RT.x)/2;
                                start_pin.second = pintmp.Pin.RT.y;
                            }
                        }
                        int layer = pintmp.LayerNum;
                        int orientation = M.LayerSet[layer].Orientation;
                        Grid_node node_tmp;
                        node_tmp.Layer = layer;
                        vector<Grid_node> possi_point;
                        for (int j = 0; j < G.GridMapSet[layer].track.size(); j++) {
                        if (orientation == 1) {
                            if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.x) {
                                node_tmp.grid_index.first = j;
                                    //break;
                                for (int j2 = 0; j2 < G.GridMapSet[layer].track[j].gridpoint.size(); j2++) {
                                    if (G.GridMapSet[layer].track[j].gridpoint[j2].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].gridpoint[j2].coord <= pintmp.Pin.RT.y && G.GridMapSet[layer].track[j].gridpoint[j2].type == 0) {
                                        node_tmp.grid_index.second = j2;
                                        possi_point.push_back(node_tmp);
                                    }
                                }
                            }    
                        }
                        else {
                             if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.y) {
                                node_tmp.grid_index.first = j;
                                    //break;
                                for (int j2 = 0; j2 < G.GridMapSet[layer].track[j].gridpoint.size(); j2++) {
                                    if (G.GridMapSet[layer].track[j].gridpoint[j2].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].gridpoint[j2].coord <= pintmp.Pin.RT.x && G.GridMapSet[layer].track[j].gridpoint[j2].type == 0) {
                                        node_tmp.grid_index.second = j2;
                                            possi_point.push_back(node_tmp);
                                        }
                                    }
                                }
                            }
                        }
                        if (possi_point.size() == 0)  {
                            cout << "invalid pins!\n";
                            for(int j=0 ; j<B.PinBitSet[0].PinShapeSet.size() ; j++)
                                result.push_back(0);
                            all.bus_result.push_back(result);
                            return 0;
                        }
                        int min_start = cal_pair_manhattan(get_coordinate(possi_point[0], G, M), start_pin);
                        Grid_node final_start = possi_point[0];
                        for (int j = 1; j < possi_point.size(); j++) {
                            int tmp_min = cal_pair_manhattan(get_coordinate(possi_point[j], G, M), start_pin);
                            if (tmp_min < min_start) {
                                final_start = possi_point[j];
                                min_start = tmp_min;
                            }
                        }
                        start_node[i] = final_start;
                    }
                }
            }
        }
        //end change invalid start
        // change invalid end
        for (int i1 = 0; i1 < 2; i1++) {
            bool invalid_end = 0;
            if (end_orientation == 1) {
                int tmp_first = get_coordinate(end_node[0], G, M).first;
                for (int i = 0; i < end_node.size(); i++)
                    if (tmp_first != get_coordinate(end_node[i], G, M).first) {
                        invalid_end = 1;
                        break;
                    }
            }
            else {
                int tmp_second = get_coordinate(end_node[0], G, M).second;
                for (int i = 0; i < end_node.size(); i++)
                    if (tmp_second != get_coordinate(end_node[i], G, M).second) {
                        invalid_end = 1;
                        break;
                    }
            }
            if (invalid_end) {
                for(int i=0 ; i<B.PinBitSet.size() ; i++){
                    PINSHAPES pintmp = B.PinBitSet[i].PinShapeSet[k+1];
                    pair<int, int> end_pin;
                    if (i1 == 0) {
                        if (end_orientation == 1) {
                            end_pin.first = pintmp.Pin.LB.x;
                            end_pin.second = (pintmp.Pin.LB.y + pintmp.Pin.RT.y)/2;
                        }
                        else {
                            end_pin.first = (pintmp.Pin.LB.x + pintmp.Pin.RT.x)/2;
                            end_pin.second = pintmp.Pin.LB.y;
                        }
                    }
                    else {
                        if (end_orientation == 1) {
                            end_pin.first = pintmp.Pin.RT.x;
                            end_pin.second = (pintmp.Pin.LB.y + pintmp.Pin.RT.y)/2;
                        }
                        else {
                            end_pin.first = (pintmp.Pin.LB.x + pintmp.Pin.RT.x)/2;
                            end_pin.second = pintmp.Pin.RT.y;
                        }
                    }
                    int layer = pintmp.LayerNum;
                    int orientation = M.LayerSet[layer].Orientation;
                    Grid_node node_tmp;
                    node_tmp.Layer = layer;
                    vector<Grid_node> possi_point;
                    for (int j = 0; j < G.GridMapSet[layer].track.size(); j++) {
                    if (orientation == 1) {
                        if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.x) {
                            node_tmp.grid_index.first = j;
                                //break;
                            for (int j2 = 0; j2 < G.GridMapSet[layer].track[j].gridpoint.size(); j2++) {
                                if (G.GridMapSet[layer].track[j].gridpoint[j2].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].gridpoint[j2].coord <= pintmp.Pin.RT.y && G.GridMapSet[layer].track[j].gridpoint[j2].type == 0) {
                                    node_tmp.grid_index.second = j2;
                                    possi_point.push_back(node_tmp);
                                }
                            }
                        }    
                    }
                    else {
                         if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.y) {
                            node_tmp.grid_index.first = j;
                                //break;
                            for (int j2 = 0; j2 < G.GridMapSet[layer].track[j].gridpoint.size(); j2++) {
                                if (G.GridMapSet[layer].track[j].gridpoint[j2].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].gridpoint[j2].coord <= pintmp.Pin.RT.x && G.GridMapSet[layer].track[j].gridpoint[j2].type == 0) {
                                    node_tmp.grid_index.second = j2;
                                        possi_point.push_back(node_tmp);
                                    }
                                }
                            }
                        }
                    }
                    if (possi_point.size() == 0)  {
                        cout << "invalid pins!\n";
                        for(int j=0 ; j<B.PinBitSet[0].PinShapeSet.size() ; j++)
                            result.push_back(0);
                        all.bus_result.push_back(result);
                        return 0;
                    }
                    int min_end = cal_pair_manhattan(get_coordinate(possi_point[0], G, M), end_pin);
                    Grid_node final_end = possi_point[0];
                    for (int j = 1; j < possi_point.size(); j++) {
                        int tmp_min = cal_pair_manhattan(get_coordinate(possi_point[j], G, M), end_pin);
                        if (tmp_min < min_end) {
                            final_end = possi_point[j];
                            min_end = tmp_min;
                        }
                    }
                    end_node[i] = final_end;
                }
            }
        }
        //end change invalid end

        int start_layer;
        if(k==0)
            start_layer = B.PinBitSet[0].PinShapeSet[0].LayerNum;
        else
            start_layer = bus_tmp.via[via_number][0].Layer;
        int end_layer = B.PinBitSet[0].PinShapeSet[k+1].LayerNum;
        //cout<<"ddddd here\n";
        
        bool start_violation = 0;
        int start_via_layer;
    
        if (k == 0 && start_node[0].grid_index.first == start_node[1].grid_index.first)
            start_violation = 1;
        if (start_violation) {
            // block start_pin
            for (int i = 0; i < B.PinBitSet.size(); i++) {
                PINSHAPES pintmp = B.PinBitSet[i].PinShapeSet[0];
                int layer = pintmp.LayerNum;
                int orientation = M.LayerSet[layer].Orientation;
                for (int j = 0; j < G.GridMapSet[layer].track.size(); j++) {
                    if (orientation == 1) {
                        if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.x) {
                            for (int ii = 0; ii < G.GridMapSet[layer].track[j].gridpoint.size(); ii++) {
                                if (G.GridMapSet[layer].track[j].gridpoint[ii].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].gridpoint[ii].coord <= pintmp.Pin.RT.y) {
                                    G.GridMapSet[layer].track[j].gridpoint[ii].type = 3;
                                }
                            }
                        }
                    }
                    else {
                        if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.y) {
                            for (int ii = 0; ii < G.GridMapSet[layer].track[j].gridpoint.size(); ii++) {
                                if (G.GridMapSet[layer].track[j].gridpoint[ii].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].gridpoint[ii].coord <= pintmp.Pin.RT.x) {
                                    G.GridMapSet[layer].track[j].gridpoint[ii].type = 3;
                                }
                            }
                        }
                    }
                }
            }
            //
            int next_start = start_layer;
            for (int i = 1; i < M.LayerSet.size(); i++)  // go lower layer
                if (start_layer-i >= 0 && M.LayerSet[start_layer-i].Orientation != M.LayerSet[start_layer].Orientation) {
                    next_start = start_layer - i;
                    break;
                }
            if (next_start == start_layer) {
                for (int i = 1; i < M.LayerSet.size(); i++)  // go upper layer
                    if (start_layer+i < M.LayerSet.size() && M.LayerSet[start_layer+i].Orientation != M.LayerSet[start_layer].Orientation) {
                        next_start = start_layer + i;
                        break;
                    }
            }
            if (next_start < start_layer) start_via_layer = next_start;
            else start_via_layer = start_layer;
            start_layer = next_start;
            for (int i = 0; i < end_node.size(); i++) {
                Grid_node tmp_start = start_node[i];
                change_layer(tmp_start, start_node[i], start_layer, G, M);
            }
        }
        
        Turn_queue tq_up, tq_down;
        tq_up.up_down = 0;
        tq_up.line_cost = 0;
        tq_down.up_down = 1;
        tq_down.line_cost = 0;
        //cout<<"a layer is "<<start_node[0].Layer<<"  b layer is "<<end_node[0].Layer<<endl;
        for (int i = 0; i < start_node.size(); i++) {
            //cout << "start " << i;
            //print_coordinate(start_node[i], G, M);
            start_node[i].manhattan = cal_manhattan(start_node[i], end_node[end_node.size()/2], G, M);
            tq_up.Turn_point.push(start_node[i]);
            tq_down.Turn_point.push(start_node[i]);
        }
        //cout<<"eat \n";
        tq_up.cost = tq_up.line_cost+cal_manhattan(start_node[start_node.size()/2], end_node[end_node.size()/2], G, M)*DIS_PARAMETER;
        tq_down.cost = tq_up.line_cost+cal_manhattan(start_node[start_node.size()/2], end_node[end_node.size()/2], G, M)*DIS_PARAMETER;
        tq_up.spv_cnt = 0;
        tq_down.spv_cnt = 0;
        tq_up.guide_coord = get_coordinate(start_node[start_node.size()/2], G, M);
        tq_down.guide_coord = get_coordinate(start_node[start_node.size()/2], G, M);
        R.Astar_queue.push(tq_up);
        R.Astar_queue.push(tq_down);
        // handle start_node orientation violation
        
        
        // handle end_node orientation violation
        bool end_violation = 0;
        
        int end_via_layer = 0;
        if (end_node[0].grid_index.first == end_node[1].grid_index.first)
            end_violation = 1;
        if (end_violation) {
            //block endpin
            for (int i = 0; i < B.PinBitSet.size(); i++) {
                PINSHAPES pintmp = B.PinBitSet[i].PinShapeSet[k+1];
                int layer = pintmp.LayerNum;
                int orientation = M.LayerSet[layer].Orientation;
                for (int j = 0; j < G.GridMapSet[layer].track.size(); j++) {
                    if (orientation == 1) {
                        if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.x) {
                            for (int ii = 0; ii < G.GridMapSet[layer].track[j].gridpoint.size(); ii++) {
                                if (G.GridMapSet[layer].track[j].gridpoint[ii].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].gridpoint[ii].coord <= pintmp.Pin.RT.y) {
                                    G.GridMapSet[layer].track[j].gridpoint[ii].type = 3;
                                }
                            }
                        }
                    }
                    else {
                        if (G.GridMapSet[layer].track[j].coord >= pintmp.Pin.LB.y && G.GridMapSet[layer].track[j].coord <= pintmp.Pin.RT.y) {
                            for (int ii = 0; ii < G.GridMapSet[layer].track[j].gridpoint.size(); ii++) {
                                if (G.GridMapSet[layer].track[j].gridpoint[ii].coord >= pintmp.Pin.LB.x && G.GridMapSet[layer].track[j].gridpoint[ii].coord <= pintmp.Pin.RT.x) {
                                    G.GridMapSet[layer].track[j].gridpoint[ii].type = 3;
                                }
                            }
                        }
                    }
                }
            }
            //
            int next_end = end_layer;
            for (int i = 1; i < M.LayerSet.size(); i++)  // go lower layer
                if (end_layer-i >= 0 && M.LayerSet[end_layer-i].Orientation != M.LayerSet[end_layer].Orientation) {
                    next_end = end_layer - i;
                    break;
                }
            if (next_end == end_layer) {
                for (int i = 1; i < M.LayerSet.size(); i++)  // go upper layer
                    if (end_layer+i < M.LayerSet.size() && M.LayerSet[end_layer+i].Orientation != M.LayerSet[end_layer].Orientation) {
                        next_end = end_layer + i;
                        break;
                    }
            }
            if (next_end < end_layer) end_via_layer = next_end;
            else end_via_layer = end_layer;
            end_layer = next_end;
            for (int i = 0; i < end_node.size(); i++) {
                Grid_node tmp_end = end_node[i];
                change_layer(tmp_end, end_node[i], end_layer, G, M);
            }
        }
        // end of handle end_node orientation violation
        /*for (int i = 0; i < start_node.size(); i++) {
            cout << "Start_node[" << i << "]: ";
            print_coordinate(start_node[i], G, M);
        }*/
        /*cout << "end_node.size() = " << end_node.size() << endl;
        for (int i = 0; i < end_node.size(); i++) {
            cout << "end_node[" << i << "]: ";
            print_coordinate(end_node[i], G, M);
        }*/
        solution_turn.push(start_node);
        //cerr << "finish start and end\n";
        // each iteration has one priority queue, find one direction
        bool find_the_solution = 0;
        Turn_queue T_solution;
        int iter_cnt = 0;
        int current_layer;
        
        if (k == 0) {
            block_pin_activate(B_block, bus_id, 0);
        }
        block_pin_activate(B_block, bus_id, k+1);
        //cout<<"debug here\n";
        while (!find_the_solution && !R.Astar_queue.empty() && !time_out) {
            Turn_queue T = R.Astar_queue.top();
            R.Astar_queue.pop();
            vector<Grid_node> GN_origin(B.PinBitSet.size());
            vector<Grid_node> start_tmp;
            bool empty_queue = 0;
            if (T.Turn_point.size() >= GN_origin.size()) {
                for (int i = 0; i < GN_origin.size(); i++) {
                    if (!T.Turn_point.empty()) {
                        bool find_start = 0;
                        int or_tmp = M.LayerSet[T.Turn_point.top().Layer].Orientation;
                        while (!find_start && !T.Turn_point.empty()) {
                            bool violate_start = 0;
                            if (or_tmp == 1) {
                                for (int j = 0; j < i; j++)
                                    if (abs(get_coordinate(T.Turn_point.top(), G, M).first - get_coordinate(GN_origin[j], G, M).first) < (B.Width[T.Turn_point.top().Layer]/2)+M.LayerSet[T.Turn_point.top().Layer].Width) {
                                        violate_start = 1;
                                        break;
                                    }
                            }
                            else {
                                for (int j = 0; j < i; j++)
                                    if (abs(get_coordinate(T.Turn_point.top(), G, M).second-get_coordinate(GN_origin[j], G, M).second) < (B.Width[T.Turn_point.top().Layer]/2)+M.LayerSet[T.Turn_point.top().Layer].Width) {
                                        violate_start = 1;
                                        break;
                                    }
                            }
                            if (violate_start == 0) {
                                find_start = 1;
                            }
                            else {
                                start_tmp.push_back(T.Turn_point.top());
                                T.Turn_point.pop();
                            }
                        }
                        if (find_start) {
                            GN_origin[i] = T.Turn_point.top();
                            T.Turn_point.pop();
                        }
                        else {
                            empty_queue = 1;
                            break;
                        }
                    }
                    else {
                        empty_queue = 1;
                        break;
                    }
                }
                for (int i = 0; i < start_tmp.size(); i++)
                    T.Turn_point.push(start_tmp[i]);
            }
            else empty_queue = 1;
            // empty, go back
            if (empty_queue == 1)  {
                iter_cnt++;
                continue;
            }
            current_layer = GN_origin[0].Layer;
            
            //cerr << "finish pop\n";
            sort(GN_origin.begin(), GN_origin.end());
            int line_end = GN_origin[0].grid_index.second;
            //code_change
            //cout << "test ( " << T.guide_coord.first << ", " << T.guide_coord.second << " )\n";
            //cout << "test size: " << T.Turn_point.size() << endl;
           // cout << "test pre_turnpoints:" << T.pre_turnpoints.size() << endl;
            pair<int, int> tmp_guide = get_coordinate(GN_origin[GN_origin.size()/2], G, M);
            if (M.LayerSet[current_layer].Orientation == 1) {
                tmp_guide.second = T.guide_coord.second;
                if (T.pre_turnpoints.size() == 0) {
                    T.cost = cal_pair_manhattan(tmp_guide, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER;
                    T.guide_coord = tmp_guide;
                }
                else {
                    if (T.last_dir == 0)
                        T.line_cost += tmp_guide.first-T.guide_coord.first;
                    else
                        T.line_cost -= tmp_guide.first-T.guide_coord.first;
                    T.guide_coord = tmp_guide;
                    T.cost = cal_pair_manhattan(T.guide_coord, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER + T.line_cost;
                }
            }
            else  {
                tmp_guide.first = T.guide_coord.first;
                if (T.pre_turnpoints.size() == 0) {
                    T.cost = cal_pair_manhattan(tmp_guide, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER;
                    T.guide_coord = tmp_guide;
                }
                else {
                    if (T.last_dir == 0)
                        T.line_cost += tmp_guide.second-T.guide_coord.second;
                    else
                        T.line_cost -= tmp_guide.second-T.guide_coord.second;
                    T.guide_coord = tmp_guide;
                    T.cost = cal_pair_manhattan(T.guide_coord, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER + T.line_cost;
                }
            }
            //end_change
            //cout << "before run\n";
            while (!find_the_solution && !time_out) {
                time_2 = time(NULL);
                if (time_2 - time_1 > M.bus_run_time) {
                    cerr<<"timeout\n";
                    time_out = 1;
                    break;
                }
                /*if (bus_id == 1 || bus_id == 5) {
                for (int i = 0; i < GN_origin.size(); i++) {
                    cout << "GN_origin[" << i << "] = ";
                    print_coordinate(GN_origin[i], G, M);
                }
                cout << "current layer = " << current_layer << endl;
                if (T.up_down == 0) cout << "GO up\n";
                else cout << "GO down\n";
                cout << "guide_coord: ( " << T.guide_coord.first << ", " << T.guide_coord.second << " )\n";
                cout << "cost = " << T.cost << ", line_cost = " << T.line_cost << endl;
                cout << "Turn: " << T.pre_turnpoints.size() << endl;
                int pause_;
                cin >> pause_;
                }*/
                vector<Grid_node> GN = GN_origin;
                bool tri_flag = 0;
                
                if (T.triangle_turns.size() > 0) {
                    if (T.triangle_turns[T.triangle_turns.size()-1].idx == T.pre_turnpoints.size()) tri_flag = 1;
                }
                if (T.up_down == 0 && !T.pre_turnpoints.empty() && !tri_flag) {
                    Grid_node tmp_node, tmp_node2;
                    change_layer(GN[0], tmp_node, T.pre_turnpoints[T.pre_turnpoints.size()-1][0].Layer , G, M);
                    tmp_node.grid_index.first = T.pre_turnpoints[T.pre_turnpoints.size()-1][0].grid_index.first;
                    change_layer(tmp_node, tmp_node2, current_layer, G, M);
                    for (int i = 0; i < GN.size(); i++)
                        GN[i].grid_index.second = tmp_node2.grid_index.second;
                }
                else if (!T.pre_turnpoints.empty() && !tri_flag) {
                    Grid_node tmp_node, tmp_node2;
                    change_layer(GN[0], tmp_node, T.pre_turnpoints[T.pre_turnpoints.size()-1][0].Layer , G, M);
                    tmp_node.grid_index.first = T.pre_turnpoints[T.pre_turnpoints.size()-1][GN_origin.size()-1].grid_index.first;
                    change_layer(tmp_node, tmp_node2, current_layer, G, M);
                    for (int i = 0; i < GN.size(); i++)
                        GN[i].grid_index.second = tmp_node2.grid_index.second;
                }
                Grid_node Origin_end = GN[0];
                bool obstacle_occur = 0;
                bool partial_solution = 0;
                bool not_right_way = 0;
                bool zero_seg = 0;
                int first_obs_line = 0;
                vector<int> obstacle_occur_node;
                vector<bool> run_1_node(GN_origin.size(), 0);
                if (T.pre_turnpoints.size() >= 2) {
                    for (int i = 0; i < GN_origin.size(); i++) {
                        if (G.GridMapSet[current_layer].track[GN_origin[i].grid_index.first].coord == G.GridMapSet[T.pre_turnpoints[T.pre_turnpoints.size()-2][i].Layer].track[T.pre_turnpoints[T.pre_turnpoints.size()-2][i].grid_index.first].coord) {
                            obstacle_occur_node.push_back(i);
                        }
                    }
                    if (obstacle_occur_node.size() > 0) zero_seg = 1;
                } 
                if (T.up_down == 0 && !zero_seg) { // GO UP
                    while (!obstacle_occur) {
                        /*if (k == 1) {
                        cout << "GN[0].first = " << GN[0].grid_index.first << ", GN[0].second = " << GN[0].grid_index.second << endl;
                        print_coordinate(GN[0], G, M);
                        cout << "GN[1].first = " << GN[1].grid_index.first << ", GN[1].second = " << GN[1].grid_index.second << endl;
                        print_coordinate(GN[1], G, M);
                        cout << "GN[2].first = " << GN[2].grid_index.first << ", GN[2].second = " << GN[2].grid_index.second << endl;
                        print_coordinate(GN[2], G, M);
                        cout << "GN[3].first = " << GN[3].grid_index.first << ", GN[3].second = " << GN[3].grid_index.second << endl;
                        print_coordinate(GN[3], G, M);
                        cout << "current layer = " << current_layer << endl;
                        cout << "next:\n";
                        cout << "GN[0].first = " << GN[0].grid_index.first << ", GN[0].second = " << GN[0].grid_index.second+1 << endl;
                        cout << "GN[1].first = " << GN[1].grid_index.first << ", GN[1].second = " << GN[1].grid_index.second+1 << endl;
                        cout << "GN[2].first = " << GN[2].grid_index.first << ", GN[2].second = " << GN[2].grid_index.second+1 << endl;
                        cout << "GN[3].first = " << GN[3].grid_index.first << ", GN[3].second = " << GN[3].grid_index.second+1 << endl;
                        cout << "type = " << G.GridMapSet[current_layer].track[GN[0].grid_index.first].gridpoint[GN[0].grid_index.second+1].type << G.GridMapSet[current_layer].track[GN[1].grid_index.first].gridpoint[GN[1].grid_index.second+1].type << G.GridMapSet[current_layer].track[GN[2].grid_index.first].gridpoint[GN[2].grid_index.second+1].type << G.GridMapSet[current_layer].track[GN[3].grid_index.first].gridpoint[GN[3].grid_index.second+1].type << endl;
                        cout << "current layer = " << current_layer << endl;
                        }*/
                        int correspond_cnt = 0;
                        for (int i = 0; i < GN.size(); i++)
                            for (int j = 0; j < end_node.size(); j++) {
                                if (end_node[j].Layer == GN[i].Layer && end_node[j].grid_index == GN[i].grid_index) {
                                    correspond_cnt++;
                                    //cout << "find one!!\n";
                                    break;
                                }
                                if (j == end_node.size()-1)
                                    obstacle_occur_node.push_back(i);
                            }
                        if (correspond_cnt == end_node.size()) {
                            if (M.LayerSet[current_layer].Orientation == 1 && T.pre_turnpoints.size() != 0) {
                                if (get_coordinate(T.pre_turnpoints[T.pre_turnpoints.size()-1][GN.size()-1], G, M).second >= get_coordinate(GN[0], G, M).second) {
                                    not_right_way = 1;
                                    break;
                                }
                            }
                            else if (T.pre_turnpoints.size() != 0) {
                                if (get_coordinate(T.pre_turnpoints[T.pre_turnpoints.size()-1][GN.size()-1], G, M).first >= get_coordinate(GN[0], G, M).first) {
                                    not_right_way = 1;
                                    break;
                                }
                            }
                            find_the_solution = 1;
                            T.pre_turnpoints.push_back(GN_origin);
                            T_solution = T;
                            for (int i = 0; i < run_1_node.size(); i++) if (run_1_node[i]) T_solution.spv_cnt++;
                            // cout << "solution find !!\n";
                            break;
                        }
                        else if (correspond_cnt > 0) {
                            if (M.LayerSet[current_layer].Orientation == 1 && T.pre_turnpoints.size() != 0) {
                                if (get_coordinate(T.pre_turnpoints[T.pre_turnpoints.size()-1][GN.size()-1], G, M).second >= get_coordinate(GN[0], G, M).second) {
                                    not_right_way = 1;
                                    break;
                                }
                            }
                            else if (T.pre_turnpoints.size() != 0) {
                                if (get_coordinate(T.pre_turnpoints[T.pre_turnpoints.size()-1][GN.size()-1], G, M).first >= get_coordinate(GN[0], G, M).first) {
                                    not_right_way = 1;
                                    break;
                                }
                            }
                            partial_solution = 1;
                            //cout << "tune the solution!!\n";
                            break;
                        }
                        else obstacle_occur_node.clear();
                        line_end = GN[0].grid_index.second;
                        //int pause_;
                        //cin >> pause_;
                        for (int i = 0; i < GN.size(); i++) {
                            Map_grid map_grid;
                            map_grid.Layer = current_layer;
                            map_grid.grid_index = GN[i].grid_index;
                            map_grid.grid_index.second++;
                            if (GN[i].grid_index.second+1 == G.GridMapSet[current_layer].track[GN[i].grid_index.first].gridpoint.size()) {
                                obstacle_occur_node.push_back(i);
                              //  cout << "boundary\n";
                            }
                            else if (G.GridMapSet[current_layer].track[GN[i].grid_index.first].gridpoint[GN[i].grid_index.second+1].type == 0 && T.path_check.find(map_grid) == T.path_check.end()) {
                                GN[i].grid_index.second++;
                            //    cout << "clear\n";
                            }
                            else if (G.GridMapSet[current_layer].track[GN[i].grid_index.first].gridpoint[GN[i].grid_index.second+1].type == 1 && T.path_check.find(map_grid) == T.path_check.end()) {
                                GN[i].grid_index.second++;
                                run_1_node[i] = 1;
                            //    cout << "clear\n";
                            }
                            else {
                                //cout << "(" << current_layer << ", " << GN[i].grid_index.first << ", " << GN[i].grid_index.second+1 << ") ";
                                //cout << "type = " << G.GridMapSet[current_layer].track[GN[i].grid_index.first].gridpoint[GN[i].grid_index.second+1].type << endl;
                                //cout << "obastacle maybe\n";
                                first_obs_line = i;
                                obstacle_occur_node.push_back(i);
                            }
                        }
                        if (obstacle_occur_node.size() != 0) {
                            obstacle_occur = 1;
                        }
                    }
                    /*if (bus_id == 1 || bus_id == 5) {
                        for (int i = 0; i < GN.size(); i++) {
                            cout << "GN_[" << i << "] = ";
                            print_coordinate(GN[i], G, M);
                        }
                    }
                    cerr << "finish go up\n";*/
                }
                else if (!zero_seg) { // GO DOWN
                    while (!obstacle_occur) {
                        /*if (k == 1) {
                        cout << "GN[0].first = " << GN[0].grid_index.first << ", GN[0].second = " << GN[0].grid_index.second << endl;
                        print_coordinate(GN[0], G, M);
                        cout << "GN[1].first = " << GN[1].grid_index.first << ", GN[1].second = " << GN[1].grid_index.second << endl;
                        print_coordinate(GN[1], G, M);
                        cout << "GN[2].first = " << GN[2].grid_index.first << ", GN[2].second = " << GN[2].grid_index.second << endl;
                        print_coordinate(GN[2], G, M);
                        cout << "current layer = " << current_layer << endl;
                        cout << "next:\n";
                        cout << "GN[0].first = " << GN[0].grid_index.first << ", GN[0].second = " << GN[0].grid_index.second-1 << endl;
                        cout << "GN[1].first = " << GN[1].grid_index.first << ", GN[1].second = " << GN[1].grid_index.second-1 << endl;
                        cout << "GN[2].first = " << GN[2].grid_index.first << ", GN[2].second = " << GN[2].grid_index.second-1 << endl;
                        cout << "type = " << G.GridMapSet[current_layer].track[GN[0].grid_index.first].gridpoint[GN[0].grid_index.second-1].type << endl;
                        cout << "current layer = " << current_layer << endl;
                        }*/
                        int correspond_cnt = 0;
                        for (int i = 0; i < GN.size(); i++)
                            for (int j = 0; j < end_node.size(); j++) {
                                if (end_node[j].Layer == GN[i].Layer && end_node[j].grid_index == GN[i].grid_index) {
                                    correspond_cnt++;
                                    break;
                                }
                                if (j == end_node.size()-1)
                                    obstacle_occur_node.push_back(i);
                            }
                        if (correspond_cnt == end_node.size()) {
                            if (M.LayerSet[current_layer].Orientation == 1 && T.pre_turnpoints.size() != 0) {
                                if (get_coordinate(T.pre_turnpoints[T.pre_turnpoints.size()-1][0], G, M).second <= get_coordinate(GN[0], G, M).second) {
                                    not_right_way = 1;
                                    break;
                                }
                            }
                            else if (T.pre_turnpoints.size() != 0) {
                                if (get_coordinate(T.pre_turnpoints[T.pre_turnpoints.size()-1][0], G, M).first <= get_coordinate(GN[0], G, M).first) {
                                    not_right_way = 1;
                                    break;
                                }
                            }
                            find_the_solution = 1;
                            T.pre_turnpoints.push_back(GN_origin);
                            T_solution = T;
                            for (int i = 0; i < run_1_node.size(); i++) if (run_1_node[i]) T_solution.spv_cnt++;
                            // cout << "solution find !!\n";
                            break;
                        }
                        else if (correspond_cnt > 0) {
                            if (M.LayerSet[current_layer].Orientation == 1 && T.pre_turnpoints.size() != 0) {
                                if (get_coordinate(T.pre_turnpoints[T.pre_turnpoints.size()-1][0], G, M).second <= get_coordinate(GN[0], G, M).second) {
                                    not_right_way = 1;
                                    break;
                                }
                            }
                            else if (T.pre_turnpoints.size() != 0) {
                                if (get_coordinate(T.pre_turnpoints[T.pre_turnpoints.size()-1][0], G, M).first <= get_coordinate(GN[0], G, M).first) {
                                    not_right_way = 1;
                                    break;
                                }
                            }
                            partial_solution = 1;
                            //cout << "tune the solution!!\n";
                            break;
                        }
                        else obstacle_occur_node.clear();
                        line_end = GN[0].grid_index.second;
                        //int pause_;
                        //cin >> pause_;
                        for (int i = 0; i < GN.size(); i++) {
                            Map_grid map_grid;
                            map_grid.Layer = current_layer;
                            map_grid.grid_index = GN[i].grid_index;
                            map_grid.grid_index.second--;
                            if (GN[i].grid_index.second == 0) {
                                obstacle_occur_node.push_back(i);
                            }
                            else if (G.GridMapSet[current_layer].track[GN[i].grid_index.first].gridpoint[GN[i].grid_index.second-1].type == 0 && T.path_check.find(map_grid) == T.path_check.end()) {
                               GN[i].grid_index.second--;
                            }
                            else if (G.GridMapSet[current_layer].track[GN[i].grid_index.first].gridpoint[GN[i].grid_index.second-1].type == 1 && T.path_check.find(map_grid) == T.path_check.end()) {
                               GN[i].grid_index.second--;
                               run_1_node[i] = 1;
                            }
                            else {
                                first_obs_line = i;
                                obstacle_occur_node.push_back(i);
                            }
                        }
                        if (obstacle_occur_node.size() != 0) {
                            obstacle_occur = 1;
                        }
                    }
                    /*if (bus_id == 1 || bus_id == 5) {
                    cerr << "finish go down\n";
                    for (int i = 0; i < GN.size(); i++) {
                        cout << "GN_[" << i << "] = ";
                        print_coordinate(GN[i], G, M);
                    }
                    cout << "current layer = " << current_layer << endl;
                    }*/
                }
                if (not_right_way && !zero_seg) {
                    //cout << "not right way\n";
                    break;
                }
                if (T.pre_turnpoints.size() == 0 && partial_solution) {
                    vector<int> tmp_obs;
                    for (int i = 0; i < GN_origin.size(); i++) {
                        bool find_obs = 0;
                        for (int j = 0; j < obstacle_occur_node.size(); j++) {
                            if (obstacle_occur_node[j] == i) {
                                find_obs = 1;
                                break;
                            }
                        }
                        if (!find_obs) tmp_obs.push_back(i);
                    }
                    obstacle_occur_node = tmp_obs;
                    partial_solution = 0;
                }
                if (obstacle_occur_node.size() != GN.size() && !find_the_solution && !zero_seg) {
                    vector<bool> obstacle_double_check(GN.size(), 0);
                    for (int i = 0; i < obstacle_occur_node.size(); i++)
                        obstacle_double_check[obstacle_occur_node[i]] = 1;
                    vector<int> obstacle_double_occur_node;
                    bool obstacle_double_occur = 0;
                    int running_cnt = 0;
                    if (T.up_down == 1) {
                        while (!obstacle_double_occur) {
                            running_cnt++;
                            int correspond_cnt = 0;
                            vector<int> partial_obs;
                            for (int i = 0; i < GN.size(); i++) {
                                if (obstacle_double_check[i] == 1) continue;
                                for (int j = 0; j < end_node.size(); j++) {
                                    if (end_node[j].Layer == GN[i].Layer && end_node[j].grid_index == GN[i].grid_index) {
                                        correspond_cnt++;
                                        //cout << "find one!!\n";
                                        //print_coordinate(end_node[j], G, M);
                                        break;
                                    }
                                    if (j == end_node.size()-1)
                                        partial_obs.push_back(i);
                                }
                            }
                             if (correspond_cnt == end_node.size()) {
                                 find_the_solution = 1;
                                 // cout << "solution find !!\n";
                                 T.pre_turnpoints.push_back(GN_origin);
                                 T_solution = T;
                                 for (int i = 0; i < run_1_node.size(); i++) if (run_1_node[i]) T_solution.spv_cnt++;
                                 break;
                             }
                             else if (correspond_cnt > 0) {
                                 partial_solution = 1;
                                 //cout << "tune the solution!!\n";
                                 for (int i = 0; i < partial_obs.size(); i++)
                                     obstacle_occur_node.push_back(partial_obs[i]);
                                 break;
                             }
                            bool bd_touch = 0;
                            for (int i = 0; i < GN.size(); i++) {
                                if (obstacle_double_check[i] == 0) {
                                    Map_grid map_grid;
                                    map_grid.Layer = current_layer;
                                    map_grid.grid_index = GN[i].grid_index;
                                    map_grid.grid_index.second--;
                                    if (GN[i].grid_index.second == 0) {
                                        obstacle_double_occur_node.push_back(i);
                                        obstacle_double_check[i] = 1;
                                        bd_touch = 1;
                                    }
                                    else if (G.GridMapSet[current_layer].track[GN[i].grid_index.first].gridpoint[GN[i].grid_index.second-1].type == 0 && T.path_check.find(map_grid) == T.path_check.end()) {
                                        GN[i].grid_index.second--;
                                    }
                                    else if (G.GridMapSet[current_layer].track[GN[i].grid_index.first].gridpoint[GN[i].grid_index.second-1].type == 1 && T.path_check.find(map_grid) == T.path_check.end()) {
                                        GN[i].grid_index.second--;
                                        run_1_node[i] = 1;
                                    }
                                    else {
                                        obstacle_double_occur_node.push_back(i);
                                        obstacle_double_check[i] = 1;
                                    }
                                }
                            }
                            if (bd_touch) {
                                obstacle_double_occur = 1;
                            }
                            else if (running_cnt > GN.size()*1.5) obstacle_double_occur = 1;
                        }
                    }
                    else {
                        while (!obstacle_double_occur) {
                            running_cnt++;
                            int correspond_cnt = 0;
                            vector<int> partial_obs;
                            for (int i = 0; i < GN.size(); i++) {
                                if (obstacle_double_check[i] == 1) continue;
                                for (int j = 0; j < end_node.size(); j++) {
                                    if (end_node[j].Layer == GN[i].Layer && end_node[j].grid_index == GN[i].grid_index) {
                                        correspond_cnt++;
                                        //cout << "find one!!\n";
                                        break;
                                    }
                                    if (j == end_node.size()-1)
                                        partial_obs.push_back(i);
                                }
                            }
                            if (correspond_cnt == end_node.size()) {
                                find_the_solution = 1;
                                // cout << "solution find !!\n";
                                T.pre_turnpoints.push_back(GN_origin);
                                T_solution = T;
                                for (int i = 0; i < run_1_node.size(); i++) if (run_1_node[i]) T_solution.spv_cnt++;
                                break;
                            }
                            else if (correspond_cnt > 0) {
                                partial_solution = 1;
                                //cout << "tune the solution!!\n";
                                for (int i = 0; i < partial_obs.size(); i++)
                                    obstacle_occur_node.push_back(partial_obs[i]);
                                break;
                            }
                            bool bd_touch = 0;
                            for (int i = 0; i < GN.size(); i++) {
                                if (obstacle_double_check[i] == 0) {
                                    Map_grid map_grid;
                                    map_grid.Layer = current_layer;
                                    map_grid.grid_index = GN[i].grid_index;
                                    map_grid.grid_index.second++;
                                    if (GN[i].grid_index.second+1 == G.GridMapSet[current_layer].track[GN[i].grid_index.first].gridpoint.size()) {
                                        obstacle_double_occur_node.push_back(i);
                                        obstacle_double_check[i] = 1;
                                        bd_touch = 1;
                                    }
                                    else if (G.GridMapSet[current_layer].track[GN[i].grid_index.first].gridpoint[GN[i].grid_index.second+1].type == 0 && T.path_check.find(map_grid) == T.path_check.end()) {
                                        GN[i].grid_index.second++;
                                    }
                                    else if (G.GridMapSet[current_layer].track[GN[i].grid_index.first].gridpoint[GN[i].grid_index.second+1].type == 1 && T.path_check.find(map_grid) == T.path_check.end()) {
                                        GN[i].grid_index.second++;
                                        run_1_node[i] = 1;
                                    }
                                    else {
                                        obstacle_double_occur_node.push_back(i);
                                        obstacle_double_check[i] = 1;
                                    }
                                }
                            }
                            if (bd_touch) {
                                obstacle_double_occur = 1;
                            }
                            else if (running_cnt > GN.size()*1.5) obstacle_double_occur = 1;
                        }
                    }
                    if (((float)(obstacle_occur_node.size()+obstacle_double_occur_node.size()))/((float)GN_origin.size()) > 0.6 && !partial_solution) {
                        for (int i = 0; i < obstacle_double_occur_node.size(); i++)
                            obstacle_occur_node.push_back(obstacle_double_occur_node[i]);
                    }
                }
                // Not finish but mistaken to finish
                if (T.pre_turnpoints.size() == 0 && partial_solution) {
                    vector<int> tmp_obs;
                    for (int i = 0; i < GN_origin.size(); i++) {
                        bool find_obs = 0;
                        for (int j = 0; j < obstacle_occur_node.size(); j++) {
                            if (obstacle_occur_node[j] == i) {
                                find_obs = 1;
                                break;
                            }
                        }
                        if (!find_obs) tmp_obs.push_back(i);
                    }
                    obstacle_occur_node = tmp_obs;
                    partial_solution = 0;
                }
                //if (not_right_way) break;
                //cerr << "ran into obstacle\n";
                //cerr << "run cnt = " << line_end - GN_origin[0].grid_index.second +1 << endl;
                int possi_turn = 0;
                bool try_next = 0;
                if (T.up_down == 0) possi_turn = line_end - Origin_end.grid_index.second;
                else possi_turn = Origin_end.grid_index.second - line_end;
                if (possi_turn < GN_origin.size()) try_next = 1;
                if (((float)obstacle_occur_node.size())/((float)GN_origin.size()) > 0.65 && !partial_solution && !try_next && !zero_seg) { // BIG OBSTACLE OCCUR
                    //cerr << "big one : " << ((float)obstacle_occur_node.size())/((float)GN_origin.size()) << endl;
                    Turn_queue T_new_up, T_new_down;
                    T_new_up.triangle_turns = T.triangle_turns;
                    T_new_down.triangle_turns = T.triangle_turns;
                    T_new_up.used_grid_node = T.used_grid_node;
                    T_new_down.used_grid_node = T.used_grid_node;
                    T_new_up.pre_turnpoints = T.pre_turnpoints;
                    T_new_down.pre_turnpoints = T.pre_turnpoints;
                    T_new_up.path_check = T.path_check;
                    T_new_down.path_check = T.path_check;
                    T_new_up.line_cost = T.line_cost;
                    T_new_down.line_cost = T.line_cost;
                    T_new_up.spv_cnt = T.spv_cnt;
                    T_new_down.spv_cnt = T.spv_cnt;
                    T_new_up.up_down = 0;
                    T_new_down.up_down = 1;
                    int next_layer = current_layer;
                    // find next layer
                    if (current_layer > end_node[0].Layer) {
                        for (int i = 1; i < M.LayerSet.size(); i++)  // go lower layer
                            if (current_layer-i >= 0 && M.LayerSet[current_layer-i].Orientation != M.LayerSet[current_layer].Orientation) {
                                next_layer = current_layer - i;
                                break;
                            }
                        if (next_layer == current_layer) {
                            for (int i = 1; i < M.LayerSet.size(); i++)  // go upper layer
                                if (current_layer+i < M.LayerSet.size() && M.LayerSet[current_layer+i].Orientation != M.LayerSet[current_layer].Orientation) {
                                    next_layer = current_layer + i;
                                    break;
                                }
                        }
                    }
                    else {
                        for (int i = 1; i < M.LayerSet.size(); i++)  // go upper layer
                            if (current_layer+i < M.LayerSet.size() && M.LayerSet[current_layer+i].Orientation != M.LayerSet[current_layer].Orientation) {
                                next_layer = current_layer + i;
                                break;
                            }
                        if (next_layer == current_layer) {
                            for (int i = 1; i < M.LayerSet.size(); i++)  // go lower layer
                                if (current_layer-i >= 0 && M.LayerSet[current_layer-i].Orientation != M.LayerSet[current_layer].Orientation) {
                                    next_layer = current_layer - i;
                                    break;
                                }
                        }
                    }
                    //push possible node into Turn_queue
                    if (T.up_down == 0) {
                        for (int i = Origin_end.grid_index.second+1; i <= line_end; i++) {
                            Grid_node new_node;
                            Grid_node old_node = GN_origin[0];
                            old_node.grid_index.second = i;
                            // go down
                            bool find_node = change_layer(old_node, new_node, next_layer, G, M);
                            //cout << "down:";
                            //print_coordinate(new_node, G, M);
                            if (find_node) {
                                if (G.GridMapSet[next_layer].track[new_node.grid_index.first].width >= B.Width[next_layer]) {
                                    new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    T_new_down.Turn_point.push(new_node);
                                    //cout << "down_find\n";
                                }
                                else if (new_node.grid_index.first+1 < G.GridMapSet[next_layer].track.size() && G.GridMapSet[next_layer].track[new_node.grid_index.first+1].coord == G.GridMapSet[next_layer].track[new_node.grid_index.first].coord) {
                                	if (G.GridMapSet[next_layer].track[new_node.grid_index.first+1].width >= B.Width[next_layer]) {
                                		new_node.grid_index.first++;
                                    	new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    	T_new_down.Turn_point.push(new_node);
                                	}
                                }
                                else if (new_node.grid_index.first-1 >= 0 && G.GridMapSet[next_layer].track[new_node.grid_index.first-1].coord == G.GridMapSet[next_layer].track[new_node.grid_index.first].coord) {
                                	if (G.GridMapSet[next_layer].track[new_node.grid_index.first-1].width >= B.Width[next_layer]) {
                                		new_node.grid_index.first--;
                                    	new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    	T_new_down.Turn_point.push(new_node);
                                	}
                                }
                            }
                            // go up
                            old_node = GN_origin[GN_origin.size()-1];
                            old_node.grid_index.second = i;
                            find_node = change_layer(old_node, new_node, next_layer, G, M);
                            //cout << "up:";
                            //print_coordinate(new_node, G, M);
                            if (find_node) {
                                if (G.GridMapSet[next_layer].track[new_node.grid_index.first].width >= B.Width[next_layer]) {
                                    new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    T_new_up.Turn_point.push(new_node);
                                    //cout << "down_find\n";
                                }
                                else if (new_node.grid_index.first+1 < G.GridMapSet[next_layer].track.size() && G.GridMapSet[next_layer].track[new_node.grid_index.first+1].coord == G.GridMapSet[next_layer].track[new_node.grid_index.first].coord) {
                                	if (G.GridMapSet[next_layer].track[new_node.grid_index.first+1].width >= B.Width[next_layer]) {
                                		new_node.grid_index.first++;
                                    	new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    	T_new_up.Turn_point.push(new_node);
                                	}
                                }
                                else if (new_node.grid_index.first-1 >= 0 && G.GridMapSet[next_layer].track[new_node.grid_index.first-1].coord == G.GridMapSet[next_layer].track[new_node.grid_index.first].coord) {
                                	if (G.GridMapSet[next_layer].track[new_node.grid_index.first-1].width >= B.Width[next_layer]) {
                                		new_node.grid_index.first--;
                                    	new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    	T_new_up.Turn_point.push(new_node);
                                	}
                                }
                            }
                        }
                    }
                    else {
                        for (int i = Origin_end.grid_index.second-1; i >= line_end; i--) {
                            Grid_node new_node;
                            Grid_node old_node = GN_origin[0];
                            old_node.grid_index.second = i;
                            // go down
                            bool find_node = change_layer(old_node, new_node, next_layer, G, M);
                            if (find_node) {
                                if (G.GridMapSet[next_layer].track[new_node.grid_index.first].width >= B.Width[next_layer]) {
                                    new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    T_new_down.Turn_point.push(new_node);
                                }
                                else if (new_node.grid_index.first+1 < G.GridMapSet[next_layer].track.size() && G.GridMapSet[next_layer].track[new_node.grid_index.first+1].coord == G.GridMapSet[next_layer].track[new_node.grid_index.first].coord) {
                                	if (G.GridMapSet[next_layer].track[new_node.grid_index.first+1].width >= B.Width[next_layer]) {
                                		new_node.grid_index.first++;
                                    	new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    	T_new_down.Turn_point.push(new_node);
                                	}
                                }
                                else if (new_node.grid_index.first-1 >= 0 && G.GridMapSet[next_layer].track[new_node.grid_index.first-1].coord == G.GridMapSet[next_layer].track[new_node.grid_index.first].coord) {
                                	if (G.GridMapSet[next_layer].track[new_node.grid_index.first-1].width >= B.Width[next_layer]) {
                                		new_node.grid_index.first--;
                                    	new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    	T_new_down.Turn_point.push(new_node);
                                	}
                                }
                            }
                            // go up
                            old_node = GN_origin[GN_origin.size()-1];
                            old_node.grid_index.second = i;
                            find_node = change_layer(old_node, new_node, next_layer, G, M);
                            if (find_node) {
                                if (G.GridMapSet[next_layer].track[new_node.grid_index.first].width >= B.Width[next_layer]) {
                                    new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    T_new_up.Turn_point.push(new_node);
                                }
                                else if (new_node.grid_index.first+1 < G.GridMapSet[next_layer].track.size() && G.GridMapSet[next_layer].track[new_node.grid_index.first+1].coord == G.GridMapSet[next_layer].track[new_node.grid_index.first].coord) {
                                	if (G.GridMapSet[next_layer].track[new_node.grid_index.first+1].width >= B.Width[next_layer]) {
                                		new_node.grid_index.first++;
                                    	new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    	T_new_up.Turn_point.push(new_node);
                                	}
                                }
                                else if (new_node.grid_index.first-1 >= 0 && G.GridMapSet[next_layer].track[new_node.grid_index.first-1].coord == G.GridMapSet[next_layer].track[new_node.grid_index.first].coord) {
                                	if (G.GridMapSet[next_layer].track[new_node.grid_index.first-1].width >= B.Width[next_layer]) {
                                		new_node.grid_index.first--;
                                    	new_node.manhattan = cal_manhattan (new_node, end_node[end_node.size()/2], G, M);
                                    	T_new_up.Turn_point.push(new_node);
                                	}
                                }
                            }
                        }
                    }
                    if (T.pre_turnpoints.size() > 0) {
                        if (T.last_dir == 0) {
                            vector<Grid_node> GN_use = T.pre_turnpoints[T.pre_turnpoints.size()-1];
                            int bus_space = (M.LayerSet[GN_use[0].Layer].Width + (B.Width[GN_use[0].Layer]/2));
                            Grid_node cmp_node;
                            change_layer(GN_origin[0], cmp_node, GN_use[0].Layer, G, M);
                            while (GN_use[0].grid_index.second <= cmp_node.grid_index.second) {
                                for (int i = 0; i < GN_use.size(); i++) {
                                    for (int j = GN_use[i].grid_index.first; j >= 0; j--) {
                                        if (G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord-G.GridMapSet[GN_use[0].Layer].track[j].coord <= bus_space) {
                                            Map_grid map_grid;
                                            map_grid.grid_index = GN_use[0].grid_index;
                                            map_grid.Layer = GN_use[0].Layer;
                                            map_grid.grid_index.first = j;
                                            if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T.path_check.find(map_grid) == T.path_check.end()) {
                                                T_new_up.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                                T_new_down.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                                T_new_up.path_check[map_grid] = 1;
                                                T_new_down.path_check[map_grid] = 1;
                                            }
                                        }
                                        else break;
                                    }
                                    for (int j = GN_use[i].grid_index.first; j < G.GridMapSet[GN_use[0].Layer].track.size(); j++) {
                                        if (G.GridMapSet[GN_use[0].Layer].track[j].coord-G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord <= bus_space) {
                                            Map_grid map_grid;
                                            map_grid.grid_index = GN_use[0].grid_index;
                                            map_grid.Layer = GN_use[0].Layer;
                                            map_grid.grid_index.first = j;
                                            if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T.path_check.find(map_grid) == T.path_check.end()) {
                                                T_new_up.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                                T_new_down.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                                T_new_up.path_check[map_grid] = 1;
                                                T_new_down.path_check[map_grid] = 1;
                                            }
                                        }
                                        else break;
                                    }
                                }
                                GN_use[0].grid_index.second++;
                            }
                        }
                        else {
                            vector<Grid_node> GN_use = T.pre_turnpoints[T.pre_turnpoints.size()-1];
                            int bus_space = (M.LayerSet[GN_use[0].Layer].Width + (B.Width[GN_use[0].Layer]/2));
                            Grid_node cmp_node;
                            change_layer(GN_origin[GN_origin.size()-1], cmp_node, GN_use[0].Layer, G, M);
                            while (GN_use[0].grid_index.second > cmp_node.grid_index.second) {
                                for (int i = 0; i < GN_use.size(); i++) {
                                    for (int j = GN_use[i].grid_index.first; j >= 0; j--) {
                                        if (G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord-G.GridMapSet[GN_use[0].Layer].track[j].coord <= bus_space) {
                                            Map_grid map_grid;
                                            map_grid.grid_index = GN_use[0].grid_index;
                                            map_grid.Layer = GN_use[0].Layer;
                                            map_grid.grid_index.first = j;
                                            if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T.path_check.find(map_grid) == T.path_check.end()) {
                                                T_new_up.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                                T_new_down.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                                T_new_up.path_check[map_grid] = 1;
                                                T_new_down.path_check[map_grid] = 1;
                                            }
                                        }
                                        else break;
                                    }
                                    for (int j = GN_use[i].grid_index.first; j < G.GridMapSet[GN_use[0].Layer].track.size(); j++) {
                                        if (G.GridMapSet[GN_use[0].Layer].track[j].coord-G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord <= bus_space) {
                                            Map_grid map_grid;
                                            map_grid.grid_index = GN_use[0].grid_index;
                                            map_grid.Layer = GN_use[0].Layer;
                                            map_grid.grid_index.first = j;
                                            if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T.path_check.find(map_grid) == T.path_check.end()) {
                                                T_new_up.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                                T_new_down.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                                T_new_up.path_check[map_grid] = 1;
                                                T_new_down.path_check[map_grid] = 1;
                                            }
                                        }
                                        else break;
                                    }
                                }
                                GN_use[0].grid_index.second--;
                            }
                        }
                    }
                    //end pushing
                    vector<bool> gn_use_flag(GN_origin.size(), 0);
                    for (int i = 0; i < obstacle_occur_node.size(); i++)
                        gn_use_flag[obstacle_occur_node[i]] = 1;
                    for (int i = 0; i < GN_origin.size(); i++)
                        if (gn_use_flag[i] == 0)
                            T.Turn_point.push(GN_origin[i]);
                    T_new_up.last_dir = T.up_down;
                    T_new_down.last_dir = T.up_down;

                    int run_1_cnt = 0;
                    for (int i = 0; i < run_1_node.size(); i++)
                        if (run_1_node[i]) run_1_cnt++;
                    T_new_up.spv_cnt += run_1_cnt;
                    T_new_down.spv_cnt += run_1_cnt;

                    T_new_up.pre_turnpoints.push_back(GN_origin);
                    T_new_down.pre_turnpoints.push_back(GN_origin);
                    if (!T_new_up.Turn_point.empty()) {
                        Grid_node tmp_node = T_new_up.Turn_point.top();
                        pair<int, int> tmp_guide = get_coordinate(tmp_node, G, M);
                        T_new_up.line_cost += ((run_1_cnt > 0)?RUN_PARA:1) * cal_pair_manhattan (tmp_guide, T.guide_coord);
                        T_new_up.guide_coord = tmp_guide;
                        T_new_up.cost = cal_pair_manhattan(T_new_up.guide_coord, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER + T_new_up.line_cost;
                        //cout << "up:   ( " << T_new_up.guide_coord.first << ", " << T_new_up.guide_coord.second << " )\n";
                        //cout << "up size = " << T_new_up.Turn_point.size() << endl;
                        if (T_new_up.Turn_point.size() >= end_node.size() && T_new_down.spv_cnt*M.DELTA < M.EPSILON)
                            R.Astar_queue.push(T_new_up);
                    }
                    if (!T_new_down.Turn_point.empty())  {
                        Grid_node tmp_node = T_new_down.Turn_point.top();
                        pair<int, int> tmp_guide = get_coordinate(tmp_node, G, M);
                        T_new_down.line_cost += ((run_1_cnt > 0)?RUN_PARA:1) * cal_pair_manhattan (tmp_guide, T.guide_coord);
                        T_new_down.guide_coord = tmp_guide;
                        T_new_down.cost = cal_pair_manhattan(T_new_down.guide_coord, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER + T_new_down.line_cost;
                        //cout << "down: ( " << T_new_down.guide_coord.first << ", " << T_new_down.guide_coord.second << " )\n";
                        //cout << "down size = " << T_new_down.Turn_point.size() << endl;
                        if (T_new_down.Turn_point.size() >= end_node.size() && T_new_down.spv_cnt*M.DELTA < M.EPSILON)
                            R.Astar_queue.push(T_new_down);
                    }
                    
                    
                    
                    if (M.LayerSet[current_layer].Orientation == 1 && !T.Turn_point.empty()) {
                        pair<int, int> tmp_guide = get_coordinate(T.Turn_point.top(), G, M);
                        tmp_guide.second = T.guide_coord.second;
                        if (T.pre_turnpoints.size() == 0) {
                            T.cost = cal_pair_manhattan(tmp_guide, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER;
                            T.guide_coord = tmp_guide;
                        }
                        else {
                            if (T.last_dir == 0)
                                T.line_cost += tmp_guide.first-T.guide_coord.first;
                            else
                                T.line_cost -= tmp_guide.first-T.guide_coord.first;
                            T.guide_coord = tmp_guide;
                            T.cost = cal_pair_manhattan(T.guide_coord, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER + T.line_cost;
                        }
                    }
                    else if (!T.Turn_point.empty())  {
                        pair<int, int> tmp_guide = get_coordinate(T.Turn_point.top(), G, M);
                        tmp_guide.first = T.guide_coord.first;
                        if (T.pre_turnpoints.size() == 0) {
                            T.cost = cal_pair_manhattan(tmp_guide, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER;
                            T.guide_coord = tmp_guide;
                        }
                        else {
                            if (T.last_dir == 0)
                                T.line_cost += tmp_guide.second-T.guide_coord.second;
                            else
                                T.line_cost -= tmp_guide.second-T.guide_coord.second;
                            T.guide_coord = tmp_guide;
                            T.cost = cal_pair_manhattan(T.guide_coord, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER + T.line_cost;
                        }
                    }
                    //cout << "T size = " << T.Turn_point.size() << endl;
                    if (T.Turn_point.size() >= end_node.size())
                        R.Astar_queue.push(T);
                    break;
                }
                else if (!find_the_solution) { // find next possible path
                    //cerr << "small one : " << ((float)obstacle_occur_node.size())/((float)GN_origin.size()) << endl;
                    bool q_empty = 0;
                    vector<Grid_node> T_tmp;
                    vector<bool> obs_not_occur(GN_origin.size(), 0);
                    for (int i = 0; i < obstacle_occur_node.size(); i++)
                        obs_not_occur[obstacle_occur_node[i]] = 1;
                    if (T.up_down == 0) {
                        for (int i = 0; i < obstacle_occur_node.size(); i++) {
                            if (T.Turn_point.empty()) {
                                q_empty = 1;
                                break;
                            }
                            else {
                                bool find_space = 0;
                                while (!find_space && !T.Turn_point.empty()) {
                                    if (M.LayerSet[GN_origin[0].Layer].Orientation == 1) {
                                        bool violate_spacing = 0;
                                        if (!partial_solution) {
                                            for (int s_i = 0; s_i < GN_origin.size(); s_i++) {
                                                if (obs_not_occur[s_i] == 1) continue;
                                                if (abs(get_coordinate(GN_origin[s_i], G, M).first - get_coordinate(T.Turn_point.top(), G, M).first) < (B.Width[GN_origin[0].Layer]/2)+M.LayerSet[GN_origin[0].Layer].Width) {
                                                    violate_spacing = 1;
                                                    break;
                                                }
                                            }
                                        }
                                        if (violate_spacing == 0) {
                                            find_space = 1;
                                        }
                                        else {
                                            T_tmp.push_back(T.Turn_point.top());
                                            T.Turn_point.pop();
                                        }
                                    }
                                    else {
                                        bool violate_spacing = 0;
                                        if (!partial_solution) {
                                            for (int s_i = 0; s_i < GN_origin.size(); s_i++) {
                                                if (obs_not_occur[s_i] == 1) continue;
                                                if (abs(get_coordinate(GN_origin[s_i], G, M).second - get_coordinate(T.Turn_point.top(), G, M).second) < (B.Width[GN_origin[0].Layer]/2)+M.LayerSet[GN_origin[0].Layer].Width) {
                                                    violate_spacing = 1;
                                                    break;
                                                }
                                            }
                                        }
                                        if (violate_spacing == 0) {
                                            find_space = 1;
                                        }
                                        else {
                                            T_tmp.push_back(T.Turn_point.top());
                                            T.Turn_point.pop();
                                        }
                                    }
                                }
                                if (find_space) {
                                    obs_not_occur[obstacle_occur_node[i]] = 0;
                                    GN_origin[obstacle_occur_node[i]] = T.Turn_point.top();
                                    T.Turn_point.pop();
                                }
                                else {
                                    q_empty = 1;
                                    break;
                                }
                            }
                        }
                        if (q_empty == 1) break;
                        for (int i = 0; i < T_tmp.size(); i++)
                            T.Turn_point.push(T_tmp[i]);
                        T_tmp.clear();
                        sort(GN_origin.begin(), GN_origin.end());
                    }
                    else {
                        for (int i = 0; i < obstacle_occur_node.size(); i++) {
                            if (T.Turn_point.empty()) {
                                q_empty = 1;
                                //cerr << "queue empty(down)\n";
                                break;
                            }
                            else {
                                bool find_space = 0;
                                while (!find_space && ! T.Turn_point.empty()) {
                                    if (M.LayerSet[GN_origin[0].Layer].Orientation == 1) {
                                        bool violate_spacing = 0;
                                        if (!partial_solution) {
                                            for (int s_i = 0; s_i < GN_origin.size(); s_i++) {
                                                if (obs_not_occur[s_i] == 1) continue;
                                                if (abs(get_coordinate(GN_origin[s_i], G, M).first - get_coordinate(T.Turn_point.top(), G, M).first) <  (B.Width[GN_origin[0].Layer]/2)+M.LayerSet[GN_origin[0].Layer].Width) {
                                                    violate_spacing = 1;
                                                    break;
                                                }
                                            }
                                        }
                                        if (violate_spacing == 0) {
                                            find_space = 1;
                                        }
                                        else {
                                            T_tmp.push_back(T.Turn_point.top());
                                            T.Turn_point.pop();
                                        }
                                    }
                                    else {
                                        bool violate_spacing = 0;
                                        if (!partial_solution) {
                                            for (int s_i = 0; s_i < GN_origin.size(); s_i++) {
                                                if (obs_not_occur[s_i] == 1) continue;
                                                if (abs(get_coordinate(GN_origin[s_i], G, M).second - get_coordinate(T.Turn_point.top(), G, M).second) <  (B.Width[GN_origin[0].Layer]/2)+M.LayerSet[GN_origin[0].Layer].Width) {
                                                    violate_spacing = 1;
                                                    break;
                                                }
                                            }
                                        }
                                        if (violate_spacing == 0) {
                                            find_space = 1;
                                        }
                                        else {
                                            T_tmp.push_back(T.Turn_point.top());
                                            T.Turn_point.pop();
                                        }
                                    }
                                }
                                if (find_space) {
                                    obs_not_occur[obstacle_occur_node[i]] = 0;
                                    GN_origin[obstacle_occur_node[i]] = T.Turn_point.top();
                                    T.Turn_point.pop();
                                }
                                else {
                                    q_empty = 1;
                                    break;
                                }
                            }
                        }
                        if (q_empty == 1) break;
                        for (int i = 0; i < T_tmp.size(); i++)
                            T.Turn_point.push(T_tmp[i]);
                        T_tmp.clear();
                        sort(GN_origin.begin(), GN_origin.end());
                    }
                    //code_change
                    pair<int, int> tmp_guide = get_coordinate(GN_origin[GN_origin.size()/2], G, M);
                    if (M.LayerSet[current_layer].Orientation == 1) {
                        tmp_guide.second = T.guide_coord.second;
                        if (T.pre_turnpoints.size() == 0) {
                            T.cost = cal_pair_manhattan(tmp_guide, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER;
                            T.guide_coord = tmp_guide;
                        }
                        else {
                            if (T.last_dir == 0)
                                T.line_cost += tmp_guide.first-T.guide_coord.first;
                            else
                                T.line_cost -= tmp_guide.first-T.guide_coord.first;
                            T.guide_coord = tmp_guide;
                            T.cost = cal_pair_manhattan(T.guide_coord, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER + T.line_cost;
                        }
                    }
                    else  {
                        tmp_guide.first = T.guide_coord.first;
                        if (T.pre_turnpoints.size() == 0) {
                            T.cost = cal_pair_manhattan(tmp_guide, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER;
                            T.guide_coord = tmp_guide;
                        }
                        else {
                            if (T.last_dir == 0)
                                T.line_cost += tmp_guide.second-T.guide_coord.second;
                            else
                                T.line_cost -= tmp_guide.second-T.guide_coord.second;
                            T.guide_coord = tmp_guide;
                            T.cost = cal_pair_manhattan(T.guide_coord, get_coordinate(end_node[end_node.size()/2], G, M))*DIS_PARAMETER + T.line_cost;
                        }
                    }
                    // end_change
                }
            }
            iter_cnt++;
        }
        if (find_the_solution && T_solution.spv_cnt*M.DELTA < M.EPSILON) {
            //bus_result build here
            if(k==0){
                result.push_back(1);
                result.push_back(1);
            }
            else
                result.push_back(1);

            // block usage segment
            // block the last-1 segment
            if (T_solution.last_dir == 0) {
                vector<Grid_node> GN_use = T_solution.pre_turnpoints[T_solution.pre_turnpoints.size()-2];
                int bus_space = (M.LayerSet[GN_use[0].Layer].Width + (B.Width[GN_use[0].Layer]/2));
                Grid_node cmp_node;
                change_layer(T_solution.pre_turnpoints[T_solution.pre_turnpoints.size()-1][GN_use.size()-1], cmp_node, GN_use[0].Layer, G, M);
                while (GN_use[0].grid_index.second <= cmp_node.grid_index.second) {
                    for (int i = 0; i < GN_use.size(); i++) {
                        for (int j = GN_use[i].grid_index.first; j >= 0; j--) {
                            if (G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord-G.GridMapSet[GN_use[0].Layer].track[j].coord <= bus_space) {
                                Map_grid map_grid;
                                map_grid.grid_index = GN_use[0].grid_index;
                                map_grid.Layer = GN_use[0].Layer;
                                map_grid.grid_index.first = j;
                                if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T_solution.path_check.find(map_grid) == T_solution.path_check.end()) {
                                    T_solution.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                        for (int j = GN_use[i].grid_index.first; j < G.GridMapSet[GN_use[0].Layer].track.size(); j++) {
                            if (G.GridMapSet[GN_use[0].Layer].track[j].coord-G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord <= bus_space) {
                                Map_grid map_grid;
                                map_grid.grid_index = GN_use[0].grid_index;
                                map_grid.Layer = GN_use[0].Layer;
                                map_grid.grid_index.first = j;
                                if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T_solution.path_check.find(map_grid) == T_solution.path_check.end()) {
                                    T_solution.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                    }
                    GN_use[0].grid_index.second++;
                }
            }
            else {
                vector<Grid_node> GN_use = T_solution.pre_turnpoints[T_solution.pre_turnpoints.size()-2];
                int bus_space = (M.LayerSet[GN_use[0].Layer].Width + (B.Width[GN_use[0].Layer]/2));
                Grid_node cmp_node;
                change_layer(T_solution.pre_turnpoints[T_solution.pre_turnpoints.size()-1][0], cmp_node, GN_use[0].Layer, G, M);
                while (GN_use[0].grid_index.second >= cmp_node.grid_index.second) {
                    for (int i = 0; i < GN_use.size(); i++) {
                        for (int j = GN_use[i].grid_index.first; j >= 0; j--) {
                            if (G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord-G.GridMapSet[GN_use[0].Layer].track[j].coord <= bus_space) {
                                Map_grid map_grid;
                                map_grid.grid_index = GN_use[0].grid_index;
                                map_grid.Layer = GN_use[0].Layer;
                                map_grid.grid_index.first = j;
                                if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T_solution.path_check.find(map_grid) == T_solution.path_check.end()) {
                                    T_solution.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                        for (int j = GN_use[i].grid_index.first; j < G.GridMapSet[GN_use[0].Layer].track.size(); j++) {
                            if (G.GridMapSet[GN_use[0].Layer].track[j].coord-G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord <= bus_space) {
                                Map_grid map_grid;
                                map_grid.grid_index = GN_use[0].grid_index;
                                map_grid.Layer = GN_use[0].Layer;
                                map_grid.grid_index.first = j;
                                if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T_solution.path_check.find(map_grid) == T_solution.path_check.end()) {
                                    T_solution.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                    }
                    GN_use[0].grid_index.second--;
                }
            }
            
            // end block the last-1 segment
            // block the last segment
            if (T_solution.last_dir == 0) {
                vector<Grid_node> GN_use = T_solution.pre_turnpoints[T_solution.pre_turnpoints.size()-1];
                int bus_space = (M.LayerSet[GN_use[0].Layer].Width + (B.Width[GN_use[0].Layer]/2));
                while (GN_use[0].grid_index.second <= end_node[0].grid_index.second) {
                    for (int i = 0; i < GN_use.size(); i++) {
                        for (int j = GN_use[i].grid_index.first; j >= 0; j--) {
                            if (G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord-G.GridMapSet[GN_use[0].Layer].track[j].coord <= bus_space) {
                                Map_grid map_grid;
                                map_grid.grid_index = GN_use[0].grid_index;
                                map_grid.Layer = GN_use[0].Layer;
                                map_grid.grid_index.first = j;
                                if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T_solution.path_check.find(map_grid) == T_solution.path_check.end()) {
                                    T_solution.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                        for (int j = GN_use[i].grid_index.first; j < G.GridMapSet[GN_use[0].Layer].track.size(); j++) {
                            if (G.GridMapSet[GN_use[0].Layer].track[j].coord-G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord <= bus_space) {
                                Map_grid map_grid;
                                map_grid.grid_index = GN_use[0].grid_index;
                                map_grid.Layer = GN_use[0].Layer;
                                map_grid.grid_index.first = j;
                                if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T_solution.path_check.find(map_grid) == T_solution.path_check.end()) {
                                    T_solution.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                    }
                    GN_use[0].grid_index.second++;
                }
            }
            else {
                vector<Grid_node> GN_use = T_solution.pre_turnpoints[T_solution.pre_turnpoints.size()-1];
                int bus_space = (M.LayerSet[GN_use[0].Layer].Width + (B.Width[GN_use[0].Layer]/2));
                while (GN_use[0].grid_index.second >= end_node[0].grid_index.second) {
                    for (int i = 0; i < GN_use.size(); i++) {
                        for (int j = GN_use[i].grid_index.first; j >= 0; j--) {
                            if (G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord-G.GridMapSet[GN_use[0].Layer].track[j].coord <= bus_space) {
                                Map_grid map_grid;
                                map_grid.grid_index = GN_use[0].grid_index;
                                map_grid.Layer = GN_use[0].Layer;
                                map_grid.grid_index.first = j;
                                if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T_solution.path_check.find(map_grid) == T_solution.path_check.end()) {
                                    T_solution.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                        for (int j = GN_use[i].grid_index.first; j < G.GridMapSet[GN_use[0].Layer].track.size(); j++) {
                            if (G.GridMapSet[GN_use[0].Layer].track[j].coord-G.GridMapSet[GN_use[0].Layer].track[GN_use[i].grid_index.first].coord <= bus_space) {
                                Map_grid map_grid;
                                map_grid.grid_index = GN_use[0].grid_index;
                                map_grid.Layer = GN_use[0].Layer;
                                map_grid.grid_index.first = j;
                                if (G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type == 0 && T_solution.path_check.find(map_grid) == T_solution.path_check.end()) {
                                    T_solution.used_grid_node.push_back(&(G.GridMapSet[GN_use[0].Layer].track[j].gridpoint[GN_use[0].grid_index.second].type));
                                }
                            }
                            else break;
                        }
                    }
                    GN_use[0].grid_index.second--;
                }
            }
            // end block the last segment
            // end block usage segment
            for (int i = 0; i < T_solution.used_grid_node.size(); i++) {
                *(T_solution.used_grid_node[i]) = 1;
                all.types[bus_id].push_back(T_solution.used_grid_node[i]);
            }
            cout << "Output the " << k+1 << " two-pin net" << endl;
            if (end_violation) {
                cout << "Layer " << end_via_layer << endl;
                pair<int, int> tmp_end_via;
                for (int i = 0; i < end_node.size(); i++) {
                    tmp_end_via = get_coordinate(end_node[i], G, M);
                    cout << "Seg" << i << ": ( " << tmp_end_via.first << ", " << tmp_end_via.second << " ) - ( " << tmp_end_via.first << ", " << tmp_end_via.second << " )" << endl;
                    route route_tmp;
                    route_tmp.Layer = end_via_layer;
                    route_tmp.route_shape.LB.x = tmp_end_via.first;
                    route_tmp.route_shape.LB.y = tmp_end_via.second;
                    route_tmp.route_shape.RT.x = tmp_end_via.first;
                    route_tmp.route_shape.RT.y = tmp_end_via.second;
                    bus_tmp.bit_routes[i].routes.push_back(route_tmp);
                }
            }
            int start_route_layer = start_node[0].Layer;
            int route_layer = end_node[0].Layer;
            int next_route_layer=start_route_layer;
            vector<pair<int, int> > last_turn;
            vector<pair<int, int> > seg_end;
            vector<pair<int, int> > seg_start;
            bool change=false;
            
            for (int i = 0; i < end_node.size(); i++) {
                last_turn.push_back(get_coordinate(end_node[i], G, M));
                if (!end_violation) {
                    if (M.LayerSet[route_layer].Orientation == 1) {
                        if (get_coordinate(T_solution.pre_turnpoints[T_solution.pre_turnpoints.size()-1][i], G, M).second < last_turn[i].second)
                            last_turn[i].second = B.PinBitSet[i].PinShapeSet[k+1].Pin.LB.y;
                        else
                            last_turn[i].second = B.PinBitSet[i].PinShapeSet[k+1].Pin.RT.y;
                    }
                    else {
                        if (get_coordinate(T_solution.pre_turnpoints[T_solution.pre_turnpoints.size()-1][i], G, M).first < last_turn[i].first)
                            last_turn[i].first = B.PinBitSet[i].PinShapeSet[k+1].Pin.LB.x;
                        else
                            last_turn[i].first = B.PinBitSet[i].PinShapeSet[k+1].Pin.RT.x;
                    }
                }
            }
            int poss_change = -1;
            int poss_layer = -1;
            int tri_id = T_solution.triangle_turns.size()-1;
            vector<Seg_tmp> seg_tmp;
            for (int i = T_solution.pre_turnpoints.size()-1; i >= 1; i--) {
                route route_tmp;
                route_tmp.Layer = route_layer;
                seg_end = last_turn;
                seg_start = seg_end;
                if (i == 1) {
                    if (M.LayerSet[route_layer].Orientation == 1) {
                        for (int j = 0; j < seg_start.size(); j++)
                            seg_start[j].second = get_coordinate(start_node[j], G, M).second;
                    }
                    else {
                        for (int j = 0; j < seg_start.size(); j++)
                            seg_start[j].first = get_coordinate(start_node[j], G, M).first;
                    }
                }
                else {
                    if (M.LayerSet[route_layer].Orientation == 1) {
                        for (int j = 0; j < seg_start.size(); j++)
                            seg_start[j].second = get_coordinate(T_solution.pre_turnpoints[i-1][j], G, M).second;
                    }
                    else {
                        for (int j = 0; j < seg_start.size(); j++)
                            seg_start[j].first = get_coordinate(T_solution.pre_turnpoints[i-1][j], G, M).first;
                    }
                }
                if (tri_id >= 0 && seg_end.size() >= 2) {
                    if (T_solution.triangle_turns[tri_id].idx == i+1) {
                        int cur_tri_type = 0;
                        if (M.LayerSet[route_layer].Orientation == 1) {
                            if (seg_end[0].first < seg_end[1].first) {
                                if (seg_end[0].second > seg_start[0].second) {
                                    if (seg_end[0].second > seg_end[1].second)
                                        cur_tri_type = 1;
                                    else
                                        cur_tri_type = 0;
                                }
                                else {
                                    if (seg_end[0].second > seg_end[1].second)
                                        cur_tri_type = 0;
                                    else
                                        cur_tri_type = 1;
                                }
                            }
                            else {
                                if (seg_end[0].second > seg_start[0].second) {
                                    if (seg_end[0].second > seg_end[1].second)
                                        cur_tri_type = 0;
                                    else
                                        cur_tri_type = 1;
                                }
                                else {
                                    if (seg_end[0].second > seg_end[1].second)
                                        cur_tri_type = 1;
                                    else
                                        cur_tri_type = 0;
                                }
                            }
                        }
                        else {
                            if (seg_end[0].second < seg_end[1].second) {
                                if (seg_end[0].first > seg_start[0].first) {
                                    if (seg_end[0].first > seg_end[1].first)
                                        cur_tri_type = 1;
                                    else
                                        cur_tri_type = 0;
                                }
                                else {
                                    if (seg_end[0].first > seg_end[1].first) cur_tri_type = 0;
                                    else
                                        cur_tri_type = 1;
                                }
                            }
                            else {
                                if (seg_end[0].first > seg_start[0].first) {
                                    if (seg_end[0].first > seg_end[1].first)
                                        cur_tri_type = 0;
                                    else
                                        cur_tri_type = 1;
                                }
                                else {
                                    if (seg_end[0].first > seg_end[1].first)
                                        cur_tri_type = 1;
                                    else
                                        cur_tri_type = 0;
                                }
                            }
                        }
                        if (cur_tri_type != T_solution.triangle_turns[tri_id].tri_type) {
                            if (M.LayerSet[route_layer].Orientation == 1) {
                                for (int j = 0; j < seg_end.size()/2; j++) {
                                    int tmp_change = seg_end[j].second;
                                    seg_end[j].second = seg_end[seg_end.size()-1-j].second;
                                    seg_end[seg_end.size()-1-j].second = tmp_change;
                                    pair<int, int> tmp_seg = seg_end[j];
                                    seg_end[j] = seg_end[seg_end.size()-1-j];
                                    seg_end[seg_end.size()-1-j] = tmp_seg;
                                    tmp_seg = seg_start[j];
                                    seg_start[j] = seg_start[seg_end.size()-1-j];
                                    seg_start[seg_end.size()-1-j] = tmp_seg;
                                }
                            }
                            else {
                                for (int j = 0; j < seg_end.size()/2; j++) {
                                    int tmp_change = seg_end[j].first;
                                    seg_end[j].first = seg_end[seg_end.size()-1-j].first;
                                    seg_end[seg_end.size()-1-j].first = tmp_change;
                                    pair<int, int> tmp_seg = seg_end[j];
                                    seg_end[j] = seg_end[seg_end.size()-1-j];
                                    seg_end[seg_end.size()-1-j] = tmp_seg;
                                    tmp_seg = seg_start[j];
                                    seg_start[j] = seg_start[seg_end.size()-1-j];
                                    seg_start[seg_end.size()-1-j] = tmp_seg;
                                }
                            }
                            seg_tmp[seg_tmp.size()-1].seg_start = seg_end;
                            if (i == 1) {
                                if (M.LayerSet[route_layer].Orientation == 1) {
                                    for (int j = 0; j < seg_start.size(); j++)
                                        seg_start[j].second = get_coordinate(start_node[j], G, M).second;
                                }
                                else {
                                    for (int j = 0; j < seg_start.size(); j++)
                                        seg_start[j].first = get_coordinate(start_node[j], G, M).first;
                                }
                            }
                        }
                        tri_id--;
                    }
                    else if (poss_change == -1 && i != T_solution.pre_turnpoints.size()-1) {
                        poss_change = seg_tmp.size();
                        poss_layer = route_layer;
                    }
                }
                Seg_tmp st;
                st.seg_end = seg_end;
                st.seg_start = seg_start;
                seg_tmp.push_back(st);
                last_turn = seg_start;
                route_layer = T_solution.pre_turnpoints[i-1][0].Layer;
            }
            // last_turn
            seg_end = last_turn;
            for (int i = 0; i < seg_start.size(); i++) {
                if (k == 0)
                    seg_start[i] = get_coordinate(start_node[i], G, M);
                else
                    seg_start[i] = get_coordinate(via_node[i], G, M);
                if (k == 0 && !start_violation) {
                    if (M.LayerSet[route_layer].Orientation == 1) {
                        if (seg_start[i].second < seg_end[i].second)
                            seg_start[i].second = B.PinBitSet[i].PinShapeSet[0].Pin.RT.y;
                        else
                            seg_start[i].second = B.PinBitSet[i].PinShapeSet[0].Pin.LB.y;
                    }
                    else {
                        if (seg_start[i].first < seg_end[i].first)
                            seg_start[i].first = B.PinBitSet[i].PinShapeSet[0].Pin.RT.x;
                        else
                            seg_start[i].first = B.PinBitSet[i].PinShapeSet[0].Pin.LB.x;
                    }
                }
            }
            if (tri_id >= 0 && seg_end.size() >= 2) {
                if (T_solution.triangle_turns[tri_id].idx == 1) {
                    int cur_tri_type = 0;
                    if (M.LayerSet[route_layer].Orientation == 1) {
                        if (seg_end[0].first < seg_end[1].first) {
                            if (seg_end[0].second > seg_start[0].second) {
                                if (seg_end[0].second > seg_end[1].second)
                                    cur_tri_type = 1;
                                else
                                    cur_tri_type = 0;
                            }
                            else {
                                if (seg_end[0].second > seg_end[1].second)
                                    cur_tri_type = 0;
                                else
                                    cur_tri_type = 1;
                            }
                        }
                        else {
                            if (seg_end[0].second > seg_start[0].second) {
                                if (seg_end[0].second > seg_end[1].second)
                                    cur_tri_type = 0;
                                else
                                    cur_tri_type = 1;
                            }
                            else {
                                if (seg_end[0].second > seg_end[1].second)
                                    cur_tri_type = 1;
                                else
                                    cur_tri_type = 0;
                            }
                        }
                    }
                    else {
                        if (seg_end[0].second < seg_end[1].second) {
                            if (seg_end[0].first > seg_start[0].first) {
                                if (seg_end[0].first > seg_end[1].first)
                                    cur_tri_type = 1;
                                else
                                    cur_tri_type = 0;
                            }
                            else {
                                if (seg_end[0].first > seg_end[1].first) cur_tri_type = 0;
                                else
                                    cur_tri_type = 1;
                            }
                        }
                        else {
                            if (seg_end[0].first > seg_start[0].first) {
                                if (seg_end[0].first > seg_end[1].first)
                                    cur_tri_type = 0;
                                else
                                    cur_tri_type = 1;
                            }
                            else {
                                if (seg_end[0].first > seg_end[1].first)
                                    cur_tri_type = 1;
                                else
                                    cur_tri_type = 0;
                            }
                        }
                    }
                    if (cur_tri_type != T_solution.triangle_turns[tri_id].tri_type) {
                        if (poss_change != -1) {
                            if (M.LayerSet[route_layer].Orientation == 1) {
                                for (int i = 0; i < seg_end.size()/2; i++) {
                                    int tmp_change = seg_end[i].second;
                                    seg_end[i].second = seg_end[seg_end.size()-1-i].second;
                                    seg_end[seg_end.size()-1-i].second = tmp_change;
                                }
                            }
                            else {
                                for (int i = 0; i < seg_end.size()/2; i++) {
                                    int tmp_change = seg_end[i].first;
                                    seg_end[i].first = seg_end[seg_end.size()-1-i].first;
                                    seg_end[seg_end.size()-1-i].first = tmp_change;
                                }
                            }
                            for (int i = seg_tmp.size()-1; i >= poss_change; i--) {
                                for (int j = 0; j < seg_end.size()/2; j++) {
                                    pair<int, int> tmp_seg = seg_tmp[i].seg_end[j];
                                    seg_tmp[i].seg_end[j] = seg_tmp[i].seg_end[seg_end.size()-1-j];
                                    seg_tmp[i].seg_end[seg_end.size()-1-j] = tmp_seg;
                                    tmp_seg = seg_tmp[i].seg_start[j];
                                    seg_tmp[i].seg_start[j] = seg_tmp[i].seg_start[seg_end.size()-1-j];
                                    seg_tmp[i].seg_start[seg_end.size()-1-j] = tmp_seg;
                                }
                            }
                            seg_tmp[seg_tmp.size()-1].seg_start = seg_end;
                            if (M.LayerSet[poss_layer].Orientation == 1) {
                                for (int i = 0; i < seg_end.size()/2; i++) {
                                    int tmp_change = seg_tmp[poss_change].seg_end[i].second;
                                    seg_tmp[poss_change].seg_end[i].second = seg_tmp[poss_change].seg_end[seg_end.size()-1-i].second;
                                    seg_tmp[poss_change].seg_end[seg_end.size()-1-i].second = tmp_change;
                                }
                            }
                            else {
                                for (int i = 0; i < seg_end.size()/2; i++) {
                                    int tmp_change = seg_tmp[poss_change].seg_end[i].first;
                                    seg_tmp[poss_change].seg_end[i].first = seg_tmp[poss_change].seg_end[seg_end.size()-1-i].first;
                                    seg_tmp[poss_change].seg_end[seg_end.size()-1-i].first = tmp_change;
                                }
                            }
                            seg_tmp[poss_change-1].seg_start = seg_tmp[poss_change].seg_end;
                        }
                    }
                    tri_id--;
                }
            }
            Seg_tmp st;
            st.seg_end = seg_end;
            st.seg_start = seg_start;
            seg_tmp.push_back(st);
            //
            last_turn = seg_tmp[0].seg_end;
            route_layer = end_node[0].Layer;
            for (int i = T_solution.pre_turnpoints.size()-1; i >= 0; i--) {
                route route_tmp;
                route_tmp.Layer = route_layer;
                //next_route_layer = T_solution.pre_turnpoints[i+1][0].Layer;
                cout << "Layer " << route_layer << endl;
                vector<Grid_node> vec_grid;
                seg_end = last_turn;
                seg_start = seg_tmp[T_solution.pre_turnpoints.size()-1-i].seg_start;
                // via add
                
                if(!change && i==1 && i > 0){
                    Grid_node grid_tmp;
                    next_route_layer = T_solution.pre_turnpoints[i-1][0].Layer;
                    for(int j=0 ; j<seg_start.size() ; j++){
                        grid_tmp.grid_index=seg_start[j];
                        grid_tmp.Layer=(next_route_layer>route_layer)?route_layer : next_route_layer;
                        vec_grid.push_back(grid_tmp);

                    }
                    bus_tmp.via.push_back(vec_grid);
                    for(int j=0 ; j<vec_grid.size() ; j++){
                        cout<<"via["<<j<<"]  "<<vec_grid[j].grid_index.first<<" "<<vec_grid[j].grid_index.second<<endl;
                    }
                    vec_grid.clear();
                    /*for(int j=0 ; j<seg_start.size() ; j++){
                        grid_tmp.grid_index=seg_start[j];
                        grid_tmp.Layer=(next_route_layer>route_layer)?next_route_layer : route_layer;
                        vec_grid.push_back(grid_tmp);
                    }
                    bus_tmp.via.push_back(vec_grid);
                    vec_grid.clear();*/
                    change=true;
                }
                else if(!change && i > 0)
                    change=true;
                else if (i > 0) {
                    if(i == 1){
                        Grid_node grid_tmp;
                        next_route_layer = T_solution.pre_turnpoints[i-1][0].Layer;
                        for(int j=0 ; j<seg_start.size() ; j++){
                            grid_tmp.grid_index=seg_start[j];
                            grid_tmp.Layer=(next_route_layer > route_layer)? route_layer : next_route_layer;
                            vec_grid.push_back(grid_tmp);
                        }
                        bus_tmp.via.push_back(vec_grid);
                        for(int j=0 ; j<vec_grid.size() ; j++){
                            //cout<<"via["<<j<<"]  "<<vec_grid[j].grid_index.first<<" "<<vec_grid[j].grid_index.second<<endl;
                        }
                        vec_grid.clear();
                        /*for(int j=0 ; j<seg_start.size() ; j++){
                            grid_tmp.grid_index=seg_start[j];
                            grid_tmp.Layer=(next_route_layer > route_layer)? next_route_layer : route_layer;
                            vec_grid.push_back(grid_tmp);
                        }
                        bus_tmp.via.push_back(vec_grid);
                        vec_grid.clear();*/
                        next_route_layer = T_solution.pre_turnpoints[i+1][0].Layer;
                        for(int j=0 ; j<seg_end.size() ; j++){
                            grid_tmp.grid_index=seg_end[j];
                            grid_tmp.Layer=(next_route_layer > route_layer)? route_layer : next_route_layer;
                            vec_grid.push_back(grid_tmp);
                        }
                        bus_tmp.via.push_back(vec_grid);
                        for(int j=0 ; j<vec_grid.size() ; j++){
                            cout<<"via["<<j<<"]  "<<vec_grid[j].grid_index.first<<" "<<vec_grid[j].grid_index.second<<endl;
                        }
                        vec_grid.clear();
                        /*for(int j=0 ; j<seg_end.size() ; j++){
                            grid_tmp.grid_index=seg_end[j];
                            grid_tmp.Layer=(next_route_layer > route_layer)? next_route_layer : route_layer;
                            vec_grid.push_back(grid_tmp);
                        }
                        bus_tmp.via.push_back(vec_grid);
                        vec_grid.clear();*/
                    }
                    else{
                        Grid_node grid_tmp;
                        next_route_layer = T_solution.pre_turnpoints[i+1][0].Layer;
                        for(int j=0 ; j<seg_end.size() ; j++){
                            grid_tmp.grid_index=seg_end[j];
                            grid_tmp.Layer=(next_route_layer > route_layer)? route_layer : next_route_layer;
                            vec_grid.push_back(grid_tmp);
                        }
                        bus_tmp.via.push_back(vec_grid);
                        for(int j=0 ; j<vec_grid.size() ; j++){
                            cout<<"via["<<j<<"]  "<<vec_grid[j].grid_index.first<<" "<<vec_grid[j].grid_index.second<<endl;
                        }
                        vec_grid.clear();
                        /*for(int j=0 ; j<seg_end.size() ; j++){
                            grid_tmp.grid_index=seg_end[j];
                            grid_tmp.Layer=(next_route_layer > route_layer)? next_route_layer : route_layer;
                            vec_grid.push_back(grid_tmp);
                        }
                        bus_tmp.via.push_back(vec_grid);
                        cout << bus_tmp.via.size() << endl;
                        vec_grid.clear();*/
                    }
                }
                 
                // end via add
                for (int j = 0; j < seg_start.size(); j++) {
                    if (M.LayerSet[route_layer].Orientation == 1) {
                        if (seg_end[j].second > seg_start[j].second) {
                            route_tmp.route_shape.LB.x = seg_start[j].first-GDS_WIDTH;
                            route_tmp.route_shape.LB.y = seg_start[j].second;
                            route_tmp.route_shape.RT.x = seg_end[j].first+GDS_WIDTH;
                            route_tmp.route_shape.RT.y = seg_end[j].second;
                        }
                        else {
                            route_tmp.route_shape.LB.x = seg_end[j].first-GDS_WIDTH;
                            route_tmp.route_shape.LB.y = seg_end[j].second;
                            route_tmp.route_shape.RT.x = seg_start[j].first+GDS_WIDTH;
                            route_tmp.route_shape.RT.y = seg_start[j].second;
                        }
                    }
                    else {
                        if (seg_end[j].first > seg_start[j].first) {
                            route_tmp.route_shape.LB.x = seg_start[j].first;
                            route_tmp.route_shape.LB.y = seg_start[j].second-GDS_WIDTH;
                            route_tmp.route_shape.RT.x = seg_end[j].first;
                            route_tmp.route_shape.RT.y = seg_end[j].second+GDS_WIDTH;
                        }
                        else {
                            route_tmp.route_shape.LB.x = seg_end[j].first;
                            route_tmp.route_shape.LB.y = seg_end[j].second-GDS_WIDTH;
                            route_tmp.route_shape.RT.x = seg_start[j].first;
                            route_tmp.route_shape.RT.y = seg_start[j].second+GDS_WIDTH;
                        }
                    }
                    cout << "Seg" << j << ": ( " << seg_start[j].first << ", " << seg_start[j].second << " ) - ( " << seg_end[j].first << ", " << seg_end[j].second << " )" << endl;
                    bus_tmp.bit_routes[j].routes.push_back(route_tmp);
                }
                if (i > 0) {
                    last_turn = seg_start;
                    route_layer = T_solution.pre_turnpoints[i-1][0].Layer;
                }
            }
            
            
            
            if (start_violation) {
                route route_tmp;
                cout << "Layer " << start_via_layer << endl;
                pair<int, int> tmp_start_via;
                for (int i = 0; i < start_node.size(); i++) {
                    tmp_start_via = get_coordinate(start_node[i], G, M);
                    cout << "Seg" << i << ": ( " << tmp_start_via.first << ", " << tmp_start_via.second << " ) - ( " << tmp_start_via.first << ", " << tmp_start_via.second << " )" << endl;
                    route_tmp.Layer = start_via_layer;
                    route_tmp.route_shape.LB.x = tmp_start_via.first;
                    route_tmp.route_shape.LB.y = tmp_start_via.second;
                    route_tmp.route_shape.RT.x = tmp_start_via.first;
                    route_tmp.route_shape.RT.y = tmp_start_via.second;
                    bus_tmp.bit_routes[i].routes.push_back(route_tmp);
                }
            }
            turn=0;
        }
        else if(turn!=0 && !find_the_solution){
            cout<<"\n\n\ninto via resolution here\n\n\n";
            turn++;
            k--;
            if(via_queue.size()>0)
            	via_queue.pop();
            if(via_queue.size()==0){
            	result.push_back(0);
            	k++;
            }
        }
        else if(!find_the_solution && k!=0 && !time_out){
            cout<<"\n\n\ninto via resolution here\n\n\n";
            turn++;
            k--;
            if(via_queue.size()>0)
            	via_queue.pop(); 
        }
        else{
            result.push_back(0);
            continue;
        }
        if (k == 0) block_pin_deactivate(B_block, bus_id, 0);
        block_pin_deactivate(B_block, bus_id, k+1);
    }
    all.path[bus_id] = bus_tmp;
    all.bus_result[bus_id] = result;
    return 0;
}

bool f_bus_router_func (All_route &all, GridMap &G, MainInput &M) {
    cout << "f-Bus Router start\n";
    BUSES_block B_block;
    //all all;
    init_pin_blockage (B_block, G, M);
    //map
    for(int i = 0 ; i < M.BusSet.size() ; i++){
        if (all.bus_result[i][0] == 1) continue;
        cout << "Re-Running Bus " << i << endl;
        bool ans_flag = 0;
        ans_flag = f_one_bus_router(all, G, M.BusSet[i], M, B_block, i );
        cout << "Finish Re-Running Bus " << i << endl;
    }
    cout << "f-Bus Router end\n";
    cout<<"the bus result is :\n";
    for(int i=0 ; i<all.bus_result.size() ; i++){
        for(int j=0 ; j<all.bus_result[i].size() ; j++){
            cout<<all.bus_result[i][j]<<" ";
        }
        cout<<endl;
    }
    return 0;
}

CC = g++ -std=c++11 -O3 -static
SRC_DIR = .

$(SRC_DIR)/cada048 : $(SRC_DIR)/main.o $(SRC_DIR)/input.o $(SRC_DIR)/gridmap.o $(SRC_DIR)/router_Astar.o $(SRC_DIR)/none_via_router.o $(SRC_DIR)/finish_router.o  $(SRC_DIR)/output.o $(SRC_DIR)/block_pin.o
	${CC} $^ -o $@
$(SRC_DIR)/main.o : $(SRC_DIR)/main.cpp $(SRC_DIR)/input.o $(SRC_DIR)/gridmap.o $(SRC_DIR)/router_Astar.o $(SRC_DIR)/output.o $(SRC_DIR)/block_pin.o 
	${CC} -c $< -o $@
$(SRC_DIR)/input.o : $(SRC_DIR)/input.cpp
	${CC} -c $< -o $@
$(SRC_DIR)/gridmap.o : $(SRC_DIR)/gridmap.cpp
	${CC} -c $< -o $@
$(SRC_DIR)/router_Astar.o : $(SRC_DIR)/router_Astar.cpp
	${CC} -c $< -o $@
$(SRC_DIR)/none_via_router.o : $(SRC_DIR)/none_via_router.cpp
	${CC} -c $< -o $@
$(SRC_DIR)/finish_router.o : $(SRC_DIR)/finish_router.cpp
	${CC} -c $< -o $@
$(SRC_DIR)/output.o : $(SRC_DIR)/output.cpp
	${CC} -c $< -o $@
$(SRC_DIR)/block_pin.o : $(SRC_DIR)/block_pin.cpp
	${CC} -c $< -o $@
clean : 
	rm -rf $(SRC_DIR)/*.o $(SRC_DIR)/cada048

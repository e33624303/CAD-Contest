CXX		=	g++ -std=c++0x -I./GDT-4.0.4 -DDEBUG -O3
EXE		=	main
SRC_DIR	=	src
OBJ_DIR	=	objs
GDT_DIR =	GDT-4.0.4

all: $(EXE)


$(EXE): $(OBJ_DIR)/main.o $(OBJ_DIR)/stoupper.o $(OBJ_DIR)/sRemoveWhiteSpace.o $(OBJ_DIR)/sRemoveTrailingZeros.o $(OBJ_DIR)/sRemoveSpaces.o $(OBJ_DIR)/sfind.o $(OBJ_DIR)/mystrncpy.o $(OBJ_DIR)/match_string.o $(OBJ_DIR)/get_field.o $(OBJ_DIR)/gdsStream.o $(OBJ_DIR)/gds2gdt.o $(OBJ_DIR)/gdt2gds.o $(OBJ_DIR)/clipper.o  
	$(CXX) $^ -o $@ -lrt

$(OBJ_DIR)/stoupper.o: $(GDT_DIR)/stoupper.C $(GDT_DIR)/kvstypes.h $(GDT_DIR)/kvsstring_c.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/sRemoveWhiteSpace.o: $(GDT_DIR)/sRemoveWhiteSpace.C $(GDT_DIR)/gcc4.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/sRemoveTrailingZeros.o: $(GDT_DIR)/sRemoveTrailingZeros.C $(GDT_DIR)/kvstypes.h $(GDT_DIR)/kvsstring_c.h $(GDT_DIR)/gcc4.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/sRemoveSpaces.o: $(GDT_DIR)/sRemoveSpaces.C $(GDT_DIR)/kvstypes.h $(GDT_DIR)/kvsstring_c.h $(GDT_DIR)/gcc4.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/sfind.o: $(GDT_DIR)/sfind.C $(GDT_DIR)/kvstypes.h $(GDT_DIR)/kvsstring_c.h $(GDT_DIR)/gcc4.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/mystrncpy.o: $(GDT_DIR)/mystrncpy.C $(GDT_DIR)/gcc4.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/match_string.o: $(GDT_DIR)/match_string.C $(GDT_DIR)/kvstypes.h $(GDT_DIR)/kvsstring_c.h $(GDT_DIR)/gcc4.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/get_field.o: $(GDT_DIR)/get_field.C $(GDT_DIR)/kvstypes.h $(GDT_DIR)/kvsstring_c.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/gdsStream.o: $(GDT_DIR)/gdsStream.C $(GDT_DIR)/gdsStream.h $(GDT_DIR)/gcc4.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/gds2gdt.o: $(GDT_DIR)/gds2gdt.C $(GDT_DIR)/gds2gdt.h $(GDT_DIR)/kvsstring_c.h $(GDT_DIR)/gdsStream.h $(GDT_DIR)/gcc4.h $(GDT_DIR)/sourceForge.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/gdt2gds.o: $(GDT_DIR)/gdt2gds_new.C $(GDT_DIR)/gdt2gds.h $(GDT_DIR)/kvsstring_c.h $(GDT_DIR)/gdsStream.h $(GDT_DIR)/gcc4.h $(GDT_DIR)/sourceForge.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/clipper.o: $(SRC_DIR)/clipper/clipper.cpp $(SRC_DIR)/clipper/clipper.hpp | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/Segment.o: $(SRC_DIR)/Segment.cpp | obj_dir
	$(CXX) -c $< -o $@



$(OBJ_DIR)/main.o: $(SRC_DIR)/main.cpp $(SRC_DIR)/RTree.h $(SRC_DIR)/clipper/clipper.hpp $(GDT_DIR)/gdt2gds.h $(SRC_DIR)/GuideParser.h | obj_dir
	$(CXX) -c $< -o $@



obj_dir:
	mkdir -p $(OBJ_DIR)


clean:
	rm -rf $(OBJ_DIR) $(EXE)


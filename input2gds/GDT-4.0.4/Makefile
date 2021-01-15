#OPT		=	-DDEBUG -g
OPT		=	-O3
CFLAGS	=	-I. -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 $(OPT) -D_GNU_SOURCE -static -L /usr/lib/x86_64-redhat-linux5E/lib64
CC		:=	$(CC) $(CFLAGS)
CXX		:=	$(CXX) $(CFLAGS)
COMMON_OBJ	=	sRemoveTrailingZeros.o get_field.o stoupper.o match_string.o sfind.o sRemoveWhiteSpace.o sRemoveSpaces.o mystrncpy.o gdsStream.o
GDS2GDT_OBJ	=	gds2gdt.o $(COMMON_OBJ)
GDT2GDS_OBJ	=	gdt2gds.o $(COMMON_OBJ)

all: gds2gdt gdt2gds

gds2gdt: $(GDS2GDT_OBJ)
	$(CXX) $^ -o $@
	
gdt2gds: $(GDT2GDS_OBJ)
	$(CXX) $^ -o $@
	
clean:
	rm -f gds2gdt gdt2gds $(GDS2GDT_OBJ) $(GDT2GDS_OBJ)

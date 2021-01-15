#ifndef _GDT2GDS__
#define _GDT2GDS__

#include <stdio.h>
#include <string>

extern char* mystrncpy(char *dest, char *src, size_t n);
extern char* sRemoveTrailingZeros(char* parent, char* child);
extern void sStdWhiteSpace(char* parent, int isText);
extern void rmLeadingAndTrailingSpace(char* parent);
extern char* sRemoveWhiteSpace(char* parent, char* child);
extern char* sUnescapedString(char* parent, char* child);
//extern void print_help();
extern int getUnescapedPropString(char* parent, char* child, int position);
extern int getUnescapedTextString(char* parent, char* child, int position);
extern int getField(char* string, char* field_string, int position);
extern int sIsPosInt(char* string);
extern int gdt2gds(std::string ,std::string );

#endif

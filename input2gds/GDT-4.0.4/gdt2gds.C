/*
 ******************************************************************************
 * Copyright 2003-2014 by Ken Schumack
 *
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author:
 *           Ken Schumack (Schumack@cpan.org)
 *
 * Suggestions for improvements may be sent to Ken Schumack.
 ******************************************************************************
*/
// @(#) $Id: gdt2gds.C 96 2014-12-04 22:03:27Z schumack $
#include <stdio.h>
#include <math.h>
#include <gdsStream.h>
#include <kvsstring_c.h>
//#include <iostream.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
//#include <ostream.h>
#include <ostream>
using namespace std;
#include <gcc4.h>
#include <sourceForge.h>

#define LINELENGTH 1000000 //gdt lines can be this long
#define ERRORSTRING "ERROR:  "
#define NUMPROPS 30 //can handle this many props on an element

extern char* mystrncpy(char *dest, char *src, size_t n);
extern char* sRemoveTrailingZeros(char* parent, char* child);
extern void sStdWhiteSpace(char* parent, int isText);
extern void rmLeadingAndTrailingSpace(char* parent);
extern char* sRemoveWhiteSpace(char* parent, char* child);
extern char* sUnescapedString(char* parent, char* child);
extern void print_help();
extern int getUnescapedPropString(char* parent, char* child, int position);
extern int getUnescapedTextString(char* parent, char* child, int position);
extern int getField(char* string, char* field_string, int position);
extern int sIsPosInt(char* string);

//////////////////////////////////////////////////////////////////////////////
// M A I N
//////////////////////////////////////////////////////////////////////////////
/***** GET FIELD *************************************************************/
/* Returns position of string passed in which ends the field starting at the */
/* position passed in with the integer variable. The field string is updated */
/* to the new field.                                                         */
/* ie.                                                                       */
/*      for (n = numfields(string), i=0; n>0; n--) {                         */
/*          i = getField(string, name, i);                                   */
/*          printf("%s\n", name);                                            */
/*      }                                                                    */
/* or                                                                        */
/*      i=0                                                                  */
/*      while ((i=getField(string, name, i)) != ERR) printf("%s\n", name);   */
/*                                                         -KVS              */
/*****************************************************************************/
int getField(char* string, char* field_string, int position)
{
    int     i, j, k;

    for(i=position; (string[i] != '\0') &&   (isspace(string[i])); i++) ;  /* skip white space */
    for(j=i, k=0;  ((string[j] != '\0') && (!(isspace(string[j])))); j++, k++) field_string[k] = string[j];
    field_string[k] = '\0';

    if(string[i] == '\0') return(ERR);
    else return(j);
}

/*****************************************************************************/
int sIsPosInt(char* string)
{
    int i,len;
    len = strlen(string);
    for(i=0;i<len;i++)
    {
        if (! isdigit(string[i])) return(0);
    }
    return(1);
}

/***** rmLeadingAndTrailingSpace ********************************************/
/* Removes all occurances of white space at beginning of a string    -kvs   */
/****************************************************************************/
void rmLeadingAndTrailingSpace(char* parent)
{
    int childIndex = 0,
        parentIndex = 0,
        lastChar = 0,
        length = 0;
    char c;
    static char child[LINELENGTH];

    if (parent[0] != '#')
    {
        length = strlen(parent);
        lastChar = length;
        for(childIndex=0; childIndex<=lastChar; childIndex++)
        {
            child[childIndex] = '\0';
        }
        childIndex = 0;
        //find 1st non white space character position
        while (isspace(parent[parentIndex])) parentIndex++;
        //find last non white space character position
        while (isspace(parent[lastChar]) || (parent[lastChar] == '\0')) lastChar--;

        for(childIndex=0; parentIndex<=lastChar; parentIndex++)
        {
            c = parent[parentIndex];
            child[childIndex++] = c;
        }
        child[childIndex] = '\0';
        strcpy(parent,child);
    }
}

/***** sStdWhiteSpace *******************************************************/
/* Removes all occurances of white space fore and aft in a string    -kvs   */
/* and standardize to one space between fields                              */
/****************************************************************************/
void sStdWhiteSpace(char* parent, int isText)
{
    int childIndex = 0,
        parentIndex = 0,
        lastChar = 0,
        inSpace = 0,
        length = 0,
        okToModify = 1;
    char c;
    static char child[LINELENGTH];

    if (parent[0] != '#')
    {
        length = strlen(parent);
        lastChar = length;
        for(childIndex=0; childIndex<=lastChar; childIndex++)
        {
            child[childIndex] = '\0';
        }
        childIndex = 0;
        //find 1st non white space character position
        while (isspace(parent[parentIndex])) parentIndex++;
        //find last non white space character position
        while (isspace(parent[lastChar]) || (parent[lastChar] == '\0')) lastChar--;

        for(childIndex=0; parentIndex<=lastChar; parentIndex++)
        {
            c = parent[parentIndex];
            if (isspace(c)) inSpace++;
            else            inSpace = 0;
            if (! inSpace)
            {
                if (isText && (c == '\''))
                {
                    if (okToModify) // the first ' of a text string
                    {
                        okToModify = 0;
                    }
                    else // the last ' of a text string
                    {
                        okToModify = 1;
                    }
                }
                // add space before ) or } if not there already
                if (
                    okToModify &&
                    ((c == ')') || (c == '}')) &&
                    (parentIndex > 0) &&
                    (parent[(parentIndex - 1)] != ' ')
                )
                {
                    child[childIndex++] = ' ';
                }

                child[childIndex] = c;

                // add space after '(' if not there already
                if ((c == '(') && (parent[(parentIndex + 1)] != ' '))
                {
                    child[++childIndex] = ' ';
                }
                childIndex++;
            }
            else if (inSpace == 1) //first space only
            {
                child[childIndex++] = ' ';
                inSpace++;
            }
        }
        child[childIndex] = '\0';
        strcpy(parent,child);
    }
}

/***** getUnescapedPropString *****************************************************/
/* remove enclosing 's and escape characters                         -kvs   */
/****************************************************************************/
int getUnescapedPropString(char* parent, char* child, int position)
{
    int i, j;
    Boolean done = FALSE;

    for(i=(position + 1), j=0; (! done); i++)
    {
        if (parent[i] == '\0')
        {
            done = TRUE;
        }
        else if ((parent[i] == '\'') && (parent[(i - 1)] != '\\') && (parent[(i + 1)] == ' ') && (parent[(i + 2)] == '}'))
        {
            done = TRUE;
            i+=2;
        }
        else
        {
            if (parent[i] == '\\') i++;
            child[j++] = parent[i];
        }
    }
    child[j] = '\0';
    return(i);
}

/***** getUnescapedTextString ***********************************************/
/* remove enclosing 's and escape characters                         -kvs   */
/****************************************************************************/
int getUnescapedTextString(char* parent, char* child, int position)
{
    int i, j;
    Boolean done = FALSE;

    for(i=(position + 1), j=0; (! done); i++)
    {
        if (parent[i] == '\0')
        {
            done = TRUE;
        }
        else if ((parent[i] == '\'') &&
                (parent[(i - 1)] != '\\') &&
                ((parent[(i + 1)] == ' ') || (parent[(i + 1)] == '}'))
        )
        {
            done = TRUE;
            i++;
        }
        else
        {
            if (parent[i] == '\\') i++;
            child[j++] = parent[i];
        }
    }
    child[j] = '\0';
    return(i);
}

/***** sUnescapedString *****************************************************/
/* remove enclosing 's and escape characters                         -kvs   */
/****************************************************************************/
char* sUnescapedString(char* parent, char* child)
{
    int i, j, k;

    k = strlen(parent);
    for(i=1, j=0; i<k; i++)
    {
        if (parent[i] == '\\') i++;
        child[j] = parent[i];
    }
    child[j] = '\0';
    return(child);
}

// ****************************************************************************
// * print_help()
// ****************************************************************************


/*

__END__
use pod2html to make web page help
use pod2man to make man page

=pod

=head1 NAME

gdt2gds - tool to convert textual gdt format to binary gds2

=head1 SYNOPSIS

gdt2gds [options] [inputFile] [outputFile]

=head1 OPTIONS

Note: options are case-insensitive and can be shortened as long they remain unique.

  -help
    print this and exit

  -inFilename <fileName>
    default is STDIN
    also filename of '-' means STDIN

  -outFilename <fileName>
    default is STDOUT
    also filename of '-' means STDOUT

  -precision <integer>
    Currenly a NOOP. Only included to be symmetric with gds2gdt

  -version
    print version of program and quit

=head1 EXAMPLES

gdt2gds test.gdt test.gds

cat test.gdt | gdt2gds | gzip > test.gds.gz

=cut

*/

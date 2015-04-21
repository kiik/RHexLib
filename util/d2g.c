/*
 * This file is part of RHexLib, 
 *
 * Copyright (c) 2001 The University of Michigan, its Regents,
 * Fellows, Employees and Agents. All rights reserved, and distributed as
 * free software under the following license.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * 1) Redistributions of source code must retain the above copyright
 * notice, this list of conditions, the following disclaimer and the
 * file called "CREDITS" which accompanies this distribution.
 * 
 * 2) Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions, the following disclaimer and the file
 * called "CREDITS" which accompanies this distribution in the
 * documentation and/or other materials provided with the distribution.
 * 
 * 3) Neither the name of the University of Michigan, Ann Arbor or the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*********************************************************************
 * $Id: d2g.c,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * d2g <filename>
 *
 * Takes a *.dsc file ( see sm-setup.pl for the format ) and outputs a
 * *.daVinci file for the daVinci graph layout program.
 *
 * Created       : Eric Klavins, 06/03/2001
 * Last Modified : Eric Klavins, 06/03/2001
 *
 ********************************************************************/

#include <stdio.h>
#include <stdlib.h>

// a boolean type
typedef enum { false=0, true=1 } BOOL;

// a node in the graph
typedef struct node {

  char name[100];            // name of the node
  struct edge * elist;       // list of edgest
  struct node * next;        // next node in a linked list of all nodes
  BOOL isinit;               // is this the initial node?
  

} NODE;

// an edge in the graph
typedef struct edge {

  // name of the edge and the names of its source and destination
  // nodes
  char name[100], srcname[100], destname[100];

  // pointers to the source and dest nodes
  struct node * src, * dest;

  // next edge in list of all edges and next edge in a node's list of edges
  struct edge * next, * next2;

} EDGE;


// returns a new node
NODE * NewNode ( char * name ) {

  NODE * n = (NODE *) malloc ( sizeof ( NODE ) );

  if ( n != NULL ) {

    n->isinit = false;
    strcpy ( n->name, name );
    n->elist = NULL;
    n->next = NULL;

  }

  return n;

}

// returns a new edge
EDGE * NewEdge ( char * name, char * from, char * to ) {

  EDGE * e = (EDGE *) malloc ( sizeof ( EDGE ) );

  if ( e != NULL ) {

    strcpy ( e->name, name );
    strcpy ( e->srcname, from );
    strcpy ( e->destname, to );
    e->src = NULL;
    e->dest = NULL;
    e->next = e->next2 = NULL;

  }

  return e;

}

// returns the first node with name in a list of nodes pointed to by head 
//   or returns NULL if there is no such node
NODE * LookUp ( NODE * head, char * name ) {

  NODE * temp = head;

  while ( temp != NULL ) {

    if ( !strcmp ( temp->name, name ) ) 
      return temp;

    temp = temp->next;

  }

  return NULL;

}

// adds a node with name to the list pointed to by head
void AddNode ( NODE ** head, char * name ) {

  if ( LookUp ( *head, name ) == NULL ) {

    if ( *head == NULL ) {

      *head = NewNode ( name );

    } else {
   
      NODE * temp = NewNode ( name );
      temp->next = *head;
      *head = temp;

    }

  }

}

// adds an edge to the list of edges pointed to by head (uses the next
// pointer)
void AddEdge ( EDGE ** head, char * name, char * from, char * to ) {

  if ( *head == NULL ) {

    *head = NewEdge ( name, from, to );

  } else {

    EDGE * temp = NewEdge ( name, from, to );
    temp->next = *head;
    *head = temp;  

  }

}

// prints out a simple representation of the graph, for debugging
void PrintSimple ( NODE * head ) {

  NODE * ntemp = head;
  EDGE * etemp = NULL;

  printf ( "\n" );

  while ( ntemp != NULL ) {

    if ( ntemp->isinit ) printf ( "  *" );
    else printf ( "  " );
    printf ( "%s:\n", ntemp->name );
    etemp = ntemp->elist;

    while ( etemp != NULL ) {
      printf ( "    --%s--> %s\n", etemp->name, etemp->dest->name );
      etemp = etemp->next2;
    }

    ntemp = ntemp->next;

  }

  printf ( "\n" );

}

// prints out the graph using daVinci's cryptic term representation language
void PrintDaVinci ( NODE * head ) {

  NODE * ntemp = head;
  EDGE * etemp = NULL;
  int count = 0;

  printf ( "[\n" );

  while ( ntemp != NULL ) {

    if ( ntemp->isinit ) 
   
      printf ( "l(\"%s\",n(\"\",[a(\"OBJECT\",\"%s\"), a(\"COLOR\", \"#00eebb\"), a(\"FONTFAMILY\", \"times\" ) ],[",
           ntemp->name, ntemp->name );

    else

      printf ( "l(\"%s\",n(\"\",[a(\"OBJECT\",\"%s\"), a(\"COLOR\", \"#bbbbbb\"), a(\"FONTFAMILY\", \"times\" ) ],[",
           ntemp->name, ntemp->name );
 
    // process edges here
    etemp = ntemp->elist;

    while ( etemp != NULL ) {

      printf ( "l ( \"%s->%s\", e ( \"\", [ a(\"_DIR\", \"none\")  ], 
       l ( \"A\%d\", n ( \"\", [ a ( \"OBJECT\", \"%s\" ), a (\"_GO\", \"text\" ), a(\"FONTFAMILY\", \"times\" ), a(\"FONTSTYLE\", \"italic\" ) ],
          [
            l ( \"%s->%s\", e ( \"\", [], r(\"%s\") ) )
          ] ))))", ntemp->name, etemp->name, count++, etemp->name, etemp->name, 
         etemp->dest->name, etemp->dest->name );

      if ( etemp->next2 != NULL ) printf ( "," );
      printf ( "\n" );

      etemp = etemp->next2;

    }

    printf ( "]))" );
   
    if ( ntemp->next != NULL ) printf ( "," );
    printf ( "\n" );
    ntemp = ntemp->next;

  }

  printf ( "]" );

}

// reads a graph in from the description file and sets up the graph
// data structure. returns a pointer to the structure
NODE * ReadDscFile ( char * path ) {

  FILE * fp;
  char str[40], from[40], trans[40], to[40];
  NODE * nlist = NULL;
  EDGE * elist = NULL;

  fp = fopen ( path, "r" );

  if ( fp == NULL ) {

    fprintf ( stderr, "Can't open %s\n", path );
    return NULL;

  }

  while ( ! feof ( fp ) ) {

    fscanf ( fp, "%s", str );

    if ( ! strcmp ( str, "Transition" ) ) {

      fscanf ( fp, "%s", from );
      fscanf ( fp, "%s", trans );
      fscanf ( fp, "%s", to );

      AddNode ( &nlist, from );
      AddNode ( &nlist, to );
      AddEdge ( &elist, trans, from, to );

    } 

    else if ( ! strcmp ( str, "Initial" ) ) {

      NODE * temp;

      fscanf ( fp, "%s", str );
      temp = LookUp ( nlist, str );
      temp->isinit = true;

    }

  }

  fclose ( fp );

  // resolve node pointers from names in transitions
  {

    EDGE * etemp = elist;
    NODE * src = NULL;

    while ( etemp != NULL ) {

      etemp->src = LookUp ( nlist, etemp->srcname );
      etemp->dest = LookUp ( nlist, etemp->destname );

      src = etemp->src;
      
      if ( src->elist == NULL ) {

        src->elist = etemp;
        etemp->next2 = NULL;

      } else {

        etemp->next2 = src->elist;
        src->elist = etemp;

      }

      etemp = etemp->next;

    }

  }

  return nlist;

}

int main ( int argc, char * argv[] ) {

  // read in description file
  NODE * nlist = ReadDscFile ( argv[1] );

  // print out graph
  PrintDaVinci ( nlist ); 

  return 0;

}













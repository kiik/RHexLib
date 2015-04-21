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
 * $Id: DiscreteSystem.hh,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * DiscreteSystem class
 *
 * Created       : Haldun Komsuoglu, 01/18/2001
 * Last Modified : Haldun Komsuoglu, 06/27/2001
 *
 ********************************************************************/

#ifndef _DISCRETESYSTEM_HH
#define _DISCRETESYSTEM_HH


#include "Buffer.hh"


/*---------------------------------------------------------------------*/
/* Module specific types ----------------------------------------------*/
/*---------------------------------------------------------------------*/

/* Parameter types list */
typedef enum { INPUT_BUFFER, 
               OUTPUT_BUFFER,
               INPUT_COEFF,
               OUTPUT_COEFF, 
               SUBSAMPLE_RATIO } DSParamType;

// Continues in the end of the header...




/*---------------------------------------------------------------------*/
/* The DiscreteSystem class -------------------------------------------*/
/*---------------------------------------------------------------------*/
class DiscreteSystem{

 private:

  Filter_Buffer     x;  // Input buffer
  Filter_Buffer     y;  // Output buffer
  Filter_Buffer     a;  // Coefficients for inputs, x
  Filter_Buffer     b;  // Coefficients for outputs, y

  int               SubSampleRate;    // Subsampling rate given in samples
                                      // wrt the base samping frequency
  int               SubSampleCounter; // Subsampling counter variable

 public:

  // Default filter constructor (generates an identity system)
  DiscreteSystem( void );

  // (Re)Builds the sub data structures for a discrete system with 
  // specified input and output buffer lenghts
  void BuildStructure( int, int );

  // Sets the paramters of the difference equation describing the
  // filter
  void SetParam( DSParamType, double[], int );

  // Resets the input & output buffers
  void ResetBuffers( void ) { x.Redefine( x.size() );
                              y.Redefine( y.size() );};

  // Gets a new input sample, processes the difference equation and
  // updates the corresponding buffers.
  void Process( double );

  // Returns the output of the filter
  double output() { return y.lastEntry(); };

  // Displays the current buffer and coefficient list with a specified
  // discrete system name
  void display( char* Dname ) {
    char str[128];
    sprintf( str, "%s -> a", Dname );
    a.display( str );
    sprintf( str, "%s -> x", Dname );
    x.display( str );
    sprintf( str, "%s -> b", Dname );
    b.display( str );
    sprintf( str, "%s -> y", Dname );
    y.display( str );
    printf("\n");
  };

  // Displays the current buffer and coefficient list with a default name
  void display( void ) { display( "S" ); };


};


/*---------------------------------------------------------------------*/
/* Module specific types (cont'd) -------------------------------------*/
/*---------------------------------------------------------------------*/

/* One way linked filter linked list record */
struct DiscreteSystemLinkedList{
  DiscreteSystem             *sys;
  DiscreteSystemLinkedList   *next;
};


#endif


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
 * $Id: Buffer.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * 1-D signal processing buffer class (derived from LinkedList1D)
 *
 * Created       : Haldun Komsuoglu, 01/21/2001
 * Last Modified : Haldun Komsuoglu, 06/27/2001
 *
 ********************************************************************/

#include "Buffer.hh"


//----------------------------------------------------------------------
//+++++++++++++++++ Filter_Buffer (FILO) Class +++++++++++++++++++++++++
//----------------------------------------------------------------------


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Filter_Buffer::MultiplySum( B1, B2 )
//
//   Multiplies the entries of two buffers element-by-element starting from
//   their last entered values and returns the total of the results
//
//   B1     : (Filter_Buffer&) Buffer #1
//   B1     : (Filter_Buffer&) Buffer #2
//   offset : (int) the offset for shifting the buffers with respect to each
//            other.
//
//   return : (double) The total of the element by element multiplication
//            of the entries of B1 and B2
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double MultiplySum( const Filter_Buffer& B1, 
                    const Filter_Buffer& B2,
                    const int            offset ) {
  
  double  *B1pntr;
  double  *B2pntr;
  double   total  = 0;
  int      i;
  
  if ( B1.size() != B2.size() ) {
    printf( "Error: (Buffer.cc) Multiplication of buffers of dirrent size...");
    return 0;
  }
  
  if (B1pntr == NULL) return 0;

  B2pntr = B2.nextEntry( B2.IOpntr );
  B1pntr = B1.nextEntry( B1.IOpntr );
  for (i = 0 ; i < offset ; i++)
    B2pntr = B2.nextEntry( B2pntr );

  for (i = 0 ; i < B1.size() ; i++) {
    total = total + (*B1pntr * *B2pntr);
    B1pntr = B1.nextEntry( B1pntr );
    B2pntr = B2.nextEntry( B2pntr );
  }

  return total;

};
//----------------------------------------------------------------------



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// filter_Buffer::display( char* Bname)
//
//   Displays the buffer starting from the last entry moveing towards the
//   end of it.
//
//   Bname  : (char*) Null terminated string that indicates the name of the
//            variable
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void Filter_Buffer::display( char* Bname) {

  double  *B;
  int     i = 0;

  if (Head == NULL) return;

  printf( "%s =", Bname );
  B = IOpntr;
  for (i = 0 ; i < N ; i++) {
    printf("[%d]( % f )  ", i, *B);
    B = nextEntry( B );
  }
  printf("\n");

};
//----------------------------------------------------------------------


//----------------------------------------------------------------------
//++++++++++++++++++++++++++ FIFO Class ++++++++++++++++++++++++++++++++
//----------------------------------------------------------------------


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// FIFO::display( char* Bname) 
//
//   Displays the buffer starting from the last entry moveing towards the
//   end of it.
//
//   Bname  : (char*) Null terminated string that indicates the name of the
//            variable
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void FIFO_Buffer::display( char* Bname) {

  double  *B;
  int     i = 0;

  if (Head == NULL) return;

  printf( "%s =", Bname );
  B = READpntr;
  for (i = 0 ; i < N ; i++) {
    printf("[%d]( % f )  ", i, *B);
    B = nextEntry( B );
  }
  printf("\n");

};
//----------------------------------------------------------------------

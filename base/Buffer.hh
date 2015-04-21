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
 * $Id: Buffer.hh,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * 1-D buffer class
 *
 * Created       : Haldun Komsuoglu, 01/27/2001
 * Last Modified : Haldun Komsuoglu, 06/27/2001
 *
 ********************************************************************/

#ifndef _BUFFER_HH
#define _BUFFER_HH

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define ELEMENT_SIZE (sizeof( double ))

//---------------------------------------------------------------------
// The Buffer base class ----------------------------------------------
//---------------------------------------------------------------------
class Buffer  {

public:

  // --- Constructor / Destructor definitions

  // Default constructor
  Buffer( void ) { 
    N    = 0; 
    Head = NULL;
    End  = NULL;
  };

  // Buffer constructor with a given buffer length. Buffer is set to zero.
  Buffer( const int M ) {
    N    = M;
    Head = new double[ M ];
    End  = Head + N - 1;
    memset( Head, 0, M * ELEMENT_SIZE );
  };

  // Buffer constructor with element value initialization
  Buffer( const double a[], const int M) {
    N    = M;
    Head = new double[ M ];
    End  = Head + N - 1;
    memcpy( Head, a, M * ELEMENT_SIZE );
  };

  // Destructor
  ~Buffer( void ) {
    if ( Head != NULL ) delete[] Head;
    N    = 0;
    Head = NULL;
    End  = NULL;
   };
  
  // --- Basic Buffer functionality

  // Returns the value stored in the entry specified by the index
  // with respect to the physical begining of the buffer
  double getEntry( int index ) const { 
    if ((index >= N) || (index < 0)) {
      printf( "Error: (Buffer.hh) Index exceeds buffer size" );
      return 0;
    }
    return *(Head + index);
  };


  // Sets the value of the entry specified by the index
  // with respect to the physical begining of the buffer
  void setEntry( int index, double val ) { 
    if ((index >= N) || (index < 0)) {
      printf( "Error: (Buffer.hh) Index exceeds buffer size" );
      return;
    }
    double * Pntr = (Head + index);
    *Pntr = val;
  };


  // Redefines a buffer (like a constructor)
  void Redefine( const int M) {
    if ( Head != NULL ) delete[] Head;
    N    = M;
    Head = new double[ M ];
    End  = Head + N - 1;
    memset( Head, 0, M * ELEMENT_SIZE );
  };

  // Fills the buffer with the given array of values
  void Fill( const double a[], const int M) {
    if (M != N) {
      printf( "Error: (Buffer.hh) Array size does not match the buffer size..." );
      return;
    }
    memcpy( Head, a, M*ELEMENT_SIZE );
  };

  // Returns the size of the buffer in doubles
  int size( void ) const { return N; };

  // Buffer equation operator: recreates this buffer with the exact structure
  // of the given buffer
  void operator= ( const Buffer& Bequ) {
    if ( length_compare( Bequ ) )
      memcpy( Head, Bequ.Head, Bequ.size() * ELEMENT_SIZE );
    else {
      if ( Head != NULL ) delete[] Head;
      Head = new double[ Bequ.size() ];
      N = Bequ.size();
      End  = Head + N - 1;
      memcpy( Head, Bequ.Head, Bequ.size()*ELEMENT_SIZE );
    }
  };

protected:

  int        N;     // Buffer length
  double   * Head;  // Head of the physical memory block allocated for the
                    // buffer
  double   * End;   // The address of the memory location right after
                    // the last location in the buffer memory block

private:

  // Compares the size of a given buffer with that of this buffer and
  // returns 0 if not equal and 1 if equal
  int length_compare( const Buffer& Bcmp) { return (Bcmp.size() == N); };

};



//---------------------------------------------------------------------
// The Filter_Buffer class --------------------------------------------
//---------------------------------------------------------------------
class Filter_Buffer : public Buffer  {

public:
  
  // --- Constructor / Destructor definitions

  // Default constructor
  Filter_Buffer( void ) : Buffer() {
    IOpntr = NULL;
  };

  // Buffer constructor with a given buffer length. Buffer is set to zero.
  Filter_Buffer( const int M ) : Buffer( M ) {
    IOpntr = Head;
  };

  // Buffer constructor with element value initialization
  Filter_Buffer( const double a[], const int M ) : Buffer( a, M ) {
    IOpntr = Head;
  };

  // --- FILO specific functionality

  // Enters new value to the buffer
  void push( double val ) {
    if (IOpntr == NULL) {
      printf( "Error: (Buffer.hh) Pushing into a zero size buffer..." );
      return;
    } 
    *IOpntr = val;
    IOpntr  = prevEntry( IOpntr );
  };

  // Redefines a buffer (like a constructor)
  void Redefine( const int M) {
    Buffer::Redefine( M );
    IOpntr = Head;
  };

  // Multiplies element-by-element two same size filter buffers and returns 
  // the total sum of the results
  friend double MultiplySum( const Filter_Buffer&, const Filter_Buffer&, const int );

  // Returns the last value entered to the buffer
  double lastEntry( void ) { return *nextEntry(IOpntr); };

  // Displays the buffer with the given name indicated
  void display( char* );

  // Displays the buffer with the given name indicated
  void display( void ) { display( "Buffer :" ); };


protected:

  // Determines the pointer to the next entry in the circular filter buffer
  double *nextEntry( double* curr ) const {
    curr++;
    if (curr > End)
      curr = Head;
    return curr;
  };

  // Determines the pointer to the next entry in the circular filter buffer
  double *prevEntry( double* curr ) const {
    curr--;
    if (curr < Head)
      curr = End;
    return curr;
  };


private:

  double   *IOpntr;    // Pointer to the entry where the push/pop (I/O)
                       // will happen

};



//---------------------------------------------------------------------
// The FIFO Buffer class ----------------------------------------------
//---------------------------------------------------------------------
class FIFO_Buffer : public Buffer  {

public:
  
  // --- Constructor / Destructor definitions

  // Default constructor
  FIFO_Buffer( void ) : Buffer() {
    WRITEpntr = NULL;
        READpntr  = NULL;
  };

  // Buffer constructor with a given buffer length. Buffer is set to zero.
  FIFO_Buffer( const int M ) : Buffer( M ) {
    WRITEpntr = Head;
        READpntr  = Head;
  };

  // Buffer constructor with element value initialization
  FIFO_Buffer( const double a[], const int M ) : Buffer( a, M ) {
    WRITEpntr = Head;
        READpntr  = Head;
  };

  // --- FIFO specific functionality

  // Enters new value to the buffer
  void push( double val ) {
    if (WRITEpntr == NULL) {
      printf( "Error: (Buffer.hh) Pushing into a zero size buffer..." );
      return;
    } 
    *WRITEpntr = val;
    WRITEpntr  = nextEntry( WRITEpntr );
  };

  // Returns the next value from the buffer
  double *pop( void ) {
    double * dummy;
        if (READpntr == WRITEpntr) {
      return NULL;
    } 
        dummy     = READpntr;
    READpntr  = nextEntry( READpntr );
        return dummy;
  };


  // Redefines a buffer (like a constructor)
  void Redefine( const int M) {
    Buffer::Redefine( M );
    WRITEpntr = Head;
        READpntr  = Head;
  };

  // Determines if the buffer is empty or not
  int isEmpty( void ) { return (READpntr == WRITEpntr); };

  // Displays the buffer with the given name indicated
  void display( char* );

  // Displays the buffer with the given name indicated
  void display( void ) { display( "Buffer :" ); };


protected:

  // Determines the pointer to the next entry in the circular filter buffer
  double *nextEntry( double* curr ) const {
    curr++;
    if (curr > End)
      curr = Head;
    return curr;
  };

  // Determines the pointer to the next entry in the circular filter buffer
  double *prevEntry( double* curr ) const {
    curr--;
    if (curr < Head)
      curr = End;
    return curr;
  };


private:

  double  * WRITEpntr;  // Pointer to the entry where the push (write)
                        // will happen
  double  * READpntr;   // Pointer to the entry where the pop (read)
                        // will happen

};



#endif


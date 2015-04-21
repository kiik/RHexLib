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
 * $Id: Floats.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * Implementation for Floats, a utility class
 *
 * Created       : Laura McWilliams, 06/29/2001
 * Last Modified : Laura McWilliams, 06/29/2001
 *
 ********************************************************************/

#include "Floats.hh"
#include "ModuleManager.hh"

// implementations for Floats class------------------------------------------

// c-tor w/ count as parameter---------------------------------------------
Floats::Floats( int in_count ) : count( in_count ){
  if( ( array = new float[ in_count ] ) == NULL )
    MMFatalError( "Floats::Floats", "new could not allocate enough memory" );
}


// c-tor with array of floats as parameter
Floats::Floats( float* in_array, int size ) : count( size ){
  int i;
  
  if( ( array = new float[ size ] ) == NULL )
    MMFatalError( "Floats::Floats", "new could not allocate enough memory" );

  for( i = 0; i < count; i++ )
    array[i] = in_array[i];
}


// copy c-tor
Floats::Floats( const Floats& in ){
  count = in.getCount();
  int i;

  if( ( array = new float[ count ] ) == NULL )
    MMFatalError( "Floats::Floats", "new could not allocate enough memory" );
  
  for( i = 0; i < count; i++ )
    array[i] = in.get(i);
}//---------------------------------------------------------------------------


// d-tor----------------------
Floats::~Floats( void ){
  if( array ) delete[] array;
}


// assignment operator-----------------------------------------------------
Floats& Floats::operator=( const Floats& rhs ){
  if( this == &rhs ) return *this;    // avoid self-assignment
  int i;

  if( array ) delete[] array;
  
  count = rhs.getCount();
  if( (array = new float[ count ]) == NULL )
    MMFatalError( "Floats::operator=", "new could not allocate memory" );
  
  for( i = 0; i < count; i++ )
    array[i] = rhs.get(i);
  return *this;
}


// get function-------------------------------------------
float Floats::get( int index ) const{
  if( index >= count || index < 0 )
    MMFatalError( "Floats::get", "Index out of range!" );
  return array[ index ];
}

void Floats::set( int index, float f ){
  if( index >= count || index < 0 )
    MMFatalError( "Floats::set", "Index out of range!" );
  array[index] = f;
}//-------------------------------------------------------







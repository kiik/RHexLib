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
 * $Id: Strings.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * Implementation for Strings, a utility class
 *
 * Created       : Laura McWilliams, 06/29/2001
 * Last Modified : Laura McWilliams, 06/29/2001
 *
 ********************************************************************/

#include "Strings.hh"
#include "ModuleManager.hh"


// utility functions--------------------------------------------------

// mystrdup- same as old C function, but uses new to allocate
// returns NULL if new fails
static char* mystrdup( char* in )
{
  size_t csize;
  char *zret;

  csize = strlen ( in ) + 1;
  zret = new char[ csize ];
  if( zret ) memcpy (zret, in, csize);
  return zret;
}


//Strings implementations----------------------------------------------------

// c-tor with number of strings as parameter--------------------------------
Strings::Strings( int in_count ) : count( in_count ){
  if( ( array = new char*[ in_count ] ) == NULL )
    MMFatalError( "Strings::Strings", 
                  "new failed to allocate enough memory" );
  memset( array, 0, count * sizeof( char* ) );
}


// c-tor with array of char*s as parameter
Strings::Strings( char** in_array, int size ) : count( size ){
  int i;

  if( (array = new char*[ size ] ) == NULL )
    MMFatalError( "Strings::Strings", 
                  "new could not allocate enough memory" );

  for( i = 0; i < count; i++ ){
    if( (array[i] = mystrdup( in_array[i] )) == NULL )
      MMFatalError( "Strings::Strings", 
                    "new could not allocated enough memory" );
  }
}


// copy c-tor
Strings::Strings( const Strings& in ){
  int i;
  count = in.getCount();

  if( (array = new char*[ count ] ) == NULL )
    MMFatalError( "Strings::Strings", 
                  "new could not allocate enough memory" );

  for( i = 0; i < count; i++ ){
    if( (array[i] = mystrdup( in.get(i) )) == NULL )
      MMFatalError( "Strings::Strings", 
                    "new could not allocate enough memory");
  }
}//------------------------------------------------------------------------


// d-tor-----------------------
Strings::~Strings( void ){
  int i;
  for( i = 0; i < count; i++ )
    if( array[i]) delete[] array[i];
  if( array ) delete[] array;
}//----------------------------


// assignment operator---------------------------------------------
Strings& Strings::operator=( const Strings& rhs ){
  if( this == &rhs ) return *this; // prevent self-assignment

  int i;
  for( i = 0; i < count; i++ )
    if( array[i] ) delete array[i];
  if( array ) delete[] array;

  count = rhs.getCount();

  if( ( array = new char*[ count ]) == NULL )
    MMFatalError( "Strings::operator=", "new cannot allocate enough memory" );
  for( i = 0; i < count; i++ )
    array[i] = mystrdup( rhs.get(i) );

  return *this;
}

// get function- gets the string at particular index----------
char* Strings::get( int index ) const{
  if( index >= count || index < 0 )
    MMFatalError( "Strings::get", "Index is out of range!" );
  return array[index];
}

// set function- sets the string at a particular index to s
void Strings::set( int index, char* s ){
  if( index >= count || index < 0 )
    MMFatalError( "Strings::set", "Index is out of range!" );
  if( array[index] ) delete[] array[index];
  if( (array[index] = mystrdup( s )) == NULL )
    MMFatalError( "Strings::set", "mystrdup failed" );
}//-----------------------------------------------------------




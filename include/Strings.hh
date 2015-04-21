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
 * $Id: Strings.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Header file for Strings, a utility class
 *
 * Created       : Laura McWilliams, 06/29/2001
 * Last Modified : Laura McWilliams, 06/29/2001
 *
 ********************************************************************/

#ifndef _STRINGS_HH
#define _STRINGS_HH

#include <stdio.h>

//----------------------------------------------------------------------------
class Strings {
public:
  Strings( int in_count );           // c-tors
  Strings( char** in_array, int size );
  Strings( void ) : count( 0 ), array( NULL ) {}

  Strings( const Strings& in );            // copy c-tor
  ~Strings( void );                  // d-tor

  Strings &operator=( const Strings& rhs ); 

  char* get( int index ) const;      // return string at given index
  void set( int index, char* s );    // set string at index
	

  int getCount( void ) const { return count; }
  char **getArray( void ) const	{ return array; }

private:
  int          count;                // number of strings in the Stringz
  char**       array;

};// end of Strings class declaration-----------------------------------------

#endif

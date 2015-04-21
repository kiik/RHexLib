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
 * $Id: sysutil.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * RHexLib OS utility functions.
 *
 * This file implements the functions defined in sysutil.hh
 *
 * Created       : Uluc Saranli, 01/16/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// System includes
#ifdef _LINUX_
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#endif

// Local includes
#include "sysutil.hh"


#ifdef _LINUX_ // Need to implement the kbhit() functionality

// keyboardIO class. Used to implement the kbhit() function
class keyboardIO {

private:
  struct termios origTerm, newTerm;
  int peek;

public:
  keyboardIO( void );
  ~keyboardIO();

  int kbhit( void );
  int getch( void );

};

static keyboardIO globalKBIO;

// keyboardIO::keyboardIO : Sets stdin appropriately for kbhit() functionality
keyboardIO::keyboardIO( void ) {

  peek = -1;

  // Save the old terminal properties
  tcgetattr( 0, &origTerm );
  newTerm = origTerm;

  newTerm.c_lflag &= ~ICANON; //Disable canonical mode.(Do not buffer by line )
  newTerm.c_lflag &= ~ECHO;   // Do not echo input characters

  // uluc: I dont think we need this. Removed Jan 16, 2001
  //  newTerm.c_lflag &= ~ISIG;   //Do not generate signals on INTR, QUIT, etc.

  newTerm.c_cc[ VMIN ] = 1;
  newTerm.c_cc[ VTIME ] = 0;

  tcsetattr( 0, TCSANOW, &newTerm );
}

// keyboardIO::keyboardIO : Undoes what the constructor does
keyboardIO::~keyboardIO( ) {

  // Restore the old terminal properties
  tcsetattr( 0, TCSANOW, &origTerm );

}

int keyboardIO::kbhit() {

  char ch;
  int nread;

  if( peek != -1 ) return 1;

  // Set terminal timeout to 0
  newTerm.c_cc[ VMIN ] = 0;
  tcsetattr( 0, TCSANOW, &newTerm );

  // Attempt to read characters
  nread = read( 0, &ch, 1 );

  // Set terminal timeout to 1
  newTerm.c_cc[ VMIN ] = 1;
  tcsetattr( 0, TCSANOW, &newTerm );

  // Buffer the character if any is read
  if ( nread == 1 ) {
   peek = ch;
   return 1;
  }

  // Failure. Keyboard is not hit.
  return 0;
}

int keyboardIO::getch() {

  char ch;

  if( peek != -1 ) {
    // Return any buffered value
    ch = peek;
    peek = -1;
    return ch;
  }

  // If there weren't any buffered values, attempt to read
  read( 0, &ch,1 );

  return ch;
}

// The extern interface
int kbhit( void ) { return globalKBIO.kbhit(); }
int getch( void ) { return globalKBIO.getch(); }

#endif

